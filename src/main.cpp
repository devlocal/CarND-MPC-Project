#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// Controls latency, seconds
constexpr double controls_latency_sec = 0.1;

// Time spent inside Solve()
double solve_latency = 0;
constexpr double solve_alpha = 0.5;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// 25 degrees in radians
const double deg2rad_25 = -deg2rad(25);

// Kilometers in 1 mile
constexpr double kKmInMile = 1.61;

// true to print debug output
constexpr bool kPrintDebugOutput = false;

// true to draw trajectory (yellow and green lines)
constexpr bool kDrawTrajectory = true;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/*
 * Transforms waypoints from global coordinates to car coordinates.
 * Performs an in-place transformation, does not return any value.
 */
void globalToCar(vector<double> &ptsx, vector<double> &ptsy, double px, double py, double psi) {
  // Map ptsx and ptsy from global to vehicle coordinates
  for (size_t i = 0; i < ptsx.size(); ++i) {
    double offset_x = ptsx[i] - px;
    double offset_y = ptsy[i] - py;

    ptsx[i] = offset_x * cos(-psi) - offset_y * sin(-psi);
    ptsy[i] = offset_x * sin(-psi) + offset_y * cos(-psi);
  }
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (kPrintDebugOutput) {
      cout << sdata << endl;
    }
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= kKmInMile;

          // Steering angle is expressed in radians
          double delta = j[1]["steering_angle"];
          delta *= -1;
          double throttle = j[1]["throttle"];

          assert(ptsx.size() == ptsy.size());

          // Map ptsx and ptsy from global to vehicle coordinates
          globalToCar(ptsx, ptsy, px, py, psi);

          // Fit the polynomial
          Eigen::VectorXd ptsx_evec = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsy_evec = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
          auto coeffs = polyfit(ptsx_evec, ptsy_evec, 3);

          // Sum up control latency and latency induced by Solve()
          double latency = controls_latency_sec + solve_latency;

          // Predict future vehicle state in vehicle coordinates
          double future_px;
          double future_py;
          double yaw_rate = (v / Lf) * delta;

          if (kCtrvModel && abs(yaw_rate) > 1e-16) {
            double v_yr = v / yaw_rate;
            future_px = v_yr * (sin(yaw_rate * latency));
            future_py = v_yr * (1 - cos(yaw_rate * latency));
          } else {
            future_px = v * latency;
            future_py = 0;
          }

          double future_psi = yaw_rate * latency;
          double future_v = v + throttle * latency;

          // Compute current error
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          // Predict future error
          double future_cte  = cte + v * sin(epsi) * latency;
          double future_epsi = epsi + future_psi;

          Eigen::VectorXd state(6);
          state << future_px, future_py, future_psi, future_v, future_cte, future_epsi;

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          clock_t begin = clock();
          vector<double> solution = mpc.Solve(state, coeffs);
          clock_t end = clock();
          double elapsed_sec = static_cast<double>(end - begin) / CLOCKS_PER_SEC;

          // Track time of first 100 calls to Solve()
          solve_latency = solve_alpha * solve_latency + (1 - solve_alpha) * elapsed_sec;

          double steer_value = solution[0] / deg2rad_25;
          double throttle_value = solution[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          if (kDrawTrajectory) {
            mpc_x_vals = mpc.xpts();
            mpc_y_vals = mpc.ypts();
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          if (kDrawTrajectory) {
            next_x_vals = ptsx;
            next_y_vals = ptsy;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (kPrintDebugOutput) {
            std::cout << msg << std::endl;
          }
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
