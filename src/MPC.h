#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

extern const double Lf;
extern const bool kCtrvModel;

class MPC {
 public:
  MPC();

  virtual ~MPC() = default;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);

  /**
   * Get x-coordinates of computed track points
   */
  const std::vector<double>& xpts() const { return xpts_; }

  /**
   * Get y-coordinates of computed track points
   */
  const std::vector<double>& ypts() const { return ypts_; }

private:
  std::vector<double> xpts_;
  std::vector<double> ypts_;

};

#endif /* MPC_H */
