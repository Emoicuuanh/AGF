#ifndef MPC_AGF_PLANNER_MPC_CONTROLLER_H_
#define MPC_AGF_PLANNER_MPC_CONTROLLER_H_

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace OsqpEigen
{
class Solver;
}

namespace mpc_agf_planner
{
struct MpcParams
{
  double wheelbase = 1.2;        // Fixed axle to steered wheel (m)
  double steer_sign = 1.0;       // +1: positive steering turns left when driving forward
  double tau_steer = 0.4;        // Steering motor time constant (s)
  double max_steer = 1.2;        // |delta_cmd| limit, centre angle (rad)
  double max_steer_rate = 1.0;   // Steering rate limit (rad/s)
  double delay = 0.1;            // Command-to-motion latency (s)
  double dt = 0.1;               // Prediction step (s)
  int horizon = 25;              // Prediction steps
  double q_ey = 40.0;            // Weight on lateral error
  double q_epsi = 6.0;           // Weight on heading error
  double r_steer = 0.3;          // Weight on (delta_cmd - delta_ff)
  double r_dsteer = 1.0;         // Weight on delta_cmd change per step
  double qf_scale = 10.0;        // Terminal weight multiplier
};

// State at the start of the horizon, in path-error coordinates
struct MpcState
{
  double e_y;    // m
  double e_psi;  // rad
  double delta;  // Measured steering angle, centre angle (rad)
};

struct MpcResult
{
  bool solved = false;            // false = fallback law was used
  double delta_cmd = 0.0;         // First command to send (rad)
  std::vector<double> e_y_pred;   // Predicted lateral error, steps 1..N
  std::vector<double> delta_pred; // Predicted steering commands, steps 0..N-1
  double solve_ms = 0.0;
};

/**
 * Linear time-varying MPC on a kinematic tricycle with first-order steering lag.
 *
 *   e_y'   = v * sin(e_psi)
 *   e_psi' = v * tan(delta) * steer_sign / L - |v| * kappa
 *   delta' = (delta_cmd - delta) / tau
 *
 * Linearized each step around delta_ff = atan(steer_sign * sign(v) * L * kappa)
 * and solved as a sparse QP with OSQP.
 */
class MpcController
{
public:
  MpcController();
  ~MpcController();

  void setParams(const MpcParams& params);
  const MpcParams& params() const { return p_; }

  // Drop the warm start (call on every new plan)
  void reset();

  /**
   * @param x0        Measured state
   * @param v         Signed speed for each step, size >= horizon
   * @param kappa     Path curvature for each step, size >= horizon + 1
   * @param last_cmd  Steering command sent in the previous cycle
   */
  MpcResult solve(const MpcState& x0, const std::vector<double>& v,
                  const std::vector<double>& kappa, double last_cmd);

  // Steering the path curvature needs at speed sign v_sign
  double feedforward(double kappa, double v_sign) const;

  // Propagate x0 through `delay` seconds of the command already sent
  MpcState compensateDelay(const MpcState& x0, double v, double kappa,
                           double last_cmd) const;

  // Simple stabilizing law used when the QP fails
  double fallback(const MpcState& x0, double v, double kappa) const;

private:
  void buildHessian();
  void buildConstraints(const MpcState& x0, const std::vector<double>& v,
                        double last_cmd, const std::vector<double>& dff);

  int stateIndex(int k, int i) const { return 3 * (k - 1) + i; }  // k = 1..N
  int inputIndex(int k) const { return 3 * p_.horizon + k; }      // k = 0..N-1

  MpcParams p_;
  std::unique_ptr<OsqpEigen::Solver> solver_;
  bool initialized_ = false;

  Eigen::SparseMatrix<double> hessian_;
  Eigen::SparseMatrix<double> constraints_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lower_;
  Eigen::VectorXd upper_;
};

}  // namespace mpc_agf_planner

#endif  // MPC_AGF_PLANNER_MPC_CONTROLLER_H_
