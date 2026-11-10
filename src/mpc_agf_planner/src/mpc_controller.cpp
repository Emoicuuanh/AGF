#include <mpc_agf_planner/mpc_controller.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include <OsqpEigen/OsqpEigen.h>

namespace mpc_agf_planner
{
namespace
{
double clampAbs(double x, double limit)
{
  return std::max(-limit, std::min(limit, x));
}

double signOf(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}
}  // namespace

MpcController::MpcController() : solver_(new OsqpEigen::Solver()) {}

MpcController::~MpcController() = default;

void MpcController::setParams(const MpcParams& params)
{
  p_ = params;
  p_.horizon = std::max(2, p_.horizon);
  p_.tau_steer = std::max(1e-3, p_.tau_steer);
  p_.dt = std::max(1e-3, p_.dt);
  reset();
}

void MpcController::reset()
{
  // Safe to call on a partly set up solver: each clear checks its own state
  solver_->clearSolver();
  solver_->data()->clearHessianMatrix();
  solver_->data()->clearLinearConstraintsMatrix();
  initialized_ = false;
}

double MpcController::feedforward(double kappa, double v_sign) const
{
  return std::atan(p_.steer_sign * v_sign * p_.wheelbase * kappa);
}

MpcState MpcController::compensateDelay(const MpcState& x0, double v,
                                        double kappa, double last_cmd) const
{
  MpcState x = x0;
  const double h = 0.01;
  int steps = static_cast<int>(std::round(p_.delay / h));
  for (int i = 0; i < steps; ++i)
  {
    double d_rate = clampAbs((last_cmd - x.delta) / p_.tau_steer, p_.max_steer_rate);
    x.e_y += h * v * std::sin(x.e_psi);
    x.e_psi += h * (v * std::tan(x.delta) * p_.steer_sign / p_.wheelbase -
                    std::fabs(v) * kappa);
    x.delta += h * d_rate;
  }
  return x;
}

double MpcController::fallback(const MpcState& x0, double v, double kappa) const
{
  // Desired heading error pulls e_y to zero; steer to reach that heading
  const double k_y = 1.0, k_psi = 1.0;
  double vs = signOf(v);
  double u = feedforward(kappa, vs) -
             p_.steer_sign * (k_psi * k_y * x0.e_y + vs * k_psi * x0.e_psi);
  return clampAbs(u, p_.max_steer);
}

void MpcController::buildHessian()
{
  const int N = p_.horizon;
  const int n = 4 * N;
  std::vector<Eigen::Triplet<double>> t;
  t.reserve(6 * N);
  for (int k = 1; k <= N; ++k)
  {
    double scale = (k == N) ? p_.qf_scale : 1.0;
    t.emplace_back(stateIndex(k, 0), stateIndex(k, 0), 2.0 * p_.q_ey * scale);
    t.emplace_back(stateIndex(k, 1), stateIndex(k, 1), 2.0 * p_.q_epsi * scale);
  }
  for (int k = 0; k < N; ++k)
  {
    // r (u_k - ff_k)^2 + r_d (u_k - u_{k-1})^2, upper triangle only
    double diag = 2.0 * p_.r_steer + 2.0 * p_.r_dsteer;
    if (k + 1 < N)
    {
      diag += 2.0 * p_.r_dsteer;
    }
    t.emplace_back(inputIndex(k), inputIndex(k), diag);
    if (k + 1 < N)
    {
      t.emplace_back(inputIndex(k), inputIndex(k + 1), -2.0 * p_.r_dsteer);
    }
  }
  hessian_.resize(n, n);
  hessian_.setFromTriplets(t.begin(), t.end());
  hessian_.makeCompressed();
}

void MpcController::buildConstraints(const MpcState& x0,
                                     const std::vector<double>& v,
                                     double last_cmd,
                                     const std::vector<double>& dff)
{
  const int N = p_.horizon;
  const int n = 4 * N;
  const int m = 5 * N;
  const double dt = p_.dt;
  const double alpha = std::exp(-dt / p_.tau_steer);  // exact discretized lag
  const double rate = p_.max_steer_rate * dt;

  std::vector<Eigen::Triplet<double>> t;
  t.reserve(12 * N);
  lower_.setZero(m);
  upper_.setZero(m);

  // Dynamics: x_{k+1} - A_k x_k - B_k u_k = c_k (rows 0..3N-1)
  for (int k = 0; k < N; ++k)
  {
    double vk = v[k];
    double cos_ff = std::cos(dff[k]);
    double b = vk * p_.steer_sign / (p_.wheelbase * cos_ff * cos_ff);
    int row = 3 * k;

    // Structural entries are always inserted (even if zero) so the sparsity
    // pattern never changes and OSQP can update values in place.
    t.emplace_back(row + 0, stateIndex(k + 1, 0), 1.0);
    t.emplace_back(row + 1, stateIndex(k + 1, 1), 1.0);
    t.emplace_back(row + 2, stateIndex(k + 1, 2), 1.0);
    t.emplace_back(row + 2, inputIndex(k), -(1.0 - alpha));

    double c1 = -dt * b * dff[k];
    if (k == 0)
    {
      // x_0 is known: move A_0 x_0 to the right-hand side
      double r0 = x0.e_y + dt * vk * x0.e_psi;
      double r1 = x0.e_psi + dt * b * x0.delta + c1;
      double r2 = alpha * x0.delta;
      lower_(row + 0) = upper_(row + 0) = r0;
      lower_(row + 1) = upper_(row + 1) = r1;
      lower_(row + 2) = upper_(row + 2) = r2;
    }
    else
    {
      t.emplace_back(row + 0, stateIndex(k, 0), -1.0);
      t.emplace_back(row + 0, stateIndex(k, 1), -dt * vk);
      t.emplace_back(row + 1, stateIndex(k, 1), -1.0);
      t.emplace_back(row + 1, stateIndex(k, 2), -dt * b);
      t.emplace_back(row + 2, stateIndex(k, 2), -alpha);
      lower_(row + 1) = upper_(row + 1) = c1;
    }
  }

  // Steering limit (rows 3N..4N-1)
  for (int k = 0; k < N; ++k)
  {
    int row = 3 * N + k;
    t.emplace_back(row, inputIndex(k), 1.0);
    lower_(row) = -p_.max_steer;
    upper_(row) = p_.max_steer;
  }

  // Steering rate limit (rows 4N..5N-1)
  double last = clampAbs(last_cmd, p_.max_steer);
  for (int k = 0; k < N; ++k)
  {
    int row = 4 * N + k;
    t.emplace_back(row, inputIndex(k), 1.0);
    if (k == 0)
    {
      lower_(row) = last - rate;
      upper_(row) = last + rate;
    }
    else
    {
      t.emplace_back(row, inputIndex(k - 1), -1.0);
      lower_(row) = -rate;
      upper_(row) = rate;
    }
  }

  constraints_.resize(m, n);
  constraints_.setFromTriplets(t.begin(), t.end());
  constraints_.makeCompressed();
}

MpcResult MpcController::solve(const MpcState& x0, const std::vector<double>& v,
                               const std::vector<double>& kappa,
                               double last_cmd)
{
  auto t_start = std::chrono::steady_clock::now();
  MpcResult res;
  const int N = p_.horizon;
  if (static_cast<int>(v.size()) < N || static_cast<int>(kappa.size()) < N)
  {
    res.delta_cmd = fallback(x0, v.empty() ? 0.0 : v[0],
                             kappa.empty() ? 0.0 : kappa[0]);
    return res;
  }

  std::vector<double> dff(N);
  for (int k = 0; k < N; ++k)
  {
    dff[k] = feedforward(kappa[k], signOf(v[k]));
  }

  buildConstraints(x0, v, last_cmd, dff);

  gradient_.setZero(4 * N);
  for (int k = 0; k < N; ++k)
  {
    gradient_(inputIndex(k)) += -2.0 * p_.r_steer * dff[k];
  }
  gradient_(inputIndex(0)) += -2.0 * p_.r_dsteer * clampAbs(last_cmd, p_.max_steer);

  bool ok = true;
  if (!initialized_)
  {
    buildHessian();
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setMaxIteration(4000);
    solver_->settings()->setAbsoluteTolerance(1e-5);
    solver_->settings()->setRelativeTolerance(1e-5);
    solver_->data()->setNumberOfVariables(4 * N);
    solver_->data()->setNumberOfConstraints(5 * N);
    ok = solver_->data()->setHessianMatrix(hessian_) &&
         solver_->data()->setGradient(gradient_) &&
         solver_->data()->setLinearConstraintsMatrix(constraints_) &&
         solver_->data()->setLowerBound(lower_) &&
         solver_->data()->setUpperBound(upper_) && solver_->initSolver();
    initialized_ = ok;
  }
  else
  {
    ok = solver_->updateGradient(gradient_) &&
         solver_->updateLinearConstraintsMatrix(constraints_) &&
         solver_->updateBounds(lower_, upper_);
  }

  if (ok)
  {
    ok = solver_->solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
  }
  if (ok)
  {
    OsqpEigen::Status status = solver_->getStatus();
    ok = status == OsqpEigen::Status::Solved ||
         status == OsqpEigen::Status::SolvedInaccurate;
  }

  if (ok)
  {
    const Eigen::VectorXd z = solver_->getSolution();
    res.solved = std::isfinite(z(inputIndex(0)));
    if (res.solved)
    {
      res.delta_cmd = clampAbs(z(inputIndex(0)), p_.max_steer);
      res.e_y_pred.resize(N);
      res.delta_pred.resize(N);
      for (int k = 0; k < N; ++k)
      {
        res.e_y_pred[k] = z(stateIndex(k + 1, 0));
        res.delta_pred[k] = z(inputIndex(k));
      }
    }
  }
  if (!res.solved)
  {
    // Rebuild the solver next cycle in case its internal state is broken
    reset();
    res.delta_cmd = fallback(x0, v[0], kappa[0]);
  }

  res.solve_ms = std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - t_start)
                     .count();
  return res;
}

}  // namespace mpc_agf_planner
