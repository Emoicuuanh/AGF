// Closed-loop tests of the MPC path tracking core against a kinematic tricycle
// with steering lag, steering rate limit and command delay.
#include <mpc_agf_planner/mpc_controller.h>
#include <mpc_agf_planner/path_reference.h>
#include <mpc_agf_planner/speed_profile.h>
#include <mpc_agf_planner/steering_geometry.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

using namespace mpc_agf_planner;

namespace
{
const double kCtrlDt = 0.05;  // planner period (20 Hz)

struct Segment
{
  double length;
  double kappa;
};

std::vector<Point2> makePath(double x, double y, double th,
                             const std::vector<Segment>& segs, double step = 0.1)
{
  std::vector<Point2> p{{x, y}};
  for (const auto& sg : segs)
  {
    int n = static_cast<int>(std::round(sg.length / step));
    for (int i = 0; i < n; ++i)
    {
      double thm = th + sg.kappa * step / 2;
      x += step * std::cos(thm);
      y += step * std::sin(thm);
      th += sg.kappa * step;
      p.push_back({x, y});
    }
  }
  return p;
}

struct Robot
{
  double x, y, th, delta, v;  // delta = measured (left) wheel angle
};

struct SimResult
{
  bool reached = false;
  double time = 0.0;
  double final_e_y = 0.0;
  double final_e_psi = 0.0;
  double max_cmd = 0.0;
  double max_cmd_step = 0.0;
  double max_solve_ms = 0.0;
  double mean_solve_ms = 0.0;
  int fallbacks = 0;
  std::vector<double> t, e_y;
};

struct SimConfig
{
  bool reverse = false;
  double xy_tolerance = 0.02;
  double v_max = 0.7;
  double time_limit = 60.0;
  double track_width = 0.0;  // Ackermann, left wheel measured and commanded
  double max_wheel = 1.2;    // max_angular_z
  MpcParams model;  // the plant uses the same model as the controller
};

// Mirrors MpcAgfPlannerROS::trackPath() without ROS
SimResult simulate(const std::vector<Point2>& raw, Robot robot, const SimConfig& cfg)
{
  const double dir = cfg.reverse ? -1.0 : 1.0;
  const double a_acc = 0.4, a_brake = 1.2, v_min_mpc = 0.05;
  MpcParams mp = cfg.model;
  SteeringGeometry geom;
  geom.wheelbase = mp.wheelbase;
  geom.track_width = cfg.track_width;
  geom.steer_sign = mp.steer_sign;
  mp.max_steer = geom.maxCentreAngle(cfg.max_wheel);
  MpcController mpc;
  mpc.setParams(mp);

  PathReference path;
  path.build(raw, cfg.reverse, 0.05, robot.th);
  SpeedLimits lim{cfg.v_max, 0.25, 0.3, 0.1};
  computeSpeedProfile(path, lim);

  std::deque<double> in_flight;  // commands waiting out the delay
  int delay_cycles = static_cast<int>(std::round(mp.delay / kCtrlDt));
  double plant_cmd = 0.0, last_cmd = 0.0;  // plant_cmd: wheel, last_cmd: centre
  long hint = -1;
  SimResult res;
  double solve_sum = 0.0;
  int solves = 0;

  for (double t = 0.0; t < cfg.time_limit; t += kCtrlDt)
  {
    std::size_t idx = path.nearest(robot.x, robot.y, hint, 41);
    hint = static_cast<long>(idx);
    PathErrors err = path.errors(robot.x, robot.y, robot.th, idx);
    res.t.push_back(t);
    res.e_y.push_back(err.e_y);

    const RefPoint& goal = path.back();
    double gx = goal.x - robot.x, gy = goal.y - robot.y;
    double along = (std::cos(goal.yaw) * gx + std::sin(goal.yaw) * gy) * dir;
    if (std::hypot(gx, gy) < cfg.xy_tolerance ||
        (err.s >= path.length() - 0.05 && along < 0.0))
    {
      res.reached = true;
      res.time = t;
      res.final_e_y = err.e_y;
      res.final_e_psi = err.e_psi;
      break;
    }

    double v_target = std::min(path.sampleAtS(err.s).v, cfg.v_max);
    double v_now = std::fabs(robot.v);
    double v_cmd = v_target > v_now ? std::min(v_target, v_now + a_acc * kCtrlDt)
                                    : std::max(v_target, v_now - a_brake * kCtrlDt);

    const int N = mp.horizon;
    std::vector<double> v_h(N), k_h(N + 1);
    double s = err.s;
    for (int k = 0; k <= N; ++k)
    {
      RefPoint rp = path.sampleAtS(s);
      k_h[k] = rp.kappa;
      if (k < N)
      {
        double vk = std::min(std::min(rp.v, cfg.v_max), v_cmd + a_acc * mp.dt * k);
        v_h[k] = dir * vk;
        s += vk * mp.dt;
      }
    }

    double cmd = last_cmd;
    if (v_cmd >= v_min_mpc)
    {
      MpcState x0{err.e_y, err.e_psi, geom.leftToCentre(robot.delta)};
      x0 = mpc.compensateDelay(x0, dir * v_cmd, k_h[0], last_cmd);
      MpcResult r = mpc.solve(x0, v_h, k_h, last_cmd);
      if (!r.solved)
      {
        ++res.fallbacks;
      }
      cmd = r.delta_cmd;
      solve_sum += r.solve_ms;
      ++solves;
      res.max_solve_ms = std::max(res.max_solve_ms, r.solve_ms);
    }
    double wheel_cmd = std::max(-cfg.max_wheel, std::min(cfg.max_wheel, geom.centreToLeft(cmd)));
    res.max_cmd = std::max(res.max_cmd, std::fabs(wheel_cmd));
    res.max_cmd_step = std::max(res.max_cmd_step, std::fabs(cmd - last_cmd));
    last_cmd = cmd;

    // Plant: delayed command, rate-limited first-order steering, kinematics
    in_flight.push_back(wheel_cmd);
    if (static_cast<int>(in_flight.size()) > delay_cycles)
    {
      plant_cmd = in_flight.front();
      in_flight.pop_front();
    }
    robot.v = dir * v_cmd;
    const double h = 0.005;
    for (int i = 0; i < static_cast<int>(kCtrlDt / h + 0.5); ++i)
    {
      double rate = (plant_cmd - robot.delta) / mp.tau_steer;
      rate = std::max(-mp.max_steer_rate, std::min(mp.max_steer_rate, rate));
      robot.delta += rate * h;
      robot.x += h * robot.v * std::cos(robot.th);
      robot.y += h * robot.v * std::sin(robot.th);
      robot.th = normalizeAngle(robot.th + h * robot.v * std::tan(geom.leftToCentre(robot.delta)) *
                                               mp.steer_sign / mp.wheelbase);
    }
  }
  res.mean_solve_ms = solves ? solve_sum / solves : 0.0;
  return res;
}

double maxAbsAfter(const SimResult& r, double t0)
{
  double m = 0.0;
  for (std::size_t i = 0; i < r.t.size(); ++i)
  {
    if (r.t[i] >= t0)
    {
      m = std::max(m, std::fabs(r.e_y[i]));
    }
  }
  return m;
}

void expectConstraints(const SimResult& r, const MpcParams& p, double max_wheel = 1.2)
{
  EXPECT_LE(r.max_cmd, max_wheel + 1e-6);
  // First-step rate constraint is applied per MPC step (mpc_dt)
  EXPECT_LE(r.max_cmd_step, p.max_steer_rate * p.dt + 1e-3);
  EXPECT_EQ(r.fallbacks, 0);
}
}  // namespace

TEST(PathReference, ResamplesAndComputesCurvature)
{
  const double R = 2.0;
  PathReference path;
  path.build(makePath(0, 0, 0, {{R * M_PI / 2, 1.0 / R}}, 0.2), false, 0.05, 0.0);
  ASSERT_GT(path.size(), 50u);
  for (std::size_t i = 1; i + 1 < path.size(); ++i)
  {
    EXPECT_NEAR(path.at(i).s - path.at(i - 1).s, 0.05, 1e-6);
  }
  EXPECT_NEAR(path.at(path.size() / 2).kappa, 1.0 / R, 0.02);
  EXPECT_NEAR(path.at(path.size() / 2).yaw, M_PI / 4, 0.05);
}

TEST(PathReference, ReverseFlipsHeadingAndKeepsCurvature)
{
  auto raw = makePath(0, 0, 0, {{2.0, 0.0}, {2.0, 0.5}});
  PathReference fwd, rev;
  fwd.build(raw, false, 0.05, 0.0);
  rev.build(raw, true, 0.05, 0.0);
  ASSERT_EQ(fwd.size(), rev.size());
  std::size_t i = fwd.size() * 3 / 4;
  EXPECT_NEAR(std::fabs(normalizeAngle(rev.at(i).yaw - fwd.at(i).yaw)), M_PI, 1e-6);
  EXPECT_NEAR(rev.at(i).kappa, fwd.at(i).kappa, 1e-9);
  // Left of the reference heading is the other side when reversing
  PathErrors ef = fwd.errors(1.0, 0.1, 0.0, fwd.nearest(1.0, 0.1, -1, 0));
  PathErrors er = rev.errors(1.0, 0.1, M_PI, rev.nearest(1.0, 0.1, -1, 0));
  EXPECT_NEAR(ef.e_y, 0.1, 1e-6);
  EXPECT_NEAR(er.e_y, -0.1, 1e-6);
}

TEST(SteeringGeometry, LeftWheelIsInnerWhenTurningLeft)
{
  for (double sign : {1.0, -1.0})
  {
    SteeringGeometry g;
    g.wheelbase = 1.2;
    g.track_width = 0.9;
    g.steer_sign = sign;
    const double centre = sign * 0.4;  // a left turn
    double left = g.centreToLeft(centre);
    // Inner wheel: tan(left) = L / (R - W/2) with R = L / tan(centre)
    double r = g.wheelbase / std::tan(0.4);
    EXPECT_NEAR(sign * left, std::atan(g.wheelbase / (r - g.track_width / 2)), 1e-9);
    EXPECT_GT(std::fabs(left), std::fabs(centre));
    // Turning right the left wheel is the outer one
    EXPECT_LT(std::fabs(g.centreToLeft(-centre)), std::fabs(centre));
    for (double a = -1.4; a <= 1.4; a += 0.1)
    {
      EXPECT_NEAR(g.leftToCentre(g.centreToLeft(a)), a, 1e-9);
    }
  }
  // Inner wheel past 90 degrees stays continuous
  SteeringGeometry g;
  g.track_width = 0.9;
  EXPECT_GT(g.centreToLeft(1.4), M_PI / 2);
  EXPECT_NEAR(g.leftToCentre(g.centreToLeft(1.4)), 1.4, 1e-9);
  // Centre limit keeps the inner wheel at max
  double c = g.maxCentreAngle(1.2);
  EXPECT_NEAR(g.centreToLeft(c), 1.2, 1e-9);
  // No track width = single centre wheel
  SteeringGeometry single;
  EXPECT_NEAR(single.centreToLeft(0.7), 0.7, 1e-12);
}

TEST(SpeedProfile, SlowsForCurvesAndStopsAtGoal)
{
  PathReference path;
  path.build(makePath(0, 0, 0, {{3.0, 0.0}, {3.0, 1.0}, {3.0, 0.0}}), false, 0.05, 0.0);
  computeSpeedProfile(path, {0.7, 0.25, 0.3, 0.1});
  EXPECT_NEAR(path.sampleAtS(1.0).v, 0.7, 1e-6);
  EXPECT_NEAR(path.sampleAtS(4.5).v, 0.5, 0.01);  // sqrt(0.25 / 1.0)
  EXPECT_LT(path.sampleAtS(2.9).v, 0.7);          // brakes before the curve
  EXPECT_NEAR(path.back().v, 0.1, 1e-6);          // creep at the goal
}

TEST(SpeedProfile, SlowsWhereSteeringMustTurnFast)
{
  PathReference path;
  // Straight into a 90 degree corner of radius 1 m, then straight
  path.build(makePath(0, 0, 0, {{3.0, 0.0}, {M_PI / 2, 1.0}, {3.0, 0.0}}), false, 0.05, 0.0);
  SpeedLimits loose{0.7, 10.0, 0.3, 0.1};  // a_lat_max high: no curvature limit
  computeSpeedProfile(path, loose);
  SpeedLimits rate = loose;
  rate.wheelbase = 1.28;
  rate.steer_rate = 0.5;
  PathReference path_rate = path;
  computeSpeedProfile(path_rate, rate);
  // Corner entry: the steering angle needed goes from 0 to atan(1.28) within
  // the 0.5 m curvature smoothing span, so the wheel limits the speed there
  EXPECT_NEAR(path.sampleAtS(3.0).v, 0.7, 1e-6);
  EXPECT_LT(path_rate.sampleAtS(3.0).v, 0.4);
  // Far from the corner the limit has no effect
  EXPECT_NEAR(path_rate.sampleAtS(0.5).v, 0.7, 1e-6);
}

TEST(SpeedProfile, CreepsBeforeGoalTolerance)
{
  PathReference path;
  path.build(makePath(0, 0, 0, {{5.0, 0.0}}), false, 0.05, 0.0);
  SpeedLimits lim{0.7, 0.25, 0.3, 0.1};
  lim.creep_dist = 0.3;
  computeSpeedProfile(path, lim);
  // Already at creep speed when entering a 0.1 m goal tolerance
  EXPECT_NEAR(path.sampleAtS(4.9).v, 0.1, 1e-6);
  EXPECT_NEAR(path.sampleAtS(4.7).v, 0.1, 1e-6);
  // Braking curve ends at the start of the creep zone
  EXPECT_NEAR(path.sampleAtS(4.2).v, std::sqrt(0.01 + 2.0 * 0.3 * 0.5), 0.01);
  EXPECT_NEAR(path.sampleAtS(1.0).v, 0.7, 1e-6);
}

TEST(Mpc, StraightLineOffsetConverges)
{
  SimConfig cfg;
  auto raw = makePath(0, 0, 0, {{10.0, 0.0}});
  SimResult r = simulate(raw, {0.0, 0.3, 0.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(maxAbsAfter(r, 8.0), 0.02);
  // No overshoot past 5 cm to the other side
  double overshoot = 0.0;
  for (double e : r.e_y)
  {
    overshoot = std::max(overshoot, -e);
  }
  EXPECT_LT(overshoot, 0.05);
  expectConstraints(r, cfg.model);
}

TEST(Mpc, CircleSteadyStateError)
{
  SimConfig cfg;
  const double R = 2.0;
  auto raw = makePath(0, 0, 0, {{1.0, 0.0}, {R * 1.5 * M_PI, 1.0 / R}});
  SimResult r = simulate(raw, {0.0, 0.0, 0.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(maxAbsAfter(r, 6.0), 0.02);
  expectConstraints(r, cfg.model);
}

TEST(Mpc, ReverseStraightDock)
{
  SimConfig cfg;
  cfg.reverse = true;
  cfg.xy_tolerance = 0.01;
  // Robot faces +x and backs up along -x
  auto raw = makePath(0, 0, M_PI, {{5.0, 0.0}});
  SimResult r = simulate(raw, {0.0, 0.1, 5.0 * M_PI / 180.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(std::fabs(r.final_e_y), 0.01);
  EXPECT_LT(std::fabs(r.final_e_psi), 0.02);
  expectConstraints(r, cfg.model);
}

TEST(Mpc, ReverseTurnIntoDock)
{
  SimConfig cfg;
  cfg.reverse = true;
  cfg.xy_tolerance = 0.01;
  auto raw = makePath(0, 0, M_PI, {{1.0, 0.0}, {2.0 * M_PI / 2, 0.5}, {2.0, 0.0}});
  SimResult r = simulate(raw, {0.0, 0.0, 0.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(std::fabs(r.final_e_y), 0.01);
  EXPECT_LT(std::fabs(r.final_e_psi), 0.02);
  expectConstraints(r, cfg.model);
}

TEST(Mpc, NegativeSteerSignTracks)
{
  SimConfig cfg;
  cfg.model.steer_sign = -1.0;
  auto raw = makePath(0, 0, 0, {{2.0, 0.0}, {3.0, 0.5}, {2.0, 0.0}});
  SimResult r = simulate(raw, {0.0, -0.2, 0.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(maxAbsAfter(r, 6.0), 0.03);
  expectConstraints(r, cfg.model);
}

TEST(Mpc, AckermannLeftWheelTracksBothTurnDirections)
{
  for (double sign : {1.0, -1.0})
  {
    SimConfig cfg;
    cfg.track_width = 1.2;  // AGF forklift
    cfg.model.steer_sign = sign;
    auto raw = makePath(0, 0, 0, {{2.0, 0.0}, {3.0, 0.6}, {1.0, 0.0}, {3.0, -0.6}, {2.0, 0.0}});
    SimResult r = simulate(raw, {0.0, 0.2, 0.0, 0.0, 0.0}, cfg);
    ASSERT_TRUE(r.reached);
    EXPECT_LT(maxAbsAfter(r, 6.0), 0.03);
    expectConstraints(r, cfg.model);
  }
  // Reverse dock with Ackermann
  SimConfig cfg;
  cfg.track_width = 1.2;  // AGF forklift
  cfg.reverse = true;
  cfg.xy_tolerance = 0.01;
  auto raw = makePath(0, 0, M_PI, {{1.0, 0.0}, {2.0 * M_PI / 2, 0.5}, {2.0, 0.0}});
  SimResult r = simulate(raw, {0.0, 0.0, 0.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(std::fabs(r.final_e_y), 0.01);
  EXPECT_LT(std::fabs(r.final_e_psi), 0.02);
}

TEST(Mpc, ShortGoalDoesNotSwerve)
{
  SimConfig cfg;
  auto raw = makePath(0, 0, 0, {{0.2, 0.0}}, 0.05);
  SimResult r = simulate(raw, {0.0, 0.02, 0.0, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LE(r.max_cmd, 0.3);
}

TEST(Mpc, SolveTimeAndFallback)
{
  SimConfig cfg;
  auto raw = makePath(0, 0, 0, {{2.0, 0.0}, {2.5, 0.6}, {1.0, 0.0}, {2.5, -0.6}, {2.0, 0.0}});
  SimResult r = simulate(raw, {0.0, 0.25, 0.1, 0.0, 0.0}, cfg);
  ASSERT_TRUE(r.reached);
  EXPECT_LT(r.mean_solve_ms, 5.0);
  EXPECT_EQ(r.fallbacks, 0);

  // Bad input: the controller still returns a bounded command
  MpcController mpc;
  mpc.setParams(cfg.model);
  MpcResult bad = mpc.solve({0.1, 0.0, 0.0}, {}, {}, 0.0);
  EXPECT_FALSE(bad.solved);
  EXPECT_LE(std::fabs(bad.delta_cmd), cfg.model.max_steer);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
