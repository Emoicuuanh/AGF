#include <mpc_agf_planner/speed_profile.h>

#include <algorithm>
#include <cmath>

namespace mpc_agf_planner
{
void computeSpeedProfile(PathReference& path, const SpeedLimits& limits)
{
  auto& pts = path.points();
  if (pts.empty())
  {
    return;
  }

  // Curvature limit
  for (auto& p : pts)
  {
    double v = limits.v_max;
    if (std::fabs(p.kappa) > 1e-6)
    {
      v = std::min(v, std::sqrt(limits.a_lat_max / std::fabs(p.kappa)));
    }
    p.v = v;
  }

  // Steering-rate limit: at a curve entry or exit the steering angle the path
  // needs changes over a short distance, and the wheel can only turn so fast
  if (limits.wheelbase > 0.0 && limits.steer_rate > 0.0)
  {
    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
    {
      double d_delta = std::fabs(std::atan(limits.wheelbase * pts[i + 1].kappa) -
                                 std::atan(limits.wheelbase * pts[i].kappa));
      if (d_delta > 1e-6)
      {
        double v = limits.steer_rate * (pts[i + 1].s - pts[i].s) / d_delta;
        pts[i].v = std::min(pts[i].v, v);
        pts[i + 1].v = std::min(pts[i + 1].v, v);
      }
    }
  }

  // Creep zone: the planner stops at the goal tolerance, so arriving there at
  // creep speed keeps the coast-down after the stop short
  const double s_end = pts.back().s;
  for (auto& p : pts)
  {
    if (s_end - p.s <= limits.creep_dist)
    {
      p.v = std::min(p.v, limits.v_creep);
    }
  }

  // Backward pass: be able to brake to creep speed before the creep zone
  pts.back().v = std::min(pts.back().v, limits.v_creep);
  for (std::size_t i = pts.size() - 1; i-- > 0;)
  {
    double ds = pts[i + 1].s - pts[i].s;
    double v_brake = std::sqrt(pts[i + 1].v * pts[i + 1].v + 2.0 * limits.a_dec * ds);
    pts[i].v = std::min(pts[i].v, v_brake);
  }

  // Never plan below creep speed; the goal check stops the robot
  for (auto& p : pts)
  {
    p.v = std::max(p.v, limits.v_creep);
  }
}

}  // namespace mpc_agf_planner
