#ifndef MPC_AGF_PLANNER_SPEED_PROFILE_H_
#define MPC_AGF_PLANNER_SPEED_PROFILE_H_

#include <mpc_agf_planner/path_reference.h>

namespace mpc_agf_planner
{
struct SpeedLimits
{
  double v_max;      // m/s
  double a_lat_max;  // Lateral acceleration limit in curves (m/s^2)
  double a_dec;      // Planned braking (m/s^2)
  double v_creep;    // Minimum speed so the robot always reaches the goal (m/s)
  double creep_dist = 0.0;  // Last stretch before the goal driven at v_creep (m)
  double wheelbase = 0.0;   // For the steering-rate limit (m), 0 disables it
  double steer_rate = 0.0;  // Steering motor rate (rad/s), 0 disables the limit
};

/**
 * Fills RefPoint::v along the path: curvature limit sqrt(a_lat_max/|kappa|),
 * steering-rate limit (slow enough that the wheel can turn from one sample's
 * steering angle to the next's in time: v <= steer_rate * ds / d_delta),
 * v_creep over the last creep_dist, then a backward pass so the robot has
 * braked to v_creep by the time it reaches that stretch.
 * Acceleration from rest is applied at command time, not here, so the profile
 * stays valid if the robot stops part-way along the path.
 */
void computeSpeedProfile(PathReference& path, const SpeedLimits& limits);

}  // namespace mpc_agf_planner

#endif  // MPC_AGF_PLANNER_SPEED_PROFILE_H_
