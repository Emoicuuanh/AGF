#ifndef MPC_AGF_PLANNER_STEERING_GEOMETRY_H_
#define MPC_AGF_PLANNER_STEERING_GEOMETRY_H_

#include <algorithm>
#include <cmath>

namespace mpc_agf_planner
{
/**
 * Ackermann steering with the angle measured and commanded at the LEFT wheel.
 *
 * The MPC works with the virtual centre-wheel angle (bicycle model), so the
 * planner converts /steering_angle to the centre angle on the way in and the
 * MPC's centre command back to a left-wheel angle on the way out.
 *
 * With phi = steer_sign * angle (positive = turning left) and k = W / (2 L):
 *   tan(phi_left)   = tan(phi_centre) / (1 - k tan(phi_centre))
 *   tan(phi_centre) = tan(phi_left)   / (1 + k tan(phi_left))
 * Turning left, the left wheel is the inner wheel and has the larger angle.
 * The atan2 forms stay valid when the inner wheel passes 90 degrees.
 * track_width = 0 makes both conversions the identity (single centre wheel).
 */
struct SteeringGeometry
{
  double wheelbase = 1.2;    // Fixed axle to steered axle (m)
  double track_width = 0.0;  // Distance between the two steering pivots (m)
  double steer_sign = 1.0;   // +1: positive angle turns left when driving forward

  double k() const { return track_width / (2.0 * wheelbase); }

  double leftToCentre(double left) const
  {
    double phi = steer_sign * left;
    double c = std::atan2(std::sin(phi), std::cos(phi) + k() * std::sin(phi));
    return steer_sign * c;
  }

  double centreToLeft(double centre) const
  {
    double phi = steer_sign * centre;
    double l = std::atan2(std::sin(phi), std::cos(phi) - k() * std::sin(phi));
    return steer_sign * l;
  }

  /**
   * Largest centre angle (both directions) that keeps the inner wheel within
   * max_wheel. Turning left the left wheel is inner; turning right the right
   * wheel is, and by symmetry it reaches max_wheel at the same centre angle.
   */
  double maxCentreAngle(double max_wheel) const
  {
    double m = std::fabs(max_wheel);
    return std::fabs(std::atan2(std::sin(m), std::cos(m) + k() * std::sin(m)));
  }
};

}  // namespace mpc_agf_planner

#endif  // MPC_AGF_PLANNER_STEERING_GEOMETRY_H_
