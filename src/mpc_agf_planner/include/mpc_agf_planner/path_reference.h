#ifndef MPC_AGF_PLANNER_PATH_REFERENCE_H_
#define MPC_AGF_PLANNER_PATH_REFERENCE_H_

#include <cstddef>
#include <vector>

namespace mpc_agf_planner
{
struct Point2
{
  double x;
  double y;
};

// One sample of the resampled reference path
struct RefPoint
{
  double x;
  double y;
  double yaw;    // Heading the robot should have here (tangent, +pi when reversing)
  double kappa;  // Curvature along the direction of travel (1/m), left positive
  double s;      // Arc length from the path start (m)
  double v;      // Speed profile, unsigned (m/s), filled by computeSpeedProfile()
};

// Errors of a robot pose relative to the reference path
struct PathErrors
{
  double e_y;    // Lateral offset, positive = left of the reference heading (m)
  double e_psi;  // Heading error, robot yaw - reference yaw (rad)
  double s;      // Arc length of the robot's projection on the path (m)
};

/**
 * Resampled global plan with arc length, heading and curvature.
 * Replaces findPointInPath() and the erase-based pruning of the old tracker.
 */
class PathReference
{
public:
  /**
   * @param raw          Global plan positions
   * @param reverse      True when the robot drives the path backwards
   * @param ds           Resampling step (m)
   * @param fallback_yaw Heading used when the path has a single point
   * @param kappa_half_span Curvature is the tangent change over +-this arc
   *                     length (m); smaller follows tight corners more sharply,
   *                     larger filters noisy (grid) plans
   */
  void build(const std::vector<Point2>& raw, bool reverse, double ds,
             double fallback_yaw, double kappa_half_span = 0.25);

  bool empty() const { return pts_.empty(); }
  std::size_t size() const { return pts_.size(); }
  double length() const { return pts_.empty() ? 0.0 : pts_.back().s; }
  bool reverse() const { return reverse_; }

  const RefPoint& at(std::size_t i) const { return pts_[i]; }
  const RefPoint& back() const { return pts_.back(); }
  std::vector<RefPoint>& points() { return pts_; }
  const std::vector<RefPoint>& points() const { return pts_; }

  /**
   * Nearest sample to (x, y). With hint >= 0 only a window around the hint is
   * searched, so the reference cannot jump to another part of a self-crossing
   * path. hint < 0 searches the whole path.
   */
  std::size_t nearest(double x, double y, long hint, std::size_t window) const;

  // Lateral/heading errors of a pose against sample idx
  PathErrors errors(double x, double y, double yaw, std::size_t idx) const;

  // Sample interpolated at arc length s (clamped to the path)
  RefPoint sampleAtS(double s) const;

private:
  std::vector<RefPoint> pts_;
  bool reverse_ = false;
};

double normalizeAngle(double angle);

}  // namespace mpc_agf_planner

#endif  // MPC_AGF_PLANNER_PATH_REFERENCE_H_
