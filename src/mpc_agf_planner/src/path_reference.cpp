#include <mpc_agf_planner/path_reference.h>

#include <algorithm>
#include <cmath>

namespace mpc_agf_planner
{
double normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }
  return angle;
}

void PathReference::build(const std::vector<Point2>& raw, bool reverse,
                          double ds, double fallback_yaw,
                          double kappa_half_span)
{
  pts_.clear();
  reverse_ = reverse;

  // Drop consecutive duplicates
  std::vector<Point2> p;
  for (const auto& q : raw)
  {
    if (p.empty() || std::hypot(q.x - p.back().x, q.y - p.back().y) > 1e-6)
    {
      p.push_back(q);
    }
  }
  if (p.empty())
  {
    return;
  }

  // Tangent at each plan vertex: bisector of its two segments. Interpolating
  // these along each segment keeps heading and curvature smooth even when the
  // plan's vertices are sparse (resampled points on a straight chord would
  // otherwise all get the chord's direction).
  const std::size_t nv = p.size();
  std::vector<double> yaw_vertex(nv, fallback_yaw);
  if (nv > 1)
  {
    for (std::size_t j = 0; j < nv; ++j)
    {
      double ux = 0.0, uy = 0.0;
      if (j > 0)
      {
        double l = std::hypot(p[j].x - p[j - 1].x, p[j].y - p[j - 1].y);
        ux += (p[j].x - p[j - 1].x) / l;
        uy += (p[j].y - p[j - 1].y) / l;
      }
      if (j + 1 < nv)
      {
        double l = std::hypot(p[j + 1].x - p[j].x, p[j + 1].y - p[j].y);
        ux += (p[j + 1].x - p[j].x) / l;
        uy += (p[j + 1].y - p[j].y) / l;
      }
      if (std::hypot(ux, uy) < 1e-9)
      {
        // 180 degree cusp: keep the incoming direction
        ux = p[j].x - p[j - 1].x;
        uy = p[j].y - p[j - 1].y;
      }
      yaw_vertex[j] = std::atan2(uy, ux);
    }
  }
  else if (reverse)
  {
    // fallback_yaw is already a robot heading; undo the flip applied below
    yaw_vertex[0] = fallback_yaw + M_PI;
  }

  // Resample at a fixed step ds, keeping the exact final point
  std::vector<Point2> r;
  std::vector<double> s;
  std::vector<double> yaw_tan;
  r.push_back(p[0]);
  s.push_back(0.0);
  yaw_tan.push_back(yaw_vertex[0]);
  double carry = 0.0;  // distance travelled since the last emitted sample
  double total = 0.0;
  for (std::size_t i = 1; i < nv; ++i)
  {
    double dx = p[i].x - p[i - 1].x;
    double dy = p[i].y - p[i - 1].y;
    double seg = std::hypot(dx, dy);
    double turn = normalizeAngle(yaw_vertex[i] - yaw_vertex[i - 1]);
    double pos = ds - carry;  // position of the next sample on this segment
    while (pos <= seg)
    {
      double t = pos / seg;
      r.push_back({p[i - 1].x + t * dx, p[i - 1].y + t * dy});
      s.push_back(total + pos);
      yaw_tan.push_back(normalizeAngle(yaw_vertex[i - 1] + t * turn));
      pos += ds;
    }
    carry = seg - (pos - ds);
    total += seg;
  }
  if (std::hypot(r.back().x - p.back().x, r.back().y - p.back().y) > 1e-6)
  {
    r.push_back(p.back());
    s.push_back(total);
    yaw_tan.push_back(yaw_vertex[nv - 1]);
  }

  const std::size_t n = r.size();
  pts_.resize(n);

  // Curvature = tangent change over +-half_span of arc, which also filters
  // the noise of grid-based global plans
  const double half_span = std::max(ds, kappa_half_span);
  const std::size_t m =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::round(half_span / ds)));
  std::vector<double> kappa_smooth(n, 0.0);
  for (std::size_t i = 0; i < n && n > 2; ++i)
  {
    std::size_t a = (i < m) ? 0 : i - m;
    std::size_t b = std::min(n - 1, i + m);
    double dsum = s[b] - s[a];
    if (dsum > 1e-9)
    {
      kappa_smooth[i] = normalizeAngle(yaw_tan[b] - yaw_tan[a]) / dsum;
    }
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    pts_[i].x = r[i].x;
    pts_[i].y = r[i].y;
    pts_[i].yaw = normalizeAngle(reverse ? yaw_tan[i] + M_PI : yaw_tan[i]);
    pts_[i].kappa = kappa_smooth[i];
    pts_[i].s = s[i];
    pts_[i].v = 0.0;
  }
}

std::size_t PathReference::nearest(double x, double y, long hint,
                                   std::size_t window) const
{
  if (pts_.empty())
  {
    return 0;
  }
  std::size_t from = 0;
  std::size_t to = pts_.size();
  if (hint >= 0)
  {
    std::size_t h = std::min(static_cast<std::size_t>(hint), pts_.size() - 1);
    std::size_t back = std::min<std::size_t>(5, h);
    from = h - back;
    to = std::min(pts_.size(), h + window + 1);
  }
  std::size_t best = from;
  double best_d = 1e18;
  for (std::size_t i = from; i < to; ++i)
  {
    double dx = pts_[i].x - x;
    double dy = pts_[i].y - y;
    double d = dx * dx + dy * dy;
    if (d < best_d)
    {
      best_d = d;
      best = i;
    }
  }
  return best;
}

PathErrors PathReference::errors(double x, double y, double yaw,
                                 std::size_t idx) const
{
  const RefPoint& r = pts_[idx];
  double dx = x - r.x;
  double dy = y - r.y;
  double c = std::cos(r.yaw);
  double sn = std::sin(r.yaw);
  PathErrors e;
  e.e_y = -sn * dx + c * dy;
  e.e_psi = normalizeAngle(yaw - r.yaw);
  // Along-track offset; the heading points against the path when reversing
  double along = c * dx + sn * dy;
  if (reverse_)
  {
    along = -along;
  }
  e.s = std::max(0.0, std::min(length(), r.s + along));
  return e;
}

RefPoint PathReference::sampleAtS(double s) const
{
  if (pts_.size() == 1 || s <= 0.0)
  {
    return pts_.front();
  }
  if (s >= length())
  {
    return pts_.back();
  }
  auto it = std::upper_bound(
      pts_.begin(), pts_.end(), s,
      [](double value, const RefPoint& p) { return value < p.s; });
  std::size_t hi = static_cast<std::size_t>(it - pts_.begin());
  std::size_t lo = hi - 1;
  const RefPoint& a = pts_[lo];
  const RefPoint& b = pts_[hi];
  double t = (s - a.s) / std::max(1e-9, b.s - a.s);
  RefPoint out = a;
  out.x = a.x + t * (b.x - a.x);
  out.y = a.y + t * (b.y - a.y);
  out.yaw = normalizeAngle(a.yaw + t * normalizeAngle(b.yaw - a.yaw));
  out.kappa = a.kappa + t * (b.kappa - a.kappa);
  out.v = a.v + t * (b.v - a.v);
  out.s = s;
  return out;
}

}  // namespace mpc_agf_planner
