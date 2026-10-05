#include "tugger_nav/sim_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tugger_nav {

namespace {
double approach(double cur, double target, double up, double down, double dt) {
  // `up` limits growth of |v|, `down` limits shrinking of |v| (braking).
  const double diff = target - cur;
  const bool braking = std::abs(target) < std::abs(cur) || cur * target < 0.0;
  const double lim = (braking ? down : up) * dt;
  return cur + std::clamp(diff, -lim, lim);
}
}  // namespace

void stepDiffDrive(RobotState & s, const RobotParams & p, double v_cmd, double w_cmd, double dt) {
  v_cmd = std::clamp(v_cmd, -p.max_speed, p.max_speed);
  w_cmd = std::clamp(w_cmd, -p.max_ang_speed, p.max_ang_speed);
  s.v = approach(s.v, v_cmd, p.max_accel, p.max_decel, dt);
  s.w = approach(s.w, w_cmd, p.max_ang_accel, p.max_ang_accel, dt);
  if (std::abs(s.w) < 1e-9) {
    s.x += s.v * dt * std::cos(s.yaw);
    s.y += s.v * dt * std::sin(s.yaw);
  } else {
    const double r = s.v / s.w;
    const double yaw2 = s.yaw + s.w * dt;
    s.x += r * (std::sin(yaw2) - std::sin(s.yaw));
    s.y -= r * (std::cos(yaw2) - std::cos(s.yaw));
    s.yaw = yaw2;
  }
  s.yaw = std::atan2(std::sin(s.yaw), std::cos(s.yaw));
}

void Person::reset() {
  next = 1;
  triggered = false;
  wait_left = 0.0;
  if (!waypoints.empty()) {
    x = waypoints[0].x;
    y = waypoints[0].y;
  }
}

void Person::step(double dt, double robot_x, double robot_y) {
  if (!triggered) {
    if (std::hypot(robot_x - trigger_x, robot_y - trigger_y) > trigger_radius) {
      return;
    }
    triggered = true;
    wait_left = waypoints.empty() ? 0.0 : waypoints[0].wait;
  }
  double budget = dt;
  while (budget > 0.0 && !done()) {
    if (wait_left > 0.0) {
      const double w = std::min(wait_left, budget);
      wait_left -= w;
      budget -= w;
      continue;
    }
    if (next >= waypoints.size()) {
      break;
    }
    const Waypoint & wp = waypoints[next];
    const double d = std::hypot(wp.x - x, wp.y - y);
    const double reach = speed * budget;
    if (reach >= d) {
      x = wp.x;
      y = wp.y;
      budget -= speed > 0.0 ? d / speed : budget;
      wait_left = wp.wait;
      ++next;
    } else {
      x += (wp.x - x) / d * reach;
      y += (wp.y - y) / d * reach;
      budget = 0.0;
    }
  }
}

double rayCircle(double ox, double oy, double dx, double dy, double cx, double cy, double r) {
  const double fx = ox - cx;
  const double fy = oy - cy;
  const double b = fx * dx + fy * dy;
  const double c = fx * fx + fy * fy - r * r;
  const double disc = b * b - c;
  if (disc < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double sq = std::sqrt(disc);
  const double t1 = -b - sq;
  if (t1 >= 0.0) {
    return t1;
  }
  const double t2 = -b + sq;
  return t2 >= 0.0 ? 0.0 : std::numeric_limits<double>::infinity();  // origin inside circle -> 0
}

void simulateScan(const Grid & g, const std::vector<Person> & people, double sx, double sy, double syaw,
                  double angle_min, double angle_increment, int count, double range_max, double noise_sigma,
                  std::mt19937 & rng, std::vector<float> & ranges) {
  std::normal_distribution<double> noise(0.0, noise_sigma);
  ranges.assign(static_cast<size_t>(count), static_cast<float>(range_max));
  for (int i = 0; i < count; ++i) {
    const double a = syaw + angle_min + i * angle_increment;
    const double dx = std::cos(a);
    const double dy = std::sin(a);
    double r = raycast(g, sx, sy, a, range_max);
    for (const auto & pp : people) {
      r = std::min(r, rayCircle(sx, sy, dx, dy, pp.x, pp.y, pp.radius));
    }
    if (r < range_max) {
      r = std::clamp(r + (noise_sigma > 0.0 ? noise(rng) : 0.0), 0.0, range_max);
      ranges[static_cast<size_t>(i)] = static_cast<float>(r);
    } else {
      ranges[static_cast<size_t>(i)] = std::numeric_limits<float>::infinity();  // no return
    }
  }
}

double footprintToCircle(const RobotState & s, const RobotParams & p, double cx, double cy, double r) {
  const double c = std::cos(s.yaw);
  const double sn = std::sin(s.yaw);
  const double lx = c * (cx - s.x) + sn * (cy - s.y);
  const double ly = -sn * (cx - s.x) + c * (cy - s.y);
  const double hx = p.length / 2.0;
  const double hy = p.width / 2.0;
  const double qx = std::abs(lx) - hx;
  const double qy = std::abs(ly) - hy;
  const double outside = std::hypot(std::max(qx, 0.0), std::max(qy, 0.0));
  const double inside = std::min(std::max(qx, qy), 0.0);
  return outside + inside - r;
}

double footprintToStatic(const Grid & g, const std::vector<float> & dist, const RobotState & s,
                         const RobotParams & p, double spacing) {
  const double c = std::cos(s.yaw);
  const double sn = std::sin(s.yaw);
  const double hx = p.length / 2.0;
  const double hy = p.width / 2.0;
  const double corners[5][2] = {{hx, hy}, {hx, -hy}, {-hx, -hy}, {-hx, hy}, {hx, hy}};
  double best = std::numeric_limits<double>::infinity();
  for (int e = 0; e < 4; ++e) {
    const double ax = corners[e][0];
    const double ay = corners[e][1];
    const double bx = corners[e + 1][0];
    const double by = corners[e + 1][1];
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(bx - ax, by - ay) / spacing)));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      const double lx = ax + t * (bx - ax);
      const double ly = ay + t * (by - ay);
      int ix = 0;
      int iy = 0;
      g.toCell(s.x + c * lx - sn * ly, s.y + sn * lx + c * ly, ix, iy);
      if (g.occ(ix, iy)) {
        return 0.0;
      }
      best = std::min(best, static_cast<double>(dist[static_cast<size_t>(iy) * g.width + ix]));
    }
  }
  return std::max(0.0, best - g.resolution / 2.0);
}

}  // namespace tugger_nav
