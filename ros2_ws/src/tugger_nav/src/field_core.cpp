#include "tugger_nav/field_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tugger_nav {

void FieldConfig::validate() const {
  auto pos = [](double v, const char * name) {
    if (!(v > 0.0) || !std::isfinite(v)) {
      throw std::invalid_argument(std::string(name) + " must be > 0");
    }
  };
  auto nonneg = [](double v, const char * name) {
    if (!(v >= 0.0) || !std::isfinite(v)) {
      throw std::invalid_argument(std::string(name) + " must be >= 0");
    }
  };
  pos(robot_length, "robot_length");
  pos(robot_width, "robot_width");
  pos(a_brake, "a_brake");
  pos(hold_time, "hold_time");
  pos(scan_timeout, "scan_timeout");
  nonneg(t_response, "t_response");
  nonneg(k_approach, "k_approach");
  nonneg(c_margin, "c_margin");
  nonneg(lateral_margin, "lateral_margin");
  nonneg(warning_time, "warning_time");
  nonneg(warning_margin, "warning_margin");
  nonneg(warning_lateral, "warning_lateral");
  nonneg(warning_speed, "warning_speed");
  if (speed_bands.empty()) {
    throw std::invalid_argument("speed_bands must not be empty");
  }
  for (size_t i = 0; i < speed_bands.size(); ++i) {
    pos(speed_bands[i], "speed_bands");
    if (i > 0 && speed_bands[i] <= speed_bands[i - 1]) {
      throw std::invalid_argument("speed_bands must be strictly increasing");
    }
  }
}

double stoppingTime(const FieldConfig & c, double v) {
  v = std::abs(v);
  return c.t_response + v / c.a_brake;
}

double protectiveLength(const FieldConfig & c, double v) {
  v = std::abs(v);
  return v * c.t_response + v * v / (2.0 * c.a_brake) + c.k_approach * stoppingTime(c, v) + c.c_margin;
}

double warningLength(const FieldConfig & c, double v) {
  return protectiveLength(c, v) + std::abs(v) * c.warning_time + c.warning_margin;
}

int bandIndex(const FieldConfig & c, double speed) {
  speed = std::abs(speed);
  for (size_t i = 0; i < c.speed_bands.size(); ++i) {
    if (speed <= c.speed_bands[i] + 1e-9) {
      return static_cast<int>(i);
    }
  }
  return static_cast<int>(c.speed_bands.size()) - 1;
}

FieldSet selectFieldSet(const FieldConfig & c, double speed) {
  FieldSet fs;
  fs.index = bandIndex(c, speed);
  fs.design_speed = c.speed_bands[static_cast<size_t>(fs.index)];
  const double front = c.robot_length / 2.0;
  fs.protective.x0 = front;
  fs.protective.x1 = front + protectiveLength(c, fs.design_speed);
  fs.protective.half_width = c.robot_width / 2.0 + c.lateral_margin;
  fs.warning.x0 = front;
  fs.warning.x1 = front + warningLength(c, fs.design_speed);
  fs.warning.half_width = fs.protective.half_width + c.warning_lateral;
  return fs;
}

ScanResult evaluateScan(const FieldSet & fs, const std::vector<float> & ranges, double angle_min,
                        double angle_increment, double range_min, double range_max, double scanner_x) {
  ScanResult r;
  double best_p = std::numeric_limits<double>::infinity();
  double best_w = std::numeric_limits<double>::infinity();
  double wx = 0.0;
  double wy = 0.0;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double d = ranges[i];
    if (!std::isfinite(d) || d < range_min || d > range_max) {
      continue;
    }
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    const double x = scanner_x + d * std::cos(a);
    const double y = d * std::sin(a);
    if (!fs.warning.contains(x, y)) {  // the warning field contains the protective field
      continue;
    }
    const double dist = x - fs.protective.x0;
    if (fs.protective.contains(x, y)) {
      r.protective = true;
      if (dist < best_p) {
        best_p = dist;
        r.nearest_x = x;
        r.nearest_y = y;
      }
    } else {
      r.warning = true;
      if (dist < best_w) {
        best_w = dist;
        wx = x;
        wy = y;
      }
    }
  }
  if (r.protective) {
    r.warning = true;  // a protective violation is also a warning violation
  } else if (r.warning) {
    r.nearest_x = wx;
    r.nearest_y = wy;
  }
  return r;
}

void FieldStateMachine::enter(MonitorState s) {
  if (s == state_) {
    return;
  }
  if (s == MonitorState::ProtectiveStop) {
    ++stops_;
  } else if (s == MonitorState::Warning) {
    ++warnings_;
  }
  state_ = s;
}

MonitorState FieldStateMachine::onScan(double t, bool protective_violated, bool warning_violated) {
  if (!have_scan_) {
    have_scan_ = true;
    stops_ = 0;  // the initial stopped state is not counted as a protective stop
  }
  last_scan_t_ = t;
  timed_out_ = false;
  warning_now_ = warning_violated || protective_violated;
  if (protective_violated) {
    last_violation_t_ = t;
    enter(MonitorState::ProtectiveStop);
    return state_;
  }
  if (state_ == MonitorState::ProtectiveStop && t - last_violation_t_ < cfg_.hold_time) {
    return state_;  // still holding
  }
  enter(warning_now_ ? MonitorState::Warning : MonitorState::Clear);
  return state_;
}

MonitorState FieldStateMachine::onTick(double t) {
  if (t - last_scan_t_ > cfg_.scan_timeout) {
    timed_out_ = true;
    last_violation_t_ = t;  // a timeout also needs the full hold time after scans return
    if (have_scan_) {
      enter(MonitorState::ProtectiveStop);
    }
  }
  return state_;
}

void FieldStateMachine::gate(double & v, double & w) const {
  if (v < 0.0) {
    v = 0.0;  // no rear scanner in this setup: reversing is not allowed through the monitor
  }
  if (state_ == MonitorState::ProtectiveStop) {
    v = 0.0;
    w = 0.0;
  } else if (state_ == MonitorState::Warning && v > cfg_.warning_speed) {
    const double s = cfg_.warning_speed / v;
    v *= s;
    w *= s;
  }
}

const char * toString(MonitorState s) {
  switch (s) {
    case MonitorState::Clear: return "CLEAR";
    case MonitorState::Warning: return "WARNING";
    case MonitorState::ProtectiveStop: return "PROTECTIVE_STOP";
  }
  return "?";
}

}  // namespace tugger_nav
