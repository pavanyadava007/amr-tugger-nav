// Speed-dependent protective / warning fields and the monitor state machine.
//
// This mimics how an AMR safety laser scanner is configured (field sets switched by speed), and sizes the
// protective field with stopping-distance reasoning in the spirit of ISO 13855 (S = K * T + C):
//   T_stop = t_response + v / a_brake                      (time until standstill)
//   L_prot = v * t_response + v^2 / (2 a_brake)            (robot travel until standstill)
//          + k_approach * T_stop                           (person approaching during that time)
//          + c_margin                                      (measurement tolerance / supplement)
// It is NOT a certified safety function: it runs in ordinary ROS 2 software on a non-safe computer.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace tugger_nav {

struct FieldConfig {
  double robot_length = 1.0;          // footprint, base_link at the centre [m]
  double robot_width = 0.7;
  double t_response = 0.2;            // scanner + software + drive response time [s]
  double a_brake = 0.8;               // assumed (conservative) braking deceleration [m/s^2]
  double k_approach = 0.0;            // assumed approach speed of a person toward the robot [m/s]
  double c_margin = 0.15;             // supplement for measurement error [m]
  double lateral_margin = 0.10;       // protective field half-width = robot_width / 2 + lateral_margin
  double warning_time = 1.0;          // warning field = protective + v * warning_time + warning_margin
  double warning_margin = 0.5;
  double warning_lateral = 0.30;      // extra half-width of the warning field
  double warning_speed = 0.3;         // speed cap inside the warning state [m/s]
  double hold_time = 1.0;             // protective field must be clear this long before restart [s]
  double scan_timeout = 0.3;          // no scan for this long -> protective stop (fail safe) [s]
  std::vector<double> speed_bands = {0.2, 0.4, 0.6, 0.8, 1.0, 1.2};  // upper speed of each field set [m/s]

  // Throws std::invalid_argument with a reason when a value is not usable.
  void validate() const;
};

// A rectangle ahead of the robot, in base_link: x in [x0, x1], |y| <= half_width.
struct Field {
  double x0 = 0.0;
  double x1 = 0.0;
  double half_width = 0.0;
  bool contains(double x, double y) const { return x >= x0 && x <= x1 && y >= -half_width && y <= half_width; }
  double length() const { return x1 - x0; }
};

struct FieldSet {
  int index = 0;
  double design_speed = 0.0;  // upper speed of the band the fields are sized for
  Field protective;
  Field warning;
};

double stoppingTime(const FieldConfig & c, double v);
double protectiveLength(const FieldConfig & c, double v);
double warningLength(const FieldConfig & c, double v);

// Field set for a speed: the smallest band whose upper speed is >= |speed| (top band above that).
FieldSet selectFieldSet(const FieldConfig & c, double speed);

// Index of the speed band for |speed|.
int bandIndex(const FieldConfig & c, double speed);

struct ScanResult {
  bool protective = false;
  bool warning = false;
  double nearest_x = 0.0;  // nearest violating point (base_link), protective first, then warning
  double nearest_y = 0.0;
};

// Check scan points against the field set. Scanner pose in base_link: (sx, 0) facing forward.
// Invalid ranges (NaN, inf, outside [range_min, range_max]) are ignored.
ScanResult evaluateScan(const FieldSet & fs, const std::vector<float> & ranges, double angle_min,
                        double angle_increment, double range_min, double range_max, double scanner_x);

enum class MonitorState : uint8_t { Clear = 0, Warning = 1, ProtectiveStop = 2 };

// Clear / Warning / ProtectiveStop with restart only after the protective field has been clear for hold_time.
class FieldStateMachine {
public:
  explicit FieldStateMachine(const FieldConfig & c) : cfg_(c) {}
  // Feed the result of one scan evaluation at time t [s].
  MonitorState onScan(double t, bool protective_violated, bool warning_violated);
  // Called periodically; enters ProtectiveStop if no scan has arrived for scan_timeout.
  MonitorState onTick(double t);
  // Gate a velocity command (v, w) according to the current state.
  void gate(double & v, double & w) const;
  MonitorState state() const { return state_; }
  uint32_t protectiveStops() const { return stops_; }
  uint32_t warningEntries() const { return warnings_; }
  bool scanTimedOut() const { return timed_out_; }

private:
  void enter(MonitorState s);
  FieldConfig cfg_;
  MonitorState state_ = MonitorState::ProtectiveStop;  // start stopped until the first clear scans
  double last_violation_t_ = -1e9;
  double last_scan_t_ = -1e9;
  bool have_scan_ = false;
  bool timed_out_ = false;
  bool warning_now_ = false;
  uint32_t stops_ = 0;
  uint32_t warnings_ = 0;
};

const char * toString(MonitorState s);

}  // namespace tugger_nav
