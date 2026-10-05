// protective_field_monitor: sits between the Nav2 controller (cmd_vel_nav) and the base (cmd_vel).
// Selects a protective / warning field set from the speed, checks every scan against it, and gates the
// velocity command: slow down in the warning field, stop in the protective field, restart only after the
// protective field has been clear for hold_time. With enabled:=false it only observes (pass-through).
// NOT a certified safety function (see field_core.hpp).
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tugger_msgs/msg/field_status.hpp>

#include "tugger_nav/field_core.hpp"

using namespace std::chrono_literals;

namespace tugger_nav {

class ProtectiveFieldMonitor : public rclcpp::Node {
public:
  ProtectiveFieldMonitor() : Node("protective_field_monitor") {
    enabled_ = declare_parameter("enabled", true);
    cfg_.robot_length = declare_parameter("robot_length", cfg_.robot_length);
    cfg_.robot_width = declare_parameter("robot_width", cfg_.robot_width);
    cfg_.t_response = declare_parameter("t_response", cfg_.t_response);
    cfg_.a_brake = declare_parameter("a_brake", cfg_.a_brake);
    cfg_.k_approach = declare_parameter("k_approach", cfg_.k_approach);
    cfg_.c_margin = declare_parameter("c_margin", cfg_.c_margin);
    cfg_.lateral_margin = declare_parameter("lateral_margin", cfg_.lateral_margin);
    cfg_.warning_time = declare_parameter("warning_time", cfg_.warning_time);
    cfg_.warning_margin = declare_parameter("warning_margin", cfg_.warning_margin);
    cfg_.warning_lateral = declare_parameter("warning_lateral", cfg_.warning_lateral);
    cfg_.warning_speed = declare_parameter("warning_speed", cfg_.warning_speed);
    cfg_.hold_time = declare_parameter("hold_time", cfg_.hold_time);
    cfg_.scan_timeout = declare_parameter("scan_timeout", cfg_.scan_timeout);
    cfg_.speed_bands = declare_parameter("speed_bands", cfg_.speed_bands);
    scanner_x_ = declare_parameter("scanner_x", 0.5);
    cfg_.validate();  // throws on a bad configuration: the node does not start
    sm_ = std::make_unique<FieldStateMachine>(cfg_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ = create_publisher<tugger_msgs::msg::FieldStatus>("field_monitor/status", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr m) { onScan(*m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
      v_meas_ = m->twist.twist.linear.x;
      ox_ = m->pose.pose.position.x;
      oy_ = m->pose.pose.position.y;
      oyaw_ = 2.0 * std::atan2(m->pose.pose.orientation.z, m->pose.pose.orientation.w);
    });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) { onCmd(*m); });
    timer_ = create_wall_timer(50ms, [this] { onTick(); });
    RCLCPP_INFO(get_logger(), "enabled=%s, field set 0: protective %.2f m, top: protective %.2f m / warning %.2f m",
                enabled_ ? "true" : "false", selectFieldSet(cfg_, 0.0).protective.length(),
                selectFieldSet(cfg_, 1e9).protective.length(), selectFieldSet(cfg_, 1e9).warning.length());
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan & scan) {
    const auto t0 = std::chrono::steady_clock::now();
    const double sel = std::max(std::abs(v_meas_), std::max(0.0, v_cmd_));
    const FieldSet fs = selectFieldSet(cfg_, sel);
    const ScanResult r = evaluateScan(fs, scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                                      scan.range_max, scanner_x_);
    const MonitorState before = sm_->state();
    const MonitorState now = sm_->onScan(this->now().seconds(), r.protective, r.warning);
    if (r.protective && !prev_protective_) {
      ++intrusions_;
    }
    prev_protective_ = r.protective;
    const double eval_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    if (enabled_ && now == MonitorState::ProtectiveStop && before != MonitorState::ProtectiveStop) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());  // stop immediately, do not wait for the next command
    }
    tugger_msgs::msg::FieldStatus st;
    st.header.stamp = scan.header.stamp;
    st.header.frame_id = "odom";
    st.enabled = enabled_;
    st.state = static_cast<uint8_t>(now);
    st.field_set = static_cast<uint8_t>(fs.index);
    st.selection_speed = sel;
    st.protective_length = fs.protective.length();
    st.protective_width = 2.0 * fs.protective.half_width;
    st.warning_length = fs.warning.length();
    st.warning_width = 2.0 * fs.warning.half_width;
    st.protective_violated = r.protective;
    st.warning_violated = r.warning;
    const double c = std::cos(oyaw_);
    const double s = std::sin(oyaw_);
    st.nearest_violation.x = ox_ + c * r.nearest_x - s * r.nearest_y;
    st.nearest_violation.y = oy_ + s * r.nearest_x + c * r.nearest_y;
    st.protective_stops = sm_->protectiveStops();
    st.warning_entries = sm_->warningEntries();
    st.protective_intrusions = intrusions_;
    st.eval_time_us = eval_us;
    status_pub_->publish(st);
  }

  void onCmd(const geometry_msgs::msg::Twist & in) {
    v_cmd_ = in.linear.x;
    geometry_msgs::msg::Twist out = in;
    if (enabled_) {
      sm_->gate(out.linear.x, out.angular.z);
    }
    cmd_pub_->publish(out);
  }

  void onTick() {
    const MonitorState before = sm_->state();
    const MonitorState now = sm_->onTick(this->now().seconds());
    if (enabled_ && now == MonitorState::ProtectiveStop) {
      if (before != MonitorState::ProtectiveStop) {
        RCLCPP_WARN(get_logger(), "no scan for %.2f s: protective stop", cfg_.scan_timeout);
      }
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  FieldConfig cfg_;
  bool enabled_ = true;
  double scanner_x_ = 0.5;
  std::unique_ptr<FieldStateMachine> sm_;
  double v_meas_ = 0.0, v_cmd_ = 0.0, ox_ = 0.0, oy_ = 0.0, oyaw_ = 0.0;
  bool prev_protective_ = false;
  uint32_t intrusions_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<tugger_msgs::msg::FieldStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tugger_nav

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tugger_nav::ProtectiveFieldMonitor>());
  rclcpp::shutdown();
  return 0;
}
