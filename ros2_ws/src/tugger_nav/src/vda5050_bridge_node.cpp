// vda5050_bridge: receives VDA 5050 v2 orders / instantActions as JSON on ROS topics (the payload an MQTT
// broker would carry on uagv/v2/<manufacturer>/<serial>/order), validates them against the official schema,
// executes the base with Nav2 (NavigateThroughPoses, or NavigateToPose for a single goal) and publishes the
// VDA 5050 state (also schema-checked before it is sent).
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tugger_msgs/msg/field_status.hpp>

#include "tugger_nav/vda5050_core.hpp"

using namespace std::chrono_literals;
using NTP = nav2_msgs::action::NavigateThroughPoses;
using NTPose = nav2_msgs::action::NavigateToPose;

namespace tugger_nav {

class Vda5050Bridge : public rclcpp::Node {
public:
  Vda5050Bridge() : Node("vda5050_bridge") {
    const std::string share = ament_index_cpp::get_package_share_directory("tugger_nav");
    const std::string schema_dir = declare_parameter("schema_dir", share + "/vda5050");
    manufacturer_ = declare_parameter("manufacturer", "PortfolioSim");
    serial_ = declare_parameter("serial_number", "tugger-01");
    map_id_ = declare_parameter("map_id", "warehouse");
    tolerance_ = declare_parameter("node_tolerance", 0.6);
    start_tolerance_ = declare_parameter("start_tolerance", 1.0);
    segment_min_distance_ = declare_parameter("segment_min_distance", 1.5);
    order_schema_ = std::make_unique<vda::SchemaValidator>(schema_dir + "/order.schema");
    state_schema_ = std::make_unique<vda::SchemaValidator>(schema_dir + "/state.schema");
    instant_schema_ = std::make_unique<vda::SchemaValidator>(schema_dir + "/instantActions.schema");

    state_pub_ = create_publisher<std_msgs::msg::String>("vda5050/state", rclcpp::QoS(10).reliable());
    order_sub_ = create_subscription<std_msgs::msg::String>(
      "vda5050/order", rclcpp::QoS(10).reliable(), [this](std_msgs::msg::String::SharedPtr m) { onOrder(m->data); });
    instant_sub_ = create_subscription<std_msgs::msg::String>(
      "vda5050/instantActions", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::SharedPtr m) { onInstant(m->data); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
      x_ = m->pose.pose.position.x;
      y_ = m->pose.pose.position.y;
      theta_ = 2.0 * std::atan2(m->pose.pose.orientation.z, m->pose.pose.orientation.w);
      vx_ = m->twist.twist.linear.x;
      omega_ = m->twist.twist.angular.z;
      const bool driving = std::abs(vx_) > 0.01 || std::abs(omega_) > 0.01;
      bool changed = driving != driving_;
      driving_ = driving;
      while (tracker_.updatePosition(x_, y_, tolerance_)) {
        changed = true;
      }
      if (changed) {
        publishState();
      }
    });
    field_sub_ = create_subscription<tugger_msgs::msg::FieldStatus>(
      "field_monitor/status", 10, [this](tugger_msgs::msg::FieldStatus::SharedPtr m) {
        const bool stop = m->enabled && m->state == tugger_msgs::msg::FieldStatus::PROTECTIVE_STOP;
        const bool changed = stop != protective_stop_ || m->protective_violated != field_violation_;
        protective_stop_ = stop;
        field_violation_ = m->protective_violated;
        if (changed) {
          publishState();
        }
      });
    ntp_ = rclcpp_action::create_client<NTP>(this, "navigate_through_poses");
    ntpose_ = rclcpp_action::create_client<NTPose>(this, "navigate_to_pose");
    timer_ = create_wall_timer(1s, [this] { publishState(); });
  }

private:
  void reject(const std::string & type, const std::string & what) {
    vda::Error e;
    e.type = type;
    e.description = what;
    errors_.push_back(e);
    RCLCPP_WARN(get_logger(), "%s: %s", type.c_str(), what.c_str());
    publishState();
  }

  void onOrder(const std::string & text) {
    vda::json doc;
    try {
      doc = vda::json::parse(text);
    } catch (const std::exception & e) {
      reject("validationError", std::string("order is not JSON: ") + e.what());
      return;
    }
    const std::string why = order_schema_->validate(doc);
    if (!why.empty()) {
      reject("validationError", "order does not match the VDA 5050 schema: " + why);
      return;
    }
    vda::Order order;
    std::vector<vda::Error> errs = vda::parseOrder(doc, map_id_, order);
    if (errs.empty()) {
      const auto d = tracker_.submit(order, x_, y_, start_tolerance_, errs);
      if (d == vda::OrderTracker::Decision::Ignored) {
        return;
      }
      if (d == vda::OrderTracker::Decision::Accepted || d == vda::OrderTracker::Decision::Updated) {
        errors_.clear();
        RCLCPP_INFO(get_logger(), "order %s/%ld %s, %zu base nodes to go", order.order_id.c_str(),
                    static_cast<long>(order.update_id), d == vda::OrderTracker::Decision::Accepted ? "accepted" : "updated",
                    tracker_.remainingBase().size());
        sendGoal();
        publishState();
        return;
      }
    }
    for (const auto & e : errs) {
      errors_.push_back(e);
      RCLCPP_WARN(get_logger(), "%s: %s", e.type.c_str(), e.description.c_str());
    }
    publishState();
  }

  void onInstant(const std::string & text) {
    vda::json doc;
    try {
      doc = vda::json::parse(text);
    } catch (const std::exception & e) {
      reject("validationError", std::string("instantActions is not JSON: ") + e.what());
      return;
    }
    const std::string why = instant_schema_->validate(doc);
    if (!why.empty()) {
      reject("validationError", "instantActions does not match the VDA 5050 schema: " + why);
      return;
    }
    for (const auto & a : doc.at("actions")) {
      if (a.at("actionType") == "cancelOrder") {
        ++goal_generation_;  // results of the cancelled goal are ignored
        ntp_->async_cancel_all_goals();
        ntpose_->async_cancel_all_goals();
        tracker_.cancel();
        RCLCPP_INFO(get_logger(), "cancelOrder");
      } else {
        reject("unsupportedActionError", "instant action " + a.at("actionType").get<std::string>() + " is not supported");
      }
    }
    publishState();
  }

  geometry_msgs::msg::PoseStamped poseOf(const vda::Node & n, double px, double py) const {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    p.pose.position.x = n.x;
    p.pose.position.y = n.y;
    // No theta in the order: face along the edge that leads to the node.
    const double yaw = n.theta ? *n.theta : std::atan2(n.y - py, n.x - px);
    p.pose.orientation.z = std::sin(yaw / 2.0);
    p.pose.orientation.w = std::cos(yaw / 2.0);
    return p;
  }

  void sendGoal() {
    const auto base = tracker_.remainingBase();
    if (base.empty()) {
      return;
    }
    const uint64_t gen = ++goal_generation_;
    double px = x_;
    double py = y_;
    const size_t len = vda::segmentLength(base, x_, y_, segment_min_distance_);
    segment_end_seq_ = base[len - 1].seq;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (size_t i = 0; i < len; ++i) {
      const vda::Node & n = base[i];
      poses.push_back(poseOf(n, px, py));
      px = n.x;
      py = n.y;
    }
    if (poses.size() == 1) {
      if (!ntpose_->wait_for_action_server(2s)) {
        reject("navigationError", "navigate_to_pose action server not available");
        return;
      }
      NTPose::Goal g;
      g.pose = poses.front();
      rclcpp_action::Client<NTPose>::SendGoalOptions o;
      o.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<NTPose>::WrappedResult & r) {
        onResult(gen, r.code);
      };
      ntpose_->async_send_goal(g, o);
    } else {
      if (!ntp_->wait_for_action_server(2s)) {
        reject("navigationError", "navigate_through_poses action server not available");
        return;
      }
      NTP::Goal g;
      g.poses = poses;
      rclcpp_action::Client<NTP>::SendGoalOptions o;
      o.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<NTP>::WrappedResult & r) {
        onResult(gen, r.code);
      };
      ntp_->async_send_goal(g, o);
    }
  }

  void onResult(uint64_t gen, rclcpp_action::ResultCode code) {
    if (gen != goal_generation_) {
      return;  // a newer goal replaced this one
    }
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      const bool at_end = tracker_.traverseThrough(segment_end_seq_, x_, y_, tolerance_) ||
                          tracker_.lastNodeSeq() >= segment_end_seq_;
      if (!at_end) {
        reject("navigationError", "Nav2 reported success but the segment end node was not reached");
      } else if (tracker_.active()) {
        sendGoal();  // next segment of a closed loop
      }
    } else if (code == rclcpp_action::ResultCode::ABORTED) {
      reject("navigationError", "Nav2 aborted the navigation goal");
    }
    publishState();
  }

  void publishState() {
    vda::StateInputs in;
    in.header_id = header_id_++;
    in.timestamp = vda::isoTimestampNow();
    in.manufacturer = manufacturer_;
    in.serial_number = serial_;
    in.map_id = map_id_;
    in.x = x_;
    in.y = y_;
    in.theta = theta_;
    in.vx = vx_;
    in.omega = omega_;
    in.driving = driving_;
    in.protective_stop = protective_stop_;
    in.field_violation = field_violation_;
    in.errors = errors_;
    const vda::json state = vda::buildState(tracker_, in);
    const std::string why = state_schema_->validate(state);
    if (!why.empty()) {
      RCLCPP_ERROR(get_logger(), "state does not match the VDA 5050 schema: %s", why.c_str());
    }
    std_msgs::msg::String m;
    m.data = state.dump();
    state_pub_->publish(m);
  }

  std::unique_ptr<vda::SchemaValidator> order_schema_, state_schema_, instant_schema_;
  std::string manufacturer_, serial_, map_id_;
  double tolerance_ = 0.6, start_tolerance_ = 1.0, segment_min_distance_ = 1.5;
  int64_t segment_end_seq_ = 0;
  vda::OrderTracker tracker_;
  std::vector<vda::Error> errors_;
  int64_t header_id_ = 0;
  uint64_t goal_generation_ = 0;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0, vx_ = 0.0, omega_ = 0.0;
  bool driving_ = false, protective_stop_ = false, field_violation_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_sub_, instant_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tugger_msgs::msg::FieldStatus>::SharedPtr field_sub_;
  rclcpp_action::Client<NTP>::SharedPtr ntp_;
  rclcpp_action::Client<NTPose>::SharedPtr ntpose_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tugger_nav

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tugger_nav::Vda5050Bridge>());
  rclcpp::shutdown();
  return 0;
}
