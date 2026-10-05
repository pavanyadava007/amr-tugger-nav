// Lightweight warehouse simulator: diff-drive robot driven by /cmd_vel, /odom + TF, ray-cast /scan against the
// map from map_server plus scripted people, /clock (the simulator owns time), and ground-truth metrics.
#include <atomic>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

#include "tugger_nav/sim_core.hpp"

using nlohmann::json;
using namespace std::chrono_literals;

namespace tugger_nav {

class WarehouseSim : public rclcpp::Node {
public:
  WarehouseSim() : Node("warehouse_sim") {
    rtf_ = declare_parameter("real_time_factor", 1.0);
    dt_ = declare_parameter("step", 0.01);
    scan_period_ = 1.0 / declare_parameter("scan_rate", 15.0);
    odom_period_ = 1.0 / declare_parameter("odom_rate", 50.0);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);
    rp_.length = declare_parameter("robot_length", 1.0);
    rp_.width = declare_parameter("robot_width", 0.7);
    rp_.max_accel = declare_parameter("max_accel", 0.8);
    rp_.max_decel = declare_parameter("max_decel", 1.0);
    rp_.max_ang_accel = declare_parameter("max_ang_accel", 2.0);
    scanner_x_ = declare_parameter("scanner_x", 0.5);
    fov_ = declare_parameter("scan_fov", 270.0 * M_PI / 180.0);
    beams_ = static_cast<int>(declare_parameter("scan_beams", 541));
    range_max_ = declare_parameter("scan_range_max", 20.0);
    noise_ = declare_parameter("scan_noise", 0.01);
    rng_.seed(static_cast<unsigned>(declare_parameter("seed", 1)));
    robot_.x = declare_parameter("start_x", 6.0);
    robot_.y = declare_parameter("start_y", 6.0);
    robot_.yaw = declare_parameter("start_yaw", 0.0);

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    gt_pub_ = create_publisher<std_msgs::msg::String>("sim/ground_truth", 10);
    metrics_pub_ = create_publisher<std_msgs::msg::String>("sim/metrics", rclcpp::QoS(10).reliable());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) {
      std::lock_guard<std::mutex> l(mu_);
      v_cmd_ = m->linear.x;
      w_cmd_ = m->angular.z;
      last_cmd_t_ = t_;
    });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(), [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { onMap(*m); });
    scenario_sub_ = create_subscription<std_msgs::msg::String>(
      "sim/scenario", rclcpp::QoS(10).reliable(), [this](std_msgs::msg::String::SharedPtr m) { onScenario(m->data); });

    resetMetrics("startup");
    thread_ = std::thread([this] { loop(); });
  }

  ~WarehouseSim() override {
    stop_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid & m) {
    Grid g;
    g.width = static_cast<int>(m.info.width);
    g.height = static_cast<int>(m.info.height);
    g.resolution = m.info.resolution;
    g.origin_x = m.info.origin.position.x;
    g.origin_y = m.info.origin.position.y;
    g.occupied.resize(m.data.size());
    for (size_t i = 0; i < m.data.size(); ++i) {
      g.occupied[i] = m.data[i] >= 65 || m.data[i] < 0 ? 1 : 0;
    }
    auto dist = distanceTransform(g);
    std::lock_guard<std::mutex> l(mu_);
    grid_ = std::move(g);
    dist_ = std::move(dist);
    have_map_ = true;
    RCLCPP_INFO(get_logger(), "map %dx%d at %.3f m", grid_.width, grid_.height, grid_.resolution);
  }

  void onScenario(const std::string & text) {
    json j;
    try {
      j = json::parse(text);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "bad scenario: %s", e.what());
      return;
    }
    std::vector<Person> people;
    for (const auto & jp : j.value("people", json::array())) {
      Person p;
      for (const auto & w : jp.at("waypoints")) {
        p.waypoints.push_back({w.at(0).get<double>(), w.at(1).get<double>(), w.size() > 2 ? w.at(2).get<double>() : 0.0});
      }
      p.speed = jp.value("speed", 1.2);
      p.radius = jp.value("radius", 0.25);
      const auto & tr = jp.at("trigger");
      p.trigger_x = tr.at(0).get<double>();
      p.trigger_y = tr.at(1).get<double>();
      p.trigger_radius = tr.at(2).get<double>();
      p.reset();
      people.push_back(p);
    }
    std::lock_guard<std::mutex> l(mu_);
    people_ = people;
    if (j.contains("robot")) {
      robot_ = RobotState{};
      robot_.x = j["robot"].at("x").get<double>();
      robot_.y = j["robot"].at("y").get<double>();
      robot_.yaw = j["robot"].at("yaw").get<double>();
      v_cmd_ = 0.0;
      w_cmd_ = 0.0;
    }
    resetMetricsLocked(j.value("id", std::string("unnamed")));
    RCLCPP_INFO(get_logger(), "scenario '%s' with %zu people", scenario_id_.c_str(), people_.size());
  }

  void resetMetrics(const std::string & id) {
    std::lock_guard<std::mutex> l(mu_);
    resetMetricsLocked(id);
  }
  void resetMetricsLocked(const std::string & id) {
    scenario_id_ = id;
    min_people_ = std::numeric_limits<double>::infinity();
    min_static_ = std::numeric_limits<double>::infinity();
    people_contacts_ = 0;
    static_contacts_ = 0;
    contact_log_ = json::array();
    in_people_contact_ = false;
    in_static_contact_ = false;
    distance_ = 0.0;
  }

  builtin_interfaces::msg::Time stamp() const {
    builtin_interfaces::msg::Time s;
    const double t = t_;
    s.sec = static_cast<int32_t>(std::floor(t));
    s.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
    return s;
  }

  void loop() {
    const auto wall0 = std::chrono::steady_clock::now();
    const double t0 = t_;
    double next_scan = t_;
    double next_odom = t_;
    double next_gt = t_;
    double next_metrics = t_;
    auto rtf_wall = std::chrono::steady_clock::now();
    double rtf_sim = t_;
    publishStaticTf();
    while (rclcpp::ok() && !stop_) {
      {
        std::lock_guard<std::mutex> l(mu_);
        stepLocked();
      }
      rosgraph_msgs::msg::Clock c;
      c.clock = stamp();
      clock_pub_->publish(c);
      if (t_ >= next_odom) {
        next_odom += odom_period_;
        publishOdom();
      }
      if (t_ >= next_scan) {
        next_scan += scan_period_;
        publishScan();
      }
      if (t_ >= next_gt) {
        next_gt += 0.1;
        publishGroundTruth();
      }
      if (t_ >= next_metrics) {
        next_metrics += 0.5;
        const auto now = std::chrono::steady_clock::now();
        const double wall = std::chrono::duration<double>(now - rtf_wall).count();
        if (wall > 0.0) {
          measured_rtf_ = (t_ - rtf_sim) / wall;
        }
        rtf_wall = now;
        rtf_sim = t_;
        publishMetrics();
      }
      // Pace the loop: simulated time advances real_time_factor times as fast as wall time.
      const auto target = wall0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>((t_ - t0) / rtf_));
      std::this_thread::sleep_until(target);
    }
  }

  void stepLocked() {
    double v = v_cmd_;
    double w = w_cmd_;
    if (t_ - last_cmd_t_ > cmd_timeout_) {
      v = 0.0;
      w = 0.0;
    }
    const double x0 = robot_.x;
    const double y0 = robot_.y;
    stepDiffDrive(robot_, rp_, v, w, dt_);
    distance_ += std::hypot(robot_.x - x0, robot_.y - y0);
    for (auto & p : people_) {
      p.step(dt_, robot_.x, robot_.y);
    }
    t_ = t_ + dt_;
    // Ground-truth clearance every step.
    double mp = std::numeric_limits<double>::infinity();
    const Person * nearest = nullptr;
    for (const auto & p : people_) {
      const double d = footprintToCircle(robot_, rp_, p.x, p.y, p.radius);
      if (d < mp) {
        mp = d;
        nearest = &p;
      }
    }
    if (std::isfinite(mp)) {
      min_people_ = std::min(min_people_, std::max(0.0, mp));
      const bool contact = mp <= 0.0;
      if (contact && !in_people_contact_) {
        ++people_contacts_;
        // Person centre in the robot frame: ahead of the front edge = the robot drove into the person;
        // otherwise the person reached the robot's side or rear.
        const double c = std::cos(robot_.yaw);
        const double sn = std::sin(robot_.yaw);
        const double lx = c * (nearest->x - robot_.x) + sn * (nearest->y - robot_.y);
        const double ly = -sn * (nearest->x - robot_.x) + c * (nearest->y - robot_.y);
        contact_log_.push_back({{"t", t_.load()}, {"v", robot_.v}, {"x", robot_.x}, {"y", robot_.y}, {"lx", lx},
                                {"ly", ly}, {"front", lx > rp_.length / 2.0}});
      }
      in_people_contact_ = contact;
    }
    if (have_map_) {
      const double ms = footprintToStatic(grid_, dist_, robot_, rp_);
      min_static_ = std::min(min_static_, ms);
      const bool contact = ms <= 0.0;
      if (contact && !in_static_contact_) {
        ++static_contacts_;
      }
      in_static_contact_ = contact;
    }
  }

  void publishStaticTf() {
    geometry_msgs::msg::TransformStamped m2o;
    m2o.header.stamp = stamp();
    m2o.header.frame_id = "map";
    m2o.child_frame_id = "odom";  // perfect localisation: odom coincides with map
    m2o.transform.rotation.w = 1.0;
    geometry_msgs::msg::TransformStamped b2l;
    b2l.header.stamp = stamp();
    b2l.header.frame_id = "base_link";
    b2l.child_frame_id = "laser";
    b2l.transform.translation.x = scanner_x_;
    b2l.transform.rotation.w = 1.0;
    static_tf_->sendTransform({m2o, b2l});
  }

  void publishOdom() {
    RobotState s;
    {
      std::lock_guard<std::mutex> l(mu_);
      s = robot_;
    }
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp();
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = s.x;
    o.pose.pose.position.y = s.y;
    o.pose.pose.orientation.z = std::sin(s.yaw / 2.0);
    o.pose.pose.orientation.w = std::cos(s.yaw / 2.0);
    o.twist.twist.linear.x = s.v;
    o.twist.twist.angular.z = s.w;
    odom_pub_->publish(o);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = o.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = s.x;
    tf.transform.translation.y = s.y;
    tf.transform.rotation = o.pose.pose.orientation;
    tf_->sendTransform(tf);
  }

  void publishScan() {
    sensor_msgs::msg::LaserScan scan;
    {
      std::lock_guard<std::mutex> l(mu_);
      if (!have_map_) {
        return;
      }
      const double sx = robot_.x + scanner_x_ * std::cos(robot_.yaw);
      const double sy = robot_.y + scanner_x_ * std::sin(robot_.yaw);
      scan.angle_min = static_cast<float>(-fov_ / 2.0);
      scan.angle_increment = static_cast<float>(fov_ / (beams_ - 1));
      scan.angle_max = static_cast<float>(fov_ / 2.0);
      simulateScan(grid_, people_, sx, sy, robot_.yaw, scan.angle_min, scan.angle_increment, beams_, range_max_,
                   noise_, rng_, scan.ranges);
    }
    scan.header.stamp = stamp();
    scan.header.frame_id = "laser";
    scan.range_min = 0.05f;
    scan.range_max = static_cast<float>(range_max_);
    scan.scan_time = static_cast<float>(scan_period_);
    scan_pub_->publish(scan);
  }

  void publishGroundTruth() {
    json j;
    {
      std::lock_guard<std::mutex> l(mu_);
      json people = json::array();
      for (const auto & p : people_) {
        people.push_back({std::round(p.x * 1000) / 1000, std::round(p.y * 1000) / 1000});
      }
      j = {{"t", t_.load()}, {"x", robot_.x}, {"y", robot_.y}, {"yaw", robot_.yaw}, {"v", robot_.v}, {"w", robot_.w},
           {"people", people}, {"id", scenario_id_}};
    }
    std_msgs::msg::String m;
    m.data = j.dump();
    gt_pub_->publish(m);
  }

  void publishMetrics() {
    json j;
    {
      std::lock_guard<std::mutex> l(mu_);
      auto num = [](double v) { return std::isfinite(v) ? json(v) : json(nullptr); };
      j = {{"id", scenario_id_}, {"t", t_.load()}, {"min_clearance_people", num(min_people_)},
           {"min_clearance_static", num(min_static_)}, {"people_contacts", people_contacts_},
           {"static_contacts", static_contacts_}, {"distance", distance_}, {"rtf", measured_rtf_},
           {"contacts", contact_log_}};
    }
    std_msgs::msg::String m;
    m.data = j.dump();
    metrics_pub_->publish(m);
  }

  // parameters
  double rtf_ = 1.0, dt_ = 0.01, scan_period_ = 1.0 / 15, odom_period_ = 0.02, cmd_timeout_ = 0.5;
  double scanner_x_ = 0.5, fov_ = 4.71, range_max_ = 20.0, noise_ = 0.01;
  int beams_ = 541;
  RobotParams rp_;
  // state (guarded by mu_)
  std::mutex mu_;
  Grid grid_;
  std::vector<float> dist_;
  bool have_map_ = false;
  RobotState robot_;
  std::vector<Person> people_;
  double v_cmd_ = 0.0, w_cmd_ = 0.0, last_cmd_t_ = -1.0;
  std::atomic<double> t_{1.0};
  std::mt19937 rng_;
  std::string scenario_id_;
  double min_people_ = 0.0, min_static_ = 0.0, distance_ = 0.0, measured_rtf_ = 0.0;
  int people_contacts_ = 0, static_contacts_ = 0;
  json contact_log_ = json::array();  // robot speed and pose at each person contact onset
  bool in_people_contact_ = false, in_static_contact_ = false;
  // ROS
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gt_pub_, metrics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scenario_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace tugger_nav

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tugger_nav::WarehouseSim>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
