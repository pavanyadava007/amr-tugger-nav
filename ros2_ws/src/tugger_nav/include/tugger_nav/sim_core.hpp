// Simulation core: differential-drive kinematics with acceleration limits, scripted people, laser scan,
// and ground-truth clearance. No ROS dependency, so it is unit tested directly.
#pragma once
#include <random>
#include <string>
#include <vector>

#include "tugger_nav/grid.hpp"

namespace tugger_nav {

struct RobotParams {
  double length = 1.0;        // footprint, base_link at the centre [m]
  double width = 0.7;
  double max_accel = 0.8;     // linear acceleration limit [m/s^2]
  double max_decel = 1.0;     // linear braking limit [m/s^2]
  double max_ang_accel = 2.0; // [rad/s^2]
  double max_speed = 1.2;     // [m/s]
  double max_ang_speed = 1.5; // [rad/s]
};

struct RobotState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double w = 0.0;
};

// Move the velocities toward the command within the acceleration limits, then integrate the pose (exact arc).
void stepDiffDrive(RobotState & s, const RobotParams & p, double v_cmd, double w_cmd, double dt);

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double wait = 0.0;  // seconds to stand still after reaching this waypoint
};

// A scripted person: stands at waypoints[0] until the robot comes within trigger_radius of (trigger_x, trigger_y),
// then walks the waypoints at constant speed. People do not react to the robot (worst case for the robot).
struct Person {
  std::vector<Waypoint> waypoints;
  double speed = 1.2;
  double radius = 0.25;
  double trigger_x = 0.0;
  double trigger_y = 0.0;
  double trigger_radius = 5.0;
  // runtime
  double x = 0.0;
  double y = 0.0;
  size_t next = 1;
  bool triggered = false;
  double wait_left = 0.0;
  void reset();
  void step(double dt, double robot_x, double robot_y);
  bool done() const { return next >= waypoints.size() && wait_left <= 0.0; }
};

// Simulated 2D scan from a scanner at (sx, sy, syaw): static map plus people (circles), Gaussian range noise.
void simulateScan(const Grid & g, const std::vector<Person> & people, double sx, double sy, double syaw,
                  double angle_min, double angle_increment, int count, double range_max, double noise_sigma,
                  std::mt19937 & rng, std::vector<float> & ranges);

// Distance from a ray to a circle, or +inf if it misses.
double rayCircle(double ox, double oy, double dx, double dy, double cx, double cy, double r);

// Signed distance between the robot footprint rectangle and a circle (<= 0 means contact).
double footprintToCircle(const RobotState & s, const RobotParams & p, double cx, double cy, double r);

// Clearance between the robot footprint and static obstacles: minimum distance transform value over points
// sampled every `spacing` metres along the footprint outline, minus half a cell. 0 means contact.
double footprintToStatic(const Grid & g, const std::vector<float> & dist, const RobotState & s,
                         const RobotParams & p, double spacing = 0.05);

}  // namespace tugger_nav
