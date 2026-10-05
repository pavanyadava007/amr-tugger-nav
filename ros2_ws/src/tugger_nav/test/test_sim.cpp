#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "tugger_nav/grid.hpp"
#include "tugger_nav/sim_core.hpp"

using namespace tugger_nav;

namespace {
// 10 x 10 m grid at 0.1 m with a wall at x in [6, 7).
Grid wallGrid() {
  Grid g;
  g.width = 100;
  g.height = 100;
  g.resolution = 0.1;
  g.occupied.assign(100 * 100, 0);
  for (int y = 0; y < 100; ++y) {
    for (int x = 60; x < 70; ++x) {
      g.occupied[static_cast<size_t>(y) * 100 + x] = 1;
    }
  }
  return g;
}
}  // namespace

TEST(Grid, RaycastHitsWallAtExactDistance) {
  const Grid g = wallGrid();
  EXPECT_NEAR(raycast(g, 2.05, 5.05, 0.0, 20.0), 3.95, 1e-9);
  // 45 degrees: wall at x = 6 reached after (6 - 2.05) * sqrt(2)
  EXPECT_NEAR(raycast(g, 2.05, 2.05, M_PI / 4, 20.0), 3.95 * std::sqrt(2.0), 1e-6);
  EXPECT_NEAR(raycast(g, 2.05, 5.05, 0.0, 1.0), 1.0, 1e-12);  // max range
  // Leaving the map counts as a hit on the border.
  EXPECT_NEAR(raycast(g, 2.05, 5.05, M_PI, 20.0), 2.05, 1e-9);
}

TEST(Grid, DistanceTransformMatchesBruteForce) {
  Grid g;
  g.width = 23;
  g.height = 17;
  g.resolution = 0.05;
  g.occupied.assign(23 * 17, 0);
  std::mt19937 rng(3);
  for (auto & c : g.occupied) {
    c = (rng() % 13 == 0) ? 1 : 0;
  }
  const auto d = distanceTransform(g);
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      double best = std::numeric_limits<double>::infinity();
      for (int yy = 0; yy < g.height; ++yy) {
        for (int xx = 0; xx < g.width; ++xx) {
          if (g.occupied[static_cast<size_t>(yy) * g.width + xx]) {
            best = std::min(best, std::hypot(xx - x, yy - y) * g.resolution);
          }
        }
      }
      EXPECT_NEAR(d[static_cast<size_t>(y) * g.width + x], best, 1e-5) << x << "," << y;
    }
  }
}

TEST(DiffDrive, AccelerationAndBrakingLimits) {
  RobotParams p;
  p.max_accel = 0.5;
  p.max_decel = 1.0;
  RobotState s;
  for (int i = 0; i < 100; ++i) {
    stepDiffDrive(s, p, 1.0, 0.0, 0.01);
  }
  EXPECT_NEAR(s.v, 0.5, 1e-9);  // 1 s at 0.5 m/s^2
  EXPECT_NEAR(s.x, 0.25, 1e-2);
  // Braking from 0.5 m/s takes 0.5 s and 0.125 m.
  const double x0 = s.x;
  int steps = 0;
  while (s.v > 1e-9) {
    stepDiffDrive(s, p, 0.0, 0.0, 0.01);
    ++steps;
  }
  EXPECT_NEAR(steps, 50, 1);
  EXPECT_NEAR(s.x - x0, 0.125, 5e-3);
}

TEST(DiffDrive, ConstantTurnStaysOnCircle) {
  RobotParams p;
  p.max_accel = 100.0;
  p.max_ang_accel = 100.0;
  RobotState s;
  for (int i = 0; i < 1000; ++i) {
    stepDiffDrive(s, p, 0.5, 0.5, 0.01);  // radius 1 m around (0, 1)
    EXPECT_NEAR(std::hypot(s.x, s.y - 1.0), 1.0, 1e-2);
  }
}

TEST(People, WaitForTriggerThenWalkAndWait) {
  Person p;
  p.waypoints = {{0.0, 0.0, 0.0}, {2.0, 0.0, 1.0}, {2.0, 2.0, 0.0}};
  p.speed = 1.0;
  p.trigger_x = 10.0;
  p.trigger_y = 0.0;
  p.trigger_radius = 3.0;
  p.reset();
  p.step(1.0, 0.0, 0.0);  // robot far away: no motion
  EXPECT_DOUBLE_EQ(p.x, 0.0);
  p.step(1.0, 8.0, 0.0);  // triggered
  EXPECT_NEAR(p.x, 1.0, 1e-12);
  p.step(1.5, 8.0, 0.0);  // reaches (2, 0) after 1 s, then waits
  EXPECT_NEAR(p.x, 2.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  p.step(1.0, 8.0, 0.0);  // 0.5 s wait left, then 0.5 m north
  EXPECT_NEAR(p.y, 0.5, 1e-12);
  p.step(5.0, 8.0, 0.0);
  EXPECT_TRUE(p.done());
  EXPECT_NEAR(p.y, 2.0, 1e-12);
}

TEST(Scan, PeopleOccludeTheWall) {
  const Grid g = wallGrid();
  Person p;
  p.waypoints = {{4.0, 5.0, 0.0}};
  p.radius = 0.25;
  p.reset();
  std::mt19937 rng(1);
  std::vector<float> r;
  simulateScan(g, {p}, 2.0, 5.0, 0.0, -0.2, 0.2, 3, 20.0, 0.0, rng, r);
  EXPECT_NEAR(r[1], 1.75, 1e-6);  // person surface
  EXPECT_NEAR(r[0], 4.0 / std::cos(0.2), 1e-3);  // wall, beside the person
}

TEST(Clearance, FootprintToCircle) {
  RobotParams p;  // 1.0 x 0.7
  RobotState s;
  EXPECT_NEAR(footprintToCircle(s, p, 2.0, 0.0, 0.25), 1.25, 1e-12);
  EXPECT_NEAR(footprintToCircle(s, p, 0.0, 1.0, 0.25), 0.4, 1e-12);
  EXPECT_LT(footprintToCircle(s, p, 0.6, 0.0, 0.25), 0.0);  // overlap
  s.yaw = M_PI / 2;  // rotated: long side along y
  EXPECT_NEAR(footprintToCircle(s, p, 0.0, 2.0, 0.25), 1.25, 1e-12);
}

TEST(Clearance, FootprintToStatic) {
  const Grid g = wallGrid();
  const auto d = distanceTransform(g);
  RobotParams p;
  RobotState s;
  s.x = 4.0;
  s.y = 5.0;
  // Front edge at x = 4.5, wall cells start at x = 6.0: cell-centre distance 1.5 minus half a cell.
  EXPECT_NEAR(footprintToStatic(g, d, s, p), 1.5, 0.06);
  s.x = 5.6;
  EXPECT_DOUBLE_EQ(footprintToStatic(g, d, s, p), 0.0);
}
