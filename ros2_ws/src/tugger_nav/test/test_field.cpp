#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "tugger_nav/field_core.hpp"

using namespace tugger_nav;

namespace {
FieldConfig cfg() {
  FieldConfig c;
  c.robot_length = 1.0;
  c.robot_width = 0.7;
  c.t_response = 0.2;
  c.a_brake = 0.8;
  c.k_approach = 0.0;
  c.c_margin = 0.15;
  c.lateral_margin = 0.1;
  c.warning_time = 1.0;
  c.warning_margin = 0.5;
  c.warning_lateral = 0.3;
  c.speed_bands = {0.2, 0.4, 0.6, 0.8, 1.0, 1.2};
  return c;
}
// A scan with a single return at (angle, range) and no return elsewhere.
std::vector<float> oneHit(int n, double angle_min, double inc, double angle, double range) {
  std::vector<float> r(static_cast<size_t>(n), std::numeric_limits<float>::infinity());
  r[static_cast<size_t>(std::lround((angle - angle_min) / inc))] = static_cast<float>(range);
  return r;
}
}  // namespace

TEST(FieldGeometry, ProtectiveLengthMatchesFormula) {
  const auto c = cfg();
  // v = 1: 1 * 0.2 + 1 / 1.6 + 0 + 0.15 = 0.975
  EXPECT_NEAR(protectiveLength(c, 1.0), 0.975, 1e-12);
  EXPECT_NEAR(protectiveLength(c, 0.0), 0.15, 1e-12);
  EXPECT_NEAR(protectiveLength(c, -1.0), protectiveLength(c, 1.0), 1e-12);
}

TEST(FieldGeometry, ApproachSpeedAddsKTimesStoppingTime) {
  auto c = cfg();
  c.k_approach = 1.6;  // ISO 13855 walking speed
  const double t_stop = 0.2 + 1.0 / 0.8;
  EXPECT_NEAR(stoppingTime(c, 1.0), t_stop, 1e-12);
  EXPECT_NEAR(protectiveLength(c, 1.0), 0.975 + 1.6 * t_stop, 1e-12);
}

TEST(FieldGeometry, LengthsGrowWithSpeed) {
  const auto c = cfg();
  double prev_p = -1.0;
  double prev_w = -1.0;
  for (double v = 0.0; v <= 1.5; v += 0.05) {
    EXPECT_GT(protectiveLength(c, v), prev_p);
    EXPECT_GT(warningLength(c, v), prev_w);
    EXPECT_GT(warningLength(c, v), protectiveLength(c, v));
    prev_p = protectiveLength(c, v);
    prev_w = warningLength(c, v);
  }
}

TEST(FieldGeometry, BandSelectionUsesUpperSpeedOfBand) {
  const auto c = cfg();
  EXPECT_EQ(bandIndex(c, 0.0), 0);
  EXPECT_EQ(bandIndex(c, 0.2), 0);
  EXPECT_EQ(bandIndex(c, 0.21), 1);
  EXPECT_EQ(bandIndex(c, -0.5), 2);
  EXPECT_EQ(bandIndex(c, 1.2), 5);
  EXPECT_EQ(bandIndex(c, 5.0), 5);  // above the top band: top field set
  const FieldSet fs = selectFieldSet(c, 0.55);
  EXPECT_EQ(fs.index, 2);
  EXPECT_DOUBLE_EQ(fs.design_speed, 0.6);
  // The field set is sized for the band's upper speed, so it is never shorter than needed inside the band.
  EXPECT_NEAR(fs.protective.length(), protectiveLength(c, 0.6), 1e-12);
  EXPECT_GE(fs.protective.length(), protectiveLength(c, 0.55));
}

TEST(FieldGeometry, FieldsStartAtFrontEdgeAndContainEachOther) {
  const auto c = cfg();
  const FieldSet fs = selectFieldSet(c, 1.0);
  EXPECT_DOUBLE_EQ(fs.protective.x0, 0.5);
  EXPECT_DOUBLE_EQ(fs.warning.x0, 0.5);
  EXPECT_DOUBLE_EQ(fs.protective.half_width, 0.45);
  EXPECT_DOUBLE_EQ(fs.warning.half_width, 0.75);
  EXPECT_GT(fs.warning.x1, fs.protective.x1);
  EXPECT_TRUE(fs.protective.contains(1.0, 0.0));
  EXPECT_FALSE(fs.protective.contains(1.0, 0.5));
  EXPECT_TRUE(fs.warning.contains(1.0, 0.5));
  EXPECT_FALSE(fs.protective.contains(0.4, 0.0));  // inside the robot body, not the field
}

TEST(FieldGeometry, ConfigValidationRejectsBadValues) {
  auto c = cfg();
  EXPECT_NO_THROW(c.validate());
  c.a_brake = 0.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = cfg();
  c.speed_bands = {0.5, 0.4};
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = cfg();
  c.speed_bands.clear();
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = cfg();
  c.t_response = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(ScanEvaluation, PointStraightAheadIsClassifiedByDistance) {
  const auto c = cfg();
  const FieldSet fs = selectFieldSet(c, 1.0);  // protective to x = 0.5 + 0.975, warning to 0.5 + 2.475
  const int n = 541;
  const double amin = -2.356;
  const double inc = 4.712 / 540;
  const double sx = 0.5;  // scanner on the front edge
  auto r = evaluateScan(fs, oneHit(n, amin, inc, 0.0, 0.5), amin, inc, 0.05, 20.0, sx);
  EXPECT_TRUE(r.protective);
  EXPECT_TRUE(r.warning);
  EXPECT_NEAR(r.nearest_x, 1.0, 1e-3);
  r = evaluateScan(fs, oneHit(n, amin, inc, 0.0, 1.5), amin, inc, 0.05, 20.0, sx);
  EXPECT_FALSE(r.protective);
  EXPECT_TRUE(r.warning);
  r = evaluateScan(fs, oneHit(n, amin, inc, 0.0, 3.0), amin, inc, 0.05, 20.0, sx);
  EXPECT_FALSE(r.warning);
}

TEST(ScanEvaluation, SidePointsOutsideTheWidthAreIgnored) {
  const auto c = cfg();
  const FieldSet fs = selectFieldSet(c, 1.0);
  const int n = 541;
  const double amin = -2.356;
  const double inc = 4.712 / 540;
  // Point 0.6 m to the left, 0.3 m ahead of the scanner: outside protective (0.45) but inside warning (0.75).
  const double ang = std::atan2(0.6, 0.3);
  const auto r = evaluateScan(fs, oneHit(n, amin, inc, ang, std::hypot(0.3, 0.6)), amin, inc, 0.05, 20.0, 0.5);
  EXPECT_FALSE(r.protective);
  EXPECT_TRUE(r.warning);
}

TEST(ScanEvaluation, InvalidRangesAreIgnored) {
  const auto c = cfg();
  const FieldSet fs = selectFieldSet(c, 1.0);
  std::vector<float> r(11, std::numeric_limits<float>::quiet_NaN());
  r[5] = 0.01f;  // below range_min
  r[4] = -1.0f;
  r[3] = std::numeric_limits<float>::infinity();
  const auto res = evaluateScan(fs, r, -0.1, 0.02, 0.05, 20.0, 0.5);
  EXPECT_FALSE(res.protective);
  EXPECT_FALSE(res.warning);
}

TEST(ScanEvaluation, NearestProtectivePointIsReported) {
  const auto c = cfg();
  const FieldSet fs = selectFieldSet(c, 1.0);
  std::vector<float> r(3, std::numeric_limits<float>::infinity());
  r[0] = 0.8f;
  r[1] = 0.4f;
  r[2] = 0.9f;
  const auto res = evaluateScan(fs, r, -0.01, 0.01, 0.05, 20.0, 0.5);
  ASSERT_TRUE(res.protective);
  EXPECT_NEAR(res.nearest_x, 0.9, 1e-3);
}

class StateMachine : public ::testing::Test {
protected:
  FieldConfig c = cfg();
  FieldStateMachine sm{c};
};

TEST_F(StateMachine, StartsStoppedAndClearsAfterFirstCleanScan) {
  EXPECT_EQ(sm.state(), MonitorState::ProtectiveStop);
  EXPECT_EQ(sm.onScan(10.0, false, false), MonitorState::Clear);
  EXPECT_EQ(sm.protectiveStops(), 0u);  // the initial stop is not counted
}

TEST_F(StateMachine, WarningAndBack) {
  sm.onScan(0.0, false, false);
  EXPECT_EQ(sm.onScan(0.1, false, true), MonitorState::Warning);
  EXPECT_EQ(sm.onScan(0.2, false, true), MonitorState::Warning);
  EXPECT_EQ(sm.onScan(0.3, false, false), MonitorState::Clear);
  EXPECT_EQ(sm.warningEntries(), 1u);
}

TEST_F(StateMachine, ProtectiveStopHoldsUntilClearForHoldTime) {
  sm.onScan(0.0, false, false);
  EXPECT_EQ(sm.onScan(1.0, true, true), MonitorState::ProtectiveStop);
  EXPECT_EQ(sm.protectiveStops(), 1u);
  EXPECT_EQ(sm.onScan(1.5, false, false), MonitorState::ProtectiveStop);   // 0.5 s clear < 1.0 s
  EXPECT_EQ(sm.onScan(1.9, true, true), MonitorState::ProtectiveStop);     // violated again: timer restarts
  EXPECT_EQ(sm.onScan(2.8, false, false), MonitorState::ProtectiveStop);   // 0.9 s
  EXPECT_EQ(sm.onScan(3.0, false, false), MonitorState::Clear);            // 1.1 s clear
  EXPECT_EQ(sm.protectiveStops(), 1u);
}

TEST_F(StateMachine, RestartGoesToWarningIfWarningFieldStillOccupied) {
  sm.onScan(0.0, false, false);
  sm.onScan(1.0, true, true);
  EXPECT_EQ(sm.onScan(2.1, false, true), MonitorState::Warning);
  EXPECT_EQ(sm.warningEntries(), 1u);
}

TEST_F(StateMachine, ScanTimeoutStopsAndNeedsHoldTimeAfterScansReturn) {
  sm.onScan(0.0, false, false);
  EXPECT_EQ(sm.onTick(0.2), MonitorState::Clear);
  EXPECT_EQ(sm.onTick(0.5), MonitorState::ProtectiveStop);  // > 0.3 s without a scan
  EXPECT_TRUE(sm.scanTimedOut());
  EXPECT_EQ(sm.onScan(0.6, false, false), MonitorState::ProtectiveStop);
  EXPECT_EQ(sm.onScan(1.5, false, false), MonitorState::Clear);
}

TEST_F(StateMachine, GateScalesStopsAndBlocksReverse) {
  sm.onScan(0.0, false, false);
  double v = 1.0;
  double w = 0.5;
  sm.gate(v, w);
  EXPECT_DOUBLE_EQ(v, 1.0);
  sm.onScan(0.1, false, true);  // warning: cap at 0.3 m/s, keep the curvature
  v = 1.0;
  w = 0.5;
  sm.gate(v, w);
  EXPECT_DOUBLE_EQ(v, 0.3);
  EXPECT_DOUBLE_EQ(w, 0.15);
  v = 0.2;
  w = 0.4;
  sm.gate(v, w);  // already slower than the cap
  EXPECT_DOUBLE_EQ(v, 0.2);
  EXPECT_DOUBLE_EQ(w, 0.4);
  sm.onScan(0.2, true, true);
  v = 1.0;
  w = 0.5;
  sm.gate(v, w);
  EXPECT_DOUBLE_EQ(v, 0.0);
  EXPECT_DOUBLE_EQ(w, 0.0);
  FieldStateMachine sm2(c);
  sm2.onScan(0.0, false, false);
  v = -0.5;
  w = 0.0;
  sm2.gate(v, w);
  EXPECT_DOUBLE_EQ(v, 0.0);
}
