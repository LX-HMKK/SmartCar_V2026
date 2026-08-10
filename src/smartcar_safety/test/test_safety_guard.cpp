#include <gtest/gtest.h>

#include "smartcar_safety/guard.hpp"

namespace smartcar_safety {
namespace {

SafetyGuard make_guard(double minimum_voltage = 10.0,
                       double voltage_timeout = 1.0) {
  return SafetyGuard(5.0, 5.0, 5.0, 5.0, minimum_voltage, voltage_timeout,
                     0.30, true, true, true);
}

void mark_healthy(SafetyGuard &guard, double now_sec) {
  ASSERT_TRUE(guard.mark_command(now_sec, 0.0));
  guard.mark_scan(now_sec);
  guard.mark_odom(now_sec);
  guard.mark_raw_odom(now_sec);
  guard.mark_voltage(12.0F, now_sec);
}

}  // namespace

TEST(SafetyGuardTest, RejectsStaleVoltageWhenThresholdEnabled) {
  auto guard = make_guard(10.0, 0.50);
  mark_healthy(guard, 10.0);

  ASSERT_TRUE(guard.mark_command(10.60, 0.0));
  guard.mark_scan(10.60);
  guard.mark_odom(10.60);
  guard.mark_raw_odom(10.60);
  const auto verdict = guard.evaluate(10.60);

  EXPECT_FALSE(verdict.allowed);
  EXPECT_EQ(verdict.reason, "voltage_stale");
}

TEST(SafetyGuardTest, RejectsBothSpeedDirectionsUntilExplicitlyCleared) {
  auto guard = make_guard();
  mark_healthy(guard, 10.0);

  for (const double speed : {0.300001, -0.300001}) {
    EXPECT_FALSE(guard.mark_command(10.10, speed));
    auto verdict = guard.evaluate(10.10);
    EXPECT_FALSE(verdict.allowed);
    EXPECT_EQ(verdict.reason, "command_speed_limit_exceeded");

    EXPECT_FALSE(guard.mark_command(10.20, speed > 0.0 ? 0.30 : -0.30));
    guard.clear_command_speed_limit_fault();
    EXPECT_TRUE(guard.mark_command(10.30, speed > 0.0 ? 0.30 : -0.30));
    verdict = guard.evaluate(10.30);
    EXPECT_TRUE(verdict.allowed);
    EXPECT_EQ(verdict.reason, "ok");
  }
}

TEST(SafetyGuardTest, RequiresFreshDepthPointsWhenConfigured) {
  SafetyGuard guard(1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 0.30, true, true,
                    true, 1.0, true);
  ASSERT_TRUE(guard.mark_command(10.0, 0.0));
  guard.mark_scan(10.0);
  guard.mark_odom(10.0);
  guard.mark_raw_odom(10.0);
  EXPECT_EQ(guard.evaluate(10.10).reason, "depth_points_missing");

  guard.mark_depth_points(10.0);
  EXPECT_TRUE(guard.evaluate(10.20).allowed);

  ASSERT_TRUE(guard.mark_command(11.10, 0.0));
  guard.mark_scan(11.10);
  guard.mark_odom(11.10);
  guard.mark_raw_odom(11.10);
  EXPECT_EQ(guard.evaluate(11.10).reason, "depth_points_stale");
}

}  // namespace smartcar_safety
