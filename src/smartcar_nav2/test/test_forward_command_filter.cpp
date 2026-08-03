#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "smartcar_nav2/forward_command_filter.hpp"

namespace
{

using smartcar_nav2::ForwardCommand;
using smartcar_nav2::ForwardCommandFilterStatus;
using smartcar_nav2::ForwardCommandLimits;

constexpr double kVxMax = 0.30;
constexpr double kWzMax = 1.3636363636363635;
constexpr double kTurningRadius = 0.22;

ForwardCommandLimits limits()
{
  return ForwardCommandLimits{kVxMax, kWzMax, kTurningRadius};
}

void expectZero(const ForwardCommand & command)
{
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_z, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

}  // namespace

TEST(ForwardCommandFilter, RejectsNonFiniteCommandAsAllZero)
{
  const std::array<double ForwardCommand::*, 6> fields = {
    &ForwardCommand::linear_x, &ForwardCommand::linear_y,
    &ForwardCommand::linear_z, &ForwardCommand::angular_x,
    &ForwardCommand::angular_y, &ForwardCommand::angular_z};
  for (const auto field : fields) {
    ForwardCommand input{0.10, 0.0, 0.0, 0.0, 0.0, 0.10};
    input.*field = std::numeric_limits<double>::quiet_NaN();
    const auto result = smartcar_nav2::enforceForwardCommandLimits(input, limits());
    EXPECT_EQ(result.status, ForwardCommandFilterStatus::kNonFiniteCommand);
    expectZero(result.command);
  }
}

TEST(ForwardCommandFilter, RejectsReverseVelocityAsAllZero)
{
  const auto result = smartcar_nav2::enforceForwardCommandLimits(
    ForwardCommand{-0.01, 1.0, 2.0, 3.0, 4.0, 5.0}, limits());
  EXPECT_EQ(result.status, ForwardCommandFilterStatus::kReverseVelocity);
  expectZero(result.command);
}

TEST(ForwardCommandFilter, ClampsAngularSpeedToCurvatureAndStaticLimit)
{
  const auto result = smartcar_nav2::enforceForwardCommandLimits(
    ForwardCommand{kVxMax, 0.0, 0.0, 0.0, 0.0, 2.0}, limits());
  ASSERT_EQ(result.status, ForwardCommandFilterStatus::kAccepted);
  EXPECT_NEAR(result.command.angular_z, kVxMax / kTurningRadius, 1.0e-12);

  auto static_limits = limits();
  static_limits.vx_max = 0.40;
  const auto static_limit = smartcar_nav2::enforceForwardCommandLimits(
    ForwardCommand{0.40, 0.0, 0.0, 0.0, 0.0, -2.0}, static_limits);
  ASSERT_EQ(static_limit.status, ForwardCommandFilterStatus::kAccepted);
  EXPECT_DOUBLE_EQ(static_limit.command.angular_z, -kWzMax);
}

TEST(ForwardCommandFilter, ZeroLinearSpeedClearsAngularSpeed)
{
  const auto result = smartcar_nav2::enforceForwardCommandLimits(
    ForwardCommand{0.0, 0.0, 0.0, 0.0, 0.0, 0.20}, limits());
  ASSERT_EQ(result.status, ForwardCommandFilterStatus::kAccepted);
  expectZero(result.command);
}

TEST(ForwardCommandFilter, ClearsUnsupportedTwistComponents)
{
  const auto result = smartcar_nav2::enforceForwardCommandLimits(
    ForwardCommand{0.10, 0.01, 0.02, 0.03, 0.04, 0.01}, limits());
  ASSERT_EQ(result.status, ForwardCommandFilterStatus::kAccepted);
  EXPECT_DOUBLE_EQ(result.command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(result.command.linear_z, 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular_x, 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular_y, 0.0);
}

TEST(ForwardCommandFilter, TranslatesSpeedLimitsWithoutLiftingStaticGuard)
{
  const auto percentage = smartcar_nav2::translateForwardSpeedLimit(
    50.0, true, kVxMax);
  ASSERT_TRUE(percentage.valid);
  EXPECT_DOUBLE_EQ(percentage.forwarded_speed_limit, 50.0);
  EXPECT_TRUE(percentage.forwarded_percentage);
  EXPECT_DOUBLE_EQ(percentage.guard_scale, 0.5);

  const auto absolute = smartcar_nav2::translateForwardSpeedLimit(
    0.15, false, kVxMax);
  ASSERT_TRUE(absolute.valid);
  EXPECT_NEAR(absolute.guard_scale, 0.5, 1.0e-12);

  const auto unlimited = smartcar_nav2::translateForwardSpeedLimit(
    150.0, true, kVxMax);
  ASSERT_TRUE(unlimited.valid);
  EXPECT_DOUBLE_EQ(unlimited.guard_scale, 1.0);
}

TEST(ForwardCommandFilter, InvalidLimitsFailClosed)
{
  auto invalid = limits();
  invalid.vx_max = 0.0;
  const auto result = smartcar_nav2::enforceForwardCommandLimits(
    ForwardCommand{0.10, 0.0, 0.0, 0.0, 0.0, 0.0}, invalid);
  EXPECT_EQ(result.status, ForwardCommandFilterStatus::kInvalidLimits);
  expectZero(result.command);
}
