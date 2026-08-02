#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "smartcar_nav2/reverse_command_filter.hpp"

namespace
{

using smartcar_nav2::ReverseCommand;
using smartcar_nav2::ReverseCommandFilterStatus;
using smartcar_nav2::ReverseCommandLimits;

constexpr double kVxMin = -0.09;
constexpr double kWzMax = 0.42;
constexpr double kTurningRadius = 0.22;

ReverseCommandLimits limits()
{
  return ReverseCommandLimits{kVxMin, kWzMax, kTurningRadius};
}

void expectZero(const ReverseCommand & command)
{
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_z, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

}  // namespace

TEST(ReverseCommandFilter, RejectsNonFiniteCommandAsAllZero)
{
  const std::array<double ReverseCommand::*, 6> fields = {
    &ReverseCommand::linear_x,
    &ReverseCommand::linear_y,
    &ReverseCommand::linear_z,
    &ReverseCommand::angular_x,
    &ReverseCommand::angular_y,
    &ReverseCommand::angular_z,
  };
  for (const auto field : fields) {
    ReverseCommand input{-0.05, 0.0, 0.0, 0.0, 0.0, 0.05};
    input.*field = std::numeric_limits<double>::quiet_NaN();
    const auto result = smartcar_nav2::enforceReverseCommandLimits(input, limits());
    EXPECT_EQ(result.status, ReverseCommandFilterStatus::kNonFiniteCommand);
    expectZero(result.command);
  }
}

TEST(ReverseCommandFilter, RejectsAnyForwardVelocityAsAllZero)
{
  const auto result = smartcar_nav2::enforceReverseCommandLimits(
    ReverseCommand{0.001, 1.0, 2.0, 3.0, 4.0, 5.0}, limits());
  EXPECT_EQ(result.status, ReverseCommandFilterStatus::kForwardVelocity);
  expectZero(result.command);
}

TEST(ReverseCommandFilter, MapsVirtualForwardSpeedWithoutFlippingYawRate)
{
  const ReverseCommand virtual_command{0.05, 1.0, 2.0, 3.0, 4.0, -0.10};
  const auto reverse_command =
    smartcar_nav2::mapVirtualForwardCommandToReverse(virtual_command);
  EXPECT_DOUBLE_EQ(reverse_command.linear_x, -0.05);
  EXPECT_DOUBLE_EQ(reverse_command.linear_y, -virtual_command.linear_y);
  EXPECT_DOUBLE_EQ(reverse_command.angular_z, virtual_command.angular_z);

  const auto filtered = smartcar_nav2::enforceReverseCommandLimits(
    reverse_command, limits());
  ASSERT_EQ(filtered.status, ReverseCommandFilterStatus::kAccepted);
  EXPECT_DOUBLE_EQ(filtered.command.linear_x, -0.05);
  EXPECT_DOUBLE_EQ(filtered.command.angular_z, virtual_command.angular_z);
}

TEST(ReverseCommandFilter, RejectsVirtualReverseLeakAfterMapping)
{
  const auto reverse_command = smartcar_nav2::mapVirtualForwardCommandToReverse(
    ReverseCommand{-0.01, 0.0, 0.0, 0.0, 0.0, 0.02});
  const auto filtered = smartcar_nav2::enforceReverseCommandLimits(
    reverse_command, limits());
  EXPECT_EQ(filtered.status, ReverseCommandFilterStatus::kForwardVelocity);
  expectZero(filtered.command);
}

TEST(ReverseCommandFilter, ClampsReverseSpeedToConfiguredMinimum)
{
  const auto result = smartcar_nav2::enforceReverseCommandLimits(
    ReverseCommand{-0.30, 0.0, 0.0, 0.0, 0.0, 0.0}, limits());
  ASSERT_EQ(result.status, ReverseCommandFilterStatus::kAccepted);
  EXPECT_DOUBLE_EQ(result.command.linear_x, kVxMin);
}

TEST(ReverseCommandFilter, ClampsAngularSpeedToCurvatureAndStaticLimit)
{
  const auto curvature_limited = smartcar_nav2::enforceReverseCommandLimits(
    ReverseCommand{-0.09, 0.0, 0.0, 0.0, 0.0, 0.50}, limits());
  ASSERT_EQ(curvature_limited.status, ReverseCommandFilterStatus::kAccepted);
  EXPECT_NEAR(
    curvature_limited.command.angular_z,
    std::abs(kVxMin) / kTurningRadius, 1.0e-12);

  auto static_limit = limits();
  static_limit.vx_min = -0.30;
  const auto statically_limited = smartcar_nav2::enforceReverseCommandLimits(
    ReverseCommand{-0.20, 0.0, 0.0, 0.0, 0.0, -0.50}, static_limit);
  ASSERT_EQ(statically_limited.status, ReverseCommandFilterStatus::kAccepted);
  EXPECT_DOUBLE_EQ(statically_limited.command.angular_z, -kWzMax);
}

TEST(ReverseCommandFilter, ZeroLinearSpeedClearsAngularSpeed)
{
  const auto result = smartcar_nav2::enforceReverseCommandLimits(
    ReverseCommand{0.0, 0.0, 0.0, 0.0, 0.0, 0.20}, limits());
  ASSERT_EQ(result.status, ReverseCommandFilterStatus::kAccepted);
  expectZero(result.command);
}

TEST(ReverseCommandFilter, InvalidLimitsFailClosed)
{
  auto invalid = limits();
  invalid.vx_min = 0.0;
  const auto result = smartcar_nav2::enforceReverseCommandLimits(
    ReverseCommand{-0.05, 0.0, 0.0, 0.0, 0.0, 0.0}, invalid);
  EXPECT_EQ(result.status, ReverseCommandFilterStatus::kInvalidLimits);
  expectZero(result.command);
}

TEST(ReverseCommandFilter, PreservesPercentageAndNoLimitRepresentation)
{
  const auto percentage = smartcar_nav2::translateReverseSpeedLimit(
    50.0, true, kVxMin);
  ASSERT_TRUE(percentage.valid);
  EXPECT_DOUBLE_EQ(percentage.forwarded_speed_limit, 50.0);
  EXPECT_TRUE(percentage.forwarded_percentage);
  EXPECT_DOUBLE_EQ(percentage.guard_scale, 0.5);

  const auto no_limit = smartcar_nav2::translateReverseSpeedLimit(
    0.0, false, kVxMin);
  ASSERT_TRUE(no_limit.valid);
  EXPECT_TRUE(no_limit.no_limit);
  EXPECT_DOUBLE_EQ(no_limit.forwarded_speed_limit, 0.0);
  EXPECT_FALSE(no_limit.forwarded_percentage);
  EXPECT_DOUBLE_EQ(no_limit.guard_scale, 1.0);
}

TEST(ReverseCommandFilter, ConvertsAbsoluteSpeedUsingReverseMagnitude)
{
  const auto absolute = smartcar_nav2::translateReverseSpeedLimit(
    0.045, false, kVxMin);
  ASSERT_TRUE(absolute.valid);
  EXPECT_NEAR(absolute.forwarded_speed_limit, 50.0, 1.0e-12);
  EXPECT_TRUE(absolute.forwarded_percentage);
  EXPECT_DOUBLE_EQ(absolute.guard_scale, 0.5);
}

TEST(ReverseCommandFilter, PercentageCannotLiftStaticGuard)
{
  const auto percentage = smartcar_nav2::translateReverseSpeedLimit(
    150.0, true, kVxMin);
  ASSERT_TRUE(percentage.valid);
  EXPECT_DOUBLE_EQ(percentage.forwarded_speed_limit, 150.0);
  EXPECT_TRUE(percentage.forwarded_percentage);
  EXPECT_DOUBLE_EQ(percentage.guard_scale, 1.0);
}
