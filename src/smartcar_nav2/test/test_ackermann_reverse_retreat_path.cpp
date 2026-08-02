#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "smartcar_nav2/ackermann_reverse_retreat_path.hpp"
#include "smartcar_nav2/ackermann_reverse_retreat_odom_guard.hpp"
#include "smartcar_nav2/footprint_sweep_collision_source.hpp"

namespace
{

constexpr double kHalfPi = 1.57079632679489661923;

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "odom_combined";
  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(orientation);
  return result;
}

nav_msgs::msg::OccupancyGrid keepoutMask()
{
  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = "odom_combined";
  mask.info.resolution = 0.05F;
  mask.info.width = 100U;
  mask.info.height = 100U;
  mask.info.origin.orientation.w = 1.0;
  mask.data.assign(
    static_cast<std::size_t>(mask.info.width) * static_cast<std::size_t>(mask.info.height), 0);
  return mask;
}

smartcar_nav2::CostmapFootprintSweepOptions retreatSweepOptions()
{
  smartcar_nav2::CostmapFootprintSweepOptions options;
  options.half_length_m = 0.30;
  options.half_width_m = 0.16;
  options.sample_spacing_m = 0.025;
  options.lethal_cost_threshold = 253U;
  return options;
}

smartcar_nav2::AckermannReverseRetreatOdomSample odomSample(
  double x, double y, std::uint64_t sequence,
  const std::chrono::steady_clock::time_point & received_at)
{
  smartcar_nav2::AckermannReverseRetreatOdomSample sample;
  sample.x = x;
  sample.y = y;
  sample.sequence = sequence;
  sample.received_at = received_at;
  return sample;
}

TEST(AckermannReverseRetreatPath, UsesPhysicalNegativeXWithoutChangingYaw)
{
  nav_msgs::msg::Path path;
  ASSERT_TRUE(smartcar_nav2::buildAckermannReverseRetreatPath(
    pose(1.0, 2.0, kHalfPi), "odom_combined", 0.15, path));
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_NEAR(path.poses.front().pose.position.x, 1.0, 1.0e-12);
  EXPECT_NEAR(path.poses.front().pose.position.y, 2.0, 1.0e-12);
  EXPECT_NEAR(path.poses.back().pose.position.x, 1.0, 1.0e-12);
  EXPECT_NEAR(path.poses.back().pose.position.y, 1.85, 1.0e-12);
  EXPECT_NEAR(
    tf2::getYaw(path.poses.back().pose.orientation), kHalfPi, 1.0e-12);
}

TEST(AckermannReverseRetreatPath, RejectsUnsafeFramesAndDistances)
{
  nav_msgs::msg::Path path;
  auto wrong_frame = pose(0.0, 0.0, 0.0);
  wrong_frame.header.frame_id = "map";
  EXPECT_FALSE(smartcar_nav2::buildAckermannReverseRetreatPath(
    wrong_frame, "odom_combined", 0.15, path));
  EXPECT_FALSE(smartcar_nav2::buildAckermannReverseRetreatPath(
    pose(0.0, 0.0, 0.0), "odom_combined", 0.049, path));
  EXPECT_FALSE(smartcar_nav2::buildAckermannReverseRetreatPath(
    pose(0.0, 0.0, 0.0), "odom_combined", 0.251, path));

  auto invalid_orientation = pose(0.0, 0.0, 0.0);
  invalid_orientation.pose.orientation.w = 0.0;
  EXPECT_FALSE(smartcar_nav2::buildAckermannReverseRetreatPath(
    invalid_orientation, "odom_combined", 0.15, path));
}

TEST(AckermannReverseRetreatPath, StaticKeepoutSweepFailsClosedBeforeRetreat)
{
  nav_msgs::msg::Path path;
  ASSERT_TRUE(smartcar_nav2::buildAckermannReverseRetreatPath(
    pose(1.0, 1.0, 0.0), "odom_combined", 0.15, path));
  const auto options = retreatSweepOptions();

  auto mask = keepoutMask();
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &mask, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kClear);
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      nullptr, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kNoMask);

  auto wrong_frame = mask;
  wrong_frame.header.frame_id = "map";
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &wrong_frame, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kWrongFrame);

  auto malformed = mask;
  malformed.info.resolution = 0.0F;
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &malformed, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kMalformed);

  auto rotated_origin = mask;
  rotated_origin.info.origin.orientation.z = 0.1;
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &rotated_origin, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kMalformed);

  auto occupied = mask;
  occupied.data[20U * 100U + 18U] = 100;
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &occupied, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kOccupiedOrUnknown);

  auto too_small = mask;
  too_small.info.width = 1U;
  too_small.info.height = 1U;
  too_small.data.assign(1U, 0);
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &too_small, "odom_combined", path, options),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kOutOfBounds);
}

TEST(AckermannReverseRetreatPath, OdomGuardCapsCumulativeTravelEvenWhenNetDisplacementIsSmall)
{
  const auto now = std::chrono::steady_clock::now();
  smartcar_nav2::AckermannReverseRetreatOdomLimits limits;
  limits.maximum_age = std::chrono::milliseconds(500);
  limits.maximum_step_m = 0.10;
  limits.maximum_travel_m = 0.19;
  limits.maximum_displacement_m = 0.19;
  smartcar_nav2::AckermannReverseRetreatOdomGuard guard;
  ASSERT_TRUE(guard.arm(odomSample(0.0, 0.0, 1U, now), limits, now));

  EXPECT_EQ(
    guard.observe(odomSample(-0.06, 0.0, 2U, now), now),
    smartcar_nav2::AckermannReverseRetreatOdomResult::kClear);
  EXPECT_EQ(
    guard.observe(odomSample(0.0, 0.0, 3U, now), now),
    smartcar_nav2::AckermannReverseRetreatOdomResult::kClear);
  EXPECT_EQ(
    guard.observe(odomSample(-0.07, 0.0, 4U, now), now),
    smartcar_nav2::AckermannReverseRetreatOdomResult::kTravelExceeded);
  EXPECT_NEAR(guard.displacement_m(), 0.0, 1.0e-12);
}

TEST(AckermannReverseRetreatPath, OdomGuardCapsDisplacementAndRejectsBadSamples)
{
  const auto now = std::chrono::steady_clock::now();
  smartcar_nav2::AckermannReverseRetreatOdomLimits limits;
  limits.maximum_age = std::chrono::milliseconds(500);
  limits.maximum_step_m = 0.25;
  limits.maximum_travel_m = 0.25;
  limits.maximum_displacement_m = 0.19;
  smartcar_nav2::AckermannReverseRetreatOdomGuard guard;
  ASSERT_TRUE(guard.arm(odomSample(0.0, 0.0, 1U, now), limits, now));

  EXPECT_EQ(
    guard.observe(odomSample(-0.19, 0.0, 2U, now), now),
    smartcar_nav2::AckermannReverseRetreatOdomResult::kDisplacementExceeded);

  ASSERT_TRUE(guard.arm(odomSample(0.0, 0.0, 1U, now), limits, now));
  EXPECT_EQ(
    guard.observe(
      odomSample(0.0, 0.0, 2U, now - std::chrono::milliseconds(501)), now),
    smartcar_nav2::AckermannReverseRetreatOdomResult::kStale);
  EXPECT_EQ(
    guard.observe(odomSample(-0.26, 0.0, 2U, now), now),
    smartcar_nav2::AckermannReverseRetreatOdomResult::kStepTooLarge);
}

}  // namespace
