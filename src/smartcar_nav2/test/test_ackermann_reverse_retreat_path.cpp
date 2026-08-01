#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "smartcar_nav2/ackermann_reverse_retreat_path.hpp"

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

}  // namespace
