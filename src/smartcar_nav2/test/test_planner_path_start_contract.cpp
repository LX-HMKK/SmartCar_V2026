#include <cmath>
#include <initializer_list>

#include "gtest/gtest.h"

#include "smartcar_nav2/planner_path_start_contract.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

nav_msgs::msg::Path makePath(
  const std::initializer_list<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom_combined";
  path.poses.assign(poses.begin(), poses.end());
  return path;
}

}  // namespace

TEST(PlannerPathStartContract, AcceptsAnExactNav2StartPose)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto path = makePath({start, makePose(1.0, 0.0, 0.0)});

  const auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.join_gap_m, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_error_rad, 0.0);
  EXPECT_DOUBLE_EQ(smartcar_nav2::plannerPathLengthIncludingStartJoin(path, result), 1.0);
}

TEST(PlannerPathStartContract, ChargesAnAllowedQuantizationGapToTheEdgeLength)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto path = makePath({
      makePose(0.04, 0.0, 0.09),
      makePose(1.04, 0.0, 0.09),
    });

  const auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.join_gap_m, 0.04, 1.0e-12);
  EXPECT_NEAR(result.yaw_error_rad, 0.09, 1.0e-12);
  EXPECT_NEAR(
    smartcar_nav2::plannerPathLengthIncludingStartJoin(path, result), 1.04, 1.0e-12);
}

TEST(PlannerPathStartContract, SnapsAnAllowedQuantizedStartToTheRequestedPose)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  auto path = makePath({
      makePose(0.04, 0.0, 0.09),
      makePose(1.04, 0.0, 0.09),
    });
  const auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(smartcar_nav2::snapPlannerPathStartToRequestedPose(path, start, result));
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y, start.pose.position.y);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.orientation.z, start.pose.orientation.z);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.orientation.w, start.pose.orientation.w);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 1.04);
}

TEST(PlannerPathStartContract, RefusesToSnapAnUnvalidatedStart)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  auto path = makePath({
      makePose(0.06, 0.0, 0.0),
      makePose(1.0, 0.0, 0.0),
    });
  const auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);

  ASSERT_FALSE(result.valid);
  EXPECT_FALSE(smartcar_nav2::snapPlannerPathStartToRequestedPose(path, start, result));
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 0.06);
}

TEST(PlannerPathStartContract, RejectsAStartPositionOutsideTheQuantizationAllowance)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto path = makePath({
      makePose(smartcar_nav2::kPlannerPathStartPositionToleranceM + 1.0e-3, 0.0, 0.0),
      makePose(1.0, 0.0, 0.0),
    });

  const auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "start_position_mismatch");
  EXPECT_TRUE(std::isnan(smartcar_nav2::plannerPathLengthIncludingStartJoin(path, result)));
}

TEST(PlannerPathStartContract, RejectsAStartYawOutsideTheQuantizationAllowance)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto path = makePath({
      makePose(0.0, 0.0, smartcar_nav2::kPlannerPathStartYawToleranceRad + 1.0e-3),
      makePose(1.0, 0.0, 0.0),
    });

  const auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "start_yaw_mismatch");
}

TEST(PlannerPathStartContract, RequiresPathAndFirstPoseFramesToMatchTheRequestedStart)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  auto path = makePath({start, makePose(1.0, 0.0, 0.0)});

  path.header.frame_id = "map";
  auto result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "path_frame_mismatch");

  path.header.frame_id = start.header.frame_id;
  path.poses.front().header.frame_id = "map";
  result = smartcar_nav2::validatePlannerPathStartContinuity(path, start);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "first_pose_frame_mismatch");
}
