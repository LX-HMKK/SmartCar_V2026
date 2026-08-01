#include <cmath>
#include <initializer_list>
#include <limits>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "smartcar_nav2/reverse_path_utils.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(orientation);
  return pose;
}

geometry_msgs::msg::Quaternion makeZeroQuaternion()
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 0.0;
  return orientation;
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

TEST(ReversePathUtils, HalfTurnIsAnInvolutionForPlanarYaw)
{
  const auto original = makePose(1.0, -2.0, 0.6);
  geometry_msgs::msg::PoseStamped once;
  geometry_msgs::msg::PoseStamped twice;
  ASSERT_TRUE(smartcar_nav2::rotatePoseYawByPi(original, once));
  ASSERT_TRUE(smartcar_nav2::rotatePoseYawByPi(once, twice));
  EXPECT_DOUBLE_EQ(twice.pose.position.x, original.pose.position.x);
  EXPECT_DOUBLE_EQ(twice.pose.position.y, original.pose.position.y);
  EXPECT_NEAR(
    std::remainder(
      tf2::getYaw(twice.pose.orientation) - tf2::getYaw(original.pose.orientation),
      2.0 * kPi),
    0.0, 1.0e-12);
}

TEST(ReversePathUtils, AcceptsFiniteContinuousReversePath)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-1.0, 0.0, 0.0);
  const auto path = makePath(
  {
    start,
    makePose(-0.5, 0.0, 0.0),
    goal,
  });

  const auto result = smartcar_nav2::validateReversePath(
    path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(ReversePathUtils, RejectsForwardSegmentAndCusp)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-0.25, 0.0, 0.0);
  const auto path = makePath(
  {
    start,
    makePose(-0.5, 0.0, 0.0),
    goal,
  });

  const auto result = smartcar_nav2::validateReversePath(
    path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "segment_not_reverse");
  EXPECT_EQ(result.segment_index, 1u);
}

TEST(ReversePathUtils, RejectsSegmentOutsideMaximumDirectionError)
{
  const auto start = makePose(0.0, 0.0, 0.50);
  const auto goal = makePose(-1.0, 0.0, 0.50);
  const auto path = makePath(
  {
    start,
    makePose(-0.5, 0.0, 0.50),
    goal,
  });
  smartcar_nav2::ReversePathValidationOptions options;
  options.maximum_direction_error = 0.35;

  const auto result = smartcar_nav2::validateReversePath(path, start, goal, options);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "segment_not_reverse");
  EXPECT_GT(result.observed_value, result.limit);
}

TEST(ReversePathUtils, RejectsCurvatureAboveConfiguredLimit)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-0.05, 0.05, -kPi / 2.0);
  const auto path = makePath(
  {
    start,
    makePose(-0.05, 0.0, -kPi / 4.0),
    goal,
  });
  smartcar_nav2::ReversePathValidationOptions options;
  options.maximum_direction_error = 0.8;
  options.goal_yaw_tolerance = 0.01;

  const auto result = smartcar_nav2::validateReversePath(path, start, goal, options);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "curvature_exceeded");
}

TEST(ReversePathUtils, IgnoresQuantizedYawSpikesOnStraightGeometry)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-0.04, 0.0, 0.10);
  const auto path = makePath(
  {
    start,
    makePose(-0.02, 0.0, 0.05),
    goal,
  });

  const auto result = smartcar_nav2::validateReversePath(
    path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(ReversePathUtils, RejectsInvalidPoseAndWrongGoal)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-1.0, 0.0, 0.0);
  auto invalid_path = makePath({start, goal});
  invalid_path.poses[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  auto result = smartcar_nav2::validateReversePath(
    invalid_path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "pose_invalid");

  const auto short_path = makePath({start, makePose(-0.5, 0.0, 0.0)});
  result = smartcar_nav2::validateReversePath(
    short_path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "goal_position_mismatch");
}

TEST(ReversePathUtils, RejectsZeroQuaternionExpectedOrientations)
{
  const auto path_start = makePose(0.0, 0.0, 0.0);
  const auto path_goal = makePose(-1.0, 0.0, 0.0);
  auto expected_start = path_start;
  auto expected_goal = path_goal;
  expected_start.pose.orientation = makeZeroQuaternion();
  expected_goal.pose.orientation = makeZeroQuaternion();
  const auto path = makePath(
  {
    path_start,
    makePose(-0.5, 0.0, 0.0),
    path_goal,
  });

  const auto result = smartcar_nav2::validateReversePath(
    path, expected_start, expected_goal,
    smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "expected_pose_invalid");
}

TEST(ReversePathUtils, RejectsZeroQuaternionRotation)
{
  geometry_msgs::msg::Quaternion output;
  EXPECT_FALSE(smartcar_nav2::rotateYawByPi(makeZeroQuaternion(), output));
}

TEST(ReversePathUtils, RejectsUnconstrainedQuaternionInPlannedPath)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-1.0, 0.0, 0.0);
  auto path = makePath(
  {
    start,
    makePose(-0.5, 0.0, 0.0),
    goal,
  });
  path.poses[1].pose.orientation = makeZeroQuaternion();

  const auto result = smartcar_nav2::validateReversePath(
    path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "pose_invalid");
  EXPECT_EQ(result.segment_index, 1u);
}

TEST(ReversePathUtils, RejectsInvalidMaximumDirectionError)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-1.0, 0.0, 0.0);
  const auto path = makePath({start, goal});
  smartcar_nav2::ReversePathValidationOptions options;

  options.maximum_direction_error = 0.0;
  auto result = smartcar_nav2::validateReversePath(path, start, goal, options);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "options_invalid");

  options.maximum_direction_error = kPi / 2.0;
  result = smartcar_nav2::validateReversePath(path, start, goal, options);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "options_invalid");
}

TEST(ReversePathUtils, RejectsFrameMismatchAndZeroLengthSegments)
{
  const auto start = makePose(0.0, 0.0, 0.0);
  const auto goal = makePose(-1.0, 0.0, 0.0);
  auto path = makePath({start, start, goal});
  auto result = smartcar_nav2::validateReversePath(
    path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "segment_too_short");

  path = makePath({start, goal});
  path.poses[1].header.frame_id = "map";
  result = smartcar_nav2::validateReversePath(
    path, start, goal, smartcar_nav2::ReversePathValidationOptions());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "pose_frame_mismatch");
}
