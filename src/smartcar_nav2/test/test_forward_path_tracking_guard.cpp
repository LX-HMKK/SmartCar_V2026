#include <cmath>
#include <initializer_list>

#include "gtest/gtest.h"

#include "smartcar_nav2/forward_path_tracking_guard.hpp"

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

TEST(ForwardPathTrackingGuard, ProjectsStationCrossTrackAndHeading)
{
  const auto path = makePath(
  {
    makePose(0.0, 0.0, 0.0),
    makePose(1.0, 0.0, 0.0),
    makePose(1.0, 1.0, 1.5707963267948966),
  });

  const auto projection = smartcar_nav2::projectForwardPathTrackingPose(
    path, makePose(0.70, 0.20, 0.10));

  ASSERT_TRUE(projection.valid) << projection.reason;
  EXPECT_EQ(projection.segment_index, 0U);
  EXPECT_NEAR(projection.station_m, 0.70, 1.0e-12);
  EXPECT_NEAR(projection.cross_track_m, 0.20, 1.0e-12);
  EXPECT_NEAR(projection.path_heading_error_rad, 0.10, 1.0e-12);
  EXPECT_TRUE(smartcar_nav2::forwardPathTrackingCrossTrackExceeded(projection, 0.12));
}

TEST(ForwardPathTrackingGuard, UsesTheClosestLaterSegment)
{
  const auto path = makePath(
  {
    makePose(0.0, 0.0, 0.0),
    makePose(1.0, 0.0, 0.0),
    makePose(1.0, 1.0, 1.5707963267948966),
  });

  const auto projection = smartcar_nav2::projectForwardPathTrackingPose(
    path, makePose(1.08, 0.75, 1.5707963267948966));

  ASSERT_TRUE(projection.valid) << projection.reason;
  EXPECT_EQ(projection.segment_index, 1U);
  EXPECT_NEAR(projection.station_m, 1.75, 1.0e-12);
  EXPECT_NEAR(projection.cross_track_m, 0.08, 1.0e-12);
  EXPECT_NEAR(projection.path_heading_error_rad, 0.0, 1.0e-12);
  EXPECT_FALSE(smartcar_nav2::forwardPathTrackingCrossTrackExceeded(projection, 0.12));
}

TEST(ForwardPathTrackingGuard, ExtractsSignedErrorAndLocalTangentCurvature)
{
  // Three poses on a 0.50 m left arc sampled at 0.10 rad intervals. The
  // tangent quaternions are the planner contract that curvature tracking uses
  // instead of allowing a distant carrot to shortcut this narrow egress.
  const auto path = makePath(
  {
    makePose(0.0, 0.0, 0.0),
    makePose(0.5 * std::sin(0.10), 0.5 * (1.0 - std::cos(0.10)), 0.10),
    makePose(0.5 * std::sin(0.20), 0.5 * (1.0 - std::cos(0.20)), 0.20),
  });
  const auto projection = smartcar_nav2::projectForwardPathTrackingPose(
    path, makePose(0.5 * std::sin(0.05), 0.5 * (1.0 - std::cos(0.05)) - 0.01, 0.05));

  ASSERT_TRUE(projection.valid) << projection.reason;
  EXPECT_LT(projection.signed_cross_track_m, 0.0);
  EXPECT_NEAR(projection.remaining_path_m, 0.075, 0.002);
  const auto curvature = smartcar_nav2::forwardPathTrackingLocalCurvature(path, projection);
  ASSERT_TRUE(curvature.valid) << curvature.reason;
  EXPECT_NEAR(curvature.curvature_m_inv, 2.0, 0.02);
}

TEST(ForwardPathTrackingGuard, DetectsTightCurveInsidePreviewWindow)
{
  // A short straight leads to a 0.22 m left arc. The controller must start
  // its steering-settle speed cap before it reaches the first tight sample.
  const auto path = makePath(
  {
    makePose(0.0, 0.0, 0.0),
    makePose(0.10, 0.0, 0.0),
    makePose(0.10 + 0.22 * std::sin(0.20), 0.22 * (1.0 - std::cos(0.20)), 0.20),
    makePose(0.10 + 0.22 * std::sin(0.40), 0.22 * (1.0 - std::cos(0.40)), 0.40),
  });
  const auto projection = smartcar_nav2::projectForwardPathTrackingPose(
    path, makePose(0.02, 0.0, 0.0));

  ASSERT_TRUE(projection.valid) << projection.reason;
  EXPECT_TRUE(smartcar_nav2::forwardPathTrackingTightTurnAhead(
    path, projection, 0.30, 0.15));
  EXPECT_FALSE(smartcar_nav2::forwardPathTrackingTightTurnAhead(
    path, projection, 0.20, 0.15));
  EXPECT_FALSE(smartcar_nav2::forwardPathTrackingTightTurnAhead(
    path, projection, 0.30, 0.04));
}

TEST(ForwardPathTrackingGuard, RejectsMalformedOrIncompatiblePaths)
{
  const auto robot = makePose(0.0, 0.0, 0.0);
  auto frame_mismatch = makePath({makePose(0.0, 0.0, 0.0), makePose(1.0, 0.0, 0.0)});
  frame_mismatch.poses.front().header.frame_id = "map";
  auto projection = smartcar_nav2::projectForwardPathTrackingPose(frame_mismatch, robot);
  EXPECT_FALSE(projection.valid);
  EXPECT_EQ(projection.reason, "path_pose_invalid");
  EXPECT_TRUE(smartcar_nav2::forwardPathTrackingCrossTrackExceeded(projection, 0.12));

  const auto zero_length = makePath({makePose(0.0, 0.0, 0.0), makePose(0.0, 0.0, 0.0)});
  projection = smartcar_nav2::projectForwardPathTrackingPose(zero_length, robot);
  EXPECT_FALSE(projection.valid);
  EXPECT_EQ(projection.reason, "path_has_no_nonzero_segment");
}

TEST(ForwardPathTrackingGuard, EnablesShortLookaheadOnlyNearTheEndpoint)
{
  const auto path = makePath(
  {
    makePose(0.0, 0.0, 0.0),
    makePose(1.0, 0.0, 0.0),
    makePose(2.0, 0.0, 0.0),
  });
  const auto far_projection = smartcar_nav2::projectForwardPathTrackingPose(
    path, makePose(1.20, 0.0, 0.0));
  const auto near_projection = smartcar_nav2::projectForwardPathTrackingPose(
    path, makePose(1.60, 0.0, 0.0));

  ASSERT_TRUE(far_projection.valid);
  ASSERT_TRUE(near_projection.valid);
  EXPECT_FALSE(smartcar_nav2::forwardPathTrackingTerminalLookaheadActive(
    path, far_projection, 0.05, 0.50));
  EXPECT_TRUE(smartcar_nav2::forwardPathTrackingTerminalLookaheadActive(
    path, near_projection, 0.05, 0.50));
  EXPECT_FALSE(smartcar_nav2::forwardPathTrackingTerminalLookaheadActive(
    path, near_projection, 0.60, 0.50));
}
