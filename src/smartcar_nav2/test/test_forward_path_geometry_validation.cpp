#include <cmath>
#include <initializer_list>

#include "gtest/gtest.h"

#include "smartcar_nav2/forward_path_geometry_validation.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

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

smartcar_nav2::ForwardPathGeometryValidationOptions options()
{
  return smartcar_nav2::ForwardPathGeometryValidationOptions{
    0.22, 0.20, 0.35, 0.15, 1.0e-4};
}

}  // namespace

TEST(ForwardPathGeometryValidation, AcceptsForwardArcAtConfiguredRadius)
{
  constexpr double radius = 0.22;
  const auto point = [=](double angle) {
      return makePose(
        radius * std::sin(angle), radius * (1.0 - std::cos(angle)), angle);
    };
  const auto path = makePath({point(0.0), point(0.05), point(0.10), point(0.15)});

  const auto validation = smartcar_nav2::validateForwardPathGeometry(
    path, options(), true);

  EXPECT_TRUE(validation.valid) << validation.reason;
}

TEST(ForwardPathGeometryValidation, RejectsQuantizedOrientationCurvatureSpike)
{
  // Each yaw remains forward-facing, but a 0.20 rad bin transition over
  // 0.04 m would demand a radius below the configured Ackermann envelope.
  const auto path = makePath({
    makePose(0.0, 0.0, 0.0),
    makePose(0.04, 0.0, 0.20),
    makePose(0.08, 0.0, 0.30),
  });

  const auto validation = smartcar_nav2::validateForwardPathGeometry(
    path, options(), false);

  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "orientation_curvature_exceeded");
  EXPECT_GT(validation.observed_value, validation.limit);
}

TEST(ForwardPathGeometryValidation, RejectsCentrelineCurvatureBelowMinimumRadius)
{
  constexpr double radius = 0.15;
  // Keep sampled headings within the allowed forward-direction cone so this
  // isolates the independent centreline curvature check.
  const auto path = makePath({
    makePose(0.0, 0.0, 0.0),
    makePose(radius * std::sin(0.10), radius * (1.0 - std::cos(0.10)), 0.05),
    makePose(radius * std::sin(0.20), radius * (1.0 - std::cos(0.20)), 0.10),
  });

  const auto validation = smartcar_nav2::validateForwardPathGeometry(
    path, options(), false);

  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "geometric_curvature_exceeded");
  EXPECT_GT(validation.observed_value, validation.limit);
}

TEST(ForwardPathGeometryValidation, RejectsLockedGoalTerminalTangentMismatch)
{
  const auto path = makePath({
    makePose(0.0, 0.0, 0.0),
    makePose(1.0, 0.0, 0.0),
    makePose(2.0, 0.0, 0.20),
  });

  const auto validation = smartcar_nav2::validateForwardPathGeometry(
    path, options(), true);

  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "terminal_tangent_mismatch");
  EXPECT_GT(validation.observed_value, validation.limit);
}

TEST(ForwardPathGeometryValidation, RejectsReverseProjection)
{
  const auto path = makePath({
    makePose(0.0, 0.0, kPi),
    makePose(1.0, 0.0, kPi),
    makePose(2.0, 0.0, kPi),
  });

  const auto validation = smartcar_nav2::validateForwardPathGeometry(
    path, options(), false);

  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.reason, "segment_not_forward");
  EXPECT_LT(validation.observed_value, validation.limit);
}
