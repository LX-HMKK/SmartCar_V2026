#include <cmath>

#include "gtest/gtest.h"

#include "smartcar_nav2/departure_connector.hpp"
#include "smartcar_nav2/forward_path_lateral_profile.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pStart()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.orientation.w = 1.0;
  return pose;
}

smartcar_nav2::ForwardPathLateralProfileStart profileStart()
{
  smartcar_nav2::ForwardPathLateralProfileStart start;
  start.frame_id = "odom_combined";
  start.position_tolerance_m = 0.001;
  start.yaw_tolerance_rad = 0.001;
  return start;
}

smartcar_nav2::DepartureConnectorOptions connectorOptions()
{
  smartcar_nav2::DepartureConnectorOptions options;
  options.minimum_turning_radius_m = 0.22;
  options.radius_margin_m = 0.28;
  options.sample_spacing_m = 0.025;
  return options;
}

constexpr double kPDepartureHighRightTurnRadiusM = 0.23;

}  // namespace

TEST(ForwardPathLateralProfile, KeepsTheLeftBudgetAndTapersThePDepartureRightSide)
{
  const auto initial = smartcar_nav2::forwardPathLateralEnvelopeAtStation(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.0, 0.12);
  ASSERT_TRUE(initial.has_value());
  EXPECT_NEAR(initial->left_cross_track_error_m, 0.12, 1.0e-12);
  EXPECT_NEAR(initial->right_cross_track_error_m, 0.075, 1.0e-12);

  const auto tapered = smartcar_nav2::forwardPathLateralEnvelopeAtStation(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.20, 0.12);
  ASSERT_TRUE(tapered.has_value());
  EXPECT_NEAR(tapered->left_cross_track_error_m, 0.12, 1.0e-12);
  EXPECT_NEAR(tapered->right_cross_track_error_m, 0.0075, 1.0e-12);

  const auto staged = smartcar_nav2::forwardPathLateralEnvelopeAtStation(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.225, 0.12);
  ASSERT_TRUE(staged.has_value());
  EXPECT_NEAR(staged->left_cross_track_error_m, 0.12, 1.0e-12);
  EXPECT_NEAR(staged->right_cross_track_error_m, 0.0075, 1.0e-12);

  const auto right_turn = smartcar_nav2::forwardPathLateralEnvelopeAtStation(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 1.507, 0.12);
  ASSERT_TRUE(right_turn.has_value());
  EXPECT_NEAR(right_turn->left_cross_track_error_m, 0.12, 1.0e-12);
  EXPECT_NEAR(right_turn->right_cross_track_error_m, 0.050, 1.0e-12);

  const auto late = smartcar_nav2::forwardPathLateralEnvelopeAtStation(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 4.0, 0.12);
  ASSERT_TRUE(late.has_value());
  EXPECT_NEAR(late->left_cross_track_error_m, 0.12, 1.0e-12);
  EXPECT_NEAR(late->right_cross_track_error_m, 0.050, 1.0e-12);
}

TEST(ForwardPathLateralProfile, EnforcesTheSignedPathSide)
{
  constexpr auto kProfile = smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1;
  EXPECT_FALSE(smartcar_nav2::forwardPathLateralCrossTrackExceeded(
    kProfile, 0.225, 0.119, 0.12));
  EXPECT_FALSE(smartcar_nav2::forwardPathLateralCrossTrackExceeded(
    kProfile, 0.225, -0.0075, 0.12));
  EXPECT_TRUE(smartcar_nav2::forwardPathLateralCrossTrackExceeded(
    kProfile, 0.225, -0.0076, 0.12));
  EXPECT_FALSE(smartcar_nav2::forwardPathLateralCrossTrackExceeded(
    kProfile, 1.507, -0.001, 0.12));
  EXPECT_TRUE(smartcar_nav2::forwardPathLateralCrossTrackExceeded(
    kProfile, 1.507, -0.0501, 0.12));
}

TEST(ForwardPathLateralProfile, MatchesOnlyTheGeneratedPConnectorPrefix)
{
  const auto connectors = smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
    pStart(), -0.75, -0.50, 0.025, 144U,
    kPDepartureHighRightTurnRadiusM, connectorOptions());
  ASSERT_EQ(connectors.size(), 1U);

  const auto signature = profileStart();
  EXPECT_EQ(
    smartcar_nav2::forwardPathLateralProfileMatchesPlan(
      smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1,
      connectors.front().path, signature),
    smartcar_nav2::ForwardPathLateralProfilePathMatch::kMatches);

  auto moved_start = connectors.front().path;
  moved_start.poses.front().pose.position.x += 0.002;
  EXPECT_EQ(
    smartcar_nav2::forwardPathLateralProfileMatchesPlan(
      smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, moved_start, signature),
    smartcar_nav2::ForwardPathLateralProfilePathMatch::kDoesNotMatch);

  auto malformed = connectors.front().path;
  malformed.poses.front().header.frame_id = "map";
  EXPECT_EQ(
    smartcar_nav2::forwardPathLateralProfileMatchesPlan(
      smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, malformed, signature),
    smartcar_nav2::ForwardPathLateralProfilePathMatch::kInvalid);
}

TEST(ForwardPathLateralProfile, RejectsWrongKinematicBudget)
{
  EXPECT_TRUE(smartcar_nav2::forwardPathLateralProfileConfigurationValid(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12, 0.22));
  EXPECT_FALSE(smartcar_nav2::forwardPathLateralProfileConfigurationValid(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.10, 0.22));
  EXPECT_FALSE(smartcar_nav2::forwardPathLateralProfileConfigurationValid(
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12, 0.25));
}
