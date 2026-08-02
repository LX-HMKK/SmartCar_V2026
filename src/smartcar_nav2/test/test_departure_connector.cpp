#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"
#include "smartcar_nav2/departure_connector.hpp"
#include "smartcar_nav2/local_costmap_tracking_envelope.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pStart()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom_combined";
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav2_msgs::msg::Costmap pFieldCostmap()
{
  nav2_msgs::msg::Costmap map;
  map.header.frame_id = "odom_combined";
  map.metadata.resolution = 0.025F;
  map.metadata.size_x = 220U;
  map.metadata.size_y = 220U;
  map.metadata.origin.position.x = -0.75;
  map.metadata.origin.position.y = -0.50;
  map.metadata.origin.orientation.w = 1.0;
  map.data.assign(
    static_cast<std::size_t>(map.metadata.size_x) * map.metadata.size_y, 0U);
  for (std::size_t y = 0U; y < map.metadata.size_y; ++y) {
    const double world_y = map.metadata.origin.position.y +
      (static_cast<double>(y) + 0.5) * map.metadata.resolution;
    for (std::size_t x = 0U; x < map.metadata.size_x; ++x) {
      const double world_x = map.metadata.origin.position.x +
        (static_cast<double>(x) + 0.5) * map.metadata.resolution;
      // Keep the P-side south boundary, physical A-zone cubes, the A1
      // raw-costmap witness, and the B walls as independent hard constraints.
      // The raw witness begins farther left than the cube because it includes
      // the live obstacle layer's observed lethal cells; losing it would let
      // the old 0.45 m first arc regress despite a clear centreline.
      if (world_y < -0.25 ||
        (world_x >= 0.65 && world_x <= 0.95 &&
        world_y >= 0.225 && world_y <= 0.55) ||
        (world_x >= 0.95 && world_x <= 1.25 &&
        world_y >= 0.15 && world_y <= 0.45) ||
        (world_x >= 0.85 && world_x <= 1.15 &&
        world_y >= 0.65 && world_y <= 0.95) ||
        (world_x >= 1.65 && world_x <= 1.95 &&
        world_y >= 0.75 && world_y <= 1.05) ||
        (world_x >= 2.30 && world_x <= 2.60 &&
        world_y >= 0.0 && world_y <= 0.30) ||
        (world_x >= 0.0 && world_x <= 2.0 &&
        world_y >= 1.75 && world_y <= 2.25) ||
        (world_x >= 3.0 && world_x <= 5.0 &&
        world_y >= 1.75 && world_y <= 2.25))
      {
        map.data[y * static_cast<std::size_t>(map.metadata.size_x) + x] = 254U;
      }
    }
  }
  return map;
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

TEST(DepartureConnector, BuildsOnlyForwardPDepartureArcs)
{
  const auto start = pStart();
  const auto options = connectorOptions();
  const auto connectors = smartcar_nav2::buildLeftDepartureConnectors(start, options);

  ASSERT_EQ(connectors.size(), 2U);
  for (const auto & connector : connectors) {
    const auto validation = smartcar_nav2::validateForwardConnectorPath(
      connector.path, start, options);
    EXPECT_TRUE(validation.valid) << validation.reason;
    EXPECT_NEAR(connector.radius_m, 0.50, 1.0e-12);
    EXPECT_LE(
      connector.arc_angle_rad,
      smartcar_nav2::kDepartureConnectorPi / 9.0 + 1.0e-12);
    EXPECT_GT(connector.length_m, 0.0);
    EXPECT_GT(connector.path.poses.back().pose.position.y, 0.0);
    EXPECT_GT(
      smartcar_nav2::departureConnectorYaw(
        connector.path.poses.back().pose.orientation),
      0.0);
    EXPECT_LE(
      validation.maximum_curvature,
      1.0 / options.minimum_turning_radius_m + options.curvature_tolerance);
  }
}

TEST(DepartureConnector, ClearsSouthBoundaryAndFirstAZoneConeWithThePaddedFootprint)
{
  const auto map = pFieldCostmap();
  const auto connectors = smartcar_nav2::buildLeftDepartureConnectors(
    pStart(), connectorOptions());
  const smartcar_nav2::CostmapFootprintSweepOptions footprint{
    0.30, 0.16, 0.025, 254U};

  ASSERT_EQ(connectors.size(), 2U);
  for (const auto & connector : connectors) {
    EXPECT_EQ(
      smartcar_nav2::costmapFootprintPathSweep(connector.path, map, footprint),
      smartcar_nav2::CostmapFootprintSweepResult::kClear);
  }
}

TEST(DepartureConnector, RefusesAnInvalidRadiusMargin)
{
  auto options = connectorOptions();
  options.radius_margin_m = 0.0;
  EXPECT_TRUE(
    smartcar_nav2::buildLeftDepartureConnectors(pStart(), options).empty());
}

TEST(DepartureConnector, RadiusGateEnablesOnlyTheSimulationEnvelope)
{
  auto options = connectorOptions();
  options.radius_margin_m = 0.28;
  EXPECT_TRUE(
    smartcar_nav2::departureConnectorRadiusWithinMaximum(options, 0.50));
  EXPECT_TRUE(smartcar_nav2::departureConnectorTerminalRadiusWithinEnvelope(
      options, 0.22, 0.50));
  EXPECT_TRUE(smartcar_nav2::departureConnectorHighRightTurnRadiusWithinEnvelope(
      options, kPDepartureHighRightTurnRadiusM, 0.50));
  EXPECT_FALSE(smartcar_nav2::departureConnectorHighRightTurnRadiusWithinEnvelope(
      options, 0.21, 0.50));
  EXPECT_FALSE(smartcar_nav2::departureConnectorHighRightTurnRadiusWithinEnvelope(
      options, 0.51, 0.50));

  options.minimum_turning_radius_m = 0.55;
  EXPECT_FALSE(
    smartcar_nav2::departureConnectorRadiusWithinMaximum(options, 0.50));
  EXPECT_FALSE(smartcar_nav2::departureConnectorTerminalRadiusWithinEnvelope(
      options, 0.22, 0.50));
  EXPECT_FALSE(smartcar_nav2::departureConnectorHighRightTurnRadiusWithinEnvelope(
      options, kPDepartureHighRightTurnRadiusM, 0.50));
}

TEST(DepartureConnector, BuildsLatticeAlignedArcAndStraightCandidates)
{
  constexpr double kOriginX = -0.75;
  constexpr double kOriginY = -0.50;
  constexpr double kResolution = 0.025;
  constexpr std::size_t kHeadingBins = 144U;
  const auto options = connectorOptions();

  const auto connectors = smartcar_nav2::buildLeftDepartureLatticeConnectors(
    pStart(), kOriginX, kOriginY, kResolution, kHeadingBins, options);

  ASSERT_FALSE(connectors.empty());
  const auto & connector = connectors.front();
  ASSERT_TRUE(connector.lattice_aligned);
  EXPECT_GE(
    connector.radius_m + 1.0e-9, options.minimum_turning_radius_m);
  EXPECT_LE(
    connector.radius_m,
    options.minimum_turning_radius_m + options.radius_margin_m + 1.0e-9);
  EXPECT_GT(connector.straight_length_m, 0.0);
  EXPECT_LT(connector.straight_length_m, 4.0 * kResolution + 1.0e-9);
  ASSERT_FALSE(connector.path.poses.empty());
  const auto & endpoint = connector.path.poses.back();
  const double grid_x = (endpoint.pose.position.x - kOriginX) / kResolution - 0.5;
  const double grid_y = (endpoint.pose.position.y - kOriginY) / kResolution - 0.5;
  EXPECT_NEAR(grid_x, std::round(grid_x), 1.0e-9);
  EXPECT_NEAR(grid_y, std::round(grid_y), 1.0e-9);
  const double heading_step = 2.0 * smartcar_nav2::kDepartureConnectorPi /
    static_cast<double>(kHeadingBins);
  const double heading_bin = smartcar_nav2::departureConnectorYaw(endpoint.pose.orientation) /
    heading_step;
  EXPECT_NEAR(heading_bin, std::round(heading_bin), 1.0e-9);
  EXPECT_NEAR(
    connector.arc_angle_rad,
    5.0 * smartcar_nav2::kDepartureConnectorPi / 36.0, 1.0e-9);
  EXPECT_TRUE(
    smartcar_nav2::validateForwardConnectorPath(connector.path, pStart(), options).valid);
  for (const auto & candidate : connectors) {
    smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;
    const auto sweep_result = smartcar_nav2::costmapFootprintPathSweep(
      candidate.path, pFieldCostmap(), smartcar_nav2::CostmapFootprintSweepOptions{
        0.30, 0.16, 0.025, 254U}, &diagnostic);
    EXPECT_EQ(
      sweep_result,
      smartcar_nav2::CostmapFootprintSweepResult::kClear)
      << "arc_deg=" << candidate.arc_angle_rad * 180.0 / smartcar_nav2::kDepartureConnectorPi
      << " radius_m=" << candidate.radius_m
      << " sample=(" << diagnostic.sample_pose.pose.position.x << ","
      << diagnostic.sample_pose.pose.position.y << ","
      << smartcar_nav2::departureConnectorYaw(diagnostic.sample_pose.pose.orientation) << ")"
      << " blocking=(" << diagnostic.blocking_cell_world_x << ","
      << diagnostic.blocking_cell_world_y << ")";
  }
}

TEST(DepartureConnector, LatticeCandidatesFollowTheSuppliedGridOrigin)
{
  constexpr double kResolution = 0.025;
  const auto options = connectorOptions();
  const auto first = smartcar_nav2::buildLeftDepartureLatticeConnectors(
    pStart(), -0.75, -0.50, kResolution, 144U, options);
  const auto shifted = smartcar_nav2::buildLeftDepartureLatticeConnectors(
    pStart(), -0.74, -0.49, kResolution, 144U, options);

  ASSERT_FALSE(first.empty());
  ASSERT_FALSE(shifted.empty());
  const auto & first_endpoint = first.front().path.poses.back().pose.position;
  const auto & shifted_endpoint = shifted.front().path.poses.back().pose.position;
  EXPECT_GT(std::hypot(
      first_endpoint.x - shifted_endpoint.x,
      first_endpoint.y - shifted_endpoint.y),
    1.0e-4);
  for (const auto * connector : {&first.front(), &shifted.front()}) {
    EXPECT_TRUE(connector->lattice_aligned);
    EXPECT_TRUE(
      smartcar_nav2::validateForwardConnectorPath(connector->path, pStart(), options).valid);
  }
}

TEST(DepartureConnector, PDepartureUsesHighRightTurnBeforeEastSmacHandoff)
{
  constexpr double kOriginX = -0.75;
  constexpr double kOriginY = -0.50;
  constexpr double kResolution = 0.025;
  constexpr std::size_t kHeadingBins = 144U;
  auto options = connectorOptions();
  options.radius_margin_m = 0.28;

  const auto connectors = smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
    pStart(), kOriginX, kOriginY, kResolution, kHeadingBins,
    kPDepartureHighRightTurnRadiusM, options);

  ASSERT_EQ(connectors.size(), 1U);
  const auto & connector = connectors.front();
  ASSERT_TRUE(connector.lattice_aligned);
  ASSERT_FALSE(connector.path.poses.empty());
  const auto validation = smartcar_nav2::validateForwardConnectorPath(
    connector.path, pStart(), options);
  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_GE(
    connector.radius_m + 1.0e-9, options.minimum_turning_radius_m);
  EXPECT_LE(
    connector.radius_m,
    options.minimum_turning_radius_m + options.radius_margin_m + 1.0e-9);
  EXPECT_NEAR(connector.radius_m, 0.22, 1.0e-9);
  EXPECT_NEAR(
    connector.high_right_turn_radius_m, kPDepartureHighRightTurnRadiusM, 1.0e-9);
  EXPECT_NEAR(
    connector.arc_angle_rad,
    13.0 * smartcar_nav2::kDepartureConnectorPi / 72.0, 1.0e-9);
  EXPECT_NEAR(connector.length_m, 3.027415, 1.0e-5);
  EXPECT_GE(connector.straight_length_m, 2.00);
  EXPECT_LE(connector.straight_length_m, 2.45);

  const auto & endpoint = connector.path.poses.back();
  const double grid_x = (endpoint.pose.position.x - kOriginX) / kResolution - 0.5;
  const double grid_y = (endpoint.pose.position.y - kOriginY) / kResolution - 0.5;
  EXPECT_NEAR(grid_x, std::round(grid_x), 1.0e-9);
  EXPECT_NEAR(grid_y, std::round(grid_y), 1.0e-9);
  EXPECT_NEAR(
    smartcar_nav2::departureConnectorYaw(endpoint.pose.orientation),
    0.0, 1.0e-9);
  EXPECT_NEAR(endpoint.pose.position.x, 1.9625, 1.0e-9);
  // The 32.5-degree lead-in leaves the lattice one row farther north than
  // the former 20-degree connector, preserving clearance from the P-side
  // boundary without widening the right-side tracking allowance.
  EXPECT_NEAR(endpoint.pose.position.y, 1.2875, 1.0e-9);

  double maximum_yaw = -std::numeric_limits<double>::infinity();
  for (const auto & pose : connector.path.poses) {
    const double yaw = smartcar_nav2::departureConnectorYaw(pose.pose.orientation);
    maximum_yaw = std::max(maximum_yaw, yaw);
  }
  EXPECT_NEAR(maximum_yaw, smartcar_nav2::kDepartureConnectorPi / 2.0, 1.0e-9);

  std::size_t high_right_start = connector.path.poses.size();
  for (std::size_t index = 1U; index < connector.path.poses.size(); ++index) {
    const double previous_yaw = smartcar_nav2::departureConnectorYaw(
      connector.path.poses[index - 1U].pose.orientation);
    const double current_yaw = smartcar_nav2::departureConnectorYaw(
      connector.path.poses[index].pose.orientation);
    if (std::abs(previous_yaw - smartcar_nav2::kDepartureConnectorPi / 2.0) < 1.0e-9 &&
      current_yaw < smartcar_nav2::kDepartureConnectorPi / 2.0 - 1.0e-9)
    {
      high_right_start = index - 1U;
      break;
    }
  }
  ASSERT_LT(high_right_start, connector.path.poses.size());
  std::size_t high_right_end = connector.path.poses.size();
  for (std::size_t index = high_right_start + 1U;
    index < connector.path.poses.size(); ++index)
  {
    const double yaw = smartcar_nav2::departureConnectorYaw(
      connector.path.poses[index].pose.orientation);
    if (std::abs(yaw) < 1.0e-9) {
      high_right_end = index;
      break;
    }
  }
  ASSERT_LT(high_right_end, connector.path.poses.size());
  const auto & high_right_begin_pose = connector.path.poses[high_right_start].pose;
  const auto & high_right_end_pose = connector.path.poses[high_right_end].pose;
  EXPECT_NEAR(
    high_right_end_pose.position.x - high_right_begin_pose.position.x,
    kPDepartureHighRightTurnRadiusM, 1.0e-9);
  EXPECT_NEAR(
    high_right_end_pose.position.y - high_right_begin_pose.position.y,
    kPDepartureHighRightTurnRadiusM, 1.0e-9);

  smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;
  const auto sweep_result = smartcar_nav2::costmapFootprintPathSweep(
    connector.path, pFieldCostmap(), smartcar_nav2::CostmapFootprintSweepOptions{
      0.30, 0.16, 0.025, 254U}, &diagnostic);
  EXPECT_EQ(sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kClear)
    << "sample=(" << diagnostic.sample_pose.pose.position.x << ","
    << diagnostic.sample_pose.pose.position.y << ","
    << smartcar_nav2::departureConnectorYaw(diagnostic.sample_pose.pose.orientation) << ")"
    << " blocking=(" << diagnostic.blocking_cell_world_x << ","
      << diagnostic.blocking_cell_world_y << ")";
}

TEST(DepartureConnector, PDepartureLeadInUsesTheConfiguredClearanceEnvelope)
{
  constexpr double kOriginX = -0.75;
  constexpr double kOriginY = -0.50;
  constexpr double kResolution = 0.025;
  constexpr std::size_t kHeadingBins = 144U;
  auto options = connectorOptions();
  options.radius_margin_m = 0.28;
  const double lead_in_radius =
    options.minimum_turning_radius_m + options.radius_margin_m - 2.0 * kResolution;

  const auto connectors = smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
    pStart(), kOriginX, kOriginY, kResolution, kHeadingBins,
    kPDepartureHighRightTurnRadiusM, options);

  ASSERT_EQ(connectors.size(), 1U);
  const auto & path = connectors.front().path;
  const std::size_t first_arc_samples = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(
      lead_in_radius * 13.0 * smartcar_nav2::kDepartureConnectorPi / 72.0 /
      options.sample_spacing_m)));
  ASSERT_GT(path.poses.size(), first_arc_samples);
  const auto & first_arc_end = path.poses.at(first_arc_samples).pose;
  EXPECT_NEAR(
    first_arc_end.position.x,
    lead_in_radius * std::sin(13.0 * smartcar_nav2::kDepartureConnectorPi / 72.0),
    1.0e-9);
  EXPECT_NEAR(
    first_arc_end.position.y,
    lead_in_radius * (1.0 - std::cos(13.0 * smartcar_nav2::kDepartureConnectorPi / 72.0)),
    1.0e-9);
  EXPECT_TRUE(
    smartcar_nav2::validateForwardConnectorPath(path, pStart(), options).valid);
}

TEST(DepartureConnector, PDepartureClearsTheOneSidedTrackingEnvelope)
{
  constexpr double kOriginX = -0.75;
  constexpr double kOriginY = -0.50;
  constexpr double kResolution = 0.025;
  constexpr std::size_t kHeadingBins = 144U;
  auto options = connectorOptions();
  options.radius_margin_m = 0.28;

  const auto connectors = smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
    pStart(), kOriginX, kOriginY, kResolution, kHeadingBins,
    kPDepartureHighRightTurnRadiusM, options);
  ASSERT_EQ(connectors.size(), 1U);

  smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;
  const auto result = smartcar_nav2::localCostmapTrackingProfiledFootprintPathSweep(
    connectors.front().path, pFieldCostmap(),
    smartcar_nav2::CostmapFootprintSweepOptions{0.30, 0.16, 0.025, 254U},
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12, &diagnostic);
  EXPECT_EQ(result, smartcar_nav2::CostmapFootprintSweepResult::kClear)
    << "sample=(" << diagnostic.sample_pose.pose.position.x << ","
    << diagnostic.sample_pose.pose.position.y << ","
    << smartcar_nav2::departureConnectorYaw(diagnostic.sample_pose.pose.orientation) << ")"
    << " blocking=(" << diagnostic.blocking_cell_world_x << ","
    << diagnostic.blocking_cell_world_y << ")";
}

TEST(DepartureConnector, PDepartureRslTerminalUsesSimulationMinimumRadiusAndSweep)
{
  constexpr double kOriginX = -0.75;
  constexpr double kOriginY = -0.50;
  constexpr double kResolution = 0.025;
  constexpr std::size_t kHeadingBins = 144U;
  auto options = connectorOptions();
  options.radius_margin_m = 0.28;

  const auto departures = smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
    pStart(), kOriginX, kOriginY, kResolution, kHeadingBins,
    kPDepartureHighRightTurnRadiusM, options);
  ASSERT_EQ(departures.size(), 1U);
  const auto & departure = departures.front().path;

  geometry_msgs::msg::PoseStamped task_goal;
  task_goal.header.frame_id = "odom_combined";
  task_goal.pose.position.x = 3.1272949273;
  task_goal.pose.position.y = 0.9765623266;
  task_goal.pose.orientation = smartcar_nav2::departureConnectorQuaternionFromYaw(0.3026758824);

  nav_msgs::msg::Path terminal;
  ASSERT_TRUE(smartcar_nav2::buildPDepartureRslTerminalConnector(
    departure.poses.back(), task_goal, 0.22, options, terminal));
  ASSERT_GE(terminal.poses.size(), 3U);
  const double terminal_chord_m = std::hypot(
    task_goal.pose.position.x - departure.poses.back().pose.position.x,
    task_goal.pose.position.y - departure.poses.back().pose.position.y);
  EXPECT_LE(
    smartcar_nav2::departureConnectorPathLength(terminal),
    terminal_chord_m * 1.60);
  const auto & terminal_pose = terminal.poses.back();
  EXPECT_NEAR(terminal_pose.pose.position.x, task_goal.pose.position.x, 1.0e-6);
  EXPECT_NEAR(terminal_pose.pose.position.y, task_goal.pose.position.y, 1.0e-6);
  EXPECT_NEAR(
    smartcar_nav2::departureConnectorAngularDistance(
      smartcar_nav2::departureConnectorYaw(terminal_pose.pose.orientation),
      smartcar_nav2::departureConnectorYaw(task_goal.pose.orientation)),
    0.0, 1.0e-6);

  nav_msgs::msg::Path complete = departure;
  complete.poses.insert(complete.poses.end(), std::next(terminal.poses.begin()), terminal.poses.end());
  const auto validation = smartcar_nav2::validateForwardConnectorPath(
    complete, pStart(), options);
  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_LT(validation.length_m, 4.61);
  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      complete, pFieldCostmap(), smartcar_nav2::CostmapFootprintSweepOptions{
        0.30, 0.16, 0.025, 254U}),
    smartcar_nav2::CostmapFootprintSweepResult::kClear);

  // Exercise the full staged departure and the deterministic RSL terminal,
  // rather than only its first 0.50 m planning horizon. The 50 mm right
  // allowance begins only after the northbound escape and must not reopen the
  // P-side boundary or the A-zone keepouts.
  smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;
  EXPECT_EQ(
    smartcar_nav2::localCostmapTrackingProfiledFootprintPathSweep(
      complete, pFieldCostmap(), smartcar_nav2::CostmapFootprintSweepOptions{
        0.30, 0.16, 0.025, 254U},
      smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12, &diagnostic),
    smartcar_nav2::CostmapFootprintSweepResult::kClear)
    << "sample=(" << diagnostic.sample_pose.pose.position.x << ","
    << diagnostic.sample_pose.pose.position.y << ","
    << smartcar_nav2::departureConnectorYaw(diagnostic.sample_pose.pose.orientation) << ")"
    << " blocking=(" << diagnostic.blocking_cell_world_x << ","
    << diagnostic.blocking_cell_world_y << ")";
}

TEST(DepartureConnector, ScansForTheMinimumSmootherPDepartureRslTerminal)
{
  constexpr double kOriginX = -0.75;
  constexpr double kOriginY = -0.50;
  constexpr double kResolution = 0.025;
  constexpr std::size_t kHeadingBins = 144U;
  auto options = connectorOptions();
  options.radius_margin_m = 0.28;

  const auto departures = smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
    pStart(), kOriginX, kOriginY, kResolution, kHeadingBins,
    kPDepartureHighRightTurnRadiusM, options);
  ASSERT_EQ(departures.size(), 1U);

  geometry_msgs::msg::PoseStamped task_goal;
  task_goal.header.frame_id = "odom_combined";
  task_goal.pose.position.x = 3.1272949273;
  task_goal.pose.position.y = 0.9765623266;
  task_goal.pose.orientation = smartcar_nav2::departureConnectorQuaternionFromYaw(0.3026758824);

  constexpr int kMinimumRadiusCentimetres = 22;
  constexpr int kMaximumRadiusCentimetres = 50;
  double minimum_smoother_radius_m = std::numeric_limits<double>::quiet_NaN();
  double minimum_smoother_length_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t clear_candidate_count = 0U;
  for (int centimetres = kMinimumRadiusCentimetres;
    centimetres <= kMaximumRadiusCentimetres; ++centimetres)
  {
    const double radius_m = static_cast<double>(centimetres) / 100.0;
    nav_msgs::msg::Path terminal;
    if (!smartcar_nav2::buildPDepartureRslTerminalConnector(
        departures.front().path.poses.back(), task_goal, radius_m, options, terminal))
    {
      continue;
    }
    nav_msgs::msg::Path complete = departures.front().path;
    complete.poses.insert(
      complete.poses.end(), std::next(terminal.poses.begin()), terminal.poses.end());
    if (!smartcar_nav2::validateForwardConnectorPath(complete, pStart(), options).valid ||
      smartcar_nav2::costmapFootprintPathSweep(
        complete, pFieldCostmap(), smartcar_nav2::CostmapFootprintSweepOptions{
          0.30, 0.16, 0.025, 254U}) != smartcar_nav2::CostmapFootprintSweepResult::kClear ||
      smartcar_nav2::localCostmapTrackingProfiledFootprintPathSweep(
        complete, pFieldCostmap(), smartcar_nav2::CostmapFootprintSweepOptions{
          0.30, 0.16, 0.025, 254U},
        smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12) !=
      smartcar_nav2::CostmapFootprintSweepResult::kClear)
    {
      continue;
    }
    ++clear_candidate_count;
    if (!std::isfinite(minimum_smoother_radius_m) &&
      radius_m > options.minimum_turning_radius_m + 1.0e-9)
    {
      minimum_smoother_radius_m = radius_m;
      minimum_smoother_length_m = smartcar_nav2::departureConnectorPathLength(complete);
    }
  }

  ::testing::Test::RecordProperty(
    "minimum_smoother_rsl_radius_m", std::to_string(minimum_smoother_radius_m));
  ::testing::Test::RecordProperty(
    "minimum_smoother_rsl_length_m", std::to_string(minimum_smoother_length_m));
  ::testing::Test::RecordProperty(
    "clear_rsl_radius_candidate_count", std::to_string(clear_candidate_count));
  ASSERT_TRUE(std::isfinite(minimum_smoother_radius_m));
  EXPECT_GT(minimum_smoother_radius_m, options.minimum_turning_radius_m);
  EXPECT_LT(minimum_smoother_radius_m,
    options.minimum_turning_radius_m + options.radius_margin_m + 1.0e-9);
  EXPECT_TRUE(std::isfinite(minimum_smoother_length_m));
  EXPECT_NEAR(minimum_smoother_radius_m, 0.23, 1.0e-9);
  EXPECT_NEAR(minimum_smoother_length_m, 4.241263, 1.0e-5);
  EXPECT_EQ(clear_candidate_count, 29U);
}

TEST(DepartureConnector, RefusesInvalidLatticeMetadata)
{
  const auto options = connectorOptions();
  EXPECT_TRUE(smartcar_nav2::buildLeftDepartureLatticeConnectors(
      pStart(), -0.75, -0.50, 0.0, 144U, options).empty());
  EXPECT_TRUE(smartcar_nav2::buildLeftDepartureLatticeConnectors(
      pStart(), -0.75, -0.50, 0.025, 0U, options).empty());
  EXPECT_TRUE(smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
      pStart(), -0.75, -0.50, 0.025, 144U, 0.21, options).empty());
  EXPECT_TRUE(smartcar_nav2::buildPDepartureEscapeLatticeConnectors(
      pStart(), -0.75, -0.50, 0.025, 144U, 0.51, options).empty());
}

}  // namespace
