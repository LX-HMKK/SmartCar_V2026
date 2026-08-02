#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "smartcar_nav2/local_costmap_tracking_envelope.hpp"

namespace
{

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

nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path result;
  result.header.frame_id = "odom_combined";
  result.poses.assign(poses);
  return result;
}

nav2_msgs::msg::Costmap costmap()
{
  nav2_msgs::msg::Costmap result;
  result.header.frame_id = "odom_combined";
  result.metadata.resolution = 0.1F;
  result.metadata.size_x = 40U;
  result.metadata.size_y = 40U;
  result.metadata.origin.orientation.w = 1.0;
  result.data.assign(
    static_cast<std::size_t>(result.metadata.size_x) * result.metadata.size_y,
    0U);
  return result;
}

nav2_msgs::msg::Costmap fineCostmap()
{
  nav2_msgs::msg::Costmap result;
  result.header.frame_id = "odom_combined";
  result.metadata.resolution = 0.025F;
  result.metadata.size_x = 120U;
  result.metadata.size_y = 120U;
  result.metadata.origin.orientation.w = 1.0;
  result.data.assign(
    static_cast<std::size_t>(result.metadata.size_x) * result.metadata.size_y,
    0U);
  return result;
}

nav_msgs::msg::OccupancyGrid occupancyGrid()
{
  nav_msgs::msg::OccupancyGrid result;
  result.header.frame_id = "odom_combined";
  result.header.stamp.sec = 42;
  result.info.map_load_time.sec = 1;
  result.info.resolution = 0.1F;
  result.info.width = 4U;
  result.info.height = 3U;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(
    static_cast<std::size_t>(result.info.width) * result.info.height, 0);
  return result;
}

void setCost(nav2_msgs::msg::Costmap & map, std::size_t x, std::size_t y, std::uint8_t cost)
{
  map.data[y * static_cast<std::size_t>(map.metadata.size_x) + x] = cost;
}

void setCostAtWorld(nav2_msgs::msg::Costmap & map, double x, double y, std::uint8_t cost)
{
  const auto map_x = static_cast<std::size_t>(std::floor(x / map.metadata.resolution));
  const auto map_y = static_cast<std::size_t>(std::floor(y / map.metadata.resolution));
  setCost(map, map_x, map_y, cost);
}

const smartcar_nav2::CostmapFootprintSweepOptions kNominalOptions{
  0.20, 0.10, 0.025, 254U};

}  // namespace

TEST(LocalCostmapTrackingEnvelope, RejectsAPathOnlyTheTrackingTubeWouldClip)
{
  auto map = costmap();
  // This obstacle is outside the nominal 0.10 m half width, but the
  // controller can legally be 0.12 m north of the centreline.
  setCost(map, 15U, 12U, 254U);
  const auto candidate = path({pose(0.5, 1.0, 0.0), pose(2.0, 1.0, 0.0)});

  const auto nominal = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    candidate, map, kNominalOptions, 1.0);
  EXPECT_EQ(nominal.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kClear);
  EXPECT_TRUE(nominal.horizon_covered);

  auto envelope_options = kNominalOptions;
  envelope_options.half_width_m += 0.12;
  const auto envelope = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    candidate, map, envelope_options, 1.0);
  EXPECT_EQ(envelope.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
  EXPECT_TRUE(envelope.horizon_covered);
  EXPECT_TRUE(envelope.diagnostic.has_blocking_cell);
}

TEST(LocalCostmapTrackingEnvelope, StopsAtTheRequestedControllerHorizon)
{
  auto map = costmap();
  setCost(map, 24U, 10U, 254U);
  const auto candidate = path({pose(0.5, 1.0, 0.0), pose(3.0, 1.0, 0.0)});

  const auto envelope = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    candidate, map, kNominalOptions, 0.50);
  EXPECT_EQ(envelope.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kClear);
  EXPECT_TRUE(envelope.horizon_covered);
  EXPECT_NEAR(envelope.covered_horizon_m, 0.50, 1.0e-9);
}

TEST(LocalCostmapTrackingEnvelope, ReportsWhenThePathEndsBeforeTheRequiredHorizon)
{
  const auto envelope = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    path({pose(0.5, 1.0, 0.0), pose(0.7, 1.0, 0.0)}),
    costmap(), kNominalOptions, 0.50);
  EXPECT_EQ(envelope.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kClear);
  EXPECT_FALSE(envelope.horizon_covered);
  EXPECT_NEAR(envelope.covered_horizon_m, 0.20, 1.0e-9);
}

TEST(LocalCostmapTrackingEnvelope, ConvertsFilteredOccupancyGridWithFailClosedUnknowns)
{
  auto grid = occupancyGrid();
  grid.data[0] = 100;
  grid.data[1] = 99;
  grid.data[2] = -1;
  const auto converted = smartcar_nav2::localCostmapTrackingOccupancyGridToCostmap(grid);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(converted->header.frame_id, "odom_combined");
  EXPECT_EQ(converted->metadata.update_time.sec, 42);
  EXPECT_EQ(converted->metadata.size_x, 4U);
  EXPECT_EQ(converted->metadata.size_y, 3U);
  EXPECT_EQ(converted->data[0], 254U);
  EXPECT_LT(converted->data[1], 254U);
  EXPECT_EQ(converted->data[2], 255U);
}

TEST(LocalCostmapTrackingEnvelope, RejectsMalformedFilteredOccupancyGrid)
{
  auto grid = occupancyGrid();
  grid.data.pop_back();
  EXPECT_FALSE(
    smartcar_nav2::localCostmapTrackingOccupancyGridToCostmap(grid).has_value());

  grid = occupancyGrid();
  grid.data[0] = std::numeric_limits<std::int8_t>::max();
  EXPECT_FALSE(
    smartcar_nav2::localCostmapTrackingOccupancyGridToCostmap(grid).has_value());
}

TEST(LocalCostmapTrackingEnvelope, PDepartureProfileLeavesOnlyTheSouthErrorBudgetNarrow)
{
  auto south_obstacle = fineCostmap();
  // At station 0.225 m, a symmetric 0.12 m tube reaches this cell while the
  // P profile holds its south/right allowance to 7.5 mm. The full left
  // allowance must remain intact, which is checked below with the reflected
  // obstacle.
  setCostAtWorld(south_obstacle, 0.4875, 0.8625, 254U);
  const auto candidate = path({pose(0.25, 1.0, 0.0), pose(0.75, 1.0, 0.0)});
  // A short longitudinal body isolates the station-specific side allowance;
  // the production sweep retains its full 0.30 m half-length.
  const smartcar_nav2::CostmapFootprintSweepOptions short_body{
    0.01, 0.10, 0.025, 254U};
  const auto symmetric = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    candidate, south_obstacle, short_body, 0.50,
    smartcar_nav2::kForwardPathLateralProfileSymmetric, 0.12);
  EXPECT_EQ(
    symmetric.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);

  const auto p_departure = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    candidate, south_obstacle, short_body, 0.50,
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12);
  EXPECT_EQ(
    p_departure.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kClear);
  EXPECT_TRUE(p_departure.horizon_covered);

  auto north_obstacle = fineCostmap();
  setCostAtWorld(north_obstacle, 0.4875, 1.1375, 254U);
  const auto left_side = smartcar_nav2::localCostmapTrackingEnvelopeSweep(
    candidate, north_obstacle, short_body, 0.50,
    smartcar_nav2::kForwardPathLateralProfilePDepartureSouthV1, 0.12);
  EXPECT_EQ(
    left_side.sweep_result, smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
}
