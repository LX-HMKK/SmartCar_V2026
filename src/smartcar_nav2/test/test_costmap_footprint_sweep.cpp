#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"
#include "smartcar_nav2/footprint_sweep_collision_source.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(orientation);
  return result;
}

nav2_msgs::msg::Costmap costmap()
{
  nav2_msgs::msg::Costmap result;
  result.metadata.resolution = 0.1F;
  result.metadata.size_x = 40U;
  result.metadata.size_y = 40U;
  result.metadata.origin.orientation.w = 1.0;
  result.data.assign(
    static_cast<std::size_t>(result.metadata.size_x) * result.metadata.size_y,
    0U);
  return result;
}

nav_msgs::msg::OccupancyGrid keepoutMask()
{
  nav_msgs::msg::OccupancyGrid result;
  result.header.frame_id = "odom_combined";
  result.info.resolution = 0.1F;
  result.info.width = 3U;
  result.info.height = 3U;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(9U, 0);
  return result;
}

nav_msgs::msg::OccupancyGrid wideKeepoutMask()
{
  nav_msgs::msg::OccupancyGrid result;
  result.header.frame_id = "odom_combined";
  result.info.resolution = 0.1F;
  result.info.width = 40U;
  result.info.height = 40U;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(1600U, 0);
  return result;
}

nav_msgs::msg::OccupancyGrid pFinishSouthBoundaryMask()
{
  nav_msgs::msg::OccupancyGrid result;
  result.header.frame_id = "odom_combined";
  result.info.resolution = 0.025F;
  result.info.width = 100U;
  result.info.height = 100U;
  result.info.origin.position.x = -0.75;
  result.info.origin.position.y = -0.50;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(10000U, 0);
  // The top row of the P-side exterior ring ends at y=-0.25. Its cell centre
  // is (-0.1125, -0.2625), where the p_finish 45-degree terminal pose has a
  // real 6.7 mm clearance.
  result.data[9U * 100U + 25U] = 100;
  return result;
}

nav2_msgs::msg::Costmap pFinishSouthBoundaryCostmap()
{
  nav2_msgs::msg::Costmap result;
  result.metadata.resolution = 0.025F;
  result.metadata.size_x = 100U;
  result.metadata.size_y = 100U;
  result.metadata.origin.position.x = -0.75;
  result.metadata.origin.position.y = -0.50;
  result.metadata.origin.orientation.w = 1.0;
  result.data.assign(10000U, 0U);
  result.data[9U * 100U + 25U] = 254U;
  return result;
}

void setCost(nav2_msgs::msg::Costmap & map, std::size_t x, std::size_t y, std::uint8_t cost)
{
  map.data[y * static_cast<std::size_t>(map.metadata.size_x) + x] = cost;
}

nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path result;
  result.poses.assign(poses);
  return result;
}

const smartcar_nav2::CostmapFootprintSweepOptions kOptions{
  0.2491, 0.095, 0.025, 254U};

TEST(CostmapFootprintSweep, RejectsLethalCellUnderVehicleFootprint)
{
  auto map = costmap();
  setCost(map, 20U, 20U, 254U);

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(path({pose(2.0, 2.0, 0.0)}), map, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
}

TEST(CostmapFootprintSweep, RejectsLethalCellCrossedBetweenPathSamples)
{
  auto map = costmap();
  setCost(map, 20U, 20U, 254U);

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(1.0, 2.0, 0.0), pose(3.0, 2.0, 0.0)}), map, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
}

TEST(CostmapFootprintSweep, ReportsFirstLethalInterpolationPoseAndCell)
{
  auto map = costmap();
  setCost(map, 20U, 20U, 254U);
  smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(1.0, 2.0, 0.0), pose(3.0, 2.0, 0.0)}), map, kOptions, &diagnostic),
    smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);

  EXPECT_EQ(
    diagnostic.result, smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
  ASSERT_TRUE(diagnostic.has_sample_pose);
  EXPECT_EQ(diagnostic.segment_start_pose_index, 0U);
  EXPECT_EQ(diagnostic.segment_end_pose_index, 1U);
  EXPECT_EQ(diagnostic.segment_sample_index, 31U);
  EXPECT_EQ(diagnostic.segment_sample_count, 80U);
  EXPECT_NEAR(diagnostic.segment_fraction, 0.3875, 1.0e-12);
  EXPECT_NEAR(diagnostic.sample_pose.pose.position.x, 1.775, 1.0e-12);
  EXPECT_NEAR(diagnostic.sample_pose.pose.position.y, 2.00, 1.0e-12);
  ASSERT_TRUE(diagnostic.has_blocking_cell);
  EXPECT_EQ(diagnostic.blocking_cell_x, 20U);
  EXPECT_EQ(diagnostic.blocking_cell_y, 20U);
  EXPECT_EQ(diagnostic.blocking_cell_cost, 254U);
  EXPECT_NEAR(diagnostic.blocking_cell_world_x, 2.05, 1.0e-6);
  EXPECT_NEAR(diagnostic.blocking_cell_world_y, 2.05, 1.0e-6);
  EXPECT_STREQ(
    smartcar_nav2::costmapFootprintSweepCellCostName(diagnostic.blocking_cell_cost),
    "lethal_obstacle");
}

TEST(CostmapFootprintSweep, AttributesBlockingCellAgainstStaticKeepoutMask)
{
  auto mask = keepoutMask();
  mask.data[4U] = 100;
  EXPECT_EQ(
    smartcar_nav2::keepoutMaskCellStateAt(&mask, "odom_combined", 0.15, 0.15),
    smartcar_nav2::KeepoutMaskCellState::kOccupied);
  EXPECT_STREQ(
    smartcar_nav2::keepoutCollisionSourceName(
      smartcar_nav2::KeepoutMaskCellState::kOccupied),
    "static_keepout_filter_mask");

  EXPECT_EQ(
    smartcar_nav2::keepoutMaskCellStateAt(&mask, "odom_combined", 0.05, 0.05),
    smartcar_nav2::KeepoutMaskCellState::kFree);
  mask.data[3U] = -1;
  EXPECT_EQ(
    smartcar_nav2::keepoutMaskCellStateAt(&mask, "odom_combined", 0.05, 0.15),
    smartcar_nav2::KeepoutMaskCellState::kUnknown);
  EXPECT_EQ(
    smartcar_nav2::keepoutMaskCellStateAt(&mask, "odom_combined", 0.35, 0.15),
    smartcar_nav2::KeepoutMaskCellState::kOutOfBounds);
  EXPECT_EQ(
    smartcar_nav2::keepoutMaskCellStateAt(&mask, "wrong_frame", 0.15, 0.15),
    smartcar_nav2::KeepoutMaskCellState::kWrongFrame);
}

TEST(CostmapFootprintSweep, SweepsStaticKeepoutMaskAsAFullBodyConstraint)
{
  auto mask = wideKeepoutMask();
  mask.data[20U * 40U + 20U] = 100;
  smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;

  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &mask, "odom_combined", path({pose(2.0, 2.0, 0.0)}), kOptions, &diagnostic),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kOccupiedOrUnknown);
  ASSERT_TRUE(diagnostic.has_blocking_cell);
  EXPECT_EQ(diagnostic.blocking_cell_cost, 254U);

  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &mask, "odom_combined", path({pose(0.1, 2.0, 0.0)}), kOptions),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kOutOfBounds);

  mask.data[20U * 40U + 20U] = -1;
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &mask, "odom_combined", path({pose(2.0, 2.0, 0.0)}), kOptions),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kOccupiedOrUnknown);
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      nullptr, "odom_combined", path({pose(2.0, 2.0, 0.0)}), kOptions),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kNoMask);
}

TEST(CostmapFootprintSweep, PreservesExactPFinishClearanceAtSouthBoundary)
{
  constexpr double kPFinishYaw = 0.78539816339744830962;
  auto mask = pFinishSouthBoundaryMask();
  auto raw_costmap = pFinishSouthBoundaryCostmap();

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(0.0, 0.0, kPFinishYaw)}), raw_costmap, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kClear);

  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &mask, "odom_combined", path({pose(0.0, 0.0, kPFinishYaw)}), kOptions),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kClear);

  // Crossing the true exterior edge by less than one centimetre remains
  // blocked in both raw and static-mask sweeps.
  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(0.0, -0.007, kPFinishYaw)}), raw_costmap, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
  EXPECT_EQ(
    smartcar_nav2::staticKeepoutMaskFootprintPathSweep(
      &mask, "odom_combined", path({pose(0.0, -0.007, kPFinishYaw)}), kOptions),
    smartcar_nav2::StaticKeepoutMaskSweepResult::kOccupiedOrUnknown);
}

TEST(CostmapFootprintSweep, TreatsOnlyPhysicalOrUnknownCellsAsBlocked)
{
  auto inscribed = costmap();
  setCost(inscribed, 20U, 20U, 253U);
  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(2.0, 2.0, 0.0)}), inscribed, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kClear);

  auto lethal = costmap();
  setCost(lethal, 20U, 20U, 254U);
  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(2.0, 2.0, 0.0)}), lethal, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);

  auto unknown = costmap();
  setCost(unknown, 20U, 20U, 255U);
  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(2.0, 2.0, 0.0)}), unknown, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kLethalOverlap);
}

TEST(CostmapFootprintSweep, AllowsAPathWhosePaddedFootprintIsClear)
{
  auto map = costmap();
  setCost(map, 30U, 30U, 254U);

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(1.0, 1.0, 0.0), pose(1.8, 1.0, 0.0)}), map, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kClear);
}

TEST(CostmapFootprintSweep, FailsClosedWhenFootprintLeavesRollingCostmap)
{
  auto map = costmap();
  smartcar_nav2::CostmapFootprintSweepDiagnostic diagnostic;

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(0.1, 0.1, 0.0)}), map, kOptions, &diagnostic),
    smartcar_nav2::CostmapFootprintSweepResult::kOutOfBounds);
  EXPECT_EQ(
    diagnostic.result, smartcar_nav2::CostmapFootprintSweepResult::kOutOfBounds);
  EXPECT_TRUE(diagnostic.has_sample_pose);
  EXPECT_TRUE(diagnostic.has_boundary_point);
  EXPECT_LT(diagnostic.boundary_world_x, 0.0);
  EXPECT_GE(diagnostic.boundary_world_y, 0.0);
}

TEST(CostmapFootprintSweep, SeparatesInvalidInputFromOutOfBounds)
{
  auto map = costmap();
  auto invalid_pose = pose(1.0, 1.0, 0.0);
  invalid_pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(path({invalid_pose}), map, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kInvalidInput);
}

}  // namespace
