#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "smartcar_nav2/costmap_footprint_sweep.hpp"

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
  0.30, 0.16, 0.025, 253U};

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

TEST(CostmapFootprintSweep, TreatsInscribedAndUnknownCellsAsBlocked)
{
  auto inscribed = costmap();
  setCost(inscribed, 20U, 20U, 253U);
  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(
      path({pose(2.0, 2.0, 0.0)}), inscribed, kOptions),
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

  EXPECT_EQ(
    smartcar_nav2::costmapFootprintPathSweep(path({pose(0.1, 0.1, 0.0)}), map, kOptions),
    smartcar_nav2::CostmapFootprintSweepResult::kOutOfBounds);
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
