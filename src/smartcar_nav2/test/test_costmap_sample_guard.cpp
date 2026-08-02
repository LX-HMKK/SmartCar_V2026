#include <chrono>
#include <cstdint>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "smartcar_nav2/costmap_sample_guard.hpp"

namespace
{

nav2_msgs::msg::Costmap::SharedPtr validCostmap(std::int32_t seconds, std::uint32_t nanoseconds)
{
  auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
  costmap->header.frame_id = "odom_combined";
  costmap->header.stamp.sec = seconds;
  costmap->header.stamp.nanosec = nanoseconds;
  costmap->metadata.resolution = 0.1F;
  costmap->metadata.size_x = 2U;
  costmap->metadata.size_y = 2U;
  costmap->data.assign(4U, 0U);
  return costmap;
}

nav_msgs::msg::OccupancyGrid validKeepoutMask()
{
  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = "odom_combined";
  mask.info.resolution = 1.0F;
  mask.info.width = 3U;
  mask.info.height = 3U;
  mask.info.origin.orientation.w = 1.0;
  mask.data.assign(9U, 0);
  return mask;
}

smartcar_nav2::CostmapSample sample(
  std::int32_t seconds, std::uint32_t nanoseconds, std::uint64_t sequence = 1U)
{
  const auto costmap = validCostmap(seconds, nanoseconds);
  return {
    costmap,
    std::chrono::steady_clock::now(),
    sequence,
    *smartcar_nav2::costmapSourceStampNanoseconds(*costmap)};
}

TEST(CostmapSampleGuard, RequiresValidAndCurrentSourceTimestamp)
{
  auto current = sample(10, 100000000U);
  const auto steady_now = std::chrono::steady_clock::now();
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      current, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10100000000LL), steady_now),
    smartcar_nav2::CostmapSampleFreshness::kFresh);

  auto stale_source = sample(9, 0U);
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      stale_source, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10100000000LL), steady_now),
    smartcar_nav2::CostmapSampleFreshness::kSourceStampStale);

  auto invalid_stamp = sample(1, 0U);
  invalid_stamp.costmap->header.stamp.sec = 0;
  invalid_stamp.stamp_ns = 0;
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      invalid_stamp, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10100000000LL), steady_now),
    smartcar_nav2::CostmapSampleFreshness::kInvalidStamp);
}

TEST(CostmapSampleGuard, RequiresNewSampleAfterClearBarrier)
{
  auto current = sample(10, 200000000U, 7U);
  const auto steady_now = std::chrono::steady_clock::now();
  const auto stamp_ns = current.stamp_ns;

  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      current, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10200000000LL), steady_now, stamp_ns),
    smartcar_nav2::CostmapSampleFreshness::kBeforeStampBarrier);
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      current, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10200000000LL), steady_now, 0, 7U),
    smartcar_nav2::CostmapSampleFreshness::kBeforeSequenceBarrier);

  auto newer = sample(10, 300000000U, 8U);
  const auto newer_steady_now = std::chrono::steady_clock::now();
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      newer, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10300000000LL), newer_steady_now, stamp_ns, 7U),
    smartcar_nav2::CostmapSampleFreshness::kFresh);
}

TEST(CostmapSampleGuard, UsesMetadataUpdateTimeWhenHeaderIsUnset)
{
  auto costmap = validCostmap(0, 0U);
  costmap->metadata.update_time.sec = 10;
  costmap->metadata.update_time.nanosec = 300000000U;
  const auto source_stamp = smartcar_nav2::costmapSourceStampNanoseconds(*costmap);
  ASSERT_TRUE(source_stamp.has_value());
  EXPECT_EQ(*source_stamp, 10300000000LL);

  smartcar_nav2::CostmapSample metadata_sample{
    costmap,
    std::chrono::steady_clock::now(),
    1U,
    *source_stamp};
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      metadata_sample, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), std::chrono::steady_clock::now()),
    smartcar_nav2::CostmapSampleFreshness::kFresh);
}

TEST(CostmapSampleGuard, RejectsDuplicateOrReplayedSourceStamps)
{
  auto current = validCostmap(10, 100000000U);
  current->metadata.update_time.sec = 10;
  current->metadata.update_time.nanosec = 300000000U;
  EXPECT_FALSE(
    smartcar_nav2::costmapHasStrictlyNewerSourceStamp(*current, 10300000000LL));
  EXPECT_TRUE(
    smartcar_nav2::costmapHasStrictlyNewerSourceStamp(*current, 10200000000LL));
}

TEST(CostmapSampleGuard, RequiresPostClearScanAndCostmapAssociation)
{
  auto current = sample(10, 300000000U, 8U);
  current.scan_sequence = 3U;
  current.scan_stamp_ns = 10200000000LL;
  const auto steady_now = std::chrono::steady_clock::now();

  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      current, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), steady_now,
      10100000000LL, 7U, 10100000000LL, 2U, true),
    smartcar_nav2::CostmapSampleFreshness::kFresh);

  auto preclear_map = current;
  preclear_map.sequence = 7U;
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      preclear_map, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), steady_now,
      10100000000LL, 7U, 10100000000LL, 2U, true),
    smartcar_nav2::CostmapSampleFreshness::kBeforeSequenceBarrier);

  auto queued_before_clear = sample(10, 240000000U, 8U);
  queued_before_clear.scan_sequence = 3U;
  queued_before_clear.scan_stamp_ns = 10200000000LL;
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      queued_before_clear, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), steady_now,
      10250000000LL, 7U, 10100000000LL, 2U, true),
    smartcar_nav2::CostmapSampleFreshness::kBeforeStampBarrier);

  auto preclear_scan = current;
  preclear_scan.scan_sequence = 2U;
  preclear_scan.scan_stamp_ns = 10100000000LL;
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      preclear_scan, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), steady_now,
      10100000000LL, 7U, 10100000000LL, 2U, true),
    smartcar_nav2::CostmapSampleFreshness::kBeforeScanStampBarrier);

  auto missing_association = current;
  missing_association.scan_sequence = 0U;
  missing_association.scan_stamp_ns = 0;
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      missing_association, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), steady_now,
      10100000000LL, 7U, 10100000000LL, 2U, true),
    smartcar_nav2::CostmapSampleFreshness::kMissingScanAssociation);

  auto map_before_scan = current;
  map_before_scan.scan_stamp_ns = 10400000000LL;
  EXPECT_EQ(
    smartcar_nav2::costmapSampleFreshness(
      map_before_scan, "odom_combined", std::chrono::milliseconds(500),
      rclcpp::Time(10400000000LL), steady_now,
      10100000000LL, 7U, 10100000000LL, 2U, true),
    smartcar_nav2::CostmapSampleFreshness::kCostmapBeforeAssociatedScan);
}

TEST(CostmapSampleGuard, RequiresLethalWitnessAtScanEndpoint)
{
  auto costmap = validCostmap(10, 0U);
  costmap->data[3U] = 253U;
  const std::vector<std::pair<double, double>> endpoint{{0.15, 0.15}};

  EXPECT_TRUE(smartcar_nav2::costmapHasLethalObservationAtPoints(
    *costmap, endpoint, 253U, 0.0));

  costmap->data.assign(4U, 0U);
  EXPECT_FALSE(smartcar_nav2::costmapHasLethalObservationAtPoints(
    *costmap, endpoint, 253U, 0.12));

  costmap->data[3U] = 253U;
  const std::vector<std::pair<double, double>> elsewhere{{0.01, 0.01}};
  EXPECT_FALSE(smartcar_nav2::costmapHasLethalObservationAtPoints(
    *costmap, elsewhere, 253U, 0.0));
}

TEST(CostmapSampleGuard, BoundsAssociatedScanToItsCostmapUpdate)
{
  auto current = sample(10, 300000000U, 8U);
  current.scan_sequence = 3U;
  current.scan_stamp_ns = 10000000000LL;

  EXPECT_EQ(
    smartcar_nav2::costmapScanAssociationFreshness(
      current, std::chrono::milliseconds(500)),
    smartcar_nav2::CostmapScanAssociationFreshness::kFresh);

  auto delayed = current;
  delayed.scan_stamp_ns = 9000000000LL;
  EXPECT_EQ(
    smartcar_nav2::costmapScanAssociationFreshness(
      delayed, std::chrono::milliseconds(500)),
    smartcar_nav2::CostmapScanAssociationFreshness::kScanTooOld);

  auto map_before_scan = current;
  map_before_scan.scan_stamp_ns = 10400000000LL;
  EXPECT_EQ(
    smartcar_nav2::costmapScanAssociationFreshness(
      map_before_scan, std::chrono::milliseconds(500)),
    smartcar_nav2::CostmapScanAssociationFreshness::kCostmapBeforeScan);
}

TEST(CostmapSampleGuard, RemovesStaticKeepoutCellsFromScanWitnesses)
{
  auto mask = validKeepoutMask();
  mask.data[4U] = 100;
  mask.data[5U] = -1;
  const std::vector<std::pair<double, double>> endpoints{
    {0.5, 0.5}, {1.5, 1.5}, {2.5, 1.5}, {3.5, 0.5}};
  std::vector<std::pair<double, double>> filtered;

  EXPECT_EQ(
    smartcar_nav2::filterPointsOutsideStaticKeepoutMask(
      endpoints, &mask, "odom_combined", filtered),
    smartcar_nav2::StaticKeepoutMaskFilterResult::kFiltered);
  ASSERT_EQ(filtered.size(), 1U);
  EXPECT_EQ(filtered.front(), std::make_pair(0.5, 0.5));

  EXPECT_EQ(
    smartcar_nav2::filterPointsOutsideStaticKeepoutMask(
      endpoints, nullptr, "odom_combined", filtered),
    smartcar_nav2::StaticKeepoutMaskFilterResult::kNoMask);
  EXPECT_EQ(filtered, endpoints);

  EXPECT_EQ(
    smartcar_nav2::filterPointsOutsideStaticKeepoutMask(
      endpoints, &mask, "wrong_frame", filtered),
    smartcar_nav2::StaticKeepoutMaskFilterResult::kWrongFrame);
  EXPECT_TRUE(filtered.empty());
}

}  // namespace
