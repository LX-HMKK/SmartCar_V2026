#include <limits>

#include "gtest/gtest.h"

#include "smartcar_nav2/odom_distance_budget_tracker.hpp"

namespace smartcar_nav2
{
namespace
{

TEST(OdomDistanceBudgetTracker, UsesConfiguredRatioOrAbsoluteSlack)
{
  OdomDistanceBudgetTracker tracker;
  ASSERT_TRUE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{3.0, 4.0}, 2.0, 0.80, 0.50));
  EXPECT_DOUBLE_EQ(tracker.budget_m(), 10.0);

  ASSERT_TRUE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{0.1, 0.0}, 2.0, 0.80, 0.50));
  EXPECT_DOUBLE_EQ(tracker.budget_m(), 0.9);
}

TEST(OdomDistanceBudgetTracker, UsesTheTerminalThroughPoseForTheActionBudget)
{
  OdomDistanceBudgetTracker tracker;
  const std::vector<OdomPlanarPose> goals{
    OdomPlanarPose{100.0, 0.0},
    OdomPlanarPose{3.0, 4.0},
  };
  ASSERT_TRUE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, goals, 2.0, 0.80, 0.50));
  EXPECT_DOUBLE_EQ(tracker.budget_m(), 10.0);
}

TEST(OdomDistanceBudgetTracker, AccumulatesAPathLoopInsteadOfNetDisplacement)
{
  OdomDistanceBudgetTracker tracker;
  ASSERT_TRUE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{1.0, 0.0}, 2.0, 0.80, 2.0));
  EXPECT_EQ(
    tracker.update(OdomPlanarPose{0.0, 1.0}),
    OdomDistanceBudgetUpdate::kAccepted);
  EXPECT_EQ(
    tracker.update(OdomPlanarPose{1.0, 1.0}),
    OdomDistanceBudgetUpdate::kBudgetExceeded);
  EXPECT_DOUBLE_EQ(tracker.travelled_m(), 2.0);
  EXPECT_DOUBLE_EQ(tracker.budget_m(), 2.0);
}

TEST(OdomDistanceBudgetTracker, RejectsInvalidConfigurationAndSamples)
{
  OdomDistanceBudgetTracker tracker;
  EXPECT_FALSE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{1.0, 0.0}, 0.99, 0.80, 0.50));
  EXPECT_FALSE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{1.0, 0.0}, 2.0, -0.01, 0.50));
  EXPECT_FALSE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{1.0, 0.0}, 2.0, 0.80, 0.0));
  EXPECT_FALSE(tracker.initialize(
      OdomPlanarPose{std::numeric_limits<double>::quiet_NaN(), 0.0},
      OdomPlanarPose{1.0, 0.0}, 2.0, 0.80, 0.50));
  EXPECT_FALSE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, std::vector<OdomPlanarPose>{}, 2.0, 0.80, 0.50));
  EXPECT_FALSE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0},
      std::vector<OdomPlanarPose>{
        OdomPlanarPose{1.0, 0.0},
        OdomPlanarPose{std::numeric_limits<double>::quiet_NaN(), 0.0},
      },
      2.0, 0.80, 0.50));

  ASSERT_TRUE(tracker.initialize(
      OdomPlanarPose{0.0, 0.0}, OdomPlanarPose{1.0, 0.0}, 2.0, 0.80, 0.50));
  EXPECT_EQ(
    tracker.update(OdomPlanarPose{0.51, 0.0}),
    OdomDistanceBudgetUpdate::kStepTooLarge);
  EXPECT_EQ(
    tracker.update(OdomPlanarPose{
      std::numeric_limits<double>::quiet_NaN(), 0.0}),
    OdomDistanceBudgetUpdate::kInvalidSample);
}

}  // namespace
}  // namespace smartcar_nav2
