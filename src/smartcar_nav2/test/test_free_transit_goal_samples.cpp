#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "smartcar_nav2/free_transit_goal_samples.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool containsHeading(const std::vector<double> & headings, double expected)
{
  for (const double heading : headings) {
    if (std::abs(std::remainder(heading - expected, 2.0 * kPi)) < 1.0e-9) {
      return true;
    }
  }
  return false;
}

TEST(FreeTransitGoalSamples, IncludesWorldAlignedCorridorTangents)
{
  const auto headings = smartcar_nav2::freeTransitHeadingHints(
    -kPi / 4.0, {0.0, -kPi / 2.0}, 12);

  EXPECT_TRUE(containsHeading(headings, -kPi / 4.0));
  EXPECT_TRUE(containsHeading(headings, 0.0));
  EXPECT_TRUE(containsHeading(headings, -kPi / 2.0));
  EXPECT_TRUE(containsHeading(headings, kPi / 2.0));
}

TEST(FreeTransitGoalSamples, RetainsReferenceRelativeLattice)
{
  const auto headings = smartcar_nav2::freeTransitHeadingHints(
    -kPi / 4.0, {}, 12);

  EXPECT_TRUE(containsHeading(headings, -kPi / 4.0));
  EXPECT_TRUE(containsHeading(headings, -kPi / 12.0));
  EXPECT_TRUE(containsHeading(headings, 0.0));
}

TEST(FreeTransitGoalSamples, DeduplicatesEquivalentHeadings)
{
  const auto headings = smartcar_nav2::freeTransitHeadingHints(
    0.0, {2.0 * kPi, -2.0 * kPi}, 12);

  ASSERT_EQ(headings.size(), 12U);
  EXPECT_TRUE(containsHeading(headings, 0.0));
}

TEST(FreeTransitGoalSamples, CapsTheDeterministicHeadingSetAtItsBudget)
{
  const auto first = smartcar_nav2::freeTransitHeadingHints(
    -kPi / 4.0, {0.0, -kPi / 2.0}, 4);
  const auto second = smartcar_nav2::freeTransitHeadingHints(
    -kPi / 4.0, {0.0, -kPi / 2.0}, 4);

  ASSERT_EQ(first.size(), 4U);
  ASSERT_EQ(first, second);
  ASSERT_FALSE(first.empty());
  EXPECT_TRUE(containsHeading(first, -kPi / 4.0));
}

TEST(FreeTransitGoalSamples, LockedGoalAlternativesStayInsideAuthoredTolerance)
{
  const double authored = 0.30;
  const double tolerance = 0.15;
  const auto headings = smartcar_nav2::lockedGoalHeadingHints(authored, tolerance);

  ASSERT_EQ(headings.size(), 3U);
  EXPECT_TRUE(containsHeading(headings, authored));
  for (const double heading : headings) {
    EXPECT_LE(
      std::abs(std::remainder(heading - authored, 2.0 * kPi)),
      tolerance + 1.0e-12);
  }
  EXPECT_TRUE(containsHeading(headings, authored + 0.10));
  EXPECT_TRUE(containsHeading(headings, authored - 0.10));
}

TEST(FreeTransitGoalSamples, LockedGoalAlternativesCollapseForZeroTolerance)
{
  const auto headings = smartcar_nav2::lockedGoalHeadingHints(-0.4, 0.0);
  ASSERT_EQ(headings.size(), 1U);
  EXPECT_TRUE(containsHeading(headings, -0.4));
}

TEST(FreeTransitGoalSamples, LockedGoalAlternativesRemainBoundedForLooseTolerance)
{
  const auto headings = smartcar_nav2::lockedGoalHeadingHints(0.0, 0.50);
  ASSERT_EQ(headings.size(), 3U);
  EXPECT_TRUE(containsHeading(headings, 0.10));
  EXPECT_TRUE(containsHeading(headings, -0.10));
}

}  // namespace
