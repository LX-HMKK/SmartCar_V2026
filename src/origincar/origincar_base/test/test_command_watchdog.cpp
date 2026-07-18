#include <gtest/gtest.h>

#include <limits>

#include "origincar_base/command_watchdog.hpp"


TEST(CommandWatchdogTest, DoesNotStopBeforeAnyCommand)
{
  CommandWatchdog watchdog(0.35);

  EXPECT_FALSE(watchdog.consume_stop(10.0));
}

TEST(CommandWatchdogTest, DoesNotStopBeforeTimeout)
{
  CommandWatchdog watchdog(0.35);
  watchdog.mark_command(1.0);

  EXPECT_FALSE(watchdog.consume_stop(1.34));
}

TEST(CommandWatchdogTest, StopsOnceAtTimeout)
{
  CommandWatchdog watchdog(0.35);
  watchdog.mark_command(1.0);

  EXPECT_TRUE(watchdog.consume_stop(1.35));
}

TEST(CommandWatchdogTest, DoesNotRepeatStopWithoutNewCommand)
{
  CommandWatchdog watchdog(0.35);
  watchdog.mark_command(1.0);

  ASSERT_TRUE(watchdog.consume_stop(1.35));
  EXPECT_FALSE(watchdog.consume_stop(2.0));
}

TEST(CommandWatchdogTest, RearmsAfterNewCommand)
{
  CommandWatchdog watchdog(0.35);
  watchdog.mark_command(1.0);
  ASSERT_TRUE(watchdog.consume_stop(1.35));

  watchdog.mark_command(2.0);
  EXPECT_FALSE(watchdog.consume_stop(2.34));
  EXPECT_TRUE(watchdog.consume_stop(2.35));
}

TEST(CommandWatchdogTest, RejectsInvalidTimeout)
{
  EXPECT_THROW(CommandWatchdog(0.0), std::invalid_argument);
  EXPECT_THROW(CommandWatchdog(-1.0), std::invalid_argument);
  EXPECT_THROW(
    CommandWatchdog(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    CommandWatchdog(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}
