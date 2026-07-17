#include <gtest/gtest.h>

#include <stdexcept>

#include "origincar_base/command_mode.hpp"


TEST(CommandModeTest, TwistModeAcceptsOnlyTwistCommands)
{
  const auto mode = command_mode_from_string("twist");

  EXPECT_TRUE(accepts_command(mode, CommandType::kTwist));
  EXPECT_FALSE(accepts_command(mode, CommandType::kAckermann));
}

TEST(CommandModeTest, AckermannModeAcceptsOnlyAckermannCommands)
{
  const auto mode = command_mode_from_string("ackermann");

  EXPECT_TRUE(accepts_command(mode, CommandType::kAckermann));
  EXPECT_FALSE(accepts_command(mode, CommandType::kTwist));
}

TEST(CommandModeTest, RejectsLegacyInactiveTopicSentinelAsMode)
{
  EXPECT_THROW(command_mode_from_string("none"), std::invalid_argument);
}
