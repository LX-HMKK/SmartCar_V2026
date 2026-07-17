#include <gtest/gtest.h>

#include <stdexcept>

#include "origincar_base/command_mode.hpp"


TEST(CommandModeTest, ActiveTwistDispatchesSideEffectsAndWriterOnce)
{
  const auto mode = command_mode_from_string("twist");
  int side_effect_count = 0;
  int writer_count = 0;

  EXPECT_TRUE(
    dispatch_command(
      mode, CommandType::kTwist,
      [&side_effect_count]() {++side_effect_count;},
      [&writer_count]() {++writer_count;}));
  EXPECT_EQ(side_effect_count, 1);
  EXPECT_EQ(writer_count, 1);
}

TEST(CommandModeTest, ActiveAckermannDispatchesSideEffectsAndWriterOnce)
{
  const auto mode = command_mode_from_string("ackermann");
  int side_effect_count = 0;
  int writer_count = 0;

  EXPECT_TRUE(
    dispatch_command(
      mode, CommandType::kAckermann,
      [&side_effect_count]() {++side_effect_count;},
      [&writer_count]() {++writer_count;}));
  EXPECT_EQ(side_effect_count, 1);
  EXPECT_EQ(writer_count, 1);
}

TEST(CommandModeTest, InactiveCommandDispatchesNoSideEffectsOrWriter)
{
  int side_effect_count = 0;
  int writer_count = 0;
  const auto side_effect = [&side_effect_count]() {++side_effect_count;};
  const auto writer = [&writer_count]() {++writer_count;};

  EXPECT_FALSE(
    dispatch_command(
      CommandMode::kTwist, CommandType::kAckermann, side_effect, writer));
  EXPECT_FALSE(
    dispatch_command(
      CommandMode::kAckermann, CommandType::kTwist, side_effect, writer));
  EXPECT_EQ(side_effect_count, 0);
  EXPECT_EQ(writer_count, 0);
}

TEST(CommandModeTest, RejectsLegacyInactiveTopicSentinelAsMode)
{
  EXPECT_THROW(command_mode_from_string("none"), std::invalid_argument);
}
