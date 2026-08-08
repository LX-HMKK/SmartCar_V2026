#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "origincar_base/serial_frame.hpp"

namespace
{

constexpr std::size_t kFrameSize = 24;
constexpr std::size_t kCommandSize = 11;
constexpr std::uint8_t kFrameHeader = 0x7b;
constexpr std::uint8_t kFrameTail = 0x7d;

template<std::size_t N>
std::array<std::uint8_t, N> make_protocol_frame(std::uint8_t seed)
{
  std::array<std::uint8_t, N> frame{};
  frame[0] = kFrameHeader;
  for (std::size_t index = 1; index < N - 2; ++index) {
    frame[index] = static_cast<std::uint8_t>(seed + index);
  }
  frame[N - 1] = kFrameTail;

  std::uint8_t checksum = 0;
  for (std::size_t index = 0; index < N - 2; ++index) {
    checksum ^= frame[index];
  }
  frame[N - 2] = checksum;
  return frame;
}

std::array<std::uint8_t, kFrameSize> make_frame(std::uint8_t seed = 0)
{
  return make_protocol_frame<kFrameSize>(seed);
}

template<std::size_t N>
std::array<std::uint8_t, N> make_stop_frame()
{
  std::array<std::uint8_t, N> frame{};
  frame[0] = kFrameHeader;
  frame[N - 2] = kFrameHeader;
  frame[N - 1] = kFrameTail;
  return frame;
}

template<std::size_t N>
void append_array(
  XorFrameStreamParser<kFrameSize> & parser,
  const std::array<std::uint8_t, N> & bytes)
{
  parser.append(bytes.data(), bytes.size());
}

TEST(SerialFrameTest, AcceptsAlignedFrame)
{
  const auto frame = make_frame();
  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> normalized{};

  append_array(parser, frame);
  EXPECT_TRUE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  EXPECT_EQ(normalized, frame);
}

TEST(SerialFrameTest, ReassemblesPartialReads)
{
  const auto frame = make_frame();
  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> normalized{};

  parser.append(frame.data(), 5);
  EXPECT_FALSE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  parser.append(frame.data() + 5, 7);
  EXPECT_FALSE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  parser.append(frame.data() + 12, frame.size() - 12);
  EXPECT_TRUE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  EXPECT_EQ(normalized, frame);
}

TEST(SerialFrameTest, IgnoresHeaderAndTailMarkersInsidePayload)
{
  auto frame = make_frame();
  frame[4] = kFrameHeader;
  frame[11] = kFrameTail;
  frame[kFrameSize - 2] = 0;
  for (std::size_t index = 0; index < kFrameSize - 2; ++index) {
    frame[kFrameSize - 2] ^= frame[index];
  }
  const std::array<std::uint8_t, 3> noise = {0x01, 0x02, 0x03};
  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> normalized{};

  append_array(parser, noise);
  append_array(parser, frame);
  EXPECT_TRUE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  EXPECT_EQ(normalized, frame);
}

TEST(SerialFrameTest, ResynchronizesAcrossAdjacentFramesAtFixedReadOffset)
{
  const auto first = make_frame(0x10);
  const auto second = make_frame(0x40);
  const auto third = make_frame(0x70);
  constexpr std::size_t offset = 9;
  std::array<std::uint8_t, kFrameSize> first_read{};
  std::array<std::uint8_t, kFrameSize> second_read{};
  for (std::size_t index = 0; index < kFrameSize; ++index) {
    first_read[index] = index < kFrameSize - offset ?
      first[index + offset] : second[index - (kFrameSize - offset)];
    second_read[index] = index < kFrameSize - offset ?
      second[index + offset] : third[index - (kFrameSize - offset)];
  }

  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> normalized{};
  append_array(parser, first_read);
  EXPECT_FALSE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  append_array(parser, second_read);
  EXPECT_TRUE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  EXPECT_EQ(normalized, second);
}

TEST(SerialFrameTest, SkipsCorruptedFrameAndReturnsFollowingValidFrame)
{
  auto corrupted = make_frame(0x20);
  corrupted[kFrameSize - 2] ^= 0x01;
  const auto valid = make_frame(0x50);
  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> normalized{};

  append_array(parser, corrupted);
  append_array(parser, valid);
  EXPECT_TRUE(parser.pop_frame(kFrameHeader, kFrameTail, normalized));
  EXPECT_EQ(normalized, valid);
}

TEST(SerialFrameTest, DrainsAdjacentFramesAndReturnsOnlyNewest)
{
  const auto first = make_frame(0x10);
  const auto second = make_frame(0x30);
  const auto third = make_frame(0x50);
  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> latest{};
  XorFrameDrainStats stats;

  append_array(parser, first);
  append_array(parser, second);
  append_array(parser, third);

  EXPECT_TRUE(
    parser.pop_latest_frame(
      kFrameHeader, kFrameTail, latest, stats));
  EXPECT_EQ(latest, third);
  EXPECT_EQ(stats.valid_frames, 3U);
  EXPECT_EQ(stats.invalid_frames, 0U);
  EXPECT_EQ(stats.discarded_bytes, 0U);
  EXPECT_EQ(parser.buffered_size(), 0U);
}

TEST(SerialFrameTest, DrainReportsCorruptCandidatesAndDiscardedNoise)
{
  auto corrupted = make_frame(0x20);
  corrupted[kFrameSize - 2] ^= 0x01;
  const auto valid = make_frame(0x60);
  const std::array<std::uint8_t, 3> noise = {0x01, 0x02, 0x03};
  XorFrameStreamParser<kFrameSize> parser;
  std::array<std::uint8_t, kFrameSize> latest{};
  XorFrameDrainStats stats;

  append_array(parser, noise);
  append_array(parser, corrupted);
  append_array(parser, valid);

  EXPECT_TRUE(
    parser.pop_latest_frame(
      kFrameHeader, kFrameTail, latest, stats));
  EXPECT_EQ(latest, valid);
  EXPECT_EQ(stats.valid_frames, 1U);
  EXPECT_GE(stats.invalid_frames, 1U);
  EXPECT_GT(stats.discarded_bytes, noise.size());
}

TEST(SerialFrameTest, SelectorKeepsNewestFrameAcrossBoundedReadCycles)
{
  const auto second = make_frame(0x30);
  const auto third = make_frame(0x50);
  LatestFrameSelector<kFrameSize> selector;
  std::array<std::uint8_t, kFrameSize> latest{};

  selector.offer(second, 2U, 1.0);
  selector.offer(third, 1U, 1.01);

  EXPECT_TRUE(selector.take_latest(latest));
  EXPECT_EQ(latest, third);
  EXPECT_FALSE(selector.take_latest(latest));
  EXPECT_EQ(selector.stats().valid_frames, 3U);
  EXPECT_EQ(selector.stats().dropped_frames, 2U);
  EXPECT_EQ(selector.stats().expired_frames, 0U);
  EXPECT_EQ(selector.stats().backlog_stale_frames, 0U);
  EXPECT_EQ(selector.stats().coalescing_events, 2U);
}

TEST(SerialFrameTest, DropsDeferredFrameWhenBacklogClearsWithoutFreshFrame)
{
  const auto valid = make_frame(0x30);
  LatestFrameSelector<kFrameSize> selector;
  std::array<std::uint8_t, kFrameSize> latest{};

  selector.offer(valid, 1U, 1.0);
  EXPECT_EQ(
    choose_pending_frame_action(true, false, true),
    PendingFrameAction::kPublish);
  EXPECT_EQ(
    choose_pending_frame_action(true, false, false),
    PendingFrameAction::kDefer);
  EXPECT_EQ(
    choose_pending_frame_action(false, true, false),
    PendingFrameAction::kDiscardBacklogStale);
  EXPECT_TRUE(selector.discard_backlog_stale());
  EXPECT_FALSE(selector.take_latest(latest));
  EXPECT_EQ(selector.stats().valid_frames, 1U);
  EXPECT_EQ(selector.stats().dropped_frames, 1U);
  EXPECT_EQ(selector.stats().expired_frames, 0U);
  EXPECT_EQ(selector.stats().backlog_stale_frames, 1U);

  EXPECT_EQ(
    choose_pending_frame_action(false, true, true),
    PendingFrameAction::kPublish);
  EXPECT_EQ(
    choose_pending_frame_action(false, false, false),
    PendingFrameAction::kPublish);
}

TEST(SerialFrameTest, SelectorExpiresValidFrameAfterBadBacklog)
{
  const auto valid = make_frame(0x30);
  auto corrupted = make_frame(0x60);
  corrupted[kFrameSize - 2] ^= 0x01;
  XorFrameStreamParser<kFrameSize> parser;
  LatestFrameSelector<kFrameSize> selector;
  std::array<std::uint8_t, kFrameSize> latest{};
  XorFrameDrainStats stats;

  append_array(parser, valid);
  ASSERT_TRUE(
    parser.pop_latest_frame(
      kFrameHeader, kFrameTail, latest, stats));
  selector.offer(latest, stats.valid_frames, 1.0);

  for (std::size_t cycle = 0; cycle < 3U; ++cycle) {
    append_array(parser, corrupted);
    EXPECT_FALSE(
      parser.pop_latest_frame(
        kFrameHeader, kFrameTail, latest, stats));
    EXPECT_GT(stats.invalid_frames, 0U);
  }

  EXPECT_TRUE(selector.expire_if_older_than(1.101, 0.1));
  EXPECT_FALSE(selector.take_latest(latest));
  EXPECT_EQ(selector.stats().valid_frames, 1U);
  EXPECT_EQ(selector.stats().dropped_frames, 1U);
  EXPECT_EQ(selector.stats().expired_frames, 1U);
  EXPECT_EQ(selector.stats().backlog_stale_frames, 0U);
}

TEST(SerialFrameTest, RecoveryRetryRealignsEveryShortWritePrefix)
{
  auto command = make_protocol_frame<kCommandSize>(0x70);
  command[3] = kFrameHeader;
  command[6] = kFrameTail;
  command[kCommandSize - 2] = 0;
  for (std::size_t index = 0; index < kCommandSize - 2; ++index) {
    command[kCommandSize - 2] ^= command[index];
  }
  const auto stop = make_stop_frame<kCommandSize>();
  const auto recovery = make_fail_closed_recovery_stream(stop);

  ASSERT_EQ(command[3], kFrameHeader);
  ASSERT_EQ(command[6], kFrameTail);

  for (std::size_t command_prefix = 0;
    command_prefix < command.size(); ++command_prefix)
  {
    for (std::size_t recovery_prefix = 0;
      recovery_prefix < recovery.size(); ++recovery_prefix)
    {
      XorFrameStreamParser<kCommandSize> parser;
      std::array<std::uint8_t, kCommandSize> latest{};
      XorFrameDrainStats stats;

      parser.append(command.data(), command_prefix);
      parser.append(recovery.data(), recovery_prefix);
      parser.append(recovery.data(), recovery.size());

      ASSERT_TRUE(
        parser.pop_latest_frame(
          kFrameHeader, kFrameTail, latest, stats));
      EXPECT_EQ(latest, stop);
      EXPECT_GE(stats.valid_frames, 1U);
    }
  }
}

TEST(SerialFrameTest, ReadSizeIsBoundedByAvailabilityBudgetAndBuffer)
{
  EXPECT_EQ(bounded_serial_read_size(200U, 96U, 48U), 48U);
  EXPECT_EQ(bounded_serial_read_size(20U, 96U, 48U), 20U);
  EXPECT_EQ(bounded_serial_read_size(200U, 30U, 48U), 30U);
}

}  // namespace
