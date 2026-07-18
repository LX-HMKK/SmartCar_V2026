#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "origincar_base/serial_frame.hpp"

namespace
{

constexpr std::size_t kFrameSize = 24;
constexpr std::uint8_t kFrameHeader = 0x7b;
constexpr std::uint8_t kFrameTail = 0x7d;

std::array<std::uint8_t, kFrameSize> make_frame(std::uint8_t seed = 0)
{
  std::array<std::uint8_t, kFrameSize> frame{};
  frame[0] = kFrameHeader;
  for (std::size_t index = 1; index < kFrameSize - 2; ++index) {
    frame[index] = static_cast<std::uint8_t>(seed + index);
  }
  frame[kFrameSize - 1] = kFrameTail;

  std::uint8_t checksum = 0;
  for (std::size_t index = 0; index < kFrameSize - 2; ++index) {
    checksum ^= frame[index];
  }
  frame[kFrameSize - 2] = checksum;
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

}  // namespace
