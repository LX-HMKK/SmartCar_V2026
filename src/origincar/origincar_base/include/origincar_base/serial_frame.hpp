#ifndef ORIGINCAR_BASE__SERIAL_FRAME_HPP_
#define ORIGINCAR_BASE__SERIAL_FRAME_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>

template<std::size_t N>
class XorFrameStreamParser
{
public:
  static_assert(N >= 3, "frame must contain data, checksum, and tail bytes");

  void append(const std::uint8_t * data, std::size_t size)
  {
    for (std::size_t index = 0; index < size; ++index) {
      buffer_.push_back(data[index]);
    }
  }

  bool pop_frame(
    std::uint8_t frame_header,
    std::uint8_t frame_tail,
    std::array<std::uint8_t, N> & frame)
  {
    while (true) {
      const auto header = std::find(buffer_.begin(), buffer_.end(), frame_header);
      if (header == buffer_.end()) {
        buffer_.clear();
        return false;
      }
      buffer_.erase(buffer_.begin(), header);
      if (buffer_.size() < N) {
        return false;
      }

      if (buffer_[N - 1] == frame_tail && checksum_matches()) {
        std::copy_n(buffer_.begin(), N, frame.begin());
        buffer_.erase(buffer_.begin(), buffer_.begin() + N);
        return true;
      }

      buffer_.pop_front();
    }
  }

  std::size_t buffered_size() const
  {
    return buffer_.size();
  }

private:
  bool checksum_matches() const
  {
    std::uint8_t checksum = 0;
    for (std::size_t index = 0; index < N - 2; ++index) {
      checksum ^= buffer_[index];
    }
    return buffer_[N - 2] == checksum;
  }

  std::deque<std::uint8_t> buffer_;
};

#endif  // ORIGINCAR_BASE__SERIAL_FRAME_HPP_
