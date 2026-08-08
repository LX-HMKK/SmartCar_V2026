#ifndef ORIGINCAR_BASE__SERIAL_FRAME_HPP_
#define ORIGINCAR_BASE__SERIAL_FRAME_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>

struct XorFrameDrainStats
{
  XorFrameDrainStats()
  : valid_frames(0), invalid_frames(0), discarded_bytes(0)
  {
  }

  std::size_t valid_frames;
  std::size_t invalid_frames;
  std::size_t discarded_bytes;
};

struct LatestFrameSelectionStats
{
  LatestFrameSelectionStats()
  : valid_frames(0), dropped_frames(0), expired_frames(0),
    backlog_stale_frames(0), coalescing_events(0)
  {
  }

  std::uint64_t valid_frames;
  std::uint64_t dropped_frames;
  std::uint64_t expired_frames;
  std::uint64_t backlog_stale_frames;
  std::uint64_t coalescing_events;
};

enum class PendingFrameAction
{
  kDefer,
  kDiscardBacklogStale,
  kPublish
};

inline PendingFrameAction choose_pending_frame_action(
  bool backlog_remaining,
  bool pending_was_deferred,
  bool received_valid_frame_this_cycle)
{
  // Publish the newest complete frame immediately. Remaining bytes may only
  // be a partial next frame; waiting for the serial buffer to drain would
  // turn a valid latest-sample update into avoidable odometry starvation.
  if (received_valid_frame_this_cycle) {
    return PendingFrameAction::kPublish;
  }
  if (backlog_remaining) {
    return PendingFrameAction::kDefer;
  }
  if (pending_was_deferred && !received_valid_frame_this_cycle) {
    return PendingFrameAction::kDiscardBacklogStale;
  }
  return PendingFrameAction::kPublish;
}

inline std::size_t bounded_serial_read_size(
  std::size_t available_bytes,
  std::size_t remaining_budget,
  std::size_t buffer_capacity)
{
  return std::min(
    available_bytes, std::min(remaining_budget, buffer_capacity));
}

template<std::size_t N>
std::array<std::uint8_t, N * 2U> make_fail_closed_recovery_stream(
  const std::array<std::uint8_t, N> & stop_frame)
{
  std::array<std::uint8_t, N * 2U> recovery{};
  std::copy(stop_frame.begin(), stop_frame.end(), recovery.begin() + N);
  return recovery;
}

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
    XorFrameDrainStats ignored_stats;
    return pop_frame(frame_header, frame_tail, frame, ignored_stats);
  }

  bool pop_latest_frame(
    std::uint8_t frame_header,
    std::uint8_t frame_tail,
    std::array<std::uint8_t, N> & latest_frame,
    XorFrameDrainStats & stats)
  {
    stats = XorFrameDrainStats();
    std::array<std::uint8_t, N> candidate{};
    bool found_frame = false;
    while (pop_frame(frame_header, frame_tail, candidate, stats)) {
      latest_frame = candidate;
      found_frame = true;
    }
    return found_frame;
  }

  std::size_t buffered_size() const
  {
    return buffer_.size();
  }

private:
  bool pop_frame(
    std::uint8_t frame_header,
    std::uint8_t frame_tail,
    std::array<std::uint8_t, N> & frame,
    XorFrameDrainStats & stats)
  {
    while (true) {
      const auto header = std::find(buffer_.begin(), buffer_.end(), frame_header);
      if (header == buffer_.end()) {
        stats.discarded_bytes += buffer_.size();
        buffer_.clear();
        return false;
      }
      stats.discarded_bytes +=
        static_cast<std::size_t>(std::distance(buffer_.begin(), header));
      buffer_.erase(buffer_.begin(), header);
      if (buffer_.size() < N) {
        return false;
      }

      if (buffer_[N - 1] == frame_tail && checksum_matches()) {
        std::copy_n(buffer_.begin(), N, frame.begin());
        buffer_.erase(buffer_.begin(), buffer_.begin() + N);
        ++stats.valid_frames;
        return true;
      }

      ++stats.invalid_frames;
      ++stats.discarded_bytes;
      buffer_.pop_front();
    }
  }
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

template<std::size_t N>
class LatestFrameSelector
{
public:
  LatestFrameSelector()
  : has_latest_(false)
  {
  }

  void offer(
    const std::array<std::uint8_t, N> & latest_frame,
    std::size_t valid_frames,
    double received_time_sec)
  {
    if (valid_frames == 0) {
      return;
    }

    const std::uint64_t candidate_frames =
      static_cast<std::uint64_t>(valid_frames) + (has_latest_ ? 1U : 0U);
    stats_.valid_frames += static_cast<std::uint64_t>(valid_frames);
    if (candidate_frames > 1U) {
      stats_.dropped_frames += candidate_frames - 1U;
      ++stats_.coalescing_events;
    }
    latest_frame_ = latest_frame;
    latest_received_time_sec_ = received_time_sec;
    has_latest_ = true;
  }

  bool expire_if_older_than(double now_sec, double max_age_sec)
  {
    if (!has_latest_) {
      return false;
    }
    const double age_sec = now_sec - latest_received_time_sec_;
    if (
      std::isfinite(age_sec) && age_sec >= 0.0 &&
      age_sec <= max_age_sec)
    {
      return false;
    }

    has_latest_ = false;
    ++stats_.dropped_frames;
    ++stats_.expired_frames;
    return true;
  }

  bool discard_backlog_stale()
  {
    if (!has_latest_) {
      return false;
    }
    has_latest_ = false;
    ++stats_.dropped_frames;
    ++stats_.backlog_stale_frames;
    return true;
  }

  bool take_latest(std::array<std::uint8_t, N> & latest_frame)
  {
    if (!has_latest_) {
      return false;
    }
    latest_frame = latest_frame_;
    has_latest_ = false;
    return true;
  }

  bool has_latest() const
  {
    return has_latest_;
  }

  const LatestFrameSelectionStats & stats() const
  {
    return stats_;
  }

private:
  bool has_latest_;
  double latest_received_time_sec_ = 0.0;
  std::array<std::uint8_t, N> latest_frame_{};
  LatestFrameSelectionStats stats_;
};

#endif  // ORIGINCAR_BASE__SERIAL_FRAME_HPP_
