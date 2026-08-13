#include "smartcar_safety/direction_guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace smartcar_safety {
namespace {

constexpr TwistComponents ZERO_COMMAND{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void require_positive_finite(const char *name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive finite");
  }
}

void require_nonnegative_finite(const char *name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be nonnegative finite");
  }
}

bool all_finite(const TwistComponents &components) {
  return std::all_of(components.begin(), components.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

DirectionGuard::DirectionGuard(DirectionGuardConfig config,
                               std::uint64_t boot_epoch)
    : config_(std::move(config)), boot_epoch_(boot_epoch) {
  require_positive_finite("candidate_timeout_sec",
                          config_.candidate_timeout_sec);
  require_nonnegative_finite("permit_timeout_sec", config_.permit_timeout_sec);
  require_positive_finite("prepare_timeout_sec", config_.prepare_timeout_sec);
  require_positive_finite("raw_odom_timeout_sec",
                          config_.raw_odom_timeout_sec);
  require_nonnegative_finite("stop_settle_sec", config_.stop_settle_sec);
  require_nonnegative_finite("stop_linear_speed_threshold",
                             config_.stop_linear_speed_threshold);
  require_nonnegative_finite("stop_angular_speed_threshold",
                             config_.stop_angular_speed_threshold);
  require_nonnegative_finite("zero_epsilon", config_.zero_epsilon);
  require_nonnegative_finite("direction_epsilon",
                             config_.direction_epsilon);
  require_positive_finite("forward_recovery_max_reverse_speed",
                          config_.forward_recovery_max_reverse_speed);
  if (config_.forward_recovery_max_reverse_speed >
      kForwardRecoveryMaxReverseSpeed) {
    throw std::invalid_argument(
        "forward_recovery_max_reverse_speed exceeds native BackUp cap");
  }
  if (boot_epoch_ == 0) {
    throw std::invalid_argument("boot_epoch must be nonzero");
  }
  reset_stop_barrier(0.0, true);
}

PrepareMotionResult DirectionGuard::prepare(MotionDirection direction,
                                            std::uint64_t generation,
                                            const ActionUuid &action_uuid,
                                            double now_sec) {
  PrepareMotionResult result;
  result.boot_epoch = boot_epoch_;
  if (!valid_time(now_sec)) {
    latch_fault("fault_clock_invalid");
    result.status = status_;
    return result;
  }
  if (phase_ == DirectionGuardPhase::Faulted) {
    result.status = "fault_latched";
    return result;
  }
  if (phase_ != DirectionGuardPhase::Stopped) {
    result.status = "not_stopped";
    return result;
  }
  if (direction != MotionDirection::Forward &&
      direction != MotionDirection::Reverse &&
      direction != MotionDirection::ForwardRecovery) {
    result.status = "direction_invalid";
    return result;
  }
  if (generation == 0) {
    result.status = "generation_invalid";
    return result;
  }
  if (uuid_is_zero(action_uuid)) {
    result.status = "action_uuid_invalid";
    return result;
  }
  if (consumed_action_uuids_.count(action_uuid) != 0) {
    result.status = "action_uuid_replayed";
    return result;
  }
  if (!stop_ready(now_sec)) {
    result.status = "stop_barrier_not_ready";
    return result;
  }

  if (next_lease_id_ == 0) {
    latch_fault("fault_lease_id_exhausted");
    result.status = status_;
    return result;
  }
  const LeaseIdentity identity{
      boot_epoch_, next_lease_id_++, generation, action_uuid};
  identity_ = identity;
  consumed_action_uuids_.insert(action_uuid);
  direction_ = direction;
  phase_ = DirectionGuardPhase::Prepared;
  prepared_at_ = now_sec;
  activated_at_.reset();
  last_permit_at_.reset();
  active_candidate_seen_ = false;
  status_ = direction == MotionDirection::Forward
                ? "prepared_forward"
                : direction == MotionDirection::ForwardRecovery
                      ? "prepared_forward_recovery"
                      : "prepared_reverse";

  result.success = true;
  result.status = status_;
  result.lease_id = identity.lease_id;
  return result;
}

GuardOperationResult DirectionGuard::activate(const LeaseIdentity &identity,
                                              double now_sec) {
  if (!valid_time(now_sec)) {
    latch_fault("fault_clock_invalid");
    return {false, status_};
  }
  if (phase_ == DirectionGuardPhase::Faulted) {
    return {false, "fault_latched"};
  }
  if (phase_ != DirectionGuardPhase::Prepared) {
    return {false, "not_prepared"};
  }
  if (prepared_expired(now_sec)) {
    latch_fault("fault_prepare_timeout");
    return {false, status_};
  }
  if (!identity_matches(identity)) {
    return {false, "identity_mismatch"};
  }
  if (!raw_odom_ready(now_sec)) {
    return {false, "raw_odom_not_stopped"};
  }
  if (last_candidate_.has_value() &&
      !direction_is_allowed(last_candidate_.value()[0])) {
    latch_fault("fault_wrong_direction");
    return {false, status_};
  }

  phase_ = DirectionGuardPhase::Active;
  activated_at_ = now_sec;
  last_permit_at_ = now_sec;
  active_candidate_seen_ =
      last_candidate_.has_value() && last_candidate_at_.has_value() &&
      last_candidate_at_.value() >= prepared_at_.value_or(now_sec) &&
      now_sec >= last_candidate_at_.value() &&
      now_sec - last_candidate_at_.value() <= config_.candidate_timeout_sec &&
      std::abs(last_candidate_.value()[0]) > config_.direction_epsilon;
  if (!active_candidate_seen_) {
    last_candidate_.reset();
    last_candidate_at_.reset();
  }
  status_ = direction_ == MotionDirection::Forward
                ? "active_forward"
                : direction_ == MotionDirection::ForwardRecovery
                      ? "active_forward_recovery"
                      : "active_reverse";
  return {true, status_};
}

GuardOperationResult DirectionGuard::renew(const LeaseIdentity &identity,
                                           double now_sec) {
  if (!valid_time(now_sec)) {
    latch_fault("fault_clock_invalid");
    return {false, status_};
  }
  if (phase_ == DirectionGuardPhase::Faulted) {
    return {false, "fault_latched"};
  }
  if (phase_ != DirectionGuardPhase::Active) {
    return {false, "not_active"};
  }
  if (active_expired(now_sec)) {
    return {false, status_};
  }
  if (!identity_matches(identity)) {
    return {false, "identity_mismatch"};
  }
  last_permit_at_ = now_sec;
  return {true, "renewed"};
}

GuardOperationResult DirectionGuard::stop(double now_sec) {
  if (!valid_time(now_sec)) {
    latch_fault("fault_clock_invalid");
    return {false, status_};
  }
  const bool candidate_quiescent =
      !last_candidate_.has_value() || candidate_is_zero(last_candidate_.value());
  phase_ = DirectionGuardPhase::Stopped;
  direction_ = MotionDirection::Stop;
  status_ = "stopped";
  identity_.reset();
  prepared_at_.reset();
  activated_at_.reset();
  last_permit_at_.reset();
  active_candidate_seen_ = false;
  // Humble's velocity smoother stops publishing after it reaches exact zero.
  // Silence or an already-zero last sample is therefore quiescent in STOP.
  reset_stop_barrier(now_sec, candidate_quiescent);
  return {true, status_};
}

TwistComponents DirectionGuard::on_candidate(const TwistComponents &candidate,
                                             double now_sec) {
  if (!valid_time(now_sec) || !all_finite(candidate)) {
    latch_fault("fault_candidate_invalid");
    return zero_command();
  }

  last_candidate_ = candidate;
  last_candidate_at_ = now_sec;
  if (phase_ == DirectionGuardPhase::Stopped) {
    if (now_sec >= stop_started_at_ && candidate_is_zero(candidate)) {
      if (!candidate_zero_since_.has_value()) {
        candidate_zero_since_ = now_sec;
      }
    } else {
      candidate_zero_since_.reset();
    }
    return zero_command();
  }
  if (phase_ == DirectionGuardPhase::Faulted) {
    return zero_command();
  }
  if (!candidate_has_only_supported_axes(candidate)) {
    latch_fault("fault_unsupported_twist");
    return zero_command();
  }
  if (const auto warning = forward_recovery_command_warning(candidate);
      warning.has_value()) {
    // Reject only this malformed BackUp sample. The enclosing forward lease
    // stays active so Nav2 can emit its next valid command.
    status_ = warning.value();
    last_candidate_ = zero_command();
    active_candidate_seen_ = false;
    return zero_command();
  }
  if (!direction_is_allowed(candidate[0])) {
    latch_fault("fault_wrong_direction");
    return zero_command();
  }
  if (phase_ != DirectionGuardPhase::Active) {
    return zero_command();
  }
  if (status_.rfind("warning_", 0) == 0) {
    status_ = "active_forward_recovery";
  }
  if (candidate_is_zero(candidate)) {
    // VelocitySmoother may become silent after publishing an explicit zero
    // while Nav2 clears a costmap and replans. Keep forwarding zero, but do
    // not mistake that quiescent state for stale nonzero motion.
    active_candidate_seen_ = false;
  } else if (std::abs(candidate[0]) > config_.direction_epsilon) {
    active_candidate_seen_ = true;
  }
  if (active_expired(now_sec)) {
    return zero_command();
  }
  if (std::abs(candidate[0]) <= config_.direction_epsilon) {
    return zero_command();
  }
  return candidate;
}

void DirectionGuard::on_raw_odom(const TwistComponents &twist, bool finite,
                                 double now_sec) {
  if (!valid_time(now_sec)) {
    latch_fault("fault_clock_invalid");
    return;
  }
  const bool stream_continuous =
      last_odom_at_.has_value() && now_sec >= last_odom_at_.value() &&
      now_sec - last_odom_at_.value() <= config_.raw_odom_timeout_sec;
  if (!stream_continuous) {
    odom_stopped_since_.reset();
  }
  last_odom_at_ = now_sec;
  const bool stopped = finite && all_finite(twist) &&
                       std::all_of(twist.begin(), twist.begin() + 3,
                                   [this](double value) {
                                     return std::abs(value) <=
                                            config_.stop_linear_speed_threshold;
                                   }) &&
                       std::all_of(twist.begin() + 3, twist.end(),
                                   [this](double value) {
                                     return std::abs(value) <=
                                            config_.stop_angular_speed_threshold;
                                   });
  if (now_sec < stop_started_at_ || !stopped) {
    odom_stopped_since_.reset();
    return;
  }
  if (!odom_stopped_since_.has_value()) {
    odom_stopped_since_ = now_sec;
  }
}

TwistComponents DirectionGuard::evaluate(double now_sec) {
  if (!valid_time(now_sec)) {
    latch_fault("fault_clock_invalid");
    return zero_command();
  }
  if (phase_ == DirectionGuardPhase::Prepared && prepared_expired(now_sec)) {
    latch_fault("fault_prepare_timeout");
    return zero_command();
  }
  if (phase_ != DirectionGuardPhase::Active) {
    return zero_command();
  }
  if (active_expired(now_sec) || !last_candidate_.has_value()) {
    return zero_command();
  }
  const auto &candidate = last_candidate_.value();
  if (!candidate_has_only_supported_axes(candidate)) {
    latch_fault("fault_unsupported_twist");
    return zero_command();
  }
  if (!direction_is_allowed(candidate[0])) {
    latch_fault("fault_wrong_direction");
    return zero_command();
  }
  if (const auto warning = forward_recovery_command_warning(candidate);
      warning.has_value()) {
    status_ = warning.value();
    return zero_command();
  }
  if (std::abs(candidate[0]) <= config_.direction_epsilon) {
    return zero_command();
  }
  return candidate;
}

bool DirectionGuard::stop_ready(double now_sec) const {
  if (!valid_time(now_sec) || phase_ != DirectionGuardPhase::Stopped ||
      !candidate_zero_since_.has_value() || !raw_odom_ready(now_sec)) {
    return false;
  }
  return now_sec - candidate_zero_since_.value() >= config_.stop_settle_sec;
}

DirectionGuardPhase DirectionGuard::phase() const { return phase_; }

MotionDirection DirectionGuard::direction() const { return direction_; }

std::string DirectionGuard::status() const { return status_; }

std::uint64_t DirectionGuard::boot_epoch() const { return boot_epoch_; }

std::optional<LeaseIdentity> DirectionGuard::identity() const {
  return identity_;
}

const TwistComponents &DirectionGuard::zero_command() { return ZERO_COMMAND; }

bool DirectionGuard::valid_time(double now_sec) {
  return std::isfinite(now_sec) && now_sec >= 0.0;
}

bool DirectionGuard::uuid_is_zero(const ActionUuid &uuid) {
  return std::all_of(uuid.begin(), uuid.end(),
                     [](std::uint8_t value) { return value == 0; });
}

bool DirectionGuard::identity_matches(const LeaseIdentity &identity) const {
  return identity_.has_value() &&
         identity.boot_epoch == identity_->boot_epoch &&
         identity.lease_id == identity_->lease_id &&
         identity.generation == identity_->generation &&
         identity.action_uuid == identity_->action_uuid;
}

bool DirectionGuard::candidate_is_zero(
    const TwistComponents &candidate) const {
  return std::all_of(candidate.begin(), candidate.end(), [this](double value) {
    return std::abs(value) <= config_.zero_epsilon;
  });
}

bool DirectionGuard::candidate_has_only_supported_axes(
    const TwistComponents &candidate) const {
  return std::abs(candidate[1]) <= config_.zero_epsilon &&
         std::abs(candidate[2]) <= config_.zero_epsilon &&
         std::abs(candidate[3]) <= config_.zero_epsilon &&
         std::abs(candidate[4]) <= config_.zero_epsilon;
}

bool DirectionGuard::direction_is_allowed(double linear_x) const {
  if (direction_ == MotionDirection::Forward) {
    return linear_x >= -config_.direction_epsilon;
  }
  if (direction_ == MotionDirection::Reverse) {
    return linear_x <= config_.direction_epsilon;
  }
  if (direction_ == MotionDirection::ForwardRecovery) {
    return linear_x >= -config_.forward_recovery_max_reverse_speed;
  }
  return false;
}

std::optional<std::string> DirectionGuard::forward_recovery_command_warning(
    const TwistComponents &candidate) const {
  if (direction_ != MotionDirection::ForwardRecovery) {
    return std::nullopt;
  }
  const double linear_x = candidate[0];
  if (linear_x >= -config_.direction_epsilon) {
    return std::nullopt;
  }
  if (linear_x < -config_.forward_recovery_max_reverse_speed) {
    return "warning_recovery_reverse_speed_rejected";
  }
  if (std::abs(candidate[5]) > config_.zero_epsilon) {
    return "warning_recovery_reverse_turn_rejected";
  }
  return std::nullopt;
}

bool DirectionGuard::raw_odom_ready(double now_sec) const {
  return last_odom_at_.has_value() && odom_stopped_since_.has_value() &&
         now_sec >= last_odom_at_.value() &&
         now_sec - last_odom_at_.value() <= config_.raw_odom_timeout_sec &&
         now_sec >= odom_stopped_since_.value() &&
         now_sec - odom_stopped_since_.value() >= config_.stop_settle_sec;
}

bool DirectionGuard::prepared_expired(double now_sec) const {
  return !prepared_at_.has_value() || now_sec < prepared_at_.value() ||
         now_sec - prepared_at_.value() > config_.prepare_timeout_sec;
}

bool DirectionGuard::active_expired(double now_sec) {
  if (!activated_at_.has_value() || now_sec < activated_at_.value()) {
    latch_fault("fault_candidate_timeout");
    return true;
  }
  if (!active_candidate_seen_) {
    return false;
  }
  double candidate_reference = activated_at_.value();
  if (last_candidate_at_.has_value() &&
      last_candidate_at_.value() > candidate_reference) {
    candidate_reference = last_candidate_at_.value();
  }
  if (now_sec < candidate_reference ||
      now_sec - candidate_reference > config_.candidate_timeout_sec) {
    latch_fault("fault_candidate_timeout");
    return true;
  }
  return false;
}

void DirectionGuard::latch_fault(const std::string &reason) {
  phase_ = DirectionGuardPhase::Faulted;
  direction_ = MotionDirection::Stop;
  status_ = reason;
  identity_.reset();
  prepared_at_.reset();
  activated_at_.reset();
  last_permit_at_.reset();
  active_candidate_seen_ = false;
}

void DirectionGuard::reset_stop_barrier(double now_sec,
                                        bool candidate_quiescent) {
  stop_started_at_ = now_sec;
  candidate_zero_since_ = candidate_quiescent
                              ? std::optional<double>(now_sec)
                              : std::nullopt;
  odom_stopped_since_.reset();
  last_candidate_at_.reset();
  last_odom_at_.reset();
  last_candidate_.reset();
}

}  // namespace smartcar_safety
