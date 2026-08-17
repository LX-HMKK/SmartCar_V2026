#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <set>
#include <string>

namespace smartcar_safety {

using ActionUuid = std::array<std::uint8_t, 16>;
using TwistComponents = std::array<double, 6>;

// Mission recovery may only use this native Nav2 BackUp speed. Configurations
// may lower it, but can never raise the command boundary above this cap.
constexpr double kForwardRecoveryMaxReverseSpeed{0.25};

enum class MotionDirection : std::uint8_t {
  Stop = 0,
  Forward = 1,
  Reverse = 2,
  ForwardRecovery = 3,
};

enum class DirectionGuardPhase : std::uint8_t {
  Stopped,
  Prepared,
  Active,
  Faulted,
};

struct DirectionGuardConfig {
  double candidate_timeout_sec{0.40};
  // Zero disables lease-expiry as an in-motion stop condition.
  double permit_timeout_sec{0.0};
  double prepare_timeout_sec{5.0};
  double raw_odom_timeout_sec{0.25};
  double stop_settle_sec{0.25};
  double stop_linear_speed_threshold{0.01};
  double stop_angular_speed_threshold{0.05};
  double zero_epsilon{1.0e-6};
  double direction_epsilon{1.0e-4};
  double forward_recovery_max_reverse_speed{
      kForwardRecoveryMaxReverseSpeed};
};

struct LeaseIdentity {
  std::uint64_t boot_epoch{0};
  std::uint64_t lease_id{0};
  std::uint64_t generation{0};
  ActionUuid action_uuid{};
};

struct GuardOperationResult {
  bool success{false};
  std::string status;
};

struct PrepareMotionResult : GuardOperationResult {
  std::uint64_t boot_epoch{0};
  std::uint64_t lease_id{0};
};

class DirectionGuard {
public:
  DirectionGuard(DirectionGuardConfig config, std::uint64_t boot_epoch);

  PrepareMotionResult prepare(MotionDirection direction, std::uint64_t generation,
                              const ActionUuid &action_uuid, double now_sec);
  GuardOperationResult activate(const LeaseIdentity &identity, double now_sec);
  GuardOperationResult renew(const LeaseIdentity &identity, double now_sec);
  GuardOperationResult stop(double now_sec);

  TwistComponents on_candidate(const TwistComponents &candidate, double now_sec);
  void on_raw_odom(const TwistComponents &twist, bool finite, double now_sec);
  TwistComponents evaluate(double now_sec);

  bool stop_ready(double now_sec) const;
  DirectionGuardPhase phase() const;
  MotionDirection direction() const;
  std::string status() const;
  std::uint64_t boot_epoch() const;
  std::optional<LeaseIdentity> identity() const;

  static const TwistComponents &zero_command();

private:
  static bool valid_time(double now_sec);
  static bool uuid_is_zero(const ActionUuid &uuid);
  bool identity_matches(const LeaseIdentity &identity) const;
  bool candidate_is_zero(const TwistComponents &candidate) const;
  bool candidate_has_only_supported_axes(
      const TwistComponents &candidate) const;
  bool direction_is_allowed(double linear_x) const;
  std::optional<std::string> forward_recovery_command_warning(
      const TwistComponents &candidate) const;
  bool raw_odom_ready(double now_sec) const;
  bool prepared_expired(double now_sec) const;
  bool active_expired(double now_sec);
  void latch_fault(const std::string &reason);
  void reset_stop_barrier(double now_sec, bool candidate_quiescent);

  DirectionGuardConfig config_;
  std::uint64_t boot_epoch_;
  std::uint64_t next_lease_id_{1};
  DirectionGuardPhase phase_{DirectionGuardPhase::Stopped};
  MotionDirection direction_{MotionDirection::Stop};
  std::string status_{"stopped"};
  std::optional<LeaseIdentity> identity_;
  std::set<ActionUuid> consumed_action_uuids_;

  double stop_started_at_{0.0};
  std::optional<double> candidate_zero_since_;
  std::optional<double> odom_stopped_since_;
  std::optional<double> last_candidate_at_;
  std::optional<double> last_odom_at_;
  std::optional<double> prepared_at_;
  std::optional<double> activated_at_;
  std::optional<double> last_permit_at_;
  std::optional<TwistComponents> last_candidate_;
  bool active_candidate_seen_{false};
};

}  // namespace smartcar_safety
