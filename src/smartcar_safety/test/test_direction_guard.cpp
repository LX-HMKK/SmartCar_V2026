#include "smartcar_safety/direction_guard.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include "gtest/gtest.h"

namespace {

using smartcar_safety::ActionUuid;
using smartcar_safety::DirectionGuard;
using smartcar_safety::DirectionGuardConfig;
using smartcar_safety::DirectionGuardPhase;
using smartcar_safety::LeaseIdentity;
using smartcar_safety::MotionDirection;
using smartcar_safety::TwistComponents;

DirectionGuardConfig test_config() {
  DirectionGuardConfig config;
  config.candidate_timeout_sec = 0.20;
  config.permit_timeout_sec = 0.0;
  config.prepare_timeout_sec = 1.00;
  config.raw_odom_timeout_sec = 0.20;
  config.stop_settle_sec = 0.10;
  config.stop_linear_speed_threshold = 0.01;
  config.stop_angular_speed_threshold = 0.05;
  config.zero_epsilon = 1.0e-6;
  config.direction_epsilon = 1.0e-4;
  config.forward_recovery_max_reverse_speed = 0.15;
  return config;
}

ActionUuid uuid(std::uint8_t value) {
  ActionUuid result{};
  result[0] = value;
  return result;
}

TwistComponents command(double linear_x, double angular_z = 0.0) {
  return {linear_x, 0.0, 0.0, 0.0, 0.0, angular_z};
}

void expect_zero(const TwistComponents &value) {
  EXPECT_EQ(value, DirectionGuard::zero_command());
}

void satisfy_stop_barrier(DirectionGuard &guard, double start) {
  ASSERT_TRUE(guard.stop(start).success);
  expect_zero(guard.on_candidate(command(0.0), start + 0.01));
  guard.on_raw_odom(command(0.0), true, start + 0.01);
  guard.on_raw_odom(command(0.0), true, start + 0.12);
  ASSERT_TRUE(guard.stop_ready(start + 0.12));
}

LeaseIdentity prepare_and_activate(DirectionGuard &guard,
                                   MotionDirection direction,
                                   std::uint64_t generation,
                                   const ActionUuid &action_uuid,
                                   double start) {
  satisfy_stop_barrier(guard, start);
  const auto prepared =
      guard.prepare(direction, generation, action_uuid, start + 0.12);
  EXPECT_TRUE(prepared.success) << prepared.status;
  const LeaseIdentity identity{prepared.boot_epoch, prepared.lease_id,
                               generation, action_uuid};
  const double signed_speed =
      (direction == MotionDirection::Forward ||
       direction == MotionDirection::ForwardRecovery)
          ? 0.1
          : -0.1;
  expect_zero(guard.on_candidate(command(signed_speed), start + 0.13));
  guard.on_raw_odom(command(0.0), true, start + 0.13);
  const auto activated = guard.activate(identity, start + 0.14);
  EXPECT_TRUE(activated.success) << activated.status;
  return identity;
}

TEST(DirectionGuardTest, StartsStoppedAndRequiresPostStopBarrier) {
  DirectionGuard guard(test_config(), 42);
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Stopped);
  expect_zero(guard.evaluate(0.0));

  const auto too_early =
      guard.prepare(MotionDirection::Reverse, 1, uuid(1), 0.01);
  EXPECT_FALSE(too_early.success);
  EXPECT_EQ(too_early.status, "stop_barrier_not_ready");

  satisfy_stop_barrier(guard, 0.1);
  const auto ready = guard.prepare(MotionDirection::Reverse, 1, uuid(1), 0.22);
  EXPECT_TRUE(ready.success);
  EXPECT_EQ(ready.boot_epoch, 42U);
  EXPECT_NE(ready.lease_id, 0U);
}

TEST(DirectionGuardTest, SilentOrAlreadyZeroSmootherIsQuiescentInStop) {
  DirectionGuard guard(test_config(), 42);
  ASSERT_TRUE(guard.stop(0.0).success);
  guard.on_raw_odom(command(0.0), true, 0.01);
  guard.on_raw_odom(command(0.0), true, 0.12);
  EXPECT_TRUE(guard.stop_ready(0.12));
}

TEST(DirectionGuardTest, StopBarrierRequiresFullCandidateZeroAndContinuousOdomStop) {
  DirectionGuard guard(test_config(), 42);
  ASSERT_TRUE(guard.stop(0.0).success);
  expect_zero(guard.on_candidate(command(0.0, 0.1), 0.01));
  guard.on_raw_odom(command(0.0), true, 0.01);
  guard.on_raw_odom(command(0.0), true, 0.12);
  EXPECT_FALSE(guard.stop_ready(0.12));

  expect_zero(guard.on_candidate(command(0.0), 0.13));
  guard.on_raw_odom(command(0.02), true, 0.14);
  guard.on_raw_odom(command(0.0), true, 0.15);
  guard.on_raw_odom(command(0.0), true, 0.26);
  EXPECT_TRUE(guard.stop_ready(0.26));
}

TEST(DirectionGuardTest, PreparedStateNeverForwardsMotion) {
  DirectionGuard guard(test_config(), 42);
  satisfy_stop_barrier(guard, 0.0);
  const auto prepared =
      guard.prepare(MotionDirection::Reverse, 1, uuid(1), 0.12);
  ASSERT_TRUE(prepared.success);
  expect_zero(guard.on_candidate(command(-0.1), 0.13));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Prepared);
}

TEST(DirectionGuardTest, FirstMotionCandidateStartsTheCandidateWatchdog) {
  DirectionGuard guard(test_config(), 42);
  satisfy_stop_barrier(guard, 0.0);
  const auto prepared =
      guard.prepare(MotionDirection::Reverse, 1, uuid(1), 0.12);
  ASSERT_TRUE(prepared.success);
  const LeaseIdentity identity{prepared.boot_epoch, prepared.lease_id, 1,
                               uuid(1)};
  ASSERT_TRUE(guard.activate(identity, 0.14).success);

  ASSERT_TRUE(guard.renew(identity, 0.40).success);
  expect_zero(guard.evaluate(0.41));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
  expect_zero(guard.on_candidate(command(0.0), 0.42));
  ASSERT_TRUE(guard.renew(identity, 0.65).success);
  expect_zero(guard.evaluate(0.66));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);

  EXPECT_EQ(guard.on_candidate(command(-0.1), 0.67), command(-0.1));
  ASSERT_TRUE(guard.renew(identity, 0.85).success);
  expect_zero(guard.evaluate(0.88));
  EXPECT_EQ(guard.status(), "fault_candidate_timeout");
}

TEST(DirectionGuardTest, ReverseLeasePreservesAngularSignForCandidateAndReplay) {
  DirectionGuard guard(test_config(), 42);
  const auto identity = prepare_and_activate(
      guard, MotionDirection::Reverse, 1, uuid(1), 0.0);
  EXPECT_EQ(guard.on_candidate(command(-0.12, 0.2), 0.15),
            command(-0.12, 0.2));
  EXPECT_EQ(guard.evaluate(0.16), command(-0.12, 0.2));

  EXPECT_EQ(guard.on_candidate(command(-0.12, -0.3), 0.17),
            command(-0.12, -0.3));
  EXPECT_EQ(guard.evaluate(0.18), command(-0.12, -0.3));
  EXPECT_TRUE(guard.renew(identity, 0.20).success);

  const auto tiny_positive = guard.on_candidate(command(0.00005, 0.2), 0.21);
  expect_zero(tiny_positive);
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
}

TEST(DirectionGuardTest, ForwardLeasePreservesAngularSignForCandidateAndReplay) {
  DirectionGuard guard(test_config(), 42);
  const auto identity = prepare_and_activate(
      guard, MotionDirection::Forward, 1, uuid(1), 0.0);
  EXPECT_EQ(guard.on_candidate(command(0.12, -0.2), 0.15),
            command(0.12, -0.2));
  EXPECT_EQ(guard.evaluate(0.16), command(0.12, -0.2));

  EXPECT_EQ(guard.on_candidate(command(0.12, 0.3), 0.17),
            command(0.12, 0.3));
  EXPECT_EQ(guard.evaluate(0.18), command(0.12, 0.3));
  EXPECT_TRUE(guard.renew(identity, 0.20).success);
  expect_zero(guard.on_candidate(command(-0.00005, -0.2), 0.21));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
}

TEST(DirectionGuardTest, ForwardRecoveryLeaseAllowsOnlyCappedNativeBackUp) {
  DirectionGuard guard(test_config(), 42);
  const auto identity = prepare_and_activate(
      guard, MotionDirection::ForwardRecovery, 1, uuid(1), 0.0);

  EXPECT_EQ(guard.on_candidate(command(0.12, 0.2), 0.15),
            command(0.12, 0.2));
  EXPECT_EQ(guard.on_candidate(command(-0.15), 0.16), command(-0.15));
  EXPECT_EQ(guard.evaluate(0.17), command(-0.15));
  EXPECT_TRUE(guard.renew(identity, 0.18).success);
}

TEST(DirectionGuardTest, ForwardRecoveryRejectsAnySpeedBeyondNativeBackUpCap) {
  DirectionGuard guard(test_config(), 42);
  prepare_and_activate(guard, MotionDirection::ForwardRecovery, 1, uuid(1),
                       0.0);

  expect_zero(guard.on_candidate(command(-0.150001), 0.15));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
  EXPECT_EQ(guard.status(), "warning_recovery_reverse_speed_rejected");
  EXPECT_EQ(guard.on_candidate(command(0.12, 0.2), 0.16),
            command(0.12, 0.2));
  EXPECT_EQ(guard.status(), "active_forward_recovery");
}

TEST(DirectionGuardTest, ForwardRecoveryDoesNotLatchOnRepeatedReverse) {
  DirectionGuard guard(test_config(), 42);
  prepare_and_activate(guard, MotionDirection::ForwardRecovery, 1, uuid(1),
                       0.0);

  EXPECT_EQ(guard.on_candidate(command(-0.15), 0.15), command(-0.15));
  expect_zero(guard.on_candidate(command(0.0), 0.16));
  EXPECT_EQ(guard.on_candidate(command(-0.10), 0.17), command(-0.10));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
}

TEST(DirectionGuardTest, ForwardRecoveryRejectsTurningWhileBackingUp) {
  DirectionGuard guard(test_config(), 42);
  prepare_and_activate(guard, MotionDirection::ForwardRecovery, 1, uuid(1),
                       0.0);

  expect_zero(guard.on_candidate(command(-0.10, 0.01), 0.15));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
  EXPECT_EQ(guard.status(), "warning_recovery_reverse_turn_rejected");
  EXPECT_EQ(guard.on_candidate(command(0.10), 0.16), command(0.10));
}

TEST(DirectionGuardTest, ForwardRecoveryDoesNotLatchOnElapsedReverseTime) {
  DirectionGuard guard(test_config(), 42);
  prepare_and_activate(guard, MotionDirection::ForwardRecovery, 1, uuid(1),
                       0.0);

  EXPECT_EQ(guard.on_candidate(command(-0.10), 0.15), command(-0.10));
  EXPECT_EQ(guard.on_candidate(command(-0.10), 4.66), command(-0.10));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
}

TEST(DirectionGuardTest, ForwardRecoveryCapCannotExceedNativeBackUpCap) {
  auto config = test_config();
  config.forward_recovery_max_reverse_speed = 0.151;
  EXPECT_THROW(DirectionGuard(config, 42), std::invalid_argument);
}

TEST(DirectionGuardTest, WrongDirectionLatchesAndHoldsFullZero) {
  DirectionGuard guard(test_config(), 42);
  const auto identity = prepare_and_activate(
      guard, MotionDirection::Reverse, 1, uuid(1), 0.0);

  expect_zero(guard.on_candidate(command(0.01, 0.5), 0.15));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Faulted);
  EXPECT_EQ(guard.status(), "fault_wrong_direction");
  expect_zero(guard.on_candidate(command(-0.1), 0.16));
  EXPECT_FALSE(guard.renew(identity, 0.17).success);

  EXPECT_TRUE(guard.stop(0.18).success);
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Stopped);
  EXPECT_FALSE(guard.stop_ready(0.18));
}

TEST(DirectionGuardTest, NonFiniteCandidateLatchesInEveryPhase) {
  DirectionGuard guard(test_config(), 42);
  auto invalid = command(0.0);
  invalid[4] = std::numeric_limits<double>::quiet_NaN();
  expect_zero(guard.on_candidate(invalid, 0.01));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Faulted);
  EXPECT_EQ(guard.status(), "fault_candidate_invalid");
}

TEST(DirectionGuardTest, UnsupportedTwistAxesLatchBeforeForwarding) {
  DirectionGuard guard(test_config(), 42);
  prepare_and_activate(guard, MotionDirection::Reverse, 1, uuid(1), 0.0);
  auto unsupported = command(-0.1, 0.2);
  unsupported[1] = 0.01;
  expect_zero(guard.on_candidate(unsupported, 0.15));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Faulted);
  EXPECT_EQ(guard.status(), "fault_unsupported_twist");
}

TEST(DirectionGuardTest, EveryRawOdomAxisMustRemainStopped) {
  DirectionGuard guard(test_config(), 42);
  ASSERT_TRUE(guard.stop(0.0).success);
  expect_zero(guard.on_candidate(command(0.0), 0.01));
  auto moving_laterally = command(0.0);
  moving_laterally[1] = 0.02;
  guard.on_raw_odom(moving_laterally, true, 0.01);
  guard.on_raw_odom(command(0.0), true, 0.02);
  guard.on_raw_odom(command(0.0), true, 0.11);
  EXPECT_FALSE(guard.stop_ready(0.11));
  guard.on_raw_odom(command(0.0), true, 0.13);
  EXPECT_TRUE(guard.stop_ready(0.13));
}

TEST(DirectionGuardTest, RawOdomGapResetsContinuousStopDwell) {
  DirectionGuard guard(test_config(), 42);
  ASSERT_TRUE(guard.stop(0.0).success);
  guard.on_raw_odom(command(0.0), true, 0.01);
  guard.on_raw_odom(command(0.0), true, 0.25);
  EXPECT_FALSE(guard.stop_ready(0.25));
  guard.on_raw_odom(command(0.0), true, 0.36);
  EXPECT_TRUE(guard.stop_ready(0.36));
}

TEST(DirectionGuardTest, CandidateTimeoutLatchesWhenLeaseExpiryIsDisabled) {
  {
    DirectionGuard guard(test_config(), 42);
    const auto identity = prepare_and_activate(
        guard, MotionDirection::Reverse, 1, uuid(1), 0.0);
    ASSERT_TRUE(guard.renew(identity, 0.25).success);
    expect_zero(guard.evaluate(0.36));
    EXPECT_EQ(guard.status(), "fault_candidate_timeout");
  }
  {
    DirectionGuard guard(test_config(), 42);
    prepare_and_activate(guard, MotionDirection::Reverse, 1, uuid(1), 0.0);
    guard.on_candidate(command(-0.1), 0.30);
    EXPECT_EQ(guard.evaluate(0.45), command(-0.1));
    EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
    expect_zero(guard.evaluate(0.51));
    EXPECT_EQ(guard.status(), "fault_candidate_timeout");
  }
}

TEST(DirectionGuardTest, ExplicitZeroCandidateKeepsRecoveryQuiescent) {
  DirectionGuard guard(test_config(), 42);
  const auto identity = prepare_and_activate(
      guard, MotionDirection::Forward, 1, uuid(1), 0.0);

  EXPECT_EQ(guard.on_candidate(command(0.12), 0.15), command(0.12));
  expect_zero(guard.on_candidate(command(0.0), 0.16));
  ASSERT_TRUE(guard.renew(identity, 0.35).success);
  expect_zero(guard.evaluate(0.37));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Active);
  EXPECT_EQ(guard.on_candidate(command(0.12), 0.38), command(0.12));
}

TEST(DirectionGuardTest, IdentityMismatchCannotRenewOrActivate) {
  DirectionGuard guard(test_config(), 42);
  satisfy_stop_barrier(guard, 0.0);
  const auto prepared =
      guard.prepare(MotionDirection::Reverse, 7, uuid(1), 0.12);
  ASSERT_TRUE(prepared.success);
  expect_zero(guard.on_candidate(command(-0.1), 0.13));
  guard.on_raw_odom(command(0.0), true, 0.13);

  LeaseIdentity wrong_epoch{43, prepared.lease_id, 7, uuid(1)};
  EXPECT_FALSE(guard.activate(wrong_epoch, 0.14).success);
  LeaseIdentity correct{42, prepared.lease_id, 7, uuid(1)};
  ASSERT_TRUE(guard.activate(correct, 0.15).success);

  LeaseIdentity wrong_generation = correct;
  wrong_generation.generation = 6;
  EXPECT_FALSE(guard.renew(wrong_generation, 0.20).success);
  EXPECT_TRUE(guard.renew(correct, 0.21).success);
}

TEST(DirectionGuardTest, StopInvalidatesLeaseAndUuidCannotBeReplayed) {
  DirectionGuard guard(test_config(), 42);
  const auto old_identity = prepare_and_activate(
      guard, MotionDirection::Reverse, 1, uuid(1), 0.0);
  ASSERT_TRUE(guard.stop(0.2).success);
  EXPECT_FALSE(guard.renew(old_identity, 0.21).success);
  satisfy_stop_barrier(guard, 0.3);

  const auto replay =
      guard.prepare(MotionDirection::Reverse, 1, uuid(1), 0.42);
  EXPECT_FALSE(replay.success);
  EXPECT_EQ(replay.status, "action_uuid_replayed");
  const auto next =
      guard.prepare(MotionDirection::Reverse, 1, uuid(2), 0.42);
  EXPECT_TRUE(next.success);
  EXPECT_NE(next.lease_id, old_identity.lease_id);
}

TEST(DirectionGuardTest, PreparedLeaseExpiresFailClosed) {
  DirectionGuard guard(test_config(), 42);
  satisfy_stop_barrier(guard, 0.0);
  ASSERT_TRUE(
      guard.prepare(MotionDirection::Reverse, 1, uuid(1), 0.12).success);
  expect_zero(guard.evaluate(1.13));
  EXPECT_EQ(guard.phase(), DirectionGuardPhase::Faulted);
  EXPECT_EQ(guard.status(), "fault_prepare_timeout");
}

}  // namespace
