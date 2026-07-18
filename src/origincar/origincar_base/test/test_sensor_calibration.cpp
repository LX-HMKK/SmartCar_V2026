#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "origincar_base/sensor_calibration.hpp"

namespace
{

TEST(SensorCalibrationTest, DefaultsPreserveExistingCalibration)
{
  const SensorCalibration calibration;

  EXPECT_DOUBLE_EQ(calibration.longitudinal_velocity_scale, 1.03);
  EXPECT_DOUBLE_EQ(calibration.lateral_velocity_scale, 1.125);
  EXPECT_DOUBLE_EQ(calibration.yaw_velocity_scale, 1.0);
  EXPECT_DOUBLE_EQ(calibration.gyro_z_scale, 1.0);
  EXPECT_DOUBLE_EQ(calibration.gyro_z_bias, 0.0);
  EXPECT_DOUBLE_EQ(calibration.steering_command_scale, 0.5);
  EXPECT_DOUBLE_EQ(calibration.steering_command_offset_rad, 0.0);
  EXPECT_DOUBLE_EQ(calibration.max_calibrated_steering_command_rad, 0.225);
}

TEST(SensorCalibrationTest, CalibratesSensorSampleExactlyOnce)
{
  SensorCalibration calibration;
  calibration.longitudinal_velocity_scale = 2.0;
  calibration.lateral_velocity_scale = -3.0;
  calibration.yaw_velocity_scale = 4.0;
  calibration.gyro_z_scale = 1.5;
  calibration.gyro_z_bias = 0.25;

  const SensorSample calibrated = calibrate_sensor_sample(
    SensorSample{1.0, -2.0, 0.5, 2.0}, calibration);

  EXPECT_DOUBLE_EQ(calibrated.vx, 2.0);
  EXPECT_DOUBLE_EQ(calibrated.vy, 6.0);
  EXPECT_DOUBLE_EQ(calibrated.wheel_wz, 2.0);
  EXPECT_DOUBLE_EQ(calibrated.gyro_z, 2.75);
}

TEST(SensorCalibrationTest, AppliesAckermannLateralConstraintBeforeCalibration)
{
  EXPECT_DOUBLE_EQ(constrained_lateral_velocity(0.4, true), 0.0);
  EXPECT_DOUBLE_EQ(constrained_lateral_velocity(0.4, false), 0.4);
  EXPECT_DOUBLE_EQ(constrained_lateral_velocity(-0.4, false), -0.4);
}

TEST(SensorCalibrationTest, AppliesSteeringAffineCalibration)
{
  SensorCalibration calibration;
  calibration.steering_command_scale = -0.5;
  calibration.steering_command_offset_rad = 0.1;

  EXPECT_DOUBLE_EQ(calibrate_steering_command(0.4, calibration), -0.1);
}

TEST(SensorCalibrationTest, RejectsSteeringOutsideConfiguredPhysicalLimit)
{
  SensorCalibration calibration;
  calibration.steering_command_scale = 2.0;
  calibration.max_calibrated_steering_command_rad = 0.45;

  EXPECT_THROW(
    calibrate_steering_command(0.3, calibration),
    std::invalid_argument);

  calibration.max_calibrated_steering_command_rad = 0.0;
  EXPECT_THROW(validate_sensor_calibration(calibration), std::invalid_argument);
}

TEST(SensorCalibrationTest, EncodesFiniteProtocolValuesWithinInt16Range)
{
  EXPECT_EQ(encode_protocol_milli_value("positive", 32.767), 32767);
  EXPECT_EQ(encode_protocol_milli_value("negative", -32.768), -32768);
  EXPECT_EQ(encode_protocol_milli_value("truncated", 0.1239), 123);
  EXPECT_EQ(encode_protocol_milli_value("positive fraction", 32.7679), 32767);
  EXPECT_EQ(encode_protocol_milli_value("negative fraction", -32.7689), -32768);
}

TEST(SensorCalibrationTest, RejectsInvalidProtocolValues)
{
  EXPECT_THROW(
    encode_protocol_milli_value("too positive", 32.768),
    std::invalid_argument);
  EXPECT_THROW(
    encode_protocol_milli_value("too negative", -32.769),
    std::invalid_argument);
  EXPECT_THROW(
    encode_protocol_milli_value(
      "not finite", std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    encode_protocol_milli_value("scaled overflow", std::numeric_limits<double>::max()),
    std::invalid_argument);
}

TEST(SensorCalibrationTest, RejectsNonFiniteOrZeroCalibration)
{
  const std::array<double, 3> invalid_values = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
  };

  for (const double invalid : invalid_values) {
    SensorCalibration calibration;
    calibration.gyro_z_bias = invalid;
    EXPECT_THROW(validate_sensor_calibration(calibration), std::invalid_argument);
  }

  SensorCalibration calibration;
  calibration.longitudinal_velocity_scale = 0.0;
  EXPECT_THROW(validate_sensor_calibration(calibration), std::invalid_argument);
}

TEST(SensorCalibrationTest, RejectsNonFiniteCalibratedResult)
{
  SensorCalibration calibration;
  calibration.longitudinal_velocity_scale =
    std::numeric_limits<double>::max();

  EXPECT_THROW(
    calibrate_sensor_sample(
      SensorSample{2.0, 0.0, 0.0, 0.0}, calibration),
    std::invalid_argument);
}

TEST(SensorCalibrationTest, BuildsValidatedThreeAndSixDimensionalCovariance)
{
  const auto covariance3 = diagonal_covariance<3>(
    validated_covariance_diagonal<3>({1.0, 2.0, 3.0}, "imu"));
  EXPECT_DOUBLE_EQ(covariance3[0], 1.0);
  EXPECT_DOUBLE_EQ(covariance3[4], 2.0);
  EXPECT_DOUBLE_EQ(covariance3[8], 3.0);
  EXPECT_DOUBLE_EQ(covariance3[1], 0.0);

  const auto covariance6 = diagonal_covariance<6>(
    validated_covariance_diagonal<6>(
      {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}, "odom"));
  for (std::size_t index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(covariance6[index * 6 + index], index + 1.0);
  }
  EXPECT_DOUBLE_EQ(covariance6[1], 0.0);
}

TEST(SensorCalibrationTest, RejectsInvalidCovarianceDiagonals)
{
  EXPECT_THROW(
    validated_covariance_diagonal<3>({1.0, 2.0}, "short"),
    std::invalid_argument);
  EXPECT_THROW(
    validated_covariance_diagonal<3>({1.0, 0.0, 3.0}, "zero"),
    std::invalid_argument);
  EXPECT_THROW(
    validated_covariance_diagonal<3>(
      {1.0, std::numeric_limits<double>::infinity(), 3.0}, "infinite"),
    std::invalid_argument);
}

TEST(SensorCalibrationTest, IntegrationUsesAlreadyCalibratedVelocity)
{
  const PlanarPose next = integrate_planar(
    PlanarPose{1.0, 2.0, 0.0},
    SensorSample{3.0, 0.5, 0.25, 0.0},
    2.0);

  EXPECT_DOUBLE_EQ(next.x, 7.0);
  EXPECT_DOUBLE_EQ(next.y, 3.0);
  EXPECT_DOUBLE_EQ(next.yaw, 0.5);
}

TEST(SensorCalibrationTest, IntegrationClockSkipsFirstAndInvalidIntervals)
{
  IntegrationClock clock(0.25);

  IntegrationDelta delta = clock.update(10.0);
  EXPECT_FALSE(delta.should_integrate);
  EXPECT_DOUBLE_EQ(delta.dt_sec, 0.0);

  delta = clock.update(10.05);
  EXPECT_TRUE(delta.should_integrate);
  EXPECT_NEAR(delta.dt_sec, 0.05, 1e-12);

  delta = clock.update(10.05);
  EXPECT_FALSE(delta.should_integrate);

  delta = clock.update(9.0);
  EXPECT_FALSE(delta.should_integrate);

  delta = clock.update(9.1);
  EXPECT_TRUE(delta.should_integrate);
  EXPECT_NEAR(delta.dt_sec, 0.1, 1e-12);

  delta = clock.update(9.5);
  EXPECT_FALSE(delta.should_integrate);

  delta = clock.update(9.6);
  EXPECT_TRUE(delta.should_integrate);
  EXPECT_NEAR(delta.dt_sec, 0.1, 1e-12);
}

TEST(SensorCalibrationTest, IntegrationClockRejectsInvalidMaximumInterval)
{
  EXPECT_THROW(IntegrationClock(0.0), std::invalid_argument);
  EXPECT_THROW(IntegrationClock(-0.1), std::invalid_argument);
  EXPECT_THROW(
    IntegrationClock(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

}  // namespace
