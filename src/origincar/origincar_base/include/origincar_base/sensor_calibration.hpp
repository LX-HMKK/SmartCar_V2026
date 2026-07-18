#ifndef ORIGINCAR_BASE__SENSOR_CALIBRATION_HPP_
#define ORIGINCAR_BASE__SENSOR_CALIBRATION_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

struct SensorCalibration
{
  SensorCalibration()
  : longitudinal_velocity_scale(1.03),
    lateral_velocity_scale(1.125),
    yaw_velocity_scale(1.0),
    gyro_z_scale(1.0),
    gyro_z_bias(0.0),
    steering_command_scale(0.5),
    steering_command_offset_rad(0.0),
    max_calibrated_steering_command_rad(0.225)
  {
  }

  double longitudinal_velocity_scale;
  double lateral_velocity_scale;
  double yaw_velocity_scale;
  double gyro_z_scale;
  double gyro_z_bias;
  double steering_command_scale;
  double steering_command_offset_rad;
  double max_calibrated_steering_command_rad;
};

struct SensorSample
{
  SensorSample(double vx_value, double vy_value, double wheel_wz_value, double gyro_z_value)
  : vx(vx_value), vy(vy_value), wheel_wz(wheel_wz_value), gyro_z(gyro_z_value)
  {
  }

  double vx;
  double vy;
  double wheel_wz;
  double gyro_z;
};

struct PlanarPose
{
  PlanarPose(double x_value, double y_value, double yaw_value)
  : x(x_value), y(y_value), yaw(yaw_value)
  {
  }

  double x;
  double y;
  double yaw;
};

struct IntegrationDelta
{
  IntegrationDelta(bool integrate, double delta_sec)
  : should_integrate(integrate), dt_sec(delta_sec)
  {
  }

  bool should_integrate;
  double dt_sec;
};

inline void require_finite_value(const std::string & name, double value)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
}

inline void require_finite_nonzero_value(const std::string & name, double value)
{
  require_finite_value(name, value);
  if (value == 0.0) {
    throw std::invalid_argument(name + " must be nonzero");
  }
}

inline void validate_sensor_calibration(const SensorCalibration & calibration)
{
  require_finite_nonzero_value(
    "longitudinal_velocity_scale", calibration.longitudinal_velocity_scale);
  require_finite_nonzero_value(
    "lateral_velocity_scale", calibration.lateral_velocity_scale);
  require_finite_nonzero_value("yaw_velocity_scale", calibration.yaw_velocity_scale);
  require_finite_nonzero_value("gyro_z_scale", calibration.gyro_z_scale);
  require_finite_value("gyro_z_bias", calibration.gyro_z_bias);
  require_finite_nonzero_value(
    "steering_command_scale", calibration.steering_command_scale);
  require_finite_value(
    "steering_command_offset_rad", calibration.steering_command_offset_rad);
  require_finite_value(
    "max_calibrated_steering_command_rad",
    calibration.max_calibrated_steering_command_rad);
  if (calibration.max_calibrated_steering_command_rad <= 0.0) {
    throw std::invalid_argument(
            "max_calibrated_steering_command_rad must be greater than zero");
  }
}

inline void validate_sensor_sample(const SensorSample & sample)
{
  require_finite_value("vx", sample.vx);
  require_finite_value("vy", sample.vy);
  require_finite_value("wheel_wz", sample.wheel_wz);
  require_finite_value("gyro_z", sample.gyro_z);
}

inline double constrained_lateral_velocity(double raw_vy, bool is_ackermann)
{
  require_finite_value("raw_vy", raw_vy);
  return is_ackermann ? 0.0 : raw_vy;
}

inline SensorSample calibrate_sensor_sample(
  const SensorSample & raw,
  const SensorCalibration & calibration)
{
  validate_sensor_sample(raw);
  validate_sensor_calibration(calibration);

  const SensorSample calibrated(
    raw.vx * calibration.longitudinal_velocity_scale,
    raw.vy * calibration.lateral_velocity_scale,
    raw.wheel_wz * calibration.yaw_velocity_scale,
    raw.gyro_z * calibration.gyro_z_scale - calibration.gyro_z_bias);
  validate_sensor_sample(calibrated);
  return calibrated;
}

inline double calibrate_steering_command(
  double requested_rad,
  const SensorCalibration & calibration)
{
  require_finite_value("requested steering angle", requested_rad);
  validate_sensor_calibration(calibration);
  const double calibrated =
    requested_rad * calibration.steering_command_scale +
    calibration.steering_command_offset_rad;
  require_finite_value("calibrated steering angle", calibrated);
  if (std::abs(calibrated) > calibration.max_calibrated_steering_command_rad) {
    throw std::invalid_argument("calibrated steering angle exceeds configured limit");
  }
  return calibrated;
}

inline std::int16_t encode_protocol_milli_value(
  const std::string & name,
  double value)
{
  require_finite_value(name, value);
  const double scaled = value * 1000.0;
  require_finite_value(name + " scaled value", scaled);
  const double truncated = std::trunc(scaled);

  const double minimum = static_cast<double>(std::numeric_limits<std::int16_t>::min());
  const double maximum = static_cast<double>(std::numeric_limits<std::int16_t>::max());
  if (truncated < minimum || truncated > maximum) {
    throw std::invalid_argument(name + " exceeds signed 16-bit protocol range");
  }
  return static_cast<std::int16_t>(truncated);
}

template<std::size_t N>
inline std::array<double, N> validated_covariance_diagonal(
  const std::vector<double> & values,
  const std::string & name)
{
  if (values.size() != N) {
    throw std::invalid_argument(name + " must contain exactly " + std::to_string(N) + " values");
  }

  std::array<double, N> result;
  for (std::size_t index = 0; index < N; ++index) {
    require_finite_value(name, values[index]);
    if (values[index] <= 0.0) {
      throw std::invalid_argument(name + " values must be greater than zero");
    }
    result[index] = values[index];
  }
  return result;
}

template<std::size_t N>
inline std::array<double, N * N> diagonal_covariance(
  const std::array<double, N> & diagonal)
{
  std::array<double, N * N> covariance;
  covariance.fill(0.0);
  for (std::size_t index = 0; index < N; ++index) {
    covariance[index * N + index] = diagonal[index];
  }
  return covariance;
}

inline PlanarPose integrate_planar(
  const PlanarPose & pose,
  const SensorSample & calibrated,
  double dt_sec)
{
  require_finite_value("pose x", pose.x);
  require_finite_value("pose y", pose.y);
  require_finite_value("pose yaw", pose.yaw);
  validate_sensor_sample(calibrated);
  require_finite_value("integration dt", dt_sec);
  if (dt_sec <= 0.0) {
    throw std::invalid_argument("integration dt must be greater than zero");
  }

  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const PlanarPose next(
    pose.x + (calibrated.vx * cos_yaw - calibrated.vy * sin_yaw) * dt_sec,
    pose.y + (calibrated.vx * sin_yaw + calibrated.vy * cos_yaw) * dt_sec,
    pose.yaw + calibrated.wheel_wz * dt_sec);
  require_finite_value("integrated pose x", next.x);
  require_finite_value("integrated pose y", next.y);
  require_finite_value("integrated pose yaw", next.yaw);
  return next;
}

class IntegrationClock
{
public:
  explicit IntegrationClock(double max_integration_dt_sec = 0.25)
  : initialized_(false),
    last_sample_time_sec_(0.0),
    max_integration_dt_sec_(max_integration_dt_sec)
  {
    require_finite_value("max integration dt", max_integration_dt_sec_);
    if (max_integration_dt_sec_ <= 0.0) {
      throw std::invalid_argument("max integration dt must be greater than zero");
    }
  }

  IntegrationDelta update(double sample_time_sec)
  {
    if (!std::isfinite(sample_time_sec)) {
      return IntegrationDelta(false, 0.0);
    }
    if (!initialized_) {
      initialized_ = true;
      last_sample_time_sec_ = sample_time_sec;
      return IntegrationDelta(false, 0.0);
    }

    const double dt_sec = sample_time_sec - last_sample_time_sec_;
    last_sample_time_sec_ = sample_time_sec;
    if (
      !std::isfinite(dt_sec) || dt_sec <= 0.0 ||
      dt_sec > max_integration_dt_sec_)
    {
      return IntegrationDelta(false, 0.0);
    }
    return IntegrationDelta(true, dt_sec);
  }

private:
  bool initialized_;
  double last_sample_time_sec_;
  double max_integration_dt_sec_;
};

#endif  // ORIGINCAR_BASE__SENSOR_CALIBRATION_HPP_
