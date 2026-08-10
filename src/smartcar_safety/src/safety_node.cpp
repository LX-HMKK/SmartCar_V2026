#include "smartcar_safety/guard.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <smartcar_interfaces/srv/hold_steering_calibration.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>

using geometry_msgs::msg::Twist;
using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using nav_msgs::msg::Odometry;
using smartcar_interfaces::srv::HoldSteeringCalibration;
using std_msgs::msg::Float32;
using std_msgs::msg::String;
using std_srvs::srv::SetBool;

namespace {

const std::array<double, 6> ZERO_COMPONENTS = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

bool sanitize_twist(const Twist &msg, std::array<double, 6> &out) {
  std::array<double, 6> fields = {msg.linear.x,  msg.linear.y,  msg.linear.z,
                                  msg.angular.x, msg.angular.y, msg.angular.z};
  for (const auto &f : fields) {
    if (!std::isfinite(f)) {
      out = ZERO_COMPONENTS;
      return false;
    }
  }
  out = fields;
  return true;
}

Twist twist_from_components(const std::array<double, 6> &components) {
  Twist result;
  result.linear.x = components[0];
  result.linear.y = components[1];
  result.linear.z = components[2];
  result.angular.x = components[3];
  result.angular.y = components[4];
  result.angular.z = components[5];
  return result;
}

bool components_are_quiescent(const std::array<double, 6> &components) {
  constexpr double zero_epsilon = 1.0e-6;
  for (const double component : components) {
    if (std::abs(component) > zero_epsilon) {
      return false;
    }
  }
  return true;
}

}  // namespace

class SafetyNode : public rclcpp::Node {
public:
  SafetyNode() : Node("safety_node") {
    // Declare parameters with same names and defaults as the Python version.
    declare_parameter("command_timeout_sec", 0.30);
    declare_parameter("scan_timeout_sec", 0.35);
    declare_parameter("odom_timeout_sec", 0.35);
    declare_parameter("raw_odom_timeout_sec", 0.25);
    declare_parameter("depth_points_timeout_sec", 1.0);
    declare_parameter("odom_throttle_interval_sec", 0.05);
    declare_parameter("minimum_voltage", 0.0);
    declare_parameter("voltage_timeout_sec", 1.0);
    declare_parameter("max_linear_speed_mps", 0.30);
    declare_parameter("publish_frequency_hz", 20.0);
    declare_parameter("require_scan", true);
    declare_parameter("require_odom", true);
    declare_parameter("require_raw_odom", true);
    declare_parameter("require_depth_points", false);
    declare_parameter("depth_points_topic", "/smartcar/depth/points");
    declare_parameter("wheelbase", 0.189);
    declare_parameter("max_steering_angle", 0.70);
    declare_parameter("ackermann_frame_id", "odom_combined");
    declare_parameter("allow_steering_calibration", false);
    declare_parameter("steering_calibration_max_hold_sec", 15.0);
    declare_parameter("emergency_stop_on_start", false);

    // Construct the guard.
    guard_ = smartcar_safety::SafetyGuard(
        get_parameter("command_timeout_sec").as_double(),
        get_parameter("scan_timeout_sec").as_double(),
        get_parameter("odom_timeout_sec").as_double(),
        get_parameter("raw_odom_timeout_sec").as_double(),
        get_parameter("minimum_voltage").as_double(),
        get_parameter("voltage_timeout_sec").as_double(),
        get_parameter("max_linear_speed_mps").as_double(),
        get_parameter("require_scan").as_bool(),
        get_parameter("require_odom").as_bool(),
        get_parameter("require_raw_odom").as_bool(),
        get_parameter("depth_points_timeout_sec").as_double(),
        get_parameter("require_depth_points").as_bool());

    if (get_parameter("emergency_stop_on_start").as_bool()) {
      guard_.set_emergency_stop(true);
    }

    double frequency_hz = get_parameter("publish_frequency_hz").as_double();
    if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
      throw std::invalid_argument(
          "publish_frequency_hz must be positive finite");
    }

    wheelbase_ = get_parameter("wheelbase").as_double();
    max_steering_angle_ = get_parameter("max_steering_angle").as_double();
    ackermann_frame_id_ = get_parameter("ackermann_frame_id").as_string();
    odom_throttle_interval_ =
        std::max(0.0, get_parameter("odom_throttle_interval_sec").as_double());

    zero_command_ = twist_from_components(ZERO_COMPONENTS);

    // QoS profiles matching the Python version.
    auto latest_reliable = rclcpp::QoS(1).reliable().keep_last(1);
    auto latest_sensor = rclcpp::QoS(1).best_effort().keep_last(1);
    auto status_qos = rclcpp::QoS(1)
                          .reliable()
                          .keep_last(1)
                          .transient_local();

    // Publishers.
    safe_pub_ = create_publisher<Twist>("/cmd_vel_safe", 10);
    ackermann_pub_ =
        create_publisher<AckermannDriveStamped>("/ackermann_cmd", 10);
    status_pub_ =
        create_publisher<String>("/smartcar/safety/status", status_qos);

    // Subscriptions.
    // scan/odom/raw_odom callbacks discard message content — they only need
    // arrival timestamps (heartbeat-only, per commit cb55b54).  Unlike Python,
    // C++ deserialization is cheap (~100 ns vs ~50 us), so the raw=True
    // optimization is unnecessary here.  rclcpp::SerializedMessage also has
    // known subscriber dispatch issues on Humble, so plain typed subscriptions
    // are safer and fast enough.  Measured: safety_node_cpp at 6.4 % CPU
    // (Python equivalent: 33.3 %).
    cmd_sub_ = create_subscription<Twist>(
        "/cmd_vel", latest_reliable,
        [this](Twist::SharedPtr msg) { on_command(msg); });
    scan_sub_ = create_subscription<LaserScan>(
        "/scan", latest_sensor,
        [this](LaserScan::SharedPtr /*msg*/) { on_scan(); });
    odom_sub_ = create_subscription<Odometry>(
        "/odom_combined", latest_reliable,
        [this](Odometry::SharedPtr /*msg*/) { on_odom(); });
    raw_odom_sub_ = create_subscription<Odometry>(
        "/odom", latest_reliable,
        [this](Odometry::SharedPtr /*msg*/) { on_raw_odom(); });
    if (get_parameter("require_depth_points").as_bool()) {
      const auto depth_points_topic =
          get_parameter("depth_points_topic").as_string();
      if (depth_points_topic.empty()) {
        throw std::invalid_argument(
            "depth_points_topic must be non-empty when required");
      }
      depth_points_sub_ = create_subscription<PointCloud2>(
          depth_points_topic, latest_sensor,
          [this](PointCloud2::SharedPtr /*msg*/) { on_depth_points(); });
    }
    voltage_sub_ = create_subscription<Float32>(
        "/PowerVoltage", latest_reliable,
        [this](Float32::SharedPtr msg) { on_voltage(msg); });

    // Service.
    emergency_stop_srv_ = create_service<SetBool>(
        "/smartcar/safety/emergency_stop",
        [this](const SetBool::Request::SharedPtr request,
               SetBool::Response::SharedPtr response) {
          on_emergency_stop(request, response);
        });
    steering_calibration_srv_ = create_service<HoldSteeringCalibration>(
        "/smartcar/safety/steering_calibration_hold",
        [this](const HoldSteeringCalibration::Request::SharedPtr request,
               HoldSteeringCalibration::Response::SharedPtr response) {
          on_steering_calibration_hold(request, response);
        });

    // Timer.
    auto period = std::chrono::duration<double>(1.0 / frequency_hz);
    timer_ = create_wall_timer(period, [this]() { on_timer(); });

    // Publish initial status.
    double now = now_sec();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto result = guard_.evaluate(now);
      publish_status_if_changed(result);
    }
  }

private:
  static double now_sec() {
    static const auto epoch = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - epoch)
        .count();
  }

  void on_command(const Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    double now = now_sec();
    std::array<double, 6> components;
    if (!sanitize_twist(*msg, components)) {
      steering_calibration_.reset();
      last_command_components_.reset();
      last_command_message_.reset();
      guard_.mark_command_invalid();
      publish_zero_command();
    } else if (!guard_.mark_command(now, components[0])) {
      steering_calibration_.reset();
      last_command_components_.reset();
      last_command_message_.reset();
      publish_zero_command();
    } else {
      if (steering_calibration_.has_value() &&
          !components_are_quiescent(components)) {
        steering_calibration_.reset();
      }
      last_command_components_ = components;
      last_command_message_ = twist_from_components(components);
    }
    auto result = guard_.evaluate(now);
    publish_status_if_changed(result);
  }

  void on_scan() {
    std::lock_guard<std::mutex> lock(mutex_);
    double now = now_sec();
    guard_.mark_scan(now);
  }

  void on_odom() {
    std::lock_guard<std::mutex> lock(mutex_);
    double now = now_sec();
    if (last_odom_processed_at_.has_value() &&
        now - last_odom_processed_at_.value() < odom_throttle_interval_) {
      return;
    }
    last_odom_processed_at_ = now;
    guard_.mark_odom(now);
  }

  void on_raw_odom() {
    std::lock_guard<std::mutex> lock(mutex_);
    double now = now_sec();
    if (last_raw_odom_processed_at_.has_value() &&
        now - last_raw_odom_processed_at_.value() < odom_throttle_interval_) {
      return;
    }
    last_raw_odom_processed_at_ = now;
    guard_.mark_raw_odom(now);
  }

  void on_depth_points() {
    std::lock_guard<std::mutex> lock(mutex_);
    guard_.mark_depth_points(now_sec());
  }

  void on_voltage(const Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    double now = now_sec();
    guard_.mark_voltage(msg->data, now);
    auto result = guard_.evaluate(now);
    publish_status_if_changed(result);
  }

  void on_emergency_stop(const SetBool::Request::SharedPtr request,
                         SetBool::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->data) {
      steering_calibration_.reset();
    } else {
      guard_.clear_command_speed_limit_fault();
    }
    guard_.set_emergency_stop(request->data);
    double now = now_sec();
    auto result = guard_.evaluate(now);
    publish_status_if_changed(result);
    response->success = true;
    response->message = request->data
                            ? "emergency stop latched"
                            : "emergency stop and speed-limit latch cleared";
  }

  void on_steering_calibration_hold(
      const HoldSteeringCalibration::Request::SharedPtr request,
      HoldSteeringCalibration::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = now_sec();
    const double requested_angle = request->steering_angle;
    const double requested_duration = request->duration_sec;

    if (requested_duration == 0.0) {
      steering_calibration_.reset();
      response->success = true;
      response->status = "steering_calibration_cancelled";
      return;
    }

    if (!get_parameter("allow_steering_calibration").as_bool()) {
      response->success = false;
      response->status = "steering_calibration_disabled";
      return;
    }

    const double max_hold =
        get_parameter("steering_calibration_max_hold_sec").as_double();
    if (!std::isfinite(requested_angle) || !std::isfinite(requested_duration) ||
        !std::isfinite(max_hold) || max_hold <= 0.0 ||
        requested_duration < 0.0 || requested_duration > max_hold ||
        std::abs(requested_angle) > max_steering_angle_) {
      response->success = false;
      response->status = "steering_calibration_request_invalid";
      return;
    }

    const auto verdict = guard_.evaluate(now);
    if (!verdict.allowed) {
      response->success = false;
      response->status = "steering_calibration_safety_blocked:" +
                         verdict.reason;
      return;
    }
    if (!last_command_components_.has_value() ||
        !components_are_quiescent(last_command_components_.value())) {
      response->success = false;
      response->status = "steering_calibration_command_not_quiescent";
      return;
    }

    steering_calibration_ = SteeringCalibration{
        requested_angle, now + requested_duration};
    response->success = true;
    response->status = "steering_calibration_active";
  }

  void on_timer() {
    Twist command;
    smartcar_safety::SafetyVerdict result;
    std::optional<double> steering_override;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double now = now_sec();
      result = guard_.evaluate(now);
      if (result.allowed && last_command_message_.has_value()) {
        command = last_command_message_.value();
      } else {
        command = zero_command_;
      }
      if (steering_calibration_.has_value()) {
        const bool command_quiescent = last_command_components_.has_value() &&
            components_are_quiescent(last_command_components_.value());
        if (!result.allowed || !command_quiescent ||
            now >= steering_calibration_->expires_at) {
          steering_calibration_.reset();
        } else {
          steering_override = steering_calibration_->angle;
        }
      }
    }
    // Publish outside the lock — ROS 2 publish may block on DDS transport.
    safe_pub_->publish(command);
    publish_ackermann(command, steering_override);
    publish_status_if_changed(result);
  }

  void publish_zero_command() {
    safe_pub_->publish(zero_command_);
    publish_ackermann(zero_command_);
  }

  void publish_ackermann(const Twist &twist,
                         const std::optional<double> steering_override =
                             std::nullopt) {
    double speed = twist.linear.x;
    double angular = twist.angular.z;
    double steering;
    if (steering_override.has_value()) {
      steering = steering_override.value();
    } else if (std::abs(speed) < 0.001) {
      steering = 0.0;
    } else {
      steering = std::atan(wheelbase_ * angular / speed);
      if (steering > max_steering_angle_) {
        steering = max_steering_angle_;
      } else if (steering < -max_steering_angle_) {
        steering = -max_steering_angle_;
      }
    }
    AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = ackermann_frame_id_;
    msg.drive.steering_angle = steering;
    msg.drive.speed = speed;
    ackermann_pub_->publish(msg);
  }

  struct SteeringCalibration {
    double angle;
    double expires_at;
  };

  void publish_status_if_changed(const smartcar_safety::SafetyVerdict &result) {
    const std::string &reason = result.reason;
    if (last_status_reason_.has_value() &&
        last_status_reason_.value() == reason) {
      return;
    }
    String msg;
    msg.data = reason;
    status_pub_->publish(msg);
    last_status_reason_ = reason;
  }

  smartcar_safety::SafetyGuard guard_{0.30, 0.35, 0.35, 0.25, 0.0, 1.0,
                                       0.30, true, true, true};
  double wheelbase_;
  double max_steering_angle_;
  std::string ackermann_frame_id_;
  double odom_throttle_interval_;
  Twist zero_command_;

  rclcpp::Publisher<Twist>::SharedPtr safe_pub_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::Publisher<String>::SharedPtr status_pub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr depth_points_sub_;
  rclcpp::Subscription<Float32>::SharedPtr voltage_sub_;
  rclcpp::Service<SetBool>::SharedPtr emergency_stop_srv_;
  rclcpp::Service<HoldSteeringCalibration>::SharedPtr steering_calibration_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::optional<std::array<double, 6>> last_command_components_;
  std::optional<Twist> last_command_message_;
  std::optional<std::string> last_status_reason_;
  std::optional<double> last_odom_processed_at_;
  std::optional<double> last_raw_odom_processed_at_;
  std::optional<SteeringCalibration> steering_calibration_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
