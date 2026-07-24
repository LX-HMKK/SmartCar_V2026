#include "smartcar_safety/direction_guard.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "smartcar_interfaces/srv/activate_motion.hpp"
#include "smartcar_interfaces/srv/prepare_motion.hpp"
#include "smartcar_interfaces/srv/renew_motion.hpp"
#include "smartcar_interfaces/srv/stop_motion.hpp"
#include "std_msgs/msg/string.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace {

using ActivateMotion = smartcar_interfaces::srv::ActivateMotion;
using PrepareMotion = smartcar_interfaces::srv::PrepareMotion;
using RenewMotion = smartcar_interfaces::srv::RenewMotion;
using StopMotion = smartcar_interfaces::srv::StopMotion;
using smartcar_safety::ActionUuid;
using smartcar_safety::DirectionGuard;
using smartcar_safety::DirectionGuardConfig;
using smartcar_safety::LeaseIdentity;
using smartcar_safety::MotionDirection;
using smartcar_safety::TwistComponents;

std::uint64_t make_boot_epoch() {
  std::random_device random;
  const auto steady_ticks = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::uint64_t value = (static_cast<std::uint64_t>(random()) << 32U) ^
                        static_cast<std::uint64_t>(random()) ^ steady_ticks;
  return value == 0 ? 1 : value;
}

ActionUuid to_action_uuid(const unique_identifier_msgs::msg::UUID &message) {
  ActionUuid result{};
  std::copy(message.uuid.begin(), message.uuid.end(), result.begin());
  return result;
}

LeaseIdentity identity_from(std::uint64_t boot_epoch, std::uint64_t lease_id,
                            std::uint64_t generation,
                            const unique_identifier_msgs::msg::UUID &uuid) {
  return LeaseIdentity{boot_epoch, lease_id, generation, to_action_uuid(uuid)};
}

TwistComponents components_from(const geometry_msgs::msg::Twist &message) {
  return {message.linear.x,  message.linear.y,  message.linear.z,
          message.angular.x, message.angular.y, message.angular.z};
}

geometry_msgs::msg::Twist twist_from(const TwistComponents &components) {
  geometry_msgs::msg::Twist message;
  message.linear.x = components[0];
  message.linear.y = components[1];
  message.linear.z = components[2];
  message.angular.x = components[3];
  message.angular.y = components[4];
  message.angular.z = components[5];
  return message;
}

bool odom_twist_is_finite(const geometry_msgs::msg::Twist &message) {
  const auto components = components_from(message);
  return std::all_of(components.begin(), components.end(),
                     [](double value) { return std::isfinite(value); });
}

class DirectionGuardNode : public rclcpp::Node {
public:
  DirectionGuardNode() : Node("direction_guard") {
    declare_parameter("candidate_timeout_sec", 0.15);
    declare_parameter("permit_timeout_sec", 0.25);
    declare_parameter("prepare_timeout_sec", 5.0);
    declare_parameter("raw_odom_timeout_sec", 0.25);
    declare_parameter("stop_settle_sec", 0.25);
    declare_parameter("stop_linear_speed_threshold", 0.01);
    declare_parameter("stop_angular_speed_threshold", 0.05);
    declare_parameter("zero_epsilon", 1.0e-6);
    declare_parameter("direction_epsilon", 1.0e-4);
    declare_parameter("publish_frequency_hz", 20.0);

    DirectionGuardConfig config;
    config.candidate_timeout_sec =
        get_parameter("candidate_timeout_sec").as_double();
    config.permit_timeout_sec =
        get_parameter("permit_timeout_sec").as_double();
    config.prepare_timeout_sec =
        get_parameter("prepare_timeout_sec").as_double();
    config.raw_odom_timeout_sec =
        get_parameter("raw_odom_timeout_sec").as_double();
    config.stop_settle_sec = get_parameter("stop_settle_sec").as_double();
    config.stop_linear_speed_threshold =
        get_parameter("stop_linear_speed_threshold").as_double();
    config.stop_angular_speed_threshold =
        get_parameter("stop_angular_speed_threshold").as_double();
    config.zero_epsilon = get_parameter("zero_epsilon").as_double();
    config.direction_epsilon = get_parameter("direction_epsilon").as_double();
    guard_ = std::make_unique<DirectionGuard>(config, make_boot_epoch());
    guard_->stop(now_sec());

    const double publish_frequency =
        get_parameter("publish_frequency_hz").as_double();
    if (!std::isfinite(publish_frequency) || publish_frequency <= 0.0) {
      throw std::invalid_argument("publish_frequency_hz must be positive finite");
    }

    const auto command_qos = rclcpp::QoS(1).reliable().keep_last(1);
    const auto status_qos =
        rclcpp::QoS(1).reliable().keep_last(1).transient_local();
    command_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", command_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/smartcar/direction_guard/status", status_qos);
    candidate_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_candidate", command_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr message) {
          on_candidate(*message);
        });
    raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", command_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr message) {
          on_raw_odom(*message);
        });

    prepare_service_ = create_service<PrepareMotion>(
        "/smartcar/direction_guard/prepare",
        [this](const PrepareMotion::Request::SharedPtr request,
               PrepareMotion::Response::SharedPtr response) {
          on_prepare(request, response);
        });
    activate_service_ = create_service<ActivateMotion>(
        "/smartcar/direction_guard/activate",
        [this](const ActivateMotion::Request::SharedPtr request,
               ActivateMotion::Response::SharedPtr response) {
          on_activate(request, response);
        });
    renew_service_ = create_service<RenewMotion>(
        "/smartcar/direction_guard/renew",
        [this](const RenewMotion::Request::SharedPtr request,
               RenewMotion::Response::SharedPtr response) {
          on_renew(request, response);
        });
    stop_service_ = create_service<StopMotion>(
        "/smartcar/direction_guard/stop",
        [this](const StopMotion::Request::SharedPtr request,
               StopMotion::Response::SharedPtr response) {
          on_stop(request, response);
        });

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency);
    timer_ = create_wall_timer(period, [this]() { on_timer(); });
    publish_command(DirectionGuard::zero_command());
    publish_status_if_changed("stopped");
  }

private:
  static double now_sec() {
    static const auto epoch = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        epoch)
        .count();
  }

  void on_candidate(const geometry_msgs::msg::Twist &message) {
    TwistComponents command;
    std::string status;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      command = guard_->on_candidate(components_from(message), now_sec());
      status = guard_->status();
    }
    publish_command(command);
    publish_status_if_changed(status);
  }

  void on_raw_odom(const nav_msgs::msg::Odometry &message) {
    std::string status;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      guard_->on_raw_odom(components_from(message.twist.twist),
                          odom_twist_is_finite(message.twist.twist), now_sec());
      status = guard_->status();
    }
    publish_status_if_changed(status);
  }

  void on_prepare(const PrepareMotion::Request::SharedPtr request,
                  PrepareMotion::Response::SharedPtr response) {
    smartcar_safety::PrepareMotionResult result;
    std::string status;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      result = guard_->prepare(static_cast<MotionDirection>(request->direction),
                               request->generation,
                               to_action_uuid(request->action_uuid), now_sec());
      status = guard_->status();
    }
    response->success = result.success;
    response->status = result.status;
    response->boot_epoch = result.boot_epoch;
    response->lease_id = result.lease_id;
    publish_command(DirectionGuard::zero_command());
    publish_status_if_changed(status);
  }

  void on_activate(const ActivateMotion::Request::SharedPtr request,
                   ActivateMotion::Response::SharedPtr response) {
    TwistComponents command;
    smartcar_safety::GuardOperationResult result;
    std::string status;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      result = guard_->activate(
          identity_from(request->boot_epoch, request->lease_id,
                        request->generation, request->action_uuid),
          now_sec());
      command = guard_->evaluate(now_sec());
      status = guard_->status();
    }
    response->success = result.success;
    response->status = result.status;
    publish_command(command);
    publish_status_if_changed(status);
  }

  void on_renew(const RenewMotion::Request::SharedPtr request,
                RenewMotion::Response::SharedPtr response) {
    TwistComponents command;
    smartcar_safety::GuardOperationResult result;
    std::string status;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      result = guard_->renew(
          identity_from(request->boot_epoch, request->lease_id,
                        request->generation, request->action_uuid),
          now_sec());
      command = guard_->evaluate(now_sec());
      status = guard_->status();
    }
    response->success = result.success;
    response->status = result.status;
    publish_command(command);
    publish_status_if_changed(status);
  }

  void on_stop(const StopMotion::Request::SharedPtr /*request*/,
               StopMotion::Response::SharedPtr response) {
    smartcar_safety::GuardOperationResult result;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      result = guard_->stop(now_sec());
    }
    response->success = result.success;
    response->status = result.status;
    publish_command(DirectionGuard::zero_command());
    publish_status_if_changed(result.status);
  }

  void on_timer() {
    TwistComponents command;
    std::string status;
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      command = guard_->evaluate(now_sec());
      status = guard_->status();
    }
    publish_command(command);
    publish_status_if_changed(status);
  }

  void publish_command(const TwistComponents &components) {
    command_pub_->publish(twist_from(components));
  }

  void publish_status_if_changed(const std::string &status) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (last_status_.has_value() && last_status_.value() == status) {
      return;
    }
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    last_status_ = status;
  }

  std::unique_ptr<DirectionGuard> guard_;
  std::mutex guard_mutex_;
  std::mutex status_mutex_;
  std::optional<std::string> last_status_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr candidate_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Service<PrepareMotion>::SharedPtr prepare_service_;
  rclcpp::Service<ActivateMotion>::SharedPtr activate_service_;
  rclcpp::Service<RenewMotion>::SharedPtr renew_service_;
  rclcpp::Service<StopMotion>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionGuardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
