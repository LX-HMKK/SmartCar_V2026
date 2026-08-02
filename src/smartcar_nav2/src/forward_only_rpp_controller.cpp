#include "smartcar_nav2/forward_only_rpp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"

namespace smartcar_nav2
{

void ForwardOnlyRPPController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::configure(
    parent, std::move(name), std::move(tf), std::move(costmap_ros));
  readSafetyParameters();
}

void ForwardOnlyRPPController::readSafetyParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ForwardOnlyRPPController parent node expired");
  }

  double desired_linear_velocity = 0.0;
  double max_angular_velocity = 0.0;
  double min_turning_radius = 0.0;
  double forward_path_max_cross_track_error = 0.0;
  std::string forward_path_lateral_profile;
  ForwardPathLateralProfileStart forward_path_lateral_profile_start;
  double forward_terminal_lookahead_m = 0.0;
  double forward_terminal_activation_distance_m = 0.0;
  bool forward_path_use_curvature_tracking = false;
  double forward_path_heading_gain = 0.0;
  double forward_path_cross_track_gain = 0.0;
  double forward_path_collision_projection_m = 0.0;
  double forward_path_tight_turn_speed_mps = 0.0;
  double forward_path_tight_turn_radius_m = 0.0;
  double forward_path_tight_turn_preview_m = 0.0;
  bool allow_reversing = true;
  bool use_rotate_to_heading = true;
  const std::string prefix = plugin_name_ + ".";
  // RPP does not know the wrapper's final-command parameters, so declare
  // them here before reading their YAML overrides. Unsafe defaults preserve
  // fail-closed startup when an override is absent or misspelled.
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_max_angular_velocity", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_min_turning_radius", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "allow_reversing", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "use_rotate_to_heading", rclcpp::ParameterValue(true));
  // Zero disables only the unvalidated path-deviation threshold used by the
  // physical vehicle's base config. Current-footprint collision checking is
  // still mandatory for every ForwardOnlyRPPController instance.
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_max_cross_track_error", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile",
    rclcpp::ParameterValue(std::string(kForwardPathLateralProfileSymmetric)));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile_frame_id", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile_start_x_m", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile_start_y_m", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile_start_yaw_rad", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile_start_position_tolerance_m",
    rclcpp::ParameterValue(0.001));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_lateral_profile_start_yaw_tolerance_rad",
    rclcpp::ParameterValue(0.001));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_terminal_lookahead_m", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_terminal_activation_distance_m", rclcpp::ParameterValue(0.0));
  // The exact-curvature tracker is opt-in and only enabled by the Gazebo
  // overlay. Defaults deliberately make an accidental enable fail closed.
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_use_curvature_tracking", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_heading_gain", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_cross_track_gain", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_collision_projection_m", rclcpp::ParameterValue(0.0));
  // A zero-valued triple disables the optional pre-emptive tight-turn speed
  // cap. It is deliberately only enabled by the local Gazebo overlay.
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_tight_turn_speed_mps", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_tight_turn_radius_m", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "forward_path_tight_turn_preview_m", rclcpp::ParameterValue(0.0));
  if (!node->get_parameter(prefix + "desired_linear_vel", desired_linear_velocity) ||
    !node->get_parameter(
      prefix + "forward_max_angular_velocity", max_angular_velocity) ||
    !node->get_parameter(
      prefix + "forward_min_turning_radius", min_turning_radius) ||
    !node->get_parameter(prefix + "allow_reversing", allow_reversing) ||
    !node->get_parameter(prefix + "use_rotate_to_heading", use_rotate_to_heading) ||
    !node->get_parameter(
      prefix + "forward_path_max_cross_track_error", forward_path_max_cross_track_error) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile", forward_path_lateral_profile) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile_frame_id",
      forward_path_lateral_profile_start.frame_id) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile_start_x_m",
      forward_path_lateral_profile_start.x_m) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile_start_y_m",
      forward_path_lateral_profile_start.y_m) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile_start_yaw_rad",
      forward_path_lateral_profile_start.yaw_rad) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile_start_position_tolerance_m",
      forward_path_lateral_profile_start.position_tolerance_m) ||
    !node->get_parameter(
      prefix + "forward_path_lateral_profile_start_yaw_tolerance_rad",
      forward_path_lateral_profile_start.yaw_tolerance_rad) ||
    !node->get_parameter(
      prefix + "forward_terminal_lookahead_m", forward_terminal_lookahead_m) ||
    !node->get_parameter(
      prefix + "forward_terminal_activation_distance_m",
      forward_terminal_activation_distance_m) ||
    !node->get_parameter(
      prefix + "forward_path_use_curvature_tracking",
      forward_path_use_curvature_tracking) ||
    !node->get_parameter(
      prefix + "forward_path_heading_gain", forward_path_heading_gain) ||
    !node->get_parameter(
      prefix + "forward_path_cross_track_gain", forward_path_cross_track_gain) ||
    !node->get_parameter(
      prefix + "forward_path_collision_projection_m",
      forward_path_collision_projection_m) ||
    !node->get_parameter(
      prefix + "forward_path_tight_turn_speed_mps",
      forward_path_tight_turn_speed_mps) ||
    !node->get_parameter(
      prefix + "forward_path_tight_turn_radius_m",
      forward_path_tight_turn_radius_m) ||
    !node->get_parameter(
      prefix + "forward_path_tight_turn_preview_m",
      forward_path_tight_turn_preview_m))
  {
    throw std::runtime_error(
            "ForwardOnlyRPPController could not read forward Ackermann parameters");
  }

  const bool can_track_planner_radius =
    max_angular_velocity + 1.0e-9 >= desired_linear_velocity / min_turning_radius;
  const bool tight_turn_speed_cap_enabled =
    forward_path_tight_turn_speed_mps > 0.0 || forward_path_tight_turn_radius_m > 0.0 ||
    forward_path_tight_turn_preview_m > 0.0;
  if (!finiteForwardValue(desired_linear_velocity) ||
    !finiteForwardValue(max_angular_velocity) ||
    !finiteForwardValue(min_turning_radius) || desired_linear_velocity <= 0.0 ||
    max_angular_velocity <= 0.0 || min_turning_radius <= 0.0 ||
    allow_reversing || use_rotate_to_heading || !can_track_planner_radius ||
    !finiteForwardValue(forward_path_max_cross_track_error) ||
    forward_path_max_cross_track_error < 0.0 ||
    !forwardPathLateralProfileStartValid(
      forward_path_lateral_profile, forward_path_lateral_profile_start) ||
    !forwardPathLateralProfileConfigurationValid(
      forward_path_lateral_profile, forward_path_max_cross_track_error, min_turning_radius) ||
    !finiteForwardValue(forward_terminal_lookahead_m) ||
    !finiteForwardValue(forward_terminal_activation_distance_m) ||
    forward_terminal_lookahead_m < 0.0 ||
    forward_terminal_activation_distance_m < 0.0 ||
    ((forward_terminal_lookahead_m > 0.0 || forward_terminal_activation_distance_m > 0.0) &&
    (forward_terminal_lookahead_m <= 0.0 ||
    forward_terminal_activation_distance_m < forward_terminal_lookahead_m)) ||
    (forward_path_use_curvature_tracking &&
    (!finiteForwardValue(forward_path_heading_gain) ||
    !finiteForwardValue(forward_path_cross_track_gain) ||
    !finiteForwardValue(forward_path_collision_projection_m) ||
    forward_path_heading_gain < 0.0 || forward_path_cross_track_gain < 0.0 ||
    forward_path_collision_projection_m <= 0.0)) ||
    !finiteForwardValue(forward_path_tight_turn_speed_mps) ||
    !finiteForwardValue(forward_path_tight_turn_radius_m) ||
    !finiteForwardValue(forward_path_tight_turn_preview_m) ||
    (tight_turn_speed_cap_enabled &&
    (!forward_path_use_curvature_tracking || forward_path_tight_turn_speed_mps <= 0.0 ||
    forward_path_tight_turn_speed_mps > desired_linear_velocity ||
    forward_path_tight_turn_radius_m < min_turning_radius ||
    forward_path_tight_turn_preview_m < 0.0)))
  {
    throw std::runtime_error(
            "ForwardOnlyRPPController requires forward-only non-rotating "
            "Ackermann limits compatible with the planner radius");
  }

  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    configured_limits_ = ForwardCommandLimits{
      desired_linear_velocity, max_angular_velocity, min_turning_radius};
    active_limits_ = configured_limits_;
    configured_ = true;
  }
  {
    std::lock_guard<std::mutex> lock(path_tracking_mutex_);
    forward_path_max_cross_track_error_ = forward_path_max_cross_track_error;
    forward_path_lateral_profile_ = forward_path_lateral_profile;
    forward_path_lateral_profile_start_ = forward_path_lateral_profile_start;
    confirmed_lateral_profile_ = kForwardPathLateralProfileSymmetric;
    confirmed_lateral_profile_invalid_ = false;
    forward_terminal_lookahead_m_ = forward_terminal_lookahead_m;
    forward_terminal_activation_distance_m_ = forward_terminal_activation_distance_m;
    forward_path_use_curvature_tracking_ = forward_path_use_curvature_tracking;
    forward_path_heading_gain_ = forward_path_heading_gain;
    forward_path_cross_track_gain_ = forward_path_cross_track_gain;
    forward_path_collision_projection_m_ = forward_path_collision_projection_m;
    forward_path_tight_turn_speed_mps_ = forward_path_tight_turn_speed_mps;
    forward_path_tight_turn_radius_m_ = forward_path_tight_turn_radius_m;
    forward_path_tight_turn_preview_m_ = forward_path_tight_turn_preview_m;
  }
}

void ForwardOnlyRPPController::setPlan(const nav_msgs::msg::Path & path)
{
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::setPlan(path);
  std::string requested_lateral_profile;
  ForwardPathLateralProfileStart lateral_profile_start;
  {
    std::lock_guard<std::mutex> lock(path_tracking_mutex_);
    requested_lateral_profile = forward_path_lateral_profile_;
    lateral_profile_start = forward_path_lateral_profile_start_;
  }
  std::string confirmed_lateral_profile = kForwardPathLateralProfileSymmetric;
  bool lateral_profile_invalid = false;
  if (requested_lateral_profile == kForwardPathLateralProfilePDepartureSouthV1) {
    const auto match = forwardPathLateralProfileMatchesPlan(
      requested_lateral_profile, path, lateral_profile_start);
    if (match == ForwardPathLateralProfilePathMatch::kMatches) {
      confirmed_lateral_profile = requested_lateral_profile;
    } else if (match == ForwardPathLateralProfilePathMatch::kInvalid) {
      lateral_profile_invalid = true;
    }
  }
  std::lock_guard<std::mutex> lock(path_tracking_mutex_);
  confirmed_plan_ = path;
  confirmed_lateral_profile_ = confirmed_lateral_profile;
  confirmed_lateral_profile_invalid_ = lateral_profile_invalid;
}

geometry_msgs::msg::TwistStamped
ForwardOnlyRPPController::computeCurvatureTrackingCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker,
  const nav_msgs::msg::Path & confirmed_plan,
  const ForwardPathTrackingProjection & projection)
{
  static_cast<void>(goal_checker);

  double heading_gain = 0.0;
  double cross_track_gain = 0.0;
  double collision_projection_m = 0.0;
  double tight_turn_speed_mps = 0.0;
  double tight_turn_radius_m = 0.0;
  double tight_turn_preview_m = 0.0;
  {
    std::lock_guard<std::mutex> lock(path_tracking_mutex_);
    heading_gain = forward_path_heading_gain_;
    cross_track_gain = forward_path_cross_track_gain_;
    collision_projection_m = forward_path_collision_projection_m_;
    tight_turn_speed_mps = forward_path_tight_turn_speed_mps_;
    tight_turn_radius_m = forward_path_tight_turn_radius_m_;
    tight_turn_preview_m = forward_path_tight_turn_preview_m_;
  }
  const auto path_curvature = forwardPathTrackingLocalCurvature(confirmed_plan, projection);
  if (!path_curvature.valid) {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController has no valid local path curvature: " +
            path_curvature.reason);
  }

  ForwardCommandLimits limits;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    limits = active_limits_;
  }
  if (!validForwardCommandLimits(limits) ||
    !finiteForwardValue(collision_projection_m) || collision_projection_m <= 0.0)
  {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController curvature tracking has invalid limits");
  }
  const double maximum_curvature = 1.0 / limits.min_turning_radius;
  if (!finiteForwardValue(path_curvature.curvature_m_inv) ||
    std::abs(path_curvature.curvature_m_inv) > maximum_curvature + 0.25)
  {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController rejected path curvature outside the Ackermann envelope");
  }

  // The centreline curvature comes from the already full-footprint-swept
  // plan. Feedback is intentionally mild: it closes model error without
  // converting a safe 0.50 m departure arc into an unswept 0.33 m shortcut.
  const double heading_to_path = -projection.path_heading_error_rad;
  const double requested_curvature = std::clamp(
    path_curvature.curvature_m_inv + heading_gain * heading_to_path -
    cross_track_gain * projection.signed_cross_track_m,
    -maximum_curvature, maximum_curvature);

  std::lock_guard<std::mutex> reinit_lock(mutex_);
  if (!costmap_ros_ || !costmap_) {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController curvature tracking has no costmap");
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *(costmap_->getMutex()));
  const auto transformed_plan = transformGlobalPlan(robot_pose);

  double linear_velocity = desired_linear_vel_;
  if (tight_turn_speed_mps > 0.0 &&
    forwardPathTrackingTightTurnAhead(
      confirmed_plan, projection, tight_turn_radius_m, tight_turn_preview_m))
  {
    // At 0.15 m/s Fortress has not reached the required steering angle by a
    // 0.22 m arc's first samples. Preserve the accepted curvature but leave
    // the simulated steering actuator time to attain it before body drift
    // reaches the independent cross-track guard.
    linear_velocity = std::min(linear_velocity, tight_turn_speed_mps);
  }
  double direction_sign = 1.0;
  applyConstraints(
    requested_curvature, robot_speed,
    costAtPose(robot_pose.pose.position.x, robot_pose.pose.position.y),
    transformed_plan, linear_velocity, direction_sign);
  const auto filtered = enforceForwardCommandLimits(
    ForwardCommand{linear_velocity, 0.0, 0.0, 0.0, 0.0,
      linear_velocity * requested_curvature},
    limits);
  if (filtered.status != ForwardCommandFilterStatus::kAccepted) {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController curvature tracking rejected command: " +
            std::string(forwardCommandFilterStatusName(filtered.status)));
  }

  const double projection_distance = std::min(
    collision_projection_m, std::max(0.0, projection.remaining_path_m));
  if (use_collision_detection_ && isCollisionImminent(
      robot_pose, filtered.command.linear_x, filtered.command.angular_z,
      projection_distance))
  {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController curvature tracking detected collision ahead");
  }

  geometry_msgs::msg::TwistStamped command;
  command.header = robot_pose.header;
  command.twist.linear.x = filtered.command.linear_x;
  command.twist.linear.y = filtered.command.linear_y;
  command.twist.linear.z = filtered.command.linear_z;
  command.twist.angular.x = filtered.command.angular_x;
  command.twist.angular.y = filtered.command.angular_y;
  command.twist.angular.z = filtered.command.angular_z;
  return command;
}

geometry_msgs::msg::TwistStamped ForwardOnlyRPPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  double robot_yaw = 0.0;
  if (!forwardPathTrackingPoseYaw(robot_pose, robot_yaw) || !costmap_ros_ ||
    robot_pose.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController rejected an invalid current robot pose");
  }

  nav_msgs::msg::Path confirmed_plan;
  double maximum_cross_track_error = 0.0;
  std::string confirmed_lateral_profile;
  bool confirmed_lateral_profile_invalid = false;
  double terminal_lookahead_m = 0.0;
  double terminal_activation_distance_m = 0.0;
  bool use_curvature_tracking = false;
  {
    std::lock_guard<std::mutex> lock(path_tracking_mutex_);
    confirmed_plan = confirmed_plan_;
    maximum_cross_track_error = forward_path_max_cross_track_error_;
    confirmed_lateral_profile = confirmed_lateral_profile_;
    confirmed_lateral_profile_invalid = confirmed_lateral_profile_invalid_;
    terminal_lookahead_m = forward_terminal_lookahead_m_;
    terminal_activation_distance_m = forward_terminal_activation_distance_m_;
    use_curvature_tracking = forward_path_use_curvature_tracking_;
  }
  const auto projection = projectForwardPathTrackingPose(confirmed_plan, robot_pose);
  if (confirmed_lateral_profile_invalid) {
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController rejected an invalid P-departure lateral-profile plan");
  }
  if (maximum_cross_track_error > 0.0) {
    const auto envelope = forwardPathLateralEnvelopeAtStation(
      confirmed_lateral_profile, projection.station_m, maximum_cross_track_error);
    const double left_limit = envelope.has_value() ?
      envelope->left_cross_track_error_m : 0.0;
    const double right_limit = envelope.has_value() ?
      envelope->right_cross_track_error_m : 0.0;
    const bool cross_track_exceeded = !projection.valid || !envelope.has_value() ||
      !finiteForwardValue(projection.signed_cross_track_m) ||
      projection.signed_cross_track_m > left_limit ||
      projection.signed_cross_track_m < -right_limit;
    if (cross_track_exceeded) {
      if (clock_) {
        RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 1000,
          "Forward-only RPP stopped before leaving accepted path: reason=%s profile=%s "
          "station=%.3f signed_cross_track=%.3f left_limit=%.3f right_limit=%.3f "
          "heading_error=%.3f segment=%zu",
          projection.reason.c_str(), confirmed_lateral_profile.c_str(), projection.station_m,
          projection.signed_cross_track_m, left_limit, right_limit, projection.path_heading_error_rad,
          projection.segment_index);
      } else {
        RCLCPP_ERROR(
          logger_,
          "Forward-only RPP stopped before leaving accepted path: %s profile=%s",
          projection.reason.c_str(), confirmed_lateral_profile.c_str());
      }
      // ControllerServer turns PlannerException into a zero command and the
      // existing BT clear/replan branch owns recovery. Returning a locally
      // constructed zero Twist here would hide the failure from that branch.
      throw nav2_core::PlannerException(
              "ForwardOnlyRPPController cross-track guard rejected current pose");
    }
  }

  // RPP's regular projection checks the commanded arc. Check the actual
  // current complete footprint as well so a late costmap update cannot leave
  // the controller free to continue a shortcut or a loop for one more cycle.
  if (inCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y, robot_yaw))
  {
    if (clock_) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 1000,
        "Forward-only RPP stopped because the current full footprint is in collision");
    } else {
      RCLCPP_ERROR(
        logger_, "Forward-only RPP stopped because the current full footprint is in collision");
    }
    throw nav2_core::PlannerException(
            "ForwardOnlyRPPController current footprint is in collision");
  }

  if (use_curvature_tracking) {
    return computeCurvatureTrackingCommand(
      robot_pose, robot_speed, goal_checker, confirmed_plan, projection);
  }

  const bool terminal_lookahead_active = forwardPathTrackingTerminalLookaheadActive(
    confirmed_plan, projection, terminal_lookahead_m, terminal_activation_distance_m);
  geometry_msgs::msg::TwistStamped command;
  if (!terminal_lookahead_active) {
    command = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::
      computeVelocityCommands(robot_pose, robot_speed, goal_checker);
  } else {
    // RPP's collision projection must use the same short carrot as the final
    // path tracker. A fixed long carrot crosses the locked terminal arc,
    // makes the vehicle turn after the goal, and turns the cross-track guard
    // into a late stop. Restore the configured values immediately so the P
    // departure and ordinary avoidance paths retain their tested envelope.
    const double configured_lookahead = lookahead_dist_;
    const double configured_min_lookahead = min_lookahead_dist_;
    const double configured_max_lookahead = max_lookahead_dist_;
    const bool configured_velocity_scaled = use_velocity_scaled_lookahead_dist_;
    lookahead_dist_ = terminal_lookahead_m;
    min_lookahead_dist_ = terminal_lookahead_m;
    max_lookahead_dist_ = terminal_lookahead_m;
    use_velocity_scaled_lookahead_dist_ = false;
    try {
      command = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::
        computeVelocityCommands(robot_pose, robot_speed, goal_checker);
    } catch (...) {
      lookahead_dist_ = configured_lookahead;
      min_lookahead_dist_ = configured_min_lookahead;
      max_lookahead_dist_ = configured_max_lookahead;
      use_velocity_scaled_lookahead_dist_ = configured_velocity_scaled;
      throw;
    }
    lookahead_dist_ = configured_lookahead;
    min_lookahead_dist_ = configured_min_lookahead;
    max_lookahead_dist_ = configured_max_lookahead;
    use_velocity_scaled_lookahead_dist_ = configured_velocity_scaled;
  }

  ForwardCommandLimits limits;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    limits = active_limits_;
  }

  const ForwardCommand input{
    command.twist.linear.x,
    command.twist.linear.y,
    command.twist.linear.z,
    command.twist.angular.x,
    command.twist.angular.y,
    command.twist.angular.z};
  const auto filtered = enforceForwardCommandLimits(input, limits);
  if (filtered.status != ForwardCommandFilterStatus::kAccepted) {
    if (clock_) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 1000,
        "Forward-only RPP rejected command: %s",
        forwardCommandFilterStatusName(filtered.status));
    } else {
      RCLCPP_ERROR(
        logger_, "Forward-only RPP rejected command: %s",
        forwardCommandFilterStatusName(filtered.status));
    }
  }

  command.twist.linear.x = filtered.command.linear_x;
  command.twist.linear.y = filtered.command.linear_y;
  command.twist.linear.z = filtered.command.linear_z;
  command.twist.angular.x = filtered.command.angular_x;
  command.twist.angular.y = filtered.command.angular_y;
  command.twist.angular.z = filtered.command.angular_z;
  return command;
}

void ForwardOnlyRPPController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  ForwardCommandLimits configured;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    if (!configured_) {
      RCLCPP_ERROR(logger_, "Forward-only RPP received speed limit before configure");
      return;
    }
    configured = configured_limits_;
  }

  const auto translation = translateForwardSpeedLimit(
    speed_limit, percentage, configured.vx_max);
  if (!translation.valid) {
    RCLCPP_ERROR(
      logger_, "Forward-only RPP ignored invalid speed limit %.6f", speed_limit);
    return;
  }

  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController::setSpeedLimit(
    translation.forwarded_speed_limit, translation.forwarded_percentage);

  std::lock_guard<std::mutex> lock(limits_mutex_);
  active_limits_.vx_max = configured_limits_.vx_max * translation.guard_scale;
  active_limits_.wz_max = configured_limits_.wz_max * translation.guard_scale;
  active_limits_.min_turning_radius = configured_limits_.min_turning_radius;
}

}  // namespace smartcar_nav2

PLUGINLIB_EXPORT_CLASS(smartcar_nav2::ForwardOnlyRPPController, nav2_core::Controller)
