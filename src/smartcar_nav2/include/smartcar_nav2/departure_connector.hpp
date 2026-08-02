#ifndef SMARTCAR_NAV2__DEPARTURE_CONNECTOR_HPP_
#define SMARTCAR_NAV2__DEPARTURE_CONNECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace smartcar_nav2
{

// These connectors are internal planning geometry, not authored route points.
// They only provide a safe leftward departure from P when a centreline-only
// Hybrid-A* start would otherwise swing the padded rear footprint south of the
// field boundary.
constexpr double kDepartureConnectorPi = 3.14159265358979323846;

struct DepartureConnectorOptions
{
  double minimum_turning_radius_m{0.55};
  double radius_margin_m{0.08};
  double sample_spacing_m{0.025};
  double curvature_tolerance{0.20};
  double maximum_direction_error{0.35};
  double minimum_segment_length{1.0e-4};
};

struct DepartureConnector
{
  nav_msgs::msg::Path path;
  double radius_m{0.0};
  double high_right_turn_radius_m{0.0};
  double arc_angle_rad{0.0};
  double straight_length_m{0.0};
  double length_m{0.0};
  bool lattice_aligned{false};
};

struct DepartureConnectorValidationResult
{
  bool valid{false};
  std::string reason;
  double maximum_curvature{0.0};
  double length_m{0.0};
};

inline bool departureConnectorFinite(double value)
{
  return std::isfinite(value);
}

inline bool departureConnectorUnitQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!departureConnectorFinite(orientation.x) || !departureConnectorFinite(orientation.y) ||
    !departureConnectorFinite(orientation.z) || !departureConnectorFinite(orientation.w))
  {
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  return departureConnectorFinite(norm) && std::abs(norm - 1.0) <= 1.0e-3;
}

inline double departureConnectorYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double sine = 2.0 * (
    orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosine = 1.0 - 2.0 * (
    orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sine, cosine);
}

inline geometry_msgs::msg::Quaternion departureConnectorQuaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

inline double departureConnectorAngularDistance(double first, double second)
{
  return std::abs(std::remainder(second - first, 2.0 * kDepartureConnectorPi));
}

inline double departureConnectorPathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const double delta_x =
      path.poses[index].pose.position.x - path.poses[index - 1U].pose.position.x;
    const double delta_y =
      path.poses[index].pose.position.y - path.poses[index - 1U].pose.position.y;
    length += std::hypot(delta_x, delta_y);
  }
  return length;
}

inline bool departureConnectorOptionsValid(const DepartureConnectorOptions & options)
{
  const std::array<double, 6> values = {
    options.minimum_turning_radius_m,
    options.radius_margin_m,
    options.sample_spacing_m,
    options.curvature_tolerance,
    options.maximum_direction_error,
    options.minimum_segment_length,
  };
  if (!std::all_of(values.begin(), values.end(), departureConnectorFinite)) {
    return false;
  }
  return options.minimum_turning_radius_m > 0.0 &&
         options.minimum_turning_radius_m <= 5.0 &&
         options.radius_margin_m > 0.0 && options.radius_margin_m <= 1.0 &&
         options.sample_spacing_m > 0.0 && options.sample_spacing_m <= 0.10 &&
         options.curvature_tolerance >= 0.0 &&
         options.maximum_direction_error > 0.0 &&
         options.maximum_direction_error < kDepartureConnectorPi / 2.0 &&
         options.minimum_segment_length > 0.0;
}

inline bool departureConnectorRadiusWithinMaximum(
  const DepartureConnectorOptions & options,
  double maximum_active_radius_m)
{
  if (!departureConnectorOptionsValid(options) ||
    !departureConnectorFinite(maximum_active_radius_m) ||
    maximum_active_radius_m <= 0.0)
  {
    return false;
  }
  return options.minimum_turning_radius_m + options.radius_margin_m <=
         maximum_active_radius_m + 1.0e-9;
}

inline bool departureConnectorTerminalRadiusWithinEnvelope(
  const DepartureConnectorOptions & options,
  double terminal_radius_m,
  double maximum_active_radius_m)
{
  if (!departureConnectorOptionsValid(options) ||
    !departureConnectorFinite(terminal_radius_m) ||
    !departureConnectorFinite(maximum_active_radius_m))
  {
    return false;
  }
  return terminal_radius_m >= options.minimum_turning_radius_m - 1.0e-9 &&
         terminal_radius_m <= maximum_active_radius_m + 1.0e-9;
}

// The high P-departure right turn is tracked after northbound travel. It may
// be smoother than the global lower bound, but must remain inside the same
// simulator-only connector envelope.
inline bool departureConnectorHighRightTurnRadiusWithinEnvelope(
  const DepartureConnectorOptions & options,
  double high_right_turn_radius_m,
  double maximum_active_radius_m)
{
  if (!departureConnectorOptionsValid(options) ||
    !departureConnectorFinite(high_right_turn_radius_m) ||
    !departureConnectorFinite(maximum_active_radius_m))
  {
    return false;
  }
  return high_right_turn_radius_m >= options.minimum_turning_radius_m - 1.0e-9 &&
         high_right_turn_radius_m <= maximum_active_radius_m + 1.0e-9;
}

inline DepartureConnectorValidationResult validateForwardConnectorPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & expected_start,
  const DepartureConnectorOptions & options)
{
  DepartureConnectorValidationResult result;
  if (!departureConnectorOptionsValid(options)) {
    result.reason = "options_invalid";
    return result;
  }
  if (path.poses.size() < 3U) {
    result.reason = "path_too_short";
    return result;
  }
  if (path.header.frame_id.empty() || path.header.frame_id != expected_start.header.frame_id) {
    result.reason = "path_frame_mismatch";
    return result;
  }
  if (!departureConnectorFinite(expected_start.pose.position.x) ||
    !departureConnectorFinite(expected_start.pose.position.y) ||
    !departureConnectorUnitQuaternion(expected_start.pose.orientation))
  {
    result.reason = "expected_start_invalid";
    return result;
  }
  const auto & first = path.poses.front();
  if (first.header.frame_id != path.header.frame_id ||
    std::hypot(
      first.pose.position.x - expected_start.pose.position.x,
      first.pose.position.y - expected_start.pose.position.y) > 1.0e-6 ||
    departureConnectorAngularDistance(
      departureConnectorYaw(first.pose.orientation),
      departureConnectorYaw(expected_start.pose.orientation)) > 1.0e-6)
  {
    result.reason = "start_mismatch";
    return result;
  }

  const double minimum_projection = std::cos(options.maximum_direction_error);
  const double curvature_limit =
    1.0 / options.minimum_turning_radius_m + options.curvature_tolerance;
  for (std::size_t index = 0U; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index];
    if (pose.header.frame_id != path.header.frame_id ||
      !departureConnectorFinite(pose.pose.position.x) ||
      !departureConnectorFinite(pose.pose.position.y) ||
      !departureConnectorUnitQuaternion(pose.pose.orientation))
    {
      result.reason = "pose_invalid";
      return result;
    }
    if (index + 1U >= path.poses.size()) {
      continue;
    }
    const auto & next = path.poses[index + 1U];
    const double delta_x = next.pose.position.x - pose.pose.position.x;
    const double delta_y = next.pose.position.y - pose.pose.position.y;
    const double segment_length = std::hypot(delta_x, delta_y);
    if (!departureConnectorFinite(segment_length) ||
      segment_length < options.minimum_segment_length)
    {
      result.reason = "segment_too_short";
      return result;
    }
    const double current_yaw = departureConnectorYaw(pose.pose.orientation);
    const double next_yaw = departureConnectorYaw(next.pose.orientation);
    const double current_projection =
      (delta_x * std::cos(current_yaw) + delta_y * std::sin(current_yaw)) / segment_length;
    const double next_projection =
      (delta_x * std::cos(next_yaw) + delta_y * std::sin(next_yaw)) / segment_length;
    if (current_projection < minimum_projection || next_projection < minimum_projection) {
      result.reason = "segment_not_forward";
      return result;
    }
  }

  for (std::size_t index = 1U; index + 1U < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1U].pose.position;
    const double first_x = current.x - previous.x;
    const double first_y = current.y - previous.y;
    const double second_x = next.x - current.x;
    const double second_y = next.y - current.y;
    const double chord_x = next.x - previous.x;
    const double chord_y = next.y - previous.y;
    const double first_length = std::hypot(first_x, first_y);
    const double second_length = std::hypot(second_x, second_y);
    const double chord_length = std::hypot(chord_x, chord_y);
    if (first_length < options.minimum_segment_length ||
      second_length < options.minimum_segment_length ||
      chord_length < options.minimum_segment_length)
    {
      result.reason = "curvature_segment_too_short";
      return result;
    }
    const double curvature = 2.0 * std::abs(first_x * chord_y - first_y * chord_x) /
      (first_length * second_length * chord_length);
    if (!departureConnectorFinite(curvature) || curvature > curvature_limit) {
      result.reason = "curvature_exceeded";
      return result;
    }
    result.maximum_curvature = std::max(result.maximum_curvature, curvature);
  }

  result.length_m = departureConnectorPathLength(path);
  if (!departureConnectorFinite(result.length_m) || result.length_m <= 0.0) {
    result.reason = "length_invalid";
    return result;
  }
  result.valid = true;
  result.reason = "ok";
  return result;
}

inline double departureConnectorPositiveAngle(double angle)
{
  if (!departureConnectorFinite(angle)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  angle = std::fmod(angle, 2.0 * kDepartureConnectorPi);
  return angle < 0.0 ? angle + 2.0 * kDepartureConnectorPi : angle;
}

// The P escape prefix terminates on a Smac lattice state north of the A-zone.
// A locked task yaw needs a geometric terminal tangent, not merely a final
// pose quaternion that a controller cannot follow. This bounded RSL connector
// joins that prefix to the authored task pose at the active simulator radius.
// It is intentionally only used by the P-specific caller below; all other
// navigation edges continue through normal Smac planning.
inline bool buildPDepartureRslTerminalConnector(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double turning_radius_m,
  const DepartureConnectorOptions & options,
  nav_msgs::msg::Path & connector)
{
  connector = nav_msgs::msg::Path();
  if (!departureConnectorOptionsValid(options) || start.header.frame_id.empty() ||
    start.header.frame_id != goal.header.frame_id ||
    !departureConnectorFinite(start.pose.position.x) ||
    !departureConnectorFinite(start.pose.position.y) ||
    !departureConnectorFinite(goal.pose.position.x) ||
    !departureConnectorFinite(goal.pose.position.y) ||
    !departureConnectorUnitQuaternion(start.pose.orientation) ||
    !departureConnectorUnitQuaternion(goal.pose.orientation) ||
    !departureConnectorFinite(turning_radius_m) ||
    turning_radius_m < options.minimum_turning_radius_m - 1.0e-9 ||
    turning_radius_m > options.minimum_turning_radius_m + options.radius_margin_m + 1.0e-9)
  {
    return false;
  }

  const double radius = turning_radius_m;
  const double start_yaw = departureConnectorYaw(start.pose.orientation);
  const double goal_yaw = departureConnectorYaw(goal.pose.orientation);
  const double delta_x = goal.pose.position.x - start.pose.position.x;
  const double delta_y = goal.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(delta_x, delta_y);
  if (!departureConnectorFinite(radius) || !departureConnectorFinite(start_yaw) ||
    !departureConnectorFinite(goal_yaw) || !departureConnectorFinite(distance) ||
    radius <= 0.0 || distance <= options.minimum_segment_length)
  {
    return false;
  }

  const double normalized_distance = distance / radius;
  const double bearing = std::atan2(delta_y, delta_x);
  const double alpha = departureConnectorPositiveAngle(start_yaw - bearing);
  const double beta = departureConnectorPositiveAngle(goal_yaw - bearing);
  if (!departureConnectorFinite(normalized_distance) || !departureConnectorFinite(bearing) ||
    !departureConnectorFinite(alpha) || !departureConnectorFinite(beta))
  {
    return false;
  }

  const double straight_squared = -2.0 + normalized_distance * normalized_distance +
    2.0 * std::cos(alpha - beta) -
    2.0 * normalized_distance * (std::sin(alpha) + std::sin(beta));
  if (!departureConnectorFinite(straight_squared) || straight_squared < -1.0e-9) {
    return false;
  }
  const double straight_normalized = std::sqrt(std::max(0.0, straight_squared));
  const double tangent = std::atan2(
    std::cos(alpha) + std::cos(beta),
    normalized_distance - std::sin(alpha) - std::sin(beta)) -
    std::atan2(2.0, straight_normalized);
  const double right_angle = departureConnectorPositiveAngle(alpha - tangent);
  const double left_angle = departureConnectorPositiveAngle(beta - tangent);
  if (!departureConnectorFinite(straight_normalized) || !departureConnectorFinite(tangent) ||
    !departureConnectorFinite(right_angle) || !departureConnectorFinite(left_angle))
  {
    return false;
  }

  connector.header = start.header;
  connector.poses.push_back(start);
  geometry_msgs::msg::PoseStamped current = start;
  const auto append_arc = [&connector, &current, radius, &options](bool left, double angle) {
      const double length = radius * angle;
      if (length <= options.minimum_segment_length) {
        return true;
      }
      const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(length / options.sample_spacing_m)));
      const double initial_yaw = departureConnectorYaw(current.pose.orientation);
      const double curvature = (left ? 1.0 : -1.0) / radius;
      for (std::size_t index = 1U; index <= samples; ++index) {
        const double yaw = initial_yaw + (left ? angle : -angle) *
          static_cast<double>(index) / static_cast<double>(samples);
        geometry_msgs::msg::PoseStamped pose = current;
        pose.pose.position.x = current.pose.position.x +
          (std::sin(yaw) - std::sin(initial_yaw)) / curvature;
        pose.pose.position.y = current.pose.position.y -
          (std::cos(yaw) - std::cos(initial_yaw)) / curvature;
        pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
        connector.poses.push_back(pose);
      }
      current = connector.poses.back();
      return true;
    };
  const auto append_straight = [&connector, &current, &options](double length) {
      if (length <= options.minimum_segment_length) {
        return true;
      }
      const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(length / options.sample_spacing_m)));
      const double yaw = departureConnectorYaw(current.pose.orientation);
      for (std::size_t index = 1U; index <= samples; ++index) {
        const double fraction = static_cast<double>(index) / static_cast<double>(samples);
        geometry_msgs::msg::PoseStamped pose = current;
        pose.pose.position.x += length * fraction * std::cos(yaw);
        pose.pose.position.y += length * fraction * std::sin(yaw);
        connector.poses.push_back(pose);
      }
      current = connector.poses.back();
      return true;
    };

  if (!append_arc(false, right_angle) ||
    !append_straight(straight_normalized * radius) ||
    !append_arc(true, left_angle))
  {
    return false;
  }
  if (connector.poses.size() < 3U ||
    std::hypot(
      current.pose.position.x - goal.pose.position.x,
      current.pose.position.y - goal.pose.position.y) > 1.0e-5 ||
    departureConnectorAngularDistance(
      departureConnectorYaw(current.pose.orientation), goal_yaw) > 1.0e-5)
  {
    connector = nav_msgs::msg::Path();
    return false;
  }

  auto & terminal = connector.poses.back();
  terminal.pose.position = goal.pose.position;
  terminal.pose.orientation = goal.pose.orientation;
  terminal.header = start.header;
  return validateForwardConnectorPath(connector, start, options).valid;
}

inline bool buildPDepartureRslTerminalConnector(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const DepartureConnectorOptions & options,
  nav_msgs::msg::Path & connector)
{
  return buildPDepartureRslTerminalConnector(
    start, goal, options.minimum_turning_radius_m, options, connector);
}

inline std::vector<DepartureConnector> buildLeftDepartureConnectors(
  const geometry_msgs::msg::PoseStamped & start,
  const DepartureConnectorOptions & options)
{
  std::vector<DepartureConnector> connectors;
  if (!departureConnectorOptionsValid(options) || start.header.frame_id.empty() ||
    !departureConnectorFinite(start.pose.position.x) ||
    !departureConnectorFinite(start.pose.position.y) ||
    !departureConnectorUnitQuaternion(start.pose.orientation))
  {
    return connectors;
  }

  const double radius = options.minimum_turning_radius_m + options.radius_margin_m;
  if (!departureConnectorFinite(radius) || radius < options.minimum_turning_radius_m ||
    radius > 6.0)
  {
    return connectors;
  }
  const double initial_yaw = departureConnectorYaw(start.pose.orientation);
  if (!departureConnectorFinite(initial_yaw)) {
    return connectors;
  }

  // The larger departure radius clears the south lethal ring with a full
  // costmap-cell margin. Keep the arc aligned with the P-to-A bearing: a
  // deeper turn would make the next Smac segment demand an infeasible right
  // correction, and the old 45 degree fallback clipped cone_a1.
  constexpr std::array<double, 2> kArcAngles = {
    kDepartureConnectorPi / 9.0,
    kDepartureConnectorPi / 12.0,
  };
  connectors.reserve(kArcAngles.size());
  for (const double arc_angle : kArcAngles) {
    const double arc_length = radius * arc_angle;
    const std::size_t samples = std::max<std::size_t>(
      2U, static_cast<std::size_t>(std::ceil(arc_length / options.sample_spacing_m)));
    DepartureConnector connector;
    connector.radius_m = radius;
    connector.arc_angle_rad = arc_angle;
    connector.path.header = start.header;
    connector.path.poses.reserve(samples + 1U);
    for (std::size_t index = 0U; index <= samples; ++index) {
      const double fraction = static_cast<double>(index) / static_cast<double>(samples);
      const double yaw = initial_yaw + arc_angle * fraction;
      geometry_msgs::msg::PoseStamped pose = start;
      pose.pose.position.x = start.pose.position.x + radius * (
        std::sin(yaw) - std::sin(initial_yaw));
      pose.pose.position.y = start.pose.position.y - radius * (
        std::cos(yaw) - std::cos(initial_yaw));
      pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
      connector.path.poses.push_back(std::move(pose));
    }
    connector.length_m = departureConnectorPathLength(connector.path);
    connectors.push_back(std::move(connector));
  }
  return connectors;
}

// Build simulation-internal P departure candidates whose terminal pose lies on
// the same Cartesian and angular lattice consumed by Smac.  Unlike the legacy
// pure-arc helper above, each candidate is a left arc followed by a tangent
// forward segment.  This avoids asking a planner to quantize the endpoint of
// an already executable arc and then requiring an unsafe spatial snap.
//
// The costmap is assumed to have an axis-aligned origin.  The caller supplies
// the live costmap metadata rather than a field-specific coordinate so this
// helper never encodes a particular P or A task location.
inline std::vector<DepartureConnector> buildLeftDepartureLatticeConnectors(
  const geometry_msgs::msg::PoseStamped & start,
  double costmap_origin_x_m,
  double costmap_origin_y_m,
  double costmap_resolution_m,
  std::size_t heading_bins,
  const DepartureConnectorOptions & options)
{
  std::vector<DepartureConnector> connectors;
  if (!departureConnectorOptionsValid(options) || start.header.frame_id.empty() ||
    !departureConnectorFinite(start.pose.position.x) ||
    !departureConnectorFinite(start.pose.position.y) ||
    !departureConnectorUnitQuaternion(start.pose.orientation) ||
    !departureConnectorFinite(costmap_origin_x_m) ||
    !departureConnectorFinite(costmap_origin_y_m) ||
    !departureConnectorFinite(costmap_resolution_m) || costmap_resolution_m <= 0.0 ||
    costmap_resolution_m > 1.0 || heading_bins < 4U)
  {
    return connectors;
  }

  const double maximum_radius =
    options.minimum_turning_radius_m + options.radius_margin_m;
  if (!departureConnectorFinite(maximum_radius) ||
    maximum_radius < options.minimum_turning_radius_m)
  {
    return connectors;
  }
  const double initial_yaw = departureConnectorYaw(start.pose.orientation);
  if (!departureConnectorFinite(initial_yaw)) {
    return connectors;
  }

  const double heading_step = 2.0 * kDepartureConnectorPi /
    static_cast<double>(heading_bins);
  if (!departureConnectorFinite(heading_step) || heading_step <= 0.0) {
    return connectors;
  }

  struct CandidateGeometry
  {
    double terminal_x{0.0};
    double terminal_y{0.0};
    double terminal_yaw{0.0};
    double second_radius_m{0.0};
    double straight_length_m{0.0};
    double priority{0.0};
  };
  std::vector<CandidateGeometry> candidates;

  // A shallow departure leaves just enough room for Hybrid-A* to turn right
  // below A1, where the padded rear body clips P's south keepout. A short
  // left-right S keeps the rear above that boundary, then restores a nearly
  // forward lattice state below A1 before handing control to Smac. This is
  // internal P-only geometry, not a route waypoint or an unchecked prefix.
  constexpr double kFirstArcAngle = 5.0 * kDepartureConnectorPi / 36.0;
  constexpr std::array<double, 3> kDesiredTurns = {
    0.0,
    kDepartureConnectorPi / 72.0,
    -kDepartureConnectorPi / 72.0,
  };
  constexpr int kCellSearchRadius = 3;
  const double first_radius = std::max(
    options.minimum_turning_radius_m,
    maximum_radius - 3.0 * costmap_resolution_m);
  const double preferred_second_radius = options.minimum_turning_radius_m;
  if (!departureConnectorFinite(first_radius) ||
    !departureConnectorFinite(preferred_second_radius) ||
    first_radius < options.minimum_turning_radius_m || first_radius > maximum_radius ||
    preferred_second_radius < options.minimum_turning_radius_m ||
    preferred_second_radius > maximum_radius)
  {
    return connectors;
  }
  const double cosine_initial = std::cos(initial_yaw);
  const double sine_initial = std::sin(initial_yaw);
  const double sine_first_arc = std::sin(kFirstArcAngle);
  const double cosine_first_arc = std::cos(kFirstArcAngle);
  const double first_endpoint_local_x = first_radius * sine_first_arc;
  const double first_endpoint_local_y = first_radius * (1.0 - cosine_first_arc);
  for (std::size_t turn_index = 0U; turn_index < kDesiredTurns.size(); ++turn_index) {
    const double requested_yaw = initial_yaw + kDesiredTurns[turn_index];
    const double terminal_yaw = std::round(requested_yaw / heading_step) * heading_step;
    const double terminal_turn = std::remainder(
      terminal_yaw - initial_yaw, 2.0 * kDepartureConnectorPi);
    if (!departureConnectorFinite(terminal_turn) ||
      terminal_turn >= kFirstArcAngle - 1.0e-6 ||
      terminal_turn <= -kDepartureConnectorPi / 2.0)
    {
      continue;
    }
    const double sine_terminal = std::sin(terminal_turn);
    const double cosine_terminal = std::cos(terminal_turn);
    const double sine_delta = sine_first_arc - sine_terminal;
    const double cosine_delta = cosine_terminal - cosine_first_arc;
    const double determinant = sine_delta * sine_terminal -
      cosine_delta * cosine_terminal;
    if (std::abs(determinant) <= 1.0e-9) {
      continue;
    }

    const double preferred_local_x = first_endpoint_local_x +
      preferred_second_radius * sine_delta + costmap_resolution_m * cosine_terminal;
    const double preferred_local_y = first_endpoint_local_y +
      preferred_second_radius * cosine_delta + costmap_resolution_m * sine_terminal;
    const double preferred_x = start.pose.position.x +
      cosine_initial * preferred_local_x - sine_initial * preferred_local_y;
    const double preferred_y = start.pose.position.y +
      sine_initial * preferred_local_x + cosine_initial * preferred_local_y;
    const auto nearest_cell_x = static_cast<long long>(std::llround(
      (preferred_x - costmap_origin_x_m) / costmap_resolution_m - 0.5));
    const auto nearest_cell_y = static_cast<long long>(std::llround(
      (preferred_y - costmap_origin_y_m) / costmap_resolution_m - 0.5));

    std::optional<CandidateGeometry> best_for_turn;
    for (int offset_x = -kCellSearchRadius; offset_x <= kCellSearchRadius; ++offset_x) {
      for (int offset_y = -kCellSearchRadius; offset_y <= kCellSearchRadius; ++offset_y) {
        const double terminal_x = costmap_origin_x_m +
          (static_cast<double>(nearest_cell_x + offset_x) + 0.5) * costmap_resolution_m;
        const double terminal_y = costmap_origin_y_m +
          (static_cast<double>(nearest_cell_y + offset_y) + 0.5) * costmap_resolution_m;
        const double world_x = terminal_x - start.pose.position.x;
        const double world_y = terminal_y - start.pose.position.y;
        const double local_x = cosine_initial * world_x + sine_initial * world_y;
        const double local_y = -sine_initial * world_x + cosine_initial * world_y;
        const double connector_x = local_x - first_endpoint_local_x;
        const double connector_y = local_y - first_endpoint_local_y;
        const double second_radius =
          (connector_x * sine_terminal - connector_y * cosine_terminal) / determinant;
        const double straight_length =
          (-cosine_delta * connector_x + sine_delta * connector_y) / determinant;
        if (!departureConnectorFinite(second_radius) ||
          !departureConnectorFinite(straight_length) ||
          second_radius < options.minimum_turning_radius_m - 1.0e-9 ||
          second_radius > maximum_radius + 1.0e-9 || straight_length < -1.0e-9 ||
          straight_length > 4.0 * costmap_resolution_m + 1.0e-9)
        {
          continue;
        }
        const double priority = static_cast<double>(turn_index) * 10.0 +
          std::abs(second_radius - preferred_second_radius) +
          std::hypot(terminal_x - preferred_x, terminal_y - preferred_y);
        CandidateGeometry geometry{
          terminal_x, terminal_y, terminal_yaw, second_radius,
          std::max(0.0, straight_length), priority};
        if (!best_for_turn.has_value() || geometry.priority < best_for_turn->priority) {
          best_for_turn = geometry;
        }
      }
    }
    if (best_for_turn.has_value()) {
      candidates.push_back(*best_for_turn);
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const CandidateGeometry & first, const CandidateGeometry & second) {
      return first.priority < second.priority;
  });
  connectors.reserve(candidates.size());
  for (const auto & geometry : candidates) {
    const double second_arc_angle = std::abs(std::remainder(
      geometry.terminal_yaw - (initial_yaw + kFirstArcAngle),
      2.0 * kDepartureConnectorPi));
    if (!departureConnectorFinite(second_arc_angle) || second_arc_angle <= 1.0e-6 ||
      second_arc_angle >= kDepartureConnectorPi)
    {
      continue;
    }
    const double first_arc_length = first_radius * kFirstArcAngle;
    const double second_arc_length = geometry.second_radius_m * second_arc_angle;
    const std::size_t first_arc_samples = std::max<std::size_t>(
      2U, static_cast<std::size_t>(std::ceil(first_arc_length / options.sample_spacing_m)));
    const std::size_t second_arc_samples = std::max<std::size_t>(
      2U, static_cast<std::size_t>(std::ceil(second_arc_length / options.sample_spacing_m)));
    DepartureConnector connector;
    connector.radius_m = std::min(first_radius, geometry.second_radius_m);
    connector.arc_angle_rad = kFirstArcAngle;
    connector.straight_length_m = geometry.straight_length_m;
    connector.lattice_aligned = true;
    connector.path.header = start.header;
    connector.path.poses.reserve(first_arc_samples + second_arc_samples + 1U + static_cast<std::size_t>(
      std::ceil(geometry.straight_length_m / options.sample_spacing_m)));
    for (std::size_t index = 0U; index <= first_arc_samples; ++index) {
      const double fraction = static_cast<double>(index) / static_cast<double>(first_arc_samples);
      const double yaw = initial_yaw + kFirstArcAngle * fraction;
      geometry_msgs::msg::PoseStamped pose = start;
      pose.pose.position.x = start.pose.position.x + first_radius * (
        std::sin(yaw) - std::sin(initial_yaw));
      pose.pose.position.y = start.pose.position.y - first_radius * (
        std::cos(yaw) - std::cos(initial_yaw));
      pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
      connector.path.poses.push_back(std::move(pose));
    }
    const double first_arc_yaw = initial_yaw + kFirstArcAngle;
    const auto first_arc_end = connector.path.poses.back();
    for (std::size_t index = 1U; index <= second_arc_samples; ++index) {
      const double fraction = static_cast<double>(index) /
        static_cast<double>(second_arc_samples);
      const double yaw = first_arc_yaw + std::remainder(
        geometry.terminal_yaw - first_arc_yaw,
        2.0 * kDepartureConnectorPi) * fraction;
      geometry_msgs::msg::PoseStamped pose = first_arc_end;
      pose.pose.position.x = first_arc_end.pose.position.x + geometry.second_radius_m * (
        std::sin(first_arc_yaw) - std::sin(yaw));
      pose.pose.position.y = first_arc_end.pose.position.y + geometry.second_radius_m * (
        std::cos(yaw) - std::cos(first_arc_yaw));
      pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
      connector.path.poses.push_back(std::move(pose));
    }
    const std::size_t straight_samples = static_cast<std::size_t>(std::ceil(
      geometry.straight_length_m / options.sample_spacing_m));
    for (std::size_t index = 1U; index <= straight_samples; ++index) {
      const double fraction = static_cast<double>(index) /
        static_cast<double>(straight_samples);
      geometry_msgs::msg::PoseStamped pose = connector.path.poses.back();
      const auto & arc_end = connector.path.poses[first_arc_samples + second_arc_samples];
      pose.pose.position.x = arc_end.pose.position.x +
        geometry.straight_length_m * fraction * std::cos(geometry.terminal_yaw);
      pose.pose.position.y = arc_end.pose.position.y +
        geometry.straight_length_m * fraction * std::sin(geometry.terminal_yaw);
      pose.pose.orientation = departureConnectorQuaternionFromYaw(geometry.terminal_yaw);
      connector.path.poses.push_back(std::move(pose));
    }
    if (connector.path.poses.empty()) {
      continue;
    }
    auto & terminal_pose = connector.path.poses.back();
    terminal_pose.pose.position.x = geometry.terminal_x;
    terminal_pose.pose.position.y = geometry.terminal_y;
    terminal_pose.pose.orientation = departureConnectorQuaternionFromYaw(geometry.terminal_yaw);
    const auto validation = validateForwardConnectorPath(connector.path, start, options);
    if (!validation.valid) {
      continue;
    }
    connector.length_m = departureConnectorPathLength(connector.path);
    connectors.push_back(std::move(connector));
  }
  return connectors;
}

// P starts beside the south field edge. With the simulator's 0.22 m minimum
// radius, leaving the first right turn to Smac gives RPP an exact-minimum
// radius reversal while it is still carrying a lateral tracking error.
//
// This P-only egress reaches the A-zone's upper lane first, then makes its
// right turn at a deliberately large radius before following a short eastward
// line. It never makes the old low-altitude right correction beside A4. The
// final pose is an exact Smac lattice state; every sample remains subject to
// the caller's live raw-costmap and static-keepout footprint sweeps.
inline std::vector<DepartureConnector> buildPDepartureEscapeLatticeConnectors(
  const geometry_msgs::msg::PoseStamped & start,
  double costmap_origin_x_m,
  double costmap_origin_y_m,
  double costmap_resolution_m,
  std::size_t heading_bins,
  double high_right_turn_radius_m,
  const DepartureConnectorOptions & options)
{
  std::vector<DepartureConnector> connectors;
  if (!departureConnectorOptionsValid(options) || start.header.frame_id.empty() ||
    !departureConnectorFinite(start.pose.position.x) ||
    !departureConnectorFinite(start.pose.position.y) ||
    !departureConnectorUnitQuaternion(start.pose.orientation) ||
    !departureConnectorFinite(costmap_origin_x_m) ||
    !departureConnectorFinite(costmap_origin_y_m) ||
    !departureConnectorFinite(costmap_resolution_m) || costmap_resolution_m <= 0.0 ||
    costmap_resolution_m > 1.0 || heading_bins < 4U)
  {
    return connectors;
  }

  const double maximum_radius =
    options.minimum_turning_radius_m + options.radius_margin_m;
  const double initial_yaw = departureConnectorYaw(start.pose.orientation);
  const double heading_step = 2.0 * kDepartureConnectorPi /
    static_cast<double>(heading_bins);
  if (!departureConnectorFinite(maximum_radius) ||
    maximum_radius < options.minimum_turning_radius_m ||
    !departureConnectorHighRightTurnRadiusWithinEnvelope(
      options, high_right_turn_radius_m, maximum_radius) ||
    !departureConnectorFinite(initial_yaw) || !departureConnectorFinite(heading_step) ||
    heading_step <= 0.0)
  {
    return connectors;
  }

  // Do not create a prefix that asks Smac to repair an angular handoff. The
  // initial left turn, northbound leg, and final eastbound handoff must all
  // be exact lattice headings.
  //
  // P is only 0.25 m north of the south keepout. A shallow lead-in can pass
  // a nominal path sweep yet leave no room for RPP's closed-loop correction.
  // Turn slightly earlier with a 32.5-degree lead-in, but retain two cells of
  // radius below the active envelope so the swept front stays west of A1.
  // 32.5 degrees is exactly thirteen 2.5-degree bins in the 144-bin lattice.
  constexpr double kFirstArcAngle = 13.0 * kDepartureConnectorPi / 72.0;
  constexpr double kSecondArcAngle = kDepartureConnectorPi / 2.0 - kFirstArcAngle;
  constexpr double kPreferredNorthLengthM = 0.804;
  constexpr double kMinimumNorthLengthM = 0.70;
  constexpr double kMaximumNorthLengthM = 0.90;
  constexpr double kPreferredEastLengthM = 1.3775;
  constexpr double kMinimumEastLengthM = 1.25;
  constexpr double kMaximumEastLengthM = 1.55;
  constexpr int kCellSearchRadius = 4;
  const double lattice_yaw = std::round(initial_yaw / heading_step) * heading_step;
  const double first_arc_end_yaw = initial_yaw + kFirstArcAngle;
  const double north_yaw = initial_yaw + kDepartureConnectorPi / 2.0;
  const double terminal_yaw = initial_yaw;
  const double lattice_first_arc_end_yaw =
    std::round(first_arc_end_yaw / heading_step) * heading_step;
  const double lattice_north_yaw = std::round(north_yaw / heading_step) * heading_step;
  const double lattice_terminal_yaw = std::round(terminal_yaw / heading_step) * heading_step;
  if (departureConnectorAngularDistance(initial_yaw, lattice_yaw) > 1.0e-9 ||
    departureConnectorAngularDistance(first_arc_end_yaw, lattice_first_arc_end_yaw) > 1.0e-9 ||
    departureConnectorAngularDistance(north_yaw, lattice_north_yaw) > 1.0e-9 ||
    departureConnectorAngularDistance(terminal_yaw, lattice_terminal_yaw) > 1.0e-9)
  {
    return connectors;
  }

  // The staged turn clears both the south boundary and A1 with the full
  // padded footprint plus the one-sided controller envelope. Two local grid
  // cells below the configured maximum turn the body north early enough to
  // clear the south edge without letting its front reach A1. The result stays
  // explicitly bounded by the same P-only simulator gate.
  const double first_radius = maximum_radius - 2.0 * costmap_resolution_m;
  const double second_left_radius = options.minimum_turning_radius_m;
  if (!departureConnectorFinite(first_radius) || first_radius < second_left_radius ||
    first_radius > maximum_radius || !departureConnectorFinite(second_left_radius) ||
    second_left_radius > maximum_radius + 1.0e-9)
  {
    return connectors;
  }

  const double sine_first = std::sin(kFirstArcAngle);
  const double cosine_first = std::cos(kFirstArcAngle);
  const double terminal_x_second_radius_scale = 1.0 - sine_first;
  if (!departureConnectorFinite(terminal_x_second_radius_scale) ||
    terminal_x_second_radius_scale <= 1.0e-9 ||
    !departureConnectorFinite(kSecondArcAngle) || kSecondArcAngle <= 0.0)
  {
    return connectors;
  }
  const double first_end_local_x = first_radius * sine_first;
  const double first_end_local_y = first_radius * (1.0 - cosine_first);
  const double north_start_local_x = first_end_local_x +
    second_left_radius * terminal_x_second_radius_scale;
  const double north_start_local_y = first_end_local_y + second_left_radius * cosine_first;
  const double cosine_initial = std::cos(initial_yaw);
  const double sine_initial = std::sin(initial_yaw);
  const double preferred_local_x = north_start_local_x + high_right_turn_radius_m +
    kPreferredEastLengthM;
  const double preferred_local_y = north_start_local_y + kPreferredNorthLengthM +
    high_right_turn_radius_m;
  const double preferred_x = start.pose.position.x +
    cosine_initial * preferred_local_x - sine_initial * preferred_local_y;
  const double preferred_y = start.pose.position.y +
    sine_initial * preferred_local_x + cosine_initial * preferred_local_y;
  const auto nearest_cell_x = static_cast<long long>(std::llround(
    (preferred_x - costmap_origin_x_m) / costmap_resolution_m - 0.5));
  const auto nearest_cell_y = static_cast<long long>(std::llround(
    (preferred_y - costmap_origin_y_m) / costmap_resolution_m - 0.5));

  struct CandidateGeometry
  {
    double terminal_x{0.0};
    double terminal_y{0.0};
    double north_length_m{0.0};
    double east_length_m{0.0};
    double priority{0.0};
  };
  std::optional<CandidateGeometry> selected;
  for (int offset_x = -kCellSearchRadius; offset_x <= kCellSearchRadius; ++offset_x) {
    for (int offset_y = -kCellSearchRadius; offset_y <= kCellSearchRadius; ++offset_y) {
      const double terminal_x = costmap_origin_x_m +
        (static_cast<double>(nearest_cell_x + offset_x) + 0.5) * costmap_resolution_m;
      const double terminal_y = costmap_origin_y_m +
        (static_cast<double>(nearest_cell_y + offset_y) + 0.5) * costmap_resolution_m;
      const double world_x = terminal_x - start.pose.position.x;
      const double world_y = terminal_y - start.pose.position.y;
      const double local_x = cosine_initial * world_x + sine_initial * world_y;
      const double local_y = -sine_initial * world_x + cosine_initial * world_y;
      const double north_length = local_y - north_start_local_y - high_right_turn_radius_m;
      const double east_length = local_x - north_start_local_x - high_right_turn_radius_m;
      if (!departureConnectorFinite(north_length) || !departureConnectorFinite(east_length) ||
        north_length < kMinimumNorthLengthM - 1.0e-9 ||
        north_length > kMaximumNorthLengthM + 1.0e-9 ||
        east_length < kMinimumEastLengthM - 1.0e-9 ||
        east_length > kMaximumEastLengthM + 1.0e-9)
      {
        continue;
      }
      const double priority = std::abs(north_length - kPreferredNorthLengthM) +
        std::abs(east_length - kPreferredEastLengthM) +
        std::hypot(terminal_x - preferred_x, terminal_y - preferred_y);
      CandidateGeometry candidate{
        terminal_x, terminal_y, north_length, east_length, priority};
      if (!selected.has_value() || candidate.priority < selected->priority) {
        selected = candidate;
      }
    }
  }
  if (!selected.has_value()) {
    return connectors;
  }

  const double first_arc_length = first_radius * kFirstArcAngle;
  const double second_arc_length = second_left_radius * kSecondArcAngle;
  const double right_arc_length = high_right_turn_radius_m * kDepartureConnectorPi / 2.0;
  const std::size_t first_arc_samples = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(first_arc_length / options.sample_spacing_m)));
  const std::size_t second_arc_samples = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(second_arc_length / options.sample_spacing_m)));
  const std::size_t north_samples = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(
      selected->north_length_m / options.sample_spacing_m)));
  const std::size_t right_arc_samples = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(right_arc_length / options.sample_spacing_m)));
  const std::size_t east_samples = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(
      selected->east_length_m / options.sample_spacing_m)));

  DepartureConnector connector;
  connector.radius_m = std::min(first_radius, second_left_radius);
  connector.high_right_turn_radius_m = high_right_turn_radius_m;
  connector.arc_angle_rad = kFirstArcAngle;
  connector.straight_length_m = selected->north_length_m + selected->east_length_m;
  connector.lattice_aligned = true;
  connector.path.header = start.header;
  connector.path.poses.reserve(
    first_arc_samples + second_arc_samples + north_samples + right_arc_samples + east_samples +
    1U);
  for (std::size_t index = 0U; index <= first_arc_samples; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(first_arc_samples);
    const double yaw = initial_yaw + kFirstArcAngle * fraction;
    geometry_msgs::msg::PoseStamped pose = start;
    pose.pose.position.x = start.pose.position.x + first_radius * (
      std::sin(yaw) - std::sin(initial_yaw));
    pose.pose.position.y = start.pose.position.y - first_radius * (
      std::cos(yaw) - std::cos(initial_yaw));
    pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
    connector.path.poses.push_back(std::move(pose));
  }
  const auto first_end = connector.path.poses.back();
  for (std::size_t index = 1U; index <= second_arc_samples; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(second_arc_samples);
    const double yaw = first_arc_end_yaw + kSecondArcAngle * fraction;
    geometry_msgs::msg::PoseStamped pose = first_end;
      pose.pose.position.x = first_end.pose.position.x + second_left_radius * (
        std::sin(yaw) - std::sin(first_arc_end_yaw));
      pose.pose.position.y = first_end.pose.position.y - second_left_radius * (
        std::cos(yaw) - std::cos(first_arc_end_yaw));
    pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
    connector.path.poses.push_back(std::move(pose));
  }
  const auto north_start = connector.path.poses.back();
  for (std::size_t index = 1U; index <= north_samples; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(north_samples);
    geometry_msgs::msg::PoseStamped pose = north_start;
    pose.pose.position.x = north_start.pose.position.x + selected->north_length_m * fraction *
      std::cos(north_yaw);
    pose.pose.position.y = north_start.pose.position.y + selected->north_length_m * fraction *
      std::sin(north_yaw);
    pose.pose.orientation = departureConnectorQuaternionFromYaw(north_yaw);
    connector.path.poses.push_back(std::move(pose));
  }
  const auto right_start = connector.path.poses.back();
  for (std::size_t index = 1U; index <= right_arc_samples; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(right_arc_samples);
    const double yaw = north_yaw - kDepartureConnectorPi / 2.0 * fraction;
    geometry_msgs::msg::PoseStamped pose = right_start;
      pose.pose.position.x = right_start.pose.position.x + high_right_turn_radius_m * (
        std::sin(north_yaw) - std::sin(yaw));
      pose.pose.position.y = right_start.pose.position.y + high_right_turn_radius_m * (
        std::cos(yaw) - std::cos(north_yaw));
    pose.pose.orientation = departureConnectorQuaternionFromYaw(yaw);
    connector.path.poses.push_back(std::move(pose));
  }
  const auto east_start = connector.path.poses.back();
  for (std::size_t index = 1U; index <= east_samples; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(east_samples);
    geometry_msgs::msg::PoseStamped pose = east_start;
    pose.pose.position.x = east_start.pose.position.x + selected->east_length_m * fraction *
      std::cos(terminal_yaw);
    pose.pose.position.y = east_start.pose.position.y + selected->east_length_m * fraction *
      std::sin(terminal_yaw);
    pose.pose.orientation = departureConnectorQuaternionFromYaw(terminal_yaw);
    connector.path.poses.push_back(std::move(pose));
  }
  auto & terminal_pose = connector.path.poses.back();
  terminal_pose.pose.position.x = selected->terminal_x;
  terminal_pose.pose.position.y = selected->terminal_y;
  terminal_pose.pose.orientation = departureConnectorQuaternionFromYaw(terminal_yaw);
  const auto validation = validateForwardConnectorPath(connector.path, start, options);
  if (!validation.valid) {
    return connectors;
  }
  connector.length_m = validation.length_m;
  connectors.push_back(std::move(connector));
  return connectors;
}

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__DEPARTURE_CONNECTOR_HPP_
