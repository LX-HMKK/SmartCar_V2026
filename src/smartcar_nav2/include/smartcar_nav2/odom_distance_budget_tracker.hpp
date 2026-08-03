#ifndef SMARTCAR_NAV2__ODOM_DISTANCE_BUDGET_TRACKER_HPP_
#define SMARTCAR_NAV2__ODOM_DISTANCE_BUDGET_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace smartcar_nav2
{

struct OdomPlanarPose
{
  double x{0.0};
  double y{0.0};
};

enum class OdomDistanceBudgetUpdate
{
  kAccepted,
  kBudgetExceeded,
  kInvalidSample,
  kStepTooLarge,
};

// Tracks physical XY travel for one navigation action. The budget is based on
// the initial straight-line task distance, but the measured value accumulates
// every odometry segment so a loop cannot hide behind a small net displacement.
class OdomDistanceBudgetTracker
{
public:
  bool initialize(
    const OdomPlanarPose & start,
    const OdomPlanarPose & goal,
    double max_distance_ratio,
    double distance_slack_m,
    double max_odom_step_m)
  {
    if (!isFinite(start) || !isFinite(goal) ||
      !std::isfinite(max_distance_ratio) || max_distance_ratio < 1.0 ||
      !std::isfinite(distance_slack_m) || distance_slack_m < 0.0 ||
      !std::isfinite(max_odom_step_m) || max_odom_step_m <= 0.0)
    {
      reset();
      return false;
    }

    const double direct_distance = distance(start, goal);
    budget_m_ = std::max(
      max_distance_ratio * direct_distance,
      direct_distance + distance_slack_m);
    if (!std::isfinite(budget_m_) || budget_m_ <= 0.0) {
      reset();
      return false;
    }

    last_pose_ = start;
    travelled_m_ = 0.0;
    max_odom_step_m_ = max_odom_step_m;
    initialized_ = true;
    return true;
  }

  // NavigateThroughPoses carries the complete ordered target list when its
  // action begins. The terminal target defines one fixed action-wide budget;
  // later RemovePassedGoals ticks must not reduce that budget mid-navigation.
  bool initialize(
    const OdomPlanarPose & start,
    const std::vector<OdomPlanarPose> & goals,
    double max_distance_ratio,
    double distance_slack_m,
    double max_odom_step_m)
  {
    if (goals.empty() || !std::all_of(goals.begin(), goals.end(), isFinite)) {
      reset();
      return false;
    }
    return initialize(
      start, goals.back(), max_distance_ratio, distance_slack_m, max_odom_step_m);
  }

  OdomDistanceBudgetUpdate update(const OdomPlanarPose & sample)
  {
    if (!initialized_ || !isFinite(sample)) {
      return OdomDistanceBudgetUpdate::kInvalidSample;
    }

    const double step_m = distance(last_pose_, sample);
    if (!std::isfinite(step_m) || step_m > max_odom_step_m_) {
      return OdomDistanceBudgetUpdate::kStepTooLarge;
    }

    travelled_m_ += step_m;
    last_pose_ = sample;
    if (!std::isfinite(travelled_m_)) {
      return OdomDistanceBudgetUpdate::kInvalidSample;
    }
    if (travelled_m_ >= budget_m_) {
      return OdomDistanceBudgetUpdate::kBudgetExceeded;
    }
    return OdomDistanceBudgetUpdate::kAccepted;
  }

  void reset()
  {
    initialized_ = false;
    last_pose_ = OdomPlanarPose{};
    travelled_m_ = 0.0;
    budget_m_ = 0.0;
    max_odom_step_m_ = 0.0;
  }

  bool initialized() const { return initialized_; }
  double travelled_m() const { return travelled_m_; }
  double budget_m() const { return budget_m_; }

  static bool isFinite(const OdomPlanarPose & pose)
  {
    return std::isfinite(pose.x) && std::isfinite(pose.y);
  }

private:
  static double distance(const OdomPlanarPose & first, const OdomPlanarPose & second)
  {
    return std::hypot(second.x - first.x, second.y - first.y);
  }

  bool initialized_{false};
  OdomPlanarPose last_pose_{};
  double travelled_m_{0.0};
  double budget_m_{0.0};
  double max_odom_step_m_{0.0};
};

}  // namespace smartcar_nav2

#endif  // SMARTCAR_NAV2__ODOM_DISTANCE_BUDGET_TRACKER_HPP_
