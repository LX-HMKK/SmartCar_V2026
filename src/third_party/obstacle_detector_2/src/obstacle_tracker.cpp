/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "obstacle_detector/obstacle_tracker.h"

using namespace std::chrono_literals;
using namespace obstacle_detector;
using namespace arma;
using namespace std;

ObstacleTracker::ObstacleTracker(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local){
  nh_ = nh;
  nh_local_ = nh_local;
  p_active_ = false;
//   params_srv_ = nh_->create_service<std_srvs::srv::Empty>("params", 
//                                                           std::bind(
//                                                                 &ObstacleTracker::updateParams,
//                                                                 this, 
//                                                                 std::placeholders::_1,
//                                                                 std::placeholders::_2,
//                                                                 std::placeholders::_3
//                                                           ));

  timer_ = nh_->create_wall_timer(1000ms, std::bind(&ObstacleTracker::timerCallback, this));
  initialize();
}

ObstacleTracker::~ObstacleTracker() {
}

void ObstacleTracker::updateParamsUtil(){
  bool prev_active = p_active_;

  nh_->declare_parameter("active", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("copy_segments", rclcpp::PARAMETER_BOOL);
  // Before using compensate_robot_velocity, consider using pointcloud input from map frame instead of base_link frame
  nh_->declare_parameter("compensate_robot_velocity", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("sensor_rate", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("loop_rate", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);

  nh_->declare_parameter("tracking_duration", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("min_correspondence_cost", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("std_correspondence_dev", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("process_variance", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("process_rate_variance", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("measurement_variance", rclcpp::PARAMETER_DOUBLE);

  nh_->get_parameter_or("active", p_active_, true);
  nh_->get_parameter_or("copy_segments", p_copy_segments_, true);
  nh_->get_parameter_or("compensate_robot_velocity", p_compensate_robot_velocity_, false);
  nh_->get_parameter_or("sensor_rate", p_sensor_rate_, 10.0);
  nh_->get_parameter_or("loop_rate", p_loop_rate_, 100.0);
  nh_->get_parameter_or("frame_id", p_frame_id_, string("map"));

  nh_->get_parameter_or("tracking_duration", p_tracking_duration_, 2.0);
  nh_->get_parameter_or("min_correspondence_cost", p_min_correspondence_cost_, 0.3);
  nh_->get_parameter_or("std_correspondence_dev", p_std_correspondence_dev_, 0.15);
  nh_->get_parameter_or("process_variance", p_process_variance_, 0.01);
  nh_->get_parameter_or("process_rate_variance", p_process_rate_variance_, 0.1);
  nh_->get_parameter_or("measurement_variance", p_measurement_variance_, 1.0);

  p_sampling_time_ = 1.0 / p_loop_rate_;
  obstacles_.header.frame_id = p_frame_id_;

  TrackedCircleObstacle::setSamplingTime(p_sampling_time_);
  TrackedCircleObstacle::setCounterSize(static_cast<int>(p_loop_rate_ * p_tracking_duration_));
  TrackedCircleObstacle::setCovariances(p_process_variance_, p_process_rate_variance_, p_measurement_variance_);
  TrackedSegmentObstacle::setSamplingTime(p_sampling_time_);
  TrackedSegmentObstacle::setCounterSize(static_cast<int>(p_loop_rate_ * p_tracking_duration_));
  TrackedSegmentObstacle::setCovariances(p_process_variance_, p_process_rate_variance_, p_measurement_variance_);

  timer_ = nh_->create_wall_timer(p_sampling_time_ * 1s, std::bind(&ObstacleTracker::timerCallback, this));

  if (p_active_ != prev_active) {
    if (p_active_) {
      if(p_compensate_robot_velocity_){
        odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ObstacleTracker::odomCallback, this, std::placeholders::_1));
      }
      obstacles_sub_ = nh_->create_subscription<obstacle_detector::msg::Obstacles>(
            "raw_obstacles", 10, std::bind(&ObstacleTracker::obstaclesCallback, this, std::placeholders::_1));
      obstacles_pub_ = nh_->create_publisher<obstacle_detector::msg::Obstacles>("tracked_obstacles", 10);
      obstacles_vis_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacles_visualization", 10);
    }
    else {
      // Send empty message
      auto obstacles_msg = obstacle_detector::msg::Obstacles();
      obstacles_msg.header.frame_id = obstacles_.header.frame_id;
      obstacles_msg.header.stamp = nh_->get_clock()->now();
      obstacles_pub_->publish(obstacles_msg);

      tracked_circle_obstacles_.clear();
      untracked_circle_obstacles_.clear();
      tracked_segment_obstacles_.clear();
      untracked_segment_obstacles_.clear();
    }
  }
}

void ObstacleTracker::updateParams(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<std_srvs::srv::Empty::Request> &req, 
                                     const std::shared_ptr<std_srvs::srv::Empty::Response> &res) {
  updateParamsUtil();
}

void ObstacleTracker::timerCallback() {
  updateObstacles();
  publishObstacles();
  publishVisualizationObstacles();
}

void ObstacleTracker::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  odom_ = *msg;
}

void ObstacleTracker::obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  obstaclesCallbackCircles(new_obstacles);
  obstaclesCallbackSegments(new_obstacles);
}

void ObstacleTracker::obstaclesCallbackCircles(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  if (new_obstacles->circles.size() > 0){
    radius_margin_ = new_obstacles->circles[0].radius - new_obstacles->circles[0].true_radius;
  }

  int N = new_obstacles->circles.size();
  int T = tracked_circle_obstacles_.size();
  int U = untracked_circle_obstacles_.size();

  if (T + U == 0) {
    untracked_circle_obstacles_.assign(new_obstacles->circles.begin(), new_obstacles->circles.end());
    return;
  }

  mat cost_matrix;
  calculateCostMatrix(new_obstacles->circles, cost_matrix);

  vector<int> row_min_indices;
  calculateRowMinIndices(cost_matrix, row_min_indices, T, U);

  vector<int> col_min_indices;
  calculateColMinIndices(cost_matrix, col_min_indices, T, U);

  vector<int> used_old_obstacles;
  vector<int> used_new_obstacles;

  vector<TrackedCircleObstacle> new_tracked_obstacles;
  vector<obstacle_detector::msg::CircleObstacle> new_untracked_obstacles;

  // Check for fusion (only tracked obstacles)
  for (int i = 0; i < T-1; ++i) {
    if (fusionObstacleUsed(i, col_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fusion_indices;
    fusion_indices.push_back(i);

    for (int j = i+1; j < T; ++j) {
      if (fusionObstaclesCorrespond(i, j, col_min_indices, used_old_obstacles))
        fusion_indices.push_back(j);
    }

    if (fusion_indices.size() > 1) {
      fuseObstacles(fusion_indices, col_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.insert(used_old_obstacles.end(), fusion_indices.begin(), fusion_indices.end());
      used_new_obstacles.push_back(col_min_indices[i]);
    }
  }

  // Check for fission (only tracked obstacles)
  for (int i = 0; i < N-1; ++i) {
    if (fissionObstacleUsed(i, T, row_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fission_indices;
    fission_indices.push_back(i);

    for (int j = i+1; j < N; ++j) {
      if (fissionObstaclesCorrespond(i, j, row_min_indices, used_new_obstacles))
        fission_indices.push_back(j);
    }

    if (fission_indices.size() > 1) {
      fissureObstacle(fission_indices, row_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.push_back(row_min_indices[i]);
      used_new_obstacles.insert(used_new_obstacles.end(), fission_indices.begin(), fission_indices.end());
    }
  }

  // Check for other possibilities
  for (int n = 0; n < N; ++n) {
    if (find(used_new_obstacles.begin(), used_new_obstacles.end(), n) != used_new_obstacles.end())
      continue;

    if (row_min_indices[n] == -1) {
      new_untracked_obstacles.push_back(new_obstacles->circles[n]);
    }
    else if (find(used_old_obstacles.begin(), used_old_obstacles.end(), row_min_indices[n]) == used_old_obstacles.end()) {
      if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
        tracked_circle_obstacles_[row_min_indices[n]].correctState(new_obstacles->circles[n]);
      }
      else if (row_min_indices[n] >= T) {
        TrackedCircleObstacle to(untracked_circle_obstacles_[row_min_indices[n] - T]);
        to.correctState(new_obstacles->circles[n]);
        for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
          to.updateState();

        new_tracked_obstacles.push_back(to);
      }

      used_new_obstacles.push_back(n);
    }
  }

  // Remove tracked obstacles that are no longer existent due to fusion or fission and insert new ones
  // Sort in descending order to remove from back of the list
  sort(used_old_obstacles.rbegin(), used_old_obstacles.rend());
  for (int idx : used_old_obstacles){
    tracked_circle_obstacles_.erase(tracked_circle_obstacles_.begin() + idx);
  }

  tracked_circle_obstacles_.insert(tracked_circle_obstacles_.end(), new_tracked_obstacles.begin(), new_tracked_obstacles.end());

  // Remove old untracked obstacles and save new ones
  untracked_circle_obstacles_.clear();
  untracked_circle_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());
}

void ObstacleTracker::obstaclesCallbackSegments(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  int N = new_obstacles->segments.size();
  int T = tracked_segment_obstacles_.size();
  int U = untracked_segment_obstacles_.size();

  if (T + U == 0) {
    untracked_segment_obstacles_.assign(new_obstacles->segments.begin(), new_obstacles->segments.end());
    return;
  }

  mat cost_matrix;
  calculateCostMatrix(new_obstacles->segments, cost_matrix);

  vector<int> row_min_indices;
  calculateRowMinIndices(cost_matrix, row_min_indices, T, U);

  vector<int> col_min_indices;
  calculateColMinIndices(cost_matrix, col_min_indices, T, U);

  vector<int> used_old_obstacles;
  vector<int> used_new_obstacles;

  vector<TrackedSegmentObstacle> new_tracked_obstacles;
  vector<obstacle_detector::msg::SegmentObstacle> new_untracked_obstacles;

  // Check for fusion (only tracked obstacles)
  for (int i = 0; i < T-1; ++i) {
    if (fusionObstacleUsed(i, col_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fusion_indices;
    fusion_indices.push_back(i);

    for (int j = i+1; j < T; ++j) {
      if (fusionObstaclesCorrespond(i, j, col_min_indices, used_old_obstacles))
        fusion_indices.push_back(j);
    }

    if (fusion_indices.size() > 1) {
      fuseObstacles(fusion_indices, col_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.insert(used_old_obstacles.end(), fusion_indices.begin(), fusion_indices.end());
      used_new_obstacles.push_back(col_min_indices[i]);
    }
  }

  // Check for fission (only tracked obstacles)
  for (int i = 0; i < N-1; ++i) {
    if (fissionObstacleUsed(i, T, row_min_indices, used_new_obstacles, used_old_obstacles))
      continue;

    vector<int> fission_indices;
    fission_indices.push_back(i);

    for (int j = i+1; j < N; ++j) {
      if (fissionObstaclesCorrespond(i, j, row_min_indices, used_new_obstacles))
        fission_indices.push_back(j);
    }

    if (fission_indices.size() > 1) {
      fissureObstacle(fission_indices, row_min_indices, new_tracked_obstacles, new_obstacles);

      // Mark used old and new obstacles
      used_old_obstacles.push_back(row_min_indices[i]);
      used_new_obstacles.insert(used_new_obstacles.end(), fission_indices.begin(), fission_indices.end());
    }
  }

  // Check for other possibilities
  for (int n = 0; n < N; ++n) {
    if (find(used_new_obstacles.begin(), used_new_obstacles.end(), n) != used_new_obstacles.end())
      continue;

    if (row_min_indices[n] == -1) {
      new_untracked_obstacles.push_back(new_obstacles->segments[n]);
    }
    else if (find(used_old_obstacles.begin(), used_old_obstacles.end(), row_min_indices[n]) == used_old_obstacles.end()) {
      if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
        tracked_segment_obstacles_[row_min_indices[n]].correctState(new_obstacles->segments[n]);
      }
      else if (row_min_indices[n] >= T) {
        TrackedSegmentObstacle to(untracked_segment_obstacles_[row_min_indices[n] - T]);
        to.correctState(new_obstacles->segments[n]);
        for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
          to.updateState();

        new_tracked_obstacles.push_back(to);
      }

      used_new_obstacles.push_back(n);
    }
  }

  // Remove tracked obstacles that are no longer existent due to fusion or fission and insert new ones
  // Sort in descending order to remove from back of the list
  sort(used_old_obstacles.rbegin(), used_old_obstacles.rend());
  for (int idx : used_old_obstacles)
    tracked_segment_obstacles_.erase(tracked_segment_obstacles_.begin() + idx);

  tracked_segment_obstacles_.insert(tracked_segment_obstacles_.end(), new_tracked_obstacles.begin(), new_tracked_obstacles.end());

  // Remove old untracked obstacles and save new ones
  untracked_segment_obstacles_.clear();
  untracked_segment_obstacles_.assign(new_untracked_obstacles.begin(), new_untracked_obstacles.end());
}

double ObstacleTracker::obstacleCostFunction(const obstacle_detector::msg::CircleObstacle& new_obstacle, const obstacle_detector::msg::CircleObstacle& old_obstacle) {
  mat distribution = mat(2, 2).zeros();
  vec relative_position = vec(2).zeros();

  double cost = 0.0;
  double penalty = 1.0;
  double tp = 1.0 / p_sensor_rate_;

  double direction = atan2(old_obstacle.velocity.y, old_obstacle.velocity.x);

  geometry_msgs::msg::Point new_center = transformPoint(new_obstacle.center, 0.0, 0.0, -direction);
  geometry_msgs::msg::Point old_center = transformPoint(old_obstacle.center, 0.0, 0.0, -direction);

  distribution(0, 0) = pow(p_std_correspondence_dev_, 2.0) + squaredLength(old_obstacle.velocity) * pow(tp, 2.0);
  distribution(1, 1) = pow(p_std_correspondence_dev_, 2.0);

  relative_position(0) = new_center.x - old_center.x - tp * length(old_obstacle.velocity);
  relative_position(1) = new_center.y - old_center.y;

  cost = sqrt(pow(new_obstacle.center.x - old_obstacle.center.x, 2.0) + pow(new_obstacle.center.y - old_obstacle.center.y, 2.0) + pow(new_obstacle.radius - old_obstacle.radius, 2.0));

  mat a = -0.5 * trans(relative_position) * distribution * relative_position;
  penalty = exp(a(0, 0));

  // TODO: Check values for cost/penalty in common situations
  // return cost / penalty;
  return cost / 1.0;
}

double ObstacleTracker::obstacleCostFunction(const obstacle_detector::msg::SegmentObstacle& new_obstacle, const obstacle_detector::msg::SegmentObstacle& old_obstacle) {
  return sqrt(pow(new_obstacle.first_point.x - old_obstacle.first_point.x, 2.0) + pow(new_obstacle.first_point.y - old_obstacle.first_point.y, 2.0) + pow(new_obstacle.last_point.x - old_obstacle.last_point.x, 2.0) + pow(new_obstacle.last_point.y - old_obstacle.last_point.y, 2.0));
}

void ObstacleTracker::calculateCostMatrix(const vector<obstacle_detector::msg::CircleObstacle>& new_obstacles, mat& cost_matrix) {
  /*
   * Cost between two obstacles represents their difference.
   * The bigger the cost, the less similar they are.
   * N rows of cost_matrix represent new obstacles.
   * T+U columns of cost matrix represent old tracked and untracked obstacles.
   */
  int N = new_obstacles.size();
  int T = tracked_circle_obstacles_.size();
  int U = untracked_circle_obstacles_.size();

  cost_matrix = mat(N, T + U, fill::zeros);

  for (int n = 0; n < N; ++n) {
    for (int t = 0; t < T; ++t)
      cost_matrix(n, t) = obstacleCostFunction(new_obstacles[n], tracked_circle_obstacles_[t].getObstacle());

    for (int u = 0; u < U; ++u)
      cost_matrix(n, u + T) = obstacleCostFunction(new_obstacles[n], untracked_circle_obstacles_[u]);
  }
}

void ObstacleTracker::calculateCostMatrix(const vector<obstacle_detector::msg::SegmentObstacle>& new_obstacles, mat& cost_matrix) {
  /*
   * Cost between two obstacles represents their difference.
   * The bigger the cost, the less similar they are.
   * N rows of cost_matrix represent new obstacles.
   * T+U columns of cost matrix represent old tracked and untracked obstacles.
   */
  int N = new_obstacles.size();
  int T = tracked_segment_obstacles_.size();
  int U = untracked_segment_obstacles_.size();

  cost_matrix = mat(N, T + U, fill::zeros);

  for (int n = 0; n < N; ++n) {
    for (int t = 0; t < T; ++t)
      cost_matrix(n, t) = obstacleCostFunction(new_obstacles[n], tracked_segment_obstacles_[t].getObstacle());

    for (int u = 0; u < U; ++u)
      cost_matrix(n, u + T) = obstacleCostFunction(new_obstacles[n], untracked_segment_obstacles_[u]);
  }
}

void ObstacleTracker::calculateRowMinIndices(const mat& cost_matrix, vector<int>& row_min_indices, const int T, const int U) {
  /*
   * Vector of row minimal indices keeps the indices of old obstacles (tracked and untracked)
   * that have the minimum cost related to each of new obstacles, i.e. row_min_indices[n]
   * keeps the index of old obstacle that has the minimum cost with n-th new obstacle.
   */
  int N = cost_matrix.n_rows;

  row_min_indices.assign(N, -1); // Minimum index -1 means no correspondence has been found

  for (int n = 0; n < N; ++n) {
    double min_cost = p_min_correspondence_cost_;

    for (int t = 0; t < T; ++t) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        row_min_indices[n] = t;
      }
    }

    for (int u = 0; u < U; ++u) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        row_min_indices[n] = u + T;
      }
    }
  }
}

void ObstacleTracker::calculateColMinIndices(const mat& cost_matrix, vector<int>& col_min_indices, const int T, const int U) {
  /*
   * Vector of column minimal indices keeps the indices of new obstacles that has the minimum
   * cost related to each of old (tracked and untracked) obstacles, i.e. col_min_indices[i]
   * keeps the index of new obstacle that has the minimum cost with i-th old obstacle.
   */
  int N = cost_matrix.n_rows;

  col_min_indices.assign(T + U, -1); // Minimum index -1 means no correspondence has been found

  for (int t = 0; t < T; ++t) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, t) < min_cost) {
        min_cost = cost_matrix(n, t);
        col_min_indices[t] = n;
      }
    }
  }

  for (int u = 0; u < U; ++u) {
    double min_cost = p_min_correspondence_cost_;

    for (int n = 0; n < N; ++n) {
      if (cost_matrix(n, u + T) < min_cost) {
        min_cost = cost_matrix(n, u + T);
        col_min_indices[u + T] = n;
      }
    }
  }
}

bool ObstacleTracker::fusionObstacleUsed(const int idx, const vector<int> &col_min_indices, const vector<int> &used_new, const vector<int> &used_old) {
  /*
   * This function returns true if:
   * - idx-th old obstacle was already used
   * - obstacle to which idx-th old obstacle corresponds was already used
   * - there is no corresponding obstacle
   */

  return (find(used_old.begin(), used_old.end(), idx) != used_old.end() ||
          find(used_new.begin(), used_new.end(), col_min_indices[idx]) != used_new.end() ||
          col_min_indices[idx] < 0);
}

bool ObstacleTracker::fusionObstaclesCorrespond(const int idx, const int jdx, const vector<int>& col_min_indices, const vector<int>& used_old) {
  /*
   * This function returns true if:
   * - both old obstacles correspond to the same new obstacle
   * - jdx-th old obstacle was not yet used
   */

  return (col_min_indices[idx] == col_min_indices[jdx] &&
          find(used_old.begin(), used_old.end(), jdx) == used_old.end());
}

bool ObstacleTracker::fissionObstacleUsed(const int idx, const int T, const vector<int>& row_min_indices, const vector<int>& used_new, const vector<int>& used_old) {
  /*
   * This function returns true if:
   * - idx-th new obstacle was already used
   * - obstacle to which idx-th new obstacle corresponds was already used
   * - there is no corresponding obstacle
   * - obstacle to which idx-th new obstacle corresponds is untracked
   */

  return (find(used_new.begin(), used_new.end(), idx) != used_new.end() ||
          find(used_old.begin(), used_old.end(), row_min_indices[idx]) != used_old.end() ||
          row_min_indices[idx] < 0 ||
          row_min_indices[idx] >= T);
}

bool ObstacleTracker::fissionObstaclesCorrespond(const int idx, const int jdx, const vector<int>& row_min_indices, const vector<int>& used_new) {
  /*
   * This function returns true if:
   * - both new obstacles correspond to the same old obstacle
   * - jdx-th new obstacle was not yet used
   */

  return (row_min_indices[idx] == row_min_indices[jdx] &&
          find(used_new.begin(), used_new.end(), jdx) == used_new.end());
}

void ObstacleTracker::fuseObstacles(const vector<int>& fusion_indices, const vector<int> &col_min_indices,
                                    vector<TrackedCircleObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  obstacle_detector::msg::CircleObstacle c;

  double sum_var_x  = 0.0;
  double sum_var_y  = 0.0;
  double sum_var_vx = 0.0;
  double sum_var_vy = 0.0;
  double sum_var_r  = 0.0;

  for (int idx : fusion_indices) {
    c.center.x += tracked_circle_obstacles_[idx].getObstacle().center.x / tracked_circle_obstacles_[idx].getKFx().P(0,0);
    c.center.y += tracked_circle_obstacles_[idx].getObstacle().center.y / tracked_circle_obstacles_[idx].getKFy().P(0,0);
    // assume z coordinate of obstacles never changes
    c.center.z = tracked_circle_obstacles_[idx].getObstacle().center.z;
    c.velocity.x += tracked_circle_obstacles_[idx].getObstacle().velocity.x / tracked_circle_obstacles_[idx].getKFx().P(1,1);
    c.velocity.y += tracked_circle_obstacles_[idx].getObstacle().velocity.y / tracked_circle_obstacles_[idx].getKFy().P(1,1);
    c.radius += tracked_circle_obstacles_[idx].getObstacle().radius / tracked_circle_obstacles_[idx].getKFr().P(0,0);

    sum_var_x += 1.0 / tracked_circle_obstacles_[idx].getKFx().P(0,0);
    sum_var_y += 1.0 / tracked_circle_obstacles_[idx].getKFy().P(0,0);
    sum_var_vx += 1.0 / tracked_circle_obstacles_[idx].getKFx().P(1,1);
    sum_var_vy += 1.0 / tracked_circle_obstacles_[idx].getKFy().P(1,1);
    sum_var_r += 1.0 / tracked_circle_obstacles_[idx].getKFr().P(0,0);
  }

  c.center.x /= sum_var_x;
  c.center.y /= sum_var_y;
  c.velocity.x /= sum_var_vx;
  c.velocity.y /= sum_var_vy;
  c.radius /= sum_var_r;

  TrackedCircleObstacle to(c);
  to.correctState(new_obstacles->circles[col_min_indices[fusion_indices.front()]]);
  for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
    to.updateState();

  new_tracked.push_back(to);
}

void ObstacleTracker::fuseObstacles(const vector<int>& fusion_indices, const vector<int> &col_min_indices,
                                    vector<TrackedSegmentObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  obstacle_detector::msg::SegmentObstacle c;

  double sum_var_x1  = 0.0;
  double sum_var_y1  = 0.0;
  double sum_var_x2  = 0.0;
  double sum_var_y2  = 0.0;
  double sum_var_vx1 = 0.0;
  double sum_var_vy1 = 0.0;
  double sum_var_vx2 = 0.0;
  double sum_var_vy2 = 0.0;

  for (int idx : fusion_indices) {
    c.first_point.x += tracked_segment_obstacles_[idx].getObstacle().first_point.x / tracked_segment_obstacles_[idx].getKFx1().P(0,0);
    c.first_point.y += tracked_segment_obstacles_[idx].getObstacle().first_point.y / tracked_segment_obstacles_[idx].getKFy1().P(0,0);
    // assume z coordinate of obstacles never changes
    c.first_point.z = tracked_segment_obstacles_[idx].getObstacle().first_point.z;
    c.last_point.x += tracked_segment_obstacles_[idx].getObstacle().last_point.x / tracked_segment_obstacles_[idx].getKFx2().P(0,0);
    c.last_point.y += tracked_segment_obstacles_[idx].getObstacle().last_point.y / tracked_segment_obstacles_[idx].getKFy2().P(0,0);
    c.last_point.z = tracked_segment_obstacles_[idx].getObstacle().last_point.z;
    c.first_velocity.x += tracked_segment_obstacles_[idx].getObstacle().first_velocity.x / tracked_segment_obstacles_[idx].getKFx1().P(1,1);
    c.first_velocity.y += tracked_segment_obstacles_[idx].getObstacle().first_velocity.y / tracked_segment_obstacles_[idx].getKFy1().P(1,1);
    c.last_velocity.x += tracked_segment_obstacles_[idx].getObstacle().last_velocity.x / tracked_segment_obstacles_[idx].getKFx2().P(1,1);
    c.last_velocity.y += tracked_segment_obstacles_[idx].getObstacle().last_velocity.y / tracked_segment_obstacles_[idx].getKFy2().P(1,1);

    sum_var_x1 += 1.0 / tracked_segment_obstacles_[idx].getKFx1().P(0,0);
    sum_var_y1 += 1.0 / tracked_segment_obstacles_[idx].getKFy1().P(0,0);
    sum_var_x2 += 1.0 / tracked_segment_obstacles_[idx].getKFx2().P(0,0);
    sum_var_y2 += 1.0 / tracked_segment_obstacles_[idx].getKFy2().P(0,0);
    sum_var_vx1 += 1.0 / tracked_segment_obstacles_[idx].getKFx1().P(1,1);
    sum_var_vy1 += 1.0 / tracked_segment_obstacles_[idx].getKFy1().P(1,1);
    sum_var_vx2 += 1.0 / tracked_segment_obstacles_[idx].getKFx2().P(1,1);
    sum_var_vy2 += 1.0 / tracked_segment_obstacles_[idx].getKFy2().P(1,1);
  }

  c.first_point.x /= sum_var_x1;
  c.first_point.y /= sum_var_y1;
  c.last_point.x /= sum_var_x2;
  c.last_point.y /= sum_var_y2;
  c.first_velocity.x /= sum_var_vx1;
  c.first_velocity.y /= sum_var_vy1;
  c.last_velocity.x /= sum_var_vx2;
  c.last_velocity.y /= sum_var_vy2;

  TrackedSegmentObstacle to(c);
  to.correctState(new_obstacles->segments[col_min_indices[fusion_indices.front()]]);
  for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
    to.updateState();

  new_tracked.push_back(to);
}

void ObstacleTracker::fissureObstacle(const vector<int>& fission_indices, const vector<int>& row_min_indices,
                                      vector<TrackedCircleObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  // For each new obstacle taking part in fission create a tracked obstacle from the original old one and update it with the new one
  for (int idx : fission_indices) {
    TrackedCircleObstacle to = tracked_circle_obstacles_[row_min_indices[idx]];
    // Ensure the fissured obstacle gets a new id
    to.setNewUid();
    to.correctState(new_obstacles->circles[idx]);
    for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
      to.updateState();

    new_tracked.push_back(to);
  }
}

void ObstacleTracker::fissureObstacle(const vector<int>& fission_indices, const vector<int>& row_min_indices,
                                      vector<TrackedSegmentObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  // For each new obstacle taking part in fission create a tracked obstacle from the original old one and update it with the new one
  for (int idx : fission_indices) {
    TrackedSegmentObstacle to = tracked_segment_obstacles_[row_min_indices[idx]];
    // Ensure the fissured obstacle gets a new id
    to.setNewUid();
    to.correctState(new_obstacles->segments[idx]);
    for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
      to.updateState();

    new_tracked.push_back(to);
  }
}

void ObstacleTracker::updateObstacles() {
  for (int i = 0; i < tracked_circle_obstacles_.size(); ++i) {
    if (!tracked_circle_obstacles_[i].hasFaded())
      tracked_circle_obstacles_[i].updateState();
    else
      tracked_circle_obstacles_.erase(tracked_circle_obstacles_.begin() + i--);
  }

  for (int i = 0; i < tracked_segment_obstacles_.size(); ++i) {
    if (!tracked_segment_obstacles_[i].hasFaded())
      tracked_segment_obstacles_[i].updateState();
    else
      tracked_segment_obstacles_.erase(tracked_segment_obstacles_.begin() + i--);
  }
}

void ObstacleTracker::publishObstacles() {
  auto obstacles_msg = obstacle_detector::msg::Obstacles();

  obstacles_.circles.clear();
  obstacles_.segments.clear();

  for (auto& tracked_circle_obstacle : tracked_circle_obstacles_) {
    obstacle_detector::msg::CircleObstacle ob = tracked_circle_obstacle.getObstacle();
    ob.true_radius = ob.radius - radius_margin_;
    // Compensate robot velocity from obstacle velocity
    // Velocities are in robot's frame, x forward y leftwards
    if (p_compensate_robot_velocity_)
    {
      double distance = sqrt(pow(ob.center.x, 2) + pow(ob.center.y, 2));
      double angle = atan2(ob.center.y, ob.center.x);
      ob.velocity.x += odom_.twist.twist.linear.x - odom_.twist.twist.angular.z * distance * sin(angle);
      ob.velocity.y += odom_.twist.twist.linear.y + odom_.twist.twist.angular.z * distance * cos(angle);
    }
    obstacles_.circles.push_back(ob);
  }
  for (auto& tracked_segment_obstacle : tracked_segment_obstacles_) {
    obstacle_detector::msg::SegmentObstacle ob = tracked_segment_obstacle.getObstacle();
    // Compensate robot velocity from obstacle velocity
    // Velocities are in robot's frame, x forward y leftwards
    if (p_compensate_robot_velocity_)
    {
      double distance_first = sqrt(pow(ob.first_point.x, 2) + pow(ob.first_point.y, 2));
      double distance_last = sqrt(pow(ob.last_point.x, 2) + pow(ob.last_point.y, 2));
      double angle_first = atan2(ob.first_point.y, ob.first_point.x);
      double angle_last = atan2(ob.last_point.y, ob.last_point.x);
      ob.first_velocity.x += odom_.twist.twist.linear.x - odom_.twist.twist.angular.z * distance_first * sin(angle_first);
      ob.first_velocity.y += odom_.twist.twist.linear.y + odom_.twist.twist.angular.z * distance_first * cos(angle_first);
      ob.last_velocity.x += odom_.twist.twist.linear.x - odom_.twist.twist.angular.z * distance_last * sin(angle_last);
      ob.last_velocity.y += odom_.twist.twist.linear.y + odom_.twist.twist.angular.z * distance_last * cos(angle_last);
    }
    obstacles_.segments.push_back(ob);
  }

  obstacles_msg = obstacles_;
  obstacles_msg.header.stamp = nh_->get_clock()->now();

  obstacles_pub_->publish(obstacles_msg);
}


visualization_msgs::msg::Marker ObstacleTracker::getMarkerBase(uid_t uid){
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = nh_->get_clock()->now();
  marker.header.frame_id = obstacles_.header.frame_id;
  marker.id = uid;
  marker.lifetime = rclcpp::Duration(p_sampling_time_ * 1s);

  marker.color.r = ((int)((uid + 1) * 0.2799979960764232 * 10000)%10) * 0.1;
  marker.color.g = ((int)((uid + 1) * 0.18779357508452654 * 10000)%10) * 0.1;
  marker.color.b = ((int)((uid + 1) * 0.5754993762271469 * 10000)%10) * 0.1;
  marker.color.a = 1.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.action = visualization_msgs::msg::Marker::ADD;
  return marker;
}

visualization_msgs::msg::Marker ObstacleTracker::getMarkerCircle(obstacle_detector::msg::CircleObstacle& ob){
    auto circ_marker = getMarkerBase(ob.uid);
    circ_marker.ns = "tracked_obstacles_blobs";
    auto scale = ob.true_radius;
    // debug: min scale for better visualization
    if (scale < 0.2) scale = 0.2;
    circ_marker.scale.x = circ_marker.scale.y = scale;
    circ_marker.scale.z = 0.01;
    circ_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    
    circ_marker.pose.position.x = ob.center.x;
    circ_marker.pose.position.y = ob.center.y;
    // tracked obstacles are a bit higher than the raw ones, so that they can be visualized together
    circ_marker.pose.position.z = ob.center.z + 0.1;
    return circ_marker;
}

visualization_msgs::msg::Marker ObstacleTracker::getMarkerVelocityArrow(uid_t uid, 
                                                                        double px,
                                                                        double py,
                                                                        double vx,
                                                                        double vy){
    auto arrow_marker = getMarkerBase(uid);
    arrow_marker.ns = "tracked_obstacles_velocities";
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.scale.y = 0.07;  // width
    arrow_marker.scale.z = 0.01; // height
    arrow_marker.color.r = arrow_marker.color.g = arrow_marker.color.b = arrow_marker.color.a = 1.0;
    
    arrow_marker.pose.position.x = px;
    arrow_marker.pose.position.y = py;
    double vel_magnitude = std::sqrt(vx*vx + vy*vy);
    double alpha = std::atan2(vy, vx);
    // assume rotation around the z-axis (0, 0, 1)
    arrow_marker.scale.x = vel_magnitude;
    arrow_marker.pose.orientation.z = std::sin(alpha/2);
    arrow_marker.pose.orientation.w = std::cos(alpha/2);
    return arrow_marker; 
}

visualization_msgs::msg::Marker ObstacleTracker::getMarkerSegment(obstacle_detector::msg::SegmentObstacle& ob){
    auto seg_marker = getMarkerBase(ob.uid);
    seg_marker.ns = "tracked_obstacles_blobs";
    seg_marker.scale.x = 0.1;
    seg_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    auto seg_fp = geometry_msgs::msg::Point();
    seg_fp.x = ob.first_point.x;
    seg_fp.y = ob.first_point.y;
    seg_fp.z = ob.first_point.z + 0.1;
    auto seg_lp = geometry_msgs::msg::Point();
    seg_lp.x = ob.last_point.x;
    seg_lp.y = ob.last_point.y;
    seg_lp.z = ob.last_point.z + 0.1;
    seg_marker.points.push_back(seg_fp);
    seg_marker.points.push_back(seg_lp);
    return seg_marker;
}

visualization_msgs::msg::Marker ObstacleTracker::getMarkerText(uid_t uid){
    auto text_marker = getMarkerBase(uid);
    text_marker.ns = "tracked_obstacles_ids";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.text = std::to_string(uid);
    text_marker.scale.x = text_marker.scale.y = text_marker.scale.z = 0.2;
    text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0;
    // text id is above marker
    text_marker.pose.position.z = 0.2;
    return text_marker;
}

void ObstacleTracker::publishVisualizationObstacles() {
  auto obstacles_vis_msg = visualization_msgs::msg::MarkerArray();

  //clean up previous markers
  auto marker_d = getMarkerBase(0);
  marker_d.action = visualization_msgs::msg::Marker::DELETEALL;
  obstacles_vis_msg.markers.push_back(marker_d);

  for (auto& tracked_circle_obstacle : tracked_circle_obstacles_) {
    obstacle_detector::msg::CircleObstacle ob = tracked_circle_obstacle.getObstacle();
    auto circ_marker = getMarkerCircle(ob);
    auto text_marker = getMarkerText(ob.uid);
    text_marker.pose.position.z += ob.center.z;
    text_marker.pose.position.x = ob.center.x;
    text_marker.pose.position.y = ob.center.y;
    auto arrow_vel = getMarkerVelocityArrow(ob.uid, ob.center.x, ob.center.y, ob.velocity.x, ob.velocity.y);
    arrow_vel.pose.position.z += ob.center.z;
    obstacles_vis_msg.markers.push_back(circ_marker);
    obstacles_vis_msg.markers.push_back(text_marker);
    obstacles_vis_msg.markers.push_back(arrow_vel);
  }

  for (auto& tracked_segment_obstacle : tracked_segment_obstacles_) {
    obstacle_detector::msg::SegmentObstacle ob = tracked_segment_obstacle.getObstacle();
    auto seg_marker = getMarkerSegment(ob);
    auto text_marker = getMarkerText(ob.uid);
    double px = (ob.first_point.x + ob.last_point.x)/2.0;
    double py = (ob.first_point.y + ob.last_point.y)/2.0;
    double vx = (ob.first_velocity.x + ob.last_velocity.x)/2.0;
    double vy = (ob.first_velocity.y + ob.last_velocity.y)/2.0;
    text_marker.pose.position.z += ob.first_point.z;
    text_marker.pose.position.x = px;
    text_marker.pose.position.y = py;
    auto arrow_vel = getMarkerVelocityArrow(ob.uid, px, py, vx, vy);
    arrow_vel.pose.position.z += ob.first_point.z;
    obstacles_vis_msg.markers.push_back(seg_marker);
    obstacles_vis_msg.markers.push_back(text_marker);
  }
  obstacles_vis_pub_->publish(obstacles_vis_msg);
}

// Ugly initialization of static members of tracked obstacles...
int    TrackedCircleObstacle::s_fade_counter_size_     = 0;
double TrackedCircleObstacle::s_sampling_time_         = 100.0;
double TrackedCircleObstacle::s_process_variance_      = 0.0;
double TrackedCircleObstacle::s_process_rate_variance_ = 0.0;
double TrackedCircleObstacle::s_measurement_variance_  = 0.0;
int    TrackedSegmentObstacle::s_fade_counter_size_     = 0;
double TrackedSegmentObstacle::s_sampling_time_         = 100.0;
double TrackedSegmentObstacle::s_process_variance_      = 0.0;
double TrackedSegmentObstacle::s_process_rate_variance_ = 0.0;
double TrackedSegmentObstacle::s_measurement_variance_  = 0.0;
