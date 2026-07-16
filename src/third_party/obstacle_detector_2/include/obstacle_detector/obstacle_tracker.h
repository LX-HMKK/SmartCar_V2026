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

#pragma once

#include <list>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <armadillo>
#include "std_srvs/srv/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

#include "obstacle_detector/utilities/tracked_circle_obstacle.h"
#include "obstacle_detector/utilities/tracked_segment_obstacle.h"
#include "obstacle_detector/utilities/math_utilities.h"

namespace obstacle_detector
{

class ObstacleTracker {
public:
  ObstacleTracker(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local);
  ~ObstacleTracker();

private:
  void updateParamsUtil();
  void updateParams(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<std_srvs::srv::Empty::Request> &req, const std::shared_ptr<std_srvs::srv::Empty::Response> &res);
  void timerCallback();
  void obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);
  void obstaclesCallbackCircles(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);
  void obstaclesCallbackSegments(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void initialize() { std_srvs::srv::Empty empt; updateParamsUtil(); }

  double obstacleCostFunction(const obstacle_detector::msg::CircleObstacle& new_obstacle, const obstacle_detector::msg::CircleObstacle& old_obstacle);
  double obstacleCostFunction(const obstacle_detector::msg::SegmentObstacle& new_obstacle, const obstacle_detector::msg::SegmentObstacle& old_obstacle);
  void calculateCostMatrix(const std::vector<obstacle_detector::msg::CircleObstacle>& new_obstacles, arma::mat& cost_matrix);
  void calculateCostMatrix(const std::vector<obstacle_detector::msg::SegmentObstacle>& new_obstacles, arma::mat& cost_matrix);
  void calculateRowMinIndices(const arma::mat& cost_matrix, std::vector<int>& row_min_indices, const int T, const int U);
  void calculateColMinIndices(const arma::mat& cost_matrix, std::vector<int>& col_min_indices, const int T, const int U);

  bool fusionObstacleUsed(const int idx, const std::vector<int>& col_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  bool fusionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& col_min_indices, const std::vector<int>& used_old);
  bool fissionObstacleUsed(const int idx, const int T, const std::vector<int>& row_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  bool fissionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new);

  void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
                     std::vector<TrackedCircleObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);
  void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
                       std::vector<TrackedCircleObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);
  void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
                     std::vector<TrackedSegmentObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);
  void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
                       std::vector<TrackedSegmentObstacle>& new_tracked, const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles);

  void updateObstacles();
  void publishObstacles();
  void publishVisualizationObstacles();
  visualization_msgs::msg::Marker getMarkerBase(uid_t uid);
  visualization_msgs::msg::Marker getMarkerCircle(obstacle_detector::msg::CircleObstacle& ob);
  visualization_msgs::msg::Marker getMarkerSegment(obstacle_detector::msg::SegmentObstacle& ob);
  visualization_msgs::msg::Marker getMarkerText(uid_t uid);
  visualization_msgs::msg::Marker getMarkerVelocityArrow(uid_t uid, double px, double py, double vx, double vy);

  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<rclcpp::Node> nh_local_;

  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_vis_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr params_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  double radius_margin_;
  obstacle_detector::msg::Obstacles obstacles_;
  nav_msgs::msg::Odometry odom_;

  std::vector<TrackedCircleObstacle> tracked_circle_obstacles_;
  std::vector<obstacle_detector::msg::CircleObstacle> untracked_circle_obstacles_;
  std::vector<TrackedSegmentObstacle> tracked_segment_obstacles_;
  std::vector<obstacle_detector::msg::SegmentObstacle> untracked_segment_obstacles_;

  // Parameters
  bool p_active_;
  bool p_copy_segments_;
  bool p_compensate_robot_velocity_;

  double p_tracking_duration_;
  double p_loop_rate_;
  double p_sampling_time_;
  double p_sensor_rate_;
  double p_min_correspondence_cost_;
  double p_std_correspondence_dev_;
  double p_process_variance_;
  double p_process_rate_variance_;
  double p_measurement_variance_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
