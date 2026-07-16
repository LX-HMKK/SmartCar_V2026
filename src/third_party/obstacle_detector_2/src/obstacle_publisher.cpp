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

#include "obstacle_detector/obstacle_publisher.h"

using namespace std;
using namespace obstacle_detector;

ObstaclePublisher::ObstaclePublisher(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local) {
    nh_ = nh;
  nh_local_ = nh_local;
  p_active_ = false;
//   params_srv_ = nh_->create_service<std_srvs::srv::Empty>("params", 
//                                                           std::bind(
//                                                                 &ObstaclePublisher::updateParams,
//                                                                 this, 
//                                                                 std::placeholders::_1,
//                                                                 std::placeholders::_2,
//                                                                 std::placeholders::_3
//                                                           ));

  t_ = 0.0;
  timer_ = nh_->create_wall_timer(1000ms, std::bind(&ObstaclePublisher::timerCallback, this));
  initialize();
}

ObstaclePublisher::~ObstaclePublisher() {
}


void ObstaclePublisher::updateParamsUtil() {
  bool prev_active = p_active_;

  nh_->declare_parameter("active", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("reset", rclcpp::PARAMETER_BOOL);
  // Before using compensate_robot_velocity, consider using pointcloud input from map frame instead of base_link frame
  nh_->declare_parameter("fusion_example", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("fission_example", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("loop_rate", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("radius_margin", rclcpp::PARAMETER_DOUBLE);

  nh_->get_parameter_or("active", p_active_, true);
  nh_->get_parameter_or("reset", p_reset_, false);
  nh_->get_parameter_or("fusion_example", p_fusion_example_, false);
  nh_->get_parameter_or("fission_example", p_fission_example_, false);
  nh_->get_parameter_or("loop_rate", p_loop_rate_, 10.0);
  nh_->get_parameter_or("radius_margin", p_radius_margin_, 0.25);

  nh_->declare_parameter("x_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);
  nh_->declare_parameter("y_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);
  nh_->declare_parameter("r_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);
  nh_->declare_parameter("vx_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);
  nh_->declare_parameter("vy_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);

  nh_->get_parameter("x_vector", p_x_vector_);
  nh_->get_parameter("y_vector", p_y_vector_);
  nh_->get_parameter("r_vector", p_r_vector_);
  nh_->get_parameter("vx_vector", p_vx_vector_);
  nh_->get_parameter("vy_vector", p_vy_vector_);

  nh_->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
  nh_->get_parameter_or("frame_id", p_frame_id_, string("map"));

  p_sampling_time_ = 1.0 / p_loop_rate_;
  timer_ = nh_->create_wall_timer(p_sampling_time_ * 1s, std::bind(&ObstaclePublisher::timerCallback, this));

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacles_pub_ = nh_->create_publisher<obstacle_detector::msg::Obstacles>("obstacles", 10);
    }
    // else {
    //   obstacle_pub_.shutdown();
    //   timer_.stop();
    // }
  }

  obstacles_.header.frame_id = p_frame_id_;
  obstacles_.circles.clear();

  if (p_x_vector_.size() != p_y_vector_.size() || p_x_vector_.size() != p_r_vector_.size() ||
      p_x_vector_.size() != p_vx_vector_.size() || p_x_vector_.size() != p_vy_vector_.size())
    return;

  for (int idx = 0; idx < p_x_vector_.size(); ++idx) {
    obstacle_detector::msg::CircleObstacle circle;
    circle.center.x = p_x_vector_[idx];
    circle.center.y = p_y_vector_[idx];
    circle.radius = p_r_vector_[idx];
    circle.true_radius = p_r_vector_[idx] - p_radius_margin_;;

    circle.velocity.x = p_vx_vector_[idx];
    circle.velocity.y = p_vy_vector_[idx];

    obstacles_.circles.push_back(circle);
  }

  if (p_reset_)
    reset();
}

void ObstaclePublisher::updateParams(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<std_srvs::srv::Empty::Request> &req, 
                                     const std::shared_ptr<std_srvs::srv::Empty::Response> &res) {
  updateParamsUtil();
}

void ObstaclePublisher::timerCallback() {
  t_ += p_sampling_time_;

  calculateObstaclesPositions(p_sampling_time_);

  if (p_fusion_example_)
    fusionExample(t_);
  else if (p_fission_example_)
    fissionExample(t_);

  if (obstacles_.circles.size() > 0)
    publishObstacles();
}

void ObstaclePublisher::calculateObstaclesPositions(double dt) {
  for (auto& circ : obstacles_.circles) {
    circ.center.x += circ.velocity.x * dt;
    circ.center.y += circ.velocity.y * dt;
  }
}

void ObstaclePublisher::fusionExample(double t) {
  obstacle_detector::msg::CircleObstacle circ1, circ2;

  obstacles_.circles.clear();

  if (t < 5.0) {
    circ1.center.x = -1.20 + 0.2 * t;
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    circ2.center.x = 1.20 - 0.2 * t;
    circ2.center.y = 0.0;
    circ2.radius = 0.20;

    obstacles_.circles.push_back(circ1);
    obstacles_.circles.push_back(circ2);
  }
  else if (t < 15.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20 + 0.20 * exp(-(t - 5.0) / 1.0);

    obstacles_.circles.push_back(circ1);
  }
  else  if (t > 20.0)
    reset();

  circ1.true_radius = circ1.radius;
  circ2.true_radius = circ2.radius;
}

void ObstaclePublisher::fissionExample(double t) {
  obstacle_detector::msg::CircleObstacle circ1, circ2;

  obstacles_.circles.clear();

  if (t < 5.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    obstacles_.circles.push_back(circ1);
  }
  else if (t < 6.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20 + 0.20 * (1.0 - exp(-(t - 5.0) / 1.0));

    obstacles_.circles.push_back(circ1);
  }
  else if (t < 16.0){
    circ1.center.x = -0.20 - 0.2 * (t - 6.0);
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    circ2.center.x = 0.20 + 0.2 * (t - 6.0);
    circ2.center.y = 0.0;
    circ2.radius = 0.20;

    obstacles_.circles.push_back(circ1);
    obstacles_.circles.push_back(circ2);
  }
  else if (t > 20.0)
    reset();

  circ1.true_radius = circ1.radius;
  circ2.true_radius = circ2.radius;
}

void ObstaclePublisher::publishObstacles() {
  obstacle_detector::msg::Obstacles  obstacles_msg;
  obstacles_msg = obstacles_;

  obstacles_msg.header.stamp = nh_->get_clock()->now();
  obstacles_pub_->publish(obstacles_msg);
}

void ObstaclePublisher::reset() {
  t_ = 0.0;
  p_reset_ = false;
//   nh_local_.setParam("reset", false);
}
