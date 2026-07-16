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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

namespace obstacle_detector
{

class ObstaclePublisher
{
public:
  ObstaclePublisher(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local);
  ~ObstaclePublisher();

private:
  void updateParamsUtil();
  void updateParams(const std::shared_ptr<rmw_request_id_t> request_header, 
                    const std::shared_ptr<std_srvs::srv::Empty::Request> &req, 
                    const std::shared_ptr<std_srvs::srv::Empty::Response> &res);
  void timerCallback();

  void initialize() { std_srvs::srv::Empty empt; updateParamsUtil(); }

  void calculateObstaclesPositions(double dt);
  void fusionExample(double t);
  void fissionExample(double t);
  void publishObstacles();
  void reset();

  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<rclcpp::Node> nh_local_;

  rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr params_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  obstacle_detector::msg::Obstacles obstacles_;
  double t_;

  // Parameters
  bool p_active_;
  bool p_reset_;
  bool p_fusion_example_;
  bool p_fission_example_;

  double p_loop_rate_;
  double p_sampling_time_;
  double p_radius_margin_;

  std::vector<double> p_x_vector_;
  std::vector<double> p_y_vector_;
  std::vector<double> p_r_vector_;

  std::vector<double> p_vx_vector_;
  std::vector<double> p_vy_vector_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
