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

#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

#include "obstacle_detector/utilities/tracked_obstacle.h"
#include "obstacle_detector/utilities/kalman.h"

namespace obstacle_detector
{

class TrackedSegmentObstacle {
public:
  TrackedSegmentObstacle(const obstacle_detector::msg::SegmentObstacle& obstacle) : obstacle_(obstacle), kf_x1_(0, 1, 2), kf_y1_(0, 1, 2), kf_x2_(0, 1, 2), kf_y2_(0, 1, 2) {
    fade_counter_ = s_fade_counter_size_;
    setNewUid();
    initKF();
  }

  void predictState() {
    kf_x1_.predictState();
    kf_y1_.predictState();
    kf_x2_.predictState();
    kf_y2_.predictState();

    obstacle_.first_point.x = kf_x1_.q_pred(0);
    obstacle_.first_point.y = kf_y1_.q_pred(0);
    obstacle_.last_point.x = kf_x2_.q_pred(0);
    obstacle_.last_point.y = kf_y2_.q_pred(0);

    obstacle_.first_velocity.x = kf_x1_.q_pred(1);
    obstacle_.first_velocity.y = kf_y1_.q_pred(1);
    obstacle_.last_velocity.x = kf_x2_.q_pred(1);
    obstacle_.last_velocity.y = kf_y2_.q_pred(1);

    fade_counter_--;
  }

  void correctState(const obstacle_detector::msg::SegmentObstacle& new_obstacle) {
    kf_x1_.y(0) = new_obstacle.first_point.x;
    kf_y1_.y(0) = new_obstacle.first_point.y;
    kf_x2_.y(0) = new_obstacle.last_point.x;
    kf_y2_.y(0) = new_obstacle.last_point.y;

    kf_x1_.correctState();
    kf_y1_.correctState();
    kf_x2_.correctState();
    kf_y2_.correctState();

    obstacle_.first_point.x = kf_x1_.q_est(0);
    obstacle_.first_point.y = kf_y1_.q_est(0);
    obstacle_.last_point.x = kf_x2_.q_est(0);
    obstacle_.last_point.y = kf_y2_.q_est(0);

    obstacle_.first_velocity.x = kf_x1_.q_est(1);
    obstacle_.first_velocity.y = kf_y1_.q_est(1);
    obstacle_.last_velocity.x = kf_x2_.q_est(1);
    obstacle_.last_velocity.y = kf_y2_.q_est(1);

    fade_counter_ = s_fade_counter_size_;
  }

  void updateState() {
    kf_x1_.predictState();
    kf_y1_.predictState();
    kf_x2_.predictState();
    kf_y2_.predictState();

    kf_x1_.correctState();
    kf_y1_.correctState();
    kf_x2_.correctState();
    kf_y2_.correctState();

    obstacle_.first_point.x = kf_x1_.q_est(0);
    obstacle_.first_point.y = kf_y1_.q_est(0);
    obstacle_.last_point.x = kf_x2_.q_est(0);
    obstacle_.last_point.y = kf_y2_.q_est(0);

    obstacle_.first_velocity.x = kf_x1_.q_est(1);
    obstacle_.first_velocity.y = kf_y1_.q_est(1);
    obstacle_.last_velocity.x = kf_x2_.q_est(1);
    obstacle_.last_velocity.y = kf_y2_.q_est(1);

    fade_counter_--;
  }

  static void setSamplingTime(double tp) {
    s_sampling_time_ = tp;
  }

  static void setCounterSize(int size) {
    s_fade_counter_size_ = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    s_process_variance_ = process_var;
    s_process_rate_variance_ = process_rate_var;
    s_measurement_variance_ = measurement_var;
  }

  void setNewUid() { obstacle_.uid = uid_next_++; }
  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const obstacle_detector::msg::SegmentObstacle& getObstacle() const { return obstacle_; }
  const KalmanFilter& getKFx1() const { return kf_x1_; }
  const KalmanFilter& getKFy1() const { return kf_y1_; }
  const KalmanFilter& getKFx2() const { return kf_x2_; }
  const KalmanFilter& getKFy2() const { return kf_y2_; }

private:
  void initKF() {
    kf_x1_.A(0, 1) = s_sampling_time_;
    kf_y1_.A(0, 1) = s_sampling_time_;
    kf_x2_.A(0, 1) = s_sampling_time_;
    kf_y2_.A(0, 1) = s_sampling_time_;

    kf_x1_.C(0, 0) = 1.0;
    kf_y1_.C(0, 0) = 1.0;
    kf_x2_.C(0, 0) = 1.0;
    kf_y2_.C(0, 0) = 1.0;

    kf_x1_.R(0, 0) = s_measurement_variance_;
    kf_y1_.R(0, 0) = s_measurement_variance_;
    kf_x2_.R(0, 0) = s_measurement_variance_;
    kf_y2_.R(0, 0) = s_measurement_variance_;

    kf_x1_.Q(0, 0) = s_process_variance_;
    kf_y1_.Q(0, 0) = s_process_variance_;
    kf_x2_.Q(0, 0) = s_process_variance_;
    kf_y2_.Q(0, 0) = s_process_variance_;

    kf_x1_.Q(1, 1) = s_process_rate_variance_;
    kf_y1_.Q(1, 1) = s_process_rate_variance_;
    kf_x2_.Q(1, 1) = s_process_rate_variance_;
    kf_y2_.Q(1, 1) = s_process_rate_variance_;

    kf_x1_.q_pred(0) = obstacle_.first_point.x;
    kf_y1_.q_pred(0) = obstacle_.first_point.y;
    kf_x2_.q_pred(0) = obstacle_.last_point.x;
    kf_y2_.q_pred(0) = obstacle_.last_point.y;

    kf_x1_.q_pred(1) = obstacle_.first_velocity.x;
    kf_y1_.q_pred(1) = obstacle_.first_velocity.y;
    kf_x2_.q_pred(1) = obstacle_.last_velocity.x;
    kf_y2_.q_pred(1) = obstacle_.last_velocity.y;

    kf_x1_.q_est(0) = obstacle_.first_point.x;
    kf_y1_.q_est(0) = obstacle_.first_point.y;
    kf_x2_.q_est(0) = obstacle_.last_point.x;
    kf_y2_.q_est(0) = obstacle_.last_point.y;

    kf_x1_.q_est(1) = obstacle_.first_velocity.x;
    kf_y1_.q_est(1) = obstacle_.first_velocity.y;
    kf_x2_.q_est(1) = obstacle_.last_velocity.x;
    kf_y2_.q_est(1) = obstacle_.last_velocity.y;
  }

  obstacle_detector::msg::SegmentObstacle obstacle_;

  KalmanFilter kf_x1_;
  KalmanFilter kf_y1_;
  KalmanFilter kf_x2_;
  KalmanFilter kf_y2_;

  int fade_counter_;

  // Common variables
  static int s_fade_counter_size_;
  static double s_sampling_time_;
  static double s_process_variance_;
  static double s_process_rate_variance_;
  static double s_measurement_variance_;
};

}
