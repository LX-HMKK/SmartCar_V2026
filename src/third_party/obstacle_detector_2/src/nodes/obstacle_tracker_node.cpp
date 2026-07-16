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

using namespace obstacle_detector;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto tracker_node = rclcpp::Node::make_shared("obstacle_tracker");
  try {
    RCLCPP_INFO(tracker_node->get_logger(), "[Obstacle Tracker]: Initializing node");
    ObstacleTracker ot(tracker_node, tracker_node);
    rclcpp::spin(tracker_node);
  }
  catch (const char* s) {
    RCLCPP_FATAL_STREAM(tracker_node->get_logger(), "[Obstacle Tracker]: "  << s);
  }
  catch (const std::exception &exc) {
    auto eptr = std::current_exception(); // capture
    RCLCPP_FATAL_STREAM(tracker_node->get_logger(), "[Obstacle Tracker]: " << exc.what());
  }
  catch (...){
    RCLCPP_FATAL_STREAM(tracker_node->get_logger(), "[Obstacle Tracker]: Unknown error");
  }
  rclcpp::shutdown();
  return 0;
}
