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

#include "obstacle_detector/obstacle_extractor.h"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/math_utilities.h"

using namespace std;
using namespace obstacle_detector;

ObstacleExtractor::ObstacleExtractor(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local){
  nh_ = nh;
  nh_local_ = nh_local;
  p_active_ = false;
//   params_srv_ = nh_->create_service<std_srvs::srv::Empty>("params", 
//                                                           std::bind(
//                                                                 &ObstacleExtractor::updateParams,
//                                                                 this, 
//                                                                 std::placeholders::_1,
//                                                                 std::placeholders::_2,
//                                                                 std::placeholders::_3
//                                                           ));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  time_last_marker_published_ = nh_->get_clock()->now() - rclcpp::Duration(10, 0);
  initialize();
}

ObstacleExtractor::~ObstacleExtractor() {

}

void ObstacleExtractor::updateParamsUtil(){
  bool prev_active = p_active_;
  nh_->declare_parameter("active", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("use_scan", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("use_pcl", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("use_pcl2", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("use_split_and_merge", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("circles_from_visibles", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("discard_converted_segments", rclcpp::PARAMETER_BOOL);
  nh_->declare_parameter("transform_coordinates", rclcpp::PARAMETER_BOOL);

  nh_->declare_parameter("min_group_points", rclcpp::PARAMETER_INTEGER);

  nh_->declare_parameter("max_group_distance", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("distance_proportion", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_split_distance", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_merge_separation", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_merge_spread", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_circle_radius", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("radius_enlargement", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("min_x_limit", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_x_limit", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("min_y_limit", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("max_y_limit", rclcpp::PARAMETER_DOUBLE);
  nh_->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);

  nh_->get_parameter_or("active", p_active_, true);
  nh_->get_parameter_or("use_scan", p_use_scan_, true);
  nh_->get_parameter_or("use_pcl", p_use_pcl_, true);
  nh_->get_parameter_or("use_pcl2", p_use_pcl_2_, true);
  nh_->get_parameter_or("use_split_and_merge", p_use_split_and_merge_, true);
  nh_->get_parameter_or("circles_from_visibles", p_circles_from_visibles_, true);
  nh_->get_parameter_or("discard_converted_segments", p_discard_converted_segments_, true);
  nh_->get_parameter_or("transform_coordinates", p_transform_coordinates_, true);

  nh_->get_parameter_or("min_group_points", p_min_group_points_, 5);

  nh_->get_parameter_or("max_group_distance", p_max_group_distance_, 0.1);
  nh_->get_parameter_or("distance_proportion", p_distance_proportion_, 0.00628);
  nh_->get_parameter_or("max_split_distance", p_max_split_distance_, 0.2);
  nh_->get_parameter_or("max_merge_separation", p_max_merge_separation_, 0.2);
  nh_->get_parameter_or("max_merge_spread", p_max_merge_spread_, 0.2);
  nh_->get_parameter_or("max_circle_radius", p_max_circle_radius_, 0.6);
  nh_->get_parameter_or("radius_enlargement", p_radius_enlargement_, 0.25);
  nh_->get_parameter_or("min_x_limit", p_min_x_limit_, -10.0);
  nh_->get_parameter_or("max_x_limit", p_max_x_limit_,  10.0);
  nh_->get_parameter_or("min_y_limit", p_min_y_limit_, -10.0);
  nh_->get_parameter_or("max_y_limit", p_max_y_limit_,  10.0);
  nh_->get_parameter_or("frame_id", p_frame_id_, std::string{"map"});

  if (p_active_ != prev_active) {
    if (p_active_) {
      if (p_use_scan_){
        RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Using LaserScan topic");
        scan_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ObstacleExtractor::scanCallback, this, std::placeholders::_1));
      }else if (p_use_pcl_){
        RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Using PointCloud1 topic");
        pcl_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud>(
            "pcl", 10, std::bind(&ObstacleExtractor::pclCallback, this, std::placeholders::_1));
      }else if (p_use_pcl_2_){
        RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Using PointCloud2 topic");
        pcl2_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pcl2", 10, std::bind(&ObstacleExtractor::pcl2Callback, this, std::placeholders::_1));
      }
      obstacles_pub_ = nh_->create_publisher<obstacle_detector::msg::Obstacles>("raw_obstacles", 10);
      obstacles_vis_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("raw_obstacles_visualization", 10);
    }
    else {
      // Send empty message
      auto obstacles_msg = obstacle_detector::msg::Obstacles();
      obstacles_msg.header.frame_id = p_frame_id_;
      obstacles_msg.header.stamp = nh_->get_clock()->now();
      obstacles_pub_->publish(obstacles_msg);
    }
  }
}

void ObstacleExtractor::updateParams(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<std_srvs::srv::Empty::Request> &req, 
                                     const std::shared_ptr<std_srvs::srv::Empty::Response> &res) {
  updateParamsUtil();
}

void ObstacleExtractor::scanCallback(const sensor_msgs::msg::LaserScan& scan_msg) {
  base_frame_id_ = scan_msg.header.frame_id;
  stamp_ = scan_msg.header.stamp;

  double phi = scan_msg.angle_min;

  for (const float r : scan_msg.ranges) {
    if (r >= scan_msg.range_min && r <= scan_msg.range_max)
      input_points_.push_back(Point::fromPoolarCoords(r, phi));

    phi += scan_msg.angle_increment;
  }

  processPoints();
}

void ObstacleExtractor::pclCallback(const sensor_msgs::msg::PointCloud& pcl_msg) {
  base_frame_id_ = pcl_msg.header.frame_id;
  stamp_ = pcl_msg.header.stamp;

  const auto range_channel = std::find_if(pcl_msg.channels.begin(), pcl_msg.channels.end(), [] (const sensor_msgs::msg::ChannelFloat32& channel) { return channel.name == "range"; } );
  int i = 0;
  for (const geometry_msgs::msg::Point32& point : pcl_msg.points) {
    auto point_copy = Point(point.x, point.y, 0., point.z);
    if (range_channel != pcl_msg.channels.end()) {
      point_copy.range = range_channel->values.at(i++);
      assert(point_copy.range >= 0.0);
      RCLCPP_INFO_STREAM_ONCE(nh_->get_logger(), "Point cloud contains range information, an example value is " << point_copy.range);
    } else {
      RCLCPP_WARN_ONCE(nh_->get_logger(), "Point cloud does not contain range information, assuming point cloud origin aligns with lidar origin");
    }
    input_points_.push_back(point_copy);
  }

  processPoints();
}

void ObstacleExtractor::pcl2Callback(sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg) {
  base_frame_id_ = pcl_msg->header.frame_id;
  stamp_ = pcl_msg->header.stamp;

  const size_t number_of_points = pcl_msg->height * pcl_msg->width;
  sensor_msgs::PointCloud2Iterator<float> iter_x(*pcl_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pcl_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pcl_msg, "z");
  for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z){
    double point_x = (*iter_x);
    double point_y = (*iter_y);
    double point_z = (*iter_z);
    auto point_copy = Point(point_x, point_y, 0., point_z);
    input_points_.push_back(point_copy);
    RCLCPP_WARN_ONCE(nh_->get_logger(), "Assuming point cloud origin aligns with lidar origin");
    // RCLCPP_WARN_STREAM(
    // nh_->get_logger(),
    // "px " << point_x << ", py " << point_y << ", pz " << point_z);
  }
  processPoints();
}

void ObstacleExtractor::processPoints() {
  segments_.clear();
  circles_.clear();

  groupPoints();  // Grouping points simultaneously detects segments
  mergeSegments();

  detectCircles();
  mergeCircles();

  transformObstacles();
  publishObstacles();
  publishVisualizationObstacles();

  input_points_.clear();
}

void ObstacleExtractor::groupPoints() {
  static double sin_dp = sin(2.0 * p_distance_proportion_);

  PointSet point_set;
  point_set.begin = input_points_.begin();
  point_set.end = input_points_.begin();
  point_set.num_points = 1;
  point_set.is_visible = true;

  for (PointIterator point = input_points_.begin()++; point != input_points_.end(); ++point) {
    double range = (*point).getRange();
    double distance = (*point - *point_set.end).length();

    if (distance < p_max_group_distance_ + range * p_distance_proportion_) {
      point_set.end = point;
      point_set.num_points++;
    }
    else {
      double prev_range = (*point_set.end).getRange();

      // Heron's equation
      double p = (range + prev_range + distance) / 2.0;
      double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
      double sin_d = 2.0 * S / (range * prev_range); // Sine of angle between beams

      // TODO: This condition can be fulfilled if the point are on the opposite sides
      // of the scanner (angle = 180 deg). Needs another check.
      if (abs(sin_d) < sin_dp && range < prev_range)
        point_set.is_visible = false;

      detectSegments(point_set);

      // Begin new point set
      point_set.begin = point;
      point_set.end = point;
      point_set.num_points = 1;
      point_set.is_visible = (abs(sin_d) > sin_dp || range < prev_range);
    }
  }

  detectSegments(point_set); // Check the last point set too!
}

void ObstacleExtractor::detectSegments(const PointSet& point_set) {
  if (point_set.num_points < p_min_group_points_)
    return;

  Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

  if (p_use_split_and_merge_)
    segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  int split_index = 0; // Natural index of splitting point (counting from 1)
  int point_index = 0; // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).getRange();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }

  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0.0 && split_index > p_min_group_points_ && split_index < point_set.num_points - p_min_group_points_) {
    set_divider = input_points_.insert(set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  } else {  // Add the segment
    if (!p_use_split_and_merge_)
      segment = fitSegment(point_set);

    segments_.push_back(segment);
  }
}

void ObstacleExtractor::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr; // Check the new segment against others
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareSegments(const Segment& s1, const Segment& s2, Segment& merged_segment) {
  if (&s1 == &s2)
    return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool ObstacleExtractor::checkSegmentsProximity(const Segment& s1, const Segment& s2) {
  return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
          s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
}

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment& segment, const Segment& s1, const Segment& s2) {
  return (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
          segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s2.last_point)  < p_max_merge_spread_);
}

void ObstacleExtractor::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    if (p_circles_from_visibles_) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) {
          segment_is_visible = false;
          break;
        }
      }
      if (!segment_is_visible)
        continue;
    }

    Circle circle(*segment);
    circle.radius += p_radius_enlargement_;

    if (circle.radius < p_max_circle_radius_) {
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleExtractor::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle merged_circle;

      if (compareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circles_.insert(i, merged_circle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleExtractor::compareCircles(const Circle& c1, const Circle& c2, Circle& merged_circle) {
  if (&c1 == &c2)
    return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }

  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }

  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius / (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += max(c1.radius, c2.radius);

    if (circle.radius < p_max_circle_radius_) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(), c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(), c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }

  return false;
}

void ObstacleExtractor::publishVisualizationObstacles(){
  auto obstacles_vis_msg = visualization_msgs::msg::MarkerArray();
  int id = 0;
  for (const Segment& s : segments_) {
    auto seg_marker = visualization_msgs::msg::Marker();
    seg_marker.header.stamp = stamp_;
    seg_marker.header.frame_id = published_obstacles_frame_id_;
    seg_marker.action = visualization_msgs::msg::Marker::ADD;
    seg_marker.id = id++;
    seg_marker.ns = "raw_obstacles";
    seg_marker.scale.x = 0.1;
    seg_marker.color.g = 1.0;
    seg_marker.color.a = 1.0;
    seg_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    auto seg_fp = geometry_msgs::msg::Point();
    seg_fp.x = s.first_point.x;
    seg_fp.y = s.first_point.y;
    seg_fp.z = s.first_point.z;
    auto seg_lp = geometry_msgs::msg::Point();
    seg_lp.x = s.last_point.x;
    seg_lp.y = s.last_point.y;
    seg_lp.z = s.last_point.z;
    seg_marker.points.push_back(seg_fp);
    seg_marker.points.push_back(seg_lp);

    seg_marker.pose.orientation.w = 1.0;   
    obstacles_vis_msg.markers.push_back(seg_marker);
  }

  for (const Circle& c : circles_) {
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_) {
        auto circ_marker = visualization_msgs::msg::Marker();
        circ_marker.header.stamp = stamp_;
        circ_marker.header.frame_id = published_obstacles_frame_id_;
        circ_marker.action = visualization_msgs::msg::Marker::ADD;
        circ_marker.id = id++;
        circ_marker.ns = "raw_obstacles";
        // fake a bigger obstacle radius for visualization purposes
        double rad = c.radius;
        if (rad < 0.2){rad = 0.2;}
        circ_marker.scale.x = rad;
        circ_marker.scale.y = rad;
        circ_marker.scale.z = 0.01;
        circ_marker.color.g = 1.0;
        circ_marker.color.a = 1.0;
        circ_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        
        circ_marker.pose.position.x = c.center.x;
        circ_marker.pose.position.y = c.center.y;
        circ_marker.pose.position.z = c.center.z;

        circ_marker.pose.orientation.x = 0.0;
        circ_marker.pose.orientation.y = 0.0;
        circ_marker.pose.orientation.z = 0.0;
        circ_marker.pose.orientation.w = 1.0;
        obstacles_vis_msg.markers.push_back(circ_marker);
    }
  }

  // clean up remaining ids
  while(id < num_active_markers_){
    visualization_msgs::msg::Marker markerD;
    markerD.header.stamp = stamp_;
    markerD.header.frame_id = published_obstacles_frame_id_;
    markerD.ns = "raw_obstacles";
    markerD.id = id++;  
    markerD.action = visualization_msgs::msg::Marker::DELETE;
    obstacles_vis_msg.markers.push_back(markerD);
  }
  num_active_markers_ = id + 1;
  obstacles_vis_pub_->publish(obstacles_vis_msg);
  time_last_marker_published_ = nh_->get_clock()->now();
}

void ObstacleExtractor::transformObstacles() {
  if (p_transform_coordinates_) {
    geometry_msgs::msg::TransformStamped m_transform;
    if (base_frame_id_[0] == '/') base_frame_id_.erase(0, 1);
    if (p_frame_id_[0] == '/') p_frame_id_.erase(0, 1);

    try {
        m_transform = tf_buffer_->lookupTransform(p_frame_id_, base_frame_id_, tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        nh_->get_logger(), "Could not transformmm %s to %s: %s",
        p_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
        return;
    }

    for (Segment& s : segments_) {
      s.first_point = transformPoint(s.first_point, m_transform);
      s.last_point = transformPoint(s.last_point, m_transform);
    }

    for (Circle& c : circles_){
      c.center = transformPoint(c.center, m_transform);
    }

    published_obstacles_frame_id_ = p_frame_id_;
  }
  else{
    published_obstacles_frame_id_ = base_frame_id_;
  }

}

void ObstacleExtractor::publishObstacles() {
  auto obstacles_msg = obstacle_detector::msg::Obstacles();
  obstacles_msg.header.stamp = stamp_;
  obstacles_msg.header.frame_id = published_obstacles_frame_id_;

  for (const Segment& s : segments_) {
    obstacle_detector::msg::SegmentObstacle segment;
    segment.first_point.x = s.first_point.x;
    segment.first_point.y = s.first_point.y;
    segment.first_point.z = s.first_point.z;
    segment.last_point.x = s.last_point.x;
    segment.last_point.y = s.last_point.y;
    segment.last_point.z = s.last_point.z;

    obstacles_msg.segments.push_back(segment);
  }

  for (const Circle& c : circles_) {
    if (c.center.x > p_min_x_limit_ && c.center.x < p_max_x_limit_ &&
        c.center.y > p_min_y_limit_ && c.center.y < p_max_y_limit_) {
        obstacle_detector::msg::CircleObstacle circle;

        circle.center.x = c.center.x;
        circle.center.y = c.center.y;
        circle.center.z = c.center.z;
        circle.velocity.x = 0.0;
        circle.velocity.y = 0.0;
        circle.radius = c.radius;
        circle.true_radius = c.radius - p_radius_enlargement_;

        obstacles_msg.circles.push_back(circle);
    }
  }
  obstacles_pub_->publish(obstacles_msg);
}
