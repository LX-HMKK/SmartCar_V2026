#ifndef RF2O_LASER_ODOMETRY__CLASERODOMETRY2DNODE_H_
#define RF2O_LASER_ODOMETRY__CLASERODOMETRY2DNODE_H_

#include "rf2o_laser_odometry/CLaserOdometry2D.h"

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <std_srvs/srv/trigger.hpp>

namespace rf2o {

class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  CLaserOdometry2DNode();
  void process();
  void publish();
  bool setLaserPoseFromTf();
  void reset(bool reset_pose = true);

  CLaserOdometry2D rf2o_ref;
  bool new_scan_available;

  double freq;

  std::string         laser_scan_topic;
  std::string         odom_topic;
  std::string         base_frame_id;
  std::string         odom_frame_id;
  std::string         init_pose_from_topic;
  std::string         reset_service;
  std::vector<double> pose_covariance_diagonal;
  std::vector<double> twist_covariance_diagonal;

  sensor_msgs::msg::LaserScan      last_scan;
  bool                        GT_pose_initialized;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::Odometry     initial_robot_pose;

  //Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  initPose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  odom_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;

  bool scan_available();

  //CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);
  void resetCallBack(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

inline CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  //Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_laser");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom_combined");
  this->get_parameter("odom_frame_id", odom_frame_id);
  const bool publish_tf = this->declare_parameter<bool>("publish_tf", false);
  if (publish_tf)
  {
    throw std::invalid_argument(
      "publish_tf must be false; robot_localization is the only odometry TF owner");
  }
  this->declare_parameter<std::string>("init_pose_from_topic", "");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);
  this->declare_parameter<std::string>(
    "reset_service", "/smartcar/localization/reset_laser_odometry");
  this->get_parameter("reset_service", reset_service);
  this->declare_parameter<std::vector<double>>(
    "pose_covariance_diagonal", {0.05, 0.05, 1e6, 1e6, 1e6, 0.03});
  this->get_parameter("pose_covariance_diagonal", pose_covariance_diagonal);
  this->declare_parameter<std::vector<double>>(
    "twist_covariance_diagonal", {0.04, 1e6, 1e6, 1e6, 1e6, 0.04});
  this->get_parameter("twist_covariance_diagonal", twist_covariance_diagonal);

  if (freq <= 0.0 || !std::isfinite(freq))
  {
    throw std::invalid_argument("freq must be finite and positive");
  }
  if (pose_covariance_diagonal.size() != 6 || twist_covariance_diagonal.size() != 6)
  {
    throw std::invalid_argument("covariance diagonals must contain six values");
  }
  for (const double value : pose_covariance_diagonal)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("pose covariance must be finite and positive");
    }
  }
  for (const double value : twist_covariance_diagonal)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("twist covariance must be finite and positive");
    }
  }

  //Publishers and Subscribers
  //--------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  reset_srv = this->create_service<std_srvs::srv::Trigger>(
    reset_service,
    std::bind(
      &CLaserOdometry2DNode::resetCallBack,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  //init pose??
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }


  //Init variables
  reset();
}

}  // namespace rf2o

#endif  // RF2O_LASER_ODOMETRY__CLASERODOMETRY2DNODE_H_
