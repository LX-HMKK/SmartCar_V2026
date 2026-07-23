/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.h"

#include <algorithm>
#include <cstddef>

namespace rf2o {

namespace {

constexpr std::size_t kMinimumScanPoints = 33;

}  // namespace

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;

  // Set laser pose on the robot (through tF)
  // This allow estimation of the odometry with respect to the robot base reference system.
  geometry_msgs::msg::TransformStamped tf_laser;
  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "%s",ex.what());
    return false;
  }

  //TF:transform -> Eigen::Isometry3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}

bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DNode::process()
{
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    if (rf2o_ref.current_scan_time <= rf2o_ref.last_odom_time)
    {
      RCLCPP_WARN(get_logger(), "Dropping non-increasing laser scan timestamp");
      new_scan_available = false;
      return;
    }
    //Process odometry estimation
    const Pose3d pose_before_update = rf2o_ref.getPose();
    if (!rf2o_ref.odometryCalculation(last_scan))
    {
      RCLCPP_WARN(get_logger(), "RF2O rejected the laser scan; reinitializing");
      reset(false);
      return;
    }
    if (!rf2o_ref.getPose().matrix().allFinite() ||
      !std::isfinite(rf2o_ref.lin_speed) || !std::isfinite(rf2o_ref.ang_speed))
    {
      RCLCPP_ERROR(get_logger(), "RF2O produced a non-finite estimate; reinitializing");
      rf2o_ref.getPose() = pose_before_update;
      reset(false);
      return;
    }
    publish();
    new_scan_available = false; //avoids the possibility to run twice on the same laser scan
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for laser scans and TF");
  }
}

void CLaserOdometry2DNode::reset(bool reset_pose)
{
  geometry_msgs::msg::Pose next_initial_pose;
  next_initial_pose.orientation.w = 1.0;
  if (!reset_pose && rf2o_ref.is_initialized() && rf2o_ref.getPose().matrix().allFinite())
  {
    const Pose3d & current_pose = rf2o_ref.getPose();
    next_initial_pose.position.x = current_pose.translation()(0);
    next_initial_pose.position.y = current_pose.translation()(1);
    next_initial_pose.position.z = 0.0;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, getYaw(current_pose.rotation()));
    next_initial_pose.orientation = tf2::toMsg(orientation);
  }

  new_scan_available = false;
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan = true;
  rf2o_ref.lin_speed = 0.0;
  rf2o_ref.ang_speed = 0.0;
  initial_robot_pose.pose.pose = next_initial_pose;
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    if (new_scan->header.frame_id.empty())
    {
      RCLCPP_WARN(get_logger(), "Dropping laser scan with an empty frame_id");
      return;
    }
    if (new_scan->ranges.size() < kMinimumScanPoints)
    {
      RCLCPP_WARN(
        get_logger(), "Dropping laser scan with fewer than %zu points", kMinimumScanPoints);
      return;
    }
    if (!std::isfinite(new_scan->angle_min) || !std::isfinite(new_scan->angle_max) ||
      new_scan->angle_max <= new_scan->angle_min)
    {
      RCLCPP_WARN(get_logger(), "Dropping laser scan with invalid angular bounds");
      return;
    }

    const rclcpp::Time scan_time(new_scan->header.stamp);
    if (rf2o_ref.is_initialized() && scan_time <= rf2o_ref.last_odom_time)
    {
      RCLCPP_WARN(get_logger(), "Dropping non-increasing laser scan timestamp");
      return;
    }

    if (!rf2o_ref.first_laser_scan && new_scan->ranges.size() != rf2o_ref.width)
    {
      RCLCPP_WARN(
        get_logger(),
        "Laser scan width changed; preserving pose and reinitializing RF2O");
      reset(false);
      return;
    }

    //Keep in memory the last received laser_scan
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = scan_time;

    //Initialize module on first scan
    if (rf2o_ref.first_laser_scan == false)
    {
      new_scan_available = true;
    }
    else
    {
      if (!setLaserPoseFromTf())
        return;
      rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
      rf2o_ref.first_laser_scan = false;
    }
  }
}

void CLaserOdometry2DNode::resetCallBack(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  reset();
  response->success = true;
  response->message = "laser odometry reset; waiting for the next scan pair";
}

void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  //Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}

void CLaserOdometry2DNode::publish()
{
  RCLCPP_DEBUG(get_logger(), "[rf2o] Publishing Odom Topic");
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  nav_msgs::msg::Odometry odom;

  odom.header.stamp = rf2o_ref.last_odom_time;
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
  for (std::size_t index = 0; index < 6; ++index)
  {
    odom.pose.covariance[index * 6 + index] = pose_covariance_diagonal[index];
    odom.twist.covariance[index * 6 + index] = twist_covariance_diagonal[index];
  }
  //publish the message
  odom_pub->publish(odom);
}

} /* namespace rf2o */

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>() ;
  rclcpp::Rate rate(myLaserOdomNode->freq);
  while (rclcpp::ok()){
      rclcpp::spin_some(myLaserOdomNode);
      myLaserOdomNode->process();
      rate.sleep();
  }
  rclcpp::shutdown();
  return 0;

}
