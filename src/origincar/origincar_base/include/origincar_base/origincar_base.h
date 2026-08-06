
#ifndef _ORIGINCAR_BASE_H_
#define _ORIGINCAR_BASE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>

#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h>
#include <sys/stat.h>

#include <serial/serial.h>
#include "origincar_base/command_mode.hpp"
#include "origincar_base/command_watchdog.hpp"
#include "origincar_base/sensor_calibration.hpp"
#include "origincar_base/serial_frame.hpp"
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "origincar_msg/msg/data.hpp"
#include "origincar_msg/msg/sign.hpp"  // 匹配信号发送
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
using namespace std;


#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

#define GYROSCOPE_RATIO   0.00026644f

#define ACCEl_RATIO 	  1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

typedef struct __Vel_Pos_Data_
{
	double X;
	double Y;
	double Z;

} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
	short accele_x_data;
	short accele_y_data;
	short accele_z_data;
    short gyros_x_data;
	short gyros_y_data;
	short gyros_z_data;

} MPU6050_DATA;

typedef struct _SEND_DATA_
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_speed;
	float Y_speed;
	float Z_speed;
	unsigned char Frame_Tail;
} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;
	float Y_speed;
	float Z_speed;
	float Power_Voltage;
	unsigned char Frame_Tail;
} RECEIVE_DATA;

class origincar_base : public rclcpp::Node

{
public:
	origincar_base();
	~origincar_base();
	void start();
	void on_serial_tick();
	void Publish_Odom(const rclcpp::Time & sensor_time);

public : 
	serial::Serial Stm32_Serial;

private:
	void declare_parameters();
	void get_parameters();


	void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
	void Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl);
	void Send_Stop_Command();
	void Prepare_Stop_Command();
	void Write_Command();
	void Handle_Serial_Write_Failure(
		const std::string & detail, bool short_write = false);
	void Handle_Serial_Read_Failure(const std::string & detail);
	void Record_Sensor_Frame_Timing(double sensor_time_sec);
	void Maybe_Log_Serial_Diagnostics(double now_sec);

	void Publish_ImuSensor(const rclcpp::Time & sensor_time);
	void Publish_Voltage();
	auto createQuaternionMsgFromYaw(double yaw);

	bool Get_Sensor_Data(rclcpp::Time & sensor_time);
	unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);
	short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);
	float Odom_Trans(uint8_t Data_High,uint8_t Data_Low);

	void Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch);

private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr Akm_Cmd_Vel_Sub;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher; 

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher;

	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotpose_publisher;
	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotvel_publisher;
	// rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tf_pub_;

	rclcpp::TimerBase::SharedPtr test_timer;

	rclcpp::TimerBase::SharedPtr serial_timer_;

	rclcpp::TimerBase::SharedPtr odom_timer;
	rclcpp::TimerBase::SharedPtr imu_timer;
	rclcpp::TimerBase::SharedPtr voltage_timer;

	rclcpp::TimerBase::SharedPtr robotpose_timer;
	rclcpp::TimerBase::SharedPtr robotvel_timer;

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr Sign_Switch_Sub;

	string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id, akm_cmd_vel, test;
	std::string cmd_vel;
	int serial_baud_rate;
	int serial_read_timeout_ms;
	int serial_write_timeout_ms;
	double command_timeout_sec;
	CommandMode command_mode;
	std::unique_ptr<CommandWatchdog> command_watchdog;
	SensorCalibration sensor_calibration_;
	StationaryGyroBiasEstimator stationary_gyro_bias_estimator_;
	IntegrationClock integration_clock_;
	XorFrameStreamParser<RECEIVE_DATA_SIZE> sensor_frame_parser_;
	LatestFrameSelector<RECEIVE_DATA_SIZE> latest_sensor_frame_selector_;
	std::array<double, 36> odom_pose_covariance_;
	std::array<double, 36> odom_twist_covariance_;
	std::array<double, 9> imu_angular_velocity_covariance_;
	std::array<double, 9> imu_linear_acceleration_covariance_;
	RECEIVE_DATA Receive_Data;
	SEND_DATA Send_Data;

	Vel_Pos_Data Robot_Pos;
	Vel_Pos_Data Robot_Vel;
	MPU6050_DATA Mpu6050_Data;
	float Power_voltage;
	std::uint64_t serial_bytes_read_;
	std::uint64_t serial_bad_frames_;
	std::uint64_t serial_discarded_bytes_;
	std::uint64_t serial_backlog_deferrals_;
	std::uint64_t serial_short_reads_;
	std::uint64_t serial_read_failures_;
	std::uint64_t serial_write_failures_;
	std::uint64_t serial_short_writes_;
	std::uint64_t serial_published_frames_;
	std::size_t serial_backlog_high_watermark_;
	double last_sensor_frame_time_sec_;
	double latest_sensor_frame_interval_sec_;
	double max_sensor_frame_interval_sec_;
	double last_serial_diagnostic_time_sec_;
	rclcpp::Time pending_sensor_frame_time_;
	bool pending_sensor_frame_deferred_by_backlog_;
	bool has_sensor_frame_time_;
	bool serial_failure_latched_;
    size_t count_;
};


#endif //_ORIGINCAR_BASE_H_
