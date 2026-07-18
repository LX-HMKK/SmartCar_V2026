#include "origincar_base/origincar_base.h"
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" 
#include "origincar_msg/msg/data.hpp"

using std::placeholders::_1;
using namespace std;
sensor_msgs::msg::Imu Mpu6050;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    {
      origincar_base Robot_Control;
      Robot_Control.Control();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 0;
}

short origincar_base::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    return transition_16;
}

float origincar_base::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001;
    return data_return;
  }

void origincar_base::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
    std::int16_t encoded_speed;
    std::int16_t encoded_steering;
    try {
      const double calibrated_steering = calibrate_steering_command(
        akm_ctl->drive.steering_angle, sensor_calibration_);
      encoded_speed = encode_protocol_milli_value(
        "Ackermann speed", akm_ctl->drive.speed);
      encoded_steering = encode_protocol_milli_value(
        "calibrated steering angle", calibrated_steering);
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR(this->get_logger(), "Rejected Ackermann command: %s", error.what());
      Send_Stop_Command();
      return;
    }

    const bool dispatched = dispatch_command(
      command_mode,
      CommandType::kAckermann,
      [this, encoded_speed, encoded_steering]() {
        command_watchdog->mark_command(rclcpp::Node::now().seconds());

        Send_Data.tx[0]=FRAME_HEADER;
        Send_Data.tx[1] = 0;
        Send_Data.tx[2] = 0;

        const std::uint16_t speed_bits = static_cast<std::uint16_t>(encoded_speed);
        Send_Data.tx[4] = speed_bits & 0xff;
        Send_Data.tx[3] = (speed_bits >> 8) & 0xff;

        Send_Data.tx[5] = 0;
        Send_Data.tx[6] = 0;

        const std::uint16_t steering_bits = static_cast<std::uint16_t>(encoded_steering);
        Send_Data.tx[8] = steering_bits & 0xff;
        Send_Data.tx[7] = (steering_bits >> 8) & 0xff;

        Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);
        Send_Data.tx[10]=FRAME_TAIL;
      },
      [this]() {Write_Command();});

    if (!dispatched) {
      RCLCPP_WARN(this->get_logger(), "Ignored Ackermann command in Twist mode");
    }
}

void origincar_base::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    std::int16_t encoded_x;
    std::int16_t encoded_y;
    std::int16_t encoded_z;
    try {
      encoded_x = encode_protocol_milli_value("Twist linear.x", twist_aux->linear.x);
      encoded_y = encode_protocol_milli_value("Twist linear.y", twist_aux->linear.y);
      encoded_z = encode_protocol_milli_value("Twist angular.z", twist_aux->angular.z);
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR(this->get_logger(), "Rejected Twist command: %s", error.what());
      Send_Stop_Command();
      return;
    }

    const bool dispatched = dispatch_command(
      command_mode,
      CommandType::kTwist,
      [this, encoded_x, encoded_y, encoded_z]() {
        command_watchdog->mark_command(rclcpp::Node::now().seconds());

        Send_Data.tx[0]=FRAME_HEADER;
        Send_Data.tx[1] = 0;
        Send_Data.tx[2] = 0;

        const std::uint16_t x_bits = static_cast<std::uint16_t>(encoded_x);
        Send_Data.tx[4] = x_bits & 0xff;
        Send_Data.tx[3] = (x_bits >> 8) & 0xff;

        const std::uint16_t y_bits = static_cast<std::uint16_t>(encoded_y);
        Send_Data.tx[6] = y_bits & 0xff;
        Send_Data.tx[5] = (y_bits >> 8) & 0xff;

        const std::uint16_t z_bits = static_cast<std::uint16_t>(encoded_z);
        Send_Data.tx[8] = z_bits & 0xff;
        Send_Data.tx[7] = (z_bits >> 8) & 0xff;

        Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);
        Send_Data.tx[10]=FRAME_TAIL;
      },
      [this]() {Write_Command();});

    if (!dispatched) {
      RCLCPP_WARN(this->get_logger(), "Ignored Twist command in Ackermann mode");
    }
}

void origincar_base::Write_Command()
{
    try {
      if (Stm32_Serial.isOpen()) {
        Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
      }
    } catch (const serial::IOException & error) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to send data through serial port: %s",
        error.what());
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
}

void origincar_base::Send_Stop_Command()
{
    Send_Data.tx[0] = FRAME_HEADER;
    for (size_t i = 1; i < 9; ++i) {
      Send_Data.tx[i] = 0;
    }
    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
    Send_Data.tx[10] = FRAME_TAIL;
    Write_Command();
}

void origincar_base::Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch)
{
  (void)sign_switch;
    /* if (sign_switch->data == -1) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 0.5;
         Robot_Pos.Y = 0.2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }
     else if (sign_switch->data == 6) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 2;
         Robot_Pos.Y = 2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }*/
}

void origincar_base::Publish_ImuSensor(const rclcpp::Time & sensor_time)
{
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = sensor_time;
    Imu_Data_Pub.header.frame_id = gyro_frame_id; 

    Imu_Data_Pub.orientation.x = 0.0;
    Imu_Data_Pub.orientation.y = 0.0;
    Imu_Data_Pub.orientation.z = 0.0;
    Imu_Data_Pub.orientation.w = 1.0;
    Imu_Data_Pub.orientation_covariance.fill(0.0);
    Imu_Data_Pub.orientation_covariance[0] = -1.0;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance = imu_angular_velocity_covariance_;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
    Imu_Data_Pub.linear_acceleration_covariance = imu_linear_acceleration_covariance_;

    imu_publisher->publish(Imu_Data_Pub);

}

void origincar_base::Publish_Odom(const rclcpp::Time & sensor_time)
{
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    origincar_msg::msg::Data robotpose;
    origincar_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = sensor_time;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id;

    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;

    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = odom_pose_covariance_;


    odom.twist.twist.linear.x =  Robot_Vel.X;
    odom.twist.twist.linear.y =  Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z; 
    odom.twist.covariance = odom_twist_covariance_;

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

    odom_publisher->publish(odom);
    robotpose_publisher->publish(robotpose);
    robotvel_publisher->publish(robotvel); 
}

void origincar_base::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    if (Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher->publish(voltage_msgs);
    }
}

unsigned char origincar_base::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum = 0, k;

    if (mode == 0) {
      for(k=0; k < Count_Number; k++) {
        check_sum = check_sum^Receive_Data.rx[k];
      }
    } else if (mode == 1) {
      for (k=0; k < Count_Number; k++) {
        check_sum = check_sum^Send_Data.tx[k];
      }
    }

    return check_sum;
}

bool origincar_base::Get_Sensor_Data()
{
    short transition_16 = 0;
    std::array<std::uint8_t, RECEIVE_DATA_SIZE> read_buffer{};
    size_t received_size = 0;
    try {
      received_size = Stm32_Serial.read(
        read_buffer.data(), read_buffer.size());
    } catch (const serial::IOException & error) {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to read sensor data: %s", error.what());
      Send_Stop_Command();
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
      return false;
    }
    if (received_size > 0) {
      sensor_frame_parser_.append(read_buffer.data(), received_size);
    }
    std::array<std::uint8_t, RECEIVE_DATA_SIZE> normalized_frame{};
    if (!sensor_frame_parser_.pop_frame(
        FRAME_HEADER, FRAME_TAIL, normalized_frame))
    {
      return false;
    }
    memcpy(Receive_Data.rx, normalized_frame.data(), normalized_frame.size());

    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];
    Receive_Data.Flag_Stop=Receive_Data.rx[1];
    Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);
    Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);
    Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);
    Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);
    Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);
    Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);

    Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
    Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
    Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

    Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
    Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;

    try {
      const double raw_vy = constrained_lateral_velocity(
        Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]),
        command_mode == CommandMode::kAckermann);
      const SensorSample calibrated = calibrate_sensor_sample(
        SensorSample(
          Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]),
          raw_vy,
          Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]),
          Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO),
        sensor_calibration_);
      Robot_Vel.X = calibrated.vx;
      Robot_Vel.Y = calibrated.vy;
      Robot_Vel.Z = calibrated.wheel_wz;
      Mpu6050.angular_velocity.z = calibrated.gyro_z;
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR(this->get_logger(), "Rejected sensor frame: %s", error.what());
      return false;
    }

    transition_16 = 0;
    transition_16 |=  Receive_Data.rx[20]<<8;
    transition_16 |=  Receive_Data.rx[21];
    Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;

    return true;
}

void origincar_base::Control()
{
    while(rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());

      const rclcpp::Time command_time = rclcpp::Node::now();
      if (command_watchdog->consume_stop(command_time.seconds())) {
        Send_Stop_Command();
        RCLCPP_WARN(this->get_logger(), "Command timeout; sent one stop command");
      }

      if (true == Get_Sensor_Data()) {
        const rclcpp::Time sensor_time = rclcpp::Node::now();
        const IntegrationDelta delta = integration_clock_.update(sensor_time.seconds());
        if (delta.should_integrate) {
          try {
            const PlanarPose integrated = integrate_planar(
              PlanarPose(Robot_Pos.X, Robot_Pos.Y, Robot_Pos.Z),
              SensorSample(
                Robot_Vel.X,
                Robot_Vel.Y,
                Robot_Vel.Z,
                Mpu6050.angular_velocity.z),
              delta.dt_sec);
            Robot_Pos.X = integrated.x;
            Robot_Pos.Y = integrated.y;
            Robot_Pos.Z = integrated.yaw;
          } catch (const std::invalid_argument & error) {
            RCLCPP_ERROR(this->get_logger(), "Rejected odometry integration: %s", error.what());
          }
        }

        Publish_ImuSensor(sensor_time);
        Publish_Voltage();
        Publish_Odom(sensor_time);
      }
    }
}

origincar_base::origincar_base()
: rclcpp::Node ("origincar_base"), command_mode(CommandMode::kAckermann)
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<int>("serial_baud_rate", 115200);
  this->declare_parameter<int>("serial_read_timeout_ms", 100);
  this->declare_parameter<double>("command_timeout_sec", 0.35);
  this->declare_parameter<double>("max_integration_dt_sec", 0.25);
  this->declare_parameter<std::string>("command_mode", "ackermann");
  this->declare_parameter<double>("longitudinal_velocity_scale", 1.03);
  this->declare_parameter<double>("lateral_velocity_scale", 1.125);
  this->declare_parameter<double>("yaw_velocity_scale", 1.0);
  this->declare_parameter<double>("gyro_z_scale", 1.0);
  this->declare_parameter<double>("gyro_z_bias", 0.0);
  this->declare_parameter<double>("steering_command_scale", 0.5);
  this->declare_parameter<double>("steering_command_offset_rad", 0.0);
  this->declare_parameter<double>("max_calibrated_steering_command_rad", 0.225);
  this->declare_parameter<std::vector<double>>(
    "odom_pose_covariance_diagonal",
    std::vector<double>{0.25, 0.25, 1e6, 1e6, 1e6, 0.50});
  this->declare_parameter<std::vector<double>>(
    "odom_twist_covariance_diagonal",
    std::vector<double>{0.04, 0.01, 1e6, 1e6, 1e6, 0.25});
  this->declare_parameter<std::vector<double>>(
    "imu_angular_velocity_covariance_diagonal",
    std::vector<double>{1.0, 1.0, 0.02});
  this->declare_parameter<std::vector<double>>(
    "imu_linear_acceleration_covariance_diagonal",
    std::vector<double>{0.25, 0.25, 0.25});

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("serial_read_timeout_ms", serial_read_timeout_ms);
  this->get_parameter("command_timeout_sec", command_timeout_sec);
  const std::string command_mode_name = this->get_parameter("command_mode").as_string();
  command_mode = command_mode_from_string(command_mode_name);

  sensor_calibration_.longitudinal_velocity_scale =
    this->get_parameter("longitudinal_velocity_scale").as_double();
  sensor_calibration_.lateral_velocity_scale =
    this->get_parameter("lateral_velocity_scale").as_double();
  sensor_calibration_.yaw_velocity_scale =
    this->get_parameter("yaw_velocity_scale").as_double();
  sensor_calibration_.gyro_z_scale =
    this->get_parameter("gyro_z_scale").as_double();
  sensor_calibration_.gyro_z_bias =
    this->get_parameter("gyro_z_bias").as_double();
  sensor_calibration_.steering_command_scale =
    this->get_parameter("steering_command_scale").as_double();
  sensor_calibration_.steering_command_offset_rad =
    this->get_parameter("steering_command_offset_rad").as_double();
  sensor_calibration_.max_calibrated_steering_command_rad =
    this->get_parameter("max_calibrated_steering_command_rad").as_double();

  try {
    validate_sensor_calibration(sensor_calibration_);
    integration_clock_ = IntegrationClock(
      this->get_parameter("max_integration_dt_sec").as_double());
    odom_pose_covariance_ = diagonal_covariance<6>(
      validated_covariance_diagonal<6>(
        this->get_parameter("odom_pose_covariance_diagonal").as_double_array(),
        "odom_pose_covariance_diagonal"));
    odom_twist_covariance_ = diagonal_covariance<6>(
      validated_covariance_diagonal<6>(
        this->get_parameter("odom_twist_covariance_diagonal").as_double_array(),
        "odom_twist_covariance_diagonal"));
    imu_angular_velocity_covariance_ = diagonal_covariance<3>(
      validated_covariance_diagonal<3>(
        this->get_parameter(
          "imu_angular_velocity_covariance_diagonal").as_double_array(),
        "imu_angular_velocity_covariance_diagonal"));
    imu_linear_acceleration_covariance_ = diagonal_covariance<3>(
      validated_covariance_diagonal<3>(
        this->get_parameter(
          "imu_linear_acceleration_covariance_diagonal").as_double_array(),
        "imu_linear_acceleration_covariance_diagonal"));
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(this->get_logger(), "Invalid sensor parameter: %s", error.what());
    throw;
  }

  if (serial_read_timeout_ms > 100) {
    RCLCPP_WARN(this->get_logger(), "serial_read_timeout_ms capped at 100 ms");
    serial_read_timeout_ms = 100;
  } else if (serial_read_timeout_ms < 1) {
    RCLCPP_WARN(this->get_logger(), "serial_read_timeout_ms raised to 1 ms");
    serial_read_timeout_ms = 1;
  }
  command_watchdog = std::make_unique<CommandWatchdog>(command_timeout_sec);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);

  robotpose_publisher = create_publisher<origincar_msg::msg::Data>("robotpose", 10);

  robotvel_publisher = create_publisher<origincar_msg::msg::Data>("robotvel", 10);

  if (command_mode == CommandMode::kTwist) {
    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel, 1, std::bind(&origincar_base::Cmd_Vel_Callback, this, _1));
  } else {
    Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        akm_cmd_vel, 1, std::bind(&origincar_base::Akm_Cmd_Vel_Callback, this, _1));
  }

  // Sign_Switch_Sub = create_subscription<std_msgs::msg::Int32>(
  //     "/sign4return", 1, std::bind(&origincar_base::Sign_Switch_Callback, this, _1));
  try  {
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(serial_read_timeout_ms);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(),"origincar_base can not open serial port,Please check the serial port cable! ");
  }
  if(Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(),"origincar_base serial port opened");
  }
}


origincar_base::~origincar_base()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down");
  Send_Stop_Command();
}
