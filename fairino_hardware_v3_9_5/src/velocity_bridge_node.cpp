#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/empty.hpp"
#include "libfairino/include/robot.h"
#include "fairino_hardware/velocity_bridge_node.hpp"

#include <memory>
#include <mutex>
#include <cmath>
#include <string>

VelocityBridge::VelocityBridge(std::shared_ptr<FRRobot> shared_robot) : Node("fr5_velocity_bridge"), ptr_robot_(shared_robot)
{
  // Parameters  
  declare_parameter<std::string>("robot_name", "fr5");
  declare_parameter<std::string>("controller_ip", "192.168.58.2");
  declare_parameter<double>("velocity_command_freq", 30.0);

  robot_name_ = get_parameter("robot_name").as_string();
  std::string controller_ip = get_parameter("controller_ip").as_string();
  double freq = get_parameter("velocity_command_freq").as_double();
  dt_ = 1.0 / freq;

  // Topic names matching diff_ik_node's convention
  std::string cmd_topic = robot_name_ + "/move";
  std::string hold_topic = robot_name_ + "/hold_robot";

  RCLCPP_INFO(get_logger(), "Connecting to FR5 at %s", controller_ip.c_str());
  RCLCPP_INFO(get_logger(), "Subscribing to %s", cmd_topic.c_str());
  RCLCPP_INFO(get_logger(), "Control frequency: %.1f Hz (dt = %.4f s)", freq, dt_);

  // ptr_robot_->ServoMoveStart(1);

  // Read current joint positions to initialise the integrator
  JointPos initial_pos;
  ptr_robot_->GetActualJointPosDegree(0, &initial_pos);
  float speed[6];
  ptr_robot_->GetActualJointSpeedsDegree(0, speed);
  RCLCPP_INFO(get_logger(), "Initial joint speeds (deg/s): %.2f %.2f %.2f %.2f %.2f %.2f",
              speed[0], speed[1], speed[2], speed[3], speed[4], speed[5]);
  // Start the servo after reading initial position to avoid a jump
  for (int i = 0; i < 6; i++) {
    current_positions_[i] = initial_pos.jPos[i];  // degrees
  }
  RCLCPP_INFO(get_logger(), "Initial joint positions (deg): %.2f %.2f %.2f %.2f %.2f %.2f",
              current_positions_[0], current_positions_[1], current_positions_[2],
              current_positions_[3], current_positions_[4], current_positions_[5]);

  // Subscriptions
  vel_cmd_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    cmd_topic, rclcpp::QoS(2),
    std::bind(&VelocityBridge::velocityCmdCb, this, std::placeholders::_1));

  hold_robot_sub_ = create_subscription<std_msgs::msg::Empty>(
    hold_topic, rclcpp::QoS(1),
    std::bind(&VelocityBridge::holdRobotCb, this, std::placeholders::_1));

  // Control timer — integration and ServoJ streaming loop
  auto dt_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / freq));
  control_timer_ = create_wall_timer(
    dt_ms, std::bind(&VelocityBridge::controlLoop, this));
  
  vel_lpf_gain_ = 1.0;        // low-pass filter gain (0=heavy filter, 1=no filter)
  prev_positions_.resize(6, 0.0);
  prev_velocities_.resize(6, 0.0);
  first_reading_ = true;


  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  RCLCPP_INFO(get_logger(), "Velocity bridge started");
}

VelocityBridge::~VelocityBridge()
{
  if (ptr_robot_) {
    ptr_robot_->ServoMoveEnd(1);
    ptr_robot_->StopMotion();
  }
}

void VelocityBridge::velocityCmdCb(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_cmd_ = *msg;
  has_cmd_ = true;
  RCLCPP_INFO(get_logger(), "Received velocity command");
}

void VelocityBridge::holdRobotCb(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_cmd_ = false;
  latest_cmd_.velocities.assign(6, 0.0);
  RCLCPP_INFO(get_logger(), "Hold robot — stopping servo integration");
}

void VelocityBridge::controlLoop()
{
  static bool is_servo_ready = false;

  if (!ptr_robot_) {
    RCLCPP_INFO(get_logger(), "FRRobot pointer not initialized");
    return;
  }

  JointPos hardware_pos;
  JointPos cmd;
  ExaxisPos ext{0, 0, 0, 0};

  if (ptr_robot_->GetActualJointPosDegree(0, &hardware_pos) != 0) {
    is_servo_ready = false;
    return;
  }

  // float speed_trial[6];
  // ptr_robot_->GetActualJointSpeedsDegree(0, speed_trial);
  // RCLCPP_INFO(get_logger(), "Initial joint speeds (deg/s): %.2f %.2f %.2f %.2f %.2f %.2f",
  //             speed_trial[0], speed_trial[1], speed_trial[2], speed_trial[3], speed_trial[4], speed_trial[5]);

  if (!is_servo_ready) {
      if (ptr_robot_->ServoMoveStart(1) == 0) {
          is_servo_ready = true;
          RCLCPP_INFO(this->get_logger(), "UDP Servo Gateway opened successfully.");
      }
      return; // Wait for next tick to start sending commands
  }

  // Publish joint states
  auto js_msg = sensor_msgs::msg::JointState();
  js_msg.header.stamp = this->get_clock()->now();
  for (int i = 0; i < 6; i++) {
    js_msg.name.push_back("j" + std::to_string(i+1));
    js_msg.position.push_back(hardware_pos.jPos[i] * M_PI / 180.0);
  }

  if (js_msg.position.size() < 6) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Joint state has only %zu positions, skipping", js_msg.position.size());
    return;
  }

  std::vector<double> cur_pos_rad = {
      hardware_pos.jPos[0] * M_PI / 180.0,
      hardware_pos.jPos[1] * M_PI / 180.0,
      hardware_pos.jPos[2] * M_PI / 180.0,
      hardware_pos.jPos[3] * M_PI / 180.0,
      hardware_pos.jPos[4] * M_PI / 180.0,
      hardware_pos.jPos[5] * M_PI / 180.0
    };

  auto now_time = now();
  if (first_reading_) {
    // First call — just store the positions, publish zero velocities
    first_reading_ = false;
    prev_timestamp_ = now_time;
    prev_positions_ = cur_pos_rad;
    js_msg.velocity = std::vector<double>(6, 0.0);
  } else {
    double dt = (now_time - prev_timestamp_).seconds();
   
    if (dt > 0.0) {
      for (int i = 0; i < 6; ++i) {
        double raw_vel = (cur_pos_rad[i] - prev_positions_[i]) / dt;
        prev_velocities_[i] = vel_lpf_gain_ * raw_vel + (1.0 - vel_lpf_gain_) * prev_velocities_[i];
        js_msg.velocity.push_back(prev_velocities_[i]);
      }
    } else {
      js_msg.velocity = prev_velocities_;
    }
    // Save state for next iteration
    prev_positions_ = cur_pos_rad;
    prev_timestamp_ = now_time;
  }
    
  joint_state_publisher_->publish(js_msg);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_cmd_) {
      // Hold still — just keep sending the current position
      for (int i = 0; i < 6; i++) {
        current_positions_[i] = hardware_pos.jPos[i]; 
        cmd.jPos[i] = current_positions_[i];
      }
      return;
    }

    // Integrate velocity → position
    // Velocities from diff_ik_node are in rad/s
    for (int i = 0; i < 6 && i < static_cast<int>(latest_cmd_.velocities.size()); i++) {
      double v_rad_s = latest_cmd_.velocities[i];
      double v_deg_s = v_rad_s * 180.0 / M_PI;
      current_positions_[i] += v_deg_s * dt_;
    }

    // Drift correction: blend towards the joint_states feedback (gentle pull, 1%)
    for (int i = 0; i < 6; i++) {
      // double error = hardware_pos.jPos[i] - current_positions_[i];
      // current_positions_[i] += 0.01 * error;  // 1% convergence per cycle
      cmd.jPos[i] = current_positions_[i];
    }

    RCLCPP_INFO(get_logger(), "Velocity sending: ");
    ptr_robot_->ServoJ(&cmd, &ext, 0, 0, static_cast<float>(dt_), 0, 0, 0, 1);
    RCLCPP_INFO(get_logger(), "Velocity sent");
  }
}