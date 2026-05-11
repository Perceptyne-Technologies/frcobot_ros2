#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/empty.hpp"
#include "libfairino/include/robot.h"

#include <memory>
#include <mutex>
#include <cmath>
#include <string>

class VelocityBridge : public rclcpp::Node
{
public:
  explicit VelocityBridge(std::shared_ptr<FRRobot> shared_robot);
  ~VelocityBridge();

private:
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void velocityCmdCb(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
  void holdRobotCb(const std_msgs::msg::Empty::SharedPtr msg);
  void controlLoop();

  // FR5 SDK
  std::shared_ptr<FRRobot> ptr_robot_;

  // Integration state
  double current_positions_[6] = {0.0};  // degrees
  double joint_state_positions_[6] = {0.0};  // degrees, from /joint_states
  double dt_ = 0.008;  // seconds

  // Command state
  trajectory_msgs::msg::JointTrajectoryPoint latest_cmd_;
  bool has_cmd_ = false;
  std::mutex mutex_;
  std::string robot_name_;

  // ROS
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr vel_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hold_robot_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  bool first_reading_;
  double vel_lpf_gain_;
  rclcpp::Time prev_timestamp_;
  std::vector<double> prev_positions_;
  std::vector<double> prev_velocities_;
};