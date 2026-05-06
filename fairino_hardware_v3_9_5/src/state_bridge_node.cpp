#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <deque>

class StateBridge : public rclcpp::Node
{
public:
  StateBridge() : Node("fr5_state_bridge")
  {
    // Parameters
    declare_parameter<std::string>("robot_name", "fr5");
    declare_parameter<double>("publish_rate", 100.0);

    robot_name_ = get_parameter("robot_name").as_string();
    double rate = get_parameter("publish_rate").as_double();

    // Fixed joint names (6-DOF robot arm)
    joint_names_ = {"j1", "j2", "j3", "j4", "j5", "j6"};

    // Velocity estimation parameters
    vel_lpf_gain_ = 1.0;        // low-pass filter gain (0=heavy filter, 1=no filter)
    prev_positions_.resize(6, 0.0);
    prev_velocities_.resize(6, 0.0);
    first_reading_ = true;

    // Subscriber to the custom non-real-time state topic (published by fr_command_server)
    nonrt_state_sub_ = create_subscription<fairino_msgs::msg::RobotNonrtState>(
      "nonrt_state_data", rclcpp::QoS(10),
      std::bind(&StateBridge::stateCb, this, std::placeholders::_1));

    // Publisher for MoveIt-compatible /joint_states
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::QoS(10));

    // Timer to publish at the desired rate
    auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    pub_timer_ = create_wall_timer(
      period_ms, std::bind(&StateBridge::publishJointStates, this));

    RCLCPP_INFO(get_logger(), "State bridge started — publishing /joint_states at %.1f Hz", rate);
  }

private:
  void stateCb(const fairino_msgs::msg::RobotNonrtState::SharedPtr msg)
  {
    // Store the latest state (thread-safe via the atomic shared_ptr swap)
    auto new_state = std::make_shared<fairino_msgs::msg::RobotNonrtState>(*msg);
    std::atomic_store(&latest_state_, new_state);
  }

  void publishJointStates()
  {
    auto state = std::atomic_load(&latest_state_);
    if (!state) {
      return;  // no data received yet
    }

    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now();
    js_msg.name = joint_names_;

    // Convert from degrees (robot SDK) to radians (MoveIt standard)
    std::vector<double> cur_pos_rad = {
      state->j1_cur_pos * M_PI / 180.0,
      state->j2_cur_pos * M_PI / 180.0,
      state->j3_cur_pos * M_PI / 180.0,
      state->j4_cur_pos * M_PI / 180.0,
      state->j5_cur_pos * M_PI / 180.0,
      state->j6_cur_pos * M_PI / 180.0
    };

    js_msg.position = cur_pos_rad;
    js_msg.velocity.resize(6);

    // Estimate velocities via numerical differentiation + low-pass filter
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
          js_msg.velocity[i] = prev_velocities_[i];
        }
      } else {
        js_msg.velocity = prev_velocities_;
      }
      // Save state for next iteration
      prev_positions_ = cur_pos_rad;
      prev_timestamp_ = now_time;
    }

    joint_state_pub_->publish(js_msg);
  }

  // ROS
  rclcpp::Subscription<fairino_msgs::msg::RobotNonrtState>::SharedPtr nonrt_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // State
  std::shared_ptr<fairino_msgs::msg::RobotNonrtState> latest_state_;
  std::string robot_name_;
  std::vector<std::string> joint_names_;

  // Velocity estimation
  bool first_reading_;
  double vel_lpf_gain_;
  rclcpp::Time prev_timestamp_;
  std::vector<double> prev_positions_;
  std::vector<double> prev_velocities_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
