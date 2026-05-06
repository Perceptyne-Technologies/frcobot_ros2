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
  VelocityBridge() : Node("fr5_velocity_bridge")
  {
    // Parameters
    declare_parameter<std::string>("robot_name", "fr5");
    declare_parameter<std::string>("controller_ip", "192.168.58.2");
    declare_parameter<double>("velocity_command_freq", 125.0);

    robot_name_ = get_parameter("robot_name").as_string();
    std::string controller_ip = get_parameter("controller_ip").as_string();
    double freq = get_parameter("velocity_command_freq").as_double();
    dt_ = 1.0 / freq;

    // Topic names matching diff_ik_node's convention
    std::string cmd_topic = robot_name_ + "/command_move";
    std::string hold_topic = robot_name_ + "/hold_robot";

    RCLCPP_INFO(get_logger(), "Connecting to FR5 at %s", controller_ip.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing to %s", cmd_topic.c_str());
    RCLCPP_INFO(get_logger(), "Control frequency: %.1f Hz (dt = %.4f s)", freq, dt_);

    // Connect to FR5 controller
    ptr_robot_ = std::make_unique<FRRobot>();
    errno_t ret = ptr_robot_->RPC(controller_ip.c_str());
    if (ret != 0) {
      RCLCPP_FATAL(get_logger(), "Failed to connect to FR5 at %s", controller_ip.c_str());
      throw std::runtime_error("FR5 connection failed");
    }

    // Read current joint positions to initialise the integrator
    JointPos initial_pos;
    ptr_robot_->GetActualJointPosDegree(0, &initial_pos);
    for (int i = 0; i < 6; i++) {
      current_positions_[i] = initial_pos.jPos[i];  // degrees
    }
    RCLCPP_INFO(get_logger(), "Initial joint positions (deg): %.2f %.2f %.2f %.2f %.2f %.2f",
                current_positions_[0], current_positions_[1], current_positions_[2],
                current_positions_[3], current_positions_[4], current_positions_[5]);

    // Subscriptions
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(2),
      std::bind(&VelocityBridge::jointStateCb, this, std::placeholders::_1));

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

    RCLCPP_INFO(get_logger(), "Velocity bridge started");
  }

  ~VelocityBridge()
  {
    if (ptr_robot_) {
      ptr_robot_->StopMotion();
      ptr_robot_->CloseRPC();
    }
  }

private:
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Map by joint name — assumes joint_states has j1..j6
    for (size_t i = 0; i < msg->name.size(); i++) {
      if (msg->name[i] == "j1") joint_state_positions_[0] = msg->position[i] * 180.0 / M_PI;
      if (msg->name[i] == "j2") joint_state_positions_[1] = msg->position[i] * 180.0 / M_PI;
      if (msg->name[i] == "j3") joint_state_positions_[2] = msg->position[i] * 180.0 / M_PI;
      if (msg->name[i] == "j4") joint_state_positions_[3] = msg->position[i] * 180.0 / M_PI;
      if (msg->name[i] == "j5") joint_state_positions_[4] = msg->position[i] * 180.0 / M_PI;
      if (msg->name[i] == "j6") joint_state_positions_[5] = msg->position[i] * 180.0 / M_PI;
    }
    got_joint_state_ = true;
  }

  void velocityCmdCb(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = *msg;
    has_cmd_ = true;
  }

  void holdRobotCb(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_cmd_ = false;
    RCLCPP_INFO(get_logger(), "Hold robot — stopping servo integration");
  }

  void controlLoop()
  {
    JointPos cmd;
    ExaxisPos ext{0, 0, 0, 0};

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!has_cmd_) {
        // Hold still — just keep sending the current position
        for (int i = 0; i < 6; i++) {
          cmd.jPos[i] = current_positions_[i];
        }
        ptr_robot_->ServoJ(&cmd, &ext, 0, 0, static_cast<float>(dt_), 0, 0);
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
      if (got_joint_state_) {
        for (int i = 0; i < 6; i++) {
          double error = joint_state_positions_[i] - current_positions_[i];
          current_positions_[i] += 0.01 * error;  // 1% convergence per cycle
        }
      }

      for (int i = 0; i < 6; i++) {
        cmd.jPos[i] = current_positions_[i];
      }
    }

    ptr_robot_->ServoJ(&cmd, &ext, 0, 0, static_cast<float>(dt_), 0, 0);
  }

  // FR5 SDK
  std::unique_ptr<FRRobot> ptr_robot_;

  // Integration state
  double current_positions_[6] = {0.0};  // degrees
  double joint_state_positions_[6] = {0.0};  // degrees, from /joint_states
  double dt_ = 0.008;  // seconds
  bool got_joint_state_ = false;

  // Command state
  trajectory_msgs::msg::JointTrajectoryPoint latest_cmd_;
  bool has_cmd_ = false;
  std::mutex mutex_;
  std::string robot_name_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr vel_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hold_robot_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
