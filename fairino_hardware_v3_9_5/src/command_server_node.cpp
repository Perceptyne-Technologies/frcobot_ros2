#include "fairino_hardware/command_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "fairino_hardware/velocity_bridge_node.hpp"

int main(int argc, char *argv[]){
    //该main函数用于创建简化指令客户端的app
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor mulexecutor;

    //创建用户指令节点
    auto command_server_node = std::make_shared<robot_command_thread>("fr_command_server");
    mulexecutor.add_node(command_server_node);
    //创建非实时状态反馈获取节点
    // auto CNDE_node = std::make_shared<CNDE_recv_thread>("CNDE_thread");
    // mulexecutor.add_node(CNDE_node);//状态反馈节点加入执行器
    //创建非实时状态反馈获取节点

    bool connected = false;
    int timeout_count = 0;
    while (rclcpp::ok() && !connected && timeout_count < 20) { // 10 second timeout
        if (command_server_node->_ptr_robot) {
            // We call a non-intrusive SDK function to check if the socket is alive
            int robot_state = 0;
            // GetRobotRealTimeState is a good way to verify the TCP 8080 port is open
            ROBOT_STATE_PKG tmp;
            if (command_server_node->_ptr_robot->GetRobotRealTimeState(&tmp) == 0) {
                connected = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        timeout_count++;
    }

    if (!connected) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Robot connection timeout! Bridge will not start.");
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Hardware connection verified. Starting Velocity Bridge...");

    auto velocity_bridge_node = std::make_shared<VelocityBridge>(command_server_node->_ptr_robot);
    mulexecutor.add_node(velocity_bridge_node);//状态反馈节点加入执行器

    mulexecutor.spin();
    rclcpp::shutdown();

    return 0;
}
