// File name: simple_movej_client.cpp
#include <rclcpp/rclcpp.hpp>
#include <rokae_msgs/srv/move_j.hpp>
#include <iostream>

int main(int argc, char** argv) {
    // InitializationROS 2
    rclcpp::init(argc, argv);
    
    // Create Node
    auto node = rclcpp::Node::make_shared("movej_client");
    
    // Create MoveJ service client
    auto movej_client = node->create_client<rokae_msgs::srv::MoveJ>("/rokae_driver/movej");
    
    // Waiting for service to be available (wait up to 10 seconds)）
    if (!movej_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "MoveJService Unavailable");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "MoveJService connected");
    
    // Create request message
    auto request = std::make_shared<rokae_msgs::srv::MoveJ::Request>();
    
    // Set target joint angle to [1,1,1,1,1,1] radian
    request->joint_positions = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    
    // Set the speed parameter (this is necessary according to your callback function)）
    request->velocity = 0.2;  // The speed value can be adjusted, for example, 0.1 represents 10% of the maximum speed.
    
    RCLCPP_INFO(node->get_logger(), "Send MoveJ request");
    RCLCPP_INFO(node->get_logger(), "Target joint angle: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] rad",
                request->joint_positions[0], request->joint_positions[1],
                request->joint_positions[2], request->joint_positions[3],
                request->joint_positions[4], request->joint_positions[5]);
    RCLCPP_INFO(node->get_logger(), "Speed: %.2f", request->velocity);
    
    // Send the request and wait for a response
    auto future = movej_client->async_send_request(request);
    
    // Waiting for service response (waiting up to 30 seconds, because MoveJ may take time to execute)）
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(30)) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        try {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "MoveJExecution successful: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "MoveJExecution failed: %s", response->message.c_str());
                return 1;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Abnormal: %s", e.what());
            return 1;
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call timeout");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Program completed");
    rclcpp::shutdown();
    return 0;
}