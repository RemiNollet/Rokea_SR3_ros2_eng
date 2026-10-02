// //Use MoveIt only for trajectory planning, and call SDK interfaces for motion control
// // #include <rclcpp/rclcpp.hpp>
// // #include <moveit/move_group_interface/move_group_interface.h>
// // #include <moveit/planning_scene_interface/planning_scene_interface.h>
// // #include <rokae/robot.h>
// // #include <rokae/motion_control_rt.h>
// // #include <rokae/data_types.h>

// // #include <chrono>
// // #include <thread>
// // #include <vector>
// // #include <memory>

// // class MoveJOpenLoop : public rclcpp::Node
// // {
// // public:
// //     MoveJOpenLoop()
// //     : Node("movej_open_loop_node")
// //     {
    
// //     }

// //     void run()
// //     {
// //         // ============= Initialization SDK =============
// //         std::error_code ec;
// //         robot_ = std::make_shared<rokae::xMateRobot>("192.168.21.10", "192.168.21.131");
// //         robot_->setOperateMode(rokae::OperateMode::automatic, ec);
// //         robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
// //         robot_->setPowerState(true, ec);
// //         robot_->startReceiveRobotState(std::chrono::milliseconds(1),
// //                                        {rokae::RtSupportedFields::jointPos_m});

// //         // Create MoveGroupInterface
// //         moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), "rokae_arm");

// //         arm.setPlanningTime(10.0);
// //         arm.setMaxVelocityScalingFactor(0.2);
// //         arm.setMaxAccelerationScalingFactor(0.2);

// //         // ============= Set target points for trajectory planning rather than actual movement =============
// //         std::vector<double> joint_target = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
// //         //std::vector<double> joint_target = {0, 0, 0, 0, 0, 0};
// //         arm.setJointValueTarget(joint_target);


// //         // auto base_rci = robot_->getRtMotionController().lock();
// //         // rci_ = std::dynamic_pointer_cast<RtType>(base_rci);
// //         rci_ = robot_->getRtMotionController().lock();

// //         // ============= Trajectory Planning =============
// //         moveit::planning_interface::MoveGroupInterface::Plan plan;
// //         bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

// //         if (!success) {
// //             RCLCPP_ERROR(this->get_logger(), "Planning failure！");
// //             return;
// //         }

// //         RCLCPP_INFO(this->get_logger(), "Planning successful, start open-loop execution...");

// //         // ============= Manually send trajectory points (Open loop) =============
// //         const auto &trajectory = plan.trajectory_.joint_trajectory;
// //         // Reference start time for recording track（wall-clock，steady）
// //         auto start_wall = std::chrono::steady_clock::now();
// //         // Convert the ROS Duration of the i-th point to steady_clock absolute target time： start_wall + point.time_from_start
// //         for (const auto &point : trajectory.points) {   //Traverse trajectory points
// //             // Check the length of positions (and the number of joints)）
// //             if (point.positions.size() == 0) {
// //                 RCLCPP_WARN(this->get_logger(), "The trajectory point positions are empty, skipping this point");
// //                 continue;
// //             }

// //             std::array<double, 6> target_point{};
// //             std::copy(point.positions.begin(), point.positions.end(), target_point.begin());

// //             // put builtin_interfaces::msg::Duration convert to rclcpp::Duration
// //             rclcpp::Duration ros_dur(point.time_from_start);

// //             // use nanoseconds() get int64_t Number of nanoseconds, converted to chrono::nanoseconds
// //             auto target_time = start_wall + std::chrono::nanoseconds(ros_dur.nanoseconds());

// //             // Wait until the target time arrives (if the target time has passed, send immediately)）
// //             auto now = std::chrono::steady_clock::now();
// //             if (target_time > now) {
// //                 std::this_thread::sleep_for(target_time - now);
// //             }

// //             // Issue the command (note to check whether the SDK interface is synchronous or asynchronous, and whether a higher-precision sending method is required)）
// //             rci_->MoveJ(0.1, robot_->jointPos(ec), target_point);
// //             if (ec) {
// //                 RCLCPP_ERROR(this->get_logger(), "Failed to issue command: %s", ec.message().c_str());
// //                 break;
// //             }
// //         }

// //         RCLCPP_INFO(this->get_logger(), "Trajectory issuance completed (open loop）。");
// //     }

// // private:
// //     std::shared_ptr<rokae::xMateRobot> robot_;
// //     using RtType = rokae::RtMotionControl<rokae::WorkType::collaborative, 6>;
// //     std::shared_ptr<RtType> rci_;
// // };

// // int main(int argc, char **argv)
// // {
// //     rclcpp::init(argc, argv);

// //     auto node = std::make_shared<MoveJOpenLoop>();
// //     node->run();

// //     rclcpp::shutdown();
// //     return 0;
// // }



//moveitSimultaneously perform trajectory planning and trajectory execution issuance
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <memory>
#include <cmath>

const std::string kControllerName = "position_joint_trajectory_controller";

class MoveJDemo : public rclcpp::Node
{
public:
    MoveJDemo()
    : Node("movej_demo_node")
    {
        controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
    }

    void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)    //mg It is a variable of smart pointer type, used to receive smart pointers passed in from outside.
    {
        move_group_ = mg;
    }

    void movej()
    {
        auto& arm = *move_group_;   //*move_group_ Dereference → get one MoveGroupInterface& Reference, therefore arm is essentially a reference

        arm.setPlanningTime(45.0);
        // arm.setPoseReferenceFrame("xMateCR12_base");    //***_base Comments available
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.2);
        arm.setGoalOrientationTolerance(0.2);
        arm.setMaxAccelerationScalingFactor(0.05);
        arm.setMaxVelocityScalingFactor(0.05);

        //std::vector<double> joint_target = {0, 0, 0, 0, 0, 0};   //Six-axis test data
        std::vector<double> joint_target = {1, 1, 1, 1, 1, 1};
        // std::vector<double> joint_target = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};   
        //std::vector<double> joint_target = {1, 1, 1, 1, 1, 1, 1};    //Seven-axis test data
        arm.setJointValueTarget(joint_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "The plan is successful and is being executed...");
            auto result = arm.execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Execution failed, try restarting the controller...");
            arm.stop();
            arm.clearPoseTargets();
            arm.setStartStateToCurrentState();
            reset_controller(kControllerName);
            rclcpp::sleep_for(std::chrono::seconds(2)); // Waiting for the controller to switch
            // if (!reset_controller(kControllerName)) {
            //     RCLCPP_ERROR(this->get_logger(), "Controller recovery failed, subsequent planning may continue to fail！");
            // }
        }

        } else {
            RCLCPP_ERROR(this->get_logger(), "Joint space planning failure！");
        }
    }

    bool movej_sequence(const std::vector<std::vector<double>>& joint_targets)
    {
        auto& arm = *move_group_;

        arm.setPlanningTime(45.0);
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.2);
        arm.setGoalOrientationTolerance(0.2);
        arm.setMaxAccelerationScalingFactor(0.05);
        arm.setMaxVelocityScalingFactor(0.05);

        const size_t dof = arm.getCurrentJointValues().size();

        for (size_t i = 0; i < joint_targets.size(); ++i) {
            const auto& target = joint_targets[i];

            if (target.size() != dof) {
                RCLCPP_ERROR(this->get_logger(),
                    "Target dimension error #%zu: expected %zu, actual %zu",
                    i + 1, dof, target.size());
                return false;
            }

            arm.setStartStateToCurrentState();
            arm.setJointValueTarget(target);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "The %zu-th target planning failed", i + 1);
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "The %zu-th target was successfully planned, starting execution", i + 1);
            auto result = arm.execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "Execution of target #%zu failed, attempting to restart the controller", i + 1);
                arm.stop();
                arm.clearPoseTargets();
                arm.setStartStateToCurrentState();
                reset_controller(kControllerName);
                return false;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(300));
        }

        return true;
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    bool reset_controller(const std::string& controller_name)
    {
        // 1. Check if the service is available
        if (!controller_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Service /controller_manager/switch_controller Not available！");
            return false;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        // deactivate + activate Replace the old stop/start
        request->deactivate_controllers.push_back(controller_name);
        request->activate_controllers.push_back(controller_name);
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT; // Strict mode
        request->timeout = rclcpp::Duration::from_seconds(5.0); // Prevent long delays when switching

        // 2. Send switch request
        auto future = controller_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Call /switch_controller Service failed！");
            return false;
        }

        if (!future.get()->ok) {
            RCLCPP_ERROR(this->get_logger(), "Controller switch failed, the resource may be occupied！");
            return false;
        }

        // 3. Query controller status
        auto list_client = this->create_client<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers");

        if (!list_client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Service /controller_manager/list_controllers Not available！");
            return false;
        }

        const int max_retries = 10;
        for (int i = 0; i < max_retries; ++i) {
            auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
            auto list_future = list_client->async_send_request(list_req);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), list_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to query controller status, retrying...");
                continue;
            }

            auto response = list_future.get();
            for (const auto& ctrl : response->controller) {
                if (ctrl.name == controller_name) {
                    if (ctrl.state == "active") {
                        RCLCPP_INFO(this->get_logger(), "Controller [%s] Already reactivated！", controller_name.c_str());
                        return true;
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Controller [%s] Current status: %s, waiting...",
                                    controller_name.c_str(), ctrl.state.c_str());
                    }
                }
            }

            rclcpp::sleep_for(std::chrono::milliseconds(500)); // Wait and check again
        }

        RCLCPP_ERROR(this->get_logger(), "Controller [%s] Did not enter active state after reboot！", controller_name.c_str());
        return false;
    }

};


//ROS2 The communication model is asynchronous: receiving a message does not automatically execute user code; it must be scheduled and executed by the rclcpp executor. When calling arm.plan() / arm.getCurrentState() etc., MoveIt will internally wait for a response from the action/server or wait to subscribe to /joint_states The callback results. The handling of these returns/callbacks depends on the executor spin。
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 1) Create node
    auto node = std::make_shared<MoveJDemo>();

    // 2) Create an executor and add the node in
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 3) Start the spin thread (the executor will continuously schedule callbacks)）
    std::thread spinner([&executor]() {
        executor.spin();
    });

    // Optional: Give the executor some time to discover action server / topics
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 4) Create MoveGroupInterface (or it can also be created in advance, as long as spin is running)）
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");     
    node->set_move_group(move_group);

    // Wait a little longer to ensure that action/server/params are detected
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5) Execute your blocking process（plan/execute）
    // node->movej();

    // Two joint targets, executed in sequence
    std::vector<std::vector<double>> targets = {
        {0.5, 0.5, 0.5, 0.5, 0.5, 0.5},  // Target 1 (6-axis example）
        // {1.0,  1.0, 1.0,  1.0, 1.0, 1.0}    // Target 2 (6-axis example）
    };

    bool ok = node->movej_sequence(targets);
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Sequential movement execution failed");
    }

    // 6) End: shutdown and wait for spinner to exit
    rclcpp::shutdown();
    spinner.join();
    return 0;
}




// /*Loop execution trajectory planning*/
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <controller_manager_msgs/srv/switch_controller.hpp>
// #include <controller_manager_msgs/srv/list_controllers.hpp>
// #include <memory>
// #include <cmath>
// #include <random>

// const std::string kControllerName = "position_joint_trajectory_controller";

// class MoveJDemo : public rclcpp::Node
// {
// public:
//     MoveJDemo()
//     : Node("movej_demo_node")
//     {
//         controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
//             "/controller_manager/switch_controller");

//         // Initialize joint limits
//         lower_limits_ = {-3.04, -3.04, -3.04, -3.04, -3.04, -3.04};
//         upper_limits_ = {3.04, 3.04, 3.04, 3.04, 3.04, 3.04};
        
//         // Initialize the random number generator
//         random_engine_ = std::mt19937(std::random_device{}());
//     }

//     void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
//     {
//         move_group_ = mg;
//     }

//     // Generate random joint targets
//     std::vector<double> generate_random_joint_target()
//     {
//         std::vector<double> joint_target;
//         joint_target.reserve(6);
        
//         for (size_t i = 0; i < 6; ++i) {
//             std::uniform_real_distribution<double> dist(lower_limits_[i], upper_limits_[i]);
//             joint_target.push_back(dist(random_engine_));
//         }
        
//         RCLCPP_INFO(this->get_logger(), "Generate random joint targets: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
//                    joint_target[0], joint_target[1], joint_target[2], 
//                    joint_target[3], joint_target[4], joint_target[5]);
        
//         return joint_target;
//     }

//     void movej(const std::vector<double>& joint_target)
//     {
//         auto& arm = *move_group_;

//         arm.setPlanningTime(45.0);
//         arm.setPoseReferenceFrame("xMateCR7_base");    //Change to the corresponding base(Under the corresponding model SRDF)
//         arm.allowReplanning(true);
//         arm.setGoalPositionTolerance(0.2);
//         arm.setGoalOrientationTolerance(0.2);
//         arm.setMaxAccelerationScalingFactor(0.05);
//         arm.setMaxVelocityScalingFactor(0.05);

//         // Use the incoming joint target
//         arm.setJointValueTarget(joint_target);

//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//         if (success) {
//             // Check the trajectory safety
//             bool safety = checkTrajectorySafety(plan.trajectory_);
            
//             if (safety) {
//                 RCLCPP_INFO(this->get_logger(), "Trajectory is secure, executing...");
//                 auto result = arm.execute(plan);
                
//                 if (result != moveit::core::MoveItErrorCode::SUCCESS) {
//                     RCLCPP_WARN(this->get_logger(), "Execution failed, try restarting the controller...");
//                     arm.stop();
//                     arm.clearPoseTargets();
//                     arm.setStartStateToCurrentState();
//                     reset_controller(kControllerName);
//                     rclcpp::sleep_for(std::chrono::seconds(2));
//                 } else {
//                     RCLCPP_INFO(this->get_logger(), "Trajectory execution successful");
//                 }
//             } else {
//                 RCLCPP_ERROR(this->get_logger(), "The trajectory is unsafe, regenerate the target...");
//                 // The trajectory is unsafe, return false to let the main loop regenerate the target
//                 return;
//             }

//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Joint space planning failure！");
//         }
        
//         // Wait for a period of time after execution is completed
//         rclcpp::sleep_for(std::chrono::seconds(2));
//     }

// private:
//     rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//     std::vector<double> lower_limits_;
//     std::vector<double> upper_limits_;
//     std::mt19937 random_engine_;

//     bool checkTrajectorySafety(const moveit_msgs::msg::RobotTrajectory& trajectory)
//     {
//         const auto& joint_trajectory = trajectory.joint_trajectory;
//         bool is_safe = true;
        
//         for (size_t i = 0; i < joint_trajectory.points.size(); ++i) {
//             const auto& point = joint_trajectory.points[i];
            
//             for (size_t j = 0; j < point.positions.size() && j < lower_limits_.size(); ++j) {
//                 double position = point.positions[j];
                
//                 if (position < lower_limits_[j] || position > upper_limits_[j]) {
//                     RCLCPP_WARN(this->get_logger(), 
//                                "Trajectory point %zu, joint %zu position %f exceeds limit [%f, %f]", 
//                                i, j, position, lower_limits_[j], upper_limits_[j]);
//                     is_safe = false;           
//                 }
//             }
//         }
//         return is_safe;
//     }
  
//     bool reset_controller(const std::string& controller_name)
//     {
//         // 1. Check if the service is available
//         if (!controller_client_->wait_for_service(std::chrono::seconds(3))) {
//             RCLCPP_ERROR(this->get_logger(), "Service /controller_manager/switch_controller Not available！");
//             return false;
//         }

//         auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
//         // deactivate + activate Replace the old stop/start
//         request->deactivate_controllers.push_back(controller_name);
//         request->activate_controllers.push_back(controller_name);
//         request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT; // Strict mode
//         request->timeout = rclcpp::Duration::from_seconds(5.0); // Prevent long pauses during switching

//         // 2. Send switch request
//         auto future = controller_client_->async_send_request(request);
//         if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
//             rclcpp::FutureReturnCode::SUCCESS)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Call /switch_controller Service failed！");
//             return false;
//         }

//         if (!future.get()->ok) {
//             RCLCPP_ERROR(this->get_logger(), "Controller switch failed, the resource may be occupied！");
//             return false;
//         }

//         // 3. Query controller status
//         auto list_client = this->create_client<controller_manager_msgs::srv::ListControllers>(
//             "/controller_manager/list_controllers");

//         if (!list_client->wait_for_service(std::chrono::seconds(3))) {
//             RCLCPP_ERROR(this->get_logger(), "Service /controller_manager/list_controllers Not available！");
//             return false;
//         }

//         const int max_retries = 10;
//         for (int i = 0; i < max_retries; ++i) {
//             auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
//             auto list_future = list_client->async_send_request(list_req);

//             if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), list_future) !=
//                 rclcpp::FutureReturnCode::SUCCESS)
//             {
//                 RCLCPP_WARN(this->get_logger(), "Failed to query controller status, retrying...");
//                 continue;
//             }

//             auto response = list_future.get();
//             for (const auto& ctrl : response->controller) {
//                 if (ctrl.name == controller_name) {
//                     if (ctrl.state == "active") {
//                         RCLCPP_INFO(this->get_logger(), "Controller [%s] Already reactivated！", controller_name.c_str());
//                         return true;
//                     } else {
//                         RCLCPP_INFO(this->get_logger(), "Controller [%s] Current status: %s, waiting...",
//                                     controller_name.c_str(), ctrl.state.c_str());
//                     }
//                 }
//             }

//             rclcpp::sleep_for(std::chrono::milliseconds(500)); // Wait and check again
//         }

//         RCLCPP_ERROR(this->get_logger(), "Controller [%s] Did not enter active state after reboot！", controller_name.c_str());
//         return false;
//     }

// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);

//     // Create node
//     auto node = std::make_shared<MoveJDemo>();

//     // Create an executor and add the node in
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);

//     // Start spin thread
//     std::thread spinner([&executor]() {
//         executor.spin();
//     });

//     // Give the executor some time to discover action server / topics
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));

//     // Create MoveGroupInterface
//     auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");
//     node->set_move_group(move_group);

//     // Wait a little longer to ensure that action/server/params are detected
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     RCLCPP_INFO(node->get_logger(), "Start random joint target cyclic movements...");

//     // Main loop: continuously generate random targets and execute
//     int cycle_count = 0;
//     while (rclcpp::ok()) {
//         cycle_count++;
//         RCLCPP_INFO(node->get_logger(), "=== The %dth loop ===", cycle_count);
        
//         // Generate random joint targets
//         auto joint_target = node->generate_random_joint_target();
        
//         // Perform exercise
//         node->movej(joint_target);
        
//         RCLCPP_INFO(node->get_logger(), "Cycle %d completed, preparing for the next exercise...", cycle_count);
//     }

//     RCLCPP_INFO(node->get_logger(), "Node closed, stop movement loop");

//     // End: shutdown and wait for spinner to exit
//     rclcpp::shutdown();
//     spinner.join();
//     return 0;
// }