//moveitSimultaneously perform trajectory planning and trajectory execution issuance
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <memory>
#include <cmath>
#include <thread>
#include <vector>

const std::string kControllerName = "joint_s_line";

class MoveJDemo : public rclcpp::Node
{
public:
    MoveJDemo()
    : Node("joint_s_line_node")
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

    // 5) Construct 6 target points and execute them in sequence; the program ends after completion.
    const auto current = move_group->getCurrentJointValues();
    if (current.size() < 6) {
        RCLCPP_ERROR(node->get_logger(), "The current number of joints is less than 6, unable to execute a six-point trajectory (actual: %zu）", current.size());
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    auto make_target = [&current](double j1, double j2, double j3, double j4, double j5, double j6) {
        auto t = current;
        t[0] = j1;
        t[1] = j2;
        t[2] = j3;
        t[3] = j4;
        t[4] = j5;
        t[5] = j6;
        return t;
    };

    std::vector<std::vector<double>> targets = {
        make_target(0.5,  0.5,  0.5,  0.5,  0.5,  0.5),
        make_target(0.7,  0.4,  0.3,  0.6,  0.4,  0.6),
        make_target(0.6,  0.2,  0.0,  0.5,  0.2,  0.4),
        make_target(0.3, -0.1, -0.2,  0.2, -0.1,  0.2),
        make_target(0.1, -0.2, -0.3,  0.0, -0.2,  0.0),
        make_target(0.0,  0.0,  0.0,  0.0,  0.0,  0.0)
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