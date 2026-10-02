//moveitSimultaneously perform trajectory planning and trajectory execution issuance
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <memory>
#include <cmath>
#include <thread>
#include <algorithm>

const std::string kControllerName = "joint_position_controller";
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class MoveJDemo : public rclcpp::Node
{
public:
    MoveJDemo()
    : Node("joint_position_control_node")
    {
        controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/position_joint_trajectory_controller/follow_joint_trajectory");
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

    bool execute_cosine_s_curve_trajectory(
        double t1,
        double t2,
        double t3,
        double dt,
        const std::vector<int>& signs)
    {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface Uninitialized");
            return false;
        }

        if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server Not available");
            return false;
        }

        const auto joint_names = move_group_->getJointNames();
        const auto jnt_pos = move_group_->getCurrentJointValues();
        const size_t dof = jnt_pos.size();

        if (joint_names.size() != dof || dof == 0) {
            RCLCPP_ERROR(this->get_logger(), "Joint name and joint position dimensions are inconsistent or empty");
            return false;
        }

        if (dof < signs.size()) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient degrees of freedom, expected at least %zu, actual %zu", signs.size(), dof);
            return false;
        }

        auto make_duration_msg = [](double seconds) {
            builtin_interfaces::msg::Duration d;
            const int32_t sec = static_cast<int32_t>(std::floor(seconds));
            const double frac = seconds - static_cast<double>(sec);
            d.sec = sec;
            d.nanosec = static_cast<uint32_t>(frac * 1e9);
            return d;
        };

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.2);
        traj.joint_names = joint_names;

        if (t1 <= 0.0 || t2 <= 0.0 || t3 <= 0.0 || dt <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory time parameter is illegal, required t1/t2/t3/dt > 0");
            return false;
        }

        const double amplitude = M_PI / 20.0;
        const double total_time = t1 + t2 + t3;
        const int steps = static_cast<int>(std::ceil(total_time / dt));

        traj.points.reserve(static_cast<size_t>(steps) + 1);

        for (int i = 0; i <= steps; ++i) {
            const double t = std::min(i * dt, total_time);

            double q_start = 0.0;
            double q_end = 0.0;
            double seg_t = 0.0;
            double seg_T = 0.0;

            if (t <= t1) {
                q_start = 0.0;
                q_end = amplitude;
                seg_t = t;
                seg_T = t1;
            } else if (t <= (t1 + t2)) {
                q_start = amplitude;
                q_end = -amplitude;
                seg_t = t - t1;
                seg_T = t2;
            } else {
                q_start = -amplitude;
                q_end = 0.0;
                seg_t = t - t1 - t2;
                seg_T = t3;
            }

            const double tau = std::clamp(seg_t / seg_T, 0.0, 1.0);
            const double s = 0.5 * (1.0 - std::cos(M_PI * tau));
            const double s_dot = 0.5 * M_PI / seg_T * std::sin(M_PI * tau);
            const double s_ddot = 0.5 * (M_PI * M_PI) / (seg_T * seg_T) * std::cos(M_PI * tau);

            const double delta = q_start + (q_end - q_start) * s;
            const double vel = (q_end - q_start) * s_dot;
            const double acc = (q_end - q_start) * s_ddot;

            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions = jnt_pos;
            p.velocities.assign(dof, 0.0);
            p.accelerations.assign(dof, 0.0);

            for (size_t j = 0; j < signs.size(); ++j) {
                const double s = static_cast<double>(signs[j]);
                p.positions[j] = jnt_pos[j] + s * delta;
                p.velocities[j] = s * vel;
                p.accelerations[j] = s * acc;
            }

            p.time_from_start = make_duration_msg(t);
            traj.points.push_back(std::move(p));
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory = std::move(traj);
        goal.goal_time_tolerance = make_duration_msg(0.5);

        auto goal_future = trajectory_action_client_->async_send_goal(goal);
        if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Sending trajectory goal timed out");
            return false;
        }

        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory goal was rejected");
            return false;
        }

        auto result_future = trajectory_action_client_->async_get_result(goal_handle);
        const auto wait_timeout = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(total_time + 10.0));

        if (result_future.wait_for(wait_timeout) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Waiting for trajectory execution result timed out");
            return false;
        }

        const auto wrapped_result = result_future.get();
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed，action result code: %d", static_cast<int>(wrapped_result.code));
            return false;
        }

        if (wrapped_result.result && wrapped_result.result->error_code != 0) {
            RCLCPP_ERROR(this->get_logger(), "Controller returns error code: %d", wrapped_result.result->error_code);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Three-segment cosine trajectory execution completed（0 -> +A -> -A -> 0）");
        return true;
    }

    bool move_to_pre_position(const std::vector<double>& first_six_target)
    {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface Uninitialized");
            return false;
        }

        auto& arm = *move_group_;
        const auto current = arm.getCurrentJointValues();
        const size_t dof = current.size();

        if (dof < first_six_target.size()) {
            RCLCPP_ERROR(this->get_logger(), "Insufficient degrees of freedom, expected at least %zu, actual %zu", first_six_target.size(), dof);
            return false;
        }

        auto target = current;
        for (size_t i = 0; i < first_six_target.size(); ++i) {
            target[i] = first_six_target[i];
        }

        arm.setPlanningTime(20.0);
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.01);
        arm.setGoalOrientationTolerance(0.01);
        arm.setMaxAccelerationScalingFactor(0.1);
        arm.setMaxVelocityScalingFactor(0.1);
        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Pre-position pose planning failed");
            return false;
        }

        if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Pre-position pose execution failed");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Arrived at the pre-position (0.5, 0.5, 0.5, 0.5, 0.5, 0.5)");
        return true;
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
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

    // 5) First move to the pre-position pose
    const std::vector<double> pre_position = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    bool ok = node->move_to_pre_position(pre_position);
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to pre-position pose");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    // 6) Send three segments of cosine trajectories at once：0 -> +A -> -A -> 0
    const std::vector<int> signs = {+1, +1, -1, +1, -1, +1};
    ok = node->execute_cosine_s_curve_trajectory(
        2.0,   // t1: 0 -> +A
        2.0,   // t2: +A -> -A
        2.0,   // t3: -A -> 0
        0.02,  // dt
        signs
    );
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Sequential movement execution failed");
    }

    // 6) End: shutdown and wait for spinner to exit
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
