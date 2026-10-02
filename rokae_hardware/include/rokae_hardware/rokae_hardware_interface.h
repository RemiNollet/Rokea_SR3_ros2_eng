#include <memory>
#include <string>
#include <type_traits>
#include <vector>

// ROS interface
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

//#include <urdf/model.h>

#include <realtime_tools/realtime_publisher.h>

// ROS control interface
// hardware_interface
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"



// #include "hardware_interface/joint_command_handle.hpp"
// #include "hardware_interface/joint_state_handle.hpp"
// Controller interface


#include <iostream>
// Rokae sdk
#include <rokae/robot.h>
#include <rokae/data_types.h>
//#include "rokae_msgs/rokae_msgs/msg/external_force.h"
#include "stdlib.h"


namespace rokae_hardware    // Override SystemInterface virtuals so ROS 2 can call RokaeHardwareInterface methods
{
    template <unsigned short DoF>
    class RokaeHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // RokaeHardwareInterface() = default;
        RCLCPP_SHARED_PTR_DEFINITIONS(RokaeHardwareInterface)
        RokaeHardwareInterface();
        virtual ~RokaeHardwareInterface()
        {
            // RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start() called");
            if (robot_) {
                robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
            }
        }

        

        // ROS 2 lifecycle interface using rclcpp::Node::SharedPtr
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::return_type start();
        hardware_interface::return_type stop();

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        // Other functions can be implemented through custom functions
        bool initParameters(std::shared_ptr<rclcpp::Node> node);
        bool initRobot();
        bool waitForValidState(size_t max_attempts = 50, int delay_ms = 100);
        void setInitPosition();
        bool initROSInterface(std::shared_ptr<rclcpp::Node> node);
        void publishExternalForce();

        void enforceLimits(const rclcpp::Duration & period);
        void busy_wait(int milliseconds);

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        bool checkControllerClaims(const std::set<std::string> & claimed_resources);
        

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        

    public: 
        using RobotType = typename std::conditional<(DoF == 7), rokae::xMateErProRobot, rokae::xMateRobot>::type;
        std::shared_ptr<RobotType> robot_;
        std::string robot_ip_;
        std::string local_ip_;
        unsigned rt_network_tolerance_ = 80;
        std::error_code ec;
        //const → Immutable ；static → All objects share one copy
        static const size_t num_joints_ = DoF;   // const: immutable after init; static: shared class member. Axis count comes from template DoF, so it is not stored per instance.
        //size_t num_joints_ = DoF;
        //size_t num_joints_ = 7;
        std::vector<std::string> joint_names_;

        using RtType = rokae::RtMotionControl<rokae::WorkType::collaborative, DoF>;
        std::shared_ptr<RtType> rci_; //

        std::vector<double> joint_position_state_;
        std::vector<double> joint_velocity_state_;
        std::vector<double> joint_torque_state_;

        // External force
        std::vector<double> ext_force_in_stiff_;
        std::vector<double> ext_force_in_base_;

        // Commands
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_torque_command_;
        std::vector<double> internal_joint_position_command_;
         // Custom failure flag, needs to be set by an external caller (such as a controller status callback)
        std::atomic<bool> controller_aborted_{false};
        // Current joint status and command buffer
        std::vector<double> state_positions_;
        std::vector<double> cmd_positions_;

        // Controller
        bool joint_position_controller_running_ = false;
        bool joint_velocity_controller_running_ = false;
        bool joint_torque_controller_running_ = false;
        bool controllers_initialized_ = false;

        long times_loop_ = 0;
        /// ServoJ_T for setServoJoint(ServoJ_T, ...), in seconds; should match the ros2_control write period. Default 0.004 ≈ 250 Hz.
        double servo_joint_period_s_ = 0.004;
        /// If true, call SDK setServoJoint. Some firmware reports invalid data key(-258); default false matches the older .bak behavior.
        bool enable_servoj_ = false;

        // ROS 2 publisher example
        // rclcpp::Publisher<rokae_msgs::msg::ExternalForce>::SharedPtr ext_force_in_stiff_pub_;
        // rclcpp::Publisher<rokae_msgs::msg::ExternalForce>::SharedPtr ext_force_in_pub_;

        // pvi
        long long time_us1 = 0;
        long long time_us2 = 0;
        unsigned long long getLocalTimeUs()
        {
            auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
            return tp.time_since_epoch().count();
        }
    };
}
