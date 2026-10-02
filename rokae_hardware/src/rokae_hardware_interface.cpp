#include <vector>
#include <string>
#include <thread>
#include <fstream>  // Add header file
#include <iostream>
#include <sstream>
#include <chrono>
#include <cmath>
#include <algorithm>

// Rokae sdk
#include <rokae/robot.h>
#include <rokae/data_types.h>
#include <rokae/motion_control_rt.h>
//#include "rokae_msgs/include/rokae_msgs/rokae_msgs/msg/external_force.h"
#include "rokae_hardware/rokae_hardware_interface.h"
#include "pluginlib/class_list_macros.hpp"



namespace rokae_hardware
{
namespace
{
template <typename T>
std::string vector_to_string(const std::vector<T> &values)
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}
}  // namespace

    //using Rokae6DOFHardware = RokaeHardwareInterface<6>;
    //RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start() called");

template <unsigned short DoF>
RokaeHardwareInterface<DoF>::RokaeHardwareInterface()
//RokaeHardwareInterface::RokaeHardwareInterface()
{
    //RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start() called");

}


template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_init(const hardware_interface::HardwareInfo & info)   //controll managerThe configure parameters will only be called after the controller starts.
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)   //controll managerThe configure parameters will only be called after the controller starts.
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "rokae get joint start()");

    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Reading hardware parameters:");
    info_ = info;   //mlgbDo not save info to member variablesinfo_Read an egg joint count

// 1. Directly frominfo_.jointsGet joint names (do not follow upresize!）
    joint_names_.clear();
    for (const auto& joint : info_.joints) {
        joint_names_.push_back(joint.name);
    }
    std::cout << num_joints_ << std::endl;
    std::cout << joint_position_state_.size() << std::endl;
    //num_joints_ = joint_names_.size();

    joint_position_state_.resize(DoF,0.0);
    joint_velocity_state_.resize(DoF,0.0);
    joint_torque_state_.resize(DoF,0.0);

    // External force
    ext_force_in_stiff_.resize(DoF, 0.0);
    ext_force_in_base_.resize(DoF, 0.0);

    // Controller
    // joint_position_controller_running_ = false;
    // joint_velocity_controller_running_ = false;
    // //joint_torque_controller_running_ = false;
    // controllers_initialized_ = false;

    // Commands
    joint_position_command_.resize(DoF,0.0);
    joint_velocity_command_.resize(DoF,0.0);
    joint_torque_command_.resize(DoF,0.0);
    internal_joint_position_command_.resize(DoF,0.0);

    std::cout << DoF << std::endl;
    std::cout << joint_position_state_.size() << std::endl;

    auto it_servo_t = info_.hardware_parameters.find("servo_joint_period_s");
    if (it_servo_t != info_.hardware_parameters.end()) {
        try {
            const double v = std::stod(it_servo_t->second);
            if (v > 0.0 && v < 1.0) {
                servo_joint_period_s_ = v;
            } else {
                RCLCPP_WARN(
                    rclcpp::get_logger("RokaeHardwareInterface"),
                    "servo_joint_period_s=%s out of range (0,1), keep default %.6f",
                    it_servo_t->second.c_str(), servo_joint_period_s_);
            }
        } catch (const std::exception &) {
            RCLCPP_WARN(
                rclcpp::get_logger("RokaeHardwareInterface"),
                "Invalid servo_joint_period_s '%s', keep default %.6f",
                it_servo_t->second.c_str(), servo_joint_period_s_);
        }
    }
    RCLCPP_INFO(
        rclcpp::get_logger("RokaeHardwareInterface"),
        "servo_joint_period_s (setServoJoint T) = %.6f s (~%.0f Hz)",
        servo_joint_period_s_, 1.0 / servo_joint_period_s_);

    auto it_enable_servoj = info_.hardware_parameters.find("enable_servoj");
    if (it_enable_servoj != info_.hardware_parameters.end()) {
        const auto &v = it_enable_servoj->second;
        enable_servoj_ = (v == "true" || v == "1" || v == "True" || v == "TRUE");
    }
    RCLCPP_INFO(
        rclcpp::get_logger("RokaeHardwareInterface"),
        "enable_servoj=%s (false=Skip setServoJoint, only startMove)",
        enable_servoj_ ? "true" : "false");

    return hardware_interface::CallbackReturn::SUCCESS;
}


// ROS2Interface Registration
// Register state interfaces - inform the ROS2 Control framework which state data the hardware can provide (data read from the hardware)）
template <unsigned short DoF>
std::vector<hardware_interface::StateInterface> RokaeHardwareInterface<DoF>::export_state_interfaces()
//std::vector<hardware_interface::StateInterface> RokaeHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start called export_state_interfaces");
    std::cout << DoF << std::endl;
    RCLCPP_INFO(
    rclcpp::get_logger("RokaeHardwareInterface"),
    "Joint count matches! Expected: %u, Actual: %zu", DoF, joint_names_.size());   //Be especially careful, it was originally written as"Joint count matches! Expected: %ld, Actual: %ld", DoF, joint_names_.size());It may likely cause a mismatch between formatting parameters and actual types, resulting in memory corruption, which in turn leads to stack smashing detected。

    for (const auto& name : joint_names_)
    {
        RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Joint name: %s\n", name.c_str());
        std::cout << "Joint name: " << name << std::endl;
    }
    

    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (size_t i = 0; i < info_.joints.size(); ++i) {
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_state_[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]));

    //state_interfaces.emplace_back(hardware_interface::StateInterface(
        //info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_torque_state_[i]));
  //}


    for (size_t i = 0; i < DoF; ++i) {
        RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Export state interface: %s/position", joint_names_[i].c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "position", &joint_position_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "velocity", &joint_velocity_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "effort", &joint_torque_state_[i]));
    }
    return state_interfaces;
}

//Register command interface - informs the ROS2 Control framework of which command data the hardware can receive (control commands sent to the hardware)）
template <unsigned short DoF>
std::vector<hardware_interface::CommandInterface> RokaeHardwareInterface<DoF>::export_command_interfaces()
//std::vector<hardware_interface::CommandInterface> RokaeHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start called export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < DoF; ++i) {
        RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Export command interface: %s/position", joint_names_[i].c_str());
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "position", &joint_position_command_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "velocity", &joint_velocity_command_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "effort", &joint_torque_command_[i]));
    }

    return command_interfaces;
}


template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_configure(const rclcpp_lifecycle::State & previous_state)    //const It is to ensure that the function internals cannot be modified previous_state，It only allows reading the current state of the object and does not allow it to be modified.  
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "rokae get robot_ip start()");

    auto it_robot = info_.hardware_parameters.find("robot_ip");
    if (it_robot != info_.hardware_parameters.end()) {
        robot_ip_ = it_robot->second;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "No 'robot_ip' found in hardware info.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 2. Obtain local_ip
    auto it_local = info_.hardware_parameters.find("local_ip");
    if (it_local != info_.hardware_parameters.end()) {
        local_ip_ = it_local->second;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "No 'local_ip' found in hardware info.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3. Obtain rt_network_tolerance（Default 80）
    auto it_tol = info_.hardware_parameters.find("rt_network_tolerance");
    if (it_tol != info_.hardware_parameters.end()) {
        try {
            int value = std::stoi(it_tol->second);
            if (value < 0) value = 0;
            if (value > 100) value = 100;
            rt_network_tolerance_ = static_cast<unsigned>(value);
        } catch (...) {
            RCLCPP_WARN(
                rclcpp::get_logger("RokaeHardwareInterface"),
                "Invalid rt_network_tolerance '%s', fallback to default %u",
                it_tol->second.c_str(), rt_network_tolerance_);
        }
    }

        
    if (!initRobot()) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot initialization failed.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3. Print Check
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("RokaeHardwareInterface"),
        "Using robot_ip: " << robot_ip_
        << ", local_ip: " << local_ip_
        << ", rt_network_tolerance: " << rt_network_tolerance_);

    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "on_configure() finished successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;

}


template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_activate(const rclcpp_lifecycle::State & previous_state)   
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)    //To connect the robot, it is necessary to call the base class interface functions in ROS2.
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "on_activate() called");

    try {
        robot_->updateRobotState(std::chrono::milliseconds(1));
    } catch (const std::exception &e) {
        RCLCPP_WARN(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "on_activate: updateRobotState: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "on_activate: updateRobotState threw non-std exception");
    }

    const int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_position_state_);
    const int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocity_state_);
    const int ret_torque = robot_->getStateData("tau_m", joint_torque_state_);
    RCLCPP_INFO(
        rclcpp::get_logger("RokaeHardwareInterface"),
        "on_activate: initial RT getStateData ret pos=%d vel=%d tau=%d | joint_pos=%s | tau=%s",
        ret_pos, ret_vel, ret_torque,
        vector_to_string(joint_position_state_).c_str(),
        vector_to_string(joint_torque_state_).c_str());
    
    // The initialization command is the current state
    for (size_t i = 0; i < DoF; ++i) {
        joint_position_command_[i] = joint_position_state_[i];
        joint_velocity_command_[i] = joint_velocity_state_[i];
        joint_torque_command_[i] = joint_torque_state_[i];
    }
    setInitPosition();
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Controller init successful!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_deactivate(const rclcpp_lifecycle::State & previous_state)   
//Ensure the completeness of the robot's lifecycle
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "on_deactivate() called");
    //rci_->stopLoop();    // Stop the internal MotionControl thread to ensure that before the end of the lifecycle, the SDK loop has completely exited, preventing the SDK internal thread from still calling callbacks after the ROS 2 hardware interface object is destroyed, which would cause a dangling this access and potentially hang the entire thread.
    // Here you can disconnect the robot or clean up resources
        if (rci_) {
        rci_->stopLoop();  // Stop the internal thread
        rci_.reset();      // Clear the smart pointer to avoid access after destruction
    }

    // if (robot_) {
    //     robot_.reset();    // Ensure robot_ Also destruct
    // }
    return hardware_interface::CallbackReturn::SUCCESS;
}


template <unsigned short DoF>
bool RokaeHardwareInterface<DoF>::initRobot()
//bool RokaeHardwareInterface::initRobot()
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start connect rokae");
    try {
        if constexpr (DoF == 7) {
            // 7-axis families (AR/Pro/SR4) must use 7-axis SDK robot class.
            robot_ = std::make_shared<rokae::xMateErProRobot>(robot_ip_, local_ip_);
        } else {
            // 5/6-axis families keep the original xMateRobot class.
            robot_ = std::make_shared<rokae::xMateRobot>(robot_ip_, local_ip_);
        }
    } catch (const rokae::NetworkException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot instantiation failed: %s", e.what());
        return false;
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot type mismatch: %s", e.what());
        return false;
    }

    try {
        robot_->connectToRobot(ec);
        if (ec.value() != 0)
            throw rokae::ExecutionException("connect failed");
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot connection failed: %s", e.what());
        return false;
    }

    try {
        robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        // Must be configured before switching to RtCommand (SDK note).
        ec.clear();
        robot_->setRtNetworkTolerance(rt_network_tolerance_, ec);
        if (ec.value() != 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("RokaeHardwareInterface"),
                "setRtNetworkTolerance(%u) failed: %s (code=%d)",
                rt_network_tolerance_, ec.message().c_str(), ec.value());
        }
        //robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        ec.clear();
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        robot_->setPowerState(true, ec);
        if (ec.value() != 0)
            throw rokae::ExecutionException("connect failed");
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Could not enable robot: %s", e.what());
        return false;
    }

    try {
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        auto base_rci = robot_->getRtMotionController().lock();
        rci_ = std::dynamic_pointer_cast<RtType>(base_rci);
        rci_->setFilterLimit(true, 50);
        robot_->startReceiveRobotState(std::chrono::milliseconds(1), {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
                                      rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tauExt_inBase,
                                      rokae::RtSupportedFields::tauExt_inStiff});
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Could not set realtime control mode: %s", e.what());
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Robot connected successfully");
    return true;
}

template <unsigned short DoF>
void RokaeHardwareInterface<DoF>::setInitPosition()
//void RokaeHardwareInterface::setInitPosition()
{
    auto robot_joint_postion = robot_->jointPos(ec);
    for (std::size_t i = 0; i < DoF; i++)
        joint_position_state_[i] = robot_joint_postion[i];
    joint_position_command_ = joint_position_state_;
}


// template <unsigned short DoF>
// hardware_interface::return_type RokaeHardwareInterface<DoF>::read(const rclcpp::Time&, const rclcpp::Duration&)
// //hardware_interface::return_type RokaeHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
// {
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start reading data from the robotread() called");
   
//     int update_success = robot_->updateRobotState(std::chrono::milliseconds(1));  ///Return 0 or 268

//     // Read status data, getStateData returning 0 usually indicates success

//     int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_position_state_);
//     int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocity_state_);
//     int ret_torque = robot_->getStateData("tau_m", joint_torque_state_);

//     if (ret_pos != 0) {
//         RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint position");
//         return hardware_interface::return_type::ERROR;
//     }
//     if (ret_vel != 0) {
//         RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint velocity");
//     }
//     if (ret_torque != 0) {
//         RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint torque");
//     }

//     internal_joint_position_command_ = joint_position_state_;
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start reading data from the robotread() called");
//     return hardware_interface::return_type::OK;
template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::read(const rclcpp::Time&, const rclcpp::Duration &period)
{
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start reading data from the robotread() called");
    static bool logged_first_successful_read = false;

    // Inspection cycle time
    double period_ms = period.seconds() * 1000.0;
    if (period_ms>10) {
        RCLCPP_INFO(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "readCycle: %.3fms", period_ms);
    }
   
    try {
        int update_success = robot_->updateRobotState(std::chrono::milliseconds(1));

        int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_position_state_);
        int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocity_state_);
        int ret_torque = robot_->getStateData("tau_m", joint_torque_state_);

        // Some robots (e.g. CR35) may leave RtSupportedFields::jointPos_m at ~0 while tau_m updates.
        // That would overwrite good values from setInitPosition()/jointPos(ec). Fall back to NRT jointPos.
        if (ret_pos == 0) {
            const bool rt_pos_all_near_zero = std::all_of(
                joint_position_state_.begin(), joint_position_state_.end(),
                [](double p) { return std::fabs(p) < 1e-8; });
            static auto last_jointpos_nrt_poll = std::chrono::steady_clock::now();
            static bool logged_jointpos_fallback = false;
            const auto now_poll = std::chrono::steady_clock::now();
            if (rt_pos_all_near_zero &&
                (now_poll - last_jointpos_nrt_poll) >= std::chrono::milliseconds(50)) {
                last_jointpos_nrt_poll = now_poll;
                try {
                    const auto jp = robot_->jointPos(ec);
                    for (size_t i = 0; i < DoF; ++i) {
                        joint_position_state_[i] = jp[i];
                    }
                    if (!logged_jointpos_fallback) {
                        logged_jointpos_fallback = true;
                        RCLCPP_WARN(
                            rclcpp::get_logger("RokaeHardwareInterface"),
                            "RT stream jointPos_m was all ~0; using jointPos(ec) for /joint_states positions "
                            "(polled every 50 ms while RT positions stay zero). Verify SDK RT fields for this model.");
                    }
                } catch (const std::exception &e) {
                    static auto last_jointpos_fallback_err = std::chrono::steady_clock::now();
                    const auto nw = std::chrono::steady_clock::now();
                    if (nw - last_jointpos_fallback_err >= std::chrono::seconds(5)) {
                        last_jointpos_fallback_err = nw;
                        RCLCPP_WARN(
                            rclcpp::get_logger("RokaeHardwareInterface"),
                            "jointPos(ec) fallback failed: %s", e.what());
                    }
                }
            }
        }

        if (!logged_first_successful_read && ret_pos == 0) {
            logged_first_successful_read = true;
            RCLCPP_INFO(
                rclcpp::get_logger("RokaeHardwareInterface"),
                "read: first successful getStateData (ret pos=%d vel=%d tau=%d update_ret=%d) "
                "joint_pos=%s joint_vel=%s tau=%s — per-cycle getStateData INFO logs disabled.",
                ret_pos, ret_vel, ret_torque, update_success,
                vector_to_string(joint_position_state_).c_str(),
                vector_to_string(joint_velocity_state_).c_str(),
                vector_to_string(joint_torque_state_).c_str());
        }
        
        if (ret_pos != 0) {
            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint position");
            return hardware_interface::return_type::ERROR;
        }
        if (ret_vel != 0) {
            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint velocity");
        }
        if (ret_torque != 0) {
            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint torque");
        }

        internal_joint_position_command_ = joint_position_state_;
        RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Successfully read data from the robot");

        return hardware_interface::return_type::OK;

    } catch (const rokae::RealtimeMotionException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), 
                     "Real-time motion anomaly: %s", e.what());  
        static auto last_recover_try = std::chrono::steady_clock::now() - std::chrono::seconds(1);
        const auto now = std::chrono::steady_clock::now();
        if ((now - last_recover_try) < std::chrono::milliseconds(200))
        {
            return hardware_interface::return_type::OK;
        }
        last_recover_try = now;

        try
        {
            if (!robot_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "readRecovery failed: robot_empty");
                return hardware_interface::return_type::ERROR;
            }

            robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
            if (ec.value() != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                             "readRecovery failed: setMotionControlMode error: %s", ec.message().c_str());
                return hardware_interface::return_type::ERROR;
            }

            auto base_rci = robot_->getRtMotionController().lock();
            auto new_rci = std::dynamic_pointer_cast<RtType>(base_rci);
            if (!new_rci)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "readRestore failed: Failed to get RT controller");
                return hardware_interface::return_type::ERROR;
            }
            rci_ = new_rci;

            setInitPosition();
            internal_joint_position_command_ = joint_position_state_;
            joint_position_command_ = joint_position_state_;
            joint_velocity_command_.assign(DoF, 0.0);
            joint_torque_command_.assign(DoF, 0.0);

            // Must stop any existing motion session before starting a new one.
            // Skipping this causes "Exercise has already started, please do not call it repeatedly" from startMove().
            try { rci_->stopMove(); } catch (...) {}

            if (joint_position_controller_running_)
            {
                rci_->startMove(rokae::RtControllerMode::jointPosition);
            }
            else if (joint_torque_controller_running_)
            {
                rci_->startMove(rokae::RtControllerMode::torque);
            }

            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "readThe stage has automatically resumed the RT session");
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &recover_e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                         "readRecovery exception: %s", recover_e.what());
            return hardware_interface::return_type::ERROR;
        }

    } 
}


// template <unsigned short DoF>
// hardware_interface::return_type RokaeHardwareInterface<DoF>::write(const rclcpp::Time&, const rclcpp::Duration &period)
// //hardware_interface::return_type RokaeHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration &period)
// {
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start writing data to the robotwrite() called");
//     // enforceLimits(period); // You need to implement the limiting logic yourself
//     //robot_->updateRobotState(std::chrono::milliseconds(1));
    
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start writing data to the robotwrite() called");
//     return hardware_interface::return_type::OK;
// }

template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::write(const rclcpp::Time&, const rclcpp::Duration &period)
{
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start writing data to the robotwrite() called");
    // Inspection cycle time
    double period_ms = period.seconds() * 1000.0;
    if (period_ms>10) {
        RCLCPP_INFO(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "writeCycle: %.3fms", period_ms);
    }
    
    // Commands are only sent when the position controller is running.
    if (joint_position_controller_running_ ) {
        try {
            // Send location command normally
            rokae::JointPosition jcmd(DoF);
            for (size_t i = 0; i < DoF; i++) {
                jcmd.joints[i] = joint_position_command_[i];
            }
            
            if (rci_) {
                rci_->sendCommand(jcmd);
                // busy_wait(planPeriod*1000);
            }
            
            RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), 
                        "Send location command: %s",
                        vector_to_string(joint_position_command_).c_str());
                        
        } 
        catch (const rokae::RealtimeMotionException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                        "Write command real-time exception: %s", e.what());

            static auto last_recover_try = std::chrono::steady_clock::now() - std::chrono::seconds(1);
            const auto now = std::chrono::steady_clock::now();
            if ((now - last_recover_try) < std::chrono::milliseconds(200))
            {
                return hardware_interface::return_type::OK;
            }
            last_recover_try = now;

            try
            {
                if (!robot_)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "writeRecovery failed: robot_empty");
                    return hardware_interface::return_type::ERROR;
                }

                // Stop any existing motion before reconfiguring RT mode.
                // Wrap in try-catch because stopMove() may itself throw if motion
                // was never successfully started (e.g. after a failed read() recovery).
                if (rci_)
                {
                    try { rci_->stopMove(); } catch (...) {}
                }

                robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
                if (ec.value() != 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                                 "writeRecovery failed: setMotionControlMode error: %s", ec.message().c_str());
                    return hardware_interface::return_type::ERROR;
                }

                auto base_rci = robot_->getRtMotionController().lock();
                auto new_rci = std::dynamic_pointer_cast<RtType>(base_rci);
                if (!new_rci)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "writeRestore failed: Failed to get RT controller");
                    return hardware_interface::return_type::ERROR;
                }
                rci_ = new_rci;

                // Sync all command buffers to actual robot position so the next
                // write() does not send a stale mid-trajectory position command.
                setInitPosition();
                internal_joint_position_command_ = joint_position_state_;
                joint_position_command_ = joint_position_state_;
                joint_velocity_command_.assign(DoF, 0.0);
                joint_torque_command_.assign(DoF, 0.0);

                if (joint_position_controller_running_)
                {
                    rci_->startMove(rokae::RtControllerMode::jointPosition);
                }
                else if (joint_torque_controller_running_)
                {
                    rci_->startMove(rokae::RtControllerMode::torque);
                }

                RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "writeThe stage has automatically resumed the RT session");
                return hardware_interface::return_type::OK;
            }
            catch (const std::exception& recover_e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                            "writeRecovery exception: %s", recover_e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), 
                        "Failed to write command: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }
    
    return hardware_interface::return_type::OK;
}

// Controller switching interface: ROS2 standard
template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::prepare_command_mode_switch
(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
// hardware_interface::return_type RokaeHardwareInterface::prepare_command_mode_switch(
//     const std::vector<std::string> &start_interfaces,
//     const std::vector<std::string> &stop_interfaces)
{
    // Collect the pattern of start
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "prepare_command_mode_switch() called");
    std::vector<std::string> start_modes;
    for (const auto &key : start_interfaces) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const auto joint_full_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const auto joint_full_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            const auto joint_full_eff = info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;
            if (key == joint_full_pos) start_modes.push_back("position");
            if (key == joint_full_vel) start_modes.push_back("velocity");
            if (key == joint_full_eff) start_modes.push_back("effort");
        }
    }

    // Either none are switched, or each joint is assigned a new mode
    if (!start_modes.empty() && start_modes.size() != info_.joints.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                     "prepare_command_mode_switch: start_modes size %zu != joints %zu",
                     start_modes.size(), info_.joints.size());
        return hardware_interface::return_type::ERROR;
    }

    // All start_modes Must be the same
    if (!start_modes.empty() &&
        !std::equal(start_modes.begin() + 1, start_modes.end(), start_modes.begin())) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                     "prepare_command_mode_switch: start modes are not uniform");
        return hardware_interface::return_type::ERROR;
    }

    // stop By the same logic (requirements are consistent)）
    std::vector<std::string> stop_modes;
    for (const auto &key : stop_interfaces) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const auto joint_full_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const auto joint_full_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            const auto joint_full_eff = info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;
            if (key == joint_full_pos) stop_modes.push_back("position");
            if (key == joint_full_vel) stop_modes.push_back("velocity");
            if (key == joint_full_eff) stop_modes.push_back("effort");
        }
    }
    if (!stop_modes.empty() && (stop_modes.size() != info_.joints.size() ||
        !std::equal(stop_modes.begin() + 1, stop_modes.end(), stop_modes.begin()))) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                     "prepare_command_mode_switch: stop modes invalid");
        return hardware_interface::return_type::ERROR;
    }

    // through
    controllers_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),
                "prepare_command_mode_switch: OK, start_interfaces=%zu stop_interfaces=%zu",
                start_interfaces.size(), stop_interfaces.size());
    return hardware_interface::return_type::OK;
}



template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::perform_command_mode_switch
(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
// hardware_interface::return_type RokaeHardwareInterface::perform_command_mode_switch(
//     const std::vector<std::string> &start_interfaces,
//     const std::vector<std::string> &stop_interfaces)
{
    // Stop the required interface (only need to execute according to the first stop interface type)）
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "perform_command_mode_switch() called");
    if (!stop_interfaces.empty()) {
        const auto &first = stop_interfaces.front();
        if (first.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
            joint_position_controller_running_ = false;
            if (rci_) rci_->stopMove();
        } else if (first.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
            joint_velocity_controller_running_ = false;
            if (rci_) rci_->stopMove();
        } else if (first.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
            joint_torque_controller_running_ = false;
            if (rci_) rci_->stopMove();
        }
    }

    // Start a new interface (according to the first start interface）
    if (!start_interfaces.empty()) {
        const auto &first = start_interfaces.front();
        if (first.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
            joint_position_controller_running_ = true;
            joint_velocity_controller_running_ = false;
            joint_torque_controller_running_ = false;
            if (rci_) {
                if (enable_servoj_) {
                    ec.clear();
                    rci_->setServoJoint(
                        servo_joint_period_s_, servo_joint_period_s_ * 3.0, 1.0, ec);
                    if (ec) {
                        RCLCPP_WARN(
                            rclcpp::get_logger("RokaeHardwareInterface"),
                            "setServoJoint failed: %s (code=%d), continue with startMove only",
                            ec.message().c_str(), ec.value());
                    }
                }
                rci_->startMove(rokae::RtControllerMode::jointPosition);
            }
        } else if (first.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
            joint_velocity_controller_running_ = true;
            joint_position_controller_running_ = false;
            joint_torque_controller_running_ = false;
            if (rci_) {
                if (enable_servoj_) {
                    ec.clear();
                    rci_->setServoJoint(
                        servo_joint_period_s_, servo_joint_period_s_ * 3.0, 1.0, ec);
                    if (ec) {
                        RCLCPP_WARN(
                            rclcpp::get_logger("RokaeHardwareInterface"),
                            "setServoJoint (velocity switch) failed: %s (code=%d), continue with startMove only",
                            ec.message().c_str(), ec.value());
                    }
                }
                rci_->startMove(rokae::RtControllerMode::jointPosition);
            }
        } else if (first.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
            joint_torque_controller_running_ = true;
            joint_position_controller_running_ = false;
            joint_velocity_controller_running_ = false;
            if (rci_) {
                rci_->stopMove();
                rci_->startMove(rokae::RtControllerMode::torque);
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),
                "perform_command_mode_switch done. pos=%d vel=%d eff=%d",
                joint_position_controller_running_, joint_velocity_controller_running_,
                joint_torque_controller_running_);

    //Activated at startup[RokaeHardwareInterface]: perform_command_mode_switch done. pos=1 vel=0 eff=0

    if (joint_position_controller_running_) {
        //RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "Start writing data to the robotwrite() called");
        //std::function<rokae::JointPosition()> callback2 = std::bind(&RokaeHardwareInterface::callback, this);
        //rokae::JointPosition joints_value=callback2();
        
        // std::function<rokae::JointPosition()> callback = [this]()
        // {
        //     // joint_position_command_ It is set by the ROS2 controller through the command interface, so in theory, as long as the controller is updated joint_position_command_，The callback function will use the latest value.
        //     // RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),"Using callback functions");
        //     rokae::JointPosition jcmd(num_joints_);
        //     for (size_t i = 0; i < num_joints_; i++)
        //     {
        //         jcmd.joints[i] = joint_position_command_[i];
        //     }
        //     this->time_us2 = time_us1;
        //     this->time_us1 = getLocalTimeUs();
        //     //std::this_thread::sleep_for(std::chrono::microseconds(50));
        //     return jcmd;
        // };
        // this->rci_->setControlLoop(callback, 0, true);
        // this->rci_->startLoop(false);
    }
    else if (joint_velocity_controller_running_) {
        //for (std::size_t i = 0; i < num_joints_; i++)
            //internal_joint_position_command_[i] += joint_velocity_command_[i] * period.seconds();
    }
    else if (joint_torque_controller_running_) {
        // rci_->appendCommand(rokae::Torque(joint_torque_command_));
    }

    return hardware_interface::return_type::OK;
}



// Force control limiting (need to implement the logic yourself)）
template <unsigned short DoF>
void RokaeHardwareInterface<DoF>::enforceLimits(const rclcpp::Duration& period)
//void RokaeHardwareInterface::enforceLimits(const rclcpp::Duration& period)
{
    // TODO: usejoint_limits_interfaceOr customize amplitude limiting logic
}

// Publish with control (it is recommended to use rclcpp::Publisher, add your own locking)）
template <unsigned short DoF>
void RokaeHardwareInterface<DoF>::publishExternalForce()
//void RokaeHardwareInterface::publishExternalForce()
{
    // TODO: rclcpp::Publisher<rokae_msgs::msg::ExternalForce>::SharedPtr
}

//servoj Interface
template<unsigned short DoF>
void RokaeHardwareInterface<DoF>::busy_wait(int milliseconds) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < milliseconds) {
    }
}



template class RokaeHardwareInterface<5>;
template class RokaeHardwareInterface<6>;
template class RokaeHardwareInterface<7>;

using RokaeHardwareInterface5 = RokaeHardwareInterface<5>;
using RokaeHardwareInterface6 = RokaeHardwareInterface<6>;
using RokaeHardwareInterface7 = RokaeHardwareInterface<7>;

} // namespace rokae_hardware

PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface5,hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface6,hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface7,hardware_interface::SystemInterface)
//PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface, hardware_interface::SystemInterface)
