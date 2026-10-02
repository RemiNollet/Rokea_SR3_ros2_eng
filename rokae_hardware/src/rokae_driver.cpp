#include <rclcpp/rclcpp.hpp>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "rokae_msgs/srv/move_j.hpp"   //Check the corresponding name in install when an error occurs
#include "rokae_msgs/srv/move_c.hpp"
#include "rokae_msgs/srv/move_l.hpp"
#include "rokae_msgs/srv/get_robot_info.hpp"
#include "rokae_msgs/srv/calculate_ik.hpp"
#include "rokae_msgs/srv/calculate_fk.hpp"
#include "rokae_msgs/srv/drag_con.hpp"
#include "rokae_msgs/srv/jog_con.hpp"
#include "rokae_msgs/srv/get_do.hpp"
#include "rokae_msgs/srv/set_do.hpp"
#include "rokae_msgs/srv/get_di.hpp"
#include "rokae_msgs/srv/set_di.hpp"
#include "rokae_msgs/srv/read_register.hpp"
#include "rokae_msgs/srv/write_register.hpp"


using namespace rokae;
using sensor_msgs::msg::JointState;


rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_pub;

// Create a robot object
rokae::xMateRobot robot; ////Six-axis
// rokae::xMateErProRobot robot; //// Seven-axis
std::error_code ec;
// const std::string local_ip = "192.168.2.100";
// const std::string robot_ip = "192.168.2.160";



//  Basic Information Inquiry Service
bool get_robot_info_callback(
    const std::shared_ptr<rokae_msgs::srv::GetRobotInfo::Request> /*request*/,
    std::shared_ptr<rokae_msgs::srv::GetRobotInfo::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "get_robot_info called");
        
        auto robot_info = robot.robotInfo(ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to get robot info: " + ec.message();
            return false;
        }
        
        response->success = true;
        response->message = "Robot info retrieved successfully";
        response->version = robot_info.version;
        response->type = robot_info.type;
        response->sdk_version = robot.sdkVersion();
        
        // // Get current joint state
        // auto joint_pos = robot.jointPos(ec);
        // if (!ec) {
        //     response->joint_positions.assign(joint_pos.begin(), joint_pos.end());
        // }
        
        // // Get current Cartesian pose
        // auto posture = robot.posture(CoordinateType::flangeInBase, ec);
        // if (!ec) {
        //     response->cartesian_pose.assign(posture.begin(), posture.end());
        // }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

bool stop_rt_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    try {
        std::error_code local_ec;

        auto state_before = robot.operationState(local_ec);
        if (local_ec) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "Failed to read initial operationState: %s", local_ec.message().c_str());
            local_ec.clear();
        }

        if (state_before == rokae::OperationState::rlProgram) {
            robot.pauseProject(local_ec);
            if (local_ec) {
                RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "pauseProjectFailure: %s", local_ec.message().c_str());
                local_ec.clear();
            }
        }

        try {
            auto rt_con = robot.getRtMotionController().lock();
            if (rt_con) {
                try {
                    rt_con->stopServoJoint();
                } catch (const std::exception &e) {
                    RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "stopServoJointAbnormal: %s", e.what());
                } catch (...) {
                    RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "stopServoJointUnknown exception");
                }

                try {
                    rt_con->stopMove();
                } catch (const std::exception &e) {
                    RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "stopMoveAbnormal: %s", e.what());
                } catch (...) {
                    RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "stopMoveUnknown exception");
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "Exception occurred while obtaining RtMotionController: %s", e.what());
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "An unknown exception occurred while obtaining RtMotionController");
        }

        robot.stop(local_ec);
        if (local_ec) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), "robot.stopFailure: %s", local_ec.message().c_str());
            local_ec.clear();
        }

        auto state_after = robot.operationState(local_ec);
        if (local_ec) {
            response->success = false;
            response->message = "stop_rtFailed to read operationState after execution: " + local_ec.message();
            return true;
        }

        const bool inactive = (state_after != rokae::OperationState::rtControlling &&
                               state_after != rokae::OperationState::moving &&
                               state_after != rokae::OperationState::jogging);

        response->success = inactive;
        response->message = std::string("state_before=") + std::to_string(static_cast<int>(state_before)) +
                            ", state_after=" + std::to_string(static_cast<int>(state_after));
        return true;
    } catch (const std::exception &e) {
        response->success = false;
        response->message = std::string("stop_rtAbnormal: ") + e.what();
        return true;
    } catch (...) {
        response->success = false;
        response->message = "stop_rtUnknown exception";
        return true;
    }
}

// Enable/disable dragging requires prior setting of a super administrator
bool drag_control_callback(
    const std::shared_ptr<rokae_msgs::srv::DragCon::Request> request,
    std::shared_ptr<rokae_msgs::srv::DragCon::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "drag_control called");
        if (request->command == "on") {  
            // Enable drag mode
            
            robot.setOperateMode(rokae::OperateMode::manual, ec);
            robot.setPowerState(false, ec);
            robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
            RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "Drag enabled successfully");
            response->success = true;
            response->message = "Drag enabled successfully";

            
        } else if (request->command == "off") {
            // Turn off drag mode
            
            robot.disableDrag(ec);
            RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "Drag disabled successfully");
            response->success = true; 
            response->message = "Drag disabled successfully";
            
        } else {
            // The command is invalid
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), 
                       "Invalid command: %s. Expected 'on' or 'off'", 
                       request->command.c_str());
            response->success = false;
            response->message = "Invalid command. Use 'on' or 'off'";
        }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;  // Service call completed
}



// Forward and Inverse Kinematics Calculation Service  Radian
bool calculate_ik_callback(
    const std::shared_ptr<rokae_msgs::srv::CalculateIK::Request> request,
    std::shared_ptr<rokae_msgs::srv::CalculateIK::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "calculate_ik called");
        
        if (request->target_pose.size() != 6) {
            response->success = false;
            response->message = "Target pose must have 6 elements [x, y, z, rx, ry, rz]";
            return false;
        }
        
        // Do not use the CartesianPosition object
        std::array<double, 6> target_pose;
        std::copy(request->target_pose.begin(), request->target_pose.end(), target_pose.begin());
        
        auto model = robot.model();
        auto ik_result = model.calcIk(target_pose, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to calculate inverse kinematics: " + ec.message();
            return false;
        }
        
        response->success = true;
        response->message = "IK calculated successfully";
        response->joint_positions.assign(ik_result.begin(), ik_result.end());
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


bool calculate_fk_callback(
    const std::shared_ptr<rokae_msgs::srv::CalculateFK::Request> request,
    std::shared_ptr<rokae_msgs::srv::CalculateFK::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "calculate_fk called");
        
        if (request->joint_positions.size() != 6) {
            response->success = false;
            response->message = "Joint positions must have 6 elements";
            return true;
        }
        
        std::array<double, 6> joint_positions;
        std::copy(request->joint_positions.begin(), request->joint_positions.end(), joint_positions.begin());
        
        auto model = robot.model();
        CartesianPosition fk_result = model.calcFk(joint_positions, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to calculate forward kinematics: " + ec.message();
            return true;
        }
        
        response->success = true;
        response->message = "FK calculated successfully";
        
        // Use a loop to add elements   Usefk_result.posThe result is0
        response->cartesian_pose.clear();
        for (int i = 0; i < 3; i++) {
            response->cartesian_pose.push_back(fk_result.trans[i]);
        }
        for (int i = 0; i < 3; i++){
            response->cartesian_pose.push_back(fk_result.rpy[i]);
        }

    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return true;
    }
    
    return true;
}






// JogControl Service ReferenceJogCon.srv
bool jog_control_callback(
    const std::shared_ptr<rokae_msgs::srv::JogCon::Request> request,
    std::shared_ptr<rokae_msgs::srv::JogCon::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "jog_control called");
        
        // Parameter validation
        if (request->rate < 0.01 || request->rate > 1.0) {
            response->success = false;
            response->message = "Rate must be between 0.01 and 1.0";
            return true;
        }
        
        if (request->step <= 0) {
            response->success = false;
            response->message = "Step must be greater than 0";
            return true;
        }
        
        // Type of mapping coordinate system
        JogOpt::Space jog_space;
        
        if (request->space == "world") {
            jog_space = JogOpt::world;
        } else if (request->space == "joint") {
            jog_space = JogOpt::jointSpace;
        } else if (request->space == "tool") {
            jog_space = JogOpt::toolFrame;
        } else if (request->space == "flange") {
            jog_space = JogOpt::flange;
        } else if (request->space == "singularityAvoidMode") {
            jog_space = JogOpt::singularityAvoidMode;
        } else if (request->space == "baseParallelMode") {
            jog_space = JogOpt::baseParallelMode;
        } else if (request->space == "singularityAvoidMode"){
            jog_space = JogOpt::singularityAvoidMode;
        }else if (request->space == "baseFrame"){
            jog_space = JogOpt::baseFrame;
        }else {
            response->success = false;
            response->message = "Invalid space. Use: world, joint, tool, flange, singularityAvoidMode, baseParallelMode";
            return true;
        }
        
        // Switch to manual mode and power on
        robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot.setOperateMode(rokae::OperateMode::manual, ec);
        robot.setPowerState(true, ec);
        
        // RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
        //            "Starting jog: space=%s, rate=%.2f, step=%.2f, index=%d, direction=%s",
        //            request->space.c_str(), request->rate, request->step, 
        //            request->index, request->direction ? "positive" : "negative");
        
        // Start Jog (Step Mode)）
        robot.startJog(jog_space, request->rate, request->step, request->index, request->direction, ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to start jog: " + ec.message();
            return true;
        }
        

        float wait_time =10.0; //First randomly wait for 10 seconds
        // RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
        //            "Waiting %.2f seconds for jog to complete", wait_time);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time * 1000)));
        
        // Stop Jog (must be called）
        robot.stop(ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to stop jog: " + ec.message();
            return true;
        }
        
        response->success = true;
        response->message = "Jog completed successfully";
        
    } catch (const std::exception& e) {
        // Ensure to stop even in exceptional situationsJog
        robot.stop(ec);
        
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;  // Service call completed
}


// /*IORelated*/

//  Get digital output status  Communication in HMI -- System IO settings
bool get_do_callback(
    const std::shared_ptr<rokae_msgs::srv::GetDO::Request> request,
    std::shared_ptr<rokae_msgs::srv::GetDO::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "get_do called - board: %u, port: %u", 
                   request->board, request->port);
        
        bool state = robot.getDO(request->board, request->port, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to get DO state: " + ec.message();
            return true;
        }
        
        response->state = state;
        response->success = true;
        response->message = state ? "DO is ON" : "DO is OFF";
    
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

//DOThe signal does not exist or is a system output
bool set_do_callback(
    const std::shared_ptr<rokae_msgs::srv::SetDO::Request> request,
    std::shared_ptr<rokae_msgs::srv::SetDO::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "set_do called - board: %u, port: %u, state: %s", 
                   request->board, request->port, 
                   request->state ? "true" : "false");
        
        robot.setDO(request->board, request->port, request->state, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to set DO: " + ec.message();
            return true;
        }
        
        response->success = true;
        response->message = "DO set to " + std::string(request->state ? "ON" : "OFF");
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


// Set digital input signal
bool set_di_callback(
    const std::shared_ptr<rokae_msgs::srv::SetDI::Request> request,
    std::shared_ptr<rokae_msgs::srv::SetDI::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "set_di called - board: %u, port: %u, state: %s", 
                   request->board, request->port, 
                   request->state ? "true" : "false");
        
        robot.setOperateMode(rokae::OperateMode::manual, ec);
        robot.setSimulationMode(true, ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to enable simulation mode: " + ec.message();
            return true;
        }
        
        robot.setDI(request->board, request->port, request->state, ec); 
        

        response->success = true;
        response->message = "DI set to " + std::string(request->state ? "ON" : "OFF");
        
    } catch (const std::exception& e) {
        // Ensure that simulation mode is attempted to be turned off even in exceptional cases
        robot.setSimulationMode(false, ec);
        
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


// Get digital input status
bool get_di_callback(
    const std::shared_ptr<rokae_msgs::srv::GetDI::Request> request,
    std::shared_ptr<rokae_msgs::srv::GetDI::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "get_di called - board: %u, port: %u", 
                   request->board, request->port);

        robot.setOperateMode(rokae::OperateMode::manual, ec);
        robot.setSimulationMode(true, ec);
        
        bool state = robot.getDI(request->board, request->port, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to get DI state: " + ec.message();
            return true;
        }
        
        response->state = state;
        response->success = true;
        response->message = state ? "DI is ON" : "DI is OFF";

        
    } catch (const std::exception& e) {
        // Ensure that simulation mode is attempted to be turned off even in exceptional cases
        robot.setSimulationMode(false, ec);
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


bool read_register_callback(
    const std::shared_ptr<rokae_msgs::srv::ReadRegister::Request> request,
    std::shared_ptr<rokae_msgs::srv::ReadRegister::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "read_register called");
        
        // Clear previous error codes
        ec.clear();
        
        // Handle according to data type branches
        if (request->data_type == "float") {
            if (request->read_all) {
                std::vector<float> values;
                robot.readRegister(request->register_name, 0, values, ec);
                if (!ec) {
                    response->float_values = values;
                }
            } else {
                float value;
                robot.readRegister(request->register_name, request->index, value, ec);
                if (!ec) {
                    response->float_values.push_back(value);
                }
            }
        } 
        else if (request->data_type == "bool" || request->data_type == "bit") {
            if (request->read_all) {
                std::vector<bool> values;
                robot.readRegister(request->register_name, 0, values, ec);
                if (!ec) {
                    for (bool v : values) {
                        response->bool_values.push_back(v);
                    }
                }
            } else {
                bool value;
                robot.readRegister(request->register_name, request->index, value, ec);
                if (!ec) {
                    response->bool_values.push_back(value);
                }
            }
        }
        else if (request->data_type == "int16") {
            if (request->read_all) {
                std::vector<int> values;
                robot.readRegister(request->register_name, 0, values, ec);
                if (!ec) {
                    for (int16_t v : values) {
                        response->int16_values.push_back(v);
                    }
                }
            } else {
                int value;
                robot.readRegister(request->register_name, request->index, value, ec);
                if (!ec) {
                    response->int16_values.push_back(value);
                }
            }
        }
        else {
            response->success = false;
            response->message = "Invalid data type.";
            return false;
        }
        
        response->success = true;
        response->message = "Register read successfully";

    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception: ") + e.what();
        return false;
    }
    
    return true;
}


bool write_register_callback(
    const std::shared_ptr<rokae_msgs::srv::WriteRegister::Request> request,
    std::shared_ptr<rokae_msgs::srv::WriteRegister::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "write_register called");
        
        // Clear previous error codes
        ec.clear();
        
        // Handle according to data type branches
        if (request->data_type == "float") {
            if (request->write_all) {
                robot.writeRegister(request->register_name, 0, request->float_values, ec);
            } else {
                robot.writeRegister(request->register_name, request->index, request->float_value, ec);
            }
        } 
        else if (request->data_type == "bool" || request->data_type == "bit") {
            if (request->write_all) {
                robot.writeRegister(request->register_name, 0, request->bool_values, ec);
            } else {
                robot.writeRegister(request->register_name, request->index, request->bool_value, ec);
            }
        }
        else if (request->data_type == "int16") {
            if (request->write_all) {
                // Write one by one to avoid fmt errors in the robot library
                for (size_t i = 0; i < request->int16_values.size(); i++) {
                    int int_value = static_cast<int>(request->int16_values[i]);
                    robot.writeRegister(request->register_name, i, int_value, ec);
                }
            } else {
                int int_value = static_cast<int>(request->int16_value);
                robot.writeRegister(request->register_name, request->index, int_value, ec);
            }
        }
        else {
            response->success = false;
            response->message = "Invalid data type.";
            return false;
        }
        
        response->success = true;
        response->message = "Register written successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception: ") + e.what();
        return false;
    }
    
    return true;
}



bool movej_callback(
    const std::shared_ptr<rokae_msgs::srv::MoveJ::Request> request,
    std::shared_ptr<rokae_msgs::srv::MoveJ::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"movej called");
        // 1. Obtain target joint angle
        std::array<double, 6> target_joints;
        for (int i = 0; i < 6; i++) {
            target_joints[i] = request->joint_positions[i];
        }
        
        // 2. Get speed parameters
        double velocity = static_cast<double>(request->velocity);
        
        // 3. Get current joint angle
        std::array<double, 6> current_joints = robot.jointPos(ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to get current joint positions";
            return false;
        }
        
        // 4. Obtain real-time controller
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        auto rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            response->success = false;
            response->message = "Failed to get motion controller";
            return false;
        }
        
        // 5. ExecuteMoveJ
        rtCon->MoveJ(velocity, current_joints, target_joints);
        // rtCon->startMove(RtControllerMode::jointPosition);
        
        // 6. Setting successful response
        response->success = true;
        response->message = "movej has been executed";
        
        //7.Closerci
        // rtCon->stopMove();
        
    } catch (const std::exception& e) {
        // 8. Set error response
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

// Accept the relative offset of the end; the Cartesian solution may fail to solve inversely
bool movel_callback(
    const std::shared_ptr<rokae_msgs::srv::MoveL::Request> request,
    std::shared_ptr<rokae_msgs::srv::MoveL::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"movel called");
        // 1. Get speed parameters
        double velocity = static_cast<double>(request->velocity);
        
        // 2. Get the current end pose
        CartesianPosition start;
        Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
        if (ec) {
            response->success = false;
            response->message = "Failed to get current flange position";
            return false;
        }
        
        // 3. Decompose the initial pose into a rotation matrix and a translation vector
        Eigen::Matrix3d rot_start;
        Eigen::Vector3d trans_start;
        Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
        
        // 4. Create target translation vector (apply relative offset)）
        Eigen::Vector3d trans_end = trans_start;
        
        // Use offset instead of target_position
        trans_end[0] += request->offset[0];  // XShaft offset
        trans_end[1] += request->offset[1];  // YShaft offset
        trans_end[2] += request->offset[2];  // ZAxis Offset

// 5. Create the target pose (maintain the rotation posture of the starting point)）
        CartesianPosition target;
        Utils::transMatrixToArray(rot_start, trans_end, target.pos);
        
        // 6. Obtain real-time controller
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        auto rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            response->success = false;
            response->message = "Failed to get motion controller";
            return false;
        }
        
        // 7. ExecuteMoveL
        rtCon->MoveL(velocity, start, target);
        // rtCon->startMove(RtControllerMode::cartesianPosition);
        
        // 8. Setting successful response
        response->success = true;
        response->message = "movel has been executed";

        // 9. Closerci
        // rtCon->stopMove();
        
    } catch (const std::exception& e) {
        // 10. Set error response
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

//x-yPlane Z Offset Setting0
bool movec_callback(
    const std::shared_ptr<rokae_msgs::srv::MoveC::Request> request,
    std::shared_ptr<rokae_msgs::srv::MoveC::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"movec called");
        // 1. Get speed parameters
        double velocity = static_cast<double>(request->velocity);
        
        // 2. Obtain the current end pose as the starting point
        CartesianPosition start;
        Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
        if (ec) {
            response->success = false;
            response->message = "Failed to get current flange position";
            return false;
        }
        
        // 3. Decompose the initial pose into a rotation matrix and a translation vector
        Eigen::Matrix3d rot_start;
        Eigen::Vector3d trans_start, trans_aux, trans_end;
        Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
        
        // 4. Calculate the translation vector for the auxiliary point and the endpoint (apply relative offset)）
        trans_aux = trans_start;
        trans_end = trans_start;
        
        // Check the size of the offset array
        if (request->aux_offset.size() < 3 || request->target_offset.size() < 3) {
            response->success = false;
            response->message = "Offset arrays must have at least 3 elements";
            return false;
        }
        
        // Apply auxiliary point offset
        trans_aux[0] += request->aux_offset[0];  // XShaft offset
        trans_aux[1] += request->aux_offset[1];  // YShaft offset
        trans_aux[2] += request->aux_offset[2];  // ZAxis offset
        
        // Apply endpoint offset
        trans_end[0] += request->target_offset[0];  // XShaft offset
        trans_end[1] += request->target_offset[1];  // YShaft offset
        trans_end[2] += request->target_offset[2];  // ZAxis Offset

// 5. Create auxiliary points and end pose (maintain the rotation posture of the starting point)）
        CartesianPosition aux, target;
        Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
        Utils::transMatrixToArray(rot_start, trans_end, target.pos);
        
        // 6. Obtain real-time controller
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        auto rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            response->success = false;
            response->message = "Failed to get motion controller";
            return false;
        }
        
        // 7. ExecuteMoveC
        rtCon->MoveC(velocity, start, aux, target);
        // rtCon->startMove(RtControllerMode::cartesianPosition);
        
        // 8. Setting successful response
        response->success = true;
        response->message = "movec has been executed";

        // 9.Closerci
        // rtCon->stopMove();
        
    } catch (const std::exception& e) {
        // 10. Set error response
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}




void joint_position_callback(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_state_pub) {
    try {
        // 1. Get joint positions from Rokae robot (May contain external shafts)
        auto joint_positions = robot.jointPos(ec);
        auto joint_velocity = robot.jointVel(ec);
        auto joint_jointTorque = robot.jointTorque(ec);
        
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), 
                       "Failed to get joint position: %s", ec.message().c_str());
            return;
        }
        
        // 2. Create and populate ROS joint state message
        JointState joint_state_msg;
        joint_state_msg.header.stamp = rclcpp::Clock().now(); // Set timestamp
        // joint_state_msg.header.frame_id = "base_link"; // Reference coordinate system

// 3. Set joint names (Adjust according to your robot model)
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            joint_state_msg.name.push_back("joint" + std::to_string(i + 1));
            joint_state_msg.position.push_back(joint_positions[i]);
            joint_state_msg.velocity.push_back(joint_velocity[i]);
            joint_state_msg.effort.push_back(joint_jointTorque[i]);
        }
 
        // 4. Post a message
        if (joint_state_pub) {
            joint_state_pub->publish(joint_state_msg);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rokae_driver"), 
                    "Error occurred while publishing joint state: %s", e.what());
    }
}


void cartesian_pose_callback(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& cartesian_pose_pub) {
    try {
        // 1. Obtain the pose of the flange relative to the base coordinate system from the Rokae robot
        std::array<double, 6> pose_array = robot.posture(rokae::CoordinateType::flangeInBase, ec);
        
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), 
                       "Failed to obtain Cartesian pose: %s", ec.message().c_str());
            return;
        }
        
        // 2. Create and populate a ROS Cartesian pose message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "base_link"; // Set the reference coordinate system as the base coordinate system

// 3. Set the position (x, y, z) - Unit: meter
        pose_msg.pose.position.x = pose_array[0];
        pose_msg.pose.position.y = pose_array[1];
        pose_msg.pose.position.z = pose_array[2];
        
        // 4. Set posture (rx, ry, rz Convert to quaternion)
        double rx = pose_array[3]; // roll
        double ry = pose_array[4]; // pitch
        double rz = pose_array[5]; // yaw
        
        // Using ROS tf2 for transformations
        tf2::Quaternion q;
        q.setRPY(rx, ry, rz);
        
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        // 5. Post a message
        if (cartesian_pose_pub) {
            cartesian_pose_pub->publish(pose_msg);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rokae_driver"), 
                    "Error occurred while publishing Cartesian pose: %s", e.what());
    }
}


void state_monitor_worker(rclcpp::Node::SharedPtr node) {
    while (rclcpp::ok()) {
        // std::error_code ec;
        
        // 1. Connection Check and Data Retrieval
        // auto joint_positions = robot.jointPos(ec);
        robot.jointPos(ec);
        
        if (ec) {
            // Connection or read failed
            RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                 "Connection error or failed to retrieve data: %s", ec.message().c_str());
        } else {
            // 2. Connection is normal, executing publish callback (currently only joint states)）
            joint_position_callback(joint_state_pub);
            cartesian_pose_callback(cartesian_pose_pub);
        }
        
        // 3. Control frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //100hzPost once
    }
}

int main(int argc , char** argv){
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("rokae_driver");
    rclcpp::Rate rate(125); 

    node->declare_parameter("robot_ip", "192.168.2.160");
    node->declare_parameter("local_ip", "192.168.2.100");
    std::string robot_ip = node->get_parameter("robot_ip").as_string();
    std::string local_ip = node->get_parameter("local_ip").as_string();

    // Connect to the robot
    try {
        robot.connectToRobot(robot_ip ,local_ip);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rokae_driver"), "%s", e.what());
        return 0;
    }
    //Set robot status
    // robot.setOperateMode(rokae::OperateMode::automatic, ec);
    // robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    // robot.setPowerState(true, ec);
    // auto rtCon = robot.getRtMotionController().lock();

    RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"rokae_driver is ready");
    
    auto get_robot_info_service = node->create_service<rokae_msgs::srv::GetRobotInfo>(
        "/rokae_driver/get_robot_info", &get_robot_info_callback);
    auto control_drag_service = node->create_service<rokae_msgs::srv::DragCon>(
        "/rokae_driver/drag_control", &drag_control_callback);
    auto jog_drag_service = node->create_service<rokae_msgs::srv::JogCon>(
        "/rokae_driver/jog_control", &jog_control_callback);
    auto stop_rt_service = node->create_service<std_srvs::srv::Trigger>(
        "/rokae_driver/stop_rt", &stop_rt_callback);

    auto get_do_service = node->create_service<rokae_msgs::srv::GetDO>(
        "/rokae_driver/get_do", &get_do_callback);
    auto set_do_service = node->create_service<rokae_msgs::srv::SetDO>(
        "/rokae_driver/set_do", &set_do_callback);
    auto set_di_service = node->create_service<rokae_msgs::srv::SetDI>(
        "/rokae_driver/set_di", &set_di_callback);    
    auto get_di_service = node->create_service<rokae_msgs::srv::GetDI>(
        "/rokae_driver/get_di", &get_di_callback);
    auto read_register_service = node->create_service<rokae_msgs::srv::ReadRegister>(
        "/rokae_driver/read_register", &read_register_callback);
    auto write_register_service = node->create_service<rokae_msgs::srv::WriteRegister>(
        "/rokae_driver/write_register", &write_register_callback);

    auto calculate_ik_service = node->create_service<rokae_msgs::srv::CalculateIK>(
        "/rokae_driver/calculate_ik", &calculate_ik_callback);
    
    auto calculate_fk_service = node->create_service<rokae_msgs::srv::CalculateFK>(
        "/rokae_driver/calculate_fk", &calculate_fk_callback);

    auto movej_service = node->create_service<rokae_msgs::srv::MoveJ>( //Service message name starts with a capital letter
    "/rokae_driver/movej",  // Service Name
    &movej_callback);          // Callback function

    auto movel_service = node->create_service<rokae_msgs::srv::MoveL>("/rokae_driver/movel",&movel_callback);
        
    auto movec_service = node->create_service<rokae_msgs::srv::MoveC>("/rokae_driver/movec",&movec_callback);

    
    joint_state_pub = node->create_publisher<JointState>("/rokae_driver/joint_states", 10 );
    cartesian_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/rokae_driver/cartesian_pose", 10);

    std::thread monitor_thread(state_monitor_worker, node);
    // Keep the node running
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Shutting down the node...");
    if (monitor_thread.joinable()) {
        monitor_thread.join();
        // RCLCPP_INFO(node->get_logger(), "The status monitoring thread has been merged。");
    }
    
    // Cleanup at the end of the node
    robot.setSimulationMode(false, ec); //diThe signal needs to enable analog mode
    robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
    robot.setPowerState(false, ec);

    rclcpp::shutdown();
    return 0;
}