/**
 * @file motion_control_rt.h
 * @brief Real-time mode motion control
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_ROKAE_MOTION_CONTROL_RT_H_
#define ROKAEAPI_ROKAE_MOTION_CONTROL_RT_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <memory>
#include "base.h"
#include "data_types.h"

namespace rokae {

 /// @cond DO_NOT_DOCUMENT
 // forward declaration
 class XService;
 class DataReceiver;
 template <MotionControlMode ControlMode>
 class MotionControl;
 template <unsigned short DoF>
 class xMateModel;
 /// @endcond

/**
 * @class BaseMotionControl
 * @brief General Motion Control Class
 */
 class XCORE_API BaseMotionControl : public Base<BaseMotionControl> {
  public:
   /// @cond DO_NOT_DOCUMENT
   explicit BaseMotionControl(std::shared_ptr<XService> comm) noexcept;
   virtual ~BaseMotionControl() noexcept;
   BaseMotionControl(const BaseMotionControl&) = delete;
   BaseMotionControl &operator=(const BaseMotionControl &) = delete;

  XCORESDK_DECLARE_IMPL
   /// @endcond
 };

 /**
  * @class MotionControl<MotionControlMode::RtCommand>
  * @brief Real-time Mode Motion Control General Class
  */
 template <>
 class XCORE_API MotionControl<MotionControlMode::RtCommand> : public BaseMotionControl {
  public:
   /**
    * @brief constructor
    * @throw NetworkException Network connection error
    */
   MotionControl(std::shared_ptr<XService> sdkRpc, XService *_rtRpc, DataReceiver *recv, const std::string &localIp);
   virtual ~MotionControl() noexcept;

   // *********************************************************************
   // *******************           Network connection           **********************

   /**
    * @brief Disconnect from the real-time control server, close the data receiving and command sending ports. This will not disconnect from the robot.。
    * If the robot is in motion, it will stop immediately once disconnected.
    */
   void disconnectNetwork() noexcept;

   /**
    * @brief Reconnect to the real-time control server
    * @param[out] ec Error code
    */
   void reconnectNetwork(error_code &ec) noexcept;

   // *********************************************************************
   // *******************           Control method           **********************

   /**
    * @brief Use periodic scheduling and set a callback function。
    * @note 1) The callback function should plan motion commands with a cycle of 1 millisecond, and the planning result is the return value of the function. The SDK filters the return value before sending it to the controller.。
    *       2) JointPositionThe length of the joint angle array and the length of the Torque joint torque array should be the same as the number of robot axes. If they are different, an error will not be reported, but it may result in unreasonable commands.
    *       3) At the end of an exercise cycle, it can be returned byCommand.setFinish()to identify, the SDK will handle stopping the motion and stopping the invocation of callback functions
    * @tparam Command JointPosition | CartesianPosition | Torque
    * @param[in] callback Callback function. According to the control mode(RtControllerMode)Different, function return values have 3 types: joint angles/Cartesian pose/torque。
    * Among them, the Cartesian pose uses a rotation matrix to represent the rotation, and pos is the pose of the end relative to the base coordinate system.。
    * @param[in] priority Task priority, 0 means unspecified. This parameter only takes effect when using a real-time operating system. If it cannot be set, an error message will be printed to the console.。
    * @param[in] useStateDataInLoop Whether it is necessary to read status data during the cycle. When set to true：
    *       1) xCore-SDKWill update real-time state data before the callback function(updateStateData()),Directly inside the callback functiongetStateData()That's it;
    *       2) The sending period of status data should be consistent with the control cycle, which is1ms: startReceiveRobotState(interval = milliseconds(1));
    */
   template<class Command>
   void setControlLoop(const std::function<Command(void)>& callback, int priority = 0, bool useStateDataInLoop = false) noexcept;

   /**
    * @brief Start executing the callback function。
    * @param[in] blocking Whether to block the thread that calls this function. If it is a non-blocking thread, it needs to callstopLoop()Stop scheduling tasks, otherwise the next cycle cannot start。
    * @throw RealtimeControlException Network error when sending command; Or the command type does not match the control mode; An error occurred while the device or controller was executing the sent command
    */
   void startLoop(bool blocking = true);

   /**
    * @brief Stop executing periodic scheduled tasks
    * @throw RealtimeControlException An exception occurred during execution
    */
   void stopLoop();

   /**
    * @brief The robot stops moving and stops receiving motion commands sent by the client.。
    * @note In addition, the JointPosition/CartesianPosition/Torque commands can be sent throughsetFinished()Indicates the end of a movement cycle; after indicating, the robot will stop moving.，
    * The presented effect and invocationstopMove()The same。
    * This function is only for real-time control and cannot be used to stop non-real-time motion commands.。
    * @throw RealtimeMotionException Failed to stop the exercise
    */
   void stopMove();

   /**
    * @brief Send JointPosition/CartesianPosition/Torque commands. Suitable for programs that send commands directly without using a scheduling cycle.。
    * @note After starting the motion, continuously call this function to send motion commands. Since the controller executes commands in cycles of 1 ms, the interval between sending commands also needs to be controlled within1ms，
    * If the sending interval is too long, it will be judged as a communication packet loss; if the interval is too short, it will cause a servo error.。
    * If you send the command directly, there is no need to use the scheduling cycle.，setControlLoop(), startLoop(), stopLoop()None of the related functions are needed。
    * Real-time motion errors can be handled throughBaseRobot::updateRobotState()Be aware that an error will throw an exception；
    * Need to callRobot_T::startReceiveRobotState()Receive data, it is recommended to interval by 1ms to avoid error messages being overwritten
    * @param[in] cmd According to the control mode(RtControllerMode)Different, there are 3 types of motion commands: joint angles/cartesian poses/torque
    * @throw ArgumentException The instruction contains an illegal value
    * @throw RealtimeStateException Not started exercising
    * @throw RealtimeControlException Network error when sending command; Or the command type does not match the control mode; An error occurred while the device or controller was executing the sent command
    */
   template<class Command>
   void sendCommand(const Command &cmd);

   // *********************************************************************
   // *****************        Obtain real-time robot status data        ******************

   /**
    * @brief Start the robot sending real-time status data. Block and wait to receive the first frame message, with a timeout of 3 seconds.
    * @param[in] fields Received robot status data, with a maximum total length of 1024 bytes
    * @throw RealtimeControlException Set unsupported status data; or the robot cannot start sending data; or the total length exceeds1024
    * @throw RealtimeStateException Data transmission has already started; or the first frame of data has not been received after a timeout
    */
   [[deprecated("Use Robot_T::startReceiveRobotState(interval, fields) instead")]]
   void startReceiveRobotState(const std::vector<std::string>& fields);

   /**
    * @brief Stop receiving real-time status data, while the controller stops sending. Can be used to reset the status data to be received.。
    * After calling this function, error messages from real-time motion will also stop being received. It is recommended to call it when the motion stops.。
    */
   [[deprecated("Use BaseRobot::stopReceiveRobotState() instead")]]
   void stopReceiveRobotState() noexcept;

   /**
    * @brief Update robot status data to the latest
    * @note throughsetControlLoop()The set callback updates the status data each time it is executed, so there is no need to call this interface within the callback function.
    * @throw RealtimeStateException An error occurred while the controller was processing the motion command, and the error bit was set.
    * @throw RealtimeControlException Unable to receive data; or the received data has errors that make it impossible to parse
    */
   [[deprecated("Use BaseRobot::updateRobotState() instead")]]
   void updateRobotState();

   /**
    * @brief Read robot status data
    * @note Make sure the type of the passed-in data matches the data type。
    * @tparam R Data Type
    * @param[in] fieldName Data Name
    * @param[out] data Numerical value
    * @return If there is no such data name; or it has not passedstartReceiveRobotState()Set as the data to be received; or the data type does not match R, return-1。
    * Successfully read return0。
    * @throw RealtimeStateException Network error
    */
   template<typename R>
   [[deprecated("Use BaseRobot::getStateData(fieldName, data) instead")]]
   int getStateData(const std::string &fieldName, R &data);

   // *******************          ServoJRelated          *********************

   /**
    * @brief Issue joint positions through the real-time mode sendCommand, allowing the setting of cycle, gain, and look-ahead time, and enable the servoJ function.
    * @param[in] ServoJ_T When sending the joint position, call the cycle of servoJ, units
    * @param[in] ServoJ_Lookahead Look-ahead time, limit on motion speed after issuing joint positions, units
    * @param[in] ServoJ_Kp Control Gain
    * @param[out] ec Error code
    */
   void setServoJoint(double ServoJ_T, double ServoJ_Lookahead, double ServoJ_Kp, error_code &ec) noexcept;

   /**
    * @brief Turn off the servoJ function; stopping the use of setServoJoint requires turning it off.
    */
   void stopServoJoint() noexcept;

   // *********************************************************************
   // ********************          Other operations            *********************

   /**
    * @brief Did a motion error occur in real-time mode motion?
    * @return true - There is an error
    */
   bool hasMotionError() noexcept;

   /**
    * @brief Automatic recovery robot when an error occurs。
    * @param[out] ec Error code
    */
   void automaticErrorRecovery(error_code &ec) noexcept;

   /// @cond DO_NOT_DOCUMENT
  XCORESDK_DECLARE_IMPLD
  /// @endcond
 };

 /**
  * @class RtMotionControl
  * @brief Real-time mode motion control
  * @tparam DoF Number of axles
  */
 template <WorkType Wt, unsigned short DoF>
 class XCORE_API RtMotionControl : public MotionControl<MotionControlMode::RtCommand> {
  public:
   /**
    * @brief constructor
    * @throw NetworkException Network connection error
    */
   RtMotionControl(std::shared_ptr<XService> sdkRpc, XService *rtRpc, DataReceiver *recv, const std::string &localIp);

   // **********************************************************************
   // ***************           Control mode & Send motion command          ***************

   /**
    * @brief Specify the control mode, the robot is ready to start moving, and this interface needs to be called before each callback execution。
    * Calling this interface will not make the robot move immediately; it will only start after a movement command is sent.。
    * @param[in] rtMode Control mode
    * @note 1) Before startMove, the parameters should be set in order, such as filter impedance parameters, etc. After the settings are completed, then call it.startMove()。
    * Executing other commands after calling startMove may fail, such as powering down operations. The correct way to stop is to callstopMove。
    * @throw RealtimeStateException Has already started exercise; repeat call after exercise
    * @throw RealtimeParameterException An unsupported control mode was specified
    * @throw RealtimeControlException The controller cannot switch to this control mode, which occurs more often when switching to force control mode.
    */
   void startMove(RtControllerMode rtMode);

   // *********************************************************************
   // *******************           Parameter Settings           **********************

   /**
    * @brief Set amplitude limiting filter parameters.
    * @param[in] limit_rate true - Amplitude Limiting On
    * @param[in] cutoff_frequency Cutoff frequency. The range is0 ~ 1000Hz，Suggestion10~100Hz.
    * @return true - Setting successful
    */
   bool setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept;

   /**
    * @brief Set the Cartesian space motion area; movement beyond the set area will stop.。
    * Non-force-controlled virtual wall. If the robot's end or the TCP end exceeds the safety area, the motor will also perform a power-down process.。
    * @param[in] lengths The length, width, and height of the safe zone cuboid, corresponding to XYZ, unit: meters
    * @param[in] frame Pose of the center of the safety zone cuboid relative to the base coordinate system
    * @param[out] ec Error code
    */
   void setCartesianLimit(const std::array<double, 3> &lengths, const std::array<double, 16> &frame, error_code &ec) noexcept;

   /**
    * @brief Set the pose of the end effector relative to the robot flange. After setting the TCP, the controller will save the configuration and restore the default settings after the robot restarts.。
    * @param[in] frame Homogeneous matrix of the end-effector coordinate system relative to the flange coordinate system, unit: rad, m
    * @param[out] ec Error code
    */
   void setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept;

   /**
    * @brief Set the mass, center of mass, and inertia matrix of the tool and payload. After setting the payload, the controller will save the payload configuration and restore the default settings when the robot restarts.。
    * @param[in] load Load Information
    * @param[out] ec Error code
    */
   void setLoad(const Load &load, error_code &ec) noexcept;

   // *************************************************************************
   // *****************            MoveInstruction (Upper computer planning)         *****************

   /**
    * @brief MoveJCommand: The upper computer plans the path and remains in a blocked state until reaching the target. If an error occurs during movement, the blocked state will stop and return.。
    * @note No longer recommended; please use non-real-time mode commandsMoveAbsJCommand。
    * @param[in] speed Speed ratio coefficient
    * @param[in] start The starting joint angles need to be the robot's current joint angles, otherwise it may cause a power-off.。
    * @param[in] target Robot target joint angle
    * @throw RealtimeMotionException An error occurred during the robot's movement
    */
   void MoveJ(double speed, const std::array<double, DoF>& start, const std::array<double, DoF>& target);

   /**
    * @brief MoveLCommand: The upper computer plans the path and remains in a blocked state until reaching the target. If an error occurs during movement, the blocked state will stop and return.。
    * @note No longer recommended; please use non-real-time mode commandsMoveLCommand。
    * @param[in] speed Speed ratio coefficient, range 0 - 1
    * @param[in] start The starting pose needs to be the robot's current pose, otherwise it may cause a power outage. If a TCP is set, it should be the pose of the tool relative to the base coordinate system.。
    * @param[in] target Robot target pose. Similarly, if a TCP is set, it should be the pose of the TCP relative to the base coordinate system.
    * @throw RealtimeParameterException Error in the initial or target pose parameters
    * @throw RealtimeMotionException An error occurred during the robot's movement
    */
   void MoveL(double speed, CartesianPosition& start, CartesianPosition& target);

   /**
    * @brief MoveCThe command is in a blocked state until reaching the target. If an error occurs during movement, the blocked state will stop and return.。
    * @note No longer recommended; please use non-real-time mode commandsMoveCCommand。
    * @param[in] speed Speed ratio coefficient
    * @param[in] start The robot's initial pose needs to be the robot's current pose. If the TCP is set, it should be the pose of the tool relative to the base coordinate system.。
    * @param[in] aux Robot-assisted point pose. Similarly, if the TCP is set, it should be the pose of the TCP relative to the base coordinate system.
    * @param[in] target Robot target pose. Similarly, if a TCP is set, it should be the pose of the TCP relative to the base coordinate system.
    * @throw RealtimeParameterException Coordinate error, unable to calculate the arc path
    * @throw RealtimeMotionException An error occurred during the robot's movement
    */
   void MoveC(double speed, CartesianPosition& start, CartesianPosition& aux, CartesianPosition& target);

 };

 /**
  * @class RtMotionControlCobot
  * @brief Collaborative Robot Real-Time Motion Control Class
  * @tparam DoF Number of axles
  */
 template <unsigned short DoF>
 class XCORE_API RtMotionControlCobot: public RtMotionControl<WorkType::collaborative, DoF> {
  public:
   using RtMotionControl<WorkType::collaborative, DoF>::RtMotionControl;

   // *********************************************************************
   // *******************           Parameter Settings           **********************

   /**
    * @brief Set the Cartesian space impedance control coefficient, effective during Cartesian space impedance motion
    * @param[in] factor Axial space impedance coefficient, unit: Nm/rad
    * xMateErProThe maximum stiffness of the model is { 3000, 3000, 3000, 3000, 300, 300, 300 }
    * The maximum stiffness of the six-axis model is { 3000, 3000, 3000, 300, 300, 300 }
    * The maximum rigidity of the five-axis model is { 3000, 3000, 3000, 300, 300 }
    * The actual effective maximum value is related to the hardware status such as the sensor. If phenomena like jitter occur, please try reducing the impedance coefficient.。
    * @param[out] ec Error code
    */
   void setJointImpedance(const std::array<double, DoF> &factor, error_code &ec) noexcept;

   /**
    * @brief Set the Cartesian space impedance control coefficient, effective when performing Cartesian impedance motion
    * @param[in] factor Impedance coefficient[ X, Y, Z, Rx, Ry, Rz], Maximum value is { 3000, 3000, 3000, 300, 300, 300 }, unit: N/m, Nm/rad
    * The actual effective maximum value is related to the hardware status such as the sensor. If phenomena like jitter occur, please try reducing the impedance coefficient.。
    * @param[out] ec Error code
    */
   void setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept;

   /**
    * @brief Set the filter cutoff frequency of the robot controller, used to smooth commands. Allowed range: 1 ~ 1000Hz, Recommended to set as10 ~ 100Hz。
    * @param[in] jointFrequency Filter cutoff frequency of joint position, unit: Hz
    * @param[in] cartesianFrequency Filter cutoff frequency of Cartesian spatial position, unit: Hz
    * @param[in] torqueFrequency Filter cutoff frequency of joint torque, unit: Hz
    * @param[out] ec Error code
    */
   void setFilterFrequency(double jointFrequency, double cartesianFrequency, double torqueFrequency, error_code &ec) noexcept;

   /**
    * @brief Set the terminal desired force, effective during Cartesian space impedance motion
    * @param[in] torque Expected force at the end of the Cartesian space, allowable range is { ±60, ±60, ±60, ±30, ±30, ±30 }, unit: N, N·m
    * @param[out] ec Error code
    */
   void setCartesianImpedanceDesiredTorque(const std::array<double, 6> &torque, error_code &ec) noexcept;

   /**
    * @brief Set filter parameters
    * @param[in] frequency Permissible range 1 ~ 1000Hz
    * @param[out] ec Error code
    */
   void setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept;

   /**
    * @brief Set robot force control coordinate system
    * @param[in] frame Transformation matrix of the force control coordinate system relative to the flange coordinate system
    * @param[in] type Category, specify which coordinate system is used as the force control task coordinate system, supported:
    *     1) World Coordinate System FrameType::world;
    *     2) Tool Coordinate System FrameType::tool;
    *     3) Path Coordinate System FrameType::path (The force control task coordinate system needs to track the process of trajectory changes)
    * @param[out] ec Error code
    */
   void setFcCoor(const std::array<double, 16> &frame, FrameType type, error_code &ec) noexcept;

   /**
    * @brief Set collision detection threshold。
    * Collision detection is only effective during position control and does not work during force control. If a collision is detected, the controller will issue a power-down command, and the motor brake will engage to disable the drive.。
    * @param[in] torqueThresholds Joint collision detection threshold, unitN。
    * xMateErProThe maximum value of the model is { 75, 75, 60, 45, 30, 30, 20 }，
    * The maximum value for other models is{ 75, 75, 45, 30, 30, 20 }
    * 5The maximum value of the shaft model is{ 75, 75, 45, 30, 20 }
    * @param[out] ec Error code
    */
   void setCollisionBehaviour(const std::array<double, DoF> &torqueThresholds, error_code &ec) noexcept;

 };

 /**
  * @class RtMotionControlIndustrial
  * @brief Industrial Model Real-Time Mode Motion Control Class
  * @tparam DoF Number of axles
  */
 template <unsigned short DoF>
 class XCORE_API RtMotionControlIndustrial: public RtMotionControl<WorkType::industrial, DoF> {
  public:
   using RtMotionControl<WorkType::industrial, DoF>::RtMotionControl;
 };

}
#endif //ROKAEAPI_ROKAE_MOTION_CONTROL_RT_H_
