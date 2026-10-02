/**
 * @file robot.h
 * @brief Robot Interaction Interface
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_ROBOT_H_
#define ROKAEAPI_ROBOT_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <cfloat>
#include "base.h"
#include "model.h"
#include "exception.h"
#include "motion_control_rt.h"
#include "planner.h"
#include "force_control.h"

namespace rokae {

 // forward declarations
 class BaseModel;

 /**
  * @struct Info
  * @brief Basic information of the robot, loaded after establishing a connection with the robot
  */
 struct Info {
   std::string id;      ///< Robot UID, can be used to distinguish connected robots
   std::string mac;     ///< MacAddress
   std::string version; ///< Controller Version
   std::string type;    ///< Robot Model Name
   int joint_num;       ///< Number of axles
 };

 /**
  * @struct StateList
  * @brief Status List
  */
 struct StateList {
   std::vector<double> joint_pos; ///< Shaft angle, unit: radian
   CartesianPosition cart_pos {}; ///< Cartesian pose
   std::vector<std::pair<std::string, bool>> digital_signals; ///< Digital IO, Signal Name and Value
   std::vector<std::pair<std::string, double>> analog_signals; ///< Analog IO, signal names and values
   OperateMode operation_mode; ///< Operating mode
   double speed_override; ///< Speed Coverage Ratio
 };

 /**
  * @class BaseRobot
  * @brief Universal Robot Interface
  */
 class XCORE_API BaseRobot : public Base<BaseRobot> {

  public:

   // **********************************************************************
   // *************       Network communication methods      ***************
   // *************                Network communication interface                ***************

   /**
    * @brief Disconnect from the robot. The robot's movement will stop before disconnecting, please pay attention to safety.
    * @param[out] ec Error code
    */
   void disconnectFromRobot(error_code &ec) noexcept;

   /**
    * @brief Set connection disconnect callback function
    * @param[in] handler Callback function, parameter is bool, true - connect | false-Disconnect
    */
   void setConnectionHandler(const std::function<void(bool)> &handler) noexcept;

   // **********************************************************************
   // ***************      Motors power on/off methods      ****************
   // ***************              Motor power on/off interface             *****************

   /**
    * @brief Robot power on/off and emergency stop status
    * @param[out] ec Error code
    * @return on-Power on | off-Power off | estop-Emergency stop | gstop-The safety door is open
    */
   PowerState powerState(error_code &ec) const noexcept;

   /**
    * @brief Powering the robot on and off. Note: Only robots without external enable switches or teach pendants can be powered on in manual mode.。
    * @param[in] on true-Power on | false-Power off
    * @param[out] ec Error code
    */
   void setPowerState(bool on, error_code &ec) noexcept;


   // **********************************************************************
   // ***************     Robot operation mode methods      ****************
   // ***************            Robot Operation Mode Interface            ****************

   /**
    * @brief Check the robot's current operating mode
    * @param[out] ec Error code
    * @return Manual | Automatic
    */
   OperateMode operateMode(error_code &ec) const noexcept;

   /**
    * @brief Switch between manual and automatic mode
    * @param[in] mode Manual/Automatic
    * @param[out] ec Error code
    */
   void setOperateMode(OperateMode mode, error_code &ec) noexcept;

   /**
    * @brief Restart the industrial control computer. Note: Restarting is not allowed in automatic mode, power-off state, during movement, or in non-idle state.
    * @param[out] ec Error code
    */
   void rebootSystem(error_code& ec)noexcept;

   // **********************************************************************
   // **************       Get robot information methods      **************
   // **************           Interface for querying robot information and status           **************

   /**
    * @brief Query basic information of the robot
    * @param[out] ec Error code
    * @return Basic Information of the Robot
    */
   Info robotInfo(error_code &ec) const noexcept;

   /**
    * @brief Check the current operating status of the robot (Idle, in motion, drag to open, etc.)
    * @param[out] ec Error code
    * @return Operation Status Enum Class
    */
   OperationState operationState(error_code &ec) const noexcept;

   // **********************************************************************
   // ******************      Get robot posture methods      ***************
   // ******************           Get Robot Pose Interface           ***************

   /**
    * @brief Current joint angle of the robot, robot body + external axes, unit: \f$[rad]\f$
    * External shaft guide unit\f$[m]\f$
    * @param[out] ec Error code
    * @return Length: \f$ \mathbb{R}^{DoF+ExJnt \times 1} \f$.
    */
   std::vector<double> jointPos(error_code &ec) noexcept;

   /**
    * @brief Current joint speed of the robot, robot body + external axes, unit: \f$[\frac{rad}{s}]\f$
    * External shaft guide rail, unit\f$[\frac{m}{s}]\f$
    * @param[out] ec Error code
    * @return Length: \f$ \mathbb{R}^{DoF+ExJnt \times 1} \f$.
    */
   std::vector<double> jointVel(error_code &ec) noexcept;

   /**
    * @brief Get the current pose of the robot flange or end effector \f$^{O}T_{F}~[m][rad]\f$.
    * @param[in] ct Coordinate System Type
    * 1) flangeInBase: Flange relative to the base coordinate system;
    * 2) endInRef: The end relative to the external reference coordinate system. For example, when the handheld tool and the external workpiece are set, this type of coordinate system returns the coordinates of the tool relative to the workpiece coordinate system.。
    *              For example, if the external reference coordinate system coincides with the base coordinate system, then the returned result is equivalent to the pose of the end relative to the base coordinate system.。
    * @param[out] ec Error code
    * @return doubleArray, Length: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
    * transformation and \f$ \mathbb{R}^{3 \times 1} \f$ rotation \f$ [x, y, z, rx, ry, rz]^T \f$.
    */
   std::array<double, 6> posture(CoordinateType ct, error_code &ec) noexcept;

   /**
    * @brief Get the current pose of the robot flange or end effector
    * @param[in] ct Coordinate System Type
    * @param[out] ec Error code
    * @return Current Cartesian Position
    */
   CartesianPosition cartPosture(CoordinateType ct, error_code &ec) noexcept;

   // **********************************************************************
   // ***************      Get and set coordinate methods      *************
   // *****************           Interface for obtaining and setting coordinate systems           ***************

   /**
    * @brief Read the base coordinate system, relative to the world coordinate system
    * @param[out] ec Error code
    * @return Array, Length: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
    * transformation and \f$ \mathbb{R}^{3 \times 1} \f$ rotation \f$ [x, y, z, rx, ry, rz]^T \f$.
    */
   std::array<double, 6> baseFrame(error_code &ec) const noexcept;

   /**
    * @brief Set the base coordinate system. After setting, only the values are saved, and it takes effect after restarting the controller.
    * @param[in] frame Coordinate system, default to using custom installation method
    * @param[out] ec Error code
    */
   void setBaseFrame(const Frame &frame, error_code &ec) noexcept;

   /**
    * @brief Query current tool workpiece group information
    * @note This tool workpiece group is only for SDK motion control and is not related to RL projects..
    * @param[out] ec Error code
    */
   Toolset toolset(std::error_code &ec) const noexcept;

   /**
    * @brief Set tool and workpiece group information
    * @note This tool workpiece group is for SDK use only and is not related to RL projects.。
    * After setting, the top right corner of RobotAssist will be displayed“toolx", "wobjx", The end coordinates displayed by status monitoring will also change。
    * Apart from this interface, if the default tool workpiece is changed via RobotAssist(The option in the upper right corner), The tool workpiece group will also change accordingly.。
    * @param[in] toolset Tool and Workpiece Group Information
    * @param[out] ec Error code
    */
   void setToolset(const Toolset& toolset, error_code &ec) noexcept;

   /**
    * @brief Using the created tools and workpieces, set up the tool and workpiece group information
    * @note Set the premise:
    *   1) Using tool artifacts created in the RL project: You need to load the corresponding RL project first;
    *   2) Global tool artifact: No need to load the project, just call it directly。e.g. setToolset("g_tool_0", "g_wobj_0")
    * A set of tool workpieces cannot be both handheld and external at the same time; if there is a conflict, the tool’s position takes precedence. For example, if a tool workpiece is both handheld, no error will be returned, but the workpiece's coordinate system will become external.
    * @param[in] toolName Tool Name
    * @param[in] wobjName Workpiece Name
    * @param[out] ec Error code
    * @return Information of the tool workpiece group after setting. When an error occurs and the setting fails, the default value initialized by the Toolset type is returned.0
    */
   Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept;

   // **********************************************************************
   // ********************    Input/Output devices     *********************
   // ********************         Input/Output Devices         *********************
   /**
    * @brief Query digital input signal value
    * @param[in] board IOBoard Number
    * @param[in] port Signal port number
    * @param[out] ec Error code
    * @return true-Open | false-Close
    */
   bool getDI(unsigned int board, unsigned int port, error_code &ec) noexcept;

   /**
    * @brief Set the digital input signal, which can only be set when the input simulation mode is turned on(seesetSimulationMode())
    * @param[in] board IOBoard Number
    * @param[in] port Signal port number
    * @param[in] state true-Open | false-Close
    * @param[out] ec Error code
    */
   void setDI(unsigned board, unsigned port, bool state, error_code &ec) noexcept;

   /**
    * @brief Query digital output signal value
    * @param[in] board IOBoard Number
    * @param[in] port Signal port number
    * @param[out] ec Error code
    * @return true-Open | false-Close
    */
   bool getDO(unsigned int board, unsigned int port, error_code &ec) noexcept;

   /**
    * @brief Set digital output signal value
    * @param[in] board IOBoard Number
    * @param[in] port Signal port number
    * @param[in] state true-Open | false-Close
    * @param[out] ec Error code
    */
   void setDO(unsigned int board, unsigned int port, bool state, error_code &ec) noexcept;

   /**
    * @brief Read analog input signal value
    * @param[in] board IOBoard Number
    * @param[in] port Signal port number
    * @param[out] ec Error code
    * @return Signal value
    */
   double getAI(unsigned board, unsigned port, error_code &ec) noexcept;

   /**
    * @brief Set analog output signal
    * @param[in] board IOBoard Number
    * @param[in] port Signal port number
    * @param[in] value Output value
    * @param[out] ec Error code
    */
   void setAO(unsigned board, unsigned port, double value, error_code &ec) noexcept;

   /**
    * @brief Set input simulation mode
    * @param[in] state true - Open | false - Close
    * @param[out] ec Error code
    */
   void setSimulationMode(bool state, error_code &ec) noexcept;

   /**
    * @brief Read register values. You can read a single register, an array of registers, or read a register array by index.。
    * If you want to read the entire register array, pass a vector of the corresponding type for value, and the index value is ignored。
    * @tparam T Read numeric type
    * @param[in] name Register Name
    * @param[in] index Read elements in the register array by index, starting from 0。
    *           The following two situations will cause an error：1) Index exceeds array length; 2) Registers are not arrays, but the index is greater than0
    * @param[out] value Register values, allowed types arebool/int/float
    * @param[out] ec Error code
    */
   template<typename T>
   void readRegister(const std::string &name, unsigned index, T &value, error_code &ec) noexcept;

   /**
    * @brief Write register values. You can write a single register, an array of registers, or write a specific element in the register array by index.。
    * If you want to write the entire register array, pass a vector of the corresponding type as the value, and the index is ignored。
    * @tparam T Write numerical type
    * @param[in] name Register Name
    * @param[in] index Array index starts from 0。
    *           The following two situations will cause an error：1) Index exceeds array length; 2) Registers are not arrays, but the index is greater than0
    * @param[in] value The written value
    * @param[out] ec Error code
    */
   template<typename T>
   void writeRegister(const std::string &name, unsigned index, T value, error_code &ec) noexcept;


   // **********************************************************************
   // ******************        Other operations      **********************
   // ******************             Other operations           **********************
   /**
    * @brief Clear servo alarm
    * @param[out] ec Error code, the error code is set when there is a servo alarm and clearing fails-1
    */
   void clearServoAlarm(error_code &ec) noexcept;

   // **********************************************************************
   // ***************        Recover state methods         *****************
   // ***************              Restore Status Interface             *****************

   /**
    * @brief Restore robot status according to options
    * @param[in] item Recovery options, 1: Emergency stop recovery
    * @param[out] ec Error code
    */
   void recoverState(int item, error_code &ec) noexcept;

   /**
    * @brief Check xCore-SDK version
    * @return Version number
    */
   static std::string sdkVersion() noexcept;

   /**
    * @brief Check the latest logs of the controller
    * @param[in] count The number of queries is limited to 10.
    * @param[in] level Specify the log level; an empty set means not specified
    * @param[out] ec Error code
    * @param[in] offset Offset number, for example, 0 means starting the query from the latest log, 10 means starting the query from the 11th log
    * @return Log information
    */
   std::vector<LogInfo> queryControllerLog(unsigned count, const std::set<LogInfo::Level>& level, error_code &ec, unsigned offset = 0) noexcept;

   // **********************************************************************
   // ******************      Motion Controller        *********************
   // ******************        Motion control related interfaces          *********************

   /**
    * @brief Set motion control mode
    * @note Before calling each motion control interface, the corresponding control mode must be set.。
    * @param[in] mode Mode
    * @param[out] ec Error code
    */
   void setMotionControlMode(MotionControlMode mode, error_code &ec) noexcept;

   //  --------------    MotionControlMode::NrtCommand   ------------------
   //  ----------------         Non-real-time motion command              ------------------
   /**
    * @brief Motion reset, clear sent motion commands, clear execution information
    * @note RobotThe class will call a motion reset once during initialization. RL programs and SDK motion command switching control require a motion reset first.。
    * @param[out] ec Error code
    */
   void moveReset(error_code &ec) noexcept;

   /**
    * @brief Start/Continue Exercise
    * @param[out] ec Error code
    */
   void moveStart(error_code &ec) noexcept;

   /**
    * @brief Pause robot movement; Can be called after pausemoveStart()Continue exercising. If you need to stop completely and no longer execute the added instructions, you can callmoveReset()
    * @note Currently supports stop2 stop type, planning to stop without power interruption, seeStopLevel。
    * @param[out] ec Error code
    */
   void stop(error_code &ec) noexcept;

   /**
    * @brief Add a single or multiple movement commands, call after addingmoveStart()Start exercising
    * @tparam Command Exercise command type: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand | MoveCFCommand |
    * MoveSPCommand;
    * Cartesian space motion uses Euler angles XYZ to represent rotation, that is, usingtrans&rpythe value
    * @param[in] cmds Command list, the allowed number is 1-100, and they must be commands of the same type
    * @param[out] cmdID The ID of this instruction, which can be used to query the execution information of the instruction
    * @param[out] ec Error code, only reports errors before the instruction is sent, including:
    *                1) Network connection issue; 2) The number of instructions does not match;
    */
   template<class Command>
   void moveAppend(const std::vector<Command> &cmds, std::string &cmdID, error_code &ec) noexcept;

   /**
   * @brief Add a single or multiple movement commands, call after addingmoveStart()Start exercising
   * @tparam Command Exercise command type: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand | MoveCFCommand |
   * MoveSPCommand
   * @param[in] cmds Command list, the allowed number is 1-100, and they must be commands of the same type
   * @param[out] cmdID The ID of this instruction, which can be used to query the execution information of the instruction
   * @param[out] ec Error code, only reports errors before the instruction is sent, including:
   *                1) Network connection issue; 2) The number of instructions does not match;
   */
   template<class Command>
   void moveAppend(std::initializer_list<Command> cmds, std::string &cmdID, error_code &ec) noexcept;

   /**
    * @brief Add a single motion command, call it after addingmoveStart()Start exercising
    * @tparam Command Exercise command type: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand |
    * MoveCFCommand | MoveSPCommand | MoveWaitCommand
    * @param[in] cmd Instruction
    * @param[out] cmdID The ID of this instruction, which can be used to query the execution information of the instruction
    * @param[out] ec Error code
    */
   template<class Command>
   void moveAppend(const Command &cmd, std::string &cmdID, error_code &ec) noexcept;

   /**
    * @brief Set the default movement speed, in mm/s, initial value is100
    * @note This value represents the maximum end-effector linear velocity and automatically calculates the corresponding joint velocities.。
    *   Joint speed percentage is divided into 5 ranges based on speed:
    *         < 100 : 10%
    *     100 ~ 200 : 30%
    *     200 ~ 500 : 50%
    *     500 ~ 800 : 80%
    *         > 800 : 100%
    *   Spatial rotation speed is200°/s
    * @param[in] speed This interface does not impose range restrictions on the parameters. The actual effective range of the terminal linear velocity is(0, 4000](Collaboration), (0, 7000](Industry)。
    * @param[out] ec Error code
    */
   void setDefaultSpeed(double speed, error_code &ec) noexcept;

   /**
    * @brief Set the default bend area, unit: mm. The initial value is0 (fine, No Turning Area)
    * @note This value represents the maximum turning radius of the movement and automatically calculates the turning percentage.
    *   Divide turning percentages into 4 ranges:
    *         < 1 : 0 (fine)
    *      1 ~ 20 : 10%
    *     20 ~ 60 : 30%
    *        > 60 : 100%
    * @param[in] zone This interface does not impose range restrictions on the parameters. The actual effective range of the turning area radius is[0, 200]。
    * @param[out] ec Error code
    */
   void setDefaultZone(double zone, error_code &ec) noexcept;

   /**
    * @brief Set whether to use axis configuration data(confData)Calculate the inverse solution. The initial value isfalse
    * @param[in] forced true - Use the motion command's confData to calculate the Cartesian point inverse kinematics, and return an error if the calculation fails.;
    * false - Do not use; when performing inverse kinematics, the solution closest to the robotic arm's current joint angles will be selected.
    * @param[out] ec Error code
    */
   void setDefaultConfOpt(bool forced, error_code &ec) noexcept;

   /**
    * @brief Set whether motion commands automatically cancel the turning area. The initial value istrue
    * @param[in] enable true - Automatic Turn Cancellation Zone | false - Will not automatically cancel the turning area
    * @param[out] ec Error code
    */
   void setAutoIgnoreZone(bool enable, error_code& ec) noexcept;

   /**
    * @brief Set the maximum number of cached commands, which refers to the number of path points sent to the controller for planning, allowed range[1,1000]，Initial value is300。
    * @note If the trajectories are mostly short, you can increase this value to prevent the robot from stopping due to delayed command transmission.(If there are any unexecuted instructions after stopping, you canmoveStart()Continue;
    * @param[in] number Quantity
    * @param[out] ec Error code
    */
   void setMaxCacheSize(int number, error_code &ec) noexcept;

   /**
    * @brief Dynamically adjust the robot's movement speed, effective in non-real-time mode。
    * @param[in] scale The speed ratio of the motion command, ranging from 0.01 to 1. When the scale is set to 1, the robot will move at the original speed of the path.。
    * @param[out] ec Error code
    */
   void adjustSpeedOnline(double scale, error_code &ec) noexcept;

   /**
    * @brief Read the current acceleration/deceleration and jerk
    * @param[out] acc Percentage of system preset acceleration
    * @param[out] jerk System preset acceleration percentage
    * @param[out] ec Error code
    */
   void getAcceleration(double &acc, double &jerk, error_code &ec) noexcept;

   /**
    * @brief Adjust the acceleration/deceleration and jerk of the movement. If called during the robot's motion, the currently executing instruction will not take effect, and the next instruction will take effect.
    * @param[in] acc Percentage of the system preset acceleration, range[0.2, 1.5], Exceeding the range will not produce an error; it automatically changes to the upper or lower limit value.
    * @param[in] jerk System preset percentage of acceleration, range[0.1, 2], Exceeding the range will not produce an error; it automatically changes to the upper or lower limit value.
    * @param[out] ec Error code
    */
   void adjustAcceleration(double acc, double jerk, error_code &ec) noexcept;

   /**
    * @brief To start jogging the robot, you need to switch to manual operation mode。
    * @note After calling this interface and the robot starts moving, it must be called regardless of whether the robot has stopped on its own.stop()End the jog operation, otherwise the robot will remain in the jog running state.。
    * @param[in] space jogReference coordinate system。
    *     1) The principles for using the tool/workpiece coordinate system are the samesetToolset();
    *     2) Industrial six-axis models and xMateCR/SR six-axis models support two types of singularity avoidance methods: JogOpt::singularityAvoidMode JogOpt::baseParallelMode
    *     3) CR5The spindle model supports parallel base modeJog: JogOpt::baseParallelMode
    * @param[in] rate Rate, Range 0.01 - 1
    * @param[in] step Step size. Unit: Cartesian space - millimeters | Axis space - degrees. The step size just needs to be greater than 0, no upper limit is set.，
    *             If the robot cannot continue jogging, it will stop moving on its own.。
    * @param[in] index According to different spaces, the meaning of this parameter is as follows：
    *     1) World coordinate system, base coordinate system, flange coordinate system, tool-workpiece coordinate system:
    *       a) 6Axle Model: 0~5correspond respectivelyX, Y, Z, Rx, Ry, Rz。>5represents the external axis(If there is)
    *       b) 7Axle model 6 represents the elbow joint, >6represents the external axis(If there is)
    *     2) Axis space: Joint index, starting from 0
    *     3) Strange Avoidance Mode, Parallel Base Mode:
    *       a) 6Axle Model：0~5correspond respectivelyX, Y, Z, J4(4Axis), Ry, J6(6Axis);
    *       b) 5Axle Model：0~4correspond respectivelyX, Y, Z, Ry, J5(5Axis)
    * @param[in] direction Depending on the different space and index, the meaning of this parameter is as follows：
    *     1) Strange Avoidance Mode J4: true - ±180° | false - 0°;
    *     2) Parallel Base Mode J4 & Ry: true - ±180° | false - 0°
    *     3) Other, true - positive | false - Negative
    * @param[out] ec Error code
    */
   void startJog(JogOpt::Space space, double rate, double step, unsigned index, bool direction, error_code &ec) noexcept;

   /**
    * @brief Execute single or multiple movement commands. After being called, the robot will start moving immediately.
    * @tparam Command Exercise command type: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand | MoveCFCommand |
    * MoveSPCommand;
    * Cartesian space motion uses Euler angles XYZ to represent rotation, that is, usingtrans&rpythe value
    * @param[in] cmds Command list, the allowed number is 1-100, must be commands of the same type
    * @param[out] ec Error code, only reports errors before execution, including:
    *                1) Network connection issue; 2) The number of instructions does not match
    */
   template<class Command>
   void executeCommand(const std::vector<Command> &cmds, error_code &ec) noexcept;

   /**
    * @brief Execute single or multiple movement commands. After being called, the robot will start moving immediately.
    * @tparam Command Exercise command type: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand;
    * Cartesian space motion uses Euler angles XYZ to represent rotation, that is, usingtrans&rpythe value
    * @param[in] cmds Command list, the allowed number is 1-100, must be commands of the same type
    * @param[out] ec Error code, only reports errors before execution, including:
    *                1) Network connection issue; 2) The number of instructions does not match;
    */
   template<class Command>
   void executeCommand(std::initializer_list<Command> cmds, error_code &ec) noexcept;

   // *********************************************************************
   // *****************         Event/State Listening              ******************

   /**
    * @brief Set the callback function for receiving events
    * @param[in] eventType Event Type
    * @param[in] callback Callback function for handling events. Explanation:
    *   1) For Event::moveExecution, the callback function is executed in the same thread. Please avoid long-running operations in the function.;
    *   2) Event::safetyThen each independent thread callback has no execution time limit
    * @param[out] ec Error code
    */
   void setEventWatcher(Event eventType, const EventCallback &callback, error_code &ec) noexcept;

   /**
    * @brief Query event information. AndsetEventWatcher()The information provided during the callback is the same; the difference is that this interface uses an active query method.
    * @param[in] eventType Event Type
    * @param[out] ec Error code
    * @return Event Information
    */
   EventInfo queryEventInfo(Event eventType, error_code &ec) noexcept;

   // *********************************************************************
   // *****************        Obtain real-time robot status data        ******************

   /**
    * @brief Stop receiving real-time status data, while the controller stops sending. Can be used to reset the status data to be received.。
    */
   void stopReceiveRobotState() noexcept;

   /**
    * @brief Receive a set of robot status data. Before reading the data each cycle, this function needs to be called. It is recommended to call it according to the set sending frequency to obtain the latest data.
    * @param[in] timeout Timeout
    * @return The length of the received data. If no data is received before the timeout, then return0。
    * @throw RealtimeControlException Unable to receive data; or the received data has errors that make it impossible to parse
    * @throw RealtimeMotionException Real-time mode motion error
    */
   unsigned updateRobotState(std::chrono::steady_clock::duration timeout);

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
   int getStateData(const std::string &fieldName, R &data);

   //  --------------    MotionControlMode::NrtRLTask   ------------------
   //  ----------------         RLEngineering-related instructions            -------------------

   /**
    * @brief Query the RL project name and tasks in the industrial computer
    * @param[out] ec Error code
    * @return Project information list. If no projects have been created, an empty list is returned.
    */
   std::vector<RLProjectInfo> projectsInfo(error_code &ec) noexcept;

   /**
    * @brief Load Project
    * @param[in] name Project Name
    * @param[in] tasks The task to be executed. This parameter must be specified and cannot be empty, otherwise the project cannot be executed.。
    * @param[out] ec Error code
    */
   void loadProject(const std::string &name, const std::vector<std::string> &tasks, error_code &ec) noexcept;

   /**
    * @brief The program pointer jumps to main. After the call, it waits for the controller to finish parsing the project and then return. The blocking time depends on the project size, with a timeout set to 10 seconds.。
    * @param[out] ec Error code. The information that an error code can provide is limited, and it cannot report errors such as RL syntax errors or non-existent variables. It can be accessed throughqueryControllerLog()Query error log。
    */
   void ppToMain(error_code &ec) noexcept;

   /**
    * @brief Start running the currently loaded project
    * @param[out] ec Error code
    */
   void runProject(error_code &ec) noexcept;

   /**
    * @brief Suspend the project
    * @param[out] ec Error code
    */
   void pauseProject(error_code &ec) noexcept;

   /**
    * @brief Change the operation speed and cycle mode of the project
    * @param[in] rate Operating speed, range 0.01 - 1
    * @param[in] loop true - loop execution | false - Single execution
    * @param[out] ec Error code
    */
   void setProjectRunningOpt(double rate, bool loop, error_code &ec) noexcept;

   /**
    * @brief Query the tool information of the currently loaded project
    * @param[out] ec Error code
    * @return Tool information list. If no project is loaded or no tool is created, the information of the default tool tool0 will be returned.
    */
   std::vector<WorkToolInfo> toolsInfo(std::error_code &ec) noexcept;

   /**
    * @brief Query the artifact information of the currently loaded project
    * @param[out] ec Error code
    * @return Workpiece information list; if no project is loaded or no workpiece is created, it returns emptyvector
    */
   std::vector<WorkToolInfo> wobjsInfo(std::error_code &ec) noexcept;

   /**
    * @brief Import the local RL project archive into the controller. Block until the import is complete or fails.
    * @param[in] file_path Local .zip archive path, file size within 10M
    * @param[in] overwrite Whether to overwrite files with the same name: Yes: Overwrite; No: Rename automatically
    * @param[out] ec Error code
    * @return Project name (for example, automatic renaming, return the name after renaming）
    */
   std::string importProject(const std::string &file_path, bool overwrite, error_code &ec) noexcept;

   /**
    * @brief Delete the RL project in the controller
    * @param[in] project_name Project Name
    * @param[in] remove_all Do you want to delete all projects? The default value isfalse
    * @param[out] ec Error code
    */
   void removeProject(const std::string &project_name, error_code &ec, bool remove_all = false) noexcept;

   /**
    * @brief Import local files to the controller. Block until the import is complete or fails.
    * @param[in] src_file_path Local file path. The file size is within 10M.
    * @param[in] dest Target Path
    *   1) Transfer a single RL project .mod file: project/[Project Name]/[Task Name]</[modFile name]>
    *   2) Transfer RL Engineering configuration files in .json/.xml/.sys format: project/[Project Name]</[File name]>。
    *      Note: The configuration file name cannot be changed, for example, the task file name must be"task.xml"
    * @param[in] overwrite Overwrite files with the same name: true - Overwrite | false - Automatic renaming. Only .mod files support automatic renaming
    * @param[out] ec Local file does not exist; File format error; Transmission failed; The target path does not meet the requirements, etc.
    * @return File name after successful import
    */
   std::string importFile(std::string src_file_path, std::string dest, bool overwrite, error_code &ec) noexcept;

   /**
    * @brief Delete the files in the controller. Note: Configuration files such as project.xml, .json, etc. cannot be deleted, only replaced.
    * @param[in] file_path_list List of file paths, a single file path is as follows:
    *   1) Delete the .mod file under a certain task of a certain project: project/[Project Name]/[Task Name]/[modFile name]
    *   2) Delete a certain task of a certain project: project/[Project Name]/[Task Name]
    * @param[out] ec Parameter format error or network error. No error code is returned if the project, task, or file does not exist.
    */
   void removeFiles(std::vector<std::string> file_path_list, error_code &ec) noexcept;

   /**
    * @brief Set global tool information, or create/set the pose and load information of the tool in the RL project
    * @note Explanation:
    *   1) Global Tools: The controller supports 16 global tools, with fixed names of "g_tool_0" ~ "g_tool_15"
    *   2) RLEngineering Tools: There are many restrictions when using them, so it is not recommended to set engineering tools through this interface. It is recommended to use global tools. The restrictions are:
    *       a) You need to load a project first, and then set it up. As long as the name is not a global tool, it is considered a project tool.。
    *       b) If the tool does not exist, create it; if it exists, modify it.。
    *       c) You need to modify the tool configuration file of the project accordingly, otherwise the RL command may not be able to properly parse the tool information. Moreover, if the configuration file is not modified, the set data will not be saved.。
    *   3) Setting tool envelope information is not supported at the moment
    * @param[in] tool_info Tool Information。
    * @param[out] ec Global tools generally do not fail to be set. Tools in a project may fail to be set, for example, if the project's tool configuration file was pushed to the controller but the project was not reloaded, errors will be returned when the tool configuration is inconsistent.
    */
   void setToolInfo(const WorkToolInfo &tool_info, error_code &ec) noexcept;

   /**
    * @brief Set global workpiece information, or create/set the pose information and load information of the workpiece in the RL project
    * @note Explanation:
    *   1) Global Artifacts: The controller supports 16 global artifacts, with fixed names as "g_wobj_0" ~ "g_wobj_15"
    *   2) RLProject Workpiece: There are many usage limitations, and it is not recommended to set the project workpiece through this interface. It is recommended to use a global workpiece. The limitations are:
    *       a) You need to load a project first before setting it up. As long as the name is not a global artifact, it is considered a project artifact.。
    *       b) If the workpiece does not exist, create a new one; if it exists, modify it.。
    *       c) It is necessary to modify the workpiece configuration file in coordination with the project; otherwise, the RL command may not be able to correctly parse tool information. Moreover, if the configuration file is not modified, the set data will not be saved.。
    *   3) Setting related user coordinate systems is not currently supported; the global workpiece defaults to"g_user_0", The engineering workpiece defaults to "userframe0"
    * @param[in] wobj_info Workpiece Information
    * @param[out] ec Similarly, set up the tool interface. Global artifacts generally will not fail to be set. Artifacts in the project may fail to be set.
    */
   void setWobjInfo(const WorkToolInfo &wobj_info, error_code &ec) noexcept;

   // ---------------------------------------------------------------------
   // ---------------------           External shaft          ----------------------

   /**
    * @brief Set guide rail parameters
    * @tparam R Parameter Type
    * @param[in] name Parameter name, see value description
    * @param[in] value
    *   Parameter             |    Parameter Name         |   Data Type
    *   Switch             | enable           | bool
    *   Base coordinate system         | baseFrame         | Frame
    *   Guide rail name         | name              | std::string
    *   Encoder resolution      | encoderResolution | int
    *   Reduction ratio           | reductionRatio    | double
    *   Maximum motor speed(rpm) | motorSpeed       | int
    *   Soft limit(m), [Lower limit, upper limit]   | softLimit  | std::vector<double>
    *   Range of motion(m), [Lower limit, upper limit] | range      | std::vector<double>
    *   Maximum speed(m/s)      | maxSpeed | double
    *   Maximum acceleration（m/s^2)  | maxAcc  | double
    *   Maximum jerk(m/s^3) | maxJerk | double
    * @param[out] ec Error code. An error code is returned if the parameter name does not exist or the data type does not match.
    */
   template<typename R>
   void setRailParameter(const std::string &name, R value, error_code &ec) noexcept;

   /**
    * @brief Read rail parameters
    * @tparam R Parameter Type
    * @param[in] name Parameter name, seesetRailParameter()
    * @param[out] value Parameter values, seesetRailParameter()
    * @param[out] ec Error code: The parameter name does not exist or the data type does not match, returning an error code
    */
   template<typename R>
   void getRailParameter(const std::string &name, R &value, error_code &ec) noexcept;

   // *********************************************************************
   // ***********           NTPRelated (Non-standard feature, requires additional installation)         **********

   /**
    * @brief Configure NTP. Non-standard feature, requires additional installation.。
    * @param[in] server_ip NTPServer sideIP
    * @param[out] ec Error code, NTP service not installed correctly, or address is invalid
    */
   void configNtp(const std::string &server_ip, error_code &ec) noexcept;

   /**
    * @brief Manually synchronize the time once, the remote IP is configured through configNtp. It takes a few seconds, blocking and waiting for the synchronization to complete. The preset timeout for the interface is 12 seconds.
    * @param[out] ec Error code, NTP service is not properly installed, or cannot synchronize with the server
    */
   void syncTimeWithServer(error_code &ec) noexcept;

   /**
    * @brief Check whether the Cartesian trajectory is reachable, straight-line trajectory
    * @param[in] start Starting point
    * @param[in] start_joint Starting shaft angle [radian]
    * @param[in] target Target point
    * @param[out] ec Includes unreachable error causes
    * @return The calculated target shaft angle is only valid when there are no error codes.
    * @note Supports guide rails, the returned target axis angle is the number of axes plus the number of external axes
    */
   std::vector<double> checkPath(const CartesianPosition &start,
                                const std::vector<double> &start_joint,
                                const CartesianPosition &target,
                                error_code &ec) noexcept;

   /**
    * @brief Verify multiple linear trajectories
    * @param[in] start_joint Starting shaft angle, unit[radian]
    * @param[in] points Cartesian points require at least two points, the first point being the starting point.
    * @param[out] target_joint_calculated If the verification passes, return the calculated target axis angle
    * @param[out] ec Reason for verification failure
    * @return If the verification fails, return the index of the erroneous target point in points. In other cases, return0
    */
   int checkPath(const std::vector<double> &start_joint,
                 const std::vector<CartesianPosition> &points,
                 std::vector<double> &target_joint_calculated,
                 error_code &ec) noexcept;

   /**
    * @brief Check whether the Cartesian trajectory is reachable, including arcs and full circles
    * @param[in] start Starting point
    * @param[in] start_joint Starting shaft angle [radian]
    * @param[in] aux Reference point
    * @param[in] target Target point
    * @param[out] ec Includes unreachable error causes
    * @param[in] angle Full-circle execution angle, when not equal to zero, represents verifying the full-circle trajectory
    * @param[in] rot_type Full circular rotation type
    * @return The calculated target shaft angle is only valid when there are no error codes.
    * @note Supports guide rails, the returned target axis angle is the number of axes plus the number of external axes
    */
   std::vector<double> checkPath(const CartesianPosition &start,
                                 const std::vector<double> &start_joint,
                                 const CartesianPosition &aux,
                                 const CartesianPosition &target,
                                 error_code &ec, double angle =0.0,
                                 MoveCFCommand::RotType rot_type = MoveCFCommand::constPose) noexcept;

  /**
   * @brief Teach pendant hot-swapping. Note: Only some models support teach pendant hot-swapping; models that do not support it will return an error code. After not using the teach pendant, the enable button and the emergency stop button will be disabled.。
   * @param[in] enable true - Use the teach pendant | false - Do not use a teaching pendant
   * @param[out] ec The robot cannot switch states at the moment(Power on in motion/manual mode); Switching failed due to reasons such as the device model
   */
   void setTeachPendantMode(bool enable, error_code& ec) noexcept;

   /**
    * @brief Destroying the Robot object will stop the robot from moving
    */
   virtual ~BaseRobot() noexcept;

  protected:

   /// @cond DO_NOT_DOCUMENT
   BaseRobot();
   /// @endcond

  public:
   /// @cond DO_NOT_DOCUMENT
   BaseRobot(const BaseRobot&) = delete;
   BaseRobot& operator= (BaseRobot&) = delete;
   /// @endcond

  XCORESDK_DECLARE_IMPL
 };

 /**
  * @class Robot_T
  * @brief Robot Template Class
  * @tparam Wt Collaboration/Industrial Type
  * @tparam DoF Number of axles
  */
 template<WorkType Wt, unsigned short DoF>
 class XCORE_API Robot_T : virtual public BaseRobot {

  public:
   /**
    * @brief Default constructor, call connectToRobot(remoteIp) afterwards
    */
   Robot_T() = default;

   /**
    * @brief Create a robot instance and connect to the robot
    * @param[in] remoteIP Robot IP address
    * @param[in] localIP Local machine address. Used for sending and receiving interactive data in real-time mode, can be left unset; not supported by PCB 3/4-axis models.
    * @throw NetworkException Network connection error
    * @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
    */
   explicit Robot_T(const std::string &remoteIP, const std::string &localIP = "");

   /**
    * @brief Connect to the robot. The robot address is the one passed in when creating the robot instance.
    * @param[out] ec Error code
    */
   void connectToRobot(error_code &ec) noexcept;

   /**
    * @brief Connect to the robot
    * @param remoteIP Robot IP address
    * @param localIP Local machine address. Used for sending and receiving interactive data in real-time mode, can be left unset; not supported by PCB 3/4-axis models.
    * @throw NetworkException Network connection error
    * @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
    */
   void connectToRobot(const std::string &remoteIP, const std::string &localIP = "");

   // **********************************************************************
   // ******************             Status Inquiry              ******************

   /**
    * @brief Query current position, IO signal, operation mode, speed override value
    * @param ec Error code
    * @return Query Results
    */
   StateList getStateList(error_code &ec) noexcept;

   // **********************************************************************
   // ******************    Get robot joint state       ********************
   // ******************        Get robot joint status         *********************

   /**
    * @brief Current axis angle of the robot, in units: \f$[rad]\f$
    * @param[out] ec Error code
    * @return Length: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointPos(error_code &ec) noexcept;

   /**
    * @brief Current joint speed of the robot, in units: \f$[\frac{rad}{s}]\f$
    * @param[out] ec Error code
    * @return Length: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointVel(error_code &ec) noexcept;

   /**
    * @brief Joint force sensor value, unit: \f$[Nm]\f$
    * @param[out] ec Error code
    * @return Length: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointTorque(error_code &ec) noexcept;

   // **********************************************************************
   // *********************             Calibration           **********************

   /**
    * @brief Coordinate system calibration (NPoint calibration)
    * @param[in] type Coordinate system types, support tools(FrameType::tool), Workpiece(FrameType::wobj), Base coordinate system(FrameType::base)
    * @note  Calibration Methods and Precautions Supported by Each Coordinate System Type：
    *   1) Tool Coordinate System: Three-Point/Four-Point/Six-Point Calibration Method
    *   2) Workpiece coordinate system: three-point calibration. The calibration result will not be transformed relative to the user coordinate system; that is, if it is an external workpiece, the returned result is relative to the base coordinate system.。
    *   3) Base coordinate system: six-point calibration. Please make sure that kinematic constraints and feedforward are turned off before calibration.。
    *              If calibration is successful(No error code)，The controller will automatically save the calibration results, which will take effect after restarting the controller.。
    *   4) Guide rail base coordinate system: three-point calibration. If the calibration is successful(No error code)，The controller will automatically save the calibration results, which will take effect after restarting the controller.。
    * @param[in] points Axis angle list, with a list length of N. For example, when calibrating the tool coordinate system using the three-point method, 3 sets of axis angles should be provided. The unit of axis angles is radians.。
    * @param[in] is_held true - Robot handheld | false - External. Only affects the calibration of tools/workpieces
    * @param[out] ec Error code
    * @param[in] base_aux Auxiliary points used during base coordinate system calibration, unit[rice]
    * @return Calibration result: The calibration result is valid when the error code is not set.。
    */
   FrameCalibrationResult calibrateFrame(FrameType type, const std::vector<std::array<double, DoF>> &points, bool is_held,
                                         error_code &ec, const std::array<double, 3> &base_aux = {}) noexcept;

   // **********************************************************************
   // ********     Robot model for dynamic/kinematic calculation    ********
   // ********         Get the robot model class for kinematics/dynamics calculations             ********

   /**
    * @brief Get model class
    * @return ModelClass
    */
   Model_T<DoF> model() noexcept;

   // *********************************************************************
   // *****************        Obtain real-time robot status data        ******************

   /**
    * @brief Let the robot controller start sending real-time status data. Block and wait to receive the first frame message, with a timeout of 3 seconds.
    * @param[in] interval The interval at which the controller sends status data, allowed duration：1ms/2ms/4ms/8ms/1s
    * @param[in] fields The received robot status data has a maximum total length of 1024 bytes. Supported data and names are as followsdata_types.h, RtSupportedFields
    * @throw RealtimeControlException Set unsupported status data; or the robot cannot start sending data; or the total length exceeds1024
    * @throw RealtimeStateException Data transmission has already started; or the first frame of data has not been received after a timeout
    */
   void startReceiveRobotState(std::chrono::steady_clock::duration interval, const std::vector<std::string>& fields);

   /**
    * @brief Get current soft limit value
    * @param[out] limits Soft limits for each axis [Lower limit, upper limit]，Unit: Radian
    * @param[out] ec Error code
    * @return true - Already opened | false - Closed
    */
   bool getSoftLimit(std::array<double[2], DoF> &limits, error_code &ec) noexcept;

   /**
    * @brief Set the soft limit. Soft limit setting requirements：
    *     1) When the soft limit is activated, the robotic arm should be powered off and in manual mode.;
    *     2) The soft limit cannot exceed the mechanical hard limit.
    *     3) The current angles of each axis of the robotic arm should be within the set limit range.
    * @param[in] enable true - Open | false - Close。
    * @param[out] ec Error code
    * @param[in] limits Each axis[Lower limit, upper limit]，Unit: Radian。
    *     1) When limits are the default value, it is considered as only enabling the soft limit without modifying the value; If not for the default value, modify the soft limit first before turning it on
    *     2) Turning off the soft limit will not change the limit values
    */
   void setSoftLimit(bool enable, error_code &ec, const std::array<double[2], DoF> &limits = {{DBL_MAX, DBL_MAX}}) noexcept;

 };

 /**
  * @brief General-purpose Collaborative Robot
  */
 class XCORE_API BaseCobot: virtual public BaseRobot {
  public:
   using BaseRobot::BaseRobot;

   /**
    * @brief Open and drag
    * @param[in] space Drag space. Axis space dragging only supports free drag type
    * @param[in] type Drag Type
    * @param[out] ec Error code
    * @param[in] enable_drag_button true - After enabling the drag function, you can directly drag the robot without holding down the end button.
    */
   void enableDrag(DragParameter::Space space, DragParameter::Type type, error_code& ec, bool enable_drag_button = false) noexcept;

   /**
    * @brief Disable dragging
    * @param[out] ec Error code
    */
   void disableDrag(error_code& ec) noexcept;

   /**
    * @brief Start recording path
    * @param[in] duration Duration of the path, unit: seconds, range1~1800.At this time, the length is only used for range checking; the controller will not stop recording, it needs to be called.stopRecordPath()Come to stop
    * @param[out] ec Error code
    */
   void startRecordPath(int duration, error_code& ec) noexcept;

   /**
    * @brief Stop recording the path if the recording is successful(No error code)Then the path data is stored in the cache
    * @param[out] ec Error code
    */
   void stopRecordPath(error_code& ec) noexcept;

   /**
    * @brief Cancel recording, the cached path data will be deleted
    * @param[out] ec Error code
    */
   void cancelRecordPath(error_code& ec) noexcept;

   /**
    * @brief Save the recorded path
    * @param[in] name Path Name
    * @param[out] ec Error code
    * @param[in] saveAs Rename, optional parameter。
    *               If a path has been recorded but not saved, save the path with that name. If there is no unsaved path, use the already saved name"name"Rename the path to"saveAs"
    */
   void saveRecordPath(const std::string& name, error_code& ec, const std::string& saveAs = "") noexcept;

   /**
    * @brief Motion Command - Path Playback。
    * Like other movement commands, after calling replayPath, moveStart must be called to start the movement.。
    * @param[in] name The path name to be replayed
    * @param[in] rate Playback rate should be less than 3.0, with 1 being the original path rate. Note that when the rate is greater than 1, it may cause drive errors due to inability to keep up.
    * @param[out] ec Error code
    */
   void replayPath(const std::string& name, double rate, error_code& ec) noexcept;

   /**
    * @brief Delete the saved path
    * @param[in] name Path name to delete
    * @param[out] ec Error code. If the path does not exist, the error code will not be set.
    * @param[in] removeAll Whether to delete all paths, optional parameter, default is no
    */
   void removePath(const std::string& name, error_code& ec, bool removeAll = false) noexcept;

   /**
    * @brief Query all saved path names
    * @param[out] ec Error code
    * @return Name list, return an empty list if there is no path
    */
   std::vector<std::string> queryPathLists(error_code& ec) noexcept;

   /**
    * @brief Set the xPanel external power supply mode. Note: Only some models support the xPanel function; models that do not support it will return an error code.
    * @param[in] opt Mode
    * @param[out] ec Error code
    */
   void setxPanelVout(xPanelOpt::Vout opt, error_code& ec) noexcept;

   /**
    * @brief To use the 485 communication function at the CR and SR terminals, it is necessary to modify the parameter configuration of the terminals, which can be done through this interface.
    * @param[in] opt External power supply mode, 0: no output, 1: reserved，2：12v，3：24v
    * @param[in] if_rs485 Interface working mode, whether to enable terminal 485 communication
    * @param[out] ec Error code
    */
   void setxPanelRS485(xPanelOpt::Vout opt, bool if_rs485, error_code& ec) noexcept;

   /**
    * @brief Read and write Modbus registers through the xPanel terminal
    * @param[in] slave_addr Device Address 0-65535
    * @param[in] fun_cmd Function code 0x03 0x04 0x06 0x10
    * @param[in] reg_addr Register address 0-65535
    * @param[in] data_type Supported data types  int32、int16、uint32、uint16
    * @param[in] num The number of consecutive operating registers is 0-3. When the type is int16/uint16, the maximum is 3; when the type is int32/uint32 or float, the maximum is 1. When the function code is 0x06, this parameter is invalid.
    * @param[in/out] data_array Array for sending or receiving data, non-const; when the function code is 0x06, only the data in this array is used[0],At this time, the value of num is invalid. Except for the 0x06 function code, the size needs to match num.
    * @param[in] if_crc_reverse Whether to change the CRC checksum high and low bytes, default is false, required to reverse by a few manufacturers' terminal tools
    * @param[out] ec Error code
    */
   void XPRWModbusRTUReg(int slave_addr, int fun_cmd, int reg_addr, std::string data_type, int num, std::vector<int>& data_array, bool if_crc_reverse, error_code& ec) noexcept;

   /**
    * @brief Read and write Modbus coils or discrete inputs through the xPanel terminal
    * @param[in] slave_addr Device Address 0-65535
    * @param[in] fun_cmd Function code 0x01 0x02 0x05 0x0F
    * @param[in] coil_addr Coil or Discrete Input Register Address 0-65535
    * @param[in] num The number of coils or discrete inputs read/write consecutively (0-48). This value is invalid when the function code is 0x05.
    * @param[in/out] data_array Array for sending or receiving data, not const. When the function code is 0x05, only the data in this array is used.[0],At this time, the value of num is invalid. Except for the 0x05 function code, the size needs to match num.
    * @param[in] if_crc_reverse Whether to change the CRC checksum high and low bytes, default is false, required to reverse by a few manufacturers' terminal tools
    * @param[out] ec Error code
    */
   void XPRWModbusRTUCoil(int slave_addr, int fun_cmd, int coil_addr, int num, std::vector<bool>& data_array, bool if_crc_reverse, error_code& ec) noexcept;

   /**
    * @brief Directly transmit raw RTU protocol data through the xPanel terminal
    * @param[in] send_byte Send byte length  0-16
    * @param[in] rev_byte Received byte length 0-16
    * @param[in] send_data Sending byte data The array length needs to be the same assend_byte Parameter consistent
    * @param[out] rev_data The length of the received byte data array needs to matchrev_byte Parameter consistent
    * @param[out] ec Error code
    */
   void XPRS485SendData(int send_byte, int rev_byte, const std::vector<uint8_t>& send_data, std::vector<uint8_t>& rev_data, error_code& ec) noexcept;

   /**
    * @brief Get the status of the end button; unsupported models will return an error code
    * @param[out] ec Error code
    * @return The status of the end terminal button. The end terminal button numbers can be found in the illustration of the end handle in the "xCore Robot Control System User Manual".。
    */
   KeyPadState getKeypadState(error_code& ec) noexcept;

   // **********************************************************
   // *****************    Real-time interface    ***************************

   /**
    * @brief Set the network delay threshold for sending real-time motion commands, that is, the 'Packet Loss Threshold' in the RobotAssist - RCI settings interface“。
    * Please make the settings before switching to RtCommand mode, otherwise they will not take effect.。
    * @param[in] percent Allowed range 0 - 100. It is recommended to run above 20% on Linux.; WindowsIt is recommended to operate above 60%
    * @param[out] ec Error code
    */
   void setRtNetworkTolerance(unsigned percent, error_code& ec) noexcept;

   /**
    * @brief Interface compatible with RCI client settings. After setting the motion control mode to real-time mode through the SDK, the original RCI client can no longer be used to control the robot.。
    * If there is a need to use the original version, you can call this interface after switching to non-real-time mode. Then, you can enable the RCI function on RobotAssist to use the RCI client.。
    * @param[in] use true - Switch to using the first generation
    * @param[out] ec Error code
    */
   void useRciClient(bool use, error_code& ec) noexcept;

   // *********************************************************************
   // ******************            Collision Detection            *********************

   /**
    * @brief Turn off collision detection
    * @param[out] ec Error code
    */
   void disableCollisionDetection(error_code& ec) noexcept;

 };

 /**
  * @class Cobot
  * @brief Collaborative robot template class, providing interfaces for collaborative robot support features
  * @tparam DoF Number of axes
  */
 template<unsigned short DoF>
 class XCORE_API Cobot : public Robot_T<WorkType::collaborative, DoF>, public BaseCobot {
  public:
   /**
    * @brief default constructor
    */
   Cobot();

   /**
     * @brief Collaborative robot
     * @param[in] remoteIP Robot IP address
     * @param[in] localIP Local machine address. Used for sending and receiving interactive data in real-time mode, can be left unset.。
     * @throw NetworkException Network connection error
     * @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
     */
   explicit Cobot(const std::string &remoteIP, const std::string& localIP = "");

   /**
    * @brief Destruct; if the robot is moving at this time, it will stop moving.
    */
   virtual ~Cobot();


   // **********************************************************
   // *****************    Real-time interface    ***************************

   /**
    * @brief Create real-time motion control class(RtMotionControlCobot)Instance, perform real-time mode related operations through this instance pointer。
    * @note Unless this interface is called repeatedly, the client’s internal logic will not actively destruct the returned object.，
    * Including but not limited to disconnecting and connecting to robotsdisconnectFromRobot()，Switching to non-real-time motion control mode and so on, but performing real-time mode control after doing the above operations will cause abnormalities。
    * @return Controller object
    * @throw RealtimeControlException Failed to create RtMotionControl instance due to network issues
    * @throw ExecutionException Did not switch to real-time motion control mode
    */
   std::weak_ptr<RtMotionControlCobot<DoF>> getRtMotionController();


   // *********************************************************************
   // ******************            Collision Detection            *********************

   /**
    * @brief Set collision detection related parameters and enable the collision detection function。
    * @param[in] sensitivity Collision detection sensitivity, range0.01-2.0
    * @param[in] behaviour Robot behavior after collision, supportedstop1(Safe stop, stop0 and stop1 are handled in the same way), stop2(Trigger Pause）, suppleStop(Smooth stop)
    * @param[in] fallback_compliance
    *   1) When the behavior after a collision is a safe stop or trigger pause, this parameter refers to the rebound distance after the collision, unit: meters
    *   2) When the behavior after collision is compliant stop, this parameter means compliance level, range [0.0, 1.0]
    * @param[out] ec Error code
    */
   void enableCollisionDetection(const std::array<double, DoF> &sensitivity, StopLevel behaviour,
                                 double fallback_compliance, error_code &ec) noexcept;

   /**
    * @brief Force sensor calibration. The calibration process takes about 100 ms, and this function does not block while waiting for the calibration to complete.。
    * It needs to pass through before calibrationsetToolset()Set the correct load(Toolset::load), Otherwise, it will affect the accuracy of the calibration results.。
    * @param[in] all_axes true - Calibrate all axes | false - Single-axis calibration
    * @param[in] axis_index Axis subscript, range[0, DoF), Only effective when calibrating a single axis
    * @param[out] ec Error code
    */
   void calibrateForceSensor(bool all_axes, int axis_index, error_code &ec) noexcept;

   // *********************************************************************
   // *********************          Force control command            ********************

   /**
    * @brief Force Control Command Type
    * @return ForceControl_T
    */
   ForceControl_T<DoF> forceControl() noexcept;


#ifdef XMATEMODEL_LIB_SUPPORTED
   /**
    * @brief Get xMate model class
    * @throw ExecutionException Failed to read model parameters from the controller
    */
   xMateModel<DoF> model();
#endif

  XCORESDK_DECLARE_IMPLD
 };

 /**
  * @class IndustrialRobot
  * @brief Industrial Robot Template Class
  * @tparam DoF Number of axles
  */
 template<unsigned short DoF>
 class XCORE_API IndustrialRobot: public Robot_T<WorkType::industrial,DoF> {
  public:
   using Robot_T<WorkType::industrial,DoF>::Robot_T;
 };

 // ***********************************************************************
 // ***************      Robot classes for instantiate      ***************
 // ***************           Instantiable robot class             ****************

 /**
  * @class xMateRobot
  * @brief 6Axis collaborative robots, including xMateCR7/12, xMateSR3/4, xMateER3/7
  */
 class XCORE_API xMateRobot : public Cobot<6> {
  public:
   /**
    * @brief default constructor
    */
   xMateRobot() = default;

   /**
    * @brief Create a robot instance and connect
    * @param remoteIP Robot IP address
    * @param localIP Local machine address, needs to be set when sending and receiving data in real time
    * @throw NetworkException Network connection error
    * @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
    */
   explicit xMateRobot(const std::string &remoteIP, const std::string& localIP = "");

   /**
    * @brief Turn on/off the singularity avoidance feature. Only applicable to certain models.:
    *   1) Four-axis locking: Supports xMateCR and xMateSR models；
    *   2) Sacrifice posture: Supports all collaborative six-axis models；
    *   3) Axis space interpolation: Not supported
    * @param[in] method Strange Avoidance Method
    * @param[in] enable true - Turn on the feature | false - Close;
    * For the four-axis locking method, make sure the four axes are at zero position before opening.。
    * @param[in] limit Different avoidance methods, the meaning of this parameter is respectively:
    *   1) Sacrifice posture: Allowed posture error, range (0, PI*2], unit radian
    *   2) Quad-axis Lock: No Parameters
    * @param[out] ec Error code
    */
   void setAvoidSingularity(AvoidSingularityMethod method, bool enable, double limit, error_code &ec) noexcept;

   /**
    * @brief Check whether it is in a state of avoiding singularities
    * @param[in] method The way of strange avoidance
    * @param[out] ec Error code
    * @return true - Already opened
    */
   bool getAvoidSingularity(AvoidSingularityMethod method, error_code &ec) noexcept;
 };

 /**
  * @class xMateCr5Robot
  * @brief 5Axis collaborative robots, including XMC17_5/XMC25_5
  */
 class XCORE_API xMateCr5Robot : public Cobot<5> {
  public:
   /**
	* @brief default constructor
	*/
   xMateCr5Robot() = default;

   /**
	* @brief Create a robot instance and connect
	* @param remoteIP Robot IP address
	* @param localIP Local machine address, needs to be set when sending and receiving data in real time
	* @throw NetworkException Network connection error
	* @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
	*/
   explicit xMateCr5Robot(const std::string &remoteIP, const std::string& localIP = "");

 };

 /**
  * @class xMateErProRobot
  * @brief 7Axis collaborative robots, including xMateER3 Pro / xMateER7 Pro
  */
 class XCORE_API xMateErProRobot : public Cobot<7> {
  public:
   /**
    * @brief default constructor
    */
   xMateErProRobot() = default;

   /**
    * @brief Create a collaborative 7-axis robot instance and connect
    * @param remoteIP Robot IP address
    * @param localIP Local machine address, needs to be set when sending and receiving data in real time
    * @throw NetworkException Network connection error
    * @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
    */
   explicit xMateErProRobot(const std::string &remoteIP, const std::string& localIP = "");
 };

 /**
  * @class StandardRobot
  * @brief Standard Industrial 6-Axis Model
  */
 class XCORE_API StandardRobot: public IndustrialRobot<6> {
  public:

   /**
    * @brief Default constructor
    */
   StandardRobot();

   /**
     * @brief Industrial 6-axis robot
     * @param[in] remoteIP Robot IP address
     * @param[in] localIP Local machine address. Used for sending and receiving interactive data in real-time mode, and needs to be set when using real-time motion control mode.。
     * @throw NetworkException Network connection error
     * @throw ExecutionException The robot instance does not match the connected model, or is not authorizedSDK
     */
   explicit StandardRobot(const std::string& remoteIP, const std::string& localIP = "");

   // *********************************************************************
   // ******************            Collision Detection            *********************

   /**
    * @brief Set parameters related to collision detection and enable the collision detection function. Industrial models only support stop1 (safety stop)）
    * @param[in] sensitivity Collision detection sensitivity, range0.01-2.0
    * @param[in] fallback Retreat distance after collision, unit: meters
    * @param[out] ec Error code
    */
   void enableCollisionDetection(const std::array<double, 6> &sensitivity, double fallback, error_code &ec) noexcept;

   /**
    * @brief Turn off collision detection
    * @param[out] ec Error code
    */
   void disableCollisionDetection(error_code &ec) noexcept;

   /**
    * @brief Create real-time motion control class(RtMotionControlIndustrial)Instance, perform real-time mode related operations through this instance pointer。
    * @note Unless this interface is called repeatedly, the client’s internal logic will not actively destruct the returned object.，
    * Including but not limited to disconnecting and connecting to robotsdisconnectFromRobot()，Switching to non-real-time motion control mode and so on, but performing real-time mode control after doing the above operations will cause abnormalities。
    * @return Controller object
    * @throw RealtimeControlException Failed to create RtMotionControl instance due to network issues
    * @throw ExecutionException Did not switch to real-time motion control mode
    */
   std::weak_ptr<RtMotionControlIndustrial<6>> getRtMotionController();

   /**
    * @brief Set the network latency threshold for sending real-time motion commands。
    * Please make the settings before switching to RtCommand mode, otherwise they will not take effect.。
    * @param[in] percent Permissible range0 - 100
    * @param[out] ec Error code
    */
   void setRtNetworkTolerance(unsigned percent, error_code &ec) noexcept;

   /**
    * @brief Turn on/off the singularity avoidance function
    * @param[in] method Unusual avoidance methods, all three methods are supported
    * @param[in] enable true - Turn on the feature | false - Close;
    * For the four-axis locking method, make sure the four axes are at zero position before opening.
    * @param[in] threshold Different avoidance methods, the meaning of this parameter is respectively:
    *   1) Sacrifice posture: Allowed posture error, range (0, PI*2], unit radian
    *   2) Axis space interpolation: avoidance radius, range[0.005, 10], unit meter
    *   3) Quad-axis Lock: No Parameters
    * @param[out] ec Error code
    */
   void setAvoidSingularity(AvoidSingularityMethod method, bool enable, double threshold, error_code &ec) noexcept;

   /**
    * @brief Check whether it is in a state of avoiding singularities
    * @param[in] method The way of strange avoidance
    * @param[out] ec Error code
    * @return true - Already opened
    */
   bool getAvoidSingularity(AvoidSingularityMethod method, error_code &ec) noexcept;

  XCORESDK_DECLARE_IMPLD
 };

 /**
  * @class PCB3Robot
  * @brief PCB3Axle Model
  */
 class XCORE_API PCB3Robot: public IndustrialRobot<3> {
  public:
   /**
    * @brief Default constructor
    */
   PCB3Robot() = default;

   /**
    * @brief Create a PCB3 robot instance and connect
    * @param remoteIP Robot IP address
    * @throw NetworkException Network connection error
    */
   explicit PCB3Robot(const std::string &remoteIP);
 };

 /**
  * @class PCB4Robot
  * @brief PCB4Axle Model
  */
 class XCORE_API PCB4Robot: public IndustrialRobot<4> {
  public:
   /**
    * @brief Default constructor
    */
   PCB4Robot() = default;

   /**
    * @brief Create a PCB4 robot instance and connect
    * @param remoteIP Robot IP address
    */
   explicit PCB4Robot(const std::string &remoteIP);
 };

 /// @cond DO_NOT_DOCUMENT
 // backward-compatible, using alias
 using XMateRobot = xMateRobot;
 using XMateErProRobot = xMateErProRobot;
 /// @endcond

}  // namespace rokae

#endif // ROKAEAPI_ROBOT_H_
