/**
 * @file data_types.h
 * @brief Define data structures and enum classes
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_
#define ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <array>
#include <vector>
#include <string>
#include <any>
#include <unordered_map>
#include "base.h"

namespace rokae {

 /// @cond DO_NOT_DOCUMENT
 const int USE_DEFAULT = -1;
 const int Unknown = -1;
 /// @endcond

// *********************         Enum class          **********************
// *********************           Enum class             **********************

 /**
  * @enum OperationState
  * @brief Robot operating status
  */
  enum class OperationState {
    idle             = 0, ///< Robot stationary
    jog              = 1, ///< jogState(Not exercised)
    rtControlling    = 2, ///< In real-time mode control
    drag             = 3, ///< Drag Enabled
    rlProgram        = 4, ///< RLDuring project operation
    demo             = 5, ///< DemoIn demonstration
    dynamicIdentify  = 6, ///< In dynamic identification
    frictionIdentify = 7, ///< In the process of identifying friction
    loadIdentify     = 8, ///< Load identification in progress
    moving           = 9, ///< Robot in motion
    jogging          = 10, ///< JogIn motion
    unknown          = Unknown ///< Unknown
  };

 /**
  * @enum WorkType
  * @brief Aircraft Type
  */
 enum class WorkType {
   industrial,   ///< Industrial robot
   collaborative ///< Collaborative robot
 };

 /**
  * @enum OperateMode
  * @brief Robot operating mode
  */
 enum class OperateMode {
   manual    = 0,      ///< Manual
   automatic = 1,      ///< Automatic
   unknown   = Unknown ///< Unknown(An exception occurred)
 };

 /**
  * @enum PowerState
  * @brief Robot power on/off and emergency stop status
  */
 enum class PowerState {
   on      = 0, ///< Power on
   off     = 1, ///< Power off
   estop   = 2, ///< The emergency stop has been pressed
   gstop   = 3, ///< The safety door is open
   unknown = Unknown ///< Unknown(An exception occurred)
 };

 /**
  * @brief Pose coordinate system type
  */
 enum class CoordinateType {
   flangeInBase, ///< Flange relative to the base coordinate system
   endInRef      ///< The end relative to the external coordinate system
 };

 /**
  * @enum MotionControlMode
  * @brief SDKMotion control mode
  */
 enum class MotionControlMode : unsigned {
   Idle,       ///< Free
   NrtCommand, ///< Execute motion commands in non-real-time mode
   NrtRLTask,  ///< Run RL project in non-real-time mode
   RtCommand,  ///< Real-time mode control
 };

 /**
  * @enum RtControllerMode
  * @brief Controller real-time control mode
  */
 enum class RtControllerMode : unsigned {
   jointPosition,      ///< Real-time spatial position control
   cartesianPosition,  ///< Real-time Cartesian space position control
   jointImpedance,     ///< Real-time Cartesian Impedance Control
   cartesianImpedance, ///< Real-time Cartesian Space Impedance Control
   torque              ///< Real-time torque control
 };

 namespace RtSupportedFields {
  /// Note: The data type follows the data name
  /// ArrayXD = std::array<double, DoF> , DoFNumber of axes
  /// Array6D = std::array<double, 6>, of this type
  constexpr const char *jointPos_m = "q_m";   ///< Joint angle [rad] - ArrayXD
  constexpr const char *jointPos_c = "q_c";   ///< Command Joint Angle [rad] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *jointVel_m = "dq_m";  ///< Joint speed [rad/s]- ArrayXD
  constexpr const char *jointVel_c = "dq_c";  ///< Command Joint Velocity [rad/s] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *jointAcc_m = "ddq_m"; ///< Joint acceleration [rad/s^2] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *jointAcc_c = "ddq_c"; ///< Command joint acceleration [rad/s^2] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *tcpPose_m  = "pos_m"; ///< End pose, relative to the base coordinate system, row-priority homogeneous transformation matrix - Array16D
  constexpr const char *tcpPoseAbc_m = "pos_abc_m"; ///< End pose, relative to the base coordinate system [X,Y,Z,Rx,Ry,Rz] - Array6D
  constexpr const char *tcpPose_c  = "pos_c"; ///< The sent end pose command is relative to the base coordinate system, row-major homogeneous transformation matrix - Array16D. The data is only valid after real-time mode control is enabled.。
  constexpr const char *tcpVel_m   = "pos_vel_m"; ///< Robot end speed - Array6D. Data is valid only after enabling real-time mode control。
  constexpr const char *tcpVel_c   = "pos_vel_c"; ///< Instruction robot end-effector velocity - Array6D. Data is only valid after enabling real-time mode control。
  constexpr const char *tcpAcc_m   = "pos_acc_m"; ///< Robot end-effector acceleration - Array6D. Data is valid only after real-time mode control is enabled。
  constexpr const char *tcpAcc_c   = "pos_acc_c"; ///< Command robot end acceleration - Array6D. Data is only valid after enabling real-time mode control。
  constexpr const char *exJointPos_m = "ex_q_m"; ///< External shaft value [rad] Guide rail[m] - Array6D The actual number of valid data is equal to the number of external axes
  constexpr const char *exJointVel_m = "ex_dq_m"; ///< External shaft speed [rad/s] Guide rail[m/s] - Array6D The actual number of valid data is equal to the number of external axes
  constexpr const char *exMotor_m = "ex_motor_m"; ///< External axis motor position - The actual number of valid data in Array6D is equal to the number of external axes
  constexpr const char *elbow_m    = "psi_m";     ///< Arm angle [rad] - double
  constexpr const char *elbow_c    = "psi_c";     ///< Command arm angle [rad] - double。Data is only valid after turning on real-time mode control。
  constexpr const char *elbowVel_c = "psi_vel_c"; ///< Command arm angular velocity [rad/s] - double。Data is only valid after turning on real-time mode control。
  constexpr const char *elbowAcc_c = "psi_acc_c"; ///< Command arm angular acceleration [rad/s] - double。Data is only valid after turning on real-time mode control。
  constexpr const char *tau_m      = "tau_m";     ///< Joint torque [Nm] - ArrayXD
  constexpr const char *tau_c      = "tau_c";     ///< Command Joint Torque [Nm] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *tauFiltered_m    = "tau_filtered_m"; ///< Filtered joint torque [Nm] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *tauVel_c         = "tau_vel_c";      ///< Command Torque Differential [Nm/s] - ArrayXD。Data is only valid after turning on real-time mode control。
  constexpr const char *tauExt_inBase    = "tau_ext_base";   ///< External torque in the base coordinate system [Nm] - Array6D。Data is only valid after turning on real-time mode control。
  constexpr const char *tauExt_inStiff   = "tau_ext_stiff";  ///< External Torque in the Force-Control Coordinate System [Nm] - Array6D。Data is only valid after turning on real-time mode control。
  constexpr const char *theta_m          = "theta_m";        ///< Motor position - ArrayXD
  constexpr const char *thetaVel_m       = "theta_vel_m";        ///< Motor position differentiation - ArrayXD
  constexpr const char *motorTau         = "motor_tau";          ///< Motor torque - ArrayXD
  constexpr const char *motorTauFiltered = "motor_tau_filtered"; ///< Filtered motor torque - ArrayXD. Data is only valid after enabling real-time mode control.。
  constexpr const char *keypads  = "io_keypad";    ///< End key status - ArrayXD
 }

 /**
  * @enum StopLevel
  * @brief Robot Stop Motion Level
  */
 enum class StopLevel {
   stop0, ///< Power off after quickly stopping the robot's movement
   stop1, ///< Plan to cut off power after the robot stops moving, stopping on the original path
   stop2,  ///< Plan to stop the robot's movement without cutting off power, stopping on the original path
   suppleStop ///< Soft stop, only applicable to collaborative models
 };

 /**
  * @struct DragParameter
  * @brief Robot drag mode parameters, including drag type and space
  */
 struct DragParameter {
   /**
    * @brief Drag space
    */
   enum Space {
     jointSpace     = 0, ///< Axial space
     cartesianSpace = 1  ///< Cartesian space
   };
   /**
    * @brief Drag Type
    */
   enum Type {
     translationOnly = 0, ///< Translate only
     rotationOnly    = 1, ///< Rotate only
     freely          = 2  ///< Free drag
   };
 };

 /**
  * @enum FrameType
  * @brief Coordinate System Type
  */
 enum class FrameType {
   world  = 0, ///< World Coordinate System
   base   = 1, ///< Base coordinate system
   flange = 2, ///< Flange coordinate system
   tool   = 3, ///< Tool Coordinate System
   wobj   = 4, ///< Workpiece coordinate system
   path   = 5, ///< Path Coordinate System
   rail   = 6  ///< Guide Rail Base Coordinate System
 };

 /**
  * @struct JogOpt
  * @brief JogOption: Coordinate System
  */
 struct JogOpt {
   /**
    * @brief JogCoordinate system
    */
   enum Space {
     world = 0, ///< World Coordinate System
     flange, ///< Flange coordinate system
     baseFrame, ///< Base coordinate system
     toolFrame, ///< Tool Coordinate System
     wobjFrame, ///< Workpiece coordinate system
     jointSpace, ///< Axial space
     singularityAvoidMode, ///< Singular avoidance mode, suitable for industrial six-axis, xMateCR, and xMateSR models, the avoidance method is locking the fourth axis
     baseParallelMode ///< Parallel base mode, only applicable to xMateCR and xMateSR models
   };
 };

 /**
  * @struct xPanelOpt
  * @brief xPanelConfiguration: External Power Supply Mode
  */
 struct xPanelOpt {
   /**
    * @brief Power supply mode
    */
   enum Vout {
     off,       ///< Do not output
     reserve,   ///< Reserve
     supply12v, ///< Output12V
     supply24v, ///< Output24V
   };
 };

 /**
  * @brief Strange Avoidance Method
  */
 enum class AvoidSingularityMethod {
   lockAxis4, ///< Four-axis locking
   wrist,     ///< sacrificial posture
   jointWay   ///< Axis space short trajectory interpolation
 };

 /**
  * @brief Event Information - map type
  */
 typedef std::unordered_map<std::string, std::any> EventInfo;
 /**
  * @brief Event callback function type
  */
 typedef std::function<void(const EventInfo &)> EventCallback;

 /**
  * @brief Event Type
  */
 enum class Event {
   moveExecution, ///< Non-real-time motion command execution information
   safety,        ///< Safety (Is there a collision)
   rlExecution,   ///< RLExecution Status
   logReporter    ///< Controller log reporting
 };

 /**
  * @brief Event Information Field
  */
 namespace EventInfoKey {
  /**
   * Non-real-time motion command execution information
   */
  namespace MoveExecution {
   constexpr const char *ID = "cmdID";     ///< Path ID, corresponding callmoveAppend()the second parameter; Typestring
   constexpr const char *ReachTarget = "reachTarget"; ///< Has the trajectory reached the target point?; Typebool
   constexpr const char *WaypointIndex = "wayPointIndex"; ///< The index of the currently executing trajectory target point, starting from 0; Typeint
   constexpr const char *Error = "error"; ///< Error code, error before or during execution of motion command; Typeerror_code
   constexpr const char *Remark = "remark"; ///< Other execution information, currently including alert information for targets that are too close; Typestring
   constexpr const char *CustomInfo = "customInfo"; ///< User-defined information, corresponding toNrtCommand::customInfo; Typestring
  }
  /**
   * @brief Safety-related
   */
  namespace Safety {
   constexpr const char *Collided = "collided"; ///< Is there a collision; Type bool, true - collision occurred | false-Not occurred or restored
  }

  /**
   * @brief RLProgram execution status
   */
  namespace RlExecution {
   constexpr const char *TaskName = "taskName"; ///< Name of the task being executed; Typestring
   constexpr const char *LookaheadLine = "lookaheadLine"; ///< Forward-looking Bank Code; Typeint
   constexpr const char *LookaheadFile = "lookaheadFile"; ///< Previewed file name; Typestring
   constexpr const char *ExecuteLine = "executeLine"; ///< Execution line number; Typeint
   constexpr const char *ExecuteFile = "executeFile"; ///< The name of the file being executed; Typestring
  }

  /**
   * @brief Controller log reporting
   */
  namespace LogReporter {
   constexpr const char* Ecode = "ecode"; ///< Controller Log Error Code; Typeint
   constexpr const char* Edetail = "edetail"; ///< Controller log error information; Typestring
  }

 }

// *******************          Data types            ********************
// *******************           Data Structure              ********************
#if defined(XCORESDK_SUPPRESS_DLL_WARNING)
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

 /**
  * @class Frame
  * @brief Coordinate system
  */
 class XCORE_API Frame {
  public:
   /**
    * @brief default constructor
    */
   Frame() = default;

   /**
    * @brief Initializationtrans & rpy
    * @param trans Translation amount
    * @param rpy Euler anglesXYZ
    */
   Frame(const std::array<double, 3> &trans, const std::array<double, 3> &rpy);

   /**
    * @brief Initializationtrans & rpy
    * @param frame [X, Y, Z, Rx, Ry, Rz]
    */
   Frame(const std::array<double, 6> &frame);

   /**
    * @brief Initializationpos
    * @param matrix 4*4Transformation Matrix
    */
   Frame(const std::array<double,16> &matrix);

   /**
    * @brief Initialization
    * @param values Initialize when the length is 6trans & rot = [X, Y, Z, Rx, Ry, Rz];
    *               Initialize when the length is 16pos
    * @throw ArgumentException Initialization list length error
    */
   Frame(std::initializer_list<double> values);

   std::array<double, 3> trans {}; ///< Translation amount [X, Y, Z], Unit: meter
   std::array<double, 3> rpy {};   ///< Euler angles [Rx, Ry, Rz], Unit: Radian
   std::array<double, 16> pos {};  ///< Row-priority homogeneous transformation matrix. Only used for real-time mode Cartesian position/impedance control.。
 };

 /**
  * @class Finishable
  * @brief Has one exercise cycle ended?
  */
 class XCORE_API Finishable {
  public:
   /**
    * @brief Has the exercise loop ended?
    */
   uint8_t isFinished() const;

   /**
    * @brief Indicates that the exercise cycle has ended
    */
   void setFinished();

  protected:
   uint8_t finished { 0 }; ///< Used to determine whether to end a movement loop
 };

 /**
  * @class CartesianPosition
  * @brief Descartes point
  */
 class XCORE_API CartesianPosition : public Frame, public Finishable {
  public:
   using Frame::Frame;
   /**
    * @brief Offset
    */
   struct Offset {
     /**
      * @brief Offset type
      */
     enum Type {
       none,   ///< No offset
       offs,   ///< Offset Relative to Workpiece Coordinate System
       relTool ///< Offset Relative to Tool Coordinate System
     };

     /**
      * @brief default constructor
      */
     Offset() = default;

     /**
      * @brief constructor
      */
     Offset(Type type, const Frame &frame);

     Type type { none }; ///< Offset type
     Frame frame { };    ///< Offset relative to the specified tool/workpiece coordinate system
   };

   double elbow { 0 };      ///< Arm angle, suitable for 7-axis robots, unit: radians
   bool hasElbow { false }; ///< Is there a humeral angle?
   std::vector<int> confData; ///< Axle configuration data, length is8: [cf1, cf2, cf3, cf4, cf5, cf6, cf7, cfx]
   std::vector<double> external; ///< External joint values Unit: radians|Meter. Rail unit: meter
 };

 /**
  * @class JointPosition
  * @brief Joint position
  */
 class XCORE_API JointPosition : public Finishable {
  public:
   /**
    * @brief default constructor
    */
   JointPosition() = default;
   /**
    * @param joints The length should match the number of robot axes. External joints can be optional.
    */
   JointPosition(std::initializer_list<double> joints);

   /**
    * @brief constructor
    * @param joints Shaft angle
    */
   JointPosition(std::vector<double> joints);

   /**
    * @brief Initializationjoints
    * @param n The length should match the number of axes of the machine model
    * @param v Initial value
    */
   JointPosition(size_t n, double v = 0);

   std::vector<double> joints; ///< Joint angle value, unit: radians
   std::vector<double> external; ///< External joint values, unit: radian|Meter. Rail unit: meter
 };

 /**
  * @class Torque
  * @brief Joint torque, excluding gravity and friction
  */
 class XCORE_API Torque : public Finishable {
  public:
   Torque() = default;

   /**
    * @brief constructor
    * @param tau Torque command value
    */
   Torque(std::vector<double> tau);

   /**
    * @brief constructor
    * @param tau Torque command value
    */
   Torque(std::initializer_list<double> tau);

   /**
    * @brief Initializationtau
    * @param n The length should match the number of axes of the machine model
    * @param v Initial value
    */
   Torque(size_t n, double v = 0);

   std::vector<double> tau; ///< Expected joint torque, unit: Nm
 };

 /**
   * @class Load
   * @brief Load Information
   */
 class XCORE_API Load {
  public:
   Load() = default;
   /**
    * @param m Quality
    * @param cog center of mass
    * @param inertia Inertia
    */
   Load(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   double mass { 0 };  ///< Load mass, unit: kilogram
   std::array<double, 3> cog {};     ///< center of mass [x, y, z], Unit: meter
   std::array<double, 3> inertia {}; ///< Inertia [ix, iy, iz], Unit: kilogram·square meter
 };

 /**
  * @class Toolset
  * @brief Tool-workpiece group information, calculated based on the coordinates, load, and robot handheld settings of a pair of tools and workpieces
  * @note Does not explicitly distinguish between handheld/external. This can be understood as follows: for handheld tools, the load and the robot's end coordinate system belong to the tool, while the reference coordinate system belongs to the workpiece.；
  *       Conversely, if holding the workpiece, the load and end coordinate system come from the workpiece, and the reference coordinate system comes from the tool.
  */
 class XCORE_API Toolset {
  public:
   Toolset() = default;
   /**
    * @param load Load Information
    * @param end End coordinate system
    * @param ref Reference coordinate system
    */
   Toolset(const Load &load, const Frame &end, const Frame &ref);

   Load load {}; ///< Robot end-effector payload
   Frame end {}; ///< Transformation from the robot end-effector coordinate system to the flange coordinate system
   Frame ref {}; ///< Coordinate transformation from robot reference frame to world coordinate system
 };

 /**
  * @class FrameCalibrationResult
  * @brief Coordinate System Calibration Results
  */
 class XCORE_API FrameCalibrationResult {
  public:
   FrameCalibrationResult() = default;
   Frame frame {};  ///< Calibration Results
   std::array<double, 3> errors {}; ///< The deviation of sample points from the TCP calibration value, listed as minimum, average, and maximum, unitm
 };

 /**
  * @class RLProjectInfo
  * @brief RLProject Information
  */
 class XCORE_API RLProjectInfo {
  public:
   /**
    * @brief constructor
    * @param name RLProject Name
    */
   explicit RLProjectInfo(std::string name);

   std::string name; ///< Project Name
   std::vector<std::string> taskList; ///< Task Name List
 };

 /**
  * @class WorkToolInfo
  * @brief Tool/Workpiece information. The coordinate system of the workpiece has been transformed relative to its user coordinate system.
  */
 class XCORE_API WorkToolInfo {
  public:
   WorkToolInfo() = default;

   /**
    * @brief constructor
    * @param name Name
    * @param isHeld Is it handheld by a robot?
    * @param posture Pose
    * @param load Load
    */
   WorkToolInfo(std::string name, bool isHeld, const Frame &posture, const Load &load);

   std::string name {};  ///< Name
   std::string alias {}; ///< Description
   bool robotHeld {};    ///< Is it handheld by a robot?
   Frame pos {};         ///< Pose
   Load load {};         ///< Load
 };

 /**
  * @class NrtCommand
  * @brief Non-real-time motion command
  */
 class XCORE_API NrtCommand {
  public:

   /**
    * @brief Maximum linear velocity of the robot end, unitmm/s
    * @see setDefaultSpeed()
    */
   double speed { USE_DEFAULT };

   /**
    * @brief Turning area radius size, unitmm
    * @see setDefaultZone()
    */
   double zone { USE_DEFAULT };

   /**
    * @brief Custom information, can be returned in the exercise information feedback
    */
   std::string customInfo {};

   NrtCommand() = default;

   /**
    * @brief constructor
    * @param speed The speed of this instruction, in unitsmm/s
    * @param zone Turning area of this instruction, unitmm
    */
   NrtCommand(double speed, double zone);

   virtual ~NrtCommand() = default;
 };

 /**
  * @class MoveAbsJCommand
  * @brief Motion Command - Axis MovementMoveAbsJ
  */
 class XCORE_API MoveAbsJCommand : public NrtCommand{
  public:
   /**
    * @param target Target shaft angle
    * @param speed End effector linear speed, unit mm/s, joint speeds are divided into several intervals based on the magnitude of the end effector linear speed, see detailssetDefaultSpeed()
    * @param zone Turning area, unitmm
    */
   MoveAbsJCommand(JointPosition target, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   JointPosition target; ///< Target checkpoint position

   double jointSpeed { USE_DEFAULT }; ///< Joint speed percentage, range[0, 1]。Effective when greater than or equal to 0; when less than 0, the joint speed calculated by speed is still used
 };

 /**
  * @brief Motion dwell command. Can be inserted between two motion commands; after the previous motion has reached its position, it waits for a period of time before executing the next one.。
  * There will be no information feedback after this command is executed.
  */
 class XCORE_API MoveWaitCommand : public NrtCommand {
  public:

   /**
    * @brief Constructor
    * @param duration Duration
    */
   MoveWaitCommand(std::chrono::steady_clock::duration duration);

   std::chrono::steady_clock::duration duration_; ///< Duration of stay, minimum effective duration1ms
 };

 /**
  * @class MoveJCommand
  * @brief Motion Command - Axis MovementMoveJ
  */
 class XCORE_API MoveJCommand : public NrtCommand {
  public:
   /**
    * @param target Target Cartesian Point
    * @param speed End effector linear speed, unit mm/s, joint speeds are divided into several intervals based on the magnitude of the end effector linear speed, see detailssetDefaultSpeed()
    * @param zone Turning area, unitmm
    */
   MoveJCommand(CartesianPosition target, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   CartesianPosition target; ///< Target Cartesian Point
   CartesianPosition::Offset offset; ///< Offset Options

   double jointSpeed { USE_DEFAULT }; ///< Joint speed percentage, range[0, 1]。Effective when greater than or equal to 0; when less than 0, the joint speed calculated by speed is still used
 };

 /**
  * @class MoveLCommand
  * @brief Motion Command - End-Effector Linear TrajectoryMoveL
  */
 class XCORE_API MoveLCommand : public NrtCommand {
  public:
   /**
    * @param target Target Cartesian Point
    * @param speed End point linear velocity, unitmm/s
    * @param zone Turning area, unitmm
    */
   MoveLCommand(CartesianPosition target, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   CartesianPosition target; ///< Target Cartesian Point
   CartesianPosition::Offset offset; ///< Offset Options

   double rotSpeed { USE_DEFAULT }; ///< Spatial rotation speed, in units of rad/s. Effective when greater than or equal to 0; when less than 0, the rotation speed defaults to200°/s
 };

 /**
  * @class MoveCCommand
  * @brief Motion Command - Arc TrajectoryMoveC
  */
 class XCORE_API MoveCCommand : public NrtCommand {
  public:
   /**
    * @param target Target point
    * @param aux Reference point
    * @param speed End effector linear speed, unit mm/s, joint speeds are divided into several intervals based on the magnitude of the end effector linear speed, see detailssetDefaultSpeed()
    * @param zone Turning area, unitmm
    */
   MoveCCommand(CartesianPosition target, CartesianPosition aux, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   CartesianPosition target; ///< Target Cartesian Point
   CartesianPosition::Offset targetOffset; ///< Offset Options
   CartesianPosition aux;    ///< Auxiliary Point
   CartesianPosition::Offset auxOffset; ///< Offset Options

   double rotSpeed { USE_DEFAULT }; ///< Spatial rotation speed, in units of rad/s. Effective when greater than or equal to 0; when less than 0, the rotation speed defaults to200°/s
 };

 /**
  * @brief Motion Command - Full Circle TrajectoryMoveCF
  */
 class XCORE_API MoveCFCommand : public MoveCCommand{
  public:
   /**
    * @brief Full-sphere attitude rotation type
    */
   enum RotType {
     constPose, ///< Unchanging posture
     rotAxis,   ///< Rotating shaft
     fixedAxis  ///< Fixed-axis rotation
   };

   /**
    * @param target Target point
    * @param aux Reference point
    * @param speed End point linear velocity, unitmm/s
    * @param zone Turning area, unitmm
    * @param angle Execution angle, unit: radian
    */
   MoveCFCommand(const CartesianPosition &target, const CartesianPosition &aux, double angle, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   double angle { 0 }; ///< Full circle execution angle, unit: radian
   RotType rotType { constPose }; ///< Full-circle attitude rotation mode
 };

 /**
  * @brief Motion Command - Spiral TrajectoryMoveSP
  */
 class XCORE_API MoveSPCommand : public NrtCommand {
  public:
   /**
    * @param target Final posture
    * @param r0 Initial radius [m]
    * @param rStep Change in radius per unit angle of rotation [m/rad]
    * @param angle Total rotation angle [rad]
    * @param dir Rotation direction, true - clockwise | false - anticlockwise
    * @param speed End point linear velocity, unitmm/s
    */
   MoveSPCommand(const CartesianPosition &target, double r0, double rStep, double angle, bool dir, double speed = USE_DEFAULT);

   CartesianPosition target; ///< End Cartesian point, use only the point's RPY to specify the end pose
   CartesianPosition::Offset targetOffset; ///< Offset Options
   double radius { 0 };      ///< Initial radius, unit: meters
   double radius_step { 0 }; ///< Change in radius per unit angle of rotation, unit: meters/radian
   double angle { 0 };       ///< Total rotation angle, unit: radian
   bool direction;           ///< Rotation direction, true - clockwise | false - counterclockwise

   double rotSpeed { USE_DEFAULT }; ///< Spatial rotation speed, in units of rad/s. Effective when greater than or equal to 0; when less than 0, the rotation speed defaults to200°/s
 };

 /**
  * @class LogInfo
  * @brief Controller log information
  */
 class XCORE_API LogInfo {
  public:
   /**
    * @brief Log Level
    */
   enum Level {
     info,    ///< Notice
     warning, ///< Warning
     error    ///< Error
   };

   /**
    * @brief constructor
    * @param id Log ID
    * @param ts Date and Time
    * @param ct Content
    * @param r Fixing method
    */
   LogInfo(int id, std::string ts, std::string ct, std::string r);

   const int id;                ///< Log ID
   const std::string timestamp; ///< Date and Time
   const std::string content;   ///< Log content
   const std::string repair;    ///< Fixing method
 };

 /**
  * @brief End key status
  */
 struct KeyPadState {
   bool key1_state = false; ///< CR1number
   bool key2_state = false; ///< CR2number
   bool key3_state = false; ///< CR3number
   bool key4_state = false; ///< CR4number
   bool key5_state = false; ///< CR5number
   bool key6_state = false; ///< CR6number
   bool key7_state = false; ///< CR7number
 };

#if defined(XCORESDK_SUPPRESS_DLL_WARNING)
#pragma warning(pop)
#endif // if defined(XCORESDK_SUPPRESS_DLL_WARNING)

}  // namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_
