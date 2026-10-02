/**
 * @file force_control.h
 * @brief Non-real-time mode force control command
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef XCORESDK_INCLUDE_ROKAE_FORCE_CONTROL_H_
#define XCORESDK_INCLUDE_ROKAE_FORCE_CONTROL_H_

#include "base.h"
#include "data_types.h"

namespace rokae {

 // forward declarations
 class XService;

 /**
  * @class BaseForceControl
  * @brief Force Control Command Type
  */
 class XCORE_API BaseForceControl {

  public:

   /**
    * @brief Force Control Initialization
    * @param[in] frame_type Force control coordinate system, supports world/wobj/tool/base/flange. Tool and workpiece coordinate systems use it.setToolset()Set coordinate system
    * @param[out] ec Error code
    */
   void fcInit(FrameType frame_type, error_code &ec) noexcept;

   /**
    * @brief Start force control，fcInit()Call later。
    * If you need to execute motion commands in force control mode，fcStart()Can be executed afterwards。
    * Note, if infcStart()Previously passedmoveAppend()The movement commands have been issued but the movement has not started; after fcStart, these movement commands will be executed.。
    * @param[out] ec Error code
    */
   void fcStart(error_code &ec) noexcept;

   /**
    * @brief Stop Force Control
    * @param[out] ec Error code
    */
   void fcStop(error_code &ec) noexcept;

   /**
    * @brief Set impedance control type
    * @param[in] type 0 - Joint impedance | 1 - Descartes impedance
    * @param[out] ec Error code
    */
   void setControlType(int type, error_code &ec) noexcept;

   /**
    * @brief Set the load information used by the force control module，fcStart()Can be called later。
    * @param[in] load Load
    * @param[out] ec Error code
    */
   void setLoad(const Load &load, error_code &ec) noexcept;

   /**
    * @brief Set Cartesian impedance stiffness。fcInit()Effective after the call
    * The maximum stiffness varies for different models. Please refer to the description of the SetCartCtrlStiffVec command in the 'xCore Control System Manual'.
    * @param[in] stiffness In order: impedance force stiffness in X, Y, Z directions[N/m], X Y ZDirectional Impedance Torque Stiffness[Nm/rad]
    * @param[out] ec Error code
    */
   void setCartesianStiffness(const std::array<double, 6> &stiffness, error_code &ec) noexcept;

   /**
    * @brief Set Cartesian null-space impedance stiffness。fcInit()Effective after the call
    * @param[in] stiffness Scope[0,4], Values greater than 4 will be set to 4 by default, unitNm/rad
    * @param[out] ec Error code
    */
   void setCartesianNullspaceStiffness(double stiffness, error_code &ec) noexcept;

   /**
    * @brief Set Cartesian desired force/torque。fcStart()Can be called later
    * @param[in] value In order: Cartesian desired force in X, Y, Z directions, range[-60,60], unitN; X Y ZDirectional Cartesian expected torque, range[-10,10], unitNm
    * @param[out] ec Error code
    */
   void setCartesianDesiredForce(const std::array<double, 6> &value, error_code &ec) noexcept;

   /**
    * @brief Set up a sinusoidal search motion around a single-axis rotation。
    * Set the impedance control type to Cartesian impedance(namelysetControlType(1))afterwards, startOverlay()Previously called and effective。
    * The maximum amplitude and maximum frequency of the search motion vary for each model. Please refer to the description of the SetSineOverlay command in the xCore Control System Manual.。
    * @param[in] line_dir Search motion reference axis: 0 - X | 1 - Y | 2 - Z
    * @param[in] amplify Search motion amplitude, unitNm
    * @param[in] frequency Search exercise frequency, unitHz
    * @param[in] phase Search motion phase, range[0, PI], unit radian
    * @param[in] bias Search motion bias, range[0, 10], unitNm
    * @param[out] ec Error code
    */
   void setSineOverlay(int line_dir, double amplify, double frequency, double phase,
                       double bias, error_code &ec) noexcept;

   /**
    * @brief Set Lisa in the plane to move as if searching
    * Set the impedance control type to Cartesian impedance(namelysetControlType(1))afterwards, startOverlay()Previously called and effective。
    * @param[in] plane Search motion reference plane: 0 - XY | 1 - XZ | 2 - YZ
    * @param[in] amplify_one Search motion amplitude in one direction, range[0, 20], unitNm
    * @param[in] frequency_one Search motion one-direction frequency, range[0, 5], unitHz
    * @param[in] amplify_two Search motion in the second direction amplitude, range[0, 20]unitNm
    * @param[in] frequency_two Search motion second direction frequency, range[0, 5], unitHz
    * @param[in] phase_diff Phase deviation of search motion in two directions, range[0, PI], unit radian
    * @param[out] ec Error code
    */
   void setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two,
                            double frequency_two, double phase_diff, error_code &ec) noexcept;

   /**
    * @brief Start the search operation。fcStart()Effective after the call
    * Search movement set for the preceding sequence setSineOverlay()or setLissajousOverlay()superposition
    * @param[out] ec Error code
    */
   void startOverlay(error_code &ec) noexcept;

   /**
    * @brief Stop searching for movement
    * @param[out] ec Error code
    */
   void stopOverlay(error_code &ec) noexcept;

   /**
    * @brief Pause search motion。startOverlay()Effective after the call
    * @param[out] ec Error code
    */
   void pauseOverlay(error_code &ec) noexcept;

   /**
    * @brief Resume the paused search operation。pauseOverlay()Effective after the call。
    * @param[out] ec Error code
    */
   void restartOverlay(error_code &ec) noexcept;

   /**
    * @brief Set termination conditions related to contact force
    * @param[in] range Force limits in all directions { X_min, X_max, Y_min, Y_max, Z_min, Z_max }, unitN。
    * When setting the lower limit, a negative value indicates the maximum in the negative direction; When setting an upper limit, a negative value indicates the minimum value in the negative direction。
    * @param[in] isInside true - Stop waiting when the limit is exceeded; false - Stop waiting when the conditions are met
    * @param[in] timeout Timeout, Range[1, 600], unit: second
    * @param[out] ec Error code
    */
   void setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec) noexcept;

   /**
    * @brief Set the termination conditions related to contact torque
    * @param[in] range Torque limits in all directions { X_min, X_max, Y_min, Y_max, Z_min, Z_max }, unitNm。
    * When setting the lower limit, a negative value indicates the maximum in the negative direction; When setting an upper limit, a negative value indicates the minimum value in the negative direction。
    * @param[in] isInside true - Stop waiting when the limit is exceeded; false - Stop waiting when the conditions are met
    * @param[in] timeout Timeout, Range[1, 600], unit: second
    * @param[out] ec Error code
    */
   void setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec) noexcept;

   /**
    * @brief Set termination conditions related to contact position
    * @param[in] supervising_frame The reference coordinate system in which the cuboid is located, relative to the external workpiece coordinate system。
    * The external workpiece coordinate system is established bysetToolset()set (Toolset::ref)
    * @param[in] box Define a cuboid { X_start, X_end, Y_start, Y_end, Z_start, Z_end }, unit meter
    * @param[in] isInside true - Stop waiting when the limit is exceeded; false - Stop waiting when the conditions are met
    * @param[in] timeout Timeout, Range[1, 600], unit: second
    * @param[out] ec Error code
    */
   void setPoseBoxCondition(const Frame &supervising_frame, const std::array<double, 6> &box, bool isInside,
                           double timeout, error_code &ec) noexcept;

   /**
    * @brief Activate the pre-configured termination conditions and wait until these conditions are met or a timeout occurs
    * @param[out] ec Error code
    */
   void waitCondition(error_code &ec) noexcept;

   /**
    * @brief Enable/Disable Force Control Module Protection Monitoring。
    * After setting the monitoring parameters, they do not take effect immediately; callfcMonitor(true)It takes effect after and continues until calledfcMotion(false)After finishing。
    * After it ends, the protection threshold is restored to the default value, meaning there will still be a protective effect, and after monitoring is turned off, it will no longer be the user-set parameters.。
    * @param[in] enable true - Open | false - Close
    * @param[out] ec Error code
    * @see setCartesianMaxVel() setJointMaxVel() setJointMaxMomentum() setJointMaxEnergy()
    */
   void fcMonitor(bool enable, error_code &ec) noexcept;

   /**
    * @brief Maximum speed of the robotic arm end relative to the base coordinate system in force control mode
    * @param[in] velocity in order：X Y Z [m/s], A B C [rad/s], Scope >=0
    * @param[out] ec Error code
    */
   void setCartesianMaxVel(const std::array<double, 6> &velocity, error_code &ec) noexcept;

   /**
    * @brief Impedance Limiting Command - Cartesian。fcStart()Takes effect when started, invalid after fcStop。
    * Description: In some force-controlled assembly scenarios, motion is performed under impedance control, while also aiming to limit the end-effector force, which also serves as a form of end-effector force monitoring.。
    * @param[out] max_wrench in order：XYZ [N], ABC [Nm], Scope [0, 1000], Default maximum value
    * @param[out] ec Error code
    */
   void setCartesianControlMaxWrench(const std::array<double, 6> &max_wrench, error_code &ec) noexcept;

   /**
    * @brief  Impedance Speed Limiting - Cartesian。fcStart()Takes effect when started, invalid after fcStop。
    * Description: Taking the example of pressing down with an expected force until it contacts this scene, it is generally achieved by setting the expected force. This interface can perform speed-limiting processing for the contact process.。
    * @param[out] max_cart_vel Scope XYZ - [0, 3.0], unitm/s, ABC - [0, 10.0], Unit rad/s, default maximum setting
    * @param[out] ec Error code
    */
   void setCartesianControlMaxVel(const std::array<double, 6> &max_cart_vel, error_code &ec) noexcept;

   /// @cond DO_NOT_DOCUMENT
   explicit BaseForceControl(std::shared_ptr<XService> rpc);
   virtual ~BaseForceControl();

  XCORESDK_DECLARE_IMPL
   /// @endcond
 };

 /**
  * @brief Force Control Command Type
  * @tparam DoF Number of axles
  */
 template<unsigned short DoF>
 class XCORE_API ForceControl_T : public BaseForceControl {
  public:

   using BaseForceControl::BaseForceControl;

   /**
    * @brief Obtain current torque information
    * @param[in] ref_type Reference frame relative to torque：
    *     1) FrameType::world - Moment information of the end relative to the world coordinate system
    *     2) FrameType::flange - Torque information of the end relative to the flange
    *     3) FrameType::tool - Torque information of the end relative to the TCP point
    * @param[out] joint_torque_measured Axial space measurement force information, torque on each axis measured by the force sensor, unitNm
    * @param[out] external_torque_measured External force information in the joint space, torque information on each joint calculated by the controller based on the robot model and measured forces, unitNm
    * @param[out] cart_torque All directions in Cartesian space[X, Y, Z]Torque received, unitNm
    * @param[out] cart_force All directions in Cartesian space[X, Y, Z]Force received, unitN
    * @param[out] ec Error code
    */
   void getEndTorque(FrameType ref_type, std::array<double, DoF> &joint_torque_measured, std::array<double, DoF> &external_torque_measured,
                     std::array<double, 3> &cart_torque, std::array<double, 3> &cart_force, error_code &ec) noexcept;

   /**
    * @brief Set joint impedance stiffness。fcInit()Effective after the call
    * The maximum stiffness varies for each model. Please refer to the instructions of the SetJntCtrlStiffVec command in the 'xCore Control System Manual'.
    * @param[in] stiffness Stiffness of each axis
    * @param[out] ec Error code
    */
   void setJointStiffness(const std::array<double, DoF> &stiffness, error_code &ec) noexcept;

   /**
    * @brief Set joint desired torque。fcStart()Can be called later
    * @param[in] torque Torque value, range[-30,30], unitNm
    * @param[out] ec Error code
    */
   void setJointDesiredTorque(const std::array<double, DoF> &torque, error_code &ec) noexcept;

   /**
    * @brief Set the maximum speed of the axis in force control mode
    * @param[in] velocity Shaft speed [rad/s]，Scope >=0
    * @param[out] ec Error code
    */
   void setJointMaxVel(const std::array<double, DoF> &velocity, error_code &ec) noexcept;

   /**
    * @brief Set the maximum axis momentum in force control mode。
    * Calculation method：F*t，It can be understood as impulse, where F is the reading from the torque sensor, and t is the control cycle. If the momentum threshold is exceeded for more than 30 cycles, protection is triggered.
    * @param[in] momentum Momentum [N·s]，Scope >=0
    * @param[out] ec Error code
    */
   void setJointMaxMomentum(const std::array<double, DoF> &momentum, error_code &ec) noexcept;

   /**
    * @brief Set the maximum kinetic energy of the axis in force control mode。
    * Calculation method：F*v，It can be understood as power, F is the torque sensor reading, v is the joint speed, and if it exceeds the kinetic energy threshold for more than 30 cycles, protection is triggered.
    * @param[in] energy Kinetic energy [N·rad/s]，Scope >=0
    * @param[out] ec Error code
    */
   void setJointMaxEnergy(const std::array<double, DoF> &energy, error_code &ec) noexcept;

   /**
    * @brief Impedance Limit Command - Joint。fcStart()Takes effect when started, invalid after fcStop。
    * Description: In some force-controlled assembly scenarios, motion is performed under impedance control, while also aiming to limit the end-effector force, which also serves as a form of end-effector force monitoring.。
    * @param[in] max_torque Force limiting amplitude, unit: Nm, range [0, 1000], Default maximum value
    * @param[out] ec Error code
    */
   void setJointControlMaxTorque(const std::array<double, DoF> &max_torque, error_code &ec) noexcept;

   /**
    * @brief Impedance Speed Limiting - Joint。fcStart()Takes effect when started, invalid after fcStop。
    * Description: Taking the example of pressing down with an expected force until it contacts this scene, it is generally achieved by setting the expected force. This interface can perform speed-limiting processing for the contact process.。
    * @param[in] max_joint_vel Maximum joint velocity, unit: rad/s, range [0, 10.0], Default maximum value
    * @param[out] ec Error code
    */
   void setJointControlMaxVel(const std::array<double, DoF> &max_joint_vel, error_code &ec) noexcept;

   /**
    * @brief Set the control bandwidth for each axis force。fcStart()Takes effect when started, invalid after fcStop。
    * @param[in] gain Bandwidth of each axis, range [0, 60], Default value for each axis is 20, no units
    * @param[out] ec Error code
    */
   void setFcGain(const std::array<double, DoF> &gain, error_code &ec) noexcept;

   /**
    * @brief Set the friction compensation coefficient for each axis。fcStart()Takes effect when started, invalid after fcStop。
    * @param[in] fric Compensation factor, range [0, 1], Default value for each axis is 0.9, no unit
    * @param[out] ec Error code
    */
   void setFriction(const std::array<double, DoF> &fric, error_code &ec) noexcept;

 };

} // namespace rokae

#endif //XCORESDK_INCLUDE_ROKAE_FORCE_CONTROL_H_
