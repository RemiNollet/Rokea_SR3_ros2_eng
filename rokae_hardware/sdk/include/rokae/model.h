/**
 * @file model.h
 * @brief xMateModelModel Library
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_MODEL_H
#define ROKAEAPI_MODEL_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "data_types.h"

namespace rokae {

 // forward declarations
 class BaseRobot;
 class XService;
 struct Info;

 /**
  * @class BaseModel
  * @brief General Model Class
  */
 class XCORE_API BaseModel : public Base<BaseModel>{
  public:

   /// @cond DO_NOT_DOCUMENT
   explicit BaseModel(std::shared_ptr<XService> rpc);
   virtual ~BaseModel();
   /// @endcond

  XCORESDK_DECLARE_IMPL
 };

/**
 * @class Model_T
 * @brief Model Template Class
 * @tparam DoF Number of axles
 */
 template<unsigned short DoF>
 class XCORE_API Model_T : public BaseModel {

  public:
   using BaseModel::BaseModel;

   /**
    * @brief Calculate the inverse solution based on the pose. Inverse solution selection strategy:
    *   1) When the default Conf is off, select the solution closest to the current position
    *   2) When the default Conf is turned on, use confData to calculate
    * @param[in] posture The end-effector pose of the robot, relative to the external reference coordinate system. The reference coordinate system is defined bysetToolset()set
    * @param[out] ec Error code
    * @return Axis angle, unit: radian
    */
   std::array<double, DoF> calcIk(CartesianPosition posture, error_code &ec) noexcept;

   /**
    * @brief Calculate the inverse solution under the given tool workpiece coordinate system based on the pose. Inverse solution selection strategy:
    *   1) When the default Conf is off, select the solution closest to the current position
    *   2) When the default Conf is turned on, use confData to calculate
    * @param[in] posture Robot end-effector pose relative to the external reference coordinate system
    * @param[in] tool_set Tool-workpiece coordinate system
    * @param[out] ec Error code
    * @return Axis angle, unit: radian
    */
   std::array<double,DoF> calcIk(CartesianPosition posture,const Toolset &tool_set, error_code& ec) noexcept;

   /**
    * @brief Calculate the forward solution based on the axis angle。
    * @param[in] joints Axis angle, unit: radians
    * @param[out] ec Error code
    * @return The end-effector pose of the robot, relative to the external reference coordinate system. The reference coordinate system is defined bysetToolset()set
    */
   CartesianPosition calcFk(const std::array<double, DoF> &joints, error_code &ec) noexcept;

   /**
    * @brief Calculate the forward solution in the given tool-workpiece coordinate system based on the axis angles
    * @param joints Axis angle, unit: radians
    * @param tool_set Tool-workpiece coordinate system
    * @param ec Error code
    * @return Robot end-effector pose relative to the external reference coordinate system
    */
   CartesianPosition calcFk(const std::array<double,DoF> &joints,const Toolset &tool_set, error_code& ec) noexcept;

 };

 /**
  * @enum SegmentFrame
  * @brief Connecting rod number
  */
 enum class SegmentFrame : unsigned {
   joint1 = 1, joint2 = 2, joint3 = 3, joint4 = 4, joint5 = 5,
   joint6 = 6, joint7 = 7, flange = 8, endEffector = 9, stiffness = 10 };

 /**
  * @enum TorqueType
  * @brief Torque Type
  */
 enum class TorqueType {
   full,     ///< Joint torque, calculated from the dynamic model
   inertia,  ///< Inertial force
   coriolis, ///< Coriolis force
   friction, ///< Friction
   gravity   ///< Gravity
 };

#ifdef XMATEMODEL_LIB_SUPPORTED

 /**
  * @class xMateModel
  * @brief xMateModel library. Supported models: xMateER series, XMC7/12, XMS3/4/5。
  * Note: For the XMS5 model, you need to upgrade to a special version of the model file before using the model library. Otherwise, the calculation results will be incorrect. Please contact Luoshi technical support staff to upgrade the model file.。
  * @tparam DoF Number of axles
  */
 template <unsigned short DoF>
 class XCORE_API xMateModel : public Model_T<DoF> {

  public:
   /**
    * @brief Create an xMateModel instance
    * @throw ExecutionException Failed to load model
    */
   explicit xMateModel(std::shared_ptr<XService> rpc, const Info& info);
   ~xMateModel();

   /**
    * @brief Set load parameters, used only during calculations, and do not pass the parameters to the robot controller. After setting, the results of the dynamic calculations will change accordingly.
    * @param[in] mass Quality
    * @param[in] cog Center of mass, unit: m
    * @param[in] inertia Inertia
    * @see Load
    */
   void setLoad(double mass, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   /**
    * @brief Set the TCP tool to be used only during calculations and not pass the parameters to the robot controller. After setting the TCP, the forward and inverse kinematics results and input parameters change accordingly.
    * @param[in] f_t_ee Pose of the end effector relative to the flange
    * @param[in] ee_t_k Pose of the stiffness coordinate system relative to the end effector
    */
   void setTcpCoor(const std::array<double, 16> &f_t_ee, const std::array<double, 16> &ee_t_k);

   /**
    * @brief Obtain Cartesian spatial position
    * @param[in] jntPos Need to calculate the joint angles of the Cartesian pose
    * @param[in] nr Specify coordinate system, default value isflange
    * @return Vectorized 4x4 pose matrix, row-major.
    */
   std::array<double, 16> getCartPose(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief Obtain Cartesian space velocity
    * @param[in] jntPos Joint angles that need to calculate Cartesian space velocity
    * @param[in] jntVel Joint angular velocity that needs to be calculated for Cartesian space velocity
    * @param[in] nr Specify coordinate system, default value isflange
    * @return Calculation Result
    */
   std::array<double, 6> getCartVel(const std::array<double, DoF> &jntPos, const std::array<double, DoF> &jntVel,
                                    SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief Obtain Cartesian space acceleration
    * @param[in] jntPos Joint angles that need to calculate Cartesian space velocity
    * @param[in] jntVel Joint angular velocity that needs to be calculated for Cartesian space velocity
    * @param[in] jntAcc Joint angular accelerations required to calculate Cartesian space velocity
    * @param[in] nr Specify coordinate system
    * @return Calculation Result
    */
   std::array<double, 6> getCartAcc(const std::array<double, DoF> &jntPos,
                                    const std::array<double, DoF> &jntVel,
                                    const std::array<double, DoF> &jntAcc,
                                    SegmentFrame nr = SegmentFrame::flange );

   /**
    * @brief Inverse kinematics is used to obtain joint space positions. A single pose may correspond to multiple joint angles, and the principle for selecting jntPos is to choose a solution that is closest to jntInit.。
    * @param[in] cartPos Franka Emika Cartesian Space Pose
    * @param[in] elbow Arm angle
    * @param[in] jntInit Initial joint angle
    * @param[out] jntPos Joint space position
    * @return Calculate the inverse solution result -
    *    1) -1, -2, -3: No solution, the reason is that cartPos exceeds the robot's workspace;
    *    2) -4, -5: jntPosThere is a significant difference from jntInit. It is generally believed that jntInit represents the robot's current position, and the difference between jntPos and jntInit can be equivalent to the motor speed.。
    *               If the robot axis exceeds the rated speed, it will return -4 or-5;
    *    3) -6, -7: jntPosExceeding soft limit;
    *    4)	-8: Robot Strange；
    */
    int getJointPos(const std::array<double, 16> &cartPos,
                    double elbow,
                    const std::array<double, DoF> &jntInit,
                    std::array<double, DoF> &jntPos);

   /**
    * @brief Obtain joint space velocity through inverse solution
    * @param[in] cartVel Cartesian space velocity
    * @param[in] jntPos Joint angle at this time
    * @return Calculation Result
    */
   std::array<double, DoF> getJointVel(const std::array<double, 6> &cartVel, const std::array<double, DoF> &jntPos);

   /**
    * @brief Inverse solution to obtain joint space acceleration
    * @param[in] cartAcc Fractal Cartesian Space Acceleration
    * @param[in] jntPos Joint angle at this time
    * @param[in] jntVel Joint angular velocity at this time
    * @return Calculation Result
    */
   std::array<double, DoF> getJointAcc(const std::array<double, 6> &cartAcc,
                                       const std::array<double, DoF> &jntPos,
                                       const std::array<double, DoF> &jntVel);

   /**
    * @brief Obtain the Jacobian matrix of the specified coordinate system relative to the base coordinate system, row-major
    * @param[in] jntPos Joint angle.
    * @param[in] nr Specify coordinate system
    * @return Calculation result, length \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief Obtain the Jacobian matrix of the specified coordinate system relative to the base coordinate system, row-major
    * @param[in] jntPos Joint angle.
    * @param[in] f_t_ee Pose of the end effector relative to the flange coordinate system.
    * @param[in] ee_t_k Pose of the stiffness coordinate system relative to the end effector.
    * @param[in] nr Specify coordinate system
    * @return Calculation result, length \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos,
                                      const std::array<double, 16> &f_t_ee,
                                      const std::array<double, 16> &ee_t_k,
                                      SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief Joint torque calculated by the model without friction, calculation result unit: Nm. If there is a load, first throughsetLoad()Set load parameters。
    * @note OriginalgetTorque() getTorqueWithFriction is no longer supported, use this interface to calculate joint torque.
    * @param[in] jntPos Joint angle
    * @param[in] jntVel Joint angular velocity
    * @param[in] jntAcc Joint angular acceleration
    * @param[out] trq_full Total joint torque
    * @param[out] trq_inertia Centrifugal force
    * @param[out] trq_coriolis Coriolis force
    * @param[out] trq_gravity Gravity moment
    */
   void getTorqueNoFriction(const std::array<double, DoF> &jntPos,
                            const std::array<double, DoF> &jntVel,
                            const std::array<double, DoF> &jntAcc,
                            std::array<double, DoF> &trq_full,
                            std::array<double, DoF> &trq_inertia,
                            std::array<double, DoF> &trq_coriolis,
                            std::array<double, DoF> &trq_gravity);

  XCORESDK_DECLARE_IMPLD
 };

 template <unsigned short DoF>
 using XMateModel = xMateModel<DoF>;

#endif // #ifdef XMATEMODEL_LIB_SUPPORTED

}  // namespace rokae

#endif // ROKAEAPI_MODEL_H
