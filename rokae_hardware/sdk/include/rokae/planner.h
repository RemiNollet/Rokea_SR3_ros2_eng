/**
 * @file planner.h
 * @brief Path planning related functions
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_PLANNER_H
#define ROKAEAPI_PLANNER_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "Eigen/Core"
#include "Eigen/Dense"
#include "base.h"
#include "data_types.h"

namespace rokae {

 /// @cond DO_NOT_DOCUMENT
 // forward-declaration
 template <unsigned short DoF>
 class Cobot;
 template <unsigned short DoF>
 class xMateModel;
 /// @endcond

/**
 * @class CartMotionGenerator
 * @brief SCartesian space motion of speed planning。
 * References: Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
 class XCORE_API CartMotionGenerator {
  public:
   /**
    * @brief Generate a smooth trajectory in Cartesian space based on the total path length and velocity factor
    * @param[in] speed_factor Speed coefficient, range[0, 1]。Final speed/acceleration = Maximum Speed / Acceleration * Speed coefficient
    * @param[in] s_goal Total path length [m]
    */
   CartMotionGenerator(double speed_factor, double s_goal);

   /**
    * @brief Destructor
    */
   ~CartMotionGenerator();

   /**
    * @brief Set Cartesian space motion parameters
    * @param[in] ds_max Maximum speed [m/s], Default value1.0m/s。
    * @param[in] dds_max_start Maximum starting acceleration [m/s^2], Default value2.5m/s2
    * @param[in] dds_max_end Maximum terminal acceleration [m/s^2], Default value2.5m/s2
    */
   void setMax(double ds_max, double dds_max_start, double dds_max_end);

   /**
    * @brief Obtain total exercise time
    * @return Exercise time, unit: seconds
    */
   double getTime();

   /**
    * @brief Arc length at time ts
    * @param[in] t Time interval since start of planning, unit: seconds
    * @param[out] delta_s_d Calculation Result
    * @return false: Motion planning is not finished | true: Motion planning finished
    */
   bool calculateDesiredValues(double t, double *delta_s_d) const;

   /**
    * @brief Synchronize current arc length
    * @param[in] s_init Initial arc length
    */
   void calculateSynchronizedValues(double s_init);

  XCORESDK_DECLARE_IMPL
 };

/**
 * @class JointMotionGenerator
 * @brief SAxis space motion of speed planning. References:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
 class XCORE_API JointMotionGenerator {
   using VectorNd = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
   using VectorNi = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  public:
   /**
    * @brief Generate a joint-space trajectory based on the joint target position and velocity coefficients, which can be used to return to zero or reach a specified position。
    * @param[in] speed_factor Speed coefficient, range[0, 1]。Final speed/acceleration of each axis = Maximum speed/acceleration of the axis in space * Speed coefficient
    * @param[in] q_goal Target joint angle [rad]
    */
   JointMotionGenerator(double speed_factor, std::array<double, 7> q_goal);
   virtual ~JointMotionGenerator();

   /**
    * @brief Set the motion parameters for axis-space S velocity planning
    * @param[in] dq_max Maximum speed [rad/s], Default valueJ1~J4 1.0rad/s, J5~J7 1.25rad/s
    * @param[in] ddq_max_start Maximum starting acceleration [rad/s^2], Default value2.5rad/s^2
    * @param[in] ddq_max_end Maximum terminal acceleration [rad/s^2], Default value2.5rad/s^2
    */
   void setMax(const std::array<double, 7> &dq_max,
               const std::array<double, 7> &ddq_max_start,
               const std::array<double, 7> &ddq_max_end);

   /**
    * @brief Obtain total exercise time
    */
   double getTime();

   /**
    * @brief Joint angle increment at computation time t
    * @param[in] t Time point, unit: seconds
    * @param[out] delta_q_d Calculation Result
    * @return false: Motion planning is not finished | true: Motion planning finished
    */
   bool calculateDesiredValues(double t, std::array<double, 7> &delta_q_d) const;

   /**
    * @brief Synchronize current axis angle value
    * @param[in] q_init Initial shaft angle
    */
   void calculateSynchronizedValues(const std::array<double, 7> &q_init);

  XCORESDK_DECLARE_IMPL
 };

#if defined(XMATEMODEL_LIB_SUPPORTED)
 /**
  * @brief Point following, where the point can be a Cartesian pose or joint angles, suitable for use in visual servo following scenarios
  * @tparam DoF Number of axles
  */
 template <unsigned short DoF>
 class XCORE_API FollowPosition {

  public:
   /**
    * Default constructor. Call init() to setup Robot instance
    */
   FollowPosition();

   /**
    * @param robot rokae::RobotExample
    * @param model rokae::xMateModelExample, throughrobot.model()Obtain
    * @param[in] endInFlange Pose of the end relative to the flange
    */
   FollowPosition(Cobot<DoF>& robot,
                  xMateModel<DoF>& model,
                  const Eigen::Transform<double, 3, Eigen::Isometry>& endInFlange = Eigen::Transform<double, 3, Eigen::Isometry>::Identity());

   ~FollowPosition();

    /**
     * @brief InitializationFollowPosition
     * @param robot rokae::RobotExample
     * @param model rokae::xMateModelExample, throughrobot.model()Obtain
     */
   void init(Cobot<DoF>& robot, XMateModel<DoF>& model);

   /**
    * @brief Start target following - Cartesian pose. This interface is non-blocking.。
    * @param[in] bMe_desire The desired target pose, which is the end effector relative to the base coordinate system, that is, the TCP pose.
    */
   void start(const Eigen::Transform<double, 3, Eigen::Isometry>& bMe_desire);

   /**
    * @brief Start target following - axis angle. This interface is non-blocking。
    * @param[in] jnt_desire Expected shaft angle
    */
   void start(const std::array<double, DoF> &jnt_desire);

   /**
    * @brief Stop target following
    */
   void stop();

   /**
    * @brief Update the desired pose。
    * @note Follow a process with acceleration and deceleration, slowing down when approaching the target point. Therefore, the updated target points should not be dense, and the update frequency should not be too fast. The update interval is recommended to be at least on the order of tens of milliseconds.
    * @param[in] bMe_desire The end relative to the base coordinate system, that is, the TCP pose
    */
   void update(const Eigen::Transform<double, 3, Eigen::Isometry>& bMe_desire);

   /**
    * @brief Update the desired target axis angle
    * @note Follow a process with acceleration and deceleration, slowing down when approaching the target point. Therefore, the updated target points should not be dense, and the update frequency should not be too fast. The update interval is recommended to be at least on the order of tens of milliseconds.
    * @param[in] jnt_desired Axis angle, unit: radians
    */
   void update(const std::array<double, DoF>& jnt_desired);

   /**
    * @brief Set the speed ratio, which can be adjusted at any time during the target following process。
    * The final speed is limited by the maximum value, and the current value cannot be changed. The maximum speed for each axis is: 120.0, 120.0, 180.0, 180.0, 200.0, 200.0, 200.0 [°/s]；
    * Maximum acceleration of each axis is500.0 [°/s^2]
    * @param[in] scale Speed ratio, default is0.5
    */
   void setScale(double scale);

  XCORESDK_DECLARE_IMPL

 };

 #endif // #XMATEMODEL_LIB_SUPPORTED

}  // namespace rokae

#endif //ROKAEAPI_PLANNER_H
