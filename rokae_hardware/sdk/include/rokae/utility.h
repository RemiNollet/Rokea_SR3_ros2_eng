/**
 * @file utility.h
 * @brief Data Type Conversion
 * @copyright Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_
#define ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "robot.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

namespace rokae {
 /**
  * @class Utils
  * @brief Data Type Conversion Utility Class
  */
 class XCORE_API Utils {

  public:

   /**
    * @brief Degrees to radians
    */
   static double degToRad(double degrees) {
     return degrees * (EIGEN_PI / 180);
   }

   /**
    * @brief Radian to degree
    */
   static double radToDeg(double rad) {
     return rad / EIGEN_PI * 180.0;
   }

   /**
    * @brief Convert array degrees to radians
    */
   template<size_t S>
   static std::array<double, S> degToRad(const std::array<double, S> &degrees) {
     std::array<double, S> ret {};
     std::transform(degrees.cbegin(), degrees.cend(), ret.begin(),
                    [](const double &d) { return degToRad(d);});
     return ret;
   }

   /**
    * @brief Convert array degrees to radians
    */
   static std::vector<double> degToRad(const std::vector<double> &degrees) {
     std::vector<double> ret {};
     std::transform(degrees.cbegin(), degrees.cend(), std::back_inserter(ret),
                    [](const double &d) { return degToRad(d);});
     return ret;
   }


   /**
    * @brief Convert array from radians to degrees
    */
   template<size_t S>
   static std::array<double, S> radToDeg(const std::array<double, S> &rad) {
     std::array<double, S> ret {};
     std::transform(rad.cbegin(), rad.cend(), ret.begin(),
                    [](const double &d) { return radToDeg(d);});
     return ret;
   }

   /**
    * @brief Convert array from radians to degrees
    */
   static std::vector<double> radToDeg(const std::vector<double> &rad) {
     std::vector<double> ret {};
     std::transform(rad.cbegin(), rad.cend(), std::back_inserter(ret),
                    [](const double &d) { return radToDeg(d);});
     return ret;
   }
   /**
    * @brief Convert transformation matrix to array
    * @param[in] rot Rotation Matrix
    * @param[in] trans Translation vector
    * @param[out] array Conversion result, row priority
    */
   static void transMatrixToArray(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, std::array<double, 16>& array) {
     array = {{rot(0, 0), rot(0, 1), rot(0, 2), trans(0),
               rot(1, 0), rot(1, 1), rot(1, 2), trans(1),
               rot(2, 0), rot(2, 1), rot(2, 2), trans(2),
               0,         0,         0,         1       }};
   }

   /**
    * @brief Convert transformation matrix to array
    * @param[in] R Transformation Matrix
    * @param[out] array Conversion result, row priority
    */
   static void transMatrixToArray_all(const Eigen::Matrix4d& R ,std::array<double, 16>& array) {
       array = { { R(0,0),R(0,1),R(0,2),R(0,3),
                   R(1,0),R(1,1),R(1,2),R(1,3),
                   R(2,0),R(2,1),R(2,2),R(2,3),
                   R(3,0),R(3,1),R(3,2),R(3,3)}};
   }

   /**
    * @brief Array to transformation matrix
    * @param[in] array Array, row-major
    * @param[out] rot Rotation Matrix
    * @param[out] trans Translation vector
    */
   static void arrayToTransMatrix(const std::array<double, 16>& array, Eigen::Matrix3d& rot, Eigen::Vector3d& trans) {
     rot << array[0], array[1], array[2], array[4], array[5], array[6], array[8], array[9], array[10];
     trans << array[3], array[7], array[11];
   }

   /**
    * @brief Array to transformation matrix
    * @param[in] array Array, row-major
    * @param[out] 4*4Transformation Matrix
    */
   static void arrayToTransMatrix_all(const std::array<double, 16>& array, Eigen::Matrix4d& R) {
       R << array[0], array[1], array[2],array[3], array[4], array[5], array[6],array[7], array[8], array[9], array[10],array[11],array[12],array[13],array[14],array[15];
   }

   /**
    * @brief The array representing the pose{X, Y, Z, Rx, Ry, Rz}Convert to row-major homogeneous transformation matrix
    * @param[in] xyz_abc Enter pose, {X, Y, Z, Rx, Ry, Rz}
    * @param[out] transMatrix Conversion Result
    */
   static void postureToTransArray(const std::array<double, 6> &xyz_abc, std::array<double, 16> &transMatrix) {
     using namespace Eigen;
     Vector3d vec;
     Matrix3d rot;
     rot = (AngleAxisd(xyz_abc[5], Vector3d::UnitZ()) *
       AngleAxisd(xyz_abc[4], Vector3d::UnitY()) *
       AngleAxisd(xyz_abc[3], Vector3d::UnitX())).toRotationMatrix();
     vec << xyz_abc[0], xyz_abc[1], xyz_abc[2];
     Utils::transMatrixToArray(rot, vec, transMatrix);
   }

   /**
    * @brief Convert the row-priority homogeneous transformation matrix into{X, Y, Z, Rx, Ry, Rz}Array
    * @param[in] transMatrix Row-priority homogeneous transformation matrix
    * @param[out]  xyz_abc Conversion Result
    */
   static void transArrayToPosture(const std::array<double, 16> &transMatrix, std::array<double, 6> &xyz_abc) {
     xyz_abc[0] = transMatrix[3];
     xyz_abc[1] = transMatrix[7];
     xyz_abc[2] = transMatrix[11];
     xyz_abc[4] = atan2(-transMatrix[8], sqrt(pow(transMatrix[0], 2.0) + pow(transMatrix[4],2.0)));
     if(fabs(xyz_abc[4]) > (M_PI/2.0 - 1E-12)) {
       xyz_abc[5] = atan2(-transMatrix[1], transMatrix[5]);
       xyz_abc[3] = 0.0;
     } else {
       xyz_abc[3] = atan2(transMatrix[9], transMatrix[10]);
       xyz_abc[5] = atan2(transMatrix[4], transMatrix[0]);
     }
   }

   /**
    * @brief Convert Euler angles to rotation matrix
    * @param[in] euler Euler angles, sequence[z, y, x], Unit: Radian
    * @param[out] matrix Rotation Matrix
    */
   static void eulerToMatrix(const Eigen::Vector3d& euler, Eigen::Matrix3d& matrix) {
     Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));
     Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
     Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
     matrix = yawAngle * pitchAngle * rollAngle;
   }

   /**
    * @brief Quaternion to Euler angles
    * @param[in] w Q1
    * @param[in] x Q2
    * @param[in] y Q3
    * @param[in] z Q4
    * @return { Rx, Ry, Rz }
    */
   static std::array<double, 3> quaternionToEuler(double w, double x, double y, double z) {
     Eigen::Quaterniond  quaterniond(w, x, y, z);
     Eigen::Vector3d euler = quaterniond.matrix().eulerAngles(2, 1, 0);
     return { euler(2), euler(1), euler(0) };
   }

   /**
    * @brief Euler angles to quaternion
    * @param[in] rpy Euler angles, unit: radians
    * @return Quaternion { Q1, Q2, Q3, Q4 }
    */
   static std::array<double, 4> eulerToQuaternion(const std::array<double, 3> &rpy) {
     using namespace Eigen;
     Quaterniond q = AngleAxisd(rpy[0], Vector3d::UnitX()) * AngleAxisd(rpy[1], Vector3d::UnitY()) *
       AngleAxisd(rpy[2], Vector3d::UnitZ());
     return std::array<double, 4>{{ q.coeffs().w(), q.coeffs().x(), q.coeffs().y(), q.coeffs().z() }};
   }

   /**
    * @brief ToolsetTransform into tools and workpieces
    * @param[in] tool_set Tool and Workpiece Group
    * @param[out] ref_trans External coordinate system transformation matrix
    * @param[out] end_trans End coordinate system transformation matrix
    */
   static inline void toolsetCalcPos(const Toolset &tool_set, std::array<double, 16> &ref_trans, std::array<double, 16> &end_trans) {
       std::array<double, 6> ref{}, end{};
       for (int i=0 ; i<3 ; i++)
       {
           ref[i] = tool_set.ref.trans[i];
           end[i] = tool_set.end.trans[i];
       }
	   for (int j = 0; j < 3; j++)
	   {
		   ref[j+3] = tool_set.ref.rpy[j];
		   end[j+3] = tool_set.end.rpy[j];
	   }

       postureToTransArray(end, end_trans);
       postureToTransArray(ref, ref_trans);
   }

   /**
    * @brief Coordinate system transformation: Convert the end-effector coordinates relative to the external reference coordinate system to the coordinates of the flange relative to the base coordinate system
    * @param[in] base_in_world Setting the base coordinate system relative to the world coordinate system
    * @param[in] tool_set Tool and Workpiece Setup
    * @param[in] end_in_ref End relative (external) reference coordinate system coordinates
    * @return Flange coordinates relative to the base coordinate system
    */
   static std::array<double, 6> EndInRefToFlanInBase(const std::array<double, 6>& base_in_world, const Toolset &tool_set,
                                                     const std::array<double, 6> &end_in_ref){
	   using namespace Eigen;

       //Handle base coordinate settings
	   Matrix4d R_BIW;
       std::array<double, 16> BIW{};//Base coordinates&World coordinates
       postureToTransArray(base_in_world, BIW);//Convert pose coordinates to a transformation matrix (array)）
	   arrayToTransMatrix_all( BIW,R_BIW);//Convert the array to4*4Matrix

//toolset directly handles
       std::array<double, 16> EIF{};//terminal&Flange Coordinates
       std::array<double, 16> RIW{};//Reference&World coordinates
	   toolsetCalcPos(tool_set, RIW, EIF);//Calculate the toolsetpos
	   Matrix4d R_EIF;
	   arrayToTransMatrix_all(EIF, R_EIF);//Convert the array to4*4Matrix
	   Matrix4d R_RIW;
	   arrayToTransMatrix_all(RIW, R_RIW);//Convert the array to4*4Matrix

       // Handle the input terminal relative reference pose
       std::array<double, 16> EIR{};//terminal&Reference
       postureToTransArray(end_in_ref, EIR);//Convert pose coordinates to a transformation matrix (array)）
	   Matrix4d R_EIR;
	   arrayToTransMatrix_all(EIR, R_EIR);//Convert the array to4*4Matrix

       // Calculate the output flange pose relative to the base coordinates
       std::array<double, 16> FIB{};//Flange&Base coordinates
       Matrix4d R_FIB;
       R_FIB=R_BIW.inverse() * R_RIW * R_EIR * R_EIF.inverse();//Matrix operations
       transMatrixToArray_all(R_FIB, FIB);//put4*4Convert matrix to array
       std::array<double, 6> flan_in_base{};
       transArrayToPosture(FIB, flan_in_base);//Convert array to pose

       return flan_in_base;
   }

   /**
    * @brief Coordinate system transformation: Convert the flange coordinates relative to the base coordinate system to the end coordinates relative to the external reference frame
     * @param[in] base_in_world Setting the base coordinate system relative to the world coordinate system
     * @param[in] tool_set Tool and Workpiece Setup
     * @param[in] flan_in_base Flange coordinates relative to the base coordinate system
     * @return End relative (external) reference coordinate system coordinates
     */
   static std::array<double, 6> FlanInBaseToEndInRef(const std::array<double, 6>& base_in_world, const Toolset &tool_set,
                                                     const std::array<double, 6> flan_in_base) {
       using namespace Eigen;

       //Handle base coordinate settings
       Matrix4d R_BIW;
       std::array<double, 16> BIW{};//Base coordinates&World coordinates
       postureToTransArray(base_in_world, BIW);//Convert pose coordinates to a transformation matrix (array)）
       arrayToTransMatrix_all(BIW, R_BIW);//Convert the array to4*4Matrix

       //toolset directly handles
       std::array<double, 16> EIF{};//terminal&Flange Coordinates
       std::array<double, 16> RIW{};//Reference&World coordinates
       toolsetCalcPos(tool_set, RIW, EIF);//Calculate the toolsetpos
	   Matrix4d R_EIF;
	   arrayToTransMatrix_all(EIF, R_EIF);//Convert the array to4*4Matrix
       Matrix4d R_RIW;
       arrayToTransMatrix_all(RIW, R_RIW);//Convert the array to4*4Matrix

       //Handle the pose of the input French relative to the base coordinates
       std::array<double, 16> FIB{};//terminal&Reference
       postureToTransArray(flan_in_base, FIB);//Convert pose coordinates to a transformation matrix (array)）
       Matrix4d R_FIB;
       arrayToTransMatrix_all(FIB, R_FIB);//Convert the array to4*4Matrix

       //Calculate the end-effector's pose relative to the reference
       std::array<double, 16> EIR{};//Flange&Base coordinates
       Matrix4d R_EIR;
       R_EIR = R_RIW.inverse() * R_BIW * R_FIB * R_EIF;//Matrix operations
       transMatrixToArray_all(R_EIR, EIR);//put4*4Convert matrix to array
       std::array<double, 6> end_in_ref{};
       transArrayToPosture(EIR, end_in_ref);//Convert array to pose

       return end_in_ref;
   }

   Utils() = delete;
   ~Utils() = delete;
 };

}  // namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_
