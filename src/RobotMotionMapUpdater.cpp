
/*
 * RobotMotionMapUpdater.cpp
 *
 *  eated on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "mapping/RobotMotionMapUpdater.hpp"

// Kindr
//#include <kindr/rotations/RotationEigen.hpp>
//#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
//#include <kindr/linear_algebra/LinearAlgebra.hpp>

using namespace std;
using namespace grid_map;
//using namespace kindr::rotations::eigen_impl;

namespace mapping {

RobotMotionMapUpdater::RobotMotionMapUpdater(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  previousReducedCovariance_.setZero();
  covarianceScale_.setOnes();
}

RobotMotionMapUpdater::~RobotMotionMapUpdater()
{

}

bool RobotMotionMapUpdater::readParameters()
{
  nodeHandle_.param("robot_motion_map_update/covariance_scale_translation_x", covarianceScale_(0, 0), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_translation_y", covarianceScale_(1, 1), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_translation_z", covarianceScale_(2, 2), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_rotation_x", covarianceScale_(3, 3), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_rotation_y", covarianceScale_(4, 4), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_rotation_z", covarianceScale_(5, 5), 1.0);
  return true;
}

bool RobotMotionMapUpdater::update(
    ElevationMap& map,  Eigen::Matrix3d robotPose,/*const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& robotPose,*/
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, const ros::Time& time)
{
   PoseCovariance robotPoseCovarianceScaled = (covarianceScale_ * robotPoseCovariance.array()).matrix();

  // Check if update necessary.
  if (previousUpdateTime_ == time) return false;

 tf::StampedTransform transform1,transform2;
  // Check if update necessary.
 // if (((robotPoseCovarianceScaled - RCovariance_).array() == 0.0).all()) return false;

  tf::TransformListener listener;

  try{
    // trans 1 parent(base_link) to sensor
    //tran 2 parent(base_link) to map

    ros::Time now = ros::Time::now();
     // listener.waitForTransform("/odom", "/sensor_link",ros::Time(0), ros::Duration(0.5));
    //  listener.lookupTransform("/odom", "/sensor_link", ros::Time(0), transform1);
    listener.waitForTransform("/base_link", "/sensor_link",ros::Time(0), ros::Duration(0.5));
    listener.lookupTransform("/base_link", "/sensor_link", ros::Time(0), transform1);
    //  listener.waitForTransform("/odom", "/map", ros::Time(0), ros::Duration(0.5));
    //  listener.lookupTransform("/odom", "/map", ros::Time(0), transform2);
    listener.waitForTransform("/base_link", "/world", ros::Time(0), ros::Duration(0.5));
    listener.lookupTransform("/base_link", "/world", ros::Time(0), transform2);
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
    }
    tf::Matrix3x3 rotationParentToMap = transform2.getBasis();
    tf::Vector3 Row1 = rotationParentToMap.getRow(0);
    tf::Vector3 Row2 = rotationParentToMap.getRow(1);
    tf::Vector3 Row3 = rotationParentToMap.getRow(2);

    Eigen::Matrix3d rotationParentToMap_;
    rotationParentToMap_<< Row1.getX(),Row1.getY(),Row1.getZ(),
             Row2.getX(),Row2.getY(),Row2.getZ(),
             Row3.getX(),Row3.getY(),Row3.getZ();
    tf::Vector3 ParentToSensor = transform1.getOrigin();

  // Initialize update data.
  Size size = map.getRawGridMap().getSize();
  Matrix varianceUpdate(size(0), size(1));
  Matrix horizontalVarianceUpdateX(size(0), size(1));
  Matrix horizontalVarianceUpdateY(size(0), size(1));
  Matrix horizontalVarianceUpdateXY(size(0), size(1));
  // Covariance matrices.
  // Eigen::Matrix3d previousPositionCovariance = previousReducedCovariance_.topLeftCorner<3, 3>();
  // Eigen::Matrix3d positionCovariance = robotPoseCovarianceScaled.topLeftCorner<3, 3>();
  // Eigen::Matrix3d previousRotationCovariance = previousReducedCovariance_.bottomRightCorner<3, 3>();
  // Eigen::Matrix3d rotationCovariance = robotPoseCovarianceScaled.bottomRightCorner<3, 3>();
  ReducedCovariance reducedCovariance;
  computeReducedCovariance(robotPose, robotPoseCovarianceScaled, reducedCovariance);
  ReducedCovariance relativeCovariance;
  computeRelativeCovariance(robotPose, reducedCovariance, relativeCovariance);

  // Retrieve covariances for (24).
  Covariance positionCovariance = relativeCovariance.topLeftCorner<3, 3>();
  Covariance rotationCovariance(Covariance::Zero());
  rotationCovariance(2, 2) = relativeCovariance(3, 3);

  // Parent to elevation map frame rotation (C_IM^T = C_SM^T * C_IS^T)
  //Eigen::Matrix3d parentToMapRotation = RotationMatrixPD(map.getPose().getRotation()).matrix().transpose();



  Eigen::Matrix3d parentToMapRotation = rotationParentToMap_;
  // Translation Jacobian (J_r)
  //Eigen::Matrix3d translationJacobian = -parentToMapRotation;
  Eigen::Matrix3d translationJacobian = -parentToMapRotation.transpose();
  // Translation variance update (for all points the same).
  Eigen::Vector3f translationVarianceUpdate = (translationJacobian *
                                        (positionCovariance) *
                                        translationJacobian.transpose()).diagonal().cast<float>();

  // Robot/sensor position (I_r_IS, for all points the same).
  //kindr::phys_quant::eigen_impl::Position3D robotPosition = robotPose.getPosition();
const Eigen::Vector3f robotPosition(ParentToSensor.getX(),ParentToSensor.getY(), ParentToSensor.getZ());
  // For each cell in map. // TODO Change to new iterator.
  for (unsigned int i = 0; i < size(0); ++i)
  {
    for (unsigned int j = 0; j < size(1); ++j)
    {
      nav_msgs::Odometry cellPositionz;//kindr::phys_quant::eigen_impl::Position3D cellPosition; // I_r_IP

      if (map.getPosition3dInRobotParentFrame(Index(i, j), cellPositionz))
      {
        Eigen::Vector3f cellPosition(cellPositionz.pose.pose.position.x,cellPositionz.pose.pose.position.y,cellPositionz.pose.pose.position.z);
        Eigen::Vector3f pointVector(cellPosition-robotPosition);//.case<float>();
        Eigen::Vector3f SkewVector = pointVector;

      Eigen::Matrix3d Skew_matrix;
      Skew_matrix << 0.0, -SkewVector(2), SkewVector(1),
          SkewVector(2), 0.0, -SkewVector(0),
        -SkewVector(1), SkewVector(0), 0.0;

      const Eigen::Matrix3d rotationJacobian = parentToMapRotation*Skew_matrix;
  // //           //* kindr::linear_algebra::getSkewMatrixFromVector((cellPosition - robotPosition).vector());

  // //       // Rotation variance update.
        Eigen::Matrix2f rotationVarianceUpdate = (rotationJacobian *
                                           (rotationCovariance) *
                                            rotationJacobian.transpose()).topLeftCorner<2,2>().cast<float>();

          // Variance update.
        varianceUpdate(i, j) = translationVarianceUpdate.z();
        horizontalVarianceUpdateX(i, j) = translationVarianceUpdate.x() + rotationVarianceUpdate(0, 0);
        horizontalVarianceUpdateY(i, j) = translationVarianceUpdate.y() + rotationVarianceUpdate(1, 1);
        horizontalVarianceUpdateXY(i, j) = rotationVarianceUpdate(0, 1);
      }
      else
      {
        // Cell invalid. // TODO Change to new functions
        varianceUpdate(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateX(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateY(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateXY(i, j) = numeric_limits<float>::infinity();
      }
      //   // Rotation Jacobian (J_q)
      //   Eigen::Matrix3d rotationJacobian = parentToMapRotation
      //       * kindr::linear_algebra::getSkewMatrixFromVector((cellPosition - robotPosition).vector());

      //   // Rotation variance update.
      //   Eigen::Vector3f rotationVarianceUpdate = (rotationJacobian *
      //                                      (rotationCovariance - previousRotationCovariance) *
      //                                      rotationJacobian.transpose()).diagonal().cast<float>();

      //   // Variance update.
      //   varianceUpdate(i, j) = translationVarianceUpdate.z() + rotationVarianceUpdate.z();
      //   horizontalVarianceUpdateX(i, j) = translationVarianceUpdate.x() + rotationVarianceUpdate.x();
      //   horizontalVarianceUpdateY(i, j) = translationVarianceUpdate.y() + rotationVarianceUpdate.y();
      // }
      // else
      // {
      //   // Cell invalid. // TODO Change to new functions
      //   varianceUpdate(i, j) = numeric_limits<float>::infinity();
      //   horizontalVarianceUpdateX(i, j) = numeric_limits<float>::infinity();
      //   horizontalVarianceUpdateY(i, j) = numeric_limits<float>::infinity();
      // }

    }
  }

  map.update(varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY, horizontalVarianceUpdateXY, time);
  previousReducedCovariance_ = reducedCovariance;
  previousRobotPose_ = robotPose;
  return true;
}
bool RobotMotionMapUpdater::computeReducedCovariance(const Eigen::Matrix3d robotPose,
                                                     const PoseCovariance& robotPoseCovariance,
                                                     ReducedCovariance& reducedCovariance)
{
double	ay = atan2(-robotPose(3,1),((robotPose(1,1))*(robotPose(1,1)) + sqrt(robotPose(2,1)*robotPose(2,1))));
double	az = atan2((robotPose(2,1)/cos(ay)),(robotPose(1,1)/cos(ay)));
double	ax = atan2((robotPose(3,2)/cos(ay)),(robotPose(3,3)/cos(ay)));


std::cout << ay << "," << az << "," << ax << std::endl;

  // Eigen::AngleAxisd eulerAngles;
  // eulerAngles(robotPose,Eigen::Vector3d(ax,ay,az));
   double tanOfPitch = tan(ay);
  // // (A.5)
   Eigen::Matrix<double, 1, 3> yawJacobian(cos(az) * tanOfPitch, sin(az) * tanOfPitch, 1.0);
   Eigen::Matrix<double, 4, 6> jacobian;
   jacobian.setZero();
   jacobian.topLeftCorner(3, 3).setIdentity();
   jacobian.bottomRightCorner(1, 3) = yawJacobian;

  // // (A.3)
   reducedCovariance = jacobian * robotPoseCovariance * jacobian.transpose();
  return true;
}

bool RobotMotionMapUpdater::computeRelativeCovariance(const Eigen::Matrix3d robotPose,
                                                      const ReducedCovariance& reducedCovariance,
                                                      ReducedCovariance& relativeCovariance)
{
 // Rotation matrix of z-align frame R_I_tilde_B.
  Eigen::Matrix3d rotationVector_I_B = robotPose;
 // Eigen::Vector3d rotationVector_I_tilde_B(0.0, 0.0, rotationVector_I_B.getRow(3));
 // const RotationMatrixPD R_I_tilde_B(rotationVector_I_tilde_B);
  Eigen::Matrix3d  R_I_tilde_B = rotationVector_I_B;
  // Compute translational velocity from finite differences.
  Eigen::Matrix3d positionInRobotFrame = (
      robotPose - previousRobotPose_);
  Eigen::Vector3d v_Delta_t;
  v_Delta_t<< positionInRobotFrame(0,2),positionInRobotFrame(1,2),positionInRobotFrame(2,2); // (A.8)

  // Jacobian F (A.8).
  Jacobian F;
  F.setIdentity();
  // TODO Why does Eigen::Vector3d::UnitZ() not work?
  //Eigen::Vector3d(0.0, 0.0, 1.0) * R_I_tilde_B.matrix() * v_Delta_t.vector();

  Eigen::Vector3d SkewVector = Eigen::Vector3d(0.0, 0.0, 1.0);

      Eigen::Matrix3d Skew_matrix;
      Skew_matrix << 0.0, -SkewVector(2), SkewVector(1),
          SkewVector(2), 0.0, -SkewVector(0),
        -SkewVector(1), SkewVector(0), 0.0;

  F.topRightCorner(3, 1) =  Skew_matrix*R_I_tilde_B*v_Delta_t;

  // Jacobian inv(G) * Delta t (A.14).
  Jacobian inv_G_Delta_t;
  inv_G_Delta_t.setZero();
  inv_G_Delta_t(3, 3) = 1.0;
  Jacobian inv_G_transpose_Delta_t(inv_G_Delta_t);
  inv_G_Delta_t.topLeftCorner(3, 3) = R_I_tilde_B.matrix().transpose();
  inv_G_transpose_Delta_t.topLeftCorner(3, 3) = R_I_tilde_B.matrix();

  // Relative (reduced) robot covariance (A.13).
  relativeCovariance = inv_G_Delta_t
      * (reducedCovariance - F * previousReducedCovariance_ * F.transpose())
      * inv_G_transpose_Delta_t;

  return true;
}

} /* namespace */
