/*
 * RobotMotionMapUpdater.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include "mapping/ElevationMap.hpp"

// Eigen
#include <Eigen/Core>

// Kindr
//#include <kindr/Core>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
namespace mapping {

/*!
 * Computes the map variance update from the pose covariance of the robot.
 */
class RobotMotionMapUpdater
{
 public:

  //typedef kindr::HomogeneousTransformationPosition3RotationQuaternionD Pose;
  
  typedef Eigen::Matrix<double, 3, 3> Covariance;
  typedef Eigen::Matrix<double, 6, 6> PoseCovariance;
  typedef Eigen::Matrix<double, 4, 4> ReducedCovariance;
  typedef Eigen::Matrix<double, 4, 4> Jacobian;

  /*!
   * Constructor.
   */
  RobotMotionMapUpdater(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RobotMotionMapUpdater();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Computes the model update for the elevation map based on the pose covariance and
   * adds the update to the map.
   * @param[in] map the elevation map to be updated.
   * @param[in] robotPose the current pose.
   * @param[in] robotPoseCovariance the current pose covariance matrix.
   * @param[in] time the time of the current update.
   * @return true if successful.
   */
  bool update(ElevationMap& map, /*const Pose&*/ Eigen::Matrix3d robotPose, const PoseCovariance& robotPoseCovariance, const ros::Time& time);

 private:

  /*!
   * Computes the reduced covariance (4x4: x, y, z, yaw) from the full pose covariance (6x6: x, y, z, roll, pitch, yaw).
   * @param[in] robotPose the robot pose.
   * @param[in] robotPoseCovariance the full pose covariance matrix (6x6).
   * @param[out] reducedCovariance the reduced covariance matrix (4x4);
   * @return true if successful.
   */
  bool computeReducedCovariance(/*const Pose&*/ Eigen::Matrix3d robotPose, const PoseCovariance& robotPoseCovariance,
                                ReducedCovariance& reducedCovariance);

  /*!
   * Computes the covariance between the new and the previous pose.
   * @param[in] robotPose the current robot pose.
   * @param[in] reducedCovariance the current robot pose covariance matrix (reduced).
   * @param[out] relativeRobotPoseCovariance the relative covariance between the current and the previous robot pose (reduced form).
   * @return true if successful.
   */
  bool computeRelativeCovariance(/*const Pose&*/ Eigen::Matrix3d robotPose, const ReducedCovariance& reducedCovariance,
                                 ReducedCovariance& relativeCovariance);

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  //! Time of the previous update.
  ros::Time previousUpdateTime_;

  //! Previous robot pose.
  //Pose previousRobotPose_;
   Eigen::Matrix<double, 3, 3> robotPose;
   Eigen::Matrix<double, 3,3> previousRobotPose_;
  //! Robot pose covariance (reduced) from the previous update.
  ReducedCovariance previousReducedCovariance_;

  //! Scaling factor for the covariance matrix (default all ones).
  Eigen::Array<double, 6, 6> covarianceScale_;
};

} /* namespace */
