/*
 * KinectSensorProcessor.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <mapping/sensor_processors/SensorProcessorBase.hpp>

namespace mapping {

/*!
 * Sensor processor for Kinect-type (PrimeSense) structured light sensors.
 */
class LaserSensorProcessor : public SensorProcessorBase
{

public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  LaserSensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

  /*!
   * Destructor.
   */
	virtual ~LaserSensorProcessor();

private:
tf::TransformListener listener;
	tf::StampedTransform transform1, transform2;
  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
	bool readParameters();

	/*!
	 * Clean the point cloud. Points below the minimal and above the maximal sensor
	 * cutoff value are dropped.
	 * @param pointCloud the point cloud to clean.
	 * @return true if successful.
	 */
	virtual bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
	virtual bool computeVariances(
			const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
			const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
			Eigen::VectorXf& variances);
};


} /* namespace elevation_mapping */
