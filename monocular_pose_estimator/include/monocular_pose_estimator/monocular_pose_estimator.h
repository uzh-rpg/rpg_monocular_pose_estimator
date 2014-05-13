/*
 * monocular_pose_estimator_node.h
 *
 *  Created on: Mar 26, 2014
 *      Author: Matthias Fässler
 */

/** \file monocular_pose_estimator_node.h
 * \brief File containing the definition of the Monocular Pose Estimator Node class
 *
 */

#ifndef MONOCULAR_POSE_ESTIMATOR_NODE_H_
#define MONOCULAR_POSE_ESTIMATOR_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <dynamic_reconfigure/server.h>
#include <monocular_pose_estimator/MonocularPoseEstimatorConfig.h>

#include "monocular_pose_estimator_lib/pose_estimator.h"

namespace monocular_pose_estimator
{

class MPENode
{
private:

  ros::NodeHandle nh_;

  image_transport::Publisher image_pub_; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher pose_pub_; //!< The ROS publisher that publishes the estimated pose.

  ros::Subscriber image_sub_; //!< The ROS subscriber to the raw camera image
  ros::Subscriber camera_info_sub_; //!< The ROS subscriber to the camera info

  dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig> dr_server_; //!< The dynamic reconfigure server
  //dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType cb_; //!< The dynamic reconfigure callback type

  geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the object

  bool have_camera_info_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the camera calibration parameters

  PoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated

public:

  MPENode();
  ~MPENode();

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

  void dynamicParametersCallback(monocular_pose_estimator::MonocularPoseEstimatorConfig &config, uint32_t level);
};

} // monocular_pose_estimator namespace

#endif /* MONOCULAR_POSE_ESTIMATOR_NODE_H_ */
