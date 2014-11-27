// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.

/*
 * visualization.cpp
 *
 *  Created on: Mar 21, 2014
 *      Author: Matthias Faessler
 */

/**
 * \file visualization.cpp
 * \brief File containing the function definitions for all visualization tasks
 *
 */

#include "monocular_pose_estimator_lib/visualization.h"
#include <stdio.h>
#include "ros/ros.h"

namespace monocular_pose_estimator
{

// Function that projects the RGB orientation vectors back onto the image
void Visualization::projectOrientationVectorsOnImage(cv::Mat &image, const std::vector<cv::Point3f> points_to_project,
                                                     const cv::Mat camera_matrix_K,
                                                     const std::vector<double> camera_distortion_coeffs)
{

  std::vector<cv::Point2f> projected_points;

  // 0 rotation
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);

  // 0 translation
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  projectPoints(points_to_project, rvec, tvec, camera_matrix_K, camera_distortion_coeffs, projected_points);

  cv::line(image, projected_points[0], projected_points[1], CV_RGB(255, 0, 0), 2);
  cv::line(image, projected_points[0], projected_points[2], CV_RGB(0, 255, 0), 2);
  cv::line(image, projected_points[0], projected_points[3], CV_RGB(0, 0, 255), 2);

}

void Visualization::createVisualizationImage(cv::Mat &image, Eigen::Matrix4d transform, const cv::Mat camera_matrix_K,
                                             const std::vector<double> camera_distortion_coeffs,
                                             cv::Rect region_of_interest,
                                             std::vector<cv::Point2f> distorted_detection_centers)
{
  const double orientation_vector_length = 0.075; //!< Length of the orientation trivectors that will be projected onto the output image

  Eigen::Matrix4d orientation_vector_points; // Matrix holding the points for the orientation trivector that will be projected onto the output image in the object body frame of reference
  orientation_vector_points.col(0) << 0, 0, 0, 1;
  orientation_vector_points.col(1) << orientation_vector_length, 0, 0, 1;
  orientation_vector_points.col(2) << 0, orientation_vector_length, 0, 1;
  orientation_vector_points.col(3) << 0, 0, orientation_vector_length, 1;

  Eigen::Matrix4d visualisation_pts = transform * orientation_vector_points;

  std::vector<cv::Point3f> points_to_project;
  points_to_project.resize(4);

  points_to_project[0].x = visualisation_pts(0, 0);
  points_to_project[0].y = visualisation_pts(1, 0);
  points_to_project[0].z = visualisation_pts(2, 0);
  points_to_project[1].x = visualisation_pts(0, 1);
  points_to_project[1].y = visualisation_pts(1, 1);
  points_to_project[1].z = visualisation_pts(2, 1);
  points_to_project[2].x = visualisation_pts(0, 2);
  points_to_project[2].y = visualisation_pts(1, 2);
  points_to_project[2].z = visualisation_pts(2, 2);
  points_to_project[3].x = visualisation_pts(0, 3);
  points_to_project[3].y = visualisation_pts(1, 3);
  points_to_project[3].z = visualisation_pts(2, 3);

  projectOrientationVectorsOnImage(image, points_to_project, camera_matrix_K, camera_distortion_coeffs);

  // Draw a circle around the detected LED
  for (int i = 0; i < distorted_detection_centers.size(); i++)
  {
    cv::circle(image, distorted_detection_centers[i], 10, CV_RGB(255, 0, 0), 2);
  }

  // Draw region of interest
  cv::rectangle(image, region_of_interest, CV_RGB(0, 0, 255), 2);
}

} // namespace
