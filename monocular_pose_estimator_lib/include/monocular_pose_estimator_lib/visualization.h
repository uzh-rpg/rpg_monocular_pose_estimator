/*
 * visualization.h
 *
 *  Created on: Mar 21, 2014
 *      Author: Matthias Faessler
 */

/**
 * \file visualization.h
 * \brief File containing the includes and the function prototypes for the visualization.cpp file.
 *
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace monocular_pose_estimator
{

class Visualization
{
public:
  /**
   * Projects the body-fixed coordinate frame onto the image.
   *
   * The function takes a list of 4 vector points, the fist one being the origin of the coordinate frame and the
   * remaining 3 points being the coordinates of the end of the illustrative vectors of the coordinate frame. It projects
   * the lines/vectors illustrating the coordinate frame onto the image.
   *
   * \param image (input & output) the image onto which the coordinate frame will be projected.
   * \param points_to_project the four 3D points that are to be projected onto the image and connected together to form the coordinate frame.\n
   *         - The first point is the origin of the coordinate frame\n
   *         - The second point is the vector from the origin along the x-axis\n
   *         - The third point is the vector from the origin along the y-axis\n
   *         - The fourth point is the vector from the origin along the z-axis
   * \param camera_matrix_K the camera calibration matrix (without distortion correction) = \f$ \left[ \begin{array}{ccc}
   *                         f_x & 0 & c_x \\
   *                         0 & f_y & c_y \\
   *                         0 & 0 & 1 \end{array} \right] \f$
   * \param camera_distortion_coeffs the vector containing the 4, 5 or 8 distortion coefficients of the camera \f$ (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) \f$
   *
   */
  static void projectOrientationVectorsOnImage(cv::Mat &image, const std::vector<cv::Point3f> points_to_project,
                                               const cv::Mat camera_matrix_K,
                                               const std::vector<double> camera_distortion_coeffs);

  static void createVisualizationImage(cv::Mat &image, Eigen::Matrix4d transform, const cv::Mat camera_matrix_K,
                                       const std::vector<double> camera_distortion_coeffs, cv::Rect region_of_interest,
                                       std::vector<cv::Point2f> distorted_detection_centers);

};

} // namespace

#endif /* VISUALIZATION_H_ */
