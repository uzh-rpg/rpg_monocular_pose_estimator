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
 * led_detector.h
 *
 *  Created on: Jul 29, 2013
 *      Author: karl Schwabe
 */

/**
 * \file led_detector.h
 * \brief File containing the includes and the function prototypes for the led_detector.cpp file.
 *
 */

#ifndef LEDDETECTOR_H_
#define LEDDETECTOR_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <algorithm>

namespace monocular_pose_estimator
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints;

class LEDDetector
{
public:
  /**
   * Detects the LEDs in the image and returns their positions in the rectified image and also draws circles around the detected LEDs in an output image.
   *
   * The image is thresholded where everything smaller than the threshold is set to zero. I.e.,
   * \f[
   \mathbf{I}^\prime (u, v) = \begin{cases} \mathbf{I}(u, v), & \mbox{if } \mathbf{I}(u, v) > \mbox{threshold}, \\ 0, & \mbox{otherwise}. \end{cases}
   * \f]
   *
   *
   * Thereafter the image is Gaussian smoothed. Contours are extracted and their areas are determined. Blobs with
   * too large, or too small an area are ignored. Blobs that are not sufficiently round are also ignored.
   *
   * The centre of the extracted blobs is calculated using the method of moments. The centre \f$(\hat{u}, \hat{v})\f$ is calculated as \n
   * \f{eqnarray*}{
   *       \hat{u} & = & M_{10} / M_{00}, \\
   *       \hat{v} & = & M_{01} / M_{00}.
   * \f}
   * where the image moments are defined as
   * \f[
   *      M_{pq} = \sum_u \sum_v u^p v^q I^\prime (u,v)
   * \f]
   *
   *
   * \param image the image in which the LEDs are to be detected
   * \param ROI the region of interest in the image to which the image search will be confined
   * \param threshold_value the threshold value for the LED detection
   * \param gaussian_sigma standard deviation \f$\sigma\f$ of the Gaussian that is to be applied to the thresholded image
   * \param min_blob_area the minimum blob area to be detected (in pixels squared)
   * \param max_blob_area the maximum blob area to be detected (in pixels squared)
   * \param max_width_height_distortion the maximum distortion ratio of the width of the blob to the height of the blob. Calculated as \f$1-\frac{\mbox{width}}{\mbox{height}}\f$
   * \param max_circular_distortion the maximum distortion circular distortion ratio, calculated as \f$ 1- \frac{\mbox{area}}{(\mbox{width}/2)^2} \f$ or \f$ 1- \frac{\mbox{area}}{(\mbox{height}/2)^2} \f$
   * \param pixel_positions (output) the vector containing the pixel positions of the centre of the detected blobs
   * \param distorted_detection_centers (output) centers of the LED detection in the distorted image (only used for visualization)
   * \param camera_matrix_K the camera calibration matrix (without distortion correction) \f$K = \left[ \begin{array}{ccc}
   *                         f_x & 0 & c_x \\
   *                         0 & f_y & c_y \\
   *                         0 & 0 & 1 \end{array} \right] \f$
   * \param camera_distortion_coeffs the vector containing the 4, 5 or 8 distortion coefficients of the camera \f$ (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) \f$
   *
   */
  static void findLeds(const cv::Mat &image, cv::Rect ROI, const int &threshold_value, const double &gaussian_sigma,
                       const double &min_blob_area, const double &max_blob_area,
                       const double &max_width_height_distortion, const double &max_circular_distortion,
                       List2DPoints &pixel_positions, std::vector<cv::Point2f> &distorted_detection_centers,
                       const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs);

  /**
   * Calculates the region of interest (ROI) in the distorted image in which the points lie.
   *
   * The pixel positions of the points in the undistorted image are bounded with a box. The corner points of the rectangle are
   * undistorted to produce a new region of interest in the distorted image. This region of interest is returned.
   *
   * \param pixel_positions position of the points in the undistorted image
   * \param image_size size of the image
   * \param border_size size of the boarder around the bounding box of points
   * \param camera_matrix_K camera matrix
   * \param camera_distortion_coeffs the distortion coefficients of the camera
   *
   * \return the rectangular region of interest to be processed in the image
   *
   */
  static cv::Rect determineROI(List2DPoints pixel_positions, cv::Size image_size, const int border_size,
                               const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs);

private:

  /**
   * Distorts points. It is the approximate inverse of the undistortPoints() function in openCV.
   *
   * The function takes a vector of rectified points and distorts them according to the distortion matrix and returns the position of the distorted points.
   *
   *
   * For the radial distortion we have:
   * \f{eqnarray*}{
   *      x_{\mbox{corrected}} &=& x \left( 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 \right) \\
   *      y_{\mbox{corrected}} &=& y \left( 1 + k_1 r^2 + k_2 r^4 + k_3 r^6 \right)
   * \f}
   *
   * For the tangential distortion we have:
   * \f{eqnarray*}{
   *      x_{\mbox{corrected}} &=& x + \left( 2 + p_1 x y + p_2 \left( r^2 + 2 x^2 \right) \right) \\
   *      y_{\mbox{corrected}} &=& y + \left( p_1 \left( r^2 + 2 y^2 \right) + 2 p_2 x y \right)
   * \f}
   * where \f$r^2=x^2+y^2\f$.
   *
   * \param src vector of undistorted/rectified points
   * \param dst (output) vector to which the distorted points will be output
   * \param camera_matrix_K camera matrix
   * \param distortion_matrix the distortion coefficients of the camera
   *
   */
  static void distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst,
                            const cv::Mat & camera_matrix_K, const std::vector<double> & distortion_matrix);

};

} // namespace

#endif /* LEDDETECTOR_H_ */
