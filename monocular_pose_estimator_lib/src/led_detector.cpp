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
 * led_detector.cpp
 *
 * Created on: July 29, 2013
 * Author: Karl Schwabe
 */

/**
 * \file led_detector.cpp
 * \brief File containing the function definitions required for detecting LEDs and visualising their detections and the pose of the tracked object.
 *
 */

#include "monocular_pose_estimator_lib/led_detector.h"

namespace monocular_pose_estimator
{

// LED detector
void LEDDetector::findLeds(const cv::Mat &image, cv::Rect ROI, const int &threshold_value, const double &gaussian_sigma,
                           const double &min_blob_area, const double &max_blob_area,
                           const double &max_width_height_distortion, const double &max_circular_distortion,
                           List2DPoints &pixel_positions, std::vector<cv::Point2f> &distorted_detection_centers,
                           const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                           const cv::Mat &camera_matrix_P)
{
  // Threshold the image
  cv::Mat bw_image;
  //cv::threshold(image, bwImage, threshold_value, 255, cv::THRESH_BINARY);
  cv::threshold(image(ROI), bw_image, threshold_value, 255, cv::THRESH_TOZERO);

  // Gaussian blur the image
  cv::Mat gaussian_image;
  cv::Size ksize; // Gaussian kernel size. If equal to zero, then the kerenl size is computed from the sigma
  ksize.width = 0;
  ksize.height = 0;
  GaussianBlur(bw_image.clone(), gaussian_image, ksize, gaussian_sigma, gaussian_sigma, cv::BORDER_DEFAULT);

  //cv::imshow( "Gaussian", gaussian_image );

  // Find all contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  unsigned numPoints = 0; // Counter for the number of detected LEDs

  // Vector for containing the detected points that will be undistorted later
  std::vector<cv::Point2f> distorted_points;

  // Identify the blobs in the image
  for (unsigned i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]); // Blob area
    cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
    double radius = (rect.width + rect.height) / 4; // Average radius

    cv::Moments mu;
    mu = cv::moments(contours[i], false);
    cv::Point2f mc;
    mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);

    // Look for round shaped blobs of the correct size
    if (area >= min_blob_area && area <= max_blob_area
        && std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width))
            <= max_width_height_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2)))) <= max_circular_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2)))) <= max_circular_distortion)
    {
      distorted_points.push_back(mc);
      numPoints++;
    }
  }

  // These will be used for the visualization
  distorted_detection_centers = distorted_points;

  if (numPoints > 0)
  {
    // Vector that will contain the undistorted points
    std::vector<cv::Point2f> undistorted_points;

    // Undistort the points
    cv::undistortPoints(distorted_points, undistorted_points, camera_matrix_K, camera_distortion_coeffs, cv::noArray(),
                        camera_matrix_P);

    // Resize the vector to hold all the possible LED points
    pixel_positions.resize(numPoints);

    // Populate the output vector of points
    for (unsigned j = 0; j < numPoints; ++j)
    {
      Eigen::Vector2d point;
      point(0) = undistorted_points[j].x;
      point(1) = undistorted_points[j].y;
      pixel_positions(j) = point;
    }
  }
}

cv::Rect LEDDetector::determineROI(List2DPoints pixel_positions, cv::Size image_size, const int border_size,
                                   const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                                   const cv::Mat &camera_matrix_P)
{
  double x_min = INFINITY;
  double x_max = 0;
  double y_min = INFINITY;
  double y_max = 0;

  for (unsigned i = 0; i < pixel_positions.size(); ++i)
  {
    if (pixel_positions(i)(0) < x_min)
    {
      x_min = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(0) > x_max)
    {
      x_max = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(1) < y_min)
    {
      y_min = pixel_positions(i)(1);
    }
    if (pixel_positions(i)(1) > y_max)
    {
      y_max = pixel_positions(i)(1);
    }
  }

  std::vector<cv::Point2f> undistorted_points;

  undistorted_points.push_back(cv::Point2f(x_min, y_min));
  undistorted_points.push_back(cv::Point2f(x_max, y_max));

  std::vector<cv::Point2f> distorted_points;

  // Distort the points
  distortPoints(undistorted_points, distorted_points, camera_matrix_K, camera_distortion_coeffs, camera_matrix_P);

  double x_min_dist = distorted_points[0].x;
  double y_min_dist = distorted_points[0].y;
  double x_max_dist = distorted_points[1].x;
  double y_max_dist = distorted_points[1].y;

  double x0 = std::max(0.0, std::min((double)image_size.width, x_min_dist - border_size));
  double x1 = std::max(0.0, std::min((double)image_size.width, x_max_dist + border_size));
  double y0 = std::max(0.0, std::min((double)image_size.height, y_min_dist - border_size));
  double y1 = std::max(0.0, std::min((double)image_size.height, y_max_dist + border_size));

  cv::Rect region_of_interest;

  // if region of interest is too small, use entire image
  // (this happens, e.g., if prediction is outside of the image)
  if (x1 - x0 < 1 || y1 - y0 < 1)
  {
    region_of_interest = cv::Rect(0, 0, image_size.width, image_size.height);
  }
  else
  {
    region_of_interest.x = x0;
    region_of_interest.y = y0;
    region_of_interest.width = x1 - x0;
    region_of_interest.height = y1 - y0;
  }

  return region_of_interest;
}

void LEDDetector::distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst,
                                const cv::Mat & camera_matrix_K, const std::vector<double> & distortion_matrix,
                                const cv::Mat & projection_matrix_P)
{
  dst.clear();
  double fx_K = camera_matrix_K.at<double>(0, 0);
  double fy_K = camera_matrix_K.at<double>(1, 1);
  double cx_K = camera_matrix_K.at<double>(0, 2);
  double cy_K = camera_matrix_K.at<double>(1, 2);

  double fx_P = projection_matrix_P.at<double>(0, 0);
  double fy_P = projection_matrix_P.at<double>(1, 1);
  double cx_P = projection_matrix_P.at<double>(0, 2);
  double cy_P = projection_matrix_P.at<double>(1, 2);

  double k1 = distortion_matrix[0];
  double k2 = distortion_matrix[1];
  double p1 = distortion_matrix[2];
  double p2 = distortion_matrix[3];
  double k3 = distortion_matrix[4];

  for (unsigned int i = 0; i < src.size(); i++)
  {
    // Project the points into the world
    const cv::Point2d &p = src[i];
    double x = (p.x - cx_P) / fx_P;
    double y = (p.y - cy_P) / fy_P;
    double xCorrected, yCorrected;

    // Correct distortion
    {
      double r2 = x * x + y * y;

      // Radial distortion
      xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

      // Tangential distortion
      xCorrected = xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
      yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);
    }

    // Project coordinates onto image plane
    {
      xCorrected = xCorrected * fx_K + cx_K;
      yCorrected = yCorrected * fy_K + cy_K;
    }
    dst.push_back(cv::Point2d(xCorrected, yCorrected));
  }
}

} // namespace
