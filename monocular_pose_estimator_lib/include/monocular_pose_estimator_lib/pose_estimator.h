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
 * PoseEstimator.h
 *
 *  Created on: Jul 29, 2013
 *      Author: Karl Schwabe
 */

/** \file pose_estimator.h
 * \brief File containing the headers and prototypes for the PoseEstimator class.
 *
 */

#ifndef POSEESTIMATOR_H_
#define POSEESTIMATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#include "monocular_pose_estimator_lib/datatypes.h"
#include "monocular_pose_estimator_lib/led_detector.h"
#include "monocular_pose_estimator_lib/visualization.h"
#include "monocular_pose_estimator_lib/combinations.h"
#include "monocular_pose_estimator_lib/p3p.h"
#include <iostream>
#include <opencv2/opencv.hpp>

namespace monocular_pose_estimator
{

/**
 * The PoseEstimator class for tracking objects with infrared LEDs and an infrared camera.
 *
 * This class produces instances of objects that can be tracked with infrared LEDs and an infrared camera.
 *
 */
class PoseEstimator
{

private:
  Eigen::Matrix4d current_pose_; //!< Homogeneous transformation matrix for storing the current pose of the tracked object
  Eigen::Matrix4d previous_pose_; //!< Homogeneous transformation matrix for storing the previously estimated pose of the tracked object
  Eigen::Matrix4d predicted_pose_; //!< Homogeneous transformation matrix for storing the predicted pose of the object. \see predictPose
  Matrix6d pose_covariance_; //!< A 6x6 covariance matrix that stores the covariance of the calculated pose
  double current_time_; //!< Stores the time of the current pose
  double previous_time_; //!< Stores the time of the previous pose
  double predicted_time_; //!< Stores the time of the predicted pose
  List4DPoints object_points_; //!< Stores the positions of the makers/LEDs on the object being tracked in the object-fixed coordinate frame using homogeneous coordinates. It is a vector of 4D points.
  List2DPoints image_points_; //!< Stores the positions of the detected marker points found in the image. It is a vector of 2D points.
  List2DPoints predicted_pixel_positions_; //!< Stores the predicted pixel positions of the markers in the image. \see predictMarkerPositionsInImage
  List3DPoints image_vectors_; //!< Stores the unit vectors leading from the camera centre of projection out to the world points/marker points - these are used in the P3P algorithm. \see setImagePoints, calculateImageVectors
  VectorXuPairs correspondences_; //!< Stores the correspondences of the LED markers and the detected markers in the image
  Matrix3x4d camera_projection_matrix_; //!< Stores the camera calibration matrix. This is the 3x4 projection matrix that projects the world points to the image coordinates stored in #image_points_.
  double back_projection_pixel_tolerance_; //!< Stores the pixel tolerance that determines whether a back projection is valid or not. \see checkCorrespondences, initialise
  double nearest_neighbour_pixel_tolerance_; //!< Stores the pixel tolerance that determines the correspondences between the LEDs and the detections in the image when predicting the position of the LEDs in the image. \see findCorrespondences
  double certainty_threshold_; //!< Stores the ratio of how many of the back-projected points must be within the #back_projection_pixel_tolerance_ for a correspondence between the LEDs and the detections to be correct.
  double valid_correspondence_threshold_; //!< Stores the ratio of how many correspondences must be considered to be correct for the total correspondences to be considered correct. \see checkCorrespondences, initialise
  unsigned histogram_threshold_; //!< Stores the minimum numbers of entries in the initialisation histogram before an entry could be used to determine a correspondence between the LEDs and the image detections. \see correspondencesFromHistogram

  std::vector<cv::Point2f> distorted_detection_centers_;

  unsigned it_since_initialized_; //!< Counter to determine whether the system has been initialised already
  cv::Rect region_of_interest_; //!< OpenCV rectangle that defines the region of interest to be processd to find the LEDs in the image
  static const unsigned min_num_leds_detected_ = 4; //!< Minimum number of LEDs that need to be detected for a pose to be calculated
  bool pose_updated_;

public:
  cv::Mat camera_matrix_K_; //!< Variable to store the camera matrix as an OpenCV matrix
  cv::Mat camera_matrix_P_; //!< Variable to store the projection matrix (as an OpenCV matrix) that projects points onto the rectified image plane.
  std::vector<double> camera_distortion_coeffs_; //!< Variable to store the camera distortion parameters

  int detection_threshold_value_; //!< The current threshold value for the image for LED detection
  double gaussian_sigma_; //!< The current standard deviation of the Gaussian that will be applied to the thresholded image for LED detection
  double min_blob_area_; //!< The the minimum blob area (in pixels squared) that will be detected as a blob/LED. Areas having an area smaller than this will not be detected as LEDs.
  double max_blob_area_; //!< The the maximum blob area (in pixels squared) that will be detected as a blob/LED. Areas having an area larger than this will not be detected as LEDs.
  double max_width_height_distortion_; //!< This is a parameter related to the circular distortion of the detected blobs. It is the maximum allowable distortion of a bounding box around the detected blob calculated as the ratio of the width to the height of the bounding rectangle. Ideally the ratio of the width to the height of the bounding rectangle should be 1.
  double max_circular_distortion_; //!< This is a parameter related to the circular distortion of the detected blobs. It is the maximum allowable distortion of a bounding box around the detected blob, calculated as the area of the blob divided by pi times half the height or half the width of the bounding rectangle.
  unsigned roi_border_thickness_; //!< This is the thickness of the boarder (in pixels) around the predicted area of the LEDs in the image that defines the region of interest for image processing and detection of the LEDs.

private:

  /**
   * Calculates the unit direction vectors leading from the camera centre of projection out to the world points/marker points - these are used in the P3P algorithm.
   *
   * The unit vectors are stored in #image_vectors_.
   *
   */
  void calculateImageVectors();

  /**
   * Calculates the Squared reprojection error between the image points and the back-projected object points.
   *
   * \param image_pts vector of image points
   * \param object_pts vector of the back-projected image points
   * \param certainty (output) the ratio of back-projected image points that are within the #back_projection_pixel_tolerance_
   *
   * \note The order of the image and the back projected image points does not matter.
   *
   * \return the squared reprojection error
   *
   */
  double calculateSquaredReprojectionErrorAndCertainty(const List2DPoints & image_pts, const List2DPoints & object_pts,
                                                       double & certainty);

  /**
   * Calculates the LED and image detection correspondences from the initialisation histogram.
   *
   * \param histogram the initialisation histogram.
   *
   * \return the correspondences between the LEDs and the image detections
   *
   * \see initialise
   *
   */
  VectorXuPairs correspondencesFromHistogram(MatrixXYu & histogram);

  /**
   * Calculates the minimum distances between two sets of 2D points and their pairs.
   *
   * The matrix of \a pairs returned contains the element number in the set \a points_a in the first column and the element
   * number of the point in set \a points_b to which it is closes to in the second column. The indexes are stored with one-based
   * counting.
   *
   * \param points_a first set of 2D points
   * \param points_b second set of 2D points
   * \param min_distances (output) vector of the minimum distances from the points in the set \a points_a to the points in set \a points_b
   *
   * \returns matrix of point pairs
   *
   * \note The indexes returned in the matrix of point pairs are stores with one-based counting.
   *
   */
  VectorXuPairs calculateMinDistancesAndPairs(const List2DPoints & points_a, const List2DPoints & points_b,
                                              Eigen::VectorXd & min_distances);

  /**
   * Calculates the squared distance between two points.
   *
   * \param p1 first point
   * \param p2 second point
   *
   * \returns the squared distance between point \a p1 and point \a p2
   *
   */
  template<typename DerivedA, typename DerivedB>
    double squareDist(const Eigen::MatrixBase<DerivedA>& p1, const Eigen::MatrixBase<DerivedB>& p2);

  /**
   * Checks whether a matrix contains only finite numbers.
   *
   * \param x matrix to be checked
   *
   * \returns
   *    - \b true if the matrix contains only finite numbers
   *    - \b false if the matrix contains complex or non-finite numbers, e.g. NaN
   *
   */
  template<typename Derived>
    bool isFinite(const Eigen::MatrixBase<Derived>& x);

  /**
   * Computes the transformation between two sets of points.
   *
   * \param object_points the set of point in the first coordinate frame
   * \param reprojected_points the point in the second coordinate frame
   *
   * \note Each column in \a object_points and \a reprojected_points contains a point.
   *
   * \returns homogeneous transformation matrix from the \a object_points coordinate frame to the \a reprojected_points coordinate frame of the form
   * \f$
   *    \left[ \begin{array}{cc}
   *    \mathbf{R} & \mathbf{t} \\
   *     0 & 1
   *     \end{array} \right]
   * \f$
   *
   */
  Eigen::Matrix4d computeTransformation(const MatrixXYd & object_points, const MatrixXYd & reprojected_points);

  /**
   * Computes the Jacobian of the projection from points in the object-fixed coordinate frame to the camera image plane.
   *
   * The projection from the object-fixed coordinates \f$\mathbf{x}\f$ to camera image plane coordinates \f$(u,v)\f$ is denoted by \f$h_{T_{co}}\f$. It is given by
   * \f[
   *      \left( \begin{array}{c}
   *               u \\ v
   *      \end{array} \right) = \mathbf{h}_{T_{co}}(\mathbf{x}) = \mbox{project}(T_{co} \mathbf{x})
   *
   * \f]
   * where \f$T_{co} \f$ is the transformation from object-fixed coordinate frame to the camera-fixed coordinate frame, \f$\mathbf{x}\f$ is the vector of points \f$x\f$, \f$y\f$, \f$z\f$ in the object-fixed coordinate frame, and the function \f$\mbox{project}\f$ is the projection of the points in the camera-fixed coordinate frame onto the camera image plane.
   *
   * The Jacobian with respect to the transformation matrix \f$T_{co}\f$ is calculated according to equation A.14 from the the PhD thesis of Ethan Eade (http://ethaneade.com/) \cite Eade:2008.
   * \f[
   *      J_{T_{co}} = \frac{\partial \mathbf{h}_{T_co}(\mathbf{x})}{\partial T_{co}} = \left[
   *      \begin{array}{cccccc}
   *              \frac{1}{z} \cdot f_x & 0                     & -\frac{x}{z^2} \cdot f_x & -\frac{xy}{z^2} \cdot f_x         & (1 + \frac{x^2}{z^2}) \cdot f_x & -\frac{y}{z} \cdot f_x \\
   *              0                     & \frac{1}{z} \cdot f_y & -\frac{y}{z^2} \cdot f_y & -(1 + \frac{y^2}{z^2}) \cdot f_y  & \frac{x y}{z^2} \cdot f_y       & \frac{x}{z} \cdot f_y
   *      \end{array} \right]
   * \f]
   *
   * \param T_c_o the homogeneous transformation matrix from the object-fixed coordinate frame to the camera-fixed coordinate frame
   * \param world_points the homogenous world points in the object-fixed coordinate frame
   * \param focal_lengths the focal lengths \f$f_x\f$ and \f$f_y\f$ of the camera
   *
   * \returns [2x6] Jacobian matrix
   *
   */
  Matrix2x6d computeJacobian(const Eigen::Matrix4d & T_c_o, const Eigen::Vector4d & world_points,
                             const Eigen::Vector2d & focal_lengths);

  /**
   * Computes the exponential map from a twist to a homogeneous transformation matrix.
   *
   * Given a twist vector
   * \f[
   *      \xi = \left( \begin{array}{c} \upsilon \\ \omega \end{array} \right)
   * \f]
   * where \f$ \upsilon = (\upsilon_x, \upsilon_y, \upsilon_z)^\top \f$ and \f$\omega = (\omega_x, \omega_y, \omega_z)^\top \f$, the exponential map to the homogeneous transformation matrix \f$ T \f$ is given by \cite Agrawal:2006
   * \f[
   *      \mathbf{T} = \left( \begin{array}{cc}
   *                   \exp \left( \hat{\omega} \right) & \mathbf{A} \upsilon \\
   *                    0 & 1
   *                    \end{array}  \right) =
   *                    \left( \begin{array}{cc}
   *                    \mathbf{R} & t \\
   *                     0 & 1
   *                    \end{array}  \right)
   * \f]
   * where \f$ \hat{\omega} \f$ is the skew symmetric matrix of \f$ \omega \f$ (see #skewSymmetricMatrix) and
   * \f[
   *      \mathbf{A} = \mathbf{I} + \frac{1-cos\left(\| \omega \| \right)}{\| \omega \|^2}\hat{\omega} + \frac{\| \omega \| - \sin \left( \omega \right)}{\| \omega \|^3} \hat{\omega}^2
   * \f]
   * and \f$\exp \left( \hat{\omega} \right)\f$ is given by the Rodrigues formula \cite Agrawal:2006 \cite Hartley:2003 \cite Murray:1994
   * \f[
   *      \exp \left( \hat{\omega} \right) = \mathbf{I} + \frac{\sin \left( \| \omega \| \right)}{\| \omega \|}\hat{\omega} + \frac{1 - \cos \left( \omega \right)}{\| \omega \|^2} \hat{\omega}^2
   * \f]
   *
   *
   * \param twist the twist, represented as \f$ \left( \begin{array}{c} \upsilon_x \\ \upsilon_y \\ \upsilon_z \\ \omega_x \\ \omega_y \\ \omega_z \end{array} \right) \f$
   *
   * \return the homogeneous transformation matrix
   *
   * \see logarithmMap, skewSymmetricMatrix
   *
   */
  Eigen::Matrix4d exponentialMap(const Vector6d & twist);

  /**
   * Computes the logarithm map from a homogeneous transformation matrix to twist coordinates.
   *
   * Given a homogeneous transformation matrix
   * \f[
   *      \xi = \left( \begin{array}{cc} \mathbf{R} & t \\ 0 & 1 \end{array} \right)
   * \f]
   * where \f$ \mathbf{R} \in SO(3) \f$ and \f$ t \in \mathbb{R}^3 \f$, the logarithm map to the twist matrix \f$ \hat{\xi} \f$ is given by \cite Agrawal:2006
   * \f[
   *      \hat{\xi} = \left( \begin{array}{cc}
   *                   \log \left( \mathbf{R} \right) & \mathbf{A}^{-1} t \\
   *                    0 & 0
   *                    \end{array}  \right) =
   *                    \left( \begin{array}{cc}
   *                    \hat{\omega} & \upsilon \\
   *                     0 & 0
   *                    \end{array}  \right)
   * \f]
   * where
   * \f[
   *      \log \left( \mathbf{R} \right) = \frac{\phi}{2 \sin \left( \phi \right)} \left( \mathbf{R} - \mathbf{R}^\top \right) \equiv \hat{\omega}
   * \f]
   * and \f$ \phi \f$ satisfies
   * \f[
   *      \textrm{trace} \left( \mathbf{R} \right) = 1 + 2 \cos \left( \phi \right), | \phi | < \pi
   * \f]
   * and where
   *\f[
   *      \mathbf{A}^{-1} = \mathbf{I} - \frac{1}{2} \hat{\omega} + \frac{2 \sin \| \omega \| - \|\omega \| \left( 1 + \cos \| \omega\| \right)}{2 \| \omega \|^2 \sin \| \omega \|} \hat{\omega}^2
   * \f]

   *
   *
   * \param trans the homogeneous transformation matrix, represented as \f$ \left( \begin{array}{cc} \mathbf{R} & t \\ 0 & 1 \end{array} \right) \f$
   *
   * \return the twist coordinates vector \f$ \xi = \left( \begin{array}{c} \upsilon \\ \omega \end{array} \right) \f$, where \f$ \upsilon = \left( \upsilon_x, \upsilon_y, \upsilon_z \right)^\top \f$ and \f$ \omega =  \left( \omega_x, \omega_y, \omega_z \right)^\top \f$
   *
   * \see exponentialMap, skewSymmetricMatrix
   *
   */
  Vector6d logarithmMap(const Eigen::Matrix4d & trans);

  /**
   * Creates a skew symmetric matrix from the a vector of length 3.
   *
   * The skew symmetric matrix \f$\hat{\omega}\f$ is constructed from the vector \f$\mathbf{\omega}\f$ as follows
   * \f[
   *      \hat{\omega} = \left[ \begin{array}{ccc}
   *                              0          & -\omega_z & \omega_y \\
   *                              \omega_z   & 0         & -\omega_x \\
   *                              -\omega_y & \omega_x & 0
   *                            \end{array} \right]
   * \f]
   * where \f$\omega_x\f$, \f$\omega_y\f$, \f$\omega_z\f$ are the three components of the vector \f$\mathbf{\omega}\f$.
   *
   * \param w the vector from which the skew symmetric matrix is to be created
   *
   * \return the skew symmetric matrix of the vector \b w
   *
   */
  Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d w);

  /**
   * Finds the element with the largest absolute value in a vector.
   *
   * \param v vector containing the elements
   *
   * \return element with largest absolute value
   *
   */
  double norm_max(const Eigen::VectorXd & v);

public:
  /**
   * Constructor.
   *
   */
  PoseEstimator();

  void augmentImage(cv::Mat &image);

  /**
   * Sets the positions of the markers on the object.
   *
   * \param positions_of_markers_on_object vector containing the position of the LEDs/markers on the object in the object-fixed coordinate frame. The coordinates are given in homogeneous coordinates.
   *
   * \see object_points
   *
   */
  void setMarkerPositions(List4DPoints positions_of_markers_on_object);

  /**
   * Returns the positions of the markers on the object in the object-fixed coordinate frame
   *
   * \return vector containing the position of the LEDs/markers on the object in the object-fixed coordinate frame. The coordinates are given in homogeneous coordinates.
   *
   * \see object_points
   *
   */
  List4DPoints getMarkerPositions();

  /**
   * Estimates the pose of the tracked object
   */
  bool estimateBodyPose(cv::Mat image, double time_to_predict);

  /**
   * Sets the time at which the pose will be calculated.
   *
   * \param time the time of the predicted pose
   *
   * \see predicted_time
   *
   */
  void setPredictedTime(double time);

  /**
   * Returns the time for which the predicted pose was calculated.
   *
   * \return the time of the predicted pose
   *
   * \see predicted_time
   *
   */
  double getPredictedTime();

  /**
   * Sets the predicted pose of the object.
   *
   * \param pose the homogeneous pose of the object with respect to the camera
   * \param time the time of the predicted pose
   *
   * \see predicted_pose predicted_time
   */
  void setPredictedPose(const Eigen::Matrix4d & pose, double time);

  /**
   * Returns the predicted pose of the object.
   *
   * \return the homogeneous pose of the object with respect to the camera
   *
   * \see predicted_pose
   *
   */
  Eigen::Matrix4d getPredictedPose();

  /**
   * Returns the covariance of the predicted pose ot the camera.
   *
   * \return the 6x6 covariance of the predicted pose
   *
   * \see covariance
   *
   */
  Matrix6d getPoseCovariance();

  /**
   * Sets the image points of the markers/LEDs that have been detected in the image.
   *
   * \param points the vector containing the image points of the detected LEDs/markers in the image.
   *
   * \see image_points
   */
  void setImagePoints(List2DPoints points);

  /**
   * Returns the image points of the detected LEDs/markers.
   *
   * \return vector containing the image positions of the detected LEDs/markers in the image
   *
   */
  List2DPoints getImagePoints();

  /**
   * Sets the predicted position of the LEDs/markers in the image.
   *
   * \param points vector containing the image coordinates of the predicted position of the makers/LEDs.
   *
   * \see predictMarkerPositionsInImage
   *
   */
  void setPredictedPixels(List2DPoints points);

  /**
   * Returns the predicted position of the LEDs/markers in the image.
   *
   * \return vector containing the image coordinates of the predicted position of the makers/LEDs.
   *
   * \see predictMarkerPositionsInImage
   *
   */
  List2DPoints getPredictedPixelPositions();

  /**
   * Sets the camera matrix.
   *
   * \param M the 3x4 homogeneous camera projection matrix that projects points in the camera-fixed coordinate frame onto the camera image plane
   *
   * \see camera_matrix
   *
   */
  void setCameraProjectionMatrix(Matrix3x4d M);

  /**
   * Returns the camera matrix.
   *
   * \return the 3x4 homogeneous camera projection matrix that projects points in the camera-fixed coordinate frame onto the camera image plane
   *
   * \see camera_matrix
   *
   */
  Matrix3x4d getCameraProjectionMatrix();

  /**
   * Sets the correspondences between the LEDs/markers on the object and the image detections.
   *
   * \param corrs the corresponces between the LEDS/marker positions (first column) and the image detections (second column) using one-based counting
   *
   * \see correspondences
   *
   */
  void setCorrespondences(VectorXuPairs corrs);

  /**
   * Returns the correspondences between the LEDs/markers on the object and the image detections.
   *
   * \return the corresponces between the LEDS/marker positions (first column) and the image detections (second column) using one-based counting
   *
   * \see correspondences
   *
   */
  VectorXuPairs getCorrespondences();

  /**
   * Sets the back-projection pixel tolerance that is used to determine whether an LED and image detection correspondence is correct.
   *
   * \param tolerance the back-projection pixel tolerance
   *
   * \see back_projection_pixel_tolerance checkCorrespondences
   *
   */
  void setBackProjectionPixelTolerance(double tolerance);

  /**
   * Returns the back-projection pixel tolerance that is used to determine whether an LED and image detection correspondence is correct.
   *
   * \return tolerance the back-projection pixel tolerance
   *
   * \see back_projection_pixel_tolerance checkCorrespondences
   *
   */
  double getBackProjectionPixelTolerance();

  /**
   * Sets the nearest-neighbour pixel tolerance that is used to determine whether the predicted position of a marker corresponds to a detection in the image.
   *
   * \param tolerance the nearest-neighbour pixel tolerance
   *
   * \see nearest_neighbour_pixel_tolerance findCorrespondences
   *
   */
  void setNearestNeighbourPixelTolerance(double tolerance);

  /**
   * Returns the nearest-neighbour pixel tolerance that is used to determine whether the predicted position of a marker corresponds to a detection in the image.
   *
   * \return tolerance the nearest-neighbour pixel tolerance
   *
   * \see nearest_neighbour_pixel_tolerance findCorrespondences
   *
   */
  double getNearestNeighbourPixelTolerance();

  /**
   * Sets the ratio of how many of the back-projected points must be within the #back_projection_pixel_tolerance_ for a correspondence between the LEDs and the detections to be correct.
   *
   * \param threshold the certainty threshold
   *
   * \see certainty_threshold
   *
   */
  void setCertaintyThreshold(double threshold);

  /**
   * Returns the ratio of how many of the back-projected points must be within the #back_projection_pixel_tolerance_ for a correspondence between the LEDs and the detections to be correct.
   *
   * \return the certainty threshold
   *
   * \see certainty_threshold
   *
   */
  double getCertaintyThreshold();

  /**
   * Sets the ratio of how many correspondences must be considered to be correct for the total correspondences to be considered correct.
   *
   * \param threshold valid_correspondence_threshold
   *
   * \see valid_correspondence_threshold
   *
   */
  void setValidCorrespondenceThreshold(double threshold);

  /**
   * Returns the ratio of how many correspondences must be considered to be correct for the total correspondences to be considered correct.
   *
   * \returns valid_correspondence_threshold
   *
   * \see valid_correspondence_threshold
   *
   */
  double getValidCorrespondenceThreshold();

  /**
   * Sets the minimum numbers of entries in the initialisation histogram before an entry could be used to determine a correspondence between the LEDs and the image detections.
   *
   * \param threshold the histogram_threshold
   *
   * \see histogram_threshold
   *
   */
  void setHistogramThreshold(unsigned threshold);

  /**
   * Returns the minimum numbers of entries in the initialisation histogram before an entry could be used to determine a correspondence between the LEDs and the image detections.
   *
   * \return the histogram_threshold
   *
   * \see histogram_threshold
   *
   */
  unsigned getHistogramThreshold();

  /**
   * Projects points onto the image plane.
   *
   * The transformation from \b point coordinate frame to the camera-fixed coordinate frame is given by
   * \f[
   *    \mathbf{y} = \mathbf{T}_{co} \mathbf{x}
   * \f]
   * where \f$\mathbf{x}\f$ is the point to be projected onto the image plane and \f$\mathbf{T_{co}}\f$ is the homogeneous transformation from the \b point coordinate frame to the camera coordinate frame and the \f$\mathbf{y}\f$ are the homogeneous coordinates of the point in the camera-fixed coordinate frame.
   *
   *
   * The projection of a point in the camera-fixed coordinate frame \f$\mathbf{y}\f$ onto the image plane is given by
   * \f{eqnarray*}{
   *            u & = & \frac{x}{z} f_x + c_x \\
   *            v & = & \frac{y}{z} f_y + c_y
   * \f}
   * where \f$x\f$, \f$y\f$ and \f$z\f$ are the components of \f$\mathbf{y}\f$ and \f$f_x\f$ and \f$f_y\f$ are focal lengths in the \f$x\f$ and \f$y\f$ directions of the camera, respectively, and \f$(c_x, c_y)\f$ is the principle point of the image plane in pixels.
   *
   * \param point the point in the camera-fixed frame of reference in homogeneous coordinates that is to be projected onto the camera image plane
   * \param transform the homogeneous transformation from the coordinate frame of \b point to the camera-fixed coordinate frame
   *
   * \return the image coordinates of the point projected onto the image plane
   *
   */
  inline Eigen::Vector2d project2d(Eigen::Vector4d point, Eigen::Matrix4d transform);

  /**
   * Predicts the pose of the of points in the image using a constant velocity model based on the previous two poses.
   *
   * The function uses the angle-axis representation to predict the pose of the object.
   *
   * Assuming the two previous poses are represented by the homogeneous transformation matrices \f$\mathbf{T}_{k-1}\f$ and \f$\mathbf{T}_{k-2}\f$
   * \f[
   *    \mathbf{T}_{k-1} = \left[ \begin{array}{cc}
   *                            \mathbf{R}_{k-1} & \mathbf{t}_{k-1} \\
   *                            0                & 1
   *                     \end{array} \right] \\
   *    \mathbf{T}_{k-2} = \left[ \begin{array}{cc}
   *                            \mathbf{R}_{k-2} & \mathbf{t}_{k-2} \\
   *                            0                & 1
   *                     \end{array} \right]
   * \f]
   * where \f$ \mathbf{R} \in SO(3) \f$ is a rotation matrix and \f$ \mathbf{t} \in \mathbb{R}^3 \f$ is a translation vector.
   *
   * The difference in rotation from time \f$t_{k-2}\f$ to \f$t_{k-1}\f$ is given by
   * \f[
   *    \Delta \mathbf{R}_{\left( k-2 \right) \left( k-1 \right)} = \mathbf{R}_{k-2}^\top  \mathbf{R}_{k-1}
   * \f]
   *
   * The angle of that rotation in the angle-axis representation is given by (Table 1.2 of \cite SpringerRoboticsHandbook:2008)
   * \f[
   *    \theta_{ \left( k-2 \right) \left( k-1 \right)} = \arccos \left( \frac{\mbox{trace}\left( \Delta \mathbf{R}_{\left( k-2 \right) \left( k-1 \right)} \right) - 1}{2} \right)
   * \f]
   * and the axis of rotation is given by (Table 1.2 of \cite SpringerRoboticsHandbook:2008)
   * \f[
   *    \mathbf{\omega} = \frac{1}{2\sin\theta_{ \left( k-2 \right) \left( k-1 \right)}} \left( \begin{array}{c} r_{32} - r_{23} \\ r_{13} - r_{31} \\ r_{21} - r_{12} \end{array} \right)
   * \f]
   * where \f$r_{ij}\f$ is the element from row \f$i\f$ row and column \f$j\f$ of the rotation matrix \f$\Delta \mathbf{R}_{\left( k-2 \right) \left( k-1 \right)}\f$.
   *
   * \note If the angle of rotation \f$\theta\f$ is zero, then any axis may be chosen for the rotation since it does not affect the rotation. The rotation matrix for a rotation of zero angle is always the identity matrix.
   *
   * The angle may be extrapolated to determine the estimated angle of rotation.
   * \f[
   *    \hat{\theta}_{\left( k-1 \right) \left( k \right)} = \frac{\theta_{\left( k-2 \right) \left( k-1 \right)}}{\left( t_{k-1} - t_{k-2} \right) } \cdot \left( t_k - t_{k-1} \right)
   * \f]
   * where \f$t_k\f$ is the current time at which the pose of the object is to be predicted.
   *
   * The change in rotation from \f$t_{k-2}\f$ to \f$t_k\f$ may then be given as (Equation 1.24 of \cite SpringerRoboticsHandbook:2008)
   * \f[
   *    \Delta \mathbf{R}_{\left( k-1 \right) \left( k \right)} = \mathbf{I}_{3 \times 3} + [\mathbf{\omega}]_{\times} \sin \hat{\theta}_{\left( k-2 \right) \left( k \right)} + [\mathbf{\omega}]_{\times}^2 \left( 1 - \cos \hat{\theta}_{\left( k-2 \right) \left( k \right)} \right)
   * \f]
   * where \f$[\mathbf{\omega}]_{\times}\f$ is the skew symmetric matrix of the vector \f$\mathbf{\omega}\f$. \see skewSymmetricMatrix
   * \note As mentioned above, if the angle of rotation is zero, the rotation matrix that results will be the identity matrix, irrespective of the rotation axis.
   *
   * The estimated rotation at time \f$t_k\f$ may then be estimated as
   * \f[
   *    \hat{\mathbf{R}}_k = \mathbf{R}_{k-1} \Delta \mathbf{R}_{ \left( k-1 \right) \left( k \right)}
   * \f]
   *
   * The derivative of the translation may be estimated using a constant velocity model as
   * \f[
   *    \dot{\mathbf{t}} = \frac{\mathbf{t}_{k-1}-\mathbf{t}_{k-2}}{t_{k-1} - t_{k-2}}
   * \f]
   *
   * Using this derivative the translation may be extrapolated to predict the translation at the current time
   * \f[
   *    \hat{\mathbf{t}}_k = \mathbf{t}_{k-1} + \dot{\mathbf{t}} \cdot \left( t_{k} - t_{k-1} \right)
   * \f]
   *
   * Putting these estimates together gives the estimated homogeneous transformation matrix
   * \f[
   *    \hat{\mathbf{T}}_k = \left[ \begin{array}{cc}
   *                            \hat{\mathbf{R}}_k & \hat{\mathbf{t}}_k \\
   *                            0                & 1
   *                       \end{array} \right] \\
   * \f]
   *
   *
   * \param time_to_predict the time at which the pose is to be calculated
   *
   */
  void predictPose(double time_to_predict);

  /**
   * Predicts the position of the markers/LEDs in the image.
   *
   * The predicted image coordinates of the makers are estimated based on the predicted pose of the tracked object.
   *
   * \see predictPose
   *
   * The predicted image coordinates are obtained by projecting the three dimensional marker position in the camera coordinate frame onto the image plane.
   * \f[
   *    \left( \begin{array}{c} \hat{u} \\ \hat{v} \end{array} \right)_i = \mathrm{project} (\mathbf{x}_i, \hat{\mathbf{T}}_{co})
   * \f]
   * where \f$\mathbf{x}_i\f$ is the 3D position of marker/LED \f$i\f$ in the object-fixed frame of reference in homogeneous coordinates and \f$\hat{\mathbf{T}}_{co} \f$ is the transformation matrix that transforms points from the object-fixed coordinate frame to the camera-fixed coordinate frame.
   *
   * \see project2d
   *
   */
  void predictMarkerPositionsInImage();

  /**
   * Finds possible correspondences between detected image points and markers/LEDs on the object.
   *
   * Looks for the nearest image detections neighbours to the predicted pixel positions of the markers/LEDs.
   * If the nearest neighbour is within the threshold #nearest_neighbour_pixel_tolerance_, then the pair is
   * considered to be a correspondence and is entered into the correspondence matrix #correspondences_.
   *
   * \note One image detection can correspond to more than one object point. This accounts for the situation
   * where there is alignment of the LEDs/markers.
   *
   * \see predictMarkerPositionsInImage
   *
   */
  void findCorrespondences();

  /**
   * Checks whether the correspondences are valid and updates the predicted pose (#predicted_pose_) of the object if they are.
   *
   * All combinations of 3 correspondences are selected and used to compute the four pose candidates using the P3P algorithm.
   * The reprojection of the remaining LEDs/markers in the set of correspondences are back projected. If the ratio of back-projected
   * LEDs/markers that are within the threshold #back_projection_pixel_tolerance_ for one of the four possible poses is within the
   * threshold #certainty_threshold_, then that correspondence is considered to be 'valid'. If the ratio of correspondences that are valid is
   * greater than #valid_correspondence_threshold_, then the correspondences are considered to be correct.
   *
   * \return
   *   - \b 0 if the correspondences are not valid
   *   - \b 1 if the correspondences are valid
   *
   */
  unsigned checkCorrespondences();

  /**
   * Performs the brute-force initialisation to determine the LED/marker and image detection correspondences and the pose of the object.
   *
   * In order to compute the pose, the correspondences between the image detections #image_points_ and the LEDs/markers #object_points_ must
   * first be determined. For every combination of three image detections and every permutation of three LEDs/markers on the objectm the P3P
   * algorithm is used to calculate the four pose candidates.
   *
   * For every pose candidate, the LEDs/markers that were not used to compute the pose candidate are projected onto the the image plane. If
   * the reprojection has a nearest-neighbour of the image detections that is within the threshold #back_projection_pixel_tolerance_, then
   * the image detection combination and LED permutation is considered to be correct. A histogram with bins for every image detection-LED/marker
   * pair is used to determine the correspondences. The histogram bin for a correspondence is increased for every pair of detections and LEDs that
   * results in a valid pose.
   *
   * Once the histogram has been formed, the correspondences are determined using the #correspondencesFromHistogram function.
   *
   * The resulting correspondences are checked with #checkCorrespondences.
   *
   * \return
   *   - \b 0 if the initilisation failed to resolve a pose
   *   - \b 1 if the initialisation was able to resolve a pose
   *
   * \see correspondencesFromHistogram, checkCorrespondences
   */
  unsigned initialise();

  /**
   * Optimises the predicted pose using a Gauss-Newton optimisation to minimise the squared reprojection error between the LEDs/markers and image detections.
   *
   * The optimisation minimises the squared reprojection error. I.e. The optimised pose is
   * \f[
   *    P^* = \operatorname*{arg\,min}_P{ \sum_{\mathbf{c}_k \in \mathcal{C}} \| \pi \left( \mathbf{l}_k, P\right) - \mathbf{d}_k \|^2  }
   * \f]
   * where \f$\mathbf{c}_k = \langle \mathbf{l}_i, \mathbf{d}_j \rangle\f$ is a correspondence between an LED \f$\mathbf{l}_i\f$ and an image detection \f$\mathbf{d}_j\f$, and \f$\pi : \mathbb{R}^3 \times SE(3) \to \mathbb{R}^2\f$ is the projection of an LED/marker onto the image plane.
   *
   * The jacobian \f$\mathbf{J}\f$ with respect to the pose \f$P\f$ is required for the optimisation. Under the assumption that each error in the reprojection is independent of the other reprojections, then the total jacobian is the sum of the jacobians for the individual reprojection errors.
   * \f[
   *    \mathbf{J} = \sum_k \mathbf{J}_k
   * \f]
   *  where \f$\mathbf{J}_k\f$ is the Jacobian for correspondence \f$\mathbf{c}_k\f$ and is calculated according to equation A.14 from the the PhD thesis of Ethan Eade (http://ethaneade.com/) \cite Eade:2008. See the function #computeJacobian for more details.
   *
   * The covariance of the pose \f$\mathbf{\Sigma}_P\f$ is calculated as \cite Bell:1993
   * \f[
   *    \mathbf{\Sigma}_P = \left( \mathbf{J}^\top \mathbf{\Sigma}_{\mathcal{D}}^{-1} \mathbf{J} \right)^{-1}
   * \f]
   * where \f$\mathbf{\Sigma}_D \in \mathbb{R}^{2 \times 2}\f$ is the covariance of the LED detections. Here \f$\mathbf{\Sigma}_D = \mathbf{I}_{2 \times 2} \cdot 1 \mathrm{\:pixel}^2 \f$.
   *
   */
  void optimisePose();

  /**
   * Updates the time index and the past poses. I.e., the current pose and time becomes the previous pose
   * and time and the predicted pose and time becomes the current pose and time.
   *
   */
  void updatePose();

  /**
   * Optimises the pose by minimising the reprojection error.
   *
   */
  void optimiseAndUpdatePose(double & time_to_predict);

  /**
   * Uses the previous poses to predict the current pose of the object and uses this prediction to calculate a region of interest in which the LED point should be found in the image.
   *
   * \param time_to_predict the time of the image, i.e. the time at which to estimate the pose of the object
   * \param image the image in which the LEDs are to be detected
   *
   */
  void predictWithROI(double & time_to_predict, const cv::Mat & image);

  /**
   * Determines the correspondences using a nearest neighbour search between the predicted position of the LEDs in the image and the actual detected LEDs in the image
   *
   */
  void findCorrespondencesAndPredictPose(double & time_to_predict);

};

}

#endif /* POSEESTIMATOR_H_ */
