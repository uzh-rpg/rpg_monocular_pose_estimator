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
 double  * PoseEstimator.cpp
 *
 *  Created on: July 29, 2013
 *      Author: Karl Schwabe
 */

/**
 * \file pose_estimator.cpp
 * \brief File containing the definitions of the functions for the PoseEstimator class
 *
 */

#include "monocular_pose_estimator_lib/pose_estimator.h"

namespace monocular_pose_estimator
{

PoseEstimator::PoseEstimator()
{
  back_projection_pixel_tolerance_ = 3;
  nearest_neighbour_pixel_tolerance_ = 5;
  certainty_threshold_ = 0.75;
  valid_correspondence_threshold_ = 0.7;

  it_since_initialized_ = 0;
}

void PoseEstimator::augmentImage(cv::Mat &image)
{
  Visualization::createVisualizationImage(image, predicted_pose_, camera_matrix_K_, camera_distortion_coeffs_,
                                          region_of_interest_, distorted_detection_centers_);
}

void PoseEstimator::setMarkerPositions(List4DPoints positions_of_markers_on_object)
{
  object_points_ = positions_of_markers_on_object;
  predicted_pixel_positions_.resize(object_points_.size());
  histogram_threshold_ = Combinations::numCombinations(object_points_.size(), 3);
}

List4DPoints PoseEstimator::getMarkerPositions()
{
  return object_points_;
}

bool PoseEstimator::estimateBodyPose(cv::Mat image, double time_to_predict)
{
  pose_updated_ = false;
  // Set up pixel positions list
  List2DPoints detected_led_positions;

  if (it_since_initialized_ < 1)  // If not yet initialised, search the whole image for the points
  {
    setPredictedTime(time_to_predict);

    region_of_interest_ = cv::Rect(0, 0, image.cols, image.rows);

    // Do detection of LEDs in image
    LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
                          max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
                          detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
                          camera_distortion_coeffs_, camera_matrix_P_);

    if (detected_led_positions.size() >= min_num_leds_detected_) // If found enough LEDs, Reinitialise
    {
      // Reinitialise
//      ROS_WARN("Initialising using brute-force correspondence search.");

      setImagePoints(detected_led_positions);

      if (initialise() == 1)
      {
        optimiseAndUpdatePose(time_to_predict);
      }
    }
    else
    { // Too few LEDs found
    }

  }
  else
  { // If initialised

    predictWithROI(time_to_predict, image);

    // Search image within ROI
    LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
                          max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
                          detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
                          camera_distortion_coeffs_, camera_matrix_P_);

    bool repeat_check = true;
    unsigned num_loops = 0;

    do
    {
      num_loops++;
      if (detected_led_positions.size() >= min_num_leds_detected_) // If enough LEDs detected
      {
        setImagePoints(detected_led_positions);
        findCorrespondencesAndPredictPose(time_to_predict);
        repeat_check = false;
      }
      else
      { // If too few LEDS detected
        if (num_loops < 2)
        { // If haven't searched image yet, search image

//          ROS_WARN("Too few LEDs detected in ROI. Searching whole image. Num LEDs detected: %d.", (int)detected_led_positions.size());

          // Search whole image
          region_of_interest_ = cv::Rect(0, 0, image.cols, image.rows);

          // Do detection of LEDs in image
          LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
                                max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
                                detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
                                camera_distortion_coeffs_, camera_matrix_P_);

        }
        else
        { // If already searched image continue
          repeat_check = false;
//          ROS_WARN("Too few LEDs detected. Num LEDs detected: %d.", (int)detected_led_positions.size());
        }
      }
    } while (repeat_check);
  }

  return pose_updated_;
}

void PoseEstimator::setPredictedPose(const Eigen::Matrix4d & pose, double time)
{
  predicted_pose_ = pose;
  predicted_time_ = time;

}

Eigen::Matrix4d PoseEstimator::getPredictedPose()
{
  return predicted_pose_;
}

Matrix6d PoseEstimator::getPoseCovariance()
{
  return pose_covariance_;
}

void PoseEstimator::setImagePoints(List2DPoints points)
{
  image_points_ = points;
  PoseEstimator::calculateImageVectors();
}

void PoseEstimator::setPredictedPixels(List2DPoints points)
{
  predicted_pixel_positions_ = points;
}

List2DPoints PoseEstimator::getPredictedPixelPositions()
{
  return predicted_pixel_positions_;
}

void PoseEstimator::setBackProjectionPixelTolerance(double tolerance)
{
  back_projection_pixel_tolerance_ = tolerance;
}

double PoseEstimator::getBackProjectionPixelTolerance()
{
  return back_projection_pixel_tolerance_;
}

void PoseEstimator::setNearestNeighbourPixelTolerance(double tolerance)
{
  nearest_neighbour_pixel_tolerance_ = tolerance;
}

double PoseEstimator::getNearestNeighbourPixelTolerance()
{
  return nearest_neighbour_pixel_tolerance_;
}

void PoseEstimator::setCertaintyThreshold(double threshold)
{
  certainty_threshold_ = threshold;
}

double PoseEstimator::getCertaintyThreshold()
{
  return certainty_threshold_;
}

void PoseEstimator::setValidCorrespondenceThreshold(double threshold)
{
  valid_correspondence_threshold_ = threshold;
}

double PoseEstimator::getValidCorrespondenceThreshold()
{
  return valid_correspondence_threshold_;
}

void PoseEstimator::setHistogramThreshold(unsigned threshold)
{
  histogram_threshold_ = threshold;
}

unsigned PoseEstimator::getHistogramThreshold()
{
  return histogram_threshold_;
}

void PoseEstimator::predictPose(double time_to_predict)
{
  predicted_time_ = time_to_predict;

  // get difference of poses, then its twist coordinates
  Vector6d delta = logarithmMap(previous_pose_.inverse() * current_pose_);

  // extrapolate
  Vector6d delta_hat = delta / (current_time_ - previous_time_) * (predicted_time_ - current_time_);

  // predict new pose
  predicted_pose_ = current_pose_ * exponentialMap(delta_hat);
}

List2DPoints PoseEstimator::getImagePoints()
{
  return image_points_;
}

void PoseEstimator::setCameraProjectionMatrix(Matrix3x4d M)
{
  camera_projection_matrix_ = M;
}

Matrix3x4d PoseEstimator::getCameraProjectionMatrix()
{
  return camera_projection_matrix_;
}

inline Eigen::Vector2d PoseEstimator::project2d(Eigen::Vector4d point, Eigen::Matrix4d transform)
{
  Eigen::Vector3d temp;
  temp = camera_projection_matrix_ * transform * point;
  temp = temp / temp(2);
  return temp.head<2>();
}

void PoseEstimator::predictMarkerPositionsInImage()
{
  for (unsigned i = 0; i < object_points_.size(); ++i)
  {
    predicted_pixel_positions_(i) = project2d((Eigen::Vector4d)object_points_(i), predicted_pose_);
  }
}

void PoseEstimator::setCorrespondences(VectorXuPairs corrs)
{
  correspondences_ = corrs;
}

VectorXuPairs PoseEstimator::getCorrespondences()
{
  return correspondences_;
}

void PoseEstimator::calculateImageVectors()
{
  unsigned num_image_points = image_points_.size();
  image_vectors_.resize(num_image_points);
  Eigen::Vector3d single_vector;

  for (unsigned i = 0; i < num_image_points; ++i)
  {
    single_vector(0) = (image_points_(i)(0) - camera_projection_matrix_(0, 2)) / camera_projection_matrix_(0, 0);
    single_vector(1) = (image_points_(i)(1) - camera_projection_matrix_(1, 2)) / camera_projection_matrix_(1, 1);
    single_vector(2) = 1;
    image_vectors_(i) = single_vector / single_vector.norm();
  }
}

double PoseEstimator::calculateSquaredReprojectionErrorAndCertainty(const List2DPoints & image_pts,
                                                                    const List2DPoints & object_pts, double & certainty)
{
  double squared_error = 0;
  unsigned num_correspondences = 0;

  //Declare distance matrix
  MatrixXYd distances(image_pts.size(), object_pts.size());

  // Build distance matrix
  for (unsigned i = 0; i < image_pts.size(); ++i)
  {
    for (unsigned j = 0; j < object_pts.size(); ++j)
    {
      distances(i, j) = squareDist((Eigen::Vector2d)image_pts(i), (Eigen::Vector2d)object_pts(j));
    }
  }
  distances = distances.cwiseSqrt(); // Square root to get distances

  double min_value;
  unsigned row_idx, col_idx;

  for (unsigned j = 1; j <= std::min(image_pts.size(), object_pts.size()); ++j)
  {
    min_value = distances.minCoeff(&row_idx, &col_idx);
    if (min_value <= back_projection_pixel_tolerance_)
    {
      squared_error += pow((double)distances(row_idx, col_idx), 2);
      num_correspondences++;
      distances.row(row_idx).setConstant(INFINITY);
      distances.col(col_idx).setConstant(INFINITY);
    }
    else
      break;
  }

  certainty = (double)num_correspondences / object_pts.size();

  return squared_error;
}

VectorXuPairs PoseEstimator::correspondencesFromHistogram(MatrixXYu & histogram)
{
  //unsigned threshold = 4;
  VectorXuPairs correspondences(histogram.cols(), 2);
  unsigned num_correspondences = 0;
  unsigned max_value;
  unsigned row_idx, col_idx;

  //for (unsigned j = 0; j < std::min(histogram.rows(), histogram.cols()); ++j)
  for (unsigned j = 0; j < histogram.cols(); ++j)
  {
    max_value = histogram.maxCoeff(&row_idx, &col_idx);

    // if (max_value == 0)
    if (max_value < histogram_threshold_)
      break;

    correspondences(num_correspondences, 0) = col_idx + 1; // Base one counting of entries, since an entry of 0 shows no correspondence.
    correspondences(num_correspondences, 1) = row_idx + 1; // Base one counting of entries, since an entry of 0 shows no correspondence.
    num_correspondences++;
    histogram.col(col_idx).setConstant(0); // Only set the column to zero
  }

  correspondences.conservativeResize(num_correspondences, 2);

  return correspondences;
}

void PoseEstimator::findCorrespondences()
{
  Eigen::VectorXd min_distances;
  VectorXuPairs pairs = calculateMinDistancesAndPairs(predicted_pixel_positions_, image_points_, min_distances);
  VectorXuPairs temp_corrs(pairs.rows(), 2);
  unsigned num_corrs = 0;

  for (unsigned i = 0; i < pairs.rows(); ++i)
  {
    if (min_distances(i) <= nearest_neighbour_pixel_tolerance_)
    {
      temp_corrs(num_corrs, 0) = pairs(i, 0);
      temp_corrs(num_corrs, 1) = pairs(i, 1);
      num_corrs++;
    }
  }

  temp_corrs.conservativeResize(num_corrs, 2);

  correspondences_ = temp_corrs;
}

unsigned PoseEstimator::checkCorrespondences()
{
  bool valid_correspondences = 0;
  unsigned num_valid_correspondences = 0;

  // The unit image vectors from the camera out to the object points are already set when the image points are set in PoseEstimator::setImagePoints()

  if (correspondences_.rows() < 4)
  {
    return valid_correspondences;
  }
  else
  {
    MatrixXYd mean_reprojected_object_points(4, object_points_.size());
    mean_reprojected_object_points.setZero();

    MatrixXYu combinations = Combinations::combinationsNoReplacement(correspondences_.rows(), 3);
    unsigned N = combinations.rows();

    for (unsigned i = 0; i < N; ++i)
    {
      //Declare and populate the feature vectors and world points required for the P3P algorithm
      Eigen::Matrix3d feature_vectors, world_points;
      Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;

      Eigen::Vector4d temp_point;
      temp_point = object_points_(correspondences_(combinations(i, 0) - 1, 0) - 1);
      world_points.col(0) = temp_point.head<3>();
      temp_point = object_points_(correspondences_(combinations(i, 1) - 1, 0) - 1);
      world_points.col(1) = temp_point.head<3>();
      temp_point = object_points_(correspondences_(combinations(i, 2) - 1, 0) - 1);
      world_points.col(2) = temp_point.head<3>();

      feature_vectors.col(0) = image_vectors_(correspondences_(combinations(i, 0) - 1, 1) - 1);
      feature_vectors.col(1) = image_vectors_(correspondences_(combinations(i, 1) - 1, 1) - 1);
      feature_vectors.col(2) = image_vectors_(correspondences_(combinations(i, 2) - 1, 1) - 1);

      // Find the unused image and object points
      unsigned total_unused_correspondences = correspondences_.rows() - 3;
      List2DPoints unused_im_points(total_unused_correspondences); // Vector to hold the indexes of the unused image points that have correspondences
      List4DPoints unused_object_points(total_unused_correspondences); // Vector to hold the indexes of the unused object points that have correspondences
      unsigned num_unused_points = 0;
      bool already_used_correspondence = 0;

      // Search for the unused object points and image points
      for (unsigned l = 0; l < correspondences_.rows(); ++l)
      {
        already_used_correspondence = 0;
        for (unsigned n = 0; n < 3; ++n)
        {
          if (combinations(i, n) - 1 == l)
            already_used_correspondence = 1;
        }
        if (!already_used_correspondence)
        {
          unused_object_points(num_unused_points) = object_points_(correspondences_(l, 0) - 1);
          unused_im_points(num_unused_points) = image_points_(correspondences_(l, 1) - 1);
          num_unused_points++;
        }
        //Break if already found all the unused object points
        if (num_unused_points == total_unused_correspondences)
          break;
      }

      // Compute the poses using the P3P algorithm
      int executed_correctly = P3P::computePoses(feature_vectors, world_points, solutions);

      // If the P3P algorithm found a solution (i.e. the world points were not aligned), then continue
      if (executed_correctly == 0)
      {

        double squared_error;
        double min_squared_error = INFINITY;
        unsigned smallest_error_idx;
        bool valid_correspondence_found = 0;

        Eigen::Matrix4d H_o_c_current;

        // Loop through the four solutions provided by the P3P algorithm
        for (unsigned j = 0; j < 4; ++j)
        {
          H_o_c_current.setIdentity();
          H_o_c_current.block<3, 4>(0, 0) = solutions(j);

          // Consider only the real poses. Complex or NaN poses are excluded/ignored
          if (isFinite(H_o_c_current))
          {
            //Back-project the unused object points onto the image plane
            List2DPoints unused_back_projected_object_points(total_unused_correspondences);
            Eigen::Vector3d temp;
            for (unsigned ii = 0; ii < total_unused_correspondences; ++ii)
            {
              unused_back_projected_object_points(ii) = project2d(unused_object_points(ii), H_o_c_current.inverse());
            }

            double certainty;
            squared_error = calculateSquaredReprojectionErrorAndCertainty(unused_im_points,
                                                                          unused_back_projected_object_points,
                                                                          certainty);

            if (certainty >= certainty_threshold_)
            {
              valid_correspondence_found = 1;
              if (squared_error < min_squared_error)
              {
                min_squared_error = squared_error;
                smallest_error_idx = j;
              }
            }
          }
        }

        if (valid_correspondence_found)
        {
          num_valid_correspondences++;
          Eigen::Matrix4d temp_solution;
          temp_solution.setIdentity();
          temp_solution.block<3, 4>(0, 0) = solutions(smallest_error_idx);

          for (unsigned jj = 0; jj < object_points_.size(); ++jj)
          {
            mean_reprojected_object_points.col(jj) = mean_reprojected_object_points.col(jj)
                + temp_solution.inverse() * object_points_(jj);
          }

        }

      }

    }

    if ((double)num_valid_correspondences / N >= valid_correspondence_threshold_)
    {
      valid_correspondences = 1;
      mean_reprojected_object_points = mean_reprojected_object_points / num_valid_correspondences;
      MatrixXYd object_points_matrix(4, object_points_.size());
      for (unsigned kk = 0; kk < object_points_.size(); ++kk)
      {
        object_points_matrix.col(kk) = object_points_(kk);
      }
      object_points_matrix.conservativeResize(3, object_points_matrix.cols());
      mean_reprojected_object_points.conservativeResize(3, mean_reprojected_object_points.cols());
      predicted_pose_ = computeTransformation(object_points_matrix, mean_reprojected_object_points);
    }

  }

  return valid_correspondences;
}

unsigned PoseEstimator::initialise()
{
  // Combinations of seen points
  RowXu seen_points_working_vector;
  seen_points_working_vector.setLinSpaced(image_points_.size(), 1, image_points_.size());
  MatrixXYu seen_points_combinations = Combinations::combinationsNoReplacement(seen_points_working_vector, 3);
  unsigned num_seen_points_combinations = seen_points_combinations.rows();

  // Permutations of object points
  MatrixXYu object_points_permutations = Combinations::permutationsNoReplacement(object_points_.size(), 3);
  unsigned num_object_points_permutations = object_points_permutations.rows();

  MatrixXYu hist_corr;
  hist_corr.setZero(image_points_.size(), object_points_.size());

  // The unit image vectors from the camera out to the object points are already set when the image points are set in PoseEstimator::setImagePoints()

  Eigen::Matrix3d feature_vectors, world_points;

  // for every combination of 3 seen points, we have to iterate through all
  // the possible permutations of 3 object points.
  for (unsigned i = 0; i < num_seen_points_combinations; ++i)
  {

    Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;

    // Build up matrix of unit feature vectors
    feature_vectors.col(0) = image_vectors_(seen_points_combinations(i, 0) - 1);
    feature_vectors.col(1) = image_vectors_(seen_points_combinations(i, 1) - 1);
    feature_vectors.col(2) = image_vectors_(seen_points_combinations(i, 2) - 1);

    // Find unused image points
    unsigned total_unused_im_points = image_points_.size() - 3;
    List2DPoints unused_im_points(total_unused_im_points); // Vector to hold the indexes of the unused image points that have correspondences
    VectorXu unused_im_points_idx(total_unused_im_points);
    unsigned num_unused_im_points = 0;
    bool already_used_im_point = 0;
    for (unsigned kk = 0; kk < image_points_.size(); ++kk)
    {
      already_used_im_point = 0;
      for (unsigned ii = 0; ii < 3; ++ii)
      {
        if (seen_points_combinations(i, ii) - 1 == kk)
          already_used_im_point = 1;
      }
      if (!already_used_im_point)
      {
        unused_im_points(num_unused_im_points) = image_points_(kk);
        unused_im_points_idx(num_unused_im_points) = kk;
        num_unused_im_points++;
      }
      if (num_unused_im_points == total_unused_im_points)
        break;
    }

    for (unsigned j = 0; j < num_object_points_permutations; ++j)
    {

      // Build up matrix of world points
      Eigen::Vector4d temp_point;
      temp_point = object_points_(object_points_permutations(j, 0) - 1);
      world_points.col(0) = temp_point.head<3>();
      temp_point = object_points_(object_points_permutations(j, 1) - 1);
      world_points.col(1) = temp_point.head<3>();
      temp_point = object_points_(object_points_permutations(j, 2) - 1);
      world_points.col(2) = temp_point.head<3>();

      // Compute the poses using the P3P algorithm
      int executed_correctly = P3P::computePoses(feature_vectors, world_points, solutions);

      // If the P3P algorithm found a solution (i.e. the world points were not aligned), then continue
      if (executed_correctly == 0)
      {

        Eigen::Matrix4d H_o_c_current;

        // Find the unused image and object points
        unsigned total_unused_object_points = object_points_.size() - 3;

        List4DPoints unused_object_points(total_unused_object_points); // Vector to hold the indexes of the unused object points that have correspondences
        VectorXu unused_object_points_idx(total_unused_object_points);
        unsigned num_unused_object_points = 0;
        bool already_used_object_point = 0;
        for (unsigned ll = 0; ll < object_points_.size(); ++ll)
        {
          already_used_object_point = 0;
          for (unsigned jj = 0; jj < 3; ++jj)
          {
            if (object_points_permutations(j, jj) - 1 == ll)
              already_used_object_point = 1;
          }
          if (!already_used_object_point)
          {
            unused_object_points(num_unused_object_points) = object_points_(ll);
            unused_object_points_idx(num_unused_object_points) = ll;
            num_unused_object_points++;
          }
          //Break if already found all the unused object points
          if (num_unused_object_points == total_unused_object_points)
            break;
        }

        // Loop through the four solutions provided by the P3P algorithm
        for (unsigned k = 0; k < 4; ++k)
        {
          // Loop through the four solutions provided by the P3P algorithm
          H_o_c_current.setIdentity();
          H_o_c_current.block<3, 4>(0, 0) = solutions(k);

          if (isFinite(H_o_c_current))
          {
            //Back-project the unused object points onto the image plane
            List2DPoints unused_back_projected_object_points(total_unused_object_points);
            Eigen::Vector3d temp;
            for (unsigned m = 0; m < total_unused_object_points; ++m)
            {
              unused_back_projected_object_points(m) = project2d(unused_object_points(m), H_o_c_current.inverse());
            }

            Eigen::VectorXd min_distances;
            MatrixXYu pairs = calculateMinDistancesAndPairs(unused_im_points, unused_back_projected_object_points,
                                                            min_distances);

            // Check that at least one of the points was within the threshold
            unsigned count_within_pixel_threshold = 0;
            for (unsigned ll = 0; ll < min_distances.size(); ++ll)
            {
              if (min_distances(ll) < back_projection_pixel_tolerance_)
              {
                count_within_pixel_threshold++;
              }
            }
            if (count_within_pixel_threshold > 0)
            {
              unsigned im_idx;
              unsigned obj_idx;
              for (unsigned mm = 0; mm < 3; ++mm)
              {
                im_idx = seen_points_combinations(i, mm) - 1; // image point index
                obj_idx = object_points_permutations(j, mm) - 1; // object point index
                hist_corr(im_idx, obj_idx) = hist_corr(im_idx, obj_idx) + 1;
              }

              for (unsigned nn = 0; nn < min_distances.size(); ++nn)
              {
                if (min_distances(nn) < back_projection_pixel_tolerance_)
                {
                  im_idx = unused_im_points_idx(pairs(nn, 0) - 1); // image point index
                  obj_idx = unused_object_points_idx(pairs(nn, 1) - 1); // object point index
                  hist_corr(im_idx, obj_idx) = hist_corr(im_idx, obj_idx) + 1;
                }
              }
            }
          }
        }
      }

    }
  }

  if (!(hist_corr.array() == 0).all())
  {
    correspondences_ = correspondencesFromHistogram(hist_corr);
    if (checkCorrespondences() == 1)
    {
      return 1; // Found a solution
    }
    else
    {
      return 0; // Failed to find a solution
    }
  }
  else
  {
    return 0; // Failed to find a solution
  }

}

void PoseEstimator::setPredictedTime(double time)
{
  predicted_time_ = time;
}

double PoseEstimator::getPredictedTime()
{
  return predicted_time_;
}

void PoseEstimator::optimisePose()
{
  // Using a Gauss-Newton Optimisation

  const double converged = 1e-13;
  const unsigned max_itr = 500;

  Eigen::Matrix4d T_new;
  Eigen::Matrix4d T_old = predicted_pose_;
  Matrix6d A;
  Vector6d b;
  Eigen::Matrix2d R; // Covariance matrix of the image points. Assume the image points points are independent
  R.setIdentity(); // Assume the variance is one pixel in u and v.
  Matrix2x6d J;
  Eigen::Vector2d focal_lengths;
  focal_lengths(0) = camera_projection_matrix_(0, 0);
  focal_lengths(1) = camera_projection_matrix_(1, 1);
  Vector6d dT;

  for (unsigned i = 0; i < max_itr; ++i)
  {
    A.setZero();
    b.setZero();

    // Compute the initial errors/residual
    for (unsigned j = 0; j < correspondences_.rows(); ++j)
    {
      if (correspondences_(j, 1) == 0)
        continue;

      // Project point into image plane
      Eigen::Vector2d p_image_plane = project2d((Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1),
                                                predicted_pose_);

      // Calculate the error
      Eigen::Vector2d e = image_points_(correspondences_(j, 1) - 1) - p_image_plane;

      // Compute Jacobian
      J = computeJacobian(predicted_pose_, (Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1), focal_lengths);

      A.noalias() += J.transpose() * R.inverse() * J;
      b.noalias() += J.transpose() * R.inverse() * e;
    }

    dT = A.ldlt().solve(b);

    // Update the model
    T_new = exponentialMap(dT) * predicted_pose_;
    T_old = predicted_pose_;
    predicted_pose_ = T_new;

    // Stop when converged
    if (norm_max(dT) <= converged)
      break;
  }

  pose_covariance_ = A.inverse();

}

void PoseEstimator::updatePose()
{
  previous_pose_ = current_pose_;
  current_pose_ = predicted_pose_;
  previous_time_ = current_time_;
  current_time_ = predicted_time_;
}

void PoseEstimator::optimiseAndUpdatePose(double & time_to_predict)
{
  optimisePose();

  if (it_since_initialized_ < 2)
  {
    it_since_initialized_++;
  }
  updatePose();
  pose_updated_ = true;
}

void PoseEstimator::predictWithROI(double & time_to_predict, const cv::Mat & image)
{
  if (it_since_initialized_ >= 2)
  { // Predict the pose if initialised. If not the pose will remain the same as the previous step (constant velocity model)
    predictPose(time_to_predict);
  }
  else
  { // If not yet fully initialised (Only one pose calculated so far)
    setPredictedTime(time_to_predict);
  }
  predictMarkerPositionsInImage();

  // Define region of interest (ROI)
  region_of_interest_ = LEDDetector::determineROI(getPredictedPixelPositions(), image.size(), roi_border_thickness_,
                                                  camera_matrix_K_, camera_distortion_coeffs_, camera_matrix_P_);
}

void PoseEstimator::findCorrespondencesAndPredictPose(double & time_to_predict)
{
  findCorrespondences();

  if (checkCorrespondences() == 1)
  { // If the correspondences were correct, update the pose
    optimiseAndUpdatePose(time_to_predict);
  }
  else
  { // Reinitialise if the correspondences weren't correct

    if (initialise() == 1)
    { // Only update pose if the initialisation found a valid pose.
      optimiseAndUpdatePose(time_to_predict);
    }
  }

}

template<typename DerivedA, typename DerivedB>
  double PoseEstimator::squareDist(const Eigen::MatrixBase<DerivedA>& p1, const Eigen::MatrixBase<DerivedB>& p2)
  {
    return (p1 - p2).squaredNorm();
  }

template<typename Derived>
  bool PoseEstimator::isFinite(const Eigen::MatrixBase<Derived>& x)
  {
    return ((x - x).array() == (x - x).array()).all();
  }

VectorXuPairs PoseEstimator::calculateMinDistancesAndPairs(const List2DPoints & points_a, const List2DPoints & points_b,
                                                           Eigen::VectorXd & min_distances)
{
  unsigned num_points_a = points_a.size();
  unsigned num_points_b = points_b.size();
  VectorXuPairs pairs(num_points_a, 2);

  // Work around since pairs.col(0).setLinSpaced(num_points_a,1,num_points_a) throws a floating point error when num_points_a = 1
  if (num_points_a == 1)
  {
    pairs(0) = 1;
  }
  else
  {
    pairs.col(0).setLinSpaced((int)num_points_a, 1, num_points_a);
  }
  pairs.col(1).setZero();

  double min_dist_squared = INFINITY;
  double dist_squared;
  Eigen::VectorXd min_dists(num_points_a);

  for (unsigned i = 0; i < num_points_a; ++i)
  {
    min_dist_squared = INFINITY;

    for (unsigned j = 0; j < num_points_b; ++j)
    {
      dist_squared = squareDist((Eigen::Vector2d)points_a(i), (Eigen::Vector2d)points_b(j));

      if (dist_squared < min_dist_squared)
      {
        min_dist_squared = dist_squared;
        pairs(i, 1) = j + 1; // Storing values as base 1 indexing
      }
    }

    min_dists(i) = min_dist_squared;

  }

  min_distances = min_dists.cwiseSqrt();

  return pairs;
}

Eigen::Matrix4d PoseEstimator::computeTransformation(const MatrixXYd & object_points,
                                                     const MatrixXYd & reprojected_points)
{
  Eigen::Vector3d mean_object_points = object_points.rowwise().sum() / object_points.cols();
  Eigen::Vector3d mean_reprojected_points = reprojected_points.rowwise().sum() / reprojected_points.cols();
  MatrixXYd object_points_bar = object_points.colwise() - mean_object_points; // object points with zero mean
  MatrixXYd reprojected_points_bar = reprojected_points.colwise() - mean_reprojected_points;

  Eigen::JacobiSVD<MatrixXYd> svd_of_points(object_points_bar * reprojected_points_bar.transpose(),
                                            Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::Matrix3d U = svd_of_points.matrixU();
  Eigen::Matrix3d V = svd_of_points.matrixV();

  Eigen::Matrix3d R = V * U.transpose();
  Eigen::Vector3d t = mean_reprojected_points - R * mean_object_points;

  Eigen::Matrix4d transform;
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = t;
  return transform;
}

Matrix2x6d PoseEstimator::computeJacobian(const Eigen::Matrix4d & T_c_o, const Eigen::Vector4d & world_points,
                                          const Eigen::Vector2d & focal_lengths)
{
  // This Jacobian is calculated according to equation A.14 from the the PhD thesis of Ethan Eade
  // See http://ethaneade.com/
  // See http://ethaneade.com/thesis_revised.pdf

  const Eigen::Vector4d point_camera_frame = T_c_o * world_points;
  double x = point_camera_frame(0);
  double y = point_camera_frame(1);
  double z = point_camera_frame(2);
  double z_2 = z * z;
  Matrix2x6d jacobian;
  jacobian(0, 0) = 1 / z * focal_lengths(0);
  jacobian(0, 1) = 0;
  jacobian(0, 2) = -x / z_2 * focal_lengths(0);
  jacobian(0, 3) = -x * y / z_2 * focal_lengths(0);
  jacobian(0, 4) = (1 + (x * x / z_2)) * focal_lengths(0);
  jacobian(0, 5) = -y / z * focal_lengths(0);

  jacobian(1, 0) = 0;
  jacobian(1, 1) = 1 / z * focal_lengths(1);
  jacobian(1, 2) = -y / z_2 * focal_lengths(1);
  jacobian(1, 3) = -(1 + y * y / z_2) * focal_lengths(1);
  jacobian(1, 4) = x * y / z_2 * focal_lengths(1);
  jacobian(1, 5) = x / z * focal_lengths(1);

  return jacobian;
}

Eigen::Matrix4d PoseEstimator::exponentialMap(const Vector6d & twist)
{
  Eigen::Vector3d upsilon = twist.head<3>();
  Eigen::Vector3d omega = twist.tail<3>();

  double theta = omega.norm();
  double theta_squared = theta * theta;

  Eigen::Matrix3d Omega = skewSymmetricMatrix(omega);
  Eigen::Matrix3d Omega_squared = Omega * Omega;
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d V;

  if (theta == 0)
  {
    rotation = Eigen::Matrix3d::Identity();
    V.setIdentity();
  }
  else
  {
    rotation = Eigen::Matrix3d::Identity() + Omega / theta * sin(theta)
        + Omega_squared / theta_squared * (1 - cos(theta));
    V = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta_squared) * Omega
        + (theta - sin(theta)) / (theta_squared * theta) * Omega_squared);
  }

  Eigen::Matrix4d transform;
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = V * upsilon;

  return transform;
}

Vector6d PoseEstimator::logarithmMap(const Eigen::Matrix4d & trans)
{
  Vector6d xi;
  Eigen::Matrix3d R = trans.block<3, 3>(0, 0);
  Eigen::Vector3d t = trans.block<3, 1>(0, 3);
  Eigen::Vector3d w, upsilon;
  Eigen::Matrix3d w_hat;
  Eigen::Matrix3d A_inv;
  double phi = 0;
  double w_norm;

  // Calculate w_hat
  if (R.isApprox(Eigen::Matrix3d::Identity(), 1e-10) == 1)
  {
    // phi has already been set to 0;
    w_hat.setZero();
  }
  else
  {
    double temp = (R.trace() - 1) / 2;
    // Force phi to be either 1 or -1 if necessary. Floating point errors can cause problems resulting in this not happening
    if (temp > 1)
    {
      temp = 1;
    }
    else if (temp < -1)
    {
      temp = -1;
    }

    phi = acos(temp);
    if (phi == 0)
    {
      w_hat.setZero();
    }
    else
    {
      w_hat = (R - R.transpose()) / (2 * sin(phi)) * phi;
    }
  }

  // Extract w from skew symmetrix matrix of w
  w << w_hat(2, 1), w_hat(0, 2), w_hat(1, 0);

  w_norm = w.norm();

  // Calculate upsilon
  if (t.isApproxToConstant(0, 1e-10) == 1)
  {
    A_inv.setZero();
  }
  else if (w_norm == 0 || sin(w_norm) == 0)
  {
    A_inv.setIdentity();
  }
  else
  {
    A_inv = Eigen::Matrix3d::Identity() - w_hat / 2
        + (2 * sin(w_norm) - w_norm * (1 + cos(w_norm))) / (2 * w_norm * w_norm * sin(w_norm)) * w_hat * w_hat;
  }

  upsilon = A_inv * t;

  // Compose twist coordinates vector
  xi.head<3>() = upsilon;
  xi.tail<3>() = w;

  return xi;
}

Eigen::Matrix3d PoseEstimator::skewSymmetricMatrix(const Eigen::Vector3d w)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return Omega;
}

double PoseEstimator::norm_max(const Eigen::VectorXd & v)
{
  double max = -1;
  for (int i = 0; i < v.size(); i++)
  {
    double abs_val = std::abs((double)v(i));
    if (abs_val > max)
    {
      max = abs_val;
    }
  }
  return max;
}

} // namespace

