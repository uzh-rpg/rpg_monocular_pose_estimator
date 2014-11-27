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
 * dataTypes.h
 *
 *  Created on: Sep 17, 2013
 *      Author: karl
 */

/**
 * \file datatypes.h
 * \brief File containing the type definitions that are used in the monocular pose estimator.
 *
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <Eigen/Dense>

namespace monocular_pose_estimator
{

// Type definitions
typedef Eigen::Matrix<double, 6, 6> Matrix6d; //!< A 6x6 matrix of doubles
typedef Eigen::Matrix<double, 2, 6> Matrix2x6d; //!< A 2x6 matrix of doubles
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d; //!< A 3x4 matrix of doubles
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXYd; //!< A matrix of doubles containing dynamic rows and columns
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> MatrixXYu; //!< A matrix of unsigned integers containing dynamic rows and columns
typedef Eigen::Matrix<double, 6, 1> Vector6d; //!< A column vector of 6 elements containing doubles
typedef Eigen::Matrix<unsigned, 3, 1> Vector3u; //!< A column vector of 3 elements containing unsigned integers
typedef Eigen::Matrix<unsigned, 4, 1> Vector4u; //!< A column vector of 4 elements containing unsigned integers
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 1> VectorXu; //!< A dynamic column vector containing unsigened integers
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 2> VectorXuPairs; //!< A matrix with a dynamic number of rows and 2 columns containing unsigned integers
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowXd; //!< A dynamic row vector containing doubles
typedef Eigen::Matrix<unsigned, 1, Eigen::Dynamic> RowXu; //!< A dynamic row vector containing unsigned integers
typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d
typedef Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, 1> List3DPoints; //!< A dynamic column vector containing Vector3D elements. \see Vector3d
typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints; //!< A dynamic column vector containing Vector4D elements. \see Vector4d

} // namespace

#endif /* DATATYPES_H_ */
