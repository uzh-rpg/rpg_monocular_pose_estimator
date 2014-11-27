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
 * combinations.h
 *
 *  Created on: Sep 17, 2013
 *      Author: Karl Schwabe
 */

/**
 * \file combinations.h
 * \brief File containing the includes and the function prototypes for the combinations.cpp file.
 *
 */

#ifndef COMBINATIONS_H_
#define COMBINATIONS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include "monocular_pose_estimator_lib/datatypes.h"

namespace monocular_pose_estimator
{

class Combinations
{
public:

  /**
   * Generates all combinations of \a K items chosen from \a N items.
   *
   * \param N the total number of items
   * \param K the number of items to be selected in each choice
   *
   * \returns matrix of all combinations, where each row is a separate combination
   *
   * \see combinationsNoReplacement(RowXu wv, unsigned K)
   *
   */
  static MatrixXYu combinationsNoReplacement(unsigned N, unsigned K);

  /**
   * Generates all combinations of \a K items from the list 1 to \a N.
   *
   * \param wv the working vector containing the set of all items from which \a K items will be chosen
   * \param K the number of items to be selected in each choice
   *
   * \returns matrix of all combinations, where each row is a separate combination
   *
   * \see combinationsNoReplacement(unsigned N, unsigned K)
   *
   */
  static MatrixXYu combinationsNoReplacement(RowXu wv, unsigned K);

  /**
   * Generates all permutations of \a K items chosen from the list 1 to \a N.
   *
   * \param N the total number of items
   * \param K the number of items to be selected in each choice
   *
   * \returns matrix of all permutations, where each row is a separate permutation
   *
   * \see permutations
   *
   */
  static MatrixXYu permutationsNoReplacement(unsigned N, unsigned K);

  /**
   * Generates all permutations of N items.
   *
   * \param N the total number of items
   *
   * \returns matrix of all permutations, where each row is a separate permutation
   *
   * \see permutationsNoReplacement
   *
   */
  static MatrixXYu permutations(unsigned N);

  /**
   * Calculates the number of combinations of \a K choices from \a N items with no replacements.
   *
   * \f[
   *      C = \frac{N!}{K!(N-K)!}
   * \f]
   *
   * \param N the total number of items
   * \param K the number of items to be selected in each choice
   *
   * \returns the number of combinations
   *
   */
  static unsigned numCombinations(unsigned N, unsigned K);

  /**
   * Calculates the number of permutations of \a K choices from \a N items with no replacements.
   *
   * \f[
   *      P = \frac{N!}{(N-K)!}
   * \f]
   *
   * \param N the total number of items
   * \param K the number of items to be selected in each choice
   *
   * \returns the number of permutations
   *
   */
  static unsigned numPermutations(unsigned N, unsigned K);

private:

  /**
   * Calculates the factorial of a positive integer number \a N
   *
   * \param N the integer whose factorial is to be calculated
   *
   * \returns \f$N!\f$
   *
   */
  static unsigned factorial(int N);

  /**
   * Calculates the product of the consecutive integers from \a k to \a n.
   *
   * \param k the starting integer for the consecutive product
   * \param n the end integer of the consecutive product
   *
   * \returns consecutive product
   *
   * \see cumulativeProduct
   *
   */
  static unsigned consecutiveProduct(unsigned k, unsigned n);

  /**
   * Calculates the cumulative product \a k to \a n.
   *
   * Each item \a i in the row vector that is returned is the product of all integers preceding it from \a k to \a i
   *
   * \param k the starting integer for the cumulative product
   * \param n the end integer of the cumulative product
   *
   * \returns row vector containing the cumulative product of the numbers preceding it
   *
   * \see consecutiveProduct
   *
   */
  static RowXd cumulativeProduct(unsigned k, unsigned n);

  /**
   * Builds a matrix from the elements in a row vector.
   *
   * Builds a matrix of the same size as \a indices. Indices contains the index (using one-based counting) of the elements in the row vector \a vec
   * that must appear in the output matrix.
   *
   * \param vec the row vector whose elements will be used to construct the matrix
   * \param indices matrix the indexes the row vector \a vec
   *
   * \returns matrix constructed from the elements of the row vector \a vec
   *
   */
  static MatrixXYu assignFromMatrixAsIndices(const RowXu &vec, const MatrixXYu &indices);

  /**
   * Sets all instances of a number in a matrix to a constant.
   *
   * The function finds all instances of the number \a checkNum and sets them equal to \a setNum.
   *
   * \param matrix the matrix in which the number will be replaced
   * \param checkNum the number to be found and replaced by \a setNum
   * \param setNum the number that will replace all instances of \a checkNum
   *
   * \returns matrix with replacement numbers
   *
   */
  static MatrixXYu setWhereEqualToConst(const MatrixXYu &matrix, const unsigned &checkNum, const unsigned &setNum);

};

} // namespace

#endif /* COMBINATIONS_H_ */
