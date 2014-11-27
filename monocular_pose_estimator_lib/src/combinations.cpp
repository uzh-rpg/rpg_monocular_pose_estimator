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
 * combinations.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: Karl Schwabe
 */

/**
 * \file combinations.cpp
 * \brief File containing the function definitions for combinations and permutations.
 *
 */

#include "monocular_pose_estimator_lib/combinations.h"

namespace monocular_pose_estimator
{

unsigned Combinations::factorial(int N)
{
  if (N == 1 || N == 0)
    return 1;
  else
    return factorial(N - 1) * N;
}

unsigned Combinations::numCombinations(unsigned N, unsigned K)
{
  return factorial(N) / (factorial(K) * factorial(N - K));
}

unsigned Combinations::numPermutations(unsigned N, unsigned K)
{
  return factorial(N) / (factorial(N - K));
}

MatrixXYu Combinations::combinationsNoReplacement(unsigned N, unsigned K)
{
  RowXu list;
  list.setLinSpaced(N, 1, N);

  return combinationsNoReplacement((RowXu)list, K);
}

MatrixXYu Combinations::combinationsNoReplacement(RowXu list, unsigned K)
{
  // Combinations without replacement.
  // wv is the working vector from which the combinations will be made.
  // Note K <= wv.size(). If not this function will not execute correctly.
  // Adapted from Matlab code by:
  //            Author:   Matt Fig
  //            Contact:  popkenai@yahoo.com
  //            Date:     5/30/2009
  //            Reference:  http://mathworld.wolfram.com/BallPicking.html
  MatrixXYu CN;
  unsigned N = list.size();
  if (K == 1)
  {
    CN = list.transpose();
    return CN;
  }
  else if (K == N)
  {
    CN = list;
    return CN;
  }

  RowXu wv;     // Working vector
  wv.setLinSpaced(K, 1, K);
  unsigned lim = K; // Limit for the working index
  unsigned idx = 1; // The index of the element of the working vector that is currently being worked on
  unsigned bc = consecutiveProduct(N - K + 1, N) / consecutiveProduct(1, K); // Pre-allocation
  unsigned step, flag;
  MatrixXYu CN_idx;
  CN_idx.setZero(bc, K);

  // Build up the indices (base 1) of the combinations
  CN_idx.row(0) = wv;
  for (unsigned i = 2; i <= bc - 1; ++i)
  {
    if (idx + lim < N)
    {
      step = idx;
      flag = 0;
    }
    else
    {
      step = 1;
      flag = 1;
    }

    for (unsigned j = 1; j <= step; ++j)
    {
      wv(K + j - idx - 1) = lim + j;
    }

    CN_idx.row(i - 1) = wv;
    idx = idx * flag + 1;
    lim = wv(K - idx);
  }

  CN_idx.row(bc - 1).setLinSpaced(K, N - K + 1, N);

  CN.setZero(bc, K);
  // Build up the combinations by selecting the numbers from the vector "list"
  for (unsigned i = 0; i < bc; ++i)
  {
    for (unsigned j = 0; j < K; ++j)
    {
      CN(i, j) = list(CN_idx(i, j) - 1);
    }
  }
  return CN;
}

MatrixXYu Combinations::permutationsNoReplacement(unsigned N, unsigned K)
{
  // Permutations without replacement.
  // Note K <= N. If not this function will not execute correctly.
  // Adapted from Matlab code by:
  //            Author:   Matt Fig
  //            Contact:  popkenai@yahoo.com
  //            Date:     5/30/2009
  //            Reference:  http://mathworld.wolfram.com/BallPicking.html
  MatrixXYu PN;
  if (N == 1 && K == 1)
  {
    RowXu r(1);
    r << 1;
    PN = r;
    return PN;
  }
  else if (N == K)
  {
    PN = permutations(N);
    return PN;
  }
  else if (K == 1)
  {
    VectorXu v;
    v.setLinSpaced(N, 1, N);
    PN = v;
    return PN;
  }

  RowXu wv; // Working vector
  wv.setLinSpaced(K, 1, K);
  RowXu final_vector; // Final vector
  final_vector.setLinSpaced(K, N - K + 1, N);
  unsigned lim = K; // Limit for the working index
  unsigned idx = 1; // The index of the element of the working vector that is currently being worked on
  unsigned bc = consecutiveProduct(N - K + 1, N); // Pre-allocation
  unsigned bc1 = bc / consecutiveProduct(1, K);  // Number of combination blocks
  unsigned l = consecutiveProduct(1, K); // Size of each permutation block
  unsigned count = 1 + l;
  unsigned step, flag;
  MatrixXYu P = permutations(K);
  PN.setZero(bc, K);
  PN.topLeftCorner(l, K) = P;

  for (unsigned i = 2; i <= bc1 - 1; ++i)
  {
    if (idx + lim < N)
    {
      step = idx;
      flag = 0;
    }
    else
    {
      step = 1;
      flag = 1;
    }

    for (unsigned j = 1; j <= step; ++j)
    {
      wv(K + j - idx - 1) = lim + j;
    }

    PN.block(count - 1, 0, l, K) = assignFromMatrixAsIndices(wv, P);
    count = count + l;
    idx = idx * flag + 1;
    lim = wv(K - idx);
  }

  PN.block(count - 1, 0, l, K) = assignFromMatrixAsIndices(final_vector, P);

  return PN;
}

MatrixXYu Combinations::permutations(unsigned N)
{
  // Helper function for the permutationsNoReplacement
  // Adapted from Matlab code by:
  //            Author:   Matt Fig
  //            Contact:  popkenai@yahoo.com
  //            Date:     5/30/2009
  //            Reference:  http://mathworld.wolfram.com/BallPicking.html
  MatrixXYu P(1, 1);
  MatrixXYu q;
  MatrixXYu t;
  MatrixXYu tIndices;
  P << 1;
  unsigned m;
  unsigned a;
  unsigned b;
  unsigned numIndices;
  RowXd g = cumulativeProduct(1, N - 1); // Holds the sizes of P
  for (unsigned n = 2; n <= N; ++n)
  {
    q = P;
    m = g(n - 2);
    P.setZero(n * m, n);
    P.topLeftCorner(m, 1).setConstant(n);
    P.topRightCorner(m, n - 1) = q;
    a = m + 1;

    for (unsigned i = n - 1; i >= 1; --i)
    {
      t = q;
      t = setWhereEqualToConst(t, i, n);
      b = a + m - 1;
      numIndices = b - a + 1;
      P.block(a - 1, 0, numIndices, 1).setConstant(i);
      P.block(a - 1, 1, numIndices, n - 1) = t;
      a = b + 1;
    }
  }
  return P;
}

unsigned Combinations::consecutiveProduct(unsigned k, unsigned n)
{
  unsigned prod = 1;
  for (unsigned i = k; i <= n; ++i)
    prod *= i;
  return prod;
}

RowXd Combinations::cumulativeProduct(unsigned k, unsigned n)
{
  RowXd cumulator(n - k + 1);
  unsigned idx = 0;
  cumulator(idx) = k;
  for (unsigned i = k + 1; i <= n; ++i)
  {
    idx++;
    cumulator(idx) = cumulator(idx - 1) * i;
  }
  return cumulator;
}

MatrixXYu Combinations::assignFromMatrixAsIndices(const RowXu& vec, const MatrixXYu& indices)
{
  // Build a matrix using the values from a vector "vec" using the coefficients in the matrix "indices".
  unsigned n = indices.rows();
  unsigned m = indices.cols();
  unsigned idx;
  MatrixXYu mat(n, m);
  for (unsigned i = 0; i < n; ++i)
  {
    for (unsigned j = 0; j < m; ++j)
    {
      idx = indices(i, j) - 1;
      mat(i, j) = vec(idx);
    }
  }
  return mat;
}

MatrixXYu Combinations::setWhereEqualToConst(const MatrixXYu& matrix, const unsigned & checkNum,
                                             const unsigned & setNum)
{
  MatrixXYu out_matrix = matrix;
  unsigned n = matrix.rows();
  unsigned m = matrix.cols();
  for (unsigned i = 0; i < n; ++i)
  {
    for (unsigned j = 0; j < m; ++j)
    {
      if (out_matrix(i, j) == checkNum)
      {
        out_matrix(i, j) = setNum;
      }
    }
  }
  return out_matrix;
}

} // namepsace
