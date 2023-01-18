/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/math/Fwd.hpp>

namespace dart::math {

/// Solves LCP using Lemke's algorithm
template <typename S, typename DerivedA, typename DerivedB, typename DerivedX>
int Lemke(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::MatrixBase<DerivedX>* x,
    std::size_t maxiter = 1000,
    const S zer_tol = 1e-5,
    const S piv_tol = 1e-8);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

#include "dart/common/Logging.hpp"
#include "dart/math/lcp/Utils.hpp"

#include <vector>

namespace dart::math {

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB, typename DerivedX>
int Lemke(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::MatrixBase<DerivedX>* x,
    std::size_t maxiter,
    const S zer_tol,
    const S piv_tol)
{
  static_assert(
      std::is_same_v<S, typename DerivedA::Scalar>,
      "S and A must have the same scalar type.");
  static_assert(
      std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar>,
      "A and b must have the same scalar type.");
  static_assert(
      std::is_same_v<typename DerivedB::Scalar, typename DerivedX::Scalar>,
      "b and x must have the same scalar type.");

  int n = b.size();

  int err = 0;

  if (b.minCoeff() >= 0) {
    DART_DEBUG("Trivial solution exists.");
    *x = math::VectorX<S>::Zero(n);
    return err;
  }

  // solve trivial case for n=1
  if (n == 1) {
    if (A(0) > 0) {
      (*x)[0] = -b(0) / A(0);
      return err;
    } else {
      *x = math::VectorX<S>::Zero(n);
      err = 4; // no solution
      return err;
    }
  }

  *x = math::VectorX<S>::Zero(2 * n);
  std::size_t iter = 0;
  // S theta = 0;
  S ratio = 0;
  int leaving = 0;
  math::VectorX<S> Be = math::VectorX<S>::Constant(n, 1);
  math::VectorX<S> tempX = b;
  std::vector<int> bas;
  std::vector<int> nonbas;

  int t = 2 * n;
  int entering = t;

  bas.clear();
  nonbas.clear();

  // TODO: here suppose initial guess z0 is [0,0,0,...], this contradicts to
  // ODE's w always initilized as 0
  for (int i = 0; i < n; ++i) {
    nonbas.push_back(i);
  }

  math::MatrixX<S> B = -math::MatrixX<S>::Identity(n, n);

  if (!bas.empty()) {
    math::MatrixX<S> B_copy = B;
    for (std::size_t i = 0; i < bas.size(); ++i) {
      B.col(i) = A.col(bas[i]);
    }
    for (std::size_t i = 0; i < nonbas.size(); ++i) {
      B.col(bas.size() + i) = B_copy.col(nonbas[i]);
    }
    // TODO: check the condition number to return err = 3
    math::JacobiSVD<math::MatrixX<S>> svd(B);
    S cond = svd.singularValues()(0)
             / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond > 1e+16) {
      (*x) = math::VectorX<S>::Zero(n);
      err = 3;
      return err;
    }
    tempX = -B.householderQr().solve(b);
  }

  // Check if initial basis provides solution
  if (tempX.minCoeff() >= 0) {
    math::VectorX<S> _x = math::VectorX<S>::Zero(2 * n);
    for (std::size_t i = 0; i < bas.size(); ++i) {
      (_x).row(bas[i]) = tempX.row(i);
    }
    (*x) = _x.head(n);
    return err;
  }

  // Determine initial leaving variable
  math::VectorX<S> minuxX = -tempX;
  int lvindex;
  S tval = minuxX.maxCoeff(&lvindex);
  for (std::size_t i = 0; i < nonbas.size(); ++i) {
    bas.push_back(nonbas[i] + n);
  }
  leaving = bas[lvindex];

  bas[lvindex] = t; // pivoting in the artificial variable

  math::VectorX<S> U = math::VectorX<S>::Zero(n);
  for (int i = 0; i < n; ++i) {
    if (tempX[i] < 0)
      U[i] = 1;
  }
  Be = -(B * U);
  tempX.noalias() += tval * U;
  tempX[lvindex] = tval;
  B.col(lvindex) = Be;

  for (iter = 0; iter < maxiter; ++iter) {
    if (leaving == t) {
      break;
    } else if (leaving < n) {
      entering = n + leaving;
      Be = math::VectorX<S>::Zero(n);
      Be[leaving] = -1;
    } else {
      entering = leaving - n;
      Be = A.col(entering);
    }

    const math::VectorX<S> d = B.householderQr().solve(Be);

    // Find new leaving variable
    std::vector<int> j;
    for (int i = 0; i < n; ++i) {
      if (d[i] > piv_tol)
        j.push_back(i);
    }
    if (j.empty()) // no new pivots - ray termination
    {
      err = 2;
      break;
    }

    std::size_t jSize = j.size();
    math::VectorX<S> minRatio(jSize);
    for (std::size_t i = 0; i < jSize; ++i) {
      minRatio[i] = (tempX[j[i]] + zer_tol) / d[j[i]];
    }
    S theta = minRatio.minCoeff();

    std::vector<int> tmpJ;
    std::vector<S> tmpd;
    for (std::size_t i = 0; i < jSize; ++i) {
      if (tempX[j[i]] / d[j[i]] <= theta) {
        tmpJ.push_back(j[i]);
        tmpd.push_back(d[j[i]]);
      }
    }

    //    if (tmpJ.empty())
    //    {
    //      LOG(WARNING) << "tmpJ should never be empty!!!";
    //      LOG(WARNING) << "dumping data:";
    //      LOG(WARNING) << "theta:" << theta;
    //      for (int i = 0; i < jSize; ++i)
    //      {
    //        LOG(WARNING) << "x(" << j[i] << "): " << x[j[i]] << "d: " <<
    //        d[j[i]];
    //      }
    //    }

    j = tmpJ;
    jSize = j.size();
    if (jSize == 0) {
      err = 4;
      break;
    }
    lvindex = -1;

    // Check if artificial among these
    for (std::size_t i = 0; i < jSize; ++i) {
      if (bas[j[i]] == t)
        lvindex = i;
    }

    if (lvindex != -1) {
      lvindex = j[lvindex]; // Always use artificial if possible
    } else {
      theta = tmpd[0];
      lvindex = 0;
      for (std::size_t i = 0; i < jSize; ++i) {
        if (tmpd[i] - theta > piv_tol) { // Bubble sorting
          theta = tmpd[i];
          lvindex = i;
        }
      }
      lvindex = j[lvindex]; // choose the first if there are multiple
    }

    leaving = bas[lvindex];

    ratio = tempX[lvindex] / d[lvindex];

    //    bool bDiverged = false;
    //    for (int i = 0; i < n; ++i) {
    //      if (isnan(x[i]) || isinf(x[i])) {
    //        bDiverged = true;
    //        break;
    //      }
    //    }
    //    if (bDiverged) {
    //      err = 4;
    //      break;
    //    }

    // Perform pivot
    tempX.noalias() -= ratio * d;
    tempX[lvindex] = ratio;
    B.col(lvindex) = Be;
    bas[lvindex] = entering;
  }

  if (iter >= maxiter && leaving != t) {
    err = 1;
  }

  if (err == 0) {
    for (std::size_t i = 0; i < bas.size(); ++i) {
      if (bas[i] < x->size()) {
        (*x)[bas[i]] = tempX[i];
      }
    }

    math::VectorX<S> _x = x->head(n);
    *x = _x;

    if (!validateLcp<S>(A, b, *x)) {
      // x = VectorX<S>::Zero(n);
      err = 3;
    }
  } else {
    *x = math::VectorX<S>::Zero(n); // solve failed, return a 0 vector
  }

  //  if (err == 1)
  //    LOG(ERROR) << "LCP Solver: Iterations exceeded limit";
  //  else if (err == 2)
  //    LOG(ERROR) << "LCP Solver: Unbounded ray";
  //  else if (err == 3)
  //    LOG(ERROR) << "LCP Solver: Solver converged with numerical issues. "
  //               << "Validation failed.";
  //  else if (err == 4)
  //    LOG(ERROR) << "LCP Solver: Iteration diverged.";

  return err;
}

} // namespace dart::math
