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

#include <Eigen/Dense>

namespace dart::math {

/// @page lcp LCP
///
/// Linear complementarity problem (LCP) in the following form:
///
/// @f[ A x + b \ge 0 @f]
/// @f[ x \ge 0 @f]
/// @f[ x^T (A x + b) = 0 @f]
///
/// where @f$ A @f$ is a square matrix, @f$ b @f$ is a vector, and @f$ x @f$
/// is a vector of unknowns.
///
/// Note that some formulations use M z + q = w notation instead of A x + b = y.
///
/// There are two types of LCP solvers in DART: direct and iterative (or
/// splitting). If A is symmetric and positive semi-definite, then the splitting
/// schemes will always converge.

template <typename S>
[[nodiscard]] S defaultLcpTolerance()
{
  if constexpr (std::is_same_v<S, float>)
    return 1e-3;
  else if constexpr (std::is_same_v<S, double>)
    return 1e-6;
  else if constexpr (std::is_same_v<S, long double>)
    return 1e-9;
}

template <typename S>
struct LcpOption
{
  /// Maximum number of iterations.
  std::size_t maxIterations = 1000;

  /// True if the given x is used as a initial guess. Otherwise, x is set to
  /// zero.
  bool warmStart = false;

  /// Tolerance for the termination condition.
  S tolerance = defaultLcpTolerance<S>();

  S lambda = 0.5;
};

/// Result of solving LCP
/// @tparam S Scalar type
template <typename S>
struct LcpResult
{
  /// Status of solving LCP
  enum class Status
  {
    SOLVED = 0,
    REACHED_MAX_ITERATION,
    NEGATIVE_DIAGONAL,
    UNSOLVABLE,
    UNKNOWN,
  };

  /// Message of solving LCP
  std::string message;

  /// Status of solving LCP
  Status status{Status::UNSOLVABLE};

  S error{0};
};

/// @brief Solve LCP using Lemke's algorithm
/// @tparam S Scalar type
/// @tparam DerivedA Derived type of A
/// @tparam DerivedB Derived type of b
/// @param A Matrix A in LCP formulation
/// @param b Vector b in LCP formulation
/// @param x Vector x in LCP formulation
/// @param option Options for solving LCP
/// @param result Result of solving LCP
/// @return True if LCP is solved successfully
template <
    typename DerivedA,
    typename DerivedB,
    typename DerivedX,
    typename S = typename DerivedA::Scalar>
bool solveLcpLemke(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::PlainObjectBase<DerivedX>* x,
    const LcpOption<S>& option = LcpOption<S>(),
    LcpResult<S>* result = nullptr);

/// @brief Solve LCP using projected Gauss-Seidel method
/// @tparam S Scalar type
/// @tparam DerivedA Derived type of A
/// @tparam DerivedB Derived type of b
/// @param A Matrix A in LCP formulation
/// @param b Vector b in LCP formulation
/// @param x Vector x in LCP formulation
/// @param option Options for solving LCP
/// @param result Result of solving LCP
/// @return True if LCP is solved successfully
template <
    typename DerivedA,
    typename DerivedB,
    typename DerivedX,
    typename S = typename DerivedA::Scalar>
bool solveLcpPgs(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::PlainObjectBase<DerivedX>* x,
    const LcpOption<S>& option = LcpOption<S>(),
    LcpResult<S>* result = nullptr);

/// Solve LCP using projected successive over-relaxation method
/// @tparam S Scalar type
/// @tparam DerivedA Derived type of A
/// @tparam DerivedB Derived type of b
/// @param A Matrix A in LCP formulation
/// @param b Vector b in LCP formulation
/// @param x Vector x in LCP formulation
/// @param option Options for solving LCP
/// @param result Result of solving LCP
/// @return True if LCP is solved successfully
template <
    typename DerivedA,
    typename DerivedB,
    typename DerivedX,
    typename S = typename DerivedA::Scalar>
bool solveLcpPsor(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::PlainObjectBase<DerivedX>* x,
    const LcpOption<S>& option = LcpOption<S>(),
    LcpResult<S>* result = nullptr);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

#include "dart/common/Logging.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/lcp/Utils.hpp"
#include "dart/math/lcp/detail/Lemke.hpp"

#include <vector>

namespace dart::math {

namespace {

//==============================================================================
template <
    typename DerivedB,
    typename DerivedX,
    typename DerivedY,
    typename S = typename DerivedX::Scalar>
[[nodiscard]] S computeLcpErrorFromXandY(
    const math::MatrixBase<DerivedB>& b,
    const math::MatrixBase<DerivedX>& x,
    const math::MatrixBase<DerivedY>& y)
{
  S error = 0;

  const auto& n = x.rows();
  for (auto i = 0; i < n; ++i) {
    const S localError = x[i] - std::max(S(0), (x[i] - y[i]));
    error += localError * localError;
  }
  error = std::sqrt(error);

  const S squaredNorm = b.squaredNorm();
  if (squaredNorm > eps<S>())
    error /= squaredNorm;

  return error;
}

} // namespace

//==============================================================================
template <typename DerivedA, typename DerivedB, typename DerivedX, typename S>
bool solveLcpLemke(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::PlainObjectBase<DerivedX>* x,
    const LcpOption<S>& option,
    LcpResult<S>* result)
{
  (void)option;

  const int errorCode = Lemke<S>(A, b, x, option.maxIterations);

  if (result) {
    if (errorCode == 0)
      (*result).status = LcpResult<S>::Status::SOLVED;
    // TODO(JS): Parse move error codes
    else
      (*result).status = LcpResult<S>::Status::UNKNOWN;
  }

  return (errorCode == 0);
}

//==============================================================================
template <typename DerivedA, typename DerivedB, typename DerivedX, typename S>
bool solveLcpPgs(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::PlainObjectBase<DerivedX>* x,
    const LcpOption<S>& option,
    LcpResult<S>* result)
{
  LcpOption<S> optionPgs = option;
  optionPgs.lambda = S(1);
  return solveLcpPsor(A, b, x, optionPgs, result);
}

//==============================================================================
template <typename DerivedA, typename DerivedB, typename DerivedX, typename S>
bool solveLcpPsor(
    const math::MatrixBase<DerivedA>& A,
    const math::MatrixBase<DerivedB>& b,
    math::PlainObjectBase<DerivedX>* x,
    const LcpOption<S>& option,
    LcpResult<S>* result)
{
  const std::size_t n = A.rows();

  // Initialize x
  if (option.warmStart)
    (*x).conservativeResize(n);
  else
    (*x).setZero(n);

  if (n == 0)
    return true;

  // Check if all the diagonal elements are non-negative
  for (std::size_t i = 0u; i < n; ++i) {
    if ((A.diagonal().array() < eps<S>()).any()) {
      if (result) {
        (*result).status = LcpResult<S>::Status::NEGATIVE_DIAGONAL;
      }
      return false;
    }
  }

  const math::VectorX<S> invM = A.diagonal().cwiseInverse();
  S new_x;
  bool converged = false;
  std::size_t iteration = 0u;

  math::VectorX<S> Ax_b;
  Ax_b = b;
  Ax_b.noalias() += A * (*x);

  while (true) {
    // Sweep forward and backward alternatively
    if (iteration % 2) {
      for (std::size_t i = n - 1; i == 0u; --i) {
        new_x = (*x)[i] - option.lambda * Ax_b[i] * invM[i];
        (*x)[i] = std::max(S(0), new_x); // project
      }
    } else {
      for (std::size_t i = 0u; i < n; ++i) {
        new_x = (*x)[i] - option.lambda * Ax_b[i] * invM[i];
        (*x)[i] = std::max(S(0), new_x); // project
      }
    }

    Ax_b = b;
    Ax_b.noalias() += A * (*x);

    S error_norm = computeLcpErrorFromXandY(b, *x, Ax_b);
    if (error_norm < option.tolerance) {
      converged = true;
      break;
    }

    if (++iteration >= option.maxIterations) {
      break;
    }
  }

  if (result) {
    if (converged)
      (*result).status = LcpResult<S>::Status::SOLVED;
    else
      (*result).status = LcpResult<S>::Status::REACHED_MAX_ITERATION;
  }

  return converged;
}

} // namespace dart::math
