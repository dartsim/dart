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

template <typename S>
[[nodiscard]] S defaultLcpValidationTolerance()
{
  if constexpr (std::is_same_v<S, float>)
    return 5e-1;
  else if constexpr (std::is_same_v<S, double>)
    return 5e-5;
  else if constexpr (std::is_same_v<S, long double>)
    return 5e-8;
}

template <typename S>
S computeLcpError(
    const math::MatrixX<S>& A,
    const math::VectorX<S>& b,
    const math::VectorX<S>& x);

template <typename S>
bool validateLcp(
    const math::MatrixX<S>& A,
    const math::VectorX<S>& b,
    const math::VectorX<S>& x,
    S tol = defaultLcpValidationTolerance<S>());

} // namespace dart::math

#include <dart/math/Constants.hpp>

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

template <typename S>
S computeLcpError(
    const math::MatrixX<S>& A,
    const math::VectorX<S>& b,
    const math::VectorX<S>& x)
{
  const auto n = x.size();
  const math::VectorX<S> y = A * x + b;

  S error = 0;

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

template <typename S>
bool validateLcp(
    const math::MatrixX<S>& A,
    const math::VectorX<S>& b,
    const math::VectorX<S>& x,
    S tol)
{
  const S error = computeLcpError(A, b, x);
  return error <= tol;
}

} // namespace dart::math
