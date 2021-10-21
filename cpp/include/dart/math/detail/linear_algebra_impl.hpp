/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cmath>

#include "dart/common/logging.hpp"
#include "dart/math/constant.hpp"
#include "dart/math/linear_algebra.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
math::Matrix<typename Derived::Scalar, 3, 3> skew(
    const math::MatrixBase<Derived>& vec)
{
  using Scalar = typename Derived::Scalar;

  // clang-format off
  return (math::Matrix<Scalar, 3, 3>() <<
          0, -vec[2], +vec[1],
    +vec[2],       0, -vec[0],
    -vec[1], +vec[0],       0
  ).finished();
  // clang-format on
}

//==============================================================================
template <typename Derived>
math::Matrix<typename Derived::Scalar, 3, 1> unskew(
    const math::MatrixBase<Derived>& mat)
{
  using Scalar = typename Derived::Scalar;

#ifndef NDEBUG
  if (std::abs(mat(0, 0)) > eps<Scalar>() || std::abs(mat(1, 1)) > eps<Scalar>()
      || std::abs(mat(2, 2)) > eps<Scalar>()) {
    DART_DEBUG("Not skew a symmetric matrix");
  }

  // TODO(JS): Check skew-symmetry
#endif
  return math::Vector3<Scalar>(mat(2, 1), mat(0, 2), mat(1, 0));
}

} // namespace dart::math
