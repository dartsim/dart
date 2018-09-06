/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_MATH_DETAIL_RANDOM_HPP_
#define DART_MATH_DETAIL_RANDOM_HPP_

#include "dart/math/Random.hpp"

namespace dart {
namespace math {

//==============================================================================
template <typename S>
S Random::uniform(
    S min,
    S max,
    typename std::enable_if<std::is_floating_point<S>::value>::type*)
{
  // Distribution objects are lightweight so we simply construct a new
  // distribution for each random number generation.
  UniformRealDist<S> d(min, max);
  return d(getRandGenerator());
}

//==============================================================================
template <typename S>
S Random::uniform(
    S min,
    S max,
    typename std::
        enable_if<detail::is_compatible_to_uniform_int_distribution<S>::value>::
            type*)
{
  // Distribution objects are lightweight so we simply construct a new
  // distribution for each random number generation.
  UniformIntDist<S> d(min, max);
  return d(getRandGenerator());
}

//==============================================================================
template <typename S>
S Random::uniform(S limit)
{
  return uniform(-std::abs(limit), std::abs(limit));
}

//==============================================================================
template <typename Derived>
typename Derived::PlainObject Random::uniform(
    const Eigen::MatrixBase<Derived>& min,
    const Eigen::MatrixBase<Derived>& max)
{
  const auto uniformFunc
      = [&](int i, int j) { return uniform(min(i, j), max(i, j)); };
  return Derived::PlainObject::NullaryExpr(min.rows(), min.cols(), uniformFunc);
}

//==============================================================================
template <typename Derived>
typename Derived::PlainObject Random::uniformVector(
    const Eigen::MatrixBase<Derived>& min,
    const Eigen::MatrixBase<Derived>& max)
{
  return uniform(min, max);
}

//==============================================================================
template <int N, typename S>
Eigen::Matrix<S, N, 1> Random::uniformVector(S min, S max)
{
  return uniformVector(
      Eigen::Matrix<S, N, 1>::Constant(min),
      Eigen::Matrix<S, N, 1>::Constant(max));
}

//==============================================================================
template <int N, typename S>
Eigen::Matrix<S, N, 1> Random::uniformVector(S limit)
{
  return uniformVector<N>(-std::abs(limit), std::abs(limit));
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
Random::uniformVectorX(
    const Eigen::MatrixBase<Derived>& min,
    const Eigen::MatrixBase<Derived>& max)
{
  return uniformVector(min, max);
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, 1> Random::uniformVectorX(
    int size, S min, S max)
{
  return uniformVector(
      Eigen::Matrix<S, Eigen::Dynamic, 1>::Constant(size, min),
      Eigen::Matrix<S, Eigen::Dynamic, 1>::Constant(size, max));
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, 1> Random::uniformVectorX(int size, S limit)
{
  return uniformVector(size, -std::abs(limit), std::abs(limit));
}

//==============================================================================
template <typename Derived>
typename Derived::PlainObject Random::uniformMatrix(
    const Eigen::MatrixBase<Derived>& min,
    const Eigen::MatrixBase<Derived>& max)
{
  return uniform(min, max);
}

//==============================================================================
template <int Rows, int Cols, typename S>
Eigen::Matrix<S, Rows, Cols> Random::uniformMatrix(S min, S max)
{
  return uniformMatrix(
      Eigen::Matrix<S, Rows, Cols>::Constant(min),
      Eigen::Matrix<S, Rows, Cols>::Constant(max));
}

//==============================================================================
template <int Rows, int Cols, typename S>
Eigen::Matrix<S, Rows, Cols> Random::uniformMatrix(S limit)
{
  return uniformMatrix(-std::abs(limit), std::abs(limit));
}

//==============================================================================
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
Random::uniformMatrixX(
    const Eigen::MatrixBase<Derived>& min,
    const Eigen::MatrixBase<Derived>& max)
{
  return uniformMatrix(min, max);
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> Random::uniformMatrixX(
    int size, S min, S max)
{
  return uniformMatrix(
      Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>::Constant(size, min),
      Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>::Constant(size, max));
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> Random::uniformMatrixX(
    int size, S limit)
{
  return uniformMatrix(size, -std::abs(limit), std::abs(limit));
}

//==============================================================================
template <typename S>
S Random::normal(
    S min,
    S max,
    typename std::enable_if<std::is_floating_point<S>::value>::type*)
{
  NormalRealDist<S> d(min, max);
  return d(getRandGenerator());
}

//==============================================================================
template <typename S>
S Random::normal(
    S mean,
    S sigma,
    typename std::
        enable_if<detail::is_compatible_to_uniform_int_distribution<S>::value>::
            type*)
{
  using DefaultFloatType = float;
  const DefaultFloatType realNormal = normal(
      static_cast<DefaultFloatType>(mean),
      static_cast<DefaultFloatType>(sigma));
  return static_cast<S>(std::round(realNormal));
}

} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_RANDOM_HPP_
