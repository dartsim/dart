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
inline void Random::setSeed(unsigned int seed)
{
  std::seed_seq seq{seed};
  getSeedMutable() = seed;
  getRandGenerator().seed(seq);
}

//==============================================================================
inline unsigned int Random::getSeed()
{
  return getSeedMutable();
}

//==============================================================================
template <typename S>
S Random::uniform(
    S min,
    S max,
    typename std::enable_if<std::is_floating_point<S>::value>::type*)
{
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
  UniformIntDist<S> d(min, max);
  return d(getRandGenerator());
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

//==============================================================================
inline uint32_t& Random::getSeedMutable()
{
  static uint32_t seed = std::random_device{}();
  return seed;
}

//==============================================================================
inline Random::GeneratorType& Random::getRandGenerator()
{
  static GeneratorType randGenerator(getSeed());
  return randGenerator;
}

} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_RANDOM_HPP_
