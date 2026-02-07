/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/simd/config.hpp>
#include <dart/simd/detail/math/constants.hpp>
#include <dart/simd/detail/math/polynomial.hpp>
#include <dart/simd/fwd.hpp>

#include <cmath>

namespace dart::simd::detail::math {

template <std::size_t W>
DART_SIMD_INLINE Vec<float, W> logSimdScalar(const Vec<float, W>& x)
{
  using VecT = Vec<float, W>;
  constexpr std::size_t A = (W * sizeof(float) >= 64)   ? 64
                            : (W * sizeof(float) >= 32) ? 32
                                                        : 16;
  alignas(A) float tmp[W];
  alignas(A) float resultTmp[W];
  x.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::log(tmp[i]);
  }
  return VecT::load(resultTmp);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> logSimdSimd(const Vec<float, W>& x)
{
  using VecT = Vec<float, W>;
  using MaskT = VecMask<float, W>;

  constexpr float invSqrt2 = 0.70710678118654752440f;
  constexpr float log2Hi = 0.693359375f;
  constexpr float log2Lo = -2.121944400546905827679e-4f;

  MaskT validMask = x >= VecT::broadcast(0.0f);

  constexpr float subnormScale = 8388608.0f;
  constexpr float fltMin = 1.175494351e-38f;
  MaskT isSubnormal
      = (x > VecT::broadcast(0.0f)) & (x < VecT::broadcast(fltMin));
  VecT xNorm = select(isSubnormal, x * VecT::broadcast(subnormScale), x);

  auto [xm, e_int] = frexpSimd(xNorm);
  VecT e = convertToFloat(e_int);
  e = select(isSubnormal, e - VecT::broadcast(23.0f), e);

  MaskT maskGeInvSqrt2 = xm >= VecT::broadcast(invSqrt2);
  e = select(maskGeInvSqrt2, e + VecT::broadcast(1.0f), e);
  VecT xmAdj = select(maskGeInvSqrt2, xm, xm + xm);
  xmAdj = xmAdj - VecT::broadcast(1.0f);

  VecT y = estrin(
      xmAdj,
      3.3333331174e-1f,
      -2.4999993993e-1f,
      2.0000714765e-1f,
      -1.6668057665e-1f,
      1.4249322787e-1f,
      -1.2420140846e-1f,
      1.1676998740e-1f,
      -1.1514610310e-1f,
      7.0376836292e-2f);

  VecT z = xmAdj * xmAdj;
  y = y * xmAdj * z;
  y = fmadd(e, VecT::broadcast(log2Lo), y);

  VecT r = xmAdj + fmadd(VecT::broadcast(-0.5f), z, y);
  r = fmadd(e, VecT::broadcast(log2Hi), r);

  VecT nInf = VecT::broadcast(-MathConstants<float>::infinity);
  VecT pInf = VecT::broadcast(MathConstants<float>::infinity);
  VecT nanVal = VecT::broadcast(MathConstants<float>::nan);

  r = select(x == pInf, pInf, r);
  r = select(x == VecT::broadcast(0.0f), nInf, r);
  r = select(~validMask, nanVal, r);

  return r;
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> logSimd(const Vec<float, W>& x)
{
  if constexpr (::dart::simd::detail::HasFullIntegerSimd<W>::value) {
    return logSimdSimd<W>(x);
  } else {
    return logSimdScalar<W>(x);
  }
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> logSimd(const Vec<double, W>& x)
{
  using VecT = Vec<double, W>;
  using MaskT = VecMask<double, W>;

  MaskT validMask = x >= VecT::broadcast(0.0);

  constexpr std::size_t A = (W * sizeof(double) >= 64)   ? 64
                            : (W * sizeof(double) >= 32) ? 32
                                                         : 16;
  alignas(A) double x_arr[W];
  x.store(x_arr);

  alignas(A) double resultArr[W];
  for (std::size_t i = 0; i < W; ++i) {
    resultArr[i] = std::log(x_arr[i]);
  }
  VecT r = VecT::load(resultArr);

  VecT nanVal = VecT::broadcast(MathConstants<double>::nan);
  r = select(~validMask, nanVal, r);

  return r;
}

} // namespace dart::simd::detail::math
