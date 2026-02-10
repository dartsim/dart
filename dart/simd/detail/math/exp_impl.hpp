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
DART_SIMD_INLINE Vec<float, W> expSimdScalar(const Vec<float, W>& x)
{
  using VecT = Vec<float, W>;
  constexpr std::size_t A = (W * sizeof(float) >= 64)   ? 64
                            : (W * sizeof(float) >= 32) ? 32
                                                        : 16;
  alignas(A) float tmp[W];
  alignas(A) float resultTmp[W];
  x.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::exp(tmp[i]);
  }
  return VecT::load(resultTmp);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> expSimdSimd(const Vec<float, W>& x)
{
  using VecT = Vec<float, W>;
  using VecI = Vec<std::int32_t, W>;
  using MaskT = VecMask<float, W>;

  constexpr float range = 88.3762588501f;
  constexpr float invLog2 = 1.44269504088896341f;
  constexpr float nlog2Hi = -0.693359375f;
  constexpr float nlog2Lo = 2.12194440e-4f;

  MaskT inSimdRange
      = (x >= VecT::broadcast(-87.0f)) & (x <= VecT::broadcast(range));

  VecT n = floor(fmadd(VecT::broadcast(invLog2), x, VecT::broadcast(0.5f)));
  VecT y = fmadd(n, VecT::broadcast(nlog2Hi), x);
  y = fmadd(n, VecT::broadcast(nlog2Lo), y);

  VecT z = estrin(
      y,
      5.0000001201e-1f,
      1.6666665459e-1f,
      4.1665795894e-2f,
      8.3334519073e-3f,
      1.3981999507e-3f,
      1.9875691500e-4f);
  z = fmadd(z, y * y, y + VecT::broadcast(1.0f));

  MaskT nInRange
      = (n >= VecT::broadcast(-126.0f)) & (n <= VecT::broadcast(127.0f));
  VecT nSafe = select(nInRange, n, VecT::broadcast(0.0f));
  VecI n_int = convertToInt(nSafe);
  VecT result = ldexpSimd(z, n_int);

  constexpr std::size_t A = (W * sizeof(float) >= 64)   ? 64
                            : (W * sizeof(float) >= 32) ? 32
                                                        : 16;
  alignas(A) float inRangeArr[W];
  select(inSimdRange, VecT::broadcast(1.0f), VecT::broadcast(0.0f))
      .store(inRangeArr);
  bool needScalar = false;
  for (std::size_t i = 0; i < W; ++i) {
    if (inRangeArr[i] == 0.0f) {
      needScalar = true;
      break;
    }
  }
  if (needScalar) {
    alignas(A) float xArr[W];
    alignas(A) float scalarArr[W];
    x.store(xArr);
    for (std::size_t i = 0; i < W; ++i) {
      scalarArr[i] = std::exp(xArr[i]);
    }
    result = select(inSimdRange, result, VecT::load(scalarArr));
  }

  return result;
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> expSimd(const Vec<float, W>& x)
{
  if constexpr (::dart::simd::detail::HasFullIntegerSimd<W>::value) {
    return expSimdSimd<W>(x);
  } else {
    return expSimdScalar<W>(x);
  }
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> expSimd(const Vec<double, W>& x)
{
  using VecT = Vec<double, W>;
  using MaskT = VecMask<double, W>;

  constexpr double range = 709.43613930310391424428;
  constexpr double invLog2 = 1.44269504088896340736;
  constexpr double nlog2Hi = -0.693145751953125;
  constexpr double nlog2Lo = -1.42860682030941723212e-6;

  MaskT inSimdRange
      = (x >= VecT::broadcast(-708.0)) & (x <= VecT::broadcast(range));

  VecT n = floor(fmadd(VecT::broadcast(invLog2), x, VecT::broadcast(0.5)));
  VecT y = fmadd(n, VecT::broadcast(nlog2Hi), x);
  y = fmadd(n, VecT::broadcast(nlog2Lo), y);

  VecT z = y * y;

  VecT p = estrin(
               z,
               9.99999999999999999910e-1,
               3.02994407707441961300e-2,
               1.26177193074810590878e-4)
           * y;

  VecT q = estrin(
      z,
      2.00000000000000000009e0,
      2.27265548208155028766e-1,
      2.52448340349684104192e-3,
      3.00198505138664455042e-6);

  VecT resultFrac = p / (q - p);
  resultFrac = resultFrac + resultFrac + VecT::broadcast(1.0);

  constexpr std::size_t A = (W * sizeof(double) >= 64)   ? 64
                            : (W * sizeof(double) >= 32) ? 32
                                                         : 16;
  alignas(A) double nArr[W];
  alignas(A) double fracArr[W];
  n.store(nArr);
  resultFrac.store(fracArr);

  alignas(A) double resultArr[W];
  for (std::size_t i = 0; i < W; ++i) {
    const bool nInRange = nArr[i] >= -1022.0 && nArr[i] <= 1023.0;
    const int exp
        = (std::isnan(nArr[i]) || !nInRange) ? 0 : static_cast<int>(nArr[i]);
    resultArr[i] = std::ldexp(fracArr[i], exp);
  }
  VecT result = VecT::load(resultArr);

  alignas(A) double inRangeArr[W];
  select(inSimdRange, VecT::broadcast(1.0), VecT::broadcast(0.0))
      .store(inRangeArr);
  bool needScalar = false;
  for (std::size_t i = 0; i < W; ++i) {
    if (inRangeArr[i] == 0.0) {
      needScalar = true;
      break;
    }
  }
  if (needScalar) {
    alignas(A) double xArr[W];
    alignas(A) double scalarArr[W];
    x.store(xArr);
    for (std::size_t i = 0; i < W; ++i) {
      scalarArr[i] = std::exp(xArr[i]);
    }
    result = select(inSimdRange, result, VecT::load(scalarArr));
  }

  return result;
}

} // namespace dart::simd::detail::math
