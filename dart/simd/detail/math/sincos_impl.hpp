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

#include <utility>

#include <cmath>

namespace dart::simd::detail::math {

template <bool ComputeSin, bool ComputeCos, std::size_t W>
DART_SIMD_INLINE void sincosImplScalar(
    Vec<float, W> x, Vec<float, W>* sinOut, Vec<float, W>* cosOut)
{
  using VecT = Vec<float, W>;
  constexpr std::size_t A = (W * sizeof(float) >= 64)   ? 64
                            : (W * sizeof(float) >= 32) ? 32
                                                        : 16;
  alignas(A) float tmp[W];
  x.store(tmp);
  if constexpr (ComputeSin && ComputeCos) {
    alignas(A) float sinTmp[W];
    alignas(A) float cosTmp[W];
    for (std::size_t i = 0; i < W; ++i) {
      sinTmp[i] = std::sin(tmp[i]);
      cosTmp[i] = std::cos(tmp[i]);
    }
    *sinOut = VecT::load(sinTmp);
    *cosOut = VecT::load(cosTmp);
  } else if constexpr (ComputeSin) {
    alignas(A) float resultTmp[W];
    for (std::size_t i = 0; i < W; ++i) {
      resultTmp[i] = std::sin(tmp[i]);
    }
    *sinOut = VecT::load(resultTmp);
  } else if constexpr (ComputeCos) {
    alignas(A) float resultTmp[W];
    for (std::size_t i = 0; i < W; ++i) {
      resultTmp[i] = std::cos(tmp[i]);
    }
    *cosOut = VecT::load(resultTmp);
  }
}

template <bool ComputeSin, bool ComputeCos, std::size_t W>
DART_SIMD_INLINE void sincosImplSimd(
    Vec<float, W> x, Vec<float, W>* sinOut, Vec<float, W>* cosOut)
{
  using VecT = Vec<float, W>;
  using VecI = Vec<std::int32_t, W>;
  using MaskT = VecMask<float, W>;

  constexpr float fourOverPi = 1.2732395447351626862f;

  VecI xSign;
  if constexpr (ComputeSin) {
    xSign = bitAnd(
        reinterpretAsInt(x),
        VecI::broadcast(static_cast<std::int32_t>(0x80000000)));
  }

  VecT xa = abs(x);
  VecT jFloat = xa * VecT::broadcast(fourOverPi);
  constexpr float maxJ = 16777216.0f;
  MaskT inRange
      = (jFloat >= VecT::broadcast(0.0f)) & (jFloat <= VecT::broadcast(maxJ));
  VecT jSafe = select(inRange, jFloat, VecT::broadcast(0.0f));
  VecI j = convertToInt(jSafe);
  j = bitAnd(j + VecI::broadcast(1), VecI::broadcast(~1));

  VecT y = convertToFloat(j);

  constexpr float pio4Part1 = 0.78515625f;
  constexpr float pio4Part2 = 2.4187564849853515625e-4f;
  constexpr float pio4Part3 = 3.77489497744594108e-8f;

  y = xa - y * VecT::broadcast(pio4Part1) - y * VecT::broadcast(pio4Part2)
      - y * VecT::broadcast(pio4Part3);

  VecT z = y * y;

  MaskT infMask = (xa == VecT::broadcast(MathConstants<float>::infinity));
  z = select(infMask, VecT::broadcast(MathConstants<float>::nan), z);

  VecT s
      = estrin(z, -1.6666654611e-1f, 8.3321608736e-3f, -1.9515295891e-4f) * z;
  VecT c = estrin(
               z,
               4.166664568298827e-2f,
               -1.388731625493765e-3f,
               2.443315711809948e-5f)
           * z;

  s = fmadd(s, y, y);
  c = fmadd(c, z, fmadd(z, VecT::broadcast(-0.5f), VecT::broadcast(1.0f)));

  VecI bit1 = shiftRight<1>(bitAnd(j, VecI::broadcast(2)));
  VecI polymaskI = bitNot(VecI::broadcast(0) - bit1);
  VecT polymaskF = reinterpretAsFloat(polymaskI);
  VecT invPolymaskF = reinterpretAsFloat(bitNot(polymaskI));

  constexpr std::size_t A = (W * sizeof(float) >= 64)   ? 64
                            : (W * sizeof(float) >= 32) ? 32
                                                        : 16;
  alignas(A) float xArr[W];
  alignas(A) float scalarSin[W];
  alignas(A) float scalarCos[W];
  bool needScalar = false;
  {
    alignas(A) float inRangeArr[W];
    select(inRange, VecT::broadcast(1.0f), VecT::broadcast(0.0f))
        .store(inRangeArr);
    for (std::size_t i = 0; i < W; ++i) {
      if (inRangeArr[i] == 0.0f) {
        needScalar = true;
        break;
      }
    }
  }
  if (needScalar) {
    x.store(xArr);
    for (std::size_t i = 0; i < W; ++i) {
      if constexpr (ComputeSin) {
        scalarSin[i] = std::sin(xArr[i]);
      }
      if constexpr (ComputeCos) {
        scalarCos[i] = std::cos(xArr[i]);
      }
    }
  }

  if constexpr (ComputeSin) {
    // Extract bit 2 of j and move to sign position (bit 31).
    // j is always even, so bit 2 determines the sin sign.
    // Avoid shifting j left by 29 (causes signed overflow UB for large j).
    VecI jBit2 = bitAnd(shiftRight<2>(j), VecI::broadcast(1));
    VecI signSinI = bitXor(shiftLeft<31>(jBit2), xSign);
    VecT signSin = reinterpretAsFloat(signSinI);
    VecT sinResult = bitOr(bitAnd(s, polymaskF), bitAnd(c, invPolymaskF));
    sinResult = bitXor(sinResult, signSin);
    if (needScalar) {
      sinResult = select(inRange, sinResult, VecT::load(scalarSin));
    }
    *sinOut = sinResult;
  }

  if constexpr (ComputeCos) {
    // Extract bit 2 of ~(j-2) and move to sign position (bit 31).
    // Equivalent to checking if ((j-2) & 4) == 0.
    // Avoid shifting large values left by 29 (causes signed overflow UB).
    VecI jMinus2 = j - VecI::broadcast(2);
    VecI jMinus2Bit2 = bitAnd(shiftRight<2>(jMinus2), VecI::broadcast(1));
    VecI signCosI = shiftLeft<31>(bitXor(jMinus2Bit2, VecI::broadcast(1)));
    VecT signCos = reinterpretAsFloat(signCosI);
    VecT cosResult = bitOr(bitAnd(c, polymaskF), bitAnd(s, invPolymaskF));
    cosResult = bitXor(cosResult, signCos);
    if (needScalar) {
      cosResult = select(inRange, cosResult, VecT::load(scalarCos));
    }
    *cosOut = cosResult;
  }
}

template <bool ComputeSin, bool ComputeCos, std::size_t W>
DART_SIMD_INLINE void sincosImpl(
    Vec<float, W> x, Vec<float, W>* sinOut, Vec<float, W>* cosOut)
{
  if constexpr (::dart::simd::detail::HasFullIntegerSimd<W>::value) {
    sincosImplSimd<ComputeSin, ComputeCos, W>(x, sinOut, cosOut);
  } else {
    sincosImplScalar<ComputeSin, ComputeCos, W>(x, sinOut, cosOut);
  }
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> sinSimd(const Vec<float, W>& x)
{
  Vec<float, W> result;
  sincosImpl<true, false, W>(x, &result, nullptr);
  return result;
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> cosSimd(const Vec<float, W>& x)
{
  Vec<float, W> result;
  sincosImpl<false, true, W>(x, nullptr, &result);
  return result;
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<float, W>, Vec<float, W>>
sincosSimd(const Vec<float, W>& x)
{
  Vec<float, W> s, c;
  sincosImpl<true, true, W>(x, &s, &c);
  return {s, c};
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> sinSimd(const Vec<double, W>& v)
{
  using VecT = Vec<double, W>;
  constexpr std::size_t A = (W * sizeof(double) >= 64)   ? 64
                            : (W * sizeof(double) >= 32) ? 32
                                                         : 16;
  alignas(A) double tmp[W];
  alignas(A) double resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::sin(tmp[i]);
  }
  return VecT::load(resultTmp);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> cosSimd(const Vec<double, W>& v)
{
  using VecT = Vec<double, W>;
  constexpr std::size_t A = (W * sizeof(double) >= 64)   ? 64
                            : (W * sizeof(double) >= 32) ? 32
                                                         : 16;
  alignas(A) double tmp[W];
  alignas(A) double resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::cos(tmp[i]);
  }
  return VecT::load(resultTmp);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<double, W>, Vec<double, W>>
sincosSimd(const Vec<double, W>& v)
{
  using VecT = Vec<double, W>;
  constexpr std::size_t A = (W * sizeof(double) >= 64)   ? 64
                            : (W * sizeof(double) >= 32) ? 32
                                                         : 16;
  alignas(A) double tmp[W];
  alignas(A) double sinTmp[W];
  alignas(A) double cosTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    sinTmp[i] = std::sin(tmp[i]);
    cosTmp[i] = std::cos(tmp[i]);
  }
  return {VecT::load(sinTmp), VecT::load(cosTmp)};
}

} // namespace dart::simd::detail::math
