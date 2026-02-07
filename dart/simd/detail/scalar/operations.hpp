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
#include <dart/simd/fwd.hpp>

#include <algorithm>

#include <cmath>
#include <cstddef>
#include <cstring>

namespace dart::simd {

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> abs(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::abs(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> sqrt(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::sqrt(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> rsqrt(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = T(1) / std::sqrt(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> rcp(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = T(1) / v[i];
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> min(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::min(a[i], b[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> max(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::max(a[i], b[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> clamp(
    const Vec<T, W>& v, const Vec<T, W>& lo, const Vec<T, W>& hi)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::clamp(v[i], lo[i], hi[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> fmadd(
    const Vec<T, W>& a, const Vec<T, W>& b, const Vec<T, W>& c)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::fma(a[i], b[i], c[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> fmsub(
    const Vec<T, W>& a, const Vec<T, W>& b, const Vec<T, W>& c)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::fma(a[i], b[i], -c[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> fnmadd(
    const Vec<T, W>& a, const Vec<T, W>& b, const Vec<T, W>& c)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::fma(-a[i], b[i], c[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> floor(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::floor(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> ceil(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::ceil(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> round(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::round(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> trunc(const Vec<T, W>& v)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = std::trunc(v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE T hsum(const Vec<T, W>& v)
{
  T result = v[0];
  for (std::size_t i = 1; i < W; ++i) {
    result += v[i];
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE T hmin(const Vec<T, W>& v)
{
  T result = v[0];
  for (std::size_t i = 1; i < W; ++i) {
    result = std::min(result, v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE T hmax(const Vec<T, W>& v)
{
  T result = v[0];
  for (std::size_t i = 1; i < W; ++i) {
    result = std::max(result, v[i]);
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE T hprod(const Vec<T, W>& v)
{
  T result = v[0];
  for (std::size_t i = 1; i < W; ++i) {
    result *= v[i];
  }
  return result;
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> select(
    const VecMask<T, W>& mask,
    const Vec<T, W>& if_true,
    const Vec<T, W>& if_false)
{
  Vec<T, W> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[i] = mask[i] ? if_true[i] : if_false[i];
  }
  return result;
}

namespace detail {

template <typename T>
struct IntTypeFor;

template <>
struct IntTypeFor<float>
{
  using type = std::uint32_t;
};

template <>
struct IntTypeFor<double>
{
  using type = std::uint64_t;
};

template <typename T>
using int_type_for_t = typename IntTypeFor<T>::type;

} // namespace detail

template <typename T, std::size_t W>
  requires std::is_floating_point_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitAnd(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  using Int = detail::int_type_for_t<T>;
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    Int ai, bi;
    std::memcpy(&ai, &aArr[i], sizeof(T));
    std::memcpy(&bi, &bArr[i], sizeof(T));
    Int ri = ai & bi;
    std::memcpy(&rArr[i], &ri, sizeof(T));
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_floating_point_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitOr(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  using Int = detail::int_type_for_t<T>;
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    Int ai, bi;
    std::memcpy(&ai, &aArr[i], sizeof(T));
    std::memcpy(&bi, &bArr[i], sizeof(T));
    Int ri = ai | bi;
    std::memcpy(&rArr[i], &ri, sizeof(T));
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_floating_point_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitXor(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  using Int = detail::int_type_for_t<T>;
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    Int ai, bi;
    std::memcpy(&ai, &aArr[i], sizeof(T));
    std::memcpy(&bi, &bArr[i], sizeof(T));
    Int ri = ai ^ bi;
    std::memcpy(&rArr[i], &ri, sizeof(T));
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_floating_point_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitAndnot(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  using Int = detail::int_type_for_t<T>;
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    Int ai, bi;
    std::memcpy(&ai, &aArr[i], sizeof(T));
    std::memcpy(&bi, &bArr[i], sizeof(T));
    Int ri = ai & (~bi);
    std::memcpy(&rArr[i], &ri, sizeof(T));
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_integral_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitAnd(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = aArr[i] & bArr[i];
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_integral_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitOr(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = aArr[i] | bArr[i];
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_integral_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitXor(
    const Vec<T, W>& a, const Vec<T, W>& b)
{
  alignas(64) T aArr[W], bArr[W], rArr[W];
  a.store(aArr);
  b.store(bArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = aArr[i] ^ bArr[i];
  }
  return Vec<T, W>::load(rArr);
}

template <typename T, std::size_t W>
  requires std::is_integral_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> bitNot(const Vec<T, W>& v)
{
  alignas(64) T vArr[W], rArr[W];
  v.store(vArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = ~vArr[i];
  }
  return Vec<T, W>::load(rArr);
}

template <int N, typename T, std::size_t W>
  requires std::is_integral_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> shiftLeft(const Vec<T, W>& v)
{
  alignas(64) T vArr[W], rArr[W];
  v.store(vArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = vArr[i] << N;
  }
  return Vec<T, W>::load(rArr);
}

template <int N, typename T, std::size_t W>
  requires std::is_integral_v<T>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> shiftRight(const Vec<T, W>& v)
{
  alignas(64) T vArr[W], rArr[W];
  v.store(vArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = vArr[i] >> N;
  }
  return Vec<T, W>::load(rArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, W> reinterpretAsInt(
    const Vec<float, W>& v)
{
  alignas(64) float srcArr[W];
  alignas(64) std::int32_t dstArr[W];
  v.store(srcArr);
  std::memcpy(dstArr, srcArr, sizeof(float) * W);
  return Vec<std::int32_t, W>::load(dstArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> reinterpretAsFloat(
    const Vec<std::int32_t, W>& v)
{
  alignas(64) std::int32_t srcArr[W];
  alignas(64) float dstArr[W];
  v.store(srcArr);
  std::memcpy(dstArr, srcArr, sizeof(float) * W);
  return Vec<float, W>::load(dstArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int64_t, W> reinterpretAsInt64(
    const Vec<double, W>& v)
{
  alignas(64) double srcArr[W];
  alignas(64) std::int64_t dstArr[W];
  v.store(srcArr);
  std::memcpy(dstArr, srcArr, sizeof(double) * W);
  return Vec<std::int64_t, W>::load(dstArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> reinterpretAsDouble(
    const Vec<std::int64_t, W>& v)
{
  alignas(64) std::int64_t srcArr[W];
  alignas(64) double dstArr[W];
  v.store(srcArr);
  std::memcpy(dstArr, srcArr, sizeof(double) * W);
  return Vec<double, W>::load(dstArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, W> convertToInt(
    const Vec<float, W>& v)
{
  alignas(64) float vArr[W];
  alignas(64) std::int32_t rArr[W];
  v.store(vArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = static_cast<std::int32_t>(vArr[i]);
  }
  return Vec<std::int32_t, W>::load(rArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> convertToFloat(
    const Vec<std::int32_t, W>& v)
{
  alignas(64) std::int32_t vArr[W];
  alignas(64) float rArr[W];
  v.store(vArr);
  for (std::size_t i = 0; i < W; ++i) {
    rArr[i] = static_cast<float>(vArr[i]);
  }
  return Vec<float, W>::load(rArr);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<float, W>, Vec<std::int32_t, W>>
frexpSimd(const Vec<float, W>& v)
{
  constexpr std::uint32_t expMask = 0x7F800000;
  constexpr std::uint32_t mantissaMask = 0x007FFFFF;
  constexpr std::uint32_t halfExp = 0x3F000000;

  alignas(64) float vArr[W];
  alignas(64) float mantArr[W];
  alignas(64) std::int32_t expArr[W];
  v.store(vArr);

  for (std::size_t i = 0; i < W; ++i) {
    std::uint32_t bits;
    std::memcpy(&bits, &vArr[i], sizeof(float));

    std::int32_t biasedExp = static_cast<std::int32_t>((bits & expMask) >> 23);
    std::int32_t expAdjust = 0;

    if (biasedExp == 0) {
      if ((bits & mantissaMask) == 0) {
        mantArr[i] = vArr[i];
        expArr[i] = 0;
        continue;
      }
      float normalized = vArr[i] * 8388608.0f;
      std::memcpy(&bits, &normalized, sizeof(float));
      biasedExp = static_cast<std::int32_t>((bits & expMask) >> 23);
      expAdjust = -23;
    }

    expArr[i] = biasedExp - 127 + expAdjust;
    std::uint32_t mantissaBits = (bits & mantissaMask) | halfExp;
    std::memcpy(&mantArr[i], &mantissaBits, sizeof(float));
  }
  return {Vec<float, W>::load(mantArr), Vec<std::int32_t, W>::load(expArr)};
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> ldexpSimd(
    const Vec<float, W>& x, const Vec<std::int32_t, W>& exp)
{
  alignas(64) float xArr[W], rArr[W];
  alignas(64) std::int32_t expArr[W];
  x.store(xArr);
  exp.store(expArr);
  for (std::size_t i = 0; i < W; ++i) {
    if (expArr[i] <= -127 || expArr[i] >= 128) {
      rArr[i] = std::ldexp(xArr[i], expArr[i]);
    } else {
      std::uint32_t pow2Bits = static_cast<std::uint32_t>(expArr[i] + 0x7f)
                               << 23;
      float pow2;
      std::memcpy(&pow2, &pow2Bits, sizeof(float));
      rArr[i] = xArr[i] * pow2;
    }
  }
  return Vec<float, W>::load(rArr);
}

} // namespace dart::simd
