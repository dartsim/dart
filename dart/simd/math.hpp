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
#include <dart/simd/detail/math/exp_impl.hpp>
#include <dart/simd/detail/math/log_impl.hpp>
#include <dart/simd/detail/math/polynomial.hpp>
#include <dart/simd/detail/math/sincos_impl.hpp>
#include <dart/simd/fwd.hpp>

#include <limits>
#include <type_traits>
#include <utility>

#include <cmath>

namespace dart::simd {

namespace detail::math {

template <typename T, std::size_t W>
constexpr std::size_t alignment_v = (W * sizeof(T) >= 64)   ? 64
                                    : (W * sizeof(T) >= 32) ? 32
                                                            : 16;

}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> copysign(
    const Vec<float, W>& magnitude, const Vec<float, W>& signSource)
{
  using VecT = Vec<float, W>;
  using VecI = Vec<std::int32_t, W>;
  constexpr std::int32_t signMask = static_cast<std::int32_t>(0x80000000);
  VecT signBits
      = bitAnd(signSource, reinterpretAsFloat(VecI::broadcast(signMask)));
  VecT absMag
      = bitAnd(magnitude, reinterpretAsFloat(VecI::broadcast(~signMask)));
  return bitOr(absMag, signBits);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> copysign(
    const Vec<double, W>& magnitude, const Vec<double, W>& signSource)
{
  using VecT = Vec<double, W>;
  using VecI = Vec<std::int64_t, W>;
  constexpr std::int64_t signMask
      = static_cast<std::int64_t>(0x8000000000000000LL);
  VecT signBits
      = bitAnd(signSource, reinterpretAsDouble(VecI::broadcast(signMask)));
  VecT absMag
      = bitAnd(magnitude, reinterpretAsDouble(VecI::broadcast(~signMask)));
  return bitOr(absMag, signBits);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> mulsign(
    const Vec<float, W>& value, const Vec<float, W>& signSource)
{
  using VecT = Vec<float, W>;
  using VecI = Vec<std::int32_t, W>;
  constexpr std::int32_t signMask = static_cast<std::int32_t>(0x80000000);
  VecT signBits
      = bitAnd(signSource, reinterpretAsFloat(VecI::broadcast(signMask)));
  return bitXor(value, signBits);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> mulsign(
    const Vec<double, W>& value, const Vec<double, W>& signSource)
{
  using VecT = Vec<double, W>;
  using VecI = Vec<std::int64_t, W>;
  constexpr std::int64_t signMask
      = static_cast<std::int64_t>(0x8000000000000000LL);
  VecT signBits
      = bitAnd(signSource, reinterpretAsDouble(VecI::broadcast(signMask)));
  return bitXor(value, signBits);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> sign(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = (tmp[i] > T{0}) ? T{1} : ((tmp[i] < T{0}) ? T{-1} : T{0});
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE VecMask<T, W> isnan(const Vec<T, W>& v)
{
  // Store to scalar, classify with std::isnan, reload as Vec, then compare
  // to produce a VecMask. Avoids _CMP_NEQ_OQ vs _CMP_NEQ_UQ semantics.
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T flags[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    flags[i] = std::isnan(tmp[i]) ? T{1} : T{0};
  }
  return Vec<T, W>::load(flags) == Vec<T, W>::broadcast(T{1});
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE VecMask<T, W> isinf(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T flags[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    flags[i] = std::isinf(tmp[i]) ? T{1} : T{0};
  }
  return Vec<T, W>::load(flags) == Vec<T, W>::broadcast(T{1});
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE VecMask<T, W> isfinite(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T flags[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    flags[i] = std::isfinite(tmp[i]) ? T{1} : T{0};
  }
  return Vec<T, W>::load(flags) == Vec<T, W>::broadcast(T{1});
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> exp(const Vec<float, W>& v)
{
  return detail::math::expSimd<W>(v);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> exp(const Vec<double, W>& v)
{
  return detail::math::expSimd<W>(v);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> exp2(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::exp2(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<float, W> log(const Vec<float, W>& v)
{
  return detail::math::logSimd<W>(v);
}

template <std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<double, W> log(const Vec<double, W>& v)
{
  return detail::math::logSimd<W>(v);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> log2(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::log2(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> log10(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::log10(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> pow(
    const Vec<T, W>& base, const Vec<T, W>& exponent)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T baseTmp[W];
  alignas(A) T expTmp[W];
  alignas(A) T resultTmp[W];
  base.store(baseTmp);
  exponent.store(expTmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::pow(baseTmp[i], expTmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> cbrt(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::cbrt(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> sin(const Vec<T, W>& v)
{
  return detail::math::sinSimd<W>(v);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> cos(const Vec<T, W>& v)
{
  return detail::math::cosSimd<W>(v);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<T, W>, Vec<T, W>> sincos(
    const Vec<T, W>& v)
{
  return detail::math::sincosSimd<W>(v);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> tan(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::tan(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> asin(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::asin(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> acos(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::acos(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> atan(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::atan(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> atan2(
    const Vec<T, W>& y, const Vec<T, W>& x)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T y_tmp[W];
  alignas(A) T x_tmp[W];
  alignas(A) T resultTmp[W];
  y.store(y_tmp);
  x.store(x_tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::atan2(y_tmp[i], x_tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> sinh(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::sinh(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> cosh(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::cosh(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> tanh(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::tanh(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<T, W>, Vec<T, W>> sincosh(
    const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T sinhTmp[W];
  alignas(A) T coshTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    sinhTmp[i] = std::sinh(tmp[i]);
    coshTmp[i] = std::cosh(tmp[i]);
  }
  return {Vec<T, W>::load(sinhTmp), Vec<T, W>::load(coshTmp)};
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> asinh(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::asinh(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> acosh(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::acosh(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> atanh(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::atanh(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> erf(const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T resultTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::erf(tmp[i]);
  }
  return Vec<T, W>::load(resultTmp);
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<T, W>, Vec<T, W>> frexp(
    const Vec<T, W>& v)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T tmp[W];
  alignas(A) T mantissaTmp[W];
  alignas(A) T exponentTmp[W];
  v.store(tmp);
  for (std::size_t i = 0; i < W; ++i) {
    int expInt;
    mantissaTmp[i] = std::frexp(tmp[i], &expInt);
    exponentTmp[i] = static_cast<T>(expInt);
  }
  return {Vec<T, W>::load(mantissaTmp), Vec<T, W>::load(exponentTmp)};
}

template <typename T, std::size_t W>
[[nodiscard]] DART_SIMD_INLINE Vec<T, W> ldexp(
    const Vec<T, W>& mantissa, const Vec<T, W>& exponent)
{
  constexpr auto A = detail::math::alignment_v<T, W>;
  alignas(A) T manTmp[W];
  alignas(A) T expTmp[W];
  alignas(A) T resultTmp[W];
  mantissa.store(manTmp);
  exponent.store(expTmp);
  for (std::size_t i = 0; i < W; ++i) {
    resultTmp[i] = std::ldexp(manTmp[i], static_cast<int>(expTmp[i]));
  }
  return Vec<T, W>::load(resultTmp);
}

} // namespace dart::simd
