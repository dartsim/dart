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

} // namespace dart::simd
