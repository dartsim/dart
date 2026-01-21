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
#include <dart/simd/detail/avx/vec.hpp>
#include <dart/simd/detail/avx/vec_mask.hpp>

#if defined(DART_SIMD_AVX) && !defined(DART_SIMD_AVX2)

namespace dart::simd {

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> abs(const Vec<float, 8>& v)
{
  return Vec<float, 8>(
      _mm256_and_ps(v.data, _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF))));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> abs(const Vec<double, 4>& v)
{
  return Vec<double, 4>(_mm256_and_pd(
      v.data, _mm256_castsi256_pd(_mm256_set1_epi64x(0x7FFFFFFFFFFFFFFF))));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> sqrt(const Vec<float, 8>& v)
{
  return Vec<float, 8>(_mm256_sqrt_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> sqrt(const Vec<double, 4>& v)
{
  return Vec<double, 4>(_mm256_sqrt_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> rsqrt(const Vec<float, 8>& v)
{
  __m256 est = _mm256_rsqrt_ps(v.data);
  __m256 half = _mm256_set1_ps(0.5f);
  __m256 three = _mm256_set1_ps(3.0f);
  __m256 muls = _mm256_mul_ps(_mm256_mul_ps(v.data, est), est);
  return Vec<float, 8>(
      _mm256_mul_ps(_mm256_mul_ps(half, est), _mm256_sub_ps(three, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> rsqrt(const Vec<double, 4>& v)
{
  return Vec<double, 4>(
      _mm256_div_pd(_mm256_set1_pd(1.0), _mm256_sqrt_pd(v.data)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> rcp(const Vec<float, 8>& v)
{
  __m256 est = _mm256_rcp_ps(v.data);
  __m256 two = _mm256_set1_ps(2.0f);
  __m256 muls = _mm256_mul_ps(v.data, est);
  return Vec<float, 8>(_mm256_mul_ps(est, _mm256_sub_ps(two, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> rcp(const Vec<double, 4>& v)
{
  return Vec<double, 4>(_mm256_div_pd(_mm256_set1_pd(1.0), v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> min(
    const Vec<float, 8>& a, const Vec<float, 8>& b)
{
  return Vec<float, 8>(_mm256_min_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> min(
    const Vec<double, 4>& a, const Vec<double, 4>& b)
{
  return Vec<double, 4>(_mm256_min_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> max(
    const Vec<float, 8>& a, const Vec<float, 8>& b)
{
  return Vec<float, 8>(_mm256_max_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> max(
    const Vec<double, 4>& a, const Vec<double, 4>& b)
{
  return Vec<double, 4>(_mm256_max_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> clamp(
    const Vec<float, 8>& v, const Vec<float, 8>& lo, const Vec<float, 8>& hi)
{
  return min(max(v, lo), hi);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> clamp(
    const Vec<double, 4>& v,
    const Vec<double, 4>& lo,
    const Vec<double, 4>& hi)
{
  return min(max(v, lo), hi);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> fmadd(
    const Vec<float, 8>& a, const Vec<float, 8>& b, const Vec<float, 8>& c)
{
  return Vec<float, 8>(_mm256_add_ps(_mm256_mul_ps(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> fmadd(
    const Vec<double, 4>& a, const Vec<double, 4>& b, const Vec<double, 4>& c)
{
  return Vec<double, 4>(_mm256_add_pd(_mm256_mul_pd(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> fmsub(
    const Vec<float, 8>& a, const Vec<float, 8>& b, const Vec<float, 8>& c)
{
  return Vec<float, 8>(_mm256_sub_ps(_mm256_mul_ps(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> fmsub(
    const Vec<double, 4>& a, const Vec<double, 4>& b, const Vec<double, 4>& c)
{
  return Vec<double, 4>(_mm256_sub_pd(_mm256_mul_pd(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> fnmadd(
    const Vec<float, 8>& a, const Vec<float, 8>& b, const Vec<float, 8>& c)
{
  return Vec<float, 8>(_mm256_sub_ps(c.data, _mm256_mul_ps(a.data, b.data)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> fnmadd(
    const Vec<double, 4>& a, const Vec<double, 4>& b, const Vec<double, 4>& c)
{
  return Vec<double, 4>(_mm256_sub_pd(c.data, _mm256_mul_pd(a.data, b.data)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> floor(const Vec<float, 8>& v)
{
  return Vec<float, 8>(_mm256_floor_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> floor(const Vec<double, 4>& v)
{
  return Vec<double, 4>(_mm256_floor_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> ceil(const Vec<float, 8>& v)
{
  return Vec<float, 8>(_mm256_ceil_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> ceil(const Vec<double, 4>& v)
{
  return Vec<double, 4>(_mm256_ceil_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> round(const Vec<float, 8>& v)
{
  return Vec<float, 8>(
      _mm256_round_ps(v.data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> round(const Vec<double, 4>& v)
{
  return Vec<double, 4>(
      _mm256_round_pd(v.data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> trunc(const Vec<float, 8>& v)
{
  return Vec<float, 8>(
      _mm256_round_ps(v.data, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> trunc(const Vec<double, 4>& v)
{
  return Vec<double, 4>(
      _mm256_round_pd(v.data, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hsum(const Vec<float, 8>& v)
{
  __m128 lo = _mm256_castps256_ps128(v.data);
  __m128 hi = _mm256_extractf128_ps(v.data, 1);
  __m128 sum = _mm_add_ps(lo, hi);
  __m128 shuf = _mm_movehdup_ps(sum);
  __m128 sums = _mm_add_ps(sum, shuf);
  shuf = _mm_movehl_ps(shuf, sums);
  sums = _mm_add_ss(sums, shuf);
  return _mm_cvtss_f32(sums);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hsum(const Vec<double, 4>& v)
{
  __m128d lo = _mm256_castpd256_pd128(v.data);
  __m128d hi = _mm256_extractf128_pd(v.data, 1);
  __m128d sum = _mm_add_pd(lo, hi);
  __m128d shuf = _mm_shuffle_pd(sum, sum, 1);
  __m128d sums = _mm_add_sd(sum, shuf);
  return _mm_cvtsd_f64(sums);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmin(const Vec<float, 8>& v)
{
  __m128 lo = _mm256_castps256_ps128(v.data);
  __m128 hi = _mm256_extractf128_ps(v.data, 1);
  __m128 mins = _mm_min_ps(lo, hi);
  __m128 shuf = _mm_movehdup_ps(mins);
  mins = _mm_min_ps(mins, shuf);
  shuf = _mm_movehl_ps(shuf, mins);
  mins = _mm_min_ss(mins, shuf);
  return _mm_cvtss_f32(mins);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmin(const Vec<double, 4>& v)
{
  __m128d lo = _mm256_castpd256_pd128(v.data);
  __m128d hi = _mm256_extractf128_pd(v.data, 1);
  __m128d mins = _mm_min_pd(lo, hi);
  __m128d shuf = _mm_shuffle_pd(mins, mins, 1);
  mins = _mm_min_sd(mins, shuf);
  return _mm_cvtsd_f64(mins);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmax(const Vec<float, 8>& v)
{
  __m128 lo = _mm256_castps256_ps128(v.data);
  __m128 hi = _mm256_extractf128_ps(v.data, 1);
  __m128 maxs = _mm_max_ps(lo, hi);
  __m128 shuf = _mm_movehdup_ps(maxs);
  maxs = _mm_max_ps(maxs, shuf);
  shuf = _mm_movehl_ps(shuf, maxs);
  maxs = _mm_max_ss(maxs, shuf);
  return _mm_cvtss_f32(maxs);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmax(const Vec<double, 4>& v)
{
  __m128d lo = _mm256_castpd256_pd128(v.data);
  __m128d hi = _mm256_extractf128_pd(v.data, 1);
  __m128d maxs = _mm_max_pd(lo, hi);
  __m128d shuf = _mm_shuffle_pd(maxs, maxs, 1);
  maxs = _mm_max_sd(maxs, shuf);
  return _mm_cvtsd_f64(maxs);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hprod(const Vec<float, 8>& v)
{
  __m128 lo = _mm256_castps256_ps128(v.data);
  __m128 hi = _mm256_extractf128_ps(v.data, 1);
  __m128 prods = _mm_mul_ps(lo, hi);
  __m128 shuf = _mm_movehdup_ps(prods);
  prods = _mm_mul_ps(prods, shuf);
  shuf = _mm_movehl_ps(shuf, prods);
  prods = _mm_mul_ss(prods, shuf);
  return _mm_cvtss_f32(prods);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hprod(const Vec<double, 4>& v)
{
  __m128d lo = _mm256_castpd256_pd128(v.data);
  __m128d hi = _mm256_extractf128_pd(v.data, 1);
  __m128d prods = _mm_mul_pd(lo, hi);
  __m128d shuf = _mm_shuffle_pd(prods, prods, 1);
  prods = _mm_mul_sd(prods, shuf);
  return _mm_cvtsd_f64(prods);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 8> select(
    const VecMask<float, 8>& mask,
    const Vec<float, 8>& if_true,
    const Vec<float, 8>& if_false)
{
  return Vec<float, 8>(
      _mm256_blendv_ps(if_false.data, if_true.data, mask.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 4> select(
    const VecMask<double, 4>& mask,
    const Vec<double, 4>& if_true,
    const Vec<double, 4>& if_false)
{
  return Vec<double, 4>(
      _mm256_blendv_pd(if_false.data, if_true.data, mask.data));
}

} // namespace dart::simd

#endif // DART_SIMD_AVX && !DART_SIMD_AVX2
