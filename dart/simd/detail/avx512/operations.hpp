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
#include <dart/simd/detail/avx512/vec.hpp>
#include <dart/simd/detail/avx512/vec_mask.hpp>

#if defined(DART_SIMD_AVX512)

namespace dart::simd {

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> abs(const Vec<float, 16>& v)
{
  return Vec<float, 16>(_mm512_abs_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> abs(const Vec<double, 8>& v)
{
  return Vec<double, 8>(_mm512_abs_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> sqrt(const Vec<float, 16>& v)
{
  return Vec<float, 16>(_mm512_sqrt_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> sqrt(const Vec<double, 8>& v)
{
  return Vec<double, 8>(_mm512_sqrt_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> rsqrt(const Vec<float, 16>& v)
{
  __m512 est = _mm512_rsqrt14_ps(v.data);
  __m512 half = _mm512_set1_ps(0.5f);
  __m512 three = _mm512_set1_ps(3.0f);
  __m512 muls = _mm512_mul_ps(_mm512_mul_ps(v.data, est), est);
  return Vec<float, 16>(
      _mm512_mul_ps(_mm512_mul_ps(half, est), _mm512_sub_ps(three, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> rsqrt(const Vec<double, 8>& v)
{
  __m512d est = _mm512_rsqrt14_pd(v.data);
  __m512d half = _mm512_set1_pd(0.5);
  __m512d three = _mm512_set1_pd(3.0);
  __m512d muls = _mm512_mul_pd(_mm512_mul_pd(v.data, est), est);
  return Vec<double, 8>(
      _mm512_mul_pd(_mm512_mul_pd(half, est), _mm512_sub_pd(three, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> rcp(const Vec<float, 16>& v)
{
  __m512 est = _mm512_rcp14_ps(v.data);
  __m512 two = _mm512_set1_ps(2.0f);
  __m512 muls = _mm512_mul_ps(v.data, est);
  return Vec<float, 16>(_mm512_mul_ps(est, _mm512_sub_ps(two, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> rcp(const Vec<double, 8>& v)
{
  __m512d est = _mm512_rcp14_pd(v.data);
  __m512d two = _mm512_set1_pd(2.0);
  __m512d muls = _mm512_mul_pd(v.data, est);
  return Vec<double, 8>(_mm512_mul_pd(est, _mm512_sub_pd(two, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> min(
    const Vec<float, 16>& a, const Vec<float, 16>& b)
{
  return Vec<float, 16>(_mm512_min_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> min(
    const Vec<double, 8>& a, const Vec<double, 8>& b)
{
  return Vec<double, 8>(_mm512_min_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 16> min(
    const Vec<std::int32_t, 16>& a, const Vec<std::int32_t, 16>& b)
{
  return Vec<std::int32_t, 16>(_mm512_min_epi32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> max(
    const Vec<float, 16>& a, const Vec<float, 16>& b)
{
  return Vec<float, 16>(_mm512_max_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> max(
    const Vec<double, 8>& a, const Vec<double, 8>& b)
{
  return Vec<double, 8>(_mm512_max_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 16> max(
    const Vec<std::int32_t, 16>& a, const Vec<std::int32_t, 16>& b)
{
  return Vec<std::int32_t, 16>(_mm512_max_epi32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> clamp(
    const Vec<float, 16>& v, const Vec<float, 16>& lo, const Vec<float, 16>& hi)
{
  return min(max(v, lo), hi);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> clamp(
    const Vec<double, 8>& v, const Vec<double, 8>& lo, const Vec<double, 8>& hi)
{
  return min(max(v, lo), hi);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> fmadd(
    const Vec<float, 16>& a, const Vec<float, 16>& b, const Vec<float, 16>& c)
{
  return Vec<float, 16>(_mm512_fmadd_ps(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> fmadd(
    const Vec<double, 8>& a, const Vec<double, 8>& b, const Vec<double, 8>& c)
{
  return Vec<double, 8>(_mm512_fmadd_pd(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> fmsub(
    const Vec<float, 16>& a, const Vec<float, 16>& b, const Vec<float, 16>& c)
{
  return Vec<float, 16>(_mm512_fmsub_ps(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> fmsub(
    const Vec<double, 8>& a, const Vec<double, 8>& b, const Vec<double, 8>& c)
{
  return Vec<double, 8>(_mm512_fmsub_pd(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> fnmadd(
    const Vec<float, 16>& a, const Vec<float, 16>& b, const Vec<float, 16>& c)
{
  return Vec<float, 16>(_mm512_fnmadd_ps(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> fnmadd(
    const Vec<double, 8>& a, const Vec<double, 8>& b, const Vec<double, 8>& c)
{
  return Vec<double, 8>(_mm512_fnmadd_pd(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> floor(const Vec<float, 16>& v)
{
  return Vec<float, 16>(_mm512_floor_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> floor(const Vec<double, 8>& v)
{
  return Vec<double, 8>(_mm512_floor_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> ceil(const Vec<float, 16>& v)
{
  return Vec<float, 16>(_mm512_ceil_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> ceil(const Vec<double, 8>& v)
{
  return Vec<double, 8>(_mm512_ceil_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> round(const Vec<float, 16>& v)
{
  return Vec<float, 16>(_mm512_roundscale_ps(
      v.data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> round(const Vec<double, 8>& v)
{
  return Vec<double, 8>(_mm512_roundscale_pd(
      v.data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> trunc(const Vec<float, 16>& v)
{
  return Vec<float, 16>(
      _mm512_roundscale_ps(v.data, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> trunc(const Vec<double, 8>& v)
{
  return Vec<double, 8>(
      _mm512_roundscale_pd(v.data, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hsum(const Vec<float, 16>& v)
{
  return _mm512_reduce_add_ps(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hsum(const Vec<double, 8>& v)
{
  return _mm512_reduce_add_pd(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmin(const Vec<float, 16>& v)
{
  return _mm512_reduce_min_ps(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmin(const Vec<double, 8>& v)
{
  return _mm512_reduce_min_pd(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmax(const Vec<float, 16>& v)
{
  return _mm512_reduce_max_ps(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmax(const Vec<double, 8>& v)
{
  return _mm512_reduce_max_pd(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hprod(const Vec<float, 16>& v)
{
  return _mm512_reduce_mul_ps(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hprod(const Vec<double, 8>& v)
{
  return _mm512_reduce_mul_pd(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 16> select(
    const VecMask<float, 16>& mask,
    const Vec<float, 16>& if_true,
    const Vec<float, 16>& if_false)
{
  return Vec<float, 16>(
      _mm512_mask_blend_ps(mask.data, if_false.data, if_true.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 8> select(
    const VecMask<double, 8>& mask,
    const Vec<double, 8>& if_true,
    const Vec<double, 8>& if_false)
{
  return Vec<double, 8>(
      _mm512_mask_blend_pd(mask.data, if_false.data, if_true.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 16> select(
    const VecMask<std::int32_t, 16>& mask,
    const Vec<std::int32_t, 16>& if_true,
    const Vec<std::int32_t, 16>& if_false)
{
  return Vec<std::int32_t, 16>(
      _mm512_mask_blend_epi32(mask.data, if_false.data, if_true.data));
}

} // namespace dart::simd

#endif // DART_SIMD_AVX512
