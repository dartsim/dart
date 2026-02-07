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
#include <dart/simd/detail/sse42/vec.hpp>
#include <dart/simd/detail/sse42/vec_mask.hpp>

#if defined(DART_SIMD_SSE42)

namespace dart::simd {

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> abs(const Vec<float, 4>& v)
{
  return Vec<float, 4>(
      _mm_and_ps(v.data, _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF))));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> abs(const Vec<double, 2>& v)
{
  return Vec<double, 2>(_mm_and_pd(
      v.data, _mm_castsi128_pd(_mm_set1_epi64x(0x7FFFFFFFFFFFFFFF))));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> sqrt(const Vec<float, 4>& v)
{
  return Vec<float, 4>(_mm_sqrt_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> sqrt(const Vec<double, 2>& v)
{
  return Vec<double, 2>(_mm_sqrt_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> rsqrt(const Vec<float, 4>& v)
{
  __m128 est = _mm_rsqrt_ps(v.data);
  __m128 half = _mm_set1_ps(0.5f);
  __m128 three = _mm_set1_ps(3.0f);
  __m128 muls = _mm_mul_ps(_mm_mul_ps(v.data, est), est);
  return Vec<float, 4>(
      _mm_mul_ps(_mm_mul_ps(half, est), _mm_sub_ps(three, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> rsqrt(const Vec<double, 2>& v)
{
  return Vec<double, 2>(_mm_div_pd(_mm_set1_pd(1.0), _mm_sqrt_pd(v.data)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> rcp(const Vec<float, 4>& v)
{
  __m128 est = _mm_rcp_ps(v.data);
  __m128 two = _mm_set1_ps(2.0f);
  __m128 muls = _mm_mul_ps(v.data, est);
  return Vec<float, 4>(_mm_mul_ps(est, _mm_sub_ps(two, muls)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> rcp(const Vec<double, 2>& v)
{
  return Vec<double, 2>(_mm_div_pd(_mm_set1_pd(1.0), v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> min(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(_mm_min_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> min(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(_mm_min_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> min(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(_mm_min_epi32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> max(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(_mm_max_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> max(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(_mm_max_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> max(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(_mm_max_epi32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> clamp(
    const Vec<float, 4>& v, const Vec<float, 4>& lo, const Vec<float, 4>& hi)
{
  return min(max(v, lo), hi);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> clamp(
    const Vec<double, 2>& v, const Vec<double, 2>& lo, const Vec<double, 2>& hi)
{
  return min(max(v, lo), hi);
}

  #if defined(DART_SIMD_FMA)

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(_mm_fmadd_ps(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(_mm_fmadd_pd(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmsub(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(_mm_fmsub_ps(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmsub(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(_mm_fmsub_pd(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fnmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(_mm_fnmadd_ps(a.data, b.data, c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fnmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(_mm_fnmadd_pd(a.data, b.data, c.data));
}

  #else

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(_mm_add_ps(_mm_mul_ps(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(_mm_add_pd(_mm_mul_pd(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmsub(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(_mm_sub_ps(_mm_mul_ps(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmsub(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(_mm_sub_pd(_mm_mul_pd(a.data, b.data), c.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fnmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(_mm_sub_ps(c.data, _mm_mul_ps(a.data, b.data)));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fnmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(_mm_sub_pd(c.data, _mm_mul_pd(a.data, b.data)));
}

  #endif

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> floor(const Vec<float, 4>& v)
{
  return Vec<float, 4>(_mm_floor_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> floor(const Vec<double, 2>& v)
{
  return Vec<double, 2>(_mm_floor_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> ceil(const Vec<float, 4>& v)
{
  return Vec<float, 4>(_mm_ceil_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> ceil(const Vec<double, 2>& v)
{
  return Vec<double, 2>(_mm_ceil_pd(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> round(const Vec<float, 4>& v)
{
  return Vec<float, 4>(
      _mm_round_ps(v.data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> round(const Vec<double, 2>& v)
{
  return Vec<double, 2>(
      _mm_round_pd(v.data, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> trunc(const Vec<float, 4>& v)
{
  return Vec<float, 4>(
      _mm_round_ps(v.data, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> trunc(const Vec<double, 2>& v)
{
  return Vec<double, 2>(
      _mm_round_pd(v.data, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hsum(const Vec<float, 4>& v)
{
  __m128 shuf = _mm_movehdup_ps(v.data);
  __m128 sums = _mm_add_ps(v.data, shuf);
  shuf = _mm_movehl_ps(shuf, sums);
  sums = _mm_add_ss(sums, shuf);
  return _mm_cvtss_f32(sums);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hsum(const Vec<double, 2>& v)
{
  __m128d shuf = _mm_shuffle_pd(v.data, v.data, 1);
  __m128d sums = _mm_add_sd(v.data, shuf);
  return _mm_cvtsd_f64(sums);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmin(const Vec<float, 4>& v)
{
  __m128 shuf = _mm_movehdup_ps(v.data);
  __m128 mins = _mm_min_ps(v.data, shuf);
  shuf = _mm_movehl_ps(shuf, mins);
  mins = _mm_min_ss(mins, shuf);
  return _mm_cvtss_f32(mins);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmin(const Vec<double, 2>& v)
{
  __m128d shuf = _mm_shuffle_pd(v.data, v.data, 1);
  __m128d mins = _mm_min_sd(v.data, shuf);
  return _mm_cvtsd_f64(mins);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmax(const Vec<float, 4>& v)
{
  __m128 shuf = _mm_movehdup_ps(v.data);
  __m128 maxs = _mm_max_ps(v.data, shuf);
  shuf = _mm_movehl_ps(shuf, maxs);
  maxs = _mm_max_ss(maxs, shuf);
  return _mm_cvtss_f32(maxs);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmax(const Vec<double, 2>& v)
{
  __m128d shuf = _mm_shuffle_pd(v.data, v.data, 1);
  __m128d maxs = _mm_max_sd(v.data, shuf);
  return _mm_cvtsd_f64(maxs);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hprod(const Vec<float, 4>& v)
{
  __m128 shuf = _mm_movehdup_ps(v.data);
  __m128 prods = _mm_mul_ps(v.data, shuf);
  shuf = _mm_movehl_ps(shuf, prods);
  prods = _mm_mul_ss(prods, shuf);
  return _mm_cvtss_f32(prods);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hprod(const Vec<double, 2>& v)
{
  __m128d shuf = _mm_shuffle_pd(v.data, v.data, 1);
  __m128d prods = _mm_mul_sd(v.data, shuf);
  return _mm_cvtsd_f64(prods);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> select(
    const VecMask<float, 4>& mask,
    const Vec<float, 4>& if_true,
    const Vec<float, 4>& if_false)
{
  return Vec<float, 4>(_mm_blendv_ps(if_false.data, if_true.data, mask.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> select(
    const VecMask<double, 2>& mask,
    const Vec<double, 2>& if_true,
    const Vec<double, 2>& if_false)
{
  return Vec<double, 2>(_mm_blendv_pd(if_false.data, if_true.data, mask.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> select(
    const VecMask<std::int32_t, 4>& mask,
    const Vec<std::int32_t, 4>& if_true,
    const Vec<std::int32_t, 4>& if_false)
{
  return Vec<std::int32_t, 4>(
      _mm_blendv_epi8(if_false.data, if_true.data, mask.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> bitAnd(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(_mm_and_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> bitAnd(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(_mm_and_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> bitOr(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(_mm_or_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> bitOr(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(_mm_or_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> bitXor(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(_mm_xor_ps(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> bitXor(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(_mm_xor_pd(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> bitAndnot(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(_mm_andnot_ps(b.data, a.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> bitAndnot(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(_mm_andnot_pd(b.data, a.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> bitAnd(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(_mm_and_si128(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> bitOr(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(_mm_or_si128(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> bitXor(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(_mm_xor_si128(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> bitNot(
    const Vec<std::int32_t, 4>& v)
{
  return Vec<std::int32_t, 4>(
      _mm_xor_si128(v.data, _mm_set1_epi32(static_cast<int>(0xFFFFFFFF))));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> reinterpretAsInt(
    const Vec<float, 4>& v)
{
  return Vec<std::int32_t, 4>(_mm_castps_si128(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> reinterpretAsFloat(
    const Vec<std::int32_t, 4>& v)
{
  return Vec<float, 4>(_mm_castsi128_ps(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> convertToInt(
    const Vec<float, 4>& v)
{
  return Vec<std::int32_t, 4>(_mm_cvttps_epi32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> convertToFloat(
    const Vec<std::int32_t, 4>& v)
{
  return Vec<float, 4>(_mm_cvtepi32_ps(v.data));
}

template <int N>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> shiftLeft(
    const Vec<std::int32_t, 4>& v)
{
  return Vec<std::int32_t, 4>(_mm_slli_epi32(v.data, N));
}

template <int N>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> shiftRight(
    const Vec<std::int32_t, 4>& v)
{
  return Vec<std::int32_t, 4>(_mm_srai_epi32(v.data, N));
}

template <>
[[nodiscard]] DART_SIMD_INLINE std::pair<Vec<float, 4>, Vec<std::int32_t, 4>>
frexpSimd(const Vec<float, 4>& v)
{
  const __m128i expMask = _mm_set1_epi32(0x7F800000);
  const __m128i mantissaMask = _mm_set1_epi32(0x007FFFFF);
  const __m128i halfExp = _mm_set1_epi32(0x3F000000);
  const __m128i bias = _mm_set1_epi32(127);

  __m128i bits = _mm_castps_si128(v.data);
  __m128i expBits = _mm_and_si128(bits, expMask);
  __m128i exponent = _mm_sub_epi32(_mm_srli_epi32(expBits, 23), bias);
  __m128i mantissaBits
      = _mm_or_si128(_mm_and_si128(bits, mantissaMask), halfExp);

  return {
      Vec<float, 4>(_mm_castsi128_ps(mantissaBits)),
      Vec<std::int32_t, 4>(exponent)};
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> ldexpSimd(
    const Vec<float, 4>& x, const Vec<std::int32_t, 4>& exp)
{
  // Fast path: x * 2^exp = x * reinterpret(shiftLeft<23>(exp + 127))
  // This is much faster than extracting/recombining exponent bits
  __m128i biasedExp = _mm_add_epi32(exp.data, _mm_set1_epi32(0x7f));
  __m128i pow2 = _mm_slli_epi32(biasedExp, 23);
  return Vec<float, 4>(_mm_mul_ps(x.data, _mm_castsi128_ps(pow2)));
}

} // namespace dart::simd

#endif // DART_SIMD_SSE42
