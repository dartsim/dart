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
#include <dart/simd/detail/neon/vec.hpp>
#include <dart/simd/detail/neon/vec_mask.hpp>

#if defined(DART_SIMD_NEON)

namespace dart::simd {

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> abs(const Vec<float, 4>& v)
{
  return Vec<float, 4>(vabsq_f32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> abs(const Vec<double, 2>& v)
{
  return Vec<double, 2>(vabsq_f64(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> sqrt(const Vec<float, 4>& v)
{
  return Vec<float, 4>(vsqrtq_f32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> sqrt(const Vec<double, 2>& v)
{
  return Vec<double, 2>(vsqrtq_f64(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> rsqrt(const Vec<float, 4>& v)
{
  float32x4_t est = vrsqrteq_f32(v.data);
  est = vmulq_f32(est, vrsqrtsq_f32(vmulq_f32(v.data, est), est));
  est = vmulq_f32(est, vrsqrtsq_f32(vmulq_f32(v.data, est), est));
  return Vec<float, 4>(est);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> rsqrt(const Vec<double, 2>& v)
{
  float64x2_t est = vrsqrteq_f64(v.data);
  est = vmulq_f64(est, vrsqrtsq_f64(vmulq_f64(v.data, est), est));
  est = vmulq_f64(est, vrsqrtsq_f64(vmulq_f64(v.data, est), est));
  est = vmulq_f64(est, vrsqrtsq_f64(vmulq_f64(v.data, est), est));
  return Vec<double, 2>(est);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> rcp(const Vec<float, 4>& v)
{
  float32x4_t est = vrecpeq_f32(v.data);
  est = vmulq_f32(est, vrecpsq_f32(v.data, est));
  est = vmulq_f32(est, vrecpsq_f32(v.data, est));
  return Vec<float, 4>(est);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> rcp(const Vec<double, 2>& v)
{
  float64x2_t est = vrecpeq_f64(v.data);
  est = vmulq_f64(est, vrecpsq_f64(v.data, est));
  est = vmulq_f64(est, vrecpsq_f64(v.data, est));
  est = vmulq_f64(est, vrecpsq_f64(v.data, est));
  return Vec<double, 2>(est);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> min(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(vminq_f32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> min(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(vminq_f64(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> min(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(vminq_s32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> max(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  return Vec<float, 4>(vmaxq_f32(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> max(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  return Vec<double, 2>(vmaxq_f64(a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> max(
    const Vec<std::int32_t, 4>& a, const Vec<std::int32_t, 4>& b)
{
  return Vec<std::int32_t, 4>(vmaxq_s32(a.data, b.data));
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

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(vfmaq_f32(c.data, a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(vfmaq_f64(c.data, a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmsub(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(vfmaq_f32(vnegq_f32(c.data), a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmsub(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(vfmaq_f64(vnegq_f64(c.data), a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fnmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  return Vec<float, 4>(vfmsq_f32(c.data, a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fnmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  return Vec<double, 2>(vfmsq_f64(c.data, a.data, b.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> floor(const Vec<float, 4>& v)
{
  return Vec<float, 4>(vrndmq_f32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> floor(const Vec<double, 2>& v)
{
  return Vec<double, 2>(vrndmq_f64(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> ceil(const Vec<float, 4>& v)
{
  return Vec<float, 4>(vrndpq_f32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> ceil(const Vec<double, 2>& v)
{
  return Vec<double, 2>(vrndpq_f64(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> round(const Vec<float, 4>& v)
{
  return Vec<float, 4>(vrndnq_f32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> round(const Vec<double, 2>& v)
{
  return Vec<double, 2>(vrndnq_f64(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> trunc(const Vec<float, 4>& v)
{
  return Vec<float, 4>(vrndq_f32(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> trunc(const Vec<double, 2>& v)
{
  return Vec<double, 2>(vrndq_f64(v.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hsum(const Vec<float, 4>& v)
{
  return vaddvq_f32(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hsum(const Vec<double, 2>& v)
{
  return vaddvq_f64(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmin(const Vec<float, 4>& v)
{
  return vminvq_f32(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmin(const Vec<double, 2>& v)
{
  return vminvq_f64(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hmax(const Vec<float, 4>& v)
{
  return vmaxvq_f32(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hmax(const Vec<double, 2>& v)
{
  return vmaxvq_f64(v.data);
}

template <>
[[nodiscard]] DART_SIMD_INLINE float hprod(const Vec<float, 4>& v)
{
  float32x2_t lo = vget_low_f32(v.data);
  float32x2_t hi = vget_high_f32(v.data);
  float32x2_t prod = vmul_f32(lo, hi);
  return vget_lane_f32(prod, 0) * vget_lane_f32(prod, 1);
}

template <>
[[nodiscard]] DART_SIMD_INLINE double hprod(const Vec<double, 2>& v)
{
  return vgetq_lane_f64(v.data, 0) * vgetq_lane_f64(v.data, 1);
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> select(
    const VecMask<float, 4>& mask,
    const Vec<float, 4>& if_true,
    const Vec<float, 4>& if_false)
{
  return Vec<float, 4>(vbslq_f32(mask.data, if_true.data, if_false.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> select(
    const VecMask<double, 2>& mask,
    const Vec<double, 2>& if_true,
    const Vec<double, 2>& if_false)
{
  return Vec<double, 2>(vbslq_f64(mask.data, if_true.data, if_false.data));
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<std::int32_t, 4> select(
    const VecMask<std::int32_t, 4>& mask,
    const Vec<std::int32_t, 4>& if_true,
    const Vec<std::int32_t, 4>& if_false)
{
  return Vec<std::int32_t, 4>(
      vbslq_s32(mask.data, if_true.data, if_false.data));
}

} // namespace dart::simd

#endif // DART_SIMD_NEON
