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

#include <cstddef>
#include <cstdint>

#if defined(DART_SIMD_NEON)

namespace dart::simd {

template <typename T, std::size_t Width>
struct VecMask;

template <>
struct Vec<float, 4>
{
  using scalar_type = float;
  static constexpr std::size_t width = 4;
  using mask_type = VecMask<float, 4>;

  float32x4_t data;

  Vec() = default;

  DART_SIMD_INLINE Vec(float32x4_t v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(vdupq_n_f32(0.0f));
  }

  DART_SIMD_INLINE static Vec broadcast(float value)
  {
    return Vec(vdupq_n_f32(value));
  }

  DART_SIMD_INLINE static Vec load(const float* ptr)
  {
    return Vec(vld1q_f32(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const float* ptr)
  {
    return Vec(vld1q_f32(ptr));
  }

  DART_SIMD_INLINE void store(float* ptr) const
  {
    vst1q_f32(ptr, data);
  }

  DART_SIMD_INLINE void storeu(float* ptr) const
  {
    vst1q_f32(ptr, data);
  }

  DART_SIMD_INLINE float operator[](std::size_t i) const
  {
    alignas(16) float tmp[4];
    vst1q_f32(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(vaddq_f32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(vsubq_f32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(vmulq_f32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    return Vec(vdivq_f32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(vnegq_f32(data));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = vaddq_f32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = vsubq_f32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = vmulq_f32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    data = vdivq_f32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator<(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator>(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const;
};

template <>
struct Vec<double, 2>
{
  using scalar_type = double;
  static constexpr std::size_t width = 2;
  using mask_type = VecMask<double, 2>;

  float64x2_t data;

  Vec() = default;

  DART_SIMD_INLINE Vec(float64x2_t v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(vdupq_n_f64(0.0));
  }

  DART_SIMD_INLINE static Vec broadcast(double value)
  {
    return Vec(vdupq_n_f64(value));
  }

  DART_SIMD_INLINE static Vec load(const double* ptr)
  {
    return Vec(vld1q_f64(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const double* ptr)
  {
    return Vec(vld1q_f64(ptr));
  }

  DART_SIMD_INLINE void store(double* ptr) const
  {
    vst1q_f64(ptr, data);
  }

  DART_SIMD_INLINE void storeu(double* ptr) const
  {
    vst1q_f64(ptr, data);
  }

  DART_SIMD_INLINE double operator[](std::size_t i) const
  {
    alignas(16) double tmp[2];
    vst1q_f64(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(vaddq_f64(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(vsubq_f64(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(vmulq_f64(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    return Vec(vdivq_f64(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(vnegq_f64(data));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = vaddq_f64(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = vsubq_f64(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = vmulq_f64(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    data = vdivq_f64(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator<(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator>(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const;
};

template <>
struct Vec<std::int32_t, 4>
{
  using scalar_type = std::int32_t;
  static constexpr std::size_t width = 4;
  using mask_type = VecMask<std::int32_t, 4>;

  int32x4_t data;

  Vec() = default;

  DART_SIMD_INLINE Vec(int32x4_t v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(vdupq_n_s32(0));
  }

  DART_SIMD_INLINE static Vec broadcast(std::int32_t value)
  {
    return Vec(vdupq_n_s32(value));
  }

  DART_SIMD_INLINE static Vec load(const std::int32_t* ptr)
  {
    return Vec(vld1q_s32(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const std::int32_t* ptr)
  {
    return Vec(vld1q_s32(ptr));
  }

  DART_SIMD_INLINE void store(std::int32_t* ptr) const
  {
    vst1q_s32(ptr, data);
  }

  DART_SIMD_INLINE void storeu(std::int32_t* ptr) const
  {
    vst1q_s32(ptr, data);
  }

  DART_SIMD_INLINE std::int32_t operator[](std::size_t i) const
  {
    alignas(16) std::int32_t tmp[4];
    vst1q_s32(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(vaddq_s32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(vsubq_s32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(vmulq_s32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    alignas(16) std::int32_t a[4], b[4], r[4];
    vst1q_s32(a, data);
    vst1q_s32(b, other.data);
    r[0] = a[0] / b[0];
    r[1] = a[1] / b[1];
    r[2] = a[2] / b[2];
    r[3] = a[3] / b[3];
    return Vec(vld1q_s32(r));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(vnegq_s32(data));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = vaddq_s32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = vsubq_s32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = vmulq_s32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    *this = *this / other;
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator<(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator>(const Vec& other) const;
  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const;
};

} // namespace dart::simd

#endif // DART_SIMD_NEON
