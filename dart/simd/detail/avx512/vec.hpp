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

#if defined(DART_SIMD_AVX512)

namespace dart::simd {

template <typename T, std::size_t Width>
struct VecMask;

template <>
struct Vec<float, 16>
{
  using scalar_type = float;
  static constexpr std::size_t width = 16;
  using mask_type = VecMask<float, 16>;

  __m512 data;

  Vec() = default;

  DART_SIMD_INLINE Vec(__m512 v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(_mm512_setzero_ps());
  }

  DART_SIMD_INLINE static Vec broadcast(float value)
  {
    return Vec(_mm512_set1_ps(value));
  }

  DART_SIMD_INLINE static Vec load(const float* ptr)
  {
    return Vec(_mm512_load_ps(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const float* ptr)
  {
    return Vec(_mm512_loadu_ps(ptr));
  }

  DART_SIMD_INLINE void store(float* ptr) const
  {
    _mm512_store_ps(ptr, data);
  }

  DART_SIMD_INLINE void storeu(float* ptr) const
  {
    _mm512_storeu_ps(ptr, data);
  }

  DART_SIMD_INLINE float operator[](std::size_t i) const
  {
    alignas(64) float tmp[16];
    _mm512_store_ps(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(_mm512_add_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(_mm512_sub_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(_mm512_mul_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    return Vec(_mm512_div_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(_mm512_xor_ps(data, _mm512_set1_ps(-0.0f)));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = _mm512_add_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = _mm512_sub_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = _mm512_mul_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    data = _mm512_div_ps(data, other.data);
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
struct Vec<double, 8>
{
  using scalar_type = double;
  static constexpr std::size_t width = 8;
  using mask_type = VecMask<double, 8>;

  __m512d data;

  Vec() = default;

  DART_SIMD_INLINE Vec(__m512d v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(_mm512_setzero_pd());
  }

  DART_SIMD_INLINE static Vec broadcast(double value)
  {
    return Vec(_mm512_set1_pd(value));
  }

  DART_SIMD_INLINE static Vec load(const double* ptr)
  {
    return Vec(_mm512_load_pd(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const double* ptr)
  {
    return Vec(_mm512_loadu_pd(ptr));
  }

  DART_SIMD_INLINE void store(double* ptr) const
  {
    _mm512_store_pd(ptr, data);
  }

  DART_SIMD_INLINE void storeu(double* ptr) const
  {
    _mm512_storeu_pd(ptr, data);
  }

  DART_SIMD_INLINE double operator[](std::size_t i) const
  {
    alignas(64) double tmp[8];
    _mm512_store_pd(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(_mm512_add_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(_mm512_sub_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(_mm512_mul_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    return Vec(_mm512_div_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(_mm512_xor_pd(data, _mm512_set1_pd(-0.0)));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = _mm512_add_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = _mm512_sub_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = _mm512_mul_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    data = _mm512_div_pd(data, other.data);
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
struct Vec<std::int32_t, 16>
{
  using scalar_type = std::int32_t;
  static constexpr std::size_t width = 16;
  using mask_type = VecMask<std::int32_t, 16>;

  __m512i data;

  Vec() = default;

  DART_SIMD_INLINE Vec(__m512i v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(_mm512_setzero_si512());
  }

  DART_SIMD_INLINE static Vec broadcast(std::int32_t value)
  {
    return Vec(_mm512_set1_epi32(value));
  }

  DART_SIMD_INLINE static Vec load(const std::int32_t* ptr)
  {
    return Vec(_mm512_load_si512(reinterpret_cast<const __m512i*>(ptr)));
  }

  DART_SIMD_INLINE static Vec loadu(const std::int32_t* ptr)
  {
    return Vec(_mm512_loadu_si512(reinterpret_cast<const __m512i*>(ptr)));
  }

  DART_SIMD_INLINE void store(std::int32_t* ptr) const
  {
    _mm512_store_si512(reinterpret_cast<__m512i*>(ptr), data);
  }

  DART_SIMD_INLINE void storeu(std::int32_t* ptr) const
  {
    _mm512_storeu_si512(reinterpret_cast<__m512i*>(ptr), data);
  }

  DART_SIMD_INLINE std::int32_t operator[](std::size_t i) const
  {
    alignas(64) std::int32_t tmp[16];
    _mm512_store_si512(reinterpret_cast<__m512i*>(tmp), data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(_mm512_add_epi32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(_mm512_sub_epi32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(_mm512_mullo_epi32(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    alignas(64) std::int32_t a[16], b[16], r[16];
    _mm512_store_si512(reinterpret_cast<__m512i*>(a), data);
    _mm512_store_si512(reinterpret_cast<__m512i*>(b), other.data);
    for (int j = 0; j < 16; ++j)
      r[j] = a[j] / b[j];
    return Vec(_mm512_load_si512(reinterpret_cast<const __m512i*>(r)));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(_mm512_sub_epi32(_mm512_setzero_si512(), data));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = _mm512_add_epi32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = _mm512_sub_epi32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = _mm512_mullo_epi32(data, other.data);
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

#endif // DART_SIMD_AVX512
