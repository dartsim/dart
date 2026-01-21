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

#if defined(DART_SIMD_AVX) && !defined(DART_SIMD_AVX2)

namespace dart::simd {

template <typename T, std::size_t Width>
struct VecMask;

template <>
struct Vec<float, 8>
{
  using scalar_type = float;
  static constexpr std::size_t width = 8;
  using mask_type = VecMask<float, 8>;

  __m256 data;

  Vec() = default;

  DART_SIMD_INLINE Vec(__m256 v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(_mm256_setzero_ps());
  }

  DART_SIMD_INLINE static Vec broadcast(float value)
  {
    return Vec(_mm256_set1_ps(value));
  }

  DART_SIMD_INLINE static Vec load(const float* ptr)
  {
    return Vec(_mm256_load_ps(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const float* ptr)
  {
    return Vec(_mm256_loadu_ps(ptr));
  }

  DART_SIMD_INLINE void store(float* ptr) const
  {
    _mm256_store_ps(ptr, data);
  }

  DART_SIMD_INLINE void storeu(float* ptr) const
  {
    _mm256_storeu_ps(ptr, data);
  }

  DART_SIMD_INLINE float operator[](std::size_t i) const
  {
    alignas(32) float tmp[8];
    _mm256_store_ps(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(_mm256_add_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(_mm256_sub_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(_mm256_mul_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    return Vec(_mm256_div_ps(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(_mm256_xor_ps(data, _mm256_set1_ps(-0.0f)));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = _mm256_add_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = _mm256_sub_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = _mm256_mul_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    data = _mm256_div_ps(data, other.data);
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
struct Vec<double, 4>
{
  using scalar_type = double;
  static constexpr std::size_t width = 4;
  using mask_type = VecMask<double, 4>;

  __m256d data;

  Vec() = default;

  DART_SIMD_INLINE Vec(__m256d v) : data(v) {}

  DART_SIMD_INLINE static Vec zero()
  {
    return Vec(_mm256_setzero_pd());
  }

  DART_SIMD_INLINE static Vec broadcast(double value)
  {
    return Vec(_mm256_set1_pd(value));
  }

  DART_SIMD_INLINE static Vec load(const double* ptr)
  {
    return Vec(_mm256_load_pd(ptr));
  }

  DART_SIMD_INLINE static Vec loadu(const double* ptr)
  {
    return Vec(_mm256_loadu_pd(ptr));
  }

  DART_SIMD_INLINE void store(double* ptr) const
  {
    _mm256_store_pd(ptr, data);
  }

  DART_SIMD_INLINE void storeu(double* ptr) const
  {
    _mm256_storeu_pd(ptr, data);
  }

  DART_SIMD_INLINE double operator[](std::size_t i) const
  {
    alignas(32) double tmp[4];
    _mm256_store_pd(tmp, data);
    return tmp[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    return Vec(_mm256_add_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    return Vec(_mm256_sub_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    return Vec(_mm256_mul_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    return Vec(_mm256_div_pd(data, other.data));
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    return Vec(_mm256_xor_pd(data, _mm256_set1_pd(-0.0)));
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    data = _mm256_add_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    data = _mm256_sub_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    data = _mm256_mul_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    data = _mm256_div_pd(data, other.data);
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

#endif // DART_SIMD_AVX && !DART_SIMD_AVX2
