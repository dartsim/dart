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

#include <cstddef>
#include <cstdint>

#if defined(DART_SIMD_AVX) && !defined(DART_SIMD_AVX2)

namespace dart::simd {

template <>
struct VecMask<float, 8>
{
  using scalar_type = float;
  static constexpr std::size_t width = 8;

  __m256 data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__m256 v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? _mm256_cmp_ps(_mm256_setzero_ps(), _mm256_setzero_ps(), _CMP_EQ_OQ)
                 : _mm256_setzero_ps())
  {
  }

  DART_SIMD_INLINE static VecMask all_true()
  {
    return VecMask(_mm256_cmp_ps(
        _mm256_setzero_ps(), _mm256_setzero_ps(), _CMP_EQ_OQ));
  }

  DART_SIMD_INLINE static VecMask all_false()
  {
    return VecMask(_mm256_setzero_ps());
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return _mm256_movemask_ps(data) == 0xFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return _mm256_movemask_ps(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return _mm256_movemask_ps(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    int mask = _mm256_movemask_ps(data);
    int count = 0;
    while (mask) {
      count += mask & 1;
      mask >>= 1;
    }
    return count;
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(_mm256_movemask_ps(data));
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (bitmask() >> i) & 1;
  }

  DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_mm256_and_ps(data, other.data));
  }

  DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_mm256_or_ps(data, other.data));
  }

  DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_mm256_xor_ps(data, other.data));
  }

  DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_mm256_xor_ps(data, all_true().data));
  }

  DART_SIMD_INLINE VecMask operator&&(const VecMask& other) const
  {
    return *this & other;
  }

  DART_SIMD_INLINE VecMask operator||(const VecMask& other) const
  {
    return *this | other;
  }

  DART_SIMD_INLINE VecMask& operator&=(const VecMask& other)
  {
    data = _mm256_and_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _mm256_or_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _mm256_xor_ps(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    __m256 xored = _mm256_xor_ps(data, other.data);
    return _mm256_movemask_ps(xored) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

template <>
struct VecMask<double, 4>
{
  using scalar_type = double;
  static constexpr std::size_t width = 4;

  __m256d data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__m256d v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? _mm256_cmp_pd(_mm256_setzero_pd(), _mm256_setzero_pd(), _CMP_EQ_OQ)
                 : _mm256_setzero_pd())
  {
  }

  DART_SIMD_INLINE static VecMask all_true()
  {
    return VecMask(_mm256_cmp_pd(
        _mm256_setzero_pd(), _mm256_setzero_pd(), _CMP_EQ_OQ));
  }

  DART_SIMD_INLINE static VecMask all_false()
  {
    return VecMask(_mm256_setzero_pd());
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return _mm256_movemask_pd(data) == 0xF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return _mm256_movemask_pd(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return _mm256_movemask_pd(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    int mask = _mm256_movemask_pd(data);
    int count = 0;
    while (mask) {
      count += mask & 1;
      mask >>= 1;
    }
    return count;
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(_mm256_movemask_pd(data));
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (bitmask() >> i) & 1;
  }

  DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_mm256_and_pd(data, other.data));
  }

  DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_mm256_or_pd(data, other.data));
  }

  DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_mm256_xor_pd(data, other.data));
  }

  DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_mm256_xor_pd(data, all_true().data));
  }

  DART_SIMD_INLINE VecMask operator&&(const VecMask& other) const
  {
    return *this & other;
  }

  DART_SIMD_INLINE VecMask operator||(const VecMask& other) const
  {
    return *this | other;
  }

  DART_SIMD_INLINE VecMask& operator&=(const VecMask& other)
  {
    data = _mm256_and_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _mm256_or_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _mm256_xor_pd(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    __m256d xored = _mm256_xor_pd(data, other.data);
    return _mm256_movemask_pd(xored) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

DART_SIMD_INLINE VecMask<float, 8> Vec<float, 8>::operator==(
    const Vec<float, 8>& other) const
{
  return VecMask<float, 8>(_mm256_cmp_ps(data, other.data, _CMP_EQ_OQ));
}

DART_SIMD_INLINE VecMask<float, 8> Vec<float, 8>::operator!=(
    const Vec<float, 8>& other) const
{
  return VecMask<float, 8>(_mm256_cmp_ps(data, other.data, _CMP_NEQ_OQ));
}

DART_SIMD_INLINE VecMask<float, 8> Vec<float, 8>::operator<(
    const Vec<float, 8>& other) const
{
  return VecMask<float, 8>(_mm256_cmp_ps(data, other.data, _CMP_LT_OQ));
}

DART_SIMD_INLINE VecMask<float, 8> Vec<float, 8>::operator<=(
    const Vec<float, 8>& other) const
{
  return VecMask<float, 8>(_mm256_cmp_ps(data, other.data, _CMP_LE_OQ));
}

DART_SIMD_INLINE VecMask<float, 8> Vec<float, 8>::operator>(
    const Vec<float, 8>& other) const
{
  return VecMask<float, 8>(_mm256_cmp_ps(data, other.data, _CMP_GT_OQ));
}

DART_SIMD_INLINE VecMask<float, 8> Vec<float, 8>::operator>=(
    const Vec<float, 8>& other) const
{
  return VecMask<float, 8>(_mm256_cmp_ps(data, other.data, _CMP_GE_OQ));
}

DART_SIMD_INLINE VecMask<double, 4> Vec<double, 4>::operator==(
    const Vec<double, 4>& other) const
{
  return VecMask<double, 4>(_mm256_cmp_pd(data, other.data, _CMP_EQ_OQ));
}

DART_SIMD_INLINE VecMask<double, 4> Vec<double, 4>::operator!=(
    const Vec<double, 4>& other) const
{
  return VecMask<double, 4>(_mm256_cmp_pd(data, other.data, _CMP_NEQ_OQ));
}

DART_SIMD_INLINE VecMask<double, 4> Vec<double, 4>::operator<(
    const Vec<double, 4>& other) const
{
  return VecMask<double, 4>(_mm256_cmp_pd(data, other.data, _CMP_LT_OQ));
}

DART_SIMD_INLINE VecMask<double, 4> Vec<double, 4>::operator<=(
    const Vec<double, 4>& other) const
{
  return VecMask<double, 4>(_mm256_cmp_pd(data, other.data, _CMP_LE_OQ));
}

DART_SIMD_INLINE VecMask<double, 4> Vec<double, 4>::operator>(
    const Vec<double, 4>& other) const
{
  return VecMask<double, 4>(_mm256_cmp_pd(data, other.data, _CMP_GT_OQ));
}

DART_SIMD_INLINE VecMask<double, 4> Vec<double, 4>::operator>=(
    const Vec<double, 4>& other) const
{
  return VecMask<double, 4>(_mm256_cmp_pd(data, other.data, _CMP_GE_OQ));
}

} // namespace dart::simd

#endif // DART_SIMD_AVX && !DART_SIMD_AVX2
