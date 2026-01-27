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

#if defined(DART_SIMD_SSE42)

namespace dart::simd {

template <>
struct VecMask<float, 4>
{
  using scalar_type = float;
  static constexpr std::size_t width = 4;

  __m128 data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__m128 v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? _mm_castsi128_ps(_mm_set1_epi32(-1)) : _mm_setzero_ps())
  {
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return _mm_movemask_ps(data) == 0xF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return _mm_movemask_ps(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return _mm_movemask_ps(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    return _mm_popcnt_u32(static_cast<unsigned int>(_mm_movemask_ps(data)));
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(_mm_movemask_ps(data));
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (bitmask() >> i) & 1;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_mm_and_ps(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_mm_or_ps(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_mm_xor_ps(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_mm_xor_ps(data, _mm_castsi128_ps(_mm_set1_epi32(-1))));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&&(const VecMask& other) const
  {
    return *this & other;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator||(const VecMask& other) const
  {
    return *this | other;
  }

  DART_SIMD_INLINE VecMask& operator&=(const VecMask& other)
  {
    data = _mm_and_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _mm_or_ps(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _mm_xor_ps(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    return _mm_movemask_ps(_mm_xor_ps(data, other.data)) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

template <>
struct VecMask<double, 2>
{
  using scalar_type = double;
  static constexpr std::size_t width = 2;

  __m128d data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__m128d v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? _mm_castsi128_pd(_mm_set1_epi64x(-1)) : _mm_setzero_pd())
  {
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return _mm_movemask_pd(data) == 0x3;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return _mm_movemask_pd(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return _mm_movemask_pd(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    return _mm_popcnt_u32(static_cast<unsigned int>(_mm_movemask_pd(data)));
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(_mm_movemask_pd(data));
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (bitmask() >> i) & 1;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_mm_and_pd(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_mm_or_pd(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_mm_xor_pd(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_mm_xor_pd(data, _mm_castsi128_pd(_mm_set1_epi64x(-1))));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&&(const VecMask& other) const
  {
    return *this & other;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator||(const VecMask& other) const
  {
    return *this | other;
  }

  DART_SIMD_INLINE VecMask& operator&=(const VecMask& other)
  {
    data = _mm_and_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _mm_or_pd(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _mm_xor_pd(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    return _mm_movemask_pd(_mm_xor_pd(data, other.data)) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

template <>
struct VecMask<std::int32_t, 4>
{
  using scalar_type = std::int32_t;
  static constexpr std::size_t width = 4;

  __m128i data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__m128i v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? _mm_set1_epi32(-1) : _mm_setzero_si128())
  {
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return _mm_movemask_epi8(data) == 0xFFFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return _mm_movemask_epi8(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return _mm_movemask_epi8(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    int mask = _mm_movemask_ps(_mm_castsi128_ps(data));
    return _mm_popcnt_u32(static_cast<unsigned int>(mask));
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(_mm_movemask_ps(_mm_castsi128_ps(data)));
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (bitmask() >> i) & 1;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_mm_and_si128(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_mm_or_si128(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_mm_xor_si128(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_mm_xor_si128(data, _mm_set1_epi32(-1)));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&&(const VecMask& other) const
  {
    return *this & other;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator||(const VecMask& other) const
  {
    return *this | other;
  }

  DART_SIMD_INLINE VecMask& operator&=(const VecMask& other)
  {
    data = _mm_and_si128(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _mm_or_si128(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _mm_xor_si128(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    __m128i xored = _mm_xor_si128(data, other.data);
    return _mm_movemask_epi8(xored) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator==(
    const Vec& other) const
{
  return VecMask<float, 4>(_mm_cmpeq_ps(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator!=(
    const Vec& other) const
{
  return VecMask<float, 4>(_mm_cmpneq_ps(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator<(
    const Vec& other) const
{
  return VecMask<float, 4>(_mm_cmplt_ps(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator<=(
    const Vec& other) const
{
  return VecMask<float, 4>(_mm_cmple_ps(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator>(
    const Vec& other) const
{
  return VecMask<float, 4>(_mm_cmpgt_ps(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator>=(
    const Vec& other) const
{
  return VecMask<float, 4>(_mm_cmpge_ps(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator==(
    const Vec& other) const
{
  return VecMask<double, 2>(_mm_cmpeq_pd(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator!=(
    const Vec& other) const
{
  return VecMask<double, 2>(_mm_cmpneq_pd(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator<(
    const Vec& other) const
{
  return VecMask<double, 2>(_mm_cmplt_pd(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator<=(
    const Vec& other) const
{
  return VecMask<double, 2>(_mm_cmple_pd(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator>(
    const Vec& other) const
{
  return VecMask<double, 2>(_mm_cmpgt_pd(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator>=(
    const Vec& other) const
{
  return VecMask<double, 2>(_mm_cmpge_pd(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator==(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(_mm_cmpeq_epi32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator!=(
    const Vec& other) const
{
  return ~(*this == other);
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator<(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(_mm_cmplt_epi32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator<=(
    const Vec& other) const
{
  return ~(*this > other);
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator>(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(_mm_cmpgt_epi32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator>=(
    const Vec& other) const
{
  return ~(*this < other);
}

} // namespace dart::simd

#endif // DART_SIMD_SSE42
