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

template <>
struct VecMask<float, 16>
{
  using scalar_type = float;
  static constexpr std::size_t width = 16;

  __mmask16 data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__mmask16 v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value) : data(value ? 0xFFFF : 0) {}

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return data == 0xFFFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return data != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return data == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    return _mm_popcnt_u32(data);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(data);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (data >> i) & 1;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_kand_mask16(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_kor_mask16(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_kxor_mask16(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_knot_mask16(data));
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
    data = _kand_mask16(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _kor_mask16(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _kxor_mask16(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    return data == other.data;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return data != other.data;
  }
};

template <>
struct VecMask<double, 8>
{
  using scalar_type = double;
  static constexpr std::size_t width = 8;

  __mmask8 data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__mmask8 v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value) : data(value ? 0xFF : 0) {}

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return data == 0xFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return data != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return data == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    return _mm_popcnt_u32(data);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(data);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (data >> i) & 1;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_kand_mask8(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_kor_mask8(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_kxor_mask8(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_knot_mask8(data));
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
    data = _kand_mask8(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _kor_mask8(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _kxor_mask8(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    return data == other.data;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return data != other.data;
  }
};

template <>
struct VecMask<std::int32_t, 16>
{
  using scalar_type = std::int32_t;
  static constexpr std::size_t width = 16;

  __mmask16 data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(__mmask16 v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value) : data(value ? 0xFFFF : 0) {}

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return data == 0xFFFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return data != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return data == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    return _mm_popcnt_u32(data);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return static_cast<std::uint32_t>(data);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return (data >> i) & 1;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(_kand_mask16(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(_kor_mask16(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(_kxor_mask16(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(_knot_mask16(data));
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
    data = _kand_mask16(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = _kor_mask16(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = _kxor_mask16(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    return data == other.data;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return data != other.data;
  }
};

DART_SIMD_INLINE VecMask<float, 16> Vec<float, 16>::operator==(
    const Vec& other) const
{
  return VecMask<float, 16>(_mm512_cmp_ps_mask(data, other.data, _CMP_EQ_OQ));
}

DART_SIMD_INLINE VecMask<float, 16> Vec<float, 16>::operator!=(
    const Vec& other) const
{
  return VecMask<float, 16>(_mm512_cmp_ps_mask(data, other.data, _CMP_NEQ_OQ));
}

DART_SIMD_INLINE VecMask<float, 16> Vec<float, 16>::operator<(
    const Vec& other) const
{
  return VecMask<float, 16>(_mm512_cmp_ps_mask(data, other.data, _CMP_LT_OQ));
}

DART_SIMD_INLINE VecMask<float, 16> Vec<float, 16>::operator<=(
    const Vec& other) const
{
  return VecMask<float, 16>(_mm512_cmp_ps_mask(data, other.data, _CMP_LE_OQ));
}

DART_SIMD_INLINE VecMask<float, 16> Vec<float, 16>::operator>(
    const Vec& other) const
{
  return VecMask<float, 16>(_mm512_cmp_ps_mask(data, other.data, _CMP_GT_OQ));
}

DART_SIMD_INLINE VecMask<float, 16> Vec<float, 16>::operator>=(
    const Vec& other) const
{
  return VecMask<float, 16>(_mm512_cmp_ps_mask(data, other.data, _CMP_GE_OQ));
}

DART_SIMD_INLINE VecMask<double, 8> Vec<double, 8>::operator==(
    const Vec& other) const
{
  return VecMask<double, 8>(_mm512_cmp_pd_mask(data, other.data, _CMP_EQ_OQ));
}

DART_SIMD_INLINE VecMask<double, 8> Vec<double, 8>::operator!=(
    const Vec& other) const
{
  return VecMask<double, 8>(_mm512_cmp_pd_mask(data, other.data, _CMP_NEQ_OQ));
}

DART_SIMD_INLINE VecMask<double, 8> Vec<double, 8>::operator<(
    const Vec& other) const
{
  return VecMask<double, 8>(_mm512_cmp_pd_mask(data, other.data, _CMP_LT_OQ));
}

DART_SIMD_INLINE VecMask<double, 8> Vec<double, 8>::operator<=(
    const Vec& other) const
{
  return VecMask<double, 8>(_mm512_cmp_pd_mask(data, other.data, _CMP_LE_OQ));
}

DART_SIMD_INLINE VecMask<double, 8> Vec<double, 8>::operator>(
    const Vec& other) const
{
  return VecMask<double, 8>(_mm512_cmp_pd_mask(data, other.data, _CMP_GT_OQ));
}

DART_SIMD_INLINE VecMask<double, 8> Vec<double, 8>::operator>=(
    const Vec& other) const
{
  return VecMask<double, 8>(_mm512_cmp_pd_mask(data, other.data, _CMP_GE_OQ));
}

DART_SIMD_INLINE VecMask<std::int32_t, 16> Vec<std::int32_t, 16>::operator==(
    const Vec& other) const
{
  return VecMask<std::int32_t, 16>(_mm512_cmpeq_epi32_mask(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 16> Vec<std::int32_t, 16>::operator!=(
    const Vec& other) const
{
  return ~(*this == other);
}

DART_SIMD_INLINE VecMask<std::int32_t, 16> Vec<std::int32_t, 16>::operator<(
    const Vec& other) const
{
  return VecMask<std::int32_t, 16>(_mm512_cmplt_epi32_mask(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 16> Vec<std::int32_t, 16>::operator<=(
    const Vec& other) const
{
  return VecMask<std::int32_t, 16>(_mm512_cmple_epi32_mask(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 16> Vec<std::int32_t, 16>::operator>(
    const Vec& other) const
{
  return VecMask<std::int32_t, 16>(_mm512_cmpgt_epi32_mask(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 16> Vec<std::int32_t, 16>::operator>=(
    const Vec& other) const
{
  return VecMask<std::int32_t, 16>(_mm512_cmpge_epi32_mask(data, other.data));
}

} // namespace dart::simd

#endif // DART_SIMD_AVX512
