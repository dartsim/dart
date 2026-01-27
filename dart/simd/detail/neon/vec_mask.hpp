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

template <>
struct VecMask<float, 4>
{
  using scalar_type = float;
  static constexpr std::size_t width = 4;

  uint32x4_t data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(uint32x4_t v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? vdupq_n_u32(0xFFFFFFFF) : vdupq_n_u32(0))
  {
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return vminvq_u32(data) == 0xFFFFFFFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return vmaxvq_u32(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return vmaxvq_u32(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    alignas(16) std::uint32_t tmp[4];
    vst1q_u32(tmp, data);
    return (tmp[0] ? 1 : 0) + (tmp[1] ? 1 : 0) + (tmp[2] ? 1 : 0)
           + (tmp[3] ? 1 : 0);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    alignas(16) std::uint32_t tmp[4];
    vst1q_u32(tmp, data);
    return ((tmp[0] ? 1u : 0u) << 0) | ((tmp[1] ? 1u : 0u) << 1)
           | ((tmp[2] ? 1u : 0u) << 2) | ((tmp[3] ? 1u : 0u) << 3);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    alignas(16) std::uint32_t tmp[4];
    vst1q_u32(tmp, data);
    return tmp[i] != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(vandq_u32(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(vorrq_u32(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(veorq_u32(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(vmvnq_u32(data));
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
    data = vandq_u32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = vorrq_u32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = veorq_u32(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    uint32x4_t xored = veorq_u32(data, other.data);
    return vmaxvq_u32(xored) == 0;
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

  uint64x2_t data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(uint64x2_t v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? vdupq_n_u64(0xFFFFFFFFFFFFFFFF) : vdupq_n_u64(0))
  {
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, data);
    return tmp[0] && tmp[1];
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, data);
    return tmp[0] || tmp[1];
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, data);
    return !tmp[0] && !tmp[1];
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, data);
    return (tmp[0] ? 1 : 0) + (tmp[1] ? 1 : 0);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, data);
    return ((tmp[0] ? 1u : 0u) << 0) | ((tmp[1] ? 1u : 0u) << 1);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, data);
    return tmp[i] != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(vandq_u64(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(vorrq_u64(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(veorq_u64(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(
        vreinterpretq_u64_u32(vmvnq_u32(vreinterpretq_u32_u64(data))));
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
    data = vandq_u64(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = vorrq_u64(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = veorq_u64(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    alignas(16) std::uint64_t tmp[2];
    vst1q_u64(tmp, veorq_u64(data, other.data));
    return tmp[0] == 0 && tmp[1] == 0;
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

  uint32x4_t data;

  VecMask() = default;

  DART_SIMD_INLINE VecMask(uint32x4_t v) : data(v) {}

  DART_SIMD_INLINE VecMask(bool value)
    : data(value ? vdupq_n_u32(0xFFFFFFFF) : vdupq_n_u32(0))
  {
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return vminvq_u32(data) == 0xFFFFFFFF;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return vmaxvq_u32(data) != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return vmaxvq_u32(data) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    alignas(16) std::uint32_t tmp[4];
    vst1q_u32(tmp, data);
    return (tmp[0] ? 1 : 0) + (tmp[1] ? 1 : 0) + (tmp[2] ? 1 : 0)
           + (tmp[3] ? 1 : 0);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    alignas(16) std::uint32_t tmp[4];
    vst1q_u32(tmp, data);
    return ((tmp[0] ? 1u : 0u) << 0) | ((tmp[1] ? 1u : 0u) << 1)
           | ((tmp[2] ? 1u : 0u) << 2) | ((tmp[3] ? 1u : 0u) << 3);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    alignas(16) std::uint32_t tmp[4];
    vst1q_u32(tmp, data);
    return tmp[i] != 0;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    return VecMask(vandq_u32(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    return VecMask(vorrq_u32(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    return VecMask(veorq_u32(data, other.data));
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    return VecMask(vmvnq_u32(data));
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
    data = vandq_u32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data = vorrq_u32(data, other.data);
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data = veorq_u32(data, other.data);
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    uint32x4_t xored = veorq_u32(data, other.data);
    return vmaxvq_u32(xored) == 0;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator==(
    const Vec& other) const
{
  return VecMask<float, 4>(vceqq_f32(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator!=(
    const Vec& other) const
{
  return ~(*this == other);
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator<(
    const Vec& other) const
{
  return VecMask<float, 4>(vcltq_f32(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator<=(
    const Vec& other) const
{
  return VecMask<float, 4>(vcleq_f32(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator>(
    const Vec& other) const
{
  return VecMask<float, 4>(vcgtq_f32(data, other.data));
}

DART_SIMD_INLINE VecMask<float, 4> Vec<float, 4>::operator>=(
    const Vec& other) const
{
  return VecMask<float, 4>(vcgeq_f32(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator==(
    const Vec& other) const
{
  return VecMask<double, 2>(vceqq_f64(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator!=(
    const Vec& other) const
{
  return ~(*this == other);
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator<(
    const Vec& other) const
{
  return VecMask<double, 2>(vcltq_f64(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator<=(
    const Vec& other) const
{
  return VecMask<double, 2>(vcleq_f64(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator>(
    const Vec& other) const
{
  return VecMask<double, 2>(vcgtq_f64(data, other.data));
}

DART_SIMD_INLINE VecMask<double, 2> Vec<double, 2>::operator>=(
    const Vec& other) const
{
  return VecMask<double, 2>(vcgeq_f64(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator==(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(vceqq_s32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator!=(
    const Vec& other) const
{
  return ~(*this == other);
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator<(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(vcltq_s32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator<=(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(vcleq_s32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator>(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(vcgtq_s32(data, other.data));
}

DART_SIMD_INLINE VecMask<std::int32_t, 4> Vec<std::int32_t, 4>::operator>=(
    const Vec& other) const
{
  return VecMask<std::int32_t, 4>(vcgeq_s32(data, other.data));
}

} // namespace dart::simd

#endif // DART_SIMD_NEON
