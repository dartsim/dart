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

#include <array>

#include <cstddef>
#include <cstdint>

namespace dart::simd {

template <typename T, std::size_t Width>
struct VecMask
{
  using scalar_type = T;
  static constexpr std::size_t width = Width;

  std::array<bool, Width> data;

  VecMask() = default;

  DART_SIMD_INLINE constexpr VecMask(bool value) noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] = value;
    }
  }

  DART_SIMD_INLINE constexpr VecMask(
      const std::array<bool, Width>& arr) noexcept
    : data(arr)
  {
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool all() const noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      if (!data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool any() const noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      if (data[i]) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool none() const noexcept
  {
    return !any();
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr int popcount() const noexcept
  {
    int count = 0;
    for (std::size_t i = 0; i < Width; ++i) {
      if (data[i]) {
        ++count;
      }
    }
    return count;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr std::uint32_t bitmask()
      const noexcept
  {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < Width; ++i) {
      if (data[i]) {
        mask |= (std::uint32_t{1} << i);
      }
    }
    return mask;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool operator[](
      std::size_t i) const noexcept
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool& operator[](
      std::size_t i) noexcept
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr VecMask operator&(
      const VecMask& other) const noexcept
  {
    VecMask result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] && other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr VecMask operator|(
      const VecMask& other) const noexcept
  {
    VecMask result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] || other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr VecMask operator^(
      const VecMask& other) const noexcept
  {
    VecMask result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] != other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr VecMask operator~() const noexcept
  {
    VecMask result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = !data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr VecMask operator&&(
      const VecMask& other) const noexcept
  {
    return *this & other;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr VecMask operator||(
      const VecMask& other) const noexcept
  {
    return *this | other;
  }

  DART_SIMD_INLINE constexpr VecMask& operator&=(const VecMask& other) noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] = data[i] && other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE constexpr VecMask& operator|=(const VecMask& other) noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] = data[i] || other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE constexpr VecMask& operator^=(const VecMask& other) noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] = data[i] != other.data[i];
    }
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool operator==(
      const VecMask& other) const noexcept
  {
    for (std::size_t i = 0; i < Width; ++i) {
      if (data[i] != other.data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE constexpr bool operator!=(
      const VecMask& other) const noexcept
  {
    return !(*this == other);
  }
};

} // namespace dart::simd
