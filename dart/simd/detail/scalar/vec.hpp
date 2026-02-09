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
struct VecMask;

template <typename T, std::size_t Width>
struct Vec
{
  using scalar_type = T;
  static constexpr std::size_t width = Width;
  using mask_type = VecMask<T, Width>;

  std::array<T, Width> data;

  Vec() = default;

  DART_SIMD_INLINE constexpr Vec(const std::array<T, Width>& arr) : data(arr) {}

  DART_SIMD_INLINE static constexpr Vec zero()
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = T{0};
    }
    return result;
  }

  DART_SIMD_INLINE static constexpr Vec broadcast(T value)
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = value;
    }
    return result;
  }

  /// Create from explicit values (variadic for any width)
  template <typename... Args>
    requires(sizeof...(Args) == Width && (std::same_as<Args, T> && ...))
  DART_SIMD_INLINE static constexpr Vec set(Args... args)
  {
    return Vec(std::array<T, Width>{static_cast<T>(args)...});
  }

  DART_SIMD_INLINE static Vec load(const T* ptr)
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = ptr[i];
    }
    return result;
  }

  DART_SIMD_INLINE static Vec loadu(const T* ptr)
  {
    return load(ptr);
  }

  DART_SIMD_INLINE void store(T* ptr) const
  {
    for (std::size_t i = 0; i < Width; ++i) {
      ptr[i] = data[i];
    }
  }

  DART_SIMD_INLINE void storeu(T* ptr) const
  {
    store(ptr);
  }

  DART_SIMD_INLINE constexpr T& operator[](std::size_t i)
  {
    return data[i];
  }

  DART_SIMD_INLINE constexpr const T& operator[](std::size_t i) const
  {
    return data[i];
  }

  DART_SIMD_INLINE constexpr Vec operator+(const Vec& other) const
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] + other.data[i];
    }
    return result;
  }

  DART_SIMD_INLINE constexpr Vec operator-(const Vec& other) const
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] - other.data[i];
    }
    return result;
  }

  DART_SIMD_INLINE constexpr Vec operator*(const Vec& other) const
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] * other.data[i];
    }
    return result;
  }

  DART_SIMD_INLINE constexpr Vec operator/(const Vec& other) const
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = data[i] / other.data[i];
    }
    return result;
  }

  DART_SIMD_INLINE constexpr Vec operator-() const
  {
    Vec result;
    for (std::size_t i = 0; i < Width; ++i) {
      result.data[i] = -data[i];
    }
    return result;
  }

  DART_SIMD_INLINE constexpr Vec& operator+=(const Vec& other)
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] += other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE constexpr Vec& operator-=(const Vec& other)
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] -= other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE constexpr Vec& operator*=(const Vec& other)
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] *= other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE constexpr Vec& operator/=(const Vec& other)
  {
    for (std::size_t i = 0; i < Width; ++i) {
      data[i] /= other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE constexpr mask_type operator==(const Vec& other) const;
  DART_SIMD_INLINE constexpr mask_type operator!=(const Vec& other) const;
  DART_SIMD_INLINE constexpr mask_type operator<(const Vec& other) const;
  DART_SIMD_INLINE constexpr mask_type operator<=(const Vec& other) const;
  DART_SIMD_INLINE constexpr mask_type operator>(const Vec& other) const;
  DART_SIMD_INLINE constexpr mask_type operator>=(const Vec& other) const;
};

} // namespace dart::simd

#include <dart/simd/detail/scalar/vec_mask.hpp>

namespace dart::simd {

template <typename T, std::size_t Width>
DART_SIMD_INLINE constexpr typename Vec<T, Width>::mask_type
Vec<T, Width>::operator==(const Vec& other) const
{
  mask_type result;
  for (std::size_t i = 0; i < Width; ++i) {
    result.data[i] = data[i] == other.data[i];
  }
  return result;
}

template <typename T, std::size_t Width>
DART_SIMD_INLINE constexpr typename Vec<T, Width>::mask_type
Vec<T, Width>::operator!=(const Vec& other) const
{
  mask_type result;
  for (std::size_t i = 0; i < Width; ++i) {
    result.data[i] = data[i] != other.data[i];
  }
  return result;
}

template <typename T, std::size_t Width>
DART_SIMD_INLINE constexpr typename Vec<T, Width>::mask_type
Vec<T, Width>::operator<(const Vec& other) const
{
  mask_type result;
  for (std::size_t i = 0; i < Width; ++i) {
    result.data[i] = data[i] < other.data[i];
  }
  return result;
}

template <typename T, std::size_t Width>
DART_SIMD_INLINE constexpr typename Vec<T, Width>::mask_type
Vec<T, Width>::operator<=(const Vec& other) const
{
  mask_type result;
  for (std::size_t i = 0; i < Width; ++i) {
    result.data[i] = data[i] <= other.data[i];
  }
  return result;
}

template <typename T, std::size_t Width>
DART_SIMD_INLINE constexpr typename Vec<T, Width>::mask_type
Vec<T, Width>::operator>(const Vec& other) const
{
  mask_type result;
  for (std::size_t i = 0; i < Width; ++i) {
    result.data[i] = data[i] > other.data[i];
  }
  return result;
}

template <typename T, std::size_t Width>
DART_SIMD_INLINE constexpr typename Vec<T, Width>::mask_type
Vec<T, Width>::operator>=(const Vec& other) const
{
  mask_type result;
  for (std::size_t i = 0; i < Width; ++i) {
    result.data[i] = data[i] >= other.data[i];
  }
  return result;
}

} // namespace dart::simd
