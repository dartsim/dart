/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the "BSD-style" License.
 */

#pragma once

#include <dart/simd/config.hpp>

#if defined(DART_SIMD_SVE)

  #include <cstddef>
  #include <cstdint>

namespace dart::simd {

// SVE predicates (svbool_t) are variable-length; this uses scalar fallback for
// cross-backend compatibility. Use predicated SVE intrinsics for performance.
template <>
struct VecMask<float, 4>
{
  using scalar_type = float;
  static constexpr std::size_t width = 4;

  alignas(16) bool data[4];

  VecMask() = default;

  DART_SIMD_INLINE VecMask(bool value)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = value;
    }
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (!data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return !any();
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    int count = 0;
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        ++count;
      }
    }
    return count;
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        mask |= (std::uint32_t{1} << i);
      }
    }
    return mask;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE bool& operator[](std::size_t i)
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] && other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] || other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] != other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = !data[i];
    }
    return result;
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
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] && other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] || other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] != other.data[i];
    }
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i] != other.data[i]) {
        return false;
      }
    }
    return true;
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

  alignas(16) bool data[2];

  VecMask() = default;

  DART_SIMD_INLINE VecMask(bool value)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = value;
    }
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    return data[0] && data[1];
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    return data[0] || data[1];
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return !data[0] && !data[1];
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    return (data[0] ? 1 : 0) + (data[1] ? 1 : 0);
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    return (data[0] ? 1u : 0u) | (data[1] ? 2u : 0u);
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE bool& operator[](std::size_t i)
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    VecMask result;
    result.data[0] = data[0] && other.data[0];
    result.data[1] = data[1] && other.data[1];
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    VecMask result;
    result.data[0] = data[0] || other.data[0];
    result.data[1] = data[1] || other.data[1];
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    VecMask result;
    result.data[0] = data[0] != other.data[0];
    result.data[1] = data[1] != other.data[1];
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    VecMask result;
    result.data[0] = !data[0];
    result.data[1] = !data[1];
    return result;
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
    data[0] = data[0] && other.data[0];
    data[1] = data[1] && other.data[1];
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    data[0] = data[0] || other.data[0];
    data[1] = data[1] || other.data[1];
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    data[0] = data[0] != other.data[0];
    data[1] = data[1] != other.data[1];
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    return data[0] == other.data[0] && data[1] == other.data[1];
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

  alignas(16) bool data[4];

  VecMask() = default;

  DART_SIMD_INLINE VecMask(bool value)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = value;
    }
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (!data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return !any();
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    int count = 0;
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        ++count;
      }
    }
    return count;
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        mask |= (std::uint32_t{1} << i);
      }
    }
    return mask;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE bool& operator[](std::size_t i)
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] && other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] || other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] != other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = !data[i];
    }
    return result;
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
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] && other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] || other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] != other.data[i];
    }
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i] != other.data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

template <>
struct VecMask<std::uint32_t, 4>
{
  using scalar_type = std::uint32_t;
  static constexpr std::size_t width = 4;

  alignas(16) bool data[4];

  VecMask() = default;

  DART_SIMD_INLINE VecMask(bool value)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = value;
    }
  }

  [[nodiscard]] DART_SIMD_INLINE bool all() const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (!data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE bool any() const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] DART_SIMD_INLINE bool none() const
  {
    return !any();
  }

  [[nodiscard]] DART_SIMD_INLINE int popcount() const
  {
    int count = 0;
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        ++count;
      }
    }
    return count;
  }

  [[nodiscard]] DART_SIMD_INLINE std::uint32_t bitmask() const
  {
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i]) {
        mask |= (std::uint32_t{1} << i);
      }
    }
    return mask;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator[](std::size_t i) const
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE bool& operator[](std::size_t i)
  {
    return data[i];
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator&(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] && other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator|(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] || other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator^(const VecMask& other) const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = data[i] != other.data[i];
    }
    return result;
  }

  [[nodiscard]] DART_SIMD_INLINE VecMask operator~() const
  {
    VecMask result;
    for (std::size_t i = 0; i < width; ++i) {
      result.data[i] = !data[i];
    }
    return result;
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
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] && other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator|=(const VecMask& other)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] || other.data[i];
    }
    return *this;
  }

  DART_SIMD_INLINE VecMask& operator^=(const VecMask& other)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = data[i] != other.data[i];
    }
    return *this;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator==(const VecMask& other) const
  {
    for (std::size_t i = 0; i < width; ++i) {
      if (data[i] != other.data[i]) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] DART_SIMD_INLINE bool operator!=(const VecMask& other) const
  {
    return !(*this == other);
  }
};

} // namespace dart::simd

#endif // DART_SIMD_SVE
