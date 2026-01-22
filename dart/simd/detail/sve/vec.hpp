/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the "BSD-style" License.
 */

#pragma once

#include <dart/simd/config.hpp>

#if defined(DART_SIMD_SVE)

#include <arm_sve.h>

#include <cstddef>
#include <cstdint>

namespace dart::simd {

template <typename T, std::size_t Width>
struct VecMask;

template <>
struct Vec<float, 4>
{
  using scalar_type = float;
  static constexpr std::size_t width = 4;
  using mask_type = VecMask<float, 4>;

  alignas(16) float data[4];

  Vec() = default;

  DART_SIMD_INLINE explicit Vec(float v)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = v;
    }
  }

  DART_SIMD_INLINE static svbool_t pg()
  {
    return svwhilelt_b32(0, width);
  }

  DART_SIMD_INLINE static Vec zero()
  {
    Vec v;
    svst1_f32(pg(), v.data, svdup_f32(0.0f));
    return v;
  }

  DART_SIMD_INLINE static Vec broadcast(float value)
  {
    Vec v;
    svst1_f32(pg(), v.data, svdup_f32(value));
    return v;
  }

  DART_SIMD_INLINE static Vec load(const float* ptr)
  {
    Vec v;
    svst1_f32(pg(), v.data, svld1_f32(pg(), ptr));
    return v;
  }

  DART_SIMD_INLINE static Vec loadu(const float* ptr)
  {
    return load(ptr);
  }

  DART_SIMD_INLINE void store(float* ptr) const
  {
    svst1_f32(pg(), ptr, svld1_f32(pg(), data));
  }

  DART_SIMD_INLINE void storeu(float* ptr) const
  {
    store(ptr);
  }

  DART_SIMD_INLINE float operator[](std::size_t i) const
  {
    return data[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    Vec res;
    svfloat32_t a = svld1_f32(pg(), data);
    svfloat32_t b = svld1_f32(pg(), other.data);
    svst1_f32(pg(), res.data, svadd_f32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    Vec res;
    svfloat32_t a = svld1_f32(pg(), data);
    svfloat32_t b = svld1_f32(pg(), other.data);
    svst1_f32(pg(), res.data, svsub_f32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    Vec res;
    svfloat32_t a = svld1_f32(pg(), data);
    svfloat32_t b = svld1_f32(pg(), other.data);
    svst1_f32(pg(), res.data, svmul_f32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    Vec res;
    svfloat32_t a = svld1_f32(pg(), data);
    svfloat32_t b = svld1_f32(pg(), other.data);
    svst1_f32(pg(), res.data, svdiv_f32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    Vec res;
    svfloat32_t a = svld1_f32(pg(), data);
    svst1_f32(pg(), res.data, svneg_f32_z(pg(), a));
    return res;
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    *this = *this + other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    *this = *this - other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    *this = *this * other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    *this = *this / other;
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] == other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] != other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] < other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] <= other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] > other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] >= other.data[i]);
    }
    return mask;
  }
};

template <>
struct Vec<double, 2>
{
  using scalar_type = double;
  static constexpr std::size_t width = 2;
  using mask_type = VecMask<double, 2>;

  alignas(16) double data[2];

  Vec() = default;

  DART_SIMD_INLINE explicit Vec(double v)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = v;
    }
  }

  DART_SIMD_INLINE static svbool_t pg()
  {
    return svwhilelt_b64(0, width);
  }

  DART_SIMD_INLINE static Vec zero()
  {
    Vec v;
    svst1_f64(pg(), v.data, svdup_f64(0.0));
    return v;
  }

  DART_SIMD_INLINE static Vec broadcast(double value)
  {
    Vec v;
    svst1_f64(pg(), v.data, svdup_f64(value));
    return v;
  }

  DART_SIMD_INLINE static Vec load(const double* ptr)
  {
    Vec v;
    svst1_f64(pg(), v.data, svld1_f64(pg(), ptr));
    return v;
  }

  DART_SIMD_INLINE static Vec loadu(const double* ptr)
  {
    return load(ptr);
  }

  DART_SIMD_INLINE void store(double* ptr) const
  {
    svst1_f64(pg(), ptr, svld1_f64(pg(), data));
  }

  DART_SIMD_INLINE void storeu(double* ptr) const
  {
    store(ptr);
  }

  DART_SIMD_INLINE double operator[](std::size_t i) const
  {
    return data[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    Vec res;
    svfloat64_t a = svld1_f64(pg(), data);
    svfloat64_t b = svld1_f64(pg(), other.data);
    svst1_f64(pg(), res.data, svadd_f64_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    Vec res;
    svfloat64_t a = svld1_f64(pg(), data);
    svfloat64_t b = svld1_f64(pg(), other.data);
    svst1_f64(pg(), res.data, svsub_f64_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    Vec res;
    svfloat64_t a = svld1_f64(pg(), data);
    svfloat64_t b = svld1_f64(pg(), other.data);
    svst1_f64(pg(), res.data, svmul_f64_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    Vec res;
    svfloat64_t a = svld1_f64(pg(), data);
    svfloat64_t b = svld1_f64(pg(), other.data);
    svst1_f64(pg(), res.data, svdiv_f64_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    Vec res;
    svfloat64_t a = svld1_f64(pg(), data);
    svst1_f64(pg(), res.data, svneg_f64_z(pg(), a));
    return res;
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    *this = *this + other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    *this = *this - other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    *this = *this * other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    *this = *this / other;
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] == other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] != other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] < other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] <= other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] > other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] >= other.data[i]);
    }
    return mask;
  }
};

template <>
struct Vec<std::int32_t, 4>
{
  using scalar_type = std::int32_t;
  static constexpr std::size_t width = 4;
  using mask_type = VecMask<std::int32_t, 4>;

  alignas(16) std::int32_t data[4];

  Vec() = default;

  DART_SIMD_INLINE explicit Vec(std::int32_t v)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = v;
    }
  }

  DART_SIMD_INLINE static svbool_t pg()
  {
    return svwhilelt_b32(0, width);
  }

  DART_SIMD_INLINE static Vec zero()
  {
    Vec v;
    svst1_s32(pg(), v.data, svdup_s32(0));
    return v;
  }

  DART_SIMD_INLINE static Vec broadcast(std::int32_t value)
  {
    Vec v;
    svst1_s32(pg(), v.data, svdup_s32(value));
    return v;
  }

  DART_SIMD_INLINE static Vec load(const std::int32_t* ptr)
  {
    Vec v;
    svst1_s32(pg(), v.data, svld1_s32(pg(), ptr));
    return v;
  }

  DART_SIMD_INLINE static Vec loadu(const std::int32_t* ptr)
  {
    return load(ptr);
  }

  DART_SIMD_INLINE void store(std::int32_t* ptr) const
  {
    svst1_s32(pg(), ptr, svld1_s32(pg(), data));
  }

  DART_SIMD_INLINE void storeu(std::int32_t* ptr) const
  {
    store(ptr);
  }

  DART_SIMD_INLINE std::int32_t operator[](std::size_t i) const
  {
    return data[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    Vec res;
    svint32_t a = svld1_s32(pg(), data);
    svint32_t b = svld1_s32(pg(), other.data);
    svst1_s32(pg(), res.data, svadd_s32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    Vec res;
    svint32_t a = svld1_s32(pg(), data);
    svint32_t b = svld1_s32(pg(), other.data);
    svst1_s32(pg(), res.data, svsub_s32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    Vec res;
    svint32_t a = svld1_s32(pg(), data);
    svint32_t b = svld1_s32(pg(), other.data);
    svst1_s32(pg(), res.data, svmul_s32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    Vec res;
    for (std::size_t i = 0; i < width; ++i) {
      res.data[i] = data[i] / other.data[i];
    }
    return res;
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    Vec res;
    svint32_t a = svld1_s32(pg(), data);
    svst1_s32(pg(), res.data, svneg_s32_z(pg(), a));
    return res;
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    *this = *this + other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    *this = *this - other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    *this = *this * other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    *this = *this / other;
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] == other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] != other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] < other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] <= other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] > other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] >= other.data[i]);
    }
    return mask;
  }
};

template <>
struct Vec<std::uint32_t, 4>
{
  using scalar_type = std::uint32_t;
  static constexpr std::size_t width = 4;
  using mask_type = VecMask<std::uint32_t, 4>;

  alignas(16) std::uint32_t data[4];

  Vec() = default;

  DART_SIMD_INLINE explicit Vec(std::uint32_t v)
  {
    for (std::size_t i = 0; i < width; ++i) {
      data[i] = v;
    }
  }

  DART_SIMD_INLINE static svbool_t pg()
  {
    return svwhilelt_b32(0, width);
  }

  DART_SIMD_INLINE static Vec zero()
  {
    Vec v;
    svst1_u32(pg(), v.data, svdup_u32(0));
    return v;
  }

  DART_SIMD_INLINE static Vec broadcast(std::uint32_t value)
  {
    Vec v;
    svst1_u32(pg(), v.data, svdup_u32(value));
    return v;
  }

  DART_SIMD_INLINE static Vec load(const std::uint32_t* ptr)
  {
    Vec v;
    svst1_u32(pg(), v.data, svld1_u32(pg(), ptr));
    return v;
  }

  DART_SIMD_INLINE static Vec loadu(const std::uint32_t* ptr)
  {
    return load(ptr);
  }

  DART_SIMD_INLINE void store(std::uint32_t* ptr) const
  {
    svst1_u32(pg(), ptr, svld1_u32(pg(), data));
  }

  DART_SIMD_INLINE void storeu(std::uint32_t* ptr) const
  {
    store(ptr);
  }

  DART_SIMD_INLINE std::uint32_t operator[](std::size_t i) const
  {
    return data[i];
  }

  DART_SIMD_INLINE Vec operator+(const Vec& other) const
  {
    Vec res;
    svuint32_t a = svld1_u32(pg(), data);
    svuint32_t b = svld1_u32(pg(), other.data);
    svst1_u32(pg(), res.data, svadd_u32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator-(const Vec& other) const
  {
    Vec res;
    svuint32_t a = svld1_u32(pg(), data);
    svuint32_t b = svld1_u32(pg(), other.data);
    svst1_u32(pg(), res.data, svsub_u32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator*(const Vec& other) const
  {
    Vec res;
    svuint32_t a = svld1_u32(pg(), data);
    svuint32_t b = svld1_u32(pg(), other.data);
    svst1_u32(pg(), res.data, svmul_u32_z(pg(), a, b));
    return res;
  }

  DART_SIMD_INLINE Vec operator/(const Vec& other) const
  {
    Vec res;
    for (std::size_t i = 0; i < width; ++i) {
      res.data[i] = data[i] / other.data[i];
    }
    return res;
  }

  DART_SIMD_INLINE Vec operator-() const
  {
    Vec res;
    for (std::size_t i = 0; i < width; ++i) {
      res.data[i] = static_cast<std::uint32_t>(-static_cast<std::int32_t>(data[i]));
    }
    return res;
  }

  DART_SIMD_INLINE Vec& operator+=(const Vec& other)
  {
    *this = *this + other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator-=(const Vec& other)
  {
    *this = *this - other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator*=(const Vec& other)
  {
    *this = *this * other;
    return *this;
  }

  DART_SIMD_INLINE Vec& operator/=(const Vec& other)
  {
    *this = *this / other;
    return *this;
  }

  DART_SIMD_INLINE mask_type operator==(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] == other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator!=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] != other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] < other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator<=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] <= other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] > other.data[i]);
    }
    return mask;
  }

  DART_SIMD_INLINE mask_type operator>=(const Vec& other) const
  {
    mask_type mask;
    for (std::size_t i = 0; i < width; ++i) {
      mask.data[i] = (data[i] >= other.data[i]);
    }
    return mask;
  }
};

} // namespace dart::simd

#endif // DART_SIMD_SVE
