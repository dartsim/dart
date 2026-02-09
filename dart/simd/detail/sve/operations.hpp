/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the "BSD-style" License.
 */

#pragma once

#include <dart/simd/config.hpp>

#if defined(DART_SIMD_SVE)

  #include <dart/simd/detail/sve/vec.hpp>

  #include <arm_sve.h>

namespace dart::simd {

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> abs(const Vec<float, 4>& v)
{
  Vec<float, 4> r;
  auto pg = Vec<float, 4>::pg();
  auto vv = svld1_f32(pg, v.data);
  svst1_f32(pg, r.data, svabs_f32_z(pg, vv));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> abs(const Vec<double, 2>& v)
{
  Vec<double, 2> r;
  auto pg = Vec<double, 2>::pg();
  auto vv = svld1_f64(pg, v.data);
  svst1_f64(pg, r.data, svabs_f64_z(pg, vv));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> sqrt(const Vec<float, 4>& v)
{
  Vec<float, 4> r;
  auto pg = Vec<float, 4>::pg();
  auto vv = svld1_f32(pg, v.data);
  svst1_f32(pg, r.data, svsqrt_f32_z(pg, vv));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> sqrt(const Vec<double, 2>& v)
{
  Vec<double, 2> r;
  auto pg = Vec<double, 2>::pg();
  auto vv = svld1_f64(pg, v.data);
  svst1_f64(pg, r.data, svsqrt_f64_z(pg, vv));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> min(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  Vec<float, 4> r;
  auto pg = Vec<float, 4>::pg();
  auto va = svld1_f32(pg, a.data);
  auto vb = svld1_f32(pg, b.data);
  svst1_f32(pg, r.data, svmin_f32_z(pg, va, vb));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> min(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  Vec<double, 2> r;
  auto pg = Vec<double, 2>::pg();
  auto va = svld1_f64(pg, a.data);
  auto vb = svld1_f64(pg, b.data);
  svst1_f64(pg, r.data, svmin_f64_z(pg, va, vb));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> max(
    const Vec<float, 4>& a, const Vec<float, 4>& b)
{
  Vec<float, 4> r;
  auto pg = Vec<float, 4>::pg();
  auto va = svld1_f32(pg, a.data);
  auto vb = svld1_f32(pg, b.data);
  svst1_f32(pg, r.data, svmax_f32_z(pg, va, vb));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> max(
    const Vec<double, 2>& a, const Vec<double, 2>& b)
{
  Vec<double, 2> r;
  auto pg = Vec<double, 2>::pg();
  auto va = svld1_f64(pg, a.data);
  auto vb = svld1_f64(pg, b.data);
  svst1_f64(pg, r.data, svmax_f64_z(pg, va, vb));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<float, 4> fmadd(
    const Vec<float, 4>& a, const Vec<float, 4>& b, const Vec<float, 4>& c)
{
  Vec<float, 4> r;
  auto pg = Vec<float, 4>::pg();
  auto va = svld1_f32(pg, a.data);
  auto vb = svld1_f32(pg, b.data);
  auto vc = svld1_f32(pg, c.data);
  svst1_f32(pg, r.data, svmla_f32_z(pg, vc, va, vb));
  return r;
}

template <>
[[nodiscard]] DART_SIMD_INLINE Vec<double, 2> fmadd(
    const Vec<double, 2>& a, const Vec<double, 2>& b, const Vec<double, 2>& c)
{
  Vec<double, 2> r;
  auto pg = Vec<double, 2>::pg();
  auto va = svld1_f64(pg, a.data);
  auto vb = svld1_f64(pg, b.data);
  auto vc = svld1_f64(pg, c.data);
  svst1_f64(pg, r.data, svmla_f64_z(pg, vc, va, vb));
  return r;
}

} // namespace dart::simd

#endif // DART_SIMD_SVE
