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

#include <dart/simd/fwd.hpp>
#include <dart/simd/geometry/matrix4x4.hpp>

#include <Eigen/Core>

#include <array>
#include <type_traits>

namespace dart::simd {

template <typename Derived>
concept EigenDenseBase = std::is_base_of_v<
    Eigen::DenseBase<std::remove_cvref_t<Derived>>,
    std::remove_cvref_t<Derived>>;

template <typename Derived>
concept EigenVectorXpr
    = EigenDenseBase<Derived>
      && (std::remove_cvref_t<Derived>::ColsAtCompileTime == 1
          || std::remove_cvref_t<Derived>::RowsAtCompileTime == 1);

template <typename T, int Size>
[[nodiscard]] inline Vec<T, static_cast<std::size_t>(Size)> toVec(
    const Eigen::Matrix<T, Size, 1>& v) noexcept
{
  static_assert(Size >= 1 && Size <= 16, "Unsupported Eigen vector size");
  return Vec<T, static_cast<std::size_t>(Size)>::loadu(v.data());
}

template <typename T, int Size>
[[nodiscard]] inline Vec<T, static_cast<std::size_t>(Size)> toVec(
    const Eigen::Matrix<T, 1, Size>& v) noexcept
{
  static_assert(Size >= 1 && Size <= 16, "Unsupported Eigen vector size");
  return Vec<T, static_cast<std::size_t>(Size)>::loadu(v.data());
}

template <typename T>
[[nodiscard]] inline Vec<T, 4> toVec3Padded(
    const Eigen::Matrix<T, 3, 1>& v, T pad = T{0}) noexcept
{
  return Vec<T, 4>::set(v[0], v[1], v[2], pad);
}

template <typename T, std::size_t W>
[[nodiscard]] inline auto toEigen(const Vec<T, W>& v) noexcept
{
  Eigen::Matrix<T, static_cast<int>(W), 1> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[static_cast<int>(i)] = v[i];
  }
  return result;
}

template <typename T>
[[nodiscard]] inline Eigen::Matrix<T, 3, 1> toEigen3(
    const Vec<T, 4>& v) noexcept
{
  return Eigen::Matrix<T, 3, 1>{v[0], v[1], v[2]};
}

template <typename T, std::size_t W>
[[nodiscard]] inline Vec<T, W> load_eigen(
    const Eigen::Matrix<T, static_cast<int>(W), 1>* ptr) noexcept
{
  return Vec<T, W>::loadu(ptr->data());
}

template <typename T, std::size_t W>
inline void store_eigen(
    Eigen::Matrix<T, static_cast<int>(W), 1>* ptr, const Vec<T, W>& v) noexcept
{
  v.storeu(ptr->data());
}

template <typename T, std::size_t N>
struct EigenSoA3
{
  static_assert(N >= 1 && N <= 16, "N must be between 1 and 16");

  Vec<T, N> x;
  Vec<T, N> y;
  Vec<T, N> z;

  EigenSoA3() = default;

  explicit EigenSoA3(const std::array<Eigen::Matrix<T, 3, 1>, N>& aos) noexcept
  {
    alignas(64) T xs[N];
    alignas(64) T ys[N];
    alignas(64) T zs[N];
    for (std::size_t i = 0; i < N; ++i) {
      xs[i] = aos[i][0];
      ys[i] = aos[i][1];
      zs[i] = aos[i][2];
    }
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
  }

  [[nodiscard]] std::array<Eigen::Matrix<T, 3, 1>, N> toAos() const noexcept
  {
    alignas(64) T xs[N];
    alignas(64) T ys[N];
    alignas(64) T zs[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);

    std::array<Eigen::Matrix<T, 3, 1>, N> result;
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = Eigen::Matrix<T, 3, 1>{xs[i], ys[i], zs[i]};
    }
    return result;
  }

  [[nodiscard]] Eigen::Matrix<T, 3, 1> get(std::size_t i) const noexcept
  {
    return Eigen::Matrix<T, 3, 1>{x[i], y[i], z[i]};
  }

  void set(std::size_t i, const Eigen::Matrix<T, 3, 1>& v) noexcept
  {
    alignas(64) T xs[N], ys[N], zs[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);
    xs[i] = v[0];
    ys[i] = v[1];
    zs[i] = v[2];
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
  }
};

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA3<T, N> transposeAosToSoa(
    const std::array<Eigen::Matrix<T, 3, 1>, N>& aos) noexcept
{
  return EigenSoA3<T, N>(aos);
}

template <typename T, std::size_t N>
[[nodiscard]] inline std::array<Eigen::Matrix<T, 3, 1>, N> transposeSoaToAos(
    const EigenSoA3<T, N>& soa) noexcept
{
  return soa.toAos();
}

template <typename T>
[[nodiscard]] inline Vec<T, 4> dot3(
    const EigenSoA3<T, 4>& a, const EigenSoA3<T, 4>& b) noexcept
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename T>
[[nodiscard]] inline EigenSoA3<T, 4> cross3(
    const EigenSoA3<T, 4>& a, const EigenSoA3<T, 4>& b) noexcept
{
  EigenSoA3<T, 4> result;
  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;
  return result;
}

using EigenSoA3f4 = EigenSoA3<float, 4>;
using EigenSoA3d4 = EigenSoA3<double, 4>;
using EigenSoA3f8 = EigenSoA3<float, 8>;
using EigenSoA3d8 = EigenSoA3<double, 8>;

template <typename T, std::size_t N>
struct EigenSoA4
{
  static_assert(N >= 1 && N <= 16, "N must be between 1 and 16");

  Vec<T, N> x;
  Vec<T, N> y;
  Vec<T, N> z;
  Vec<T, N> w;

  EigenSoA4() = default;

  explicit EigenSoA4(const std::array<Eigen::Matrix<T, 4, 1>, N>& aos) noexcept
  {
    alignas(64) T xs[N];
    alignas(64) T ys[N];
    alignas(64) T zs[N];
    alignas(64) T ws[N];
    for (std::size_t i = 0; i < N; ++i) {
      xs[i] = aos[i][0];
      ys[i] = aos[i][1];
      zs[i] = aos[i][2];
      ws[i] = aos[i][3];
    }
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
    w = Vec<T, N>::load(ws);
  }

  [[nodiscard]] std::array<Eigen::Matrix<T, 4, 1>, N> toAos() const noexcept
  {
    alignas(64) T xs[N];
    alignas(64) T ys[N];
    alignas(64) T zs[N];
    alignas(64) T ws[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);
    w.store(ws);

    std::array<Eigen::Matrix<T, 4, 1>, N> result;
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = Eigen::Matrix<T, 4, 1>{xs[i], ys[i], zs[i], ws[i]};
    }
    return result;
  }

  [[nodiscard]] Eigen::Matrix<T, 4, 1> get(std::size_t i) const noexcept
  {
    return Eigen::Matrix<T, 4, 1>{x[i], y[i], z[i], w[i]};
  }

  void set(std::size_t i, const Eigen::Matrix<T, 4, 1>& v) noexcept
  {
    alignas(64) T xs[N], ys[N], zs[N], ws[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);
    w.store(ws);
    xs[i] = v[0];
    ys[i] = v[1];
    zs[i] = v[2];
    ws[i] = v[3];
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
    w = Vec<T, N>::load(ws);
  }
};

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA4<T, N> transposeAosToSoa(
    const std::array<Eigen::Matrix<T, 4, 1>, N>& aos) noexcept
{
  return EigenSoA4<T, N>(aos);
}

template <typename T, std::size_t N>
[[nodiscard]] inline std::array<Eigen::Matrix<T, 4, 1>, N> transposeSoaToAos(
    const EigenSoA4<T, N>& soa) noexcept
{
  return soa.toAos();
}

template <typename T>
[[nodiscard]] inline Vec<T, 4> dot4(
    const EigenSoA4<T, 4>& a, const EigenSoA4<T, 4>& b) noexcept
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

using EigenSoA4f4 = EigenSoA4<float, 4>;
using EigenSoA4d4 = EigenSoA4<double, 4>;
using EigenSoA4f8 = EigenSoA4<float, 8>;
using EigenSoA4d8 = EigenSoA4<double, 8>;

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA3<T, N> transformPoints(
    const Matrix4x4<T>& m, const EigenSoA3<T, N>& points) noexcept
{
  Vec<T, N> m00 = Vec<T, N>::broadcast(m.col0[0]);
  Vec<T, N> m01 = Vec<T, N>::broadcast(m.col1[0]);
  Vec<T, N> m02 = Vec<T, N>::broadcast(m.col2[0]);
  Vec<T, N> m03 = Vec<T, N>::broadcast(m.col3[0]);
  Vec<T, N> m10 = Vec<T, N>::broadcast(m.col0[1]);
  Vec<T, N> m11 = Vec<T, N>::broadcast(m.col1[1]);
  Vec<T, N> m12 = Vec<T, N>::broadcast(m.col2[1]);
  Vec<T, N> m13 = Vec<T, N>::broadcast(m.col3[1]);
  Vec<T, N> m20 = Vec<T, N>::broadcast(m.col0[2]);
  Vec<T, N> m21 = Vec<T, N>::broadcast(m.col1[2]);
  Vec<T, N> m22 = Vec<T, N>::broadcast(m.col2[2]);
  Vec<T, N> m23 = Vec<T, N>::broadcast(m.col3[2]);

  EigenSoA3<T, N> result;
  result.x = m00 * points.x + m01 * points.y + m02 * points.z + m03;
  result.y = m10 * points.x + m11 * points.y + m12 * points.z + m13;
  result.z = m20 * points.x + m21 * points.y + m22 * points.z + m23;
  return result;
}

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA3<T, N> transformVectors(
    const Matrix4x4<T>& m, const EigenSoA3<T, N>& vectors) noexcept
{
  Vec<T, N> m00 = Vec<T, N>::broadcast(m.col0[0]);
  Vec<T, N> m01 = Vec<T, N>::broadcast(m.col1[0]);
  Vec<T, N> m02 = Vec<T, N>::broadcast(m.col2[0]);
  Vec<T, N> m10 = Vec<T, N>::broadcast(m.col0[1]);
  Vec<T, N> m11 = Vec<T, N>::broadcast(m.col1[1]);
  Vec<T, N> m12 = Vec<T, N>::broadcast(m.col2[1]);
  Vec<T, N> m20 = Vec<T, N>::broadcast(m.col0[2]);
  Vec<T, N> m21 = Vec<T, N>::broadcast(m.col1[2]);
  Vec<T, N> m22 = Vec<T, N>::broadcast(m.col2[2]);

  EigenSoA3<T, N> result;
  result.x = m00 * vectors.x + m01 * vectors.y + m02 * vectors.z;
  result.y = m10 * vectors.x + m11 * vectors.y + m12 * vectors.z;
  result.z = m20 * vectors.x + m21 * vectors.y + m22 * vectors.z;
  return result;
}

// QuaternionSoA: [x, y, z, w] layout matching Quaternion<T>
template <typename T, std::size_t N>
struct QuaternionSoA
{
  static_assert(N >= 1 && N <= 16, "N must be between 1 and 16");

  Vec<T, N> x;
  Vec<T, N> y;
  Vec<T, N> z;
  Vec<T, N> w;

  QuaternionSoA() = default;

  explicit QuaternionSoA(
      const std::array<Eigen::Quaternion<T>, N>& aos) noexcept
  {
    alignas(64) T xs[N];
    alignas(64) T ys[N];
    alignas(64) T zs[N];
    alignas(64) T ws[N];
    for (std::size_t i = 0; i < N; ++i) {
      xs[i] = aos[i].x();
      ys[i] = aos[i].y();
      zs[i] = aos[i].z();
      ws[i] = aos[i].w();
    }
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
    w = Vec<T, N>::load(ws);
  }

  [[nodiscard]] std::array<Eigen::Quaternion<T>, N> toAos() const noexcept
  {
    alignas(64) T xs[N];
    alignas(64) T ys[N];
    alignas(64) T zs[N];
    alignas(64) T ws[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);
    w.store(ws);

    std::array<Eigen::Quaternion<T>, N> result;
    for (std::size_t i = 0; i < N; ++i) {
      result[i] = Eigen::Quaternion<T>(ws[i], xs[i], ys[i], zs[i]);
    }
    return result;
  }

  [[nodiscard]] Eigen::Quaternion<T> get(std::size_t i) const noexcept
  {
    return Eigen::Quaternion<T>(w[i], x[i], y[i], z[i]);
  }

  void set(std::size_t i, const Eigen::Quaternion<T>& q) noexcept
  {
    alignas(64) T xs[N], ys[N], zs[N], ws[N];
    x.store(xs);
    y.store(ys);
    z.store(zs);
    w.store(ws);
    xs[i] = q.x();
    ys[i] = q.y();
    zs[i] = q.z();
    ws[i] = q.w();
    x = Vec<T, N>::load(xs);
    y = Vec<T, N>::load(ys);
    z = Vec<T, N>::load(zs);
    w = Vec<T, N>::load(ws);
  }
};

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA3<T, N> rotateVectors(
    const QuaternionSoA<T, N>& q, const EigenSoA3<T, N>& v) noexcept
{
  Vec<T, N> two = Vec<T, N>::broadcast(T(2));
  Vec<T, N> tx = two * (q.y * v.z - q.z * v.y);
  Vec<T, N> ty = two * (q.z * v.x - q.x * v.z);
  Vec<T, N> tz = two * (q.x * v.y - q.y * v.x);

  EigenSoA3<T, N> result;
  result.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
  result.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
  result.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
  return result;
}

using QuaternionSoAf4 = QuaternionSoA<float, 4>;
using QuaternionSoAd4 = QuaternionSoA<double, 4>;
using QuaternionSoAf8 = QuaternionSoA<float, 8>;
using QuaternionSoAd8 = QuaternionSoA<double, 8>;

} // namespace dart::simd
