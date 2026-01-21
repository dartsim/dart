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

#include <Eigen/Core>

#include <array>
#include <type_traits>

namespace dart::simd {

template <typename Derived>
concept EigenDenseBase = std::is_base_of_v<
    Eigen::DenseBase<std::remove_cvref_t<Derived>>,
    std::remove_cvref_t<Derived>>;

template <typename Derived>
concept EigenVectorXpr = EigenDenseBase<Derived>
                         && (std::remove_cvref_t<Derived>::ColsAtCompileTime
                                 == 1
                             || std::remove_cvref_t<Derived>::RowsAtCompileTime
                                    == 1);

template <typename T, int Size>
[[nodiscard]] inline Vec<T, static_cast<std::size_t>(Size)> to_vec(
    const Eigen::Matrix<T, Size, 1>& v) noexcept
{
  static_assert(Size >= 1 && Size <= 16, "Unsupported Eigen vector size");
  Vec<T, static_cast<std::size_t>(Size)> result;
  for (int i = 0; i < Size; ++i) {
    result[i] = v[i];
  }
  return result;
}

template <typename T, int Size>
[[nodiscard]] inline Vec<T, static_cast<std::size_t>(Size)> to_vec(
    const Eigen::Matrix<T, 1, Size>& v) noexcept
{
  static_assert(Size >= 1 && Size <= 16, "Unsupported Eigen vector size");
  Vec<T, static_cast<std::size_t>(Size)> result;
  for (int i = 0; i < Size; ++i) {
    result[i] = v[i];
  }
  return result;
}

template <typename T>
[[nodiscard]] inline Vec<T, 4> to_vec3_padded(
    const Eigen::Matrix<T, 3, 1>& v, T pad = T{0}) noexcept
{
  return Vec<T, 4>(std::array<T, 4>{v[0], v[1], v[2], pad});
}

template <typename T, std::size_t W>
[[nodiscard]] inline auto to_eigen(const Vec<T, W>& v) noexcept
{
  Eigen::Matrix<T, static_cast<int>(W), 1> result;
  for (std::size_t i = 0; i < W; ++i) {
    result[static_cast<int>(i)] = v[i];
  }
  return result;
}

template <typename T>
[[nodiscard]] inline Eigen::Matrix<T, 3, 1> to_eigen3(
    const Vec<T, 4>& v) noexcept
{
  return Eigen::Matrix<T, 3, 1>{v[0], v[1], v[2]};
}

template <typename T, std::size_t W>
[[nodiscard]] inline Vec<T, W> load_eigen(
    const Eigen::Matrix<T, static_cast<int>(W), 1>* ptr) noexcept
{
  return Vec<T, W>::load(ptr->data());
}

template <typename T, std::size_t W>
inline void store_eigen(
    Eigen::Matrix<T, static_cast<int>(W), 1>* ptr, const Vec<T, W>& v) noexcept
{
  v.store(ptr->data());
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

  [[nodiscard]] std::array<Eigen::Matrix<T, 3, 1>, N> to_aos() const noexcept
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
    x[i] = v[0];
    y[i] = v[1];
    z[i] = v[2];
  }
};

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA3<T, N> transpose_aos_to_soa(
    const std::array<Eigen::Matrix<T, 3, 1>, N>& aos) noexcept
{
  return EigenSoA3<T, N>(aos);
}

template <typename T, std::size_t N>
[[nodiscard]] inline std::array<Eigen::Matrix<T, 3, 1>, N> transpose_soa_to_aos(
    const EigenSoA3<T, N>& soa) noexcept
{
  return soa.to_aos();
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

} // namespace dart::simd
