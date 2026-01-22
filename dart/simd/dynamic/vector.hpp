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
#include <dart/simd/fwd.hpp>
#include <dart/simd/memory.hpp>

#include <Eigen/Core>

#include <initializer_list>
#include <numeric>

#include <cmath>

namespace dart::simd {

template <typename T>
class DynamicVector
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "DynamicVector only supports float or double");

public:
  using scalar_type = T;
  static constexpr std::size_t simd_width = preferred_width_v<T>;

private:
  aligned_vector<T> data_;

public:
  DynamicVector() = default;

  explicit DynamicVector(std::size_t size) : data_(size, T(0)) {}

  DynamicVector(std::size_t size, T value) : data_(size, value) {}

  DynamicVector(std::initializer_list<T> init) : data_(init) {}

  [[nodiscard]] std::size_t size() const noexcept
  {
    return data_.size();
  }

  [[nodiscard]] T* data() noexcept
  {
    return data_.data();
  }

  [[nodiscard]] const T* data() const noexcept
  {
    return data_.data();
  }

  [[nodiscard]] T& operator[](std::size_t i) noexcept
  {
    return data_[i];
  }

  [[nodiscard]] const T& operator[](std::size_t i) const noexcept
  {
    return data_[i];
  }

  void resize(std::size_t new_size)
  {
    data_.resize(new_size);
  }

  void resize(std::size_t new_size, T value)
  {
    data_.resize(new_size, value);
  }

  DynamicVector& operator+=(const DynamicVector& rhs)
  {
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    T* a = data_.data();
    const T* b = rhs.data_.data();

#pragma GCC unroll 4
    for (std::size_t i = 0; i < simd_end; i += simd_width) {
      auto va = Vec<T, simd_width>::load(a + i);
      auto vb = Vec<T, simd_width>::load(b + i);
      (va + vb).store(a + i);
    }
    for (std::size_t i = simd_end; i < n; ++i) {
      a[i] += b[i];
    }
    return *this;
  }

  DynamicVector& operator-=(const DynamicVector& rhs)
  {
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    T* a = data_.data();
    const T* b = rhs.data_.data();

#pragma GCC unroll 4
    for (std::size_t i = 0; i < simd_end; i += simd_width) {
      auto va = Vec<T, simd_width>::load(a + i);
      auto vb = Vec<T, simd_width>::load(b + i);
      (va - vb).store(a + i);
    }
    for (std::size_t i = simd_end; i < n; ++i) {
      a[i] -= b[i];
    }
    return *this;
  }

  DynamicVector& operator*=(T scalar)
  {
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    T* a = data_.data();
    auto vs = Vec<T, simd_width>::broadcast(scalar);

    for (std::size_t i = 0; i < simd_end; i += simd_width) {
      auto va = Vec<T, simd_width>::load(a + i);
      (va * vs).store(a + i);
    }
    for (std::size_t i = simd_end; i < n; ++i) {
      a[i] *= scalar;
    }
    return *this;
  }

  [[nodiscard]] DynamicVector operator+(const DynamicVector& rhs) const
  {
    DynamicVector result = *this;
    result += rhs;
    return result;
  }

  [[nodiscard]] DynamicVector operator-(const DynamicVector& rhs) const
  {
    DynamicVector result = *this;
    result -= rhs;
    return result;
  }

  [[nodiscard]] DynamicVector operator*(T scalar) const
  {
    DynamicVector result = *this;
    result *= scalar;
    return result;
  }

  [[nodiscard]] friend DynamicVector operator*(T scalar, const DynamicVector& v)
  {
    return v * scalar;
  }

  [[nodiscard]] T dot(const DynamicVector& rhs) const
  {
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    const T* a = data_.data();
    const T* b = rhs.data_.data();

    Vec<T, simd_width> acc = Vec<T, simd_width>::zero();
#pragma GCC unroll 4
    for (std::size_t i = 0; i < simd_end; i += simd_width) {
      auto va = Vec<T, simd_width>::load(a + i);
      auto vb = Vec<T, simd_width>::load(b + i);
      acc = acc + va * vb;
    }

    T result = hsum(acc);
    for (std::size_t i = simd_end; i < n; ++i) {
      result += a[i] * b[i];
    }
    return result;
  }

  [[nodiscard]] T sum() const
  {
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    const T* a = data_.data();

    Vec<T, simd_width> acc = Vec<T, simd_width>::zero();
#pragma GCC unroll 4
    for (std::size_t i = 0; i < simd_end; i += simd_width) {
      acc = acc + Vec<T, simd_width>::load(a + i);
    }

    T result = hsum(acc);
    for (std::size_t i = simd_end; i < n; ++i) {
      result += a[i];
    }
    return result;
  }

  [[nodiscard]] T squaredNorm() const
  {
    return dot(*this);
  }

  [[nodiscard]] T norm() const
  {
    return std::sqrt(squaredNorm());
  }

  void normalize()
  {
    T len = norm();
    if (len > T(0)) {
      *this *= (T(1) / len);
    }
  }

  [[nodiscard]] DynamicVector normalized() const
  {
    DynamicVector result = *this;
    result.normalize();
    return result;
  }

  [[nodiscard]] Eigen::Map<Eigen::VectorX<T>> as_eigen() noexcept
  {
    return Eigen::Map<Eigen::VectorX<T>>(
        data_.data(), static_cast<Eigen::Index>(data_.size()));
  }

  [[nodiscard]] Eigen::Map<const Eigen::VectorX<T>> as_eigen() const noexcept
  {
    return Eigen::Map<const Eigen::VectorX<T>>(
        data_.data(), static_cast<Eigen::Index>(data_.size()));
  }

  [[nodiscard]] static DynamicVector fromEigen(const Eigen::VectorX<T>& v)
  {
    DynamicVector result(static_cast<std::size_t>(v.size()));
    std::copy(v.data(), v.data() + v.size(), result.data_.data());
    return result;
  }
};

using DynamicVectorf = DynamicVector<float>;
using DynamicVectord = DynamicVector<double>;

} // namespace dart::simd
