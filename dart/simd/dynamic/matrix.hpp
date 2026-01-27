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
#include <dart/simd/dynamic/vector.hpp>
#include <dart/simd/fwd.hpp>
#include <dart/simd/memory.hpp>

#include <Eigen/Core>

namespace dart::simd {

template <typename T>
class DynamicMatrix
{
  static_assert(
      std::is_same_v<T, float> || std::is_same_v<T, double>,
      "DynamicMatrix only supports float or double");

public:
  using scalar_type = T;
  static constexpr std::size_t simd_width = preferred_width_v<T>;

private:
  aligned_vector<T> data_;
  std::size_t rows_{0};
  std::size_t cols_{0};

public:
  DynamicMatrix() = default;

  DynamicMatrix(std::size_t rows, std::size_t cols)
    : data_(rows * cols, T(0)), rows_(rows), cols_(cols)
  {
  }

  DynamicMatrix(std::size_t rows, std::size_t cols, T value)
    : data_(rows * cols, value), rows_(rows), cols_(cols)
  {
  }

  [[nodiscard]] std::size_t rows() const noexcept
  {
    return rows_;
  }

  [[nodiscard]] std::size_t cols() const noexcept
  {
    return cols_;
  }

  [[nodiscard]] T* data() noexcept
  {
    return data_.data();
  }

  [[nodiscard]] const T* data() const noexcept
  {
    return data_.data();
  }

  [[nodiscard]] T& operator()(std::size_t row, std::size_t col) noexcept
  {
    return data_[col * rows_ + row];
  }

  [[nodiscard]] const T& operator()(
      std::size_t row, std::size_t col) const noexcept
  {
    return data_[col * rows_ + row];
  }

  void resize(std::size_t rows, std::size_t cols)
  {
    rows_ = rows;
    cols_ = cols;
    data_.resize(rows * cols);
  }

  void setZero()
  {
    std::fill(data_.begin(), data_.end(), T(0));
  }

  void setIdentity()
  {
    setZero();
    std::size_t min_dim = std::min(rows_, cols_);
    for (std::size_t i = 0; i < min_dim; ++i) {
      (*this)(i, i) = T(1);
    }
  }

  [[nodiscard]] static DynamicMatrix identity(std::size_t n)
  {
    DynamicMatrix m(n, n);
    m.setIdentity();
    return m;
  }

  [[nodiscard]] DynamicVector<T> operator*(const DynamicVector<T>& v) const
  {
    assert(cols_ == v.size() && "Matrix columns must match vector size");
    DynamicVector<T> result(rows_);
    const T* mat = data_.data();
    const T* vec = v.data();
    T* res = result.data();

    for (std::size_t col = 0; col < cols_; ++col) {
      const T* col_data = mat + col * rows_;
      T vec_val = vec[col];
      auto vs = Vec<T, simd_width>::broadcast(vec_val);

      std::size_t row = 0;
      std::size_t simd_end = (rows_ / simd_width) * simd_width;

      DART_SIMD_PRAGMA_UNROLL
      for (; row < simd_end; row += simd_width) {
        auto vr = Vec<T, simd_width>::load(res + row);
        auto vc = Vec<T, simd_width>::loadu(col_data + row);
        (vr + vc * vs).store(res + row);
      }
      for (; row < rows_; ++row) {
        res[row] += col_data[row] * vec_val;
      }
    }
    return result;
  }

  [[nodiscard]] DynamicMatrix operator*(const DynamicMatrix& rhs) const
  {
    assert(
        cols_ == rhs.rows_
        && "Matrix dimensions must match for multiplication");
    DynamicMatrix result(rows_, rhs.cols_);

    for (std::size_t j = 0; j < rhs.cols_; ++j) {
      DynamicVector<T> col_vec(cols_);
      for (std::size_t k = 0; k < cols_; ++k) {
        col_vec[k] = rhs(k, j);
      }
      DynamicVector<T> result_col = (*this) * col_vec;
      for (std::size_t i = 0; i < rows_; ++i) {
        result(i, j) = result_col[i];
      }
    }
    return result;
  }

  DynamicMatrix& operator+=(const DynamicMatrix& rhs)
  {
    assert(
        rows_ == rhs.rows_ && cols_ == rhs.cols_
        && "Matrix dimensions must match");
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    T* a = data_.data();
    const T* b = rhs.data_.data();

    DART_SIMD_PRAGMA_UNROLL
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

  DynamicMatrix& operator-=(const DynamicMatrix& rhs)
  {
    assert(
        rows_ == rhs.rows_ && cols_ == rhs.cols_
        && "Matrix dimensions must match");
    const std::size_t n = data_.size();
    const std::size_t simd_end = (n / simd_width) * simd_width;
    T* a = data_.data();
    const T* b = rhs.data_.data();

    DART_SIMD_PRAGMA_UNROLL
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

  DynamicMatrix& operator*=(T scalar)
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

  [[nodiscard]] DynamicMatrix operator+(const DynamicMatrix& rhs) const
  {
    DynamicMatrix result = *this;
    result += rhs;
    return result;
  }

  [[nodiscard]] DynamicMatrix operator-(const DynamicMatrix& rhs) const
  {
    DynamicMatrix result = *this;
    result -= rhs;
    return result;
  }

  [[nodiscard]] DynamicMatrix operator*(T scalar) const
  {
    DynamicMatrix result = *this;
    result *= scalar;
    return result;
  }

  [[nodiscard]] DynamicMatrix transposed() const
  {
    DynamicMatrix result(cols_, rows_);
    for (std::size_t i = 0; i < rows_; ++i) {
      for (std::size_t j = 0; j < cols_; ++j) {
        result(j, i) = (*this)(i, j);
      }
    }
    return result;
  }

  [[nodiscard]] Eigen::Map<Eigen::MatrixX<T>> as_eigen() noexcept
  {
    return Eigen::Map<Eigen::MatrixX<T>>(
        data_.data(),
        static_cast<Eigen::Index>(rows_),
        static_cast<Eigen::Index>(cols_));
  }

  [[nodiscard]] Eigen::Map<const Eigen::MatrixX<T>> as_eigen() const noexcept
  {
    return Eigen::Map<const Eigen::MatrixX<T>>(
        data_.data(),
        static_cast<Eigen::Index>(rows_),
        static_cast<Eigen::Index>(cols_));
  }

  [[nodiscard]] static DynamicMatrix fromEigen(const Eigen::MatrixX<T>& m)
  {
    DynamicMatrix result(
        static_cast<std::size_t>(m.rows()), static_cast<std::size_t>(m.cols()));
    std::copy(m.data(), m.data() + m.size(), result.data_.data());
    return result;
  }
};

using DynamicMatrixf = DynamicMatrix<float>;
using DynamicMatrixd = DynamicMatrix<double>;

} // namespace dart::simd
