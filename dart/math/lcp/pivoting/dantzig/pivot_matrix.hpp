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

#include "dart/common/macros.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"

#include <Eigen/Core>

#include <vector>

namespace dart::math {

/// @brief Hybrid pivot matrix: Eigen storage + O(1) row pointer swapping
///
/// This class combines the benefits of Eigen's optimized storage and SIMD
/// operations with the O(1) row swapping optimization critical for LCP solver
/// performance. The name "PivotMatrix" reflects its use in pivoting operations
/// common in linear algebra algorithms like LCP solvers.
///
/// **Design Rationale**:
/// - Eigen storage: Aligned memory, SIMD-ready, modern C++ interface
/// - Pointer swapping: O(1) row pivots vs O(n) data copying
///
/// **Performance**: Benchmarked 2-3.8x faster than original pointer-only
/// approach
/// (see: tests/benchmark/lcpsolver/bm_row_swapping.cpp)
///
/// **Usage**:
/// ```cpp
/// PivotMatrix A(100, 100);
/// A.swapRows(5, 10);        // O(1) pivot - just swap pointers
/// double val = A(5, 3);     // Access via row pointers
/// auto& matrix = A.matrix(); // Access underlying Eigen matrix
/// ```
///
/// @tparam Scalar Scalar type (float or double)
template <typename Scalar>
class PivotMatrix
{
public:
  // Use row-major storage for efficient row-wise access via pointers
  using MatrixType
      = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using VectorType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  /// Construct a pivot matrix with given dimensions
  /// @param rows Number of rows
  /// @param cols Number of columns
  /// @param nskip Leading dimension (for compatibility with existing code)
  PivotMatrix(int rows, int cols, int nskip = -1)
    : rows_(rows),
      cols_(cols),
      nskip_(nskip < 0 ? cols : nskip),
      data_(rows, cols)
  {
    initializeRowPointers();
  }

  /// Construct a square matrix
  /// @param size Dimension (size × size)
  explicit PivotMatrix(int size) : PivotMatrix(size, size, size)
  {
    // Empty
  }

  /// Construct from Eigen matrix
  ///
  /// Copies data from the Eigen matrix into internal storage and initializes
  /// row pointers for O(1) row swapping.
  ///
  /// @param source Eigen matrix to copy from
  /// @param nskip Leading dimension (default: number of columns)
  template <typename Derived>
  explicit PivotMatrix(const Eigen::MatrixBase<Derived>& source, int nskip = -1)
    : rows_(source.rows()),
      cols_(source.cols()),
      nskip_(nskip < 0 ? source.cols() : nskip),
      data_(source.rows(), source.cols())
  {
    data_ = source;
    initializeRowPointers();
  }

  /// Construct from raw pointer array (OPTIMIZED - single copy)
  ///
  /// Copies data directly from a raw pointer array into internal storage.
  /// This avoids the double-copy overhead of going through an Eigen
  /// intermediate.
  ///
  /// @param rows Number of rows
  /// @param cols Number of columns
  /// @param data Pointer to source data (row-major with nskip leading
  /// dimension)
  /// @param nskip Leading dimension of source data
  PivotMatrix(int rows, int cols, const Scalar* data, int nskip)
    : rows_(rows), cols_(cols), nskip_(nskip), data_(rows, cols)
  {
    // Copy data directly from raw pointer (row-major)
    for (int i = 0; i < rows_; ++i) {
      for (int j = 0; j < cols_; ++j) {
        data_(i, j) = data[i * nskip + j];
      }
    }
    initializeRowPointers();
  }

  /// Destructor
  ~PivotMatrix() = default;

  // No copy constructor (expensive operation, use explicit copy if needed)
  PivotMatrix(const PivotMatrix&) = delete;
  PivotMatrix& operator=(const PivotMatrix&) = delete;

  // Move constructor and assignment
  PivotMatrix(PivotMatrix&& other) noexcept
    : rows_(other.rows_),
      cols_(other.cols_),
      nskip_(other.nskip_),
      data_(std::move(other.data_)),
      row_ptrs_(std::move(other.row_ptrs_))
  {
  }

  PivotMatrix& operator=(PivotMatrix&& other) noexcept
  {
    if (this != &other) {
      rows_ = other.rows_;
      cols_ = other.cols_;
      nskip_ = other.nskip_;
      data_ = std::move(other.data_);
      row_ptrs_ = std::move(other.row_ptrs_);
    }
    return *this;
  }

  /// Swap rows i and j (O(1) operation)
  /// @param i First row index
  /// @param j Second row index
  void swapRows(int i, int j)
  {
    DART_ASSERT(i >= 0 && i < rows_ && j >= 0 && j < rows_);
    std::swap(row_ptrs_[i], row_ptrs_[j]);
  }

  /// Access row i as a pointer (for compatibility with existing LCP code)
  /// @param i Row index
  /// @return Pointer to row data
  Scalar* operator[](int i)
  {
    DART_ASSERT(i >= 0 && i < rows_);
    return row_ptrs_[i];
  }

  /// Access row i as a pointer (const version)
  /// @param i Row index
  /// @return Const pointer to row data
  const Scalar* operator[](int i) const
  {
    DART_ASSERT(i >= 0 && i < rows_);
    return row_ptrs_[i];
  }

  /// Access element (i, j) through permuted row pointers
  /// @param i Row index
  /// @param j Column index
  /// @return Reference to element
  Scalar& operator()(int i, int j)
  {
    DART_ASSERT(i >= 0 && i < rows_ && j >= 0 && j < cols_);
    return row_ptrs_[i][j];
  }

  /// Access element (i, j) through permuted row pointers (const version)
  /// @param i Row index
  /// @param j Column index
  /// @return Const reference to element
  const Scalar& operator()(int i, int j) const
  {
    DART_ASSERT(i >= 0 && i < rows_ && j >= 0 && j < cols_);
    return row_ptrs_[i][j];
  }

  /// Get the underlying Eigen matrix (bypasses row permutation)
  /// @warning Directly modifying this matrix may invalidate row pointers
  /// @return Reference to underlying Eigen matrix
  MatrixType& matrix()
  {
    return data_;
  }

  /// Get the underlying Eigen matrix (const version)
  /// @return Const reference to underlying Eigen matrix
  const MatrixType& matrix() const
  {
    return data_;
  }

  /// Get raw data pointer (for interfacing with C-style APIs)
  /// @warning Bypasses row permutation - use with caution
  /// @return Pointer to raw data
  Scalar* data()
  {
    return data_.data();
  }

  /// Get raw data pointer (const version)
  /// @return Const pointer to raw data
  const Scalar* data() const
  {
    return data_.data();
  }

  /// Get array of row pointers (for interfacing with existing LCP code)
  /// @return Pointer to array of row pointers
  Scalar** rowPointers()
  {
    return row_ptrs_.data();
  }

  /// Get array of row pointers (const version)
  /// @return Const pointer to array of row pointers
  Scalar* const* rowPointers() const
  {
    return row_ptrs_.data();
  }

  /// Get number of rows
  int rows() const
  {
    return rows_;
  }

  /// Get number of columns
  int cols() const
  {
    return cols_;
  }

  /// Get leading dimension (nskip)
  int nskip() const
  {
    return nskip_;
  }

  /// Reset to identity permutation
  /// After calling this, row pointers point to rows 0, 1, 2, ... in order
  void resetPermutation()
  {
    initializeRowPointers();
  }

  /// Set all elements to zero
  void setZero()
  {
    data_.setZero();
  }

  /// Set all elements to a constant value
  /// @param value Value to set
  void setConstant(Scalar value)
  {
    data_.setConstant(value);
  }

  /// Resize the matrix (invalidates row pointers)
  /// @param rows New number of rows
  /// @param cols New number of columns
  void resize(int rows, int cols)
  {
    rows_ = rows;
    cols_ = cols;
    nskip_ = cols;
    data_.resize(rows, cols);
    row_ptrs_.resize(rows);
    initializeRowPointers();
  }

private:
  /// Initialize row pointers to point to matrix rows
  void initializeRowPointers()
  {
    row_ptrs_.resize(rows_);
    // With row-major storage, row i starts at data_.data() + i * cols_
    for (int i = 0; i < rows_; ++i) {
      row_ptrs_[i] = data_.data() + i * cols_;
    }
  }

  int rows_;                      ///< Number of rows
  int cols_;                      ///< Number of columns
  int nskip_;                     ///< Leading dimension (for compatibility)
  MatrixType data_;               ///< Underlying Eigen matrix storage
  std::vector<Scalar*> row_ptrs_; ///< Row pointers for O(1) swapping
};

// Convenience typedefs
using PivotMatrixd = PivotMatrix<double>;
using PivotMatrixf = PivotMatrix<float>;

} // namespace dart::math
