/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 */

#include "dart/math/lcp/pivoting/dantzig/matrix.hpp"
#include "dart/math/lcp/pivoting/dantzig/pivot_matrix.hpp"

#include <gtest/gtest.h>

#include <vector>

#include <cmath>

using namespace dart::math;

//==============================================================================
TEST(PivotMatrix, Construction)
{
  // Square matrix
  PivotMatrixd A(10);
  EXPECT_EQ(A.rows(), 10);
  EXPECT_EQ(A.cols(), 10);
  EXPECT_EQ(A.nskip(), 10);

  // Rectangular matrix
  PivotMatrixd B(5, 10);
  EXPECT_EQ(B.rows(), 5);
  EXPECT_EQ(B.cols(), 10);
  EXPECT_EQ(B.nskip(), 10);

  // With custom nskip
  PivotMatrixd C(5, 8, 12);
  EXPECT_EQ(C.rows(), 5);
  EXPECT_EQ(C.cols(), 8);
  EXPECT_EQ(C.nskip(), 12);
}

//==============================================================================
TEST(PivotMatrix, ElementAccess)
{
  PivotMatrixd A(3, 3);

  // Initialize with known values
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = i * 10 + j;
    }
  }

  // Verify via operator()
  EXPECT_DOUBLE_EQ(A(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(A(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(A(1, 2), 12.0);
  EXPECT_DOUBLE_EQ(A(2, 2), 22.0);

  // Verify via operator[]
  EXPECT_DOUBLE_EQ(A[0][0], 0.0);
  EXPECT_DOUBLE_EQ(A[1][2], 12.0);

  // Modify via operator[]
  A[1][1] = 99.0;
  EXPECT_DOUBLE_EQ(A(1, 1), 99.0);
}

//==============================================================================
TEST(PivotMatrix, RowSwapping)
{
  PivotMatrixd A(3, 4);

  // Initialize: row i contains [i, i+10, i+20, i+30]
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      A(i, j) = i + j * 10;
    }
  }

  // Before swap:
  // Row 0: [0, 10, 20, 30]
  // Row 1: [1, 11, 21, 31]
  // Row 2: [2, 12, 22, 32]

  // Swap rows 0 and 2
  A.swapRows(0, 2);

  // After swap (via operator[]):
  // Row 0 (was row 2): [2, 12, 22, 32]
  // Row 1 (unchanged): [1, 11, 21, 31]
  // Row 2 (was row 0): [0, 10, 20, 30]

  EXPECT_DOUBLE_EQ(A[0][0], 2.0);
  EXPECT_DOUBLE_EQ(A[0][1], 12.0);
  EXPECT_DOUBLE_EQ(A[0][2], 22.0);
  EXPECT_DOUBLE_EQ(A[0][3], 32.0);

  EXPECT_DOUBLE_EQ(A[1][0], 1.0);
  EXPECT_DOUBLE_EQ(A[1][1], 11.0);

  EXPECT_DOUBLE_EQ(A[2][0], 0.0);
  EXPECT_DOUBLE_EQ(A[2][1], 10.0);
  EXPECT_DOUBLE_EQ(A[2][2], 20.0);
  EXPECT_DOUBLE_EQ(A[2][3], 30.0);
}

//==============================================================================
TEST(PivotMatrix, MultipleSwaps)
{
  PivotMatrixd A(5, 3);

  // Initialize: row i contains [i, i, i]
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = static_cast<double>(i);
    }
  }

  // Perform multiple swaps
  A.swapRows(0, 4); // 0↔4: [4,1,2,3,0]
  A.swapRows(1, 3); // 1↔3: [4,3,2,1,0]
  A.swapRows(2, 4); // 2↔4: [4,3,0,1,2]

  EXPECT_DOUBLE_EQ(A[0][0], 4.0);
  EXPECT_DOUBLE_EQ(A[1][0], 3.0);
  EXPECT_DOUBLE_EQ(A[2][0], 0.0);
  EXPECT_DOUBLE_EQ(A[3][0], 1.0);
  EXPECT_DOUBLE_EQ(A[4][0], 2.0);
}

//==============================================================================
TEST(PivotMatrix, ResetPermutation)
{
  PivotMatrixd A(4, 4);

  // Initialize with row indices
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      A(i, j) = static_cast<double>(i);
    }
  }

  // Permute rows
  A.swapRows(0, 3);
  A.swapRows(1, 2);

  // Verify permutation
  EXPECT_DOUBLE_EQ(A[0][0], 3.0);
  EXPECT_DOUBLE_EQ(A[1][0], 2.0);

  // Reset to identity permutation
  A.resetPermutation();

  // Should be back to original order
  EXPECT_DOUBLE_EQ(A[0][0], 0.0);
  EXPECT_DOUBLE_EQ(A[1][0], 1.0);
  EXPECT_DOUBLE_EQ(A[2][0], 2.0);
  EXPECT_DOUBLE_EQ(A[3][0], 3.0);
}

//==============================================================================
TEST(PivotMatrix, RowPointerInterface)
{
  PivotMatrixd A(3, 3);

  // Initialize
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = i * 10 + j;
    }
  }

  // Get row pointers array
  double** rows = A.rowPointers();

  // Access via raw pointers (mimics existing LCP code)
  EXPECT_DOUBLE_EQ(rows[0][0], 0.0);
  EXPECT_DOUBLE_EQ(rows[1][2], 12.0);
  EXPECT_DOUBLE_EQ(rows[2][1], 21.0);

  // Swap via member function
  A.swapRows(0, 2);

  // Raw pointers should reflect swap
  EXPECT_DOUBLE_EQ(rows[0][0], 20.0);
  EXPECT_DOUBLE_EQ(rows[2][0], 0.0);
}

//==============================================================================
TEST(PivotMatrix, EigenMatrixAccess)
{
  PivotMatrixd A(3, 3);

  // Initialize via PivotMatrix interface first
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = (i + 1) * 10 + (j + 1);
    }
  }
  // Matrix is now:
  // [11, 12, 13]
  // [21, 22, 23]
  // [31, 32, 33]

  // Access underlying Eigen matrix
  auto& matrix = A.matrix();

  // Verify via Eigen interface
  EXPECT_DOUBLE_EQ(matrix(0, 0), 11.0);
  EXPECT_DOUBLE_EQ(matrix(1, 1), 22.0);
  EXPECT_DOUBLE_EQ(matrix(2, 2), 33.0);

  // Swap rows in PivotMatrix
  A.swapRows(0, 2);

  // Matrix is now:
  // [31, 32, 33]
  // [21, 22, 23]
  // [11, 12, 13]

  // Access through operator[] should reflect swap
  EXPECT_DOUBLE_EQ(A[0][0], 31.0);
  EXPECT_DOUBLE_EQ(A[0][1], 32.0);
  EXPECT_DOUBLE_EQ(A[0][2], 33.0);

  EXPECT_DOUBLE_EQ(A[2][0], 11.0);
  EXPECT_DOUBLE_EQ(A[2][1], 12.0);
  EXPECT_DOUBLE_EQ(A[2][2], 13.0);

  // But underlying Eigen matrix is unchanged (row data in same memory
  // locations)
  EXPECT_DOUBLE_EQ(matrix(0, 0), 11.0);
  EXPECT_DOUBLE_EQ(matrix(2, 0), 31.0);
}

//==============================================================================
TEST(PivotMatrix, SetZero)
{
  PivotMatrixd A(4, 4);

  // Set some values
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      A(i, j) = static_cast<double>(i + j);
    }
  }

  // Verify non-zero
  EXPECT_NE(A(2, 2), 0.0);

  // Set to zero
  A.setZero();

  // Verify all zeros
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_DOUBLE_EQ(A(i, j), 0.0);
    }
  }
}

//==============================================================================
TEST(PivotMatrix, SetConstant)
{
  PivotMatrixd A(3, 3);

  A.setConstant(42.0);

  // Verify all elements are 42
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(A(i, j), 42.0);
    }
  }
}

//==============================================================================
TEST(PivotMatrix, FloatType)
{
  PivotMatrixf A(2, 2);

  A(0, 0) = 1.5f;
  A(0, 1) = 2.5f;
  A(1, 0) = 3.5f;
  A(1, 1) = 4.5f;

  EXPECT_FLOAT_EQ(A(0, 0), 1.5f);
  EXPECT_FLOAT_EQ(A(1, 1), 4.5f);

  A.swapRows(0, 1);

  EXPECT_FLOAT_EQ(A[0][0], 3.5f);
  EXPECT_FLOAT_EQ(A[0][1], 4.5f);
}

//==============================================================================
TEST(PivotMatrix, MoveSemantics)
{
  PivotMatrixd A(3, 3);

  // Initialize
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A(i, j) = i * 10 + j;
    }
  }

  // Move construct
  PivotMatrixd B(std::move(A));

  EXPECT_EQ(B.rows(), 3);
  EXPECT_EQ(B.cols(), 3);
  EXPECT_DOUBLE_EQ(B(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(B(1, 2), 12.0);

  // Move assign
  PivotMatrixd C(2, 2);
  C = std::move(B);

  EXPECT_EQ(C.rows(), 3);
  EXPECT_EQ(C.cols(), 3);
  EXPECT_DOUBLE_EQ(C(2, 2), 22.0);
}

//==============================================================================
TEST(PivotMatrix, Resize)
{
  PivotMatrixd A(2, 2);

  A(0, 0) = 1.0;
  A(1, 1) = 2.0;

  // Resize to larger
  A.resize(4, 4);

  EXPECT_EQ(A.rows(), 4);
  EXPECT_EQ(A.cols(), 4);

  // Initialize new elements
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      A(i, j) = i + j;
    }
  }

  EXPECT_DOUBLE_EQ(A(3, 3), 6.0);
}

//==============================================================================
TEST(PivotMatrix, ConstructFromEigen)
{
  // Create source Eigen matrix
  Eigen::MatrixXd source(3, 4);
  source << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;

  // Construct PivotMatrix from Eigen matrix
  PivotMatrixd A(source);

  // Verify dimensions
  EXPECT_EQ(A.rows(), 3);
  EXPECT_EQ(A.cols(), 4);

  // Verify data was copied correctly
  EXPECT_DOUBLE_EQ(A(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(A(0, 3), 4.0);
  EXPECT_DOUBLE_EQ(A(1, 1), 6.0);
  EXPECT_DOUBLE_EQ(A(2, 2), 11.0);
  EXPECT_DOUBLE_EQ(A(2, 3), 12.0);

  // Verify row swapping works
  A.swapRows(0, 2);
  EXPECT_DOUBLE_EQ(A[0][0], 9.0);
  EXPECT_DOUBLE_EQ(A[2][0], 1.0);

  // Verify original Eigen matrix is unchanged
  EXPECT_DOUBLE_EQ(source(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(source(2, 0), 9.0);
}

//==============================================================================
// Performance/stress test
TEST(PivotMatrix, ManySwaps)
{
  constexpr int n = 100;
  constexpr int num_swaps = 10000;

  PivotMatrixd A(n, n);

  // Initialize
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      A(i, j) = i * 100 + j;
    }
  }

  // Perform many swaps (should be O(1) each)
  for (int k = 0; k < num_swaps; ++k) {
    int i = k % n;
    int j = (k + 1) % n;
    if (i != j) {
      A.swapRows(i, j);
    }
  }

  // Verify matrix is still valid (values should be permuted but present)
  EXPECT_TRUE(A.rows() == n);
  EXPECT_TRUE(A.cols() == n);

  // At least check some values are in reasonable range
  bool found_reasonable_value = false;
  for (int i = 0; i < n && !found_reasonable_value; ++i) {
    if (A[i][0] >= 0.0 && A[i][0] < n * 100) {
      found_reasonable_value = true;
    }
  }
  EXPECT_TRUE(found_reasonable_value);
}

//==============================================================================
TEST(DantzigMatrixImpl, LDLTAddTLEarlyReturnOnSingleElement)
{
  std::vector<double> L = {1.0};
  std::vector<double> d = {2.0};
  std::vector<double> a = {0.5};

  dLDLTAddTL(L.data(), d.data(), a.data(), 1, 1, nullptr);

  EXPECT_DOUBLE_EQ(L[0], 1.0);
  EXPECT_DOUBLE_EQ(d[0], 2.0);
}

//==============================================================================
TEST(DantzigMatrixImpl, LDLTAddTLAllocatesTempBufferWhenNull)
{
  constexpr int n = 3;
  constexpr int nskip = 3;

  std::vector<double> L = {1.0, 0.0, 0.0, 0.2, 1.0, 0.0, -0.1, 0.3, 1.0};
  std::vector<double> d = {1.0, 0.8, 1.2};
  std::vector<double> a = {0.25, -0.1, 0.15};

  dLDLTAddTL(L.data(), d.data(), a.data(), n, nskip, nullptr);

  for (int i = 0; i < n; ++i) {
    EXPECT_TRUE(std::isfinite(d[i]));
    for (int j = 0; j < n; ++j) {
      EXPECT_TRUE(std::isfinite(L[i * nskip + j]));
    }
  }
}

//==============================================================================
TEST(DantzigMatrixImpl, LDLTRemoveFirstRowUsesProvidedBuffer)
{
  constexpr int n = 3;
  constexpr int nskip = 3;

  std::vector<double> A = {4.0, 1.0, 0.0, 1.0, 3.0, 0.5, 0.0, 0.5, 2.0};
  std::vector<double*> rows(n);
  for (int i = 0; i < n; ++i) {
    rows[i] = A.data() + i * nskip;
  }

  std::vector<double> L = A;
  std::vector<double> d(n, 0.0);
  dFactorLDLT(L.data(), d.data(), n, nskip);

  std::vector<int> p = {0, 1, 2};
  const size_t tmpSize
      = dEstimateLDLTRemoveTmpbufSize<double>(n, nskip) / sizeof(double);
  std::vector<double> tmp(tmpSize, 0.0);

  dLDLTRemove(
      rows.data(), p.data(), L.data(), d.data(), n, n, 0, nskip, tmp.data());

  EXPECT_TRUE(std::isfinite(d[0]));
  EXPECT_TRUE(std::isfinite(L[0]));
}

//==============================================================================
TEST(DantzigMatrixImpl, LDLTRemoveMiddleRowAllocatesTemp)
{
  constexpr int n = 3;
  constexpr int nskip = 3;

  std::vector<double> A = {5.0, 1.5, 0.5, 1.5, 4.0, 0.8, 0.5, 0.8, 3.0};
  std::vector<double*> rows(n);
  for (int i = 0; i < n; ++i) {
    rows[i] = A.data() + i * nskip;
  }

  std::vector<double> L = A;
  std::vector<double> d(n, 0.0);
  dFactorLDLT(L.data(), d.data(), n, nskip);

  std::vector<int> p = {0, 1, 2};
  dLDLTRemove(
      rows.data(), p.data(), L.data(), d.data(), n, n, 1, nskip, nullptr);

  EXPECT_TRUE(std::isfinite(d[0]));
  EXPECT_TRUE(std::isfinite(L[0]));
}

//==============================================================================
TEST(DantzigMatrixImpl, RemoveRowColLastIsNoOp)
{
  constexpr int n = 3;
  constexpr int nskip = 3;

  std::vector<double> A = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  const std::vector<double> original = A;

  dRemoveRowCol(A.data(), n, nskip, n - 1);

  EXPECT_EQ(A, original);
}

//==============================================================================
TEST(DantzigMatrixImpl, FactorLDLTOddSizeTriggersTailCase)
{
  constexpr int n = 3;
  constexpr int nskip = 3;

  std::vector<double> A = {6.0, 1.0, 0.5, 1.0, 4.0, 0.25, 0.5, 0.25, 3.0};
  std::vector<double> d(n, 0.0);

  dFactorLDLT(A.data(), d.data(), n, nskip);

  for (int i = 0; i < n; ++i) {
    EXPECT_TRUE(std::isfinite(d[i]));
    EXPECT_NE(d[i], 0.0);
  }
}
