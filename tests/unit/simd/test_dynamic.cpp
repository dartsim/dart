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

#include <dart/simd/simd.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart::simd;

// =============================================================================
// DynamicVector Tests
// =============================================================================

class DynamicVectorTest : public ::testing::Test
{
};

TEST_F(DynamicVectorTest, Construction)
{
  DynamicVectorf v1(10);
  EXPECT_EQ(v1.size(), 10u);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_FLOAT_EQ(v1[i], 0.0f);
  }

  DynamicVectorf v2(5, 3.14f);
  EXPECT_EQ(v2.size(), 5u);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_FLOAT_EQ(v2[i], 3.14f);
  }

  DynamicVectorf v3{1.0f, 2.0f, 3.0f, 4.0f};
  EXPECT_EQ(v3.size(), 4u);
  EXPECT_FLOAT_EQ(v3[0], 1.0f);
  EXPECT_FLOAT_EQ(v3[1], 2.0f);
  EXPECT_FLOAT_EQ(v3[2], 3.0f);
  EXPECT_FLOAT_EQ(v3[3], 4.0f);
}

TEST_F(DynamicVectorTest, Arithmetic)
{
  DynamicVectorf a{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f};
  DynamicVectorf b{8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f};

  DynamicVectorf sum = a + b;
  for (std::size_t i = 0; i < 8; ++i) {
    EXPECT_FLOAT_EQ(sum[i], 9.0f);
  }

  DynamicVectorf diff = a - b;
  EXPECT_FLOAT_EQ(diff[0], -7.0f);
  EXPECT_FLOAT_EQ(diff[7], 7.0f);

  DynamicVectorf scaled = a * 2.0f;
  EXPECT_FLOAT_EQ(scaled[0], 2.0f);
  EXPECT_FLOAT_EQ(scaled[7], 16.0f);

  DynamicVectorf scaled2 = 3.0f * a;
  EXPECT_FLOAT_EQ(scaled2[0], 3.0f);
  EXPECT_FLOAT_EQ(scaled2[7], 24.0f);
}

TEST_F(DynamicVectorTest, CompoundAssignment)
{
  DynamicVectorf a{1.0f, 2.0f, 3.0f, 4.0f};
  DynamicVectorf b{4.0f, 3.0f, 2.0f, 1.0f};

  a += b;
  EXPECT_FLOAT_EQ(a[0], 5.0f);
  EXPECT_FLOAT_EQ(a[3], 5.0f);

  a -= b;
  EXPECT_FLOAT_EQ(a[0], 1.0f);
  EXPECT_FLOAT_EQ(a[3], 4.0f);

  a *= 2.0f;
  EXPECT_FLOAT_EQ(a[0], 2.0f);
  EXPECT_FLOAT_EQ(a[3], 8.0f);
}

TEST_F(DynamicVectorTest, DotProduct)
{
  DynamicVectorf a{1.0f, 2.0f, 3.0f, 4.0f};
  DynamicVectorf b{4.0f, 3.0f, 2.0f, 1.0f};

  float dot_result = a.dot(b);
  // 1*4 + 2*3 + 3*2 + 4*1 = 4 + 6 + 6 + 4 = 20
  EXPECT_FLOAT_EQ(dot_result, 20.0f);
}

TEST_F(DynamicVectorTest, Sum)
{
  DynamicVectorf v{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  EXPECT_FLOAT_EQ(v.sum(), 15.0f);
}

TEST_F(DynamicVectorTest, Norm)
{
  DynamicVectorf v{3.0f, 4.0f};
  EXPECT_FLOAT_EQ(v.squaredNorm(), 25.0f);
  EXPECT_FLOAT_EQ(v.norm(), 5.0f);
}

TEST_F(DynamicVectorTest, Normalize)
{
  DynamicVectorf v{3.0f, 4.0f};
  v.normalize();
  EXPECT_NEAR(v.norm(), 1.0f, 1e-6f);
  EXPECT_FLOAT_EQ(v[0], 0.6f);
  EXPECT_FLOAT_EQ(v[1], 0.8f);

  DynamicVectorf v2{3.0f, 4.0f};
  DynamicVectorf normalized = v2.normalized();
  EXPECT_NEAR(normalized.norm(), 1.0f, 1e-6f);
  // Original unchanged
  EXPECT_FLOAT_EQ(v2.norm(), 5.0f);
}

TEST_F(DynamicVectorTest, LargeSizeSimd)
{
  constexpr std::size_t N = 1000;
  DynamicVectorf a(N, 1.0f);
  DynamicVectorf b(N, 2.0f);

  DynamicVectorf sum = a + b;
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(sum[i], 3.0f);
  }

  float dot_result = a.dot(b);
  EXPECT_FLOAT_EQ(dot_result, static_cast<float>(N * 2));
}

TEST_F(DynamicVectorTest, EigenInterop)
{
  DynamicVectorf v{1.0f, 2.0f, 3.0f, 4.0f};

  // as_eigen() provides a Map (zero-copy view)
  auto eigen_map = v.as_eigen();
  EXPECT_EQ(eigen_map.size(), 4);
  EXPECT_FLOAT_EQ(eigen_map[0], 1.0f);
  EXPECT_FLOAT_EQ(eigen_map[3], 4.0f);

  // Modify through map
  eigen_map[0] = 10.0f;
  EXPECT_FLOAT_EQ(v[0], 10.0f);

  // from_eigen() creates a copy
  Eigen::VectorXf ev(3);
  ev << 5.0f, 6.0f, 7.0f;
  DynamicVectorf from_ev = DynamicVectorf::fromEigen(ev);
  EXPECT_EQ(from_ev.size(), 3u);
  EXPECT_FLOAT_EQ(from_ev[0], 5.0f);
  EXPECT_FLOAT_EQ(from_ev[2], 7.0f);
}

TEST_F(DynamicVectorTest, Resize)
{
  DynamicVectorf v(5, 1.0f);
  EXPECT_EQ(v.size(), 5u);

  v.resize(10);
  EXPECT_EQ(v.size(), 10u);

  v.resize(3, 2.0f);
  EXPECT_EQ(v.size(), 3u);
}

// =============================================================================
// DynamicMatrix Tests
// =============================================================================

class DynamicMatrixTest : public ::testing::Test
{
};

TEST_F(DynamicMatrixTest, Construction)
{
  DynamicMatrixf m1(3, 4);
  EXPECT_EQ(m1.rows(), 3u);
  EXPECT_EQ(m1.cols(), 4u);
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      EXPECT_FLOAT_EQ(m1(i, j), 0.0f);
    }
  }

  DynamicMatrixf m2(2, 2, 5.0f);
  for (std::size_t i = 0; i < 2; ++i) {
    for (std::size_t j = 0; j < 2; ++j) {
      EXPECT_FLOAT_EQ(m2(i, j), 5.0f);
    }
  }
}

TEST_F(DynamicMatrixTest, ColumnMajorAccess)
{
  DynamicMatrixf m(2, 3);
  // Column-major: data_[col * rows + row]
  m(0, 0) = 1.0f;
  m(1, 0) = 2.0f;
  m(0, 1) = 3.0f;
  m(1, 1) = 4.0f;
  m(0, 2) = 5.0f;
  m(1, 2) = 6.0f;

  // Verify column-major layout in raw data
  const float* data = m.data();
  EXPECT_FLOAT_EQ(data[0], 1.0f); // m(0,0)
  EXPECT_FLOAT_EQ(data[1], 2.0f); // m(1,0)
  EXPECT_FLOAT_EQ(data[2], 3.0f); // m(0,1)
  EXPECT_FLOAT_EQ(data[3], 4.0f); // m(1,1)
  EXPECT_FLOAT_EQ(data[4], 5.0f); // m(0,2)
  EXPECT_FLOAT_EQ(data[5], 6.0f); // m(1,2)
}

TEST_F(DynamicMatrixTest, Identity)
{
  auto m = DynamicMatrixf::identity(3);
  EXPECT_EQ(m.rows(), 3u);
  EXPECT_EQ(m.cols(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(m(i, j), expected);
    }
  }
}

TEST_F(DynamicMatrixTest, SetZeroAndIdentity)
{
  DynamicMatrixf m(3, 3, 5.0f);
  m.setZero();
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_FLOAT_EQ(m(i, j), 0.0f);
    }
  }

  m.setIdentity();
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_FLOAT_EQ(m(i, j), expected);
    }
  }
}

TEST_F(DynamicMatrixTest, MatrixVectorMultiply)
{
  // 2x3 matrix
  DynamicMatrixf m(2, 3);
  m(0, 0) = 1.0f;
  m(0, 1) = 2.0f;
  m(0, 2) = 3.0f;
  m(1, 0) = 4.0f;
  m(1, 1) = 5.0f;
  m(1, 2) = 6.0f;

  DynamicVectorf v{1.0f, 2.0f, 3.0f};

  DynamicVectorf result = m * v;
  EXPECT_EQ(result.size(), 2u);
  // result[0] = 1*1 + 2*2 + 3*3 = 1 + 4 + 9 = 14
  EXPECT_FLOAT_EQ(result[0], 14.0f);
  // result[1] = 4*1 + 5*2 + 6*3 = 4 + 10 + 18 = 32
  EXPECT_FLOAT_EQ(result[1], 32.0f);
}

TEST_F(DynamicMatrixTest, MatrixMatrixMultiply)
{
  // 2x3 @ 3x2 = 2x2
  DynamicMatrixf a(2, 3);
  a(0, 0) = 1.0f;
  a(0, 1) = 2.0f;
  a(0, 2) = 3.0f;
  a(1, 0) = 4.0f;
  a(1, 1) = 5.0f;
  a(1, 2) = 6.0f;

  DynamicMatrixf b(3, 2);
  b(0, 0) = 7.0f;
  b(0, 1) = 8.0f;
  b(1, 0) = 9.0f;
  b(1, 1) = 10.0f;
  b(2, 0) = 11.0f;
  b(2, 1) = 12.0f;

  DynamicMatrixf c = a * b;
  EXPECT_EQ(c.rows(), 2u);
  EXPECT_EQ(c.cols(), 2u);
  // c(0,0) = 1*7 + 2*9 + 3*11 = 7 + 18 + 33 = 58
  EXPECT_FLOAT_EQ(c(0, 0), 58.0f);
  // c(0,1) = 1*8 + 2*10 + 3*12 = 8 + 20 + 36 = 64
  EXPECT_FLOAT_EQ(c(0, 1), 64.0f);
  // c(1,0) = 4*7 + 5*9 + 6*11 = 28 + 45 + 66 = 139
  EXPECT_FLOAT_EQ(c(1, 0), 139.0f);
  // c(1,1) = 4*8 + 5*10 + 6*12 = 32 + 50 + 72 = 154
  EXPECT_FLOAT_EQ(c(1, 1), 154.0f);
}

TEST_F(DynamicMatrixTest, MatrixAddSubtract)
{
  DynamicMatrixf a(2, 2);
  a(0, 0) = 1.0f;
  a(0, 1) = 2.0f;
  a(1, 0) = 3.0f;
  a(1, 1) = 4.0f;

  DynamicMatrixf b(2, 2);
  b(0, 0) = 5.0f;
  b(0, 1) = 6.0f;
  b(1, 0) = 7.0f;
  b(1, 1) = 8.0f;

  DynamicMatrixf sum = a + b;
  EXPECT_FLOAT_EQ(sum(0, 0), 6.0f);
  EXPECT_FLOAT_EQ(sum(1, 1), 12.0f);

  DynamicMatrixf diff = b - a;
  EXPECT_FLOAT_EQ(diff(0, 0), 4.0f);
  EXPECT_FLOAT_EQ(diff(1, 1), 4.0f);
}

TEST_F(DynamicMatrixTest, ScalarMultiply)
{
  DynamicMatrixf m(2, 2);
  m(0, 0) = 1.0f;
  m(0, 1) = 2.0f;
  m(1, 0) = 3.0f;
  m(1, 1) = 4.0f;

  DynamicMatrixf scaled = m * 2.0f;
  EXPECT_FLOAT_EQ(scaled(0, 0), 2.0f);
  EXPECT_FLOAT_EQ(scaled(1, 1), 8.0f);

  m *= 3.0f;
  EXPECT_FLOAT_EQ(m(0, 0), 3.0f);
  EXPECT_FLOAT_EQ(m(1, 1), 12.0f);
}

TEST_F(DynamicMatrixTest, Transpose)
{
  DynamicMatrixf m(2, 3);
  m(0, 0) = 1.0f;
  m(0, 1) = 2.0f;
  m(0, 2) = 3.0f;
  m(1, 0) = 4.0f;
  m(1, 1) = 5.0f;
  m(1, 2) = 6.0f;

  DynamicMatrixf t = m.transposed();
  EXPECT_EQ(t.rows(), 3u);
  EXPECT_EQ(t.cols(), 2u);
  EXPECT_FLOAT_EQ(t(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(t(0, 1), 4.0f);
  EXPECT_FLOAT_EQ(t(1, 0), 2.0f);
  EXPECT_FLOAT_EQ(t(1, 1), 5.0f);
  EXPECT_FLOAT_EQ(t(2, 0), 3.0f);
  EXPECT_FLOAT_EQ(t(2, 1), 6.0f);
}

TEST_F(DynamicMatrixTest, EigenInterop)
{
  DynamicMatrixf m(2, 3);
  m(0, 0) = 1.0f;
  m(0, 1) = 2.0f;
  m(0, 2) = 3.0f;
  m(1, 0) = 4.0f;
  m(1, 1) = 5.0f;
  m(1, 2) = 6.0f;

  // as_eigen() provides a Map (zero-copy view)
  auto eigen_map = m.as_eigen();
  EXPECT_EQ(eigen_map.rows(), 2);
  EXPECT_EQ(eigen_map.cols(), 3);
  EXPECT_FLOAT_EQ(eigen_map(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(eigen_map(1, 2), 6.0f);

  // Modify through map
  eigen_map(0, 0) = 10.0f;
  EXPECT_FLOAT_EQ(m(0, 0), 10.0f);

  // from_eigen() creates a copy
  Eigen::MatrixXf em(2, 2);
  em << 1.0f, 2.0f, 3.0f, 4.0f;
  DynamicMatrixf from_em = DynamicMatrixf::fromEigen(em);
  EXPECT_EQ(from_em.rows(), 2u);
  EXPECT_EQ(from_em.cols(), 2u);
  EXPECT_FLOAT_EQ(from_em(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(from_em(1, 1), 4.0f);
}

TEST_F(DynamicMatrixTest, LargeSizeSimd)
{
  constexpr std::size_t N = 100;
  DynamicMatrixf a(N, N, 1.0f);
  DynamicMatrixf b(N, N, 2.0f);

  DynamicMatrixf sum = a + b;
  for (std::size_t i = 0; i < N; ++i) {
    for (std::size_t j = 0; j < N; ++j) {
      EXPECT_FLOAT_EQ(sum(i, j), 3.0f);
    }
  }
}

// =============================================================================
// Double Precision Tests
// =============================================================================

class DynamicVectorDoubleTest : public ::testing::Test
{
};

TEST_F(DynamicVectorDoubleTest, BasicOps)
{
  DynamicVectord a{1.0, 2.0, 3.0, 4.0};
  DynamicVectord b{4.0, 3.0, 2.0, 1.0};

  double dot_result = a.dot(b);
  EXPECT_DOUBLE_EQ(dot_result, 20.0);

  DynamicVectord sum = a + b;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(sum[i], 5.0);
  }
}

class DynamicMatrixDoubleTest : public ::testing::Test
{
};

TEST_F(DynamicMatrixDoubleTest, BasicOps)
{
  auto m = DynamicMatrixd::identity(3);
  DynamicVectord v{1.0, 2.0, 3.0};

  DynamicVectord result = m * v;
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 3.0);
}
