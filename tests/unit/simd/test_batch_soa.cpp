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

#include <array>
#include <random>

#include <cmath>

using namespace dart::simd;

// =============================================================================
// Matrix3x3SoA Tests
// =============================================================================

class Matrix3x3SoATest : public ::testing::Test
{
protected:
  static constexpr std::size_t N = 8;

  std::array<Matrix3x3f, N> generateRandomMatrices(int seed = 42)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> dist(0.1f, 10.0f);

    std::array<Matrix3x3f, N> matrices;
    for (auto& m : matrices) {
      m = Matrix3x3f(
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen));
    }
    return matrices;
  }
};

TEST_F(Matrix3x3SoATest, Construction)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<float, N> soa(matrices);

  // Verify each matrix can be retrieved
  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3f retrieved = soa.get(i);
    EXPECT_FLOAT_EQ(retrieved(0, 0), matrices[i](0, 0));
    EXPECT_FLOAT_EQ(retrieved(1, 1), matrices[i](1, 1));
    EXPECT_FLOAT_EQ(retrieved(2, 2), matrices[i](2, 2));
  }
}

TEST_F(Matrix3x3SoATest, AosToSoaRoundTrip)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<float, N> soa(matrices);
  auto back = soa.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        EXPECT_FLOAT_EQ(back[i](r, c), matrices[i](r, c));
      }
    }
  }
}

TEST_F(Matrix3x3SoATest, Determinant)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<float, N> soa(matrices);

  Vec<float, N> dets = soa.determinant();

  // Compare with scalar computation
  for (std::size_t i = 0; i < N; ++i) {
    float expected = matrices[i].determinant();
    EXPECT_NEAR(dets[i], expected, std::abs(expected) * 1e-5f);
  }
}

TEST_F(Matrix3x3SoATest, DeterminantIdentity)
{
  std::array<Matrix3x3f, N> matrices;
  for (auto& m : matrices) {
    m = Matrix3x3f::identity();
  }

  Matrix3x3SoA<float, N> soa(matrices);
  Vec<float, N> dets = soa.determinant();

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(dets[i], 1.0f);
  }
}

TEST_F(Matrix3x3SoATest, Inverse)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<float, N> soa(matrices);

  Matrix3x3SoA<float, N> invSoa = soa.inverse();
  auto inverses = invSoa.toAos();

  // Verify A * A^-1 = I for each matrix
  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3f product = matrices[i] * inverses[i];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        float expected = (r == c) ? 1.0f : 0.0f;
        EXPECT_NEAR(product(r, c), expected, 1e-4f);
      }
    }
  }
}

TEST_F(Matrix3x3SoATest, InverseIdentity)
{
  std::array<Matrix3x3f, N> matrices;
  for (auto& m : matrices) {
    m = Matrix3x3f::identity();
  }

  Matrix3x3SoA<float, N> soa(matrices);
  Matrix3x3SoA<float, N> invSoa = soa.inverse();

  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3f inv = invSoa.get(i);
    EXPECT_FLOAT_EQ(inv(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(inv(1, 1), 1.0f);
    EXPECT_FLOAT_EQ(inv(2, 2), 1.0f);
    EXPECT_NEAR(inv(0, 1), 0.0f, 1e-6f);
  }
}

TEST_F(Matrix3x3SoATest, InverseMatchesScalar)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<float, N> soa(matrices);

  Matrix3x3SoA<float, N> invSoa = soa.inverse();

  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3f scalarInv = matrices[i].inverse();
    Matrix3x3f batchInv = invSoa.get(i);

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        EXPECT_NEAR(batchInv(r, c), scalarInv(r, c), 1e-4f);
      }
    }
  }
}

// =============================================================================
// Matrix4x4SoA Tests
// =============================================================================

class Matrix4x4SoATest : public ::testing::Test
{
protected:
  static constexpr std::size_t N = 8;

  std::array<Matrix4x4f, N> generateRandomMatrices(int seed = 42)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> dist(0.1f, 10.0f);

    std::array<Matrix4x4f, N> matrices;
    for (auto& m : matrices) {
      Eigen::Matrix4f em;
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          em(r, c) = dist(gen);
        }
      }
      m = Matrix4x4f::fromEigen(em);
    }
    return matrices;
  }
};

TEST_F(Matrix4x4SoATest, Construction)
{
  auto matrices = generateRandomMatrices();
  Matrix4x4SoA<float, N> soa(matrices);

  for (std::size_t i = 0; i < N; ++i) {
    Matrix4x4f retrieved = soa.get(i);
    EXPECT_FLOAT_EQ(retrieved(0, 0), matrices[i](0, 0));
    EXPECT_FLOAT_EQ(retrieved(1, 1), matrices[i](1, 1));
    EXPECT_FLOAT_EQ(retrieved(2, 2), matrices[i](2, 2));
    EXPECT_FLOAT_EQ(retrieved(3, 3), matrices[i](3, 3));
  }
}

TEST_F(Matrix4x4SoATest, Determinant)
{
  auto matrices = generateRandomMatrices();
  Matrix4x4SoA<float, N> soa(matrices);

  Vec<float, N> dets = soa.determinant();

  for (std::size_t i = 0; i < N; ++i) {
    float expected = matrices[i].determinant();
    EXPECT_NEAR(dets[i], expected, std::abs(expected) * 1e-4f);
  }
}

TEST_F(Matrix4x4SoATest, DeterminantIdentity)
{
  std::array<Matrix4x4f, N> matrices;
  for (auto& m : matrices) {
    m = Matrix4x4f::identity();
  }

  Matrix4x4SoA<float, N> soa(matrices);
  Vec<float, N> dets = soa.determinant();

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(dets[i], 1.0f);
  }
}

TEST_F(Matrix4x4SoATest, DeterminantKnownValue)
{
  // Test with a known matrix:
  // det([[5,-2,2,7],[1,0,0,3],[-3,1,5,0],[3,-1,-9,4]]) = 88
  std::array<Matrix4x4f, N> matrices;
  Eigen::Matrix4f em;
  em << 5, -2, 2, 7, 1, 0, 0, 3, -3, 1, 5, 0, 3, -1, -9, 4;

  for (auto& m : matrices) {
    m = Matrix4x4f::fromEigen(em);
  }

  Matrix4x4SoA<float, N> soa(matrices);
  Vec<float, N> dets = soa.determinant();

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(dets[i], 88.0f, 1e-3f);
  }
}

TEST_F(Matrix4x4SoATest, Inverse)
{
  auto matrices = generateRandomMatrices();
  Matrix4x4SoA<float, N> soa(matrices);

  Matrix4x4SoA<float, N> invSoa = soa.inverse();

  // Verify A * A^-1 = I for each matrix
  for (std::size_t i = 0; i < N; ++i) {
    Matrix4x4f inv = invSoa.get(i);
    Matrix4x4f product = matrices[i] * inv;

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        float expected = (r == c) ? 1.0f : 0.0f;
        EXPECT_NEAR(product(r, c), expected, 1e-3f);
      }
    }
  }
}

TEST_F(Matrix4x4SoATest, InverseIdentity)
{
  std::array<Matrix4x4f, N> matrices;
  for (auto& m : matrices) {
    m = Matrix4x4f::identity();
  }

  Matrix4x4SoA<float, N> soa(matrices);
  Matrix4x4SoA<float, N> invSoa = soa.inverse();

  for (std::size_t i = 0; i < N; ++i) {
    Matrix4x4f inv = invSoa.get(i);
    EXPECT_FLOAT_EQ(inv(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(inv(1, 1), 1.0f);
    EXPECT_FLOAT_EQ(inv(2, 2), 1.0f);
    EXPECT_FLOAT_EQ(inv(3, 3), 1.0f);
    EXPECT_NEAR(inv(0, 1), 0.0f, 1e-6f);
  }
}

TEST_F(Matrix4x4SoATest, InverseMatchesEigen)
{
  auto matrices = generateRandomMatrices();
  Matrix4x4SoA<float, N> soa(matrices);

  Matrix4x4SoA<float, N> invSoa = soa.inverse();

  for (std::size_t i = 0; i < N; ++i) {
    Eigen::Matrix4f eigenMat = matrices[i].toEigen();
    Eigen::Matrix4f eigenInv = eigenMat.inverse();

    Matrix4x4f ourInv = invSoa.get(i);
    Eigen::Matrix4f ourInvEigen = ourInv.toEigen();

    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        EXPECT_NEAR(ourInvEigen(r, c), eigenInv(r, c), 1e-3f)
            << "Matrix " << i << " inverse mismatch at (" << r << "," << c
            << ")";
      }
    }
  }
}

// =============================================================================
// Vector3SoA Tests
// =============================================================================

class Vector3SoATest : public ::testing::Test
{
protected:
  static constexpr std::size_t N = 8;

  std::array<Vector3f, N> generateRandomVectors(int seed = 42)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

    std::array<Vector3f, N> vectors;
    for (auto& v : vectors) {
      v = Vector3f(dist(gen), dist(gen), dist(gen));
    }
    return vectors;
  }

  std::array<Vector3f, N> generateRandomNormals(int seed = 42)
  {
    auto vectors = generateRandomVectors(seed);
    for (auto& v : vectors) {
      v = v.normalized();
    }
    return vectors;
  }
};

TEST_F(Vector3SoATest, Construction)
{
  auto vectors = generateRandomVectors();
  Vector3SoA<float, N> soa(vectors);

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f retrieved = soa.get(i);
    EXPECT_FLOAT_EQ(retrieved.x(), vectors[i].x());
    EXPECT_FLOAT_EQ(retrieved.y(), vectors[i].y());
    EXPECT_FLOAT_EQ(retrieved.z(), vectors[i].z());
  }
}

TEST_F(Vector3SoATest, AosToSoaRoundTrip)
{
  auto vectors = generateRandomVectors();
  Vector3SoA<float, N> soa(vectors);
  auto back = soa.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(back[i].x(), vectors[i].x());
    EXPECT_FLOAT_EQ(back[i].y(), vectors[i].y());
    EXPECT_FLOAT_EQ(back[i].z(), vectors[i].z());
  }
}

TEST_F(Vector3SoATest, DotProduct)
{
  auto a = generateRandomVectors(1);
  auto b = generateRandomVectors(2);

  Vector3SoA<float, N> soaA(a);
  Vector3SoA<float, N> soaB(b);

  Vec<float, N> dots = dot(soaA, soaB);

  for (std::size_t i = 0; i < N; ++i) {
    float expected = dart::simd::dot(a[i], b[i]);
    EXPECT_NEAR(dots[i], expected, std::abs(expected) * 1e-5f);
  }
}

TEST_F(Vector3SoATest, CrossProduct)
{
  auto a = generateRandomVectors(1);
  auto b = generateRandomVectors(2);

  Vector3SoA<float, N> soaA(a);
  Vector3SoA<float, N> soaB(b);

  Vector3SoA<float, N> crosses = cross(soaA, soaB);
  auto results = crosses.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f expected = dart::simd::cross(a[i], b[i]);
    EXPECT_NEAR(results[i].x(), expected.x(), 1e-4f);
    EXPECT_NEAR(results[i].y(), expected.y(), 1e-4f);
    EXPECT_NEAR(results[i].z(), expected.z(), 1e-4f);
  }
}

TEST_F(Vector3SoATest, CrossProductUnitVectors)
{
  std::array<Vector3f, N> xs, ys;
  for (std::size_t i = 0; i < N; ++i) {
    xs[i] = Vector3f::unit_x();
    ys[i] = Vector3f::unit_y();
  }

  Vector3SoA<float, N> soaX(xs);
  Vector3SoA<float, N> soaY(ys);

  Vector3SoA<float, N> crosses = cross(soaX, soaY);

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f result = crosses.get(i);
    EXPECT_FLOAT_EQ(result.x(), 0.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
    EXPECT_FLOAT_EQ(result.z(), 1.0f);
  }
}

TEST_F(Vector3SoATest, Normalize)
{
  auto vectors = generateRandomVectors();
  Vector3SoA<float, N> soa(vectors);

  Vector3SoA<float, N> normalized = normalize(soa);
  auto results = normalized.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    float norm = results[i].norm();
    EXPECT_NEAR(norm, 1.0f, 1e-5f);

    // Check direction is preserved
    Vector3f expected = vectors[i].normalized();
    EXPECT_NEAR(results[i].x(), expected.x(), 1e-5f);
    EXPECT_NEAR(results[i].y(), expected.y(), 1e-5f);
    EXPECT_NEAR(results[i].z(), expected.z(), 1e-5f);
  }
}

TEST_F(Vector3SoATest, OuterProduct)
{
  auto a = generateRandomVectors(1);
  auto b = generateRandomVectors(2);

  Vector3SoA<float, N> soaA(a);
  Vector3SoA<float, N> soaB(b);

  Matrix3x3SoA<float, N> outers = outer(soaA, soaB);
  auto results = outers.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3f expected = dart::simd::outer(a[i], b[i]);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        EXPECT_NEAR(results[i](r, c), expected(r, c), 1e-4f);
      }
    }
  }
}

TEST_F(Vector3SoATest, Reflect)
{
  auto vectors = generateRandomVectors(1);
  auto normals = generateRandomNormals(2);

  Vector3SoA<float, N> soaV(vectors);
  Vector3SoA<float, N> soaN(normals);

  Vector3SoA<float, N> reflected = reflect(soaV, soaN);
  auto results = reflected.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f expected = dart::simd::reflect(vectors[i], normals[i]);
    EXPECT_NEAR(results[i].x(), expected.x(), 1e-4f);
    EXPECT_NEAR(results[i].y(), expected.y(), 1e-4f);
    EXPECT_NEAR(results[i].z(), expected.z(), 1e-4f);
  }
}

TEST_F(Vector3SoATest, ReflectSimple)
{
  // v = (1, -1, 0), n = (0, 1, 0) => reflected = (1, 1, 0)
  std::array<Vector3f, N> vs, ns;
  for (std::size_t i = 0; i < N; ++i) {
    vs[i] = Vector3f(1.0f, -1.0f, 0.0f);
    ns[i] = Vector3f(0.0f, 1.0f, 0.0f);
  }

  Vector3SoA<float, N> soaV(vs);
  Vector3SoA<float, N> soaN(ns);

  Vector3SoA<float, N> reflected = reflect(soaV, soaN);

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f result = reflected.get(i);
    EXPECT_FLOAT_EQ(result.x(), 1.0f);
    EXPECT_FLOAT_EQ(result.y(), 1.0f);
    EXPECT_FLOAT_EQ(result.z(), 0.0f);
  }
}

TEST_F(Vector3SoATest, Project)
{
  auto a = generateRandomVectors(1);
  auto b = generateRandomVectors(2);

  Vector3SoA<float, N> soaA(a);
  Vector3SoA<float, N> soaB(b);

  Vector3SoA<float, N> projected = project(soaA, soaB);
  auto results = projected.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f expected = dart::simd::project(a[i], b[i]);
    EXPECT_NEAR(results[i].x(), expected.x(), 1e-4f);
    EXPECT_NEAR(results[i].y(), expected.y(), 1e-4f);
    EXPECT_NEAR(results[i].z(), expected.z(), 1e-4f);
  }
}

TEST_F(Vector3SoATest, ProjectSimple)
{
  // project (1,1,0) onto (1,0,0) => (1,0,0)
  std::array<Vector3f, N> as, bs;
  for (std::size_t i = 0; i < N; ++i) {
    as[i] = Vector3f(1.0f, 1.0f, 0.0f);
    bs[i] = Vector3f(1.0f, 0.0f, 0.0f);
  }

  Vector3SoA<float, N> soaA(as);
  Vector3SoA<float, N> soaB(bs);

  Vector3SoA<float, N> projected = project(soaA, soaB);

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f result = projected.get(i);
    EXPECT_FLOAT_EQ(result.x(), 1.0f);
    EXPECT_FLOAT_EQ(result.y(), 0.0f);
    EXPECT_FLOAT_EQ(result.z(), 0.0f);
  }
}

TEST_F(Vector3SoATest, Arithmetic)
{
  auto a = generateRandomVectors(1);
  auto b = generateRandomVectors(2);

  Vector3SoA<float, N> soaA(a);
  Vector3SoA<float, N> soaB(b);

  // Test addition
  Vector3SoA<float, N> sum = soaA + soaB;
  auto sumResults = sum.toAos();
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(sumResults[i].x(), a[i].x() + b[i].x());
    EXPECT_FLOAT_EQ(sumResults[i].y(), a[i].y() + b[i].y());
    EXPECT_FLOAT_EQ(sumResults[i].z(), a[i].z() + b[i].z());
  }

  // Test subtraction
  Vector3SoA<float, N> diff = soaA - soaB;
  auto diffResults = diff.toAos();
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(diffResults[i].x(), a[i].x() - b[i].x());
    EXPECT_FLOAT_EQ(diffResults[i].y(), a[i].y() - b[i].y());
    EXPECT_FLOAT_EQ(diffResults[i].z(), a[i].z() - b[i].z());
  }

  // Test negation
  Vector3SoA<float, N> neg = -soaA;
  auto negResults = neg.toAos();
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(negResults[i].x(), -a[i].x());
    EXPECT_FLOAT_EQ(negResults[i].y(), -a[i].y());
    EXPECT_FLOAT_EQ(negResults[i].z(), -a[i].z());
  }
}

TEST_F(Vector3SoATest, NormalizeZeroLengthReturnsZero)
{
  std::array<Vector3f, N> vectors;
  for (std::size_t i = 0; i < N; ++i) {
    if (i % 2 == 0) {
      vectors[i] = Vector3f(0.0f, 0.0f, 0.0f);
    } else {
      vectors[i] = Vector3f(3.0f, 4.0f, 0.0f);
    }
  }

  Vector3SoA<float, N> soa(vectors);
  Vector3SoA<float, N> normalized = normalize(soa);
  auto results = normalized.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    if (i % 2 == 0) {
      EXPECT_FLOAT_EQ(results[i].x(), 0.0f)
          << "Zero vector should normalize to zero";
      EXPECT_FLOAT_EQ(results[i].y(), 0.0f);
      EXPECT_FLOAT_EQ(results[i].z(), 0.0f);
      EXPECT_FALSE(std::isnan(results[i].x())) << "Should not produce NaN";
    } else {
      EXPECT_NEAR(results[i].norm(), 1.0f, 1e-5f);
    }
  }
}

TEST_F(Vector3SoATest, ProjectOntoZeroVectorReturnsZero)
{
  std::array<Vector3f, N> as, bs;
  for (std::size_t i = 0; i < N; ++i) {
    as[i] = Vector3f(1.0f, 2.0f, 3.0f);
    if (i % 2 == 0) {
      bs[i] = Vector3f(0.0f, 0.0f, 0.0f);
    } else {
      bs[i] = Vector3f(1.0f, 0.0f, 0.0f);
    }
  }

  Vector3SoA<float, N> soaA(as);
  Vector3SoA<float, N> soaB(bs);
  Vector3SoA<float, N> projected = project(soaA, soaB);
  auto results = projected.toAos();

  for (std::size_t i = 0; i < N; ++i) {
    if (i % 2 == 0) {
      EXPECT_FLOAT_EQ(results[i].x(), 0.0f)
          << "Projection onto zero should be zero";
      EXPECT_FLOAT_EQ(results[i].y(), 0.0f);
      EXPECT_FLOAT_EQ(results[i].z(), 0.0f);
      EXPECT_FALSE(std::isnan(results[i].x())) << "Should not produce NaN";
    } else {
      EXPECT_NEAR(results[i].x(), 1.0f, 1e-5f);
      EXPECT_NEAR(results[i].y(), 0.0f, 1e-5f);
      EXPECT_NEAR(results[i].z(), 0.0f, 1e-5f);
    }
  }
}

// =============================================================================
// Double precision tests
// =============================================================================

class Matrix3x3SoADoubleTest : public ::testing::Test
{
protected:
  static constexpr std::size_t N = 4;

  std::array<Matrix3x3d, N> generateRandomMatrices(int seed = 42)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> dist(0.1, 10.0);

    std::array<Matrix3x3d, N> matrices;
    for (auto& m : matrices) {
      m = Matrix3x3d(
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen),
          dist(gen));
    }
    return matrices;
  }
};

TEST_F(Matrix3x3SoADoubleTest, Determinant)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<double, N> soa(matrices);

  Vec<double, N> dets = soa.determinant();

  for (std::size_t i = 0; i < N; ++i) {
    double expected = matrices[i].determinant();
    EXPECT_NEAR(dets[i], expected, std::abs(expected) * 1e-10);
  }
}

TEST_F(Matrix3x3SoADoubleTest, Inverse)
{
  auto matrices = generateRandomMatrices();
  Matrix3x3SoA<double, N> soa(matrices);

  Matrix3x3SoA<double, N> invSoa = soa.inverse();

  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3d inv = invSoa.get(i);
    Matrix3x3d product = matrices[i] * inv;

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        double expected = (r == c) ? 1.0 : 0.0;
        EXPECT_NEAR(product(r, c), expected, 1e-10);
      }
    }
  }
}

// =============================================================================
// Load/Store from arrays tests
// =============================================================================

class SoALoadStoreTest : public ::testing::Test
{
protected:
  static constexpr std::size_t N = 8;
};

TEST_F(SoALoadStoreTest, Matrix3x3LoadFromArrays)
{
  alignas(64) float m00[N], m10[N], m20[N];
  alignas(64) float m01[N], m11[N], m21[N];
  alignas(64) float m02[N], m12[N], m22[N];

  for (std::size_t i = 0; i < N; ++i) {
    m00[i] = static_cast<float>(i);
    m10[i] = static_cast<float>(i + 10);
    m20[i] = static_cast<float>(i + 20);
    m01[i] = static_cast<float>(i + 30);
    m11[i] = static_cast<float>(i + 40);
    m21[i] = static_cast<float>(i + 50);
    m02[i] = static_cast<float>(i + 60);
    m12[i] = static_cast<float>(i + 70);
    m22[i] = static_cast<float>(i + 80);
  }

  auto soa = Matrix3x3SoA<float, N>::loadFromArrays(
      m00, m10, m20, m01, m11, m21, m02, m12, m22);

  for (std::size_t i = 0; i < N; ++i) {
    Matrix3x3f mat = soa.get(i);
    EXPECT_FLOAT_EQ(mat(0, 0), static_cast<float>(i));
    EXPECT_FLOAT_EQ(mat(1, 0), static_cast<float>(i + 10));
    EXPECT_FLOAT_EQ(mat(2, 0), static_cast<float>(i + 20));
    EXPECT_FLOAT_EQ(mat(0, 1), static_cast<float>(i + 30));
    EXPECT_FLOAT_EQ(mat(1, 1), static_cast<float>(i + 40));
    EXPECT_FLOAT_EQ(mat(2, 1), static_cast<float>(i + 50));
    EXPECT_FLOAT_EQ(mat(0, 2), static_cast<float>(i + 60));
    EXPECT_FLOAT_EQ(mat(1, 2), static_cast<float>(i + 70));
    EXPECT_FLOAT_EQ(mat(2, 2), static_cast<float>(i + 80));
  }
}

TEST_F(SoALoadStoreTest, Matrix3x3StoreToArrays)
{
  std::array<Matrix3x3f, N> matrices;
  for (std::size_t i = 0; i < N; ++i) {
    matrices[i] = Matrix3x3f(
        static_cast<float>(i),
        static_cast<float>(i + 1),
        static_cast<float>(i + 2),
        static_cast<float>(i + 3),
        static_cast<float>(i + 4),
        static_cast<float>(i + 5),
        static_cast<float>(i + 6),
        static_cast<float>(i + 7),
        static_cast<float>(i + 8));
  }

  Matrix3x3SoA<float, N> soa(matrices);

  alignas(64) float r00[N], r10[N], r20[N];
  alignas(64) float r01[N], r11[N], r21[N];
  alignas(64) float r02[N], r12[N], r22[N];

  soa.storeToArrays(r00, r10, r20, r01, r11, r21, r02, r12, r22);

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(r00[i], matrices[i](0, 0));
    EXPECT_FLOAT_EQ(r11[i], matrices[i](1, 1));
    EXPECT_FLOAT_EQ(r22[i], matrices[i](2, 2));
  }
}

TEST_F(SoALoadStoreTest, Vector3LoadFromArrays)
{
  alignas(64) float xs[N], ys[N], zs[N];

  for (std::size_t i = 0; i < N; ++i) {
    xs[i] = static_cast<float>(i);
    ys[i] = static_cast<float>(i + 10);
    zs[i] = static_cast<float>(i + 20);
  }

  auto soa = Vector3SoA<float, N>::loadFromArrays(xs, ys, zs);

  for (std::size_t i = 0; i < N; ++i) {
    Vector3f v = soa.get(i);
    EXPECT_FLOAT_EQ(v.x(), static_cast<float>(i));
    EXPECT_FLOAT_EQ(v.y(), static_cast<float>(i + 10));
    EXPECT_FLOAT_EQ(v.z(), static_cast<float>(i + 20));
  }
}

TEST_F(SoALoadStoreTest, Vector3StoreToArrays)
{
  std::array<Vector3f, N> vectors;
  for (std::size_t i = 0; i < N; ++i) {
    vectors[i] = Vector3f(
        static_cast<float>(i),
        static_cast<float>(i + 10),
        static_cast<float>(i + 20));
  }

  Vector3SoA<float, N> soa(vectors);

  alignas(64) float xs[N], ys[N], zs[N];
  soa.storeToArrays(xs, ys, zs);

  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_FLOAT_EQ(xs[i], static_cast<float>(i));
    EXPECT_FLOAT_EQ(ys[i], static_cast<float>(i + 10));
    EXPECT_FLOAT_EQ(zs[i], static_cast<float>(i + 20));
  }
}
