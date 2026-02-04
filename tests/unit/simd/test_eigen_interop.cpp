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

#include <dart/simd/eigen/interop.hpp>
#include <dart/simd/eigen/iterator.hpp>
#include <dart/simd/simd.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace ds = dart::simd;

class EigenInteropTest : public ::testing::Test
{
protected:
  static constexpr double tol = 1e-15;
  static constexpr float tolf = 1e-6f;
};

TEST_F(EigenInteropTest, ToVecVector2d)
{
  Eigen::Vector2d v(1.0, 2.0);
  auto vec = ds::toVec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
}

TEST_F(EigenInteropTest, ToVecVector3d)
{
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  auto vec = ds::toVec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
}

TEST_F(EigenInteropTest, ToVecVector4d)
{
  Eigen::Vector4d v(1.0, 2.0, 3.0, 4.0);
  auto vec = ds::toVec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
  EXPECT_NEAR(vec[3], 4.0, tol);
}

TEST_F(EigenInteropTest, ToVecVector4f)
{
  Eigen::Vector4f v(1.0f, 2.0f, 3.0f, 4.0f);
  auto vec = ds::toVec(v);

  EXPECT_NEAR(vec[0], 1.0f, tolf);
  EXPECT_NEAR(vec[1], 2.0f, tolf);
  EXPECT_NEAR(vec[2], 3.0f, tolf);
  EXPECT_NEAR(vec[3], 4.0f, tolf);
}

TEST_F(EigenInteropTest, ToVecRowVector)
{
  Eigen::RowVector4d v(1.0, 2.0, 3.0, 4.0);
  auto vec = ds::toVec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
  EXPECT_NEAR(vec[3], 4.0, tol);
}

TEST_F(EigenInteropTest, ToVec3Padded)
{
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  auto vec = ds::toVec3Padded(v, 0.0);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
  EXPECT_NEAR(vec[3], 0.0, tol);
}

TEST_F(EigenInteropTest, ToVec3PaddedCustomPad)
{
  Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
  auto vec = ds::toVec3Padded(v, 99.0f);

  EXPECT_NEAR(vec[0], 1.0f, tolf);
  EXPECT_NEAR(vec[1], 2.0f, tolf);
  EXPECT_NEAR(vec[2], 3.0f, tolf);
  EXPECT_NEAR(vec[3], 99.0f, tolf);
}

TEST_F(EigenInteropTest, ToEigenVec2)
{
  auto vec = ds::Vec<double, 2>::set(1.0, 2.0);

  auto eigen = ds::toEigen(vec);

  EXPECT_NEAR(eigen[0], 1.0, tol);
  EXPECT_NEAR(eigen[1], 2.0, tol);
}

TEST_F(EigenInteropTest, ToEigenVec4)
{
  auto vec = ds::Vec4d::set(1.0, 2.0, 3.0, 4.0);

  Eigen::Vector4d eigen = ds::toEigen(vec);

  EXPECT_NEAR(eigen[0], 1.0, tol);
  EXPECT_NEAR(eigen[1], 2.0, tol);
  EXPECT_NEAR(eigen[2], 3.0, tol);
  EXPECT_NEAR(eigen[3], 4.0, tol);
}

TEST_F(EigenInteropTest, ToEigen3FromVec4)
{
  auto vec = ds::Vec4d::set(1.0, 2.0, 3.0, 99.0);

  Eigen::Vector3d eigen = ds::toEigen3(vec);

  EXPECT_NEAR(eigen[0], 1.0, tol);
  EXPECT_NEAR(eigen[1], 2.0, tol);
  EXPECT_NEAR(eigen[2], 3.0, tol);
}

TEST_F(EigenInteropTest, RoundTrip)
{
  Eigen::Vector4d original(1.5, 2.5, 3.5, 4.5);

  auto vec = ds::toVec(original);
  Eigen::Vector4d result = ds::toEigen(vec);

  EXPECT_NEAR(result[0], original[0], tol);
  EXPECT_NEAR(result[1], original[1], tol);
  EXPECT_NEAR(result[2], original[2], tol);
  EXPECT_NEAR(result[3], original[3], tol);
}

TEST_F(EigenInteropTest, RoundTrip3Padded)
{
  Eigen::Vector3d original(1.5, 2.5, 3.5);

  auto vec = ds::toVec3Padded(original);
  Eigen::Vector3d result = ds::toEigen3(vec);

  EXPECT_NEAR(result[0], original[0], tol);
  EXPECT_NEAR(result[1], original[1], tol);
  EXPECT_NEAR(result[2], original[2], tol);
}

TEST_F(EigenInteropTest, EigenSoA3Construction)
{
  std::array<Eigen::Vector3d, 4> aos
      = {Eigen::Vector3d(1, 2, 3),
         Eigen::Vector3d(4, 5, 6),
         Eigen::Vector3d(7, 8, 9),
         Eigen::Vector3d(10, 11, 12)};

  ds::EigenSoA3d4 soa(aos);

  EXPECT_NEAR(soa.x[0], 1.0, tol);
  EXPECT_NEAR(soa.x[1], 4.0, tol);
  EXPECT_NEAR(soa.x[2], 7.0, tol);
  EXPECT_NEAR(soa.x[3], 10.0, tol);

  EXPECT_NEAR(soa.y[0], 2.0, tol);
  EXPECT_NEAR(soa.y[1], 5.0, tol);
  EXPECT_NEAR(soa.y[2], 8.0, tol);
  EXPECT_NEAR(soa.y[3], 11.0, tol);

  EXPECT_NEAR(soa.z[0], 3.0, tol);
  EXPECT_NEAR(soa.z[1], 6.0, tol);
  EXPECT_NEAR(soa.z[2], 9.0, tol);
  EXPECT_NEAR(soa.z[3], 12.0, tol);
}

TEST_F(EigenInteropTest, EigenSoA3ToAoS)
{
  std::array<Eigen::Vector3d, 4> original
      = {Eigen::Vector3d(1, 2, 3),
         Eigen::Vector3d(4, 5, 6),
         Eigen::Vector3d(7, 8, 9),
         Eigen::Vector3d(10, 11, 12)};

  ds::EigenSoA3d4 soa(original);
  auto result = soa.toAos();

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(result[i][0], original[i][0], tol);
    EXPECT_NEAR(result[i][1], original[i][1], tol);
    EXPECT_NEAR(result[i][2], original[i][2], tol);
  }
}

TEST_F(EigenInteropTest, EigenSoA3GetSet)
{
  ds::EigenSoA3d4 soa;
  soa.x = ds::Vec4d::zero();
  soa.y = ds::Vec4d::zero();
  soa.z = ds::Vec4d::zero();

  soa.set(1, Eigen::Vector3d(10, 20, 30));

  auto v = soa.get(1);
  EXPECT_NEAR(v[0], 10.0, tol);
  EXPECT_NEAR(v[1], 20.0, tol);
  EXPECT_NEAR(v[2], 30.0, tol);
}

TEST_F(EigenInteropTest, Dot3)
{
  std::array<Eigen::Vector3d, 4> a_aos
      = {Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(1, 1, 1)};

  std::array<Eigen::Vector3d, 4> b_aos
      = {Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(1, 1, 1)};

  ds::EigenSoA3d4 a(a_aos);
  ds::EigenSoA3d4 b(b_aos);

  auto dots = ds::dot3(a, b);

  EXPECT_NEAR(dots[0], 1.0, tol);
  EXPECT_NEAR(dots[1], 1.0, tol);
  EXPECT_NEAR(dots[2], 1.0, tol);
  EXPECT_NEAR(dots[3], 3.0, tol);
}

TEST_F(EigenInteropTest, Cross3)
{
  std::array<Eigen::Vector3d, 4> a_aos
      = {Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(1, 2, 3)};

  std::array<Eigen::Vector3d, 4> b_aos
      = {Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(4, 5, 6)};

  ds::EigenSoA3d4 a(a_aos);
  ds::EigenSoA3d4 b(b_aos);

  auto cross = ds::cross3(a, b);
  auto result = cross.toAos();

  EXPECT_NEAR(result[0][0], 0.0, tol);
  EXPECT_NEAR(result[0][1], 0.0, tol);
  EXPECT_NEAR(result[0][2], 1.0, tol);

  EXPECT_NEAR(result[1][0], 1.0, tol);
  EXPECT_NEAR(result[1][1], 0.0, tol);
  EXPECT_NEAR(result[1][2], 0.0, tol);

  EXPECT_NEAR(result[2][0], 0.0, tol);
  EXPECT_NEAR(result[2][1], 1.0, tol);
  EXPECT_NEAR(result[2][2], 0.0, tol);

  EXPECT_NEAR(result[3][0], -3.0, tol);
  EXPECT_NEAR(result[3][1], 6.0, tol);
  EXPECT_NEAR(result[3][2], -3.0, tol);
}

TEST_F(EigenInteropTest, TransposeAoSToSoA)
{
  std::array<Eigen::Vector3f, 4> aos
      = {Eigen::Vector3f(1, 2, 3),
         Eigen::Vector3f(4, 5, 6),
         Eigen::Vector3f(7, 8, 9),
         Eigen::Vector3f(10, 11, 12)};

  auto soa = ds::transposeAosToSoa(aos);

  EXPECT_NEAR(soa.x[2], 7.0f, tolf);
  EXPECT_NEAR(soa.y[2], 8.0f, tolf);
  EXPECT_NEAR(soa.z[2], 9.0f, tolf);
}

TEST_F(EigenInteropTest, TransposeSoAToAoS)
{
  ds::EigenSoA3f4 soa;
  soa.x = ds::Vec4f::set(1.0f, 4.0f, 7.0f, 10.0f);
  soa.y = ds::Vec4f::set(2.0f, 5.0f, 8.0f, 11.0f);
  soa.z = ds::Vec4f::set(3.0f, 6.0f, 9.0f, 12.0f);

  auto aos = ds::transposeSoaToAos(soa);

  EXPECT_NEAR(aos[0][0], 1.0f, tolf);
  EXPECT_NEAR(aos[0][1], 2.0f, tolf);
  EXPECT_NEAR(aos[0][2], 3.0f, tolf);

  EXPECT_NEAR(aos[2][0], 7.0f, tolf);
  EXPECT_NEAR(aos[2][1], 8.0f, tolf);
  EXPECT_NEAR(aos[2][2], 9.0f, tolf);
}

TEST_F(EigenInteropTest, EigenSoA4Construction)
{
  std::array<Eigen::Vector4d, 4> aos
      = {Eigen::Vector4d(1, 2, 3, 4),
         Eigen::Vector4d(5, 6, 7, 8),
         Eigen::Vector4d(9, 10, 11, 12),
         Eigen::Vector4d(13, 14, 15, 16)};

  ds::EigenSoA4d4 soa(aos);

  EXPECT_NEAR(soa.x[0], 1.0, tol);
  EXPECT_NEAR(soa.x[1], 5.0, tol);
  EXPECT_NEAR(soa.x[2], 9.0, tol);
  EXPECT_NEAR(soa.x[3], 13.0, tol);

  EXPECT_NEAR(soa.y[0], 2.0, tol);
  EXPECT_NEAR(soa.y[1], 6.0, tol);
  EXPECT_NEAR(soa.y[2], 10.0, tol);
  EXPECT_NEAR(soa.y[3], 14.0, tol);

  EXPECT_NEAR(soa.z[0], 3.0, tol);
  EXPECT_NEAR(soa.z[1], 7.0, tol);
  EXPECT_NEAR(soa.z[2], 11.0, tol);
  EXPECT_NEAR(soa.z[3], 15.0, tol);

  EXPECT_NEAR(soa.w[0], 4.0, tol);
  EXPECT_NEAR(soa.w[1], 8.0, tol);
  EXPECT_NEAR(soa.w[2], 12.0, tol);
  EXPECT_NEAR(soa.w[3], 16.0, tol);
}

TEST_F(EigenInteropTest, EigenSoA4ToAoS)
{
  std::array<Eigen::Vector4d, 4> original
      = {Eigen::Vector4d(1, 2, 3, 4),
         Eigen::Vector4d(5, 6, 7, 8),
         Eigen::Vector4d(9, 10, 11, 12),
         Eigen::Vector4d(13, 14, 15, 16)};

  ds::EigenSoA4d4 soa(original);
  auto result = soa.toAos();

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(result[i][0], original[i][0], tol);
    EXPECT_NEAR(result[i][1], original[i][1], tol);
    EXPECT_NEAR(result[i][2], original[i][2], tol);
    EXPECT_NEAR(result[i][3], original[i][3], tol);
  }
}

TEST_F(EigenInteropTest, EigenSoA4GetSet)
{
  ds::EigenSoA4d4 soa;
  soa.x = ds::Vec4d::zero();
  soa.y = ds::Vec4d::zero();
  soa.z = ds::Vec4d::zero();
  soa.w = ds::Vec4d::zero();

  soa.set(2, Eigen::Vector4d(10, 20, 30, 40));

  auto v = soa.get(2);
  EXPECT_NEAR(v[0], 10.0, tol);
  EXPECT_NEAR(v[1], 20.0, tol);
  EXPECT_NEAR(v[2], 30.0, tol);
  EXPECT_NEAR(v[3], 40.0, tol);
}

TEST_F(EigenInteropTest, Dot4)
{
  std::array<Eigen::Vector4d, 4> a_aos
      = {Eigen::Vector4d(1, 0, 0, 0),
         Eigen::Vector4d(0, 1, 0, 0),
         Eigen::Vector4d(0, 0, 1, 0),
         Eigen::Vector4d(1, 1, 1, 1)};

  std::array<Eigen::Vector4d, 4> b_aos
      = {Eigen::Vector4d(1, 0, 0, 0),
         Eigen::Vector4d(0, 1, 0, 0),
         Eigen::Vector4d(0, 0, 1, 0),
         Eigen::Vector4d(1, 1, 1, 1)};

  ds::EigenSoA4d4 a(a_aos);
  ds::EigenSoA4d4 b(b_aos);

  auto dots = ds::dot4(a, b);

  EXPECT_NEAR(dots[0], 1.0, tol);
  EXPECT_NEAR(dots[1], 1.0, tol);
  EXPECT_NEAR(dots[2], 1.0, tol);
  EXPECT_NEAR(dots[3], 4.0, tol);
}

TEST_F(EigenInteropTest, EigenSoA4TransposeAoSToSoA)
{
  std::array<Eigen::Vector4f, 4> aos
      = {Eigen::Vector4f(1, 2, 3, 4),
         Eigen::Vector4f(5, 6, 7, 8),
         Eigen::Vector4f(9, 10, 11, 12),
         Eigen::Vector4f(13, 14, 15, 16)};

  auto soa = ds::transposeAosToSoa(aos);

  EXPECT_NEAR(soa.x[1], 5.0f, tolf);
  EXPECT_NEAR(soa.y[1], 6.0f, tolf);
  EXPECT_NEAR(soa.z[1], 7.0f, tolf);
  EXPECT_NEAR(soa.w[1], 8.0f, tolf);
}

TEST_F(EigenInteropTest, EigenSoA4TransposeSoAToAoS)
{
  ds::EigenSoA4f4 soa;
  soa.x = ds::Vec4f::set(1.0f, 5.0f, 9.0f, 13.0f);
  soa.y = ds::Vec4f::set(2.0f, 6.0f, 10.0f, 14.0f);
  soa.z = ds::Vec4f::set(3.0f, 7.0f, 11.0f, 15.0f);
  soa.w = ds::Vec4f::set(4.0f, 8.0f, 12.0f, 16.0f);

  auto aos = ds::transposeSoaToAos(soa);

  EXPECT_NEAR(aos[0][0], 1.0f, tolf);
  EXPECT_NEAR(aos[0][1], 2.0f, tolf);
  EXPECT_NEAR(aos[0][2], 3.0f, tolf);
  EXPECT_NEAR(aos[0][3], 4.0f, tolf);

  EXPECT_NEAR(aos[3][0], 13.0f, tolf);
  EXPECT_NEAR(aos[3][1], 14.0f, tolf);
  EXPECT_NEAR(aos[3][2], 15.0f, tolf);
  EXPECT_NEAR(aos[3][3], 16.0f, tolf);
}

TEST_F(EigenInteropTest, TransformPointsBatchIdentity)
{
  std::array<Eigen::Vector3d, 4> points_aos
      = {Eigen::Vector3d(1, 2, 3),
         Eigen::Vector3d(4, 5, 6),
         Eigen::Vector3d(7, 8, 9),
         Eigen::Vector3d(10, 11, 12)};

  ds::EigenSoA3d4 points(points_aos);
  auto identity = ds::Matrix4x4d::identity();

  auto result = ds::transformPoints(identity, points);
  auto result_aos = result.toAos();

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(result_aos[i][0], points_aos[i][0], tol);
    EXPECT_NEAR(result_aos[i][1], points_aos[i][1], tol);
    EXPECT_NEAR(result_aos[i][2], points_aos[i][2], tol);
  }
}

TEST_F(EigenInteropTest, TransformPointsBatchTranslation)
{
  std::array<Eigen::Vector3d, 4> points_aos
      = {Eigen::Vector3d(0, 0, 0),
         Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1)};

  ds::EigenSoA3d4 points(points_aos);

  Eigen::Matrix4d eigen_transform = Eigen::Matrix4d::Identity();
  eigen_transform(0, 3) = 10.0;
  eigen_transform(1, 3) = 20.0;
  eigen_transform(2, 3) = 30.0;
  ds::Matrix4x4d transform(eigen_transform);

  auto result = ds::transformPoints(transform, points);
  auto result_aos = result.toAos();

  EXPECT_NEAR(result_aos[0][0], 10.0, tol);
  EXPECT_NEAR(result_aos[0][1], 20.0, tol);
  EXPECT_NEAR(result_aos[0][2], 30.0, tol);

  EXPECT_NEAR(result_aos[1][0], 11.0, tol);
  EXPECT_NEAR(result_aos[1][1], 20.0, tol);
  EXPECT_NEAR(result_aos[1][2], 30.0, tol);
}

TEST_F(EigenInteropTest, TransformVectorsBatchIdentity)
{
  std::array<Eigen::Vector3d, 4> vectors_aos
      = {Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(1, 1, 1)};

  ds::EigenSoA3d4 vectors(vectors_aos);
  auto identity = ds::Matrix4x4d::identity();

  auto result = ds::transformVectors(identity, vectors);
  auto result_aos = result.toAos();

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(result_aos[i][0], vectors_aos[i][0], tol);
    EXPECT_NEAR(result_aos[i][1], vectors_aos[i][1], tol);
    EXPECT_NEAR(result_aos[i][2], vectors_aos[i][2], tol);
  }
}

TEST_F(EigenInteropTest, TransformVectorsIgnoresTranslation)
{
  std::array<Eigen::Vector3d, 4> vectors_aos
      = {Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(0, 0, 1),
         Eigen::Vector3d(1, 1, 1)};

  ds::EigenSoA3d4 vectors(vectors_aos);

  Eigen::Matrix4d eigen_transform = Eigen::Matrix4d::Identity();
  eigen_transform(0, 3) = 100.0;
  eigen_transform(1, 3) = 200.0;
  eigen_transform(2, 3) = 300.0;
  ds::Matrix4x4d transform(eigen_transform);

  auto result = ds::transformVectors(transform, vectors);
  auto result_aos = result.toAos();

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(result_aos[i][0], vectors_aos[i][0], tol);
    EXPECT_NEAR(result_aos[i][1], vectors_aos[i][1], tol);
    EXPECT_NEAR(result_aos[i][2], vectors_aos[i][2], tol);
  }
}

TEST_F(EigenInteropTest, TransformPointsBatchRotation90Z)
{
  std::array<Eigen::Vector3d, 4> points_aos
      = {Eigen::Vector3d(1, 0, 0),
         Eigen::Vector3d(0, 1, 0),
         Eigen::Vector3d(1, 1, 0),
         Eigen::Vector3d(2, 0, 0)};

  ds::EigenSoA3d4 points(points_aos);

  Eigen::Matrix4d eigen_transform = Eigen::Matrix4d::Identity();
  eigen_transform(0, 0) = 0.0;
  eigen_transform(0, 1) = -1.0;
  eigen_transform(1, 0) = 1.0;
  eigen_transform(1, 1) = 0.0;
  ds::Matrix4x4d transform(eigen_transform);

  auto result = ds::transformPoints(transform, points);
  auto result_aos = result.toAos();

  EXPECT_NEAR(result_aos[0][0], 0.0, tol);
  EXPECT_NEAR(result_aos[0][1], 1.0, tol);
  EXPECT_NEAR(result_aos[0][2], 0.0, tol);

  EXPECT_NEAR(result_aos[1][0], -1.0, tol);
  EXPECT_NEAR(result_aos[1][1], 0.0, tol);
  EXPECT_NEAR(result_aos[1][2], 0.0, tol);

  EXPECT_NEAR(result_aos[2][0], -1.0, tol);
  EXPECT_NEAR(result_aos[2][1], 1.0, tol);
  EXPECT_NEAR(result_aos[2][2], 0.0, tol);

  EXPECT_NEAR(result_aos[3][0], 0.0, tol);
  EXPECT_NEAR(result_aos[3][1], 2.0, tol);
  EXPECT_NEAR(result_aos[3][2], 0.0, tol);
}

class SimdChunksIteratorTest : public ::testing::Test
{
protected:
  static constexpr float tolf = 1e-6f;
};

TEST_F(SimdChunksIteratorTest, BasicIteration)
{
  std::vector<Eigen::Vector3f> data(8);
  for (size_t i = 0; i < 8; ++i) {
    data[i] = Eigen::Vector3f(
        static_cast<float>(i),
        static_cast<float>(i * 2),
        static_cast<float>(i * 3));
  }

  auto view = ds::simdChunks<4>(data);
  EXPECT_EQ(view.size(), 2u);
  EXPECT_EQ(view.remainder(), 0u);

  size_t chunk_idx = 0;
  for (auto chunk : view) {
    auto aos = chunk.toAos();
    for (size_t i = 0; i < 4; ++i) {
      size_t global_idx = chunk_idx * 4 + i;
      EXPECT_NEAR(aos[i][0], static_cast<float>(global_idx), tolf);
      EXPECT_NEAR(aos[i][1], static_cast<float>(global_idx * 2), tolf);
      EXPECT_NEAR(aos[i][2], static_cast<float>(global_idx * 3), tolf);
    }
    ++chunk_idx;
  }
  EXPECT_EQ(chunk_idx, 2u);
}

TEST_F(SimdChunksIteratorTest, WithRemainder)
{
  std::vector<Eigen::Vector3f> data(10);
  for (size_t i = 0; i < 10; ++i) {
    data[i] = Eigen::Vector3f(static_cast<float>(i), 0.0f, 0.0f);
  }

  auto view = ds::simdChunks<4>(data);
  EXPECT_EQ(view.size(), 2u);
  EXPECT_EQ(view.remainder(), 2u);

  size_t count = 0;
  for ([[maybe_unused]] auto chunk : view) {
    ++count;
  }
  EXPECT_EQ(count, 2u);

  auto rem = view.remainderSpan();
  EXPECT_EQ(rem.size(), 2u);
  EXPECT_NEAR(rem[0][0], 8.0f, tolf);
  EXPECT_NEAR(rem[1][0], 9.0f, tolf);
}

TEST_F(SimdChunksIteratorTest, EmptyInput)
{
  std::vector<Eigen::Vector3f> data;
  auto view = ds::simdChunks<4>(data);
  EXPECT_EQ(view.size(), 0u);
  EXPECT_EQ(view.remainder(), 0u);
  EXPECT_TRUE(view.remainderSpan().empty());

  size_t count = 0;
  for ([[maybe_unused]] auto chunk : view) {
    ++count;
  }
  EXPECT_EQ(count, 0u);
}

TEST_F(SimdChunksIteratorTest, LessThanChunkSize)
{
  std::vector<Eigen::Vector3f> data(3);
  for (size_t i = 0; i < 3; ++i) {
    data[i] = Eigen::Vector3f(static_cast<float>(i), 0.0f, 0.0f);
  }

  auto view = ds::simdChunks<4>(data);
  EXPECT_EQ(view.size(), 0u);
  EXPECT_EQ(view.remainder(), 3u);

  size_t count = 0;
  for ([[maybe_unused]] auto chunk : view) {
    ++count;
  }
  EXPECT_EQ(count, 0u);

  auto rem = view.remainderSpan();
  EXPECT_EQ(rem.size(), 3u);
}

TEST_F(SimdChunksIteratorTest, ArrayOverload)
{
  std::array<Eigen::Vector3f, 8> data;
  for (size_t i = 0; i < 8; ++i) {
    data[i] = Eigen::Vector3f(static_cast<float>(i), 0.0f, 0.0f);
  }

  auto view = ds::simdChunks<4>(data);
  EXPECT_EQ(view.size(), 2u);

  size_t count = 0;
  for ([[maybe_unused]] auto chunk : view) {
    ++count;
  }
  EXPECT_EQ(count, 2u);
}

TEST_F(SimdChunksIteratorTest, SumAllPoints)
{
  std::vector<Eigen::Vector3f> data(16);
  for (size_t i = 0; i < 16; ++i) {
    data[i] = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  }

  ds::Vec4f sum_x = ds::Vec4f::zero();
  ds::Vec4f sum_y = ds::Vec4f::zero();
  ds::Vec4f sum_z = ds::Vec4f::zero();

  for (auto chunk : ds::simdChunks<4>(data)) {
    sum_x = sum_x + chunk.x;
    sum_y = sum_y + chunk.y;
    sum_z = sum_z + chunk.z;
  }

  float total_x = ds::hsum(sum_x);
  float total_y = ds::hsum(sum_y);
  float total_z = ds::hsum(sum_z);

  EXPECT_NEAR(total_x, 16.0f, tolf);
  EXPECT_NEAR(total_y, 32.0f, tolf);
  EXPECT_NEAR(total_z, 48.0f, tolf);
}
