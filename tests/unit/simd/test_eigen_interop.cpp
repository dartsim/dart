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
  auto vec = ds::to_vec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
}

TEST_F(EigenInteropTest, ToVecVector3d)
{
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  auto vec = ds::to_vec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
}

TEST_F(EigenInteropTest, ToVecVector4d)
{
  Eigen::Vector4d v(1.0, 2.0, 3.0, 4.0);
  auto vec = ds::to_vec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
  EXPECT_NEAR(vec[3], 4.0, tol);
}

TEST_F(EigenInteropTest, ToVecVector4f)
{
  Eigen::Vector4f v(1.0f, 2.0f, 3.0f, 4.0f);
  auto vec = ds::to_vec(v);

  EXPECT_NEAR(vec[0], 1.0f, tolf);
  EXPECT_NEAR(vec[1], 2.0f, tolf);
  EXPECT_NEAR(vec[2], 3.0f, tolf);
  EXPECT_NEAR(vec[3], 4.0f, tolf);
}

TEST_F(EigenInteropTest, ToVecRowVector)
{
  Eigen::RowVector4d v(1.0, 2.0, 3.0, 4.0);
  auto vec = ds::to_vec(v);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
  EXPECT_NEAR(vec[3], 4.0, tol);
}

TEST_F(EigenInteropTest, ToVec3Padded)
{
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  auto vec = ds::to_vec3_padded(v, 0.0);

  EXPECT_NEAR(vec[0], 1.0, tol);
  EXPECT_NEAR(vec[1], 2.0, tol);
  EXPECT_NEAR(vec[2], 3.0, tol);
  EXPECT_NEAR(vec[3], 0.0, tol);
}

TEST_F(EigenInteropTest, ToVec3PaddedCustomPad)
{
  Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
  auto vec = ds::to_vec3_padded(v, 99.0f);

  EXPECT_NEAR(vec[0], 1.0f, tolf);
  EXPECT_NEAR(vec[1], 2.0f, tolf);
  EXPECT_NEAR(vec[2], 3.0f, tolf);
  EXPECT_NEAR(vec[3], 99.0f, tolf);
}

TEST_F(EigenInteropTest, ToEigenVec2)
{
  ds::Vec<double, 2> vec;
  vec[0] = 1.0;
  vec[1] = 2.0;

  auto eigen = ds::to_eigen(vec);

  EXPECT_NEAR(eigen[0], 1.0, tol);
  EXPECT_NEAR(eigen[1], 2.0, tol);
}

TEST_F(EigenInteropTest, ToEigenVec4)
{
  ds::Vec4d vec;
  vec[0] = 1.0;
  vec[1] = 2.0;
  vec[2] = 3.0;
  vec[3] = 4.0;

  Eigen::Vector4d eigen = ds::to_eigen(vec);

  EXPECT_NEAR(eigen[0], 1.0, tol);
  EXPECT_NEAR(eigen[1], 2.0, tol);
  EXPECT_NEAR(eigen[2], 3.0, tol);
  EXPECT_NEAR(eigen[3], 4.0, tol);
}

TEST_F(EigenInteropTest, ToEigen3FromVec4)
{
  ds::Vec4d vec;
  vec[0] = 1.0;
  vec[1] = 2.0;
  vec[2] = 3.0;
  vec[3] = 99.0;

  Eigen::Vector3d eigen = ds::to_eigen3(vec);

  EXPECT_NEAR(eigen[0], 1.0, tol);
  EXPECT_NEAR(eigen[1], 2.0, tol);
  EXPECT_NEAR(eigen[2], 3.0, tol);
}

TEST_F(EigenInteropTest, RoundTrip)
{
  Eigen::Vector4d original(1.5, 2.5, 3.5, 4.5);

  auto vec = ds::to_vec(original);
  Eigen::Vector4d result = ds::to_eigen(vec);

  EXPECT_NEAR(result[0], original[0], tol);
  EXPECT_NEAR(result[1], original[1], tol);
  EXPECT_NEAR(result[2], original[2], tol);
  EXPECT_NEAR(result[3], original[3], tol);
}

TEST_F(EigenInteropTest, RoundTrip3Padded)
{
  Eigen::Vector3d original(1.5, 2.5, 3.5);

  auto vec = ds::to_vec3_padded(original);
  Eigen::Vector3d result = ds::to_eigen3(vec);

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
  auto result = soa.to_aos();

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
  auto result = cross.to_aos();

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

  auto soa = ds::transpose_aos_to_soa(aos);

  EXPECT_NEAR(soa.x[2], 7.0f, tolf);
  EXPECT_NEAR(soa.y[2], 8.0f, tolf);
  EXPECT_NEAR(soa.z[2], 9.0f, tolf);
}

TEST_F(EigenInteropTest, TransposeSoAToAoS)
{
  ds::EigenSoA3f4 soa;
  soa.x = ds::Vec4f(std::array<float, 4>{1, 4, 7, 10});
  soa.y = ds::Vec4f(std::array<float, 4>{2, 5, 8, 11});
  soa.z = ds::Vec4f(std::array<float, 4>{3, 6, 9, 12});

  auto aos = ds::transpose_soa_to_aos(soa);

  EXPECT_NEAR(aos[0][0], 1.0f, tolf);
  EXPECT_NEAR(aos[0][1], 2.0f, tolf);
  EXPECT_NEAR(aos[0][2], 3.0f, tolf);

  EXPECT_NEAR(aos[2][0], 7.0f, tolf);
  EXPECT_NEAR(aos[2][1], 8.0f, tolf);
  EXPECT_NEAR(aos[2][2], 9.0f, tolf);
}
