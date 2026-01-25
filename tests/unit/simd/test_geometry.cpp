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

#include <dart/math/constants.hpp>

#include <dart/simd/simd.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart::simd;

namespace {
constexpr double pi = dart::math::pi;
}

class Vector3Test : public ::testing::Test
{
};

TEST_F(Vector3Test, Construction)
{
  Vector3f v1(1.0f, 2.0f, 3.0f);
  EXPECT_FLOAT_EQ(v1.x(), 1.0f);
  EXPECT_FLOAT_EQ(v1.y(), 2.0f);
  EXPECT_FLOAT_EQ(v1.z(), 3.0f);

  Vector3f v2 = Vector3f::zero();
  EXPECT_FLOAT_EQ(v2.x(), 0.0f);
  EXPECT_FLOAT_EQ(v2.y(), 0.0f);
  EXPECT_FLOAT_EQ(v2.z(), 0.0f);

  Vector3f v3 = Vector3f::unit_x();
  EXPECT_FLOAT_EQ(v3.x(), 1.0f);
  EXPECT_FLOAT_EQ(v3.y(), 0.0f);
  EXPECT_FLOAT_EQ(v3.z(), 0.0f);
}

TEST_F(Vector3Test, Arithmetic)
{
  Vector3f a(1.0f, 2.0f, 3.0f);
  Vector3f b(4.0f, 5.0f, 6.0f);

  Vector3f sum = a + b;
  EXPECT_FLOAT_EQ(sum.x(), 5.0f);
  EXPECT_FLOAT_EQ(sum.y(), 7.0f);
  EXPECT_FLOAT_EQ(sum.z(), 9.0f);

  Vector3f diff = b - a;
  EXPECT_FLOAT_EQ(diff.x(), 3.0f);
  EXPECT_FLOAT_EQ(diff.y(), 3.0f);
  EXPECT_FLOAT_EQ(diff.z(), 3.0f);

  Vector3f scaled = a * 2.0f;
  EXPECT_FLOAT_EQ(scaled.x(), 2.0f);
  EXPECT_FLOAT_EQ(scaled.y(), 4.0f);
  EXPECT_FLOAT_EQ(scaled.z(), 6.0f);

  Vector3f neg = -a;
  EXPECT_FLOAT_EQ(neg.x(), -1.0f);
  EXPECT_FLOAT_EQ(neg.y(), -2.0f);
  EXPECT_FLOAT_EQ(neg.z(), -3.0f);
}

TEST_F(Vector3Test, DotProduct)
{
  Vector3f a(1.0f, 2.0f, 3.0f);
  Vector3f b(4.0f, 5.0f, 6.0f);

  float d = dot(a, b);
  EXPECT_FLOAT_EQ(d, 32.0f);
}

TEST_F(Vector3Test, CrossProduct)
{
  Vector3f x = Vector3f::unit_x();
  Vector3f y = Vector3f::unit_y();
  Vector3f z = cross(x, y);

  EXPECT_FLOAT_EQ(z.x(), 0.0f);
  EXPECT_FLOAT_EQ(z.y(), 0.0f);
  EXPECT_FLOAT_EQ(z.z(), 1.0f);
}

TEST_F(Vector3Test, NormAndNormalize)
{
  Vector3f v(3.0f, 4.0f, 0.0f);

  EXPECT_FLOAT_EQ(v.squaredNorm(), 25.0f);
  EXPECT_FLOAT_EQ(v.norm(), 5.0f);

  Vector3f n = v.normalized();
  EXPECT_FLOAT_EQ(n.x(), 0.6f);
  EXPECT_FLOAT_EQ(n.y(), 0.8f);
  EXPECT_FLOAT_EQ(n.z(), 0.0f);
  EXPECT_NEAR(n.norm(), 1.0f, 1e-6f);
}

TEST_F(Vector3Test, EigenInterop)
{
  Eigen::Vector3f ev(1.0f, 2.0f, 3.0f);
  Vector3f sv = Vector3f::fromEigen(ev);

  EXPECT_FLOAT_EQ(sv.x(), ev.x());
  EXPECT_FLOAT_EQ(sv.y(), ev.y());
  EXPECT_FLOAT_EQ(sv.z(), ev.z());

  Eigen::Vector3f back = sv.toEigen();
  EXPECT_FLOAT_EQ(back.x(), ev.x());
  EXPECT_FLOAT_EQ(back.y(), ev.y());
  EXPECT_FLOAT_EQ(back.z(), ev.z());
}

class Vector4Test : public ::testing::Test
{
};

TEST_F(Vector4Test, Construction)
{
  Vector4f v(1.0f, 2.0f, 3.0f, 4.0f);
  EXPECT_FLOAT_EQ(v.x(), 1.0f);
  EXPECT_FLOAT_EQ(v.y(), 2.0f);
  EXPECT_FLOAT_EQ(v.z(), 3.0f);
  EXPECT_FLOAT_EQ(v.w(), 4.0f);
}

TEST_F(Vector4Test, DotProduct)
{
  Vector4f a(1.0f, 2.0f, 3.0f, 4.0f);
  Vector4f b(5.0f, 6.0f, 7.0f, 8.0f);

  float d = dot(a, b);
  EXPECT_FLOAT_EQ(d, 70.0f);
}

TEST_F(Vector4Test, EigenInterop)
{
  Eigen::Vector4f ev(1.0f, 2.0f, 3.0f, 4.0f);
  Vector4f sv = Vector4f::fromEigen(ev);

  EXPECT_FLOAT_EQ(sv.x(), ev.x());
  EXPECT_FLOAT_EQ(sv.y(), ev.y());
  EXPECT_FLOAT_EQ(sv.z(), ev.z());
  EXPECT_FLOAT_EQ(sv.w(), ev.w());

  Eigen::Vector4f back = sv.toEigen();
  EXPECT_TRUE(back.isApprox(ev));
}

class Matrix3x3Test : public ::testing::Test
{
};

TEST_F(Matrix3x3Test, Identity)
{
  Matrix3x3f I = Matrix3x3f::identity();

  EXPECT_FLOAT_EQ(I(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(I(1, 1), 1.0f);
  EXPECT_FLOAT_EQ(I(2, 2), 1.0f);
  EXPECT_FLOAT_EQ(I(0, 1), 0.0f);
  EXPECT_FLOAT_EQ(I(1, 0), 0.0f);
}

TEST_F(Matrix3x3Test, MatrixVectorMultiply)
{
  Matrix3x3f I = Matrix3x3f::identity();
  Vector3f v(1.0f, 2.0f, 3.0f);

  Vector3f result = I * v;
  EXPECT_FLOAT_EQ(result.x(), v.x());
  EXPECT_FLOAT_EQ(result.y(), v.y());
  EXPECT_FLOAT_EQ(result.z(), v.z());

  Matrix3x3f scale(2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 2.0f);
  Vector3f scaled = scale * v;
  EXPECT_FLOAT_EQ(scaled.x(), 2.0f);
  EXPECT_FLOAT_EQ(scaled.y(), 4.0f);
  EXPECT_FLOAT_EQ(scaled.z(), 6.0f);
}

TEST_F(Matrix3x3Test, MatrixMatrixMultiply)
{
  Matrix3x3f I = Matrix3x3f::identity();
  Matrix3x3f A(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);

  Matrix3x3f result = I * A;
  EXPECT_FLOAT_EQ(result(0, 0), A(0, 0));
  EXPECT_FLOAT_EQ(result(1, 1), A(1, 1));
  EXPECT_FLOAT_EQ(result(2, 2), A(2, 2));
}

TEST_F(Matrix3x3Test, Determinant)
{
  Matrix3x3f I = Matrix3x3f::identity();
  EXPECT_FLOAT_EQ(I.determinant(), 1.0f);

  Matrix3x3f scale(2.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 4.0f);
  EXPECT_FLOAT_EQ(scale.determinant(), 24.0f);
}

TEST_F(Matrix3x3Test, Trace)
{
  Matrix3x3f I = Matrix3x3f::identity();
  EXPECT_FLOAT_EQ(I.trace(), 3.0f);

  Matrix3x3f A(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  EXPECT_FLOAT_EQ(A.trace(), 15.0f);
}

TEST_F(Matrix3x3Test, Inverse)
{
  Matrix3x3f I = Matrix3x3f::identity();
  Matrix3x3f Iinv = I.inverse();

  EXPECT_FLOAT_EQ(Iinv(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(Iinv(1, 1), 1.0f);
  EXPECT_FLOAT_EQ(Iinv(2, 2), 1.0f);

  Matrix3x3f scale(2.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 0.0f, 0.0f, 5.0f);
  Matrix3x3f scaleInv = scale.inverse();

  EXPECT_FLOAT_EQ(scaleInv(0, 0), 0.5f);
  EXPECT_FLOAT_EQ(scaleInv(1, 1), 0.25f);
  EXPECT_FLOAT_EQ(scaleInv(2, 2), 0.2f);

  Matrix3x3f product = scale * scaleInv;
  EXPECT_NEAR(product(0, 0), 1.0f, 1e-6f);
  EXPECT_NEAR(product(1, 1), 1.0f, 1e-6f);
  EXPECT_NEAR(product(2, 2), 1.0f, 1e-6f);
  EXPECT_NEAR(product(0, 1), 0.0f, 1e-6f);
}

TEST_F(Matrix3x3Test, InverseGeneral)
{
  Eigen::Matrix3f em;
  em << 1, 2, 3, 0, 1, 4, 5, 6, 0;

  Matrix3x3f A = Matrix3x3f::fromEigen(em);
  Matrix3x3f Ainv = A.inverse();

  Matrix3x3f product = A * Ainv;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(product(i, j), expected, 1e-5f);
    }
  }
}

TEST_F(Matrix3x3Test, TryInverse)
{
  Matrix3x3f I = Matrix3x3f::identity();
  Matrix3x3f result;

  EXPECT_TRUE(I.tryInverse(result));
  EXPECT_FLOAT_EQ(result(0, 0), 1.0f);

  Matrix3x3f singular(1.0f, 2.0f, 3.0f, 2.0f, 4.0f, 6.0f, 1.0f, 1.0f, 1.0f);
  EXPECT_FALSE(singular.tryInverse(result));
}

TEST_F(Matrix3x3Test, EigenInterop)
{
  Eigen::Matrix3f em;
  em << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  Matrix3x3f sm = Matrix3x3f::fromEigen(em);
  Eigen::Matrix3f back = sm.toEigen();

  EXPECT_TRUE(back.isApprox(em));
}

class Matrix4x4Test : public ::testing::Test
{
};

TEST_F(Matrix4x4Test, Identity)
{
  Matrix4x4f I = Matrix4x4f::identity();

  EXPECT_FLOAT_EQ(I(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(I(1, 1), 1.0f);
  EXPECT_FLOAT_EQ(I(2, 2), 1.0f);
  EXPECT_FLOAT_EQ(I(3, 3), 1.0f);
  EXPECT_FLOAT_EQ(I(0, 1), 0.0f);
}

TEST_F(Matrix4x4Test, MatrixVectorMultiply)
{
  Matrix4x4f I = Matrix4x4f::identity();
  Vector4f v(1.0f, 2.0f, 3.0f, 1.0f);

  Vector4f result = I * v;
  EXPECT_FLOAT_EQ(result.x(), v.x());
  EXPECT_FLOAT_EQ(result.y(), v.y());
  EXPECT_FLOAT_EQ(result.z(), v.z());
  EXPECT_FLOAT_EQ(result.w(), v.w());
}

TEST_F(Matrix4x4Test, TransformPoint)
{
  Matrix4x4f I = Matrix4x4f::identity();
  Vector3f p(1.0f, 2.0f, 3.0f);

  Vector3f result = I.transformPoint(p);
  EXPECT_FLOAT_EQ(result.x(), p.x());
  EXPECT_FLOAT_EQ(result.y(), p.y());
  EXPECT_FLOAT_EQ(result.z(), p.z());
}

TEST_F(Matrix4x4Test, TransformVector)
{
  Matrix4x4f I = Matrix4x4f::identity();
  Vector3f v(1.0f, 2.0f, 3.0f);

  Vector3f result = I.transformVector(v);
  EXPECT_FLOAT_EQ(result.x(), v.x());
  EXPECT_FLOAT_EQ(result.y(), v.y());
  EXPECT_FLOAT_EQ(result.z(), v.z());
}

TEST_F(Matrix4x4Test, EigenInterop)
{
  Eigen::Matrix4f em = Eigen::Matrix4f::Identity();
  em(0, 3) = 10.0f;
  em(1, 3) = 20.0f;
  em(2, 3) = 30.0f;

  Matrix4x4f sm = Matrix4x4f::fromEigen(em);
  Eigen::Matrix4f back = sm.toEigen();

  EXPECT_TRUE(back.isApprox(em));
}

TEST_F(Matrix4x4Test, Determinant)
{
  Matrix4x4f I = Matrix4x4f::identity();
  EXPECT_FLOAT_EQ(I.determinant(), 1.0f);

  Eigen::Matrix4f em;
  em << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  Matrix4x4f A = Matrix4x4f::fromEigen(em);
  EXPECT_NEAR(A.determinant(), 0.0f, 1e-5f);

  Eigen::Matrix4f em2;
  em2 << 5, -2, 2, 7, 1, 0, 0, 3, -3, 1, 5, 0, 3, -1, -9, 4;
  Matrix4x4f B = Matrix4x4f::fromEigen(em2);
  EXPECT_NEAR(B.determinant(), 88.0f, 1e-4f);
}

TEST_F(Matrix4x4Test, Trace)
{
  Matrix4x4f I = Matrix4x4f::identity();
  EXPECT_FLOAT_EQ(I.trace(), 4.0f);

  Eigen::Matrix4f em;
  em << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  Matrix4x4f A = Matrix4x4f::fromEigen(em);
  EXPECT_FLOAT_EQ(A.trace(), 34.0f);
}

TEST_F(Matrix4x4Test, Inverse)
{
  Matrix4x4f I = Matrix4x4f::identity();
  Matrix4x4f Iinv = I.inverse();

  EXPECT_FLOAT_EQ(Iinv(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(Iinv(1, 1), 1.0f);
  EXPECT_FLOAT_EQ(Iinv(2, 2), 1.0f);
  EXPECT_FLOAT_EQ(Iinv(3, 3), 1.0f);

  Eigen::Matrix4f em;
  em << 5, -2, 2, 7, 1, 0, 0, 3, -3, 1, 5, 0, 3, -1, -9, 4;
  Matrix4x4f A = Matrix4x4f::fromEigen(em);
  Matrix4x4f Ainv = A.inverse();

  Matrix4x4f product = A * Ainv;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(product(i, j), expected, 1e-4f);
    }
  }
}

TEST_F(Matrix4x4Test, TryInverse)
{
  Matrix4x4f I = Matrix4x4f::identity();
  Matrix4x4f result;

  EXPECT_TRUE(I.tryInverse(result));
  EXPECT_FLOAT_EQ(result(0, 0), 1.0f);

  Eigen::Matrix4f em;
  em << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  Matrix4x4f singular = Matrix4x4f::fromEigen(em);
  EXPECT_FALSE(singular.tryInverse(result));
}

TEST_F(Matrix3x3Test, FrobeniusNorm)
{
  Matrix3x3f scale(2.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 4.0f);
  EXPECT_FLOAT_EQ(scale.squaredFrobeniusNorm(), 4.0f + 9.0f + 16.0f);
  EXPECT_FLOAT_EQ(scale.frobeniusNorm(), std::sqrt(29.0f));
}

TEST_F(Matrix3x3Test, Diagonal)
{
  Matrix3x3f A(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
  Vector3f d = A.diagonal();
  EXPECT_FLOAT_EQ(d.x(), 1.0f);
  EXPECT_FLOAT_EQ(d.y(), 5.0f);
  EXPECT_FLOAT_EQ(d.z(), 9.0f);
}

TEST_F(Matrix3x3Test, Symmetry)
{
  Matrix3x3f A = Matrix3x3f::identity();
  EXPECT_TRUE(A.isSymmetric());

  Matrix3x3f B(1, 2, 3, 2, 5, 6, 3, 6, 9);
  EXPECT_TRUE(B.isSymmetric());

  Matrix3x3f C(1, 2, 3, 4, 5, 6, 7, 8, 9);
  EXPECT_FALSE(C.isSymmetric());
}

TEST_F(Matrix3x3Test, Orthogonality)
{
  Matrix3x3f I = Matrix3x3f::identity();
  EXPECT_TRUE(I.isOrthogonal());

  // Rotation around Z by 90 degrees
  Matrix3x3f R(0, -1, 0, 1, 0, 0, 0, 0, 1);
  EXPECT_TRUE(R.isOrthogonal());

  Matrix3x3f scale(2, 0, 0, 0, 1, 0, 0, 0, 1);
  EXPECT_FALSE(scale.isOrthogonal());
}

TEST_F(Matrix4x4Test, FrobeniusNorm)
{
  Matrix4x4f scale = Matrix4x4f::identity();
  EXPECT_FLOAT_EQ(scale.squaredFrobeniusNorm(), 4.0f);
  EXPECT_FLOAT_EQ(scale.frobeniusNorm(), 2.0f);
}

TEST_F(Matrix4x4Test, Diagonal)
{
  Matrix4x4f I = Matrix4x4f::identity();
  Vector4f d = I.diagonal();
  EXPECT_FLOAT_EQ(d.x(), 1.0f);
  EXPECT_FLOAT_EQ(d.y(), 1.0f);
  EXPECT_FLOAT_EQ(d.z(), 1.0f);
  EXPECT_FLOAT_EQ(d.w(), 1.0f);
}

TEST_F(Matrix4x4Test, Symmetry)
{
  Matrix4x4f I = Matrix4x4f::identity();
  EXPECT_TRUE(I.isSymmetric());

  Eigen::Matrix4f em;
  em << 1, 2, 3, 4, 2, 5, 6, 7, 3, 6, 8, 9, 4, 7, 9, 10;
  Matrix4x4f A = Matrix4x4f::fromEigen(em);
  EXPECT_TRUE(A.isSymmetric());
}

TEST_F(Vector3Test, OuterProduct)
{
  Vector3f a(1, 2, 3);
  Vector3f b(4, 5, 6);
  Matrix3x3f M = outer(a, b);

  EXPECT_FLOAT_EQ(M(0, 0), 4.0f);
  EXPECT_FLOAT_EQ(M(0, 1), 5.0f);
  EXPECT_FLOAT_EQ(M(0, 2), 6.0f);
  EXPECT_FLOAT_EQ(M(1, 0), 8.0f);
  EXPECT_FLOAT_EQ(M(2, 2), 18.0f);
}

TEST_F(Vector3Test, Reflect)
{
  Vector3f v(1, -1, 0);
  Vector3f n(0, 1, 0);
  Vector3f r = reflect(v, n);

  EXPECT_FLOAT_EQ(r.x(), 1.0f);
  EXPECT_FLOAT_EQ(r.y(), 1.0f);
  EXPECT_FLOAT_EQ(r.z(), 0.0f);
}

TEST_F(Vector3Test, Project)
{
  Vector3f a(1, 1, 0);
  Vector3f b(1, 0, 0);
  Vector3f p = project(a, b);

  EXPECT_FLOAT_EQ(p.x(), 1.0f);
  EXPECT_FLOAT_EQ(p.y(), 0.0f);
  EXPECT_FLOAT_EQ(p.z(), 0.0f);
}

TEST_F(Vector4Test, OuterProduct)
{
  Vector4f a(1, 2, 3, 4);
  Vector4f b(5, 6, 7, 8);
  Matrix4x4f M = outer(a, b);

  EXPECT_FLOAT_EQ(M(0, 0), 5.0f);
  EXPECT_FLOAT_EQ(M(3, 3), 32.0f);
  EXPECT_FLOAT_EQ(M(1, 2), 14.0f);
}

class DoubleGeometryTest : public ::testing::Test
{
};

TEST_F(DoubleGeometryTest, Vector3d)
{
  Vector3d v(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(v.x(), 1.0);
  EXPECT_DOUBLE_EQ(v.y(), 2.0);
  EXPECT_DOUBLE_EQ(v.z(), 3.0);

  double d = dot(v, v);
  EXPECT_DOUBLE_EQ(d, 14.0);
}

TEST_F(DoubleGeometryTest, Matrix3x3d)
{
  Matrix3x3d I = Matrix3x3d::identity();
  Vector3d v(1.0, 2.0, 3.0);

  Vector3d result = I * v;
  EXPECT_DOUBLE_EQ(result.x(), v.x());
  EXPECT_DOUBLE_EQ(result.y(), v.y());
  EXPECT_DOUBLE_EQ(result.z(), v.z());
}

class QuaternionTest : public ::testing::Test
{
};

TEST_F(QuaternionTest, Identity)
{
  Quaterniond q = Quaterniond::identity();
  EXPECT_DOUBLE_EQ(q.w(), 1.0);
  EXPECT_DOUBLE_EQ(q.x(), 0.0);
  EXPECT_DOUBLE_EQ(q.y(), 0.0);
  EXPECT_DOUBLE_EQ(q.z(), 0.0);
}

TEST_F(QuaternionTest, FromAxisAngle)
{
  Vector3d axis(0.0, 0.0, 1.0);
  double angle = pi / 2.0;

  Quaterniond q = Quaterniond::fromAxisAngle(axis, angle);

  EXPECT_NEAR(q.w(), std::cos(angle / 2.0), 1e-10);
  EXPECT_NEAR(q.x(), 0.0, 1e-10);
  EXPECT_NEAR(q.y(), 0.0, 1e-10);
  EXPECT_NEAR(q.z(), std::sin(angle / 2.0), 1e-10);
}

TEST_F(QuaternionTest, Rotate)
{
  Vector3d axis(0.0, 0.0, 1.0);
  double angle = pi / 2.0;
  Quaterniond q = Quaterniond::fromAxisAngle(axis, angle);

  Vector3d v(1.0, 0.0, 0.0);
  Vector3d rotated = q.rotate(v);

  EXPECT_NEAR(rotated.x(), 0.0, 1e-10);
  EXPECT_NEAR(rotated.y(), 1.0, 1e-10);
  EXPECT_NEAR(rotated.z(), 0.0, 1e-10);
}

TEST_F(QuaternionTest, Multiplication)
{
  Vector3d axis_z(0.0, 0.0, 1.0);
  Quaterniond q1 = Quaterniond::fromAxisAngle(axis_z, pi / 2.0);
  Quaterniond q2 = Quaterniond::fromAxisAngle(axis_z, pi / 2.0);

  Quaterniond q_composed = q1 * q2;

  Vector3d v(1.0, 0.0, 0.0);
  Vector3d rotated = q_composed.rotate(v);

  EXPECT_NEAR(rotated.x(), -1.0, 1e-10);
  EXPECT_NEAR(rotated.y(), 0.0, 1e-10);
  EXPECT_NEAR(rotated.z(), 0.0, 1e-10);
}

TEST_F(QuaternionTest, Conjugate)
{
  Quaterniond q(0.5, 0.5, 0.5, 0.5);
  Quaterniond conj = q.conjugate();

  EXPECT_DOUBLE_EQ(conj.w(), q.w());
  EXPECT_DOUBLE_EQ(conj.x(), -q.x());
  EXPECT_DOUBLE_EQ(conj.y(), -q.y());
  EXPECT_DOUBLE_EQ(conj.z(), -q.z());
}

TEST_F(QuaternionTest, Inverse)
{
  Vector3d axis(0.0, 0.0, 1.0);
  Quaterniond q = Quaterniond::fromAxisAngle(axis, pi / 3.0);
  Quaterniond inv = q.inverse();

  Quaterniond product = q * inv;
  EXPECT_NEAR(product.w(), 1.0, 1e-10);
  EXPECT_NEAR(product.x(), 0.0, 1e-10);
  EXPECT_NEAR(product.y(), 0.0, 1e-10);
  EXPECT_NEAR(product.z(), 0.0, 1e-10);
}

TEST_F(QuaternionTest, Slerp)
{
  Quaterniond q1 = Quaterniond::identity();
  Vector3d axis(0.0, 0.0, 1.0);
  Quaterniond q2 = Quaterniond::fromAxisAngle(axis, pi / 2.0);

  Quaterniond mid = slerp(q1, q2, 0.5);
  Quaterniond expected = Quaterniond::fromAxisAngle(axis, pi / 4.0);

  EXPECT_NEAR(mid.w(), expected.w(), 1e-10);
  EXPECT_NEAR(mid.x(), expected.x(), 1e-10);
  EXPECT_NEAR(mid.y(), expected.y(), 1e-10);
  EXPECT_NEAR(mid.z(), expected.z(), 1e-10);
}

TEST_F(QuaternionTest, ToRotationMatrix)
{
  Vector3d axis(0.0, 0.0, 1.0);
  double angle = pi / 2.0;
  Quaterniond q = Quaterniond::fromAxisAngle(axis, angle);

  Matrix3x3d m = q.toRotationMatrix();

  EXPECT_NEAR(m(0, 0), 0.0, 1e-10);
  EXPECT_NEAR(m(0, 1), -1.0, 1e-10);
  EXPECT_NEAR(m(1, 0), 1.0, 1e-10);
  EXPECT_NEAR(m(1, 1), 0.0, 1e-10);
  EXPECT_NEAR(m(2, 2), 1.0, 1e-10);
}

TEST_F(QuaternionTest, EigenInterop)
{
  Eigen::Quaterniond eq(0.5, 0.5, 0.5, 0.5);
  eq.normalize();

  Quaterniond sq = Quaterniond::fromEigen(eq);
  EXPECT_NEAR(sq.w(), eq.w(), 1e-10);
  EXPECT_NEAR(sq.x(), eq.x(), 1e-10);
  EXPECT_NEAR(sq.y(), eq.y(), 1e-10);
  EXPECT_NEAR(sq.z(), eq.z(), 1e-10);

  Eigen::Quaterniond back = sq.toEigen();
  EXPECT_NEAR(back.w(), eq.w(), 1e-10);
  EXPECT_NEAR(back.x(), eq.x(), 1e-10);
  EXPECT_NEAR(back.y(), eq.y(), 1e-10);
  EXPECT_NEAR(back.z(), eq.z(), 1e-10);
}

class Isometry3Test : public ::testing::Test
{
};

TEST_F(Isometry3Test, Identity)
{
  Isometry3d iso = Isometry3d::identity();

  Vector3d p(1.0, 2.0, 3.0);
  Vector3d result = iso.transformPoint(p);

  EXPECT_DOUBLE_EQ(result.x(), p.x());
  EXPECT_DOUBLE_EQ(result.y(), p.y());
  EXPECT_DOUBLE_EQ(result.z(), p.z());
}

TEST_F(Isometry3Test, PureTranslation)
{
  Vector3d t(10.0, 20.0, 30.0);
  Isometry3d iso = Isometry3d::fromTranslation(t);

  Vector3d p(1.0, 2.0, 3.0);
  Vector3d result = iso.transformPoint(p);

  EXPECT_DOUBLE_EQ(result.x(), 11.0);
  EXPECT_DOUBLE_EQ(result.y(), 22.0);
  EXPECT_DOUBLE_EQ(result.z(), 33.0);

  Vector3d v(1.0, 0.0, 0.0);
  Vector3d v_result = iso.transformVector(v);
  EXPECT_DOUBLE_EQ(v_result.x(), 1.0);
  EXPECT_DOUBLE_EQ(v_result.y(), 0.0);
  EXPECT_DOUBLE_EQ(v_result.z(), 0.0);
}

TEST_F(Isometry3Test, PureRotation)
{
  Vector3d axis(0.0, 0.0, 1.0);
  Quaterniond q = Quaterniond::fromAxisAngle(axis, pi / 2.0);
  Isometry3d iso = Isometry3d::fromRotation(q);

  Vector3d p(1.0, 0.0, 0.0);
  Vector3d result = iso.transformPoint(p);

  EXPECT_NEAR(result.x(), 0.0, 1e-10);
  EXPECT_NEAR(result.y(), 1.0, 1e-10);
  EXPECT_NEAR(result.z(), 0.0, 1e-10);
}

TEST_F(Isometry3Test, Composition)
{
  Vector3d t1(1.0, 0.0, 0.0);
  Isometry3d iso1 = Isometry3d::fromTranslation(t1);

  Vector3d axis(0.0, 0.0, 1.0);
  Quaterniond q = Quaterniond::fromAxisAngle(axis, pi / 2.0);
  Isometry3d iso2 = Isometry3d::fromRotation(q);

  Isometry3d composed = iso2 * iso1;

  Vector3d p(0.0, 0.0, 0.0);
  Vector3d result = composed.transformPoint(p);

  EXPECT_NEAR(result.x(), 0.0, 1e-10);
  EXPECT_NEAR(result.y(), 1.0, 1e-10);
  EXPECT_NEAR(result.z(), 0.0, 1e-10);
}

TEST_F(Isometry3Test, Inverse)
{
  Vector3d axis(0.0, 0.0, 1.0);
  Quaterniond q = Quaterniond::fromAxisAngle(axis, pi / 4.0);
  Vector3d t(10.0, 20.0, 30.0);
  Isometry3d iso(q, t);

  Isometry3d inv = iso.inverse();
  Isometry3d composed = iso * inv;

  Vector3d p(5.0, 6.0, 7.0);
  Vector3d result = composed.transformPoint(p);

  EXPECT_NEAR(result.x(), p.x(), 1e-10);
  EXPECT_NEAR(result.y(), p.y(), 1e-10);
  EXPECT_NEAR(result.z(), p.z(), 1e-10);
}

TEST_F(Isometry3Test, EigenInterop)
{
  Eigen::Isometry3d eiso = Eigen::Isometry3d::Identity();
  eiso.rotate(Eigen::AngleAxisd(pi / 4.0, Eigen::Vector3d::UnitZ()));
  eiso.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  Isometry3d siso = Isometry3d::fromEigen(eiso);
  Eigen::Isometry3d back = siso.toEigen();

  Eigen::Vector3d p(1.0, 0.0, 0.0);
  Eigen::Vector3d expected = eiso * p;
  Eigen::Vector3d actual = back * p;

  EXPECT_NEAR(actual.x(), expected.x(), 1e-10);
  EXPECT_NEAR(actual.y(), expected.y(), 1e-10);
  EXPECT_NEAR(actual.z(), expected.z(), 1e-10);
}

TEST_F(Isometry3Test, Lerp)
{
  Isometry3d a = Isometry3d::identity();

  Vector3d axis(0.0, 0.0, 1.0);
  Quaterniond q = Quaterniond::fromAxisAngle(axis, pi / 2.0);
  Vector3d t(10.0, 0.0, 0.0);
  Isometry3d b(q, t);

  Isometry3d mid = lerp(a, b, 0.5);

  EXPECT_NEAR(mid.translation.x(), 5.0, 1e-10);
  EXPECT_NEAR(mid.translation.y(), 0.0, 1e-10);
  EXPECT_NEAR(mid.translation.z(), 0.0, 1e-10);
}
