/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include "dart/common/macro.hpp"
#include "dart/math/lie_group/r.hpp"

using namespace dart;
using namespace math;

//==============================================================================
template <typename T>
struct RTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double, long double>;

//==============================================================================
TYPED_TEST_SUITE(RTest, Types);

//==============================================================================
TYPED_TEST(RTest, Properties)
{
  using Scalar = typename TestFixture::Type;

  EXPECT_EQ(R0<Scalar>::GroupDim, 0);
  EXPECT_EQ(R1<Scalar>::GroupDim, 1);
  EXPECT_EQ(R2<Scalar>::GroupDim, 2);
  EXPECT_EQ(R3<Scalar>::GroupDim, 3);
  EXPECT_EQ(R4<Scalar>::GroupDim, 4);
  EXPECT_EQ(R5<Scalar>::GroupDim, 5);
  EXPECT_EQ(R6<Scalar>::GroupDim, 6);

  EXPECT_EQ(R0<Scalar>::MatrixDim, 0 + 1);
  EXPECT_EQ(R1<Scalar>::MatrixDim, 1 + 1);
  EXPECT_EQ(R2<Scalar>::MatrixDim, 2 + 1);
  EXPECT_EQ(R3<Scalar>::MatrixDim, 3 + 1);
  EXPECT_EQ(R4<Scalar>::MatrixDim, 4 + 1);
  EXPECT_EQ(R5<Scalar>::MatrixDim, 5 + 1);
  EXPECT_EQ(R6<Scalar>::MatrixDim, 6 + 1);
}

//==============================================================================
template <typename Scalar, int Dim>
void test_constructors_fixed_size()
{
  using RN = R<Scalar, Dim>;

  // Default constructor
  RN a;
  EXPECT_EQ(a.dimension(), Dim);

  // Copy constructor
  RN b = a;

  // Move constructor
  RN c = std::move(a);

  // From Eigen vectors
  RN d(Eigen::Matrix<Scalar, Dim, 1>::Random());
  Eigen::Matrix<Scalar, Dim, 1> vec = Eigen::Matrix<Scalar, Dim, 1>::Random();
  RN e = std::move(vec);
  EXPECT_TRUE(e.coeffs().isApprox(vec));

  // Using static functions
  RN f = RN::Zero();
  RN g = RN::Identity();
  RN h = RN::Random();

  DART_UNUSED(f, g, h);
}

//==============================================================================
TYPED_TEST(RTest, Constructors)
{
  using Scalar = typename TestFixture::Type;

  // Dynamic size
  {
    // Default constructor
    RX<Scalar> a;
    EXPECT_EQ(a.dimension(), 0);

    // Copy constructor
    RX<Scalar> b = a;

    // Move constructor
    RX<Scalar> c = std::move(a);
  }

  // Fixed size
  test_constructors_fixed_size<Scalar, 0>();
  test_constructors_fixed_size<Scalar, 1>();
  test_constructors_fixed_size<Scalar, 2>();
  test_constructors_fixed_size<Scalar, 3>();
  test_constructors_fixed_size<Scalar, 4>();
  test_constructors_fixed_size<Scalar, 5>();
  test_constructors_fixed_size<Scalar, 6>();
}

//==============================================================================
template <typename Scalar, int Dim>
void test_identity_fixed_size()
{
  using RN = R<Scalar, Dim>;

  RN a;
  a.set_identity();

  RN h = RN::Identity();
  (void)h;
}

//==============================================================================
TYPED_TEST(RTest, Identity)
{
  using Scalar = typename TestFixture::Type;

  RX<Scalar> a;
  a.set_identity();

  RX<Scalar> b(1);
  b.set_identity();

  RX<Scalar> h = RX<Scalar>::Identity();
  (void)h;

  test_identity_fixed_size<Scalar, 0>();
  test_identity_fixed_size<Scalar, 1>();
  test_identity_fixed_size<Scalar, 2>();
  test_identity_fixed_size<Scalar, 3>();
  test_identity_fixed_size<Scalar, 4>();
  test_identity_fixed_size<Scalar, 5>();
  test_identity_fixed_size<Scalar, 6>();
}

//==============================================================================
template <typename Scalar, int Dim>
void test_random_fixed_size()
{
  using RN = R<Scalar, Dim>;

  RN a;
  a.set_random();

  RN h = RN::Random();
  (void)h;
}

//==============================================================================
TYPED_TEST(RTest, Random)
{
  using Scalar = typename TestFixture::Type;

  RX<Scalar> a;
  a.set_random();

  RX<Scalar> b(1);
  b.set_random();

  RX<Scalar> h = RX<Scalar>::Random();
  (void)h;

  test_random_fixed_size<Scalar, 0>();
  test_random_fixed_size<Scalar, 1>();
  test_random_fixed_size<Scalar, 2>();
  test_random_fixed_size<Scalar, 3>();
  test_random_fixed_size<Scalar, 4>();
  test_random_fixed_size<Scalar, 5>();
  test_random_fixed_size<Scalar, 6>();
}
