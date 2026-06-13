/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "helpers/gtest_utils.hpp"

#include "dart/math/lie_groups.hpp"

#include <gtest/gtest.h>

#include <type_traits>

using namespace dart;
using namespace math;

template <typename S>
struct GroupProductTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(GroupProductTest, Types);

template <typename S>
using SE3xSO3 = GroupProduct<S, SE3, SO3>;

template <typename S>
using SevenDofFloatingArm = GroupProduct<S, SE3, R1, R1, R1, R1, R1, R1, R1>;

template <typename S>
using HumanoidConfiguration = GroupProduct<
    S,
    SE3,
    SO3,
    SO3,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1,
    R1>;

constexpr std::size_t kHumanoidScalarStart = 3;
constexpr std::size_t kHumanoidScalarJointCount = 21;

//==============================================================================
TYPED_TEST(GroupProductTest, StaticProperties)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  static_assert(std::is_same_v<typename Product::PlainObject, Product>);
  static_assert(std::is_same_v<
                typename Product::InverseType,
                GroupProductInverse<Product>>);

  EXPECT_EQ(Product::ProductSize, 2u);
  EXPECT_EQ(Product::ParamSize, SE3<S>::ParamSize + SO3<S>::ParamSize);
  EXPECT_EQ(Product::Dim, SE3<S>::Dim + SO3<S>::Dim);
  EXPECT_EQ(Product::DoF, SE3<S>::DoF + SO3<S>::DoF);
  EXPECT_EQ(Product::MatrixRepDim, SE3<S>::MatrixRepDim + SO3<S>::MatrixRepDim);
  EXPECT_EQ(Product::ParamSizes[0], SE3<S>::ParamSize);
  EXPECT_EQ(Product::ParamSizes[1], SO3<S>::ParamSize);
  EXPECT_EQ(Product::ParamSizeIndices[0], 0);
  EXPECT_EQ(Product::ParamSizeIndices[1], SE3<S>::ParamSize);
  EXPECT_EQ(Product::Tangent::RowsAtCompileTime, Product::DoF);
}

//==============================================================================
TYPED_TEST(GroupProductTest, RnModelsScalarAndVectorJointFactors)
{
  using S = typename TestFixture::Scalar;
  using Joint = R1<S>;
  using JointVector = R6<S>;

  const Joint a(S(0.25));
  const Joint b(S(-0.5));
  const Joint composed = a * b;

  EXPECT_EQ(Joint::Dim, 1);
  EXPECT_EQ(Joint::DoF, 1);
  EXPECT_EQ(Joint::ParamSize, 1);
  EXPECT_NEAR(composed.params()[0], S(-0.25), LieGroupTol<S>());
  EXPECT_TRUE((a * a.inverse()).isIdentity());
  EXPECT_TRUE(a.inverse().inverse().isApprox(a));
  EXPECT_NEAR(a.log()[0], S(0.25), LieGroupTol<S>());
  EXPECT_TRUE(a.ad(a.log()).isApprox(a.log()));

  const auto matrix = a.toMatrix();
  EXPECT_TRUE((matrix.template topLeftCorner<1, 1>().isIdentity()));
  EXPECT_NEAR(matrix(0, 1), S(0.25), LieGroupTol<S>());
  EXPECT_NEAR(matrix(1, 1), S(1), LieGroupTol<S>());
  EXPECT_TRUE(a.toAdjointMatrix().isIdentity());

  typename JointVector::Params jointParams;
  jointParams << S(0.1), S(0.2), S(0.3), S(0.4), S(0.5), S(0.6);
  const JointVector joints(jointParams);
  EXPECT_TRUE(joints.log().isApprox(joints.params()));
  EXPECT_TRUE((joints * joints.inverse()).isIdentity());
}

//==============================================================================
TYPED_TEST(GroupProductTest, DefaultConstructorInitializesIdentityComponents)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  const Product product;

  EXPECT_TRUE(product.template get<0>().isApprox(SE3<S>::Identity()));
  EXPECT_TRUE(product.template get<1>().isApprox(SO3<S>::Identity()));
  EXPECT_TRUE(product.isIdentity());
  EXPECT_TRUE(product.log().isZero(LieGroupTol<S>()));
}

//==============================================================================
TYPED_TEST(GroupProductTest, RandomAndNestedProductsExposeComponents)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;
  using Nested = GroupProduct<S, SE3, SE3xSO3, SO3>;

  const Product product = Product::Random();
  EXPECT_TRUE(
      product.template get<0>().isApprox(SE3<S>(product.template params<0>())));
  EXPECT_TRUE(
      product.template get<1>().isApprox(SO3<S>(product.template params<1>())));

  const Nested nested = Nested::Random();
  EXPECT_TRUE(
      nested.template get<0>().isApprox(SE3<S>(nested.template params<0>())));
  EXPECT_TRUE(nested.template get<1>().template get<0>().isApprox(
      SE3<S>(nested.template get<1>().template params<0>())));
  EXPECT_TRUE(nested.template get<1>().template get<1>().isApprox(
      SO3<S>(nested.template get<1>().template params<1>())));
  EXPECT_TRUE(
      nested.template get<2>().isApprox(SO3<S>(nested.template params<2>())));
}

//==============================================================================
TYPED_TEST(GroupProductTest, ConstructAssignAndCast)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  const SE3<S> se3 = SE3<S>::Random();
  const SO3<S> so3 = SO3<S>::Random();
  const Product product(se3, so3);

  EXPECT_TRUE(product.template get<0>().isApprox(se3));
  EXPECT_TRUE(product.template get<1>().isApprox(so3));
  EXPECT_TRUE(product.params(0).isApprox(se3.params()));
  EXPECT_TRUE(product.params(1).isApprox(so3.params()));

  Product assigned;
  assigned = product;
  EXPECT_TRUE(assigned.isApprox(product));

  Product moveAssigned;
  Product copy(product);
  moveAssigned = std::move(copy);
  EXPECT_TRUE(moveAssigned.isApprox(product));

  const auto asFloat = product.template cast<float>();
  const auto asDouble = product.template cast<double>();
  EXPECT_TRUE(asFloat.params().isApprox(
      product.params().template cast<float>(), LieGroupTol<S>()));
  EXPECT_TRUE(asDouble.params().isApprox(
      product.params().template cast<double>(), LieGroupTol<S>()));
}

//==============================================================================
TYPED_TEST(GroupProductTest, ComposeAndInverseAreComponentwise)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  const SE3<S> a0 = SE3<S>::Random();
  const SO3<S> a1 = SO3<S>::Random();
  const SE3<S> b0 = SE3<S>::Random();
  const SO3<S> b1 = SO3<S>::Random();

  const Product a(a0, a1);
  const Product b(b0, b1);
  const Product composed = a * b;

  EXPECT_TRUE(composed.template get<0>().isApprox(a0 * b0));
  EXPECT_TRUE(composed.template get<1>().isApprox(a1 * b1));

  const auto inverseExpr = a.inverse();
  const Product inverse = inverseExpr;
  const SE3<S> a0Inverse = a0.inverse();
  const SO3<S> a1Inverse = a1.inverse();
  EXPECT_TRUE(inverse.template get<0>().isApprox(a0Inverse));
  EXPECT_TRUE(inverse.template get<1>().isApprox(a1Inverse));
  EXPECT_TRUE((inverseExpr * a).isIdentity());
  EXPECT_TRUE((a * a.inverse()).isIdentity());
  EXPECT_TRUE(a.inverse().inverse().isApprox(a));
}

//==============================================================================
TYPED_TEST(GroupProductTest, RvalueInverseMaterializes)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  static_assert(std::is_same_v<
                decltype(Product::Random().inverse()),
                typename Product::LieGroup>);

  const Product product = Product::Random();
  const Product inverse = Product(product).inverse();
  const Product expected = product.inverse();
  EXPECT_TRUE(inverse.isApprox(expected));
}

//==============================================================================
TYPED_TEST(GroupProductTest, MatrixRepresentationsAreBlockDiagonal)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  const SE3<S> se3 = SE3<S>::Random();
  const SO3<S> so3 = SO3<S>::Random();
  const Product product(se3, so3);

  typename Product::MatrixType expectedMatrix = Product::MatrixType::Zero();
  expectedMatrix.template block<SE3<S>::MatrixRepDim, SE3<S>::MatrixRepDim>(
      0, 0) = se3.toMatrix();
  expectedMatrix.template block<SO3<S>::MatrixRepDim, SO3<S>::MatrixRepDim>(
      SE3<S>::MatrixRepDim, SE3<S>::MatrixRepDim) = so3.toMatrix();
  EXPECT_TRUE(product.toMatrix().isApprox(expectedMatrix));

  Matrix<S, Product::DoF, Product::DoF> expectedAdjoint
      = Matrix<S, Product::DoF, Product::DoF>::Zero();
  expectedAdjoint.template block<SE3<S>::DoF, SE3<S>::DoF>(0, 0)
      = se3.toAdjointMatrix();
  expectedAdjoint.template block<SO3<S>::DoF, SO3<S>::DoF>(
      SE3<S>::DoF, SE3<S>::DoF) = so3.toAdjointMatrix();
  EXPECT_TRUE(product.toAdjointMatrix().isApprox(expectedAdjoint));
}

//==============================================================================
TYPED_TEST(GroupProductTest, LogJacobianIsComponentwiseBlockDiagonal)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;
  using RnProduct = GroupProduct<S, R1, R6>;

  const Product product = Product::Random();
  Matrix<S, Product::DoF, Product::DoF> jacobian;
  const typename Product::Tangent tangent = product.log(&jacobian);

  Matrix<S, Product::DoF, Product::DoF> expectedJacobian
      = Matrix<S, Product::DoF, Product::DoF>::Zero();
  Matrix6<S> se3Jacobian;
  Matrix3<S> so3Jacobian;
  const SE3Tangent<S> se3Log = product.template get<0>().log(&se3Jacobian);
  const SO3Tangent<S> so3Log = product.template get<1>().log(&so3Jacobian);
  expectedJacobian.template block<SE3<S>::DoF, SE3<S>::DoF>(0, 0) = se3Jacobian;
  expectedJacobian.template block<SO3<S>::DoF, SO3<S>::DoF>(
      SE3<S>::DoF, SE3<S>::DoF) = so3Jacobian;

  typename Product::Tangent expectedTangent;
  expectedTangent.template head<SE3<S>::DoF>() = se3Log.params();
  expectedTangent.template tail<SO3<S>::DoF>() = so3Log.params();

  EXPECT_TRUE(tangent.isApprox(expectedTangent, LieGroupTol<S>()));
  EXPECT_TRUE(jacobian.isApprox(expectedJacobian, LieGroupTol<S>()));

  const RnProduct rnProduct = RnProduct::Random();
  Matrix<S, RnProduct::DoF, RnProduct::DoF> rnJacobian;
  const typename RnProduct::Tangent rnTangent = rnProduct.log(&rnJacobian);
  EXPECT_TRUE(rnTangent.isApprox(rnProduct.params(), LieGroupTol<S>()));
  EXPECT_TRUE(rnJacobian.isIdentity(LieGroupTol<S>()));
}

//==============================================================================
TYPED_TEST(GroupProductTest, MapReferencesUnderlyingParameters)
{
  using S = typename TestFixture::Scalar;
  using Product = SE3xSO3<S>;

  Product source = Product::Random();
  typename Product::Params params = source.params();

  const Map<const Product> constMap(params.data());
  EXPECT_TRUE(constMap.params().isApprox(params));
  EXPECT_TRUE(constMap.template get<0>().params().isApprox(
      source.template get<0>().params()));
  EXPECT_TRUE(constMap.template get<1>().params().isApprox(
      source.template get<1>().params()));

  Map<Product> map(params.data());
  const SO3<S> replacement = SO3<S>::Random();
  map.template get<1>() = replacement;

  EXPECT_TRUE(map.template get<1>().isApprox(replacement));
  EXPECT_TRUE(params.template segment<SO3<S>::ParamSize>(SE3<S>::ParamSize)
                  .isApprox(replacement.params()));
}

//==============================================================================
TYPED_TEST(GroupProductTest, StridedHumanoidMapReferencesComponents)
{
  using S = typename TestFixture::Scalar;
  using Humanoid = HumanoidConfiguration<S>;

  constexpr int kStride = 2;
  constexpr S kSentinel = S(-123);

  const Humanoid source = Humanoid::Random();
  const auto makeStorage = [&] {
    Eigen::Matrix<S, Humanoid::ParamSize * kStride, 1> storage;
    storage.setConstant(kSentinel);
    for (int i = 0; i < Humanoid::ParamSize; ++i) {
      storage[i * kStride] = source.params()[i];
    }
    return storage;
  };

  const auto verifyStridedMap = [&](const auto& stride) {
    using Stride = std::decay_t<decltype(stride)>;
    auto storage = makeStorage();

    const Eigen::Map<const Humanoid, Eigen::Unaligned, Stride> constMap(
        storage.data(), stride);
    EXPECT_TRUE(constMap.template get<0>().params().isApprox(
        source.template get<0>().params(), LieGroupTol<S>()));
    EXPECT_TRUE(constMap.template get<1>().params().isApprox(
        source.template get<1>().params(), LieGroupTol<S>()));
    EXPECT_TRUE(constMap.template get<2>().params().isApprox(
        source.template get<2>().params(), LieGroupTol<S>()));
    EXPECT_NEAR(
        constMap.template get<kHumanoidScalarStart>().params()[0],
        source.template get<kHumanoidScalarStart>().params()[0],
        LieGroupTol<S>());
    EXPECT_TRUE(constMap.log().isApprox(source.log(), LieGroupTol<S>()));

    Eigen::Map<Humanoid, Eigen::Unaligned, Stride> map(storage.data(), stride);
    map.inverseInPlace();
    const Humanoid expectedInverse = source.inverse();
    EXPECT_TRUE(map.isApprox(expectedInverse));
    for (int i = 0; i < Humanoid::ParamSize; ++i) {
      EXPECT_NEAR(
          storage[i * kStride], expectedInverse.params()[i], LieGroupTol<S>());
      EXPECT_EQ(storage[i * kStride + 1], kSentinel);
    }

    const SO3<S> replacement = SO3<S>::Random();
    map.template get<1>() = replacement;
    EXPECT_TRUE(map.template get<1>().isApprox(replacement));
    for (int i = 0; i < SO3<S>::ParamSize; ++i) {
      EXPECT_NEAR(
          storage[(SE3<S>::ParamSize + i) * kStride],
          replacement.params()[i],
          LieGroupTol<S>());
    }
    for (int i = 0; i < Humanoid::ParamSize; ++i) {
      EXPECT_EQ(storage[i * kStride + 1], kSentinel);
    }
  };

  verifyStridedMap(Eigen::InnerStride<>(kStride));
  verifyStridedMap(
      Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(
          Humanoid::ParamSize * kStride, kStride));
}

//==============================================================================
TYPED_TEST(GroupProductTest, RealWorldFloatingArmUsesScalarJointProduct)
{
  using S = typename TestFixture::Scalar;
  using Arm = SevenDofFloatingArm<S>;

  static_assert(std::is_same_v<typename Arm::template Component<0>, SE3<S>>);
  static_assert(std::is_same_v<typename Arm::template Component<1>, R1<S>>);

  EXPECT_EQ(Arm::ProductSize, 8u);
  EXPECT_EQ(Arm::DoF, SE3<S>::DoF + 7);
  EXPECT_EQ(Arm::ParamSize, SE3<S>::ParamSize + 7);

  Arm q = Arm::Identity();
  q.template get<0>() = SE3<S>::Random();
  for (std::size_t i = 1; i < Arm::ProductSize; ++i) {
    q.params(i)[0] = S(0.1) * static_cast<S>(i);
  }

  Arm dq = Arm::Identity();
  dq.template get<0>() = SE3<S>::Random();
  for (std::size_t i = 1; i < Arm::ProductSize; ++i) {
    dq.params(i)[0] = S(-0.01) * static_cast<S>(i);
  }

  const Arm next = q * dq;
  EXPECT_TRUE(next.template get<0>().isApprox(
      q.template get<0>() * dq.template get<0>()));
  for (std::size_t i = 1; i < Arm::ProductSize; ++i) {
    EXPECT_NEAR(
        next.params(i)[0], q.params(i)[0] + dq.params(i)[0], LieGroupTol<S>());
  }
  EXPECT_TRUE((next * next.inverse()).isIdentity());
}

//==============================================================================
TYPED_TEST(GroupProductTest, RealWorldHumanoidConfigurationIsComponentwise)
{
  using S = typename TestFixture::Scalar;
  using Humanoid = HumanoidConfiguration<S>;

  static_assert(
      std::is_same_v<typename Humanoid::template Component<0>, SE3<S>>);
  static_assert(
      std::is_same_v<typename Humanoid::template Component<1>, SO3<S>>);
  static_assert(
      std::is_same_v<typename Humanoid::template Component<2>, SO3<S>>);
  static_assert(std::is_same_v<
                typename Humanoid::template Component<kHumanoidScalarStart>,
                R1<S>>);
  static_assert(
      Humanoid::ParamSize
      == SE3<S>::ParamSize + 2 * SO3<S>::ParamSize
             + static_cast<int>(kHumanoidScalarJointCount));

  EXPECT_EQ(
      Humanoid::ProductSize, kHumanoidScalarStart + kHumanoidScalarJointCount);
  EXPECT_EQ(
      Humanoid::DoF,
      SE3<S>::DoF + 2 * SO3<S>::DoF
          + static_cast<int>(kHumanoidScalarJointCount));
  EXPECT_EQ(
      Humanoid::ParamSize,
      SE3<S>::ParamSize + 2 * SO3<S>::ParamSize
          + static_cast<int>(kHumanoidScalarJointCount));

  Humanoid q = Humanoid::Identity();
  q.template get<0>() = SE3<S>::Random();
  q.template get<1>() = SO3<S>::Random();
  q.template get<2>() = SO3<S>::Random();
  for (std::size_t i = kHumanoidScalarStart; i < Humanoid::ProductSize; ++i) {
    q.params(i)[0] = S(0.02) * static_cast<S>(i - kHumanoidScalarStart + 1);
  }

  Humanoid dq = Humanoid::Identity();
  dq.template get<0>() = SE3<S>::Random();
  dq.template get<1>() = SO3<S>::Random();
  dq.template get<2>() = SO3<S>::Random();
  for (std::size_t i = kHumanoidScalarStart; i < Humanoid::ProductSize; ++i) {
    dq.params(i)[0] = S(-0.005) * static_cast<S>(i - kHumanoidScalarStart + 1);
  }

  const Humanoid next = q * dq;
  EXPECT_TRUE(next.template get<0>().isApprox(
      q.template get<0>() * dq.template get<0>()));
  EXPECT_TRUE(next.template get<1>().isApprox(
      q.template get<1>() * dq.template get<1>()));
  EXPECT_TRUE(next.template get<2>().isApprox(
      q.template get<2>() * dq.template get<2>()));

  const auto tangent = next.log();
  constexpr int scalarOffset = SE3<S>::DoF + 2 * SO3<S>::DoF;
  for (std::size_t i = kHumanoidScalarStart; i < Humanoid::ProductSize; ++i) {
    const auto scalarIndex = static_cast<int>(i - kHumanoidScalarStart);
    EXPECT_NEAR(
        next.params(i)[0], q.params(i)[0] + dq.params(i)[0], LieGroupTol<S>());
    EXPECT_NEAR(
        tangent[scalarOffset + scalarIndex],
        next.params(i)[0],
        LieGroupTol<S>());
  }

  typename Humanoid::Params params = next.params();
  Map<Humanoid> map(params.data());
  map.template get<kHumanoidScalarStart>() = R1<S>(S(1.25));
  EXPECT_NEAR(params[SE3<S>::ParamSize + 2 * SO3<S>::ParamSize], S(1.25), 0);
  EXPECT_TRUE((next * next.inverse()).isIdentity());
}
