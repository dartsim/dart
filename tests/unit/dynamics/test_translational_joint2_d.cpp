/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/translational_joint2_d.hpp>

#include <gtest/gtest.h>

using namespace dart::dynamics;

TEST(TranslationalJoint2DTest, PlaneTypesAndAxes)
{
  auto skeleton = Skeleton::create("trans2d");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<TranslationalJoint2D>();
  (void)body;

  joint->setXYPlane(false);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::XY);
  EXPECT_TRUE(
      joint->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      joint->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitY()));

  joint->setYZPlane(false);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::YZ);
  EXPECT_TRUE(
      joint->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(
      joint->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitZ()));

  joint->setZXPlane(false);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::ZX);
  EXPECT_TRUE(
      joint->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(
      joint->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitX()));

  joint->setArbitraryPlane(
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ(), false);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::ARBITRARY);
  EXPECT_TRUE(
      joint->getTranslationalAxis1().isApprox(Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(
      joint->getTranslationalAxis2().isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(TranslationalJoint2DTest, PropertiesAndJacobians)
{
  auto skeleton = Skeleton::create("trans2d_props");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<TranslationalJoint2D>();
  (void)body;

  EXPECT_EQ(joint->getType(), TranslationalJoint2D::getStaticType());
  EXPECT_FALSE(joint->isCyclic(0));
  EXPECT_FALSE(joint->isCyclic(1));

  TranslationalJoint2D::UniqueProperties uniqueProps(
      TranslationalJoint2D::PlaneType::YZ);
  joint->setProperties(uniqueProps);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::YZ);

  TranslationalJoint2D::Properties props;
  props.setZXPlane();
  joint->setProperties(props);
  EXPECT_EQ(joint->getPlaneType(), TranslationalJoint2D::PlaneType::ZX);

  auto xyProps = joint->getTranslationalJoint2DProperties();
  xyProps.setXYPlane();
  joint->setProperties(xyProps);
  Eigen::VectorXd positions(2);
  positions << 1.0, -2.0;
  joint->setPositions(positions);
  const Eigen::Vector3d expectedTranslation(1.0, -2.0, 0.0);
  EXPECT_TRUE(joint->getRelativeTransform().translation().isApprox(
      expectedTranslation));

  const auto jacobian = joint->getRelativeJacobian();
  EXPECT_TRUE(jacobian.topRows<3>().isZero(0.0));
  Eigen::Matrix<double, 3, 2> expectedJacobian;
  expectedJacobian.col(0) = joint->getTranslationalAxis1();
  expectedJacobian.col(1) = joint->getTranslationalAxis2();
  expectedJacobian
      = joint->getTransformFromChildBodyNode().linear() * expectedJacobian;
  EXPECT_TRUE(jacobian.bottomRows<3>().isApprox(expectedJacobian));

  const auto jacobianDeriv = joint->getRelativeJacobianTimeDeriv();
  EXPECT_TRUE(jacobianDeriv.isZero(0.0));
}
