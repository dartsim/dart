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

#include "helpers/gtest_utils.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/constants.hpp>
#include <dart/math/geometry.hpp>

#include <gtest/gtest.h>

#include <optional>

using namespace dart;
using namespace dart::dynamics;

namespace {

SkeletonPtr makeSkeleton()
{
  auto skel = Skeleton::create();
  skel->createJointAndBodyNodePair<WeldJoint>(nullptr);
  return skel;
}

} // namespace

//==============================================================================
TEST(ShapeNodeInertia, AppliesRelativeTransform)
{
  const auto skel = makeSkeleton();
  auto* body = skel->getBodyNode(0);
  const auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.2, 0.1, 0.3));
  auto* node = body->createShapeNodeWith<VisualAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.5, -0.2, 0.1);
  tf.linear() = Eigen::AngleAxisd(math::pi / 4.0, Eigen::Vector3d::UnitZ())
                    .toRotationMatrix();
  node->setRelativeTransform(tf);

  const double mass = 3.0;
  const Inertia inertia = node->computeTransformedInertia(mass);

  const Eigen::Matrix3d localMoment = shape->computeInertia(mass);
  const Eigen::Matrix3d expectedMoment
      = tf.linear() * localMoment * tf.linear().transpose();

  EXPECT_VECTOR_DOUBLE_EQ(tf.translation(), inertia.getLocalCOM());
  EXPECT_MATRIX_DOUBLE_EQ(expectedMoment, inertia.getMoment());
}

//==============================================================================
TEST(ShapeNodeInertia, BodyNodeAggregationMatchesManualSum)
{
  const auto skel = makeSkeleton();
  auto* body = skel->getBodyNode(0);

  const auto shapeA
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.2, 0.3));
  const auto shapeB
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.15, 0.25, 0.05));

  auto* nodeA = body->createShapeNodeWith<VisualAspect>(shapeA);
  auto* nodeB = body->createShapeNodeWith<VisualAspect>(shapeB);

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  tfA.translation() = Eigen::Vector3d(0.1, 0.0, 0.05);
  nodeA->setRelativeTransform(tfA);

  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(-0.05, 0.02, -0.03);
  tfB.linear() = Eigen::AngleAxisd(math::pi / 3.0, Eigen::Vector3d::UnitY())
                     .toRotationMatrix();
  nodeB->setRelativeTransform(tfB);

  const double massA = 2.0;
  const double massB = 4.0;

  const auto combined = body->computeInertiaFromShapeNodes(
      [&](const ShapeNode* node) -> std::optional<double> {
        if (node == nodeA) {
          return massA;
        }
        if (node == nodeB) {
          return massB;
        }
        return std::nullopt;
      });
  ASSERT_TRUE(combined.has_value());

  const Eigen::Matrix6d expectedSpatial
      = nodeA->computeTransformedInertia(massA).getSpatialTensor()
        + nodeB->computeTransformedInertia(massB).getSpatialTensor();

  EXPECT_MATRIX_DOUBLE_EQ(expectedSpatial, combined->getSpatialTensor());
}

//==============================================================================
TEST(ShapeNodeInertia, DensityConversionMatchesExplicitMass)
{
  const auto skel = makeSkeleton();
  auto* body = skel->getBodyNode(0);
  const auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.2, 0.3));
  auto* node = body->createShapeNodeWith<VisualAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.05, -0.02, 0.01);
  tf.linear() = Eigen::AngleAxisd(math::pi / 6.0, Eigen::Vector3d::UnitX())
                    .toRotationMatrix();
  node->setRelativeTransform(tf);

  const double density = 7.5;
  const double expectedMass = density * shape->getVolume();

  const Inertia fromDensity
      = node->computeTransformedInertiaFromDensity(density);
  const Inertia fromMass = node->computeTransformedInertia(expectedMass);

  EXPECT_DOUBLE_EQ(expectedMass, fromDensity.getMass());
  EXPECT_VECTOR_DOUBLE_EQ(fromMass.getLocalCOM(), fromDensity.getLocalCOM());
  EXPECT_MATRIX_DOUBLE_EQ(fromMass.getMoment(), fromDensity.getMoment());
}

//==============================================================================
TEST(ShapeNodeInertia, BodyNodeAggregationDropsInvalidMasses)
{
  const auto skel = makeSkeleton();
  auto* body = skel->getBodyNode(0);
  const auto shape = std::make_shared<BoxShape>(Eigen::Vector3d::Ones() * 0.1);
  auto* node = body->createShapeNodeWith<VisualAspect>(shape);

  const auto zeroMass = body->computeInertiaFromShapeNodes(
      [&](const ShapeNode* candidate) -> std::optional<double> {
        EXPECT_EQ(node, candidate);
        return 0.0;
      });
  EXPECT_FALSE(zeroMass.has_value());

  const auto missingMass = body->computeInertiaFromShapeNodes(
      [&](const ShapeNode*) -> std::optional<double> { return std::nullopt; });
  EXPECT_FALSE(missingMass.has_value());
}
