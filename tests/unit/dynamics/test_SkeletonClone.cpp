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

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/InverseKinematics.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>

#include <Eigen/Core>
#include <assimp/mesh.h>
#include <gtest/gtest.h>

using namespace dart::dynamics;

class SoftBodyCopy : public ::testing::TestWithParam<bool>
{
};

//==============================================================================
TEST_P(SoftBodyCopy, SkinMeshTracksResizedPointMasses)
{
  auto original = Skeleton::create("soft_original");
  const auto softProperties = SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d(0.2, 0.2, 0.1), Eigen::Isometry3d::Identity(), 0.5);
  const SoftBodyNode::Properties properties(
      BodyNode::Properties(BodyNode::AspectProperties("skin_body")),
      softProperties);
  auto* soft = original
                   ->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
                       nullptr, FreeJoint::Properties(), properties)
                   .second;
  auto* skin = soft->getShapeNode(0);
  skin->setName("deformable_skin");
  const Eigen::Vector4d color(0.2, 0.4, 0.6, 0.35);
  skin->getVisualAspect()->setColor(color);
  soft->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.05)));

  SkeletonPtr destination;
  if (GetParam()) {
    destination = soft->copyAs("copy_as");
  } else {
    destination = Skeleton::create("copy_to");
    soft->copyTo(destination, nullptr);
  }
  auto* copiedSoft = destination->getSoftBodyNode(0);
  ASSERT_NE(copiedSoft, nullptr);
  ASSERT_EQ(copiedSoft->getNumShapeNodes(), soft->getNumShapeNodes());
  auto* copiedSkin = copiedSoft->getShapeNode(0);
  EXPECT_EQ(copiedSkin->getName(), skin->getName());
  EXPECT_TRUE(copiedSkin->getVisualAspect()->getRGBA().isApprox(color));
  const auto mesh
      = std::dynamic_pointer_cast<SoftMeshShape>(copiedSkin->getShape());
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->getSoftBodyNode(), copiedSoft);
  EXPECT_NE(mesh, skin->getShape());
  original.reset();

  const auto largerProperties = SoftBodyNodeHelper::makeEllipsoidProperties(
      Eigen::Vector3d(0.2, 0.2, 0.2), 6, 6, 0.5);
  ASSERT_GT(
      largerProperties.mPointProps.size(), softProperties.mPointProps.size());
  for (const auto& resizedProperties : {largerProperties, softProperties}) {
    copiedSoft->setProperties(resizedProperties);
    ASSERT_EQ(
        copiedSoft->getNumPointMasses(), resizedProperties.mPointProps.size());
    ASSERT_NE(mesh->getAssimpMesh(), nullptr);
    // Check allocation before update() would write past a stale vertex buffer.
    ASSERT_EQ(
        mesh->getAssimpMesh()->mNumVertices, copiedSoft->getNumPointMasses());
    copiedSoft->getPointMass(0)->setPositions(
        Eigen::Vector3d(0.01, 0.02, 0.03));
    mesh->update();
    const auto& vertex = mesh->getAssimpMesh()->mVertices[0];
    EXPECT_TRUE(
        Eigen::Vector3d(vertex.x, vertex.y, vertex.z)
            .isApprox(copiedSoft->getPointMass(0)->getLocalPosition(), 1e-6));
  }
}

INSTANTIATE_TEST_SUITE_P(CopyToAndCopyAs, SoftBodyCopy, ::testing::Bool());

//==============================================================================
TEST(Issue896, SkeletonCloneDeepCopiesShapes)
{
  const auto skel = Skeleton::create("original");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;

  const auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  body->createShapeNodeWith<VisualAspect, CollisionAspect>(box);

  const auto clone = skel->cloneSkeleton();
  ASSERT_TRUE(clone);

  auto* clonedBody = clone->getBodyNode(body->getName());
  ASSERT_NE(clonedBody, nullptr);
  // Use first shape node; cast checked below
  auto* clonedShapeNode = clonedBody->getShapeNodeWith<VisualAspect>(0);
  ASSERT_NE(clonedShapeNode, nullptr);

  const auto originalBox = std::dynamic_pointer_cast<BoxShape>(box);
  ASSERT_NE(originalBox, nullptr);
  const auto clonedBox
      = std::dynamic_pointer_cast<BoxShape>(clonedShapeNode->getShape());
  ASSERT_NE(clonedBox, nullptr);

  EXPECT_NE(originalBox.get(), clonedBox.get());

  const auto originalSize = originalBox->getSize();
  const Eigen::Vector3d clonedSize(0.25, 0.5, 0.75);

  clonedBox->setSize(clonedSize);
  EXPECT_EQ(originalBox->getSize(), originalSize);
  EXPECT_EQ(clonedBox->getSize(), clonedSize);
}

//==============================================================================
TEST(SkeletonClone, SoftMeshRetainsDestinationOwnerAndShapeProperties)
{
  auto original = Skeleton::create("soft_original");
  const auto softProperties = SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d(0.2, 0.2, 0.1),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(4, 4, 4),
      0.5);
  const SoftBodyNode::Properties properties(
      BodyNode::Properties(BodyNode::AspectProperties("skin_body")),
      softProperties);
  auto* soft = original
                   ->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
                       nullptr, FreeJoint::Properties(), properties)
                   .second;
  auto* skin = soft->getShapeNode(0);
  skin->setName("deformable_skin");
  const Eigen::Vector4d color(0.2, 0.4, 0.6, 0.35);
  skin->getVisualAspect()->setColor(color);
  skin->getDynamicsAspect()->setRestitutionCoeff(0.6);
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.01, 0.02, 0.03);
  skin->setRelativeTransform(offset);
  skin->createIK()->setOffset(Eigen::Vector3d(0.02, 0.01, 0.03));

  const auto core = std::make_shared<BoxShape>(Eigen::Vector3d(0.1, 0.1, 0.05));
  soft->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      core);
  original->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr, FreeJoint::Properties(), properties);
  ASSERT_EQ(original->getNumShapeNodes(), 3u);

  auto clone = original->cloneSkeleton();
  ASSERT_EQ(clone->getNumShapeNodes(), original->getNumShapeNodes());
  auto* clonedSoft = clone->getSoftBodyNode(0);
  ASSERT_EQ(clonedSoft->getNumShapeNodes(), soft->getNumShapeNodes());
  auto* clonedSkin = clonedSoft->getShapeNode(0);
  ASSERT_NE(clonedSkin, nullptr);
  const auto clonedMesh
      = std::dynamic_pointer_cast<SoftMeshShape>(clonedSkin->getShape());
  ASSERT_NE(clonedMesh, nullptr);
  EXPECT_EQ(clonedMesh->getSoftBodyNode(), clonedSoft);
  EXPECT_NE(clonedSkin->getShape(), skin->getShape());
  EXPECT_EQ(clonedSkin->getName(), skin->getName());
  EXPECT_TRUE(clonedSkin->getRelativeTransform().isApprox(offset));
  EXPECT_TRUE(clonedSkin->getVisualAspect()->getRGBA().isApprox(color));
  EXPECT_DOUBLE_EQ(clonedSkin->getDynamicsAspect()->getRestitutionCoeff(), 0.6);
  EXPECT_TRUE(clonedSkin->hasCollisionAspect());
  ASSERT_NE(clonedSkin->getIK(), nullptr);
  EXPECT_EQ(clonedSkin->getIK()->getNode(), clonedSkin);
  EXPECT_NE(clonedSkin->getIK(), skin->getIK());
  EXPECT_EQ(clonedSkin->getIK()->getOffset(), skin->getIK()->getOffset());
  EXPECT_NE(clonedSoft->getShapeNode(1)->getShape(), core);
  for (std::size_t i = 0; i < original->getNumShapeNodes(); ++i) {
    EXPECT_EQ(
        clone->getShapeNode(i)->getName(),
        original->getShapeNode(i)->getName());
    EXPECT_EQ(clone->getShapeNode(i)->getIndexInSkeleton(), i);
  }

  original.reset();
  clonedSoft->setProperties(SoftBodyNodeHelper::makeEllipsoidProperties(
      Eigen::Vector3d(0.2, 0.2, 0.2), 6, 6, 0.5));
  EXPECT_EQ(clonedSoft->getNumPointMasses(), 32u);
  ASSERT_NE(clonedMesh->getAssimpMesh(), nullptr);
  EXPECT_EQ(clonedMesh->getAssimpMesh()->mNumVertices, 32u);
  dart::simulation::World world;
  world.addSkeleton(clone);
  for (std::size_t step = 0; step < 5; ++step)
    world.step();
  for (std::size_t i = 0; i < clone->getNumSoftBodyNodes(); ++i) {
    const auto* body = clone->getSoftBodyNode(i);
    for (std::size_t j = 0; j < body->getNumPointMasses(); ++j) {
      EXPECT_TRUE(body->getPointMass(j)->getPositions().allFinite());
      EXPECT_TRUE(body->getPointMass(j)->getWorldVelocity().allFinite());
    }
  }
}
