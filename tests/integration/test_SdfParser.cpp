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

#include "TestHelpers.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/sdf/SdfParser.hpp"

#include <gtest/gtest.h>

#include <iostream>
#if HAVE_BULLET
  #include "dart/collision/bullet/bullet.hpp"
#endif

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
TEST(SdfParser, SDFSingleBodyWithoutJoint)
{
  // Regression test for #444
  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/single_bodynode_skeleton.world");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel = world->getSkeleton(0);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 1u);
  EXPECT_EQ(skel->getNumJoints(), 1u);

  BodyNodePtr bodyNode = skel->getBodyNode(0);
  EXPECT_TRUE(bodyNode != nullptr);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<VisualAspect>(), 1u);
  EXPECT_EQ(bodyNode->getNumShapeNodesWith<CollisionAspect>(), 1u);

  JointPtr joint = skel->getJoint(0);
  EXPECT_TRUE(joint != nullptr);
  EXPECT_EQ(joint->getType(), FreeJoint::getStaticType());
}

//==============================================================================
TEST(SdfParser, SDFJointProperties)
{
  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/test_skeleton_joint.world");
  EXPECT_TRUE(world != nullptr);

  SkeletonPtr skel = world->getSkeleton(0);
  EXPECT_TRUE(skel != nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 5u);
  EXPECT_EQ(skel->getNumJoints(), 5u);

  const double epsilon = 1e-7;

  auto testProperties = [epsilon](const Joint* joint, const size_t idx) {
    EXPECT_TRUE(joint->areLimitsEnforced()) << joint->getName();
    EXPECT_NEAR(joint->getPositionLowerLimit(idx), 0, epsilon);
    EXPECT_NEAR(joint->getPositionUpperLimit(idx), 3, epsilon);
    EXPECT_NEAR(joint->getDampingCoefficient(idx), 0, epsilon);
    EXPECT_NEAR(joint->getCoulombFriction(idx), 1, epsilon);
    EXPECT_NEAR(joint->getRestPosition(idx), 2, epsilon);
    EXPECT_NEAR(joint->getSpringStiffness(idx), 3, epsilon);
  };

  for (auto& joint : skel->getJoints()) {
    if (joint->getType() == PrismaticJoint::getStaticType()
        || joint->getType() == RevoluteJoint::getStaticType()
        || joint->getType() == ScrewJoint::getStaticType()) {
      testProperties(joint, 0);
    } else if (joint->getType() == UniversalJoint::getStaticType()) {
      testProperties(joint, 0);
      testProperties(joint, 1);
    } else if (joint->getType() == FreeJoint::getStaticType()) {
      EXPECT_FALSE(joint->areLimitsEnforced());
    }
  }
}

//==============================================================================
TEST(SdfParser, ParsingSDFFiles)
{
  const auto numSteps = 10u;

  // Create a list of sdf files to test with where the sdf files contains World
  std::vector<std::string> worldFiles;
  worldFiles.push_back("dart://sample/sdf/benchmark.world");
  worldFiles.push_back("dart://sample/sdf/double_pendulum.world");
  worldFiles.push_back("dart://sample/sdf/double_pendulum_with_base.world");
  worldFiles.push_back("dart://sample/sdf/empty.world");
  worldFiles.push_back("dart://sample/sdf/ground.world");
  worldFiles.push_back("dart://sample/sdf/test/single_bodynode_skeleton.world");

  std::vector<WorldPtr> worlds;
  for (const auto& worldFile : worldFiles)
    worlds.push_back(SdfParser::readWorld(worldFile));

  for (auto world : worlds) {
    EXPECT_TRUE(nullptr != world);

    for (auto i = 0u; i < numSteps; ++i)
      world->step();
  }

  // Create another list of sdf files to test with where the sdf files contains
  // Skeleton
  std::vector<common::Uri> skeletonFiles;
  skeletonFiles.push_back("dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  skeletonFiles.push_back(
      "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf");

  auto world = std::make_shared<World>();
  std::vector<SkeletonPtr> skeletons;
  for (const auto& skeletonFile : skeletonFiles)
    skeletons.push_back(SdfParser::readSkeleton(skeletonFile));

  for (auto skeleton : skeletons) {
    EXPECT_TRUE(nullptr != skeleton);

    world->addSkeleton(skeleton);
    for (auto i = 0u; i < numSteps; ++i)
      world->step();

    world->removeAllSkeletons();
  }
}

//==============================================================================
// Regression: an SDF <soft_shape> link must load as a SoftBodyNode. Before
// SdfParser forwarded the NodeType template argument to
// createJointAndBodyNodePair, every soft SDF body loaded as a rigid BodyNode
// (SoftBodyNode::Properties sliced away), so the soft-feet Atlas reported zero
// soft bodies.
TEST(SdfParser, SoftShapeLinkLoadsAsSoftBodyNode)
{
  auto softAtlas = SdfParser::readSkeleton(
      "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf");
  ASSERT_NE(nullptr, softAtlas);
  EXPECT_EQ(2u, softAtlas->getNumSoftBodyNodes());
  EXPECT_NE(nullptr, softAtlas->getSoftBodyNode("l_foot"));
  EXPECT_NE(nullptr, softAtlas->getSoftBodyNode("r_foot"));
  ASSERT_NE(nullptr, softAtlas->getBodyNode("l_foot"));
  EXPECT_NE(nullptr, softAtlas->getBodyNode("l_foot")->asSoftBodyNode());

  // Control: the rigid variant of the same model stays fully rigid.
  auto rigidAtlas = SdfParser::readSkeleton(
      "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  ASSERT_NE(nullptr, rigidAtlas);
  EXPECT_EQ(0u, rigidAtlas->getNumSoftBodyNodes());
}

//==============================================================================
TEST(SdfParser, PlaneShapeOption)
{
  const std::string sdfFilename = "dart://sample/sdf/test/plane_shape.sdf";

  SkeletonPtr legacySkeleton = SdfParser::readSkeleton(sdfFilename);
  ASSERT_TRUE(legacySkeleton != nullptr);
  ASSERT_EQ(legacySkeleton->getNumBodyNodes(), 1u);
  auto* legacyBody = legacySkeleton->getBodyNode(0);
  ASSERT_TRUE(legacyBody != nullptr);
  ASSERT_EQ(legacyBody->getNumShapeNodes(), 1u);
  EXPECT_TRUE(
      std::dynamic_pointer_cast<BoxShape>(
          legacyBody->getShapeNode(0)->getShape())
      != nullptr);

  SdfParser::Options options;
  options.mUsePlaneShapeForPlane = true;
  SkeletonPtr planeSkeleton = SdfParser::readSkeleton(sdfFilename, options);
  ASSERT_TRUE(planeSkeleton != nullptr);
  ASSERT_EQ(planeSkeleton->getNumBodyNodes(), 1u);
  auto* planeBody = planeSkeleton->getBodyNode(0);
  ASSERT_TRUE(planeBody != nullptr);
  ASSERT_EQ(planeBody->getNumShapeNodes(), 1u);

  const auto plane = std::dynamic_pointer_cast<PlaneShape>(
      planeBody->getShapeNode(0)->getShape());
  ASSERT_TRUE(plane != nullptr);
  EXPECT_TRUE(plane->getNormal().isApprox(Eigen::Vector3d::UnitZ()));
}

//==============================================================================
TEST(SdfParser, PlaneShapeBulletWorldSettles)
{
#if HAVE_BULLET
  SdfParser::Options options;
  options.mUsePlaneShapeForPlane = true;
  options.mCollisionDetector = collision::BulletCollisionDetector::create();

  WorldPtr world = SdfParser::readWorld(
      "dart://sample/sdf/test/plane_box_world.sdf", options);
  ASSERT_TRUE(world != nullptr);

  auto deactivation = world->getDeactivationOptions();
  deactivation.mEnabled = true;
  world->setDeactivationOptions(deactivation);
  world->getConstraintSolver()->getCollisionOption().maxNumContacts = 32u;

  SkeletonPtr ground = world->getSkeleton("ground");
  SkeletonPtr box = world->getSkeleton("box");
  ASSERT_TRUE(ground != nullptr);
  ASSERT_TRUE(box != nullptr);
  EXPECT_FALSE(ground->isMobile());
  EXPECT_TRUE(box->isMobile());

  ASSERT_EQ(ground->getNumBodyNodes(), 1u);
  auto* groundBody = ground->getBodyNode(0);
  ASSERT_TRUE(groundBody != nullptr);
  ASSERT_EQ(groundBody->getNumShapeNodes(), 1u);
  EXPECT_TRUE(
      std::dynamic_pointer_cast<PlaneShape>(
          groundBody->getShapeNode(0)->getShape())
      != nullptr);

  ASSERT_EQ(box->getNumBodyNodes(), 1u);
  auto* boxBody = box->getBodyNode(0);
  ASSERT_TRUE(boxBody != nullptr);

  bool sawContact = false;
  std::size_t steps = 0;
  for (; steps < 5000 && !box->isResting(); ++steps) {
    world->step();
    sawContact
        = sawContact || world->getLastCollisionResult().getNumContacts() > 0;
  }

  EXPECT_TRUE(sawContact) << "box never contacted the parsed PlaneShape";
  ASSERT_LT(steps, 5000u) << "box never went to sleep on the parsed plane";
  ASSERT_TRUE(box->isResting());

  const double z = boxBody->getTransform().translation().z();
  EXPECT_NEAR(z, 0.1, 2e-2);

  const Eigen::VectorXd frozenPositions = box->getPositions();
  for (std::size_t i = 0; i < 200; ++i) {
    world->step();
    ASSERT_TRUE(box->isResting()) << "box spuriously woke at step " << i;
    EXPECT_TRUE(box->getPositions().isApprox(frozenPositions, 1e-12))
        << "resting box drifted on parsed PlaneShape at step " << i;
  }
#else
  GTEST_SKIP() << "Bullet is unavailable";
#endif
}

//==============================================================================
TEST(SdfParser, ReadMaterial)
{
  std::string sdf_filename = "dart://sample/sdf/quad.sdf";
  SkeletonPtr skeleton = SdfParser::readSkeleton(sdf_filename);
  EXPECT_TRUE(nullptr != skeleton);
  auto bodyNode = skeleton->getBodyNode(0);

  bodyNode->eachShapeNodeWith<dart::dynamics::VisualAspect>(
      [](dart::dynamics::ShapeNode* shapeNode) {
        Eigen::Vector4d color = shapeNode->getVisualAspect()->getRGBA();
        Eigen::Vector4d expected_color(0.5, 0.6, 0.8, 1.0);
        double diff = (color - expected_color).norm();
        EXPECT_LT(diff, 1e-4);
      });
}
