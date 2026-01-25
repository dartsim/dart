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

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::collision;
using namespace dart::constraint;

namespace {

SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& position,
    double friction,
    double restitution = 0.0)
{
  auto skeleton = Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto bodyNode = pair.second;

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  auto shapeNode = bodyNode->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);

  shapeNode->getDynamicsAspect()->setFrictionCoeff(friction);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(restitution);

  skeleton->setPositions(
      (Eigen::Vector6d() << 0, 0, 0, position.x(), position.y(), position.z())
          .finished());

  return skeleton;
}

struct CollisionSetup
{
  std::shared_ptr<CollisionDetector> detector;
  std::shared_ptr<CollisionGroup> group;
  SkeletonPtr skel1;
  SkeletonPtr skel2;

  CollisionResult runCollision()
  {
    CollisionOption option;
    option.maxNumContacts = 10;
    CollisionResult result;
    group->collide(option, &result);
    return result;
  }
};

CollisionSetup createCollidingBoxes(
    double friction1,
    double friction2,
    double restitution1 = 0.0,
    double restitution2 = 0.0)
{
  CollisionSetup setup;

  setup.skel1
      = createBox("skel1", Eigen::Vector3d(0, 0, 0), friction1, restitution1);
  setup.skel2
      = createBox("skel2", Eigen::Vector3d(0.9, 0, 0), friction2, restitution2);

  setup.detector = FCLCollisionDetector::create();
  setup.group = setup.detector->createCollisionGroup();
  setup.group->addShapeFramesOf(setup.skel1.get());
  setup.group->addShapeFramesOf(setup.skel2.get());

  return setup;
}

} // namespace

//==============================================================================
// Friction Coefficient Validation Tests
//==============================================================================

TEST(ContactSurface, NaNFrictionClampsToDefault)
{
  auto setup = createCollidingBoxes(std::nan(""), 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimaryFrictionCoeff));
  EXPECT_GE(params.mPrimaryFrictionCoeff, 0.0);
}

TEST(ContactSurface, InfFrictionClampsToDefault)
{
  auto setup
      = createCollidingBoxes(std::numeric_limits<double>::infinity(), 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimaryFrictionCoeff));
  EXPECT_GE(params.mPrimaryFrictionCoeff, 0.0);
}

TEST(ContactSurface, NegativeFrictionClampsToDefault)
{
  auto setup = createCollidingBoxes(-0.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimaryFrictionCoeff));
  EXPECT_GE(params.mPrimaryFrictionCoeff, 0.0);
}

TEST(ContactSurface, ValidFrictionPassesThrough)
{
  auto setup = createCollidingBoxes(0.3, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_DOUBLE_EQ(params.mPrimaryFrictionCoeff, 0.3);
}

TEST(ContactSurface, BothBodiesNaNFrictionUseDefault)
{
  auto setup = createCollidingBoxes(std::nan(""), std::nan(""));
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimaryFrictionCoeff));
  EXPECT_DOUBLE_EQ(
      params.mPrimaryFrictionCoeff, constraint::DART_DEFAULT_FRICTION_COEFF);
}

//==============================================================================
// Restitution Coefficient Validation Tests
//==============================================================================

TEST(ContactSurface, NaNRestitutionClampsToDefault)
{
  auto setup = createCollidingBoxes(0.5, 0.5, std::nan(""), 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mRestitutionCoeff));
  EXPECT_GE(params.mRestitutionCoeff, 0.0);
  EXPECT_LE(params.mRestitutionCoeff, 1.0);
}

TEST(ContactSurface, NegativeRestitutionClampsToDefault)
{
  auto setup = createCollidingBoxes(0.5, 0.5, -0.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mRestitutionCoeff));
  EXPECT_GE(params.mRestitutionCoeff, 0.0);
  EXPECT_LE(params.mRestitutionCoeff, 1.0);
}

TEST(ContactSurface, RestitutionAboveOneClampsToDefault)
{
  auto setup = createCollidingBoxes(0.5, 0.5, 1.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mRestitutionCoeff));
  EXPECT_GE(params.mRestitutionCoeff, 0.0);
  EXPECT_LE(params.mRestitutionCoeff, 1.0);
}

TEST(ContactSurface, BothBodiesNaNRestitutionUseDefault)
{
  auto setup = createCollidingBoxes(0.5, 0.5, std::nan(""), std::nan(""));
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mRestitutionCoeff));
  EXPECT_DOUBLE_EQ(
      params.mRestitutionCoeff, constraint::DART_DEFAULT_RESTITUTION_COEFF);
}
