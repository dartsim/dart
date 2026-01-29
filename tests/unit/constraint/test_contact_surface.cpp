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

#include "dart/collision/collision_detector.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/fcl/fcl_collision_detector.hpp"
#include "dart/constraint/contact_constraint.hpp"
#include "dart/constraint/contact_surface.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::collision;
using namespace dart::constraint;
using namespace dart::test;

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

SkeletonPtr createBoxWithPrimarySecondaryFriction(
    const std::string& name,
    const Eigen::Vector3d& position,
    double primaryFriction,
    double secondaryFriction,
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

  shapeNode->getDynamicsAspect()->setPrimaryFrictionCoeff(primaryFriction);
  shapeNode->getDynamicsAspect()->setSecondaryFrictionCoeff(secondaryFriction);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(restitution);

  skeleton->setPositions(
      (Eigen::Vector6d() << 0, 0, 0, position.x(), position.y(), position.z())
          .finished());

  return skeleton;
}

SkeletonPtr createBoxWithSlipCompliance(
    const std::string& name,
    const Eigen::Vector3d& position,
    double primarySlip,
    double secondarySlip,
    double friction = 1.0)
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
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(primarySlip);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(secondarySlip);

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

CollisionSetup createCollidingBoxesWithPrimarySecondaryFriction(
    double primaryFriction1,
    double secondaryFriction1,
    double primaryFriction2,
    double secondaryFriction2,
    double restitution1 = 0.0,
    double restitution2 = 0.0)
{
  CollisionSetup setup;

  setup.skel1 = createBoxWithPrimarySecondaryFriction(
      "skel1",
      Eigen::Vector3d(0, 0, 0),
      primaryFriction1,
      secondaryFriction1,
      restitution1);
  setup.skel2 = createBoxWithPrimarySecondaryFriction(
      "skel2",
      Eigen::Vector3d(0.9, 0, 0),
      primaryFriction2,
      secondaryFriction2,
      restitution2);

  setup.detector = FCLCollisionDetector::create();
  setup.group = setup.detector->createCollisionGroup();
  setup.group->addShapeFramesOf(setup.skel1.get());
  setup.group->addShapeFramesOf(setup.skel2.get());

  return setup;
}

CollisionSetup createCollidingBoxesWithSlipCompliance(
    double primarySlip1,
    double secondarySlip1,
    double primarySlip2,
    double secondarySlip2)
{
  CollisionSetup setup;

  setup.skel1 = createBoxWithSlipCompliance(
      "skel1", Eigen::Vector3d(0, 0, 0), primarySlip1, secondarySlip1);
  setup.skel2 = createBoxWithSlipCompliance(
      "skel2", Eigen::Vector3d(0.9, 0, 0), primarySlip2, secondarySlip2);

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

TEST(ContactSurface, NegativeInfFrictionClampsToDefault)
{
  auto setup
      = createCollidingBoxes(-std::numeric_limits<double>::infinity(), 0.5);
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

TEST(ContactSurface, ZeroFrictionIsValid)
{
  auto setup = createCollidingBoxes(0.0, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_DOUBLE_EQ(params.mPrimaryFrictionCoeff, 0.0);
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
// Primary/Secondary Friction Validation Tests
//==============================================================================

TEST(ContactSurface, NaNPrimaryFrictionClampsToDefault)
{
  auto setup = createCollidingBoxesWithPrimarySecondaryFriction(
      std::nan(""), 0.5, 0.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimaryFrictionCoeff));
  EXPECT_GE(params.mPrimaryFrictionCoeff, 0.0);
}

TEST(ContactSurface, NaNSecondaryFrictionClampsToDefault)
{
  auto setup = createCollidingBoxesWithPrimarySecondaryFriction(
      0.5, std::nan(""), 0.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mSecondaryFrictionCoeff));
  EXPECT_GE(params.mSecondaryFrictionCoeff, 0.0);
}

TEST(ContactSurface, NegativePrimaryFrictionClampsToDefault)
{
  auto setup
      = createCollidingBoxesWithPrimarySecondaryFriction(-1.0, 0.5, 0.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimaryFrictionCoeff));
  EXPECT_GE(params.mPrimaryFrictionCoeff, 0.0);
}

TEST(ContactSurface, NegativeSecondaryFrictionClampsToDefault)
{
  auto setup
      = createCollidingBoxesWithPrimarySecondaryFriction(0.5, -1.0, 0.5, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mSecondaryFrictionCoeff));
  EXPECT_GE(params.mSecondaryFrictionCoeff, 0.0);
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

TEST(ContactSurface, ValidRestitutionPassesThrough)
{
  auto setup = createCollidingBoxes(0.5, 0.5, 0.8, 0.6);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_DOUBLE_EQ(params.mRestitutionCoeff, 0.8 * 0.6);
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

//==============================================================================
// Slip Compliance Validation Tests
//==============================================================================

TEST(ContactSurface, NaNPrimarySlipComplianceClampsToDefault)
{
  auto setup
      = createCollidingBoxesWithSlipCompliance(std::nan(""), 0.0, 0.1, 0.1);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimarySlipCompliance));
  EXPECT_GE(params.mPrimarySlipCompliance, 0.0);
}

TEST(ContactSurface, NegativePrimarySlipComplianceClampsToDefault)
{
  auto setup = createCollidingBoxesWithSlipCompliance(-0.5, 0.0, 0.1, 0.1);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mPrimarySlipCompliance));
  EXPECT_GE(params.mPrimarySlipCompliance, 0.0);
}

TEST(ContactSurface, NaNSecondarySlipComplianceClampsToDefault)
{
  auto setup
      = createCollidingBoxesWithSlipCompliance(0.0, std::nan(""), 0.1, 0.1);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mSecondarySlipCompliance));
  EXPECT_GE(params.mSecondarySlipCompliance, 0.0);
}

TEST(ContactSurface, NegativeSecondarySlipComplianceClampsToDefault)
{
  auto setup = createCollidingBoxesWithSlipCompliance(0.0, -0.5, 0.1, 0.1);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_TRUE(std::isfinite(params.mSecondarySlipCompliance));
  EXPECT_GE(params.mSecondarySlipCompliance, 0.0);
}

TEST(ContactSurface, ValidSlipCompliancePassesThrough)
{
  auto setup = createCollidingBoxesWithSlipCompliance(0.1, 0.2, 0.1, 0.2);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  DefaultContactSurfaceHandler handler;
  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_DOUBLE_EQ(params.mPrimarySlipCompliance, 0.2);
  EXPECT_DOUBLE_EQ(params.mSecondarySlipCompliance, 0.4);
}

//==============================================================================
// ContactSurfaceHandler Hierarchy Tests
//==============================================================================

TEST(ContactSurfaceHandler, DefaultConstructorHasNoParent)
{
  ContactSurfaceHandler handler;
  EXPECT_EQ(handler.getParent(), nullptr);
}

TEST(ContactSurfaceHandler, SetParentWorks)
{
  auto parent = std::make_shared<DefaultContactSurfaceHandler>();
  ContactSurfaceHandler handler;

  handler.setParent(parent);
  EXPECT_EQ(handler.getParent(), parent);
}

TEST(ContactSurfaceHandler, CannotSetSelfAsParent)
{
  auto handler = std::make_shared<ContactSurfaceHandler>();
  handler->setParent(handler);
  EXPECT_EQ(handler->getParent(), nullptr);
}

TEST(ContactSurfaceHandler, ConstructorWithParent)
{
  auto parent = std::make_shared<DefaultContactSurfaceHandler>();
  ContactSurfaceHandler handler(parent);
  EXPECT_EQ(handler.getParent(), parent);
}

TEST(ContactSurfaceHandler, BaseCreateParamsDelegatesToParent)
{
  auto setup = createCollidingBoxes(0.3, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  auto parent = std::make_shared<DefaultContactSurfaceHandler>();
  ContactSurfaceHandler handler(parent);

  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_DOUBLE_EQ(params.mPrimaryFrictionCoeff, 0.3);
}

TEST(ContactSurfaceHandler, BaseCreateParamsReturnsDefaultWithNoParent)
{
  auto setup = createCollidingBoxes(0.3, 0.5);
  auto result = setup.runCollision();
  ASSERT_GT(result.getNumContacts(), 0u);

  ContactSurfaceHandler handler;

  auto params = handler.createParams(result.getContact(0), 1);

  EXPECT_DOUBLE_EQ(params.mPrimaryFrictionCoeff, DART_DEFAULT_FRICTION_COEFF);
  EXPECT_DOUBLE_EQ(params.mRestitutionCoeff, DART_DEFAULT_RESTITUTION_COEFF);
}

//==============================================================================
// ContactConstraint Static Parameter Tests
//==============================================================================

TEST(ContactConstraint, SetErrorAllowanceNegativeLogsWarning)
{
  double original = ContactConstraint::getErrorAllowance();
  ContactConstraint::setErrorAllowance(-0.5);
  ContactConstraint::setErrorAllowance(original);
}

TEST(ContactConstraint, SetErrorAllowanceValidValue)
{
  double original = ContactConstraint::getErrorAllowance();

  ContactConstraint::setErrorAllowance(0.01);
  EXPECT_DOUBLE_EQ(ContactConstraint::getErrorAllowance(), 0.01);

  ContactConstraint::setErrorAllowance(original);
}

TEST(ContactConstraint, SetErrorReductionParameterNegativeLogsWarning)
{
  double original = ContactConstraint::getErrorReductionParameter();
  ContactConstraint::setErrorReductionParameter(-0.5);
  ContactConstraint::setErrorReductionParameter(original);
}

TEST(ContactConstraint, SetErrorReductionParameterAboveOneLogsWarning)
{
  double original = ContactConstraint::getErrorReductionParameter();
  ContactConstraint::setErrorReductionParameter(1.5);
  ContactConstraint::setErrorReductionParameter(original);
}

TEST(ContactConstraint, SetErrorReductionParameterValidValue)
{
  double original = ContactConstraint::getErrorReductionParameter();

  ContactConstraint::setErrorReductionParameter(0.5);
  EXPECT_DOUBLE_EQ(ContactConstraint::getErrorReductionParameter(), 0.5);

  ContactConstraint::setErrorReductionParameter(original);
}

TEST(ContactConstraint, SetMaxErrorReductionVelocityNegativeLogsWarning)
{
  double original = ContactConstraint::getMaxErrorReductionVelocity();
  ContactConstraint::setMaxErrorReductionVelocity(-0.5);
  ContactConstraint::setMaxErrorReductionVelocity(original);
}

TEST(ContactConstraint, SetMaxErrorReductionVelocityValidValue)
{
  double original = ContactConstraint::getMaxErrorReductionVelocity();

  ContactConstraint::setMaxErrorReductionVelocity(0.1);
  EXPECT_DOUBLE_EQ(ContactConstraint::getMaxErrorReductionVelocity(), 0.1);

  ContactConstraint::setMaxErrorReductionVelocity(original);
}

TEST(ContactConstraint, SetConstraintForceMixingTooSmallLogsWarning)
{
  double original = ContactConstraint::getConstraintForceMixing();
  ContactConstraint::setConstraintForceMixing(1e-12);
  ContactConstraint::setConstraintForceMixing(original);
}

TEST(ContactConstraint, SetConstraintForceMixingValidValue)
{
  double original = ContactConstraint::getConstraintForceMixing();

  ContactConstraint::setConstraintForceMixing(1e-4);
  EXPECT_DOUBLE_EQ(ContactConstraint::getConstraintForceMixing(), 1e-4);

  ContactConstraint::setConstraintForceMixing(original);
}

TEST(ContactConstraint, GetStaticType)
{
  EXPECT_EQ(ContactConstraint::getStaticType(), "ContactConstraint");
}
