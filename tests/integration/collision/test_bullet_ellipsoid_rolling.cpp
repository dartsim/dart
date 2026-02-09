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

#include <dart/config.hpp>

#if HAVE_BULLET

  #include <dart/simulation/world.hpp>

  #include <dart/collision/bullet/bullet_collision_detector.hpp>

  #include <dart/dynamics/box_shape.hpp>
  #include <dart/dynamics/ellipsoid_shape.hpp>
  #include <dart/dynamics/free_joint.hpp>
  #include <dart/dynamics/skeleton.hpp>
  #include <dart/dynamics/weld_joint.hpp>

  #include <dart/math/constants.hpp>

  #include <gtest/gtest.h>

  #include <memory>

  #include <cmath>

namespace {

struct RollingResult
{
  double xDisplacement;
  double angularSpeedY;
  std::size_t contactCount;
  double height;
};

RollingResult runRollingTrial(
    const dart::dynamics::ShapePtr& shape, double slopeRadians)
{
  using dart::dynamics::BoxShape;
  using dart::dynamics::EllipsoidShape;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::Skeleton;
  using dart::dynamics::WeldJoint;

  auto world = dart::simulation::World::create("bullet-ellipsoid-rolling");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->setCollisionDetector(
      dart::collision::BulletCollisionDetector::create());

  // Ramp
  const double rampThickness = 0.1;
  auto ramp = Skeleton::create("ramp");
  auto rampPair = ramp->createJointAndBodyNodePair<WeldJoint>();
  auto rampBody = rampPair.second;
  auto rampShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(2.0, 2.0, rampThickness));
  auto rampNode = rampBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(rampShape);
  rampNode->getDynamicsAspect()->setFrictionCoeff(0.9);

  const Eigen::Matrix3d rampRot
      = Eigen::AngleAxisd(slopeRadians, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  Eigen::Isometry3d rampTf = Eigen::Isometry3d::Identity();
  rampTf.linear() = rampRot;
  rampTf.translation()
      = -rampRot * Eigen::Vector3d(0.0, 0.0, rampThickness * 0.5);
  ramp->getJoint(0)->setTransformFromParentBodyNode(rampTf);
  world->addSkeleton(ramp);

  // Rolling object
  auto object = Skeleton::create("object");
  auto objectPair = object->createJointAndBodyNodePair<FreeJoint>();
  auto objectBody = objectPair.second;
  auto objectNode = objectBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  objectNode->getDynamicsAspect()->setFrictionCoeff(0.9);
  objectBody->setMass(1.0);

  const Eigen::Vector3d normal = rampRot * Eigen::Vector3d::UnitZ();
  double support = shape->getBoundingBox().getMax()[2];
  if (const auto ellipsoid = std::dynamic_pointer_cast<EllipsoidShape>(shape)) {
    const Eigen::Vector3d radii = ellipsoid->getRadii();
    const double denom = std::pow(normal[0] / radii[0], 2)
                         + std::pow(normal[1] / radii[1], 2)
                         + std::pow(normal[2] / radii[2], 2);
    support = 1.0 / std::sqrt(denom);
  }

  Eigen::Isometry3d objectTf = Eigen::Isometry3d::Identity();
  objectTf.translation()
      = normal.normalized() * (support + rampThickness * 0.5 + 0.02);
  dart::dynamics::FreeJoint::setTransformOf(object->getJoint(0), objectTf);
  world->addSkeleton(object);

  for (std::size_t i = 0; i < 2000; ++i) {
    world->step();
  }

  const auto& collisionResult = world->getLastCollisionResult();
  const auto transform = objectBody->getWorldTransform();
  const Eigen::Vector3d omega = objectBody->getAngularVelocity();

  return RollingResult{
      transform.translation()[0],
      std::abs(omega[1]),
      collisionResult.getNumContacts(),
      transform.translation()[2]};
}

} // namespace

//==============================================================================
TEST(BulletEllipsoidRolling, SphereEllipsoidRollsWithBullet)
{
  const double slopeRadians = 0.3; // ~17 degrees
  auto shape = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d::Constant(0.2));

  const RollingResult result = runRollingTrial(shape, slopeRadians);

  EXPECT_GT(result.xDisplacement, 0.05);
  EXPECT_GT(result.angularSpeedY, 0.2);
  EXPECT_LE(result.contactCount, 2u);
}

//==============================================================================
TEST(BulletEllipsoidRolling, EllipsoidMultiSphereApproxRolls)
{
  const double slopeRadians = 0.3; // ~17 degrees
  auto shape = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(0.2, 0.15, 0.1));

  const RollingResult result = runRollingTrial(shape, slopeRadians);

  EXPECT_GT(result.xDisplacement, 0.04);
  EXPECT_GT(result.angularSpeedY, 0.1);
  EXPECT_LE(result.contactCount, 4u);
}

//==============================================================================
TEST(BulletEllipsoidRolling, HighAspectEllipsoidFallsBackToMesh)
{
  const double slopeRadians = 0.3; // ~17 degrees
  auto shape = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(0.2, 0.2, 0.02));

  const RollingResult result = runRollingTrial(shape, slopeRadians);

  EXPECT_GT(result.contactCount, 0u);
  EXPECT_GT(result.height, 0.01);
  EXPECT_GT(result.xDisplacement, 0.0);
}

#endif // HAVE_BULLET
