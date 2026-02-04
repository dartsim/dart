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
  #include <dart/dynamics/cylinder_shape.hpp>
  #include <dart/dynamics/free_joint.hpp>
  #include <dart/dynamics/skeleton.hpp>
  #include <dart/dynamics/weld_joint.hpp>

  #include <Eigen/Geometry>
  #include <gtest/gtest.h>

  #include <memory>

  #include <cmath>

namespace {

struct RollingResult
{
  double xDisplacement;
  double angularSpeedY;
  std::size_t contactCount;
};

RollingResult runRollingTrial()
{
  using dart::collision::BulletCollisionDetector;
  using dart::dynamics::BoxShape;
  using dart::dynamics::CollisionAspect;
  using dart::dynamics::CylinderShape;
  using dart::dynamics::DynamicsAspect;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::Skeleton;
  using dart::dynamics::VisualAspect;
  using dart::dynamics::WeldJoint;
  using dart::simulation::World;

  auto world = World::create("bullet-cylinder-rolling");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
  world->setCollisionDetector(BulletCollisionDetector::create());

  // Ground plane.
  const double groundThickness = 0.1;
  auto ground = Skeleton::create("ground");
  auto groundPair = ground->createJointAndBodyNodePair<WeldJoint>();
  auto* groundBody = groundPair.second;
  auto groundShape = std::make_shared<BoxShape>(
      Eigen::Vector3d(10.0, 10.0, groundThickness));
  auto* groundNode = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(groundShape);
  groundNode->getDynamicsAspect()->setFrictionCoeff(1.0);

  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation() = Eigen::Vector3d(0.0, 0.0, -groundThickness * 0.5);
  groundBody->getParentJoint()->setTransformFromParentBodyNode(groundTf);
  world->addSkeleton(ground);

  // Rolling cylinder.
  const double radius = 0.2;
  const double height = 0.05;
  auto cylinder = Skeleton::create("cylinder");
  auto cylinderPair = cylinder->createJointAndBodyNodePair<FreeJoint>();
  auto* cylinderBody = cylinderPair.second;
  auto cylinderShape = std::make_shared<CylinderShape>(radius, height);
  auto* cylinderNode = cylinderBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(cylinderShape);
  cylinderNode->getDynamicsAspect()->setFrictionCoeff(1.0);
  cylinderBody->setMass(2.0);

  Eigen::Isometry3d cylinderTf = Eigen::Isometry3d::Identity();
  cylinderTf.translation()
      = Eigen::Vector3d(0.0, 0.0, radius + groundThickness * 0.5 + 0.01);
  cylinderTf.linear() = Eigen::Quaterniond::FromTwoVectors(
                            Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY())
                            .toRotationMatrix();
  FreeJoint::setTransformOf(cylinder->getJoint(0), cylinderTf);
  world->addSkeleton(cylinder);

  const Eigen::Vector3d torqueLocal(0.0, 0.0, 0.4);
  for (std::size_t i = 0; i < 2000; ++i) {
    cylinderBody->addExtTorque(torqueLocal, true);
    world->step();
  }

  const auto& collisionResult = world->getLastCollisionResult();
  const Eigen::Vector3d omega = cylinderBody->getAngularVelocity();
  const Eigen::Isometry3d transform = cylinderBody->getWorldTransform();

  return RollingResult{
      transform.translation().x(),
      std::abs(omega.dot(Eigen::Vector3d::UnitY())),
      collisionResult.getNumContacts()};
}

} // namespace

//==============================================================================
TEST(BulletCylinderRolling, CylinderRollsUnderTorque)
{
  const RollingResult result = runRollingTrial();

  EXPECT_GT(std::abs(result.xDisplacement), 0.05);
  EXPECT_GT(result.angularSpeedY, 0.1);
  EXPECT_GT(result.contactCount, 0u);
}

#endif // HAVE_BULLET
