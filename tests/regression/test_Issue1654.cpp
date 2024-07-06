/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <cstring>
#include <TestHelpers.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/DartResourceRetriever.hpp>

#include <gtest/gtest.h>

#include <dart/gui/osg/osg.hpp>

using namespace dart;

class CapsuleTest: public testing::Test,
                            public testing::WithParamInterface<const char*>
{
};
TEST_P(CapsuleTest, CapsuleCollision)
{
  const double capsuleRadius = 0.2;
  const double capsuleHeight = 0.6;

  auto world = dart::simulation::World::create();

  if (strcmp(GetParam(), "bullet") == 0)
  {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::BulletCollisionDetector::create());
  }
  else if (strcmp(GetParam(), "ode") == 0)
  {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::OdeCollisionDetector::create());
  }
  else {
    ASSERT_TRUE(false);
  }

  auto ground = createBox({10000, 1000, 0.1}, {0, 0, -0.05});
  ground->setMobile(false);
  world->addSkeleton(ground);

    ::osg::ref_ptr<gui::osg::RealTimeWorldNode> node
      = new gui::osg::RealTimeWorldNode(world);

  auto skeleton = dart::dynamics::Skeleton::create("test");
  dart::dynamics::FreeJoint* joint;
  BodyNode* bn;
  std::tie(joint, bn)
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  Eigen::Isometry3d T_capsule;
  T_capsule = Eigen::Translation3d(0, 0, 0.5)
              * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 1, 0));
  joint->setRelativeTransform(T_capsule);

  auto capsuleNode = bn->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleRadius, capsuleHeight));

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(capsuleNode->getShape()->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);
  // std::cout << "Inertia: " << bn->getBodyNodeProperties().mInertia.getMoment()
  //           << std::endl;

  world->addSkeleton(skeleton);
  const double tolerance = 0.05;
  for (int i = 1; i < 50000; ++i)
  {
    world->step();
    auto capsulePos = bn->getWorldTransform().translation();
    // std::cout << i << ": " << capsulePos.transpose() << std::endl;
    ASSERT_GT(capsulePos.z(), capsuleRadius - tolerance);
  }
}

INSTANTIATE_TEST_CASE_P(
    CollisionEngine,
    CapsuleTest,
    testing::Values("ode"));
    // testing::Values("bullet", "ode"));
