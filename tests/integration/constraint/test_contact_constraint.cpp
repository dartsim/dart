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

#include "dart/common/All.hpp"
#include "dart/constraint/All.hpp"
#include "dart/dynamics/All.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::test;

//==============================================================================
void testContactWithKinematicJoint(
    const math::LcpSolverPtr& lcpSolver, double tol)
{
  auto world = std::make_shared<simulation::World>();
  world->setConstraintSolver(
      std::make_unique<constraint::ConstraintSolver>(lcpSolver));

  auto skeleton1 = dynamics::Skeleton::create("skeleton1");
  auto pair1 = skeleton1->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto bodyNode1 = pair1.second;
  auto joint1 = pair1.first;
  joint1->setActuatorType(dynamics::Joint::VELOCITY);
  auto shape1
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0));
  bodyNode1->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape1);
  skeleton1->setPosition(5, 0.0);

  auto skeleton2 = dynamics::Skeleton::create("skeleton2");
  auto pair2 = skeleton2->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto bodyNode2 = pair2.second;
  auto joint2 = pair2.first;
  joint2->setActuatorType(dynamics::Joint::FORCE);
  auto shape2
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5));
  bodyNode2->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape2);
  skeleton2->setPosition(5, 0.75);

  world->addSkeleton(skeleton1);
  world->addSkeleton(skeleton2);

  for (auto i = 0u; i < 100; ++i) {
    skeleton1->setCommand(3, 0.1);

    world->step();

    EXPECT_EQ(bodyNode1->getLinearVelocity()[0], 0.1);

    // Need few steps to settle down
    if (i > 15) {
      EXPECT_NEAR(bodyNode2->getLinearVelocity()[0], 0.1, tol);
    }
  }
}

//==============================================================================
TEST(ContactConstraint, ContactWithKinematicJoint)
{
  // Split impulse removes penetration correction from the velocity solve,
  // which slightly relaxes tangential coupling in this setup.
#ifdef _WIN32
  constexpr double dantzigTol = 1e-2;
#else
  constexpr double dantzigTol = 5e-3;
#endif
  testContactWithKinematicJoint(
      std::make_shared<math::DantzigSolver>(), dantzigTol);

#ifdef DART_ARCH_32BITS
  testContactWithKinematicJoint(std::make_shared<math::PgsSolver>(), 1e-2);
#else
  testContactWithKinematicJoint(std::make_shared<math::PgsSolver>(), 5e-3);
#endif
}
