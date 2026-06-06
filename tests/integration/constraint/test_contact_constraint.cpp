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

#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/common/All.hpp"
#include "dart/constraint/All.hpp"
#include "dart/dynamics/All.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"
#include "dart/simulation/world.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::test;

namespace {

//==============================================================================
collision::Contact createMalformedContactWithNullCollisionObjects()
{
  collision::Contact contact;
  contact.collisionObject1 = nullptr;
  contact.collisionObject2 = nullptr;
  contact.point = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.force = Eigen::Vector3d::Zero();
  return contact;
}

//==============================================================================
struct ConstraintInfoStorage
{
  explicit ConstraintInfoStorage(std::size_t dim)
  {
    x.assign(dim, 0.0);
    lo.assign(dim, 0.0);
    hi.assign(dim, 0.0);
    b.assign(dim, 0.0);
    w.assign(dim, 0.0);
    findex.assign(dim, -1);

    info.x = x.data();
    info.lo = lo.data();
    info.hi = hi.data();
    info.b = b.data();
    info.w = w.data();
    info.findex = findex.data();
    info.invTimeStep = 1000.0;
  }

  constraint::ConstraintInfo info{};
  std::vector<double> x;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<double> b;
  std::vector<double> w;
  std::vector<int> findex;
};

//==============================================================================
class ExposedContactConstraint final : public constraint::ContactConstraint
{
public:
  using ContactConstraint::applyImpulse;
  using ContactConstraint::applyPositionImpulse;
  using ContactConstraint::applyUnitImpulse;
  using ContactConstraint::ContactConstraint;
  using ContactConstraint::excite;
  using ContactConstraint::getInformation;
  using ContactConstraint::getVelocityChange;
  using ContactConstraint::isActive;
  using ContactConstraint::unexcite;
  using ContactConstraint::uniteSkeletons;
  using ContactConstraint::update;
};

} // namespace

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

//==============================================================================
TEST(ContactConstraint, MalformedContactWithNullCollisionObjectsIsInactive)
{
  auto contact = createMalformedContactWithNullCollisionObjects();
  constraint::ContactSurfaceParams params;
  constraint::ContactConstraint constraint(contact, 0.001, params);

  constraint::ConstraintBase& base = constraint;
  EXPECT_EQ(0u, base.getDimension());
  EXPECT_FALSE(base.isActive());

  base.update();

  EXPECT_FALSE(base.isActive());
}

//==============================================================================
TEST(ContactConstraint, MalformedContactGuardedCallbacksAreNoops)
{
  auto contact = createMalformedContactWithNullCollisionObjects();
  constraint::ContactSurfaceParams params;
  ExposedContactConstraint constraint(contact, 0.001, params);

  EXPECT_EQ(0u, constraint.getDimension());
  EXPECT_FALSE(constraint.isActive());

  ConstraintInfoStorage storage(3u);
  constraint.getInformation(&storage.info);

  std::vector<double> values = {1.0, 2.0, 3.0};
  constraint.applyUnitImpulse(0u);
  constraint.getVelocityChange(values.data(), true);
  constraint.excite();
  constraint.unexcite();
  constraint.applyImpulse(values.data());
  constraint.applyPositionImpulse(values.data());
  constraint.uniteSkeletons();

  EXPECT_FALSE(constraint.isActive());
  EXPECT_DOUBLE_EQ(values[0], 1.0);
  EXPECT_DOUBLE_EQ(values[1], 2.0);
  EXPECT_DOUBLE_EQ(values[2], 3.0);
}

//==============================================================================
TEST(ContactConstraint, FrictionRowsUseCoefficientBoundsCoupledToNormalRow)
{
  auto skeleton1 = dynamics::Skeleton::create("skeleton1");
  auto pair1 = skeleton1->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto bodyNode1 = pair1.second;
  bodyNode1->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  skeleton1->setPosition(3, 0.0);

  auto skeleton2 = dynamics::Skeleton::create("skeleton2");
  auto pair2 = skeleton2->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto bodyNode2 = pair2.second;
  bodyNode2->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  skeleton2->setPosition(3, 0.9);

  auto detector = collision::DartCollisionDetector::create();
  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skeleton1.get());
  group->addShapeFramesOf(skeleton2.get());

  collision::CollisionOption option;
  option.maxNumContacts = 1;
  collision::CollisionResult result;
  group->collide(option, &result);
  ASSERT_GT(result.getNumContacts(), 0u);

  auto contact = result.getContact(0);

  constraint::ContactSurfaceParams params;
  params.mPrimaryFrictionCoeff = 2.5;
  params.mSecondaryFrictionCoeff = 2.5;

  ExposedContactConstraint constraint(contact, 0.001, params);
  ASSERT_EQ(constraint.getDimension(), 3u);

  ConstraintInfoStorage storage(constraint.getDimension());
  constraint.getInformation(&storage.info);

  EXPECT_DOUBLE_EQ(storage.lo[0], 0.0);
  EXPECT_EQ(storage.findex[0], -1);

  EXPECT_DOUBLE_EQ(storage.lo[1], -2.5);
  EXPECT_DOUBLE_EQ(storage.hi[1], 2.5);
  EXPECT_EQ(storage.findex[1], 0);

  EXPECT_DOUBLE_EQ(storage.lo[2], -2.5);
  EXPECT_DOUBLE_EQ(storage.hi[2], 2.5);
  EXPECT_EQ(storage.findex[2], 0);
}
