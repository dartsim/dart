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

/// \file test_contact_constraint.cpp
/// \brief Exercises a real ContactConstraint solve between two reactive bodies.
///
/// The boxed-LCP solver assembles its effective-mass matrix by calling
/// ContactConstraint::applyUnitImpulse()/getVelocityChange() per constraint
/// dimension and computes the target velocity via getRelVelocity(); those
/// methods (and their spatial-Jacobian products) only run when a finite contact
/// between reactive bodies is actually solved. This test drives that path so
/// the contact-velocity hot paths are covered and stay numerically
/// well-behaved.

#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/constraint/constraint_solver.hpp"
#include "dart/constraint/contact_surface.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/sphere_shape.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;

namespace {

class ExposedConstraintSolver : public ConstraintSolver
{
public:
  using ConstraintSolver::updateConstraints;
};

class DuplicateFirstContactDetector : public collision::DartCollisionDetector
{
public:
  bool collide(
      collision::CollisionGroup* group,
      const collision::CollisionOption& option
      = collision::CollisionOption(false, 1u, nullptr),
      collision::CollisionResult* result = nullptr) override
  {
    const bool collided
        = collision::DartCollisionDetector::collide(group, option, result);
    if (result != nullptr && result->getNumContacts() > 0u) {
      auto contact = result->getContact(0);
      result->clear();
      for (std::size_t i = 0; i < 3; ++i) {
        contact.point.x() += 1e-6 * static_cast<double>(i);
        result->addContact(contact);
      }
    }
    return collided;
  }
};

/// Counts how many contacts reach contact-constraint creation, i.e. how many
/// survive the solver's validity filters and are actually solved.
class CountingContactSurfaceHandler : public DefaultContactSurfaceHandler
{
public:
  ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      std::size_t numContactsOnCollisionObject,
      double timeStep) const override
  {
    ++mNumContactsCreated;
    mNumContactsOnCollisionObject.push_back(numContactsOnCollisionObject);
    return DefaultContactSurfaceHandler::createConstraint(
        contact, numContactsOnCollisionObject, timeStep);
  }

  mutable int mNumContactsCreated = 0;
  mutable std::vector<std::size_t> mNumContactsOnCollisionObject;
};

class ReentrantContactSurfaceHandler : public CountingContactSurfaceHandler
{
public:
  explicit ReentrantContactSurfaceHandler(ExposedConstraintSolver& nestedSolver)
    : mNestedSolver(&nestedSolver)
  {
  }

  ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      std::size_t numContactsOnCollisionObject,
      double timeStep) const override
  {
    if (!mDidReenter) {
      mDidReenter = true;
      mNestedSolver->updateConstraints();
    }

    return CountingContactSurfaceHandler::createConstraint(
        contact, numContactsOnCollisionObject, timeStep);
  }

  mutable bool mDidReenter = false;

private:
  ExposedConstraintSolver* mNestedSolver;
};

/// A dynamic (reactive) sphere body on a FreeJoint at the given position.
SkeletonPtr createDynamicSphere(
    const std::string& name, double radius, const Eigen::Vector3d& position)
{
  auto skeleton = Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;
  body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
      std::make_shared<SphereShape>(radius));
  skeleton->setPositions(
      (Eigen::Vector6d() << 0, 0, 0, position.x(), position.y(), position.z())
          .finished());
  return skeleton;
}

} // namespace

//==============================================================================
// Two overlapping reactive spheres produce a finite contact that the solver
// must process: the contact constraint is created and solved, exercising the
// ContactConstraint spatial-Jacobian velocity paths, and the resulting state
// stays finite.
TEST(ContactConstraint, ReactiveBodiesContactSolveStaysFinite)
{
  // Spheres of radius 0.1 whose centers are 0.15 apart -> they overlap.
  auto bodyA = createDynamicSphere("A", 0.1, Eigen::Vector3d(0.0, 0.0, 0.0));
  auto bodyB = createDynamicSphere("B", 0.1, Eigen::Vector3d(0.15, 0.0, 0.0));

  ConstraintSolver solver;
  solver.setTimeStep(0.001);
  solver.addSkeleton(bodyA);
  solver.addSkeleton(bodyB);

  auto handler = std::make_shared<CountingContactSurfaceHandler>();
  solver.addContactSurfaceHandler(handler);

  ASSERT_NO_FATAL_FAILURE(solver.solve());

  // The finite contact must reach contact-constraint creation and be solved
  // (this is what drives getRelVelocity()/getVelocityChange()).
  EXPECT_GT(handler->mNumContactsCreated, 0);

  // Both reactive bodies must come out of the solve with finite state.
  EXPECT_TRUE(bodyA->getVelocities().allFinite());
  EXPECT_TRUE(bodyB->getVelocities().allFinite());
  EXPECT_TRUE(bodyA->getPositions().allFinite());
  EXPECT_TRUE(bodyB->getPositions().allFinite());
}

//==============================================================================
// Solving the same contact repeatedly (steady contact) must remain stable and
// finite across steps -- this also re-enters the contact-velocity paths each
// solve.
TEST(ContactConstraint, RepeatedContactSolveIsStable)
{
  auto bodyA = createDynamicSphere("A", 0.1, Eigen::Vector3d(0.0, 0.0, 0.0));
  auto bodyB = createDynamicSphere("B", 0.1, Eigen::Vector3d(0.15, 0.0, 0.0));

  ConstraintSolver solver;
  solver.setTimeStep(0.001);
  solver.addSkeleton(bodyA);
  solver.addSkeleton(bodyB);

  for (int i = 0; i < 5; ++i) {
    ASSERT_NO_FATAL_FAILURE(solver.solve());
    ASSERT_TRUE(bodyA->getVelocities().allFinite());
    ASSERT_TRUE(bodyB->getVelocities().allFinite());
  }
}

//==============================================================================
// A custom contact handler may re-enter another solver while the outer solver
// is still creating contact constraints. Contact-pair counting must be local to
// the outer update call so the inner update cannot reset the outer counts.
TEST(ContactConstraint, ReentrantContactCreationPreservesPairCounts)
{
  ExposedConstraintSolver nestedSolver;
  common::FrameAllocator sharedFrameAllocator;
  nestedSolver.setFrameAllocator(&sharedFrameAllocator);
  nestedSolver.setTimeStep(0.001);
  nestedSolver.addSkeleton(
      createDynamicSphere("nested_A", 0.1, Eigen::Vector3d(0.0, 0.0, 0.0)));
  nestedSolver.addSkeleton(
      createDynamicSphere("nested_B", 0.1, Eigen::Vector3d(0.15, 0.0, 0.0)));

  ExposedConstraintSolver outerSolver;
  outerSolver.setFrameAllocator(&sharedFrameAllocator);
  outerSolver.setTimeStep(0.001);
  outerSolver.setCollisionDetector(
      std::make_shared<DuplicateFirstContactDetector>());
  outerSolver.addSkeleton(
      createDynamicSphere("outer_A", 0.1, Eigen::Vector3d(0.0, 0.0, 0.0)));
  outerSolver.addSkeleton(
      createDynamicSphere("outer_B", 0.1, Eigen::Vector3d(0.15, 0.0, 0.0)));

  auto handler = std::make_shared<ReentrantContactSurfaceHandler>(nestedSolver);
  outerSolver.addContactSurfaceHandler(handler);

  ASSERT_NO_FATAL_FAILURE(outerSolver.updateConstraints());

  ASSERT_TRUE(handler->mDidReenter);
  ASSERT_EQ(handler->mNumContactsOnCollisionObject.size(), 3u);
  for (const auto count : handler->mNumContactsOnCollisionObject) {
    EXPECT_EQ(count, 3u);
  }
}
