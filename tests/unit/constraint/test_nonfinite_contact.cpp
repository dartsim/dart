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

/// \file test_nonfinite_contact.cpp
/// \brief Regression test for gz-physics issue #1010:
///        https://github.com/gazebosim/gz-physics/issues/1010
///
/// A contact with a non-finite (NaN/Inf) point, normal, or penetration depth
/// used to propagate into ContactConstraint's spatial Jacobian, tripping
/// `DART_ASSERT(!math::isNan(mSpatialNormalA))` in asserts-enabled builds and
/// silently corrupting the LCP solve in release builds.
///
/// Primitive shapes now reject non-finite/non-positive dimensions, so a
/// degenerate shape can no longer be the source. But the constraint solver must
/// still defend itself against a non-finite contact coming from a source that
/// shape validation cannot cover -- a malformed mesh/heightmap or a third-party
/// collision backend. This test models that source with a collision detector
/// that runs a normal sphere-sphere collision between two *valid* shapes and
/// then corrupts the contact point to NaN, and verifies that
/// ConstraintSolver::updateConstraints() drops the contact before contact
/// constraint creation, leaving the solve crash-free and the state finite.

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

#include <limits>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::constraint;

namespace {

/// A DART collision detector that performs a normal collision check and then
/// corrupts the first reported contact point to NaN. This stands in for a
/// malformed mesh or a third-party collision backend that emits a non-finite
/// contact between otherwise valid collision objects -- the case the constraint
/// solver must defend against now that primitive shapes validate their
/// dimensions.
class NonFiniteContactDetector : public collision::DartCollisionDetector
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
      result->getContact(0).point.x()
          = std::numeric_limits<double>::quiet_NaN();
    }
    return collided;
  }
};

/// Contact-surface handler that counts how many contacts reach contact
/// constraint creation. ConstraintSolver only invokes the handler for contacts
/// that survive the validity filters in updateConstraints(), so the count is
/// exactly the number of contacts accepted into the solve.
class CountingContactSurfaceHandler : public DefaultContactSurfaceHandler
{
public:
  ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      size_t numContactsOnCollisionObject,
      double timeStep) const override
  {
    ++mNumContactsCreated;
    return DefaultContactSurfaceHandler::createConstraint(
        contact, numContactsOnCollisionObject, timeStep);
  }

  mutable int mNumContactsCreated = 0;
};

/// Creates a single-body skeleton with a FreeJoint at the given position.
SkeletonPtr createBody(
    const std::string& name,
    const ShapePtr& shape,
    const Eigen::Vector3d& position,
    bool withDynamics)
{
  auto skeleton = Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;

  if (withDynamics) {
    body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
  } else {
    body->createShapeNodeWith<CollisionAspect>(shape);
  }

  skeleton->setPositions(
      (Eigen::Vector6d() << 0, 0, 0, position.x(), position.y(), position.z())
          .finished());

  return skeleton;
}

} // namespace

//==============================================================================
// A non-finite contact (here from a detector that emits a NaN contact point
// between valid shapes) must be dropped before contact-constraint creation
// instead of corrupting the solve.
TEST(NonFiniteContact, NonFiniteContactIsSkipped)
{
  // Two valid, overlapping spheres.
  auto baseBody = createBody(
      "base",
      std::make_shared<SphereShape>(1.0),
      Eigen::Vector3d::Zero(),
      /*withDynamics=*/false);
  baseBody->setMobile(false);

  auto fallingBody = createBody(
      "falling_weight",
      std::make_shared<SphereShape>(0.1),
      Eigen::Vector3d(0.0, 0.0, 0.5),
      /*withDynamics=*/true);

  ConstraintSolver solver;
  solver.setTimeStep(0.001);
  solver.setCollisionDetector(std::make_shared<NonFiniteContactDetector>());
  solver.addSkeleton(baseBody);
  solver.addSkeleton(fallingBody);

  auto handler = std::make_shared<CountingContactSurfaceHandler>();
  solver.addContactSurfaceHandler(handler);

  // Before the fix this aborts on an assertion (asserts-enabled builds) or runs
  // the LCP solver on a NaN system (release builds).
  ASSERT_NO_FATAL_FAILURE(solver.solve());

  // Collision detection reports the overlap, and the contact it produces is
  // non-finite: this is what the solver must defend against.
  const auto& result = solver.getLastCollisionResult();
  ASSERT_GT(result.getNumContacts(), 0u);
  bool anyNonFinite = false;
  for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (!contact.point.allFinite() || !contact.normal.allFinite()
        || !std::isfinite(contact.penetrationDepth)) {
      anyNonFinite = true;
    }
  }
  EXPECT_TRUE(anyNonFinite) << "Test setup no longer produces a non-finite "
                               "contact";

  // The non-finite contact must be filtered before contact-constraint creation.
  EXPECT_EQ(handler->mNumContactsCreated, 0);

  // And the solve must leave the simulation state finite.
  EXPECT_TRUE(fallingBody->getPositions().allFinite());
  EXPECT_TRUE(fallingBody->getVelocities().allFinite());
  EXPECT_TRUE(baseBody->getVelocities().allFinite());
}
