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

// Mirror the DART extension points in gz-physics (Apache-2.0), commit
// 4e2af8c733ac8396a9d2b4dcae1f7f6aa28c5668: dartsim/src/GzCollisionDetector.hh,
// GzCollisionDetector.cc, SimulationFeatures.hh/.cc, WorldFeatures.cc,
// plugin.cc. Refresh: compare those files on gz-physics main, update the
// signatures and dispatch assertions below, then record the new SHA and rerun
// this test.

#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/RaycastOption.hpp"
#include "dart/collision/RaycastResult.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Deprecated.hpp"
#include "dart/config.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"

#if HAVE_BULLET
  #include "dart/collision/bullet/BulletCollisionDetector.hpp"
  #include "dart/collision/bullet/BulletCollisionGroup.hpp"

  #include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#endif
#if HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
  #include "dart/collision/ode/OdeCollisionGroup.hpp"
#endif

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

namespace {

using namespace dart;
using namespace dart::collision;
using namespace dart::constraint;
using namespace dart::dynamics;
using namespace dart::simulation;

// Stand-ins for gz-physics ray records keep this gate free of Gazebo libraries.
struct GzRay
{
  Eigen::Vector3d origin;
  Eigen::Vector3d target;
};

struct GzRayResult
{
  Eigen::Vector3d point;
  double fraction;
  Eigen::Vector3d normal;
};

class GzCollisionDetector
{
public:
  virtual ~GzCollisionDetector() = default;

  virtual void SetCollisionPairMaxContacts(std::size_t maxContacts)
  {
    maxCollisionPairContacts = maxContacts;
  }

  virtual std::size_t GetCollisionPairMaxContacts() const
  {
    return maxCollisionPairContacts;
  }

  virtual bool BatchRaycast(
      CollisionGroup*,
      const std::vector<GzRay>&,
      std::vector<GzRayResult>&) const
  {
    ++batchRayCalls;
    return false;
  }

  std::size_t groupsCreated = 0;
  std::size_t singleGroupCalls = 0;
  std::size_t twoGroupCalls = 0;
  std::size_t limitCalls = 0;
  std::size_t singleRayCalls = 0;
  mutable std::size_t batchRayCalls = 0;

protected:
  GzCollisionDetector() = default;

  virtual void LimitCollisionPairMaxContacts(CollisionResult*)
  {
    ++limitCalls;
  }

  std::size_t maxCollisionPairContacts
      = std::numeric_limits<std::size_t>::max();
};

#if HAVE_ODE
class GzOdeCollisionGroup : public OdeCollisionGroup
{
public:
  using OdeCollisionGroup::getOdeSpaceId;
  using OdeCollisionGroup::OdeCollisionGroup;
};

class GzOdeCollisionDetector : public OdeCollisionDetector,
                               public GzCollisionDetector
{
public:
  static std::shared_ptr<GzOdeCollisionDetector> create()
  {
    return std::shared_ptr<GzOdeCollisionDetector>(new GzOdeCollisionDetector);
  }

  std::unique_ptr<CollisionGroup> createCollisionGroup() override
  {
    ++groupsCreated;
    return std::make_unique<GzOdeCollisionGroup>(shared_from_this());
  }

  bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override
  {
    ++singleGroupCalls;
    const bool hit = OdeCollisionDetector::collide(group, option, result);
    LimitCollisionPairMaxContacts(result);
    return hit;
  }

  bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override
  {
    ++twoGroupCalls;
    const bool hit
        = OdeCollisionDetector::collide(group1, group2, option, result);
    LimitCollisionPairMaxContacts(result);
    return hit;
  }

  bool raycast(
      CollisionGroup* group,
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option = RaycastOption(),
      RaycastResult* result = nullptr) override
  {
    ++singleRayCalls;
    if (!result || option.mEnableAllHits || (to - from).norm() == 0.0)
      return false;
    result->clear();
    const Eigen::Vector3d direction = (to - from).normalized();
    const auto ray = dCreateRay(nullptr, (to - from).norm());
    dGeomRaySet(
        ray,
        from.x(),
        from.y(),
        from.z(),
        direction.x(),
        direction.y(),
        direction.z());
    const auto space
        = static_cast<GzOdeCollisionGroup*>(group)->getOdeSpaceId();
    // shortcut: flat ODE spaces only, recurse when adding nested test fixtures.
    for (int i = 0; i < dSpaceGetNumGeoms(space); ++i) {
      dContactGeom contact;
      if (dCollide(
              ray, dSpaceGetGeom(space, i), 1, &contact, sizeof(contact))) {
        RayHit hit;
        hit.mPoint
            = Eigen::Vector3d(contact.pos[0], contact.pos[1], contact.pos[2]);
        hit.mNormal = Eigen::Vector3d(
            contact.normal[0], contact.normal[1], contact.normal[2]);
        hit.mFraction = (hit.mPoint - from).norm() / (to - from).norm();
        hit.mCollisionObject
            = static_cast<CollisionObject*>(dGeomGetData(contact.g2));
        if (result->mRayHits.empty()
            || hit.mFraction < result->mRayHits[0].mFraction)
          result->mRayHits = {hit};
      }
    }
    dGeomDestroy(ray);
    return result->hasHit();
  }

  bool BatchRaycast(
      CollisionGroup* group,
      const std::vector<GzRay>& rays,
      std::vector<GzRayResult>& output) const override
  {
    ++batchRayCalls;
    output.clear();
    for (const auto& ray : rays) {
      RaycastResult result;
      if (!const_cast<GzOdeCollisionDetector*>(this)->raycast(
              group, ray.origin, ray.target, RaycastOption(), &result))
        return false;
      const auto& hit = result.mRayHits.front();
      output.push_back({hit.mPoint, hit.mFraction, hit.mNormal});
    }
    return true;
  }

protected:
  GzOdeCollisionDetector() = default;
};
#endif

#if HAVE_BULLET
class GzBulletCollisionGroup : public BulletCollisionGroup
{
public:
  explicit GzBulletCollisionGroup(const CollisionDetectorPtr& detector)
    : BulletCollisionGroup(detector)
  {
  }

  const btCollisionWorld* getCollisionWorld() const
  {
    return getBulletCollisionWorld();
  }
};

class GzBulletCollisionDetector : public BulletCollisionDetector,
                                  public GzCollisionDetector
{
public:
  static std::shared_ptr<GzBulletCollisionDetector> create()
  {
    return std::shared_ptr<GzBulletCollisionDetector>(
        new GzBulletCollisionDetector);
  }

  std::unique_ptr<CollisionGroup> createCollisionGroup() override
  {
    ++groupsCreated;
    return std::make_unique<GzBulletCollisionGroup>(shared_from_this());
  }

  bool collide(
      CollisionGroup* group,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override
  {
    ++singleGroupCalls;
    const bool hit = BulletCollisionDetector::collide(group, option, result);
    LimitCollisionPairMaxContacts(result);
    return hit;
  }

  bool collide(
      CollisionGroup* group1,
      CollisionGroup* group2,
      const CollisionOption& option = CollisionOption(false, 1u, nullptr),
      CollisionResult* result = nullptr) override
  {
    ++twoGroupCalls;
    const bool hit
        = BulletCollisionDetector::collide(group1, group2, option, result);
    LimitCollisionPairMaxContacts(result);
    return hit;
  }

  bool BatchRaycast(
      CollisionGroup* group,
      const std::vector<GzRay>& rays,
      std::vector<GzRayResult>& output) const override
  {
    ++batchRayCalls;
    const auto* world
        = static_cast<GzBulletCollisionGroup*>(group)->getCollisionWorld();
    output.clear();
    for (const auto& ray : rays) {
      const btVector3 from(ray.origin.x(), ray.origin.y(), ray.origin.z());
      const btVector3 to(ray.target.x(), ray.target.y(), ray.target.z());
      btCollisionWorld::ClosestRayResultCallback callback(from, to);
      world->rayTest(from, to, callback);
      if (!callback.hasHit())
        return false;
      const auto& point = callback.m_hitPointWorld;
      const auto& normal = callback.m_hitNormalWorld;
      output.push_back(
          {Eigen::Vector3d(point.x(), point.y(), point.z()),
           double(callback.m_closestHitFraction),
           Eigen::Vector3d(normal.x(), normal.y(), normal.z())});
    }
    return true;
  }

protected:
  GzBulletCollisionDetector() = default;
};
#endif

class GzContactSurfaceHandler : public ContactSurfaceHandler
{
public:
  ContactSurfaceParams createParams(
      const Contact& contact,
      size_t numContactsOnCollisionObject) const override
  {
    ++paramsCalls;
    contactsOnObject = numContactsOnCollisionObject;
    lastParams = ContactSurfaceHandler::createParams(
        contact, numContactsOnCollisionObject);
    return lastParams;
  }

  ContactConstraintPtr createConstraint(
      Contact& contact,
      size_t numContactsOnCollisionObject,
      double timeStep) const override
  {
    ++constraintCalls;
    auto constraint = ContactSurfaceHandler::createConstraint(
        contact, numContactsOnCollisionObject, timeStep);
    createdConstraint = bool(constraint);
    return constraint;
  }

  mutable std::size_t paramsCalls = 0;
  mutable std::size_t constraintCalls = 0;
  mutable std::size_t contactsOnObject = 0;
  mutable bool createdConstraint = false;
  mutable ContactSurfaceParams lastParams;
};

DART_SUPPRESS_DEPRECATED_BEGIN
class ShimConstraintSolver : public ConstraintSolver
{
public:
  ShimConstraintSolver() = default;

  explicit ShimConstraintSolver(double timeStep) : ConstraintSolver(timeStep) {}

private:
  void solveConstrainedGroup(ConstrainedGroup&) override {}
};
DART_SUPPRESS_DEPRECATED_END

class GazeboShim : public ::testing::Test
{
protected:
  CollisionDetector::Factory savedFactory{*CollisionDetector::getFactory()};

  void SetUp() override
  {
    // Register after DART's static initializers, including static-library
    // builds.
#if HAVE_ODE
    CollisionDetector::Registrar<GzOdeCollisionDetector> odeRegistrar{
        GzOdeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<GzOdeCollisionDetector> {
          return GzOdeCollisionDetector::create();
        }};
#endif
#if HAVE_BULLET
    CollisionDetector::Registrar<GzBulletCollisionDetector> bulletRegistrar{
        GzBulletCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<GzBulletCollisionDetector> {
          return GzBulletCollisionDetector::create();
        }};
#endif
  }

  void TearDown() override
  {
    *CollisionDetector::getFactory() = savedFactory;
  }
};

std::shared_ptr<World> makeShimWorld()
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  auto fixed = Skeleton::create("fixed");
  auto* fixedBody = fixed->createJointAndBodyNodePair<WeldJoint>().second;
  auto* fixedShape
      = fixedBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  fixedShape->getDynamicsAspect()->setFrictionCoeff(0.25);
  world->addSkeleton(fixed);

  auto moving = Skeleton::create("moving");
  auto pair = moving->createJointAndBodyNodePair<FreeJoint>();
  auto* movingShape
      = pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
          std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  movingShape->getDynamicsAspect()->setFrictionCoeff(0.75);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.75;
  pair.first->setPositions(FreeJoint::convertToPositions(transform));
  world->addSkeleton(moving);
  return world;
}

void expectSurfaceCallbacks(const GzContactSurfaceHandler& handler)
{
  EXPECT_GT(handler.constraintCalls, 0u);
  EXPECT_EQ(handler.paramsCalls, handler.constraintCalls);
  EXPECT_GT(handler.contactsOnObject, 0u);
  EXPECT_TRUE(handler.createdConstraint);
  EXPECT_DOUBLE_EQ(handler.lastParams.mPrimaryFrictionCoeff, 0.25);
  EXPECT_DOUBLE_EQ(handler.lastParams.mSecondaryFrictionCoeff, 0.25);
}

TEST_F(GazeboShim, FactoryIndependentDefaults)
{
  auto* factory = CollisionDetector::getFactory();
  factory->unregisterAllCreators();
  ASSERT_TRUE(factory->getKeys().empty());
  const auto world = World::create();
  ShimConstraintSolver solver;
  ShimConstraintSolver timedSolver(0.001);
  EXPECT_NE(
      std::dynamic_pointer_cast<FCLCollisionDetector>(
          world->getConstraintSolver()->getCollisionDetector()),
      nullptr);
  EXPECT_NE(
      std::dynamic_pointer_cast<FCLCollisionDetector>(
          solver.getCollisionDetector()),
      nullptr);
  EXPECT_NE(
      std::dynamic_pointer_cast<FCLCollisionDetector>(
          timedSolver.getCollisionDetector()),
      nullptr);
}

TEST_F(GazeboShim, ContactSurfaceCallbacks)
{
  const auto world = makeShimWorld();
  const auto handler = std::make_shared<GzContactSurfaceHandler>();
  world->getConstraintSolver()->addContactSurfaceHandler(handler);
  EXPECT_NE(
      std::dynamic_pointer_cast<DefaultContactSurfaceHandler>(
          handler->getParent()),
      nullptr);
  world->step();
  expectSurfaceCallbacks(*handler);
  EXPECT_TRUE(
      world->getConstraintSolver()->removeContactSurfaceHandler(handler));
}

#if HAVE_BULLET || HAVE_ODE
template <typename Detector>
void exerciseShimDetector()
{
  const auto detector
      = CollisionDetector::getFactory()->create(Detector::getStaticType());
  ASSERT_NE(std::dynamic_pointer_cast<Detector>(detector), nullptr);
  const auto gzDetector
      = std::dynamic_pointer_cast<GzCollisionDetector>(detector);
  ASSERT_NE(gzDetector, nullptr);
  gzDetector->SetCollisionPairMaxContacts(8u);
  EXPECT_EQ(gzDetector->GetCollisionPairMaxContacts(), 8u);

  const auto world = makeShimWorld();
  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(detector);
  EXPECT_GT(gzDetector->groupsCreated, 0u);
  const auto handler = std::make_shared<GzContactSurfaceHandler>();
  solver->addContactSurfaceHandler(handler);

  const auto groupsBefore = gzDetector->groupsCreated;
  const auto both = detector->createCollisionGroupAsSharedPtr(
      world->getSkeleton(0).get(), world->getSkeleton(1).get());
  const auto first
      = detector->createCollisionGroupAsSharedPtr(world->getSkeleton(0).get());
  const auto second
      = detector->createCollisionGroupAsSharedPtr(world->getSkeleton(1).get());
  EXPECT_EQ(gzDetector->groupsCreated, groupsBefore + 3u);
  CollisionResult result;
  const CollisionOption option(true, 8u, nullptr);
  EXPECT_TRUE(both->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
  EXPECT_TRUE(first->collide(second.get(), option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
  EXPECT_EQ(gzDetector->singleGroupCalls, 1u);
  EXPECT_EQ(gzDetector->twoGroupCalls, 1u);
  EXPECT_EQ(gzDetector->limitCalls, 2u);

  const GzRay ray{
      Eigen::Vector3d(-2.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)};
  RaycastResult rayResult;
  EXPECT_TRUE(
      both->raycast(ray.origin, ray.target, RaycastOption(), &rayResult));
  EXPECT_TRUE(rayResult.hasHit());
  std::vector<GzRayResult> batchResult;
  EXPECT_TRUE(gzDetector->BatchRaycast(both.get(), {ray}, batchResult));
  ASSERT_EQ(batchResult.size(), 1u);
  EXPECT_NEAR(batchResult[0].fraction, 0.375, 1e-6);
  EXPECT_NEAR(batchResult[0].point.x(), -0.5, 1e-6);
  EXPECT_EQ(gzDetector->batchRayCalls, 1u);
  #if HAVE_ODE
  if (std::dynamic_pointer_cast<GzOdeCollisionDetector>(detector)) {
    EXPECT_EQ(gzDetector->singleRayCalls, 2u);
  }
  #endif

  const auto beforeStep = gzDetector->singleGroupCalls;
  world->step();
  EXPECT_GT(gzDetector->singleGroupCalls, beforeStep);
  EXPECT_EQ(
      gzDetector->limitCalls,
      gzDetector->singleGroupCalls + gzDetector->twoGroupCalls);
  expectSurfaceCallbacks(*handler);
}
#endif

#if HAVE_ODE
TEST_F(GazeboShim, OdeExtensions)
{
  exerciseShimDetector<GzOdeCollisionDetector>();
}
#endif

#if HAVE_BULLET
TEST_F(GazeboShim, BulletExtensions)
{
  exerciseShimDetector<GzBulletCollisionDetector>();
}
#endif

} // namespace
