/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/gjk.hpp>
#include <dart/collision/experimental/narrow_phase/mpr.hpp>
#include <dart/collision/experimental/narrow_phase/narrow_phase.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include <cmath>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

using dart::collision::experimental::Gjk;
using dart::collision::experimental::GjkResult;
using dart::collision::experimental::Mpr;
using dart::collision::experimental::MprResult;
using dart::collision::experimental::SupportFunction;
namespace expc = dart::collision::experimental;

namespace {

constexpr double kTol = 1e-4;
constexpr double kLooseTol = 1e-3;
constexpr double kRayTol = 1e-4;
constexpr double kRayNormalTol = 1e-3;

SupportFunction makeSphereSupport(const Eigen::Vector3d& center, double radius)
{
  return [center, radius](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    if (dir.squaredNorm() < 1e-12) {
      return center + Eigen::Vector3d(radius, 0, 0);
    }
    return center + radius * dir.normalized();
  };
}

SupportFunction makeBoxSupport(
    const Eigen::Vector3d& center, const Eigen::Vector3d& halfExtents)
{
  return [center, halfExtents](const Eigen::Vector3d& dir) {
    Eigen::Vector3d result = center;
    result.x() += (dir.x() >= 0) ? halfExtents.x() : -halfExtents.x();
    result.y() += (dir.y() >= 0) ? halfExtents.y() : -halfExtents.y();
    result.z() += (dir.z() >= 0) ? halfExtents.z() : -halfExtents.z();
    return result;
  };
}

struct BackendResult
{
  bool colliding = false;
  double distance = 0.0;
  double penetration = 0.0;
};

struct RaycastBackendResult
{
  bool hit = false;
  double distance = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

bool nearVector(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol)
{
  return (a - b).norm() <= tol;
}

BackendResult queryFCL(
    const Eigen::Vector3d& pos1,
    const dart::dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& pos2,
    const dart::dynamics::ShapePtr& shape2)
{
  auto cd = dart::collision::FCLCollisionDetector::create();

  auto frame1 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto frame2 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame1->setShape(shape1);
  frame2->setShape(shape2);
  frame1->setTranslation(pos1);
  frame2->setTranslation(pos2);

  auto group = cd->createCollisionGroup(frame1.get(), frame2.get());

  BackendResult result;

  dart::collision::CollisionOption collOpt;
  collOpt.maxNumContacts = 1;
  dart::collision::CollisionResult collResult;
  result.colliding = group->collide(collOpt, &collResult);

  if (result.colliding && collResult.getNumContacts() > 0) {
    result.penetration = collResult.getContact(0).penetrationDepth;
  }

  dart::collision::DistanceOption distOpt;
  distOpt.enableNearestPoints = true;
  dart::collision::DistanceResult distResult;
  cd->distance(group.get(), distOpt, &distResult);

  if (distResult.found()) {
    result.distance = distResult.minDistance;
  }

  return result;
}

BackendResult queryExperimental(
    const Eigen::Vector3d& pos1,
    std::unique_ptr<expc::Shape> shape1,
    const Eigen::Vector3d& pos2,
    std::unique_ptr<expc::Shape> shape2)
{
  expc::CollisionWorld world;

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf1.translation() = pos1;
  tf2.translation() = pos2;

  auto obj1 = world.createObject(std::move(shape1), tf1);
  auto obj2 = world.createObject(std::move(shape2), tf2);

  BackendResult result;

  expc::DistanceResult distResult;
  expc::DistanceOption distOption;
  result.distance
      = expc::NarrowPhase::distance(obj1, obj2, distOption, distResult);

  expc::CollisionOption collOpt = expc::CollisionOption::fullContacts(1);
  expc::CollisionResult collResult;
  result.colliding
      = expc::NarrowPhase::collide(obj1, obj2, collOpt, collResult);

  if (result.colliding && collResult.numContacts() > 0) {
    result.penetration = collResult.getContact(0).depth;
  }

  return result;
}

#if DART_HAVE_BULLET
RaycastBackendResult queryBulletRaycast(
    const Eigen::Vector3d& pos,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to)
{
  auto cd = dart::collision::BulletCollisionDetector::create();

  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame->setShape(shape);
  frame->setTranslation(pos);

  auto group = cd->createCollisionGroup(frame.get());

  dart::collision::RaycastOption option;
  option.mEnableAllHits = false;
  option.mSortByClosest = false;

  dart::collision::RaycastResult result;
  cd->raycast(group.get(), from, to, option, &result);

  RaycastBackendResult out;
  if (!result.hasHit() || result.mRayHits.empty()) {
    return out;
  }

  const auto& hit = result.mRayHits.front();
  out.hit = true;
  out.point = hit.mPoint;
  out.normal = hit.mNormal;
  out.distance = hit.mFraction * (to - from).norm();

  return out;
}
#endif

RaycastBackendResult queryExperimentalRaycast(
    const Eigen::Vector3d& pos,
    std::unique_ptr<expc::Shape> shape,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to)
{
  expc::CollisionWorld world;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = pos;
  world.createObject(std::move(shape), tf);

  const Eigen::Vector3d delta = to - from;
  const double length = delta.norm();
  if (length < 1e-12) {
    return RaycastBackendResult();
  }

  expc::Ray ray(from, delta.normalized(), length);
  expc::RaycastOption option;
  expc::RaycastResult result;

  RaycastBackendResult out;
  if (world.raycast(ray, option, result)) {
    out.hit = true;
    out.distance = result.distance;
    out.point = result.point;
    out.normal = result.normal;
  }

  return out;
}

#if DART_HAVE_BULLET
BackendResult queryBullet(
    const Eigen::Vector3d& pos1,
    const dart::dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& pos2,
    const dart::dynamics::ShapePtr& shape2)
{
  auto cd = dart::collision::BulletCollisionDetector::create();

  auto frame1 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto frame2 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame1->setShape(shape1);
  frame2->setShape(shape2);
  frame1->setTranslation(pos1);
  frame2->setTranslation(pos2);

  auto group = cd->createCollisionGroup(frame1.get(), frame2.get());

  BackendResult result;

  dart::collision::CollisionOption collOpt;
  collOpt.maxNumContacts = 1;
  dart::collision::CollisionResult collResult;
  result.colliding = group->collide(collOpt, &collResult);

  if (result.colliding && collResult.getNumContacts() > 0) {
    result.penetration = collResult.getContact(0).penetrationDepth;
  }

  return result;
}
#endif

#if DART_HAVE_ODE
BackendResult queryODE(
    const Eigen::Vector3d& pos1,
    const dart::dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& pos2,
    const dart::dynamics::ShapePtr& shape2)
{
  auto cd = dart::collision::OdeCollisionDetector::create();

  auto frame1 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto frame2 = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame1->setShape(shape1);
  frame2->setShape(shape2);
  frame1->setTranslation(pos1);
  frame2->setTranslation(pos2);

  auto group = cd->createCollisionGroup(frame1.get(), frame2.get());

  BackendResult result;

  dart::collision::CollisionOption collOpt;
  collOpt.maxNumContacts = 1;
  dart::collision::CollisionResult collResult;
  result.colliding = group->collide(collOpt, &collResult);

  if (result.colliding && collResult.getNumContacts() > 0) {
    result.penetration = collResult.getContact(0).penetrationDepth;
  }

  return result;
}
#endif

} // namespace

class ReferenceBackends : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sphere1_ = std::make_shared<dart::dynamics::SphereShape>(1.0);
    sphere2_ = std::make_shared<dart::dynamics::SphereShape>(1.0);
    box1_ = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(2.0, 2.0, 2.0));
    box2_ = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(2.0, 2.0, 2.0));
  }

  dart::dynamics::ShapePtr sphere1_, sphere2_;
  dart::dynamics::ShapePtr box1_, box2_;
};

TEST_F(ReferenceBackends, SphereSphereIntersecting)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(1.5, 0.0, 0.0);
  const double r1 = 1.0, r2 = 1.0;

  auto supportA = makeSphereSupport(pos1, r1);
  auto supportB = makeSphereSupport(pos2, r2);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB, pos2 - pos1));

  MprResult mpr = Mpr::penetration(supportA, supportB, pos1, pos2);
  ASSERT_TRUE(mpr.success);

  const double expectedDepth = r1 + r2 - (pos2 - pos1).norm();
  EXPECT_NEAR(mpr.depth, expectedDepth, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(r1),
      pos2,
      std::make_unique<expc::SphereShape>(r2));
  EXPECT_TRUE(experimental.colliding);
  EXPECT_NEAR(experimental.penetration, expectedDepth, kTol);

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, fcl.penetration, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(bullet.colliding);
  EXPECT_NEAR(bullet.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, bullet.penetration, kLooseTol);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(ode.colliding);
  EXPECT_NEAR(ode.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, ode.penetration, kLooseTol);
#endif
}

TEST_F(ReferenceBackends, SphereSphereSeparated)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(3.0, 0.0, 0.0);
  const double r1 = 1.0, r2 = 1.0;

  auto supportA = makeSphereSupport(pos1, r1);
  auto supportB = makeSphereSupport(pos2, r2);

  GjkResult gjk = Gjk::query(supportA, supportB, pos2 - pos1);
  EXPECT_FALSE(gjk.intersecting);

  const double expectedDist = (pos2 - pos1).norm() - r1 - r2;
  EXPECT_NEAR(gjk.distance, expectedDist, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(r1),
      pos2,
      std::make_unique<expc::SphereShape>(r2));
  EXPECT_FALSE(experimental.colliding);
  EXPECT_NEAR(experimental.distance, expectedDist, kTol);

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);
  EXPECT_NEAR(experimental.distance, fcl.distance, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, sphere1_, pos2, sphere2_);
  EXPECT_FALSE(bullet.colliding);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(pos1, sphere1_, pos2, sphere2_);
  EXPECT_FALSE(ode.colliding);
#endif
}

TEST_F(ReferenceBackends, BoxBoxIntersecting)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(1.5, 0.5, 0.0);
  const Eigen::Vector3d halfExt(1.0, 1.0, 1.0);

  auto supportA = makeBoxSupport(pos1, halfExt);
  auto supportB = makeBoxSupport(pos2, halfExt);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB, pos2 - pos1));

  const double overlapX = 2.0 - std::abs(pos2.x() - pos1.x());
  const double overlapY = 2.0 - std::abs(pos2.y() - pos1.y());
  const double overlapZ = 2.0 - std::abs(pos2.z() - pos1.z());
  const double expectedDepth = std::min({overlapX, overlapY, overlapZ});

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::BoxShape>(halfExt),
      pos2,
      std::make_unique<expc::BoxShape>(halfExt));
  EXPECT_TRUE(experimental.colliding);
  EXPECT_NEAR(experimental.penetration, expectedDepth, kLooseTol);

  auto fcl = queryFCL(pos1, box1_, pos2, box2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, fcl.penetration, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, box1_, pos2, box2_);
  EXPECT_TRUE(bullet.colliding);
  EXPECT_NEAR(experimental.penetration, bullet.penetration, kLooseTol);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(pos1, box1_, pos2, box2_);
  EXPECT_TRUE(ode.colliding);
  EXPECT_NEAR(experimental.penetration, ode.penetration, kLooseTol);
#endif
}

TEST_F(ReferenceBackends, BoxBoxSeparated)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(3.0, 0.0, 0.0);
  const Eigen::Vector3d halfExt(1.0, 1.0, 1.0);

  auto supportA = makeBoxSupport(pos1, halfExt);
  auto supportB = makeBoxSupport(pos2, halfExt);

  GjkResult gjk = Gjk::query(supportA, supportB, pos2 - pos1);
  EXPECT_FALSE(gjk.intersecting);

  const double expectedDist = (pos2 - pos1).norm() - 2.0;
  EXPECT_NEAR(gjk.distance, expectedDist, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::BoxShape>(halfExt),
      pos2,
      std::make_unique<expc::BoxShape>(halfExt));
  EXPECT_FALSE(experimental.colliding);
  EXPECT_NEAR(experimental.distance, expectedDist, kTol);

  auto fcl = queryFCL(pos1, box1_, pos2, box2_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);
  EXPECT_NEAR(experimental.distance, fcl.distance, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, box1_, pos2, box2_);
  EXPECT_FALSE(bullet.colliding);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(pos1, box1_, pos2, box2_);
  EXPECT_FALSE(ode.colliding);
#endif
}

TEST_F(ReferenceBackends, SphereBoxIntersecting)
{
  const Eigen::Vector3d spherePos(0.0, 0.0, 0.0);
  const Eigen::Vector3d boxPos(1.8, 0.0, 0.0);
  const Eigen::Vector3d boxHalf(1.0, 1.0, 1.0);

  auto supportA = makeSphereSupport(spherePos, 1.0);
  auto supportB = makeBoxSupport(boxPos, boxHalf);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB, boxPos - spherePos));

  const double expectedDepth = 1.0 - (boxPos.x() - boxHalf.x());

  auto experimental = queryExperimental(
      spherePos,
      std::make_unique<expc::SphereShape>(1.0),
      boxPos,
      std::make_unique<expc::BoxShape>(boxHalf));
  EXPECT_TRUE(experimental.colliding);
  EXPECT_NEAR(experimental.penetration, expectedDepth, kLooseTol);

  auto fcl = queryFCL(spherePos, sphere1_, boxPos, box1_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, fcl.penetration, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(spherePos, sphere1_, boxPos, box1_);
  EXPECT_TRUE(bullet.colliding);
  EXPECT_NEAR(experimental.penetration, bullet.penetration, kLooseTol);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(spherePos, sphere1_, boxPos, box1_);
  EXPECT_TRUE(ode.colliding);
  EXPECT_NEAR(experimental.penetration, ode.penetration, kLooseTol);
#endif
}

TEST_F(ReferenceBackends, SphereBoxSeparated)
{
  const Eigen::Vector3d spherePos(0.0, 0.0, 0.0);
  const Eigen::Vector3d boxPos(3.0, 0.0, 0.0);
  const Eigen::Vector3d boxHalf(1.0, 1.0, 1.0);

  auto supportA = makeSphereSupport(spherePos, 1.0);
  auto supportB = makeBoxSupport(boxPos, boxHalf);

  GjkResult gjk = Gjk::query(supportA, supportB, boxPos - spherePos);
  EXPECT_FALSE(gjk.intersecting);

  const double expectedDist = (boxPos.x() - boxHalf.x()) - 1.0;
  EXPECT_NEAR(gjk.distance, expectedDist, kTol);

  auto experimental = queryExperimental(
      spherePos,
      std::make_unique<expc::SphereShape>(1.0),
      boxPos,
      std::make_unique<expc::BoxShape>(boxHalf));
  EXPECT_FALSE(experimental.colliding);
  EXPECT_NEAR(experimental.distance, expectedDist, kTol);

  auto fcl = queryFCL(spherePos, sphere1_, boxPos, box1_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);
  EXPECT_NEAR(experimental.distance, fcl.distance, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(spherePos, sphere1_, boxPos, box1_);
  EXPECT_FALSE(bullet.colliding);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(spherePos, sphere1_, boxPos, box1_);
  EXPECT_FALSE(ode.colliding);
#endif
}

TEST_F(ReferenceBackends, DeepPenetration)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(0.5, 0.0, 0.0);
  const double r1 = 1.0, r2 = 1.0;

  auto supportA = makeSphereSupport(pos1, r1);
  auto supportB = makeSphereSupport(pos2, r2);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB, pos2 - pos1));

  MprResult mpr = Mpr::penetration(supportA, supportB, pos1, pos2);
  ASSERT_TRUE(mpr.success);

  const double expectedDepth = r1 + r2 - (pos2 - pos1).norm();
  EXPECT_NEAR(mpr.depth, expectedDepth, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(r1),
      pos2,
      std::make_unique<expc::SphereShape>(r2));
  EXPECT_TRUE(experimental.colliding);
  EXPECT_NEAR(experimental.penetration, expectedDepth, kTol);

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, fcl.penetration, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(bullet.colliding);
  EXPECT_NEAR(bullet.penetration, expectedDepth, kLooseTol);
  EXPECT_NEAR(experimental.penetration, bullet.penetration, kLooseTol);
#endif
}

TEST_F(ReferenceBackends, SmallGap)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(2.001, 0.0, 0.0);

  auto supportA = makeSphereSupport(pos1, 1.0);
  auto supportB = makeSphereSupport(pos2, 1.0);

  GjkResult gjk = Gjk::query(supportA, supportB, pos2 - pos1);
  EXPECT_FALSE(gjk.intersecting);
  EXPECT_NEAR(gjk.distance, 0.001, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(1.0),
      pos2,
      std::make_unique<expc::SphereShape>(1.0));
  EXPECT_FALSE(experimental.colliding);
  EXPECT_NEAR(experimental.distance, 0.001, kTol);

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, 0.001, kLooseTol);
  EXPECT_NEAR(experimental.distance, fcl.distance, kLooseTol);
}

TEST_F(ReferenceBackends, SmallOverlap)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(1.999, 0.0, 0.0);

  auto supportA = makeSphereSupport(pos1, 1.0);
  auto supportB = makeSphereSupport(pos2, 1.0);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB, pos2 - pos1));

  MprResult mpr = Mpr::penetration(supportA, supportB, pos1, pos2);
  ASSERT_TRUE(mpr.success);
  EXPECT_NEAR(mpr.depth, 0.001, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(1.0),
      pos2,
      std::make_unique<expc::SphereShape>(1.0));
  EXPECT_TRUE(experimental.colliding);
  EXPECT_NEAR(experimental.penetration, 0.001, kTol);

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, 0.001, kLooseTol);
  EXPECT_NEAR(experimental.penetration, fcl.penetration, kLooseTol);
}

TEST_F(ReferenceBackends, LargeScale)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(1000.0, 0.0, 0.0);
  const double r1 = 100.0, r2 = 100.0;

  auto supportA = makeSphereSupport(pos1, r1);
  auto supportB = makeSphereSupport(pos2, r2);

  GjkResult gjk = Gjk::query(supportA, supportB, pos2 - pos1);
  EXPECT_FALSE(gjk.intersecting);

  const double expectedDist = (pos2 - pos1).norm() - r1 - r2;
  EXPECT_NEAR(gjk.distance, expectedDist, kTol);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(r1),
      pos2,
      std::make_unique<expc::SphereShape>(r2));
  EXPECT_FALSE(experimental.colliding);
  EXPECT_NEAR(experimental.distance, expectedDist, 1e-6);

  auto largeSphere1 = std::make_shared<dart::dynamics::SphereShape>(r1);
  auto largeSphere2 = std::make_shared<dart::dynamics::SphereShape>(r2);
  auto fcl = queryFCL(pos1, largeSphere1, pos2, largeSphere2);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);
  EXPECT_NEAR(experimental.distance, fcl.distance, kLooseTol);
}

TEST_F(ReferenceBackends, SmallScale)
{
  const Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  const Eigen::Vector3d pos2(0.01, 0.0, 0.0);
  const double r1 = 0.001, r2 = 0.001;

  auto supportA = makeSphereSupport(pos1, r1);
  auto supportB = makeSphereSupport(pos2, r2);

  GjkResult gjk = Gjk::query(supportA, supportB, pos2 - pos1);
  EXPECT_FALSE(gjk.intersecting);

  const double expectedDist = (pos2 - pos1).norm() - r1 - r2;
  EXPECT_NEAR(gjk.distance, expectedDist, 1e-6);

  auto experimental = queryExperimental(
      pos1,
      std::make_unique<expc::SphereShape>(r1),
      pos2,
      std::make_unique<expc::SphereShape>(r2));
  EXPECT_FALSE(experimental.colliding);
  EXPECT_NEAR(experimental.distance, expectedDist, 1e-6);

  auto smallSphere1 = std::make_shared<dart::dynamics::SphereShape>(r1);
  auto smallSphere2 = std::make_shared<dart::dynamics::SphereShape>(r2);
  auto fcl = queryFCL(pos1, smallSphere1, pos2, smallSphere2);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, 1e-5);
  EXPECT_NEAR(experimental.distance, fcl.distance, 1e-4);
}

#if DART_HAVE_BULLET
TEST_F(ReferenceBackends, RaycastSphereAgainstBullet)
{
  const Eigen::Vector3d pos(0.0, 0.0, 0.0);
  const Eigen::Vector3d from(-2.0, 0.0, 0.0);
  const Eigen::Vector3d to(2.0, 0.0, 0.0);

  auto experimental = queryExperimentalRaycast(
      pos, std::make_unique<expc::SphereShape>(1.0), from, to);
  auto bullet = queryBulletRaycast(pos, sphere1_, from, to);

  ASSERT_TRUE(experimental.hit);
  ASSERT_TRUE(bullet.hit);

  EXPECT_NEAR(experimental.distance, 1.0, kRayTol);
  EXPECT_NEAR(bullet.distance, 1.0, kRayTol);
  EXPECT_NEAR(experimental.distance, bullet.distance, kRayTol);

  EXPECT_TRUE(
      nearVector(experimental.point, Eigen::Vector3d(-1.0, 0.0, 0.0), kRayTol));
  EXPECT_TRUE(
      nearVector(bullet.point, Eigen::Vector3d(-1.0, 0.0, 0.0), kRayTol));

  EXPECT_GE(
      experimental.normal.normalized().dot(bullet.normal.normalized()),
      1.0 - kRayNormalTol);
}

TEST_F(ReferenceBackends, RaycastBoxAgainstBullet)
{
  const Eigen::Vector3d pos(0.0, 0.0, 0.0);
  const Eigen::Vector3d from(-3.0, 0.0, 0.0);
  const Eigen::Vector3d to(3.0, 0.0, 0.0);
  const Eigen::Vector3d halfExt(1.0, 1.0, 1.0);

  auto experimental = queryExperimentalRaycast(
      pos, std::make_unique<expc::BoxShape>(halfExt), from, to);
  auto bullet = queryBulletRaycast(pos, box1_, from, to);

  ASSERT_TRUE(experimental.hit);
  ASSERT_TRUE(bullet.hit);

  EXPECT_NEAR(experimental.distance, 2.0, kRayTol);
  EXPECT_NEAR(bullet.distance, 2.0, kRayTol);
  EXPECT_NEAR(experimental.distance, bullet.distance, kRayTol);

  EXPECT_TRUE(
      nearVector(experimental.point, Eigen::Vector3d(-1.0, 0.0, 0.0), kRayTol));
  EXPECT_TRUE(
      nearVector(bullet.point, Eigen::Vector3d(-1.0, 0.0, 0.0), kRayTol));

  EXPECT_GE(
      experimental.normal.normalized().dot(bullet.normal.normalized()),
      1.0 - kRayNormalTol);
}
#endif
