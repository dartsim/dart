/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <gtest/gtest.h>

#include <dart/collision/experimental/narrow_phase/gjk.hpp>
#include <dart/collision/experimental/narrow_phase/mpr.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/BoxShape.hpp>

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

namespace {

constexpr double kTol = 1e-4;
constexpr double kLooseTol = 1e-3;

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

struct BackendResult {
  bool colliding = false;
  double distance = 0.0;
  double penetration = 0.0;
};

BackendResult queryFCL(
    const Eigen::Vector3d& pos1, const dart::dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& pos2, const dart::dynamics::ShapePtr& shape2)
{
  auto cd = dart::collision::FCLCollisionDetector::create();
  
  auto frame1 = dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
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

#if DART_HAVE_BULLET
BackendResult queryBullet(
    const Eigen::Vector3d& pos1, const dart::dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& pos2, const dart::dynamics::ShapePtr& shape2)
{
  auto cd = dart::collision::BulletCollisionDetector::create();
  
  auto frame1 = dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
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
    const Eigen::Vector3d& pos1, const dart::dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& pos2, const dart::dynamics::ShapePtr& shape2)
{
  auto cd = dart::collision::OdeCollisionDetector::create();
  
  auto frame1 = dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
  auto frame2 = dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
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

}  // namespace

class ReferenceBackends : public ::testing::Test {
protected:
  void SetUp() override {
    sphere1_ = std::make_shared<dart::dynamics::SphereShape>(1.0);
    sphere2_ = std::make_shared<dart::dynamics::SphereShape>(1.0);
    box1_ = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0));
    box2_ = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0));
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

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, expectedDepth, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(bullet.colliding);
  EXPECT_NEAR(bullet.penetration, expectedDepth, kLooseTol);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(ode.colliding);
  EXPECT_NEAR(ode.penetration, expectedDepth, kLooseTol);
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

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);

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

  auto fcl = queryFCL(pos1, box1_, pos2, box2_);
  EXPECT_TRUE(fcl.colliding);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, box1_, pos2, box2_);
  EXPECT_TRUE(bullet.colliding);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(pos1, box1_, pos2, box2_);
  EXPECT_TRUE(ode.colliding);
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

  auto fcl = queryFCL(pos1, box1_, pos2, box2_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);

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

  auto fcl = queryFCL(spherePos, sphere1_, boxPos, box1_);
  EXPECT_TRUE(fcl.colliding);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(spherePos, sphere1_, boxPos, box1_);
  EXPECT_TRUE(bullet.colliding);
#endif

#if DART_HAVE_ODE
  auto ode = queryODE(spherePos, sphere1_, boxPos, box1_);
  EXPECT_TRUE(ode.colliding);
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

  auto fcl = queryFCL(spherePos, sphere1_, boxPos, box1_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);

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

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, expectedDepth, kLooseTol);

#if DART_HAVE_BULLET
  auto bullet = queryBullet(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(bullet.colliding);
  EXPECT_NEAR(bullet.penetration, expectedDepth, kLooseTol);
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

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, 0.001, kLooseTol);
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

  auto fcl = queryFCL(pos1, sphere1_, pos2, sphere2_);
  EXPECT_TRUE(fcl.colliding);
  EXPECT_NEAR(fcl.penetration, 0.001, kLooseTol);
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

  auto largeSphere1 = std::make_shared<dart::dynamics::SphereShape>(r1);
  auto largeSphere2 = std::make_shared<dart::dynamics::SphereShape>(r2);
  auto fcl = queryFCL(pos1, largeSphere1, pos2, largeSphere2);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, kLooseTol);
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

  auto smallSphere1 = std::make_shared<dart::dynamics::SphereShape>(r1);
  auto smallSphere2 = std::make_shared<dart::dynamics::SphereShape>(r2);
  auto fcl = queryFCL(pos1, smallSphere1, pos2, smallSphere2);
  EXPECT_FALSE(fcl.colliding);
  EXPECT_NEAR(fcl.distance, expectedDist, 1e-5);
}

