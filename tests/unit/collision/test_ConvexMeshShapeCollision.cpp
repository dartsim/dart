/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/collision/fcl/FCLCollisionObject.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/ConvexMeshShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <Eigen/Core>
#include <fcl/geometry/shape/convex.h>
#include <gtest/gtest.h>

#if DART_HAVE_BULLET
  #include "dart/collision/bullet/BulletCollisionDetector.hpp"
  #include "dart/collision/bullet/BulletCollisionObject.hpp"
#endif
#if DART_HAVE_ODE
  #include "dart/collision/ode/OdeCollisionDetector.hpp"
  #include "dart/collision/ode/OdeCollisionObject.hpp"
#endif

using dart::collision::FCLCollisionDetector;
using dart::collision::FCLCollisionObject;
#if DART_HAVE_BULLET
using dart::collision::BulletCollisionDetector;
using dart::collision::BulletCollisionObject;
#endif
#if DART_HAVE_ODE
using dart::collision::OdeCollisionDetector;
using dart::collision::OdeCollisionObject;
#endif
using dart::dynamics::CollisionAspect;
using dart::dynamics::ConvexMeshShape;

namespace {

struct TestFCLDetector : public FCLCollisionDetector
{
  using FCLCollisionDetector::claimCollisionObject;
};

#if DART_HAVE_BULLET
struct TestBulletDetector : public BulletCollisionDetector
{
  using BulletCollisionDetector::claimCollisionObject;
};
#endif
#if DART_HAVE_ODE
class TestOdeCollisionObject : public OdeCollisionObject
{
public:
  TestOdeCollisionObject(
      OdeCollisionDetector* detector,
      const dart::dynamics::ShapeFrame* shapeFrame)
    : OdeCollisionObject(detector, shapeFrame)
  {
  }

  using OdeCollisionObject::getOdeGeomId;
};

struct TestOdeDetector : public OdeCollisionDetector
{
  using OdeCollisionDetector::claimCollisionObject;

protected:
  std::unique_ptr<dart::collision::CollisionObject> createCollisionObject(
      const dart::dynamics::ShapeFrame* shapeFrame) override
  {
    return std::make_unique<TestOdeCollisionObject>(this, shapeFrame);
  }
};
#endif

std::shared_ptr<ConvexMeshShape> makeTetraShape()
{
  ConvexMeshShape::Vertices vertices
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ()};

  ConvexMeshShape::Triangles triangles
      = {ConvexMeshShape::TriMeshType::Triangle(0, 1, 2),
         ConvexMeshShape::TriMeshType::Triangle(0, 1, 3),
         ConvexMeshShape::TriMeshType::Triangle(0, 2, 3),
         ConvexMeshShape::TriMeshType::Triangle(1, 2, 3)};

  return std::make_shared<ConvexMeshShape>(vertices, triangles);
}

} // namespace

// Regression coverage for https://github.com/dartsim/dart/issues/872.
TEST(ConvexMeshShapeCollision, FclUsesConvexGeometry)
{
  auto detector = std::make_shared<TestFCLDetector>();
  auto skel = dart::dynamics::Skeleton::create("s");
  auto bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = makeTetraShape();
  auto shapeNode = bn->createShapeNodeWith<CollisionAspect>(shape);

  auto collObj = detector->claimCollisionObject(shapeNode);
  ASSERT_NE(collObj, nullptr);

  auto fclObj = static_cast<FCLCollisionObject*>(collObj.get())
                    ->getFCLCollisionObject();
  ASSERT_NE(fclObj, nullptr);
  ASSERT_NE(fclObj->collisionGeometry(), nullptr);

  EXPECT_EQ(fclObj->collisionGeometry()->getNodeType(), ::fcl::GEOM_CONVEX);
}

#if DART_HAVE_BULLET
TEST(ConvexMeshShapeCollision, BulletUsesConvexHullShape)
{
  auto detector = std::make_shared<TestBulletDetector>();
  auto skel = dart::dynamics::Skeleton::create("s");
  auto bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = makeTetraShape();
  auto shapeNode = bn->createShapeNodeWith<CollisionAspect>(shape);

  auto collObj = detector->claimCollisionObject(shapeNode);
  ASSERT_NE(collObj, nullptr);

  auto bulletObj = static_cast<BulletCollisionObject*>(collObj.get())
                       ->getBulletCollisionObject();
  ASSERT_NE(bulletObj, nullptr);
  ASSERT_NE(bulletObj->getCollisionShape(), nullptr);

  EXPECT_EQ(
      bulletObj->getCollisionShape()->getShapeType(),
      CONVEX_HULL_SHAPE_PROXYTYPE);
}
#endif

#if DART_HAVE_ODE
TEST(ConvexMeshShapeCollision, OdeUsesTriMeshGeometry)
{
  auto detector = std::make_shared<TestOdeDetector>();
  auto skel = dart::dynamics::Skeleton::create("s");
  auto bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = makeTetraShape();
  auto shapeNode = bn->createShapeNodeWith<CollisionAspect>(shape);

  auto collObj = detector->claimCollisionObject(shapeNode);
  ASSERT_NE(collObj, nullptr);

  auto odeObj = static_cast<TestOdeCollisionObject*>(collObj.get());
  ASSERT_NE(odeObj, nullptr);

  auto geomId = odeObj->getOdeGeomId();
  ASSERT_NE(geomId, nullptr);

  EXPECT_EQ(dGeomGetClass(geomId), dTriMeshClass);
}
#endif
