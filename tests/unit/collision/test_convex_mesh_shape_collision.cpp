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

#include "dart/collision/fcl/fcl_collision_detector.hpp"
#include "dart/collision/fcl/fcl_collision_object.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/convex_mesh_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fcl/geometry/shape/convex.h>
#include <gtest/gtest.h>

#include <array>
#include <random>

#if DART_HAVE_BULLET
  #include "dart/collision/bullet/bullet_collision_detector.hpp"
  #include "dart/collision/bullet/bullet_collision_object.hpp"
#endif
#if DART_HAVE_ODE
  #include "dart/collision/ode/ode_collision_detector.hpp"
  #include "dart/collision/ode/ode_collision_object.hpp"
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

constexpr double kPi = 3.141592653589793;

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

std::shared_ptr<ConvexMeshShape> makeEmptyConvexMeshShape()
{
  auto mesh = std::make_shared<ConvexMeshShape::TriMeshType>();
  return std::make_shared<ConvexMeshShape>(mesh);
}

Eigen::Isometry3d makeRandomTransform(
    std::mt19937& rng, double translationBound, double rotationBound)
{
  std::uniform_real_distribution<double> dist(
      -translationBound, translationBound);
  std::uniform_real_distribution<double> angleDist(
      -rotationBound, rotationBound);

  Eigen::Vector3d axis(dist(rng), dist(rng), dist(rng));
  if (axis.norm() < 1e-8) {
    axis = Eigen::Vector3d::UnitX();
  } else {
    axis.normalize();
  }

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
  tf.linear() = Eigen::AngleAxisd(angleDist(rng), axis).toRotationMatrix();

  return tf;
}

std::shared_ptr<ConvexMeshShape> makeRandomConvexMeshShape(
    std::mt19937& rng,
    double scaleMin,
    double scaleMax,
    int duplicateStride = 0)
{
  std::uniform_real_distribution<double> scaleDist(scaleMin, scaleMax);
  const Eigen::Vector3d scale(scaleDist(rng), scaleDist(rng), scaleDist(rng));
  ConvexMeshShape::Vertices vertices
      = {Eigen::Vector3d(1.0, 1.0, 1.0),
         Eigen::Vector3d(-1.0, -1.0, 1.0),
         Eigen::Vector3d(-1.0, 1.0, -1.0),
         Eigen::Vector3d(1.0, -1.0, -1.0)};

  for (auto& vertex : vertices) {
    vertex = vertex.cwiseProduct(scale);
  }

  auto mesh = std::make_shared<ConvexMeshShape::TriMeshType>();
  mesh->reserveVertices(
      vertices.size() + (duplicateStride > 0 ? vertices.size() : 0));
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    mesh->addVertex(vertices[i]);
    if (duplicateStride > 0 && (static_cast<int>(i) % duplicateStride == 0)) {
      mesh->addVertex(vertices[i]);
    }
  }

  return ConvexMeshShape::fromMesh(mesh, true);
}

bool detectCollision(
    const std::shared_ptr<dart::collision::CollisionDetector>& detector,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Isometry3d& tfA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Isometry3d& tfB)
{
  auto frameA = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto frameB = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());

  frameA->setShape(shapeA);
  frameB->setShape(shapeB);

  frameA->setTransform(tfA);
  frameB->setTransform(tfB);

  auto groupA = detector->createCollisionGroup(frameA.get());
  auto groupB = detector->createCollisionGroup(frameB.get());

  dart::collision::CollisionResult result;
  dart::collision::CollisionOption option(true, 1u, nullptr);
  return detector->collide(groupA.get(), groupB.get(), option, &result);
}

void runRandomizedConvexMeshCollisionChecks(
    const std::shared_ptr<dart::collision::CollisionDetector>& detector,
    const std::string& label)
{
  const auto box = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(0.2));

  const std::array<unsigned int, 3> seeds = {11u, 29u, 71u};
  for (const auto seed : seeds) {
    std::mt19937 rng(seed);
    auto convexShape = makeRandomConvexMeshShape(rng, 0.05, 0.2, 5);
    ASSERT_NE(convexShape, nullptr) << label << " seed " << seed;
    ASSERT_NE(convexShape->getMesh(), nullptr) << label << " seed " << seed;
    ASSERT_FALSE(convexShape->getMesh()->getTriangles().empty())
        << label << " seed " << seed;
    ASSERT_FALSE(convexShape->getMesh()->getVertices().empty())
        << label << " seed " << seed;

    const auto meshTf = makeRandomTransform(rng, 0.05, kPi / 4.0);
    const Eigen::Vector3d vertex
        = convexShape->getMesh()->getVertices().front();
    Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
    boxTf.translation() = meshTf * vertex;
    EXPECT_TRUE(detectCollision(detector, convexShape, meshTf, box, boxTf))
        << label << " seed " << seed;

    Eigen::Isometry3d farBoxTf = boxTf;
    farBoxTf.translation() += Eigen::Vector3d(5.0, 0.0, 0.0);
    EXPECT_FALSE(detectCollision(detector, convexShape, meshTf, box, farBoxTf))
        << label << " seed " << seed;
  }
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

TEST(ConvexMeshShapeCollision, FclFallsBackWithoutVertices)
{
  auto detector = std::make_shared<TestFCLDetector>();
  auto skel = dart::dynamics::Skeleton::create("s");
  auto bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = makeEmptyConvexMeshShape();
  auto shapeNode = bn->createShapeNodeWith<CollisionAspect>(shape);

  auto collObj = detector->claimCollisionObject(shapeNode);
  ASSERT_NE(collObj, nullptr);

  auto fclObj = static_cast<FCLCollisionObject*>(collObj.get())
                    ->getFCLCollisionObject();
  ASSERT_NE(fclObj, nullptr);
  ASSERT_NE(fclObj->collisionGeometry(), nullptr);

  EXPECT_NE(fclObj->collisionGeometry()->getNodeType(), ::fcl::GEOM_CONVEX);
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

TEST(ConvexMeshShapeCollision, BulletFallsBackWithoutVertices)
{
  auto detector = std::make_shared<TestBulletDetector>();
  auto skel = dart::dynamics::Skeleton::create("s");
  auto bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = makeEmptyConvexMeshShape();
  auto shapeNode = bn->createShapeNodeWith<CollisionAspect>(shape);

  auto collObj = detector->claimCollisionObject(shapeNode);
  ASSERT_NE(collObj, nullptr);

  auto bulletObj = static_cast<BulletCollisionObject*>(collObj.get())
                       ->getBulletCollisionObject();
  ASSERT_NE(bulletObj, nullptr);
  ASSERT_NE(bulletObj->getCollisionShape(), nullptr);

  EXPECT_EQ(
      bulletObj->getCollisionShape()->getShapeType(), SPHERE_SHAPE_PROXYTYPE);
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

TEST(ConvexMeshShapeCollision, OdeFallsBackWithoutVertices)
{
  auto detector = std::make_shared<TestOdeDetector>();
  auto skel = dart::dynamics::Skeleton::create("s");
  auto bn
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>().second;
  auto shape = makeEmptyConvexMeshShape();
  auto shapeNode = bn->createShapeNodeWith<CollisionAspect>(shape);

  auto collObj = detector->claimCollisionObject(shapeNode);
  ASSERT_NE(collObj, nullptr);

  auto odeObj = static_cast<TestOdeCollisionObject*>(collObj.get());
  ASSERT_NE(odeObj, nullptr);

  auto geomId = odeObj->getOdeGeomId();
  ASSERT_NE(geomId, nullptr);

  EXPECT_EQ(dGeomGetClass(geomId), dSphereClass);
}
#endif

TEST(ConvexMeshShapeCollision, FclRandomizedCollisionCases)
{
  auto detector = FCLCollisionDetector::create();
  runRandomizedConvexMeshCollisionChecks(detector, "FCL");
}

#if DART_HAVE_BULLET
TEST(ConvexMeshShapeCollision, BulletRandomizedCollisionCases)
{
  auto detector = BulletCollisionDetector::create();
  runRandomizedConvexMeshCollisionChecks(detector, "Bullet");
}
#endif

#if DART_HAVE_ODE
TEST(ConvexMeshShapeCollision, OdeRandomizedCollisionCases)
{
  auto detector = OdeCollisionDetector::create();
  runRandomizedConvexMeshCollisionChecks(detector, "ODE");
}
#endif
