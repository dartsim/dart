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

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionGroup.hpp>
#include <dart/collision/native/NativeCollisionObject.hpp>
#include <dart/collision/native/detail/NativeShapeConversion.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/MultiSphereConvexHullShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PyramidShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <dart/math/TriMesh.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

using namespace dart;

namespace {

namespace native = dart::collision::native;

//==============================================================================
class ExposedNativeCollisionObject final
  : public collision::NativeCollisionObject
{
public:
  ExposedNativeCollisionObject(
      collision::CollisionDetector* detector,
      const dynamics::ShapeFrame* shapeFrame)
    : NativeCollisionObject(detector, shapeFrame)
  {
    // Do nothing
  }

  using NativeCollisionObject::updateEngineData;
};

//==============================================================================
class ExposedNativeCollisionDetector final
  : public collision::NativeCollisionDetector
{
public:
  ExposedNativeCollisionDetector() = default;

  using collision::CollisionDetector::claimCollisionObject;

private:
  std::unique_ptr<collision::CollisionObject> createCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override
  {
    return std::make_unique<ExposedNativeCollisionObject>(this, shapeFrame);
  }
};

//==============================================================================
dynamics::SimpleFramePtr makeFrame(
    const dynamics::ShapePtr& shape,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
  auto frame = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  frame->setShape(shape);
  frame->setTranslation(translation);
  return frame;
}

//==============================================================================
class ShapeFramePairCollisionFilter : public collision::CollisionFilter
{
public:
  void addIgnoredPair(
      const dynamics::ShapeFrame* frame1, const dynamics::ShapeFrame* frame2)
  {
    mIgnoredPairs.emplace_back(frame1, frame2);
  }

  bool ignoresCollision(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    const auto* frame1 = object1->getShapeFrame();
    const auto* frame2 = object2->getShapeFrame();
    for (const auto& pair : mIgnoredPairs) {
      if ((frame1 == pair.first && frame2 == pair.second)
          || (frame1 == pair.second && frame2 == pair.first)) {
        return true;
      }
    }

    return false;
  }

private:
  std::vector<
      std::pair<const dynamics::ShapeFrame*, const dynamics::ShapeFrame*>>
      mIgnoredPairs;
};

//==============================================================================
dynamics::ConvexMeshShape::Vertices makeCubeVertices(double halfExtent = 0.5)
{
  const double h = halfExtent;
  return {
      {-h, -h, -h},
      {h, -h, -h},
      {h, h, -h},
      {-h, h, -h},
      {-h, -h, h},
      {h, -h, h},
      {h, h, h},
      {-h, h, h}};
}

//==============================================================================
dynamics::ConvexMeshShape::Triangles makeCubeTriangles()
{
  return {
      {0, 1, 2},
      {0, 2, 3},
      {4, 6, 5},
      {4, 7, 6},
      {0, 5, 1},
      {0, 4, 5},
      {2, 6, 7},
      {2, 7, 3},
      {0, 7, 4},
      {0, 3, 7},
      {1, 5, 6},
      {1, 6, 2}};
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> makePlaneTriMesh()
{
  auto mesh = std::make_shared<math::TriMesh<double>>();
  mesh->addVertex(-1.0, -1.0, 0.0);
  mesh->addVertex(1.0, -1.0, 0.0);
  mesh->addVertex(1.0, 1.0, 0.0);
  mesh->addVertex(-1.0, 1.0, 0.0);
  mesh->addTriangle(0, 1, 2);
  mesh->addTriangle(0, 2, 3);
  return mesh;
}

} // namespace

//==============================================================================
TEST(NativeCollisionDetector, FactoryKeyIsRegisteredInP3b)
{
  auto* factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(nullptr, factory);
  ASSERT_TRUE(
      factory->canCreate(collision::NativeCollisionDetector::getStaticType()));
  auto detector
      = factory->create(collision::NativeCollisionDetector::getStaticType());
  ASSERT_NE(nullptr, detector);
  EXPECT_EQ(
      collision::NativeCollisionDetector::getStaticType(), detector->getType());
}

//==============================================================================
TEST(NativeCollisionDetector, CreatesDetectorGroupAndCollidesSphereSphere)
{
  auto detector = collision::NativeCollisionDetector::create();
  ASSERT_NE(nullptr, detector);
  EXPECT_EQ("native", detector->getType());

  auto clone = detector->cloneWithoutCollisionObjects();
  ASSERT_NE(nullptr, clone);
  EXPECT_EQ("native", clone->getType());

  auto frame = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));
  auto group = detector->createCollisionGroup(frame.get());
  ASSERT_NE(nullptr, group);
  EXPECT_NE(
      nullptr, dynamic_cast<collision::NativeCollisionGroup*>(group.get()));
  EXPECT_EQ(1u, group->getNumShapeFrames());

  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  group->addShapeFrame(frame2.get());

  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  ASSERT_NE(nullptr, contact.collisionObject1);
  ASSERT_NE(nullptr, contact.collisionObject2);
  EXPECT_EQ(frame.get(), contact.collisionObject1->getShapeFrame());
  EXPECT_EQ(frame2.get(), contact.collisionObject2->getShapeFrame());
  EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.25, contact.penetrationDepth, 1e-12);
  EXPECT_TRUE(result.inCollision(frame.get()));
  EXPECT_TRUE(result.inCollision(frame2.get()));
}

//==============================================================================
TEST(NativeCollisionDetector, BinaryCheckEmitsPairOnlyContact)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(false, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  ASSERT_NE(nullptr, contact.collisionObject1);
  ASSERT_NE(nullptr, contact.collisionObject2);
  EXPECT_EQ(frame1.get(), contact.collisionObject1->getShapeFrame());
  EXPECT_EQ(frame2.get(), contact.collisionObject2->getShapeFrame());
  EXPECT_TRUE(contact.normal.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_NEAR(0.0, contact.penetrationDepth, 1e-12);
  EXPECT_TRUE(result.inCollision(frame1.get()));
  EXPECT_TRUE(result.inCollision(frame2.get()));
}

//==============================================================================
TEST(NativeCollisionDetector, RespectsGlobalContactLimit)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  auto frame3 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.0, 0.75, 0.0));
  auto group = detector->createCollisionGroup(
      frame1.get(), frame2.get(), frame3.get());

  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 1u), &result));
  EXPECT_EQ(1u, result.getNumContacts());

  collision::CollisionResult zeroLimitResult;
  EXPECT_FALSE(
      group->collide(collision::CollisionOption(true, 0u), &zeroLimitResult));
  EXPECT_EQ(0u, zeroLimitResult.getNumContacts());
}

//==============================================================================
TEST(NativeCollisionDetector, RespectsCollisionFilter)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  auto filter = std::make_shared<ShapeFramePairCollisionFilter>();
  filter->addIgnoredPair(frame1.get(), frame2.get());

  collision::CollisionResult result;
  EXPECT_FALSE(
      group->collide(collision::CollisionOption(true, 10u, filter), &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesSphereBoxInBothOrders)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto sphereFrame = makeFrame(
      std::make_shared<dynamics::SphereShape>(1.0),
      Eigen::Vector3d(0.0, 0.0, 1.5));
  auto boxFrame = makeFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  auto sphereFirstGroup
      = detector->createCollisionGroup(sphereFrame.get(), boxFrame.get());
  collision::CollisionResult sphereFirstResult;
  EXPECT_TRUE(sphereFirstGroup->collide(
      collision::CollisionOption(true, 10u), &sphereFirstResult));
  ASSERT_EQ(1u, sphereFirstResult.getNumContacts());

  const auto& sphereFirstContact = sphereFirstResult.getContact(0);
  EXPECT_EQ(sphereFrame.get(), sphereFirstContact.getShapeFrame1());
  EXPECT_EQ(boxFrame.get(), sphereFirstContact.getShapeFrame2());
  EXPECT_TRUE(
      sphereFirstContact.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(0.5, sphereFirstContact.penetrationDepth, 1e-12);

  auto boxFirstGroup
      = detector->createCollisionGroup(boxFrame.get(), sphereFrame.get());
  collision::CollisionResult boxFirstResult;
  EXPECT_TRUE(boxFirstGroup->collide(
      collision::CollisionOption(true, 10u), &boxFirstResult));
  ASSERT_EQ(1u, boxFirstResult.getNumContacts());

  const auto& boxFirstContact = boxFirstResult.getContact(0);
  EXPECT_EQ(boxFrame.get(), boxFirstContact.getShapeFrame1());
  EXPECT_EQ(sphereFrame.get(), boxFirstContact.getShapeFrame2());
  EXPECT_TRUE(
      boxFirstContact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(0.5, boxFirstContact.penetrationDepth, 1e-12);
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesCapsuleSphere)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto capsuleFrame
      = makeFrame(std::make_shared<dynamics::CapsuleShape>(0.5, 2.0));
  auto sphereFrame = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));

  auto group
      = detector->createCollisionGroup(capsuleFrame.get(), sphereFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  EXPECT_EQ(capsuleFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(sphereFrame.get(), contact.getShapeFrame2());
  EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.25, contact.penetrationDepth, 1e-12);
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesCapsuleBox)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto capsuleFrame = makeFrame(
      std::make_shared<dynamics::CapsuleShape>(0.5, 2.0),
      Eigen::Vector3d(1.25, 0.0, 0.0));
  auto boxFrame = makeFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)));

  auto group
      = detector->createCollisionGroup(capsuleFrame.get(), boxFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GE(result.getNumContacts(), 1u);

  const auto& contact = result.getContact(0);
  EXPECT_EQ(capsuleFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(boxFrame.get(), contact.getShapeFrame2());
  EXPECT_TRUE(contact.normal.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesCapsuleCapsule)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto capsuleFrame1
      = makeFrame(std::make_shared<dynamics::CapsuleShape>(0.5, 2.0));
  auto capsuleFrame2 = makeFrame(
      std::make_shared<dynamics::CapsuleShape>(0.5, 2.0),
      Eigen::Vector3d(0.75, 0.0, 0.0));

  auto group = detector->createCollisionGroup(
      capsuleFrame1.get(), capsuleFrame2.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  EXPECT_EQ(capsuleFrame1.get(), contact.getShapeFrame1());
  EXPECT_EQ(capsuleFrame2.get(), contact.getShapeFrame2());
  EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesCylinderSphere)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto cylinderFrame
      = makeFrame(std::make_shared<dynamics::CylinderShape>(0.5, 2.0));
  auto sphereFrame = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));

  auto group
      = detector->createCollisionGroup(cylinderFrame.get(), sphereFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  EXPECT_EQ(cylinderFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(sphereFrame.get(), contact.getShapeFrame2());
  EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesCylinderBox)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto cylinderFrame
      = makeFrame(std::make_shared<dynamics::CylinderShape>(0.5, 2.0));
  auto boxFrame = makeFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)),
      Eigen::Vector3d(0.75, 0.0, 0.0));

  auto group
      = detector->createCollisionGroup(cylinderFrame.get(), boxFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  EXPECT_EQ(cylinderFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(boxFrame.get(), contact.getShapeFrame2());
  EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesCylinderPlane)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto cylinderFrame = makeFrame(
      std::make_shared<dynamics::CylinderShape>(0.5, 2.0),
      Eigen::Vector3d(0.0, 0.0, 0.75));
  auto planeFrame = makeFrame(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  auto group
      = detector->createCollisionGroup(cylinderFrame.get(), planeFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  EXPECT_EQ(cylinderFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(planeFrame.get(), contact.getShapeFrame2());
  EXPECT_TRUE(contact.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesSphereMesh)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto meshFrame = makeFrame(std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), makePlaneTriMesh()));
  auto sphereFrame = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.25, -0.25, 0.25));

  auto group
      = detector->createCollisionGroup(meshFrame.get(), sphereFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  const auto& contact = result.getContact(0);
  EXPECT_EQ(meshFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(sphereFrame.get(), contact.getShapeFrame2());
  EXPECT_LT(contact.normal.z(), -0.99);
  EXPECT_GE(contact.triID1, 0);
  EXPECT_EQ(-1, contact.triID2);

  auto reverseGroup
      = detector->createCollisionGroup(sphereFrame.get(), meshFrame.get());
  collision::CollisionResult reverseResult;
  EXPECT_TRUE(reverseGroup->collide(
      collision::CollisionOption(true, 10u), &reverseResult));
  ASSERT_GT(reverseResult.getNumContacts(), 0u);

  const auto& reverseContact = reverseResult.getContact(0);
  EXPECT_EQ(sphereFrame.get(), reverseContact.getShapeFrame1());
  EXPECT_EQ(meshFrame.get(), reverseContact.getShapeFrame2());
  EXPECT_GT(reverseContact.normal.z(), 0.99);
  EXPECT_EQ(-1, reverseContact.triID1);
  EXPECT_GE(reverseContact.triID2, 0);
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesSphereConvexMesh)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto convexFrame = makeFrame(std::make_shared<dynamics::ConvexMeshShape>(
      makeCubeVertices(), makeCubeTriangles()));
  auto sphereFrame = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.75),
      Eigen::Vector3d(1.0, 0.0, 0.0));

  auto group
      = detector->createCollisionGroup(convexFrame.get(), sphereFrame.get());
  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  EXPECT_EQ(convexFrame.get(), contact.getShapeFrame1());
  EXPECT_EQ(sphereFrame.get(), contact.getShapeFrame2());
}

//==============================================================================
TEST(NativeCollisionDetector, CollidesAcrossGroups)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  auto group1 = detector->createCollisionGroup(frame1.get());
  auto group2 = detector->createCollisionGroup(frame2.get());

  collision::CollisionResult result;
  EXPECT_TRUE(group1->collide(
      group2.get(), collision::CollisionOption(true, 10u), &result));
  ASSERT_EQ(1u, result.getNumContacts());
  EXPECT_EQ(frame1.get(), result.getContact(0).getShapeFrame1());
  EXPECT_EQ(frame2.get(), result.getContact(0).getShapeFrame2());
}

//==============================================================================
TEST(NativeCollisionDetector, SkipsUnsupportedShapes)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto sphereFrame = makeFrame(std::make_shared<dynamics::SphereShape>(1.0));
  auto unsupportedFrame
      = makeFrame(std::make_shared<dynamics::MultiSphereConvexHullShape>(
          std::vector<dynamics::MultiSphereConvexHullShape::Sphere>{
              {0.25, Eigen::Vector3d::Zero()},
              {0.25, Eigen::Vector3d(0.5, 0.0, 0.0)}}));
  auto group = detector->createCollisionGroup(
      sphereFrame.get(), unsupportedFrame.get());

  collision::CollisionResult result;
  EXPECT_FALSE(group->collide(collision::CollisionOption(true, 10u), &result));
  EXPECT_EQ(0u, result.getNumContacts());
}

//==============================================================================
TEST(NativeCollisionDetector, ConvertsSphereAndBoxShapes)
{
  const dynamics::SphereShape sphere(0.75);
  auto nativeSphere = collision::detail::NativeShapeConversion::create(sphere);
  ASSERT_NE(nullptr, nativeSphere);
  ASSERT_EQ(native::ShapeType::Sphere, nativeSphere->getType());
  EXPECT_DOUBLE_EQ(
      0.75,
      static_cast<const native::SphereShape*>(nativeSphere.get())->getRadius());

  const dynamics::BoxShape box(Eigen::Vector3d(2.0, 4.0, 6.0));
  auto nativeBox = collision::detail::NativeShapeConversion::create(box);
  ASSERT_NE(nullptr, nativeBox);
  ASSERT_EQ(native::ShapeType::Box, nativeBox->getType());
  EXPECT_EQ(
      Eigen::Vector3d(1.0, 2.0, 3.0),
      static_cast<const native::BoxShape*>(nativeBox.get())->getHalfExtents());

  const dynamics::CapsuleShape capsule(0.25, 1.5);
  auto nativeCapsule
      = collision::detail::NativeShapeConversion::create(capsule);
  ASSERT_NE(nullptr, nativeCapsule);
  ASSERT_EQ(native::ShapeType::Capsule, nativeCapsule->getType());
  EXPECT_DOUBLE_EQ(
      0.25,
      static_cast<const native::CapsuleShape*>(nativeCapsule.get())
          ->getRadius());
  EXPECT_DOUBLE_EQ(
      1.5,
      static_cast<const native::CapsuleShape*>(nativeCapsule.get())
          ->getHeight());

  const dynamics::CylinderShape cylinder(0.4, 1.2);
  auto nativeCylinder
      = collision::detail::NativeShapeConversion::create(cylinder);
  ASSERT_NE(nullptr, nativeCylinder);
  ASSERT_EQ(native::ShapeType::Cylinder, nativeCylinder->getType());
  EXPECT_DOUBLE_EQ(
      0.4,
      static_cast<const native::CylinderShape*>(nativeCylinder.get())
          ->getRadius());
  EXPECT_DOUBLE_EQ(
      1.2,
      static_cast<const native::CylinderShape*>(nativeCylinder.get())
          ->getHeight());

  const dynamics::PlaneShape plane(Eigen::Vector3d(0.0, 0.0, 2.0), -0.75);
  auto nativePlane = collision::detail::NativeShapeConversion::create(plane);
  ASSERT_NE(nullptr, nativePlane);
  ASSERT_EQ(native::ShapeType::Plane, nativePlane->getType());
  EXPECT_TRUE(static_cast<const native::PlaneShape*>(nativePlane.get())
                  ->getNormal()
                  .isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_DOUBLE_EQ(
      -0.75,
      static_cast<const native::PlaneShape*>(nativePlane.get())->getOffset());

  const dynamics::ConvexMeshShape convexMesh(
      makeCubeVertices(), makeCubeTriangles());
  auto nativeConvexMesh
      = collision::detail::NativeShapeConversion::create(convexMesh);
  ASSERT_NE(nullptr, nativeConvexMesh);
  ASSERT_EQ(native::ShapeType::Convex, nativeConvexMesh->getType());
  EXPECT_EQ(
      8u,
      static_cast<const native::ConvexShape*>(nativeConvexMesh.get())
          ->getVertices()
          .size());

  const dynamics::MeshShape meshShape(
      Eigen::Vector3d(2.0, 3.0, 4.0), makePlaneTriMesh());
  auto nativeMesh = collision::detail::NativeShapeConversion::create(meshShape);
  ASSERT_NE(nullptr, nativeMesh);
  ASSERT_EQ(native::ShapeType::Mesh, nativeMesh->getType());
  const auto* mesh = static_cast<const native::MeshShape*>(nativeMesh.get());
  ASSERT_EQ(4u, mesh->getVertices().size());
  ASSERT_EQ(2u, mesh->getTriangles().size());
  EXPECT_EQ(Eigen::Vector3d(-2.0, -3.0, 0.0), mesh->getVertices()[0]);

  const dynamics::PyramidShape pyramid(1.0, 2.0, 3.0);
  auto nativePyramid
      = collision::detail::NativeShapeConversion::create(pyramid);
  ASSERT_NE(nullptr, nativePyramid);
  ASSERT_EQ(native::ShapeType::Convex, nativePyramid->getType());
  EXPECT_EQ(
      5u,
      static_cast<const native::ConvexShape*>(nativePyramid.get())
          ->getVertices()
          .size());
}

//==============================================================================
TEST(NativeCollisionDetector, LeavesUnsupportedShapesNull)
{
  auto emptyMesh = std::make_shared<dynamics::ConvexMeshShape::TriMeshType>();
  const dynamics::ConvexMeshShape emptyConvexMesh(emptyMesh);
  EXPECT_EQ(
      nullptr,
      collision::detail::NativeShapeConversion::create(emptyConvexMesh));

  auto emptyTriMesh = std::make_shared<math::TriMesh<double>>();
  const dynamics::MeshShape emptyMeshShape(
      Eigen::Vector3d::Ones(), emptyTriMesh);
  EXPECT_EQ(
      nullptr,
      collision::detail::NativeShapeConversion::create(emptyMeshShape));

  const dynamics::ConeShape cone(1.0, 2.0);
  EXPECT_EQ(nullptr, collision::detail::NativeShapeConversion::create(cone));

  const dynamics::MultiSphereConvexHullShape multiSphere(
      {{0.25, Eigen::Vector3d::Zero()},
       {0.25, Eigen::Vector3d(0.5, 0.0, 0.0)}});
  EXPECT_EQ(
      nullptr, collision::detail::NativeShapeConversion::create(multiSphere));
}

//==============================================================================
TEST(NativeCollisionDetector, ClaimedObjectTracksNativeAabb)
{
  ExposedNativeCollisionDetector detector;
  auto frame = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(1.0, 2.0, 3.0));

  auto object = detector.claimCollisionObject(frame.get());
  auto* nativeObject
      = dynamic_cast<ExposedNativeCollisionObject*>(object.get());
  ASSERT_NE(nullptr, nativeObject);
  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  EXPECT_EQ(
      native::ShapeType::Sphere, nativeObject->getNativeShape()->getType());
  EXPECT_EQ(
      Eigen::Vector3d(1.0, 2.0, 3.0),
      nativeObject->getNativeTransform().translation());
  EXPECT_EQ(Eigen::Vector3d(0.5, 1.5, 2.5), nativeObject->getNativeAabb().min);
  EXPECT_EQ(Eigen::Vector3d(1.5, 2.5, 3.5), nativeObject->getNativeAabb().max);
}

//==============================================================================
TEST(NativeCollisionDetector, ShapeIdentitySwapRebuildsNativeShape)
{
  ExposedNativeCollisionDetector detector;
  auto frame = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));

  auto object = detector.claimCollisionObject(frame.get());
  auto* nativeObject
      = dynamic_cast<ExposedNativeCollisionObject*>(object.get());
  ASSERT_NE(nullptr, nativeObject);
  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  EXPECT_EQ(
      native::ShapeType::Sphere, nativeObject->getNativeShape()->getType());

  frame->setShape(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(2.0, 4.0, 6.0)));
  nativeObject->updateEngineData();

  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  ASSERT_EQ(native::ShapeType::Box, nativeObject->getNativeShape()->getType());
  EXPECT_EQ(
      Eigen::Vector3d(-1.0, -2.0, -3.0), nativeObject->getNativeAabb().min);
  EXPECT_EQ(Eigen::Vector3d(1.0, 2.0, 3.0), nativeObject->getNativeAabb().max);
}

//==============================================================================
TEST(NativeCollisionDetector, SameTypeShapeSwapRebuildsNativeShape)
{
  ExposedNativeCollisionDetector detector;
  auto frame = makeFrame(std::make_shared<dynamics::SphereShape>(0.5));

  auto object = detector.claimCollisionObject(frame.get());
  auto* nativeObject
      = dynamic_cast<ExposedNativeCollisionObject*>(object.get());
  ASSERT_NE(nullptr, nativeObject);
  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  ASSERT_EQ(
      native::ShapeType::Sphere, nativeObject->getNativeShape()->getType());
  EXPECT_EQ(
      Eigen::Vector3d(-0.5, -0.5, -0.5), nativeObject->getNativeAabb().min);
  EXPECT_EQ(Eigen::Vector3d(0.5, 0.5, 0.5), nativeObject->getNativeAabb().max);

  frame->setShape(std::make_shared<dynamics::SphereShape>(1.25));
  nativeObject->updateEngineData();

  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  ASSERT_EQ(
      native::ShapeType::Sphere, nativeObject->getNativeShape()->getType());
  EXPECT_EQ(
      Eigen::Vector3d(-1.25, -1.25, -1.25), nativeObject->getNativeAabb().min);
  EXPECT_EQ(
      Eigen::Vector3d(1.25, 1.25, 1.25), nativeObject->getNativeAabb().max);
}
