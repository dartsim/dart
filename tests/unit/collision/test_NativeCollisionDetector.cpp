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

#include <dart/config.hpp>

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/DistanceFilter.hpp>
#include <dart/collision/DistanceOption.hpp>
#include <dart/collision/DistanceResult.hpp>
#include <dart/collision/RaycastOption.hpp>
#include <dart/collision/RaycastResult.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionGroup.hpp>
#include <dart/collision/native/NativeCollisionObject.hpp>
#include <dart/collision/native/detail/NativeShapeConversion.hpp>
#include <dart/collision/native/shapes/Shape.hpp>

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

#if HAVE_OCTOMAP
  #include <dart/dynamics/VoxelGridShape.hpp>
#endif

#include <dart/math/TriMesh.hpp>

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <set>
#include <string>
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
class ShapeFramePairCollisionFilter final : public collision::CollisionFilter
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
class ShapeFramePairDistanceFilter final : public collision::DistanceFilter
{
public:
  void addIgnoredPair(
      const dynamics::ShapeFrame* frame1, const dynamics::ShapeFrame* frame2)
  {
    mIgnoredPairs.emplace_back(frame1, frame2);
  }

  bool needDistance(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    const auto* frame1 = object1->getShapeFrame();
    const auto* frame2 = object2->getShapeFrame();
    for (const auto& pair : mIgnoredPairs) {
      if ((frame1 == pair.first && frame2 == pair.second)
          || (frame1 == pair.second && frame2 == pair.first)) {
        return false;
      }
    }

    return true;
  }

private:
  std::vector<
      std::pair<const dynamics::ShapeFrame*, const dynamics::ShapeFrame*>>
      mIgnoredPairs;
};

//==============================================================================
class OrderedShapeFrameDistanceFilter final : public collision::DistanceFilter
{
public:
  OrderedShapeFrameDistanceFilter(
      const dynamics::ShapeFrame* first, const dynamics::ShapeFrame* second)
    : mFirst(first), mSecond(second)
  {
  }

  bool needDistance(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    return object1->getShapeFrame() == mFirst
           && object2->getShapeFrame() == mSecond;
  }

private:
  const dynamics::ShapeFrame* mFirst;
  const dynamics::ShapeFrame* mSecond;
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

//==============================================================================
using ShapeFramePair
    = std::pair<const dynamics::ShapeFrame*, const dynamics::ShapeFrame*>;

struct ShapeFramePairLess
{
  bool operator()(const ShapeFramePair& lhs, const ShapeFramePair& rhs) const
  {
    const std::less<const dynamics::ShapeFrame*> isLess;
    if (isLess(lhs.first, rhs.first))
      return true;
    if (isLess(rhs.first, lhs.first))
      return false;
    return isLess(lhs.second, rhs.second);
  }
};

using ShapeFramePairSet = std::set<ShapeFramePair, ShapeFramePairLess>;

//==============================================================================
ShapeFramePair makeShapeFramePair(
    const dynamics::ShapeFrame* frame1, const dynamics::ShapeFrame* frame2)
{
  const std::less<const dynamics::ShapeFrame*> isLess;
  return isLess(frame1, frame2) ? ShapeFramePair{frame1, frame2}
                                : ShapeFramePair{frame2, frame1};
}

//==============================================================================
ShapeFramePairSet collectShapeFramePairs(
    const collision::CollisionResult& result)
{
  ShapeFramePairSet pairs;
  for (const auto& contact : result.getContacts()) {
    EXPECT_NE(nullptr, contact.collisionObject1);
    EXPECT_NE(nullptr, contact.collisionObject2);
    if (!contact.collisionObject1 || !contact.collisionObject2)
      continue;

    pairs.insert(makeShapeFramePair(
        contact.collisionObject1->getShapeFrame(),
        contact.collisionObject2->getShapeFrame()));
  }

  return pairs;
}

//==============================================================================
ShapeFramePairSet collideShapeFrames(
    const collision::CollisionDetectorPtr& detector,
    const std::vector<dynamics::SimpleFramePtr>& frames)
{
  auto group = detector->createCollisionGroup();
  for (const auto& frame : frames)
    group->addShapeFrame(frame.get());

  collision::CollisionOption option(true, 256u);
  option.maxNumContactsPerPair = 8u;

  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result)) << detector->getType();
  return collectShapeFramePairs(result);
}

//==============================================================================
struct DistanceResultSnapshot
{
  double distance = 0.0;
  collision::DistanceResult result;
};

//==============================================================================
DistanceResultSnapshot distanceShapeFrames(
    const collision::CollisionDetectorPtr& detector,
    const dynamics::SimpleFramePtr& frame1,
    const dynamics::SimpleFramePtr& frame2,
    const collision::DistanceOption& option)
{
  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  DistanceResultSnapshot snapshot;
  snapshot.distance = group->distance(option, &snapshot.result);
  return snapshot;
}

//==============================================================================
DistanceResultSnapshot distanceShapeFramesAcrossGroups(
    const collision::CollisionDetectorPtr& detector,
    const dynamics::SimpleFramePtr& frame1,
    const dynamics::SimpleFramePtr& frame2,
    const collision::DistanceOption& option)
{
  auto group1 = detector->createCollisionGroup(frame1.get());
  auto group2 = detector->createCollisionGroup(frame2.get());

  DistanceResultSnapshot snapshot;
  snapshot.distance = group1->distance(group2.get(), option, &snapshot.result);
  return snapshot;
}

//==============================================================================
void expectDistanceMatchesFcl(
    const std::string& name,
    const dynamics::ShapePtr& shape1,
    const Eigen::Vector3d& translation1,
    const dynamics::ShapePtr& shape2,
    const Eigen::Vector3d& translation2)
{
  const auto frame1 = makeFrame(shape1, translation1);
  const auto frame2 = makeFrame(shape2, translation2);

  collision::DistanceOption option(true, 0.0, nullptr);
  const auto fcl = distanceShapeFrames(
      collision::FCLCollisionDetector::create(), frame1, frame2, option);
  const auto native = distanceShapeFrames(
      collision::NativeCollisionDetector::create(), frame1, frame2, option);

  ASSERT_TRUE(fcl.result.found()) << name;
  ASSERT_TRUE(native.result.found()) << name;
  EXPECT_NEAR(fcl.distance, native.distance, 1e-9) << name;
  EXPECT_NEAR(
      fcl.result.unclampedMinDistance, native.result.unclampedMinDistance, 1e-9)
      << name;
  EXPECT_EQ(frame1.get(), native.result.shapeFrame1) << name;
  EXPECT_EQ(frame2.get(), native.result.shapeFrame2) << name;
}

//==============================================================================
struct MixedPrimitiveScene
{
  std::vector<dynamics::SimpleFramePtr> frames;
  ShapeFramePairSet expectedPairs;
};

//==============================================================================
MixedPrimitiveScene makeMixedPrimitiveScene()
{
  MixedPrimitiveScene scene;

  // DART 6 FCL does not support CapsuleShape, so this scene covers the
  // primitive subset shared by FCL, the legacy DART detector, and native.
  auto addFrame = [&](const dynamics::ShapePtr& shape,
                      const Eigen::Vector3d& translation) {
    auto frame = makeFrame(shape, translation);
    scene.frames.push_back(frame);
    return frame;
  };

  auto plane = addFrame(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d::Zero());

  auto planeSphere = addFrame(
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.0, 0.0, 0.18));
  auto planeBox = addFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.4)),
      Eigen::Vector3d(1.2, 0.0, 0.15));
  auto planeCylinder = addFrame(
      std::make_shared<dynamics::CylinderShape>(0.2, 0.5),
      Eigen::Vector3d(2.4, 0.0, 0.20));

  auto sphere1 = addFrame(
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.0, 2.0, 1.0));
  auto sphere2 = addFrame(
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.4, 2.0, 1.0));

  auto box1 = addFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.4)),
      Eigen::Vector3d(1.4, 2.0, 1.0));
  auto box2 = addFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.4)),
      Eigen::Vector3d(1.75, 2.0, 1.0));

  auto sphereBoxSphere = addFrame(
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d(2.8, 2.0, 1.0));
  auto sphereBoxBox = addFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.4)),
      Eigen::Vector3d(3.15, 2.0, 1.0));

  auto cylinderSphereCylinder = addFrame(
      std::make_shared<dynamics::CylinderShape>(0.25, 0.5),
      Eigen::Vector3d(4.2, 2.0, 1.0));
  auto cylinderSphereSphere = addFrame(
      std::make_shared<dynamics::SphereShape>(0.2),
      Eigen::Vector3d(4.55, 2.0, 1.0));

  auto cylinderBoxCylinder = addFrame(
      std::make_shared<dynamics::CylinderShape>(0.25, 0.5),
      Eigen::Vector3d(5.6, 2.0, 1.0));
  auto cylinderBoxBox = addFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.4)),
      Eigen::Vector3d(5.95, 2.0, 1.0));

  auto cylinder1 = addFrame(
      std::make_shared<dynamics::CylinderShape>(0.25, 0.5),
      Eigen::Vector3d(7.0, 2.0, 1.0));
  auto cylinder2 = addFrame(
      std::make_shared<dynamics::CylinderShape>(0.25, 0.5),
      Eigen::Vector3d(7.4, 2.0, 1.0));

  scene.expectedPairs
      = {makeShapeFramePair(plane.get(), planeSphere.get()),
         makeShapeFramePair(plane.get(), planeBox.get()),
         makeShapeFramePair(plane.get(), planeCylinder.get()),
         makeShapeFramePair(sphere1.get(), sphere2.get()),
         makeShapeFramePair(box1.get(), box2.get()),
         makeShapeFramePair(sphereBoxSphere.get(), sphereBoxBox.get()),
         makeShapeFramePair(
             cylinderSphereCylinder.get(), cylinderSphereSphere.get()),
         makeShapeFramePair(cylinderBoxCylinder.get(), cylinderBoxBox.get()),
         makeShapeFramePair(cylinder1.get(), cylinder2.get())};

  return scene;
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
TEST(NativeCollisionDetector, MixedPrimitiveSceneMatchesFclAndDart)
{
  const auto scene = makeMixedPrimitiveScene();

  auto fclDetector = collision::FCLCollisionDetector::create();
  fclDetector->setPrimitiveShapeType(collision::FCLCollisionDetector::MESH);
  fclDetector->setContactPointComputationMethod(
      collision::FCLCollisionDetector::DART);
  auto dartDetector = collision::DARTCollisionDetector::create();
  auto nativeDetector = collision::NativeCollisionDetector::create();

  const auto fclPairs = collideShapeFrames(fclDetector, scene.frames);
  const auto dartPairs = collideShapeFrames(dartDetector, scene.frames);
  const auto nativePairs = collideShapeFrames(nativeDetector, scene.frames);

  EXPECT_EQ(scene.expectedPairs, fclPairs);
  EXPECT_EQ(scene.expectedPairs, dartPairs);
  EXPECT_EQ(fclPairs, nativePairs);
}

//==============================================================================
TEST(NativeCollisionDetector, DistanceMatchesFclForSupportedPrimitivePairs)
{
  expectDistanceMatchesFcl(
      "sphere-sphere",
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d::Zero(),
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(2.0, 0.0, 0.0));
  expectDistanceMatchesFcl(
      "sphere-box",
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d::Zero(),
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(1.25, 0.0, 0.0));
  expectDistanceMatchesFcl(
      "box-box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d::Zero(),
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(1.25, 0.0, 0.0));
  expectDistanceMatchesFcl(
      "plane-sphere",
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d::Zero(),
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.0, 0.0, 0.75));
  expectDistanceMatchesFcl(
      "plane-box",
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d::Zero(),
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(0.0, 0.0, 0.75));
}

//==============================================================================
TEST(NativeCollisionDetector, DistanceWorksAcrossGroups)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(std::make_shared<dynamics::SphereShape>(0.25));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(2.0, 0.0, 0.0));

  collision::DistanceOption option(true, 0.0, nullptr);
  const auto sameGroup = distanceShapeFrames(detector, frame1, frame2, option);
  const auto acrossGroups
      = distanceShapeFramesAcrossGroups(detector, frame1, frame2, option);

  ASSERT_TRUE(sameGroup.result.found());
  ASSERT_TRUE(acrossGroups.result.found());
  EXPECT_NEAR(sameGroup.distance, acrossGroups.distance, 1e-12);
  EXPECT_NEAR(1.25, acrossGroups.distance, 1e-12);
  EXPECT_EQ(frame1.get(), acrossGroups.result.shapeFrame1);
  EXPECT_EQ(frame2.get(), acrossGroups.result.shapeFrame2);
}

//==============================================================================
TEST(NativeCollisionDetector, DistanceRespectsLowerBoundAndFilter)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(std::make_shared<dynamics::SphereShape>(0.25));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.5),
      Eigen::Vector3d(2.0, 0.0, 0.0));

  collision::DistanceOption lowerBoundOption(true, 2.0, nullptr);
  const auto lowerBound
      = distanceShapeFrames(detector, frame1, frame2, lowerBoundOption);
  ASSERT_TRUE(lowerBound.result.found());
  EXPECT_NEAR(2.0, lowerBound.distance, 1e-12);
  EXPECT_NEAR(2.0, lowerBound.result.minDistance, 1e-12);
  EXPECT_NEAR(1.25, lowerBound.result.unclampedMinDistance, 1e-12);

  auto filter = std::make_shared<ShapeFramePairDistanceFilter>();
  filter->addIgnoredPair(frame1.get(), frame2.get());
  collision::DistanceOption filteredOption(true, 0.0, filter);
  const auto filtered
      = distanceShapeFrames(detector, frame1, frame2, filteredOption);
  EXPECT_DOUBLE_EQ(0.0, filtered.distance);
  EXPECT_FALSE(filtered.result.found());

  collision::DistanceOption filteredLowerBoundOption(true, 2.0, filter);
  const auto filteredLowerBound
      = distanceShapeFrames(detector, frame1, frame2, filteredLowerBoundOption);
  EXPECT_NEAR(2.0, filteredLowerBound.distance, 1e-12);
  EXPECT_FALSE(filteredLowerBound.result.found());

  auto filteredGroup
      = detector->createCollisionGroup(frame1.get(), frame2.get());
  EXPECT_NEAR(
      2.0, filteredGroup->distance(filteredLowerBoundOption, nullptr), 1e-12);

  auto orderedFilter = std::make_shared<OrderedShapeFrameDistanceFilter>(
      frame1.get(), frame2.get());
  collision::DistanceOption orderedOption(true, 0.0, orderedFilter);
  const auto nativeOrdered
      = distanceShapeFrames(detector, frame1, frame2, orderedOption);
  const auto fclOrdered = distanceShapeFrames(
      collision::FCLCollisionDetector::create(), frame1, frame2, orderedOption);
  ASSERT_TRUE(fclOrdered.result.found());
  ASSERT_TRUE(nativeOrdered.result.found());
  EXPECT_NEAR(fclOrdered.distance, nativeOrdered.distance, 1e-12);
}

//==============================================================================
TEST(NativeCollisionDetector, DistanceLeavesMeshPrimitivePairsUnsupported)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto meshFrame = makeFrame(std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), makePlaneTriMesh()));
  auto sphereFrame = makeFrame(
      std::make_shared<dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.0, 0.0, 0.75));

  collision::DistanceOption option(true, 0.0, nullptr);
  const auto meshSphere
      = distanceShapeFrames(detector, meshFrame, sphereFrame, option);
  EXPECT_DOUBLE_EQ(0.0, meshSphere.distance);
  EXPECT_FALSE(meshSphere.result.found());

  const auto sphereMesh
      = distanceShapeFrames(detector, sphereFrame, meshFrame, option);
  EXPECT_DOUBLE_EQ(0.0, sphereMesh.distance);
  EXPECT_FALSE(sphereMesh.result.found());

  collision::DistanceOption lowerBoundOption(true, 2.0, nullptr);
  const auto meshSphereLowerBound
      = distanceShapeFrames(detector, meshFrame, sphereFrame, lowerBoundOption);
  EXPECT_NEAR(2.0, meshSphereLowerBound.distance, 1e-12);
  EXPECT_FALSE(meshSphereLowerBound.result.found());

  auto unsupportedGroup
      = detector->createCollisionGroup(meshFrame.get(), sphereFrame.get());
  EXPECT_NEAR(
      2.0, unsupportedGroup->distance(lowerBoundOption, nullptr), 1e-12);
}

//==============================================================================
TEST(NativeCollisionDetector, RaycastReportsClosestHit)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(
      std::make_shared<dynamics::SphereShape>(1.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(1.0),
      Eigen::Vector3d(2.0, 0.0, 0.0));
  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  collision::RaycastOption option;
  collision::RaycastResult result;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      &result));

  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(1u, result.mRayHits.size());
  const auto& hit = result.mRayHits[0];
  EXPECT_EQ(frame1.get(), hit.mCollisionObject->getShapeFrame());
  EXPECT_TRUE(hit.mPoint.isApprox(Eigen::Vector3d(-3.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(hit.mNormal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.2, hit.mFraction, 1e-12);

  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      nullptr));
}

//==============================================================================
TEST(NativeCollisionDetector, RaycastRespectsAllHitsSortingAndFilters)
{
  auto detector = collision::NativeCollisionDetector::create();
  auto frame1 = makeFrame(
      std::make_shared<dynamics::SphereShape>(1.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0));
  auto frame2 = makeFrame(
      std::make_shared<dynamics::SphereShape>(1.0),
      Eigen::Vector3d(2.0, 0.0, 0.0));
  auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

  collision::RaycastOption allHitsOption(true, true, nullptr);
  collision::RaycastResult result;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      allHitsOption,
      &result));

  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(2u, result.mRayHits.size());
  EXPECT_EQ(frame1.get(), result.mRayHits[0].mCollisionObject->getShapeFrame());
  EXPECT_NEAR(0.2, result.mRayHits[0].mFraction, 1e-12);
  EXPECT_EQ(frame2.get(), result.mRayHits[1].mCollisionObject->getShapeFrame());
  EXPECT_NEAR(0.6, result.mRayHits[1].mFraction, 1e-12);

  collision::RaycastOption filteredOption(
      false, false, [&](const collision::CollisionObject* object) {
        return object->getShapeFrame() == frame2.get();
      });
  result.clear();
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      filteredOption,
      &result));

  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(1u, result.mRayHits.size());
  EXPECT_EQ(frame2.get(), result.mRayHits[0].mCollisionObject->getShapeFrame());
  EXPECT_TRUE(result.mRayHits[0].mPoint.isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), 1e-12));
  EXPECT_NEAR(0.6, result.mRayHits[0].mFraction, 1e-12);

  collision::RaycastOption rejectingOption(
      false, false, [](const collision::CollisionObject*) { return false; });
  result.clear();
  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      rejectingOption,
      &result));
  EXPECT_FALSE(result.hasHit());
  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      rejectingOption,
      nullptr));
}

//==============================================================================
TEST(NativeCollisionDetector, RaycastCoversPrimitiveRows)
{
  auto detector = collision::NativeCollisionDetector::create();
  collision::RaycastOption option;

  auto expectHit = [&](const dynamics::ShapePtr& shape,
                       const Eigen::Vector3d& from,
                       const Eigen::Vector3d& to,
                       const Eigen::Vector3d& point,
                       const Eigen::Vector3d& normal,
                       double fraction) {
    auto frame = makeFrame(shape);
    auto group = detector->createCollisionGroup(frame.get());

    collision::RaycastResult result;
    ASSERT_TRUE(group->raycast(from, to, option, &result));
    ASSERT_TRUE(result.hasHit());
    ASSERT_EQ(1u, result.mRayHits.size());
    const auto& hit = result.mRayHits[0];
    EXPECT_EQ(frame.get(), hit.mCollisionObject->getShapeFrame());
    EXPECT_TRUE(hit.mPoint.isApprox(point, 1e-12));
    EXPECT_TRUE(hit.mNormal.isApprox(normal, 1e-12));
    EXPECT_NEAR(fraction, hit.mFraction, 1e-12);
  };

  expectHit(
      std::make_shared<dynamics::SphereShape>(1.0),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      -Eigen::Vector3d::UnitX(),
      1.0 / 3.0);
  expectHit(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(2.0, 2.0, 2.0)),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      -Eigen::Vector3d::UnitX(),
      1.0 / 3.0);
  expectHit(
      std::make_shared<dynamics::CapsuleShape>(0.5, 2.0),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      Eigen::Vector3d(-0.5, 0.0, 0.0),
      -Eigen::Vector3d::UnitX(),
      2.5 / 6.0);
  expectHit(
      std::make_shared<dynamics::CylinderShape>(0.5, 2.0),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      Eigen::Vector3d(-0.5, 0.0, 0.0),
      -Eigen::Vector3d::UnitX(),
      2.5 / 6.0);
  expectHit(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, -1.0),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(),
      0.5);
}

//==============================================================================
TEST(NativeCollisionDetector, RaycastCoversConvexRows)
{
  auto detector = collision::NativeCollisionDetector::create();
  collision::RaycastOption option;

  auto expectHit = [&](const dynamics::ShapePtr& shape,
                       const Eigen::Vector3d& from,
                       const Eigen::Vector3d& to,
                       const Eigen::Vector3d& point,
                       const Eigen::Vector3d& normal,
                       double fraction) {
    auto frame = makeFrame(shape);
    auto group = detector->createCollisionGroup(frame.get());

    collision::RaycastResult result;
    ASSERT_TRUE(group->raycast(from, to, option, &result));
    ASSERT_TRUE(result.hasHit());
    ASSERT_EQ(1u, result.mRayHits.size());
    const auto& hit = result.mRayHits[0];
    EXPECT_EQ(frame.get(), hit.mCollisionObject->getShapeFrame());
    EXPECT_TRUE(hit.mPoint.isApprox(point, 1e-12));
    EXPECT_TRUE(hit.mNormal.isApprox(normal, 1e-12));
    EXPECT_NEAR(fraction, hit.mFraction, 1e-12);
  };

  const dynamics::ConvexMeshShape::Vertices thinVertices{
      {-0.001, -0.1, -0.1},
      {0.001, -0.1, -0.1},
      {0.001, 0.1, -0.1},
      {-0.001, 0.1, -0.1},
      {-0.001, -0.1, 0.1},
      {0.001, -0.1, 0.1},
      {0.001, 0.1, 0.1},
      {-0.001, 0.1, 0.1}};

  expectHit(
      std::make_shared<dynamics::ConvexMeshShape>(
          thinVertices, makeCubeTriangles()),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(-0.001, 0.0, 0.0),
      -Eigen::Vector3d::UnitX(),
      0.4995);

  expectHit(
      std::make_shared<dynamics::PyramidShape>(1.0, 1.0, 1.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(-0.25, 0.0, 0.0),
      Eigen::Vector3d(-2.0, 0.0, 1.0).normalized(),
      0.4375);
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
  EXPECT_EQ(
      6u,
      static_cast<const native::ConvexShape*>(nativeConvexMesh.get())
          ->getFaces()
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
  EXPECT_EQ(
      5u,
      static_cast<const native::ConvexShape*>(nativePyramid.get())
          ->getFaces()
          .size());
}

//==============================================================================
#if HAVE_OCTOMAP
TEST(NativeCollisionDetector, ConvertsVoxelGridToCompoundShape)
{
  dynamics::VoxelGridShape voxelGrid(0.1);
  voxelGrid.updateOccupancy(Eigen::Vector3d::Zero(), true);

  auto nativeVoxelGrid
      = collision::detail::NativeShapeConversion::create(voxelGrid);
  ASSERT_NE(nullptr, nativeVoxelGrid);
  ASSERT_EQ(native::ShapeType::Compound, nativeVoxelGrid->getType());

  const auto* compound
      = static_cast<const native::CompoundShape*>(nativeVoxelGrid.get());
  ASSERT_EQ(1u, compound->numChildren());
  EXPECT_EQ(native::ShapeType::Box, compound->childShape(0).getType());

  const auto& box
      = static_cast<const native::BoxShape&>(compound->childShape(0));
  EXPECT_TRUE(
      box.getHalfExtents().isApprox(Eigen::Vector3d::Constant(0.05), 1e-12));
}

//==============================================================================
TEST(NativeCollisionDetector, VoxelGridOccupancyUpdatesNativeShape)
{
  auto detector = std::make_shared<ExposedNativeCollisionDetector>();
  auto voxelGrid = std::make_shared<dynamics::VoxelGridShape>(0.1);
  auto voxelFrame = makeFrame(voxelGrid);
  auto sphereFrame = makeFrame(std::make_shared<dynamics::SphereShape>(0.01));

  auto group
      = detector->createCollisionGroup(voxelFrame.get(), sphereFrame.get());
  auto raycastGroup = detector->createCollisionGroup(voxelFrame.get());

  collision::CollisionOption option(true, 10u);
  collision::CollisionResult result;

  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  auto object = detector->claimCollisionObject(voxelFrame.get());
  auto* nativeObject
      = dynamic_cast<ExposedNativeCollisionObject*>(object.get());
  ASSERT_NE(nullptr, nativeObject);
  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  ASSERT_EQ(
      native::ShapeType::Compound, nativeObject->getNativeShape()->getType());
  const auto* emptyCompound = static_cast<const native::CompoundShape*>(
      nativeObject->getNativeShape());
  ASSERT_EQ(0u, emptyCompound->numChildren());

  voxelGrid->updateOccupancy(Eigen::Vector3d::Zero(), true);
  nativeObject->updateEngineData();
  ASSERT_NE(nullptr, nativeObject->getNativeShape());
  ASSERT_EQ(
      native::ShapeType::Compound, nativeObject->getNativeShape()->getType());
  const auto* occupiedCompound = static_cast<const native::CompoundShape*>(
      nativeObject->getNativeShape());
  ASSERT_EQ(1u, occupiedCompound->numChildren());
  const Eigen::Vector3d voxelCenter
      = occupiedCompound->childTransform(0).translation();

  sphereFrame->setTranslation(voxelCenter + Eigen::Vector3d(0.2, 0.0, 0.0));
  collision::DistanceOption distanceOption(true, 0.0, nullptr);
  collision::DistanceResult distanceResult;
  const double distance = group->distance(distanceOption, &distanceResult);
  ASSERT_TRUE(distanceResult.found());
  EXPECT_NEAR(0.14, distance, 1e-12);
  EXPECT_NEAR(0.14, distanceResult.minDistance, 1e-12);
  EXPECT_EQ(voxelFrame.get(), distanceResult.shapeFrame1);
  EXPECT_EQ(sphereFrame.get(), distanceResult.shapeFrame2);

  sphereFrame->setTranslation(voxelCenter);
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GE(result.getNumContacts(), 1u);

  collision::RaycastResult raycastResult;
  EXPECT_TRUE(raycastGroup->raycast(
      Eigen::Vector3d(-0.2, voxelCenter.y(), voxelCenter.z()),
      Eigen::Vector3d(0.2, voxelCenter.y(), voxelCenter.z()),
      collision::RaycastOption(),
      &raycastResult));
  ASSERT_EQ(1u, raycastResult.mRayHits.size());
  EXPECT_EQ(
      voxelFrame.get(),
      raycastResult.mRayHits[0].mCollisionObject->getShapeFrame());
}
#endif

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
