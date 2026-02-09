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

#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/distance_filter.hpp"
#include "dart/collision/distance_option.hpp"
#include "dart/collision/distance_result.hpp"
#include "dart/collision/experimental/persistent_manifold_cache.hpp"
#include "dart/collision/experimental_backend/experimental_collision_detector.hpp"
#include "dart/collision/experimental_backend/shape_adapter.hpp"
#include "dart/collision/raycast_option.hpp"
#include "dart/collision/raycast_result.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/heightmap_shape.hpp"
#include "dart/dynamics/multi_sphere_convex_hull_shape.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/sphere_shape.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

std::shared_ptr<SimpleFrame> createSphereFrame(
    const std::string& name, double radius, const Eigen::Vector3d& translation)
{
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), name);
  frame->setShape(std::make_shared<SphereShape>(radius));
  frame->setTranslation(translation);
  return frame;
}

std::shared_ptr<SimpleFrame> createBoxFrame(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& translation)
{
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), name);
  frame->setShape(std::make_shared<BoxShape>(size));
  frame->setTranslation(translation);
  return frame;
}

class ExcludeFrameDistanceFilter : public DistanceFilter
{
public:
  explicit ExcludeFrameDistanceFilter(std::string excludedName)
    : mExcludedName(std::move(excludedName))
  {
  }

  bool needDistance(
      const CollisionObject* object1,
      const CollisionObject* object2) const override
  {
    if (!object1 || !object2) {
      return false;
    }

    const auto* frame1 = object1->getShapeFrame();
    const auto* frame2 = object2->getShapeFrame();
    if (!frame1 || !frame2) {
      return false;
    }

    return frame1->getName() != mExcludedName
           && frame2->getName() != mExcludedName;
  }

private:
  std::string mExcludedName;
};

} // namespace

//==============================================================================
TEST(ExperimentalCollisionBackend, PersistentManifoldCacheCreateAndRetrieve)
{
  experimental::PersistentManifoldCache cache;
  auto& manifold = cache.getOrCreate(11u, 29u);

  experimental::CachedContact contact;
  contact.localPointA = Eigen::Vector3d(0.1, 0.0, 0.0);
  contact.localPointB = Eigen::Vector3d(0.1, 0.0, 0.0);
  contact.penetrationDepth = 0.02;
  manifold.addOrReplace(contact);

  EXPECT_EQ(1u, cache.size());
  auto& same = cache.getOrCreate(11u, 29u);
  EXPECT_EQ(1, same.numContacts);
  EXPECT_NEAR(0.02, same.contacts[0].penetrationDepth, 1e-12);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, PersistentManifoldCachePairKeySymmetry)
{
  experimental::PersistentManifoldCache cache;

  auto& manifoldAB = cache.getOrCreate(3u, 9u);
  auto& manifoldBA = cache.getOrCreate(9u, 3u);

  EXPECT_EQ(&manifoldAB, &manifoldBA);
  EXPECT_EQ(1u, cache.size());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, PersistentManifoldCacheContactMatching)
{
  experimental::PersistentManifold manifold;

  experimental::CachedContact first;
  first.localPointA = Eigen::Vector3d(0.1, 0.2, 0.3);
  first.localPointB = Eigen::Vector3d(0.1, 0.2, 0.3);
  first.cachedNormalImpulse = 1.2;
  first.cachedFrictionImpulse1 = -0.3;
  first.cachedFrictionImpulse2 = 0.7;
  manifold.addOrReplace(first);

  experimental::CachedContact second;
  second.localPointA = Eigen::Vector3d(0.1005, 0.2, 0.3005);
  second.localPointB = Eigen::Vector3d(0.1005, 0.2, 0.3005);
  second.penetrationDepth = 0.04;
  manifold.addOrReplace(second);

  ASSERT_EQ(1, manifold.numContacts);
  EXPECT_NEAR(1.2, manifold.contacts[0].cachedNormalImpulse, 1e-12);
  EXPECT_NEAR(-0.3, manifold.contacts[0].cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.7, manifold.contacts[0].cachedFrictionImpulse2, 1e-12);
  EXPECT_EQ(0, manifold.contacts[0].lifetime);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, PersistentManifoldCacheReduction)
{
  experimental::PersistentManifold manifold;

  for (int i = 0; i < 6; ++i) {
    experimental::CachedContact contact;
    contact.localPointA = Eigen::Vector3d(0.04 * i, 0.03 * (i % 3), 0.0);
    contact.localPointB = contact.localPointA;
    contact.penetrationDepth = (i == 4) ? 0.25 : 0.01 * i;
    manifold.addOrReplace(contact);
  }

  EXPECT_EQ(4, manifold.numContacts);
  bool foundDeepest = false;
  for (int i = 0; i < manifold.numContacts; ++i) {
    if (std::abs(
            manifold.contacts[static_cast<std::size_t>(i)].penetrationDepth
            - 0.25)
        < 1e-12) {
      foundDeepest = true;
      break;
    }
  }
  EXPECT_TRUE(foundDeepest);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, PersistentManifoldCacheRefreshRemovesDrifted)
{
  experimental::PersistentManifold manifold;
  experimental::CachedContact contact;
  contact.localPointA = Eigen::Vector3d::Zero();
  contact.localPointB = Eigen::Vector3d::Zero();
  contact.normal = Eigen::Vector3d::UnitX();
  manifold.addOrReplace(contact);

  Eigen::Isometry3d tfA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();
  tfB.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);

  manifold.refresh(tfA, tfB, 0.04);
  EXPECT_EQ(0, manifold.numContacts);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, PersistentManifoldCacheWarmStartInCollide)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphereA = createSphereFrame("sphereA", 1.0, Eigen::Vector3d::Zero());
  auto sphereB
      = createSphereFrame("sphereB", 1.0, Eigen::Vector3d(1.5, 0.0, 0.0));
  group->addShapeFrame(sphereA.get());
  group->addShapeFrame(sphereB.get());

  CollisionResult first;
  CollisionOption option;
  option.enableContact = true;
  ASSERT_TRUE(group->collide(option, &first));
  ASSERT_GT(first.getNumContacts(), 0u);

  auto& firstContact = first.getContact(0);
  ASSERT_NE(nullptr, firstContact.userData);
  auto* cached
      = static_cast<experimental::CachedContact*>(firstContact.userData);
  cached->cachedNormalImpulse = 1.5;
  cached->cachedFrictionImpulse1 = -0.4;
  cached->cachedFrictionImpulse2 = 0.2;

  CollisionResult second;
  ASSERT_TRUE(group->collide(option, &second));
  ASSERT_GT(second.getNumContacts(), 0u);
  const auto& warm = second.getContact(0);
  EXPECT_NEAR(1.5, warm.cachedNormalImpulse, 1e-12);
  EXPECT_NEAR(-0.4, warm.cachedFrictionImpulse1, 1e-12);
  EXPECT_NEAR(0.2, warm.cachedFrictionImpulse2, 1e-12);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, SphereSphereCollide)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphereA = createSphereFrame("sphereA", 1.0, Eigen::Vector3d::Zero());
  auto sphereB
      = createSphereFrame("sphereB", 1.0, Eigen::Vector3d(1.5, 0.0, 0.0));

  group->addShapeFrame(sphereA.get());
  group->addShapeFrame(sphereB.get());

  CollisionResult result;
  CollisionOption option;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, SphereBoxCollideAcrossGroups)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto groupA = detector->createCollisionGroup();
  auto groupB = detector->createCollisionGroup();

  auto sphere
      = createSphereFrame("sphere", 1.0, Eigen::Vector3d(0.75, 0.0, 0.0));
  auto box = createBoxFrame(
      "box", Eigen::Vector3d::Ones() * 2.0, Eigen::Vector3d::Zero());

  groupA->addShapeFrame(sphere.get());
  groupB->addShapeFrame(box.get());

  CollisionResult result;
  CollisionOption option;
  EXPECT_TRUE(detector->collide(groupA.get(), groupB.get(), option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, DistanceSphereSphere)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphereA = createSphereFrame("sphereA", 1.0, Eigen::Vector3d::Zero());
  auto sphereB
      = createSphereFrame("sphereB", 1.0, Eigen::Vector3d(3.0, 0.0, 0.0));

  group->addShapeFrame(sphereA.get());
  group->addShapeFrame(sphereB.get());

  DistanceResult result;
  const double distance
      = group->distance(DistanceOption(true, 0.0, nullptr), &result);
  EXPECT_NEAR(1.0, distance, 1e-9);
  EXPECT_NEAR(1.0, result.unclampedMinDistance, 1e-9);
  ASSERT_TRUE(result.found());
  EXPECT_EQ(sphereA.get(), result.shapeFrame1);
  EXPECT_EQ(sphereB.get(), result.shapeFrame2);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, DistanceSphereSphereOverlapping)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphereA = createSphereFrame("sphereA", 1.0, Eigen::Vector3d::Zero());
  auto sphereB
      = createSphereFrame("sphereB", 1.0, Eigen::Vector3d(1.0, 0.0, 0.0));

  group->addShapeFrame(sphereA.get());
  group->addShapeFrame(sphereB.get());

  DistanceResult result;
  const double distance
      = group->distance(DistanceOption(true, -10.0, nullptr), &result);
  EXPECT_NEAR(-1.0, distance, 1e-9);
  EXPECT_NEAR(-1.0, result.unclampedMinDistance, 1e-9);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, DistanceCrossGroup)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto groupA = detector->createCollisionGroup();
  auto groupB = detector->createCollisionGroup();

  auto sphereA = createSphereFrame("sphereA", 1.0, Eigen::Vector3d::Zero());
  auto sphereB
      = createSphereFrame("sphereB", 1.0, Eigen::Vector3d(4.0, 0.0, 0.0));

  groupA->addShapeFrame(sphereA.get());
  groupB->addShapeFrame(sphereB.get());

  DistanceResult result;
  const double distance = detector->distance(
      groupA.get(), groupB.get(), DistanceOption(true, 0.0, nullptr), &result);

  EXPECT_NEAR(2.0, distance, 1e-9);
  ASSERT_TRUE(result.found());
  EXPECT_EQ(sphereA.get(), result.shapeFrame1);
  EXPECT_EQ(sphereB.get(), result.shapeFrame2);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, DistanceWithFilter)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphereA = createSphereFrame("sphereA", 1.0, Eigen::Vector3d::Zero());
  auto sphereB
      = createSphereFrame("sphereB", 1.0, Eigen::Vector3d(2.5, 0.0, 0.0));
  auto sphereC
      = createSphereFrame("sphereC", 1.0, Eigen::Vector3d(5.0, 0.0, 0.0));

  group->addShapeFrame(sphereA.get());
  group->addShapeFrame(sphereB.get());
  group->addShapeFrame(sphereC.get());

  auto filter = std::make_shared<ExcludeFrameDistanceFilter>("sphereB");

  DistanceResult result;
  const double distance
      = group->distance(DistanceOption(true, 0.0, filter), &result);

  EXPECT_NEAR(3.0, distance, 1e-9);
  ASSERT_TRUE(result.found());
  EXPECT_EQ(sphereA.get(), result.shapeFrame1);
  EXPECT_EQ(sphereC.get(), result.shapeFrame2);
}

//==============================================================================
TEST(ExperimentalCollisionBackend, RaycastHit)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphere = createSphereFrame("sphere", 1.0, Eigen::Vector3d::Zero());
  group->addShapeFrame(sphere.get());

  RaycastResult result;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      RaycastOption(false, false),
      &result));

  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(1u, result.mRayHits.size());
  EXPECT_NEAR(1.0 / 3.0, result.mRayHits[0].mFraction, 1e-9);
  EXPECT_EQ(sphere.get(), result.mRayHits[0].mCollisionObject->getShapeFrame());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, RaycastMiss)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto sphere = createSphereFrame("sphere", 1.0, Eigen::Vector3d::Zero());
  group->addShapeFrame(sphere.get());

  RaycastResult result;
  EXPECT_FALSE(group->raycast(
      Eigen::Vector3d(-3.0, 3.0, 0.0),
      Eigen::Vector3d(3.0, 3.0, 0.0),
      RaycastOption(false, false),
      &result));
  EXPECT_FALSE(result.hasHit());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, RaycastMultipleObjects)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto nearSphere
      = createSphereFrame("near", 1.0, Eigen::Vector3d(-1.0, 0.0, 0.0));
  auto farSphere
      = createSphereFrame("far", 1.0, Eigen::Vector3d(2.0, 0.0, 0.0));

  group->addShapeFrame(nearSphere.get());
  group->addShapeFrame(farSphere.get());

  RaycastResult result;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      RaycastOption(false, false),
      &result));

  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(1u, result.mRayHits.size());
  EXPECT_EQ(
      "near", result.mRayHits[0].mCollisionObject->getShapeFrame()->getName());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, RaycastFilter)
{
  auto detector = ExperimentalCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  auto nearSphere
      = createSphereFrame("near", 1.0, Eigen::Vector3d(-1.0, 0.0, 0.0));
  auto farSphere
      = createSphereFrame("far", 1.0, Eigen::Vector3d(2.0, 0.0, 0.0));

  group->addShapeFrame(nearSphere.get());
  group->addShapeFrame(farSphere.get());

  RaycastOption option(false, false, [](const CollisionObject* object) {
    return object && object->getShapeFrame()
           && object->getShapeFrame()->getName() != "near";
  });

  RaycastResult result;
  EXPECT_TRUE(group->raycast(
      Eigen::Vector3d(-5.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
      option,
      &result));

  ASSERT_TRUE(result.hasHit());
  ASSERT_EQ(1u, result.mRayHits.size());
  EXPECT_EQ(
      "far", result.mRayHits[0].mCollisionObject->getShapeFrame()->getName());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, ConeShapeAdapter)
{
  dynamics::ShapePtr cone = std::make_shared<ConeShape>(0.5, 1.2);
  auto adapted = adaptShape(cone);
  ASSERT_NE(nullptr, adapted);
  EXPECT_EQ(experimental::ShapeType::Convex, adapted->getType());

  const auto* convex
      = static_cast<const experimental::ConvexShape*>(adapted.get());
  EXPECT_EQ(33u, convex->getVertices().size());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, EllipsoidShapeAdapter)
{
  auto ellipsoid
      = std::make_shared<EllipsoidShape>(Eigen::Vector3d(2.0, 3.0, 4.0));
  ASSERT_FALSE(ellipsoid->isSphere());

  dynamics::ShapePtr ellipsoidShape = ellipsoid;
  auto adapted = adaptShape(ellipsoidShape);
  ASSERT_NE(nullptr, adapted);
  EXPECT_EQ(experimental::ShapeType::Convex, adapted->getType());

  const auto* convex
      = static_cast<const experimental::ConvexShape*>(adapted.get());
  EXPECT_LT(0u, convex->getVertices().size());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, HeightmapShapeAdapter)
{
  auto heightmap = std::make_shared<HeightmapShaped>();
  heightmap->setScale(Eigen::Vector3d(0.5, 0.5, 2.0));
  const std::vector<double> heights = {0.0, 1.0, 2.0, 3.0};
  heightmap->setHeightField(2, 2, heights);

  dynamics::ShapePtr heightmapShape = heightmap;
  auto adapted = adaptShape(heightmapShape);
  ASSERT_NE(nullptr, adapted);
  EXPECT_EQ(experimental::ShapeType::Mesh, adapted->getType());

  const auto* mesh = static_cast<const experimental::MeshShape*>(adapted.get());
  EXPECT_EQ(4u, mesh->getVertices().size());
  EXPECT_EQ(2u, mesh->getTriangles().size());
}

//==============================================================================
TEST(ExperimentalCollisionBackend, MultiSphereConvexHullShapeAdapter)
{
  MultiSphereConvexHullShape::Spheres spheres;
  spheres.emplace_back(0.3, Eigen::Vector3d::Zero());
  spheres.emplace_back(0.2, Eigen::Vector3d(1.0, 0.0, 0.0));

  dynamics::ShapePtr multiSphere
      = std::make_shared<MultiSphereConvexHullShape>(spheres);
  auto adapted = adaptShape(multiSphere);
  ASSERT_NE(nullptr, adapted);
  EXPECT_EQ(experimental::ShapeType::Convex, adapted->getType());

  const auto* convex
      = static_cast<const experimental::ConvexShape*>(adapted.get());
  EXPECT_EQ(12u, convex->getVertices().size());
}
