/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/collision_detector.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/fwd.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/collision/raycast_option.hpp>
#include <dart/collision/raycast_result.hpp>

#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <type_traits>

namespace {

static_assert(std::is_same_v<
              dart::collision::DartCollisionDetectorPtr,
              std::shared_ptr<dart::collision::DartCollisionDetector>>);
static_assert(std::is_same_v<
              dart::collision::DARTCollisionDetectorPtr,
              std::shared_ptr<dart::collision::DARTCollisionDetector>>);
DART_SUPPRESS_DEPRECATED_BEGIN
static_assert(std::is_same_v<
              dart::collision::FCLCollisionDetectorPtr,
              std::shared_ptr<dart::collision::FCLCollisionDetector>>);
DART_SUPPRESS_DEPRECATED_END

template <typename Detector>
void expectNativeBackedFacade(
    const std::shared_ptr<Detector>& detector,
    const std::string& compatibilityType)
{
  ASSERT_NE(detector, nullptr);
  EXPECT_NE(
      dynamic_cast<dart::collision::DartCollisionDetector*>(detector.get()),
      nullptr);
  EXPECT_EQ(std::string(detector->getTypeView()), compatibilityType);
}

template <typename Detector>
void expectRaycastHit(const std::shared_ptr<Detector>& detector)
{
  ASSERT_NE(detector, nullptr);

  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(1.0));

  auto group = detector->createCollisionGroup(frame.get());
  dart::collision::RaycastResult result;

  EXPECT_TRUE(detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      dart::collision::RaycastOption(),
      &result));
  EXPECT_TRUE(result.hasHit());
}

template <typename Detector>
void expectUnsupportedRaycastClearsResult(
    const std::shared_ptr<Detector>& detector)
{
  ASSERT_NE(detector, nullptr);

  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(1.0));

  auto group = detector->createCollisionGroup(frame.get());
  dart::collision::RaycastResult result;
  result.mRayHits.emplace_back();

  EXPECT_FALSE(detector->raycast(
      group.get(),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      dart::collision::RaycastOption(),
      &result));
  EXPECT_FALSE(result.hasHit());
  EXPECT_TRUE(result.mRayHits.empty());
}

} // namespace

TEST(LegacyCompatFacades, DirectClassesKeepDisplayNamesOnly)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  expectNativeBackedFacade(
      dart::collision::FCLCollisionDetector::create(), "fcl");
  expectNativeBackedFacade(
      dart::collision::BulletCollisionDetector::create(), "bullet");
  expectNativeBackedFacade(
      dart::collision::OdeCollisionDetector::create(), "ode");
  DART_SUPPRESS_DEPRECATED_END
}

TEST(LegacyCompatFacades, FactoryAliasesStillCanonicalizeToDart)
{
  auto factory = dart::collision::CollisionDetector::getFactory();
  ASSERT_NE(factory, nullptr);

  for (const std::string key : {"fcl", "fcl_mesh", "bullet", "ode"}) {
    ASSERT_TRUE(factory->canCreate(key)) << key;
    const auto detector = factory->create(key);
    ASSERT_NE(detector, nullptr) << key;
    EXPECT_EQ(std::string(detector->getTypeView()), "dart") << key;
  }
}

TEST(LegacyCompatFacades, CanonicalAndBulletDetectorsSupportRaycasts)
{
  expectRaycastHit(dart::collision::DartCollisionDetector::create());

  DART_SUPPRESS_DEPRECATED_BEGIN
  expectRaycastHit(dart::collision::BulletCollisionDetector::create());
  DART_SUPPRESS_DEPRECATED_END
}

TEST(LegacyCompatFacades, GzUnsupportedFacadesKeepRaycastsUnsupported)
{
  expectUnsupportedRaycastClearsResult(
      dart::collision::DARTCollisionDetector::create());

  DART_SUPPRESS_DEPRECATED_BEGIN
  expectUnsupportedRaycastClearsResult(
      dart::collision::FCLCollisionDetector::create());
  expectUnsupportedRaycastClearsResult(
      dart::collision::OdeCollisionDetector::create());
  DART_SUPPRESS_DEPRECATED_END
}
