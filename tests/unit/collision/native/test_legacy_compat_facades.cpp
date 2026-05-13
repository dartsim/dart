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
#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace {

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

} // namespace

TEST(LegacyCompatFacades, DirectClassesKeepDisplayNamesOnly)
{
  expectNativeBackedFacade(
      dart::collision::FCLCollisionDetector::create(), "fcl");
  expectNativeBackedFacade(
      dart::collision::BulletCollisionDetector::create(), "bullet");
  expectNativeBackedFacade(
      dart::collision::OdeCollisionDetector::create(), "ode");
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
