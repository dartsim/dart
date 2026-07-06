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

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionGroup.hpp>
#include <dart/collision/native/NativeCollisionObject.hpp>
#include <dart/collision/native/detail/NativeShapeConversion.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

#include <memory>

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

} // namespace

//==============================================================================
TEST(NativeCollisionDetector, FactoryKeyRemainsUnregisteredInP3a)
{
  auto* factory = collision::CollisionDetector::getFactory();
  ASSERT_NE(nullptr, factory);
  EXPECT_FALSE(
      factory->canCreate(collision::NativeCollisionDetector::getStaticType()));
  EXPECT_EQ(
      nullptr,
      factory->create(collision::NativeCollisionDetector::getStaticType()));
}

//==============================================================================
TEST(NativeCollisionDetector, CreatesDetectorGroupAndStubCollide)
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
}

//==============================================================================
TEST(NativeCollisionDetector, LeavesUnsupportedShapesNull)
{
  const dynamics::PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  EXPECT_EQ(nullptr, collision::detail::NativeShapeConversion::create(plane));
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
