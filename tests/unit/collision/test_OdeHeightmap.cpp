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
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#if DART_HAVE_ODE

  #include <dart/collision/CollisionGroup.hpp>
  #include <dart/collision/CollisionOption.hpp>
  #include <dart/collision/CollisionResult.hpp>
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
  #include <dart/collision/ode/OdeCollisionObject.hpp>

  #include <dart/dynamics/HeightmapShape.hpp>
  #include <dart/dynamics/SimpleFrame.hpp>
  #include <dart/dynamics/SphereShape.hpp>

  #include <gtest/gtest.h>

  #include <memory>

using dart::collision::CollisionOption;
using dart::collision::CollisionResult;
using dart::collision::OdeCollisionDetector;
using dart::dynamics::Frame;
using dart::dynamics::HeightmapShapef;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SphereShape;

namespace {

class TestOdeCollisionObject : public dart::collision::OdeCollisionObject
{
public:
  TestOdeCollisionObject(
      dart::collision::OdeCollisionDetector* detector,
      const dart::dynamics::ShapeFrame* shapeFrame)
    : dart::collision::OdeCollisionObject(detector, shapeFrame)
  {
  }

  using dart::collision::OdeCollisionObject::getOdeGeomId;
};

class TestOdeDetector : public dart::collision::OdeCollisionDetector
{
public:
  using dart::collision::OdeCollisionDetector::claimCollisionObject;

protected:
  std::unique_ptr<dart::collision::CollisionObject> createCollisionObject(
      const dart::dynamics::ShapeFrame* shapeFrame) override
  {
    return std::make_unique<TestOdeCollisionObject>(this, shapeFrame);
  }
};

} // namespace

//==============================================================================
TEST(OdeHeightmap, CollisionOriginMatchesVisualOrigin)
{
  using S = float;
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  auto ode = std::make_shared<TestOdeDetector>();
  ASSERT_TRUE(ode);

  auto terrainFrame = SimpleFrame::createShared(Frame::World());

  auto terrainShape = std::make_shared<HeightmapShapef>();
  constexpr std::size_t width = 3u;
  constexpr std::size_t depth = 4u;
  const std::vector<S> heights(width * depth, S(0.0));
  terrainShape->setHeightField(width, depth, heights);
  terrainShape->setScale(Vector3(1.5f, 2.5f, 1.0f));
  terrainFrame->setShape(terrainShape);

  auto collObj = ode->claimCollisionObject(terrainFrame.get());
  auto odeObj = static_cast<TestOdeCollisionObject*>(collObj.get());
  ASSERT_NE(odeObj, nullptr);

  const auto geomId = odeObj->getOdeGeomId();
  ASSERT_NE(geomId, nullptr);

  const dReal* offset = dGeomGetOffsetPosition(geomId);
  ASSERT_NE(offset, nullptr);

  const auto& scale = terrainShape->getScale();
  const double spanX = static_cast<double>(width - 1) * scale.x();
  const double spanY = static_cast<double>(depth - 1) * scale.y();

  EXPECT_NEAR(offset[0], -0.5 * spanX, 1e-6);
  EXPECT_NEAR(offset[1], 0.5 * spanY, 1e-6);
  EXPECT_NEAR(offset[2], 0.0, 1e-6);
}

//==============================================================================
TEST(OdeHeightmap, CollisionCenteredInXY)
{
  using S = float;
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  auto ode = OdeCollisionDetector::create();
  ASSERT_TRUE(ode);

  auto terrainFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame = SimpleFrame::createShared(Frame::World());

  auto terrainShape = std::make_shared<HeightmapShapef>();
  const std::vector<S> heights = {S(0.0), S(0.0), S(0.0), S(0.0)};
  terrainShape->setHeightField(2u, 2u, heights);
  terrainShape->setScale(Vector3(2.0f, 2.0f, 1.0f));
  terrainFrame->setShape(terrainShape);

  constexpr double radius = 0.1;
  sphereFrame->setShape(std::make_shared<SphereShape>(radius));

  auto group = ode->createCollisionGroup(terrainFrame.get(), sphereFrame.get());
  ASSERT_EQ(group->getNumShapeFrames(), 2u);

  CollisionOption option;
  option.enableContact = true;

  CollisionResult result;

  const auto& scale = terrainShape->getScale();
  const double spanX
      = static_cast<double>(terrainShape->getWidth() - 1) * scale.x();
  const double spanY
      = static_cast<double>(terrainShape->getDepth() - 1) * scale.y();
  const double testX = 0.25 * spanX;
  const double testY = 0.25 * spanY;

  for (const double signX : {1.0, -1.0}) {
    for (const double signY : {1.0, -1.0}) {
      SCOPED_TRACE(
          ::testing::Message() << "signX=" << signX << " signY=" << signY);

      result.clear();
      sphereFrame->setTranslation(
          Eigen::Vector3d(signX * testX, signY * testY, radius - 1e-2));
      EXPECT_TRUE(group->collide(option, &result));
      EXPECT_GT(result.getNumContacts(), 0u);

      result.clear();
      sphereFrame->setTranslation(
          Eigen::Vector3d(signX * testX, signY * testY, radius + 1e-2));
      EXPECT_FALSE(group->collide(option, &result));
      EXPECT_EQ(result.getNumContacts(), 0u);
    }
  }
}

#endif // DART_HAVE_ODE
