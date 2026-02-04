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

  #include <dart/collision/collision_group.hpp>
  #include <dart/collision/collision_option.hpp>
  #include <dart/collision/collision_result.hpp>
  #include <dart/collision/ode/ode_collision_detector.hpp>

  #include <dart/dynamics/heightmap_shape.hpp>
  #include <dart/dynamics/simple_frame.hpp>
  #include <dart/dynamics/sphere_shape.hpp>

  #include <gtest/gtest.h>

using dart::collision::CollisionOption;
using dart::collision::CollisionResult;
using dart::collision::OdeCollisionDetector;
using dart::dynamics::Frame;
using dart::dynamics::HeightmapShapef;
using dart::dynamics::SimpleFrame;
using dart::dynamics::SphereShape;

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
