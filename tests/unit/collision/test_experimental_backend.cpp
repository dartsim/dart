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
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/experimental_backend/experimental_collision_detector.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/sphere_shape.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <memory>

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

} // namespace

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
