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

#include "../../helpers/GTestUtils.hpp"
#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/simple_frame.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;

namespace {

std::shared_ptr<SimpleFrame> createBoxFrame(
    const std::string& name,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
  auto frame = std::make_shared<SimpleFrame>(Frame::World(), name);
  frame->setShape(std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  frame->setTranslation(translation);
  return frame;
}

class CountingFilter : public CollisionFilter
{
public:
  bool ignoresCollision(
      const CollisionObject*, const CollisionObject*) const override
  {
    ++mCallCount;
    return mReject;
  }

  mutable int mCallCount = 0;
  bool mReject = false;
};

std::unique_ptr<CollisionGroup> makeOverlappingGroup(
    std::shared_ptr<SimpleFrame>& outA, std::shared_ptr<SimpleFrame>& outB)
{
  auto detector = dart::collision::DARTCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  outA = createBoxFrame("boxA");
  outB = createBoxFrame("boxB", Eigen::Vector3d(0.25, 0, 0));

  group->addShapeFrame(outA.get());
  group->addShapeFrame(outB.get());

  return group;
}

} // namespace

//==============================================================================
TEST(CollisionGroupTests, CollidablesCanBeToggled)
{
  std::shared_ptr<SimpleFrame> frameA;
  std::shared_ptr<SimpleFrame> frameB;
  auto group = makeOverlappingGroup(frameA, frameB);

  CollisionResult result;
  CollisionOption option;

  ASSERT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());

  group->removeShapeFrame(frameA.get());
  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(0u, result.getNumContacts());

  group->addShapeFrame(frameA.get());
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(0u, result.getNumContacts());
}

//==============================================================================
TEST(CollisionGroupTests, CollisionFilterAndContactLimitAreApplied)
{
  std::shared_ptr<SimpleFrame> frameA;
  std::shared_ptr<SimpleFrame> frameB;
  auto group = makeOverlappingGroup(frameA, frameB);

  auto filter = std::make_shared<CountingFilter>();
  CollisionOption option;
  option.collisionFilter = filter;

  CollisionResult result;
  filter->mReject = true;
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(1, filter->mCallCount);
  EXPECT_EQ(0u, result.getNumContacts());

  filter->mReject = false;
  option.maxNumContacts = 1u;
  result.clear();
  filter->mCallCount = 0;

  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(1u, result.getNumContacts());
  EXPECT_EQ(1, filter->mCallCount);

  // When we raise the limit we should collect more than one contact for the
  // overlapping cubes.
  option.maxNumContacts = 8u;
  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_LT(1u, result.getNumContacts());
}
