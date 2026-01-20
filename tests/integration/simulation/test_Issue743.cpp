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

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/collision_filter.hpp>
#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

struct SelfCollisionChain
{
  SkeletonPtr skeleton;
  BodyNode* rootBody{nullptr};
  BodyNode* tipBody{nullptr};
};

SelfCollisionChain createSelfCollisionChain()
{
  SelfCollisionChain out;
  out.skeleton = Skeleton::create("issue743_self_collision_chain");

  auto [rootJoint, rootBody]
      = out.skeleton->createJointAndBodyNodePair<FreeJoint>();
  (void)rootJoint;

  auto [midJoint, midBody]
      = rootBody->createChildJointAndBodyNodePair<WeldJoint>();
  (void)midJoint;

  auto [tipJoint, tipBody]
      = midBody->createChildJointAndBodyNodePair<WeldJoint>();
  (void)tipJoint;

  out.rootBody = rootBody;
  out.tipBody = tipBody;

  // Create collision geometry only on root and tip. With adjacent body
  // checking disabled, root-vs-mid and mid-vs-tip are ignored, but the
  // root-vs-tip pair is still eligible because they are not parent-child.
  const auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Ones());

  rootBody->createShapeNodeWith<CollisionAspect>(box);

  auto* tipShapeNode = tipBody->createShapeNodeWith<CollisionAspect>(box);
  tipShapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));

  out.skeleton->enableSelfCollisionCheck();
  out.skeleton->disableAdjacentBodyCheck();

  return out;
}

} // namespace

//==============================================================================
TEST(Issue743, FilteringGrandparentSelfCollisionPairs)
{
  auto setup = createSelfCollisionChain();

  auto world = World::create();
  world->addSkeleton(setup.skeleton);

  auto* solver = world->getConstraintSolver();
  ASSERT_NE(solver, nullptr);

  CollisionResult result;
  EXPECT_TRUE(solver->getCollisionGroup()->collide(
      solver->getCollisionOption(), &result));
  EXPECT_GT(result.getNumContacts(), 0u);

  auto& option = solver->getCollisionOption();
  auto filter = std::dynamic_pointer_cast<BodyNodeCollisionFilter>(
      option.collisionFilter);
  ASSERT_NE(filter, nullptr);

  filter->addBodyNodePairToBlackList(setup.rootBody, setup.tipBody);

  CollisionResult filtered;
  EXPECT_FALSE(solver->getCollisionGroup()->collide(option, &filtered));
  EXPECT_EQ(filtered.getNumContacts(), 0u);
}

//==============================================================================
TEST(Issue743, SelfCollisionShouldNotProduceNaN)
{
  auto setup = createSelfCollisionChain();

  auto world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(setup.skeleton);

  // The collision pair is infeasible (the bodies are welded together), but the
  // simulation should remain numerically stable.
  for (auto i = 0u; i < 50u; ++i) {
    world->step();
    EXPECT_TRUE(setup.skeleton->getPositions().allFinite());
    EXPECT_TRUE(setup.skeleton->getVelocities().allFinite());
  }
}
