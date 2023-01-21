/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/test/dynamics/TestHelpers.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CollisionResult.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

//==============================================================================
TEST(Issue1231, NoContacts)
{
  dart::simulation::WorldPtr world = dart::simulation::World::create();

  double x = -0.25;
  for (const auto& name : {"1", "2"}) {
    const auto skeleton = dart::dynamics::Skeleton::create(name);
    skeleton->setMobile(false);
    auto pair
        = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto joint = pair.first;
    auto bn = pair.second;

    math::Isometry3d tf = math::Isometry3d::Identity();
    tf.translation() = math::Vector3d(x, x, x);
    joint->setTransform(tf);
    x += 0.5;

    bn->createShapeNodeWith<dart::dynamics::CollisionAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            math::Vector3d::Constant(1.0)));

    world->addSkeleton(skeleton);
  }

  world->step();
  const std::size_t num = world->getLastCollisionResult().getNumContacts();
  EXPECT_EQ(0u, num);
}
