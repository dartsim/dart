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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/shape/shape_node.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

namespace dse = dart::simulation::experimental;

TEST(WorldCollision, RigidBodyGroundContact)
{
  dse::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.001);

  auto groundMb = world.addMultiBody("ground");
  auto ground = groundMb.addLink("ground_link");

  auto groundShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10.0, 10.0, 1.0));

  dse::ShapeNodeOptions groundOptions;
  groundOptions.relativeTransform = Eigen::Isometry3d::Identity();
  groundOptions.relativeTransform.translation()
      = Eigen::Vector3d(0.0, 0.0, -0.5);
  ground.createShapeNode(groundShape, "ground_shape", groundOptions);

  auto ball = world.addRigidBody("ball");
  ball.setMass(1.0);
  ball.setInertia(Eigen::Matrix3d::Identity());
  ball.setPosition(Eigen::Vector3d(0.0, 0.0, 0.4));

  auto ballShape = std::make_shared<dart::dynamics::SphereShape>(0.5);
  ball.createShapeNode(ballShape, "ball_shape");

  world.enterSimulationMode();

  auto detector = world.getCollisionDetector();
  if (!detector) {
    GTEST_SKIP() << "Collision detector not available";
  }

  const auto& initialContacts = world.detectCollisions();
  EXPECT_GT(initialContacts.getNumContacts(), 0u);

  constexpr int steps = 1500;
  for (int i = 0; i < steps; ++i) {
    world.step();
  }

  const auto& contacts = world.getLastCollisionResult();
  EXPECT_GT(contacts.getNumContacts(), 0u);

  EXPECT_NEAR(ball.getPosition().z(), 0.5, 0.05);
}
