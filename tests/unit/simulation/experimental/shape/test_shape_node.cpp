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

TEST(ShapeNode, LinkProperties)
{
  dse::World world;
  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 2.0, 3.0));

  dse::ShapeNodeOptions options;
  options.relativeTransform = Eigen::Isometry3d::Identity();
  options.relativeTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.collidable = false;
  options.frictionCoeff = 0.7;
  options.restitutionCoeff = 0.4;

  auto node = base.createShapeNode(shape, "base_shape", options);

  EXPECT_EQ(node.getName(), "base_shape");
  EXPECT_EQ(node.getParentFrame().getEntity(), base.getEntity());
  EXPECT_EQ(node.getShape().get(), shape.get());
  EXPECT_TRUE(node.getRelativeTransform().isApprox(options.relativeTransform));
  EXPECT_FALSE(node.isCollidable());
  EXPECT_DOUBLE_EQ(node.getFrictionCoeff(), 0.7);
  EXPECT_DOUBLE_EQ(node.getRestitutionCoeff(), 0.4);

  world.enterSimulationMode();

  auto worldTf = node.getWorldTransform();
  EXPECT_TRUE(worldTf.isApprox(options.relativeTransform));

  Eigen::Isometry3d updated = Eigen::Isometry3d::Identity();
  updated.translation() = Eigen::Vector3d(-1.0, 0.5, 2.0);
  node.setRelativeTransform(updated);
  EXPECT_TRUE(node.getRelativeTransform().isApprox(updated));

  node.setCollidable(true);
  EXPECT_TRUE(node.isCollidable());
  node.setFrictionCoeff(0.2);
  EXPECT_DOUBLE_EQ(node.getFrictionCoeff(), 0.2);
  node.setRestitutionCoeff(0.1);
  EXPECT_DOUBLE_EQ(node.getRestitutionCoeff(), 0.1);
}

TEST(ShapeNode, RigidBodyWorldTransform)
{
  dse::World world;
  auto body = world.addRigidBody("ball");
  body.setPosition(Eigen::Vector3d(0.5, -1.0, 2.0));

  auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);

  dse::ShapeNodeOptions options;
  options.relativeTransform = Eigen::Isometry3d::Identity();
  options.relativeTransform.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);

  auto node = body.createShapeNode(shape, "ball_shape", options);

  world.enterSimulationMode();

  auto worldTf = node.getWorldTransform();
  Eigen::Vector3d expected
      = body.getPosition() + options.relativeTransform.translation();
  EXPECT_TRUE(worldTf.translation().isApprox(expected));
}
