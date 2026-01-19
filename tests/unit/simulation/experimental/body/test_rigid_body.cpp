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
#include <dart/simulation/experimental/world.hpp>

#include <gtest/gtest.h>

namespace dse = dart::simulation::experimental;

TEST(RigidBody, CreateWithDefaultOptions)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  EXPECT_EQ(body.getName(), "box");
  EXPECT_TRUE(body.isValid());
}

TEST(RigidBody, AutoGenerateName)
{
  dse::World world;
  auto body1 = world.addRigidBody("");
  auto body2 = world.addRigidBody("");

  EXPECT_EQ(body1.getName(), "rigid_body_001");
  EXPECT_EQ(body2.getName(), "rigid_body_002");
}

TEST(RigidBody, GetSetMass)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  EXPECT_DOUBLE_EQ(body.getMass(), 1.0);

  body.setMass(5.0);
  EXPECT_DOUBLE_EQ(body.getMass(), 5.0);
}

TEST(RigidBody, GetSetInertia)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Matrix3d defaultInertia = body.getInertia();
  EXPECT_TRUE(defaultInertia.isIdentity());

  Eigen::Matrix3d newInertia;
  newInertia << 2, 0, 0, 0, 3, 0, 0, 0, 4;
  body.setInertia(newInertia);

  Eigen::Matrix3d retrieved = body.getInertia();
  EXPECT_TRUE(retrieved.isApprox(newInertia));
}

TEST(RigidBody, GetSetPosition)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Vector3d defaultPos = body.getPosition();
  EXPECT_TRUE(defaultPos.isZero());

  Eigen::Vector3d newPos(1.0, 2.0, 3.0);
  body.setPosition(newPos);

  Eigen::Vector3d retrieved = body.getPosition();
  EXPECT_TRUE(retrieved.isApprox(newPos));
}

TEST(RigidBody, GetSetOrientation)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Quaterniond defaultOri = body.getOrientation();
  EXPECT_TRUE(defaultOri.isApprox(Eigen::Quaterniond::Identity()));

  Eigen::Quaterniond newOri(
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  body.setOrientation(newOri);

  Eigen::Quaterniond retrieved = body.getOrientation();
  EXPECT_TRUE(retrieved.isApprox(newOri));
}

TEST(RigidBody, GetSetLinearVelocity)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Vector3d defaultVel = body.getLinearVelocity();
  EXPECT_TRUE(defaultVel.isZero());

  Eigen::Vector3d newVel(1.0, 2.0, 3.0);
  body.setLinearVelocity(newVel);

  Eigen::Vector3d retrieved = body.getLinearVelocity();
  EXPECT_TRUE(retrieved.isApprox(newVel));
}

TEST(RigidBody, GetSetAngularVelocity)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Vector3d defaultVel = body.getAngularVelocity();
  EXPECT_TRUE(defaultVel.isZero());

  Eigen::Vector3d newVel(0.1, 0.2, 0.3);
  body.setAngularVelocity(newVel);

  Eigen::Vector3d retrieved = body.getAngularVelocity();
  EXPECT_TRUE(retrieved.isApprox(newVel));
}

TEST(RigidBody, ForceAccumulation)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Vector3d initialForce = body.getForce();
  EXPECT_TRUE(initialForce.isZero());

  body.addForce(Eigen::Vector3d(1.0, 0.0, 0.0));
  body.addForce(Eigen::Vector3d(0.0, 2.0, 0.0));

  Eigen::Vector3d totalForce = body.getForce();
  EXPECT_TRUE(totalForce.isApprox(Eigen::Vector3d(1.0, 2.0, 0.0)));
}

TEST(RigidBody, TorqueAccumulation)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  Eigen::Vector3d initialTorque = body.getTorque();
  EXPECT_TRUE(initialTorque.isZero());

  body.addTorque(Eigen::Vector3d(0.0, 0.0, 1.0));
  body.addTorque(Eigen::Vector3d(0.0, 0.0, 2.0));

  Eigen::Vector3d totalTorque = body.getTorque();
  EXPECT_TRUE(totalTorque.isApprox(Eigen::Vector3d(0.0, 0.0, 3.0)));
}

TEST(RigidBody, ClearForces)
{
  dse::World world;
  auto body = world.addRigidBody("box");

  body.addForce(Eigen::Vector3d(1.0, 2.0, 3.0));
  body.addTorque(Eigen::Vector3d(4.0, 5.0, 6.0));

  body.clearForces();

  EXPECT_TRUE(body.getForce().isZero());
  EXPECT_TRUE(body.getTorque().isZero());
}

TEST(RigidBody, CreateWithOptions)
{
  dse::World world;

  dse::RigidBodyOptions options;
  options.mass = 10.0;
  options.inertia = Eigen::Matrix3d::Identity() * 2.0;
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  options.linearVelocity = Eigen::Vector3d(0.1, 0.2, 0.3);
  options.angularVelocity = Eigen::Vector3d(0.4, 0.5, 0.6);

  auto body = world.addRigidBody("configured_box", options);

  EXPECT_DOUBLE_EQ(body.getMass(), 10.0);
  EXPECT_TRUE(body.getInertia().isApprox(Eigen::Matrix3d::Identity() * 2.0));
  EXPECT_TRUE(body.getPosition().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(body.getOrientation().isApprox(options.orientation));
  EXPECT_TRUE(
      body.getLinearVelocity().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(
      body.getAngularVelocity().isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
}

TEST(RigidBody, MultipleRigidBodies)
{
  dse::World world;

  auto box = world.addRigidBody("box");
  auto sphere = world.addRigidBody("sphere");
  auto cylinder = world.addRigidBody("cylinder");

  EXPECT_EQ(world.getRigidBodyCount(), 3u);

  box.setMass(1.0);
  sphere.setMass(2.0);
  cylinder.setMass(3.0);

  EXPECT_DOUBLE_EQ(box.getMass(), 1.0);
  EXPECT_DOUBLE_EQ(sphere.getMass(), 2.0);
  EXPECT_DOUBLE_EQ(cylinder.getMass(), 3.0);
}
