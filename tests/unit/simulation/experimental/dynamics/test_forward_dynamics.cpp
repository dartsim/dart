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

#include <dart/simulation/experimental/comps/link.hpp>
#include <dart/simulation/experimental/dynamics/forward_dynamics.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace dse = dart::simulation::experimental;
namespace exp_dynamics = dse::dynamics;
namespace classic = dart::dynamics;

TEST(ForwardDynamics, DefaultConstruction)
{
  exp_dynamics::ForwardDynamicsSystem fds;

  auto config = fds.getConfig();
  EXPECT_NEAR(config.gravity.z(), -9.81, 1e-10);
  EXPECT_NEAR(config.timeStep, 0.001, 1e-10);
  EXPECT_TRUE(config.useImplicitDamping);
}

TEST(ForwardDynamics, SetConfig)
{
  exp_dynamics::ForwardDynamicsSystem fds;

  exp_dynamics::ForwardDynamicsConfig config;
  config.gravity = Eigen::Vector3d(0, 0, -10.0);
  config.timeStep = 0.01;
  config.useImplicitDamping = false;

  fds.setConfig(config);

  auto newConfig = fds.getConfig();
  EXPECT_NEAR(newConfig.gravity.z(), -10.0, 1e-10);
  EXPECT_NEAR(newConfig.timeStep, 0.01, 1e-10);
  EXPECT_FALSE(newConfig.useImplicitDamping);
}

TEST(ForwardDynamics, ConfigWithGravity)
{
  exp_dynamics::ForwardDynamicsConfig config;
  config.gravity = Eigen::Vector3d(0, -9.81, 0);

  exp_dynamics::ForwardDynamicsSystem fds(config);

  EXPECT_NEAR(fds.getConfig().gravity.y(), -9.81, 1e-10);
  EXPECT_NEAR(fds.getConfig().gravity.z(), 0.0, 1e-10);
}

TEST(ForwardDynamics, SimplePendulumAcceleration)
{
  // Pendulum: base -> revolute joint -> link with inertia
  // ddq = (g/L) * sin(theta) for ideal point-mass pendulum
  constexpr double mass = 1.0;
  constexpr double length = 1.0;
  constexpr double gravity = 9.81;
  constexpr double theta = M_PI / 4.0;

  dse::World world;

  exp_dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = Eigen::Vector3d(0, 0, -gravity);
  exp_dynamics::ForwardDynamicsSystem fds(fdConfig);

  auto pendulum = world.addMultiBody("pendulum");
  auto base = pendulum.addLink("base");
  auto link = pendulum.addLink(
      "pendulum_link",
      {.parentLink = base,
       .jointName = "pivot",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitY()});

  Eigen::VectorXd pos(1);
  pos << theta;
  link.getParentJoint().setPosition(pos);

  Eigen::VectorXd vel(1);
  vel << 0.0;
  link.getParentJoint().setVelocity(vel);

  auto& registry = world.getRegistry();
  auto& linkComp = registry.get<dse::comps::Link>(link.getEntity());
  linkComp.mass.mass = mass;
  // I_yy = I_zz = m*L^2 for point mass at distance L along X axis
  linkComp.mass.inertia = Eigen::Matrix3d::Zero();
  linkComp.mass.inertia(1, 1) = mass * length * length;
  linkComp.mass.inertia(2, 2) = mass * length * length;

  world.enterSimulationMode();
  world.updateKinematics();

  fds.compute(world, pendulum);

  auto accel = link.getParentJoint().getAcceleration();
  ASSERT_EQ(accel.size(), 1);
  EXPECT_TRUE(std::isfinite(accel(0)));
}

TEST(ForwardDynamics, TwoLinkChain)
{
  dse::World world;

  exp_dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = Eigen::Vector3d(0, 0, -9.81);
  exp_dynamics::ForwardDynamicsSystem fds(fdConfig);

  auto robot = world.addMultiBody("two_link");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitZ()});
  auto link2 = robot.addLink(
      "link2",
      {.parentLink = link1,
       .jointName = "joint2",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitZ()});

  Eigen::VectorXd pos1(1), pos2(1);
  pos1 << M_PI / 6;
  pos2 << M_PI / 4;
  link1.getParentJoint().setPosition(pos1);
  link2.getParentJoint().setPosition(pos2);

  Eigen::VectorXd vel(1);
  vel << 1.0;
  link1.getParentJoint().setVelocity(vel);
  link2.getParentJoint().setVelocity(vel);

  world.enterSimulationMode();
  world.updateKinematics();

  fds.compute(world, robot);

  auto accel1 = link1.getParentJoint().getAcceleration();
  auto accel2 = link2.getParentJoint().getAcceleration();

  ASSERT_EQ(accel1.size(), 1);
  ASSERT_EQ(accel2.size(), 1);
  EXPECT_TRUE(std::isfinite(accel1(0)));
  EXPECT_TRUE(std::isfinite(accel2(0)));
}

TEST(ForwardDynamics, PrismaticJoint)
{
  dse::World world;

  exp_dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = Eigen::Vector3d(0, 0, -9.81);
  exp_dynamics::ForwardDynamicsSystem fds(fdConfig);

  auto robot = world.addMultiBody("slider");
  auto base = robot.addLink("base");
  auto slider = robot.addLink(
      "slider",
      {.parentLink = base,
       .jointName = "prismatic",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitZ()});

  Eigen::VectorXd pos(1);
  pos << 1.0;
  slider.getParentJoint().setPosition(pos);

  Eigen::VectorXd vel(1);
  vel << 0.0;
  slider.getParentJoint().setVelocity(vel);

  world.enterSimulationMode();
  world.updateKinematics();

  fds.compute(world, robot);

  auto accel = slider.getParentJoint().getAcceleration();
  ASSERT_EQ(accel.size(), 1);
  EXPECT_TRUE(std::isfinite(accel(0)));
}

TEST(ForwardDynamics, ComputeAllMultiBodies)
{
  dse::World world;

  exp_dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = Eigen::Vector3d(0, 0, -9.81);
  exp_dynamics::ForwardDynamicsSystem fds(fdConfig);

  auto robot1 = world.addMultiBody("robot1");
  auto base1 = robot1.addLink("base");
  auto link1
      = robot1.addLink("link1", {.parentLink = base1, .jointName = "j1"});

  auto robot2 = world.addMultiBody("robot2");
  auto base2 = robot2.addLink("base");
  auto link2
      = robot2.addLink("link2", {.parentLink = base2, .jointName = "j2"});

  world.enterSimulationMode();
  world.updateKinematics();

  fds.computeAll(world);

  auto accel1 = link1.getParentJoint().getAcceleration();
  auto accel2 = link2.getParentJoint().getAcceleration();

  EXPECT_TRUE(std::isfinite(accel1(0)));
  EXPECT_TRUE(std::isfinite(accel2(0)));
}

TEST(ForwardDynamics, ValidateAgainstClassicDART)
{
  constexpr double mass1 = 1.0;
  constexpr double mass2 = 0.5;
  constexpr double gravity = 9.81;
  constexpr double theta1 = M_PI / 6.0;
  constexpr double theta2 = M_PI / 4.0;
  constexpr double dtheta1 = 0.5;
  constexpr double dtheta2 = -0.3;

  auto classicSkel = classic::Skeleton::create("classic_pendulum");

  classic::BodyNode::Properties bodyProps1;
  bodyProps1.mName = "link1";
  bodyProps1.mInertia.setMass(mass1);
  bodyProps1.mInertia.setMoment(Eigen::Matrix3d::Identity());

  classic::RevoluteJoint::Properties jointProps1;
  jointProps1.mName = "joint1";
  jointProps1.mAxis = Eigen::Vector3d::UnitZ();

  auto pair1 = classicSkel->createJointAndBodyNodePair<classic::RevoluteJoint>(
      nullptr, jointProps1, bodyProps1);
  auto* classicJoint1 = pair1.first;
  auto* classicBody1 = pair1.second;

  classic::BodyNode::Properties bodyProps2;
  bodyProps2.mName = "link2";
  bodyProps2.mInertia.setMass(mass2);
  bodyProps2.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.5);

  classic::RevoluteJoint::Properties jointProps2;
  jointProps2.mName = "joint2";
  jointProps2.mAxis = Eigen::Vector3d::UnitZ();

  auto pair2
      = classicBody1->createChildJointAndBodyNodePair<classic::RevoluteJoint>(
          jointProps2, bodyProps2);
  auto* classicJoint2 = pair2.first;

  classicJoint1->setPosition(0, theta1);
  classicJoint2->setPosition(0, theta2);
  classicJoint1->setVelocity(0, dtheta1);
  classicJoint2->setVelocity(0, dtheta2);

  classicSkel->setGravity(Eigen::Vector3d(0, 0, -gravity));
  classicSkel->computeForwardDynamics();

  double classicAccel1 = classicJoint1->getAcceleration(0);
  double classicAccel2 = classicJoint2->getAcceleration(0);

  dse::World world;

  exp_dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = Eigen::Vector3d(0, 0, -gravity);
  exp_dynamics::ForwardDynamicsSystem fds(fdConfig);

  auto robot = world.addMultiBody("exp_pendulum");
  auto base = robot.addLink("base");
  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "joint1",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitZ()});
  auto link2 = robot.addLink(
      "link2",
      {.parentLink = link1,
       .jointName = "joint2",
       .jointType = dse::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitZ()});

  Eigen::VectorXd pos1(1), pos2(1);
  pos1 << theta1;
  pos2 << theta2;
  link1.getParentJoint().setPosition(pos1);
  link2.getParentJoint().setPosition(pos2);

  Eigen::VectorXd vel1(1), vel2(1);
  vel1 << dtheta1;
  vel2 << dtheta2;
  link1.getParentJoint().setVelocity(vel1);
  link2.getParentJoint().setVelocity(vel2);

  auto& registry = world.getRegistry();
  auto& linkComp1 = registry.get<dse::comps::Link>(link1.getEntity());
  linkComp1.mass.mass = mass1;
  linkComp1.mass.inertia = Eigen::Matrix3d::Identity();

  auto& linkComp2 = registry.get<dse::comps::Link>(link2.getEntity());
  linkComp2.mass.mass = mass2;
  linkComp2.mass.inertia = Eigen::Matrix3d::Identity() * 0.5;

  world.enterSimulationMode();
  world.updateKinematics();

  fds.compute(world, robot);

  double expAccel1 = link1.getParentJoint().getAcceleration()(0);
  double expAccel2 = link2.getParentJoint().getAcceleration()(0);

  EXPECT_TRUE(std::isfinite(classicAccel1));
  EXPECT_TRUE(std::isfinite(classicAccel2));
  EXPECT_TRUE(std::isfinite(expAccel1));
  EXPECT_TRUE(std::isfinite(expAccel2));

  constexpr double tolerance = 1e-10;
  EXPECT_NEAR(expAccel1, classicAccel1, tolerance);
  EXPECT_NEAR(expAccel2, classicAccel2, tolerance);
}

TEST(ForwardDynamics, ExternalForces)
{
  dse::World world;

  exp_dynamics::ForwardDynamicsConfig fdConfig;
  fdConfig.gravity = Eigen::Vector3d::Zero();
  exp_dynamics::ForwardDynamicsSystem fds(fdConfig);

  auto robot = world.addMultiBody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink(
      "link",
      {.parentLink = base,
       .jointName = "joint",
       .jointType = dse::comps::JointType::Prismatic,
       .axis = Eigen::Vector3d::UnitZ()});

  constexpr double mass = 2.0;
  link.setMass(mass);

  constexpr double appliedForce = 10.0;
  link.setExternalForce(Eigen::Vector3d(0, 0, appliedForce));

  world.enterSimulationMode();
  world.updateKinematics();

  fds.compute(world, robot);

  auto accel = link.getParentJoint().getAcceleration();
  ASSERT_EQ(accel.size(), 1);
  EXPECT_NEAR(accel(0), appliedForce / mass, 1e-10);

  link.clearExternalForces();
  EXPECT_TRUE(link.getExternalForce().isZero());
}
