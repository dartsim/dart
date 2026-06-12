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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace {

namespace sx = dart::simulation;

struct FreeBodyCase
{
  Eigen::Vector3d gravity;
  Eigen::Vector3d position;
  Eigen::Vector3d linearVelocity;
  double mass = 1.0;
  double timeStep = 0.001;
  std::size_t steps = 1;
};

struct FreeBodyRun
{
  Eigen::Vector3d position;
  Eigen::Vector3d linearVelocity;
  double time = 0.0;
  std::size_t frame = 0;
};

struct RevolutePendulumCase
{
  Eigen::Vector3d gravity;
  double length = 1.0;
  double mass = 1.0;
  Eigen::Vector3d moment = Eigen::Vector3d::Ones();
  double position = 0.0;
  double velocity = 0.0;
  double timeStep = 0.001;
  std::size_t steps = 1;
};

struct PrismaticFallCase
{
  Eigen::Vector3d gravity;
  double mass = 1.0;
  double position = 0.0;
  double velocity = 0.0;
  double timeStep = 0.001;
  std::size_t steps = 1;
};

struct TwoLinkRevoluteCase
{
  Eigen::Vector3d gravity;
  Eigen::Vector2d lengths = Eigen::Vector2d::Ones();
  Eigen::Vector2d masses = Eigen::Vector2d::Ones();
  Eigen::Vector3d proximalMoment = Eigen::Vector3d::Ones();
  Eigen::Vector3d distalMoment = Eigen::Vector3d::Ones();
  Eigen::Vector2d positions = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocities = Eigen::Vector2d::Zero();
  double timeStep = 0.001;
  std::size_t steps = 1;
};

struct GroundContactCase
{
  double sphereRadius = 0.5;
  Eigen::Vector3d initialPosition = Eigen::Vector3d(0.0, 0.0, 2.0);
  Eigen::Vector3d initialVelocity = Eigen::Vector3d::Zero();
  double mass = 1.0;
  double groundHalfHeight = 0.5;
  double timeStep = 0.005;
  std::size_t steps = 1000;
};

struct ControlledRevoluteCase
{
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  double length = 1.0;
  double mass = 1.0;
  Eigen::Vector3d moment = Eigen::Vector3d::Ones();
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double timeStep = 0.001;
  std::size_t steps = 1;
};

struct SingleDofRun
{
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  Eigen::Vector3d bodyPosition;
  double time = 0.0;
  std::size_t frame = 0;
};

struct TwoDofRun
{
  Eigen::Vector2d positions;
  Eigen::Vector2d velocities;
  Eigen::Vector2d accelerations;
  Eigen::Vector3d proximalPosition;
  Eigen::Vector3d distalPosition;
  double time = 0.0;
  std::size_t frame = 0;
};

struct GroundContactRun
{
  Eigen::Vector3d spherePosition;
  Eigen::Vector3d sphereLinearVelocity;
  Eigen::Vector3d groundPosition;
  double time = 0.0;
  std::size_t frame = 0;
};

FreeBodyRun runFreeBody(const FreeBodyCase& testCase)
{
  sx::World world;
  world.setGravity(testCase.gravity);
  world.setTimeStep(testCase.timeStep);

  sx::RigidBodyOptions options;
  options.mass = testCase.mass;
  options.position = testCase.position;
  options.linearVelocity = testCase.linearVelocity;

  auto body = world.addRigidBody("body", options);
  world.step(testCase.steps);

  return FreeBodyRun{
      .position = body.getTranslation(),
      .linearVelocity = body.getLinearVelocity(),
      .time = world.getTime(),
      .frame = world.getFrame(),
  };
}

SingleDofRun runRevolutePendulum(const RevolutePendulumCase& testCase)
{
  sx::World world;
  world.setGravity(testCase.gravity);
  world.setTimeStep(testCase.timeStep);

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(testCase.length, 0.0, 0.0);

  auto bob = robot.addLink(
      "bob",
      base,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  bob.setMass(testCase.mass);
  bob.setInertia(testCase.moment.asDiagonal());

  auto joint = bob.getParentJoint();
  joint.setPosition(Eigen::VectorXd::Constant(1, testCase.position));
  joint.setVelocity(Eigen::VectorXd::Constant(1, testCase.velocity));

  world.enterSimulationMode();
  world.step(testCase.steps);

  return SingleDofRun{
      .position = joint.getPosition()[0],
      .velocity = joint.getVelocity()[0],
      .acceleration = joint.getAcceleration()[0],
      .bodyPosition = bob.getWorldTransform().translation(),
      .time = world.getTime(),
      .frame = world.getFrame(),
  };
}

SingleDofRun runPrismaticFall(const PrismaticFallCase& testCase)
{
  sx::World world;
  world.setGravity(testCase.gravity);
  world.setTimeStep(testCase.timeStep);

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  auto body = robot.addLink(
      "body",
      base,
      sx::JointSpec{
          .name = "slider",
          .type = sx::JointType::Prismatic,
          .axis = Eigen::Vector3d::UnitZ(),
      });
  body.setMass(testCase.mass);

  auto joint = body.getParentJoint();
  joint.setPosition(Eigen::VectorXd::Constant(1, testCase.position));
  joint.setVelocity(Eigen::VectorXd::Constant(1, testCase.velocity));

  world.enterSimulationMode();
  world.step(testCase.steps);

  return SingleDofRun{
      .position = joint.getPosition()[0],
      .velocity = joint.getVelocity()[0],
      .acceleration = joint.getAcceleration()[0],
      .bodyPosition = body.getWorldTransform().translation(),
      .time = world.getTime(),
      .frame = world.getFrame(),
  };
}

TwoDofRun runTwoLinkRevoluteChain(const TwoLinkRevoluteCase& testCase)
{
  sx::World world;
  world.setGravity(testCase.gravity);
  world.setTimeStep(testCase.timeStep);

  auto robot = world.addMultibody("two_link_chain");
  auto base = robot.addLink("base");

  Eigen::Isometry3d proximalOffset = Eigen::Isometry3d::Identity();
  proximalOffset.translation() = Eigen::Vector3d(testCase.lengths[0], 0.0, 0.0);
  auto proximal = robot.addLink(
      "proximal",
      base,
      sx::JointSpec{
          .name = "shoulder",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = proximalOffset,
      });
  proximal.setMass(testCase.masses[0]);
  proximal.setInertia(testCase.proximalMoment.asDiagonal());

  Eigen::Isometry3d distalOffset = Eigen::Isometry3d::Identity();
  distalOffset.translation() = Eigen::Vector3d(testCase.lengths[1], 0.0, 0.0);
  auto distal = robot.addLink(
      "distal",
      proximal,
      sx::JointSpec{
          .name = "elbow",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = distalOffset,
      });
  distal.setMass(testCase.masses[1]);
  distal.setInertia(testCase.distalMoment.asDiagonal());

  auto shoulder = proximal.getParentJoint();
  auto elbow = distal.getParentJoint();
  shoulder.setPosition(Eigen::VectorXd::Constant(1, testCase.positions[0]));
  elbow.setPosition(Eigen::VectorXd::Constant(1, testCase.positions[1]));
  shoulder.setVelocity(Eigen::VectorXd::Constant(1, testCase.velocities[0]));
  elbow.setVelocity(Eigen::VectorXd::Constant(1, testCase.velocities[1]));

  world.enterSimulationMode();
  world.step(testCase.steps);

  return TwoDofRun{
      .positions
      = Eigen::Vector2d(shoulder.getPosition()[0], elbow.getPosition()[0]),
      .velocities
      = Eigen::Vector2d(shoulder.getVelocity()[0], elbow.getVelocity()[0]),
      .accelerations = Eigen::Vector2d(
          shoulder.getAcceleration()[0], elbow.getAcceleration()[0]),
      .proximalPosition = proximal.getWorldTransform().translation(),
      .distalPosition = distal.getWorldTransform().translation(),
      .time = world.getTime(),
      .frame = world.getFrame(),
  };
}

GroundContactRun runGroundContact(const GroundContactCase& testCase)
{
  sx::World world;
  world.setTimeStep(testCase.timeStep);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(5.0, 5.0, testCase.groundHalfHeight)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = testCase.mass;
  sphereOptions.position = testCase.initialPosition;
  sphereOptions.linearVelocity = testCase.initialVelocity;
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(
      sx::CollisionShape::makeSphere(testCase.sphereRadius));

  world.enterSimulationMode();
  world.step(testCase.steps);

  return GroundContactRun{
      .spherePosition = sphere.getTranslation(),
      .sphereLinearVelocity = sphere.getLinearVelocity(),
      .groundPosition = ground.getTranslation(),
      .time = world.getTime(),
      .frame = world.getFrame(),
  };
}

SingleDofRun runControlledRevolute(const ControlledRevoluteCase& testCase)
{
  sx::World world;
  world.setGravity(testCase.gravity);
  world.setTimeStep(testCase.timeStep);

  auto robot = world.addMultibody("driven_pendulum");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(testCase.length, 0.0, 0.0);

  auto bob = robot.addLink(
      "bob",
      base,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  bob.setMass(testCase.mass);
  bob.setInertia(testCase.moment.asDiagonal());

  auto joint = bob.getParentJoint();
  joint.setPosition(Eigen::VectorXd::Constant(1, testCase.position));
  joint.setVelocity(Eigen::VectorXd::Constant(1, testCase.velocity));

  world.enterSimulationMode();
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    joint.setForce(Eigen::VectorXd::Constant(1, testCase.torque));
    world.step(1);
  }

  return SingleDofRun{
      .position = joint.getPosition()[0],
      .velocity = joint.getVelocity()[0],
      .acceleration = joint.getAcceleration()[0],
      .bodyPosition = bob.getWorldTransform().translation(),
      .time = world.getTime(),
      .frame = world.getFrame(),
  };
}

void expectFinite(const Eigen::Vector3d& value)
{
  EXPECT_TRUE(value.allFinite()) << value.transpose();
}

void expectFinite(const Eigen::Vector2d& value)
{
  EXPECT_TRUE(value.allFinite()) << value.transpose();
}

void expectFreeBodyRegression(const FreeBodyCase& testCase)
{
  const FreeBodyRun run = runFreeBody(testCase);

  expectFinite(run.position);
  expectFinite(run.linearVelocity);
  EXPECT_NEAR(run.time, testCase.timeStep * testCase.steps, 1e-12);
  EXPECT_EQ(run.frame, testCase.steps);

  if (testCase.gravity.squaredNorm() > 0.0) {
    EXPECT_GT(
        (run.linearVelocity - testCase.linearVelocity).dot(testCase.gravity),
        0.0);
  }
}

void expectSingleDofRegression(
    const SingleDofRun& run, std::size_t steps, double timeStep)
{
  EXPECT_TRUE(std::isfinite(run.position));
  EXPECT_TRUE(std::isfinite(run.velocity));
  EXPECT_TRUE(std::isfinite(run.acceleration));
  expectFinite(run.bodyPosition);
  EXPECT_NEAR(run.time, timeStep * steps, 1e-12);
  EXPECT_EQ(run.frame, steps);
}

void expectTwoDofRegression(
    const TwoDofRun& run, std::size_t steps, double timeStep)
{
  expectFinite(run.positions);
  expectFinite(run.velocities);
  expectFinite(run.accelerations);
  expectFinite(run.proximalPosition);
  expectFinite(run.distalPosition);
  EXPECT_NEAR(run.time, timeStep * steps, 1e-12);
  EXPECT_EQ(run.frame, steps);
}

} // namespace

TEST(WorldDart7Regression, FreeRigidBodyAdvancesSingleStep)
{
  expectFreeBodyRegression(
      FreeBodyCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
          .position = Eigen::Vector3d(0.5, -0.25, 1.5),
          .linearVelocity = Eigen::Vector3d::Zero(),
          .mass = 2.0,
          .timeStep = 0.01,
          .steps = 1,
      });
}

TEST(WorldDart7Regression, FreeRigidBodyAdvancesRepeatedSteps)
{
  expectFreeBodyRegression(
      FreeBodyCase{
          .gravity = Eigen::Vector3d(0.25, -0.5, -3.0),
          .position = Eigen::Vector3d(-1.0, 0.5, 2.0),
          .linearVelocity = Eigen::Vector3d(0.75, -0.25, 0.5),
          .mass = 3.0,
          .timeStep = 0.0025,
          .steps = 8,
      });
}

TEST(WorldDart7Regression, RevolutePendulumAdvancesSingleStep)
{
  const RevolutePendulumCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .length = 1.5,
      .mass = 2.0,
      .moment = Eigen::Vector3d(0.1, 0.2, 0.3),
      .position = 0.15,
      .velocity = 0.0,
      .timeStep = 0.001,
      .steps = 1,
  };

  const SingleDofRun run = runRevolutePendulum(testCase);

  expectSingleDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_NE(run.acceleration, 0.0);
}

TEST(WorldDart7Regression, RevolutePendulumAdvancesRepeatedSteps)
{
  const RevolutePendulumCase testCase{
      .gravity = Eigen::Vector3d(0.25, 0.0, -4.5),
      .length = 0.75,
      .mass = 1.25,
      .moment = Eigen::Vector3d(0.2, 0.4, 0.6),
      .position = 0.15,
      .velocity = -0.35,
      .timeStep = 0.0005,
      .steps = 12,
  };

  const SingleDofRun run = runRevolutePendulum(testCase);

  expectSingleDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_NEAR(run.velocity, testCase.velocity, 1.0);
}

TEST(WorldDart7Regression, PrismaticJointFallsSingleStep)
{
  const PrismaticFallCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .mass = 3.0,
      .position = 0.75,
      .velocity = 0.2,
      .timeStep = 0.002,
      .steps = 1,
  };

  const SingleDofRun run = runPrismaticFall(testCase);

  expectSingleDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_LT(run.velocity, testCase.velocity);
}

TEST(WorldDart7Regression, PrismaticJointFallsRepeatedSteps)
{
  const PrismaticFallCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -6.0),
      .mass = 1.5,
      .position = -0.25,
      .velocity = 1.0,
      .timeStep = 0.001,
      .steps = 20,
  };

  const SingleDofRun run = runPrismaticFall(testCase);

  expectSingleDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_LT(run.velocity, testCase.velocity);
}

TEST(WorldDart7Regression, TwoLinkRevoluteChainAdvancesSingleStep)
{
  const TwoLinkRevoluteCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .lengths = Eigen::Vector2d(0.7, 0.5),
      .masses = Eigen::Vector2d(1.5, 0.75),
      .proximalMoment = Eigen::Vector3d(0.2, 0.3, 0.4),
      .distalMoment = Eigen::Vector3d(0.1, 0.2, 0.3),
      .positions = Eigen::Vector2d(0.2, -0.35),
      .velocities = Eigen::Vector2d(0.0, 0.0),
      .timeStep = 0.001,
      .steps = 1,
  };

  const TwoDofRun run = runTwoLinkRevoluteChain(testCase);

  expectTwoDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_GT(run.accelerations.norm(), 0.0);
}

TEST(WorldDart7Regression, TwoLinkRevoluteChainAdvancesRepeatedSteps)
{
  const TwoLinkRevoluteCase testCase{
      .gravity = Eigen::Vector3d(0.1, 0.0, -3.5),
      .lengths = Eigen::Vector2d(0.8, 0.45),
      .masses = Eigen::Vector2d(1.25, 0.9),
      .proximalMoment = Eigen::Vector3d(0.3, 0.45, 0.6),
      .distalMoment = Eigen::Vector3d(0.2, 0.25, 0.35),
      .positions = Eigen::Vector2d(-0.15, 0.3),
      .velocities = Eigen::Vector2d(0.25, -0.1),
      .timeStep = 0.0005,
      .steps = 16,
  };

  const TwoDofRun run = runTwoLinkRevoluteChain(testCase);

  expectTwoDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_GT((run.positions - testCase.positions).norm(), 0.0);
}

TEST(WorldDart7Regression, DynamicSphereRestsOnStaticGround)
{
  const GroundContactCase testCase{
      .sphereRadius = 0.5,
      .initialPosition = Eigen::Vector3d(0.0, 0.0, 2.0),
      .initialVelocity = Eigen::Vector3d::Zero(),
      .mass = 1.0,
      .groundHalfHeight = 0.5,
      .timeStep = 0.005,
      .steps = 1000,
  };

  const GroundContactRun run = runGroundContact(testCase);
  const double expectedRestZ
      = testCase.groundHalfHeight + testCase.sphereRadius;

  EXPECT_NEAR(run.spherePosition.z(), expectedRestZ, 2e-2);
  EXPECT_NEAR(run.sphereLinearVelocity.z(), 0.0, 0.1);
  EXPECT_TRUE(run.groundPosition.isApprox(Eigen::Vector3d::Zero()))
      << run.groundPosition.transpose();
  EXPECT_NEAR(run.time, testCase.timeStep * testCase.steps, 1e-12);
  EXPECT_EQ(run.frame, testCase.steps);
}

constexpr std::size_t kLongHorizonSteps = 10000;

TEST(WorldDart7Regression, RevolutePendulumLongHorizonStaysFinite)
{
  const RevolutePendulumCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .length = 1.0,
      .mass = 1.0,
      .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .position = 0.4,
      .velocity = 0.0,
      .timeStep = 0.001,
      .steps = kLongHorizonSteps,
  };

  expectSingleDofRegression(
      runRevolutePendulum(testCase), testCase.steps, testCase.timeStep);
}

TEST(WorldDart7Regression, DoublePendulumLongHorizonStaysFinite)
{
  const TwoLinkRevoluteCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .lengths = Eigen::Vector2d(1.0, 1.0),
      .masses = Eigen::Vector2d(1.0, 1.0),
      .proximalMoment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .distalMoment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .positions = Eigen::Vector2d(0.5, -0.3),
      .velocities = Eigen::Vector2d(0.0, 0.0),
      .timeStep = 0.001,
      .steps = kLongHorizonSteps,
  };

  expectTwoDofRegression(
      runTwoLinkRevoluteChain(testCase), testCase.steps, testCase.timeStep);
}

TEST(WorldDart7Regression, ControlledRevoluteTorqueHoldAdvancesSingleStep)
{
  const ControlledRevoluteCase testCase{
      .gravity = Eigen::Vector3d::Zero(),
      .length = 1.0,
      .mass = 1.0,
      .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .position = 0.0,
      .velocity = 0.0,
      .torque = 0.5,
      .timeStep = 0.001,
      .steps = 1,
  };

  const SingleDofRun run = runControlledRevolute(testCase);

  expectSingleDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_GT(run.acceleration, 0.0);
  EXPECT_GT(run.velocity, testCase.velocity);
}

TEST(WorldDart7Regression, ControlledRevoluteTorqueHoldAdvancesRepeatedSteps)
{
  const ControlledRevoluteCase testCase{
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .length = 0.8,
      .mass = 1.5,
      .moment = Eigen::Vector3d(0.2, 0.2, 0.2),
      .position = 0.1,
      .velocity = -0.2,
      .torque = 1.25,
      .timeStep = 0.0005,
      .steps = 200,
  };

  const SingleDofRun run = runControlledRevolute(testCase);

  expectSingleDofRegression(run, testCase.steps, testCase.timeStep);
  EXPECT_NEAR(run.position, testCase.position, 1.0);
}
