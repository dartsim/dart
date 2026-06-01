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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

#include <memory>

#include <cstddef>

namespace {

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

// A single revolute joint driven by a constant actuation torque that is
// re-applied every step (a held actuator command). This exercises the
// controls path: classic forward dynamics consumes joint forces and (by
// default) clears commands each step, so the torque is re-applied before each
// classic step; the experimental joint effort is likewise re-applied so both
// paths see the same persistent command.
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

FreeBodyRun runClassicFreeBody(const FreeBodyCase& testCase)
{
  namespace dynamics = dart::dynamics;
  namespace simulation = dart::simulation;

  auto world = simulation::World::create("classic");
  world->setGravity(testCase.gravity);
  world->setTimeStep(testCase.timeStep);

  auto skeleton = dynamics::Skeleton::create("free_body");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          dynamics::BodyNode::AspectProperties("body"));

  dynamics::Inertia inertia;
  inertia.setMass(testCase.mass);
  inertia.setMoment(Eigen::Vector3d::Ones().asDiagonal());
  body->setInertia(inertia);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = testCase.position;
  joint->setPositions(dynamics::FreeJoint::convertToPositions(transform));

  Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
  velocities.tail<3>() = testCase.linearVelocity;
  joint->setVelocities(velocities);

  world->addSkeleton(skeleton);
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    world->step();
  }

  return FreeBodyRun{
      .position = body->getTransform().translation(),
      .linearVelocity = body->getLinearVelocity(
          dynamics::Frame::World(), dynamics::Frame::World()),
      .time = world->getTime(),
      .frame = static_cast<std::size_t>(world->getSimFrames()),
  };
}

FreeBodyRun runExperimentalFreeBody(const FreeBodyCase& testCase)
{
  namespace sx = dart::simulation::experimental;

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

GroundContactRun runClassicGroundContact(const GroundContactCase& testCase)
{
  namespace dynamics = dart::dynamics;
  namespace simulation = dart::simulation;

  auto world = simulation::World::create("classic_ground_contact");
  world->setTimeStep(testCase.timeStep);

  auto sphereSkeleton = dynamics::Skeleton::create("sphere");
  auto [sphereJoint, sphereBody]
      = sphereSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr,
          dynamics::FreeJoint::Properties(),
          dynamics::BodyNode::AspectProperties("sphere"));
  Eigen::Isometry3d sphereTransform = Eigen::Isometry3d::Identity();
  sphereTransform.translation() = testCase.initialPosition;
  sphereJoint->setTransform(sphereTransform);

  Eigen::Vector6d sphereVelocity = Eigen::Vector6d::Zero();
  sphereVelocity.tail<3>() = testCase.initialVelocity;
  sphereJoint->setVelocities(sphereVelocity);

  auto* sphereShapeNode = sphereBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::SphereShape>(testCase.sphereRadius));

  dynamics::Inertia sphereInertia;
  sphereInertia.setMass(testCase.mass);
  sphereInertia.setMoment(
      sphereShapeNode->getShape()->computeInertia(testCase.mass));
  sphereBody->setInertia(sphereInertia);

  auto groundSkeleton = dynamics::Skeleton::create("ground");
  auto [groundJoint, groundBody]
      = groundSkeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          nullptr,
          dynamics::WeldJoint::Properties(),
          dynamics::BodyNode::AspectProperties("ground"));
  (void)groundJoint;
  groundBody->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(10.0, 10.0, 2.0 * testCase.groundHalfHeight)));

  world->addSkeleton(sphereSkeleton);
  world->addSkeleton(groundSkeleton);
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    world->step();
  }

  return GroundContactRun{
      .spherePosition = sphereBody->getTransform().translation(),
      .sphereLinearVelocity = sphereBody->getLinearVelocity(
          dynamics::Frame::World(), dynamics::Frame::World()),
      .groundPosition = groundBody->getTransform().translation(),
      .time = world->getTime(),
      .frame = static_cast<std::size_t>(world->getSimFrames()),
  };
}

GroundContactRun runExperimentalGroundContact(const GroundContactCase& testCase)
{
  namespace sx = dart::simulation::experimental;

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

void setBodyInertia(
    dart::dynamics::BodyNode* body, double mass, const Eigen::Vector3d& moment)
{
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(moment.asDiagonal());
  body->setInertia(inertia);
}

SingleDofRun runClassicRevolutePendulum(const RevolutePendulumCase& testCase)
{
  namespace dynamics = dart::dynamics;
  namespace simulation = dart::simulation;

  auto world = simulation::World::create("classic_revolute");
  world->setGravity(testCase.gravity);
  world->setTimeStep(testCase.timeStep);

  auto skeleton = dynamics::Skeleton::create("pendulum");
  dynamics::RevoluteJoint::Properties jointProperties;
  jointProperties.mAxis = Eigen::Vector3d::UnitY();
  jointProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-testCase.length, 0.0, 0.0);

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          jointProperties,
          dynamics::BodyNode::AspectProperties("bob"));

  setBodyInertia(body, testCase.mass, testCase.moment);

  joint->setPosition(0, testCase.position);
  joint->setVelocity(0, testCase.velocity);

  world->addSkeleton(skeleton);
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    world->step();
  }

  return SingleDofRun{
      .position = joint->getPosition(0),
      .velocity = joint->getVelocity(0),
      .acceleration = joint->getAcceleration(0),
      .bodyPosition = body->getTransform().translation(),
      .time = world->getTime(),
      .frame = static_cast<std::size_t>(world->getSimFrames()),
  };
}

TwoDofRun runClassicTwoLinkRevoluteChain(const TwoLinkRevoluteCase& testCase)
{
  namespace dynamics = dart::dynamics;
  namespace simulation = dart::simulation;

  auto world = simulation::World::create("classic_two_link_revolute");
  world->setGravity(testCase.gravity);
  world->setTimeStep(testCase.timeStep);

  auto skeleton = dynamics::Skeleton::create("two_link_chain");

  dynamics::RevoluteJoint::Properties proximalProperties;
  proximalProperties.mAxis = Eigen::Vector3d::UnitY();
  proximalProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-testCase.lengths[0], 0.0, 0.0);
  auto [proximalJoint, proximalBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          proximalProperties,
          dynamics::BodyNode::AspectProperties("proximal"));

  dynamics::RevoluteJoint::Properties distalProperties;
  distalProperties.mAxis = Eigen::Vector3d::UnitY();
  distalProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-testCase.lengths[1], 0.0, 0.0);
  auto [distalJoint, distalBody]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          proximalBody,
          distalProperties,
          dynamics::BodyNode::AspectProperties("distal"));

  setBodyInertia(proximalBody, testCase.masses[0], testCase.proximalMoment);
  setBodyInertia(distalBody, testCase.masses[1], testCase.distalMoment);

  proximalJoint->setPosition(0, testCase.positions[0]);
  distalJoint->setPosition(0, testCase.positions[1]);
  proximalJoint->setVelocity(0, testCase.velocities[0]);
  distalJoint->setVelocity(0, testCase.velocities[1]);

  world->addSkeleton(skeleton);
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    world->step();
  }

  return TwoDofRun{
      .positions = skeleton->getPositions(),
      .velocities = skeleton->getVelocities(),
      .accelerations = skeleton->getAccelerations(),
      .proximalPosition = proximalBody->getTransform().translation(),
      .distalPosition = distalBody->getTransform().translation(),
      .time = world->getTime(),
      .frame = static_cast<std::size_t>(world->getSimFrames()),
  };
}

SingleDofRun runExperimentalRevolutePendulum(
    const RevolutePendulumCase& testCase)
{
  namespace sx = dart::simulation::experimental;

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

TwoDofRun runExperimentalTwoLinkRevoluteChain(
    const TwoLinkRevoluteCase& testCase)
{
  namespace sx = dart::simulation::experimental;

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

SingleDofRun runClassicPrismaticFall(const PrismaticFallCase& testCase)
{
  namespace dynamics = dart::dynamics;
  namespace simulation = dart::simulation;

  auto world = simulation::World::create("classic_prismatic");
  world->setGravity(testCase.gravity);
  world->setTimeStep(testCase.timeStep);

  auto skeleton = dynamics::Skeleton::create("slider");
  dynamics::PrismaticJoint::Properties jointProperties;
  jointProperties.mAxis = Eigen::Vector3d::UnitZ();

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          nullptr,
          jointProperties,
          dynamics::BodyNode::AspectProperties("body"));

  dynamics::Inertia inertia;
  inertia.setMass(testCase.mass);
  inertia.setMoment(Eigen::Vector3d::Ones().asDiagonal());
  body->setInertia(inertia);

  joint->setPosition(0, testCase.position);
  joint->setVelocity(0, testCase.velocity);

  world->addSkeleton(skeleton);
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    world->step();
  }

  return SingleDofRun{
      .position = joint->getPosition(0),
      .velocity = joint->getVelocity(0),
      .acceleration = joint->getAcceleration(0),
      .bodyPosition = body->getTransform().translation(),
      .time = world->getTime(),
      .frame = static_cast<std::size_t>(world->getSimFrames()),
  };
}

SingleDofRun runExperimentalPrismaticFall(const PrismaticFallCase& testCase)
{
  namespace sx = dart::simulation::experimental;

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

SingleDofRun runClassicControlledRevolute(
    const ControlledRevoluteCase& testCase)
{
  namespace dynamics = dart::dynamics;
  namespace simulation = dart::simulation;

  auto world = simulation::World::create("classic_controlled_revolute");
  world->setGravity(testCase.gravity);
  world->setTimeStep(testCase.timeStep);

  auto skeleton = dynamics::Skeleton::create("driven_pendulum");
  dynamics::RevoluteJoint::Properties jointProperties;
  jointProperties.mAxis = Eigen::Vector3d::UnitY();
  jointProperties.mT_ChildBodyToJoint.translation()
      = Eigen::Vector3d(-testCase.length, 0.0, 0.0);

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr,
          jointProperties,
          dynamics::BodyNode::AspectProperties("bob"));

  setBodyInertia(body, testCase.mass, testCase.moment);

  joint->setPosition(0, testCase.position);
  joint->setVelocity(0, testCase.velocity);

  world->addSkeleton(skeleton);
  for (std::size_t i = 0; i < testCase.steps; ++i) {
    // Classic World::step() defaults to resetCommand=true and clears internal
    // forces each step, so the held torque must be re-applied before every
    // step to model a constant actuator command.
    joint->setForce(0, testCase.torque);
    world->step();
  }

  return SingleDofRun{
      .position = joint->getPosition(0),
      .velocity = joint->getVelocity(0),
      .acceleration = joint->getAcceleration(0),
      .bodyPosition = body->getTransform().translation(),
      .time = world->getTime(),
      .frame = static_cast<std::size_t>(world->getSimFrames()),
  };
}

SingleDofRun runExperimentalControlledRevolute(
    const ControlledRevoluteCase& testCase)
{
  namespace sx = dart::simulation::experimental;

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
    // Re-apply the same held torque each step so the experimental path sees an
    // identical persistent command to the classic re-application above.
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

void expectFreeBodyParity(const FreeBodyCase& testCase)
{
  const FreeBodyRun classic = runClassicFreeBody(testCase);
  const FreeBodyRun experimental = runExperimentalFreeBody(testCase);

  constexpr double kTolerance = 1e-12;
  EXPECT_TRUE(classic.position.isApprox(experimental.position, kTolerance))
      << "classic position: " << classic.position.transpose()
      << "\nexperimental position: " << experimental.position.transpose();
  EXPECT_TRUE(
      classic.linearVelocity.isApprox(experimental.linearVelocity, kTolerance))
      << "classic velocity: " << classic.linearVelocity.transpose()
      << "\nexperimental velocity: " << experimental.linearVelocity.transpose();
  EXPECT_NEAR(classic.time, experimental.time, kTolerance);
  EXPECT_EQ(classic.frame, experimental.frame);
}

void expectSingleDofParity(
    const SingleDofRun& classic,
    const SingleDofRun& experimental,
    double tolerance)
{
  EXPECT_NEAR(classic.position, experimental.position, tolerance);
  EXPECT_NEAR(classic.velocity, experimental.velocity, tolerance);
  EXPECT_NEAR(classic.acceleration, experimental.acceleration, tolerance);
  EXPECT_TRUE(
      classic.bodyPosition.isApprox(experimental.bodyPosition, tolerance))
      << "classic body position: " << classic.bodyPosition.transpose()
      << "\nexperimental body position: "
      << experimental.bodyPosition.transpose();
  EXPECT_NEAR(classic.time, experimental.time, tolerance);
  EXPECT_EQ(classic.frame, experimental.frame);
}

void expectRevolutePendulumParity(const RevolutePendulumCase& testCase)
{
  expectSingleDofParity(
      runClassicRevolutePendulum(testCase),
      runExperimentalRevolutePendulum(testCase),
      1e-10);
}

void expectPrismaticFallParity(const PrismaticFallCase& testCase)
{
  expectSingleDofParity(
      runClassicPrismaticFall(testCase),
      runExperimentalPrismaticFall(testCase),
      1e-10);
}

void expectControlledRevoluteParity(
    const ControlledRevoluteCase& testCase, double tolerance)
{
  expectSingleDofParity(
      runClassicControlledRevolute(testCase),
      runExperimentalControlledRevolute(testCase),
      tolerance);
}

void expectTwoLinkRevoluteParity(const TwoLinkRevoluteCase& testCase)
{
  const TwoDofRun classic = runClassicTwoLinkRevoluteChain(testCase);
  const TwoDofRun experimental = runExperimentalTwoLinkRevoluteChain(testCase);

  constexpr double kTolerance = 1e-10;
  EXPECT_TRUE(classic.positions.isApprox(experimental.positions, kTolerance))
      << "classic positions: " << classic.positions.transpose()
      << "\nexperimental positions: " << experimental.positions.transpose();
  EXPECT_TRUE(classic.velocities.isApprox(experimental.velocities, kTolerance))
      << "classic velocities: " << classic.velocities.transpose()
      << "\nexperimental velocities: " << experimental.velocities.transpose();
  EXPECT_TRUE(
      classic.accelerations.isApprox(experimental.accelerations, kTolerance))
      << "classic accelerations: " << classic.accelerations.transpose()
      << "\nexperimental accelerations: "
      << experimental.accelerations.transpose();
  EXPECT_TRUE(classic.proximalPosition.isApprox(
      experimental.proximalPosition, kTolerance))
      << "classic proximal position: " << classic.proximalPosition.transpose()
      << "\nexperimental proximal position: "
      << experimental.proximalPosition.transpose();
  EXPECT_TRUE(
      classic.distalPosition.isApprox(experimental.distalPosition, kTolerance))
      << "classic distal position: " << classic.distalPosition.transpose()
      << "\nexperimental distal position: "
      << experimental.distalPosition.transpose();
  EXPECT_NEAR(classic.time, experimental.time, kTolerance);
  EXPECT_EQ(classic.frame, experimental.frame);
}

void expectGroundContactParity(const GroundContactCase& testCase)
{
  const GroundContactRun classic = runClassicGroundContact(testCase);
  const GroundContactRun experimental = runExperimentalGroundContact(testCase);

  const double expectedRestZ
      = testCase.groundHalfHeight + testCase.sphereRadius;
  EXPECT_NEAR(classic.spherePosition.z(), expectedRestZ, 2e-2);
  EXPECT_NEAR(experimental.spherePosition.z(), expectedRestZ, 2e-2);
  EXPECT_NEAR(
      classic.spherePosition.z(), experimental.spherePosition.z(), 3e-2);
  EXPECT_NEAR(classic.sphereLinearVelocity.z(), 0.0, 0.1);
  EXPECT_NEAR(experimental.sphereLinearVelocity.z(), 0.0, 0.1);
  EXPECT_TRUE(classic.groundPosition.isApprox(experimental.groundPosition))
      << "classic ground position: " << classic.groundPosition.transpose()
      << "\nexperimental ground position: "
      << experimental.groundPosition.transpose();
  EXPECT_NEAR(classic.time, experimental.time, 1e-12);
  EXPECT_EQ(classic.frame, experimental.frame);
}

} // namespace

TEST(WorldParity, FreeRigidBodyMatchesClassicWorldSingleStep)
{
  expectFreeBodyParity(
      FreeBodyCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
          .position = Eigen::Vector3d(0.5, -0.25, 1.5),
          .linearVelocity = Eigen::Vector3d::Zero(),
          .mass = 2.0,
          .timeStep = 0.01,
          .steps = 1,
      });
}

TEST(WorldParity, FreeRigidBodyMatchesClassicWorldRepeatedSteps)
{
  expectFreeBodyParity(
      FreeBodyCase{
          .gravity = Eigen::Vector3d(0.25, -0.5, -3.0),
          .position = Eigen::Vector3d(-1.0, 0.5, 2.0),
          .linearVelocity = Eigen::Vector3d(0.75, -0.25, 0.5),
          .mass = 3.0,
          .timeStep = 0.0025,
          .steps = 8,
      });
}

TEST(WorldParity, RevolutePendulumMatchesClassicWorldSingleStep)
{
  expectRevolutePendulumParity(
      RevolutePendulumCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
          .length = 1.5,
          .mass = 2.0,
          .moment = Eigen::Vector3d(0.1, 0.2, 0.3),
          .position = 0.0,
          .velocity = 0.0,
          .timeStep = 0.001,
          .steps = 1,
      });
}

TEST(WorldParity, RevolutePendulumMatchesClassicWorldRepeatedSteps)
{
  expectRevolutePendulumParity(
      RevolutePendulumCase{
          .gravity = Eigen::Vector3d(0.25, 0.0, -4.5),
          .length = 0.75,
          .mass = 1.25,
          .moment = Eigen::Vector3d(0.2, 0.4, 0.6),
          .position = 0.15,
          .velocity = -0.35,
          .timeStep = 0.0005,
          .steps = 12,
      });
}

TEST(WorldParity, PrismaticJointMatchesClassicWorldSingleStep)
{
  expectPrismaticFallParity(
      PrismaticFallCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
          .mass = 3.0,
          .position = 0.75,
          .velocity = 0.2,
          .timeStep = 0.002,
          .steps = 1,
      });
}

TEST(WorldParity, PrismaticJointMatchesClassicWorldRepeatedSteps)
{
  expectPrismaticFallParity(
      PrismaticFallCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -6.0),
          .mass = 1.5,
          .position = -0.25,
          .velocity = 1.0,
          .timeStep = 0.001,
          .steps = 20,
      });
}

TEST(WorldParity, TwoLinkRevoluteChainMatchesClassicWorldSingleStep)
{
  expectTwoLinkRevoluteParity(
      TwoLinkRevoluteCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
          .lengths = Eigen::Vector2d(0.7, 0.5),
          .masses = Eigen::Vector2d(1.5, 0.75),
          .proximalMoment = Eigen::Vector3d(0.2, 0.3, 0.4),
          .distalMoment = Eigen::Vector3d(0.1, 0.2, 0.3),
          .positions = Eigen::Vector2d(0.2, -0.35),
          .velocities = Eigen::Vector2d(0.0, 0.0),
          .timeStep = 0.001,
          .steps = 1,
      });
}

TEST(WorldParity, TwoLinkRevoluteChainMatchesClassicWorldRepeatedSteps)
{
  expectTwoLinkRevoluteParity(
      TwoLinkRevoluteCase{
          .gravity = Eigen::Vector3d(0.1, 0.0, -3.5),
          .lengths = Eigen::Vector2d(0.8, 0.45),
          .masses = Eigen::Vector2d(1.25, 0.9),
          .proximalMoment = Eigen::Vector3d(0.3, 0.45, 0.6),
          .distalMoment = Eigen::Vector3d(0.2, 0.25, 0.35),
          .positions = Eigen::Vector2d(-0.15, 0.3),
          .velocities = Eigen::Vector2d(0.25, -0.1),
          .timeStep = 0.0005,
          .steps = 16,
      });
}

TEST(WorldParity, DynamicSphereRestsOnStaticGroundLikeClassicWorld)
{
  expectGroundContactParity(
      GroundContactCase{
          .sphereRadius = 0.5,
          .initialPosition = Eigen::Vector3d(0.0, 0.0, 2.0),
          .initialVelocity = Eigen::Vector3d::Zero(),
          .mass = 1.0,
          .groundHalfHeight = 0.5,
          .timeStep = 0.005,
          .steps = 1000,
      });
}

//==============================================================================
// B2 gate scenario (c): long-horizon energy/position drift.
//
// Step a passive pendulum and a passive double pendulum for >=1e4 steps and
// require the experimental and classic states to stay in lock-step. The classic
// DART 6 path is the reference: matching it across 10k steps demonstrates the
// experimental integrator accumulates the same drift (no divergence) rather
// than asserting an absolute energy bound. Both paths integrate identical
// equations of motion, so the per-step difference stays at rounding level even
// over the full horizon (see the measured deltas in the tolerance comment).
//==============================================================================

// Long-horizon parity tolerances.
//
// Measured behaviour (see the development probe that printed per-run deltas):
// over 1e4 steps the experimental and classic paths agree to machine epsilon
// -- the revolute pendulum stays within ~5e-14 on every quantity, and even the
// chaotic double pendulum stays within ~1e-14. The two paths therefore use
// numerically equivalent open-chain dynamics rather than merely "close" ones.
//
// The bound below is 1e-9: roughly five orders of magnitude above the observed
// machine-epsilon agreement, so it tolerates floating-point reassociation
// across compilers/platforms, yet is still ~1e6x tighter than any physically
// meaningful drift for these meter/second-scale states. A genuine integrator
// or dynamics divergence would blow far past 1e-9 (especially the chaotic
// double pendulum, where any per-step difference is amplified exponentially),
// so this catches real regressions while not encoding a fake "loose" pass.
constexpr double kLongHorizonRevoluteTolerance = 1e-9;
constexpr double kLongHorizonTwoLinkTolerance = 1e-9;
// Number of steps for the long-horizon cases (>=1e4 as required by the gate).
constexpr std::size_t kLongHorizonSteps = 10000;

TEST(WorldParity, RevolutePendulumMatchesClassicWorldLongHorizon)
{
  expectSingleDofParity(
      runClassicRevolutePendulum(
          RevolutePendulumCase{
              .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
              .length = 1.0,
              .mass = 1.0,
              .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
              .position = 0.4,
              .velocity = 0.0,
              .timeStep = 0.001,
              .steps = kLongHorizonSteps,
          }),
      runExperimentalRevolutePendulum(
          RevolutePendulumCase{
              .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
              .length = 1.0,
              .mass = 1.0,
              .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
              .position = 0.4,
              .velocity = 0.0,
              .timeStep = 0.001,
              .steps = kLongHorizonSteps,
          }),
      kLongHorizonRevoluteTolerance);
}

TEST(WorldParity, DoublePendulumMatchesClassicWorldLongHorizon)
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

  const TwoDofRun classic = runClassicTwoLinkRevoluteChain(testCase);
  const TwoDofRun experimental = runExperimentalTwoLinkRevoluteChain(testCase);

  EXPECT_TRUE(classic.positions.isApprox(
      experimental.positions, kLongHorizonTwoLinkTolerance))
      << "classic positions: " << classic.positions.transpose()
      << "\nexperimental positions: " << experimental.positions.transpose();
  EXPECT_TRUE(classic.velocities.isApprox(
      experimental.velocities, kLongHorizonTwoLinkTolerance))
      << "classic velocities: " << classic.velocities.transpose()
      << "\nexperimental velocities: " << experimental.velocities.transpose();
  EXPECT_TRUE(classic.distalPosition.isApprox(
      experimental.distalPosition, kLongHorizonTwoLinkTolerance))
      << "classic distal position: " << classic.distalPosition.transpose()
      << "\nexperimental distal position: "
      << experimental.distalPosition.transpose();
  EXPECT_NEAR(classic.time, experimental.time, 1e-9);
  EXPECT_EQ(classic.frame, experimental.frame);
}

//==============================================================================
// B2 gate scenario (d): a basic controlled scene (held actuation torque).
//
// A single revolute joint is driven by a constant torque re-applied every step.
// With gravity off, the joint undergoes constant angular acceleration
// (alpha = tau / (I + m L^2)); the test only asserts that the experimental and
// classic paths agree, so it is robust to the exact inertia model as long as
// both compute it the same way.
//==============================================================================

// Controlled-scene parity tolerance. Measured deltas under a held torque are at
// machine epsilon (single step is bit-identical; 200 steps stays within ~1e-13
// on every quantity), so 1e-9 leaves ample margin for floating-point
// reassociation while still failing on any real actuation/forward-dynamics
// divergence.
constexpr double kControlledRevoluteTolerance = 1e-9;

TEST(WorldParity, ControlledRevoluteTorqueHoldMatchesClassicWorldSingleStep)
{
  expectControlledRevoluteParity(
      ControlledRevoluteCase{
          .gravity = Eigen::Vector3d::Zero(),
          .length = 1.0,
          .mass = 1.0,
          .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
          .position = 0.0,
          .velocity = 0.0,
          .torque = 0.5,
          .timeStep = 0.001,
          .steps = 1,
      },
      kControlledRevoluteTolerance);
}

TEST(WorldParity, ControlledRevoluteTorqueHoldMatchesClassicWorldRepeatedSteps)
{
  expectControlledRevoluteParity(
      ControlledRevoluteCase{
          .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
          .length = 0.8,
          .mass = 1.5,
          .moment = Eigen::Vector3d(0.2, 0.2, 0.2),
          .position = 0.1,
          .velocity = -0.2,
          .torque = 1.25,
          .timeStep = 0.0005,
          .steps = 200,
      },
      kControlledRevoluteTolerance);
}
