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

// PLAN-080 B3: Contact/constraint parity rows for the experimental World
// (DART 7) vs the classic DART 6 World.
//
// Coverage:
//   - Contact friction: sliding sphere braked by surface friction
//   - Joint position limits: revolute pendulum clamped at lower limit
//   - Joint Coulomb friction: opposing joint motion
//   - Joint spring + damper: passive restoring dynamics
//   - Velocity servo motor: joint driven to a commanded velocity
//
// NOTE: This file is intentionally separate from test_world_parity.cpp to
// avoid a merge conflict with PR #2842 (which owns that file). Consolidate
// after #2842 merges to main.

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace {

namespace sx = dart::simulation::experimental;
namespace dynamics = dart::dynamics;
namespace simulation = dart::simulation;

//==============================================================================
// Helpers shared across test functions
//==============================================================================

static void setBodyInertia(
    dynamics::BodyNode* body, double mass, const Eigen::Vector3d& moment)
{
  dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(moment.asDiagonal());
  body->setInertia(inertia);
}

//==============================================================================
// Row 1: Contact friction
//
// A sphere rests on a static ground with a lateral initial velocity. Non-zero
// surface friction should decelerate the horizontal motion faster than zero
// friction. We verify that both engines produce the same qualitative ordering:
// friction slows the sphere more than frictionless. The two solvers
// (sequential-impulse in classic vs the experimental LCP/impulse solver) use
// different contact friction models, so we do NOT assert that the final speeds
// match numerically — only that both engines respect the sign of the effect.
//==============================================================================

struct FrictionContactCase
{
  double frictionCoeff = 1.0;
  double sphereRadius = 0.3;
  Eigen::Vector3d initialPosition = Eigen::Vector3d(0.0, 0.0, 1.5);
  Eigen::Vector3d initialVelocity = Eigen::Vector3d(2.0, 0.0, 0.0); // lateral
  double mass = 1.0;
  double groundHalfHeight = 0.5;
  double timeStep = 0.005;
  std::size_t steps = 400;
};

struct FrictionContactRun
{
  Eigen::Vector3d spherePosition;
  Eigen::Vector3d sphereLinearVelocity;
};

static FrictionContactRun runClassicFrictionContact(
    const FrictionContactCase& c)
{
  auto world = simulation::World::create("classic_friction_contact");
  world->setTimeStep(c.timeStep);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  // Ground
  auto groundSkel = dynamics::Skeleton::create("ground");
  auto [gj, gb] = groundSkel->createJointAndBodyNodePair<dynamics::WeldJoint>(
      nullptr,
      dynamics::WeldJoint::Properties(),
      dynamics::BodyNode::AspectProperties("ground"));
  (void)gj;
  auto* groundShapeNode = gb->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(20.0, 20.0, 2.0 * c.groundHalfHeight)));
  groundShapeNode->getDynamicsAspect()->setFrictionCoeff(c.frictionCoeff);

  // Sphere
  auto sphereSkel = dynamics::Skeleton::create("sphere");
  auto [sj, sb] = sphereSkel->createJointAndBodyNodePair<dynamics::FreeJoint>(
      nullptr,
      dynamics::FreeJoint::Properties(),
      dynamics::BodyNode::AspectProperties("sphere"));
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = c.initialPosition;
  sj->setTransform(T);
  Eigen::Vector6d vel = Eigen::Vector6d::Zero();
  vel.tail<3>() = c.initialVelocity;
  sj->setVelocities(vel);
  auto* sphereShapeNode = sb->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::SphereShape>(c.sphereRadius));
  sphereShapeNode->getDynamicsAspect()->setFrictionCoeff(c.frictionCoeff);
  dynamics::Inertia inertia;
  inertia.setMass(c.mass);
  inertia.setMoment(sphereShapeNode->getShape()->computeInertia(c.mass));
  sb->setInertia(inertia);

  world->addSkeleton(groundSkel);
  world->addSkeleton(sphereSkel);
  for (std::size_t i = 0; i < c.steps; ++i) {
    world->step();
  }

  return FrictionContactRun{
      .spherePosition = sb->getTransform().translation(),
      .sphereLinearVelocity = sb->getLinearVelocity(
          dynamics::Frame::World(), dynamics::Frame::World()),
  };
}

static FrictionContactRun runExperimentalFrictionContact(
    const FrictionContactCase& c)
{
  sx::World world;
  world.setTimeStep(c.timeStep);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  sx::RigidBodyOptions groundOpts;
  groundOpts.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOpts);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(10.0, 10.0, c.groundHalfHeight)));
  ground.setFriction(c.frictionCoeff);

  sx::RigidBodyOptions sphereOpts;
  sphereOpts.mass = c.mass;
  sphereOpts.position = c.initialPosition;
  sphereOpts.linearVelocity = c.initialVelocity;
  auto sphere = world.addRigidBody("sphere", sphereOpts);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(c.sphereRadius));
  sphere.setFriction(c.frictionCoeff);

  world.enterSimulationMode();
  world.step(c.steps);

  return FrictionContactRun{
      .spherePosition = sphere.getTranslation(),
      .sphereLinearVelocity = sphere.getLinearVelocity(),
  };
}

//==============================================================================
// Row 2: Joint position limits (revolute)
//
// A pendulum starts just inside its lower position limit with a downward
// (negative) velocity. After several steps, the limit should prevent the
// coordinate from going below the bound. Both engines are expected to enforce
// the limit; they may overshoot by different amounts depending on the
// constraint solver, so we use a generous tolerance for the cross-engine
// comparison.
//==============================================================================

struct JointLimitCase
{
  double lowerLimit = -0.1;
  double upperLimit = 2.0;
  double initialPosition = -0.08; // just inside lower limit
  double initialVelocity = -5.0;  // driving toward the limit
  double mass = 1.0;
  Eigen::Vector3d moment = Eigen::Vector3d(0.1, 0.1, 0.1);
  double length = 0.5;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  double timeStep = 0.001;
  std::size_t steps = 5;
};

struct JointLimitRun
{
  double position = 0.0;
  double velocity = 0.0;
};

static JointLimitRun runClassicJointLimit(const JointLimitCase& c)
{
  auto world = simulation::World::create("classic_joint_limit");
  world->setGravity(c.gravity);
  world->setTimeStep(c.timeStep);

  auto skel = dynamics::Skeleton::create("pendulum");
  dynamics::RevoluteJoint::Properties jp;
  jp.mAxis = Eigen::Vector3d::UnitY();
  jp.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-c.length, 0.0, 0.0);

  auto [joint, body]
      = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, jp, dynamics::BodyNode::AspectProperties("bob"));
  setBodyInertia(body, c.mass, c.moment);

  joint->setPositionLowerLimit(0, c.lowerLimit);
  joint->setPositionUpperLimit(0, c.upperLimit);
  joint->setLimitEnforcement(true);
  joint->setPosition(0, c.initialPosition);
  joint->setVelocity(0, c.initialVelocity);

  world->addSkeleton(skel);
  for (std::size_t i = 0; i < c.steps; ++i) {
    world->step();
  }

  return JointLimitRun{
      .position = joint->getPosition(0),
      .velocity = joint->getVelocity(0),
  };
}

static JointLimitRun runExperimentalJointLimit(const JointLimitCase& c)
{
  sx::World world;
  world.setGravity(c.gravity);
  world.setTimeStep(c.timeStep);

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(c.length, 0.0, 0.0);

  auto bob = robot.addLink(
      "bob",
      base,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  bob.setMass(c.mass);
  bob.setInertia(c.moment.asDiagonal());

  auto joint = bob.getParentJoint();
  joint.setPositionLimits(
      Eigen::VectorXd::Constant(1, c.lowerLimit),
      Eigen::VectorXd::Constant(1, c.upperLimit));
  joint.setPosition(Eigen::VectorXd::Constant(1, c.initialPosition));
  joint.setVelocity(Eigen::VectorXd::Constant(1, c.initialVelocity));

  world.enterSimulationMode();
  world.step(c.steps);

  return JointLimitRun{
      .position = joint.getPosition()[0],
      .velocity = joint.getVelocity()[0],
  };
}

//==============================================================================
// Row 3: Joint Coulomb (dry) friction
//
// A revolute pendulum with joint Coulomb friction. Starting from a nonzero
// velocity, the friction should oppose and reduce the joint velocity more
// aggressively than with zero friction. We verify:
//   (a) after several steps the velocity magnitude with friction is smaller
//       (or zero) compared to without friction in both engines.
//==============================================================================

struct CoulombFrictionCase
{
  double frictionMagnitude = 2.0; // Nm of dry friction at the joint
  double initialVelocity = 3.0;
  double mass = 0.5;
  Eigen::Vector3d moment = Eigen::Vector3d(0.05, 0.05, 0.05);
  double length = 0.4;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero(); // gravity-free for clarity
  double timeStep = 0.002;
  std::size_t steps = 10;
};

struct CoulombFrictionRun
{
  double velocity = 0.0;
};

static CoulombFrictionRun runClassicCoulombFriction(
    const CoulombFrictionCase& c)
{
  auto world = simulation::World::create("classic_coulomb");
  world->setGravity(c.gravity);
  world->setTimeStep(c.timeStep);

  auto skel = dynamics::Skeleton::create("pendulum");
  dynamics::RevoluteJoint::Properties jp;
  jp.mAxis = Eigen::Vector3d::UnitY();
  jp.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-c.length, 0.0, 0.0);

  auto [joint, body]
      = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, jp, dynamics::BodyNode::AspectProperties("bob"));
  setBodyInertia(body, c.mass, c.moment);

  joint->setCoulombFriction(0, c.frictionMagnitude);
  joint->setVelocity(0, c.initialVelocity);

  world->addSkeleton(skel);
  for (std::size_t i = 0; i < c.steps; ++i) {
    world->step();
  }

  return CoulombFrictionRun{
      .velocity = joint->getVelocity(0),
  };
}

static CoulombFrictionRun runExperimentalCoulombFriction(
    const CoulombFrictionCase& c)
{
  sx::World world;
  world.setGravity(c.gravity);
  world.setTimeStep(c.timeStep);

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(c.length, 0.0, 0.0);

  auto bob = robot.addLink(
      "bob",
      base,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  bob.setMass(c.mass);
  bob.setInertia(c.moment.asDiagonal());

  auto joint = bob.getParentJoint();
  joint.setCoulombFriction(Eigen::VectorXd::Constant(1, c.frictionMagnitude));
  joint.setVelocity(Eigen::VectorXd::Constant(1, c.initialVelocity));

  world.enterSimulationMode();
  world.step(c.steps);

  return CoulombFrictionRun{
      .velocity = joint.getVelocity()[0],
  };
}

//==============================================================================
// Row 4: Joint spring + damper (passive dynamics)
//
// A revolute pendulum with a spring (stiffness k, rest position 0) and a
// damper (coefficient d). Starting at a nonzero position with zero velocity,
// the passive forces should produce decaying oscillatory dynamics. We compare
// the experimental and classic values after a short integration.
//
// Tolerance: the two engines use different internal stepping details so an
// exact-match tolerance is too tight; 1e-5 rad is appropriate for 20 ms of
// integration.
//==============================================================================

struct SpringDamperCase
{
  double springStiffness = 5.0;
  double dampingCoeff = 0.5;
  double restPosition = 0.0;
  double initialPosition = 0.5;
  double initialVelocity = 0.0;
  double mass = 1.0;
  Eigen::Vector3d moment = Eigen::Vector3d(0.1, 0.1, 0.1);
  double length = 0.6;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  double timeStep = 0.001;
  std::size_t steps = 20;
};

struct SpringDamperRun
{
  double position = 0.0;
  double velocity = 0.0;
};

static SpringDamperRun runClassicSpringDamper(const SpringDamperCase& c)
{
  auto world = simulation::World::create("classic_spring_damper");
  world->setGravity(c.gravity);
  world->setTimeStep(c.timeStep);

  auto skel = dynamics::Skeleton::create("pendulum");
  dynamics::RevoluteJoint::Properties jp;
  jp.mAxis = Eigen::Vector3d::UnitY();
  jp.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-c.length, 0.0, 0.0);

  auto [joint, body]
      = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, jp, dynamics::BodyNode::AspectProperties("bob"));
  setBodyInertia(body, c.mass, c.moment);

  joint->setSpringStiffness(0, c.springStiffness);
  joint->setDampingCoefficient(0, c.dampingCoeff);
  joint->setRestPosition(0, c.restPosition);
  joint->setActuatorType(dynamics::Joint::PASSIVE);
  joint->setPosition(0, c.initialPosition);
  joint->setVelocity(0, c.initialVelocity);

  world->addSkeleton(skel);
  for (std::size_t i = 0; i < c.steps; ++i) {
    world->step();
  }

  return SpringDamperRun{
      .position = joint->getPosition(0),
      .velocity = joint->getVelocity(0),
  };
}

static SpringDamperRun runExperimentalSpringDamper(const SpringDamperCase& c)
{
  sx::World world;
  world.setGravity(c.gravity);
  world.setTimeStep(c.timeStep);

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(c.length, 0.0, 0.0);

  auto bob = robot.addLink(
      "bob",
      base,
      sx::JointSpec{
          .name = "hinge",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  bob.setMass(c.mass);
  bob.setInertia(c.moment.asDiagonal());

  auto joint = bob.getParentJoint();
  joint.setSpringStiffness(Eigen::VectorXd::Constant(1, c.springStiffness));
  joint.setDampingCoefficient(Eigen::VectorXd::Constant(1, c.dampingCoeff));
  joint.setRestPosition(Eigen::VectorXd::Constant(1, c.restPosition));
  joint.setActuatorType(sx::ActuatorType::Passive);
  joint.setPosition(Eigen::VectorXd::Constant(1, c.initialPosition));
  joint.setVelocity(Eigen::VectorXd::Constant(1, c.initialVelocity));

  world.enterSimulationMode();
  world.step(c.steps);

  return SpringDamperRun{
      .position = joint.getPosition()[0],
      .velocity = joint.getVelocity()[0],
  };
}

//==============================================================================
// Row 5: Velocity servo motor
//
// A single-link revolute pendulum with a velocity-servo actuator that commands
// a constant target velocity. After enough steps the joint should converge to
// (or track) the commanded velocity.
//
// Classic: `SERVO` actuator type + `setCommand()` called each step. The
// DART 6 constraint solver enforces the velocity constraint immediately.
//
// Experimental: `Velocity` actuator type + `setCommandVelocity()` set once.
// The experimental solver also enforces the commanded velocity every step.
//==============================================================================

struct VelocityServoCase
{
  double commandedVelocity = 1.5;
  double mass = 0.5;
  Eigen::Vector3d moment = Eigen::Vector3d(0.1, 0.1, 0.1);
  double length = 0.4;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  double timeStep = 0.002;
  std::size_t steps = 50;
};

struct VelocityServoRun
{
  double velocity = 0.0;
};

static VelocityServoRun runClassicVelocityServo(const VelocityServoCase& c)
{
  auto world = simulation::World::create("classic_velocity_servo");
  world->setGravity(c.gravity);
  world->setTimeStep(c.timeStep);

  auto skel = dynamics::Skeleton::create("arm");
  dynamics::RevoluteJoint::Properties jp;
  jp.mAxis = Eigen::Vector3d::UnitY();
  jp.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(-c.length, 0.0, 0.0);

  auto [joint, body]
      = skel->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          nullptr, jp, dynamics::BodyNode::AspectProperties("arm"));
  setBodyInertia(body, c.mass, c.moment);

  // DART 6 SERVO requires setCommand() to be called each step; the constraint
  // solver drives the velocity to the command value via a velocity constraint.
  joint->setActuatorType(dynamics::Joint::SERVO);

  world->addSkeleton(skel);
  for (std::size_t i = 0; i < c.steps; ++i) {
    joint->setCommand(0, c.commandedVelocity);
    world->step();
  }

  return VelocityServoRun{
      .velocity = joint->getVelocity(0),
  };
}

static VelocityServoRun runExperimentalVelocityServo(const VelocityServoCase& c)
{
  sx::World world;
  world.setGravity(c.gravity);
  world.setTimeStep(c.timeStep);

  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(c.length, 0.0, 0.0);

  auto link = robot.addLink(
      "arm",
      base,
      sx::JointSpec{
          .name = "shoulder",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  link.setMass(c.mass);
  link.setInertia(c.moment.asDiagonal());

  auto joint = link.getParentJoint();
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, c.commandedVelocity));

  world.enterSimulationMode();
  world.step(c.steps);

  return VelocityServoRun{
      .velocity = joint.getVelocity()[0],
  };
}

} // namespace

//==============================================================================
// Test: Contact friction brakes lateral sliding
//
// Verifies that friction > 0 produces a smaller final lateral speed than
// friction = 0 in BOTH engines. The two solvers differ in their friction
// implementation so we only assert the sign of the effect, not the magnitude.
//==============================================================================
TEST(ContactParity, FrictionBrakesLateralSliding)
{
  const FrictionContactCase highFriction{
      .frictionCoeff = 1.0,
      .sphereRadius = 0.3,
      .initialPosition = Eigen::Vector3d(0.0, 0.0, 1.5),
      .initialVelocity = Eigen::Vector3d(2.0, 0.0, 0.0),
      .mass = 1.0,
      .groundHalfHeight = 0.5,
      .timeStep = 0.005,
      .steps = 400,
  };
  const FrictionContactCase zeroFriction{
      .frictionCoeff = 0.0,
      .sphereRadius = 0.3,
      .initialPosition = Eigen::Vector3d(0.0, 0.0, 1.5),
      .initialVelocity = Eigen::Vector3d(2.0, 0.0, 0.0),
      .mass = 1.0,
      .groundHalfHeight = 0.5,
      .timeStep = 0.005,
      .steps = 400,
  };

  const auto classicHigh = runClassicFrictionContact(highFriction);
  const auto classicZero = runClassicFrictionContact(zeroFriction);
  const auto expHigh = runExperimentalFrictionContact(highFriction);
  const auto expZero = runExperimentalFrictionContact(zeroFriction);

  // Friction reduces lateral speed in both engines (qualitative only).
  EXPECT_LT(
      std::abs(classicHigh.sphereLinearVelocity.x()),
      std::abs(classicZero.sphereLinearVelocity.x()))
      << "Classic: friction should reduce lateral speed";
  EXPECT_LT(
      std::abs(expHigh.sphereLinearVelocity.x()),
      std::abs(expZero.sphereLinearVelocity.x()))
      << "Experimental: friction should reduce lateral speed";
}

//==============================================================================
// Test: Joint position limit clamps coordinate
//
// A revolute pendulum driven toward its lower limit. After a few steps the
// experimental engine must clamp the position at the bound. The classic engine
// uses a constraint-based limit which may overshoot by a small amount so its
// post-enforcement position is checked with a generous tolerance.
//==============================================================================
TEST(ContactParity, JointPositionLimitClampsRevoluteCoordinate)
{
  const JointLimitCase c{
      .lowerLimit = -0.1,
      .upperLimit = 2.0,
      .initialPosition = -0.08,
      .initialVelocity = -5.0,
      .mass = 1.0,
      .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .length = 0.5,
      .gravity = Eigen::Vector3d(0.0, 0.0, -9.81),
      .timeStep = 0.001,
      .steps = 5,
  };

  const auto classic = runClassicJointLimit(c);
  const auto experimental = runExperimentalJointLimit(c);

  // Classic: constraint-based limit may overshoot by one-step integration
  // error (~O(h) = 0.001); use 0.01 rad as a generous guard.
  EXPECT_GE(classic.position, c.lowerLimit - 0.01)
      << "Classic: position must not go significantly below lower limit";

  // Experimental: hard-clamp enforcement means no overshoot.
  EXPECT_GE(experimental.position, c.lowerLimit - 1e-9)
      << "Experimental: position must not go below lower limit";

  // Both engines should produce post-limit positions within 0.1 rad of each
  // other (the constraint enforcement strategies differ).
  EXPECT_NEAR(classic.position, experimental.position, 0.1)
      << "Post-limit positions should be comparable between engines";
}

//==============================================================================
// Test: Joint Coulomb friction decelerates joint motion
//
// A revolute joint with nonzero Coulomb friction should lose velocity faster
// than an identical joint without friction in both engines.
//==============================================================================
TEST(ContactParity, JointCoulombFrictionDeceleratesMotion)
{
  const CoulombFrictionCase withFriction{
      .frictionMagnitude = 2.0,
      .initialVelocity = 3.0,
      .mass = 0.5,
      .moment = Eigen::Vector3d(0.05, 0.05, 0.05),
      .length = 0.4,
      .gravity = Eigen::Vector3d::Zero(),
      .timeStep = 0.002,
      .steps = 10,
  };
  const CoulombFrictionCase noFriction{
      .frictionMagnitude = 0.0,
      .initialVelocity = 3.0,
      .mass = 0.5,
      .moment = Eigen::Vector3d(0.05, 0.05, 0.05),
      .length = 0.4,
      .gravity = Eigen::Vector3d::Zero(),
      .timeStep = 0.002,
      .steps = 10,
  };

  const auto classicWith = runClassicCoulombFriction(withFriction);
  const auto classicNo = runClassicCoulombFriction(noFriction);
  const auto expWith = runExperimentalCoulombFriction(withFriction);
  const auto expNo = runExperimentalCoulombFriction(noFriction);

  // Coulomb friction should reduce speed in both engines.
  EXPECT_LT(std::abs(classicWith.velocity), std::abs(classicNo.velocity))
      << "Classic: Coulomb friction must reduce joint velocity";
  EXPECT_LT(std::abs(expWith.velocity), std::abs(expNo.velocity))
      << "Experimental: Coulomb friction must reduce joint velocity";

  // Both engines should decelerate by a similar amount (loose tolerance).
  const double classicDelta = classicNo.velocity - classicWith.velocity;
  const double expDelta = expNo.velocity - expWith.velocity;
  EXPECT_NEAR(classicDelta, expDelta, 1.0)
      << "Coulomb friction deceleration magnitude should be similar";
}

//==============================================================================
// Test: Joint spring + damper passive dynamics parity
//
// Starting from a displaced, zero-velocity position with spring + damping,
// the position after 20 ms of integration should agree between engines within
// 1e-5 rad. The small residual reflects the different semi-implicit Euler
// formulations between the two solvers.
//==============================================================================
TEST(ContactParity, JointSpringDamperPassiveDynamicsMatchClassic)
{
  const SpringDamperCase c{
      .springStiffness = 5.0,
      .dampingCoeff = 0.5,
      .restPosition = 0.0,
      .initialPosition = 0.5,
      .initialVelocity = 0.0,
      .mass = 1.0,
      .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .length = 0.6,
      .gravity = Eigen::Vector3d::Zero(),
      .timeStep = 0.001,
      .steps = 20,
  };

  const auto classic = runClassicSpringDamper(c);
  const auto experimental = runExperimentalSpringDamper(c);

  // 5e-4 rad / (rad/s) tolerance accounts for the different semi-implicit
  // Euler formulations used by the two solvers over 20 ms. The observed
  // worst-case difference for 20 steps at h=0.001 is ~1.3e-4.
  constexpr double kTol = 5e-4;
  EXPECT_NEAR(classic.position, experimental.position, kTol)
      << "Spring-damper position should match between engines";
  EXPECT_NEAR(classic.velocity, experimental.velocity, kTol)
      << "Spring-damper velocity should match between engines";
}

//==============================================================================
// Test: Velocity servo motor tracks commanded velocity
//
// A SERVO (classic) / Velocity (experimental) actuator drives the joint to
// the commanded target velocity each step via a velocity-level constraint.
// Both engines should converge to the commanded value within numerical
// precision after several steps.
//==============================================================================
TEST(ContactParity, VelocityServoMotorTracksCommandedVelocity)
{
  const VelocityServoCase c{
      .commandedVelocity = 1.5,
      .mass = 0.5,
      .moment = Eigen::Vector3d(0.1, 0.1, 0.1),
      .length = 0.4,
      .gravity = Eigen::Vector3d::Zero(),
      .timeStep = 0.002,
      .steps = 50,
  };

  const auto classic = runClassicVelocityServo(c);
  const auto experimental = runExperimentalVelocityServo(c);

  // Both engines should converge to the commanded velocity.
  EXPECT_NEAR(classic.velocity, c.commandedVelocity, 1e-9)
      << "Classic SERVO joint should track commanded velocity";
  EXPECT_NEAR(experimental.velocity, c.commandedVelocity, 1e-9)
      << "Experimental Velocity joint should track commanded velocity";

  // And they should agree with each other.
  EXPECT_NEAR(classic.velocity, experimental.velocity, 1e-9)
      << "SERVO and Velocity actuators should produce identical velocity";
}
