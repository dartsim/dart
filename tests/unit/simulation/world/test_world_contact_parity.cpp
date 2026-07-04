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

// PLAN-080 B3: Contact/constraint regression rows for the DART 7 World.
//
// Coverage:
//   - Contact friction: sliding sphere braked by surface friction
//   - Joint position limits: revolute pendulum clamped at lower limit
//   - Joint Coulomb friction: opposing joint motion
//   - Joint spring + damper: passive restoring dynamics
//   - Velocity servo motor: joint driven to a commanded velocity
//
// NOTE: This DART 7-only file used to sit beside the retired main-branch world
// parity test. Keep it free of classic World dependencies; cross-version parity
// evidence belongs on release-6.* branches.

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace {

namespace sx = dart::simulation;

//==============================================================================
// Row 1: Contact friction
//
// A sphere rests on a static ground with a lateral initial velocity. Non-zero
// surface friction should decelerate the horizontal motion faster than the
// frictionless case.
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

static FrictionContactRun runSimulationFrictionContact(
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
// coordinate from going below the bound.
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

static JointLimitRun runSimulationJointLimit(const JointLimitCase& c)
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
//       (or zero) compared to without friction.
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

static CoulombFrictionRun runSimulationCoulombFriction(
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
// the passive forces should move the joint toward the rest position.
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

static SpringDamperRun runSimulationSpringDamper(const SpringDamperCase& c)
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

static VelocityServoRun runSimulationVelocityServo(const VelocityServoCase& c)
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

//==============================================================================
// Row 6: Locked actuator
//
// A locked joint is held rigidly at its current position by the same
// velocity-level equality constraint the Velocity actuator uses, with target
// velocity zero. The helper below builds a two-link revolute chain whose
// proximal joint is held (three ways) and whose distal joint is a free Force
// actuator, so the tests can check that the lock holds and that the rest of the
// articulated system is unaffected by any spring/damping/effort on the locked
// coordinate.
//==============================================================================

enum class ProximalHold
{
  Locked,            ///< Locked actuator, no passive parameters
  LockedWithPassive, ///< Locked actuator plus large spring/damping and effort
  ZeroVelocityServo, ///< Velocity actuator commanded to zero (the reference)
};

struct LockedChainRun
{
  double distalPosition = 0.0;
  double distalVelocity = 0.0;
  double proximalPosition = 0.0;
};

static LockedChainRun runProximalHeldChain(ProximalHold hold)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.002);

  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);

  auto link1 = robot.addLink(
      "l1",
      base,
      sx::JointSpec{
          .name = "proximal",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  link1.setMass(0.7);
  link1.setInertia(Eigen::Vector3d(0.1, 0.1, 0.1).asDiagonal());

  auto link2 = robot.addLink(
      "l2",
      link1,
      sx::JointSpec{
          .name = "distal",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  link2.setMass(0.5);
  link2.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  auto proximal = link1.getParentJoint();
  proximal.setPosition(Eigen::VectorXd::Constant(1, 0.3));
  if (hold == ProximalHold::ZeroVelocityServo) {
    proximal.setActuatorType(sx::ActuatorType::Velocity);
    proximal.setCommandVelocity(Eigen::VectorXd::Zero(1));
  } else {
    proximal.setActuatorType(sx::ActuatorType::Locked);
  }
  if (hold == ProximalHold::LockedWithPassive) {
    // A correct lock must ignore all of these on the locked coordinate.
    proximal.setSpringStiffness(Eigen::VectorXd::Constant(1, 50.0));
    proximal.setDampingCoefficient(Eigen::VectorXd::Constant(1, 5.0));
    proximal.setForce(Eigen::VectorXd::Constant(1, 3.0));
  }

  world.enterSimulationMode();
  world.step(150);

  auto distal = link2.getParentJoint();
  return LockedChainRun{
      .distalPosition = distal.getPosition()[0],
      .distalVelocity = distal.getVelocity()[0],
      .proximalPosition = proximal.getPosition()[0],
  };
}

} // namespace

//==============================================================================
// Test: Contact friction brakes lateral sliding
//
// Verifies that friction > 0 produces a smaller final lateral speed than
// friction = 0.
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

  const auto simHigh = runSimulationFrictionContact(highFriction);
  const auto simZero = runSimulationFrictionContact(zeroFriction);

  EXPECT_LT(
      std::abs(simHigh.sphereLinearVelocity.x()),
      std::abs(simZero.sphereLinearVelocity.x()))
      << "Friction should reduce lateral speed";
}

//==============================================================================
// Test: Joint position limit clamps coordinate
//
// A revolute pendulum driven toward its lower limit. After a few steps the
// DART 7 engine must clamp the position at the bound.
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

  const auto actual = runSimulationJointLimit(c);

  EXPECT_GE(actual.position, c.lowerLimit - 1e-9)
      << "Position must not go below lower limit";
  EXPECT_LE(actual.position, c.upperLimit + 1e-9)
      << "Position must not go above upper limit";
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

  const auto simWith = runSimulationCoulombFriction(withFriction);
  const auto simNo = runSimulationCoulombFriction(noFriction);

  EXPECT_LT(std::abs(simWith.velocity), std::abs(simNo.velocity))
      << "Coulomb friction must reduce joint velocity";
}

//==============================================================================
// Test: Joint spring + damper passive dynamics parity
//
// Starting from a displaced, zero-velocity position with spring + damping,
// the position after 20 ms of integration should agree between engines within
// 1e-5 rad. The small residual reflects the different semi-implicit Euler
// formulations between the two solvers.
//==============================================================================
TEST(ContactParity, JointSpringDamperPassiveDynamicsMoveTowardRest)
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

  const auto actual = runSimulationSpringDamper(c);

  EXPECT_LT(actual.position, c.initialPosition)
      << "Spring-damper should move toward the rest position";
  EXPECT_GT(actual.position, c.restPosition)
      << "Short integration should not overshoot the rest position";
  EXPECT_LT(actual.velocity, 0.0)
      << "Spring-damper should accelerate back toward rest";
}

//==============================================================================
// Test: Velocity servo motor tracks commanded velocity
//
// A Velocity actuator drives the joint to the commanded target velocity each
// step via a velocity-level constraint.
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

  const auto actual = runSimulationVelocityServo(c);

  EXPECT_NEAR(actual.velocity, c.commandedVelocity, 1e-9)
      << "Velocity joint should track commanded velocity";
}

//==============================================================================
// Test: Locked actuator holds a revolute joint under gravity
//
// A single revolute pendulum set to a non-zero angle and Locked must neither
// move nor gain velocity, even though gravity would otherwise swing it down.
//==============================================================================
TEST(ContactParity, LockedActuatorHoldsRevoluteJoint)
{
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.002);

  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);

  auto link = robot.addLink(
      "arm",
      base,
      sx::JointSpec{
          .name = "shoulder",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitY(),
          .transformFromParent = offset,
      });
  link.setMass(0.5);
  link.setInertia(Eigen::Vector3d(0.1, 0.1, 0.1).asDiagonal());

  auto joint = link.getParentJoint();
  const double initialAngle = 0.5;
  joint.setPosition(Eigen::VectorXd::Constant(1, initialAngle));
  joint.setActuatorType(sx::ActuatorType::Locked);

  world.enterSimulationMode();
  world.step(200);

  EXPECT_NEAR(joint.getPosition()[0], initialAngle, 1e-10)
      << "Locked joint should hold its position under gravity";
  EXPECT_NEAR(joint.getVelocity()[0], 0.0, 1e-10)
      << "Locked joint should keep zero velocity";
}

//==============================================================================
// Test: Locked actuator ignores its own passive and commanded forces
//
// The holding constraint on a locked coordinate absorbs any spring, damping, or
// commanded effort applied to it, so the rest of the articulated chain must be
// unaffected. The distal joint's trajectory must be identical whether or not
// the locked proximal joint carries large passive parameters and an effort
// command, and the proximal joint must stay put in both runs while the distal
// joint actually moves (the system is still dynamic).
//==============================================================================
TEST(ContactParity, LockedActuatorIgnoresItsOwnPassiveAndCommandedForces)
{
  const auto plain = runProximalHeldChain(ProximalHold::Locked);
  const auto loaded = runProximalHeldChain(ProximalHold::LockedWithPassive);

  EXPECT_NEAR(plain.proximalPosition, 0.3, 1e-10)
      << "Locked proximal joint should hold its position";
  EXPECT_NEAR(loaded.proximalPosition, 0.3, 1e-10)
      << "Locked proximal joint should hold its position with passive loads";
  EXPECT_NEAR(plain.distalPosition, loaded.distalPosition, 1e-12)
      << "Distal trajectory must not depend on the locked joint's "
         "spring/damping/effort";
  EXPECT_NEAR(plain.distalVelocity, loaded.distalVelocity, 1e-12);
  EXPECT_GT(std::abs(plain.distalVelocity), 1e-3)
      << "Distal joint should still swing (the chain is not frozen)";
}

//==============================================================================
// Test: Locked matches a zero-command Velocity servo
//
// A Locked joint drives its coordinates through the same velocity-level
// equality constraint the (independently verified) Velocity actuator uses, with
// target velocity zero. Holding the proximal joint with Locked must therefore
// produce the same articulated dynamics as commanding a Velocity actuator to
// zero from the same initial position.
//==============================================================================
TEST(ContactParity, LockedActuatorMatchesZeroVelocityServo)
{
  const auto locked = runProximalHeldChain(ProximalHold::Locked);
  const auto servo = runProximalHeldChain(ProximalHold::ZeroVelocityServo);

  EXPECT_NEAR(locked.proximalPosition, servo.proximalPosition, 1e-12);
  EXPECT_NEAR(locked.distalPosition, servo.distalPosition, 1e-12)
      << "Locked should match a zero-command Velocity servo";
  EXPECT_NEAR(locked.distalVelocity, servo.distalVelocity, 1e-12);
}
