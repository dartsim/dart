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

#include <dart/constraint/ball_joint_constraint.hpp>
#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/joint_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include <cmath>
#include <cstddef>

namespace {

struct ConstraintWorld
{
  dart::simulation::WorldPtr world;
  dart::dynamics::BodyNode* body1 = nullptr;
  dart::dynamics::BodyNode* body2 = nullptr;
  Eigen::Vector3d offset1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d offset2 = Eigen::Vector3d::Zero();
};

struct ConstraintSummary
{
  double initialAnchorError = 0.0;
  double maxAnchorError = 0.0;
  double finalAnchorError = 0.0;
  double maxLinearSpeed = 0.0;
};

struct JointLimitWorld
{
  dart::simulation::WorldPtr world;
  dart::dynamics::RevoluteJoint* joint = nullptr;
};

struct JointLimitSummary
{
  double initialViolation = 0.0;
  double maxViolation = 0.0;
  double finalViolation = 0.0;
  double finalPosition = 0.0;
  double maxAbsVelocity = 0.0;
};

struct ServoTrackingWorld
{
  dart::simulation::WorldPtr world;
  dart::dynamics::RevoluteJoint* joint = nullptr;
};

struct ServoTrackingSummary
{
  double maxVelocityError = 0.0;
  double maxPositionError = 0.0;
  double finalPositionError = 0.0;
  double maxAbsCommand = 0.0;
};

double dtFromState(const benchmark::State& state)
{
  return static_cast<double>(state.range(0)) * 1e-6;
}

int stepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(1));
}

int substepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(2));
}

dart::dynamics::SkeletonPtr createFreeBody(
    const std::string& name, const Eigen::Vector3d& position)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = pair.second;

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(0.2, 0.25, 0.3, 0.0, 0.0, 0.0);
  body->setInertia(inertia);

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions.tail<3>() = position;
  skeleton->setPositions(positions);

  return skeleton;
}

double anchorError(const ConstraintWorld& constraintWorld)
{
  const Eigen::Vector3d anchor1
      = constraintWorld.body1->getTransform() * constraintWorld.offset1;
  const Eigen::Vector3d anchor2
      = constraintWorld.body2->getTransform() * constraintWorld.offset2;
  return (anchor1 - anchor2).norm();
}

double maxLinearSpeed(const ConstraintWorld& constraintWorld)
{
  return std::max(
      constraintWorld.body1->getLinearVelocity().norm(),
      constraintWorld.body2->getLinearVelocity().norm());
}

ConstraintWorld createConstraintWorld(double dt)
{
  ConstraintWorld constraintWorld;
  constraintWorld.world = dart::simulation::World::create();
  constraintWorld.world->setGravity(Eigen::Vector3d::Zero());
  constraintWorld.world->setTimeStep(dt);

  auto skeleton1 = createFreeBody("constraint_body_1", Eigen::Vector3d::Zero());
  auto skeleton2
      = createFreeBody("constraint_body_2", Eigen::Vector3d(1.0, 0.0, 0.0));
  constraintWorld.body1 = skeleton1->getBodyNode(0);
  constraintWorld.body2 = skeleton2->getBodyNode(0);

  constraintWorld.world->addSkeleton(skeleton1);
  constraintWorld.world->addSkeleton(skeleton2);

  const Eigen::Vector3d anchorWorld(0.5, 0.0, 0.0);
  auto constraint = std::make_shared<dart::constraint::BallJointConstraint>(
      constraintWorld.body1, constraintWorld.body2, anchorWorld);

  constraintWorld.offset1
      = constraintWorld.body1->getTransform().inverse() * anchorWorld;
  constraintWorld.offset2
      = constraintWorld.body2->getTransform().inverse() * anchorWorld;

  constraintWorld.world->getConstraintSolver()->addConstraint(constraint);

  Eigen::Vector6d perturbed = skeleton2->getPositions();
  perturbed[3] += 0.35;
  perturbed[4] -= 0.15;
  perturbed[5] += 0.05;
  skeleton2->setPositions(perturbed);

  return constraintWorld;
}

JointLimitWorld createJointLimitWorld(double dt)
{
  JointLimitWorld limitWorld;
  limitWorld.world = dart::simulation::World::create();
  limitWorld.world->setGravity(Eigen::Vector3d::Zero());
  limitWorld.world->setTimeStep(dt);

  auto skeleton = dart::dynamics::Skeleton::create("joint_limit_skel");
  auto pair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(0.1, 0.12, 0.08, 0.0, 0.0, 0.0);
  body->setInertia(inertia);

  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setPositionLowerLimit(0, -0.1);
  joint->setPositionUpperLimit(0, 0.1);
  joint->setVelocityLowerLimit(0, -1.0);
  joint->setVelocityUpperLimit(0, 1.0);
  joint->setLimitEnforcement(true);

  skeleton->setPosition(0, 0.5);
  skeleton->setVelocity(0, 0.0);

  limitWorld.world->addSkeleton(skeleton);
  limitWorld.world->getConstraintSolver()->addConstraint(
      std::make_shared<dart::constraint::JointConstraint>(joint));
  limitWorld.joint = joint;

  return limitWorld;
}

ServoTrackingWorld createServoTrackingWorld(double dt)
{
  ServoTrackingWorld trackingWorld;
  trackingWorld.world = dart::simulation::World::create();
  trackingWorld.world->setGravity(Eigen::Vector3d::Zero());
  trackingWorld.world->setTimeStep(dt);

  auto skeleton = dart::dynamics::Skeleton::create("servo_tracking_skel");
  auto pair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(0.1, 0.12, 0.08, 0.0, 0.0, 0.0);
  body->setInertia(inertia);

  joint->setAxis(Eigen::Vector3d::UnitZ());
  joint->setActuatorType(dart::dynamics::Joint::SERVO);
  joint->setForceLowerLimit(0, -std::numeric_limits<double>::infinity());
  joint->setForceUpperLimit(0, std::numeric_limits<double>::infinity());

  trackingWorld.world->addSkeleton(skeleton);
  trackingWorld.joint = joint;
  return trackingWorld;
}

double jointLimitViolation(const dart::dynamics::RevoluteJoint* joint)
{
  const double position = joint->getPosition(0);
  const double lower = joint->getPositionLowerLimit(0);
  const double upper = joint->getPositionUpperLimit(0);
  return std::max({lower - position, position - upper, 0.0});
}

double servoCommand(double time)
{
  return std::sin(2.0 * time);
}

void stepWorld(const dart::simulation::WorldPtr& world, int substeps)
{
  if (substeps == 1) {
    world->step();
  } else {
    world->stepSubsteps(static_cast<std::size_t>(substeps));
  }
}

void runConstraintWorld(
    const dart::simulation::WorldPtr& world, int numOuterSteps, int substeps)
{
  for (int i = 0; i < numOuterSteps; ++i) {
    stepWorld(world, substeps);
  }
}

ConstraintSummary summarizeConstraintWorld(
    double dt, int numOuterSteps, int substeps)
{
  auto constraintWorld = createConstraintWorld(dt);

  ConstraintSummary summary;
  summary.initialAnchorError = anchorError(constraintWorld);
  summary.maxAnchorError = summary.initialAnchorError;
  summary.finalAnchorError = summary.initialAnchorError;

  for (int i = 0; i < numOuterSteps; ++i) {
    stepWorld(constraintWorld.world, substeps);

    const double error = anchorError(constraintWorld);
    if (!std::isfinite(error)) {
      summary.maxAnchorError = std::numeric_limits<double>::infinity();
      summary.finalAnchorError = std::numeric_limits<double>::infinity();
      return summary;
    }

    summary.maxAnchorError = std::max(summary.maxAnchorError, error);
    summary.finalAnchorError = error;
    summary.maxLinearSpeed
        = std::max(summary.maxLinearSpeed, maxLinearSpeed(constraintWorld));
  }

  return summary;
}

JointLimitSummary summarizeJointLimitWorld(
    double dt, int numOuterSteps, int substeps)
{
  auto limitWorld = createJointLimitWorld(dt);

  JointLimitSummary summary;
  summary.initialViolation = jointLimitViolation(limitWorld.joint);
  summary.maxViolation = summary.initialViolation;
  summary.finalViolation = summary.initialViolation;
  summary.finalPosition = limitWorld.joint->getPosition(0);
  summary.maxAbsVelocity = std::abs(limitWorld.joint->getVelocity(0));

  for (int i = 0; i < numOuterSteps; ++i) {
    stepWorld(limitWorld.world, substeps);

    const double violation = jointLimitViolation(limitWorld.joint);
    if (!std::isfinite(violation)) {
      summary.maxViolation = std::numeric_limits<double>::infinity();
      summary.finalViolation = std::numeric_limits<double>::infinity();
      return summary;
    }

    summary.maxViolation = std::max(summary.maxViolation, violation);
    summary.finalViolation = violation;
    summary.finalPosition = limitWorld.joint->getPosition(0);
    summary.maxAbsVelocity = std::max(
        summary.maxAbsVelocity, std::abs(limitWorld.joint->getVelocity(0)));
  }

  return summary;
}

ServoTrackingSummary summarizeServoTrackingWorld(
    double dt, int numOuterSteps, int substeps)
{
  auto trackingWorld = createServoTrackingWorld(dt);

  double expectedPosition = trackingWorld.joint->getPosition(0);
  ServoTrackingSummary summary;
  for (int i = 0; i < numOuterSteps; ++i) {
    const double command = servoCommand(trackingWorld.world->getTime());
    trackingWorld.joint->setCommand(0, command);
    stepWorld(trackingWorld.world, substeps);

    expectedPosition += command * dt;
    const double velocityError
        = std::abs(trackingWorld.joint->getVelocity(0) - command);
    const double positionError
        = std::abs(trackingWorld.joint->getPosition(0) - expectedPosition);

    if (!std::isfinite(velocityError) || !std::isfinite(positionError)) {
      summary.maxVelocityError = std::numeric_limits<double>::infinity();
      summary.maxPositionError = std::numeric_limits<double>::infinity();
      summary.finalPositionError = std::numeric_limits<double>::infinity();
      return summary;
    }

    summary.maxVelocityError
        = std::max(summary.maxVelocityError, velocityError);
    summary.maxPositionError
        = std::max(summary.maxPositionError, positionError);
    summary.finalPositionError = positionError;
    summary.maxAbsCommand = std::max(summary.maxAbsCommand, std::abs(command));
  }

  return summary;
}

void addCounters(benchmark::State& state, const ConstraintSummary& summary)
{
  state.counters["initial_anchor_error"] = summary.initialAnchorError;
  state.counters["max_anchor_error"] = summary.maxAnchorError;
  state.counters["final_anchor_error"] = summary.finalAnchorError;
  state.counters["max_linear_speed"] = summary.maxLinearSpeed;
}

void addCounters(benchmark::State& state, const JointLimitSummary& summary)
{
  state.counters["initial_limit_violation"] = summary.initialViolation;
  state.counters["max_limit_violation"] = summary.maxViolation;
  state.counters["final_limit_violation"] = summary.finalViolation;
  state.counters["final_position"] = summary.finalPosition;
  state.counters["max_abs_velocity"] = summary.maxAbsVelocity;
}

void addCounters(benchmark::State& state, const ServoTrackingSummary& summary)
{
  state.counters["max_servo_velocity_error"] = summary.maxVelocityError;
  state.counters["max_servo_position_error"] = summary.maxPositionError;
  state.counters["final_servo_position_error"] = summary.finalPositionError;
  state.counters["max_abs_command"] = summary.maxAbsCommand;
}

} // namespace

static void BM_BallConstraintWorldStep(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);

  for (auto _ : state) {
    auto constraintWorld = createConstraintWorld(dt);

    runConstraintWorld(constraintWorld.world, numOuterSteps, 1);

    double stateValue
        = anchorError(constraintWorld) + maxLinearSpeed(constraintWorld);
    benchmark::DoNotOptimize(stateValue);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numOuterSteps));
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(state, summarizeConstraintWorld(dt, numOuterSteps, 1));
  state.ResumeTiming();
}

static void BM_BallConstraintSubsteppedWorldStep(benchmark::State& state)
{
  const double outerDt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);
  const int substeps = substepsFromState(state);

  for (auto _ : state) {
    auto constraintWorld = createConstraintWorld(outerDt);

    runConstraintWorld(constraintWorld.world, numOuterSteps, substeps);

    double stateValue
        = anchorError(constraintWorld) + maxLinearSpeed(constraintWorld);
    benchmark::DoNotOptimize(stateValue);
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(internalSteps));
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(
      state, summarizeConstraintWorld(outerDt, numOuterSteps, substeps));
  state.ResumeTiming();
}

static void BM_JointLimitWorldStep(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);

  for (auto _ : state) {
    auto limitWorld = createJointLimitWorld(dt);

    runConstraintWorld(limitWorld.world, numOuterSteps, 1);

    double stateValue = jointLimitViolation(limitWorld.joint)
                        + std::abs(limitWorld.joint->getVelocity(0));
    benchmark::DoNotOptimize(stateValue);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numOuterSteps));
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(state, summarizeJointLimitWorld(dt, numOuterSteps, 1));
  state.ResumeTiming();
}

static void BM_JointLimitSubsteppedWorldStep(benchmark::State& state)
{
  const double outerDt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);
  const int substeps = substepsFromState(state);

  for (auto _ : state) {
    auto limitWorld = createJointLimitWorld(outerDt);

    runConstraintWorld(limitWorld.world, numOuterSteps, substeps);

    double stateValue = jointLimitViolation(limitWorld.joint)
                        + std::abs(limitWorld.joint->getVelocity(0));
    benchmark::DoNotOptimize(stateValue);
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(internalSteps));
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(
      state, summarizeJointLimitWorld(outerDt, numOuterSteps, substeps));
  state.ResumeTiming();
}

static void BM_ServoTrackingWorldStep(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);

  for (auto _ : state) {
    auto trackingWorld = createServoTrackingWorld(dt);

    for (int i = 0; i < numOuterSteps; ++i) {
      trackingWorld.joint->setCommand(
          0, servoCommand(trackingWorld.world->getTime()));
      stepWorld(trackingWorld.world, 1);
    }

    double stateValue = trackingWorld.joint->getPosition(0)
                        + trackingWorld.joint->getVelocity(0);
    benchmark::DoNotOptimize(stateValue);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numOuterSteps));
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(state, summarizeServoTrackingWorld(dt, numOuterSteps, 1));
  state.ResumeTiming();
}

static void BM_ServoTrackingSubsteppedWorldStep(benchmark::State& state)
{
  const double outerDt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);
  const int substeps = substepsFromState(state);

  for (auto _ : state) {
    auto trackingWorld = createServoTrackingWorld(outerDt);

    for (int i = 0; i < numOuterSteps; ++i) {
      trackingWorld.joint->setCommand(
          0, servoCommand(trackingWorld.world->getTime()));
      stepWorld(trackingWorld.world, substeps);
    }

    double stateValue = trackingWorld.joint->getPosition(0)
                        + trackingWorld.joint->getVelocity(0);
    benchmark::DoNotOptimize(stateValue);
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(internalSteps));
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(
      state, summarizeServoTrackingWorld(outerDt, numOuterSteps, substeps));
  state.ResumeTiming();
}

BENCHMARK(BM_BallConstraintWorldStep)
    ->Args({1000, 500})
    ->Args({5000, 500})
    ->Args({10000, 500});

BENCHMARK(BM_BallConstraintSubsteppedWorldStep)
    ->Args({10000, 500, 2})
    ->Args({10000, 500, 5})
    ->Args({10000, 500, 10});

BENCHMARK(BM_JointLimitWorldStep)->Args({1000, 500})->Args({10000, 500});

BENCHMARK(BM_JointLimitSubsteppedWorldStep)
    ->Args({10000, 500, 2})
    ->Args({10000, 500, 5});

BENCHMARK(BM_ServoTrackingWorldStep)->Args({1000, 1000})->Args({10000, 1000});

BENCHMARK(BM_ServoTrackingSubsteppedWorldStep)
    ->Args({10000, 1000, 2})
    ->Args({10000, 1000, 5});

BENCHMARK_MAIN();
