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

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <limits>
#include <string>

#include <cmath>
#include <cstddef>

namespace {

struct EnergyDriftSummary
{
  double maxRelativeEnergyDrift = 0.0;
  double finalRelativeEnergyDrift = 0.0;
  double initialEnergy = 0.0;
  double finalEnergy = 0.0;
};

struct ReferenceErrorSummary
{
  double positionRmsError = 0.0;
  double velocityRmsError = 0.0;
  double maxAbsPositionError = 0.0;
  double maxAbsVelocityError = 0.0;
  int referenceSubsteps = 0;
};

using SkeletonFactory = dart::dynamics::SkeletonPtr (*)(int);

double dtFromState(const benchmark::State& state)
{
  return static_cast<double>(state.range(1)) * 1e-6;
}

int linksFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(0));
}

int stepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(2));
}

int substepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(3));
}

dart::dynamics::SkeletonPtr createRevoluteChainPendulum(int numLinks)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "articulated_energy_" + std::to_string(numLinks));
  skeleton->disableSelfCollisionCheck();

  constexpr double mass = 1.0;
  constexpr double length = 1.0;
  constexpr double width = 0.05;
  const double transverseMoment
      = mass * (length * length + width * width) / 12.0;
  const double axialMoment = mass * width * width / 6.0;

  dart::dynamics::BodyNode* parent = nullptr;
  for (int i = 0; i < numLinks; ++i) {
    dart::dynamics::RevoluteJoint::Properties jointProperties;
    jointProperties.mName = "joint_" + std::to_string(i);
    jointProperties.mAxis = Eigen::Vector3d::UnitY();
    jointProperties.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    jointProperties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();

    if (parent != nullptr) {
      jointProperties.mT_ParentBodyToJoint.translation()
          = Eigen::Vector3d(0.0, 0.0, -length);
    }

    dart::dynamics::BodyNode::Properties bodyProperties;
    bodyProperties.mName = "link_" + std::to_string(i);
    bodyProperties.mInertia.setMass(mass);
    bodyProperties.mInertia.setLocalCOM(
        Eigen::Vector3d(0.0, 0.0, -0.5 * length));
    bodyProperties.mInertia.setMoment(
        transverseMoment, transverseMoment, axialMoment, 0.0, 0.0, 0.0);

    auto pair
        = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
            parent, jointProperties, bodyProperties);
    auto* joint = pair.first;
    parent = pair.second;

    joint->setDampingCoefficient(0, 0.0);
    joint->setCoulombFriction(0, 0.0);
    joint->setSpringStiffness(0, 0.0);
    joint->setLimitEnforcement(false);
  }

  Eigen::VectorXd positions(numLinks);
  Eigen::VectorXd velocities(numLinks);
  for (int i = 0; i < numLinks; ++i) {
    positions[i] = 0.35 - 0.08 * static_cast<double>(i);
    velocities[i] = 0.4 + 0.05 * static_cast<double>(i);
  }

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);

  return skeleton;
}

dart::dynamics::SkeletonPtr createBallChainPendulum(int numLinks)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "ball_articulated_energy_" + std::to_string(numLinks));
  skeleton->disableSelfCollisionCheck();

  constexpr double mass = 1.0;
  constexpr double length = 0.8;
  constexpr double width = 0.08;
  const double transverseMoment
      = mass * (length * length + width * width) / 12.0;
  const double axialMoment = mass * width * width / 6.0;

  dart::dynamics::BodyNode* parent = nullptr;
  for (int i = 0; i < numLinks; ++i) {
    dart::dynamics::BallJoint::Properties jointProperties;
    jointProperties.mName = "ball_joint_" + std::to_string(i);
    jointProperties.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
    jointProperties.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();

    if (parent != nullptr) {
      jointProperties.mT_ParentBodyToJoint.translation()
          = Eigen::Vector3d(0.0, 0.0, -length);
    }

    dart::dynamics::BodyNode::Properties bodyProperties;
    bodyProperties.mName = "ball_link_" + std::to_string(i);
    bodyProperties.mInertia.setMass(mass);
    bodyProperties.mInertia.setLocalCOM(
        Eigen::Vector3d(0.0, 0.0, -0.5 * length));
    bodyProperties.mInertia.setMoment(
        transverseMoment, transverseMoment, axialMoment, 0.0, 0.0, 0.0);

    auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::BallJoint>(
        parent, jointProperties, bodyProperties);
    auto* joint = pair.first;
    parent = pair.second;

    for (std::size_t j = 0; j < joint->getNumDofs(); ++j) {
      joint->setDampingCoefficient(j, 0.0);
      joint->setCoulombFriction(j, 0.0);
      joint->setSpringStiffness(j, 0.0);
      joint->setLimitEnforcement(false);
    }
  }

  Eigen::VectorXd positions(skeleton->getNumDofs());
  Eigen::VectorXd velocities(skeleton->getNumDofs());
  for (int i = 0; i < numLinks; ++i) {
    const int offset = 3 * i;
    positions.segment<3>(offset) = Eigen::Vector3d(
        0.12 - 0.02 * static_cast<double>(i),
        -0.07 + 0.015 * static_cast<double>(i),
        0.05 + 0.01 * static_cast<double>(i));
    velocities.segment<3>(offset) = Eigen::Vector3d(
        0.25 + 0.03 * static_cast<double>(i),
        -0.18 + 0.02 * static_cast<double>(i),
        0.12 - 0.015 * static_cast<double>(i));
  }

  skeleton->setPositions(positions);
  skeleton->setVelocities(velocities);

  return skeleton;
}

dart::simulation::WorldPtr createPendulumWorld(
    int numLinks,
    double dt,
    SkeletonFactory factory,
    dart::dynamics::SkeletonPtr* skeletonOut = nullptr)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(dt);
  auto skeleton = factory(numLinks);
  if (skeletonOut != nullptr) {
    *skeletonOut = skeleton;
  }
  world->addSkeleton(skeleton);
  return world;
}

double totalEnergy(const dart::dynamics::SkeletonPtr& skeleton)
{
  return skeleton->computeKineticEnergy() + skeleton->computePotentialEnergy();
}

EnergyDriftSummary summarizeWorldStepEnergyDrift(
    int numLinks, double dt, int numSteps, SkeletonFactory factory)
{
  dart::dynamics::SkeletonPtr skeleton;
  auto world = createPendulumWorld(numLinks, dt, factory, &skeleton);

  EnergyDriftSummary summary;
  summary.initialEnergy = totalEnergy(skeleton);
  summary.finalEnergy = summary.initialEnergy;

  const double denominator = std::max(1.0, std::abs(summary.initialEnergy));
  for (int i = 0; i < numSteps; ++i) {
    world->step();
    const double energy = totalEnergy(skeleton);

    if (!std::isfinite(energy)) {
      summary.maxRelativeEnergyDrift = std::numeric_limits<double>::infinity();
      summary.finalRelativeEnergyDrift
          = std::numeric_limits<double>::infinity();
      summary.finalEnergy = energy;
      return summary;
    }

    summary.finalEnergy = energy;
    const double relativeEnergyDrift
        = std::abs(energy - summary.initialEnergy) / denominator;
    summary.maxRelativeEnergyDrift
        = std::max(summary.maxRelativeEnergyDrift, relativeEnergyDrift);
  }

  summary.finalRelativeEnergyDrift
      = (summary.finalEnergy - summary.initialEnergy) / denominator;
  return summary;
}

EnergyDriftSummary summarizeRungeKutta4EnergyDrift(
    int numLinks, double dt, int numSteps, SkeletonFactory factory)
{
  dart::dynamics::SkeletonPtr skeleton;
  auto world = createPendulumWorld(numLinks, dt, factory, &skeleton);

  EnergyDriftSummary summary;
  summary.initialEnergy = totalEnergy(skeleton);
  summary.finalEnergy = summary.initialEnergy;

  const double denominator = std::max(1.0, std::abs(summary.initialEnergy));
  for (int i = 0; i < numSteps; ++i) {
    world->stepUnconstrainedRungeKutta4();
    const double energy = totalEnergy(skeleton);

    if (!std::isfinite(energy)) {
      summary.maxRelativeEnergyDrift = std::numeric_limits<double>::infinity();
      summary.finalRelativeEnergyDrift
          = std::numeric_limits<double>::infinity();
      summary.finalEnergy = energy;
      return summary;
    }

    summary.finalEnergy = energy;
    const double relativeEnergyDrift
        = std::abs(energy - summary.initialEnergy) / denominator;
    summary.maxRelativeEnergyDrift
        = std::max(summary.maxRelativeEnergyDrift, relativeEnergyDrift);
  }

  summary.finalRelativeEnergyDrift
      = (summary.finalEnergy - summary.initialEnergy) / denominator;
  return summary;
}

void addCounters(benchmark::State& state, const EnergyDriftSummary& summary)
{
  state.counters["max_rel_energy_drift"] = summary.maxRelativeEnergyDrift;
  state.counters["final_rel_energy_drift"] = summary.finalRelativeEnergyDrift;
  state.counters["initial_energy"] = summary.initialEnergy;
  state.counters["final_energy"] = summary.finalEnergy;
}

double rmsNorm(const Eigen::VectorXd& vector)
{
  if (vector.size() == 0) {
    return 0.0;
  }

  return std::sqrt(vector.squaredNorm() / static_cast<double>(vector.size()));
}

double maxAbsCoeff(const Eigen::VectorXd& vector)
{
  if (vector.size() == 0) {
    return 0.0;
  }

  return vector.cwiseAbs().maxCoeff();
}

void runWorldSteps(const dart::simulation::WorldPtr& world, int numSteps)
{
  for (int i = 0; i < numSteps; ++i) {
    world->step();
  }
}

void runWorldSubsteps(
    const dart::simulation::WorldPtr& world, int numOuterSteps, int substeps)
{
  for (int i = 0; i < numOuterSteps; ++i) {
    world->stepSubsteps(static_cast<std::size_t>(substeps));
  }
}

void runWorldRungeKutta4Steps(
    const dart::simulation::WorldPtr& world, int numSteps)
{
  for (int i = 0; i < numSteps; ++i) {
    world->stepUnconstrainedRungeKutta4();
  }
}

ReferenceErrorSummary summarizeRungeKutta4ReferenceError(
    int numLinks,
    double dt,
    int numSteps,
    int referenceSubsteps,
    SkeletonFactory factory)
{
  dart::dynamics::SkeletonPtr testedSkeleton;
  auto testedWorld
      = createPendulumWorld(numLinks, dt, factory, &testedSkeleton);
  runWorldRungeKutta4Steps(testedWorld, numSteps);

  dart::dynamics::SkeletonPtr referenceSkeleton;
  const double referenceDt = dt / static_cast<double>(referenceSubsteps);
  auto referenceWorld
      = createPendulumWorld(numLinks, referenceDt, factory, &referenceSkeleton);
  runWorldRungeKutta4Steps(referenceWorld, numSteps * referenceSubsteps);

  const Eigen::VectorXd positionError
      = testedSkeleton->getPositions() - referenceSkeleton->getPositions();
  const Eigen::VectorXd velocityError
      = testedSkeleton->getVelocities() - referenceSkeleton->getVelocities();

  ReferenceErrorSummary summary;
  summary.positionRmsError = rmsNorm(positionError);
  summary.velocityRmsError = rmsNorm(velocityError);
  summary.maxAbsPositionError = maxAbsCoeff(positionError);
  summary.maxAbsVelocityError = maxAbsCoeff(velocityError);
  summary.referenceSubsteps = referenceSubsteps;
  return summary;
}

void addReferenceErrorCounters(
    benchmark::State& state, const ReferenceErrorSummary& summary)
{
  state.counters["reference_substeps"] = summary.referenceSubsteps;
  state.counters["position_rms_error_vs_ref"] = summary.positionRmsError;
  state.counters["velocity_rms_error_vs_ref"] = summary.velocityRmsError;
  state.counters["max_abs_position_error_vs_ref"] = summary.maxAbsPositionError;
  state.counters["max_abs_velocity_error_vs_ref"] = summary.maxAbsVelocityError;
}

} // namespace

static void BM_ArticulatedEnergy_RevoluteChainWorldStep(benchmark::State& state)
{
  const int numLinks = linksFromState(state);
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    dart::dynamics::SkeletonPtr skeleton;
    auto world = createPendulumWorld(
        numLinks, dt, createRevoluteChainPendulum, &skeleton);

    runWorldSteps(world, numSteps);

    benchmark::DoNotOptimize(skeleton->getPositions().data());
    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numLinks * numSteps));
  state.counters["links"] = numLinks;
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numSteps);
  state.PauseTiming();
  addCounters(
      state,
      summarizeWorldStepEnergyDrift(
          numLinks, dt, numSteps, createRevoluteChainPendulum));
  state.ResumeTiming();
}

static void BM_ArticulatedEnergy_RevoluteChainSubsteppedWorldStep(
    benchmark::State& state)
{
  const int numLinks = linksFromState(state);
  const double outerDt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);
  const int substeps = substepsFromState(state);
  const double internalDt = outerDt / static_cast<double>(substeps);

  for (auto _ : state) {
    dart::dynamics::SkeletonPtr skeleton;
    auto world = createPendulumWorld(
        numLinks, outerDt, createRevoluteChainPendulum, &skeleton);

    runWorldSubsteps(world, numOuterSteps, substeps);

    benchmark::DoNotOptimize(skeleton->getPositions().data());
    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numLinks * internalSteps));
  state.counters["links"] = numLinks;
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(
      state,
      summarizeWorldStepEnergyDrift(
          numLinks, internalDt, internalSteps, createRevoluteChainPendulum));
  state.ResumeTiming();
}

static void BM_ArticulatedEnergy_RevoluteChainRungeKutta4(
    benchmark::State& state)
{
  const int numLinks = linksFromState(state);
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    dart::dynamics::SkeletonPtr skeleton;
    auto world = createPendulumWorld(
        numLinks, dt, createRevoluteChainPendulum, &skeleton);

    runWorldRungeKutta4Steps(world, numSteps);

    benchmark::DoNotOptimize(world.get());
    benchmark::DoNotOptimize(skeleton->getPositions().data());
    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numLinks * numSteps));
  state.counters["links"] = numLinks;
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numSteps);
  state.PauseTiming();
  addCounters(
      state,
      summarizeRungeKutta4EnergyDrift(
          numLinks, dt, numSteps, createRevoluteChainPendulum));
  addReferenceErrorCounters(
      state,
      summarizeRungeKutta4ReferenceError(
          numLinks, dt, numSteps, 10, createRevoluteChainPendulum));
  state.ResumeTiming();
}

static void BM_ArticulatedEnergy_BallChainWorldStep(benchmark::State& state)
{
  const int numLinks = linksFromState(state);
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    dart::dynamics::SkeletonPtr skeleton;
    auto world
        = createPendulumWorld(numLinks, dt, createBallChainPendulum, &skeleton);

    runWorldSteps(world, numSteps);

    benchmark::DoNotOptimize(skeleton->getPositions().data());
    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(3 * numLinks * numSteps));
  state.counters["links"] = numLinks;
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numSteps);
  state.PauseTiming();
  addCounters(
      state,
      summarizeWorldStepEnergyDrift(
          numLinks, dt, numSteps, createBallChainPendulum));
  state.ResumeTiming();
}

static void BM_ArticulatedEnergy_BallChainSubsteppedWorldStep(
    benchmark::State& state)
{
  const int numLinks = linksFromState(state);
  const double outerDt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);
  const int substeps = substepsFromState(state);
  const double internalDt = outerDt / static_cast<double>(substeps);

  for (auto _ : state) {
    dart::dynamics::SkeletonPtr skeleton;
    auto world = createPendulumWorld(
        numLinks, outerDt, createBallChainPendulum, &skeleton);

    runWorldSubsteps(world, numOuterSteps, substeps);

    benchmark::DoNotOptimize(skeleton->getPositions().data());
    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(3 * numLinks * internalSteps));
  state.counters["links"] = numLinks;
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(
      state,
      summarizeWorldStepEnergyDrift(
          numLinks, internalDt, internalSteps, createBallChainPendulum));
  state.ResumeTiming();
}

static void BM_ArticulatedEnergy_BallChainRungeKutta4(benchmark::State& state)
{
  const int numLinks = linksFromState(state);
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    dart::dynamics::SkeletonPtr skeleton;
    auto world
        = createPendulumWorld(numLinks, dt, createBallChainPendulum, &skeleton);

    runWorldRungeKutta4Steps(world, numSteps);

    benchmark::DoNotOptimize(world.get());
    benchmark::DoNotOptimize(skeleton->getPositions().data());
    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(3 * numLinks * numSteps));
  state.counters["links"] = numLinks;
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numSteps);
  state.PauseTiming();
  addCounters(
      state,
      summarizeRungeKutta4EnergyDrift(
          numLinks, dt, numSteps, createBallChainPendulum));
  addReferenceErrorCounters(
      state,
      summarizeRungeKutta4ReferenceError(
          numLinks, dt, numSteps, 10, createBallChainPendulum));
  state.ResumeTiming();
}

BENCHMARK(BM_ArticulatedEnergy_RevoluteChainWorldStep)
    ->Args({1, 1000, 1000})
    ->Args({2, 1000, 1000})
    ->Args({5, 1000, 1000})
    ->Args({5, 5000, 1000})
    ->Args({5, 10000, 1000});

BENCHMARK(BM_ArticulatedEnergy_RevoluteChainSubsteppedWorldStep)
    ->Args({5, 10000, 1000, 2})
    ->Args({5, 10000, 1000, 5})
    ->Args({5, 10000, 1000, 10});

BENCHMARK(BM_ArticulatedEnergy_RevoluteChainRungeKutta4)
    ->Args({5, 10000, 1000});

BENCHMARK(BM_ArticulatedEnergy_BallChainWorldStep)
    ->Args({3, 1000, 1000})
    ->Args({3, 5000, 1000});

BENCHMARK(BM_ArticulatedEnergy_BallChainSubsteppedWorldStep)
    ->Args({3, 5000, 1000, 2})
    ->Args({3, 5000, 1000, 5});

BENCHMARK(BM_ArticulatedEnergy_BallChainRungeKutta4)->Args({3, 5000, 1000});

BENCHMARK_MAIN();
