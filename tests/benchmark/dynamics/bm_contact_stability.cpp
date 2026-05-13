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

#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace {

constexpr double kBoxSide = 0.4;
constexpr double kBoxHeight = 0.5;
constexpr double kGroundThickness = 0.2;

struct ContactStackSummary
{
  double maxPenetrationDepth = 0.0;
  double maxTopHeightError = 0.0;
  double finalTopHeightError = 0.0;
  double finalKineticEnergy = 0.0;
  double maxKineticEnergy = 0.0;
  double maxContactCount = 0.0;
};

int boxesFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(0));
}

double dtFromState(const benchmark::State& state)
{
  return static_cast<double>(state.range(1)) * 1e-6;
}

int stepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(2));
}

int substepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(3));
}

constexpr double kFrictionHoldCoeff = 1.0;
constexpr double kFrictionHoldForce = 2.0;
constexpr int kFrictionHoldWarmupSteps = 50;

struct FrictionHoldSummary
{
  double maxLateralDrift = 0.0;
  double finalLateralDrift = 0.0;
  double maxLateralSpeed = 0.0;
  double maxPenetrationDepth = 0.0;
  double maxContactCount = 0.0;
};

void configureBody(
    dart::dynamics::BodyNode* body,
    const std::shared_ptr<dart::dynamics::BoxShape>& shape,
    double mass,
    double frictionCoeff)
{
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(frictionCoeff);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.0);
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("contact_stack_ground");
  auto pair = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(6.0, 6.0, kGroundThickness));
  configureBody(body, shape, 1.0, 0.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * kGroundThickness);
  pair.first->setTransformFromParentBodyNode(transform);
  ground->setMobile(false);

  return ground;
}

dart::dynamics::SkeletonPtr createDynamicBox(std::size_t index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "contact_stack_box_" + std::to_string(index));
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(kBoxSide, kBoxSide, kBoxHeight));
  configureBody(body, shape, 1.0, 0.0);

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions[5] = 0.5 * kBoxHeight + static_cast<double>(index) * kBoxHeight;
  skeleton->setPositions(positions);

  return skeleton;
}

dart::simulation::WorldPtr createContactStackWorld(
    int numBoxes, double dt, std::vector<dart::dynamics::SkeletonPtr>* boxes)
{
  auto world = dart::simulation::World::create();
  world->setTimeStep(dt);
  world->addSkeleton(createGround());

  boxes->clear();
  boxes->reserve(static_cast<std::size_t>(numBoxes));
  for (int i = 0; i < numBoxes; ++i) {
    auto box = createDynamicBox(static_cast<std::size_t>(i));
    world->addSkeleton(box);
    boxes->push_back(std::move(box));
  }

  return world;
}

dart::dynamics::SkeletonPtr createFrictionGround()
{
  auto ground = dart::dynamics::Skeleton::create("friction_hold_ground");
  auto pair = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(6.0, 6.0, kGroundThickness));
  configureBody(body, shape, 1.0, kFrictionHoldCoeff);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * kGroundThickness);
  pair.first->setTransformFromParentBodyNode(transform);
  ground->setMobile(false);

  return ground;
}

dart::dynamics::SkeletonPtr createFrictionBox(
    dart::dynamics::BodyNode** bodyOut)
{
  auto skeleton = dart::dynamics::Skeleton::create("friction_hold_box");
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(kBoxSide, kBoxSide, kBoxSide));
  configureBody(body, shape, 1.0, kFrictionHoldCoeff);

  Eigen::Vector6d positions = Eigen::Vector6d::Zero();
  positions[5] = 0.5 * kBoxSide;
  skeleton->setPositions(positions);

  if (bodyOut != nullptr) {
    *bodyOut = body;
  }

  return skeleton;
}

dart::simulation::WorldPtr createFrictionHoldWorld(
    double dt, dart::dynamics::BodyNode** boxBodyOut)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(dt);
  world->addSkeleton(createFrictionGround());
  world->addSkeleton(createFrictionBox(boxBodyOut));
  return world;
}

double computeKineticEnergy(
    const std::vector<dart::dynamics::SkeletonPtr>& boxes)
{
  double energy = 0.0;
  for (const auto& box : boxes) {
    energy += box->computeKineticEnergy();
  }
  return energy;
}

double topHeightError(const std::vector<dart::dynamics::SkeletonPtr>& boxes)
{
  if (boxes.empty()) {
    return 0.0;
  }

  const double expected
      = kBoxHeight * (static_cast<double>(boxes.size()) - 0.5);
  const double actual
      = boxes.back()->getBodyNode(0)->getWorldTransform().translation().z();
  return actual - expected;
}

double maxPenetrationDepth(const dart::simulation::WorldPtr& world)
{
  double maxDepth = 0.0;
  for (const auto& contact : world->getLastCollisionResult().getContacts()) {
    maxDepth = std::max(maxDepth, contact.penetrationDepth);
  }
  return maxDepth;
}

ContactStackSummary summarizeContactStack(
    int numBoxes, double dt, int numOuterSteps, int substeps)
{
  std::vector<dart::dynamics::SkeletonPtr> boxes;
  auto world = createContactStackWorld(numBoxes, dt, &boxes);

  ContactStackSummary summary;
  for (int i = 0; i < numOuterSteps; ++i) {
    if (substeps == 1) {
      world->step();
    } else {
      world->stepSubsteps(static_cast<std::size_t>(substeps));
    }

    const double heightError = std::abs(topHeightError(boxes));
    const double kineticEnergy = computeKineticEnergy(boxes);
    summary.maxTopHeightError
        = std::max(summary.maxTopHeightError, heightError);
    summary.maxKineticEnergy
        = std::max(summary.maxKineticEnergy, kineticEnergy);
    summary.maxPenetrationDepth
        = std::max(summary.maxPenetrationDepth, maxPenetrationDepth(world));
    summary.maxContactCount = std::max(
        summary.maxContactCount,
        static_cast<double>(world->getLastCollisionResult().getNumContacts()));
  }

  summary.finalTopHeightError = topHeightError(boxes);
  summary.finalKineticEnergy = computeKineticEnergy(boxes);
  return summary;
}

void addCounters(benchmark::State& state, const ContactStackSummary& summary)
{
  state.counters["max_penetration_depth"] = summary.maxPenetrationDepth;
  state.counters["max_top_height_error"] = summary.maxTopHeightError;
  state.counters["final_top_height_error"] = summary.finalTopHeightError;
  state.counters["final_kinetic_energy"] = summary.finalKineticEnergy;
  state.counters["max_kinetic_energy"] = summary.maxKineticEnergy;
  state.counters["max_contact_count"] = summary.maxContactCount;
}

void addFrictionCounters(
    benchmark::State& state, const FrictionHoldSummary& summary)
{
  state.counters["max_lateral_drift"] = summary.maxLateralDrift;
  state.counters["final_lateral_drift"] = summary.finalLateralDrift;
  state.counters["max_lateral_speed"] = summary.maxLateralSpeed;
  state.counters["max_penetration_depth"] = summary.maxPenetrationDepth;
  state.counters["max_contact_count"] = summary.maxContactCount;
}

Eigen::Vector2d lateralPosition(const dart::dynamics::BodyNode* body)
{
  const Eigen::Vector3d translation = body->getWorldTransform().translation();
  return translation.head<2>();
}

double lateralSpeed(const dart::dynamics::BodyNode* body)
{
  return body->getLinearVelocity().head<2>().norm();
}

void runContactStack(
    const dart::simulation::WorldPtr& world, int numOuterSteps, int substeps)
{
  for (int i = 0; i < numOuterSteps; ++i) {
    if (substeps == 1) {
      world->step();
    } else {
      world->stepSubsteps(static_cast<std::size_t>(substeps));
    }
  }
}

void stepWorld(const dart::simulation::WorldPtr& world, int substeps)
{
  if (substeps == 1) {
    world->step();
  } else {
    world->stepSubsteps(static_cast<std::size_t>(substeps));
  }
}

void runFrictionHold(
    const dart::simulation::WorldPtr& world,
    dart::dynamics::BodyNode* boxBody,
    int numOuterSteps,
    int substeps)
{
  for (int i = 0; i < kFrictionHoldWarmupSteps; ++i) {
    stepWorld(world, substeps);
  }

  for (int i = 0; i < numOuterSteps; ++i) {
    boxBody->addExtForce(Eigen::Vector3d(kFrictionHoldForce, 0.0, 0.0));
    stepWorld(world, substeps);
  }
}

FrictionHoldSummary summarizeFrictionHold(
    double dt, int numOuterSteps, int substeps)
{
  dart::dynamics::BodyNode* boxBody = nullptr;
  auto world = createFrictionHoldWorld(dt, &boxBody);

  for (int i = 0; i < kFrictionHoldWarmupSteps; ++i) {
    stepWorld(world, substeps);
  }

  const Eigen::Vector2d initialPosition = lateralPosition(boxBody);
  FrictionHoldSummary summary;
  for (int i = 0; i < numOuterSteps; ++i) {
    boxBody->addExtForce(Eigen::Vector3d(kFrictionHoldForce, 0.0, 0.0));
    stepWorld(world, substeps);

    const double drift = (lateralPosition(boxBody) - initialPosition).norm();
    summary.maxLateralDrift = std::max(summary.maxLateralDrift, drift);
    summary.maxLateralSpeed
        = std::max(summary.maxLateralSpeed, lateralSpeed(boxBody));
    summary.maxPenetrationDepth
        = std::max(summary.maxPenetrationDepth, maxPenetrationDepth(world));
    summary.maxContactCount = std::max(
        summary.maxContactCount,
        static_cast<double>(world->getLastCollisionResult().getNumContacts()));
  }

  summary.finalLateralDrift
      = (lateralPosition(boxBody) - initialPosition).norm();
  return summary;
}

double frictionDtFromState(const benchmark::State& state)
{
  return static_cast<double>(state.range(0)) * 1e-6;
}

int frictionStepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(1));
}

int frictionSubstepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(2));
}

} // namespace

static void BM_ContactStackWorldStep(benchmark::State& state)
{
  const int numBoxes = boxesFromState(state);
  const double dt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);

  for (auto _ : state) {
    std::vector<dart::dynamics::SkeletonPtr> boxes;
    auto world = createContactStackWorld(numBoxes, dt, &boxes);

    runContactStack(world, numOuterSteps, 1);

    benchmark::DoNotOptimize(boxes.back()->getPositions().data());
    benchmark::DoNotOptimize(boxes.back()->getVelocities().data());
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numBoxes * numOuterSteps));
  state.counters["boxes"] = numBoxes;
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(state, summarizeContactStack(numBoxes, dt, numOuterSteps, 1));
  state.ResumeTiming();
}

static void BM_ContactStackSubsteppedWorldStep(benchmark::State& state)
{
  const int numBoxes = boxesFromState(state);
  const double outerDt = dtFromState(state);
  const int numOuterSteps = stepsFromState(state);
  const int substeps = substepsFromState(state);

  for (auto _ : state) {
    std::vector<dart::dynamics::SkeletonPtr> boxes;
    auto world = createContactStackWorld(numBoxes, outerDt, &boxes);

    runContactStack(world, numOuterSteps, substeps);

    benchmark::DoNotOptimize(boxes.back()->getPositions().data());
    benchmark::DoNotOptimize(boxes.back()->getVelocities().data());
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numBoxes * internalSteps));
  state.counters["boxes"] = numBoxes;
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.PauseTiming();
  addCounters(
      state, summarizeContactStack(numBoxes, outerDt, numOuterSteps, substeps));
  state.ResumeTiming();
}

static void BM_FrictionHoldWorldStep(benchmark::State& state)
{
  const double dt = frictionDtFromState(state);
  const int numOuterSteps = frictionStepsFromState(state);

  for (auto _ : state) {
    dart::dynamics::BodyNode* boxBody = nullptr;
    auto world = createFrictionHoldWorld(dt, &boxBody);

    runFrictionHold(world, boxBody, numOuterSteps, 1);

    double stateValue = boxBody->getWorldTransform().translation().x()
                        + boxBody->getLinearVelocity().x();
    benchmark::DoNotOptimize(stateValue);
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(numOuterSteps));
  state.counters["substeps"] = 1;
  state.counters["simulated_seconds"] = dt * static_cast<double>(numOuterSteps);
  state.counters["friction_coeff"] = kFrictionHoldCoeff;
  state.counters["external_force_n"] = kFrictionHoldForce;
  state.PauseTiming();
  addFrictionCounters(state, summarizeFrictionHold(dt, numOuterSteps, 1));
  state.ResumeTiming();
}

static void BM_FrictionHoldSubsteppedWorldStep(benchmark::State& state)
{
  const double outerDt = frictionDtFromState(state);
  const int numOuterSteps = frictionStepsFromState(state);
  const int substeps = frictionSubstepsFromState(state);

  for (auto _ : state) {
    dart::dynamics::BodyNode* boxBody = nullptr;
    auto world = createFrictionHoldWorld(outerDt, &boxBody);

    runFrictionHold(world, boxBody, numOuterSteps, substeps);

    double stateValue = boxBody->getWorldTransform().translation().x()
                        + boxBody->getLinearVelocity().x();
    benchmark::DoNotOptimize(stateValue);
  }

  const int internalSteps = numOuterSteps * substeps;
  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(internalSteps));
  state.counters["substeps"] = substeps;
  state.counters["simulated_seconds"]
      = outerDt * static_cast<double>(numOuterSteps);
  state.counters["friction_coeff"] = kFrictionHoldCoeff;
  state.counters["external_force_n"] = kFrictionHoldForce;
  state.PauseTiming();
  addFrictionCounters(
      state, summarizeFrictionHold(outerDt, numOuterSteps, substeps));
  state.ResumeTiming();
}

BENCHMARK(BM_ContactStackWorldStep)
    ->Args({3, 1000, 1000})
    ->Args({3, 5000, 1000})
    ->Args({3, 10000, 1000});

BENCHMARK(BM_ContactStackSubsteppedWorldStep)
    ->Args({3, 10000, 1000, 2})
    ->Args({3, 10000, 1000, 5})
    ->Args({3, 10000, 1000, 10});

BENCHMARK(BM_FrictionHoldWorldStep)->Args({10000, 1000});

BENCHMARK(BM_FrictionHoldSubsteppedWorldStep)
    ->Args({10000, 1000, 2})
    ->Args({10000, 1000, 5})
    ->Args({10000, 1000, 10});

BENCHMARK_MAIN();
