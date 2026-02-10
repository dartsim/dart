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

/// @file
/// @brief Benchmarks comparing classic and experimental simulation APIs.

#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/dynamics/forward_dynamics.hpp>
#include <dart/simulation/experimental/multi_body/joint.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <benchmark/benchmark.h>

#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace classic = dart;
namespace simexp = dart::simulation::experimental;

namespace {

constexpr int kLinkCount = 10;
constexpr int kJointCount = kLinkCount - 1;

struct ClassicChain
{
  classic::simulation::WorldPtr world;
  classic::dynamics::SkeletonPtr skeleton;
  std::vector<classic::dynamics::RevoluteJoint*> joints;
  std::vector<classic::dynamics::BodyNode*> links;
};

struct ExperimentalChain
{
  std::unique_ptr<simexp::World> world;
  simexp::MultiBody multiBody;
  std::vector<simexp::Joint> joints;
  std::vector<simexp::Link> links;
};

ClassicChain createClassicChain(const int linkCount)
{
  ClassicChain chain;
  chain.world = classic::simulation::World::create("classic_bench_world");
  chain.skeleton = classic::dynamics::Skeleton::create("classic_bench_chain");
  chain.joints.reserve(static_cast<std::size_t>(linkCount - 1));
  chain.links.reserve(static_cast<std::size_t>(linkCount));

  classic::dynamics::BodyNode* parentBody = nullptr;

  for (int i = 0; i < linkCount; ++i) {
    classic::dynamics::BodyNode::Properties bodyProps;
    bodyProps.mName = "link_" + std::to_string(i);
    bodyProps.mInertia.setMass(1.0);
    bodyProps.mInertia.setMoment(Eigen::Matrix3d::Identity() * 0.01);

    classic::dynamics::RevoluteJoint::Properties jointProps;
    jointProps.mName = "joint_" + std::to_string(i);
    jointProps.mAxis = Eigen::Vector3d::UnitZ();

    classic::dynamics::RevoluteJoint* joint = nullptr;
    classic::dynamics::BodyNode* body = nullptr;

    if (i == 0) {
      auto pair
          = chain.skeleton
                ->createJointAndBodyNodePair<classic::dynamics::RevoluteJoint>(
                    nullptr, jointProps, bodyProps);
      joint = pair.first;
      body = pair.second;
    } else {
      auto pair = parentBody->createChildJointAndBodyNodePair<
          classic::dynamics::RevoluteJoint>(jointProps, bodyProps);
      joint = pair.first;
      body = pair.second;
      chain.joints.push_back(joint);
    }

    chain.links.push_back(body);
    parentBody = body;
  }

  chain.world->addSkeleton(chain.skeleton);
  return chain;
}

ExperimentalChain createExperimentalChain(const int linkCount)
{
  ExperimentalChain chain{
      std::make_unique<simexp::World>(),
      simexp::MultiBody(entt::null, nullptr),
      {},
      {}};

  chain.multiBody = chain.world->addMultiBody("experimental_bench_chain");
  chain.joints.reserve(static_cast<std::size_t>(linkCount - 1));
  chain.links.reserve(static_cast<std::size_t>(linkCount));

  auto parent = chain.multiBody.addLink("link_0");
  parent.setMass(1.0);
  parent.setInertia(Eigen::Matrix3d::Identity() * 0.01);
  chain.links.push_back(parent);

  for (int i = 1; i < linkCount; ++i) {
    auto link = chain.multiBody.addLink(
        "link_" + std::to_string(i),
        {.parentLink = parent,
         .jointName = "joint_" + std::to_string(i),
         .jointType = simexp::comps::JointType::Revolute,
         .axis = Eigen::Vector3d::UnitZ()});

    link.setMass(1.0);
    link.setInertia(Eigen::Matrix3d::Identity() * 0.01);

    chain.joints.push_back(link.getParentJoint());
    chain.links.push_back(link);
    parent = link;
  }

  return chain;
}

static void BM_Classic_CreateChain(benchmark::State& state)
{
  for (auto _ : state) {
    auto chain = createClassicChain(kLinkCount);
    benchmark::DoNotOptimize(chain.skeleton->getNumBodyNodes());
  }

  state.SetItemsProcessed(state.iterations() * kLinkCount);
}
BENCHMARK(BM_Classic_CreateChain);

static void BM_Experimental_CreateChain(benchmark::State& state)
{
  for (auto _ : state) {
    auto chain = createExperimentalChain(kLinkCount);
    benchmark::DoNotOptimize(chain.multiBody.getLinkCount());
  }

  state.SetItemsProcessed(state.iterations() * kLinkCount);
}
BENCHMARK(BM_Experimental_CreateChain);

static void BM_Classic_ForwardKinematics(benchmark::State& state)
{
  auto chain = createClassicChain(kLinkCount);
  Eigen::VectorXd positions
      = Eigen::VectorXd::Zero(chain.skeleton->getNumDofs());
  double phase = 0.0;

  for (auto _ : state) {
    phase += 0.01;
    for (int i = 0; i < positions.size(); ++i) {
      positions[i] = std::sin(phase + static_cast<double>(i) * 0.2);
    }

    chain.skeleton->setPositions(positions);
    chain.skeleton->computeForwardKinematics(true, false, false);
    for (const auto* link : chain.links) {
      benchmark::DoNotOptimize(link->getWorldTransform().matrix().data());
    }
  }

  state.SetItemsProcessed(state.iterations() * kLinkCount);
}
BENCHMARK(BM_Classic_ForwardKinematics);

static void BM_Experimental_ForwardKinematics(benchmark::State& state)
{
  auto chain = createExperimentalChain(kLinkCount);
  chain.world->enterSimulationMode();
  Eigen::VectorXd position(1);
  double phase = 0.0;

  for (auto _ : state) {
    phase += 0.01;

    for (std::size_t i = 0; i < chain.joints.size(); ++i) {
      position[0] = std::sin(phase + static_cast<double>(i) * 0.2);
      chain.joints[i].setPosition(position);
    }

    chain.world->updateKinematics();
    for (const auto& link : chain.links) {
      benchmark::DoNotOptimize(link.getWorldTransform().matrix().data());
    }
  }

  state.SetItemsProcessed(state.iterations() * kLinkCount);
}
BENCHMARK(BM_Experimental_ForwardKinematics);

static void BM_Classic_ForwardDynamicsABA(benchmark::State& state)
{
  auto chain = createClassicChain(kLinkCount);
  Eigen::VectorXd positions
      = Eigen::VectorXd::Zero(chain.skeleton->getNumDofs());
  Eigen::VectorXd velocities
      = Eigen::VectorXd::Zero(chain.skeleton->getNumDofs());
  Eigen::VectorXd forces = Eigen::VectorXd::Zero(chain.skeleton->getNumDofs());
  double phase = 0.0;

  for (auto _ : state) {
    phase += 0.01;
    for (int i = 0; i < positions.size(); ++i) {
      positions[i] = 0.1 * std::sin(phase + static_cast<double>(i) * 0.2);
      velocities[i] = 0.1 * std::cos(phase + static_cast<double>(i) * 0.2);
      forces[i] = 0.2 * std::sin(phase + static_cast<double>(i) * 0.1);
    }

    chain.skeleton->setPositions(positions);
    chain.skeleton->setVelocities(velocities);
    chain.skeleton->setForces(forces);
    chain.skeleton->computeForwardDynamics();
    benchmark::DoNotOptimize(chain.skeleton->getAccelerations().data());
  }

  state.SetItemsProcessed(state.iterations() * kJointCount);
}
BENCHMARK(BM_Classic_ForwardDynamicsABA);

static void BM_Experimental_ForwardDynamicsABA(benchmark::State& state)
{
  auto chain = createExperimentalChain(kLinkCount);
  chain.world->enterSimulationMode();
  simexp::dynamics::ForwardDynamicsSystem dynamicsSystem;
  Eigen::VectorXd position(1);
  Eigen::VectorXd velocity(1);
  Eigen::VectorXd torque(1);
  double phase = 0.0;

  for (auto _ : state) {
    phase += 0.01;

    for (std::size_t i = 0; i < chain.joints.size(); ++i) {
      position[0] = 0.1 * std::sin(phase + static_cast<double>(i) * 0.2);
      velocity[0] = 0.1 * std::cos(phase + static_cast<double>(i) * 0.2);
      torque[0] = 0.2 * std::sin(phase + static_cast<double>(i) * 0.1);
      chain.joints[i].setPosition(position);
      chain.joints[i].setVelocity(velocity);
      chain.joints[i].setTorque(torque);
    }

    chain.world->updateKinematics();
    dynamicsSystem.compute(*chain.world, chain.multiBody);
    for (const auto& joint : chain.joints) {
      const auto acceleration = joint.getAcceleration();
      benchmark::DoNotOptimize(acceleration.data());
    }
  }

  state.SetItemsProcessed(state.iterations() * kJointCount);
}
BENCHMARK(BM_Experimental_ForwardDynamicsABA);

static void BM_Classic_SingleStep(benchmark::State& state)
{
  auto chain = createClassicChain(kLinkCount);

  for (auto _ : state) {
    chain.world->step();
    benchmark::DoNotOptimize(chain.world->getTime());
  }

  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Classic_SingleStep);

static void BM_Experimental_SingleStep(benchmark::State& state)
{
  auto chain = createExperimentalChain(kLinkCount);
  chain.world->enterSimulationMode();

  for (auto _ : state) {
    chain.world->step();
    benchmark::DoNotOptimize(chain.world->getTime());
  }

  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Experimental_SingleStep);

} // namespace

BENCHMARK_MAIN();
