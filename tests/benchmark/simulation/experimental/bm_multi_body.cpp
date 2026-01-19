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
/// @brief Benchmarks for simulation-experimental module

#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multi_body/link.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/space/state_space.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <benchmark/benchmark.h>

#include <sstream>

namespace exp = dart::simulation::experimental;
using JointType = exp::comps::JointType;

//==============================================================================
// MultiBody Creation Benchmarks
//==============================================================================

/// Benchmark creating N MultiBody instances (each with 1 link)
static void BM_CreateMultiBodies(benchmark::State& state)
{
  const int numBodies = state.range(0);

  for (auto _ : state) {
    exp::World world;
    for (int i = 0; i < numBodies; ++i) {
      auto mb = world.addMultiBody("robot_" + std::to_string(i));
      mb.addLink("base");
    }
    benchmark::DoNotOptimize(world.getMultiBodyCount());
  }

  state.SetItemsProcessed(state.iterations() * numBodies);
}
BENCHMARK(BM_CreateMultiBodies)->Arg(100)->Arg(1000)->Arg(10000);

/// Benchmark creating a single MultiBody with N links in a chain
static void BM_CreateLinkChain(benchmark::State& state)
{
  const int chainLength = state.range(0);

  for (auto _ : state) {
    exp::World world;
    auto robot = world.addMultiBody("robot");
    auto prev = robot.addLink("base");

    for (int i = 1; i < chainLength; ++i) {
      prev = robot.addLink(
          "link_" + std::to_string(i),
          {.parentLink = prev,
           .jointName = "joint_" + std::to_string(i),
           .jointType = JointType::Revolute,
           .axis = {0, 0, 1}});
    }
    benchmark::DoNotOptimize(robot.getLinkCount());
  }

  state.SetItemsProcessed(state.iterations() * chainLength);
}
BENCHMARK(BM_CreateLinkChain)->Arg(10)->Arg(50)->Arg(100)->Arg(500);

/// Benchmark creating a branching tree structure
static void BM_CreateBranchingTree(benchmark::State& state)
{
  const int depth = state.range(0);
  const int branchFactor = 2;

  int totalNodes = (1 << (depth + 1)) - 1; // 2^(depth+1) - 1

  for (auto _ : state) {
    exp::World world;
    auto robot = world.addMultiBody("robot");
    auto root = robot.addLink("root");

    std::vector<exp::Link> currentLevel{root};
    int nodeCount = 1;

    for (int d = 0; d < depth; ++d) {
      std::vector<exp::Link> nextLevel;
      for (const auto& parent : currentLevel) {
        for (int b = 0; b < branchFactor; ++b) {
          auto child = robot.addLink(
              "link_" + std::to_string(nodeCount++),
              {.parentLink = parent,
               .jointName = "joint_" + std::to_string(nodeCount),
               .jointType = JointType::Revolute,
               .axis = {0, 0, 1}});
          nextLevel.push_back(child);
        }
      }
      currentLevel = std::move(nextLevel);
    }
    benchmark::DoNotOptimize(robot.getLinkCount());
  }

  state.SetItemsProcessed(state.iterations() * totalNodes);
}
BENCHMARK(BM_CreateBranchingTree)->Arg(3)->Arg(5)->Arg(7)->Arg(10);

//==============================================================================
// Serialization Benchmarks
//==============================================================================

/// Helper to create a world with N robots, each with a 6-link chain
static exp::World createRobotWorld(int numRobots, int linksPerRobot)
{
  exp::World world;
  for (int r = 0; r < numRobots; ++r) {
    auto robot = world.addMultiBody("robot_" + std::to_string(r));
    auto prev = robot.addLink("base");
    for (int l = 1; l < linksPerRobot; ++l) {
      prev = robot.addLink(
          "link_" + std::to_string(l),
          {.parentLink = prev,
           .jointName = "joint_" + std::to_string(l),
           .jointType = JointType::Revolute,
           .axis = {0, 0, 1}});
    }
  }
  return world;
}

/// Benchmark serializing a world with N robots
static void BM_SerializeWorld(benchmark::State& state)
{
  const int numRobots = state.range(0);
  const int linksPerRobot = 7;

  auto world = createRobotWorld(numRobots, linksPerRobot);

  for (auto _ : state) {
    std::stringstream ss;
    world.saveBinary(ss);
    benchmark::DoNotOptimize(ss.str().size());
  }

  state.SetItemsProcessed(state.iterations() * numRobots);
}
BENCHMARK(BM_SerializeWorld)->Arg(10)->Arg(100)->Arg(1000);

/// Benchmark deserializing a world with N robots
static void BM_DeserializeWorld(benchmark::State& state)
{
  const int numRobots = state.range(0);
  const int linksPerRobot = 7;

  auto world = createRobotWorld(numRobots, linksPerRobot);
  std::stringstream serialized;
  world.saveBinary(serialized);
  std::string data = serialized.str();

  for (auto _ : state) {
    std::stringstream ss(data);
    exp::World loadedWorld;
    loadedWorld.loadBinary(ss);
    benchmark::DoNotOptimize(loadedWorld.getMultiBodyCount());
  }

  state.SetItemsProcessed(state.iterations() * numRobots);
}
BENCHMARK(BM_DeserializeWorld)->Arg(10)->Arg(100)->Arg(1000);

/// Benchmark round-trip (serialize + deserialize) for stress testing
static void BM_SerializeRoundTrip(benchmark::State& state)
{
  const int numRobots = state.range(0);
  const int linksPerRobot = 7;

  auto world = createRobotWorld(numRobots, linksPerRobot);

  for (auto _ : state) {
    std::stringstream ss;
    world.saveBinary(ss);

    exp::World loadedWorld;
    loadedWorld.loadBinary(ss);
    benchmark::DoNotOptimize(loadedWorld.getMultiBodyCount());
  }

  state.SetItemsProcessed(state.iterations() * numRobots);
}
BENCHMARK(BM_SerializeRoundTrip)->Arg(10)->Arg(100)->Arg(1000);

//==============================================================================
// Joint State Access Benchmarks
//==============================================================================

/// Benchmark getting/setting joint positions
static void BM_JointStateAccess(benchmark::State& state)
{
  const int numJoints = state.range(0);

  exp::World world;
  auto robot = world.addMultiBody("robot");
  auto prev = robot.addLink("base");

  std::vector<exp::Joint> joints;
  for (int i = 0; i < numJoints; ++i) {
    prev = robot.addLink(
        "link_" + std::to_string(i),
        {.parentLink = prev,
         .jointName = "joint_" + std::to_string(i),
         .jointType = JointType::Revolute,
         .axis = {0, 0, 1}});
    joints.push_back(prev.getParentJoint().value());
  }

  Eigen::VectorXd pos(1);
  pos << 0.5;

  for (auto _ : state) {
    for (auto& joint : joints) {
      joint.setPosition(pos);
      auto p = joint.getPosition();
      benchmark::DoNotOptimize(p);
    }
  }

  state.SetItemsProcessed(state.iterations() * numJoints * 2);
}
BENCHMARK(BM_JointStateAccess)->Arg(10)->Arg(50)->Arg(100)->Arg(500);

//==============================================================================
// Frame Transform Benchmarks
//==============================================================================

/// Benchmark creating a chain of FreeFrames and accessing transforms
static void BM_FrameChainTransform(benchmark::State& state)
{
  const int chainLength = state.range(0);

  exp::World world;
  std::vector<exp::FreeFrame> frames;

  auto parent = world.addFreeFrame("frame_0");
  frames.push_back(parent);

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translate(Eigen::Vector3d(1, 0, 0));

  for (int i = 1; i < chainLength; ++i) {
    auto frame = world.addFreeFrame("frame_" + std::to_string(i), parent);
    frame.setLocalTransform(offset);
    frames.push_back(frame);
    parent = frame;
  }

  for (auto _ : state) {
    for (auto& frame : frames) {
      auto transform = frame.getTransform();
      benchmark::DoNotOptimize(transform);
    }
  }

  state.SetItemsProcessed(state.iterations() * chainLength);
}
BENCHMARK(BM_FrameChainTransform)->Arg(10)->Arg(50)->Arg(100)->Arg(500);

/// Benchmark setting local transforms and retrieving world transforms
static void BM_FrameTransformUpdate(benchmark::State& state)
{
  const int numFrames = state.range(0);

  exp::World world;
  std::vector<exp::FreeFrame> frames;

  for (int i = 0; i < numFrames; ++i) {
    frames.push_back(world.addFreeFrame("frame_" + std::to_string(i)));
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  double angle = 0.0;

  for (auto _ : state) {
    angle += 0.01;
    transform.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

    for (auto& frame : frames) {
      frame.setLocalTransform(transform);
      auto worldTransform = frame.getTransform();
      benchmark::DoNotOptimize(worldTransform);
    }
  }

  state.SetItemsProcessed(state.iterations() * numFrames);
}
BENCHMARK(BM_FrameTransformUpdate)->Arg(10)->Arg(100)->Arg(1000);

//==============================================================================
// StateSpace Benchmarks
//==============================================================================

/// Benchmark StateSpace creation and variable addition
static void BM_StateSpaceCreate(benchmark::State& state)
{
  const int numVariables = state.range(0);

  for (auto _ : state) {
    exp::StateSpace space;
    for (int i = 0; i < numVariables; ++i) {
      space.addVariable("var_" + std::to_string(i), 6, -3.14, 3.14);
    }
    space.finalize();
    benchmark::DoNotOptimize(space.getDimension());
  }

  state.SetItemsProcessed(state.iterations() * numVariables);
}
BENCHMARK(BM_StateSpaceCreate)->Arg(10)->Arg(100)->Arg(1000);

/// Benchmark StateSpace bounds retrieval
static void BM_StateSpaceGetBounds(benchmark::State& state)
{
  const int numVariables = state.range(0);

  exp::StateSpace space;
  for (int i = 0; i < numVariables; ++i) {
    space.addVariable("var_" + std::to_string(i), 6, -3.14, 3.14);
  }
  space.finalize();

  for (auto _ : state) {
    auto lower = space.getLowerBounds();
    auto upper = space.getUpperBounds();
    benchmark::DoNotOptimize(lower);
    benchmark::DoNotOptimize(upper);
  }

  state.SetItemsProcessed(state.iterations() * 2);
}
BENCHMARK(BM_StateSpaceGetBounds)->Arg(10)->Arg(100)->Arg(1000);

/// Benchmark StateSpace variable lookup by name
static void BM_StateSpaceVariableLookup(benchmark::State& state)
{
  const int numVariables = state.range(0);

  exp::StateSpace space;
  std::vector<std::string> names;
  for (int i = 0; i < numVariables; ++i) {
    std::string name = "var_" + std::to_string(i);
    names.push_back(name);
    space.addVariable(name, 6, -3.14, 3.14);
  }
  space.finalize();

  for (auto _ : state) {
    for (const auto& name : names) {
      auto var = space.getVariable(name);
      benchmark::DoNotOptimize(var);
    }
  }

  state.SetItemsProcessed(state.iterations() * numVariables);
}
BENCHMARK(BM_StateSpaceVariableLookup)->Arg(10)->Arg(100)->Arg(1000);

BENCHMARK_MAIN();
