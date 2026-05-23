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

#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/compute_graph_visualization.hpp>
#include <dart/simulation/experimental/compute/compute_node.hpp>
#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/compute/parallel_executor.hpp>
#include <dart/simulation/experimental/compute/rigid_body_integration_kernel.hpp>
#include <dart/simulation/experimental/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace compute = dart::simulation::experimental::compute;
namespace sx = dart::simulation::experimental;

//==============================================================================
TEST(ExperimentalComputeNode, ExecutesCallable)
{
  int counter = 0;
  compute::ComputeNode node("increment", [&counter]() { ++counter; });

  EXPECT_EQ(node.getName(), "increment");
  EXPECT_TRUE(node.isValid());

  node.execute();
  node.execute();

  EXPECT_EQ(counter, 2);
}

//==============================================================================
TEST(ExperimentalComputeNode, EmptyCallableThrows)
{
  compute::ComputeNode node("empty", nullptr);

  EXPECT_FALSE(node.isValid());
  EXPECT_THROW(node.execute(), sx::InvalidOperationException);
}

//==============================================================================
TEST(ExperimentalComputeStageMetadata, SupportsMultipleDomainsAndAccelerators)
{
  const compute::ComputeStageMetadata articulated{
      compute::ComputeStageDomain::ArticulatedBody,
      compute::ComputeStageAcceleration::TaskParallel
          | compute::ComputeStageAcceleration::DataLocality};
  const compute::ComputeStageMetadata deformable{
      compute::ComputeStageDomain::DeformableBody,
      compute::ComputeStageAcceleration::DataParallel
          | compute::ComputeStageAcceleration::Simd};
  const compute::ComputeStageMetadata fluid{
      compute::ComputeStageDomain::Fluid,
      compute::ComputeStageAcceleration::DataParallel
          | compute::ComputeStageAcceleration::Gpu};
  const compute::ComputeStageMetadata rendering{
      compute::ComputeStageDomain::Rendering,
      compute::ComputeStageAcceleration::TaskParallel
          | compute::ComputeStageAcceleration::Gpu};

  EXPECT_EQ(compute::toString(articulated.domain), "articulated_body");
  EXPECT_TRUE(
      compute::hasAcceleration(
          articulated.acceleration,
          compute::ComputeStageAcceleration::TaskParallel));
  EXPECT_TRUE(
      compute::hasAcceleration(
          articulated.acceleration,
          compute::ComputeStageAcceleration::DataLocality));
  EXPECT_EQ(compute::toString(deformable.domain), "deformable_body");
  EXPECT_TRUE(
      compute::hasAcceleration(
          deformable.acceleration, compute::ComputeStageAcceleration::Simd));
  EXPECT_EQ(compute::toString(fluid.domain), "fluid");
  EXPECT_TRUE(
      compute::hasAcceleration(
          fluid.acceleration, compute::ComputeStageAcceleration::Gpu));
  EXPECT_EQ(compute::toString(rendering.domain), "rendering");
  EXPECT_NE(
      compute::formatAccelerationMask(rendering.acceleration).find("gpu"),
      std::string::npos);
}

//==============================================================================
TEST(ExperimentalComputeGraph, EmptyGraphIsValid)
{
  compute::ComputeGraph graph;

  EXPECT_TRUE(graph.isEmpty());
  EXPECT_EQ(graph.getNodeCount(), 0u);
  EXPECT_EQ(graph.getEdgeCount(), 0u);
  EXPECT_TRUE(graph.getParallelLevels().empty());
  EXPECT_TRUE(graph.validate());
}

//==============================================================================
TEST(ExperimentalComputeGraph, RejectsDuplicateNodeNames)
{
  compute::ComputeGraph graph;
  graph.addNode("same", []() {});

  EXPECT_THROW(graph.addNode("same", []() {}), sx::InvalidArgumentException);
}

//==============================================================================
TEST(ExperimentalComputeGraph, RejectsEmptyNodeNames)
{
  compute::ComputeGraph graph;

  EXPECT_THROW(graph.addNode("", []() {}), sx::InvalidArgumentException);
}

//==============================================================================
TEST(ExperimentalComputeGraph, TracksDependenciesAndDependents)
{
  compute::ComputeGraph graph;
  auto& start = graph.addNode("start", []() {});
  auto& middle = graph.addNode("middle", []() {});
  auto& end = graph.addNode("end", []() {});

  graph.addDependency(start, middle);
  graph.addDependency(middle, end);

  auto dependencies = graph.getDependencies(end);
  ASSERT_EQ(dependencies.size(), 1u);
  EXPECT_EQ(dependencies.front(), &middle);

  auto dependents = graph.getDependents(start);
  ASSERT_EQ(dependents.size(), 1u);
  EXPECT_EQ(dependents.front(), &middle);
}

//==============================================================================
TEST(ExperimentalComputeGraph, DuplicateDependencyIsNoOp)
{
  compute::ComputeGraph graph;
  auto& start = graph.addNode("start", []() {});
  auto& end = graph.addNode("end", []() {});

  graph.addDependency(start, end);
  graph.addDependency(start, end);

  EXPECT_EQ(graph.getEdgeCount(), 1u);
}

//==============================================================================
TEST(ExperimentalComputeGraph, RejectsExternalDependencies)
{
  compute::ComputeGraph graph1;
  compute::ComputeGraph graph2;

  auto& start = graph1.addNode("start", []() {});
  auto& external = graph2.addNode("external", []() {});

  EXPECT_THROW(
      graph1.addDependency(start, external), sx::InvalidArgumentException);
}

//==============================================================================
TEST(ExperimentalComputeGraph, RejectsCycles)
{
  compute::ComputeGraph graph;
  auto& a = graph.addNode("a", []() {});
  auto& b = graph.addNode("b", []() {});
  auto& c = graph.addNode("c", []() {});

  graph.addDependency(a, b);
  graph.addDependency(b, c);

  EXPECT_THROW(graph.addDependency(c, a), sx::InvalidOperationException);
}

//==============================================================================
TEST(ExperimentalComputeGraph, TopologicalOrderUsesConstructionOrderTieBreaker)
{
  compute::ComputeGraph graph;
  auto& a = graph.addNode("a", []() {});
  auto& b = graph.addNode("b", []() {});
  auto& c = graph.addNode("c", []() {});
  auto& d = graph.addNode("d", []() {});

  graph.addDependency(a, d);
  graph.addDependency(b, d);
  graph.addDependency(c, d);

  auto order = graph.getTopologicalOrder();
  ASSERT_EQ(order.size(), 4u);
  EXPECT_EQ(order[0]->getName(), "a");
  EXPECT_EQ(order[1]->getName(), "b");
  EXPECT_EQ(order[2]->getName(), "c");
  EXPECT_EQ(order[3]->getName(), "d");
}

//==============================================================================
TEST(ExperimentalComputeGraph, ReportsStaticParallelLevels)
{
  compute::ComputeGraph graph;
  auto& start = graph.addNode("start", []() {});
  auto& left = graph.addNode("left", []() {});
  auto& right = graph.addNode("right", []() {});
  auto& end = graph.addNode("end", []() {});

  graph.addDependency(start, left);
  graph.addDependency(start, right);
  graph.addDependency(left, end);
  graph.addDependency(right, end);

  const auto levels = graph.getParallelLevels();
  ASSERT_EQ(levels.size(), 3u);
  ASSERT_EQ(levels[0].size(), 1u);
  ASSERT_EQ(levels[1].size(), 2u);
  ASSERT_EQ(levels[2].size(), 1u);
  EXPECT_EQ(levels[0][0], &start);
  EXPECT_EQ(levels[1][0], &left);
  EXPECT_EQ(levels[1][1], &right);
  EXPECT_EQ(levels[2][0], &end);
}

//==============================================================================
TEST(ExperimentalComputeGraph, ExportsDotWithMetadataAndProfile)
{
  using namespace std::chrono_literals;

  compute::ComputeGraph graph;
  auto& simulate = graph.addNode(
      "simulate",
      []() { std::this_thread::sleep_for(1ms); },
      {compute::ComputeStageDomain::RigidBody,
       compute::ComputeStageAcceleration::TaskParallel
           | compute::ComputeStageAcceleration::DataLocality});
  auto& render = graph.addNode(
      "render",
      []() { std::this_thread::sleep_for(1ms); },
      {compute::ComputeStageDomain::Rendering,
       compute::ComputeStageAcceleration::TaskParallel
           | compute::ComputeStageAcceleration::Gpu});
  graph.addDependency(simulate, render);

  compute::SequentialExecutor executor;
  const auto profile = executor.executeProfiled(graph);
  const auto dot = compute::toDot(graph, &profile);

  EXPECT_NE(dot.find("digraph ComputeGraph"), std::string::npos);
  EXPECT_NE(dot.find("simulate"), std::string::npos);
  EXPECT_NE(dot.find("render"), std::string::npos);
  EXPECT_NE(dot.find("domain=rigid_body"), std::string::npos);
  EXPECT_NE(dot.find("domain=rendering"), std::string::npos);
  EXPECT_NE(dot.find("gpu"), std::string::npos);
  EXPECT_NE(dot.find("duration_us="), std::string::npos);
  EXPECT_NE(dot.find("rank=same"), std::string::npos);
}

//==============================================================================
TEST(ExperimentalSequentialExecutor, ExecutesInDependencyOrder)
{
  compute::ComputeGraph graph;
  std::vector<std::string> order;

  auto& start = graph.addNode("start", [&]() { order.push_back("start"); });
  auto& left = graph.addNode("left", [&]() { order.push_back("left"); });
  auto& right = graph.addNode("right", [&]() { order.push_back("right"); });
  auto& end = graph.addNode("end", [&]() { order.push_back("end"); });

  graph.addDependency(start, left);
  graph.addDependency(start, right);
  graph.addDependency(left, end);
  graph.addDependency(right, end);

  compute::SequentialExecutor executor;
  executor.execute(graph);

  ASSERT_EQ(order.size(), 4u);
  EXPECT_EQ(order[0], "start");
  EXPECT_EQ(order[1], "left");
  EXPECT_EQ(order[2], "right");
  EXPECT_EQ(order[3], "end");
  EXPECT_EQ(executor.getWorkerCount(), 1u);
}

//==============================================================================
TEST(ExperimentalSequentialExecutor, ProfilesNodeLoadAndParallelism)
{
  using namespace std::chrono_literals;

  compute::ComputeGraph graph;
  auto& light = graph.addNode("light", []() {});
  auto& heavy
      = graph.addNode("heavy", []() { std::this_thread::sleep_for(20ms); });
  graph.addDependency(light, heavy);

  compute::SequentialExecutor executor;
  auto profile = executor.executeProfiled(graph);

  ASSERT_EQ(profile.nodes.size(), 2u);
  EXPECT_FALSE(profile.isEmpty());
  EXPECT_EQ(profile.workerCount, 1u);
  EXPECT_EQ(profile.maxParallelism, 1u);
  EXPECT_GT(profile.wallTime.count(), 0);
  EXPECT_GT(profile.totalNodeTime.count(), 0);
  EXPECT_GT(profile.criticalPathTime.count(), 0);
  EXPECT_GT(profile.getAverageParallelism(), 0.0);

  const auto* lightProfile = profile.getNode("light");
  const auto* heavyProfile = profile.getNode("heavy");
  ASSERT_NE(lightProfile, nullptr);
  ASSERT_NE(heavyProfile, nullptr);
  EXPECT_EQ(profile.getNode("missing"), nullptr);

  EXPECT_EQ(lightProfile->dependencyCount, 0u);
  EXPECT_EQ(lightProfile->dependentCount, 1u);
  EXPECT_EQ(lightProfile->level, 0u);
  EXPECT_EQ(lightProfile->workerIndex, 0u);
  EXPECT_EQ(heavyProfile->dependencyCount, 1u);
  EXPECT_EQ(heavyProfile->dependentCount, 0u);
  EXPECT_EQ(heavyProfile->level, 1u);
  EXPECT_EQ(heavyProfile->workerIndex, 0u);
  EXPECT_GT(heavyProfile->duration, lightProfile->duration);
  EXPECT_GE(heavyProfile->startTime, lightProfile->endTime);
}

//==============================================================================
TEST(ExperimentalParallelExecutor, RespectsDependencies)
{
  compute::ComputeGraph graph;
  std::atomic<bool> ok{true};
  std::atomic<bool> startDone{false};
  std::atomic<int> workersDone{0};
  std::atomic<bool> endDone{false};

  auto& start = graph.addNode("start", [&]() { startDone.store(true); });

  auto addWorker = [&](std::string name) -> compute::ComputeNode& {
    return graph.addNode(name, [&]() {
      if (!startDone.load()) {
        ok.store(false);
      }
      workersDone.fetch_add(1);
    });
  };

  auto& left = addWorker("left");
  auto& right = addWorker("right");
  auto& end = graph.addNode("end", [&]() {
    if (workersDone.load() != 2) {
      ok.store(false);
    }
    endDone.store(true);
  });

  graph.addDependency(start, left);
  graph.addDependency(start, right);
  graph.addDependency(left, end);
  graph.addDependency(right, end);

  compute::ParallelExecutor executor(2);
  executor.execute(graph);

  EXPECT_TRUE(ok.load());
  EXPECT_TRUE(endDone.load());
  EXPECT_GE(executor.getWorkerCount(), 1u);
}

//==============================================================================
TEST(ExperimentalParallelExecutor, PropagatesNodeExceptions)
{
  compute::ComputeGraph graph;
  graph.addNode("throwing", []() { throw std::runtime_error("task failed"); });

  compute::ParallelExecutor executor(1);

  EXPECT_THROW(executor.execute(graph), std::runtime_error);
}

//==============================================================================
TEST(ExperimentalParallelExecutor, ProfiledPropagatesNodeExceptions)
{
  compute::ComputeGraph graph;
  graph.addNode("throwing", []() { throw std::runtime_error("task failed"); });

  compute::ParallelExecutor executor(1);

  EXPECT_THROW({ (void)executor.executeProfiled(graph); }, std::runtime_error);
}

//==============================================================================
TEST(ExperimentalParallelExecutor, ProfileReportsObservedParallelism)
{
  using namespace std::chrono_literals;

  compute::ComputeGraph graph;
  std::atomic<int> active{0};
  std::atomic<int> maxActive{0};

  auto updateMax = [&](int value) {
    auto observed = maxActive.load();
    while (observed < value
           && !maxActive.compare_exchange_weak(observed, value)) {
    }
  };

  auto parallelWork = [&]() {
    const auto current = active.fetch_add(1) + 1;
    updateMax(current);
    std::this_thread::sleep_for(20ms);
    active.fetch_sub(1);
  };

  auto& start = graph.addNode("start", []() {});
  auto& left = graph.addNode("left", parallelWork);
  auto& right = graph.addNode("right", parallelWork);
  auto& end = graph.addNode("end", []() {});

  graph.addDependency(start, left);
  graph.addDependency(start, right);
  graph.addDependency(left, end);
  graph.addDependency(right, end);

  compute::ParallelExecutor executor(2);
  auto profile = executor.executeProfiled(graph);

  ASSERT_EQ(profile.nodes.size(), 4u);
  EXPECT_GE(profile.workerCount, 2u);
  EXPECT_GE(maxActive.load(), 2);
  EXPECT_GE(profile.maxParallelism, 2u);
  EXPECT_GT(profile.getAverageParallelism(), 1.0);

  const auto* leftProfile = profile.getNode("left");
  const auto* rightProfile = profile.getNode("right");
  ASSERT_NE(leftProfile, nullptr);
  ASSERT_NE(rightProfile, nullptr);
  EXPECT_EQ(leftProfile->level, 1u);
  EXPECT_EQ(rightProfile->level, 1u);
  EXPECT_LT(leftProfile->workerIndex, profile.workerCount);
  EXPECT_LT(rightProfile->workerIndex, profile.workerCount);
}

namespace {

//==============================================================================
compute::ComputeStageMetadata accessMeta(
    std::string resource, compute::ComputeAccessMode mode)
{
  compute::ComputeStageMetadata metadata;
  metadata.resources.push_back({std::move(resource), mode});
  return metadata;
}

} // namespace

//==============================================================================
TEST(ExperimentalComputeAccess, ToStringCoversModes)
{
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Read), "read");
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Write), "write");
  EXPECT_EQ(
      compute::toString(compute::ComputeAccessMode::ReadWrite), "read_write");
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Reduce), "reduce");
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Scratch), "scratch");
}

//==============================================================================
TEST(ExperimentalComputeAccess, ConflictRules)
{
  using compute::accessesConflict;
  using Mode = compute::ComputeAccessMode;

  EXPECT_FALSE(accessesConflict(Mode::Read, Mode::Read));
  EXPECT_TRUE(accessesConflict(Mode::Read, Mode::Write));
  EXPECT_TRUE(accessesConflict(Mode::Write, Mode::Write));
  EXPECT_TRUE(accessesConflict(Mode::ReadWrite, Mode::Read));
  EXPECT_FALSE(accessesConflict(Mode::Reduce, Mode::Reduce));
  EXPECT_TRUE(accessesConflict(Mode::Reduce, Mode::Write));
  EXPECT_FALSE(accessesConflict(Mode::Scratch, Mode::Write));
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, ReadReadIsSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Read));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Read));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, DisjointWritesAreSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  graph.addNode(
      "b", []() {}, accessMeta("y", compute::ComputeAccessMode::Write));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, UnorderedWriteWriteIsHazard)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));

  const auto hazards = graph.findResourceHazards();
  ASSERT_EQ(hazards.size(), 1u);
  EXPECT_EQ(hazards.front().resource, "x");
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, OrderedWriteWriteIsSafe)
{
  compute::ComputeGraph graph;
  auto& a = graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  auto& b = graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  graph.addDependency(a, b);
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, DeclaredReductionIsSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Reduce));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Reduce));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, ScratchIsSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Scratch));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, PerEntityTreeHasNoFalsePositives)
{
  // Mirrors the kinematics graph: a parent writes its own cache; two children
  // read the parent cache and write their own. Per-entity resource ids keep
  // independent siblings hazard-free.
  compute::ComputeGraph graph;

  compute::ComputeStageMetadata parentMeta;
  parentMeta.resources.push_back(
      {"cache#0", compute::ComputeAccessMode::Write});
  compute::ComputeStageMetadata child1Meta;
  child1Meta.resources.push_back({"cache#0", compute::ComputeAccessMode::Read});
  child1Meta.resources.push_back(
      {"cache#1", compute::ComputeAccessMode::Write});
  compute::ComputeStageMetadata child2Meta;
  child2Meta.resources.push_back({"cache#0", compute::ComputeAccessMode::Read});
  child2Meta.resources.push_back(
      {"cache#2", compute::ComputeAccessMode::Write});

  auto& parent = graph.addNode("parent", []() {}, parentMeta);
  auto& child1 = graph.addNode("child1", []() {}, child1Meta);
  auto& child2 = graph.addNode("child2", []() {}, child2Meta);
  graph.addDependency(parent, child1);
  graph.addDependency(parent, child2);

  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(ExperimentalComputeResourceHazards, CoarseSharedWriteIsFlagged)
{
  // Negative control: collapsing per-entity ids to one coarse id surfaces the
  // sibling write/write conflict that per-entity ids avoid.
  compute::ComputeGraph graph;

  compute::ComputeStageMetadata writeCache;
  writeCache.resources.push_back({"cache", compute::ComputeAccessMode::Write});

  auto& parent = graph.addNode("parent", []() {}, writeCache);
  auto& child1 = graph.addNode("child1", []() {}, writeCache);
  auto& child2 = graph.addNode("child2", []() {}, writeCache);
  graph.addDependency(parent, child1);
  graph.addDependency(parent, child2);

  const auto hazards = graph.findResourceHazards();
  ASSERT_EQ(hazards.size(), 1u);
  EXPECT_EQ(hazards.front().resource, "cache");
}

//==============================================================================
TEST(ExperimentalComputeGraphDot, IncludesResourceAccess)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a",
      []() {},
      accessMeta("comps::Transform", compute::ComputeAccessMode::ReadWrite));

  const auto dot = compute::toDot(graph);
  EXPECT_NE(dot.find("read_write comps::Transform"), std::string::npos);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, SemiImplicitPositionUpdate)
{
  std::vector<double> positions{1.0, 2.0, 3.0, 4.0};
  const std::vector<double> velocities{0.5, -1.0, 2.0, 0.0};

  compute::integratePositionsSemiImplicit(
      positions.data(), velocities.data(), 0.1, positions.size());

  EXPECT_DOUBLE_EQ(positions[0], 1.05);
  EXPECT_DOUBLE_EQ(positions[1], 1.9);
  EXPECT_DOUBLE_EQ(positions[2], 3.2);
  EXPECT_DOUBLE_EQ(positions[3], 4.0);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, IsScalarGeneric)
{
  // Instantiating the same kernel for float proves it is not bound to double,
  // keeping the autodiff/SIMD scalar door open.
  std::vector<float> positions{1.0F, 2.0F};
  const std::vector<float> velocities{2.0F, 4.0F};

  compute::integratePositionsSemiImplicit(
      positions.data(), velocities.data(), 0.5F, positions.size());

  EXPECT_FLOAT_EQ(positions[0], 2.0F);
  EXPECT_FLOAT_EQ(positions[1], 4.0F);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, IntegratesStateBatchPositions)
{
  compute::RigidBodyStateBatch batch;
  batch.worldCount = 1;
  batch.bodyCount = 1;
  batch.position = {0.0, 0.0, 0.0};
  batch.orientation = {1.0, 0.0, 0.0, 0.0};
  batch.linearVelocity = {1.0, 2.0, 3.0};
  batch.angularVelocity = {0.0, 0.0, 0.0};

  compute::integratePositionsSemiImplicit(
      batch.position.data(),
      batch.linearVelocity.data(),
      0.5,
      batch.position.size());

  EXPECT_DOUBLE_EQ(batch.position[0], 0.5);
  EXPECT_DOUBLE_EQ(batch.position[1], 1.0);
  EXPECT_DOUBLE_EQ(batch.position[2], 1.5);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, IntegratesStateBatchLinearStep)
{
  compute::RigidBodyStateBatch state;
  state.worldCount = 1;
  state.bodyCount = 2;
  state.position = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  state.orientation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  state.linearVelocity = {1.0, 0.0, 0.0, 0.0, 2.0, 0.0};
  state.angularVelocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  const std::vector<double> force = {0.0, 0.0, 10.0, 0.0, 0.0, 0.0};
  const std::vector<double> inverseMass = {0.5, 1.0};

  compute::integrateRigidBodyStateBatchLinear(state, force, inverseMass, 0.1);

  // Velocity updated first: body 0 z += 10 * 0.5 * 0.1 = 0.5; body 1 unchanged.
  EXPECT_DOUBLE_EQ(state.linearVelocity[2], 0.5);
  // Position then uses the updated velocity (semi-implicit Euler).
  EXPECT_DOUBLE_EQ(state.position[0], 0.1);  // body 0 x: 0 + 1.0 * 0.1
  EXPECT_DOUBLE_EQ(state.position[2], 0.05); // body 0 z: 0 + 0.5 * 0.1
  EXPECT_DOUBLE_EQ(state.position[4], 1.2);  // body 1 y: 1 + 2.0 * 0.1

  // Size mismatch is rejected.
  const std::vector<double> badForce = {0.0, 0.0, 0.0};
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatchLinear(
          state, badForce, inverseMass, 0.1),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, IntegratesOrientationAboutZ)
{
  std::vector<double> orientation = {1.0, 0.0, 0.0, 0.0}; // identity
  const std::vector<double> angular = {0.0, 0.0, 2.0};    // omega about +z

  compute::integrateOrientationsSemiImplicit(
      orientation.data(), angular.data(), 0.1, 1);

  const double w = orientation[0];
  const double x = orientation[1];
  const double y = orientation[2];
  const double z = orientation[3];

  // Remains a unit quaternion.
  EXPECT_NEAR(w * w + x * x + y * y + z * z, 1.0, 1e-12);
  // Pure z-rotation keeps the x and y components zero.
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  // tan(theta/2) = z/w = 0.5 * omega_z * dt = 0.1 (ratio preserved by
  // normalization).
  EXPECT_NEAR(z / w, 0.1, 1e-12);
  EXPECT_GT(w, 0.0);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, OrientationKernelIsScalarGeneric)
{
  std::vector<float> orientation = {1.0F, 0.0F, 0.0F, 0.0F};
  const std::vector<float> angular = {0.0F, 0.0F, 2.0F};

  compute::integrateOrientationsSemiImplicit(
      orientation.data(), angular.data(), 0.1F, 1);

  const float norm
      = orientation[0] * orientation[0] + orientation[1] * orientation[1]
        + orientation[2] * orientation[2] + orientation[3] * orientation[3];
  EXPECT_NEAR(norm, 1.0F, 1e-5F);
}

//==============================================================================
TEST(ExperimentalIntegrationKernel, IntegratesFullStateBatch)
{
  compute::RigidBodyStateBatch state;
  state.worldCount = 1;
  state.bodyCount = 1;
  state.position = {0.0, 0.0, 0.0};
  state.orientation = {1.0, 0.0, 0.0, 0.0};
  state.linearVelocity = {1.0, 0.0, 0.0};
  state.angularVelocity = {0.0, 0.0, 2.0};

  compute::RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = 1;
  model.inverseMass = {1.0};

  const std::vector<double> force = {0.0, 0.0, 4.0};

  compute::integrateRigidBodyStateBatch(state, model, force, 0.1);

  // Linear: vel.z += 4 * 1 * 0.1 = 0.4; pos += updated vel * 0.1.
  EXPECT_DOUBLE_EQ(state.linearVelocity[2], 0.4);
  EXPECT_DOUBLE_EQ(state.position[0], 0.1);
  EXPECT_DOUBLE_EQ(state.position[2], 0.04);
  // Orientation: z-rotation, x and y stay zero, unit norm, tan(theta/2)=0.1.
  EXPECT_DOUBLE_EQ(state.orientation[1], 0.0);
  EXPECT_DOUBLE_EQ(state.orientation[2], 0.0);
  const double w = state.orientation[0];
  const double z = state.orientation[3];
  EXPECT_NEAR(w * w + z * z, 1.0, 1e-12);
  EXPECT_NEAR(z / w, 0.1, 1e-12);

  // Orientation array of the wrong size is rejected.
  auto bad = state;
  bad.orientation = {1.0, 0.0, 0.0};
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatch(bad, model, force, 0.1),
      sx::InvalidArgumentException);
}
