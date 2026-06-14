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

#include <dart/config.hpp>

#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/compute/compute_graph.hpp>
#include <dart/simulation/compute/compute_graph_visualization.hpp>
#include <dart/simulation/compute/compute_node.hpp>
#include <dart/simulation/compute/compute_stage_metadata.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/rigid_body_integration_kernel.hpp>
#include <dart/simulation/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>

#include <dart/common/memory_manager.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <cmath>
#include <cstdlib>

namespace compute = dart::simulation::compute;
namespace sx = dart::simulation;

namespace common = dart::common;

namespace {

std::atomic<bool> g_heapAllocationTrackingEnabled{false};
std::atomic<std::size_t> g_heapAllocationCount{0};
std::atomic<std::size_t> g_heapAllocationBytes{0};

void recordHeapAllocation(std::size_t bytes) noexcept
{
  if (g_heapAllocationTrackingEnabled.load(std::memory_order_relaxed)) {
    g_heapAllocationCount.fetch_add(1, std::memory_order_relaxed);
    g_heapAllocationBytes.fetch_add(bytes, std::memory_order_relaxed);
  }
}

[[nodiscard]] void* allocateRaw(std::size_t bytes) noexcept
{
  return std::malloc(bytes == 0 ? 1 : bytes);
}

[[nodiscard]] void* allocateAlignedRaw(
    std::size_t bytes, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    return allocateRaw(bytes);
  }

#if defined(_WIN32)
  return _aligned_malloc(bytes == 0 ? 1 : bytes, alignment);
#else
  const auto requested = bytes == 0 ? 1 : bytes;
  if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
    return nullptr;
  }
  if (alignment < alignof(void*)) {
    alignment = alignof(void*);
  }
  void* pointer = nullptr;
  return posix_memalign(&pointer, alignment, requested) == 0 ? pointer
                                                             : nullptr;
#endif
}

void deallocateAlignedRaw(void* pointer, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    std::free(pointer);
    return;
  }

#if defined(_WIN32)
  _aligned_free(pointer);
#else
  std::free(pointer);
#endif
}

} // namespace

void* operator new(std::size_t bytes)
{
  recordHeapAllocation(bytes);
  if (auto* ptr = allocateRaw(bytes)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes)
{
  recordHeapAllocation(bytes);
  if (auto* ptr = allocateRaw(bytes)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateRaw(bytes);
}

void* operator new[](std::size_t bytes, const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateRaw(bytes);
}

void* operator new(std::size_t bytes, std::align_val_t alignment)
{
  recordHeapAllocation(bytes);
  if (auto* ptr
      = allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment))) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes, std::align_val_t alignment)
{
  recordHeapAllocation(bytes);
  if (auto* ptr
      = allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment))) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new(
    std::size_t bytes,
    std::align_val_t alignment,
    const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment));
}

void* operator new[](
    std::size_t bytes,
    std::align_val_t alignment,
    const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment));
}

void operator delete(void* pointer) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, const std::nothrow_t&) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer, const std::nothrow_t&) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](void* pointer, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete(
    void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](
    void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete(
    void* pointer, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](
    void* pointer, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

namespace {

class ScopedHeapAllocationCounter final
{
public:
  ScopedHeapAllocationCounter()
  {
#ifdef DART_CODECOV
    g_heapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
#else
    g_heapAllocationCount.store(0, std::memory_order_relaxed);
    g_heapAllocationBytes.store(0, std::memory_order_relaxed);
    g_heapAllocationTrackingEnabled.store(true, std::memory_order_relaxed);
#endif
  }

  ~ScopedHeapAllocationCounter()
  {
    stop();
  }

  ScopedHeapAllocationCounter(const ScopedHeapAllocationCounter&) = delete;
  ScopedHeapAllocationCounter& operator=(const ScopedHeapAllocationCounter&)
      = delete;

  void stop() noexcept
  {
    g_heapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t allocationCount() const noexcept
  {
#ifdef DART_CODECOV
    return 0;
#else
    return g_heapAllocationCount.load(std::memory_order_relaxed);
#endif
  }

  [[nodiscard]] std::size_t allocationBytes() const noexcept
  {
#ifdef DART_CODECOV
    return 0;
#else
    return g_heapAllocationBytes.load(std::memory_order_relaxed);
#endif
  }
};

} // namespace

//==============================================================================
TEST(SimulationComputeNode, ExecutesCallable)
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
TEST(SimulationComputeNode, EmptyCallableThrows)
{
  compute::ComputeNode node("empty", nullptr);

  EXPECT_FALSE(node.isValid());
  EXPECT_THROW(node.execute(), sx::InvalidOperationException);
}

//==============================================================================
TEST(SimulationComputeStageMetadata, SupportsMultipleDomainsAndAccelerators)
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
TEST(SimulationComputeStageMetadata, ResourceStorageUsesProvidedAllocator)
{
  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeMetadata = freeList.getAllocationCount();

  {
    ScopedHeapAllocationCounter heapCounter;
    compute::ComputeStageMetadata metadata{
        compute::ComputeStageDomain::Kinematics,
        compute::toMask(compute::ComputeStageAcceleration::TaskParallel),
        memoryManager.getFreeAllocator()};
    metadata.resources.push_back(
        {"comps::FrameCache#123456789", compute::ComputeAccessMode::Write});
    metadata.resources.push_back(
        compute::ComputeResourceAccess{
            "comps::FrameCache#987654321",
            compute::ComputeAccessMode::Read,
            memoryManager.getFreeAllocator()});
    heapCounter.stop();

    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "allocator-aware compute metadata should not allocate resource "
           "storage from the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeMetadata);
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeMetadata);
}

//==============================================================================
TEST(SimulationComputeGraph, EmptyGraphIsValid)
{
  compute::ComputeGraph graph;

  EXPECT_TRUE(graph.isEmpty());
  EXPECT_EQ(graph.getNodeCount(), 0u);
  EXPECT_EQ(graph.getEdgeCount(), 0u);
  EXPECT_TRUE(graph.getParallelLevels().empty());
  EXPECT_TRUE(graph.validate());
}

//==============================================================================
TEST(SimulationComputeGraph, AllocatorAwareGraphUsesProvidedAllocator)
{
  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeGraph = freeList.getAllocationCount();

  {
    compute::ComputeGraph graph(memoryManager.getFreeAllocator());
    auto& start = graph.addNode("allocator_start", []() {});
    auto& end = graph.addNode("allocator_end", []() {});
    const auto allocationsAfterNodes = freeList.getAllocationCount();

    graph.addDependency(start, end);
    EXPECT_GT(freeList.getAllocationCount(), allocationsAfterNodes)
        << "allocator-aware ComputeGraph should allocate edge storage from "
           "the provided allocator";

    const auto allocationsAfterEdges = freeList.getAllocationCount();
    const auto order = graph.getTopologicalOrderView();
    ASSERT_EQ(order.size(), 2u);
    EXPECT_EQ(order[0], &start);
    EXPECT_EQ(order[1], &end);
    EXPECT_GT(freeList.getAllocationCount(), allocationsAfterEdges)
        << "allocator-aware ComputeGraph should allocate topological-order "
           "cache storage from the provided allocator";

    EXPECT_EQ(graph.getNode("allocator_start"), &start);
    EXPECT_EQ(graph.getNode("allocator_end"), &end);
    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeGraph)
        << "allocator-aware ComputeGraph should allocate owned node, name "
           "lookup, edge, and topological-order storage from the provided "
           "allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeGraph);
}

//==============================================================================
TEST(SimulationComputeGraph, AllocatorAwareTraversalAvoidsGlobalHeap)
{
  common::MemoryManager memoryManager;
  compute::ComputeGraph graph(memoryManager.getFreeAllocator());
  auto& start = graph.addNode("heap_start", []() {});
  auto& middle = graph.addNode("heap_middle", []() {});
  auto& end = graph.addNode("heap_end", []() {});

  ScopedHeapAllocationCounter heapCounter;
  graph.addDependency(start, middle);
  graph.addDependency(middle, end);
  const auto order = graph.getTopologicalOrderView();
  ASSERT_EQ(order.size(), 3u);
  EXPECT_TRUE(graph.validate());
  EXPECT_TRUE(graph.findResourceHazards().empty());
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated during allocator-aware graph traversal: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

//==============================================================================
TEST(SimulationComputeGraph, RejectsDuplicateNodeNames)
{
  compute::ComputeGraph graph;
  graph.addNode("same", []() {});

  EXPECT_THROW(graph.addNode("same", []() {}), sx::InvalidArgumentException);
}

//==============================================================================
TEST(SimulationComputeGraph, RejectsEmptyNodeNames)
{
  compute::ComputeGraph graph;

  EXPECT_THROW(graph.addNode("", []() {}), sx::InvalidArgumentException);
}

//==============================================================================
TEST(SimulationComputeGraph, TracksDependenciesAndDependents)
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
TEST(SimulationComputeGraph, DuplicateDependencyIsNoOp)
{
  compute::ComputeGraph graph;
  auto& start = graph.addNode("start", []() {});
  auto& end = graph.addNode("end", []() {});

  graph.addDependency(start, end);
  graph.addDependency(start, end);

  EXPECT_EQ(graph.getEdgeCount(), 1u);
}

//==============================================================================
TEST(SimulationComputeGraph, RejectsExternalDependencies)
{
  compute::ComputeGraph graph1;
  compute::ComputeGraph graph2;

  auto& start = graph1.addNode("start", []() {});
  auto& external = graph2.addNode("external", []() {});

  EXPECT_THROW(
      graph1.addDependency(start, external), sx::InvalidArgumentException);
}

//==============================================================================
TEST(SimulationComputeGraph, RejectsCycles)
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
TEST(SimulationComputeGraph, TopologicalOrderUsesConstructionOrderTieBreaker)
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
TEST(SimulationComputeGraph, CachedTopologicalOrderInvalidatesAfterMutations)
{
  compute::ComputeGraph graph;
  auto& a = graph.addNode("a", []() {});
  auto& b = graph.addNode("b", []() {});

  const auto initialOrder = graph.getTopologicalOrderView();
  ASSERT_EQ(initialOrder.size(), 2u);
  EXPECT_EQ(initialOrder[0], &a);
  EXPECT_EQ(initialOrder[1], &b);

  const auto* cachedStorage = initialOrder.data();
  EXPECT_EQ(graph.getTopologicalOrderView().data(), cachedStorage);

  graph.addDependency(b, a);
  const auto reordered = graph.getTopologicalOrderView();
  ASSERT_EQ(reordered.size(), 2u);
  EXPECT_EQ(reordered[0], &b);
  EXPECT_EQ(reordered[1], &a);

  graph.clear();
  EXPECT_TRUE(graph.getTopologicalOrderView().empty());
}

//==============================================================================
TEST(SimulationComputeGraph, ReportsStaticParallelLevels)
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
TEST(SimulationComputeGraph, ExportsDotWithMetadataAndProfile)
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
  const auto summary = profile.toSummaryText();

  EXPECT_NE(dot.find("digraph ComputeGraph"), std::string::npos);
  EXPECT_NE(dot.find("simulate"), std::string::npos);
  EXPECT_NE(dot.find("render"), std::string::npos);
  EXPECT_NE(dot.find("domain=rigid_body"), std::string::npos);
  EXPECT_NE(dot.find("domain=rendering"), std::string::npos);
  EXPECT_NE(dot.find("gpu"), std::string::npos);
  EXPECT_NE(dot.find("rank=same"), std::string::npos);

#if DART_BUILD_PROFILE
  EXPECT_NE(dot.find("duration_us="), std::string::npos);
  EXPECT_NE(summary.find("Compute Execution Profile"), std::string::npos);
  EXPECT_NE(summary.find("workers=1"), std::string::npos);
  EXPECT_NE(summary.find("max_parallelism="), std::string::npos);
  EXPECT_NE(summary.find("simulate"), std::string::npos);
  EXPECT_NE(summary.find("render"), std::string::npos);
#else
  EXPECT_TRUE(profile.isEmpty());
  EXPECT_NE(
      summary.find("No compute execution profile captured."),
      std::string::npos);
#endif
}

//==============================================================================
TEST(SimulationSequentialExecutor, ExecutesInDependencyOrder)
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
TEST(SimulationSequentialExecutor, ProfilesNodeLoadAndParallelism)
{
#if !DART_BUILD_PROFILE
  GTEST_SKIP() << "DART_BUILD_PROFILE=OFF: compute profiling is compiled out";
#else
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
#endif
}

//==============================================================================
TEST(SimulationParallelExecutor, RespectsDependencies)
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
TEST(SimulationParallelExecutor, PropagatesNodeExceptions)
{
  compute::ComputeGraph graph;
  graph.addNode("throwing", []() { throw std::runtime_error("task failed"); });

  compute::ParallelExecutor executor(1);

  EXPECT_THROW(executor.execute(graph), std::runtime_error);
}

//==============================================================================
TEST(SimulationParallelExecutor, ProfiledPropagatesNodeExceptions)
{
  compute::ComputeGraph graph;
  graph.addNode("throwing", []() { throw std::runtime_error("task failed"); });

  compute::ParallelExecutor executor(1);

  EXPECT_THROW({ (void)executor.executeProfiled(graph); }, std::runtime_error);
}

//==============================================================================
TEST(SimulationParallelExecutor, ProfileReportsObservedParallelism)
{
#if !DART_BUILD_PROFILE
  GTEST_SKIP() << "DART_BUILD_PROFILE=OFF: compute profiling is compiled out";
#else
  using namespace std::chrono_literals;

  compute::ComputeGraph graph;
  std::atomic<int> active{0};
  std::atomic<int> maxActive{0};
  std::atomic<int> ready{0};
  std::atomic<bool> timedOutWaitingForParallelWork{false};

  auto updateMax = [&](int value) {
    auto observed = maxActive.load();
    while (observed < value
           && !maxActive.compare_exchange_weak(observed, value)) {
    }
  };

  auto parallelWork = [&]() {
    const auto current = active.fetch_add(1) + 1;
    updateMax(current);
    ready.fetch_add(1);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (ready.load() < 2) {
      if (std::chrono::steady_clock::now() >= deadline) {
        timedOutWaitingForParallelWork.store(true);
        break;
      }
      std::this_thread::yield();
    }
    std::this_thread::sleep_for(5ms);
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
  EXPECT_FALSE(timedOutWaitingForParallelWork.load());
  EXPECT_GE(maxActive.load(), 2);
  EXPECT_GE(profile.maxParallelism, 2u);
  EXPECT_GT(profile.getAverageParallelism(), 0.0);

  const auto* leftProfile = profile.getNode("left");
  const auto* rightProfile = profile.getNode("right");
  ASSERT_NE(leftProfile, nullptr);
  ASSERT_NE(rightProfile, nullptr);
  EXPECT_EQ(leftProfile->level, 1u);
  EXPECT_EQ(rightProfile->level, 1u);
  EXPECT_LT(leftProfile->workerIndex, profile.workerCount);
  EXPECT_LT(rightProfile->workerIndex, profile.workerCount);
#endif
}

//==============================================================================
TEST(SimulationParallelExecutor, InlineThresholdRunsSubThresholdGraphsInline)
{
  // The cost gate runs graphs at or below the inline threshold sequentially,
  // skipping Taskflow; the result must match the parallel path exactly.
  auto build = [](compute::ComputeGraph& graph, std::vector<int>& out) {
    out.assign(5, -1);
    for (int i = 0; i < 5; ++i) {
      graph.addNode("n" + std::to_string(i), [&out, i]() { out[i] = i * i; });
    }
  };

  compute::ParallelExecutor executor(4);
  EXPECT_EQ(executor.getInlineThreshold(), std::size_t{1});

  // Inline path: threshold at or above the node count.
  compute::ComputeGraph inlineGraph;
  std::vector<int> inlineOut;
  build(inlineGraph, inlineOut);
  executor.setInlineThreshold(5);
  EXPECT_EQ(executor.getInlineThreshold(), std::size_t{5});
  executor.execute(inlineGraph);

  // Parallel path: threshold below the node count.
  compute::ComputeGraph parallelGraph;
  std::vector<int> parallelOut;
  build(parallelGraph, parallelOut);
  executor.setInlineThreshold(0);
  executor.execute(parallelGraph);

  const std::vector<int> expected = {0, 1, 4, 9, 16};
  EXPECT_EQ(inlineOut, expected);
  EXPECT_EQ(parallelOut, expected);
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
TEST(SimulationComputeAccess, ToStringCoversModes)
{
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Read), "read");
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Write), "write");
  EXPECT_EQ(
      compute::toString(compute::ComputeAccessMode::ReadWrite), "read_write");
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Reduce), "reduce");
  EXPECT_EQ(compute::toString(compute::ComputeAccessMode::Scratch), "scratch");
}

//==============================================================================
TEST(SimulationComputeAccess, ConflictRules)
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
TEST(SimulationComputeResourceHazards, ReadReadIsSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Read));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Read));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(SimulationComputeResourceHazards, DisjointWritesAreSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  graph.addNode(
      "b", []() {}, accessMeta("y", compute::ComputeAccessMode::Write));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(SimulationComputeResourceHazards, UnorderedWriteWriteIsHazard)
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
TEST(SimulationComputeResourceHazards, OrderedWriteWriteIsSafe)
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
TEST(SimulationComputeResourceHazards, DeclaredReductionIsSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Reduce));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Reduce));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(SimulationComputeResourceHazards, ScratchIsSafe)
{
  compute::ComputeGraph graph;
  graph.addNode(
      "a", []() {}, accessMeta("x", compute::ComputeAccessMode::Scratch));
  graph.addNode(
      "b", []() {}, accessMeta("x", compute::ComputeAccessMode::Write));
  EXPECT_TRUE(graph.findResourceHazards().empty());
}

//==============================================================================
TEST(SimulationComputeResourceHazards, PerEntityTreeHasNoFalsePositives)
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
TEST(SimulationComputeResourceHazards, CoarseSharedWriteIsFlagged)
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
TEST(SimulationComputeGraphDot, IncludesResourceAccess)
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
TEST(SimulationIntegrationKernel, SemiImplicitPositionUpdate)
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
TEST(SimulationIntegrationKernel, IsScalarGeneric)
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
TEST(SimulationIntegrationKernel, IntegratesStateBatchPositions)
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
TEST(SimulationIntegrationKernel, IntegratesStateBatchLinearStep)
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
TEST(SimulationIntegrationKernel, IntegratesOrientationAboutZ)
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
  // Exponential map: z/w = tan(omega_z * dt / 2) = tan(0.1) (ratio preserved
  // by normalization).
  EXPECT_NEAR(z / w, std::tan(0.1), 1e-12);
  EXPECT_GT(w, 0.0);
}

//==============================================================================
TEST(SimulationIntegrationKernel, OrientationKernelIsScalarGeneric)
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
TEST(SimulationIntegrationKernel, IntegratesFullStateBatch)
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
  // Orientation: z-rotation via the exponential map, x and y stay zero, unit
  // norm, z/w = tan(omega_z * dt / 2) = tan(0.1).
  EXPECT_DOUBLE_EQ(state.orientation[1], 0.0);
  EXPECT_DOUBLE_EQ(state.orientation[2], 0.0);
  const double w = state.orientation[0];
  const double z = state.orientation[3];
  EXPECT_NEAR(w * w + z * z, 1.0, 1e-12);
  EXPECT_NEAR(z / w, std::tan(0.1), 1e-12);

  // Orientation array of the wrong size is rejected.
  auto bad = state;
  bad.orientation = {1.0, 0.0, 0.0};
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatch(bad, model, force, 0.1),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(SimulationIntegrationKernel, IntegratesAngularVelocityFromTorque)
{
  compute::RigidBodyStateBatch state;
  state.worldCount = 1;
  state.bodyCount = 1;
  state.position = {0.0, 0.0, 0.0};
  state.orientation = {1.0, 0.0, 0.0, 0.0}; // identity
  state.linearVelocity = {0.0, 0.0, 0.0};
  state.angularVelocity = {0.0, 0.0, 0.0};

  compute::RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = 1;
  model.inverseMass = {1.0};
  // Diagonal body inertia diag(2, 4, 8), row-major.
  model.inertia = {2.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 8.0};

  const std::vector<double> force = {0.0, 0.0, 0.0};
  const std::vector<double> torque = {2.0, 4.0, 8.0};

  compute::integrateRigidBodyStateBatch(state, model, force, torque, 0.1);

  // Identity orientation => world inertia == body inertia, so the angular
  // acceleration is I^-1 * torque = (1, 1, 1) and the velocity gains
  // accel * dt = (0.1, 0.1, 0.1).
  EXPECT_NEAR(state.angularVelocity[0], 0.1, 1e-12);
  EXPECT_NEAR(state.angularVelocity[1], 0.1, 1e-12);
  EXPECT_NEAR(state.angularVelocity[2], 0.1, 1e-12);

  // A wrong-size inertia is rejected.
  auto bad = model;
  bad.inertia = {1.0, 2.0, 3.0};
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatch(state, bad, force, torque, 0.1),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(
    SimulationIntegrationKernel,
    BatchIntegrationRejectsMismatchedAngularVelocity)
{
  compute::RigidBodyStateBatch state;
  state.worldCount = 1;
  state.bodyCount = 1;
  state.position = {0.0, 0.0, 0.0};
  state.orientation = {1.0, 0.0, 0.0, 0.0};
  state.linearVelocity = {0.0, 0.0, 0.0};
  state.angularVelocity = {0.0, 0.0}; // too short: must be 3 * bodyCount

  compute::RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = 1;
  model.inverseMass = {1.0};
  model.inertia = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  const std::vector<double> force = {0.0, 0.0, 0.0};
  const std::vector<double> torque = {0.0, 0.0, 0.0};

  // Both overloads must reject the malformed angular-velocity array up front
  // rather than read (orientation step) or write (torque step) out of bounds.
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatch(state, model, force, 0.1),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatch(state, model, force, torque, 0.1),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(SimulationIntegrationKernel, SimdOrientationMatchesScalarForLargeBatch)
{
  // A batch above the SIMD dispatch threshold exercises the vectorized
  // orientation path; it must match the scalar-generic kernel within a few ULP.
  const std::size_t bodies = 96;

  compute::RigidBodyStateBatch state;
  state.worldCount = 1;
  state.bodyCount = bodies;
  state.position.assign(3 * bodies, 0.0);
  state.linearVelocity.assign(3 * bodies, 0.0);
  state.orientation.resize(4 * bodies);
  state.angularVelocity.resize(3 * bodies);

  std::vector<double> referenceOrientation(4 * bodies);
  std::vector<double> angular(3 * bodies);
  for (std::size_t b = 0; b < bodies; ++b) {
    const double a = 0.1 + 0.01 * static_cast<double>(b);
    double qw = std::cos(a);
    double qx = std::sin(a) * 0.3;
    double qy = std::sin(a) * 0.5;
    double qz = std::sin(a) * 0.2;
    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
    state.orientation[4 * b + 0] = qw;
    state.orientation[4 * b + 1] = qx;
    state.orientation[4 * b + 2] = qy;
    state.orientation[4 * b + 3] = qz;
    referenceOrientation[4 * b + 0] = qw;
    referenceOrientation[4 * b + 1] = qx;
    referenceOrientation[4 * b + 2] = qy;
    referenceOrientation[4 * b + 3] = qz;

    // Body 0 has zero angular velocity to exercise the no-spin guard.
    const double spin = (b == 0) ? 0.0 : 1.0;
    angular[3 * b + 0] = spin * (0.2 + 0.01 * static_cast<double>(b));
    angular[3 * b + 1] = spin * (-0.1 + 0.005 * static_cast<double>(b));
    angular[3 * b + 2] = spin * 0.3;
  }
  state.angularVelocity = angular;

  compute::RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = bodies;
  model.inverseMass.assign(bodies, 1.0);
  const std::vector<double> force(3 * bodies, 0.0);

  // Zero force keeps linear state at rest, isolating the orientation update
  // (which dispatches to the SIMD path because bodies >= the threshold).
  compute::integrateRigidBodyStateBatch(state, model, force, 0.05);

  // Reference: the scalar-generic kernel on the same orientation/angular input.
  compute::integrateOrientationsSemiImplicit(
      referenceOrientation.data(), angular.data(), 0.05, bodies);

  for (std::size_t i = 0; i < 4 * bodies; ++i) {
    EXPECT_NEAR(state.orientation[i], referenceOrientation[i], 1e-12)
        << "orientation component " << i;
  }
}

//==============================================================================
TEST(SimulationIntegrationKernel, OrientationHandlesDegenerateInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Scalar kernel: a zero-norm quaternion maps to identity (matching the
  // per-entity normalizeOrIdentity), and a non-finite angular velocity leaves a
  // valid quaternion finite and unchanged.
  {
    std::vector<double> orientation
        = {0.0,
           0.0,
           0.0,
           0.0, // body 0: zero-norm quaternion
           1.0,
           0.0,
           0.0,
           0.0}; // body 1: identity quaternion
    const std::vector<double> angular
        = {0.0,
           0.0,
           1.0, // body 0: spinning
           nan,
           nan,
           nan}; // body 1: non-finite angular velocity
    compute::integrateOrientationsSemiImplicit(
        orientation.data(), angular.data(), 0.1, 2);

    EXPECT_DOUBLE_EQ(orientation[0], 1.0); // zero-norm -> identity
    EXPECT_DOUBLE_EQ(orientation[1], 0.0);
    EXPECT_DOUBLE_EQ(orientation[2], 0.0);
    EXPECT_DOUBLE_EQ(orientation[3], 0.0);
    for (std::size_t i = 4; i < 8; ++i) {
      EXPECT_TRUE(std::isfinite(orientation[i])) << "component " << i;
    }
    EXPECT_DOUBLE_EQ(orientation[4], 1.0); // NaN spin leaves identity unchanged
  }

  // SIMD path (batch above the dispatch threshold): the same guarantees, so a
  // large batch behaves identically to a small/scalar batch on degenerate input
  // rather than producing batch-size-dependent NaN contamination.
  {
    const std::size_t bodies = 96;
    compute::RigidBodyStateBatch state;
    state.worldCount = 1;
    state.bodyCount = bodies;
    state.position.assign(3 * bodies, 0.0);
    state.linearVelocity.assign(3 * bodies, 0.0);
    state.orientation.assign(4 * bodies, 0.0);
    state.angularVelocity.assign(3 * bodies, 0.0);
    for (std::size_t b = 0; b < bodies; ++b) {
      state.orientation[4 * b + 0] = 1.0; // identity for every body
    }
    state.orientation[0] = 0.0;             // body 0: zero-norm quaternion
    state.angularVelocity[3 * 1 + 0] = nan; // body 1: non-finite spin
    state.angularVelocity[3 * 1 + 1] = nan;
    state.angularVelocity[3 * 1 + 2] = nan;

    compute::RigidBodyModelBatch model;
    model.worldCount = 1;
    model.bodyCount = bodies;
    model.inverseMass.assign(bodies, 1.0);
    const std::vector<double> force(3 * bodies, 0.0);

    compute::integrateRigidBodyStateBatch(state, model, force, 0.05);

    EXPECT_DOUBLE_EQ(state.orientation[0], 1.0); // zero-norm -> identity
    EXPECT_DOUBLE_EQ(state.orientation[1], 0.0);
    EXPECT_DOUBLE_EQ(state.orientation[2], 0.0);
    EXPECT_DOUBLE_EQ(state.orientation[3], 0.0);
    for (std::size_t i = 0; i < 4 * bodies; ++i) {
      EXPECT_TRUE(std::isfinite(state.orientation[i])) << "component " << i;
    }
  }
}

//==============================================================================
TEST(SimulationIntegrationKernel, TotalKineticEnergyReductionWithinTolerance)
{
  const std::size_t bodies = 500;

  compute::RigidBodyStateBatch state;
  state.worldCount = 1;
  state.bodyCount = bodies;
  state.linearVelocity.resize(3 * bodies);

  compute::RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = bodies;
  model.inverseMass.resize(bodies);

  for (std::size_t b = 0; b < bodies; ++b) {
    state.linearVelocity[3 * b + 0] = 0.01 * static_cast<double>(b);
    state.linearVelocity[3 * b + 1] = -0.02 * static_cast<double>(b);
    state.linearVelocity[3 * b + 2] = 0.005 * static_cast<double>(b) + 1.0;
    // Every 50th body is static (zero inverse mass) and contributes nothing.
    model.inverseMass[b]
        = (b % 50 == 0) ? 0.0 : 1.0 / (1.0 + 0.1 * static_cast<double>(b));
  }

  // A single chunk reduces left-to-right (the naive reference order).
  const double naive = compute::totalKineticEnergy(state, model, bodies + 1);

  // The reduction is deterministic for a given chunking.
  EXPECT_EQ(
      compute::totalKineticEnergy(state, model, 64),
      compute::totalKineticEnergy(state, model, 64));

  // Different chunkings (different reduction orders, as different worker counts
  // would produce) agree within a small relative tolerance -- the Phase 3
  // fixed-ULP reduction-determinism gate.
  const double tolerance = 1e-12 * std::abs(naive);
  EXPECT_NEAR(compute::totalKineticEnergy(state, model, 1), naive, tolerance);
  EXPECT_NEAR(compute::totalKineticEnergy(state, model, 64), naive, tolerance);
  EXPECT_NEAR(compute::totalKineticEnergy(state, model, 256), naive, tolerance);

  EXPECT_GT(naive, 0.0);
  EXPECT_TRUE(std::isfinite(naive));

  // Inconsistent model size is rejected.
  auto bad = model;
  bad.inverseMass.pop_back();
  EXPECT_THROW(
      (void)compute::totalKineticEnergy(state, bad),
      sx::InvalidArgumentException);
}

//==============================================================================
TEST(SimulationIntegrationKernel, RollsOutStateBatchOverControlSequence)
{
  compute::RigidBodyStateBatch initial;
  initial.worldCount = 1;
  initial.bodyCount = 1;
  initial.position = {0.0, 0.0, 0.0};
  initial.orientation = {1.0, 0.0, 0.0, 0.0};
  initial.linearVelocity = {0.0, 0.0, 0.0};
  initial.angularVelocity = {0.0, 0.0, 0.0};

  compute::RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = 1;
  model.inverseMass = {1.0};

  const std::vector<std::vector<double>> controls = {
      {1.0, 0.0, 0.0}, // step 1: unit force along +x
      {0.0, 0.0, 0.0}, // step 2: no force
  };

  const auto result
      = compute::rolloutRigidBodyStateBatch(initial, model, controls, 1.0);

  // Matches applying the two steps manually.
  auto reference = initial;
  compute::integrateRigidBodyStateBatch(reference, model, controls[0], 1.0);
  compute::integrateRigidBodyStateBatch(reference, model, controls[1], 1.0);
  EXPECT_EQ(result.position, reference.position);
  EXPECT_EQ(result.linearVelocity, reference.linearVelocity);

  // Hand computation: step 1 sets vel.x = 1 and pos.x = 1; step 2 keeps vel and
  // advances pos.x to 2.
  EXPECT_DOUBLE_EQ(result.linearVelocity[0], 1.0);
  EXPECT_DOUBLE_EQ(result.position[0], 2.0);

  // An empty control sequence returns the initial state unchanged.
  const auto unchanged
      = compute::rolloutRigidBodyStateBatch(initial, model, {}, 1.0);
  EXPECT_EQ(unchanged.position, initial.position);
  EXPECT_EQ(unchanged.linearVelocity, initial.linearVelocity);
}
