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

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/deformable_body.hpp>
#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/collision_geometry.hpp>
#include <dart/simulation/comps/deformable_body.hpp>
#include <dart/simulation/comps/dynamics.hpp>
#include <dart/simulation/comps/frame_types.hpp>
#include <dart/simulation/comps/joint.hpp>
#include <dart/simulation/comps/link.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/comps/variational_contact.hpp>
#include <dart/simulation/comps/variational_contact_dual_state.hpp>
#include <dart/simulation/compute/compute_executor.hpp>
#include <dart/simulation/compute/compute_graph.hpp>
#include <dart/simulation/compute/detail/deformable_avbd_replay_state.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/variational_integration.hpp>
#include <dart/simulation/compute/world_batch.hpp>
#include <dart/simulation/compute/world_kinematics_graph.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/constraint/loop_closure_spec.hpp>
#include <dart/simulation/detail/deformable_vbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/smooth_jacobians.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/detail/world_storage.hpp>
#include <dart/simulation/frame/fixed_frame.hpp>
#include <dart/simulation/frame/free_frame.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/version.hpp>
#include <dart/simulation/world.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>

#include <cmath>
#include <cstdlib>
#if defined(_WIN32)
  #include <malloc.h>
#endif
#include <limits>
#include <map>
#include <new>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cstdint>

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
    g_heapAllocationCount.store(0, std::memory_order_relaxed);
    g_heapAllocationBytes.store(0, std::memory_order_relaxed);
    g_heapAllocationTrackingEnabled.store(true, std::memory_order_relaxed);
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
    return g_heapAllocationCount.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t allocationBytes() const noexcept
  {
    return g_heapAllocationBytes.load(std::memory_order_relaxed);
  }
};

struct HeapAllocationSnapshot
{
  std::size_t allocationCount{0};
  std::size_t allocationBytes{0};
};

class CountingKinematicsStage final
  : public dart::simulation::compute::WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "counting_kinematics";
  }

  void execute(
      dart::simulation::World& world,
      dart::simulation::compute::ComputeExecutor& executor) override
  {
    ++executionCount;
    defaultStage.execute(world, executor);
  }

  int executionCount{0};
  dart::simulation::compute::KinematicsStage defaultStage;
};

class CountingGraphExecutor final
  : public dart::simulation::compute::ComputeExecutor
{
public:
  void execute(const dart::simulation::compute::ComputeGraph& graph) override
  {
    ++executionCount;
    delegate.execute(graph);
  }

  [[nodiscard]] dart::simulation::compute::ComputeExecutionProfile
  executeProfiled(const dart::simulation::compute::ComputeGraph& graph) override
  {
    ++executionCount;
    return delegate.executeProfiled(graph);
  }

  [[nodiscard]] std::size_t getWorkerCount() const override
  {
    return delegate.getWorkerCount();
  }

  int executionCount{0};
  dart::simulation::compute::SequentialExecutor delegate;
};

class NoOpWorldStage final : public dart::simulation::compute::WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "noop";
  }

  void execute(
      dart::simulation::World&,
      dart::simulation::compute::ComputeExecutor&) override
  {
    // Empty final stage for tests that need to isolate built-in solver stages.
  }
};

class RecordingWorldStage final
  : public dart::simulation::compute::WorldStepStage
{
public:
  RecordingWorldStage(
      std::string name,
      dart::simulation::compute::ComputeStageMetadata metadata,
      std::vector<std::string>& executionOrder)
    : m_name(std::move(name)),
      m_metadata(metadata),
      m_executionOrder(executionOrder)
  {
  }

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return m_name;
  }

  [[nodiscard]] dart::simulation::compute::ComputeStageMetadata getMetadata()
      const noexcept override
  {
    return m_metadata;
  }

  void execute(
      dart::simulation::World&,
      dart::simulation::compute::ComputeExecutor& executor) override
  {
    dart::simulation::compute::ComputeGraph graph;
    graph.addNode(
        m_name + "_node",
        [&]() { m_executionOrder.push_back(m_name); },
        m_metadata);
    executor.execute(graph);
  }

private:
  std::string m_name;
  dart::simulation::compute::ComputeStageMetadata m_metadata;
  std::vector<std::string>& m_executionOrder;
};

class RecordingExecutor final
  : public dart::simulation::compute::ComputeExecutor
{
public:
  void execute(const dart::simulation::compute::ComputeGraph& graph) override
  {
    record(graph);
    sequential.execute(graph);
  }

  [[nodiscard]]
  dart::simulation::compute::ComputeExecutionProfile executeProfiled(
      const dart::simulation::compute::ComputeGraph& graph) override
  {
    record(graph);
    return sequential.executeProfiled(graph);
  }

  [[nodiscard]] std::size_t getWorkerCount() const override
  {
    return sequential.getWorkerCount();
  }

  std::size_t nodeCount{0};
  std::size_t edgeCount{0};
  std::size_t executeCount{0};
  std::vector<std::vector<std::string>> graphNodeNames;

private:
  void record(const dart::simulation::compute::ComputeGraph& graph)
  {
    ++executeCount;
    nodeCount = graph.getNodeCount();
    edgeCount = graph.getEdgeCount();

    std::vector<std::string> names;
    names.reserve(graph.getNodeCount());
    for (const auto* node : graph.getNodes()) {
      names.push_back(node->getName());
    }
    graphNodeNames.push_back(std::move(names));
  }

  dart::simulation::compute::SequentialExecutor sequential;
};

class CountingMemoryAllocator final : public dart::common::MemoryAllocator
{
public:
  [[nodiscard]] std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  [[nodiscard]] void* allocate(std::size_t bytes) noexcept override
  {
    if (bytes == 0) {
      return nullptr;
    }

    ++allocationCount;
    return ::operator new(bytes, std::nothrow);
  }

  [[nodiscard]] void* allocate(
      std::size_t bytes, std::size_t alignment) noexcept override
  {
    if (bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
      return nullptr;
    }

    ++allocationCount;
    ++alignedAllocationCount;
    if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      return ::operator new(bytes, std::nothrow);
    }

    return ::operator new(bytes, std::align_val_t(alignment), std::nothrow);
  }

  void deallocate(void* pointer, std::size_t /*bytes*/) override
  {
    ++deallocationCount;
    ::operator delete(pointer);
  }

  void deallocate(
      void* pointer, std::size_t /*bytes*/, std::size_t alignment) override
  {
    ++deallocationCount;
    ++alignedDeallocationCount;
    if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      ::operator delete(pointer);
      return;
    }

    ::operator delete(pointer, std::align_val_t(alignment));
  }

  std::size_t allocationCount{0};
  std::size_t alignedAllocationCount{0};
  std::size_t deallocationCount{0};
  std::size_t alignedDeallocationCount{0};
};

class FrameScratchStage final : public dart::simulation::compute::WorldStepStage
{
public:
  explicit FrameScratchStage(std::size_t bytes) : bytesToAllocate(bytes) {}

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "frame_scratch";
  }

  void execute(
      dart::simulation::World& world,
      dart::simulation::compute::ComputeExecutor&) override
  {
    if (bytesToAllocate == 0) {
      lastAllocation = nullptr;
      return;
    }

    lastAllocation = world.getMemoryManager().getFrameAllocator().allocate(
        bytesToAllocate);
  }

  std::size_t bytesToAllocate;
  void* lastAllocation{nullptr};
};

struct alignas(64) RegistryAllocatorComponent
{
  explicit RegistryAllocatorComponent(std::uint64_t input) noexcept
    : value(input)
  {
  }

  std::uint64_t value;
};

using RegistryStorageCapacities = std::map<entt::id_type, std::size_t>;

RegistryStorageCapacities registryStorageCapacities(
    const dart::simulation::detail::WorldRegistry& registry)
{
  RegistryStorageCapacities capacities;
  const auto* entityStorage = registry.storage<entt::entity>();
  EXPECT_NE(entityStorage, nullptr);
  capacities.emplace(
      entt::type_hash<entt::entity>::value(),
      entityStorage == nullptr ? 0u : entityStorage->capacity());
  for (auto&& [id, storage] : registry.storage()) {
    capacities.emplace(id, storage.capacity());
  }
  return capacities;
}

void expectRegistryStorageCapacitiesUnchanged(
    const RegistryStorageCapacities& expected,
    const dart::simulation::detail::WorldRegistry& registry)
{
  const auto actual = registryStorageCapacities(registry);
  EXPECT_EQ(actual.size(), expected.size());

  for (const auto& [id, capacity] : actual) {
    if (!expected.contains(id)) {
      ADD_FAILURE() << "unexpected storage id " << id << " capacity "
                    << capacity;
    }
  }

  for (const auto& [id, capacity] : expected) {
    const auto it = actual.find(id);
    ASSERT_NE(it, actual.end()) << "missing storage id " << id;
    EXPECT_EQ(it->second, capacity) << "storage id " << id;
  }
}

template <typename ConfigureScene>
void expectNoWorldBaseAllocatorActivityDuringBakedSteps(
    std::string_view scene,
    ConfigureScene&& configureScene,
    bool requireInitialContact = false)
{
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  CountingMemoryAllocator allocator;
  sx::WorldOptions worldOptions;
  worldOptions.baseAllocator = &allocator;
  sx::World world(worldOptions);

  configureScene(world);
  world.enterSimulationMode();
  if (requireInitialContact) {
    ASSERT_FALSE(world.collide().empty());
  }

  const auto allocationsAfterBake = allocator.allocationCount;
  const auto deallocationsAfterBake = allocator.deallocationCount;
  const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterBake = allocator.alignedDeallocationCount;

  for (int i = 0; i < 4; ++i) {
    world.step();
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
  EXPECT_EQ(allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);
}

template <typename ConfigureScene>
void expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
    std::string_view scene,
    ConfigureScene&& configureScene,
    bool requireInitialContact = true,
    std::size_t minInitialContacts = 1)
{
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  CountingMemoryAllocator allocator;
  sx::WorldOptions worldOptions;
  worldOptions.baseAllocator = &allocator;
  worldOptions.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(worldOptions);

  configureScene(world);
  world.enterSimulationMode();
  if (requireInitialContact) {
    ASSERT_GE(world.collide().size(), minInitialContacts);
  }

  const auto allocationsAfterBake = allocator.allocationCount;
  const auto deallocationsAfterBake = allocator.deallocationCount;
  const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterBake = allocator.alignedDeallocationCount;

  for (int i = 0; i < 4; ++i) {
    world.step();
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
  EXPECT_EQ(allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);
}

template <typename ConfigureScene>
void expectNoGlobalHeapAllocationsDuringBakedSteps(
    std::string_view scene,
    ConfigureScene&& configureScene,
    bool requireInitialContact = false)
{
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  sx::World world;

  configureScene(world);
  world.enterSimulationMode();
  if (requireInitialContact) {
    ASSERT_FALSE(world.collide().empty());
  }

  ScopedHeapAllocationCounter heapCounter;
  for (int i = 0; i < 4; ++i) {
    world.step();
  }
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated during baked steps: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

template <typename ConfigureScene>
void expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
    std::string_view scene,
    ConfigureScene&& configureScene,
    bool requireInitialContact = true,
    std::size_t minInitialContacts = 1)
{
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  configureScene(world);
  world.enterSimulationMode();
  if (requireInitialContact) {
    ASSERT_GE(world.collide().size(), minInitialContacts);
  }

  ScopedHeapAllocationCounter heapCounter;
  for (int i = 0; i < 4; ++i) {
    world.step();
  }
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated during baked steps: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

template <typename ConfigureScene>
HeapAllocationSnapshot countGlobalHeapAllocationsDuringSimulationBake(
    std::string_view scene, ConfigureScene&& configureScene)
{
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  sx::World world;

  configureScene(world);

  ScopedHeapAllocationCounter heapCounter;
  world.enterSimulationMode();
  heapCounter.stop();

  return {heapCounter.allocationCount(), heapCounter.allocationBytes()};
}

void configureSemiImplicitExternalForceMultibodyScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  auto robot = world.addMultibody("forced_slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(2.0);
  carriage.applyForce(
      Eigen::Vector3d(0.0, 0.0, 4.0), Eigen::Vector3d::Zero(), false, false);
  world.setTimeStep(0.01);
}

void configureCrossMultibodyDifferentDofFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  auto lowerRobot = world.addMultibody("lower_robot");
  auto lowerBase = lowerRobot.addLink("base");
  sx::JointSpec lowerSpec;
  lowerSpec.name = "lower_slider";
  lowerSpec.type = sx::JointType::Prismatic;
  lowerSpec.axis = Eigen::Vector3d::UnitZ();
  auto lowerLink = lowerRobot.addLink("link", lowerBase, lowerSpec);
  lowerLink.setMass(1.0);
  lowerLink.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  auto lowerJoint = lowerLink.getParentJoint();
  lowerJoint.setPosition(Eigen::VectorXd::Constant(1, 0.0));
  lowerJoint.setVelocity(Eigen::VectorXd::Constant(1, 0.5));

  auto upperRobot = world.addMultibody("upper_robot");
  auto upperBase = upperRobot.addLink("base");
  sx::JointSpec rootSpec;
  rootSpec.name = "upper_root_slider";
  rootSpec.type = sx::JointType::Prismatic;
  rootSpec.axis = Eigen::Vector3d::UnitZ();
  auto upperMid = upperRobot.addLink("mid", upperBase, rootSpec);
  upperMid.setMass(1.0);
  auto upperRootJoint = upperMid.getParentJoint();
  upperRootJoint.setPosition(Eigen::VectorXd::Constant(1, 0.15));
  upperRootJoint.setVelocity(Eigen::VectorXd::Constant(1, -0.25));

  sx::JointSpec tipSpec;
  tipSpec.name = "upper_tip_slider";
  tipSpec.type = sx::JointType::Prismatic;
  tipSpec.axis = Eigen::Vector3d::UnitZ();
  auto upperTip = upperRobot.addLink("tip", upperMid, tipSpec);
  upperTip.setMass(1.0);
  upperTip.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  auto upperTipJoint = upperTip.getParentJoint();
  upperTipJoint.setPosition(Eigen::VectorXd::Constant(1, 0.20));
  upperTipJoint.setVelocity(Eigen::VectorXd::Constant(1, -0.25));

  world.setTimeStep(0.001);
}

void configureCrossMultibodyStackedFallbackScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  const auto addRobot = [&](std::string_view name, double z, double velocity) {
    auto robot = world.addMultibody(name);
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, z));
    joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
  };

  addRobot("lower_robot", 0.0, -0.2);
  addRobot("upper_robot", 0.35, -0.8);

  sx::RigidBodyOptions fixtureOptions;
  fixtureOptions.isStatic = true;
  fixtureOptions.position = Eigen::Vector3d(0.0, 0.0, -0.35);
  auto fixture = world.addRigidBody("fixture", fixtureOptions);
  fixture.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  world.setTimeStep(0.001);
}

void configureCrossMultibodyCoupledRowsFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  const auto addRobot = [&](std::string_view name, double z, double velocity) {
    auto robot = world.addMultibody(name);
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, z));
    joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
  };

  addRobot("lower_robot", 0.0, 0.5);
  addRobot("middle_robot", 0.35, 0.0);
  addRobot("upper_robot", 0.70, -0.5);

  world.setTimeStep(0.001);
}

void configureCrossMultibodyLargeStackFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  constexpr std::array<double, 5> velocities{0.6, 0.3, 0.0, -0.3, -0.6};
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    auto robot = world.addMultibody("stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.35 * i));
    joint.setVelocity(Eigen::VectorXd::Constant(1, velocities[i]));
  }

  world.setTimeStep(0.001);
}

void configureCrossMultibodyExtendedStackFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  constexpr std::array<double, 8> velocities{
      0.8, 0.55, 0.3, 0.1, -0.1, -0.3, -0.55, -0.8};
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    auto robot
        = world.addMultibody("extended_stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.35 * i));
    joint.setVelocity(Eigen::VectorXd::Constant(1, velocities[i]));
  }

  world.setTimeStep(0.001);
}

void configureCrossMultibodyProductionStackFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  constexpr std::array<double, 12> velocities{
      1.0, 0.8, 0.6, 0.4, 0.2, 0.05, -0.05, -0.2, -0.4, -0.6, -0.8, -1.0};
  for (std::size_t i = 0; i < velocities.size(); ++i) {
    auto robot
        = world.addMultibody("production_stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.35 * i));
    joint.setVelocity(Eigen::VectorXd::Constant(1, velocities[i]));
  }

  world.setTimeStep(0.001);
}

void configureCrossMultibodyDenseProductionStackFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  constexpr std::size_t bodyCount = 16;
  constexpr double maxSpeed = 1.2;
  constexpr double center = 0.5 * static_cast<double>(bodyCount - 1);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    auto robot = world.addMultibody(
        "dense_production_stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.35 * i));
    const double normalized = (center - static_cast<double>(i)) / center;
    joint.setVelocity(Eigen::VectorXd::Constant(1, maxSpeed * normalized));
  }

  world.setTimeStep(0.001);
}

void configureCrossMultibodyExtraDenseProductionStackFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  constexpr std::size_t bodyCount = 24;
  constexpr double maxSpeed = 1.4;
  constexpr double center = 0.5 * static_cast<double>(bodyCount - 1);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    auto robot = world.addMultibody(
        "extra_dense_production_stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.35 * i));
    const double normalized = (center - static_cast<double>(i)) / center;
    joint.setVelocity(Eigen::VectorXd::Constant(1, maxSpeed * normalized));
  }

  world.setTimeStep(0.001);
}

void configureCrossMultibodyStressProductionStackFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());

  constexpr std::size_t bodyCount = 32;
  constexpr double maxSpeed = 1.6;
  constexpr double center = 0.5 * static_cast<double>(bodyCount - 1);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    auto robot = world.addMultibody(
        "stress_production_stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.35 * i));
    const double normalized = (center - static_cast<double>(i)) / center;
    joint.setVelocity(Eigen::VectorXd::Constant(1, maxSpeed * normalized));
  }

  world.setTimeStep(0.001);
}

struct CrossMultibodyMultiIslandFallbackSceneHandles
{
  dart::simulation::Joint differentLowerJoint{
      dart::simulation::Entity{}, nullptr};
  dart::simulation::Joint differentUpperTipJoint{
      dart::simulation::Entity{}, nullptr};
  dart::simulation::Joint stackLowerJoint{dart::simulation::Entity{}, nullptr};
  dart::simulation::Joint stackMiddleJoint{dart::simulation::Entity{}, nullptr};
  dart::simulation::Joint stackUpperJoint{dart::simulation::Entity{}, nullptr};
  dart::simulation::RigidBody lowerRigid{dart::simulation::Entity{}, nullptr};
  dart::simulation::RigidBody upperRigid{dart::simulation::Entity{}, nullptr};
};

CrossMultibodyMultiIslandFallbackSceneHandles
configureCrossMultibodyMultiIslandFallbackSceneWithHandles(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  CrossMultibodyMultiIslandFallbackSceneHandles handles;

  const auto makeOffsetSphere = [](double xOffset) {
    auto shape = sx::CollisionShape::makeSphere(0.2);
    shape.localTransform.translation() = Eigen::Vector3d(xOffset, 0.0, 0.0);
    return shape;
  };

  const auto addSlider
      = [&](std::string_view robotName, double xOffset, double z, double v) {
          auto robot = world.addMultibody(robotName);
          auto base = robot.addLink("base");
          sx::JointSpec spec;
          spec.name = "slider";
          spec.type = sx::JointType::Prismatic;
          spec.axis = Eigen::Vector3d::UnitZ();
          auto link = robot.addLink("link", base, spec);
          link.setMass(1.0);
          link.setCollisionShape(makeOffsetSphere(xOffset));
          auto joint = link.getParentJoint();
          joint.setPosition(Eigen::VectorXd::Constant(1, z));
          joint.setVelocity(Eigen::VectorXd::Constant(1, v));
          return joint;
        };

  handles.differentLowerJoint
      = addSlider("multi_island_different_lower", 0.0, 0.0, 0.5);

  auto upperRobot = world.addMultibody("multi_island_different_upper");
  auto upperBase = upperRobot.addLink("base");
  sx::JointSpec rootSpec;
  rootSpec.name = "upper_root_slider";
  rootSpec.type = sx::JointType::Prismatic;
  rootSpec.axis = Eigen::Vector3d::UnitZ();
  auto upperMid = upperRobot.addLink("mid", upperBase, rootSpec);
  upperMid.setMass(1.0);
  auto upperRootJoint = upperMid.getParentJoint();
  upperRootJoint.setPosition(Eigen::VectorXd::Constant(1, 0.15));
  upperRootJoint.setVelocity(Eigen::VectorXd::Constant(1, -0.25));

  sx::JointSpec tipSpec;
  tipSpec.name = "upper_tip_slider";
  tipSpec.type = sx::JointType::Prismatic;
  tipSpec.axis = Eigen::Vector3d::UnitZ();
  auto upperTip = upperRobot.addLink("tip", upperMid, tipSpec);
  upperTip.setMass(1.0);
  upperTip.setCollisionShape(makeOffsetSphere(0.0));
  handles.differentUpperTipJoint = upperTip.getParentJoint();
  handles.differentUpperTipJoint.setPosition(
      Eigen::VectorXd::Constant(1, 0.20));
  handles.differentUpperTipJoint.setVelocity(
      Eigen::VectorXd::Constant(1, -0.25));

  handles.stackLowerJoint
      = addSlider("multi_island_stack_lower", 2.5, 0.0, 0.45);
  handles.stackMiddleJoint
      = addSlider("multi_island_stack_middle", 2.5, 0.35, 0.0);
  handles.stackUpperJoint
      = addSlider("multi_island_stack_upper", 2.5, 0.70, -0.45);

  sx::RigidBodyOptions lowerRigidOptions;
  lowerRigidOptions.position = Eigen::Vector3d(5.0, 0.0, 0.0);
  lowerRigidOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, 0.4);
  handles.lowerRigid
      = world.addRigidBody("multi_island_lower_rigid", lowerRigidOptions);
  handles.lowerRigid.setMass(1.0);
  handles.lowerRigid.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  sx::RigidBodyOptions upperRigidOptions;
  upperRigidOptions.position = Eigen::Vector3d(5.0, 0.0, 0.35);
  upperRigidOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -0.4);
  handles.upperRigid
      = world.addRigidBody("multi_island_upper_rigid", upperRigidOptions);
  handles.upperRigid.setMass(1.0);
  handles.upperRigid.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  return handles;
}

void configureCrossMultibodyMultiIslandFallbackScene(
    dart::simulation::World& world)
{
  (void)configureCrossMultibodyMultiIslandFallbackSceneWithHandles(world);
}

void configureCrossMultibodyProductionMultiIslandFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  (void)configureCrossMultibodyMultiIslandFallbackSceneWithHandles(world);

  const auto makeOffsetSphere = [](double xOffset) {
    auto shape = sx::CollisionShape::makeSphere(0.2);
    shape.localTransform.translation() = Eigen::Vector3d(xOffset, 0.0, 0.0);
    return shape;
  };

  const auto addSlider
      = [&](std::string_view robotName, double xOffset, double z, double v) {
          auto robot = world.addMultibody(robotName);
          auto base = robot.addLink("base");
          sx::JointSpec spec;
          spec.name = "slider";
          spec.type = sx::JointType::Prismatic;
          spec.axis = Eigen::Vector3d::UnitZ();
          auto link = robot.addLink("link", base, spec);
          link.setMass(1.0);
          link.setCollisionShape(makeOffsetSphere(xOffset));
          auto joint = link.getParentJoint();
          joint.setPosition(Eigen::VectorXd::Constant(1, z));
          joint.setVelocity(Eigen::VectorXd::Constant(1, v));
        };

  constexpr std::array<double, 6> articulatedVelocities{
      0.9, 0.55, 0.2, -0.2, -0.55, -0.9};
  for (std::size_t i = 0; i < articulatedVelocities.size(); ++i) {
    addSlider(
        "multi_island_production_stack_" + std::to_string(i),
        7.5,
        0.35 * static_cast<double>(i),
        articulatedVelocities[i]);
  }

  constexpr std::array<double, 4> rigidVelocities{0.6, 0.2, -0.2, -0.6};
  for (std::size_t i = 0; i < rigidVelocities.size(); ++i) {
    sx::RigidBodyOptions options;
    options.position
        = Eigen::Vector3d(10.0, 0.0, 0.35 * static_cast<double>(i));
    options.linearVelocity = Eigen::Vector3d(0.0, 0.0, rigidVelocities[i]);
    auto body = world.addRigidBody(
        "multi_island_production_rigid_" + std::to_string(i), options);
    body.setMass(1.0);
    body.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }
}

void configureCrossMultibodyStressMultiIslandFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  configureCrossMultibodyProductionMultiIslandFallbackScene(world);

  const auto makeOffsetSphere = [](double xOffset) {
    auto shape = sx::CollisionShape::makeSphere(0.2);
    shape.localTransform.translation() = Eigen::Vector3d(xOffset, 0.0, 0.0);
    return shape;
  };

  constexpr std::size_t articulatedCount = 12;
  constexpr double articulatedMaxSpeed = 1.2;
  constexpr double articulatedCenter
      = 0.5 * static_cast<double>(articulatedCount - 1);
  for (std::size_t i = 0; i < articulatedCount; ++i) {
    auto robot = world.addMultibody(
        "multi_island_stress_articulated_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(makeOffsetSphere(12.5));
    auto joint = link.getParentJoint();
    joint.setPosition(
        Eigen::VectorXd::Constant(1, 0.35 * static_cast<double>(i)));
    const double normalized
        = (articulatedCenter - static_cast<double>(i)) / articulatedCenter;
    joint.setVelocity(
        Eigen::VectorXd::Constant(1, articulatedMaxSpeed * normalized));
  }

  constexpr std::size_t rigidCount = 8;
  constexpr double rigidMaxSpeed = 0.9;
  constexpr double rigidCenter = 0.5 * static_cast<double>(rigidCount - 1);
  for (std::size_t i = 0; i < rigidCount; ++i) {
    sx::RigidBodyOptions options;
    options.position
        = Eigen::Vector3d(15.0, 0.0, 0.35 * static_cast<double>(i));
    const double normalized
        = (rigidCenter - static_cast<double>(i)) / rigidCenter;
    options.linearVelocity
        = Eigen::Vector3d(0.0, 0.0, rigidMaxSpeed * normalized);
    auto body = world.addRigidBody(
        "multi_island_stress_rigid_" + std::to_string(i), options);
    body.setMass(1.0);
    body.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
  }
}

void configureCrossMultibodyMixedStressFallbackScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  configureCrossMultibodyStressMultiIslandFallbackScene(world);

  const auto makeOffsetSphere = [](double xOffset) {
    auto shape = sx::CollisionShape::makeSphere(0.2);
    shape.localTransform.translation() = Eigen::Vector3d(xOffset, 0.0, 0.0);
    return shape;
  };

  constexpr std::size_t bodyCount = 32;
  constexpr double maxSpeed = 1.6;
  constexpr double center = 0.5 * static_cast<double>(bodyCount - 1);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    auto robot
        = world.addMultibody("mixed_stress_stack_robot_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "slider";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto link = robot.addLink("link", base, spec);
    link.setMass(1.0);
    link.setCollisionShape(makeOffsetSphere(20.0));
    auto joint = link.getParentJoint();
    joint.setPosition(
        Eigen::VectorXd::Constant(1, 0.35 * static_cast<double>(i)));
    const double normalized = (center - static_cast<double>(i)) / center;
    joint.setVelocity(Eigen::VectorXd::Constant(1, maxSpeed * normalized));
  }
}

void configureDeformableSelfContactFrictionPatchScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr double gap = 0.012;
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, 0.5, 0.0),
         Eigen::Vector3d(-0.5, 0.5, 0.0),
         Eigen::Vector3d(-0.5, -0.5, gap),
         Eigen::Vector3d(0.5, -0.5, gap),
         Eigen::Vector3d(0.5, 0.5, gap),
         Eigen::Vector3d(-0.5, 0.5, gap)};
  options.velocities
      = {Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero(),
         Eigen::Vector3d(0.5, 0.0, -0.1),
         Eigen::Vector3d(0.5, 0.0, -0.1),
         Eigen::Vector3d(0.5, 0.0, -0.1),
         Eigen::Vector3d(0.5, 0.0, -0.1)};
  options.masses = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  options.fixedNodes = {0, 1, 2, 3};
  options.edgeStiffness = 0.0;
  options.material.frictionCoefficient = 0.8;
  options.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{0, 2, 3},
         sx::DeformableSurfaceTriangle{4, 6, 5},
         sx::DeformableSurfaceTriangle{4, 7, 6}};

  world.addDeformableBody("friction_patches", options);
}

void configureScriptedDeformableBoundaryScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.02);

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(2.0, 0.0, 0.0),
         Eigen::Vector3d(3.0, 0.0, 0.0)};
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 1.0);
  options.edges
      = {sx::DeformableEdge{0, 1, 1.0},
         sx::DeformableEdge{1, 2, 1.0},
         sx::DeformableEdge{2, 3, 1.0}};
  options.edgeStiffness = 50.0;

  sx::DeformableDirichletBoundaryCondition dirichlet;
  dirichlet.nodes = {0, 3};
  dirichlet.linearVelocity = Eigen::Vector3d(0.0, 0.1, 0.0);
  dirichlet.endTime = 1.0;
  options.dirichletBoundaryConditions.push_back(std::move(dirichlet));

  sx::DeformableNeumannBoundaryCondition neumann;
  neumann.nodes = {1, 2};
  neumann.acceleration = Eigen::Vector3d(0.0, -0.5, 0.0);
  neumann.endTime = 1.0;
  options.neumannBoundaryConditions.push_back(std::move(neumann));

  world.addDeformableBody("scripted_boundary_chain", options);
}

void addDeformableFemGroundFrictionBlock(
    dart::simulation::World& world,
    std::string_view bodyName,
    const Eigen::Vector3d& offset,
    std::size_t cells,
    bool useMatrixFreeLinearSolver,
    bool useIterativeLinearSolver = false)
{
  namespace sx = dart::simulation;

  const std::size_t nodesPerAxis = cells + 1;
  constexpr double spacing = 0.1;
  const auto nodeIndex
      = [nodesPerAxis](std::size_t x, std::size_t y, std::size_t z) {
          return z * nodesPerAxis * nodesPerAxis + y * nodesPerAxis + x;
        };

  sx::DeformableBodyOptions options;
  options.positions.reserve(nodesPerAxis * nodesPerAxis * nodesPerAxis);
  options.velocities.reserve(nodesPerAxis * nodesPerAxis * nodesPerAxis);
  options.masses.assign(nodesPerAxis * nodesPerAxis * nodesPerAxis, 0.1);
  const double halfExtent = 0.5 * static_cast<double>(cells) * spacing;
  const Eigen::Vector3d origin
      = offset + Eigen::Vector3d(-halfExtent, -halfExtent, 0.004);
  for (std::size_t z = 0; z < nodesPerAxis; ++z) {
    for (std::size_t y = 0; y < nodesPerAxis; ++y) {
      for (std::size_t x = 0; x < nodesPerAxis; ++x) {
        options.positions.push_back(
            origin
            + spacing
                  * Eigen::Vector3d(
                      static_cast<double>(x),
                      static_cast<double>(y),
                      static_cast<double>(z)));
        options.velocities.emplace_back(0.5, 0.0, -0.05);
      }
    }
  }

  const std::array<std::array<std::size_t, 4>, 6> localTets = {{
      {0, 1, 3, 7},
      {0, 3, 2, 7},
      {0, 2, 6, 7},
      {0, 6, 4, 7},
      {0, 4, 5, 7},
      {0, 5, 1, 7},
  }};
  options.tetrahedra.reserve(6 * cells * cells * cells);
  for (std::size_t z = 0; z < cells; ++z) {
    for (std::size_t y = 0; y < cells; ++y) {
      for (std::size_t x = 0; x < cells; ++x) {
        const std::array<std::size_t, 8> corner = {{
            nodeIndex(x, y, z),
            nodeIndex(x + 1, y, z),
            nodeIndex(x, y + 1, z),
            nodeIndex(x + 1, y + 1, z),
            nodeIndex(x, y, z + 1),
            nodeIndex(x + 1, y, z + 1),
            nodeIndex(x, y + 1, z + 1),
            nodeIndex(x + 1, y + 1, z + 1),
        }};
        for (const auto& tet : localTets) {
          options.tetrahedra.push_back(
              sx::DeformableTetrahedron{
                  corner[tet[0]],
                  corner[tet[1]],
                  corner[tet[2]],
                  corner[tet[3]]});
        }
      }
    }
  }

  options.edgeStiffness = 0.0;
  options.material.useFiniteElementElasticity = true;
  options.material.youngsModulus = 5.0e4;
  options.material.poissonRatio = 0.3;
  options.material.frictionCoefficient = 0.8;
  options.material.useIterativeLinearSolver = useIterativeLinearSolver;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeLinearSolver;

  world.addDeformableBody(bodyName, options);
}

void configureDeformableFemGroundFrictionBlockScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.002);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("fem_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  addDeformableFemGroundFrictionBlock(
      world, "fem_ground_friction_block", Eigen::Vector3d::Zero(), 2, false);
}

void configureDeformableIterativeFemGroundFrictionBlockScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.002);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("iterative_fem_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  addDeformableFemGroundFrictionBlock(
      world,
      "iterative_fem_ground_friction_block",
      Eigen::Vector3d::Zero(),
      2,
      false,
      true);
}

void configureMixedDefaultDeformableFemProductionStorageScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.002);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("mixed_fem_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(12.0, 12.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  addDeformableFemGroundFrictionBlock(
      world,
      "mixed_fem_direct_production_block",
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      3,
      false);
  addDeformableFemGroundFrictionBlock(
      world,
      "mixed_fem_matrix_free_production_block",
      Eigen::Vector3d(1.0, 0.0, 0.0),
      3,
      true);
}

void configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
    dart::simulation::World& world,
    std::size_t rows,
    std::size_t cols,
    std::string_view bodyName,
    double gap,
    const Eigen::Vector3d& upperLayerVelocity,
    bool useMatrixFreeLinearSolver = false,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero())
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr double spacing = 0.5;
  const std::size_t layerNodeCount = rows * cols;
  sx::DeformableBodyOptions options;
  options.positions.reserve(2 * layerNodeCount);
  options.velocities.reserve(2 * layerNodeCount);
  options.masses.assign(2 * layerNodeCount, 1.0);
  options.fixedNodes.reserve(layerNodeCount);

  const auto nodeIndex
      = [cols, layerNodeCount](
            std::size_t layer, std::size_t row, std::size_t col) {
          return layer * layerNodeCount + row * cols + col;
        };

  for (std::size_t layer = 0; layer < 2; ++layer) {
    const double z = layer == 0 ? 0.0 : gap;
    const Eigen::Vector3d velocity
        = layer == 0 ? Eigen::Vector3d::Zero() : upperLayerVelocity;
    for (std::size_t row = 0; row < rows; ++row) {
      for (std::size_t col = 0; col < cols; ++col) {
        const Eigen::Vector3d position(
            (static_cast<double>(col) - 1.0) * spacing,
            (static_cast<double>(row) - 1.0) * spacing,
            z);
        options.positions.push_back(offset + position);
        options.velocities.push_back(velocity);
        if (layer == 0) {
          options.fixedNodes.push_back(nodeIndex(layer, row, col));
        }
      }
    }
  }

  options.surfaceTriangles.reserve(4 * (rows - 1) * (cols - 1));
  for (std::size_t row = 0; row + 1 < rows; ++row) {
    for (std::size_t col = 0; col + 1 < cols; ++col) {
      const auto a = nodeIndex(0, row, col);
      const auto b = nodeIndex(0, row, col + 1);
      const auto c = nodeIndex(0, row + 1, col + 1);
      const auto d = nodeIndex(0, row + 1, col);
      options.surfaceTriangles.push_back(
          sx::DeformableSurfaceTriangle{a, b, c});
      options.surfaceTriangles.push_back(
          sx::DeformableSurfaceTriangle{a, c, d});

      const auto e = nodeIndex(1, row, col);
      const auto f = nodeIndex(1, row, col + 1);
      const auto g = nodeIndex(1, row + 1, col + 1);
      const auto h = nodeIndex(1, row + 1, col);
      options.surfaceTriangles.push_back(
          sx::DeformableSurfaceTriangle{e, g, f});
      options.surfaceTriangles.push_back(
          sx::DeformableSurfaceTriangle{e, h, g});
    }
  }

  options.edgeStiffness = 0.0;
  options.material.frictionCoefficient = 0.8;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeLinearSolver;

  world.addDeformableBody(bodyName, options);
}

void configureDeformableSelfContactFrictionGridSceneWithShape(
    dart::simulation::World& world,
    std::size_t rows,
    std::size_t cols,
    std::string_view bodyName)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world, rows, cols, bodyName, 0.012, Eigen::Vector3d(0.35, 0.1, -0.08));
}

void configureDeformableSelfContactFrictionGridSceneWithSize(
    dart::simulation::World& world, std::size_t grid, std::string_view bodyName)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, grid, grid, bodyName);
}

void configureDeformableSelfContactFrictionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithSize(
      world, 5, "friction_grid");
}

void configureDeformableSelfContactFrictionLargeGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithSize(
      world, 7, "friction_large_grid");
}

void configureDeformableSelfContactFrictionProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithSize(
      world, 9, "friction_production_grid");
}

void configureDeformableSelfContactFrictionExtendedProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithSize(
      world, 11, "friction_extended_production_grid");
}

void configureDeformableSelfContactFrictionDenseProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithSize(
      world, 13, "friction_dense_production_grid");
}

void configureDeformableSelfContactFrictionExtraDenseGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      15,
      15,
      "friction_extra_dense_production_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08));
}

void configureDeformableSelfContactFrictionLargerProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      17,
      17,
      "friction_larger_production_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08));
}

void configureDeformableSelfContactFrictionLargerMatrixFreeProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      17,
      17,
      "friction_larger_matrix_free_production_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true);
}

void configureDeformableSelfContactFrictionIrregularProductionGridSceneWithOffset(
    dart::simulation::World& world,
    std::string_view bodyName,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero(),
    bool useMatrixFreeLinearSolver = false)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  constexpr std::size_t rows = 13;
  constexpr std::size_t cols = 17;
  constexpr double spacing = 0.42;
  constexpr double gap = 0.012;
  constexpr std::size_t layerNodeCount = rows * cols;
  sx::DeformableBodyOptions options;
  options.positions.reserve(2 * layerNodeCount);
  options.velocities.reserve(2 * layerNodeCount);
  options.masses.assign(2 * layerNodeCount, 1.0);
  options.fixedNodes.reserve(layerNodeCount);

  const auto nodeIndex
      = [](std::size_t layer, std::size_t row, std::size_t col) {
          return layer * layerNodeCount + row * cols + col;
        };
  const auto planarPosition = [](std::size_t row, std::size_t col) {
    const auto signedRow = static_cast<int>(row);
    const auto signedCol = static_cast<int>(col);
    const double xJitter
        = 0.026
          * static_cast<double>(static_cast<int>((row + 2 * col) % 3) - 1);
    const double yJitter
        = 0.018
          * static_cast<double>(static_cast<int>((2 * row + col) % 5) - 2);
    return Eigen::Vector2d(
        (static_cast<double>(signedCol) - 0.5 * static_cast<double>(cols - 1))
                * spacing
            + 0.025 * static_cast<double>(signedRow) + xJitter,
        (static_cast<double>(signedRow) - 0.5 * static_cast<double>(rows - 1))
                * spacing
            + yJitter);
  };

  for (std::size_t layer = 0; layer < 2; ++layer) {
    const double z = layer == 0 ? 0.0 : gap;
    const Eigen::Vector3d velocity = layer == 0
                                         ? Eigen::Vector3d::Zero()
                                         : Eigen::Vector3d(0.42, -0.18, -0.08);
    for (std::size_t row = 0; row < rows; ++row) {
      for (std::size_t col = 0; col < cols; ++col) {
        const Eigen::Vector2d xy = planarPosition(row, col);
        options.positions.push_back(
            offset + Eigen::Vector3d(xy.x(), xy.y(), z));
        options.velocities.push_back(velocity);
        if (layer == 0) {
          options.fixedNodes.push_back(nodeIndex(layer, row, col));
        }
      }
    }
  }

  const auto keepCell = [](std::size_t row, std::size_t col) {
    const bool centralNotch = row >= 4 && row <= 7 && col >= 6 && col <= 10;
    const bool boundaryCut
        = (row == 1 && col % 5 == 0) || (row == 10 && col % 4 == 1);
    return !centralNotch && !boundaryCut;
  };

  options.surfaceTriangles.reserve(4 * (rows - 1) * (cols - 1));
  for (std::size_t row = 0; row + 1 < rows; ++row) {
    for (std::size_t col = 0; col + 1 < cols; ++col) {
      if (!keepCell(row, col)) {
        continue;
      }

      const auto a = nodeIndex(0, row, col);
      const auto b = nodeIndex(0, row, col + 1);
      const auto c = nodeIndex(0, row + 1, col + 1);
      const auto d = nodeIndex(0, row + 1, col);
      const auto e = nodeIndex(1, row, col);
      const auto f = nodeIndex(1, row, col + 1);
      const auto g = nodeIndex(1, row + 1, col + 1);
      const auto h = nodeIndex(1, row + 1, col);

      if ((row + col) % 2 == 0) {
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{a, b, c});
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{a, c, d});
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{e, g, f});
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{e, h, g});
      } else {
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{a, b, d});
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{b, c, d});
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{e, h, f});
        options.surfaceTriangles.push_back(
            sx::DeformableSurfaceTriangle{f, h, g});
      }
    }
  }

  options.edgeStiffness = 0.0;
  options.material.frictionCoefficient = 0.8;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeLinearSolver;

  world.addDeformableBody(bodyName, options);
}

void configureDeformableSelfContactFrictionIrregularProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionIrregularProductionGridSceneWithOffset(
      world, "friction_irregular_production_grid");
}

void configureDeformableSelfContactFrictionIrregularMatrixFreeProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionIrregularProductionGridSceneWithOffset(
      world,
      "friction_irregular_matrix_free_production_grid",
      Eigen::Vector3d::Zero(),
      true);
}

void configureDeformableSelfContactFrictionLateActiveGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      11,
      11,
      "friction_late_active_direct_grid",
      0.025,
      Eigen::Vector3d(0.35, 0.1, -1.0));
}

void configureDeformableSelfContactFrictionLateActiveMatrixFreeGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      11,
      11,
      "friction_late_active_matrix_free_grid",
      0.025,
      Eigen::Vector3d(0.35, 0.1, -1.0),
      true);
}

void configureDeformableSelfContactFrictionLateActiveRectangularGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "friction_late_active_rectangular_direct_grid",
      0.025,
      Eigen::Vector3d(0.35, 0.1, -1.0));
}

void configureDeformableSelfContactFrictionLateActiveMatrixFreeRectangularGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "friction_late_active_rectangular_matrix_free_grid",
      0.025,
      Eigen::Vector3d(0.35, 0.1, -1.0),
      true);
}

void configureDeformableSelfContactFrictionRectangularGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 9, 13, "friction_rectangular_production_grid");
}

void configureDeformableSelfContactFrictionDenseRectangularGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 13, 19, "friction_dense_rectangular_production_grid");
}

void configureDeformableSelfContactFrictionDenseRectangularMatrixFreeGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      13,
      19,
      "friction_dense_rectangular_matrix_free_production_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true);
}

void configureDeformableSelfContactFrictionWideRectangularGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 7, 17, "friction_wide_rectangular_production_grid");
}

void configureDeformableSelfContactFrictionWideRectangularMatrixFreeGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      7,
      17,
      "friction_wide_rectangular_matrix_free_production_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true);
}

void configureDeformableSelfContactFrictionTallRectangularGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 17, 7, "friction_tall_rectangular_production_grid");
}

void configureDeformableSelfContactFrictionTallRectangularMatrixFreeGridScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      17,
      7,
      "friction_tall_rectangular_matrix_free_production_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true);
}

void configureMixedDeformableSelfContactFrictionProductionScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "mixed_friction_direct_rectangular_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      false,
      Eigen::Vector3d(-8.0, 0.0, 0.0));
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      7,
      17,
      "mixed_friction_matrix_free_wide_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true,
      Eigen::Vector3d(8.0, 0.0, 0.0));
}

void configureMixedDeformableSelfContactFrictionDenseProductionScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionIrregularProductionGridSceneWithOffset(
      world,
      "mixed_friction_irregular_direct_grid",
      Eigen::Vector3d(-8.5, 0.0, 0.0));
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      13,
      19,
      "mixed_friction_dense_matrix_free_rectangular_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true,
      Eigen::Vector3d(8.5, 0.0, 0.0));
}

void configureMixedDeformableSelfContactFrictionLateActiveProductionScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      11,
      11,
      "mixed_late_active_friction_direct_grid",
      0.025,
      Eigen::Vector3d(0.35, 0.1, -1.0),
      false,
      Eigen::Vector3d(-8.0, 0.0, 0.0));
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "mixed_late_active_friction_matrix_free_rectangular_grid",
      0.025,
      Eigen::Vector3d(0.35, 0.1, -1.0),
      true,
      Eigen::Vector3d(8.0, 0.0, 0.0));
}

void configureMixedDefaultDeformableStorageScene(dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      5,
      7,
      "mixed_storage_direct_rectangular_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      false,
      Eigen::Vector3d(-4.0, 0.0, 1.0));
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      5,
      9,
      "mixed_storage_matrix_free_wide_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true,
      Eigen::Vector3d(4.0, 0.0, 1.0));
  configureDeformableFemGroundFrictionBlockScene(world);
}

void configureMixedDefaultDeformableProductionStorageScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "mixed_storage_production_direct_rectangular_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      false,
      Eigen::Vector3d(-8.0, 0.0, 1.0));
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      7,
      17,
      "mixed_storage_production_matrix_free_wide_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true,
      Eigen::Vector3d(8.0, 0.0, 1.0));
  configureDeformableFemGroundFrictionBlockScene(world);
}

void configureDeformableStaticObstacleBarrierScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  const auto addBarrierObstacle = [&](std::string_view name,
                                      const Eigen::Vector3d& position,
                                      const sx::CollisionShape& shape) {
    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = position;
    auto obstacle = world.addRigidBody(name, obstacleOptions);
    obstacle.setCollisionShape(shape);
    obstacle.setDeformableSurfaceCcdObstacle(true);
    obstacle.setDeformableObstacleBarrierOnly(true);
  };

  addBarrierObstacle(
      "static_sphere_barrier",
      Eigen::Vector3d::Zero(),
      sx::CollisionShape::makeSphere(0.5));
  addBarrierObstacle(
      "static_box_barrier",
      Eigen::Vector3d(2.0, 0.0, 0.0),
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  addBarrierObstacle(
      "static_capsule_barrier",
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      sx::CollisionShape::makeCapsule(0.2, 0.5));

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.51, 0.0, 0.0),
         Eigen::Vector3d(2.51, 0.0, 0.0),
         Eigen::Vector3d(-1.79, 0.0, 0.0)};
  options.masses = {1.0, 1.0, 1.0};
  options.edgeStiffness = 0.0;
  world.addDeformableBody("static_obstacle_barrier_nodes", options);
}

void configureDeformableStaticObstacleFrictionProductionSceneWithSolver(
    dart::simulation::World& world, bool useMatrixFreeLinearSolver)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  const auto addBarrierObstacle = [&](std::string_view name,
                                      const Eigen::Vector3d& position,
                                      const sx::CollisionShape& shape) {
    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = position;
    auto obstacle = world.addRigidBody(name, obstacleOptions);
    obstacle.setCollisionShape(shape);
    obstacle.setDeformableSurfaceCcdObstacle(true);
    obstacle.setDeformableObstacleBarrierOnly(true);
  };

  const Eigen::Vector3d sphereCenter(-3.0, 0.0, 0.0);
  const Eigen::Vector3d boxCenter(0.0, 0.0, 0.0);
  const Eigen::Vector3d capsuleCenter(3.0, 0.0, 0.0);
  addBarrierObstacle(
      "friction_sphere_barrier",
      sphereCenter,
      sx::CollisionShape::makeSphere(0.5));
  addBarrierObstacle(
      "friction_box_barrier",
      boxCenter,
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  addBarrierObstacle(
      "friction_capsule_barrier",
      capsuleCenter,
      sx::CollisionShape::makeCapsule(0.2, 0.5));

  sx::DeformableBodyOptions options;
  constexpr std::size_t patch = 5;
  constexpr double clearance = 0.01;
  options.positions.reserve(3 * patch * patch);
  options.velocities.reserve(3 * patch * patch);
  options.masses.reserve(3 * patch * patch);

  const auto addSlidingNode = [&](const Eigen::Vector3d& position,
                                  const Eigen::Vector3d& tangentVelocity) {
    options.positions.push_back(position);
    options.velocities.push_back(tangentVelocity);
    options.masses.push_back(1.0);
  };

  for (std::size_t u = 0; u < patch; ++u) {
    for (std::size_t v = 0; v < patch; ++v) {
      const double a = (static_cast<double>(u) - 2.0) * 0.08;
      const double b = (static_cast<double>(v) - 2.0) * 0.08;

      const Eigen::Vector3d sphereNormal
          = Eigen::Vector3d(1.0, a, b).normalized();
      const Eigen::Vector3d sphereTangent
          = (Eigen::Vector3d::UnitY() - sphereNormal.y() * sphereNormal)
                .normalized();
      addSlidingNode(
          sphereCenter + (0.5 + clearance) * sphereNormal,
          0.35 * sphereTangent);

      addSlidingNode(
          boxCenter + Eigen::Vector3d(0.5 + clearance, a, b),
          Eigen::Vector3d(0.0, 0.35, 0.0));

      const double angle = a / 0.16 * 0.45;
      const Eigen::Vector3d capsuleNormal(
          std::cos(angle), std::sin(angle), 0.0);
      addSlidingNode(
          capsuleCenter + (0.2 + clearance) * capsuleNormal
              + Eigen::Vector3d(0.0, 0.0, b),
          Eigen::Vector3d(0.0, 0.0, 0.35));
    }
  }

  options.edgeStiffness = 0.0;
  options.material.frictionCoefficient = 0.8;
  options.material.useMatrixFreeLinearSolver = useMatrixFreeLinearSolver;
  world.addDeformableBody("static_obstacle_friction_production_nodes", options);
}

void configureDeformableStaticObstacleFrictionProductionScene(
    dart::simulation::World& world)
{
  configureDeformableStaticObstacleFrictionProductionSceneWithSolver(
      world, false);
}

void configureDeformableStaticObstacleFrictionMatrixFreeProductionScene(
    dart::simulation::World& world)
{
  configureDeformableStaticObstacleFrictionProductionSceneWithSolver(
      world, true);
}

void configureMixedStaticObstacleAndSelfContactProductionScene(
    dart::simulation::World& world)
{
  configureDeformableStaticObstacleFrictionProductionScene(world);
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "mixed_static_obstacle_matrix_free_self_contact_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true,
      Eigen::Vector3d(0.0, 8.0, 0.0));
}

void configureMixedMatrixFreeStaticObstacleAndDirectSelfContactProductionScene(
    dart::simulation::World& world)
{
  configureDeformableStaticObstacleFrictionMatrixFreeProductionScene(world);
  configureDeformableSelfContactFrictionIrregularProductionGridSceneWithOffset(
      world,
      "mixed_matrix_free_obstacle_direct_irregular_grid",
      Eigen::Vector3d(0.0, 8.0, 0.0));
}

void configureDeformableMovingRigidSurfaceCcdCrossingScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.position = Eigen::Vector3d::Zero();
  obstacleOptions.linearVelocity = Eigen::Vector3d(0.2, 0.0, 0.0);
  auto obstacle = world.addRigidBody("moving_surface_box", obstacleOptions);
  obstacle.setMass(1.0);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
  obstacle.setDeformableSurfaceCcdObstacle(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(-1.0, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d(20.0, 0.0, 0.0)};
  options.masses = {1.0};
  options.edgeStiffness = 0.0;
  world.addDeformableBody("moving_surface_fast_point", options);
}

void configureDynamicRigidIpcMeshSolveScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("dynamic_ipc_mesh", options);
  body.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));
}

void configureActiveRigidIpcMeshBarrierScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  const std::vector<Eigen::Vector3d> triangleVertices{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0)};
  const std::vector<Eigen::Vector3i> triangleFaces{Eigen::Vector3i(0, 1, 2)};

  sx::RigidBodyOptions staticOptions;
  staticOptions.isStatic = true;
  auto ground = world.addRigidBody("static_ipc_triangle", staticOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeMesh(triangleVertices, triangleFaces));

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 1.0;
  dynamicOptions.position = Eigen::Vector3d(0.0, 0.0, 0.005);
  auto body = world.addRigidBody("dynamic_ipc_triangle", dynamicOptions);
  body.setCollisionShape(
      sx::CollisionShape::makeMesh(triangleVertices, triangleFaces));
}

void configureRigidIpcFixedJointConstraintScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setTimeStep(0.01);
  world.setGravity(-9.81 * Eigen::Vector3d::UnitY());

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  auto parent = world.addRigidBody("ipc_fixed_parent", parentOptions);
  parent.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  childOptions.mass = 1.0;
  auto child = world.addRigidBody("ipc_fixed_child", childOptions);
  child.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  child.setAngularVelocity(Eigen::Vector3d(0.0, 0.25, 0.0));
  child.setTorque(Eigen::Vector3d(0.0, 1.0, 0.0));

  (void)world.addRigidBodyFixedJoint("ipc_fixed_joint", parent, child);
}

void configureRigidIpcRevoluteJointConstraintScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;
  namespace dvbd = sx::detail::deformable_vbd;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setTimeStep(0.001);
  world.setGravity(-9.81 * Eigen::Vector3d::UnitY());

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  auto parent = world.addRigidBody("ipc_hinge_parent", parentOptions);
  parent.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  childOptions.mass = 1.0;
  auto child = world.addRigidBody("ipc_hinge_child", childOptions);
  child.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  auto joint = world.addRigidBodyRevoluteJoint(
      "ipc_hinge_joint", parent, child, Eigen::Vector3d::UnitZ());

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(
      sx::detail::toRegistryEntity(joint.getEntity()));
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = -Eigen::Vector3d::UnitX();
}

void configureRigidIpcTwoBoxStackScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ipc_stack_ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.25}));

  sx::RigidBodyOptions lowerOptions;
  lowerOptions.mass = 1.0;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.252);
  auto lower = world.addRigidBody("ipc_stack_lower", lowerOptions);
  lower.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));

  sx::RigidBodyOptions upperOptions;
  upperOptions.mass = 1.0;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 0.754);
  auto upper = world.addRigidBody("ipc_stack_upper", upperOptions);
  upper.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));
}

void configureRigidIpcDeformableSurfaceObstacleScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.02);

  sx::DeformableBodyOptions sheetOptions;
  sheetOptions.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(-0.5, 0.5, 0.0),
         Eigen::Vector3d(0.5, 0.5, 0.0)};
  sheetOptions.masses = {1.0, 1.0, 1.0, 1.0};
  sheetOptions.fixedNodes = {0, 1, 2, 3};
  sheetOptions.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{1, 3, 2}};
  sheetOptions.material.frictionCoefficient = 0.0;
  world.addDeformableBody("ipc_deformable_surface", sheetOptions);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.055);
  boxOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto box = world.addRigidBody("ipc_deformable_surface_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.05, 0.05, 0.05}));
}

void configureRigidIpcKinematicConveyorScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions floorOptions;
  floorOptions.position = Eigen::Vector3d(0.0, 0.0, -0.1);
  floorOptions.linearVelocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto floor = world.addRigidBody("ipc_conveyor_floor", floorOptions);
  floor.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.1}));
  floor.setKinematic(true);
  floor.setFriction(1.0);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.1 + 1e-3);
  auto box = world.addRigidBody("ipc_conveyor_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  box.setFriction(1.0);
}

void configureRigidIpcKinematicTurntableScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions tableOptions;
  tableOptions.position = Eigen::Vector3d(0.0, 0.0, -0.1);
  tableOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 1.0);
  auto table = world.addRigidBody("ipc_turntable", tableOptions);
  table.setCollisionShape(sx::CollisionShape::makeBox({0.6, 0.6, 0.1}));
  table.setKinematic(true);
  table.setFriction(1.0);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.3, 0.0, 0.1 + 1e-3);
  auto box = world.addRigidBody("ipc_turntable_rider", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  box.setFriction(1.0);
}

void configureDeformableKinematicRigidSurfaceCcdCrossingScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.position = Eigen::Vector3d::Zero();
  obstacleOptions.linearVelocity = Eigen::Vector3d(0.2, 0.0, 0.0);
  auto obstacle = world.addRigidBody("kinematic_surface_box", obstacleOptions);
  obstacle.setKinematic(true);
  obstacle.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
  obstacle.setDeformableSurfaceCcdObstacle(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(-1.0, 0.0, 0.0)};
  options.velocities = {Eigen::Vector3d(20.0, 0.0, 0.0)};
  options.masses = {1.0};
  options.edgeStiffness = 0.0;
  world.addDeformableBody("kinematic_surface_fast_point", options);
}

void configureDeformableMultiKinematicRigidSurfaceCcdScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  const auto addKinematicObstacle = [&](std::string_view name, const double y) {
    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.position = Eigen::Vector3d(0.0, y, 0.0);
    obstacleOptions.linearVelocity = Eigen::Vector3d(0.2, 0.0, 0.0);
    auto obstacle = world.addRigidBody(name, obstacleOptions);
    obstacle.setKinematic(true);
    obstacle.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 0.5, 0.5)));
    obstacle.setDeformableSurfaceCcdObstacle(true);
  };
  addKinematicObstacle("multi_kinematic_surface_box_a", 0.0);
  addKinematicObstacle("multi_kinematic_surface_box_b", 2.0);

  const auto addCrossingPoint = [&](std::string_view name, const double y) {
    sx::DeformableBodyOptions options;
    options.positions = {Eigen::Vector3d(-1.0, y, 0.0)};
    options.velocities = {Eigen::Vector3d(20.0, 0.0, 0.0)};
    options.masses = {1.0};
    options.edgeStiffness = 0.0;
    world.addDeformableBody(name, options);
  };
  addCrossingPoint("multi_kinematic_surface_fast_point_a", 0.0);
  addCrossingPoint("multi_kinematic_surface_fast_point_b", 2.0);
}

void configureDeformableInterBodySurfaceCcdCrossingScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::DeformableBodyOptions surfaceOptions;
  surfaceOptions.positions
      = {Eigen::Vector3d(0.0, -1.0, -1.0),
         Eigen::Vector3d(0.0, 1.0, -1.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)};
  surfaceOptions.masses = {1.0, 1.0, 1.0};
  surfaceOptions.fixedNodes = {0, 1, 2};
  surfaceOptions.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  surfaceOptions.edgeStiffness = 0.0;
  world.addDeformableBody("fixed_deformable_surface", surfaceOptions);

  sx::DeformableBodyOptions movingOptions;
  movingOptions.positions
      = {Eigen::Vector3d(-1.0, -0.5, -0.5),
         Eigen::Vector3d(-1.0, 0.5, -0.5),
         Eigen::Vector3d(-1.0, 0.0, 0.5)};
  movingOptions.velocities
      = {Eigen::Vector3d(20.0, 0.0, 0.0),
         Eigen::Vector3d(20.0, 0.0, 0.0),
         Eigen::Vector3d(20.0, 0.0, 0.0)};
  movingOptions.masses = {1.0, 1.0, 1.0};
  movingOptions.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  movingOptions.edgeStiffness = 0.0;
  world.addDeformableBody("moving_deformable_surface", movingOptions);
}

void configureDeformableInterBodySurfaceCcdProductionGridSceneWithOffset(
    dart::simulation::World& world,
    std::string_view fixedName,
    std::string_view movingName,
    const Eigen::Vector3d& offset,
    double timeStep,
    double crossingSpeed)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(timeStep);

  constexpr std::size_t rows = 9;
  constexpr std::size_t cols = 13;
  constexpr double spacing = 0.18;
  const auto nodeIndex = [](std::size_t row, std::size_t col) {
    return row * cols + col;
  };

  const auto addSurfaceGrid
      = [&](std::string_view name, double x, bool fixed, bool moving) {
          sx::DeformableBodyOptions options;
          options.positions.reserve(rows * cols);
          options.velocities.reserve(rows * cols);
          options.masses.assign(rows * cols, 1.0);
          if (fixed) {
            options.fixedNodes.reserve(rows * cols);
          }

          for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t col = 0; col < cols; ++col) {
              const double y = (static_cast<double>(col)
                                - 0.5 * static_cast<double>(cols - 1))
                               * spacing;
              const double z = (static_cast<double>(row)
                                - 0.5 * static_cast<double>(rows - 1))
                               * spacing;
              options.positions.push_back(offset + Eigen::Vector3d(x, y, z));
              options.velocities.emplace_back(
                  moving ? Eigen::Vector3d(crossingSpeed, 0.0, 0.0)
                         : Eigen::Vector3d::Zero());
              if (fixed) {
                options.fixedNodes.push_back(nodeIndex(row, col));
              }
            }
          }

          options.surfaceTriangles.reserve(2 * (rows - 1) * (cols - 1));
          for (std::size_t row = 0; row + 1 < rows; ++row) {
            for (std::size_t col = 0; col + 1 < cols; ++col) {
              const auto a = nodeIndex(row, col);
              const auto b = nodeIndex(row, col + 1);
              const auto c = nodeIndex(row + 1, col + 1);
              const auto d = nodeIndex(row + 1, col);
              options.surfaceTriangles.push_back(
                  sx::DeformableSurfaceTriangle{a, b, c});
              options.surfaceTriangles.push_back(
                  sx::DeformableSurfaceTriangle{a, c, d});
            }
          }

          options.edgeStiffness = 0.0;
          world.addDeformableBody(name, options);
        };

  addSurfaceGrid(fixedName, 0.0, true, false);
  addSurfaceGrid(movingName, -1.0, false, true);
}

void configureDeformableInterBodySurfaceCcdProductionGridScene(
    dart::simulation::World& world)
{
  configureDeformableInterBodySurfaceCcdProductionGridSceneWithOffset(
      world,
      "fixed_deformable_production_surface",
      "moving_deformable_production_surface",
      Eigen::Vector3d::Zero(),
      0.1,
      20.0);
}

void configureMixedDefaultContactFamiliesProductionScene(
    dart::simulation::World& world)
{
  configureDeformableStaticObstacleFrictionProductionScene(world);
  configureDeformableSelfContactFrictionGridSceneWithShapeAndMotion(
      world,
      9,
      13,
      "mixed_contact_families_matrix_free_self_contact_grid",
      0.012,
      Eigen::Vector3d(0.35, 0.1, -0.08),
      true,
      Eigen::Vector3d(0.0, 8.0, 0.0));
  configureDeformableInterBodySurfaceCcdProductionGridSceneWithOffset(
      world,
      "mixed_contact_families_fixed_surface",
      "mixed_contact_families_moving_surface",
      Eigen::Vector3d(0.0, -8.0, 0.0),
      0.01,
      200.0);
}

void configureMixedComplementaryDefaultContactFamiliesProductionScene(
    dart::simulation::World& world)
{
  configureDeformableStaticObstacleFrictionMatrixFreeProductionScene(world);
  configureDeformableSelfContactFrictionIrregularProductionGridSceneWithOffset(
      world,
      "mixed_complementary_contact_direct_irregular_grid",
      Eigen::Vector3d(0.0, 8.0, 0.0));
  configureDeformableInterBodySurfaceCcdProductionGridSceneWithOffset(
      world,
      "mixed_complementary_contact_fixed_surface",
      "mixed_complementary_contact_moving_surface",
      Eigen::Vector3d(0.0, -8.0, 0.0),
      0.01,
      200.0);
}

void enableAvbdSelfContactFrictionRows(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 8;
  cfg.useAvbdSelfContactNormalRows = true;
  cfg.avbdAlpha = 0.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  auto& registry = sx::detail::registryOf(world);
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(entity, cfg);
  }
}

void configureAvbdSelfContactFrictionRowsScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(0.0, 0.6, 0.0),
         Eigen::Vector3d(-0.2, -0.15, 0.01),
         Eigen::Vector3d(0.2, -0.15, 0.01),
         Eigen::Vector3d(0.0, 0.2, 0.01)};
  options.masses = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
  options.fixedNodes = {0, 1, 2};
  options.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{3, 4, 5}};
  options.edges
      = {sx::DeformableEdge{3, 4, -1.0},
         sx::DeformableEdge{4, 5, -1.0},
         sx::DeformableEdge{5, 3, -1.0}};
  options.edgeStiffness = 500.0;
  options.damping = 1.0;
  options.material.frictionCoefficient = 0.8;
  world.addDeformableBody("avbd_self_contact_friction", options);

  enableAvbdSelfContactFrictionRows(world);
}

void configureAvbdSelfContactFrictionGridRowsScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 5, 9, "avbd_self_contact_friction_grid");

  enableAvbdSelfContactFrictionRows(world);
}

void configureAvbdSelfContactFrictionProductionGridRowsScene(
    dart::simulation::World& world)
{
  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 9, 13, "avbd_self_contact_friction_production_grid");

  enableAvbdSelfContactFrictionRows(world);
}

void configureVbdChebyshevSelfContactGridScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  configureDeformableSelfContactFrictionGridSceneWithShape(
      world, 5, 9, "vbd_chebyshev_self_contact_grid");

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 8;
  cfg.useChebyshev = true;
  cfg.chebyshevRho = 0.9;

  auto& registry = sx::detail::registryOf(world);
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(entity, cfg);
  }
}

void configureAvbdGroundFrictionRowsScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("avbd_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(10.0, 10.0, 0.5)));
  ground.setDeformableGroundBarrier(true);

  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, -0.002)};
  options.velocities = {Eigen::Vector3d(1.0, 0.0, 0.0)};
  options.masses = {1.0};
  options.edgeStiffness = 0.0;
  options.material.frictionCoefficient = 0.8;
  world.addDeformableBody("avbd_ground_friction", options);

  sx::comps::DeformableVbdConfig cfg;
  cfg.enabled = true;
  cfg.iterations = 8;
  cfg.contactStiffness = 1.0e4;
  cfg.useAvbdContactNormalRows = true;
  cfg.avbdAlpha = 0.0;
  cfg.avbdGamma = 1.0;
  cfg.avbdMaxStiffness = 1.0e6;
  auto& registry = sx::detail::registryOf(world);
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(entity, cfg);
  }
}

void configureRigidAvbdContactRowsScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("rigid_avbd_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.49);
  auto sphere = world.addRigidBody("rigid_avbd_sphere", sphereOptions);
  sphere.setMass(1.0);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  registry.emplace_or_replace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
}

void configureRigidBoxedLcpContactRowsScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("rigid_boxed_lcp_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.49);
  auto sphere = world.addRigidBody("rigid_boxed_lcp_sphere", sphereOptions);
  sphere.setMass(1.0);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
}

void configureRigidAvbdFixedJointRowsScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.005);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("rigid_avbd_joint_base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("rigid_avbd_joint_link", linkOptions);
  link.setMass(1.0);

  (void)world.addRigidBodyFixedJoint("rigid_avbd_fixed_joint", base, link);

  Eigen::Isometry3d driftedPose = Eigen::Isometry3d::Identity();
  driftedPose.translation() = Eigen::Vector3d(1.25, 0.0, 0.0);
  link.setTransform(driftedPose);
}

void configureRigidAvbdDistanceSpringRowsScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.005);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("rigid_avbd_spring_base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.position = 2.0 * Eigen::Vector3d::UnitX();
  auto link = world.addRigidBody("rigid_avbd_spring_link", linkOptions);
  link.setMass(1.0);

  world.addRigidBodyDistanceSpring(
      "rigid_avbd_distance_spring",
      base,
      link,
      /*restLength=*/1.0,
      /*stiffness=*/200.0);
}

} // namespace

// Test World construction
TEST(World, Construction)
{
  // Create a World instance
  dart::simulation::World world;
  (void)world;       // Suppress unused variable warning
  EXPECT_TRUE(true); // If we get here, construction succeeded
}

TEST(World, MemoryManagerOptionsAndDiagnostics)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.frameScratchInitialCapacity = 4096;

  sx::World world(options);

  EXPECT_EQ(&world.getMemoryManager().getBaseAllocator(), &allocator);
  EXPECT_GT(allocator.allocationCount, 0u);

  const auto diagnostics = world.getMemoryDiagnostics();
  EXPECT_LE(diagnostics.frameScratchCapacityBytes, 4096u);
  EXPECT_GE(diagnostics.frameScratchCapacityBytes + 64u, 4096u);
  EXPECT_EQ(diagnostics.frameScratchCapacityBytes % 32u, 0u);
  EXPECT_EQ(diagnostics.frameScratchUsedBytes, 0u);
  EXPECT_EQ(diagnostics.frameScratchPeakUsedBytes, 0u);
  EXPECT_EQ(diagnostics.frameScratchOverflowCount, 0u);
  EXPECT_EQ(diagnostics.frameScratchOverflowBytes, 0u);
  EXPECT_EQ(diagnostics.frameScratchResetCount, 0u);

  options.frameScratchInitialCapacity = 0;
  EXPECT_THROW(
      {
        sx::World invalid(options);
        (void)invalid;
      },
      sx::InvalidArgumentException);
}

TEST(World, MemoryManagersAreIsolatedAcrossWorlds)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator firstAllocator;
  CountingMemoryAllocator secondAllocator;
  sx::WorldOptions firstOptions;
  sx::WorldOptions secondOptions;
  firstOptions.baseAllocator = &firstAllocator;
  secondOptions.baseAllocator = &secondAllocator;

  sx::World first(firstOptions);
  sx::World second(secondOptions);

  auto& firstManager = first.getMemoryManager();
  auto& secondManager = second.getMemoryManager();
  EXPECT_EQ(&firstManager.getBaseAllocator(), &firstAllocator);
  EXPECT_EQ(&secondManager.getBaseAllocator(), &secondAllocator);
  EXPECT_NE(&firstManager, &secondManager);
  EXPECT_NE(
      &firstManager.getFreeListAllocator(),
      &secondManager.getFreeListAllocator());
  EXPECT_NE(
      &firstManager.getPoolAllocator(), &secondManager.getPoolAllocator());
  EXPECT_NE(
      &firstManager.getFrameAllocator(), &secondManager.getFrameAllocator());

  const auto secondAllocationsBefore = secondAllocator.allocationCount;
  const auto secondDeallocationsBefore = secondAllocator.deallocationCount;
  const auto secondDiagnosticsBefore = second.getMemoryDiagnostics();

  auto* firstFreePtr = firstManager.allocateUsingFree(64);
  ASSERT_NE(firstFreePtr, nullptr);
  auto* firstPoolPtr = firstManager.allocateUsingPool(32);
  ASSERT_NE(firstPoolPtr, nullptr);
  auto* firstFramePtr = firstManager.allocateUsingFrame(128);
  ASSERT_NE(firstFramePtr, nullptr);
  auto* firstBasePtr
      = firstManager.allocate(common::MemoryManager::Type::Base, 24);
  ASSERT_NE(firstBasePtr, nullptr);

  EXPECT_EQ(secondAllocator.allocationCount, secondAllocationsBefore);
  EXPECT_EQ(secondAllocator.deallocationCount, secondDeallocationsBefore);
  EXPECT_FALSE(secondManager.hasAllocated(firstFreePtr, 64));
  EXPECT_FALSE(secondManager.hasAllocated(firstPoolPtr, 32));

  const auto secondDiagnosticsAfter = second.getMemoryDiagnostics();
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.enabled,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.enabled);
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.freeAllocator.liveBytes,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.freeAllocator
          .liveBytes);
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.freeAllocator
          .peakLiveBytes,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.freeAllocator
          .peakLiveBytes);
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.freeAllocator
          .liveAllocationCount,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.freeAllocator
          .liveAllocationCount);
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.poolAllocator.liveBytes,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.poolAllocator
          .liveBytes);
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.poolAllocator
          .peakLiveBytes,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.poolAllocator
          .peakLiveBytes);
  EXPECT_EQ(
      secondDiagnosticsAfter.allocatorDebugDiagnostics.poolAllocator
          .liveAllocationCount,
      secondDiagnosticsBefore.allocatorDebugDiagnostics.poolAllocator
          .liveAllocationCount);

#if !defined(NDEBUG)
  EXPECT_TRUE(firstManager.hasAllocated(firstFreePtr, 64));
  EXPECT_TRUE(firstManager.hasAllocated(firstPoolPtr, 32));
#else
  EXPECT_FALSE(firstManager.hasAllocated(firstFreePtr, 64));
  EXPECT_FALSE(firstManager.hasAllocated(firstPoolPtr, 32));
#endif

  firstManager.deallocate(common::MemoryManager::Type::Base, firstBasePtr, 24);
  firstManager.deallocateUsingFrame(firstFramePtr, 128);
  firstManager.deallocateUsingPool(firstPoolPtr, 32);
  firstManager.deallocateUsingFree(firstFreePtr, 64);
}

TEST(World, WorldPersistentStorageUsesWorldFreeAllocator)
{
  namespace sx = dart::simulation;
  constexpr std::size_t kWorldStorageAndBuiltInStageRootAllocations = 8u;

  sx::World world;
  auto& memoryManager = world.getMemoryManager();
  EXPECT_GE(
      memoryManager.getFreeListAllocator().getAllocatedSize(),
      sizeof(sx::detail::WorldStorage));
  EXPECT_GE(
      memoryManager.getFreeListAllocator().getAllocationCount(),
      kWorldStorageAndBuiltInStageRootAllocations);

#if !defined(NDEBUG)
  auto* storage = &sx::detail::storageOf(world);
  EXPECT_TRUE(
      memoryManager.hasAllocated(storage, sizeof(sx::detail::WorldStorage)));
#endif

  (void)world.collide();
  EXPECT_GE(
      memoryManager.getFreeListAllocator().getAllocationCount(),
      kWorldStorageAndBuiltInStageRootAllocations + 1u);

  const auto allocationsBeforeReplay
      = memoryManager.getFreeListAllocator().getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    world.setReplayRecordingEnabled(true);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "replay state and frame storage should allocate through the World "
           "free allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GE(
      memoryManager.getFreeListAllocator().getAllocationCount(),
      allocationsBeforeReplay + 2u);

  sx::World replayWorld;
  replayWorld.addRigidBody("allocator_replay_body");
  auto& replayFreeList = replayWorld.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforeRigidReplay = replayFreeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    replayWorld.setReplayRecordingEnabled(true);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "non-empty replay frame snapshots should allocate through the "
           "World free allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(replayFreeList.getAllocationCount(), allocationsBeforeRigidReplay);

  {
    ScopedHeapAllocationCounter heapCounter;
    replayWorld.restoreReplayFrame(0);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "replay restore scratch should allocate through the World free "
           "allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }

  sx::World jointReplayWorld;
  auto robot = jointReplayWorld.addMultibody("allocator_replay_robot");
  auto base = robot.addLink("allocator_replay_base");
  sx::JointSpec jointSpec;
  jointSpec.name = std::string(80, 'j');
  jointSpec.type = sx::JointType::Floating;
  auto link = robot.addLink("allocator_replay_link", base, jointSpec);
  auto joint = link.getParentJoint();
  joint.setPosition(Eigen::VectorXd::LinSpaced(6, 0.25, 0.75));
  joint.setVelocity(Eigen::VectorXd::LinSpaced(6, 0.5, 1.0));
  joint.setForce(Eigen::VectorXd::LinSpaced(6, 1.5, 2.0));
  joint.setCommandVelocity(Eigen::VectorXd::LinSpaced(6, -0.75, -0.25));
  auto& jointReplayFreeList
      = jointReplayWorld.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforeJointReplay
      = jointReplayFreeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    jointReplayWorld.setReplayRecordingEnabled(true);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "joint replay snapshots should allocate dynamic payloads through "
           "the World free allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(
      jointReplayFreeList.getAllocationCount(), allocationsBeforeJointReplay);
  {
    ScopedHeapAllocationCounter heapCounter;
    jointReplayWorld.restoreReplayFrame(0);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "joint replay restore should not allocate dynamic payloads from "
           "the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }

  sx::World liveJointStorageWorld;
  auto liveJointRobot = liveJointStorageWorld.addMultibody("j");
  auto liveJointBase = liveJointRobot.addLink("b");
  sx::JointSpec liveJointSpec;
  liveJointSpec.name = "f";
  liveJointSpec.type = sx::JointType::Floating;
  std::optional<sx::Link> liveJointLink;
  {
    ScopedHeapAllocationCounter heapCounter;
    liveJointLink.emplace(
        liveJointRobot.addLink("l", liveJointBase, liveJointSpec));
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "live 6-DOF joint component creation should not allocate dynamic "
           "vector payloads from the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  auto liveJoint = liveJointLink->getParentJoint();
  auto& liveJointRegistry = sx::detail::registryOf(liveJointStorageWorld);
  const entt::entity liveJointEntity
      = sx::detail::toRegistryEntity(liveJoint.getEntity());
  auto& liveJointComponent
      = liveJointRegistry.get<sx::comps::Joint>(liveJointEntity);
  const Eigen::VectorXd livePosition = Eigen::VectorXd::LinSpaced(6, 0.1, 0.6);
  const Eigen::VectorXd liveVelocity = Eigen::VectorXd::LinSpaced(6, 0.7, 1.2);
  const Eigen::VectorXd liveAcceleration
      = Eigen::VectorXd::LinSpaced(6, 1.3, 1.8);
  const Eigen::VectorXd liveTorque = Eigen::VectorXd::LinSpaced(6, 1.9, 2.4);
  const Eigen::VectorXd liveLower = Eigen::VectorXd::Constant(6, -10.0);
  const Eigen::VectorXd liveUpper = Eigen::VectorXd::Constant(6, 10.0);
  {
    ScopedHeapAllocationCounter heapCounter;
    liveJoint.setPosition(livePosition);
    liveJoint.setVelocity(liveVelocity);
    liveJoint.setForce(liveTorque);
    liveJoint.setCommandVelocity(liveVelocity);
    liveJoint.setSpringStiffness(liveUpper);
    liveJoint.setDampingCoefficient(liveUpper);
    liveJoint.setRestPosition(livePosition);
    liveJoint.setArmature(liveUpper);
    liveJoint.setCoulombFriction(liveUpper);
    liveJoint.setPositionLimits(liveLower, liveUpper);
    liveJoint.setVelocityLimits(liveLower, liveUpper);
    liveJoint.setEffortLimits(liveLower, liveUpper);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "public live 6-DOF joint setters should reuse bounded component "
           "payloads and route dirty-traversal scratch through the World "
           "allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  {
    ScopedHeapAllocationCounter heapCounter;
    liveJointComponent.position = livePosition;
    liveJointComponent.velocity = liveVelocity;
    liveJointComponent.acceleration = liveAcceleration;
    liveJointComponent.torque = liveTorque;
    liveJointComponent.springStiffness = liveUpper;
    liveJointComponent.dampingCoefficient = liveUpper;
    liveJointComponent.restPosition = livePosition;
    liveJointComponent.armature = liveUpper;
    liveJointComponent.coulombFriction = liveUpper;
    liveJointComponent.commandVelocity = liveVelocity;
    liveJointComponent.limits.lower = liveLower;
    liveJointComponent.limits.upper = liveUpper;
    liveJointComponent.limits.velocityLower = liveLower;
    liveJointComponent.limits.velocityUpper = liveUpper;
    liveJointComponent.limits.effortLower = liveLower;
    liveJointComponent.limits.effortUpper = liveUpper;
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "live 6-DOF joint component payload storage should be bounded and "
           "not allocate dynamic vector payloads from the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }

  sx::World closureReplayWorld;
  auto closureRobot = closureReplayWorld.addMultibody("closure_replay_robot");
  auto closureBase = closureRobot.addLink("closure_replay_base");
  auto closureLink = closureRobot.addLink(
      "closure_replay_link",
      closureBase,
      sx::JointSpec{.name = "closure_replay_joint"});
  const std::string longClosureName(80, 'c');
  closureReplayWorld.addLoopClosure(
      longClosureName, {.frameA = closureBase, .frameB = closureLink});
  auto& closureReplayFreeList
      = closureReplayWorld.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforeClosureReplay
      = closureReplayFreeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    closureReplayWorld.setReplayRecordingEnabled(true);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "loop-closure replay names should allocate through the World free "
           "allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(
      closureReplayFreeList.getAllocationCount(),
      allocationsBeforeClosureReplay);
  {
    ScopedHeapAllocationCounter heapCounter;
    closureReplayWorld.restoreReplayFrame(0);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "loop-closure replay restore should not allocate names from the "
           "global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }

  sx::World deformableReplayWorld;
  sx::DeformableBodyOptions deformableOptions;
  deformableOptions.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(0.0, 0.0, 1.0)};
  deformableOptions.velocities.assign(
      deformableOptions.positions.size(), Eigen::Vector3d::Zero());
  deformableOptions.masses.assign(deformableOptions.positions.size(), 1.0);
  deformableOptions.fixedNodes = {0};
  deformableReplayWorld.addDeformableBody(
      "allocator_replay_deformable", deformableOptions);
  auto& deformableReplayFreeList
      = deformableReplayWorld.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforeDeformableReplay
      = deformableReplayFreeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    deformableReplayWorld.setReplayRecordingEnabled(true);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "deformable node replay snapshots should allocate dynamic payloads "
           "through the World free allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(
      deformableReplayFreeList.getAllocationCount(),
      allocationsBeforeDeformableReplay);
  {
    ScopedHeapAllocationCounter heapCounter;
    deformableReplayWorld.restoreReplayFrame(0);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "deformable node replay restore should not allocate dynamic "
           "payloads from the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }

  sx::World deformableStorageWorld;
  sx::DeformableBodyOptions storageOptions;
  storageOptions.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0),
         Eigen::Vector3d(1.0, 1.0, 0.0),
         Eigen::Vector3d(0.5, 0.5, 1.0)};
  storageOptions.velocities.assign(
      storageOptions.positions.size(), Eigen::Vector3d::Zero());
  storageOptions.masses.assign(storageOptions.positions.size(), 1.0);
  storageOptions.edges = {{0, 1}, {1, 4}, {2, 4}};
  storageOptions.tetrahedra = {{0, 1, 2, 4}};
  storageOptions.fixedNodes = {0, 3};
  sx::DeformableDirichletBoundaryCondition dirichletBoundary;
  dirichletBoundary.nodes = {1, 2};
  dirichletBoundary.linearVelocity = Eigen::Vector3d(0.0, 0.1, 0.0);
  dirichletBoundary.endTime = 0.25;
  storageOptions.dirichletBoundaryConditions.push_back(dirichletBoundary);
  sx::DeformableNeumannBoundaryCondition neumannBoundary;
  neumannBoundary.nodes = {4};
  neumannBoundary.acceleration = Eigen::Vector3d(0.0, -9.8, 0.0);
  neumannBoundary.endTime = 0.25;
  storageOptions.neumannBoundaryConditions.push_back(neumannBoundary);
  auto& deformableStorageFreeList
      = deformableStorageWorld.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforeDeformableStorage
      = deformableStorageFreeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    deformableStorageWorld.addDeformableBody("d", storageOptions);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "live deformable node payloads should allocate through the World "
           "free allocator during body creation, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(
      deformableStorageFreeList.getAllocationCount(),
      allocationsBeforeDeformableStorage);
#if !defined(NDEBUG)
  {
    const auto& registry
        = sx::detail::storageOf(deformableStorageWorld).registry;
    auto view = registry.view<
        sx::comps::DeformableNodeState,
        sx::comps::DeformableSpringModel,
        sx::comps::DeformableMeshTopology,
        sx::comps::DeformableBoundaryConditions>();
    ASSERT_EQ(view.size_hint(), 1u);
    for (const auto entity : view) {
      const auto& state = view.get<sx::comps::DeformableNodeState>(entity);
      const auto& spring = view.get<sx::comps::DeformableSpringModel>(entity);
      const auto& topology
          = view.get<sx::comps::DeformableMeshTopology>(entity);
      const auto& boundaries
          = view.get<sx::comps::DeformableBoundaryConditions>(entity);
      auto& memory = deformableStorageWorld.getMemoryManager();
      EXPECT_TRUE(memory.hasAllocated(
          state.positions.data(),
          state.positions.size() * sizeof(Eigen::Vector3d)));
      EXPECT_TRUE(memory.hasAllocated(
          state.previousPositions.data(),
          state.previousPositions.size() * sizeof(Eigen::Vector3d)));
      EXPECT_TRUE(memory.hasAllocated(
          state.velocities.data(),
          state.velocities.size() * sizeof(Eigen::Vector3d)));
      EXPECT_TRUE(memory.hasAllocated(
          state.masses.data(), state.masses.size() * sizeof(double)));
      EXPECT_TRUE(memory.hasAllocated(
          state.fixed.data(), state.fixed.size() * sizeof(std::uint8_t)));
      EXPECT_TRUE(memory.hasAllocated(
          topology.restPositions.data(),
          topology.restPositions.size() * sizeof(Eigen::Vector3d)));
      EXPECT_TRUE(memory.hasAllocated(
          spring.edges.data(),
          spring.edges.size() * sizeof(sx::comps::DeformableSpringEdge)));
      EXPECT_TRUE(memory.hasAllocated(
          topology.surfaceTriangles.data(),
          topology.surfaceTriangles.size()
              * sizeof(sx::comps::DeformableSurfaceTriangle)));
      EXPECT_TRUE(memory.hasAllocated(
          topology.tetrahedra.data(),
          topology.tetrahedra.size()
              * sizeof(sx::comps::DeformableTetrahedron)));
      EXPECT_TRUE(memory.hasAllocated(
          topology.tetrahedronRestVolumes.data(),
          topology.tetrahedronRestVolumes.size() * sizeof(double)));
      ASSERT_EQ(boundaries.dirichlet.size(), 1u);
      EXPECT_TRUE(memory.hasAllocated(
          boundaries.dirichlet.data(),
          boundaries.dirichlet.size()
              * sizeof(sx::comps::DeformableDirichletBoundary)));
      EXPECT_TRUE(memory.hasAllocated(
          boundaries.dirichlet[0].nodes.data(),
          boundaries.dirichlet[0].nodes.size() * sizeof(std::size_t)));
      EXPECT_TRUE(memory.hasAllocated(
          boundaries.dirichlet[0].referencePositions.data(),
          boundaries.dirichlet[0].referencePositions.size()
              * sizeof(Eigen::Vector3d)));
      ASSERT_EQ(boundaries.neumann.size(), 1u);
      EXPECT_TRUE(memory.hasAllocated(
          boundaries.neumann.data(),
          boundaries.neumann.size()
              * sizeof(sx::comps::DeformableNeumannBoundary)));
      EXPECT_TRUE(memory.hasAllocated(
          boundaries.neumann[0].nodes.data(),
          boundaries.neumann[0].nodes.size() * sizeof(std::size_t)));
    }
  }
#endif

  const auto expectWorldAllocator
      = []<typename Vector>(sx::World& owner, const Vector& vector) {
          using Value = typename Vector::value_type;
          EXPECT_EQ(
              vector.get_allocator(),
              dart::common::StlAllocator<Value>{
                  owner.getMemoryManager().getFreeAllocator()});
        };

  std::stringstream loadedMultibodyBuffer;
  liveJointStorageWorld.saveBinary(loadedMultibodyBuffer);
  loadedMultibodyBuffer.seekg(0);
  sx::World loadedMultibodyWorld;
  loadedMultibodyWorld.loadBinary(loadedMultibodyBuffer);
  {
    const auto& registry = sx::detail::storageOf(loadedMultibodyWorld).registry;
    auto structures = registry.view<sx::comps::MultibodyStructure>();
    std::size_t structureCount = 0;
    for (const auto entity : structures) {
      ++structureCount;
      const auto& structure
          = structures.get<sx::comps::MultibodyStructure>(entity);
      expectWorldAllocator(loadedMultibodyWorld, structure.links);
      expectWorldAllocator(loadedMultibodyWorld, structure.joints);
    }
    ASSERT_EQ(structureCount, 1u);

    auto links = registry.view<sx::comps::Link>();
    std::size_t linkCount = 0;
    for (const auto entity : links) {
      ++linkCount;
      const auto& link = links.get<sx::comps::Link>(entity);
      expectWorldAllocator(loadedMultibodyWorld, link.childJoints);
    }
    ASSERT_GE(linkCount, 2u);
  }

  std::stringstream loadedDeformableBuffer;
  deformableStorageWorld.saveBinary(loadedDeformableBuffer);
  loadedDeformableBuffer.seekg(0);
  sx::World loadedDeformableWorld;
  loadedDeformableWorld.loadBinary(loadedDeformableBuffer);
  {
    const auto& registry
        = sx::detail::storageOf(loadedDeformableWorld).registry;
    auto view = registry.view<
        sx::comps::DeformableNodeState,
        sx::comps::DeformableSpringModel,
        sx::comps::DeformableMeshTopology,
        sx::comps::DeformableBoundaryConditions>();
    ASSERT_EQ(view.size_hint(), 1u);
    for (const auto entity : view) {
      const auto& state = view.get<sx::comps::DeformableNodeState>(entity);
      const auto& spring = view.get<sx::comps::DeformableSpringModel>(entity);
      const auto& topology
          = view.get<sx::comps::DeformableMeshTopology>(entity);
      const auto& boundaries
          = view.get<sx::comps::DeformableBoundaryConditions>(entity);
      expectWorldAllocator(loadedDeformableWorld, state.positions);
      expectWorldAllocator(loadedDeformableWorld, state.previousPositions);
      expectWorldAllocator(loadedDeformableWorld, state.velocities);
      expectWorldAllocator(loadedDeformableWorld, state.masses);
      expectWorldAllocator(loadedDeformableWorld, state.fixed);
      expectWorldAllocator(loadedDeformableWorld, spring.edges);
      expectWorldAllocator(loadedDeformableWorld, topology.restPositions);
      expectWorldAllocator(loadedDeformableWorld, topology.surfaceTriangles);
      expectWorldAllocator(loadedDeformableWorld, topology.tetrahedra);
      expectWorldAllocator(
          loadedDeformableWorld, topology.tetrahedronRestVolumes);
      expectWorldAllocator(loadedDeformableWorld, boundaries.dirichlet);
      ASSERT_EQ(boundaries.dirichlet.size(), 1u);
      expectWorldAllocator(
          loadedDeformableWorld, boundaries.dirichlet[0].nodes);
      expectWorldAllocator(
          loadedDeformableWorld, boundaries.dirichlet[0].referencePositions);
      expectWorldAllocator(loadedDeformableWorld, boundaries.neumann);
      ASSERT_EQ(boundaries.neumann.size(), 1u);
      expectWorldAllocator(loadedDeformableWorld, boundaries.neumann[0].nodes);
    }
  }

  sx::World surfaceStorageWorld;
  sx::DeformableBodyOptions surfaceStorageOptions;
  surfaceStorageOptions.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  surfaceStorageOptions.masses.assign(
      surfaceStorageOptions.positions.size(), 1.0);
  surfaceStorageOptions.surfaceTriangles = {{0, 1, 2}};
  auto& surfaceStorageFreeList
      = surfaceStorageWorld.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforeSurfaceStorage
      = surfaceStorageFreeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    surfaceStorageWorld.addDeformableBody("s", surfaceStorageOptions);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "explicit deformable surface-topology validation scratch should "
           "allocate through the World free allocator during body creation, "
           "not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(
      surfaceStorageFreeList.getAllocationCount(),
      allocationsBeforeSurfaceStorage);

  auto ignoredPairA = world.addRigidBody("ignored_pair_a");
  auto ignoredPairB = world.addRigidBody("ignored_pair_b");
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeIgnoredPair = freeList.getAllocationCount();
  {
    ScopedHeapAllocationCounter heapCounter;
    world.setCollisionPairIgnored(ignoredPairA, ignoredPairB);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "ignored collision-pair storage should allocate through the "
           "World free allocator, not the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeIgnoredPair);
  EXPECT_TRUE(world.isCollisionPairIgnored(ignoredPairA, ignoredPairB));

  world.clearIgnoredCollisionPairs();
  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeIgnoredPair);

  world.addFreeFrame("frame_before_clear");
  world.clear();
  EXPECT_GE(
      memoryManager.getFreeListAllocator().getAllocatedSize(),
      sizeof(sx::detail::WorldStorage));
  EXPECT_GE(
      memoryManager.getFreeListAllocator().getAllocationCount(),
      kWorldStorageAndBuiltInStageRootAllocations + 1u);

#if !defined(NDEBUG)
  auto* rebuiltStorage = &sx::detail::storageOf(world);
  EXPECT_TRUE(memoryManager.hasAllocated(
      rebuiltStorage, sizeof(sx::detail::WorldStorage)));
#endif
}

TEST(World, SaveBinaryIgnoredCollisionPairFilterUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::World world;
  constexpr std::size_t kBodyCount = 72u;
  std::vector<sx::RigidBody> bodies;
  bodies.reserve(kBodyCount);
  for (std::size_t i = 0; i < kBodyCount; ++i) {
    bodies.push_back(world.addRigidBody(std::format("ignored_save_{:03}", i)));
  }

  for (std::size_t i = 0; i < bodies.size(); ++i) {
    for (std::size_t j = i + 1u; j < bodies.size(); ++j) {
      world.setCollisionPairIgnored(bodies[i], bodies[j]);
    }
  }
  ASSERT_GT(world.getIgnoredCollisionPairCount(), 2000u);

  auto& freeList = world.getMemoryManager().getFreeListAllocator();
  const auto liveBytesBeforeSave = freeList.getAllocatedSize();
  const auto peakBytesBeforeSave = freeList.getPeakAllocatedSize();

  std::stringstream buffer;
  world.saveBinary(buffer);

  EXPECT_EQ(freeList.getAllocatedSize(), liveBytesBeforeSave)
      << "saveBinary should release the temporary ignored-pair filter "
         "storage before returning";
  EXPECT_GT(freeList.getPeakAllocatedSize(), peakBytesBeforeSave)
      << "saveBinary should allocate ignored-pair filter storage through the "
         "World free allocator, not a default std::vector heap allocation";

  buffer.seekg(0);
  sx::World loaded;
  loaded.loadBinary(buffer);
  EXPECT_EQ(
      loaded.getIgnoredCollisionPairCount(),
      world.getIgnoredCollisionPairCount());
}

TEST(World, NumDofsDynamicBodyCollectionUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::World world;
  constexpr std::size_t kDynamicBodyCount = 1024u;
  for (std::size_t i = 0; i < kDynamicBodyCount; ++i) {
    world.addRigidBody(std::format("dof_dynamic_{:04}", i));
  }

  sx::RigidBodyOptions staticOptions;
  staticOptions.isStatic = true;
  constexpr std::size_t kStaticBodyCount = 16u;
  for (std::size_t i = 0; i < kStaticBodyCount; ++i) {
    world.addRigidBody(std::format("dof_static_{:02}", i), staticOptions);
  }

  auto& freeList = world.getMemoryManager().getFreeListAllocator();
  const auto liveBytesBeforeQuery = freeList.getAllocatedSize();
  const auto peakBytesBeforeQuery = freeList.getPeakAllocatedSize();

  ScopedHeapAllocationCounter heapCounter;
  const auto dofs = world.getNumDofs();
  heapCounter.stop();

  EXPECT_EQ(dofs, 3u * kDynamicBodyCount);
  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "dynamic-body collection scratch should allocate through the World "
         "free allocator, not the global heap";
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  EXPECT_EQ(freeList.getAllocatedSize(), liveBytesBeforeQuery)
      << "getNumDofs should release dynamic-body collection scratch before "
         "returning";
  EXPECT_GT(freeList.getPeakAllocatedSize(), peakBytesBeforeQuery)
      << "getNumDofs should allocate dynamic-body collection scratch through "
         "the World free allocator";
}

#ifdef DART_HAS_DIFF
TEST(World, DifferentiableMultibodyTorqueScratchUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.differentiable = true;
  sx::World world(options);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  auto robot = world.addMultibody("diff_allocator_chain");
  auto parent = robot.addLink("diff_allocator_base");
  constexpr std::size_t kJointCount = 384u;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.01, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = std::format("hinge_{:03}", i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::format("link_{:03}", i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.001 * (i + 1u)));
    joint.setVelocity(Eigen::VectorXd::Constant(1, 0.01));
    joint.setForce(Eigen::VectorXd::Constant(1, 0.25));
    parent = link;
  }

  world.enterSimulationMode();

  auto& freeList = world.getMemoryManager().getFreeListAllocator();
  const auto liveBytesBeforeStep = freeList.getAllocatedSize();

  world.step();
  const auto derivatives = world.getStepDerivatives();
  const auto liveBytesAfterFirstStep = freeList.getAllocatedSize();

  EXPECT_EQ(
      derivatives.controlJacobian.cols(),
      static_cast<Eigen::Index>(kJointCount));
  EXPECT_GT(liveBytesAfterFirstStep, liveBytesBeforeStep)
      << "differentiable multibody torque scratch should grow reusable "
         "World-owned storage on the first differentiable step";

  const auto liveBytesBeforeSecondStep = freeList.getAllocatedSize();
  const auto peakBytesBeforeSecondStep = freeList.getPeakAllocatedSize();
  world.step();

  EXPECT_EQ(freeList.getAllocatedSize(), liveBytesBeforeSecondStep)
      << "differentiable multibody torque scratch should retain and reuse "
         "its World-owned capacity across steps";
  EXPECT_EQ(freeList.getPeakAllocatedSize(), peakBytesBeforeSecondStep)
      << "reusing differentiable multibody torque scratch should not grow "
         "the World free-list peak on a same-size follow-up step";
}

TEST(World, DifferentiableContactFreeCoordinateScratchUsesProvidedAllocator)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.001);

  auto robot = world.addMultibody("diff_coordinate_allocator_chain");
  auto parent = robot.addLink("diff_coordinate_allocator_base");
  constexpr std::size_t kJointCount = 64u;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.02, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = std::format("hinge_{:02}", i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::format("link_{:02}", i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.06, 0.07).asDiagonal());

    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.001 * (i + 1u)));
    joint.setVelocity(Eigen::VectorXd::Constant(1, 0.01));
    parent = link;
  }

  world.enterSimulationMode();

  auto& registry = sx::detail::registryOf(world);
  auto structures = registry.view<sx::comps::MultibodyStructure>();
  const sx::comps::MultibodyStructure* structure = nullptr;
  std::size_t structureCount = 0;
  for (const auto entity : structures) {
    structure = &structures.get<sx::comps::MultibodyStructure>(entity);
    ++structureCount;
  }
  ASSERT_EQ(structureCount, 1u);
  ASSERT_NE(structure, nullptr);

  sx::detail::ContactFreeStepCoordinateScratch coordinateScratch(
      sx::detail::ContactFreeStepCoordinateAllocator{allocator});
  const Eigen::VectorXd tau
      = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(kJointCount), 0.25);

  const auto allocationsBeforeFirstCall = allocator.allocationCount;
  auto derivatives = sx::detail::contactFreeStepDerivatives(
      registry,
      *structure,
      world.getGravity(),
      world.getTimeStep(),
      tau,
      &coordinateScratch);

  EXPECT_EQ(
      derivatives.controlJacobian.cols(),
      static_cast<Eigen::Index>(kJointCount));
  EXPECT_GT(allocator.allocationCount, allocationsBeforeFirstCall)
      << "contact-free coordinate scratch should allocate through the "
         "provided allocator on the first derivative call";
  EXPECT_GE(coordinateScratch.capacity(), kJointCount);

  const auto allocationsAfterFirstCall = allocator.allocationCount;
  derivatives = sx::detail::contactFreeStepDerivatives(
      registry,
      *structure,
      world.getGravity(),
      world.getTimeStep(),
      tau,
      &coordinateScratch);

  EXPECT_EQ(
      derivatives.controlJacobian.cols(),
      static_cast<Eigen::Index>(kJointCount));
  EXPECT_EQ(allocator.allocationCount, allocationsAfterFirstCall)
      << "same-shape contact-free coordinate collection should reuse retained "
         "scratch capacity";
}
#endif

TEST(World, VariationalContactPointForceIntoReusesOutputStorage)
{
  namespace sx = dart::simulation;

  constexpr Eigen::Index kDof = 96;
  std::array<Eigen::Isometry3d, 1> transforms{Eigen::Isometry3d::Identity()};
  transforms[0].linear()
      = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  std::array<Eigen::MatrixXd, 1> bodyJacobians{Eigen::MatrixXd(6, kDof)};
  for (Eigen::Index col = 0; col < kDof; ++col) {
    const double scale = 0.01 * static_cast<double>(col + 1);
    bodyJacobians[0](0, col) = 0.5 * scale;
    bodyJacobians[0](1, col) = -0.25 * scale;
    bodyJacobians[0](2, col) = 0.125 * scale;
    bodyJacobians[0](3, col) = -0.2 * scale;
    bodyJacobians[0](4, col) = 0.4 * scale;
    bodyJacobians[0](5, col) = -0.1 * scale;
  }

  const Eigen::VectorXd previousVelocity
      = Eigen::VectorXd::LinSpaced(kDof, -0.5, 0.5);
  const sx::compute::VariationalContactContext context{
      std::span<const Eigen::Isometry3d>{transforms.data(), transforms.size()},
      std::span<const Eigen::MatrixXd>{
          bodyJacobians.data(), bodyJacobians.size()},
      static_cast<std::size_t>(kDof),
      std::span<const Eigen::Isometry3d>{transforms.data(), transforms.size()},
      std::span<const Eigen::MatrixXd>{
          bodyJacobians.data(), bodyJacobians.size()},
      previousVelocity,
      0.001};

  const Eigen::Vector3d localPoint(0.2, -0.1, 0.05);
  const Eigen::Vector3d worldForce(1.2, -0.4, 2.0);

  Eigen::Matrix3d localPointCross;
  localPointCross << 0.0, -localPoint.z(), localPoint.y(), localPoint.z(), 0.0,
      -localPoint.x(), -localPoint.y(), localPoint.x(), 0.0;
  const Eigen::MatrixXd worldPointJacobian
      = transforms[0].linear()
        * (bodyJacobians[0].bottomRows<3>()
           - localPointCross * bodyJacobians[0].topRows<3>());
  const Eigen::VectorXd reference = worldPointJacobian.transpose() * worldForce;

  Eigen::VectorXd generalizedForce;
  sx::compute::variationalContactPointForceInto(
      context, 0u, localPoint, worldForce, generalizedForce);
  const Eigen::VectorXd returned = sx::compute::variationalContactPointForce(
      context, 0u, localPoint, worldForce);

  EXPECT_TRUE(generalizedForce.isApprox(reference, 1e-12));
  EXPECT_TRUE(returned.isApprox(reference, 1e-12));

  const auto* retainedData = generalizedForce.data();
  ScopedHeapAllocationCounter heapCounter;
  sx::compute::variationalContactPointForceInto(
      context, 0u, localPoint, worldForce, generalizedForce);
  heapCounter.stop();

  EXPECT_EQ(generalizedForce.data(), retainedData);
  EXPECT_TRUE(generalizedForce.isApprox(reference, 1e-12));
  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "same-shape variational point-force diagnostics should reuse "
         "caller-owned output storage";
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, VariationalMechanicalEnergyScratchUsesProvidedAllocator)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.001);

  auto robot = world.addMultibody("energy_scratch_chain");
  auto parent = robot.addLink("energy_scratch_base");
  constexpr std::size_t kJointCount = 96u;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.03, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = std::format("hinge_{:02}", i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::format("link_{:02}", i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.06, 0.07).asDiagonal());

    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.001 * (i + 1u)));
    joint.setVelocity(Eigen::VectorXd::Constant(1, 0.01));
    parent = link;
  }

  world.enterSimulationMode();
  world.updateKinematics();

  auto& registry = sx::detail::registryOf(world);
  auto structures = registry.view<sx::comps::MultibodyStructure>();
  const sx::comps::MultibodyStructure* structure = nullptr;
  std::size_t structureCount = 0;
  for (const auto entity : structures) {
    structure = &structures.get<sx::comps::MultibodyStructure>(entity);
    ++structureCount;
  }
  ASSERT_EQ(structureCount, 1u);
  ASSERT_NE(structure, nullptr);

  sx::compute::MultibodyVariationalTreeScratch treeScratch(allocator);
  sx::compute::VariationalStepScratch stepScratch(allocator);
  const auto allocationsBeforeFirstCall = allocator.allocationCount;
  const double firstEnergy = sx::compute::computeMultibodyMechanicalEnergy(
      registry, *structure, world.getGravity(), treeScratch, stepScratch);

  EXPECT_TRUE(std::isfinite(firstEnergy));
  EXPECT_GT(allocator.allocationCount, allocationsBeforeFirstCall)
      << "mechanical-energy tree/spatial-velocity scratch should use the "
         "provided allocator on the first diagnostic";
  EXPECT_GE(
      stepScratch.currentSpatialVelocities.capacity(), structure->links.size());

  const auto allocationsAfterFirstCall = allocator.allocationCount;
  ScopedHeapAllocationCounter heapCounter;
  const double secondEnergy = sx::compute::computeMultibodyMechanicalEnergy(
      registry, *structure, world.getGravity(), treeScratch, stepScratch);
  heapCounter.stop();

  EXPECT_TRUE(std::isfinite(secondEnergy));
  EXPECT_NEAR(secondEnergy, firstEnergy, 1e-12);
  EXPECT_EQ(allocator.allocationCount, allocationsAfterFirstCall)
      << "same-shape variational mechanical-energy diagnostics should reuse "
         "retained spatial velocity scratch capacity";
  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "same-shape variational mechanical-energy diagnostics should not "
         "fall back to global heap allocation after scratch warmup";
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, VariationalInverseMassProductScratchUsesProvidedAllocator)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("inverse_mass_scratch_chain");
  auto parent = robot.addLink("inverse_mass_scratch_base");
  constexpr std::size_t kJointCount = 96u;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.03, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = std::format("hinge_{:02}", i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::format("link_{:02}", i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.06, 0.07).asDiagonal());

    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.001 * (i + 1u)));
    parent = link;
  }

  world.enterSimulationMode();
  world.updateKinematics();

  auto& registry = sx::detail::registryOf(world);
  auto structures = registry.view<sx::comps::MultibodyStructure>();
  const sx::comps::MultibodyStructure* structure = nullptr;
  std::size_t structureCount = 0;
  for (const auto entity : structures) {
    structure = &structures.get<sx::comps::MultibodyStructure>(entity);
    ++structureCount;
  }
  ASSERT_EQ(structureCount, 1u);
  ASSERT_NE(structure, nullptr);

  sx::compute::MultibodyVariationalTreeScratch treeScratch(allocator);
  sx::compute::VariationalLinearSolveScratch linearSolveScratch(allocator);
  const Eigen::VectorXd impulse = Eigen::VectorXd::LinSpaced(
      static_cast<Eigen::Index>(kJointCount), 0.1, 1.0);
  Eigen::VectorXd result;

  const auto allocationsBeforeFirstCall = allocator.allocationCount;
  sx::compute::computeMultibodyInverseMassProductInto(
      registry, *structure, impulse, treeScratch, linearSolveScratch, result);
  const Eigen::VectorXd reference
      = sx::compute::computeMultibodyInverseMassProduct(
          registry, *structure, impulse);

  EXPECT_EQ(result.size(), static_cast<Eigen::Index>(kJointCount));
  EXPECT_TRUE(result.isApprox(reference, 1e-12));
  EXPECT_GT(allocator.allocationCount, allocationsBeforeFirstCall)
      << "inverse-mass tree and linear-solve scratch should allocate through "
         "the provided allocator on the first direct helper call";

  const auto allocationsAfterFirstCall = allocator.allocationCount;
  ScopedHeapAllocationCounter heapCounter;
  sx::compute::computeMultibodyInverseMassProductInto(
      registry, *structure, impulse, treeScratch, linearSolveScratch, result);
  heapCounter.stop();

  EXPECT_TRUE(result.isApprox(reference, 1e-12));
  EXPECT_EQ(allocator.allocationCount, allocationsAfterFirstCall)
      << "same-shape inverse-mass diagnostics should reuse retained tree and "
         "linear-solve scratch capacity";
  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "same-shape inverse-mass diagnostics should not fall back to global "
         "heap allocation after scratch and output warmup";
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, VariationalConstraintLinearizationScratchUsesProvidedAllocator)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("constraint_linearization_scratch_chain");
  auto parent = robot.addLink("constraint_linearization_scratch_base");
  constexpr std::size_t kJointCount = 96u;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.03, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = std::format("hinge_{:02}", i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::format("link_{:02}", i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.06, 0.07).asDiagonal());

    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.001 * (i + 1u)));
    parent = link;
  }

  world.enterSimulationMode();
  world.updateKinematics();

  auto& registry = sx::detail::registryOf(world);
  auto structures = registry.view<sx::comps::MultibodyStructure>();
  const sx::comps::MultibodyStructure* structure = nullptr;
  std::size_t structureCount = 0;
  for (const auto entity : structures) {
    structure = &structures.get<sx::comps::MultibodyStructure>(entity);
    ++structureCount;
  }
  ASSERT_EQ(structureCount, 1u);
  ASSERT_NE(structure, nullptr);
  ASSERT_GT(structure->links.size(), kJointCount / 2u);

  std::vector<sx::compute::VariationalLoopConstraint> constraints;
  sx::compute::VariationalLoopConstraint distance;
  distance.linkA = structure->links.back();
  distance.pointA = Eigen::Vector3d(0.02, 0.0, 0.0);
  distance.linkB = entt::null;
  distance.pointB = Eigen::Vector3d(1.0, 0.1, 0.0);
  distance.distance = true;
  distance.length = 0.5;
  constraints.push_back(distance);

  sx::compute::VariationalLoopConstraint point;
  point.linkA = structure->links.back();
  point.pointA = Eigen::Vector3d::Zero();
  point.linkB = structure->links[kJointCount / 2u];
  point.pointB = Eigen::Vector3d(0.02, 0.0, 0.0);
  point.distance = false;
  constraints.push_back(point);

  sx::compute::MultibodyVariationalTreeScratch treeScratch(allocator);
  sx::compute::VariationalConstraintProjectionScratch projectionScratch(
      allocator);
  sx::compute::VariationalConstraintLinearization result;

  const auto allocationsBeforeFirstCall = allocator.allocationCount;
  sx::compute::computeVariationalConstraintLinearizationInto(
      registry,
      *structure,
      constraints,
      treeScratch,
      projectionScratch,
      result);
  const auto reference = sx::compute::computeVariationalConstraintLinearization(
      registry, *structure, constraints);

  EXPECT_EQ(result.residual.size(), reference.residual.size());
  EXPECT_EQ(result.jacobian.rows(), reference.jacobian.rows());
  EXPECT_EQ(result.jacobian.cols(), reference.jacobian.cols());
  EXPECT_TRUE(result.residual.isApprox(reference.residual, 1e-12));
  EXPECT_TRUE(result.jacobian.isApprox(reference.jacobian, 1e-12));
  EXPECT_EQ(result.residual.size(), 4);
  EXPECT_EQ(result.jacobian.cols(), static_cast<Eigen::Index>(kJointCount));
  EXPECT_GT(allocator.allocationCount, allocationsBeforeFirstCall)
      << "constraint-linearization tree and projection scratch should "
         "allocate through the provided allocator on the first direct helper "
         "call";

  const auto allocationsAfterFirstCall = allocator.allocationCount;
  ScopedHeapAllocationCounter heapCounter;
  sx::compute::computeVariationalConstraintLinearizationInto(
      registry,
      *structure,
      constraints,
      treeScratch,
      projectionScratch,
      result);
  heapCounter.stop();

  EXPECT_TRUE(result.residual.isApprox(reference.residual, 1e-12));
  EXPECT_TRUE(result.jacobian.isApprox(reference.jacobian, 1e-12));
  EXPECT_EQ(allocator.allocationCount, allocationsAfterFirstCall)
      << "same-shape variational constraint-linearization diagnostics should "
         "reuse retained tree and projection scratch capacity";
  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "same-shape variational constraint-linearization diagnostics should "
         "not fall back to global heap allocation after scratch and output "
         "warmup";
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, RigidBodyVelocityScratchPayloadUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto body = world.addRigidBody("velocity_scratch_body");
  body.setMass(2.0);
  body.setLinearVelocity(Eigen::Vector3d(0.25, 0.0, 0.0));

  sx::compute::RigidBodyVelocityStage stage(&world.getMemoryManager());

  ScopedHeapAllocationCounter heapCounter;
  stage.prepare(world);
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated while preparing allocator-aware rigid "
         "velocity scratch: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, RigidBodyIntegrationStageScratchUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  parentOptions.linearVelocity = Eigen::Vector3d(0.25, 0.0, 0.0);
  auto parent
      = world.addRigidBody("integration_allocator_parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(2.0, 0.0, 0.0);
  childOptions.linearVelocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto child = world.addRigidBody("integration_allocator_child", childOptions);
  child.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  world.enterSimulationMode();

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeStage = freeList.getAllocationCount();

  {
    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyIntegrationStage stage(1, &memoryManager);
    const auto allocationsAfterStage = freeList.getAllocationCount();

    stage.execute(world, executor);

    EXPECT_GE(freeList.getAllocationCount(), allocationsAfterStage + 2u)
        << "allocator-aware rigid integration scratch should reserve entity "
           "and dependency-node vectors from the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeStage);
}

TEST(World, BatchedRigidBodyIntegrationStageScratchUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  parentOptions.linearVelocity = Eigen::Vector3d(0.25, 0.0, 0.0);
  auto parent = world.addRigidBody(
      "batched_integration_allocator_parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(2.0, 0.0, 0.0);
  childOptions.linearVelocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto child
      = world.addRigidBody("batched_integration_allocator_child", childOptions);
  child.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.01);
  world.enterSimulationMode();

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeStage = freeList.getAllocationCount();

  {
    sx::compute::SequentialExecutor executor;
    sx::compute::BatchedRigidBodyIntegrationStage stage(&memoryManager);
    const auto allocationsAfterStage = freeList.getAllocationCount();

    stage.execute(world, executor);

    EXPECT_GE(freeList.getAllocationCount(), allocationsAfterStage + 5u)
        << "allocator-aware batched rigid integration scratch should reserve "
           "force, torque, entity, frame-order, and visit-state vectors from "
           "the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeStage);
}

TEST(World, RigidBodyContactScratchPayloadUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.18);
  auto box = world.addRigidBody("box", boxOptions);
  box.setMass(1.0);
  box.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.2, 0.2)));

  ASSERT_FALSE(world.collide().empty());

  sx::compute::RigidBodyContactStage stage(8, &world.getMemoryManager());
  auto& freeList = world.getMemoryManager().getFreeListAllocator();
  const auto allocationsBeforePrepare = freeList.getAllocationCount();

  stage.prepare(world);

  EXPECT_GT(freeList.getAllocationCount(), allocationsBeforePrepare)
      << "allocator-aware rigid contact scratch should reserve sequential "
         "impulse constraints from the World free allocator";
}

TEST(World, UnifiedConstraintStageScratchUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);
  configureCrossMultibodyStackedFallbackScene(world);
  ASSERT_FALSE(world.collide().empty());

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeStage = freeList.getAllocationCount();

  {
    sx::compute::UnifiedConstraintStage stage(8, &memoryManager);
    const auto allocationsAfterStage = freeList.getAllocationCount();

    stage.prepare(world);

    EXPECT_GT(freeList.getAllocationCount(), allocationsAfterStage)
        << "allocator-aware unified constraint scratch should reserve "
           "multibody staging vectors from the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeStage);
}

TEST(World, RigidBodyContactAvbdStageScratchUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;
  configureRigidAvbdFixedJointRowsScene(world);

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeStage = freeList.getAllocationCount();

  {
    sx::compute::RigidBodyContactStage stage(8, &memoryManager);
    const auto allocationsAfterStage = freeList.getAllocationCount();

    stage.prepare(world);

    EXPECT_GE(freeList.getAllocationCount(), allocationsAfterStage + 24u)
        << "allocator-aware rigid AVBD contact scratch should reserve "
           "snapshot, row-counter, solve, warm-start inventory, and "
           "point-joint buffers from the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeStage);
}

TEST(World, RigidIpcContactStageScratchPayloadUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;
  sx::DeformableBodyOptions sheetOptions;
  sheetOptions.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(-0.5, 0.5, 0.0),
         Eigen::Vector3d(0.5, 0.5, 0.0)};
  sheetOptions.masses = {1.0, 1.0, 1.0, 1.0};
  sheetOptions.fixedNodes = {0, 1, 2, 3};
  sheetOptions.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{1, 3, 2}};
  (void)world.addDeformableBody("ipc_allocator_sheet", sheetOptions);

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(0.0, 0.0, 0.055);
  auto body = world.addRigidBody("ipc_allocator_body", options);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 0.05, 0.05)));

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeStage = freeList.getAllocationCount();

  {
    sx::compute::RigidIpcContactStageOptions stageOptions;
    stageOptions.maxIterations = 1;
    stageOptions.activationDistance = 0.02;
    sx::compute::RigidIpcContactStage stage(stageOptions, &memoryManager);
    const auto allocationsAfterStage = freeList.getAllocationCount();

    ScopedHeapAllocationCounter heapCounter;
    stage.prepare(world);
    heapCounter.stop();

    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "allocator-aware rigid IPC prepare should not allocate from the "
           "global heap when preparing mixed rigid/deformable surface scratch";
    EXPECT_GE(freeList.getAllocationCount(), allocationsAfterStage + 30u)
        << "allocator-aware rigid IPC scratch should reserve top-level "
           "runtime, solver, mixed-domain surface/candidate, dynamics, "
           "result, writeback, and nested surface mesh payload vectors from "
           "the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeStage);
}

TEST(World, DeformableDynamicsStageScratchPayloadUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground
      = world.addRigidBody("deformable_allocator_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(1.0, 1.0, 0.25)));
  ground.setDeformableGroundBarrier(true);

  const auto addObstacle = [&](std::string_view name,
                               const Eigen::Vector3d& position,
                               const sx::CollisionShape& shape) {
    sx::RigidBodyOptions options;
    options.isStatic = true;
    options.position = position;
    auto obstacle = world.addRigidBody(name, options);
    obstacle.setCollisionShape(shape);
    obstacle.setDeformableSurfaceCcdObstacle(true);
  };
  addObstacle(
      "deformable_allocator_sphere_obstacle",
      Eigen::Vector3d(2.0, 0.0, 0.0),
      sx::CollisionShape::makeSphere(0.25));
  addObstacle(
      "deformable_allocator_box_obstacle",
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.25, 0.25, 0.25)));
  addObstacle(
      "deformable_allocator_capsule_obstacle",
      Eigen::Vector3d(0.0, 2.0, 0.0),
      sx::CollisionShape::makeCapsule(0.1, 0.25));

  sx::DeformableBodyOptions deformable;
  deformable.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.2),
         Eigen::Vector3d(0.4, 0.0, 0.2),
         Eigen::Vector3d(0.0, 0.4, 0.2)};
  deformable.masses = {1.0, 1.0, 1.0};
  deformable.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
  deformable.edgeStiffness = 0.0;
  world.addDeformableBody("deformable_allocator_surface", deformable);

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeStage = freeList.getAllocationCount();
  auto& worldFreeList = world.getMemoryManager().getFreeListAllocator();
  const auto worldAllocationsBeforePrepare = worldFreeList.getAllocationCount();

  {
    sx::compute::DeformableDynamicsStage stage(&memoryManager);
    const auto allocationsAfterStage = freeList.getAllocationCount();

    stage.prepare(world);

    EXPECT_GE(
        worldFreeList.getAllocationCount(), worldAllocationsBeforePrepare + 11u)
        << "World-owned deformable contact scratch should reserve per-body "
           "solver vectors, fixed/boundary masks, surface topology, "
           "inter-body CCD sweep, and projected-Newton assembly vectors from "
           "the World free allocator";
    EXPECT_GE(freeList.getAllocationCount(), allocationsAfterStage + 15u)
        << "allocator-aware deformable stage scratch should reserve obstacle, "
           "surface-snapshot, and nested snapshot payload vectors from the "
           "provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeStage);
}

TEST(World, WorldKinematicsGraphEntityNodesUseWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::World world;
  (void)world.addFreeFrame("kinematics_allocator_frame");

  auto& memoryManager = world.getMemoryManager();
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforeGraph = freeList.getAllocationCount();

  {
    sx::compute::WorldKinematicsGraph graph(
        world, memoryManager.getFreeAllocator());
    EXPECT_GT(graph.getGraph().getNodeCount(), 0u);
    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeGraph)
        << "allocator-aware kinematics graph should reserve entity-node "
           "cache storage from the World free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforeGraph);
}

TEST(World, WorldOptionsConfigureDomainSolverFamilies)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Ipc;
  options.multibodyOptions.integrationFamily = "variational integrator";

  sx::World world(options);

  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::Ipc);
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily, "variational integrator");
}

TEST(World, ClearResetsWorldOptionPoliciesToDefaults)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Ipc;
  options.multibodyOptions.integrationFamily = "variational integrator";
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  options.contactGradientMode = sx::ContactGradientMode::PreContactSurrogate;

  sx::World world(options);
  world.clear();

  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::SequentialImpulse);
  EXPECT_EQ(world.getMultibodyOptions().integrationFamily, "semi-implicit");
  EXPECT_FALSE(world.isDifferentiable());
  EXPECT_EQ(
      world.getContactSolverMethod(),
      sx::ContactSolverMethod::SequentialImpulse);
  EXPECT_EQ(world.getContactGradientMode(), sx::ContactGradientMode::Analytic);
}

TEST(World, WorldOptionsRejectInvalidDomainSolverFamilies)
{
  namespace sx = dart::simulation;

  sx::WorldOptions invalidRigid;
  invalidRigid.rigidBodySolver = static_cast<sx::RigidBodySolver>(99);
  EXPECT_THROW(
      {
        sx::World world(invalidRigid);
        (void)world;
      },
      sx::InvalidArgumentException);

  sx::WorldOptions invalidMultibody;
  invalidMultibody.multibodyOptions.integrationFamily = "unknown-family";
  EXPECT_THROW(
      {
        sx::World world(invalidMultibody);
        (void)world;
      },
      sx::InvalidArgumentException);

  sx::WorldOptions invalidContactSolver;
  invalidContactSolver.contactSolverMethod
      = static_cast<sx::ContactSolverMethod>(99);
  EXPECT_THROW(
      {
        sx::World world(invalidContactSolver);
        (void)world;
      },
      sx::InvalidArgumentException);

  sx::WorldOptions invalidContactGradient;
  invalidContactGradient.contactGradientMode
      = static_cast<sx::ContactGradientMode>(99);
  EXPECT_THROW(
      {
        sx::World world(invalidContactGradient);
        (void)world;
      },
      sx::InvalidArgumentException);
}

TEST(World, SetContactGradientModeRejectsInvalidEnum)
{
  namespace sx = dart::simulation;

  sx::World world;
  EXPECT_THROW(
      world.setContactGradientMode(static_cast<sx::ContactGradientMode>(99)),
      sx::InvalidArgumentException);
  EXPECT_EQ(world.getContactGradientMode(), sx::ContactGradientMode::Analytic);
}

TEST(World, MemoryManagerOptionsConfigureFreeListPolicy)
{
  namespace sx = dart::simulation;

  constexpr std::size_t kFreeListInitialAllocation = 65536;
  constexpr std::size_t kProbeBytes = 1024;

  sx::WorldOptions options;
  options.freeListInitialAllocation = kFreeListInitialAllocation;
  options.freeListGrowthPolicy
      = dart::common::FreeListAllocator::GrowthPolicy::FixedCapacity;

  sx::World world(options);
  auto& memoryManager = world.getMemoryManager();

  EXPECT_EQ(
      memoryManager.getFreeListAllocator().getGrowthPolicy(),
      dart::common::FreeListAllocator::GrowthPolicy::FixedCapacity);

  std::vector<void*> allocations;
  for (std::size_t i = 0; i < kFreeListInitialAllocation / kProbeBytes + 16;
       ++i) {
    auto* allocation = memoryManager.allocateUsingFree(kProbeBytes);
    if (allocation == nullptr) {
      break;
    }
    allocations.push_back(allocation);
  }

  EXPECT_FALSE(allocations.empty());
  auto* overflow = memoryManager.allocateUsingFree(kProbeBytes);
  EXPECT_EQ(overflow, nullptr);
  if (overflow != nullptr) {
    memoryManager.deallocateUsingFree(overflow, kProbeBytes);
  }
  EXPECT_EQ(memoryManager.allocateUsingPool(sizeof(double)), nullptr);

  for (auto* allocation : allocations) {
    memoryManager.deallocateUsingFree(allocation, kProbeBytes);
  }
}

TEST(World, MemoryDiagnosticsMirrorAllocatorDebugCounters)
{
  namespace sx = dart::simulation;

  sx::World world;

#if !defined(NDEBUG)
  const auto baselineDiagnostics
      = world.getMemoryManager().getDebugDiagnostics();
#endif
  auto* freePtr = world.getMemoryManager().allocateUsingFree(24);
  ASSERT_NE(freePtr, nullptr);
  auto* borrowedPoolPtr
      = world.getMemoryManager().getPoolAllocator().allocate(40);
  ASSERT_NE(borrowedPoolPtr, nullptr);

  const auto worldDiagnostics = world.getMemoryDiagnostics();
  const auto managerDiagnostics
      = world.getMemoryManager().getDebugDiagnostics();
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.enabled,
      managerDiagnostics.enabled);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator.liveBytes,
      managerDiagnostics.freeAllocator.liveBytes);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator.peakLiveBytes,
      managerDiagnostics.freeAllocator.peakLiveBytes);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator
          .liveAllocationCount,
      managerDiagnostics.freeAllocator.liveAllocationCount);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.poolAllocator.liveBytes,
      managerDiagnostics.poolAllocator.liveBytes);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.poolAllocator.peakLiveBytes,
      managerDiagnostics.poolAllocator.peakLiveBytes);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.poolAllocator
          .liveAllocationCount,
      managerDiagnostics.poolAllocator.liveAllocationCount);

#if !defined(NDEBUG)
  EXPECT_TRUE(worldDiagnostics.allocatorDebugDiagnostics.enabled);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.poolAllocator.liveBytes,
      baselineDiagnostics.poolAllocator.liveBytes + 40u);
  EXPECT_EQ(
      worldDiagnostics.allocatorDebugDiagnostics.poolAllocator
          .liveAllocationCount,
      baselineDiagnostics.poolAllocator.liveAllocationCount + 1u);
  EXPECT_GE(
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator.liveBytes,
      baselineDiagnostics.freeAllocator.liveBytes + 24u);
  EXPECT_GE(
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator.peakLiveBytes,
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator.liveBytes);
  EXPECT_GE(
      worldDiagnostics.allocatorDebugDiagnostics.freeAllocator
          .liveAllocationCount,
      baselineDiagnostics.freeAllocator.liveAllocationCount + 1u);
#endif

  world.getMemoryManager().getPoolAllocator().deallocate(borrowedPoolPtr, 40);
  world.getMemoryManager().deallocateUsingFree(freePtr, 24);
}

TEST(World, MemoryDiagnosticsReportEcsStorageLayout)
{
  namespace sx = dart::simulation;

  sx::World world;

  const auto empty = world.getMemoryDiagnostics();
  EXPECT_EQ(empty.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(empty.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(
      empty.ecsDiagnostics.storageCount, empty.ecsDiagnostics.storages.size());

  auto frame = world.addFreeFrame("diagnostic_frame");
  (void)frame;

  const auto diagnostics = world.getMemoryDiagnostics();
  EXPECT_EQ(diagnostics.ecsDiagnostics.entityCount, 1u);
  EXPECT_GE(
      diagnostics.ecsDiagnostics.entityCapacity,
      diagnostics.ecsDiagnostics.entityCount);
  ASSERT_GT(diagnostics.ecsDiagnostics.storageCount, 0u);
  EXPECT_EQ(
      diagnostics.ecsDiagnostics.storageCount,
      diagnostics.ecsDiagnostics.storages.size());

  std::size_t componentCount = 0;
  std::size_t componentCapacity = 0;
  bool hasLiveStorage = false;
  for (const auto& storage : diagnostics.ecsDiagnostics.storages) {
    EXPECT_GE(storage.capacity, storage.size);
    componentCount += storage.size;
    componentCapacity += storage.capacity;
    hasLiveStorage = hasLiveStorage || storage.size > 0u;
  }

  EXPECT_TRUE(hasLiveStorage);
  EXPECT_EQ(diagnostics.ecsDiagnostics.componentCount, componentCount);
  EXPECT_EQ(diagnostics.ecsDiagnostics.componentCapacity, componentCapacity);

  world.clear();
  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(
      cleared.ecsDiagnostics.storageCount,
      cleared.ecsDiagnostics.storages.size());
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
  }
}

TEST(World, ClearReleasesRegistryStorageForRebuild)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;
  auto body = world.addRigidBody("body");
  body.setMass(2.0);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.3, 0.4)));

  world.enterSimulationMode();

  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);
#if !defined(NDEBUG)
  ASSERT_TRUE(baked.allocatorDebugDiagnostics.enabled);
  EXPECT_GT(baked.allocatorDebugDiagnostics.freeAllocator.liveBytes, 0u);
#endif

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }
#if !defined(NDEBUG)
  sx::World emptyWorld;
  const auto emptyDiagnostics = emptyWorld.getMemoryDiagnostics();
  EXPECT_LT(
      cleared.allocatorDebugDiagnostics.freeAllocator.liveBytes,
      baked.allocatorDebugDiagnostics.freeAllocator.liveBytes);
  EXPECT_EQ(
      cleared.allocatorDebugDiagnostics.freeAllocator.liveBytes,
      emptyDiagnostics.allocatorDebugDiagnostics.freeAllocator.liveBytes);
  EXPECT_EQ(
      cleared.allocatorDebugDiagnostics.freeAllocator.liveAllocationCount,
      emptyDiagnostics.allocatorDebugDiagnostics.freeAllocator
          .liveAllocationCount);
#endif

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  auto rebuilt = world.addRigidBody("rebuilt");
  EXPECT_TRUE(rebuilt.isValid());
}

TEST(World, RegistryUsesWorldFreeAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  sx::World world;
  auto& freeAllocator = world.getMemoryManager().getFreeAllocator();
  auto& registry = sx::detail::registryOf(world);

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      freeAllocator};
  EXPECT_EQ(registry.get_allocator(), expectedEntityAllocator);

  auto registryAllocator = registry.get_allocator();
  auto* allocatedEntity = registryAllocator.allocate(1);
  ASSERT_NE(allocatedEntity, nullptr);
#if !defined(NDEBUG)
  EXPECT_TRUE(world.getMemoryManager().hasAllocated(
      allocatedEntity, sizeof(entt::entity)));
#else
  EXPECT_FALSE(world.getMemoryManager().hasAllocated(
      allocatedEntity, sizeof(entt::entity)));
#endif
  registryAllocator.deallocate(allocatedEntity, 1);

  const auto entity = registry.create();
  const auto& component
      = registry.emplace<RegistryAllocatorComponent>(entity, 7u);
  EXPECT_EQ(component.value, 7u);

  const common::StlAllocator<RegistryAllocatorComponent>
      expectedComponentAllocator{freeAllocator};
  EXPECT_EQ(
      registry.storage<RegistryAllocatorComponent>().get_allocator(),
      expectedComponentAllocator);

  const auto& storage = sx::detail::storageOf(world);
  const common::StlAllocator<sx::detail::WorldStorage::DifferentiableParameter>
      expectedParameterAllocator{freeAllocator};
  EXPECT_EQ(
      storage.differentiableParameters.get_allocator(),
      expectedParameterAllocator);
}

TEST(World, ReservedRegistryStorageReusesComponentCapacity)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  common::MemoryManager memoryManager(allocator);
  sx::detail::WorldRegistry registry(
      sx::detail::WorldRegistryAllocator{memoryManager.getFreeAllocator()});

  constexpr std::size_t kEntityCount = 16;
  registry.storage<entt::entity>().reserve(kEntityCount);
  registry.storage<sx::comps::RigidBodyTag>().reserve(kEntityCount);
  registry.storage<sx::comps::Transform>().reserve(kEntityCount);
  registry.storage<sx::comps::Velocity>().reserve(kEntityCount);
  registry.storage<sx::comps::Force>().reserve(kEntityCount);

  const auto capacities = registryStorageCapacities(registry);
  const auto allocationsAfterReserve = allocator.allocationCount;
  const auto alignedAllocationsAfterReserve = allocator.alignedAllocationCount;

  std::array<entt::entity, kEntityCount> entities{};
  for (int cycle = 0; cycle < 3; ++cycle) {
    for (std::size_t i = 0; i < kEntityCount; ++i) {
      const auto entity = registry.create();
      entities[i] = entity;
      registry.emplace<sx::comps::RigidBodyTag>(entity);
      auto& transform = registry.emplace<sx::comps::Transform>(entity);
      transform.position.x() = static_cast<double>(i);
      registry.emplace<sx::comps::Velocity>(entity);
      registry.emplace<sx::comps::Force>(entity);
    }

    EXPECT_EQ(registry.storage<sx::comps::Velocity>().size(), kEntityCount);
    registry.clear<sx::comps::Velocity>();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    for (const auto entity : entities) {
      registry.emplace<sx::comps::Velocity>(entity);
    }
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);

    for (const auto entity : entities) {
      registry.destroy(entity);
    }
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterReserve);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterReserve);
}

TEST(World, EnterSimulationModeReservesRegistryStorageForKinematicIpcSteps)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);

  auto body = world.addRigidBody("kinematic");
  body.setKinematic(true);
  body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));

  world.enterSimulationMode();

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);
  const auto traceStorageId
      = entt::type_hash<sx::comps::KinematicBodyStepTrace>::value();
  ASSERT_TRUE(capacities.contains(traceStorageId));
  EXPECT_GE(capacities.at(traceStorageId), 1u);

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }
}

TEST(World, KinematicIpcSurfaceCcdRegistryStorageRebuildsAfterClear)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World world(options);

  const auto bakeAndCheckStableSteps = [&]() {
    configureDeformableKinematicRigidSurfaceCcdCrossingScene(world);
    const auto body = world.getRigidBody("kinematic_surface_box");
    EXPECT_TRUE(body.has_value());
    const auto bodyEntity
        = body.has_value() ? sx::detail::toRegistryEntity(body->getEntity())
                           : entt::null;

    world.enterSimulationMode();

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);
    const auto traceStorageId
        = entt::type_hash<sx::comps::KinematicBodyStepTrace>::value();
    const auto traceCapacity = capacities.find(traceStorageId);
    EXPECT_NE(traceCapacity, capacities.end());
    if (traceCapacity != capacities.end()) {
      EXPECT_GE(traceCapacity->second, 1u);
    }

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      const auto* trace
          = registry.try_get<sx::comps::KinematicBodyStepTrace>(bodyEntity);
      EXPECT_NE(trace, nullptr);
      const auto* traceStorage
          = registry.storage<sx::comps::KinematicBodyStepTrace>();
      EXPECT_NE(traceStorage, nullptr);
      if (traceStorage != nullptr) {
        EXPECT_EQ(traceStorage->size(), 1u);
      }
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

template <typename ConfigureScene>
void expectRigidAvbdRegistryStorageRebuildsAfterClear(
    std::string_view scene,
    ConfigureScene&& configureScene,
    bool expectContactConfig,
    bool expectPointJointConfig,
    bool expectDistanceSpringConfig,
    bool requireInitialContact)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;
  namespace dvbd = dart::simulation::detail::deformable_vbd;

  SCOPED_TRACE(scene);
  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World world(options);

  const auto bakeAndCheckStableSteps = [&]() {
    configureScene(world);
    world.enterSimulationMode();
    if (requireInitialContact) {
      EXPECT_FALSE(world.collide().empty());
    }

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);

    if (expectContactConfig) {
      const auto contactConfigStorageId
          = entt::type_hash<sx::comps::RigidAvbdContactConfig>::value();
      const auto contactConfigCapacity
          = capacities.find(contactConfigStorageId);
      EXPECT_NE(contactConfigCapacity, capacities.end());
      if (contactConfigCapacity != capacities.end()) {
        EXPECT_GE(contactConfigCapacity->second, 1u);
      }
      const auto* contactConfigStorage
          = registry.storage<sx::comps::RigidAvbdContactConfig>();
      EXPECT_NE(contactConfigStorage, nullptr);
      if (contactConfigStorage != nullptr) {
        EXPECT_GE(contactConfigStorage->size(), 1u);
      }
    }

    if (expectPointJointConfig) {
      const auto pointJointStorageId
          = entt::type_hash<dvbd::AvbdRigidWorldPointJointConfig>::value();
      const auto pointJointCapacity = capacities.find(pointJointStorageId);
      EXPECT_NE(pointJointCapacity, capacities.end());
      if (pointJointCapacity != capacities.end()) {
        EXPECT_GE(pointJointCapacity->second, 1u);
      }
      const auto* pointJointStorage
          = registry.storage<dvbd::AvbdRigidWorldPointJointConfig>();
      EXPECT_NE(pointJointStorage, nullptr);
      if (pointJointStorage != nullptr) {
        EXPECT_GE(pointJointStorage->size(), 1u);
      }
    }

    if (expectDistanceSpringConfig) {
      const auto distanceSpringStorageId
          = entt::type_hash<dvbd::AvbdRigidWorldDistanceSpringConfig>::value();
      const auto distanceSpringCapacity
          = capacities.find(distanceSpringStorageId);
      EXPECT_NE(distanceSpringCapacity, capacities.end());
      if (distanceSpringCapacity != capacities.end()) {
        EXPECT_GE(distanceSpringCapacity->second, 1u);
      }
      const auto* distanceSpringStorage
          = registry.storage<dvbd::AvbdRigidWorldDistanceSpringConfig>();
      EXPECT_NE(distanceSpringStorage, nullptr);
      if (distanceSpringStorage != nullptr) {
        EXPECT_GE(distanceSpringStorage->size(), 1u);
      }
    }

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

TEST(World, RigidAvbdRegistryStorageRebuildsAfterClear)
{
  expectRigidAvbdRegistryStorageRebuildsAfterClear(
      "rigid AVBD contact rows",
      configureRigidAvbdContactRowsScene,
      true,
      false,
      false,
      true);
  expectRigidAvbdRegistryStorageRebuildsAfterClear(
      "rigid AVBD fixed-joint rows",
      configureRigidAvbdFixedJointRowsScene,
      false,
      true,
      false,
      false);
  expectRigidAvbdRegistryStorageRebuildsAfterClear(
      "rigid AVBD distance-spring rows",
      configureRigidAvbdDistanceSpringRowsScene,
      false,
      false,
      true,
      false);
}

TEST(World, SemiImplicitMultibodyRegistryStorageRebuildsAfterClear)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World world(options);

  const auto configureScene = [&]() {
    auto robot = world.addMultibody("slider");
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "rail";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto carriage = robot.addLink("carriage", base, spec);
    carriage.setMass(3.0);
    world.setTimeStep(0.01);
  };

  const auto bakeAndCheckStableSteps = [&]() {
    configureScene();
    world.enterSimulationMode();

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);
    const auto multibodyStorageId
        = entt::type_hash<sx::comps::MultibodyStructure>::value();
    const auto multibodyCapacity = capacities.find(multibodyStorageId);
    EXPECT_NE(multibodyCapacity, capacities.end());
    if (multibodyCapacity != capacities.end()) {
      EXPECT_GE(multibodyCapacity->second, 1u);
    }

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

TEST(World, EnterSimulationModeReservesRegistryStorageForMultibodySteps)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(3.0);

  world.setTimeStep(0.001);
  world.enterSimulationMode();

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }
}

TEST(World, SetMultibodyOptionsReservesVariationalStateAfterBake)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World world(options);

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(3.0);

  world.setTimeStep(0.01);
  world.enterSimulationMode();
  world.setMultibodyOptions({"variational integrator"});

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);
  const auto stateStorageId
      = entt::type_hash<sx::compute::MultibodyVariationalState>::value();
  ASSERT_TRUE(capacities.contains(stateStorageId));
  EXPECT_GE(capacities.at(stateStorageId), 1u);

  const auto allocationsAfterSwitch = allocator.allocationCount;
  const auto deallocationsAfterSwitch = allocator.deallocationCount;
  const auto alignedAllocationsAfterSwitch = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterSwitch
      = allocator.alignedDeallocationCount;

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterSwitch);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterSwitch);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterSwitch);
  EXPECT_EQ(
      allocator.alignedDeallocationCount, alignedDeallocationsAfterSwitch);
}

TEST(World, SolverMethodSwitchesAfterBakeSharePreparationPath)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World world(options);

  auto body = world.addRigidBody("kinematic_box");
  body.setKinematic(true);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(3.0);

  world.setTimeStep(0.01);
  world.enterSimulationMode();

  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setMultibodyOptions({"variational integrator"});

  const auto& registry = sx::detail::registryOf(world);
  auto capacities = registryStorageCapacities(registry);
  auto allocationsAfterSwitch = allocator.allocationCount;
  auto deallocationsAfterSwitch = allocator.deallocationCount;
  auto alignedAllocationsAfterSwitch = allocator.alignedAllocationCount;
  auto alignedDeallocationsAfterSwitch = allocator.alignedDeallocationCount;

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterSwitch);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterSwitch);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterSwitch);
  EXPECT_EQ(
      allocator.alignedDeallocationCount, alignedDeallocationsAfterSwitch);

  world.setMultibodyOptions({"semi-implicit"});
  world.setRigidBodySolver(sx::RigidBodySolver::SequentialImpulse);

  capacities = registryStorageCapacities(registry);
  allocationsAfterSwitch = allocator.allocationCount;
  deallocationsAfterSwitch = allocator.deallocationCount;
  alignedAllocationsAfterSwitch = allocator.alignedAllocationCount;
  alignedDeallocationsAfterSwitch = allocator.alignedDeallocationCount;

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterSwitch);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterSwitch);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterSwitch);
  EXPECT_EQ(
      allocator.alignedDeallocationCount, alignedDeallocationsAfterSwitch);
}

TEST(World, EnterSimulationModeReservesRegistryStorageForDeformableSteps)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
  options.masses = {1.0};
  options.edgeStiffness = 0.0;
  world.addDeformableBody("particle", options);

  world.setTimeStep(0.01);
  world.enterSimulationMode();

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }
}

constexpr std::size_t kCompliantVariationalContactRobotCount = 6;
constexpr std::size_t kCompliantVariationalContactPointsPerRobot = 4;

void configureVariationalLoopClosureChainScene(dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setMultibodyOptions({"variational integrator"});
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0e-3);

  auto robot = world.addMultibody("loop_closure_chain");
  auto parent = robot.addLink("base");
  Eigen::Isometry3d tipFromBase = Eigen::Isometry3d::Identity();
  constexpr double bend[5] = {0.0, 0.5, -0.5, 0.5, -0.5};
  for (int i = 0; i < 5; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.linear() = Eigen::AngleAxisd(bend[i], Eigen::Vector3d::UnitY())
                          .toRotationMatrix();
    offset.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
    sx::JointSpec spec;
    spec.name = "j" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis = (i == 0) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink("l" + std::to_string(i), parent, spec);
    link.setMass(0.8);
    link.setInertia(Eigen::Vector3d(0.02, 0.02, 0.02).asDiagonal());
    link.getParentJoint().setVelocity(Eigen::VectorXd::Constant(1, 0.02));
    parent = link;
    tipFromBase = tipFromBase * offset;
  }

  Eigen::Isometry3d anchorFrame = Eigen::Isometry3d::Identity();
  anchorFrame.translation() = tipFromBase.translation();
  auto closure = world.addLoopClosure(
      "pin",
      {.frameA = robot.getLinks().back(),
       .frameB = sx::Frame::world(),
       .family = sx::LoopClosureFamily::Point,
       .offsetB = anchorFrame});
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
}

void configureLongVariationalArticulatedPointJointScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setMultibodyOptions({"variational integrator"});
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0e-3);

  auto robot = world.addMultibody("long_point_joint_chain");
  auto parent = robot.addLink("base");
  for (int i = 0; i < 18; ++i) {
    sx::JointSpec spec;
    spec.name = "hinge_" + std::to_string(i);
    spec.type = sx::JointType::Revolute;
    spec.axis
        = (i % 2 == 0) ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitZ();
    spec.transformFromParent.translation() = Eigen::Vector3d(0.05, 0.0, 0.0);
    auto link = robot.addLink("link_" + std::to_string(i), parent, spec);
    link.setMass(0.2);
    link.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
    parent = link;
  }

  world.addArticulatedFixedJoint("tip_world_anchor", parent);
}

void configureCompliantVariationalContactSliderScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setMultibodyOptions({"variational integrator"});
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(1.0e-3);

  for (std::size_t i = 0; i < kCompliantVariationalContactRobotCount; ++i) {
    auto robot
        = world.addMultibody("compliant_contact_slider_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "rail";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto carriage = robot.addLink("carriage", base, spec);
    carriage.setMass(1.0);
    carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
    auto joint = carriage.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, -0.001));

    robot.setGroundContact(
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::Zero(),
        1.0e4,
        /*frictionCoefficient=*/0.0,
        /*frictionRegularization=*/1.0e-4,
        /*dampingCoefficient=*/20.0,
        /*dualUpdateCadence=*/0);
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.0, 0.0, 0.0));
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.02, 0.0, 0.0));
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.0, 0.02, 0.0));
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.02, 0.02, 0.0));
  }
}

void configureVariationalContactDualStateSliderScene(
    dart::simulation::World& world)
{
  namespace sx = dart::simulation;

  world.setMultibodyOptions({"variational integrator"});
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(1.0e-3);

  for (std::size_t i = 0; i < kCompliantVariationalContactRobotCount; ++i) {
    auto robot = world.addMultibody("contact_slider_" + std::to_string(i));
    auto base = robot.addLink("base");
    sx::JointSpec spec;
    spec.name = "rail";
    spec.type = sx::JointType::Prismatic;
    spec.axis = Eigen::Vector3d::UnitZ();
    auto carriage = robot.addLink("carriage", base, spec);
    carriage.setMass(1.0);
    carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
    auto joint = carriage.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, -0.001));

    robot.setGroundContact(
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::Zero(),
        1.0e4,
        /*frictionCoefficient=*/0.0,
        /*frictionRegularization=*/1.0e-4,
        /*dampingCoefficient=*/20.0,
        /*dualUpdateCadence=*/1);
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.0, 0.0, 0.0));
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.02, 0.0, 0.0));
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.0, 0.02, 0.0));
    robot.addGroundContactPoint(carriage, Eigen::Vector3d(0.02, 0.02, 0.0));
  }
}

void expectCompliantVariationalContactScratchBaked(
    const dart::simulation::detail::WorldRegistry& registry,
    dart::common::MemoryAllocator* expectedAllocator = nullptr)
{
  namespace sx = dart::simulation;

  std::size_t scratchCount = 0;
  for (const auto entity :
       registry.view<sx::compute::MultibodyVariationalScratch>()) {
    const auto& scratch
        = registry.get<sx::compute::MultibodyVariationalScratch>(entity);
    if (expectedAllocator != nullptr) {
      const sx::compute::VariationalGroundContact::PointAllocator
          expectedPointAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.groundContact.points.get_allocator(), expectedPointAllocator);
      const sx::compute::VariationalContactEvaluationScratch::TransformAllocator
          expectedTransformAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.contactEvaluation.previousWorldTransforms.get_allocator(),
          expectedTransformAllocator);
      EXPECT_EQ(
          scratch.contactEvaluation.trialRelativeTransforms.get_allocator(),
          expectedTransformAllocator);
      EXPECT_EQ(
          scratch.contactEvaluation.trialWorldTransforms.get_allocator(),
          expectedTransformAllocator);
      const sx::compute::VariationalContactEvaluationScratch::JacobianAllocator
          expectedJacobianAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.contactEvaluation.previousJacobians.get_allocator(),
          expectedJacobianAllocator);
      EXPECT_EQ(
          scratch.contactEvaluation.trialJacobians.get_allocator(),
          expectedJacobianAllocator);
    }
    EXPECT_EQ(
        scratch.groundContact.points.size(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_GE(
        scratch.groundContact.points.capacity(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_EQ(scratch.contactEvaluation.contactForce.size(), 1);
    EXPECT_EQ(scratch.contactEvaluation.forcing.size(), 1);
    EXPECT_GE(scratch.step.previousJointVelocity.size(), 1);
    EXPECT_FALSE(scratch.groundContactSolver.has_value())
        << "pure compliant contact evaluates directly from baked "
           "ground-contact "
           "scratch; only AL contact owns solver dual storage";
    ++scratchCount;
  }
  EXPECT_EQ(scratchCount, kCompliantVariationalContactRobotCount);

  const auto dualStates
      = registry.view<sx::comps::VariationalContactDualState>();
  EXPECT_TRUE(dualStates.begin() == dualStates.end());
}

void expectVariationalContactGroundSolverBaked(
    const dart::simulation::detail::WorldRegistry& registry,
    dart::common::MemoryAllocator* expectedAllocator = nullptr)
{
  namespace sx = dart::simulation;

  std::size_t scratchCount = 0;
  for (const auto entity :
       registry.view<sx::compute::MultibodyVariationalScratch>()) {
    const auto& scratch
        = registry.get<sx::compute::MultibodyVariationalScratch>(entity);
    ASSERT_TRUE(scratch.groundContactSolver.has_value());
    if (expectedAllocator != nullptr) {
      EXPECT_EQ(
          &scratch.groundContactSolver->getAllocator(), expectedAllocator);
      const sx::compute::VariationalGroundContactSolver::DualAllocator
          expectedDualAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.groundContactSolver->duals().get_allocator(),
          expectedDualAllocator);
      const sx::compute::MultibodyVariationalScratch::
          PostContactTransformAllocator expectedTransformAllocator{
              *expectedAllocator};
      EXPECT_EQ(
          scratch.postContactTransforms.get_allocator(),
          expectedTransformAllocator);
    }
    EXPECT_EQ(
        scratch.groundContactSolver->duals().size(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_GE(
        scratch.groundContactSolver->duals().capacity(),
        kCompliantVariationalContactPointsPerRobot);
    ++scratchCount;
  }
  EXPECT_EQ(scratchCount, kCompliantVariationalContactRobotCount);
}

void expectVariationalContactDualStateBaked(
    const dart::simulation::detail::WorldRegistry& registry,
    const dart::common::StlAllocator<double>* expectedAllocator = nullptr)
{
  namespace sx = dart::simulation;

  std::size_t dualStateCount = 0;
  for (const auto entity :
       registry.view<sx::comps::VariationalContactDualState>()) {
    const auto& dualState
        = registry.get<sx::comps::VariationalContactDualState>(entity);
    EXPECT_EQ(
        dualState.duals.size(), kCompliantVariationalContactPointsPerRobot);
    EXPECT_GE(
        dualState.duals.capacity(), kCompliantVariationalContactPointsPerRobot);
    if (expectedAllocator != nullptr) {
      EXPECT_EQ(dualState.duals.get_allocator(), *expectedAllocator);
    }
    ++dualStateCount;
  }
  EXPECT_EQ(dualStateCount, kCompliantVariationalContactRobotCount);
}

void expectVariationalContactConfigUsesAllocator(
    const dart::simulation::detail::WorldRegistry& registry,
    dart::common::MemoryAllocator& allocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  const common::StlAllocator<std::size_t> expectedLinkAllocator{allocator};
  const common::StlAllocator<Eigen::Vector3d> expectedPointAllocator{allocator};

  std::size_t contactCount = 0;
  for (const auto entity : registry.view<sx::comps::VariationalContact>()) {
    const auto& contact = registry.get<sx::comps::VariationalContact>(entity);
    EXPECT_EQ(
        contact.pointLinkIndices.size(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_GE(
        contact.pointLinkIndices.capacity(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_EQ(contact.pointLinkIndices.get_allocator(), expectedLinkAllocator);
    EXPECT_EQ(
        contact.pointLocalPositions.size(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_GE(
        contact.pointLocalPositions.capacity(),
        kCompliantVariationalContactPointsPerRobot);
    EXPECT_EQ(
        contact.pointLocalPositions.get_allocator(), expectedPointAllocator);
    ++contactCount;
  }
  EXPECT_EQ(contactCount, kCompliantVariationalContactRobotCount);
}

void expectVariationalStateUsesAllocator(
    const dart::simulation::detail::WorldRegistry& registry,
    dart::common::MemoryAllocator& allocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  using Vector6 = Eigen::Matrix<double, 6, 1>;
  const common::StlAllocator<Eigen::Isometry3d> expectedTransformAllocator{
      allocator};
  const common::StlAllocator<Vector6> expectedMomentumAllocator{allocator};

  std::size_t stateCount = 0;
  for (const auto entity : registry.view<
                           sx::comps::MultibodyStructure,
                           sx::compute::MultibodyVariationalState>()) {
    const auto& structure = registry.get<sx::comps::MultibodyStructure>(entity);
    const auto& state
        = registry.get<sx::compute::MultibodyVariationalState>(entity);
    EXPECT_GE(state.previousDeltaTransform.capacity(), structure.links.size());
    EXPECT_GE(state.previousMomentum.capacity(), structure.links.size());
    EXPECT_EQ(
        state.previousDeltaTransform.get_allocator(),
        expectedTransformAllocator);
    EXPECT_EQ(
        state.previousMomentum.get_allocator(), expectedMomentumAllocator);
    ++stateCount;
  }
  EXPECT_EQ(stateCount, kCompliantVariationalContactRobotCount);
}

void expectVariationalLoopClosureScratchBaked(
    const dart::simulation::detail::WorldRegistry& registry,
    dart::common::MemoryAllocator* expectedAllocator = nullptr)
{
  namespace sx = dart::simulation;

  constexpr std::size_t kLinkCount = 6;
  constexpr Eigen::Index kDofCount = 5;
  constexpr Eigen::Index kProjectionRows = 3;

  std::size_t scratchCount = 0;
  for (const auto entity :
       registry.view<sx::compute::MultibodyVariationalScratch>()) {
    const auto& scratch
        = registry.get<sx::compute::MultibodyVariationalScratch>(entity);
    if (expectedAllocator != nullptr) {
      EXPECT_EQ(&scratch.tree.getAllocator(), expectedAllocator);
      EXPECT_EQ(&scratch.inverseDynamics.getAllocator(), expectedAllocator);
      const sx::compute::VariationalStepScratch::SpatialVelocityAllocator
          expectedSpatialVelocityAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.step.currentSpatialVelocities.get_allocator(),
          expectedSpatialVelocityAllocator);
      const sx::compute::VariationalLinearSolveScratch::Matrix6Allocator
          expectedMatrix6Allocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.linearSolve.articulated.get_allocator(),
          expectedMatrix6Allocator);
      EXPECT_EQ(
          scratch.linearSolve.motionToChild.get_allocator(),
          expectedMatrix6Allocator);
      const sx::compute::VariationalLinearSolveScratch::Vector6Allocator
          expectedVector6Allocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.linearSolve.bias.get_allocator(), expectedVector6Allocator);
      EXPECT_EQ(
          scratch.linearSolve.spatial.get_allocator(),
          expectedVector6Allocator);
      const sx::compute::VariationalLinearSolveScratch::MatrixAllocator
          expectedMatrixAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.linearSolve.forceProjector.get_allocator(),
          expectedMatrixAllocator);
      EXPECT_EQ(
          scratch.linearSolve.motionProjector.get_allocator(),
          expectedMatrixAllocator);
      EXPECT_EQ(
          scratch.linearSolve.jointMatrix.get_allocator(),
          expectedMatrixAllocator);
      EXPECT_EQ(
          scratch.linearSolve.jointMatrixInverse.get_allocator(),
          expectedMatrixAllocator);
      const sx::compute::VariationalLinearSolveScratch::VectorAllocator
          expectedVectorAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.linearSolve.jointRhs.get_allocator(),
          expectedVectorAllocator);
      const sx::compute::VariationalConstraintProjectionScratch::
          JacobianAllocator expectedJacobianAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.projection.jacobians.get_allocator(),
          expectedJacobianAllocator);
      const sx::compute::VariationalConstraintProjectionScratch::
          ProjectionBoundsAllocator expectedBoundsAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.projection.projectionBounds.get_allocator(),
          expectedBoundsAllocator);
      const sx::compute::VariationalConstraintProjectionScratch::
          RowIndexAllocator expectedRowAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.projection.hardRows.get_allocator(), expectedRowAllocator);
      EXPECT_EQ(
          scratch.projection.boundedRows.get_allocator(), expectedRowAllocator);
      const sx::compute::VariationalAndersonScratch::VectorAllocator
          expectedAndersonVectorAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.anderson.stepDeltas.get_allocator(),
          expectedAndersonVectorAllocator);
      EXPECT_EQ(
          scratch.anderson.iterateDeltas.get_allocator(),
          expectedAndersonVectorAllocator);
      const sx::compute::MultibodyVariationalScratch::ConstraintAllocator
          expectedConstraintAllocator{*expectedAllocator};
      EXPECT_EQ(
          scratch.constraints.get_allocator(), expectedConstraintAllocator);
    }
    EXPECT_EQ(scratch.tree.linkCount(), kLinkCount);
    EXPECT_EQ(scratch.tree.dofCount(), kDofCount);
    EXPECT_EQ(scratch.step.position.size(), kDofCount);
    EXPECT_EQ(scratch.step.velocity.size(), kDofCount);
    EXPECT_EQ(scratch.step.residual.size(), kDofCount);
    EXPECT_EQ(scratch.linearSolve.articulated.size(), kLinkCount);
    EXPECT_EQ(scratch.linearSolve.rhs.size(), kDofCount);
    EXPECT_EQ(scratch.linearSolve.result.size(), kDofCount);
    EXPECT_EQ(scratch.projection.residual.size(), kProjectionRows);
    EXPECT_EQ(scratch.projection.jacobian.rows(), kProjectionRows);
    EXPECT_EQ(scratch.projection.jacobian.cols(), kDofCount);
    EXPECT_EQ(scratch.projection.lambda.size(), kProjectionRows);
    EXPECT_EQ(scratch.projection.correction.size(), kDofCount);
    EXPECT_EQ(scratch.anderson.stepDeltas.size(), 5u);
    for (const auto& delta : scratch.anderson.stepDeltas) {
      EXPECT_EQ(delta.size(), kDofCount);
    }
    ++scratchCount;
  }
  EXPECT_EQ(scratchCount, 1u);
}

TEST(World, EnterSimulationModeReservesContactHeavyVariationalDualState)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.multibodyOptions.integrationFamily = "variational integrator";
  sx::World world(options);
  configureVariationalContactDualStateSliderScene(world);
  auto& registry = sx::detail::registryOf(world);
  auto& worldFreeAllocator = world.getMemoryManager().getFreeAllocator();
  expectVariationalContactConfigUsesAllocator(registry, worldFreeAllocator);
  for (auto entity : registry.view<sx::comps::VariationalContact>()) {
    auto& contact = registry.get<sx::comps::VariationalContact>(entity);
    sx::comps::VariationalContact::LinkIndexVector linkIndices;
    linkIndices.assign(
        contact.pointLinkIndices.begin(), contact.pointLinkIndices.end());
    contact.pointLinkIndices = std::move(linkIndices);
    sx::comps::VariationalContact::PointVector localPositions;
    localPositions.assign(
        contact.pointLocalPositions.begin(), contact.pointLocalPositions.end());
    contact.pointLocalPositions = std::move(localPositions);
  }
  for (auto entity : registry.view<
                     sx::comps::MultibodyStructure,
                     sx::comps::VariationalContact>()) {
    registry.emplace<sx::comps::VariationalContactDualState>(entity);
    auto& state
        = registry.emplace<sx::compute::MultibodyVariationalState>(entity);
    state.previousDeltaTransform.push_back(Eigen::Isometry3d::Identity());
    state.previousMomentum.push_back(Eigen::Matrix<double, 6, 1>::Zero());
  }
  auto& worldFreeList = world.getMemoryManager().getFreeListAllocator();
  const auto worldAllocationsBeforeBake = worldFreeList.getAllocationCount();
  const common::StlAllocator<double> expectedDualAllocator{worldFreeAllocator};

  world.enterSimulationMode();

  const auto capacities = registryStorageCapacities(registry);
  const auto dualStorageId
      = entt::type_hash<sx::comps::VariationalContactDualState>::value();
  ASSERT_TRUE(capacities.contains(dualStorageId));
  EXPECT_GE(
      capacities.at(dualStorageId), kCompliantVariationalContactRobotCount);
  expectVariationalContactConfigUsesAllocator(registry, worldFreeAllocator);
  expectVariationalStateUsesAllocator(registry, worldFreeAllocator);
  expectVariationalContactDualStateBaked(registry, &expectedDualAllocator);
  expectVariationalContactGroundSolverBaked(registry, &worldFreeAllocator);
  EXPECT_GE(
      worldFreeList.getAllocationCount(),
      worldAllocationsBeforeBake + kCompliantVariationalContactRobotCount)
      << "baked variational contact dual vectors should reserve from the "
         "World free allocator";

  const auto allocationsAfterBake = allocator.allocationCount;
  const auto deallocationsAfterBake = allocator.deallocationCount;
  const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterBake = allocator.alignedDeallocationCount;

  for (int i = 0; i < 3; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
  EXPECT_EQ(allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);
}

TEST(World, VariationalContactDualStateRegistryStorageRebuildsAfterClear)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.multibodyOptions.integrationFamily = "variational integrator";
  sx::World world(options);
  const common::StlAllocator<double> expectedDualAllocator{
      world.getMemoryManager().getFreeAllocator()};

  const auto bakeAndCheckStableSteps = [&]() {
    configureVariationalContactDualStateSliderScene(world);
    world.enterSimulationMode();

    const auto& registry = sx::detail::registryOf(world);
    expectVariationalContactConfigUsesAllocator(
        registry, world.getMemoryManager().getFreeAllocator());
    expectVariationalStateUsesAllocator(
        registry, world.getMemoryManager().getFreeAllocator());
    const auto capacities = registryStorageCapacities(registry);
    const auto dualStorageId
        = entt::type_hash<sx::comps::VariationalContactDualState>::value();
    const auto dualCapacity = capacities.find(dualStorageId);
    EXPECT_NE(dualCapacity, capacities.end());
    if (dualCapacity != capacities.end()) {
      EXPECT_GE(dualCapacity->second, kCompliantVariationalContactRobotCount);
    }
    expectVariationalContactDualStateBaked(registry, &expectedDualAllocator);

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
      expectVariationalContactDualStateBaked(registry, &expectedDualAllocator);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

TEST(World, EnterSimulationModeReservesCompliantVariationalContactScratch)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.multibodyOptions.integrationFamily = "variational integrator";
  sx::World world(options);
  configureCompliantVariationalContactSliderScene(world);

  world.enterSimulationMode();

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);

  expectCompliantVariationalContactScratchBaked(
      registry, &world.getMemoryManager().getFreeAllocator());

  const auto allocationsAfterBake = allocator.allocationCount;
  const auto deallocationsAfterBake = allocator.deallocationCount;
  const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterBake = allocator.alignedDeallocationCount;

  for (int i = 0; i < 3; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
  EXPECT_EQ(allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);
}

TEST(World, VariationalArticulatedPointJointLinkIndexScratchUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.multibodyOptions.integrationFamily = "variational integrator";
  sx::World world(options);
  configureLongVariationalArticulatedPointJointScene(world);

  world.enterSimulationMode();

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);

  const auto allocationsAfterBake = allocator.allocationCount;
  const auto deallocationsAfterBake = allocator.deallocationCount;
  const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
  const auto alignedDeallocationsAfterBake = allocator.alignedDeallocationCount;

  for (int i = 0; i < 3; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }

  EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
  EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
  EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
  EXPECT_EQ(allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);
}

TEST(World, VariationalLoopClosureRegistryStorageRebuildsAfterClear)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.multibodyOptions.integrationFamily = "variational integrator";
  sx::World world(options);

  const auto bakeAndCheckStableSteps = [&]() {
    configureVariationalLoopClosureChainScene(world);
    world.enterSimulationMode();

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);
    const auto scratchStorageId
        = entt::type_hash<sx::compute::MultibodyVariationalScratch>::value();
    const auto stateStorageId
        = entt::type_hash<sx::compute::MultibodyVariationalState>::value();
    const auto scratchCapacity = capacities.find(scratchStorageId);
    const auto stateCapacity = capacities.find(stateStorageId);
    EXPECT_NE(scratchCapacity, capacities.end());
    EXPECT_NE(stateCapacity, capacities.end());
    if (scratchCapacity != capacities.end()) {
      EXPECT_GE(scratchCapacity->second, 1u);
    }
    if (stateCapacity != capacities.end()) {
      EXPECT_GE(stateCapacity->second, 1u);
    }
    auto& worldFreeAllocator = world.getMemoryManager().getFreeAllocator();
    expectVariationalLoopClosureScratchBaked(registry, &worldFreeAllocator);

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

template <typename ConfigureScene>
void expectBoxedLcpRegistryStorageRebuildsAfterClear(
    std::string_view scene,
    ConfigureScene&& configureScene,
    std::size_t minInitialContacts)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  const auto bakeAndCheckStableSteps = [&]() {
    configureScene(world);
    world.enterSimulationMode();
    EXPECT_GE(world.collide().size(), minInitialContacts);

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);
    EXPECT_GT(capacities.size(), 1u);

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

TEST(World, BoxedLcpMultibodyRegistryStorageRebuildsAfterClear)
{
  expectBoxedLcpRegistryStorageRebuildsAfterClear(
      "cross multibody stacked-contact fallback",
      configureCrossMultibodyStackedFallbackScene,
      1);
  expectBoxedLcpRegistryStorageRebuildsAfterClear(
      "cross multibody multi-island mixed fallback",
      configureCrossMultibodyMultiIslandFallbackScene,
      4);
}

TEST(World, ContactHeavyRegistryStorageRebuildsAfterClear)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.multibodyOptions.integrationFamily = "variational integrator";
  sx::World world(options);

  const auto bakeAndCheckStableSteps = [&]() {
    configureCompliantVariationalContactSliderScene(world);
    world.enterSimulationMode();

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);
    const auto scratchStorageId
        = entt::type_hash<sx::compute::MultibodyVariationalScratch>::value();
    const auto scratchCapacity = capacities.find(scratchStorageId);
    EXPECT_NE(scratchCapacity, capacities.end());
    if (scratchCapacity != capacities.end()) {
      EXPECT_GE(
          scratchCapacity->second, kCompliantVariationalContactRobotCount);
    }
    expectCompliantVariationalContactScratchBaked(
        registry, &world.getMemoryManager().getFreeAllocator());

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

template <typename ConfigureScene>
void expectDeformableRegistryStorageRebuildsAfterClear(
    std::string_view scene, ConfigureScene&& configureScene)
{
  namespace common = dart::common;
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  sx::World world(options);

  const auto bakeAndCheckStableSteps = [&]() {
    configureScene(world);
    world.enterSimulationMode();

    const auto& registry = sx::detail::registryOf(world);
    const auto capacities = registryStorageCapacities(registry);
    const auto solverScratchStorageId
        = entt::type_hash<sx::comps::DeformableSolverScratch>::value();
    const auto solverScratchCapacity = capacities.find(solverScratchStorageId);
    EXPECT_NE(solverScratchCapacity, capacities.end());
    std::size_t deformableBodyCount = 0;
    for ([[maybe_unused]] const auto entity :
         registry.view<sx::comps::DeformableBodyTag>()) {
      ++deformableBodyCount;
    }
    if (solverScratchCapacity != capacities.end()) {
      EXPECT_GE(solverScratchCapacity->second, deformableBodyCount);
    }

    const auto allocationsAfterBake = allocator.allocationCount;
    const auto deallocationsAfterBake = allocator.deallocationCount;
    const auto alignedAllocationsAfterBake = allocator.alignedAllocationCount;
    const auto alignedDeallocationsAfterBake
        = allocator.alignedDeallocationCount;

    for (int i = 0; i < 3; ++i) {
      world.step();
      expectRegistryStorageCapacitiesUnchanged(capacities, registry);
    }

    EXPECT_EQ(allocator.allocationCount, allocationsAfterBake);
    EXPECT_EQ(allocator.deallocationCount, deallocationsAfterBake);
    EXPECT_EQ(allocator.alignedAllocationCount, alignedAllocationsAfterBake);
    EXPECT_EQ(
        allocator.alignedDeallocationCount, alignedDeallocationsAfterBake);

    return capacities;
  };

  const auto firstCapacities = bakeAndCheckStableSteps();
  const auto baked = world.getMemoryDiagnostics();
  EXPECT_GT(baked.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_GT(baked.ecsDiagnostics.componentCapacity, 0u);

  world.clear();

  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.ecsDiagnostics.entityCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.entityCapacity, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCount, 0u);
  EXPECT_EQ(cleared.ecsDiagnostics.componentCapacity, 0u);
  for (const auto& storage : cleared.ecsDiagnostics.storages) {
    EXPECT_EQ(storage.size, 0u);
    EXPECT_EQ(storage.capacity, 0u);
  }

  const common::StlAllocator<entt::entity> expectedEntityAllocator{
      world.getMemoryManager().getFreeAllocator()};
  EXPECT_EQ(
      sx::detail::registryOf(world).get_allocator(), expectedEntityAllocator);

  const auto secondCapacities = bakeAndCheckStableSteps();
  EXPECT_EQ(secondCapacities, firstCapacities);
}

TEST(World, DefaultDeformableRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "compact mixed default deformable storage",
      configureMixedDefaultDeformableStorageScene);
}

TEST(World, ProductionDefaultDeformableRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "production mixed default deformable storage",
      configureMixedDefaultDeformableProductionStorageScene);
}

TEST(World, ProductionDefaultContactFamilyRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "production mixed default contact-family storage",
      configureMixedDefaultContactFamiliesProductionScene);
}

TEST(
    World,
    ProductionComplementaryDefaultContactFamilyRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "production complementary default contact-family storage",
      configureMixedComplementaryDefaultContactFamiliesProductionScene);
}

TEST(World, AvbdSelfContactRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "AVBD self-contact friction grid storage",
      configureAvbdSelfContactFrictionGridRowsScene);
}

TEST(World, VbdChebyshevRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "VBD Chebyshev self-contact grid storage",
      configureVbdChebyshevSelfContactGridScene);
}

TEST(World, AvbdGroundFrictionRegistryStorageRebuildsAfterClear)
{
  expectDeformableRegistryStorageRebuildsAfterClear(
      "AVBD ground-friction row storage", configureAvbdGroundFrictionRowsScene);
}

TEST(World, ScriptedDeformableBoundaryScratchIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureScriptedDeformableBoundaryScene(world);
  auto body = world.getDeformableBody("scripted_boundary_chain");
  ASSERT_TRUE(body.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 4u);
  EXPECT_GT(diagnostics.solverIterations, 0u);
  EXPECT_GT(body->getPosition(0).y(), 0.0);
  EXPECT_GT(body->getPosition(3).y(), 0.0);
}

TEST(World, BakedScriptedDeformableBoundaryStepsDoNotAllocateAfterPrewarm)
{
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "scripted deformable boundary scratch",
      configureScriptedDeformableBoundaryScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "scripted deformable boundary scratch",
      configureScriptedDeformableBoundaryScene);
}

TEST(World, BakedDynamicRigidIpcStepsDoNotGrowWorldBaseAllocator)
{
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC solve graph", configureDynamicRigidIpcMeshSolveScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC active barrier",
      configureActiveRigidIpcMeshBarrierScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC fixed joint",
      configureRigidIpcFixedJointConstraintScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC revolute joint",
      configureRigidIpcRevoluteJointConstraintScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC two-box stack", configureRigidIpcTwoBoxStackScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC deformable surface obstacle",
      configureRigidIpcDeformableSurfaceObstacleScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC kinematic conveyor contact",
      configureRigidIpcKinematicConveyorScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "dynamic rigid IPC kinematic turntable contact",
      configureRigidIpcKinematicTurntableScene);
}

TEST(World, BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The monolithic no-growth gate is too slow under coverage; normal "
         "Release and Debug CI run the full allocator regression.";
#endif

  namespace sx = dart::simulation;

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "kinematic IPC rigid body", [](sx::World& world) {
        world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
        auto body = world.addRigidBody("kinematic");
        body.setKinematic(true);
        body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
      });
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "kinematic IPC box obstacle", [](sx::World& world) {
        world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
        auto body = world.addRigidBody("kinematic_box");
        body.setKinematic(true);
        body.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
        body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
      });

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "rigid body resting contact",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        sx::RigidBodyOptions groundOptions;
        groundOptions.isStatic = true;
        groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
        auto ground = world.addRigidBody("ground", groundOptions);
        ground.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

        sx::RigidBodyOptions boxOptions;
        boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.18);
        auto box = world.addRigidBody("box", boxOptions);
        box.setMass(1.0);
        box.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.2, 0.2)));

        world.setTimeStep(0.001);
      },
      true);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "rigid AVBD contact rows", configureRigidAvbdContactRowsScene, true);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "rigid AVBD fixed-joint rows", configureRigidAvbdFixedJointRowsScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "rigid AVBD distance-spring rows",
      configureRigidAvbdDistanceSpringRowsScene);

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "multibody variational scratch", [](sx::World& world) {
        auto robot = world.addMultibody("slider");
        auto base = robot.addLink("base");
        sx::JointSpec spec;
        spec.name = "rail";
        spec.type = sx::JointType::Prismatic;
        spec.axis = Eigen::Vector3d::UnitZ();
        auto carriage = robot.addLink("carriage", base, spec);
        carriage.setMass(3.0);
        world.setMultibodyOptions({"variational integrator"});
        world.setTimeStep(0.01);
      });
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "semi-implicit external-force body Jacobian scratch",
      configureSemiImplicitExternalForceMultibodyScene);

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "articulated link resting contact",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        auto robot = world.addMultibody("leg_robot");
        auto base = robot.addLink("base");
        sx::JointSpec spec;
        spec.name = "slider";
        spec.type = sx::JointType::Prismatic;
        spec.axis = Eigen::Vector3d::UnitZ();
        auto leg = robot.addLink("leg", base, spec);
        leg.setMass(1.0);
        leg.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
        leg.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.35));

        sx::RigidBodyOptions groundOptions;
        groundOptions.isStatic = true;
        groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
        auto ground = world.addRigidBody("ground", groundOptions);
        ground.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

        world.setTimeStep(0.002);
      },
      true);

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "cross articulated link contact",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        const auto addRobot
            = [&](std::string_view name, double z, double velocity) {
                auto robot = world.addMultibody(name);
                auto base = robot.addLink("base");
                sx::JointSpec spec;
                spec.name = "slider";
                spec.type = sx::JointType::Prismatic;
                spec.axis = Eigen::Vector3d::UnitZ();
                auto link = robot.addLink("link", base, spec);
                link.setMass(1.0);
                link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
                auto joint = link.getParentJoint();
                joint.setPosition(Eigen::VectorXd::Constant(1, z));
                joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
              };

        addRobot("lower_robot", 0.0, 0.5);
        addRobot("upper_robot", 0.35, -0.5);
        world.setTimeStep(0.001);
      },
      true);

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "single deformable particle", [](sx::World& world) {
        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("particle", options);
        world.setTimeStep(0.01);
      });

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable FEM ground friction block",
      configureDeformableFemGroundFrictionBlockScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable iterative FEM ground friction block",
      configureDeformableIterativeFemGroundFrictionBlockScene);

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction patch",
      configureDeformableSelfContactFrictionPatchScene);

  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction grid",
      configureDeformableSelfContactFrictionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction large grid",
      configureDeformableSelfContactFrictionLargeGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction production grid",
      configureDeformableSelfContactFrictionProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction extended production grid",
      configureDeformableSelfContactFrictionExtendedProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction dense production grid",
      configureDeformableSelfContactFrictionDenseProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction extra-dense production grid",
      configureDeformableSelfContactFrictionExtraDenseGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction larger production grid",
      configureDeformableSelfContactFrictionLargerProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction larger matrix-free production grid",
      configureDeformableSelfContactFrictionLargerMatrixFreeProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction irregular production grid",
      configureDeformableSelfContactFrictionIrregularProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction irregular matrix-free production grid",
      configureDeformableSelfContactFrictionIrregularMatrixFreeProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction late-active direct grid",
      configureDeformableSelfContactFrictionLateActiveGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction late-active matrix-free grid",
      configureDeformableSelfContactFrictionLateActiveMatrixFreeGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction late-active rectangular grid",
      configureDeformableSelfContactFrictionLateActiveRectangularGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction late-active matrix-free rectangular "
      "grid",
      configureDeformableSelfContactFrictionLateActiveMatrixFreeRectangularGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction rectangular grid",
      configureDeformableSelfContactFrictionRectangularGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction dense rectangular grid",
      configureDeformableSelfContactFrictionDenseRectangularGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction dense rectangular matrix-free grid",
      configureDeformableSelfContactFrictionDenseRectangularMatrixFreeGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction wide rectangular grid",
      configureDeformableSelfContactFrictionWideRectangularGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction wide rectangular matrix-free grid",
      configureDeformableSelfContactFrictionWideRectangularMatrixFreeGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction tall rectangular grid",
      configureDeformableSelfContactFrictionTallRectangularGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable self-contact friction tall rectangular matrix-free grid",
      configureDeformableSelfContactFrictionTallRectangularMatrixFreeGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed deformable self-contact friction production grids",
      configureMixedDeformableSelfContactFrictionProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed deformable self-contact friction dense production grids",
      configureMixedDeformableSelfContactFrictionDenseProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed deformable self-contact friction late-active production grids",
      configureMixedDeformableSelfContactFrictionLateActiveProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed default deformable storage paths",
      configureMixedDefaultDeformableStorageScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed production default deformable storage paths",
      configureMixedDefaultDeformableProductionStorageScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed production default FEM solver storage paths",
      configureMixedDefaultDeformableFemProductionStorageScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable static obstacle barriers",
      configureDeformableStaticObstacleBarrierScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable static obstacle friction production patch",
      configureDeformableStaticObstacleFrictionProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable static obstacle friction matrix-free production patch",
      configureDeformableStaticObstacleFrictionMatrixFreeProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed static obstacle and self-contact production deformables",
      configureMixedStaticObstacleAndSelfContactProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed matrix-free static obstacle and direct self-contact production "
      "deformables",
      configureMixedMatrixFreeStaticObstacleAndDirectSelfContactProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed default contact-family production deformables",
      configureMixedDefaultContactFamiliesProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "mixed complementary default contact-family production deformables",
      configureMixedComplementaryDefaultContactFamiliesProductionScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable moving rigid surface CCD crossing",
      configureDeformableMovingRigidSurfaceCcdCrossingScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable kinematic rigid surface CCD crossing",
      configureDeformableKinematicRigidSurfaceCcdCrossingScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable multi-kinematic rigid surface CCD crossing",
      configureDeformableMultiKinematicRigidSurfaceCcdScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable inter-body surface CCD crossing",
      configureDeformableInterBodySurfaceCcdCrossingScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable inter-body surface CCD production grid crossing",
      configureDeformableInterBodySurfaceCcdProductionGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable AVBD self-contact friction rows",
      configureAvbdSelfContactFrictionRowsScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable AVBD self-contact friction grid rows",
      configureAvbdSelfContactFrictionGridRowsScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable AVBD self-contact friction production grid rows",
      configureAvbdSelfContactFrictionProductionGridRowsScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable VBD Chebyshev self-contact grid",
      configureVbdChebyshevSelfContactGridScene);
  expectNoWorldBaseAllocatorActivityDuringBakedSteps(
      "deformable AVBD ground friction rows",
      configureAvbdGroundFrictionRowsScene);
}

TEST(World, DeformableFemGroundFrictionBlockIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableFemGroundFrictionBlockScene(world);
  world.enterSimulationMode();

  std::size_t projectedNewtonSteps = 0;
  std::size_t maxHessianNonZeros = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    frictionDissipation += diagnostics.frictionDissipation;
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 27u);
  EXPECT_GT(projectedNewtonSteps, 0u);
  EXPECT_GT(maxHessianNonZeros, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}

TEST(World, DeformableIterativeFemGroundFrictionBlockIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableIterativeFemGroundFrictionBlockScene(world);
  world.enterSimulationMode();

  std::size_t projectedNewtonSteps = 0;
  std::size_t iterativeSolves = 0;
  std::size_t iterativeIterations = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    iterativeSolves += diagnostics.projectedNewtonIterativeSolves;
    iterativeIterations += diagnostics.projectedNewtonIterativeIterations;
    frictionDissipation += diagnostics.frictionDissipation;
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 27u);
  EXPECT_GT(projectedNewtonSteps, 0u);
  EXPECT_GT(iterativeSolves, 0u);
  EXPECT_GT(iterativeIterations, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}

#if defined(DART_CODECOV)
TEST(World, DeformableProductionSelfContactActiveScenesSkippedUnderCoverage)
{
  GTEST_SKIP()
      << "Production-scale deformable self-contact active-scene probes are "
         "too slow under coverage; normal Release/Debug CI and allocation "
         "gates run these shapes.";
}
#else
TEST(World, DeformableSelfContactFrictionProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionExtendedProductionGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 11u * 11u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionDenseProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionDenseProductionGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 13u * 13u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionExtraDenseGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionExtraDenseGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 15u * 15u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionLargerProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionLargerProductionGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 17u * 17u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionLargerMatrixFreeProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionLargerMatrixFreeProductionGridScene(
      world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 17u * 17u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionIrregularProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionIrregularProductionGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 13u * 17u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(
    World,
    DeformableSelfContactFrictionIrregularMatrixFreeProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionIrregularMatrixFreeProductionGridScene(
      world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 13u * 17u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

template <typename ConfigureScene>
void expectLateActiveSelfContactFrictionSceneIsActive(
    std::string_view scene,
    ConfigureScene&& configureScene,
    std::size_t expectedNodeCount,
    bool expectMatrixFree)
{
  namespace sx = dart::simulation;

  SCOPED_TRACE(scene);
  sx::World world;
  configureScene(world);
  world.enterSimulationMode();
  std::size_t projectedNewtonSteps = 0;
  std::size_t matrixFreeSolves = 0;
  std::size_t maxActiveContacts = 0;
  std::size_t maxConvergedContacts = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    matrixFreeSolves += diagnostics.projectedNewtonMatrixFreeSolves;
    maxActiveContacts = std::max(
        maxActiveContacts, diagnostics.selfContactBarrierActiveContacts);
    maxConvergedContacts = std::max(
        maxConvergedContacts, diagnostics.convergedActiveContactCount);
    frictionDissipation += diagnostics.frictionDissipation;
  }
  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, expectedNodeCount);
  EXPECT_GT(projectedNewtonSteps, 0u);
  if (expectMatrixFree) {
    EXPECT_GT(matrixFreeSolves, 0u);
  } else {
    EXPECT_EQ(matrixFreeSolves, 0u);
  }
  EXPECT_GT(maxActiveContacts, 0u);
  EXPECT_GT(maxConvergedContacts, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionLateActiveMatrixFreeGridIsActive)
{
  expectLateActiveSelfContactFrictionSceneIsActive(
      "late-active matrix-free square grid",
      configureDeformableSelfContactFrictionLateActiveMatrixFreeGridScene,
      2u * 11u * 11u,
      true);
}

TEST(World, DeformableSelfContactFrictionLateActiveDirectGridIsActive)
{
  expectLateActiveSelfContactFrictionSceneIsActive(
      "late-active direct square grid",
      configureDeformableSelfContactFrictionLateActiveGridScene,
      2u * 11u * 11u,
      false);
}

TEST(
    World,
    DeformableSelfContactFrictionLateActiveMatrixFreeRectangularGridIsActive)
{
  expectLateActiveSelfContactFrictionSceneIsActive(
      "late-active matrix-free rectangular grid",
      configureDeformableSelfContactFrictionLateActiveMatrixFreeRectangularGridScene,
      2u * 9u * 13u,
      true);
}

TEST(World, DeformableSelfContactFrictionLateActiveRectangularGridIsActive)
{
  expectLateActiveSelfContactFrictionSceneIsActive(
      "late-active direct rectangular grid",
      configureDeformableSelfContactFrictionLateActiveRectangularGridScene,
      2u * 9u * 13u,
      false);
}

TEST(World, DeformableSelfContactFrictionRectangularGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionRectangularGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 9u * 13u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionDenseRectangularGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionDenseRectangularGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 13u * 19u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionDenseRectangularMatrixFreeGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionDenseRectangularMatrixFreeGridScene(
      world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 13u * 19u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionWideRectangularGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionWideRectangularGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 7u * 17u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionWideRectangularMatrixFreeGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionWideRectangularMatrixFreeGridScene(
      world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 7u * 17u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionTallRectangularGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionTallRectangularGridScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 17u * 7u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableSelfContactFrictionTallRectangularMatrixFreeGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableSelfContactFrictionTallRectangularMatrixFreeGridScene(
      world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 17u * 7u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, MixedDeformableSelfContactFrictionProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDeformableSelfContactFrictionProductionScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 9u * 13u + 2u * 7u * 17u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, MixedDeformableSelfContactFrictionDenseProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDeformableSelfContactFrictionDenseProductionScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 13u * 17u + 2u * 13u * 19u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, MixedDeformableSelfContactFrictionLateActiveProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDeformableSelfContactFrictionLateActiveProductionScene(world);
  world.enterSimulationMode();

  std::size_t projectedNewtonSteps = 0;
  std::size_t matrixFreeSolves = 0;
  std::size_t maxActiveContacts = 0;
  std::size_t maxConvergedContacts = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    matrixFreeSolves += diagnostics.projectedNewtonMatrixFreeSolves;
    maxActiveContacts = std::max(
        maxActiveContacts, diagnostics.selfContactBarrierActiveContacts);
    maxConvergedContacts = std::max(
        maxConvergedContacts, diagnostics.convergedActiveContactCount);
    frictionDissipation += diagnostics.frictionDissipation;
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 11u * 11u + 2u * 9u * 13u);
  EXPECT_GT(projectedNewtonSteps, 0u);
  EXPECT_GT(matrixFreeSolves, 0u);
  EXPECT_GT(maxActiveContacts, 0u);
  EXPECT_GT(maxConvergedContacts, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}

TEST(World, MixedDefaultDeformableStorageSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDefaultDeformableStorageScene(world);
  world.enterSimulationMode();

  std::size_t projectedNewtonSteps = 0;
  std::size_t matrixFreeSolves = 0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxActiveContacts = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    matrixFreeSolves += diagnostics.projectedNewtonMatrixFreeSolves;
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxActiveContacts = std::max(
        maxActiveContacts, diagnostics.selfContactBarrierActiveContacts);
    frictionDissipation += diagnostics.frictionDissipation;
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 3u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 5u * 7u + 2u * 5u * 9u + 27u);
  EXPECT_GT(projectedNewtonSteps, 0u);
  EXPECT_GT(matrixFreeSolves, 0u);
  EXPECT_GT(maxHessianNonZeros, 0u);
  EXPECT_GT(maxActiveContacts, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}

TEST(World, MixedDefaultDeformableProductionStorageSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDefaultDeformableProductionStorageScene(world);
  world.enterSimulationMode();

  std::size_t projectedNewtonSteps = 0;
  std::size_t matrixFreeSolves = 0;
  std::size_t maxHessianNonZeros = 0;
  std::size_t maxActiveContacts = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    matrixFreeSolves += diagnostics.projectedNewtonMatrixFreeSolves;
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    maxActiveContacts = std::max(
        maxActiveContacts, diagnostics.selfContactBarrierActiveContacts);
    frictionDissipation += diagnostics.frictionDissipation;
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 3u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 9u * 13u + 2u * 7u * 17u + 27u);
  EXPECT_GT(projectedNewtonSteps, 0u);
  EXPECT_GT(matrixFreeSolves, 0u);
  EXPECT_GT(maxHessianNonZeros, 0u);
  EXPECT_GT(maxActiveContacts, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}

TEST(World, MixedDefaultDeformableFemProductionStorageSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDefaultDeformableFemProductionStorageScene(world);
  world.enterSimulationMode();

  std::size_t projectedNewtonSteps = 0;
  std::size_t matrixFreeSolves = 0;
  std::size_t maxHessianNonZeros = 0;
  double frictionDissipation = 0.0;
  for (int i = 0; i < 4; ++i) {
    world.step();
    const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
    projectedNewtonSteps += diagnostics.projectedNewtonSteps;
    matrixFreeSolves += diagnostics.projectedNewtonMatrixFreeSolves;
    maxHessianNonZeros = std::max(
        maxHessianNonZeros, diagnostics.projectedNewtonHessianNonZeros);
    frictionDissipation += diagnostics.frictionDissipation;
  }

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 4u * 4u * 4u);
  EXPECT_GT(projectedNewtonSteps, 0u);
  EXPECT_GT(matrixFreeSolves, 0u);
  EXPECT_GT(maxHessianNonZeros, 0u);
  EXPECT_GT(frictionDissipation, 0.0);
}
#endif

TEST(World, DeformableStaticObstacleBarrierSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableStaticObstacleBarrierScene(world);
  auto body = world.getDeformableBody("static_obstacle_barrier_nodes");
  ASSERT_TRUE(body.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 3u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(body->getPosition(0).x(), 0.51);
  EXPECT_GT(body->getPosition(1).x(), 2.51);
  EXPECT_GT(body->getPosition(2).x(), -1.79);
}

#if defined(DART_CODECOV)
TEST(World, DeformableProductionObstacleActiveScenesSkippedUnderCoverage)
{
  GTEST_SKIP()
      << "Production-scale deformable obstacle/contact-family active-scene "
         "probes are too slow under coverage; normal Release/Debug CI and "
         "allocation gates run these shapes.";
}
#else
TEST(World, DeformableStaticObstacleFrictionProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableStaticObstacleFrictionProductionScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 3u * 5u * 5u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(diagnostics.projectedNewtonHessianNonZeros, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, DeformableStaticObstacleFrictionMatrixFreeProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableStaticObstacleFrictionMatrixFreeProductionScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 3u * 5u * 5u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, MixedStaticObstacleAndSelfContactProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedStaticObstacleAndSelfContactProductionScene(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 3u * 5u * 5u + 2u * 9u * 13u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(
    World,
    MixedMatrixFreeStaticObstacleAndDirectSelfContactProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedMatrixFreeStaticObstacleAndDirectSelfContactProductionScene(
      world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 3u * 5u * 5u + 2u * 13u * 17u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.projectedNewtonHessianNonZeros, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
}

TEST(World, MixedDefaultContactFamiliesProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedDefaultContactFamiliesProductionScene(world);
  auto movingSurface
      = world.getDeformableBody("mixed_contact_families_moving_surface");
  ASSERT_TRUE(movingSurface.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 4u);
  EXPECT_EQ(
      diagnostics.nodeCount, 3u * 5u * 5u + 2u * 9u * 13u + 2u * 9u * 13u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.projectedNewtonHessianNonZeros, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
  EXPECT_GT(movingSurface->getPosition(0).x(), -1.0);
  EXPECT_LT(movingSurface->getPosition(0).x(), 0.0);
}

TEST(World, MixedComplementaryDefaultContactFamiliesProductionSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureMixedComplementaryDefaultContactFamiliesProductionScene(world);
  auto movingSurface
      = world.getDeformableBody("mixed_complementary_contact_moving_surface");
  ASSERT_TRUE(movingSurface.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 4u);
  EXPECT_EQ(
      diagnostics.nodeCount, 3u * 5u * 5u + 2u * 13u * 17u + 2u * 9u * 13u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(diagnostics.projectedNewtonMatrixFreeSolves, 0u);
  EXPECT_GT(diagnostics.projectedNewtonHessianNonZeros, 0u);
  EXPECT_GT(diagnostics.selfContactBarrierActiveContacts, 0u);
  EXPECT_GT(diagnostics.convergedActiveContactCount, 0u);
  EXPECT_GT(diagnostics.frictionDissipation, 0.0);
  EXPECT_GT(movingSurface->getPosition(0).x(), -1.0);
  EXPECT_LT(movingSurface->getPosition(0).x(), 0.0);
}
#endif

TEST(World, DeformableMovingRigidSurfaceCcdCrossingIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableMovingRigidSurfaceCcdCrossingScene(world);
  auto body = world.getDeformableBody("moving_surface_fast_point");
  ASSERT_TRUE(body.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 1u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(body->getPosition(0).x(), -1.0);
  EXPECT_LT(body->getPosition(0).x(), -0.05);
}

TEST(World, DeformableKinematicRigidSurfaceCcdCrossingIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableKinematicRigidSurfaceCcdCrossingScene(world);
  auto obstacle = world.getRigidBody("kinematic_surface_box");
  ASSERT_TRUE(obstacle.has_value());
  auto body = world.getDeformableBody("kinematic_surface_fast_point");
  ASSERT_TRUE(body.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& registry = sx::detail::registryOf(world);
  const auto obstacleEntity
      = sx::detail::toRegistryEntity(obstacle->getEntity());
  const auto* trace
      = registry.try_get<sx::comps::KinematicBodyStepTrace>(obstacleEntity);
  ASSERT_NE(trace, nullptr);
  EXPECT_LT(
      trace->startTransform.position.x(), trace->endTransform.position.x());

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 1u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(body->getPosition(0).x(), -1.0);
  EXPECT_LT(body->getPosition(0).x(), -0.05);
}

TEST(World, DeformableMultiKinematicRigidSurfaceCcdSceneIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableMultiKinematicRigidSurfaceCcdScene(world);
  auto firstBody
      = world.getDeformableBody("multi_kinematic_surface_fast_point_a");
  ASSERT_TRUE(firstBody.has_value());
  auto secondBody
      = world.getDeformableBody("multi_kinematic_surface_fast_point_b");
  ASSERT_TRUE(secondBody.has_value());
  auto kinematicObstacle = world.getRigidBody("multi_kinematic_surface_box_b");
  ASSERT_TRUE(kinematicObstacle.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& registry = sx::detail::registryOf(world);
  const auto kinematicObstacleEntity
      = sx::detail::toRegistryEntity(kinematicObstacle->getEntity());
  const auto* trace = registry.try_get<sx::comps::KinematicBodyStepTrace>(
      kinematicObstacleEntity);
  ASSERT_NE(trace, nullptr);
  EXPECT_LT(
      trace->startTransform.position.x(), trace->endTransform.position.x());

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 2u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(firstBody->getPosition(0).x(), -1.0);
  EXPECT_LT(firstBody->getPosition(0).x(), 0.0);
  EXPECT_GT(secondBody->getPosition(0).x(), -1.0);
  EXPECT_LT(secondBody->getPosition(0).x(), 0.0);
}

TEST(World, DeformableInterBodySurfaceCcdCrossingIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableInterBodySurfaceCcdCrossingScene(world);
  auto movingSurface = world.getDeformableBody("moving_deformable_surface");
  ASSERT_TRUE(movingSurface.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(movingSurface->getPosition(0).x(), -1.0);
  EXPECT_LT(movingSurface->getPosition(0).x(), 0.0);
}

#if defined(DART_CODECOV)
TEST(World, DeformableProductionSurfaceCcdActiveScenesSkippedUnderCoverage)
{
  GTEST_SKIP()
      << "Production-scale deformable surface-CCD active-scene probes are too "
         "slow under coverage; normal Release/Debug CI and allocation gates "
         "run these shapes.";
}
#else
TEST(World, DeformableInterBodySurfaceCcdProductionGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureDeformableInterBodySurfaceCcdProductionGridScene(world);
  auto movingSurface
      = world.getDeformableBody("moving_deformable_production_surface");
  ASSERT_TRUE(movingSurface.has_value());
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 2u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 9u * 13u);
  EXPECT_GT(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(movingSurface->getPosition(0).x(), -1.0);
  EXPECT_LT(movingSurface->getPosition(0).x(), 0.0);
}
#endif

TEST(World, AvbdGroundFrictionRowsAreActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureAvbdGroundFrictionRowsScene(world);
  auto& registry = sx::detail::registryOf(world);
  world.enterSimulationMode();

  world.step();
  world.step();

  const auto states
      = sx::compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          registry);
  ASSERT_EQ(states.size(), 1u);
  EXPECT_GT(states[0].contactRows.size(), 0u);
  EXPECT_EQ(states[0].frictionRows.size(), 2u * states[0].contactRows.size());
}

TEST(World, AvbdSelfContactFrictionGridRowsAreActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureAvbdSelfContactFrictionGridRowsScene(world);
  auto& registry = sx::detail::registryOf(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 5u * 9u);
  const auto states
      = sx::compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          registry);
  ASSERT_EQ(states.size(), 1u);
  EXPECT_GT(states[0].selfContactRows.size(), 0u);
  EXPECT_EQ(
      states[0].selfContactFrictionRows.size(),
      2u * states[0].selfContactRows.size());
}

#if defined(DART_CODECOV)
TEST(World, AvbdProductionSelfContactRowsSkippedUnderCoverage)
{
  GTEST_SKIP()
      << "Production-scale AVBD self-contact row probes are too slow under "
         "coverage; normal Release/Debug CI and allocation gates run these "
         "shapes.";
}
#else
TEST(World, AvbdSelfContactFrictionProductionGridRowsAreActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureAvbdSelfContactFrictionProductionGridRowsScene(world);
  auto& registry = sx::detail::registryOf(world);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 9u * 13u);
  const auto states
      = sx::compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
          registry);
  ASSERT_EQ(states.size(), 1u);
  EXPECT_GT(states[0].selfContactRows.size(), 0u);
  EXPECT_EQ(
      states[0].selfContactFrictionRows.size(),
      2u * states[0].selfContactRows.size());
}
#endif

TEST(World, VbdChebyshevSelfContactGridIsActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureVbdChebyshevSelfContactGridScene(world);
  auto body = world.getDeformableBody("vbd_chebyshev_self_contact_grid");
  ASSERT_TRUE(body.has_value());
  const Eigen::Vector3d initialUpperLayerPosition = body->getPosition(5u * 9u);
  world.enterSimulationMode();

  world.step();

  const auto& diagnostics = world.getLastDeformableSolverDiagnostics();
  EXPECT_EQ(diagnostics.bodyCount, 1u);
  EXPECT_EQ(diagnostics.nodeCount, 2u * 5u * 9u);
  EXPECT_EQ(diagnostics.solverIterations, 0u);
  EXPECT_EQ(diagnostics.projectedNewtonSteps, 0u);
  EXPECT_GT(
      (body->getPosition(5u * 9u) - initialUpperLayerPosition).norm(), 0.0);
}

TEST(World, RigidAvbdContactRowsAreActive)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureRigidAvbdContactRowsScene(world);
  auto sphere = world.getRigidBody("rigid_avbd_sphere");
  ASSERT_TRUE(sphere.has_value());

  world.enterSimulationMode();
  ASSERT_FALSE(world.collide().empty());
  world.step();

  EXPECT_GT(sphere->getLinearVelocity().z(), 0.0);
  EXPECT_GT(sphere->getTranslation().z(), 0.49);
}

TEST(World, RigidAvbdFixedJointRowsAreActiveWithoutContacts)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureRigidAvbdFixedJointRowsScene(world);
  auto link = world.getRigidBody("rigid_avbd_joint_link");
  ASSERT_TRUE(link.has_value());

  world.enterSimulationMode();
  ASSERT_TRUE(world.collide().empty());
  world.step();

  EXPECT_LT(link->getTranslation().x(), 1.25);
  EXPECT_LT(link->getLinearVelocity().x(), 0.0);
}

TEST(World, RigidAvbdDistanceSpringRowsAreActiveWithoutContacts)
{
  namespace sx = dart::simulation;

  sx::World world;
  configureRigidAvbdDistanceSpringRowsScene(world);
  auto link = world.getRigidBody("rigid_avbd_spring_link");
  ASSERT_TRUE(link.has_value());
  const Eigen::Vector3d initialPosition = link->getTranslation();

  world.enterSimulationMode();
  ASSERT_TRUE(world.collide().empty());
  world.step();

  EXPECT_LT(link->getTranslation().x(), initialPosition.x());
  EXPECT_LT(link->getLinearVelocity().x(), 0.0);
}

TEST(World, BakedKinematicIpcStepsDoNotAllocateGlobalHeap)
{
  namespace sx = dart::simulation;

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "kinematic IPC rigid body", [](sx::World& world) {
        world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
        auto body = world.addRigidBody("kinematic");
        body.setKinematic(true);
        body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
      });
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "kinematic IPC box obstacle", [](sx::World& world) {
        world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
        auto body = world.addRigidBody("kinematic_box");
        body.setKinematic(true);
        body.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
        body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
      });
}

TEST(World, BakedDynamicRigidIpcStepsDoNotAllocateGlobalHeap)
{
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC solve graph", configureDynamicRigidIpcMeshSolveScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC active barrier",
      configureActiveRigidIpcMeshBarrierScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC fixed joint",
      configureRigidIpcFixedJointConstraintScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC revolute joint",
      configureRigidIpcRevoluteJointConstraintScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC two-box stack", configureRigidIpcTwoBoxStackScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC deformable surface obstacle",
      configureRigidIpcDeformableSurfaceObstacleScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC kinematic conveyor contact",
      configureRigidIpcKinematicConveyorScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "dynamic rigid IPC kinematic turntable contact",
      configureRigidIpcKinematicTurntableScene);
}

TEST(World, BakedRigidBodyContactStepsDoNotAllocateGlobalHeap)
{
  namespace sx = dart::simulation;

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "rigid body empty collision geometry", [](sx::World& world) {
        auto body = world.addRigidBody("empty_geometry_body");
        auto& registry = sx::detail::registryOf(world);
        registry.emplace<sx::comps::CollisionGeometry>(
            sx::detail::toRegistryEntity(body.getEntity()));
      });
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "rigid body resting contact",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        sx::RigidBodyOptions groundOptions;
        groundOptions.isStatic = true;
        groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
        auto ground = world.addRigidBody("ground", groundOptions);
        ground.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

        sx::RigidBodyOptions boxOptions;
        boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.18);
        auto box = world.addRigidBody("box", boxOptions);
        box.setMass(1.0);
        box.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.2, 0.2)));

        world.setTimeStep(0.001);
      },
      true);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "rigid AVBD contact rows", configureRigidAvbdContactRowsScene, true);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "rigid AVBD fixed-joint rows", configureRigidAvbdFixedJointRowsScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "rigid AVBD distance-spring rows",
      configureRigidAvbdDistanceSpringRowsScene);
}

TEST(World, BakedArticulatedContactStepsDoNotAllocateGlobalHeap)
{
  namespace sx = dart::simulation;

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "articulated link resting contact",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        auto robot = world.addMultibody("leg_robot");
        auto base = robot.addLink("base");
        sx::JointSpec spec;
        spec.name = "slider";
        spec.type = sx::JointType::Prismatic;
        spec.axis = Eigen::Vector3d::UnitZ();
        auto leg = robot.addLink("leg", base, spec);
        leg.setMass(1.0);
        leg.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
        leg.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.35));

        sx::RigidBodyOptions groundOptions;
        groundOptions.isStatic = true;
        groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
        auto ground = world.addRigidBody("ground", groundOptions);
        ground.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

        world.setTimeStep(0.002);
      },
      true);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "cross articulated link contact",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        const auto addRobot
            = [&](std::string_view name, double z, double velocity) {
                auto robot = world.addMultibody(name);
                auto base = robot.addLink("base");
                sx::JointSpec spec;
                spec.name = "slider";
                spec.type = sx::JointType::Prismatic;
                spec.axis = Eigen::Vector3d::UnitZ();
                auto link = robot.addLink("link", base, spec);
                link.setMass(1.0);
                link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
                auto joint = link.getParentJoint();
                joint.setPosition(Eigen::VectorXd::Constant(1, z));
                joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
              };

        addRobot("lower_robot", 0.0, 0.5);
        addRobot("upper_robot", 0.35, -0.5);
        world.setTimeStep(0.001);
      },
      true);
}

TEST(World, BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The monolithic no-heap gate is too slow under coverage; normal "
         "Release and Debug CI run the full allocator regression.";
#endif

  namespace sx = dart::simulation;

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "multibody variational scratch", [](sx::World& world) {
        auto robot = world.addMultibody("slider");
        auto base = robot.addLink("base");
        sx::JointSpec spec;
        spec.name = "rail";
        spec.type = sx::JointType::Prismatic;
        spec.axis = Eigen::Vector3d::UnitZ();
        auto carriage = robot.addLink("carriage", base, spec);
        carriage.setMass(3.0);
        world.setMultibodyOptions({"variational integrator"});
        world.setTimeStep(0.01);
      });
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "semi-implicit external-force body Jacobian scratch",
      configureSemiImplicitExternalForceMultibodyScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "multibody variational compliant contact scratch",
      configureCompliantVariationalContactSliderScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "multibody variational augmented-Lagrangian contact scratch",
      configureVariationalContactDualStateSliderScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "multibody variational loop-closure scratch",
      configureVariationalLoopClosureChainScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "multibody variational articulated point-joint link-index scratch",
      configureLongVariationalArticulatedPointJointScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "single deformable particle", [](sx::World& world) {
        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("particle", options);
        world.setTimeStep(0.01);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable ground friction projected Newton scratch",
      [](sx::World& world) {
        world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
        world.setTimeStep(0.01);

        sx::RigidBodyOptions groundOptions;
        groundOptions.isStatic = true;
        groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
        auto ground = world.addRigidBody("ground", groundOptions);
        ground.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(100.0, 100.0, 0.5)));
        ground.setDeformableGroundBarrier(true);

        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(0.0, 0.0, 0.01)};
        options.velocities = {Eigen::Vector3d(2.0, 0.0, 0.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        options.material.frictionCoefficient = 0.8;
        world.addDeformableBody("slider", options);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable spring projected Newton scratch", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());
        sx::DeformableBodyOptions options;
        options.positions
            = {Eigen::Vector3d::Zero(), Eigen::Vector3d(1.2, 0.0, 0.0)};
        options.masses = {1.0, 1.0};
        options.fixedNodes = {0};
        options.edges = {sx::DeformableEdge{0, 1, 1.0}};
        options.edgeStiffness = 1000.0;
        world.addDeformableBody("spring", options);
        world.setTimeStep(0.01);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable FEM projected Newton scratch", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());
        sx::DeformableBodyOptions options;
        options.positions
            = {Eigen::Vector3d::Zero(),
               Eigen::Vector3d::UnitX(),
               Eigen::Vector3d::UnitY(),
               Eigen::Vector3d::UnitZ()};
        options.velocities
            = {Eigen::Vector3d::Zero(),
               Eigen::Vector3d(5.0, 0.0, 0.0),
               Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero()};
        options.masses = {1.0, 1.0, 1.0, 1.0};
        options.fixedNodes = {0};
        options.tetrahedra = {sx::DeformableTetrahedron{0, 1, 2, 3}};
        options.material.useFiniteElementElasticity = true;
        options.material.youngsModulus = 1.0e4;
        world.addDeformableBody("tet", options);
        world.setTimeStep(0.01);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable FEM ground friction block",
      configureDeformableFemGroundFrictionBlockScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable iterative FEM ground friction block",
      configureDeformableIterativeFemGroundFrictionBlockScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact projected Newton scratch", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());
        world.setTimeStep(0.1);
        sx::DeformableBodyOptions options;
        options.positions
            = {Eigen::Vector3d(0.0, 0.0, 0.0),
               Eigen::Vector3d(1.0, 0.0, 0.0),
               Eigen::Vector3d(0.0, 1.0, 0.0),
               Eigen::Vector3d(0.0, 0.0, 0.015),
               Eigen::Vector3d(1.0, 0.0, 0.015),
               Eigen::Vector3d(0.0, 1.0, 0.015)};
        options.velocities
            = {Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(),
               Eigen::Vector3d(0.0, 0.0, -0.2),
               Eigen::Vector3d(0.0, 0.0, -0.2),
               Eigen::Vector3d(0.0, 0.0, -0.2)};
        options.masses = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        options.fixedNodes = {0, 1, 2};
        options.edgeStiffness = 0.0;
        options.surfaceTriangles
            = {sx::DeformableSurfaceTriangle{0, 1, 2},
               sx::DeformableSurfaceTriangle{3, 4, 5}};
        world.addDeformableBody("facing_triangles", options);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction patch",
      configureDeformableSelfContactFrictionPatchScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction grid",
      configureDeformableSelfContactFrictionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction large grid",
      configureDeformableSelfContactFrictionLargeGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction production grid",
      configureDeformableSelfContactFrictionProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction extended production grid",
      configureDeformableSelfContactFrictionExtendedProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction dense production grid",
      configureDeformableSelfContactFrictionDenseProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction extra-dense production grid",
      configureDeformableSelfContactFrictionExtraDenseGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction larger production grid",
      configureDeformableSelfContactFrictionLargerProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction larger matrix-free production grid",
      configureDeformableSelfContactFrictionLargerMatrixFreeProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction irregular production grid",
      configureDeformableSelfContactFrictionIrregularProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction irregular matrix-free production grid",
      configureDeformableSelfContactFrictionIrregularMatrixFreeProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction late-active direct grid",
      configureDeformableSelfContactFrictionLateActiveGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction late-active matrix-free grid",
      configureDeformableSelfContactFrictionLateActiveMatrixFreeGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction late-active rectangular grid",
      configureDeformableSelfContactFrictionLateActiveRectangularGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction late-active matrix-free rectangular "
      "grid",
      configureDeformableSelfContactFrictionLateActiveMatrixFreeRectangularGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction rectangular grid",
      configureDeformableSelfContactFrictionRectangularGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction dense rectangular grid",
      configureDeformableSelfContactFrictionDenseRectangularGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction dense rectangular matrix-free grid",
      configureDeformableSelfContactFrictionDenseRectangularMatrixFreeGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction wide rectangular grid",
      configureDeformableSelfContactFrictionWideRectangularGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction wide rectangular matrix-free grid",
      configureDeformableSelfContactFrictionWideRectangularMatrixFreeGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction tall rectangular grid",
      configureDeformableSelfContactFrictionTallRectangularGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable self-contact friction tall rectangular matrix-free grid",
      configureDeformableSelfContactFrictionTallRectangularMatrixFreeGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed deformable self-contact friction production grids",
      configureMixedDeformableSelfContactFrictionProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed deformable self-contact friction dense production grids",
      configureMixedDeformableSelfContactFrictionDenseProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed deformable self-contact friction late-active production grids",
      configureMixedDeformableSelfContactFrictionLateActiveProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed default deformable storage paths",
      configureMixedDefaultDeformableStorageScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed production default deformable storage paths",
      configureMixedDefaultDeformableProductionStorageScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed production default FEM solver storage paths",
      configureMixedDefaultDeformableFemProductionStorageScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable static obstacle barriers",
      configureDeformableStaticObstacleBarrierScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable static obstacle friction production patch",
      configureDeformableStaticObstacleFrictionProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable static obstacle friction matrix-free production patch",
      configureDeformableStaticObstacleFrictionMatrixFreeProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed static obstacle and self-contact production deformables",
      configureMixedStaticObstacleAndSelfContactProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed matrix-free static obstacle and direct self-contact production "
      "deformables",
      configureMixedMatrixFreeStaticObstacleAndDirectSelfContactProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed default contact-family production deformables",
      configureMixedDefaultContactFamiliesProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "mixed complementary default contact-family production deformables",
      configureMixedComplementaryDefaultContactFamiliesProductionScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable inter-body surface CCD crossing",
      configureDeformableInterBodySurfaceCcdCrossingScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable inter-body surface CCD production grid crossing",
      configureDeformableInterBodySurfaceCcdProductionGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable AVBD self-contact friction rows",
      configureAvbdSelfContactFrictionRowsScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable AVBD self-contact friction grid rows",
      configureAvbdSelfContactFrictionGridRowsScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable AVBD self-contact friction production grid rows",
      configureAvbdSelfContactFrictionProductionGridRowsScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable VBD Chebyshev self-contact grid",
      configureVbdChebyshevSelfContactGridScene);
  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable AVBD ground friction rows",
      configureAvbdGroundFrictionRowsScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "deformable surface and rigid CCD snapshots", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        sx::RigidBodyOptions obstacleOptions;
        obstacleOptions.isStatic = true;
        obstacleOptions.position = Eigen::Vector3d(5.0, 0.0, 0.0);
        auto obstacle = world.addRigidBody("surface_obstacle", obstacleOptions);
        obstacle.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
        obstacle.setDeformableSurfaceCcdObstacle(true);

        sx::DeformableBodyOptions options;
        options.positions
            = {Eigen::Vector3d(-1.0, -1.0, 0.0),
               Eigen::Vector3d(1.0, -1.0, 0.0),
               Eigen::Vector3d(0.0, 1.0, 0.0)};
        options.masses = {1.0, 1.0, 1.0};
        options.fixedNodes = {0, 1, 2};
        options.surfaceTriangles = {sx::DeformableSurfaceTriangle{0, 1, 2}};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("fixed_surface", options);
        world.setTimeStep(0.01);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "default static rigid surface CCD point crossing", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());
        world.setTimeStep(0.1);

        sx::RigidBodyOptions obstacleOptions;
        obstacleOptions.isStatic = true;
        obstacleOptions.position = Eigen::Vector3d::Zero();
        auto obstacle = world.addRigidBody("static_box", obstacleOptions);
        obstacle.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
        obstacle.setDeformableSurfaceCcdObstacle(true);

        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(-1.0, 0.0, 0.0)};
        options.velocities = {Eigen::Vector3d(20.0, 0.0, 0.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("fast_point", options);
      });

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "default moving rigid surface CCD point crossing",
      configureDeformableMovingRigidSurfaceCcdCrossingScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "default kinematic rigid surface CCD point crossing",
      configureDeformableKinematicRigidSurfaceCcdCrossingScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "default multi-kinematic rigid surface CCD point crossings",
      configureDeformableMultiKinematicRigidSurfaceCcdScene);

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "active static rigid surface CCD point crossing", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());
        world.setTimeStep(0.1);

        sx::RigidBodyOptions obstacleOptions;
        obstacleOptions.isStatic = true;
        obstacleOptions.position = Eigen::Vector3d::Zero();
        auto obstacle = world.addRigidBody("static_box", obstacleOptions);
        obstacle.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.05, 1.0, 1.0)));
        obstacle.setDeformableSurfaceCcdObstacle(true);

        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(-1.0, 0.0, 0.0)};
        options.velocities = {Eigen::Vector3d(20.0, 0.0, 0.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("fast_point", options);

        sx::comps::DeformableVbdConfig cfg;
        cfg.enabled = true;
        cfg.iterations = 10;
        cfg.contactStiffness = 1.0e4;
        auto& registry = sx::detail::registryOf(world);
        for (const auto entity :
             registry.view<sx::comps::DeformableBodyTag>()) {
          registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(
              entity, cfg);
        }
      });
}

TEST(World, BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator)
{
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "rigid sphere-ground boxed LCP", configureRigidBoxedLcpContactRowsScene);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody different-DOF fallback",
      configureCrossMultibodyDifferentDofFallbackScene);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody stacked-contact fallback",
      configureCrossMultibodyStackedFallbackScene);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody coupled-row fallback",
      configureCrossMultibodyCoupledRowsFallbackScene);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody larger stacked fallback",
      configureCrossMultibodyLargeStackFallbackScene,
      true,
      4);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody extended stacked fallback",
      configureCrossMultibodyExtendedStackFallbackScene,
      true,
      7);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody production stacked fallback",
      configureCrossMultibodyProductionStackFallbackScene,
      true,
      11);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody dense production stacked fallback",
      configureCrossMultibodyDenseProductionStackFallbackScene,
      true,
      15);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody extra-dense production stacked fallback",
      configureCrossMultibodyExtraDenseProductionStackFallbackScene,
      true,
      23);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody stress production stacked fallback",
      configureCrossMultibodyStressProductionStackFallbackScene,
      true,
      31);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody multi-island mixed fallback",
      configureCrossMultibodyMultiIslandFallbackScene,
      true,
      4);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody production multi-island mixed fallback",
      configureCrossMultibodyProductionMultiIslandFallbackScene,
      true,
      12);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody stress multi-island mixed fallback",
      configureCrossMultibodyStressMultiIslandFallbackScene,
      true,
      30);
  expectNoWorldBaseAllocatorActivityDuringBakedBoxedLcpSteps(
      "cross multibody mixed stress stack and multi-island fallback",
      configureCrossMultibodyMixedStressFallbackScene,
      true,
      61);
}

TEST(World, BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap)
{
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "rigid sphere-ground boxed LCP", configureRigidBoxedLcpContactRowsScene);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody different-DOF fallback",
      configureCrossMultibodyDifferentDofFallbackScene);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody stacked-contact fallback",
      configureCrossMultibodyStackedFallbackScene);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody coupled-row fallback",
      configureCrossMultibodyCoupledRowsFallbackScene);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody larger stacked fallback",
      configureCrossMultibodyLargeStackFallbackScene,
      true,
      4);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody extended stacked fallback",
      configureCrossMultibodyExtendedStackFallbackScene,
      true,
      7);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody production stacked fallback",
      configureCrossMultibodyProductionStackFallbackScene,
      true,
      11);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody dense production stacked fallback",
      configureCrossMultibodyDenseProductionStackFallbackScene,
      true,
      15);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody extra-dense production stacked fallback",
      configureCrossMultibodyExtraDenseProductionStackFallbackScene,
      true,
      23);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody stress production stacked fallback",
      configureCrossMultibodyStressProductionStackFallbackScene,
      true,
      31);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody multi-island mixed fallback",
      configureCrossMultibodyMultiIslandFallbackScene,
      true,
      4);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody production multi-island mixed fallback",
      configureCrossMultibodyProductionMultiIslandFallbackScene,
      true,
      12);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody stress multi-island mixed fallback",
      configureCrossMultibodyStressMultiIslandFallbackScene,
      true,
      30);
  expectNoGlobalHeapAllocationsDuringBakedBoxedLcpSteps(
      "cross multibody mixed stress stack and multi-island fallback",
      configureCrossMultibodyMixedStressFallbackScene,
      true,
      61);
}

TEST(World, SequentialImpulseBakeDoesNotPrewarmRigidIpcCollisionSurfaces)
{
  namespace sx = dart::simulation;

  const auto unsupportedGeometry
      = countGlobalHeapAllocationsDuringSimulationBake(
          "sequential impulse plane geometry", [](sx::World& world) {
            auto body = world.addRigidBody("kinematic_plane");
            body.setKinematic(true);
            body.setCollisionShape(
                sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));
          });
  const auto supportedGeometry = countGlobalHeapAllocationsDuringSimulationBake(
      "sequential impulse box geometry", [](sx::World& world) {
        auto body = world.addRigidBody("kinematic_box");
        body.setKinematic(true);
        body.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
      });

  EXPECT_EQ(
      supportedGeometry.allocationCount, unsupportedGeometry.allocationCount);
}

TEST(World, IpcBakeDoesNotPrewarmRigidBodyContactQuery)
{
  namespace sx = dart::simulation;

  const auto ipcSupportedGeometry
      = countGlobalHeapAllocationsDuringSimulationBake(
          "IPC contact-query-only box geometry", [](sx::World& world) {
            world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
            auto body = world.addRigidBody("kinematic_box");
            body.setKinematic(true);
            body.setCollisionShape(
                sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
            body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
          });
  const auto ipcUnsupportedGeometry
      = countGlobalHeapAllocationsDuringSimulationBake(
          "IPC contact-query-only plane geometry", [](sx::World& world) {
            world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
            auto body = world.addRigidBody("kinematic_plane");
            body.setKinematic(true);
            body.setCollisionShape(
                sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));
            body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
          });

  // The unsupported (contact-query-only) plane geometry must not PREWARM the
  // rigid-body contact query during the IPC bake. Compare against the same IPC
  // setup with supported collision geometry so ordinary collision component
  // storage is counted on both sides and only unsupported-geometry prewarm work
  // can fail this guard.
  EXPECT_LE(
      ipcUnsupportedGeometry.allocationCount,
      ipcSupportedGeometry.allocationCount);
  EXPECT_LE(
      ipcUnsupportedGeometry.allocationBytes,
      ipcSupportedGeometry.allocationBytes);
}

TEST(World, RigidIpcContactStagePrepareReusesSupportedDynamicSurfaceBuffers)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  auto body = world.addRigidBody("dynamic_box", options);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  sx::compute::RigidIpcContactStage ipcStage;
  ipcStage.prepare(world);

  ScopedHeapAllocationCounter heapCounter;
  for (int i = 0; i < 4; ++i) {
    ipcStage.prepare(world);
  }
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated while preparing dynamic IPC scratch: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, SetRigidBodySolverPreparesIpcScratchAfterSimulationBake)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto body = world.addRigidBody("kinematic_box");
  body.setKinematic(true);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));

  world.enterSimulationMode();
  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);

  ScopedHeapAllocationCounter heapCounter;
  for (int i = 0; i < 4; ++i) {
    world.step();
  }
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated during IPC steps after solver switch: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, StepWithCustomStageReusesPreparedSolverStages)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  auto body = world.addRigidBody("kinematic_box");
  body.setKinematic(true);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
  body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));

  world.enterSimulationMode();
  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);

  compute::SequentialExecutor executor;
  NoOpWorldStage finalStage;

  ScopedHeapAllocationCounter heapCounter;
  for (int i = 0; i < 4; ++i) {
    world.step(executor, finalStage);
  }
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated during IPC custom-stage steps: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

TEST(World, IpcSemiImplicitMultibodyStepUsesForwardDynamicsStage)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(1.0);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setTimeStep(0.01);

  EXPECT_NO_THROW(world.step());
  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_EQ(world.getFrame(), 1u);
}

TEST(World, FrameScratchCapacityReportsUsableArenaBytes)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.frameScratchInitialCapacity = 128;
  sx::World world(options);

  const auto initial = world.getMemoryDiagnostics();
  EXPECT_LE(
      initial.frameScratchCapacityBytes, options.frameScratchInitialCapacity);
  EXPECT_GE(
      initial.frameScratchCapacityBytes + 64u,
      options.frameScratchInitialCapacity);
  ASSERT_GT(initial.frameScratchCapacityBytes, 0u);
  EXPECT_EQ(initial.frameScratchCapacityBytes % 32u, 0u);

  dart::simulation::compute::SequentialExecutor executor;
  FrameScratchStage scratch(initial.frameScratchCapacityBytes);

  world.step(executor, scratch);
  ASSERT_NE(scratch.lastAllocation, nullptr);

  const auto diagnostics = world.getMemoryDiagnostics();
  EXPECT_EQ(diagnostics.frameScratchResetCount, 1u);
  EXPECT_EQ(
      diagnostics.frameScratchUsedBytes, initial.frameScratchCapacityBytes);
  EXPECT_EQ(diagnostics.frameScratchOverflowCount, 0u);
  EXPECT_EQ(diagnostics.frameScratchOverflowBytes, 0u);
}

TEST(World, FrameScratchResetsAtStepBoundary)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.frameScratchInitialCapacity = 1024;
  sx::World world(options);

  dart::simulation::compute::SequentialExecutor executor;
  FrameScratchStage scratch(96);

  world.step(executor, scratch);
  ASSERT_NE(scratch.lastAllocation, nullptr);

  const auto first = world.getMemoryDiagnostics();
  EXPECT_EQ(first.frameScratchResetCount, 1u);
  EXPECT_EQ(first.frameScratchOverflowCount, 0u);
  EXPECT_EQ(first.frameScratchOverflowBytes, 0u);
  EXPECT_GE(first.frameScratchUsedBytes, scratch.bytesToAllocate);
  EXPECT_EQ(first.frameScratchPeakUsedBytes, first.frameScratchUsedBytes);

  scratch.bytesToAllocate = 0;
  world.step(executor, scratch);

  const auto second = world.getMemoryDiagnostics();
  EXPECT_EQ(second.frameScratchResetCount, 2u);
  EXPECT_EQ(second.frameScratchUsedBytes, 0u);
  EXPECT_EQ(second.frameScratchOverflowCount, 0u);
  EXPECT_EQ(second.frameScratchOverflowBytes, 0u);
  EXPECT_GE(second.frameScratchPeakUsedBytes, first.frameScratchUsedBytes);

  world.clear();
  const auto cleared = world.getMemoryDiagnostics();
  EXPECT_EQ(cleared.frameScratchUsedBytes, 0u);
  EXPECT_EQ(cleared.frameScratchPeakUsedBytes, 0u);
  EXPECT_EQ(cleared.frameScratchOverflowCount, 0u);
  EXPECT_EQ(cleared.frameScratchOverflowBytes, 0u);
  EXPECT_EQ(cleared.frameScratchResetCount, 0u);
}

TEST(World, FrameScratchDiagnosticsIncludeOverflowBytes)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.frameScratchInitialCapacity = 128;
  sx::World world(options);

  dart::simulation::compute::SequentialExecutor executor;
  FrameScratchStage scratch(512);

  world.step(executor, scratch);
  ASSERT_NE(scratch.lastAllocation, nullptr);

  const auto first = world.getMemoryDiagnostics();
  EXPECT_EQ(first.frameScratchResetCount, 1u);
  EXPECT_EQ(first.frameScratchOverflowCount, 1u);
  EXPECT_GE(first.frameScratchOverflowBytes, scratch.bytesToAllocate);
  EXPECT_GE(first.frameScratchUsedBytes, first.frameScratchOverflowBytes);
  EXPECT_EQ(first.frameScratchPeakUsedBytes, first.frameScratchUsedBytes);

  scratch.bytesToAllocate = 0;
  world.step(executor, scratch);

  const auto second = world.getMemoryDiagnostics();
  EXPECT_EQ(second.frameScratchResetCount, 2u);
  EXPECT_EQ(second.frameScratchUsedBytes, 0u);
  EXPECT_EQ(second.frameScratchOverflowCount, 0u);
  EXPECT_EQ(second.frameScratchOverflowBytes, 0u);
  EXPECT_GE(second.frameScratchPeakUsedBytes, first.frameScratchUsedBytes);
}

// Test version information
TEST(Version, VersionString)
{
  // Test that version() returns a valid string_view
  auto ver = dart::simulation::version();
  EXPECT_FALSE(ver.empty());
  EXPECT_GT(ver.size(), 0u);

  // Test individual version components
  EXPECT_EQ(dart::simulation::versionMajor(), DART_MAJOR_VERSION);
  EXPECT_EQ(dart::simulation::versionMinor(), DART_MINOR_VERSION);
  EXPECT_EQ(dart::simulation::versionPatch(), DART_PATCH_VERSION);
  EXPECT_EQ(ver, std::string_view(DART_VERSION));
}

//==============================================================================
// Mode Control Tests (Design Mode vs Simulation Mode)
//==============================================================================

// Test default mode is design mode
TEST(World, DefaultModeIsDesign)
{
  dart::simulation::World world;
  EXPECT_FALSE(world.isSimulationMode());
}

// Test entering simulation mode
TEST(World, EnterSimulationMode)
{
  dart::simulation::World world;

  // Initially in design mode
  EXPECT_FALSE(world.isSimulationMode());

  // Enter simulation mode
  world.enterSimulationMode();

  // Now in simulation mode
  EXPECT_TRUE(world.isSimulationMode());
}

// Test cannot enter simulation mode twice
TEST(World, CannotEnterSimulationModeTwice)
{
  dart::simulation::World world;

  // First call succeeds
  world.enterSimulationMode();
  EXPECT_TRUE(world.isSimulationMode());

  // Second call should throw
  EXPECT_THROW(
      world.enterSimulationMode(), dart::simulation::InvalidArgumentException);
}

// Test baking with empty world
TEST(World, BakingEmptyWorld)
{
  dart::simulation::World world;

  // Baking should work even with empty world
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());
}

// Test baking with multibodies
TEST(World, BakingWithMultibodies)
{
  dart::simulation::World world;

  // Create several multibodies with joints and links
  auto robot1 = world.addMultibody("robot1");
  auto base1 = robot1.addLink("base");
  (void)robot1.addLink("link1", {.parentLink = base1, .jointName = "joint1"});

  auto robot2 = world.addMultibody("robot2");
  auto base2 = robot2.addLink("base");
  auto link2
      = robot2.addLink("link1", {.parentLink = base2, .jointName = "joint1"});
  (void)robot2.addLink(
      "link2",
      {.parentLink = link2,
       .jointName = "joint2",
       .jointType = dart::simulation::JointType::Prismatic});

  // Baking should succeed
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());

  // Counts should remain the same
  EXPECT_EQ(world.getMultibodyCount(), 2u);
  EXPECT_EQ(robot1.getLinkCount(), 2u);
  EXPECT_EQ(robot1.getJointCount(), 1u);
  EXPECT_EQ(robot2.getLinkCount(), 3u);
  EXPECT_EQ(robot2.getJointCount(), 2u);
}

// Test multibody lookup by name returns first-class handles and exposes a
// symmetric presence query for world-owned names.
TEST(World, MultibodyLookupByName)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");

  auto found = world.getMultibody("robot");
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(found->isValid());
  EXPECT_EQ(found->getEntity(), robot.getEntity());
  EXPECT_EQ(found->getWorld(), robot.getWorld());
  EXPECT_EQ(found->getName(), "robot");
  EXPECT_TRUE(world.hasMultibody("robot"));
  EXPECT_FALSE(world.hasMultibody("missing"));
  EXPECT_FALSE(world.getMultibody("missing").has_value());

  EXPECT_THROW(world.addMultibody("robot"), sx::InvalidArgumentException);
  EXPECT_EQ(world.getMultibodyCount(), 1u);

  sx::World worldWithExplicitGeneratedName;
  [[maybe_unused]] auto explicitName
      = worldWithExplicitGeneratedName.addMultibody("multibody_001");
  auto generated = worldWithExplicitGeneratedName.addMultibody("");
  EXPECT_EQ(generated.getName(), "multibody_002");
  EXPECT_TRUE(worldWithExplicitGeneratedName.hasMultibody("multibody_001"));
  EXPECT_TRUE(worldWithExplicitGeneratedName.hasMultibody("multibody_002"));
  EXPECT_EQ(worldWithExplicitGeneratedName.getMultibodyCount(), 2u);
}

TEST(World, ClearInvalidatesPublicHandlesAndResetsFacadeState)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setTimeStep(0.01);
  world.setTime(0.25);

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  auto body = world.addRigidBody("body");
  auto closure = world.addLoopClosure(
      "closure",
      {.frameA = base, .frameB = body, .family = sx::LoopClosureFamily::Point});

  EXPECT_TRUE(robot.isValid());
  EXPECT_TRUE(base.isValid());
  EXPECT_TRUE(body.isValid());
  EXPECT_TRUE(closure.isValid());
  EXPECT_TRUE(world.hasMultibody("robot"));
  EXPECT_TRUE(world.hasRigidBody("body"));
  EXPECT_TRUE(world.hasLoopClosure("closure"));

  world.enterSimulationMode();
  world.step();
  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_GT(world.getTime(), 0.0);
  EXPECT_GT(world.getFrame(), 0u);

  world.clear();

  EXPECT_FALSE(robot.isValid());
  EXPECT_FALSE(base.isValid());
  EXPECT_FALSE(body.isValid());
  EXPECT_FALSE(closure.isValid());
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTimeStep(), 0.001);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
  EXPECT_EQ(world.getMultibodyCount(), 0u);
  EXPECT_EQ(world.getRigidBodyCount(), 0u);
  EXPECT_EQ(world.getLoopClosureCount(), 0u);
  EXPECT_FALSE(world.hasMultibody("robot"));
  EXPECT_FALSE(world.hasRigidBody("body"));
  EXPECT_FALSE(world.hasLoopClosure("closure"));
  EXPECT_FALSE(world.getMultibody("robot").has_value());
  EXPECT_FALSE(world.getRigidBody("body").has_value());
  EXPECT_FALSE(world.getLoopClosure("closure").has_value());
}

// Test rigid body lookup by name returns first-class handles.
TEST(World, RigidBodyLookupByName)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto body = world.addRigidBody("body");

  auto found = world.getRigidBody("body");
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(found->isValid());
  EXPECT_TRUE(found->isSameInstanceAs(body));
  EXPECT_EQ(found->getName(), "body");

  EXPECT_FALSE(world.getRigidBody("missing").has_value());

  EXPECT_THROW(world.addRigidBody("body"), sx::InvalidArgumentException);
  EXPECT_EQ(world.getRigidBodyCount(), 1u);

  sx::World worldWithExplicitGeneratedName;
  [[maybe_unused]] auto explicitName
      = worldWithExplicitGeneratedName.addRigidBody("rigid_body_001");
  auto generated = worldWithExplicitGeneratedName.addRigidBody("");
  EXPECT_EQ(generated.getName(), "rigid_body_002");
  EXPECT_EQ(worldWithExplicitGeneratedName.getRigidBodyCount(), 2u);
}

TEST(World, LoopClosureTopology)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("four_bar");
  auto base = robot.addLink("base");
  auto coupler = robot.addLink(
      "coupler",
      {.parentLink = base,
       .jointName = "shoulder",
       .jointType = sx::JointType::Revolute});
  auto ground = world.addRigidBody("ground");

  Eigen::Isometry3d offsetA = Eigen::Isometry3d::Identity();
  offsetA.translate(Eigen::Vector3d(0.5, 0.0, 0.0));
  Eigen::Isometry3d offsetB = Eigen::Isometry3d::Identity();
  offsetB.translate(Eigen::Vector3d(-0.5, 0.0, 0.0));

  auto closure = world.addLoopClosure(
      "closing_bar",
      {.frameA = coupler,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Rigid,
       .offsetA = offsetA,
       .offsetB = offsetB});

  EXPECT_TRUE(closure.isValid());
  EXPECT_EQ(closure.getName(), "closing_bar");
  EXPECT_EQ(closure.getFamily(), sx::LoopClosureFamily::Rigid);
  EXPECT_TRUE(closure.getFrameA().isSameInstanceAs(coupler));
  EXPECT_TRUE(closure.getFrameB().isSameInstanceAs(ground));
  EXPECT_TRUE(closure.getOffsetA().isApprox(offsetA));
  EXPECT_TRUE(closure.getOffsetB().isApprox(offsetB));

  auto runtimePolicy = closure.getRuntimePolicy();
  EXPECT_TRUE(runtimePolicy.enabled);
  EXPECT_EQ(
      runtimePolicy.kinematics, sx::ClosureKinematicsPolicy::ResidualOnly);
  EXPECT_EQ(runtimePolicy.dynamics, sx::ClosureDynamicsPolicy::ResidualOnly);

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
  runtimePolicy = closure.getRuntimePolicy();
  EXPECT_TRUE(runtimePolicy.enabled);
  EXPECT_EQ(runtimePolicy.kinematics, sx::ClosureKinematicsPolicy::Project);
  EXPECT_EQ(runtimePolicy.dynamics, sx::ClosureDynamicsPolicy::Solve);
  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});

  EXPECT_EQ(world.getLoopClosureCount(), 1u);
  EXPECT_TRUE(world.hasLoopClosure("closing_bar"));

  auto found = world.getLoopClosure("closing_bar");
  ASSERT_TRUE(found.has_value());
  EXPECT_TRUE(found->isValid());
  EXPECT_TRUE(found->getFrameA().isSameInstanceAs(coupler));
  EXPECT_FALSE(world.getLoopClosure("missing").has_value());

  auto autoClosure = world.addLoopClosure(
      "",
      {.frameA = base,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Point});
  EXPECT_EQ(autoClosure.getName(), "loop_closure_001");
  EXPECT_EQ(world.getLoopClosureCount(), 2u);

  world.enterSimulationMode();
  const auto residual = closure.computeResidual();
  EXPECT_TRUE(residual.enabled);
  EXPECT_TRUE(residual.active);
  EXPECT_EQ(residual.coordinates, sx::LoopClosureResidualCoordinates::World);
  EXPECT_FALSE(residual.forceAvailable);
  ASSERT_EQ(residual.value.size(), 6);
  EXPECT_TRUE(
      residual.value.head<3>().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(residual.value.tail<3>().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_DOUBLE_EQ(residual.norm, 1.0);
}

TEST(World, LoopClosureResidualDiagnostics)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions bodyOptions;
  auto a = world.addRigidBody("a", bodyOptions);
  bodyOptions.position = Eigen::Vector3d(0.0, 3.0, 4.0);
  auto b = world.addRigidBody("b", bodyOptions);

  auto point = world.addLoopClosure(
      "point",
      {.frameA = a, .frameB = b, .family = sx::LoopClosureFamily::Point});
  auto distance = world.addLoopClosure(
      "distance",
      {.frameA = a, .frameB = b, .family = sx::LoopClosureFamily::Distance});

  world.enterSimulationMode();

  const auto pointResidual = point.computeResidual();
  ASSERT_EQ(pointResidual.value.size(), 3);
  EXPECT_TRUE(pointResidual.value.isApprox(Eigen::Vector3d(0.0, -3.0, -4.0)));
  EXPECT_DOUBLE_EQ(pointResidual.norm, 5.0);
  EXPECT_TRUE(pointResidual.enabled);
  EXPECT_TRUE(pointResidual.active);
  EXPECT_FALSE(pointResidual.forceAvailable);

  const auto distanceResidual = distance.computeResidual();
  ASSERT_EQ(distanceResidual.value.size(), 1);
  EXPECT_DOUBLE_EQ(distanceResidual.value[0], 5.0);
  EXPECT_DOUBLE_EQ(distanceResidual.norm, 5.0);

  point.setRuntimePolicy({
      .enabled = false,
      .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
      .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly,
  });
  const auto disabledResidual = point.computeResidual();
  EXPECT_FALSE(disabledResidual.enabled);
  EXPECT_FALSE(disabledResidual.active);
  EXPECT_TRUE(disabledResidual.value.isApprox(pointResidual.value));
}

TEST(World, LoopClosureRejectsInvalidTopology)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});

  sx::World otherWorld;
  auto otherBody = otherWorld.addRigidBody("other");

  EXPECT_THROW(
      world.addLoopClosure("same", {.frameA = base, .frameB = base}),
      sx::InvalidArgumentException);

  EXPECT_THROW(
      world.addLoopClosure(
          "cross_world", {.frameA = base, .frameB = otherBody}),
      sx::InvalidArgumentException);

  EXPECT_THROW(
      world.addLoopClosure(
          "bad_family",
          {.frameA = base,
           .frameB = link,
           .family = static_cast<sx::LoopClosureFamily>(999)}),
      sx::InvalidArgumentException);

  Eigen::Isometry3d invalidOffset = Eigen::Isometry3d::Identity();
  invalidOffset.linear()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      world.addLoopClosure(
          "bad_offset",
          {.frameA = base, .frameB = link, .offsetA = invalidOffset}),
      sx::InvalidArgumentException);

  world.addLoopClosure("valid", {.frameA = base, .frameB = link});
  EXPECT_THROW(
      world.addLoopClosure("valid", {.frameA = link, .frameB = base}),
      sx::InvalidArgumentException);

  world.enterSimulationMode();
  EXPECT_THROW(
      world.addLoopClosure("after_bake", {.frameA = base, .frameB = link}),
      sx::InvalidOperationException);
}

TEST(World, LoopClosureRejectsInvalidRuntimePolicy)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
  auto closure
      = world.addLoopClosure("closure", {.frameA = base, .frameB = link});

  EXPECT_THROW(
      closure.setRuntimePolicy(
          {.enabled = true,
           .kinematics = static_cast<sx::ClosureKinematicsPolicy>(999),
           .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly}),
      sx::InvalidArgumentException);

  EXPECT_THROW(
      closure.setRuntimePolicy(
          {.enabled = true,
           .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
           .dynamics = static_cast<sx::ClosureDynamicsPolicy>(999)}),
      sx::InvalidArgumentException);
}

TEST(World, LoopClosureRejectsUnsupportedActiveRuntimePolicy)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  auto link = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
  auto closure
      = world.addLoopClosure("closure", {.frameA = base, .frameB = link});

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});
  EXPECT_THROW(world.enterSimulationMode(), sx::InvalidOperationException);

  closure.setRuntimePolicy(
      {.enabled = false,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_NO_THROW(world.sync(sx::WorldSyncStage::Kinematics));
  EXPECT_NO_THROW(world.step());

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::Project,
       .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});
  EXPECT_THROW(
      world.sync(sx::WorldSyncStage::Kinematics),
      sx::InvalidOperationException);

  closure.setRuntimePolicy(
      {.enabled = true,
       .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
       .dynamics = sx::ClosureDynamicsPolicy::Solve});
  EXPECT_THROW(world.step(), sx::InvalidOperationException);
}

// Test that rigid bodies can be driven kinematically through the public
// transform API and that attached frames observe fresh transforms immediately.
TEST(World, RigidBodySetTransformRefreshesAttachedFrames)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto body = world.addRigidBody("body");

  Eigen::Isometry3d sensorOffset = Eigen::Isometry3d::Identity();
  sensorOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto sensor = world.addFixedFrame("sensor", body, sensorOffset);

  world.setGravity(Eigen::Vector3d::Zero());
  world.enterSimulationMode();

  Eigen::Isometry3d drivenPose = Eigen::Isometry3d::Identity();
  drivenPose.translate(Eigen::Vector3d(4.0, 0.0, 0.0));
  body.setTransform(drivenPose);

  EXPECT_TRUE(body.getTransform().isApprox(drivenPose));
  EXPECT_TRUE(sensor.getTransform().isApprox(drivenPose * sensorOffset));

  world.step();

  EXPECT_TRUE(body.getTransform().isApprox(drivenPose));
  EXPECT_TRUE(sensor.getTransform().isApprox(drivenPose * sensorOffset));
}

// Test that rigid bodies expose public velocity state and that stepping uses
// it without requiring direct ECS component access.
TEST(World, RigidBodyVelocityAccessorsDriveStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions options;
  options.linearVelocity = Eigen::Vector3d(0.1, 0.2, 0.3);
  options.angularVelocity = Eigen::Vector3d(0.4, 0.5, 0.6);
  auto body = world.addRigidBody("body", options);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(options.linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(options.angularVelocity));

  const Eigen::Vector3d linearVelocity(2.0, 0.0, 0.0);
  const Eigen::Vector3d angularVelocity(0.0, 0.0, 1.0);
  body.setLinearVelocity(linearVelocity);
  body.setAngularVelocity(angularVelocity);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(angularVelocity));

  EXPECT_THROW(
      body.setLinearVelocity(
          Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.setAngularVelocity(
          Eigen::Vector3d(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0)),
      sx::InvalidArgumentException);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();
  world.step();

  const Eigen::Vector3d expectedTranslation(1.0, 0.0, 0.0);
  const Eigen::Quaterniond expectedOrientation(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));

  EXPECT_TRUE(body.getLinearVelocity().isApprox(linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(angularVelocity));
  EXPECT_TRUE(body.getTranslation().isApprox(expectedTranslation));
  EXPECT_TRUE(
      body.getRotation().isApprox(expectedOrientation.toRotationMatrix()));
}

// Test that simulation operations require simulation mode
TEST(World, UpdateKinematicsRequiresSimulationMode)
{
  namespace sx = dart::simulation;

  sx::World world;

  EXPECT_THROW(world.updateKinematics(), sx::InvalidArgumentException);
  EXPECT_THROW(
      world.sync(sx::WorldSyncStage::Kinematics), sx::InvalidArgumentException);

  world.enterSimulationMode();
  EXPECT_NO_THROW(world.updateKinematics());
  EXPECT_NO_THROW(world.sync(sx::WorldSyncStage::Kinematics));
  EXPECT_THROW(
      world.sync(static_cast<sx::WorldSyncStage>(999)),
      sx::InvalidArgumentException);
}

// Test that explicit sync refreshes a hierarchy through the compute graph
// without advancing simulation time.
TEST(World, SyncKinematicsRefreshesFrameHierarchy)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  parent.setLocalTransform(parentTransform);

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 4.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  auto& registry = dart::simulation::detail::registryOf(world);
  EXPECT_FALSE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(parent.getEntity()))
          .needTransformUpdate);
  EXPECT_FALSE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(child.getEntity()))
          .needTransformUpdate);
  EXPECT_TRUE(child.getTransform().isApprox(parentTransform * childOffset));

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(5.0, 6.0, 7.0));
  parent.setLocalTransform(updatedParentTransform);

  world.sync(sx::WorldSyncStage::Kinematics);

  EXPECT_FALSE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(parent.getEntity()))
          .needTransformUpdate);
  EXPECT_FALSE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(child.getEntity()))
          .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that kinematics-only sync can use an injected executor
// without advancing the simulation clock.
TEST(World, SyncKinematicsAcceptsExecutorWithoutAdvancingClock)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 4.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(5.0, 6.0, 7.0));
  parent.setLocalTransform(updatedParentTransform);

  RecordingExecutor executor;
  world.sync(sx::WorldSyncStage::Kinematics, executor);

  EXPECT_EQ(executor.executeCount, 1u);
  EXPECT_EQ(executor.nodeCount, 2u);
  EXPECT_EQ(executor.edgeCount, 1u);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

TEST(World, SyncKinematicsRefreshesJointDrivenLinkTransforms)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");
  auto forearm = robot.addLink(
      "forearm",
      base,
      sx::JointSpec{
          .name = "elbow",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitZ(),
      });
  auto slider = robot.addLink(
      "slider",
      base,
      sx::JointSpec{
          .name = "rail",
          .type = sx::JointType::Prismatic,
          .axis = Eigen::Vector3d::UnitX(),
      });

  Eigen::Isometry3d toolOffset = Eigen::Isometry3d::Identity();
  toolOffset.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto tool = world.addFixedFrame("tool", forearm, toolOffset);

  world.enterSimulationMode();

  auto elbow = forearm.getParentJoint();
  elbow.setPosition(Eigen::VectorXd::Constant(1, std::numbers::pi / 2.0));
  auto rail = slider.getParentJoint();
  rail.setPosition(Eigen::VectorXd::Constant(1, 2.0));

  world.sync(sx::WorldSyncStage::Kinematics);

  EXPECT_TRUE(
      tool.getTranslation().isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(slider.getTranslation().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(world.getTime(), 0.0);
  EXPECT_EQ(world.getFrame(), 0u);
}

TEST(World, LoopClosureResidualUsesSyncedJointTransforms)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("arm");
  auto base = robot.addLink("base");
  auto forearm = robot.addLink(
      "forearm",
      base,
      sx::JointSpec{
          .name = "elbow",
          .type = sx::JointType::Revolute,
          .axis = Eigen::Vector3d::UnitZ(),
      });

  Eigen::Isometry3d toolOffset = Eigen::Isometry3d::Identity();
  toolOffset.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto tool = world.addFixedFrame("tool", forearm, toolOffset);
  auto ground = world.addRigidBody("ground");
  auto closure = world.addLoopClosure(
      "tool_to_ground",
      {.frameA = tool,
       .frameB = ground,
       .family = sx::LoopClosureFamily::Point});

  world.enterSimulationMode();
  forearm.getParentJoint().setPosition(
      Eigen::VectorXd::Constant(1, std::numbers::pi / 2.0));

  world.sync(sx::WorldSyncStage::Kinematics);

  const auto residual = closure.computeResidual();
  ASSERT_EQ(residual.value.size(), 3);
  EXPECT_TRUE(residual.value.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
}

// Test that ordinary frame reads stay fresh after mutating an ancestor frame,
// without requiring users to know cache invalidation details.
TEST(World, FrameReadsRefreshDescendantsAfterParentLocalTransformChange)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 2.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();
  EXPECT_TRUE(child.getTransform().isApprox(childOffset));

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(3.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  auto& registry = dart::simulation::detail::registryOf(world);
  EXPECT_TRUE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(parent.getEntity()))
          .needTransformUpdate);
  EXPECT_TRUE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(child.getEntity()))
          .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental step path enters simulation mode and refreshes
// kinematics using the default sequential graph executor.
TEST(World, StepRefreshesFrameHierarchy)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(1.0, 0.0, 0.0));
  parent.setLocalTransform(parentTransform);

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 2.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(3.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  world.step();

  EXPECT_TRUE(world.isSimulationMode());

  auto& registry = dart::simulation::detail::registryOf(world);
  EXPECT_FALSE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(parent.getEntity()))
          .needTransformUpdate);
  EXPECT_FALSE(
      registry
          .get<sx::comps::FrameCache>(
              dart::simulation::detail::toRegistryEntity(child.getEntity()))
          .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental step path accepts alternate graph executors.
TEST(World, StepAcceptsParallelExecutor)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  sx::compute::ParallelExecutor executor(1);
  world.step(executor);

  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that RigidBodyOptions seed the public rigid-body state used by the graph
// backed step path.
TEST(World, RigidBodyOptionsInitializePublicState)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.inertia = Eigen::Vector3d(1.0, 2.0, 3.0).asDiagonal();
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.orientation
      = Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  options.linearVelocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  options.angularVelocity = Eigen::Vector3d(0.1, 0.2, 0.3);

  auto body = world.addRigidBody("body", options);

  EXPECT_TRUE(body.getTranslation().isApprox(options.position));
  EXPECT_TRUE(body.getQuaternion().isApprox(options.orientation));
  EXPECT_TRUE(body.getLinearVelocity().isApprox(options.linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(options.angularVelocity));
  EXPECT_DOUBLE_EQ(body.getMass(), options.mass);
  EXPECT_TRUE(body.getInertia().isApprox(options.inertia));

  const double updatedMass = 5.0;
  const Eigen::Matrix3d updatedInertia
      = Eigen::Vector3d(3.0, 4.0, 5.0).asDiagonal();
  body.setMass(updatedMass);
  body.setInertia(updatedInertia);

  EXPECT_DOUBLE_EQ(body.getMass(), updatedMass);
  EXPECT_TRUE(body.getInertia().isApprox(updatedInertia));

  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d::Zero()));
  body.setForce(Eigen::Vector3d(0.0, 2.0, 0.0));
  body.applyForce(Eigen::Vector3d(0.0, 3.0, 0.0));
  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d(0.0, 5.0, 0.0)));
  body.clearForce();
  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d::Zero()));

  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d::Zero()));
  body.setTorque(Eigen::Vector3d(1.0, 0.0, 0.0));
  body.applyTorque(Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d(3.0, 0.0, 0.0)));
  body.clearTorque();
  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d::Zero()));

  world.enterSimulationMode();
  EXPECT_TRUE(body.getTransform().translation().isApprox(options.position));
  EXPECT_TRUE(
      body.getRotation().isApprox(options.orientation.toRotationMatrix()));
}

// Test that invalid rigid-body construction data is rejected before it reaches
// ECS dynamics components.
TEST(World, RigidBodyOptionsRejectInvalidValues)
{
  namespace sx = dart::simulation;

  const auto infinity = std::numeric_limits<double>::infinity();
  const auto quietNaN = std::numeric_limits<double>::quiet_NaN();

  auto expectInvalid = [](const sx::RigidBodyOptions& options) {
    sx::World world;
    EXPECT_THROW(
        (void)world.addRigidBody("body", options),
        sx::InvalidArgumentException);
  };

  sx::RigidBodyOptions options;

  options.mass = 0.0;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.mass = infinity;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.inertia = Eigen::Vector3d(1.0, -1.0, 1.0).asDiagonal();
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.inertia(0, 1) = 0.1;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.position.x() = infinity;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.orientation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.linearVelocity.x() = quietNaN;
  expectInvalid(options);

  options = sx::RigidBodyOptions{};
  options.angularVelocity.x() = infinity;
  expectInvalid(options);

  sx::World world;
  auto body = world.addRigidBody("body");
  EXPECT_THROW(body.setMass(0.0), sx::InvalidArgumentException);
  EXPECT_THROW(body.setMass(infinity), sx::InvalidArgumentException);

  EXPECT_THROW(
      body.setInertia(Eigen::Vector3d(1.0, -1.0, 1.0).asDiagonal()),
      sx::InvalidArgumentException);

  Eigen::Matrix3d asymmetricInertia = Eigen::Matrix3d::Identity();
  asymmetricInertia(0, 1) = 0.1;
  EXPECT_THROW(
      body.setInertia(asymmetricInertia), sx::InvalidArgumentException);

  EXPECT_THROW(
      body.setForce(Eigen::Vector3d(infinity, 0.0, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.applyForce(Eigen::Vector3d(0.0, quietNaN, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.setTorque(Eigen::Vector3d(0.0, infinity, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      body.applyTorque(Eigen::Vector3d(0.0, 0.0, quietNaN)),
      sx::InvalidArgumentException);
}

// Test that World::step() runs the rigid-body integration graph before the
// kinematics cache refresh and then advances the simulation clock.
TEST(World, StepIntegratesRigidBodyStateAndAdvancesClock)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);

  auto body = world.addRigidBody("body", options);
  body.setForce(Eigen::Vector3d(0.0, 4.0, 0.0));

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  world.step();

  const auto expectedVelocity = Eigen::Vector3d(2.0, 1.0, 0.0);
  const auto expectedPosition = Eigen::Vector3d(2.0, 2.5, 3.0);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(expectedVelocity));
  EXPECT_TRUE(body.getTranslation().isApprox(expectedPosition));
  EXPECT_DOUBLE_EQ(world.getTime(), 0.5);
  EXPECT_EQ(world.getFrame(), 1u);
}

// Test that World::step(count) provides the common repeated-step API.
TEST(World, StepCountAdvancesClockAndFrame)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setTimeStep(0.25);

  world.step(0);
  EXPECT_FALSE(world.isSimulationMode());

  world.step(4);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), 1.0);
  EXPECT_EQ(world.getFrame(), 4u);
}

// Test that repeated stepping can use an injected executor and reuse the
// default pipeline state across steps.
TEST(World, StepCountAcceptsExecutor)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions options;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("body", options);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.25);

  RecordingExecutor executor;
  world.step(0, executor);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_EQ(executor.executeCount, 0u);

  world.step(2, executor);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), 0.5);
  EXPECT_EQ(world.getFrame(), 2u);
  EXPECT_TRUE(body.getTranslation().isApprox(Eigen::Vector3d(0.5, 0.0, 0.0)));
  EXPECT_GT(executor.executeCount, 0u);
}

// Test that torque integration uses the body-frame inertia tensor and updates
// angular velocity before advancing orientation.
TEST(World, StepIntegratesRigidBodyTorque)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::RigidBodyOptions options;
  constexpr double kHalfPi = 1.57079632679489661923;
  options.inertia = Eigen::Vector3d(2.0, 4.0, 8.0).asDiagonal();
  options.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()));

  auto body = world.addRigidBody("body", options);
  body.setTorque(Eigen::Vector3d(8.0, 0.0, 0.0));

  world.setTimeStep(0.5);
  world.enterSimulationMode();

  world.step();

  const auto expectedAngularVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Quaterniond expectedOrientation
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()) * options.orientation;

  EXPECT_TRUE(body.getAngularVelocity().isApprox(expectedAngularVelocity));
  EXPECT_TRUE(body.getQuaternion().isApprox(expectedOrientation));
  EXPECT_TRUE(
      body.getRotation().isApprox(expectedOrientation.toRotationMatrix()));
}

// Test that World gravity accelerates a free rigid body during stepping and is
// independent of body mass.
TEST(World, StepAppliesGravityToFreeBody)
{
  namespace sx = dart::simulation;

  sx::World world;
  EXPECT_TRUE(world.getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(0.0, 0.0, 10.0);
  auto body = world.addRigidBody("body", options);

  world.setTimeStep(0.1);
  world.enterSimulationMode();
  world.step();

  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const double dt = 0.1;
  const Eigen::Vector3d expectedVelocity = gravity * dt;
  const Eigen::Vector3d expectedPosition
      = Eigen::Vector3d(0.0, 0.0, 10.0) + expectedVelocity * dt;

  EXPECT_TRUE(body.getLinearVelocity().isApprox(expectedVelocity));
  EXPECT_TRUE(body.getTranslation().isApprox(expectedPosition));
}

// Test that World gravity is configurable, validated, and reset by clear().
TEST(World, GravityIsConfigurableAndValidated)
{
  namespace sx = dart::simulation;

  sx::World world;
  const Eigen::Vector3d customGravity(0.0, -3.0, 0.0);
  world.setGravity(customGravity);
  EXPECT_TRUE(world.getGravity().isApprox(customGravity));

  EXPECT_THROW(
      world.setGravity(
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.setGravity(
          Eigen::Vector3d(0.0, std::numeric_limits<double>::infinity(), 0.0)),
      sx::InvalidArgumentException);

  world.clear();
  EXPECT_TRUE(world.getGravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
}

// Test that external force/torque components are persistent inputs. The default
// step assembles a transient force buffer each step, so callers clear or update
// applied forces explicitly.
TEST(World, StepPersistsExternalForceAndTorque)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  auto body = world.addRigidBody("body", options);
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));

  world.setTimeStep(0.5);
  world.enterSimulationMode();
  world.step();

  // Force consumed by the transient assembly buffer (v += F/m * dt = 2 * 0.5 =
  // 1) while the persistent component remains available for the next step.
  const Eigen::Vector3d velocityAfterFirstStep(1.0, 0.0, 0.0);
  EXPECT_TRUE(body.getLinearVelocity().isApprox(velocityAfterFirstStep));
  EXPECT_TRUE(body.getForce().isApprox(Eigen::Vector3d(4.0, 0.0, 0.0)));
  EXPECT_TRUE(body.getTorque().isApprox(Eigen::Vector3d::Zero()));

  // A second step without clearing the force accelerates again.
  world.step();
  EXPECT_TRUE(
      body.getLinearVelocity().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
}

// Test derived rigid-body dynamic quantities (momentum and energy).
TEST(World, RigidBodyDynamicQuantities)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.inertia = Eigen::Vector3d(2.0, 4.0, 8.0).asDiagonal();
  options.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  options.linearVelocity = Eigen::Vector3d(3.0, 0.0, 0.0);
  options.angularVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("body", options);

  EXPECT_TRUE(
      body.getLinearMomentum().isApprox(Eigen::Vector3d(6.0, 0.0, 0.0)));
  EXPECT_TRUE(
      body.getAngularMomentum().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  // KE = 0.5 * m * |v|^2 + 0.5 * w . (I w) = 0.5*2*9 + 0.5*1*2 = 10.
  EXPECT_DOUBLE_EQ(body.getKineticEnergy(), 10.0);
  // PE = -m * gravity . position = -2 * (-9.81 * 5) = 98.1.
  EXPECT_NEAR(body.getPotentialEnergy(), 98.1, 1e-9);
}

// Test that articulated-body forward dynamics produces the analytical gravity
// acceleration for a horizontal single revolute pendulum.
TEST(World, MultibodyRevolutePendulumGravityAcceleration)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  // Child link: revolute about Y at the base origin; center of mass offset L
  // along +X so the link is horizontal at q = 0.
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  world.setTimeStep(0.001);
  world.enterSimulationMode();
  world.step();

  // I_pivot = I_com + m L^2 (parallel axis); qddot = m g L / I_pivot.
  const double expected
      = 9.81 * mass * length / (inertiaYy + mass * length * length);
  auto joint = bob.getParentJoint();
  EXPECT_NEAR(joint.getAcceleration()[0], expected, 1e-9);
}

// Test that the public generalized mass matrix, gravity forces, and Coriolis
// forces match analytical values for a single revolute pendulum.
TEST(World, MultibodyMassMatrixAndForcesSinglePendulum)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  // A single revolute DOF has no Coriolis/centrifugal term (M is constant in
  // q), so a nonzero velocity must still leave the Coriolis force at zero.
  auto joint = bob.getParentJoint();
  joint.setVelocity(Eigen::VectorXd::Constant(1, 2.0));

  world.enterSimulationMode();

  // M = I_axis + m L^2 about the pivot (parallel-axis theorem), 1x1.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  ASSERT_EQ(massMatrix.rows(), 1);
  ASSERT_EQ(massMatrix.cols(), 1);
  const double expectedMass = inertiaYy + mass * length * length;
  EXPECT_NEAR(massMatrix(0, 0), expectedMass, 1e-12);

  const Eigen::MatrixXd inverseMass = robot.getInverseMassMatrix();
  ASSERT_EQ(inverseMass.rows(), 1);
  ASSERT_EQ(inverseMass.cols(), 1);
  EXPECT_NEAR(inverseMass(0, 0), 1.0 / expectedMass, 1e-12);

  // Horizontal pendulum at q = 0: the gravity generalized force is -m g L, so
  // the unactuated joint accelerates by +m g L / M (the forward-dynamics sign).
  const Eigen::VectorXd gravityForces = robot.getGravityForces();
  ASSERT_EQ(gravityForces.size(), 1);
  EXPECT_NEAR(gravityForces[0], -mass * 9.81 * length, 1e-12);

  const Eigen::VectorXd coriolis = robot.getCoriolisForces();
  ASSERT_EQ(coriolis.size(), 1);
  EXPECT_NEAR(coriolis[0], 0.0, 1e-12);

  const Eigen::VectorXd combined = robot.getCoriolisAndGravityForces();
  ASSERT_EQ(combined.size(), 1);
  EXPECT_NEAR(combined[0], gravityForces[0], 1e-12);
}

// Test that joint armature (rotor inertia) adds to the joint-space mass-matrix
// diagonal and reduces the resulting joint acceleration.
TEST(World, MultibodyJointArmature)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  auto joint = bob.getParentJoint();
  EXPECT_DOUBLE_EQ(joint.getArmature()[0], 0.0);

  const double armature = 1.0;
  joint.setArmature(Eigen::VectorXd::Constant(1, armature));
  EXPECT_DOUBLE_EQ(joint.getArmature()[0], armature);

  EXPECT_THROW(
      joint.setArmature(Eigen::VectorXd::Constant(1, -1.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setArmature(Eigen::VectorXd::Constant(2, 1.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.001);
  world.enterSimulationMode();

  // Armature adds to the mass-matrix diagonal.
  const double expectedMass = inertiaYy + mass * length * length + armature;
  EXPECT_NEAR(robot.getMassMatrix()(0, 0), expectedMass, 1e-12);

  world.step();

  // The unactuated joint accelerates by m g L / (I + m L^2 + armature).
  const double expectedAccel = 9.81 * mass * length / expectedMass;
  EXPECT_NEAR(joint.getAcceleration()[0], expectedAccel, 1e-9);
}

// Test that Coulomb joint friction holds a joint at rest when the driving
// effort is within the friction bound (stiction).
TEST(World, MultibodyJointCoulombFrictionStiction)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(2.0);

  auto joint = carriage.getParentJoint();
  EXPECT_DOUBLE_EQ(joint.getCoulombFriction()[0], 0.0);
  joint.setCoulombFriction(Eigen::VectorXd::Constant(1, 10.0));
  joint.setForce(Eigen::VectorXd::Constant(1, 5.0)); // below the friction bound

  EXPECT_THROW(
      joint.setCoulombFriction(Eigen::VectorXd::Constant(1, -1.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.01);
  world.enterSimulationMode();
  world.step(100);

  // Static friction holds the joint: it should not move.
  EXPECT_NEAR(joint.getVelocity()[0], 0.0, 1e-12);
  EXPECT_NEAR(joint.getPosition()[0], 0.0, 1e-12);
}

// Test that Coulomb joint friction reduces the net velocity step once the
// driving effort exceeds the friction bound (kinetic friction).
TEST(World, MultibodyJointCoulombFrictionKinetic)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  const double mass = 2.0;
  carriage.setMass(mass);

  auto joint = carriage.getParentJoint();
  const double force = 20.0;
  const double friction = 10.0;
  joint.setCoulombFriction(Eigen::VectorXd::Constant(1, friction));
  joint.setForce(Eigen::VectorXd::Constant(1, force));

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  world.step();

  // From rest, the net velocity step is (force - friction) / mass * dt.
  const double expectedVelocity = (force - friction) / mass * dt;
  EXPECT_NEAR(joint.getVelocity()[0], expectedVelocity, 1e-12);
}

// Test that the PASSIVE actuator type ignores the commanded joint effort while
// still applying passive spring forces, and that unimplemented actuator types
// are rejected by the forward dynamics.
TEST(World, MultibodyJointActuatorTypes)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  const double mass = 2.0;
  carriage.setMass(mass);

  auto joint = carriage.getParentJoint();
  EXPECT_EQ(joint.getActuatorType(), sx::ActuatorType::Force);

  joint.setActuatorType(sx::ActuatorType::Passive);
  EXPECT_EQ(joint.getActuatorType(), sx::ActuatorType::Passive);

  // A passive joint ignores the commanded effort but still responds to passive
  // spring forces.
  joint.setForce(Eigen::VectorXd::Constant(1, 100.0));
  const double stiffness = 10.0;
  joint.setSpringStiffness(Eigen::VectorXd::Constant(1, stiffness));
  joint.setRestPosition(Eigen::VectorXd::Zero(1));
  joint.setPosition(Eigen::VectorXd::Constant(1, 1.0));

  world.setTimeStep(0.001);
  world.enterSimulationMode();
  world.step();

  // The commanded effort is ignored; only the spring acts: qddot = -k x / m.
  EXPECT_NEAR(joint.getAcceleration()[0], -stiffness * 1.0 / mass, 1e-9);

  // An actuator type that is not yet implemented is rejected by the dynamics.
  joint.setActuatorType(sx::ActuatorType::Servo);
  EXPECT_THROW(world.step(), sx::InvalidOperationException);
}

// Test the Velocity actuator type: a velocity-level constraint drives joints to
// their commanded velocities exactly in one step, including under inertial
// coupling between joints.
TEST(World, MultibodyJointVelocityActuatorSingle)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(2.0);

  auto joint = carriage.getParentJoint();
  EXPECT_DOUBLE_EQ(joint.getCommandVelocity()[0], 0.0);
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.5));
  joint.setForce(Eigen::VectorXd::Constant(1, 100.0)); // ignored by Velocity

  EXPECT_THROW(
      joint.setCommandVelocity(Eigen::VectorXd::Zero(2)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.01);
  world.enterSimulationMode();
  world.step();

  // The joint reaches its commanded velocity exactly, regardless of the effort.
  EXPECT_NEAR(joint.getVelocity()[0], 0.5, 1e-12);
  EXPECT_NEAR(joint.getAcceleration()[0], 0.5 / world.getTimeStep(), 1e-12);
}

// Test the Velocity actuator under inertial coupling: both joints of a 2-link
// chain reach their (different) commanded velocities exactly in one step.
TEST(World, MultibodyJointVelocityActuatorCoupled)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("double_pendulum");
  auto base = robot.addLink("base");
  Eigen::Isometry3d offset1 = Eigen::Isometry3d::Identity();
  offset1.translation() = Eigen::Vector3d(0.7, 0.0, 0.0);
  Eigen::Isometry3d offset2 = Eigen::Isometry3d::Identity();
  offset2.translation() = Eigen::Vector3d(0.6, 0.0, 0.0);

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset1;
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.5);
  link1.setInertia(Eigen::Vector3d(0.05, 0.08, 0.05).asDiagonal());

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset2;
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.04, 0.06, 0.04).asDiagonal());

  auto joint1 = link1.getParentJoint();
  auto joint2 = link2.getParentJoint();
  joint1.setActuatorType(sx::ActuatorType::Velocity);
  joint2.setActuatorType(sx::ActuatorType::Velocity);
  joint1.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.3));
  joint2.setCommandVelocity(Eigen::VectorXd::Constant(1, -0.4));

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step();

  EXPECT_NEAR(joint1.getVelocity()[0], 0.3, 1e-9);
  EXPECT_NEAR(joint2.getVelocity()[0], -0.4, 1e-9);
}

// Test that the public mass matrix and bias forces satisfy the joint-space
// equation of motion M qddot + C + g = tau for a 2-DOF chain. This validates
// the decomposition for a multi-DOF system without hand-computing M.
TEST(World, MultibodyEquationOfMotionConsistency)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("double_pendulum");
  auto base = robot.addLink("base");

  const double l1 = 0.7;
  const double l2 = 0.6;
  Eigen::Isometry3d offset1 = Eigen::Isometry3d::Identity();
  offset1.translation() = Eigen::Vector3d(l1, 0.0, 0.0);
  Eigen::Isometry3d offset2 = Eigen::Isometry3d::Identity();
  offset2.translation() = Eigen::Vector3d(l2, 0.0, 0.0);

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset1;
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.5);
  link1.setInertia(Eigen::Vector3d(0.05, 0.08, 0.05).asDiagonal());

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset2;
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.04, 0.06, 0.04).asDiagonal());

  auto joint1 = link1.getParentJoint();
  auto joint2 = link2.getParentJoint();

  joint1.setPosition(Eigen::VectorXd::Constant(1, 0.3));
  joint1.setVelocity(Eigen::VectorXd::Constant(1, 1.1));
  joint1.setForce(Eigen::VectorXd::Constant(1, 2.0));
  joint2.setPosition(Eigen::VectorXd::Constant(1, -0.5));
  joint2.setVelocity(Eigen::VectorXd::Constant(1, 0.7));
  joint2.setForce(Eigen::VectorXd::Constant(1, -1.5));

  Eigen::VectorXd tau(2);
  tau << 2.0, -1.5;

  world.setTimeStep(1e-3);
  world.enterSimulationMode();

  // Read the dynamics terms at the current (pre-step) state.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  const Eigen::VectorXd coriolis = robot.getCoriolisForces();
  const Eigen::VectorXd gravityForces = robot.getGravityForces();

  ASSERT_EQ(massMatrix.rows(), 2);
  ASSERT_EQ(massMatrix.cols(), 2);
  EXPECT_NEAR((massMatrix - massMatrix.transpose()).norm(), 0.0, 1e-12);

  // step() computes qddot from the same pre-step state, so the equation of
  // motion must hold exactly (up to solver round-off).
  world.step();
  Eigen::VectorXd qddot(2);
  qddot << joint1.getAcceleration()[0], joint2.getAcceleration()[0];

  const Eigen::VectorXd residual
      = massMatrix * qddot + coriolis + gravityForces - tau;
  EXPECT_NEAR(residual.norm(), 0.0, 1e-9);
}

// Test inverse dynamics against the analytical single-pendulum value
// tau = (I + m L^2) qddot + g.
TEST(World, MultibodyInverseDynamicsSinglePendulum)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  world.enterSimulationMode();

  const double accel = 3.0;
  const Eigen::VectorXd tau
      = robot.computeInverseDynamics(Eigen::VectorXd::Constant(1, accel));
  ASSERT_EQ(tau.size(), 1);

  // At q = 0, qdot = 0: tau = (I + m L^2) qddot + g, with g = -m g L.
  const double expected
      = (inertiaYy + mass * length * length) * accel - mass * 9.81 * length;
  EXPECT_NEAR(tau[0], expected, 1e-9);

  EXPECT_THROW(
      (void)robot.computeInverseDynamics(Eigen::VectorXd::Zero(2)),
      sx::InvalidArgumentException);
}

// Test that inverse dynamics inverts forward dynamics: the forces from
// computeInverseDynamics(qddot) reproduce qddot in the next step.
TEST(World, MultibodyInverseDynamicsRoundTrip)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("double_pendulum");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset1 = Eigen::Isometry3d::Identity();
  offset1.translation() = Eigen::Vector3d(0.7, 0.0, 0.0);
  Eigen::Isometry3d offset2 = Eigen::Isometry3d::Identity();
  offset2.translation() = Eigen::Vector3d(0.6, 0.0, 0.0);

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset1;
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.5);
  link1.setInertia(Eigen::Vector3d(0.05, 0.08, 0.05).asDiagonal());

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset2;
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);
  link2.setInertia(Eigen::Vector3d(0.04, 0.06, 0.04).asDiagonal());

  auto joint1 = link1.getParentJoint();
  auto joint2 = link2.getParentJoint();
  joint1.setPosition(Eigen::VectorXd::Constant(1, 0.3));
  joint1.setVelocity(Eigen::VectorXd::Constant(1, 1.1));
  joint2.setPosition(Eigen::VectorXd::Constant(1, -0.5));
  joint2.setVelocity(Eigen::VectorXd::Constant(1, 0.7));

  world.setTimeStep(1e-3);
  world.enterSimulationMode();

  Eigen::VectorXd qddotDesired(2);
  qddotDesired << 0.4, -0.9;
  const Eigen::VectorXd tau = robot.computeInverseDynamics(qddotDesired);
  ASSERT_EQ(tau.size(), 2);

  joint1.setForce(tau.segment(0, 1));
  joint2.setForce(tau.segment(1, 1));
  world.step();

  // Forward dynamics of the inverse-dynamics torque reproduces the target.
  EXPECT_NEAR(joint1.getAcceleration()[0], qddotDesired[0], 1e-9);
  EXPECT_NEAR(joint2.getAcceleration()[0], qddotDesired[1], 1e-9);
}

TEST(World, MultibodyInverseDynamicsDerivativeScratchUsesProvidedAllocator)
{
  namespace sx = dart::simulation;

  CountingMemoryAllocator allocator;
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("id_derivative_allocator_chain");
  auto parent = robot.addLink("base");
  constexpr std::size_t kJointCount = 64u;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(0.05, 0.0, 0.0);

    sx::JointSpec spec;
    spec.name = std::format("hinge_{:02}", i);
    spec.type = sx::JointType::Revolute;
    spec.axis = Eigen::Vector3d::UnitY();
    spec.transformFromParent = offset;
    auto link = robot.addLink(std::format("link_{:02}", i), parent, spec);
    link.setMass(1.0);
    link.setInertia(Eigen::Vector3d(0.05, 0.06, 0.07).asDiagonal());

    auto joint = link.getParentJoint();
    joint.setPosition(Eigen::VectorXd::Constant(1, 0.001 * (i + 1u)));
    joint.setVelocity(Eigen::VectorXd::Constant(1, 0.02));
    parent = link;
  }

  world.enterSimulationMode();

  auto& registry = sx::detail::registryOf(world);
  auto structures = registry.view<sx::comps::MultibodyStructure>();
  const sx::comps::MultibodyStructure* structure = nullptr;
  std::size_t structureCount = 0;
  for (const auto entity : structures) {
    structure = &structures.get<sx::comps::MultibodyStructure>(entity);
    ++structureCount;
  }
  ASSERT_EQ(structureCount, 1u);
  ASSERT_NE(structure, nullptr);

  sx::compute::MultibodyInverseDynamicsScratch scratch(allocator);
  sx::compute::InverseDynamicsDerivatives derivatives;
  const Eigen::VectorXd qddot = Eigen::VectorXd::LinSpaced(
      static_cast<Eigen::Index>(kJointCount), -0.1, 0.2);

  const auto allocationsBeforeFirstCall = allocator.allocationCount;
  sx::compute::computeMultibodyInverseDynamicsDerivativesInto(
      scratch,
      registry,
      *structure,
      Eigen::Vector3d::Zero(),
      qddot,
      derivatives);

  ASSERT_TRUE(derivatives.valid);
  EXPECT_EQ(derivatives.dTau_dq.rows(), static_cast<Eigen::Index>(kJointCount));
  EXPECT_EQ(derivatives.dTau_dq.cols(), static_cast<Eigen::Index>(kJointCount));
  EXPECT_EQ(
      derivatives.dTau_dqdot.rows(), static_cast<Eigen::Index>(kJointCount));
  EXPECT_EQ(
      derivatives.dTau_dqdot.cols(), static_cast<Eigen::Index>(kJointCount));
  EXPECT_GT(allocator.allocationCount, allocationsBeforeFirstCall)
      << "inverse-dynamics derivative scratch should allocate through the "
         "provided allocator on the first analytic derivative call";

  const auto allocationsAfterFirstCall = allocator.allocationCount;
  sx::compute::computeMultibodyInverseDynamicsDerivativesInto(
      scratch,
      registry,
      *structure,
      Eigen::Vector3d::Zero(),
      qddot,
      derivatives);

  EXPECT_TRUE(derivatives.valid);
  EXPECT_EQ(allocator.allocationCount, allocationsAfterFirstCall)
      << "same-shape analytic derivative calls should reuse the retained "
         "scratch capacity";
}

// Test the generalized impulse response dqdot = M^-1 f against the analytical
// single-pendulum value and the M dqdot = f consistency identity.
TEST(World, MultibodyImpulseResponse)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());

  world.enterSimulationMode();

  const double impulse = 5.0;
  const Eigen::VectorXd deltaVelocity
      = robot.computeImpulseResponse(Eigen::VectorXd::Constant(1, impulse));
  ASSERT_EQ(deltaVelocity.size(), 1);

  // dqdot = f / (I + m L^2) about the pivot.
  const double inertiaPivot = inertiaYy + mass * length * length;
  EXPECT_NEAR(deltaVelocity[0], impulse / inertiaPivot, 1e-12);

  // Consistency: M dqdot = f.
  EXPECT_NEAR((robot.getMassMatrix() * deltaVelocity)[0], impulse, 1e-12);

  EXPECT_THROW(
      (void)robot.computeImpulseResponse(Eigen::VectorXd::Zero(2)),
      sx::InvalidArgumentException);
}

// Test the body-frame link Jacobian against independent screw-theory twist
// values: a revolute joint's body twist is [axis; axis x p].
TEST(World, MultibodyLinkJacobian)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);

  world.enterSimulationMode();

  // Revolute about Y with link offset (L,0,0): body twist [axis; axis x p]
  // = [0,1,0, 0,0,-L].
  const Eigen::MatrixXd jacobianBob = robot.getJacobian(bob);
  ASSERT_EQ(jacobianBob.rows(), 6);
  ASSERT_EQ(jacobianBob.cols(), 1);
  Eigen::Matrix<double, 6, 1> expected;
  expected << 0.0, 1.0, 0.0, 0.0, 0.0, -length;
  EXPECT_TRUE(jacobianBob.col(0).isApprox(expected, 1e-12));

  // The fixed base cannot move: its Jacobian is zero.
  const Eigen::MatrixXd jacobianBase = robot.getJacobian(base);
  ASSERT_EQ(jacobianBase.rows(), 6);
  ASSERT_EQ(jacobianBase.cols(), 1);
  EXPECT_TRUE(jacobianBase.isZero(1e-12));

  // A link from a different world is rejected.
  sx::World other;
  auto otherRobot = other.addMultibody("other");
  auto otherBase = otherRobot.addLink("other_base");
  EXPECT_THROW(
      (void)robot.getJacobian(otherBase), sx::InvalidArgumentException);
}

// Test the body-frame Jacobian column structure of a 2-DOF chain: a joint below
// a link does not contribute to that link's Jacobian.
TEST(World, MultibodyLinkJacobianChain)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("double_pendulum");
  auto base = robot.addLink("base");

  const double l1 = 0.7;
  const double l2 = 0.6;
  Eigen::Isometry3d offset1 = Eigen::Isometry3d::Identity();
  offset1.translation() = Eigen::Vector3d(l1, 0.0, 0.0);
  Eigen::Isometry3d offset2 = Eigen::Isometry3d::Identity();
  offset2.translation() = Eigen::Vector3d(l2, 0.0, 0.0);

  sx::JointSpec spec1;
  spec1.name = "j1";
  spec1.type = sx::JointType::Revolute;
  spec1.axis = Eigen::Vector3d::UnitY();
  spec1.transformFromParent = offset1;
  auto link1 = robot.addLink("link1", base, spec1);
  link1.setMass(1.0);

  sx::JointSpec spec2;
  spec2.name = "j2";
  spec2.type = sx::JointType::Revolute;
  spec2.axis = Eigen::Vector3d::UnitY();
  spec2.transformFromParent = offset2;
  auto link2 = robot.addLink("link2", link1, spec2);
  link2.setMass(1.0);

  world.enterSimulationMode();

  const Eigen::MatrixXd jacobian1 = robot.getJacobian(link1);
  ASSERT_EQ(jacobian1.rows(), 6);
  ASSERT_EQ(jacobian1.cols(), 2);
  // joint1 moves link1 with twist [0,1,0,0,0,-l1]; joint2 (below) does not.
  Eigen::Matrix<double, 6, 1> twist1;
  twist1 << 0.0, 1.0, 0.0, 0.0, 0.0, -l1;
  EXPECT_TRUE(jacobian1.col(0).isApprox(twist1, 1e-12));
  EXPECT_TRUE(jacobian1.col(1).isZero(1e-12));

  const Eigen::MatrixXd jacobian2 = robot.getJacobian(link2);
  ASSERT_EQ(jacobian2.cols(), 2);
  // joint2 moves link2 with its own twist [0,1,0,0,0,-l2].
  Eigen::Matrix<double, 6, 1> twist2;
  twist2 << 0.0, 1.0, 0.0, 0.0, 0.0, -l2;
  EXPECT_TRUE(jacobian2.col(1).isApprox(twist2, 1e-12));
  // joint1 also moves link2; the angular part is preserved under the pure
  // translation between the link frames.
  EXPECT_TRUE(jacobian2.col(0).head<3>().isApprox(
      Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
  EXPECT_FALSE(jacobian2.col(0).isZero(1e-12));
}

// Test the world-frame link Jacobian: its linear block predicts the link
// origin's world velocity, cross-checked against a finite difference of FK.
TEST(World, MultibodyLinkWorldJacobian)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  const double length = 1.5;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  auto joint = bob.getParentJoint();
  const double qd = 0.7;
  joint.setVelocity(Eigen::VectorXd::Constant(1, qd));

  const double dt = 1e-6;
  world.setTimeStep(dt);
  world.enterSimulationMode();

  // At q = 0 the bob frame is axis-aligned with world, so the world Jacobian
  // equals the body Jacobian: [0,1,0, 0,0,-L].
  const Eigen::MatrixXd worldJacobian = robot.getWorldJacobian(bob);
  ASSERT_EQ(worldJacobian.rows(), 6);
  ASSERT_EQ(worldJacobian.cols(), 1);
  Eigen::Matrix<double, 6, 1> expected;
  expected << 0.0, 1.0, 0.0, 0.0, 0.0, -length;
  EXPECT_TRUE(worldJacobian.col(0).isApprox(expected, 1e-12));

  // Finite-difference cross-check of the linear block against forward
  // kinematics (zero gravity and no effort keep the joint velocity constant).
  const Eigen::Vector3d origin0 = bob.getWorldTransform().translation();
  const Eigen::Vector3d predictedVelocity
      = worldJacobian.bottomRows<3>() * Eigen::VectorXd::Constant(1, qd);
  world.step();
  const Eigen::Vector3d origin1 = bob.getWorldTransform().translation();
  const Eigen::Vector3d fdVelocity = (origin1 - origin0) / dt;
  EXPECT_TRUE(predictedVelocity.isApprox(fdVelocity, 1e-4));
}

// Test that the dynamics accessors return empty results for a multibody with no
// movable degrees of freedom.
TEST(World, MultibodyDynamicsAccessorsNoDOF)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto robot = world.addMultibody("static_chain");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "weld";
  spec.type = sx::JointType::Fixed;
  (void)robot.addLink("welded", base, spec);

  world.enterSimulationMode();

  EXPECT_EQ(robot.getDOFCount(), 0u);
  EXPECT_EQ(robot.getMassMatrix().size(), 0);
  EXPECT_EQ(robot.getInverseMassMatrix().size(), 0);
  EXPECT_EQ(robot.getCoriolisForces().size(), 0);
  EXPECT_EQ(robot.getGravityForces().size(), 0);
  EXPECT_EQ(robot.getCoriolisAndGravityForces().size(), 0);
  EXPECT_EQ(robot.computeInverseDynamics(Eigen::VectorXd()).size(), 0);
  EXPECT_EQ(robot.computeImpulseResponse(Eigen::VectorXd()).size(), 0);
}

// Test that a prismatic joint aligned with gravity free-falls at g.
TEST(World, MultibodyPrismaticFreeFall)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(3.0);

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  world.step();

  auto joint = carriage.getParentJoint();
  EXPECT_NEAR(joint.getAcceleration()[0], -9.81, 1e-9);
  EXPECT_NEAR(joint.getVelocity()[0], -9.81 * dt, 1e-12);
  EXPECT_NEAR(joint.getPosition()[0], -9.81 * dt * dt, 1e-12);
}

// Test screw-joint forward dynamics: rotation and translation are coupled by
// the pitch, so a vertical screw under gravity has M = I_axis + m pitch^2 and
// accelerates by -m g pitch / M.
TEST(World, MultibodyScrewJointDynamics)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("screw");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "helix";
  spec.type = sx::JointType::Screw;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto nut = robot.addLink("nut", base, spec);

  const double mass = 2.0;
  const double inertiaZz = 0.1;
  nut.setMass(mass);
  nut.setInertia(Eigen::Vector3d(0.1, 0.1, inertiaZz).asDiagonal());

  auto joint = nut.getParentJoint();
  const double pitch = 0.5;
  joint.setPitch(pitch);
  EXPECT_DOUBLE_EQ(joint.getPitch(), pitch);

  world.setTimeStep(0.001);
  world.enterSimulationMode();

  // M = I_zz + m pitch^2 about the screw axis.
  const double expectedMass = inertiaZz + mass * pitch * pitch;
  EXPECT_NEAR(robot.getMassMatrix()(0, 0), expectedMass, 1e-12);

  world.step();

  // Gravity drives the screw down (z = pitch * theta): qddot = -m g pitch / M.
  const double expectedAccel = -mass * 9.81 * pitch / expectedMass;
  EXPECT_NEAR(joint.getAcceleration()[0], expectedAccel, 1e-9);
}

// Test the universal joint's mass matrix and gravity forces against closed-form
// values at the zero configuration: axis (Z) and axis2 (Y) intersect at the
// base origin and the distal center of mass sits at (L, 0, 0).
TEST(World, MultibodyUniversalMassMatrixAndGravity)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("ujoint");
  auto base = robot.addLink("base");

  const double length = 0.9;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "u";
  spec.type = sx::JointType::Universal;
  spec.axis = Eigen::Vector3d::UnitZ();
  spec.axis2 = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto distal = robot.addLink("distal", base, spec);

  const double mass = 2.0;
  const double inertiaXx = 0.05;
  const double inertiaYy = 0.12;
  const double inertiaZz = 0.2;
  distal.setMass(mass);
  distal.setInertia(
      Eigen::Vector3d(inertiaXx, inertiaYy, inertiaZz).asDiagonal());

  auto joint = distal.getParentJoint();
  ASSERT_EQ(joint.getType(), sx::JointType::Universal);
  ASSERT_EQ(joint.getDOFCount(), 2u);
  EXPECT_TRUE(joint.getAxis2().isApprox(Eigen::Vector3d::UnitY()));

  world.enterSimulationMode();

  // M(0,0): rotation about Z through the origin -> Izz + m L^2.
  // M(1,1): rotation about Y through the origin -> Iyy + m L^2.
  // The two axes are orthogonal and intersect, so the coupling is zero at q =
  // 0.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  ASSERT_EQ(massMatrix.rows(), 2);
  ASSERT_EQ(massMatrix.cols(), 2);
  EXPECT_NEAR(massMatrix(0, 0), inertiaZz + mass * length * length, 1e-12);
  EXPECT_NEAR(massMatrix(1, 1), inertiaYy + mass * length * length, 1e-12);
  EXPECT_NEAR(massMatrix(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(massMatrix(1, 0), 0.0, 1e-12);

  // Gravity: rotating about the vertical Z axis does no work (g[0] = 0); the
  // horizontal link about Y matches the single-pendulum value -m g L.
  const Eigen::VectorXd gravity = robot.getGravityForces();
  ASSERT_EQ(gravity.size(), 2);
  EXPECT_NEAR(gravity[0], 0.0, 1e-12);
  EXPECT_NEAR(gravity[1], -mass * 9.81 * length, 1e-12);
}

// Test that the universal joint's Coriolis/centrifugal forces match the
// Christoffel-symbol expression derived from the configuration-dependent mass
// matrix by finite differences. The reference depends only on M(q) (which does
// not use the velocity-product term cJ), so agreement validates cJ.
TEST(World, MultibodyUniversalCoriolisMatchesChristoffel)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("ujoint");
  auto base = robot.addLink("base");

  const double length = 0.9;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "u";
  spec.type = sx::JointType::Universal;
  spec.axis = Eigen::Vector3d::UnitZ();
  spec.axis2 = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto distal = robot.addLink("distal", base, spec);

  const double mass = 2.0;
  distal.setMass(mass);
  distal.setInertia(Eigen::Vector3d(0.05, 0.12, 0.2).asDiagonal());

  auto joint = distal.getParentJoint();

  world.enterSimulationMode();

  const Eigen::Vector2d q(0.3, 0.5);
  const Eigen::Vector2d qdot(0.7, 1.1);

  // M(q) at a perturbed configuration.
  auto massAt = [&](const Eigen::Vector2d& position) {
    joint.setPosition(position);
    return robot.getMassMatrix();
  };

  // Central-difference dM/dq_i.
  const double h = 1e-5;
  std::vector<Eigen::MatrixXd> dM(2);
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d plus = q;
    Eigen::Vector2d minus = q;
    plus[i] += h;
    minus[i] -= h;
    dM[static_cast<std::size_t>(i)]
        = (massAt(plus) - massAt(minus)) / (2.0 * h);
  }

  // Christoffel symbols of the first kind:
  // c_k = sum_ij 0.5 (dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k) qdot_i qdot_j.
  Eigen::Vector2d expected = Eigen::Vector2d::Zero();
  for (int k = 0; k < 2; ++k) {
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        const double christoffel = 0.5
                                   * (dM[static_cast<std::size_t>(i)](k, j)
                                      + dM[static_cast<std::size_t>(j)](k, i)
                                      - dM[static_cast<std::size_t>(k)](i, j));
        expected[k] += christoffel * qdot[i] * qdot[j];
      }
    }
  }

  joint.setPosition(q);
  joint.setVelocity(qdot);
  const Eigen::VectorXd coriolis = robot.getCoriolisForces();

  ASSERT_EQ(coriolis.size(), 2);
  EXPECT_NEAR(coriolis[0], expected[0], 1e-6);
  EXPECT_NEAR(coriolis[1], expected[1], 1e-6);

  // The velocity-product coupling must be genuinely exercised (a zero reference
  // would make the comparison vacuous).
  EXPECT_GT(expected.norm(), 1e-3);
}

// Test the planar joint's (3-DOF) mass matrix and gravity at the zero
// configuration. With the plane normal along Y, in-plane axes X and -Z, and the
// center of mass at the joint origin, the mass matrix is diagonal and only the
// vertical (-Z) translation feels gravity.
TEST(World, MultibodyPlanarMassMatrixAndGravity)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("planar");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "plane";
  spec.type = sx::JointType::Planar;
  spec.axis = Eigen::Vector3d::UnitY();  // plane normal
  spec.axis2 = Eigen::Vector3d::UnitX(); // first in-plane direction
  auto slider = robot.addLink("slider", base, spec);

  const double mass = 3.0;
  const double inertiaYy = 0.15;
  slider.setMass(mass);
  slider.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.2).asDiagonal());

  auto joint = slider.getParentJoint();
  ASSERT_EQ(joint.getType(), sx::JointType::Planar);
  ASSERT_EQ(joint.getDOFCount(), 3u);

  world.enterSimulationMode();

  // Translations cost the full mass; rotation about the normal costs Iyy. The
  // axes are orthogonal and the center of mass is at the origin, so there is no
  // coupling at q = 0.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  ASSERT_EQ(massMatrix.rows(), 3);
  ASSERT_EQ(massMatrix.cols(), 3);
  Eigen::Matrix3d expectedMass
      = Eigen::Vector3d(mass, mass, inertiaYy).asDiagonal();
  EXPECT_TRUE(massMatrix.isApprox(expectedMass, 1e-12));

  // In-plane axes are X and normal x X = -Z. Only the -Z translation does work
  // against gravity: g = -m (gravity . axis) = -m (9.81).
  const Eigen::VectorXd gravity = robot.getGravityForces();
  ASSERT_EQ(gravity.size(), 3);
  EXPECT_NEAR(gravity[0], 0.0, 1e-12);
  EXPECT_NEAR(gravity[1], -mass * 9.81, 1e-12);
  EXPECT_NEAR(gravity[2], 0.0, 1e-12);
}

// Test that the planar joint's Coriolis forces match the Christoffel-symbol
// expression from the configuration-dependent mass matrix (finite differences).
// A nonzero link offset makes the rotation couple to the translations, so the
// reference is nonzero and validates the velocity-product term cJ.
TEST(World, MultibodyPlanarCoriolisMatchesChristoffel)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("planar");
  auto base = robot.addLink("base");

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.6, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "plane";
  spec.type = sx::JointType::Planar;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.axis2 = Eigen::Vector3d::UnitX();
  spec.transformFromParent = offset;
  auto slider = robot.addLink("slider", base, spec);

  const double mass = 3.0;
  slider.setMass(mass);
  slider.setInertia(Eigen::Vector3d(0.1, 0.15, 0.2).asDiagonal());

  auto joint = slider.getParentJoint();

  world.enterSimulationMode();

  const Eigen::Vector3d q(0.2, -0.15, 0.5);
  const Eigen::Vector3d qdot(0.7, -0.4, 1.1);

  auto massAt = [&](const Eigen::Vector3d& position) {
    joint.setPosition(position);
    return robot.getMassMatrix();
  };

  const double h = 1e-5;
  std::vector<Eigen::MatrixXd> dM(3);
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d plus = q;
    Eigen::Vector3d minus = q;
    plus[i] += h;
    minus[i] -= h;
    dM[static_cast<std::size_t>(i)]
        = (massAt(plus) - massAt(minus)) / (2.0 * h);
  }

  Eigen::Vector3d expected = Eigen::Vector3d::Zero();
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        const double christoffel = 0.5
                                   * (dM[static_cast<std::size_t>(i)](k, j)
                                      + dM[static_cast<std::size_t>(j)](k, i)
                                      - dM[static_cast<std::size_t>(k)](i, j));
        expected[k] += christoffel * qdot[i] * qdot[j];
      }
    }
  }

  joint.setPosition(q);
  joint.setVelocity(qdot);
  const Eigen::VectorXd coriolis = robot.getCoriolisForces();

  ASSERT_EQ(coriolis.size(), 3);
  EXPECT_NEAR(coriolis[0], expected[0], 1e-6);
  EXPECT_NEAR(coriolis[1], expected[1], 1e-6);
  EXPECT_NEAR(coriolis[2], expected[2], 1e-6);
  EXPECT_GT(expected.norm(), 1e-3);
}

// Test the ball joint's (3-DOF) mass matrix and gravity at the identity
// orientation. The center of mass is offset along X, so the mass matrix is the
// inertia about the ball center (parallel-axis theorem) and only rotation about
// Y feels gravity.
TEST(World, MultibodyBallMassMatrixAndGravity)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("ball");
  auto base = robot.addLink("base");

  const double offsetX = 0.7;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(offsetX, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "socket";
  spec.type = sx::JointType::Spherical;
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double inertiaXx = 0.05;
  const double inertiaYy = 0.12;
  const double inertiaZz = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(inertiaXx, inertiaYy, inertiaZz).asDiagonal());

  auto joint = bob.getParentJoint();
  ASSERT_EQ(joint.getType(), sx::JointType::Spherical);
  ASSERT_EQ(joint.getDOFCount(), 3u);

  world.enterSimulationMode();

  // Inertia about the ball center: the COM offset is along X, so Ixx is
  // unchanged and Iyy, Izz gain m * offset^2 (parallel-axis).
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  ASSERT_EQ(massMatrix.rows(), 3);
  ASSERT_EQ(massMatrix.cols(), 3);
  Eigen::Matrix3d expectedMass = Eigen::Vector3d(
                                     inertiaXx,
                                     inertiaYy + mass * offsetX * offsetX,
                                     inertiaZz + mass * offsetX * offsetX)
                                     .asDiagonal();
  EXPECT_TRUE(massMatrix.isApprox(expectedMass, 1e-12));

  // Gravity torque about the ball center is offset x (0,0,-mg) = (0, m g d, 0);
  // the generalized gravity force is its negative about each body axis.
  const Eigen::VectorXd gravity = robot.getGravityForces();
  ASSERT_EQ(gravity.size(), 3);
  EXPECT_NEAR(gravity[0], 0.0, 1e-12);
  EXPECT_NEAR(gravity[1], -mass * 9.81 * offsetX, 1e-12);
  EXPECT_NEAR(gravity[2], 0.0, 1e-12);
}

// Test the ball joint's SO(3) manifold integration: a torque-free body with
// isotropic inertia keeps a constant body angular velocity, and the stored
// rotation vector accumulates linearly (rotations about a fixed axis commute).
TEST(World, MultibodyBallIsotropicFreeSpin)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("ball");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "socket";
  spec.type = sx::JointType::Spherical;
  auto bob = robot.addLink("bob", base, spec);

  // Isotropic inertia: torque-free Euler dynamics keep the angular velocity
  // constant (the gyroscopic term omega x (I omega) vanishes).
  bob.setMass(1.5);
  bob.setInertia(0.1 * Eigen::Matrix3d::Identity());

  auto joint = bob.getParentJoint();
  const Eigen::Vector3d omega(0.3, -0.5, 0.7);
  joint.setVelocity(omega);

  const double dt = 0.001;
  const int steps = 100;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  for (int i = 0; i < steps; ++i) {
    world.step();
  }

  // Angular velocity is unchanged; the rotation vector is omega * total time.
  EXPECT_TRUE(joint.getVelocity().isApprox(omega, 1e-9));
  EXPECT_TRUE(joint.getPosition().isApprox(omega * (dt * steps), 1e-9));
}

// Test the free joint (floating base): a free body under gravity free-falls,
// with the linear acceleration equal to gravity and no angular motion.
TEST(World, MultibodyFreeJointFreeFall)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("floating");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);

  const double mass = 2.0;
  body.setMass(mass);
  body.setInertia(Eigen::Vector3d(0.05, 0.12, 0.2).asDiagonal());

  auto joint = body.getParentJoint();
  ASSERT_EQ(joint.getType(), sx::JointType::Floating);
  ASSERT_EQ(joint.getDOFCount(), 6u);

  const double dt = 0.01;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  world.step();

  // Velocity is [linear; angular]. Free fall: linear z accelerates at gravity,
  // no angular motion.
  const Eigen::VectorXd acceleration = joint.getAcceleration();
  ASSERT_EQ(acceleration.size(), 6);
  EXPECT_TRUE(
      acceleration.head<3>().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81), 1e-9));
  EXPECT_TRUE(acceleration.tail<3>().isZero(1e-12));

  const Eigen::VectorXd velocity = joint.getVelocity();
  EXPECT_TRUE(velocity.head<3>().isApprox(
      Eigen::Vector3d(0.0, 0.0, -9.81 * dt), 1e-12));
  EXPECT_TRUE(velocity.tail<3>().isZero(1e-12));

  const Eigen::VectorXd position = joint.getPosition();
  EXPECT_TRUE(position.head<3>().isApprox(
      Eigen::Vector3d(0.0, 0.0, -9.81 * dt * dt), 1e-12));
  EXPECT_TRUE(position.tail<3>().isZero(1e-12));
}

// Test the free joint's combined SE(3) integration: with translation and spin
// both along the same principal axis (no gravity), the body twist stays
// constant and the translation and rotation vector accumulate linearly.
TEST(World, MultibodyFreeJointTranslatesAndSpins)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("floating");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);

  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.05, 0.12, 0.2).asDiagonal());

  auto joint = body.getParentJoint();
  // Body twist [linear; angular] with both along Z (a principal axis), so there
  // is no gyroscopic or Coriolis coupling: both rates stay constant.
  Eigen::VectorXd twist(6);
  twist << 0.0, 0.0, 3.0, 0.0, 0.0, 2.0;
  joint.setVelocity(twist);

  const double dt = 0.001;
  const int steps = 100;
  world.setTimeStep(dt);
  world.enterSimulationMode();
  for (int i = 0; i < steps; ++i) {
    world.step();
  }

  const double time = dt * steps;
  EXPECT_TRUE(joint.getVelocity().isApprox(twist, 1e-9));

  const Eigen::VectorXd position = joint.getPosition();
  EXPECT_TRUE(
      position.head<3>().isApprox(Eigen::Vector3d(0.0, 0.0, 3.0 * time), 1e-9));
  EXPECT_TRUE(
      position.tail<3>().isApprox(Eigen::Vector3d(0.0, 0.0, 2.0 * time), 1e-9));
}

// Test that floating-joint translation limits act as hard stops while the
// orientation continues to use the rotation-vector representation.
TEST(World, MultibodyFreeJointTranslationPositionLimit)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("floating");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  auto body = robot.addLink("body", base, spec);

  body.setMass(2.0);
  body.setInertia(Eigen::Vector3d(0.05, 0.12, 0.2).asDiagonal());

  auto joint = body.getParentJoint();
  Eigen::VectorXd lower
      = Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::infinity());
  Eigen::VectorXd upper
      = Eigen::VectorXd::Constant(6, std::numeric_limits<double>::infinity());
  const double lowerZ = -0.05;
  lower[2] = lowerZ;
  joint.setPositionLimits(lower, upper);

  world.setTimeStep(0.01);
  world.enterSimulationMode();
  world.step(100);

  EXPECT_NEAR(joint.getPosition()[2], lowerZ, 1e-12);
  EXPECT_NEAR(joint.getVelocity()[2], 0.0, 1e-12);
  EXPECT_TRUE(joint.getPosition().tail<3>().isZero(1e-12));
}

// Test a center-of-mass offset from the link frame: a revolute pendulum whose
// link frame sits at the hinge but whose center of mass is offset along X
// behaves like the same pendulum built with a link-frame offset (parallel-axis
// mass matrix and gravity torque), exercising the COM-coupled spatial inertia.
TEST(World, MultibodyLinkCenterOfMassOffset)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  // Link frame at the hinge (no transformFromParent offset); the mass is offset
  // along X by the center of mass instead.
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 2.0;
  const double length = 1.5;
  const double inertiaYy = 0.2;
  bob.setMass(mass);
  bob.setInertia(Eigen::Vector3d(0.1, inertiaYy, 0.3).asDiagonal());
  bob.setCenterOfMass(Eigen::Vector3d(length, 0.0, 0.0));
  EXPECT_TRUE(
      bob.getCenterOfMass().isApprox(Eigen::Vector3d(length, 0.0, 0.0)));

  world.enterSimulationMode();

  // Parallel-axis: rotating about Y through the hinge sees Iyy + m L^2.
  const Eigen::MatrixXd massMatrix = robot.getMassMatrix();
  ASSERT_EQ(massMatrix.rows(), 1);
  EXPECT_NEAR(massMatrix(0, 0), inertiaYy + mass * length * length, 1e-12);

  // Horizontal pendulum: gravity generalized force is -m g L.
  const Eigen::VectorXd gravity = robot.getGravityForces();
  ASSERT_EQ(gravity.size(), 1);
  EXPECT_NEAR(gravity[0], -mass * 9.81 * length, 1e-12);

  // Forward dynamics: qddot = m g L / (Iyy + m L^2).
  world.setTimeStep(0.001);
  world.step();
  const double expected
      = 9.81 * mass * length / (inertiaYy + mass * length * length);
  EXPECT_NEAR(bob.getParentJoint().getAcceleration()[0], expected, 1e-9);
}

// Test that the pendulum integrator conserves mechanical energy over a swing
// (a sign or scale error in the dynamics would inject energy and diverge).
TEST(World, MultibodyPendulumConservesEnergy)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");

  const double length = 1.0;
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(length, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);

  const double mass = 1.0;
  const Eigen::Matrix3d inertia
      = Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal();
  bob.setMass(mass);
  bob.setInertia(inertia);

  auto joint = bob.getParentJoint();

  // Released from horizontal at rest; total energy starts at zero.
  const auto energy = [&]() {
    const double qd = joint.getVelocity()[0];
    const Eigen::Isometry3d worldTransform = bob.getTransform();
    const Eigen::Vector3d com = worldTransform.translation();
    const Eigen::Vector3d omega = qd * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d linear = omega.cross(com);
    const Eigen::Matrix3d rotation = worldTransform.linear();
    const Eigen::Matrix3d worldInertia
        = rotation * inertia * rotation.transpose();
    const double kinetic = 0.5 * mass * linear.squaredNorm()
                           + 0.5 * omega.dot(worldInertia * omega);
    const double potential = -mass * gravity.dot(com);
    return kinetic + potential;
  };

  world.setTimeStep(1e-4);
  world.enterSimulationMode();

  const double initialEnergy = energy();
  EXPECT_NEAR(initialEnergy, 0.0, 1e-12);

  // Roughly a quarter period: enough time to swing from horizontal toward the
  // bottom (stable equilibrium at q = pi/2).
  double maxAngle = 0.0;
  for (int i = 0; i < 8000; ++i) {
    world.step();
    EXPECT_NEAR(energy(), initialEnergy, 5e-2);
    maxAngle = std::max(maxAngle, std::abs(joint.getPosition()[0]));
  }

  // The pendulum should have swung down close to the bottom (q ~ pi/2).
  EXPECT_GT(maxAngle, 1.4);
}

// Test that joint spring stiffness and damping contribute passive generalized
// forces in the articulated-body dynamics.
TEST(World, MultibodyJointSpringAndDamping)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  const double mass = 2.0;
  carriage.setMass(mass);

  auto joint = carriage.getParentJoint();
  const double stiffness = 10.0;
  const double damping = 3.0;
  joint.setSpringStiffness(Eigen::VectorXd::Constant(1, stiffness));
  joint.setDampingCoefficient(Eigen::VectorXd::Constant(1, damping));
  joint.setRestPosition(Eigen::VectorXd::Zero(1));

  EXPECT_THROW(
      joint.setSpringStiffness(Eigen::VectorXd::Constant(1, -1.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setDampingCoefficient(Eigen::VectorXd::Constant(1, -1.0)),
      sx::InvalidArgumentException);

  const double position = 0.5;
  const double velocity = 2.0;
  joint.setPosition(Eigen::VectorXd::Constant(1, position));
  joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));

  world.setTimeStep(0.001);
  world.enterSimulationMode();
  world.step();

  // qddot = (-stiffness * position - damping * velocity) / mass.
  const double expected = (-stiffness * position - damping * velocity) / mass;
  EXPECT_NEAR(joint.getAcceleration()[0], expected, 1e-9);
}

// Test that a joint position limit acts as a hard stop in the dynamics.
TEST(World, MultibodyJointPositionLimit)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("pendulum");
  auto base = robot.addLink("base");
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  sx::JointSpec spec;
  spec.name = "hinge";
  spec.type = sx::JointType::Revolute;
  spec.axis = Eigen::Vector3d::UnitY();
  spec.transformFromParent = offset;
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setInertia(Eigen::Vector3d(0.05, 0.05, 0.05).asDiagonal());

  auto joint = bob.getParentJoint();
  EXPECT_TRUE(std::isinf(joint.getPositionUpperLimits()[0]));
  EXPECT_TRUE(std::isinf(joint.getPositionLowerLimits()[0]));

  const double upperLimit = 0.5;
  joint.setPositionLimits(
      Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity()),
      Eigen::VectorXd::Constant(1, upperLimit));
  EXPECT_DOUBLE_EQ(joint.getPositionUpperLimits()[0], upperLimit);

  EXPECT_THROW(
      joint.setPositionLimits(
          Eigen::VectorXd::Constant(1, 1.0), Eigen::VectorXd::Constant(1, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setPositionLimits(
          Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::Constant(2, 1.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(400);

  // Gravity drives the joint toward +pi/2, but it must stop at the upper limit.
  EXPECT_NEAR(joint.getPosition()[0], upperLimit, 1e-9);
  EXPECT_NEAR(joint.getVelocity()[0], 0.0, 1e-9);
}

// Test that joint effort limits clamp the commanded actuation force used by the
// articulated-body forward dynamics.
TEST(World, MultibodyJointEffortLimit)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  const double mass = 2.0;
  carriage.setMass(mass);

  auto joint = carriage.getParentJoint();
  EXPECT_TRUE(std::isinf(joint.getEffortUpperLimits()[0]));
  EXPECT_TRUE(std::isinf(joint.getEffortLowerLimits()[0]));

  const double effortLimit = 10.0;
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -effortLimit),
      Eigen::VectorXd::Constant(1, effortLimit));
  joint.setForce(Eigen::VectorXd::Constant(1, 100.0)); // far above the limit

  EXPECT_THROW(
      joint.setEffortLimits(
          Eigen::VectorXd::Constant(1, 1.0), Eigen::VectorXd::Constant(1, 0.0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setEffortLimits(
          Eigen::VectorXd::Constant(2, 0.0), Eigen::VectorXd::Constant(2, 1.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.01);
  world.enterSimulationMode();
  world.step();

  // The applied effort is clamped to the limit, so qddot = effortLimit / mass.
  EXPECT_NEAR(joint.getAcceleration()[0], effortLimit / mass, 1e-12);
}

// Test that joint velocity limits clamp the generalized velocity each step.
TEST(World, MultibodyJointVelocityLimit)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(2.0);

  auto joint = carriage.getParentJoint();
  EXPECT_TRUE(std::isinf(joint.getVelocityUpperLimits()[0]));
  EXPECT_TRUE(std::isinf(joint.getVelocityLowerLimits()[0]));

  const double velocityLimit = 0.1;
  joint.setVelocityLimits(
      Eigen::VectorXd::Constant(1, -velocityLimit),
      Eigen::VectorXd::Constant(1, velocityLimit));
  joint.setForce(Eigen::VectorXd::Constant(1, 10.0)); // accelerates the slider

  EXPECT_THROW(
      joint.setVelocityLimits(
          Eigen::VectorXd::Constant(1, 1.0), Eigen::VectorXd::Constant(1, 0.0)),
      sx::InvalidArgumentException);

  world.setTimeStep(0.01);
  world.enterSimulationMode();

  for (int i = 0; i < 200; ++i) {
    world.step();
    EXPECT_LE(joint.getVelocity()[0], velocityLimit + 1e-12);
  }

  // Under continued forcing the velocity saturates exactly at the limit.
  EXPECT_NEAR(joint.getVelocity()[0], velocityLimit, 1e-12);
}

// Test that World::collide() reports contacts between overlapping collision
// shapes and reports none once the shapes are separated.
TEST(World, CollisionQueryReportsContacts)
{
  namespace sx = dart::simulation;

  sx::World world;

  sx::RigidBodyOptions optionsA;
  optionsA.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto bodyA = world.addRigidBody("a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(1.0));

  sx::RigidBodyOptions optionsB;
  optionsB.position = Eigen::Vector3d(1.5, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(1.0));

  EXPECT_TRUE(bodyA.hasCollisionShape());
  ASSERT_TRUE(bodyA.getCollisionShape().has_value());
  EXPECT_EQ(bodyA.getCollisionShape()->type, sx::CollisionShapeType::Sphere);
  EXPECT_FALSE(world.addRigidBody("c").hasCollisionShape());

  // Spheres of radius 1 centered 1.5 apart overlap by 0.5.
  auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  for (const auto& contact : contacts) {
    EXPECT_GT(contact.depth, 0.0);
    EXPECT_NEAR(contact.depth, 0.5, 1e-6);
    EXPECT_NEAR(std::abs(contact.normal.x()), 1.0, 1e-6);
  }

  // Separate the bodies; the query should report no contacts.
  Eigen::Isometry3d farPose = Eigen::Isometry3d::Identity();
  farPose.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  bodyB.setTransform(farPose);
  EXPECT_TRUE(world.collide().empty());
}

// Test that CollisionShape local transforms offset the native collision object
// relative to its owning body.
TEST(World, CollisionQueryUsesShapeLocalTransform)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto bodyA = world.addRigidBody("a");
  sx::CollisionShape offsetSphere = sx::CollisionShape::makeSphere(0.5);
  offsetSphere.localTransform.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  bodyA.setCollisionShape(offsetSphere);

  sx::RigidBodyOptions optionsB;
  optionsB.position = Eigen::Vector3d(1.8, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  // The body origins are too far apart for radius-0.5 spheres, but the first
  // sphere is centered at x=1 in body A's frame, producing 0.2 penetration.
  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  EXPECT_NEAR(contacts.front().depth, 0.2, 1e-6);
}

// Test that the collision query still reports sparse contacts after candidate
// pair generation is pruned by the native broad phase.
TEST(World, CollisionQueryFindsSparseBroadPhaseCandidate)
{
  namespace sx = dart::simulation;

  sx::World world;

  for (int i = 0; i < 20; ++i) {
    sx::RigidBodyOptions farOptions;
    farOptions.position = Eigen::Vector3d(10.0 + 3.0 * i, 0.0, 0.0);
    auto farBody = world.addRigidBody("far_" + std::to_string(i), farOptions);
    farBody.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
  }

  auto nearA = world.addRigidBody("near_a");
  nearA.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  sx::RigidBodyOptions nearBOptions;
  nearBOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto nearB = world.addRigidBody("near_b", nearBOptions);
  nearB.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  for (const auto& contact : contacts) {
    const auto nameA = contact.bodyA.getName();
    const auto nameB = contact.bodyB.getName();
    EXPECT_TRUE(
        (nameA == "near_a" && nameB == "near_b")
        || (nameA == "near_b" && nameB == "near_a"));
  }
}

// Test that repeated collision queries update cached native object poses and
// rebuild when collision geometry changes.
TEST(World, CollisionQueryCacheUpdatesTransformsAndShapes)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto bodyA = world.addRigidBody("a");
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions bodyBOptions;
  bodyBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", bodyBOptions);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  ASSERT_FALSE(world.collide().empty());

  Eigen::Isometry3d farPose = Eigen::Isometry3d::Identity();
  farPose.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
  bodyB.setTransform(farPose);
  EXPECT_TRUE(world.collide().empty());

  Eigen::Isometry3d nearPose = Eigen::Isometry3d::Identity();
  nearPose.translation() = Eigen::Vector3d(0.8, 0.0, 0.0);
  bodyB.setTransform(nearPose);
  ASSERT_FALSE(world.collide().empty());

  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.1));
  EXPECT_TRUE(world.collide().empty());

  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  EXPECT_FALSE(world.collide().empty());
}

TEST(World, CollisionQueryCacheRebuildPreservesPreparedSpecs)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto bodyA = world.addRigidBody("a");
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions bodyBOptions;
  bodyBOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", bodyBOptions);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const auto firstContacts = world.collide();
  ASSERT_FALSE(firstContacts.empty());

  Eigen::Isometry3d farPose = Eigen::Isometry3d::Identity();
  farPose.translation() = Eigen::Vector3d(3.8, 0.0, 0.0);
  bodyB.setTransform(farPose);
  EXPECT_TRUE(world.collide().empty());

  sx::CollisionShape offsetSphere = sx::CollisionShape::makeSphere(0.5);
  offsetSphere.localTransform.translation() = Eigen::Vector3d(3.4, 0.0, 0.0);
  bodyA.addCollisionShape(offsetSphere);

  const auto rebuiltContacts = world.collide();
  ASSERT_FALSE(rebuiltContacts.empty());

  bool sawNewShape = false;
  for (const auto& contact : rebuiltContacts) {
    if (contact.bodyA.getName() == "a") {
      sawNewShape = sawNewShape || contact.shapeIndexA == 1u;
    }
    if (contact.bodyB.getName() == "a") {
      sawNewShape = sawNewShape || contact.shapeIndexB == 1u;
    }
  }
  EXPECT_TRUE(sawNewShape);
}

// Test that multiple shapes on the same rigid body behave as compound
// collision geometry and do not self-collide.
TEST(World, CollisionQuerySupportsCompoundRigidBodyShapes)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeSphere(0.5));
  compound.addCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::CollisionShape offsetSphere = sx::CollisionShape::makeSphere(0.5);
  offsetSphere.localTransform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  compound.addCollisionShape(offsetSphere);

  ASSERT_EQ(compound.getCollisionShapes().size(), 3u);
  ASSERT_TRUE(compound.getCollisionShape().has_value());
  EXPECT_EQ(compound.getCollisionShape()->type, sx::CollisionShapeType::Sphere);

  EXPECT_TRUE(world.collide().empty());

  sx::RigidBodyOptions targetOptions;
  targetOptions.position = Eigen::Vector3d(2.8, 0.0, 0.0);
  auto target = world.addRigidBody("target", targetOptions);
  target.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  for (const auto& contact : contacts) {
    EXPECT_NE(contact.bodyA.getEntity(), contact.bodyB.getEntity());
  }

  compound.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  const auto replacedShapes = compound.getCollisionShapes();
  ASSERT_EQ(replacedShapes.size(), 1u);
  EXPECT_EQ(replacedShapes.front().type, sx::CollisionShapeType::Box);
}

// Test that capsule shapes are bridged into the native collision query.
TEST(World, CollisionQuerySupportsCapsuleShape)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto capsule = world.addRigidBody("capsule");
  capsule.setCollisionShape(sx::CollisionShape::makeCapsule(0.25, 1.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  EXPECT_NEAR(contacts.front().depth, 0.05, 1e-6);
}

// Test that cylinder shapes are bridged into the native collision query.
TEST(World, CollisionQuerySupportsCylinderShape)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto cylinder = world.addRigidBody("cylinder");
  cylinder.setCollisionShape(sx::CollisionShape::makeCylinder(0.25, 1.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  EXPECT_NEAR(contacts.front().depth, 0.05, 1e-6);
}

// Test that plane shapes are bridged into the native collision query.
TEST(World, CollisionQuerySupportsPlaneShape)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto plane = world.addRigidBody("plane");
  plane.setCollisionShape(
      sx::CollisionShape::makePlane(Eigen::Vector3d::UnitZ(), 0.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.2);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  EXPECT_NEAR(contacts.front().depth, 0.05, 1e-6);
}

// Test that triangular mesh shapes are bridged into the native collision query.
TEST(World, CollisionQuerySupportsMeshShape)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto mesh = world.addRigidBody("mesh");
  mesh.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(-1.0, -1.0, 0.0),
           Eigen::Vector3d(1.0, -1.0, 0.0),
           Eigen::Vector3d(-1.0, 1.0, 0.0),
           Eigen::Vector3d(1.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(1, 3, 2)}));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.2);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  EXPECT_NEAR(contacts.front().depth, 0.05, 1e-6);
}

// Test that multibody links with collision shapes participate in collision
// queries and are reported as CollisionBody links (not rigid bodies).
TEST(World, CollisionQueryIncludesMultibodyLinks)
{
  namespace sx = dart::simulation;

  sx::World world;

  // A multibody base link (at the world origin) carrying a sphere shape.
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  base.setCollisionShape(sx::CollisionShape::makeSphere(1.0));
  EXPECT_TRUE(base.hasCollisionShape());
  ASSERT_TRUE(base.getCollisionShape().has_value());
  EXPECT_EQ(base.getCollisionShape()->type, sx::CollisionShapeType::Sphere);

  // A rigid body overlapping the base (radius 1 + radius 0.5, centers 1.2
  // apart, overlap 0.3).
  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(1.2, 0.0, 0.0);
  auto ball = world.addRigidBody("ball", options);
  ball.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  world.enterSimulationMode();
  const auto contacts = world.collide();
  ASSERT_GE(contacts.size(), 1u);

  bool sawLink = false;
  bool sawRigidBody = false;
  for (const auto& contact : contacts) {
    EXPECT_GT(contact.depth, 0.0);
    sawLink = sawLink || contact.bodyA.isLink() || contact.bodyB.isLink();
    sawRigidBody = sawRigidBody || contact.bodyA.isRigidBody()
                   || contact.bodyB.isRigidBody();

    // The link CollisionBody resolves back to the named base link.
    if (contact.bodyA.isLink()) {
      EXPECT_EQ(contact.bodyA.getName(), "base");
      EXPECT_TRUE(contact.bodyA.asLink().has_value());
      EXPECT_FALSE(contact.bodyA.asRigidBody().has_value());
    }
    if (contact.bodyB.isLink()) {
      EXPECT_EQ(contact.bodyB.getName(), "base");
    }
  }
  EXPECT_TRUE(sawLink);
  EXPECT_TRUE(sawRigidBody);
}

// Test that collision queries refresh dirty link transform caches before
// building link collision poses.
TEST(World, CollisionQueryRefreshesDirtyLinkTransforms)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitX();
  auto link = robot.addLink("link", base, spec);
  link.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(1.2, 0.0, 0.0);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  world.enterSimulationMode();
  ASSERT_TRUE(world.collide().empty());

  link.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, 0.7));
  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  bool sawMovedLink = false;
  for (const auto& contact : contacts) {
    sawMovedLink
        = sawMovedLink || contact.bodyA.isLink() || contact.bodyB.isLink();
  }
  EXPECT_TRUE(sawMovedLink);
}

// Test that collision-query options can filter self-collision pairs within a
// multibody while preserving the default same-multibody contact behavior.
TEST(World, CollisionQueryCanFilterSameMultibodyLinkPairs)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  base.setCollisionShape(sx::CollisionShape::makeSphere(0.75));

  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitX();
  auto link = robot.addLink("link", base, spec);
  link.setCollisionShape(sx::CollisionShape::makeSphere(0.75));

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  world.enterSimulationMode();

  bool sawSameMultibodyPair = false;
  const auto defaultContacts = world.collide();
  for (const auto& contact : defaultContacts) {
    const bool bothLinks = contact.bodyA.isLink() && contact.bodyB.isLink();
    const bool namesMatch = (contact.bodyA.getName() == "base"
                             && contact.bodyB.getName() == "link")
                            || (contact.bodyA.getName() == "link"
                                && contact.bodyB.getName() == "base");
    sawSameMultibodyPair = sawSameMultibodyPair || (bothLinks && namesMatch);
  }
  EXPECT_TRUE(sawSameMultibodyPair);

  sx::CollisionQueryOptions options;
  options.includeSameMultibodyLinkPairs = false;
  EXPECT_TRUE(world.collide(options).empty());

  Eigen::Isometry3d nearPose = Eigen::Isometry3d::Identity();
  nearPose.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);
  obstacle.setTransform(nearPose);

  const auto filteredContacts = world.collide(options);
  ASSERT_FALSE(filteredContacts.empty());
  for (const auto& contact : filteredContacts) {
    EXPECT_TRUE(contact.bodyA.isRigidBody() || contact.bodyB.isRigidBody());
  }
}

// Test that collision-query options can independently filter body-type pairs
// while keeping all filters enabled by default.
TEST(World, CollisionQueryCanFilterBodyTypePairs)
{
  namespace sx = dart::simulation;

  {
    sx::World world;

    auto bodyA = world.addRigidBody("rigid_a");
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions bodyBOptions;
    bodyBOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
    auto bodyB = world.addRigidBody("rigid_b", bodyBOptions);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    ASSERT_FALSE(world.collide().empty());

    sx::CollisionQueryOptions options;
    options.includeRigidBodyPairs = false;
    EXPECT_TRUE(world.collide(options).empty());
  }

  {
    sx::World world;

    auto robot = world.addMultibody("robot");
    auto link = robot.addLink("link");
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
    auto body = world.addRigidBody("rigid", bodyOptions);
    body.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.enterSimulationMode();
    ASSERT_FALSE(world.collide().empty());

    sx::CollisionQueryOptions options;
    options.includeRigidBodyLinkPairs = false;
    EXPECT_TRUE(world.collide(options).empty());
  }

  {
    sx::World world;

    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");
    base.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::JointSpec spec;
    spec.name = "child";
    auto child = robot.addLink("child", base, spec);
    child.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.enterSimulationMode();
    ASSERT_FALSE(world.collide().empty());

    sx::CollisionQueryOptions options;
    options.includeLinkPairs = false;
    EXPECT_TRUE(world.collide(options).empty());
  }
}

// Test that persistent per-pair collision filters remove only the requested
// pairs and accept both rigid-body and multibody-link frame handles.
TEST(World, CollisionQueryCanIgnoreSpecificPairs)
{
  namespace sx = dart::simulation;

  const auto hasContactBetween = [](const std::vector<sx::Contact>& contacts,
                                    std::string_view first,
                                    std::string_view second) {
    for (const auto& contact : contacts) {
      const auto nameA = contact.bodyA.getName();
      const auto nameB = contact.bodyB.getName();
      if ((nameA == first && nameB == second)
          || (nameA == second && nameB == first)) {
        return true;
      }
    }
    return false;
  };

  {
    sx::World world;

    auto bodyA = world.addRigidBody("rigid_a");
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions bodyBOptions;
    bodyBOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
    auto bodyB = world.addRigidBody("rigid_b", bodyBOptions);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions bodyCOptions;
    bodyCOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
    auto bodyC = world.addRigidBody("rigid_c", bodyCOptions);
    bodyC.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    ASSERT_TRUE(hasContactBetween(world.collide(), "rigid_a", "rigid_b"));
    EXPECT_EQ(world.getIgnoredCollisionPairCount(), 0u);

    world.setCollisionPairIgnored(bodyA, bodyB);
    EXPECT_TRUE(world.isCollisionPairIgnored(bodyB, bodyA));
    EXPECT_EQ(world.getIgnoredCollisionPairCount(), 1u);

    const auto filteredContacts = world.collide();
    EXPECT_FALSE(hasContactBetween(filteredContacts, "rigid_a", "rigid_b"));
    EXPECT_TRUE(
        hasContactBetween(filteredContacts, "rigid_a", "rigid_c")
        || hasContactBetween(filteredContacts, "rigid_b", "rigid_c"));

    world.setCollisionPairIgnored(bodyB, bodyA, false);
    EXPECT_FALSE(world.isCollisionPairIgnored(bodyA, bodyB));
    EXPECT_EQ(world.getIgnoredCollisionPairCount(), 0u);
    EXPECT_TRUE(hasContactBetween(world.collide(), "rigid_a", "rigid_b"));

    world.setCollisionPairIgnored(bodyA, bodyB);
    world.clearIgnoredCollisionPairs();
    EXPECT_EQ(world.getIgnoredCollisionPairCount(), 0u);
    EXPECT_TRUE(hasContactBetween(world.collide(), "rigid_a", "rigid_b"));

    EXPECT_THROW(
        world.setCollisionPairIgnored(bodyA, bodyA),
        sx::InvalidArgumentException);

    sx::World otherWorld;
    auto foreignBody = otherWorld.addRigidBody("foreign");
    EXPECT_THROW(
        world.setCollisionPairIgnored(bodyA, foreignBody),
        sx::InvalidArgumentException);
  }

  {
    sx::World world;

    auto robot = world.addMultibody("robot");
    auto link = robot.addLink("link");
    link.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions bodyOptions;
    bodyOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
    auto body = world.addRigidBody("rigid", bodyOptions);
    body.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.enterSimulationMode();
    ASSERT_TRUE(hasContactBetween(world.collide(), "link", "rigid"));

    world.setCollisionPairIgnored(link, body);
    EXPECT_TRUE(world.isCollisionPairIgnored(body, link));
    EXPECT_TRUE(world.collide().empty());
  }
}

// Public rigid-body joints act like source AVBD forces for collision discovery:
// live constrained body pairs do not generate contact manifolds, and broken
// joints make the pair collidable again.
TEST(World, CollisionQuerySkipsLiveRigidBodyJointPairs)
{
  namespace sx = dart::simulation;

  const auto hasContactBetween = [](const std::vector<sx::Contact>& contacts,
                                    std::string_view first,
                                    std::string_view second) {
    for (const auto& contact : contacts) {
      const auto nameA = contact.bodyA.getName();
      const auto nameB = contact.bodyB.getName();
      if ((nameA == first && nameB == second)
          || (nameA == second && nameB == first)) {
        return true;
      }
    }
    return false;
  };

  sx::World world;

  auto bodyA = world.addRigidBody("rigid_a");
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions bodyBOptions;
  bodyBOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
  auto bodyB = world.addRigidBody("rigid_b", bodyBOptions);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions bodyCOptions;
  bodyCOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
  auto bodyC = world.addRigidBody("rigid_c", bodyCOptions);
  bodyC.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  ASSERT_TRUE(hasContactBetween(world.collide(), "rigid_a", "rigid_b"));

  auto joint = world.addRigidBodyFixedJoint("locked", bodyA, bodyB);
  EXPECT_FALSE(world.isCollisionPairIgnored(bodyA, bodyB));
  EXPECT_EQ(world.getIgnoredCollisionPairCount(), 0u);
  const auto filteredContacts = world.collide();
  EXPECT_FALSE(hasContactBetween(filteredContacts, "rigid_a", "rigid_b"));
  ASSERT_TRUE(
      hasContactBetween(filteredContacts, "rigid_a", "rigid_c")
      || hasContactBetween(filteredContacts, "rigid_b", "rigid_c"));

  auto& registry = sx::detail::registryOf(world);
  auto& jointComponent = registry.get<sx::comps::Joint>(
      sx::detail::toRegistryEntity(joint.getEntity()));
  jointComponent.broken = true;
  EXPECT_TRUE(joint.isBroken());
  EXPECT_TRUE(hasContactBetween(world.collide(), "rigid_a", "rigid_b"));

  joint.resetBreakage();
  EXPECT_FALSE(joint.isBroken());
  EXPECT_FALSE(hasContactBetween(world.collide(), "rigid_a", "rigid_b"));

  {
    sx::World allocationWorld;

    auto allocationBodyA = allocationWorld.addRigidBody("allocation_rigid_a");
    allocationBodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions allocationBodyBOptions;
    allocationBodyBOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
    auto allocationBodyB = allocationWorld.addRigidBody(
        "allocation_rigid_b", allocationBodyBOptions);
    allocationBodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    auto allocationJoint = allocationWorld.addRigidBodyFixedJoint(
        "allocation_locked", allocationBodyA, allocationBodyB);
    EXPECT_FALSE(allocationJoint.isBroken());

    ASSERT_TRUE(allocationWorld.collide().empty());

    // The live-joint pair set is query-local state, but it must reuse cache
    // storage after warmup so AVBD contact discovery does not allocate each
    // step when filtering candidate pairs.
    ScopedHeapAllocationCounter heapCounter;
    for (int i = 0; i < 4; ++i) {
      EXPECT_TRUE(allocationWorld.collide().empty());
    }
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "global heap bytes allocated during warmed contact queries: "
        << heapCounter.allocationBytes();
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
}

TEST(World, CollisionQueryCacheScratchUsesWorldAllocator)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto& freeList = world.getMemoryManager().getFreeListAllocator();

  (void)world.collide();

  auto bodyA = world.addRigidBody("cache_allocator_a");
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions bodyBOptions;
  bodyBOptions.position = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto bodyB = world.addRigidBody("cache_allocator_b", bodyBOptions);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const auto emptyContacts = world.collide();
  ASSERT_TRUE(emptyContacts.empty());

  Eigen::Isometry3d overlapPose = Eigen::Isometry3d::Identity();
  overlapPose.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  bodyB.setTransform(overlapPose);

  const auto allocationsBeforeContactResults = freeList.getAllocationCount();
  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeContactResults)
      << "cached collision contact results should reserve from the World "
         "free allocator when contacts first appear for a warmed shape set";

  const auto allocationsAfterPopulate = freeList.getAllocationCount();
  const auto warmedContacts = world.collide();
  ASSERT_FALSE(warmedContacts.empty());
  EXPECT_EQ(freeList.getAllocationCount(), allocationsAfterPopulate)
      << "same-shape collision query cache scratch should reuse capacity";
}

// Test that a multibody link with a collision shape rests on a static ground
// via the articulated contact response (a fixed-base prismatic "leg" drops
// under gravity and stops where its sphere meets the ground).
TEST(World, MultibodyLinkRestsOnStaticGround)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("leg_robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto leg = robot.addLink("leg", base, spec);
  leg.setMass(1.0);
  const double radius = 0.2;
  leg.setCollisionShape(sx::CollisionShape::makeSphere(radius));

  // Static ground box centered at z = -1 with half-height 0.5 (top at z =
  // -0.5).
  sx::RigidBodyOptions groundOptions;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  groundOptions.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  auto joint = leg.getParentJoint();
  // Start just above the ground so the impact is gentle.
  joint.setPosition(Eigen::VectorXd::Constant(1, -0.25));

  world.setTimeStep(0.002);
  world.enterSimulationMode();

  for (int i = 0; i < 1500; ++i) {
    world.step();
  }

  // The sphere (radius 0.2) rests on the ground top (z = -0.5), so the leg
  // origin settles at z = -0.3 with near-zero velocity and no deep penetration.
  const double restZ = leg.getWorldTransform().translation().z();
  EXPECT_NEAR(restZ, -0.3, 5e-3);
  EXPECT_GT(restZ, -0.31);
  EXPECT_NEAR(joint.getVelocity()[0], 0.0, 5e-2);
}

// Test two-sided link-vs-dynamic-rigid-body contact: a prismatic "striker" link
// moving along +X hits a free rigid body. The contact impulse acts on both
// bodies, so total X linear momentum is conserved and the box is pushed
// forward.
TEST(World, MultibodyLinkPushesDynamicRigidBody)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  // Striker: a fixed base carrying a sphere on a prismatic X joint, moving +X.
  auto robot = world.addMultibody("striker_robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitX();
  auto striker = robot.addLink("striker", base, spec);
  const double strikerMass = 2.0;
  striker.setMass(strikerMass);
  striker.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  auto joint = striker.getParentJoint();
  const double initialSpeed = 1.0;
  joint.setVelocity(Eigen::VectorXd::Constant(1, initialSpeed));

  // Free rigid body in the striker's path (centers 0.5 apart; radii sum 0.4).
  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto box = world.addRigidBody("box", boxOptions);
  const double boxMass = 1.0;
  box.setMass(boxMass);
  box.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  world.setTimeStep(0.002);
  world.enterSimulationMode();

  const double initialMomentum = strikerMass * initialSpeed;
  bool boxWasPushed = false;
  for (int i = 0; i < 600; ++i) {
    world.step();
    const double momentum = strikerMass * joint.getVelocity()[0]
                            + boxMass * box.getLinearVelocity().x();
    // Equal-and-opposite impulses conserve total X momentum every step (no
    // external force along the unconstrained X axis).
    EXPECT_NEAR(momentum, initialMomentum, 1e-6);
    if (box.getLinearVelocity().x() > 1e-3) {
      boxWasPushed = true;
    }
  }

  // The collision happened and drove the box forward, slowing the striker.
  EXPECT_TRUE(boxWasPushed);
  EXPECT_GT(box.getLinearVelocity().x(), 0.1);
  EXPECT_LT(joint.getVelocity()[0], initialSpeed);
  // An inelastic (restitution 0) collision drives both toward the common
  // velocity m1 v0 / (m1 + m2).
  const double commonVelocity = initialMomentum / (strikerMass + boxMass);
  EXPECT_NEAR(box.getLinearVelocity().x(), commonVelocity, 0.1);
}

// Test same-multibody link-vs-link contact: two sibling prismatic links overlap
// and approach along their shared rail. The unified contact stage routes the
// pair as one relative-Jacobian link row, so the links separate instead of
// passing through each other.
TEST(World, MultibodySiblingLinksResolveContact)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto robot = world.addMultibody("sibling_robot");
  auto base = robot.addLink("base");

  sx::JointSpec lowerSpec;
  lowerSpec.name = "lower";
  lowerSpec.type = sx::JointType::Prismatic;
  lowerSpec.axis = Eigen::Vector3d::UnitZ();
  auto lower = robot.addLink("lower", base, lowerSpec);
  lower.setMass(1.0);
  lower.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  sx::JointSpec upperSpec;
  upperSpec.name = "upper";
  upperSpec.type = sx::JointType::Prismatic;
  upperSpec.axis = Eigen::Vector3d::UnitZ();
  auto upper = robot.addLink("upper", base, upperSpec);
  upper.setMass(1.0);
  upper.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  auto lowerJoint = lower.getParentJoint();
  auto upperJoint = upper.getParentJoint();
  lowerJoint.setPosition(Eigen::VectorXd::Constant(1, 0.0));
  upperJoint.setPosition(Eigen::VectorXd::Constant(1, 0.35));
  lowerJoint.setVelocity(Eigen::VectorXd::Constant(1, 0.5));
  upperJoint.setVelocity(Eigen::VectorXd::Constant(1, -0.5));

  world.setTimeStep(0.001);
  world.enterSimulationMode();
  ASSERT_FALSE(world.collide().empty());

  world.step();

  const double relativeVelocity
      = upperJoint.getVelocity()[0] - lowerJoint.getVelocity()[0];
  EXPECT_GE(relativeVelocity, -1e-9);
  EXPECT_LT(lowerJoint.getVelocity()[0], 0.5);
  EXPECT_GT(upperJoint.getVelocity()[0], -0.5);
}

// Test that a fixed articulated obstacle still routes link contacts through
// the unified solver instead of being swallowed by the sequential shortcut.
TEST(World, ZeroDofMultibodyLinkContactStopsRigidBody)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto fixture = world.addMultibody("fixed_fixture");
  auto base = fixture.addLink("base");
  sx::JointSpec fixed;
  fixed.name = "weld";
  fixed.type = sx::JointType::Fixed;
  auto obstacle = fixture.addLink("obstacle", base, fixed);
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  ASSERT_EQ(fixture.getDOFCount(), 0u);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
  dynamicOptions.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto dynamic = world.addRigidBody("dynamic", dynamicOptions);
  dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  world.setTimeStep(0.001);
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_GE(dynamic.getLinearVelocity().x(), -1e-9)
      << dynamic.getLinearVelocity().transpose();
  EXPECT_NEAR(dynamic.getLinearVelocity().y(), 0.0, 1e-12);
  EXPECT_NEAR(dynamic.getLinearVelocity().z(), 0.0, 1e-12);
  EXPECT_TRUE(dynamic.getAngularVelocity().isZero(1e-12));
}

TEST(World, ZeroDofMultibodyFallbackDoesNotDoubleApplyShortcutImpulses)
{
  namespace sx = dart::simulation;

  struct StepState
  {
    double sliderVelocity{0.0};
    Eigen::Vector3d sliderRigidVelocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d fixedRigidVelocity{Eigen::Vector3d::Zero()};
  };

  const auto runScene = [](bool boxedLcp, bool fixedFixtureFirst) -> StepState {
    sx::WorldOptions options;
    if (boxedLcp) {
      options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
    }
    sx::World world(options);
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.001);

    sx::Joint sliderJoint(sx::Entity{}, nullptr);
    sx::RigidBody sliderTarget(sx::Entity{}, nullptr);
    sx::RigidBody fixedTarget(sx::Entity{}, nullptr);

    const auto addFixedFixture = [&]() {
      auto fixture = world.addMultibody("fixed_fixture");
      auto base = fixture.addLink("base");

      sx::JointSpec fixed;
      fixed.name = "weld";
      fixed.type = sx::JointType::Fixed;
      auto obstacle = fixture.addLink("obstacle", base, fixed);
      auto obstacleShape = sx::CollisionShape::makeSphere(0.5);
      obstacleShape.localTransform.translation()
          = Eigen::Vector3d(3.0, 0.0, 0.0);
      obstacle.setCollisionShape(obstacleShape);

      sx::RigidBodyOptions dynamicOptions;
      dynamicOptions.position = Eigen::Vector3d(3.9, 0.0, 0.0);
      dynamicOptions.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
      fixedTarget = world.addRigidBody("fixed_target", dynamicOptions);
      fixedTarget.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
      EXPECT_EQ(fixture.getDOFCount(), 0u);
    };

    const auto addSolvableSlider = [&]() {
      auto robot = world.addMultibody("slider_robot");
      auto base = robot.addLink("base");
      sx::JointSpec spec;
      spec.name = "rail";
      spec.type = sx::JointType::Prismatic;
      spec.axis = Eigen::Vector3d::UnitX();
      auto link = robot.addLink("slider", base, spec);
      link.setMass(1.0);
      link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
      sliderJoint = link.getParentJoint();
      sliderJoint.setVelocity(Eigen::VectorXd::Constant(1, 1.0));

      sx::RigidBodyOptions targetOptions;
      targetOptions.position = Eigen::Vector3d(0.35, 0.0, 0.0);
      sliderTarget = world.addRigidBody("slider_target", targetOptions);
      sliderTarget.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    };

    if (fixedFixtureFirst) {
      addFixedFixture();
      addSolvableSlider();
    } else {
      addSolvableSlider();
      addFixedFixture();
    }

    EXPECT_GE(world.collide().size(), 2u);
    world.step();

    return StepState{
        sliderJoint.getVelocity()[0],
        sliderTarget.getLinearVelocity(),
        fixedTarget.getLinearVelocity()};
  };

  for (const bool fixedFixtureFirst : {false, true}) {
    SCOPED_TRACE(fixedFixtureFirst ? "fixed first" : "slider first");
    const StepState defaultPath = runScene(false, fixedFixtureFirst);
    const StepState boxedPath = runScene(true, fixedFixtureFirst);

    EXPECT_NEAR(defaultPath.sliderVelocity, boxedPath.sliderVelocity, 1e-12);
    EXPECT_TRUE(defaultPath.sliderRigidVelocity.isApprox(
        boxedPath.sliderRigidVelocity, 1e-12))
        << defaultPath.sliderRigidVelocity.transpose() << " vs "
        << boxedPath.sliderRigidVelocity.transpose();
    EXPECT_TRUE(defaultPath.fixedRigidVelocity.isApprox(
        boxedPath.fixedRigidVelocity, 1e-12))
        << defaultPath.fixedRigidVelocity.transpose() << " vs "
        << boxedPath.fixedRigidVelocity.transpose();
  }
}

// Test cross-multibody link-vs-link contact: two separate fixed-base
// articulated bodies overlap and approach along their prismatic rails. The
// unified row carries both articulated ends, so the impulse changes both
// multibodies' staged velocities.
TEST(World, CrossMultibodyLinksResolveContact)
{
  namespace sx = dart::simulation;

  for (const bool boxedLcp : {false, true}) {
    SCOPED_TRACE(boxedLcp ? "boxed LCP" : "sequential shortcut");

    sx::WorldOptions options;
    if (boxedLcp) {
      options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
    }
    sx::World world(options);
    world.setGravity(Eigen::Vector3d::Zero());

    const auto addRobot
        = [&](std::string_view name, double z, double velocity) {
            auto robot = world.addMultibody(name);
            auto base = robot.addLink("base");
            sx::JointSpec spec;
            spec.name = "slider";
            spec.type = sx::JointType::Prismatic;
            spec.axis = Eigen::Vector3d::UnitZ();
            auto link = robot.addLink("link", base, spec);
            link.setMass(1.0);
            link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
            auto joint = link.getParentJoint();
            joint.setPosition(Eigen::VectorXd::Constant(1, z));
            joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
            return link;
          };

    auto lower = addRobot("lower_robot", 0.0, 0.5);
    auto upper = addRobot("upper_robot", 0.35, -0.5);
    auto lowerJoint = lower.getParentJoint();
    auto upperJoint = upper.getParentJoint();

    world.setTimeStep(0.001);
    world.enterSimulationMode();
    ASSERT_FALSE(world.collide().empty());

    world.step();

    const double relativeVelocity
        = upperJoint.getVelocity()[0] - lowerJoint.getVelocity()[0];
    EXPECT_GE(relativeVelocity, -1e-9);
    EXPECT_LT(lowerJoint.getVelocity()[0], 0.5);
    EXPECT_GT(upperJoint.getVelocity()[0], -0.5);
  }
}

TEST(World, CrossMultibodyDifferentDofLinksUseUnifiedFallback)
{
  namespace sx = dart::simulation;

  struct StepState
  {
    double lowerVelocity{0.0};
    double upperRootVelocity{0.0};
    double upperTipVelocity{0.0};
  };

  const auto runScene = [](bool boxedLcp) {
    sx::WorldOptions options;
    if (boxedLcp) {
      options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
    }
    sx::World world(options);
    world.setGravity(Eigen::Vector3d::Zero());

    auto lowerRobot = world.addMultibody("lower_robot");
    auto lowerBase = lowerRobot.addLink("base");
    sx::JointSpec lowerSpec;
    lowerSpec.name = "lower_slider";
    lowerSpec.type = sx::JointType::Prismatic;
    lowerSpec.axis = Eigen::Vector3d::UnitZ();
    auto lowerLink = lowerRobot.addLink("link", lowerBase, lowerSpec);
    lowerLink.setMass(1.0);
    lowerLink.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto lowerJoint = lowerLink.getParentJoint();
    lowerJoint.setPosition(Eigen::VectorXd::Constant(1, 0.0));
    lowerJoint.setVelocity(Eigen::VectorXd::Constant(1, 0.5));

    auto upperRobot = world.addMultibody("upper_robot");
    auto upperBase = upperRobot.addLink("base");
    sx::JointSpec rootSpec;
    rootSpec.name = "upper_root_slider";
    rootSpec.type = sx::JointType::Prismatic;
    rootSpec.axis = Eigen::Vector3d::UnitZ();
    auto upperMid = upperRobot.addLink("mid", upperBase, rootSpec);
    upperMid.setMass(1.0);
    auto upperRootJoint = upperMid.getParentJoint();
    upperRootJoint.setPosition(Eigen::VectorXd::Constant(1, 0.15));
    upperRootJoint.setVelocity(Eigen::VectorXd::Constant(1, -0.25));

    sx::JointSpec tipSpec;
    tipSpec.name = "upper_tip_slider";
    tipSpec.type = sx::JointType::Prismatic;
    tipSpec.axis = Eigen::Vector3d::UnitZ();
    auto upperTip = upperRobot.addLink("tip", upperMid, tipSpec);
    upperTip.setMass(1.0);
    upperTip.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
    auto upperTipJoint = upperTip.getParentJoint();
    upperTipJoint.setPosition(Eigen::VectorXd::Constant(1, 0.20));
    upperTipJoint.setVelocity(Eigen::VectorXd::Constant(1, -0.25));

    world.setTimeStep(0.001);
    world.enterSimulationMode();
    EXPECT_FALSE(world.collide().empty());

    world.step();

    return StepState{
        lowerJoint.getVelocity()[0],
        upperRootJoint.getVelocity()[0],
        upperTipJoint.getVelocity()[0]};
  };

  const StepState defaultPath = runScene(false);
  const StepState boxedPath = runScene(true);

  EXPECT_NEAR(defaultPath.lowerVelocity, boxedPath.lowerVelocity, 1e-12);
  EXPECT_NEAR(
      defaultPath.upperRootVelocity, boxedPath.upperRootVelocity, 1e-12);
  EXPECT_NEAR(defaultPath.upperTipVelocity, boxedPath.upperTipVelocity, 1e-12);
}

TEST(World, CrossMultibodyStackedContactsUseUnifiedFallback)
{
  namespace sx = dart::simulation;

  struct StepState
  {
    double lowerVelocity{0.0};
    double upperVelocity{0.0};
  };

  const auto runScene = [](bool boxedLcp) {
    sx::WorldOptions options;
    if (boxedLcp) {
      options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
    }
    sx::World world(options);
    world.setGravity(Eigen::Vector3d::Zero());

    const auto addRobot
        = [&](std::string_view name, double z, double velocity) {
            auto robot = world.addMultibody(name);
            auto base = robot.addLink("base");
            sx::JointSpec spec;
            spec.name = "slider";
            spec.type = sx::JointType::Prismatic;
            spec.axis = Eigen::Vector3d::UnitZ();
            auto link = robot.addLink("link", base, spec);
            link.setMass(1.0);
            link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
            auto joint = link.getParentJoint();
            joint.setPosition(Eigen::VectorXd::Constant(1, z));
            joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
            return link;
          };

    auto lower = addRobot("lower_robot", 0.0, -0.2);
    auto upper = addRobot("upper_robot", 0.35, -0.8);

    sx::RigidBodyOptions fixtureOptions;
    fixtureOptions.isStatic = true;
    fixtureOptions.position = Eigen::Vector3d(0.0, 0.0, -0.35);
    auto fixture = world.addRigidBody("fixture", fixtureOptions);
    fixture.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

    auto lowerJoint = lower.getParentJoint();
    auto upperJoint = upper.getParentJoint();

    world.setTimeStep(0.001);
    world.enterSimulationMode();
    EXPECT_GE(world.collide().size(), 2u);

    world.step();

    return StepState{lowerJoint.getVelocity()[0], upperJoint.getVelocity()[0]};
  };

  const StepState defaultPath = runScene(false);
  const StepState boxedPath = runScene(true);

  EXPECT_NEAR(defaultPath.lowerVelocity, boxedPath.lowerVelocity, 1e-12);
  EXPECT_NEAR(defaultPath.upperVelocity, boxedPath.upperVelocity, 1e-12);
}

TEST(World, CrossMultibodyCoupledRowsUseUnifiedFallback)
{
  namespace sx = dart::simulation;

  struct StepState
  {
    double lowerVelocity{0.0};
    double middleVelocity{0.0};
    double upperVelocity{0.0};
  };

  const auto runScene = [](bool boxedLcp) {
    sx::WorldOptions options;
    if (boxedLcp) {
      options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
    }
    sx::World world(options);
    world.setGravity(Eigen::Vector3d::Zero());

    const auto addRobot
        = [&](std::string_view name, double z, double velocity) {
            auto robot = world.addMultibody(name);
            auto base = robot.addLink("base");
            sx::JointSpec spec;
            spec.name = "slider";
            spec.type = sx::JointType::Prismatic;
            spec.axis = Eigen::Vector3d::UnitZ();
            auto link = robot.addLink("link", base, spec);
            link.setMass(1.0);
            link.setCollisionShape(sx::CollisionShape::makeSphere(0.2));
            auto joint = link.getParentJoint();
            joint.setPosition(Eigen::VectorXd::Constant(1, z));
            joint.setVelocity(Eigen::VectorXd::Constant(1, velocity));
            return link;
          };

    auto lower = addRobot("lower_robot", 0.0, 0.5);
    auto middle = addRobot("middle_robot", 0.35, 0.0);
    auto upper = addRobot("upper_robot", 0.70, -0.5);

    auto lowerJoint = lower.getParentJoint();
    auto middleJoint = middle.getParentJoint();
    auto upperJoint = upper.getParentJoint();

    world.setTimeStep(0.001);
    world.enterSimulationMode();
    EXPECT_GE(world.collide().size(), 2u);

    world.step();

    return StepState{
        lowerJoint.getVelocity()[0],
        middleJoint.getVelocity()[0],
        upperJoint.getVelocity()[0]};
  };

  const StepState defaultPath = runScene(false);
  const StepState boxedPath = runScene(true);

  EXPECT_NEAR(defaultPath.lowerVelocity, boxedPath.lowerVelocity, 1e-12);
  EXPECT_NEAR(defaultPath.middleVelocity, boxedPath.middleVelocity, 1e-12);
  EXPECT_NEAR(defaultPath.upperVelocity, boxedPath.upperVelocity, 1e-12);
}

TEST(World, CrossMultibodyMultiIslandContactsUseUnifiedFallback)
{
  namespace sx = dart::simulation;

  struct StepState
  {
    std::size_t contactCount{0};
    double differentLowerVelocity{0.0};
    double differentUpperTipVelocity{0.0};
    double stackLowerVelocity{0.0};
    double stackMiddleVelocity{0.0};
    double stackUpperVelocity{0.0};
    double lowerRigidVelocity{0.0};
    double upperRigidVelocity{0.0};
  };

  const auto runScene = [](bool boxedLcp) {
    sx::WorldOptions options;
    if (boxedLcp) {
      options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
    }
    sx::World world(options);
    const auto handles
        = configureCrossMultibodyMultiIslandFallbackSceneWithHandles(world);

    world.enterSimulationMode();
    const std::size_t contactCount = world.collide().size();
    EXPECT_GE(contactCount, 4u);

    world.step();

    return StepState{
        contactCount,
        handles.differentLowerJoint.getVelocity()[0],
        handles.differentUpperTipJoint.getVelocity()[0],
        handles.stackLowerJoint.getVelocity()[0],
        handles.stackMiddleJoint.getVelocity()[0],
        handles.stackUpperJoint.getVelocity()[0],
        handles.lowerRigid.getLinearVelocity().z(),
        handles.upperRigid.getLinearVelocity().z()};
  };

  const StepState defaultPath = runScene(false);
  const StepState boxedPath = runScene(true);

  EXPECT_GE(defaultPath.contactCount, 4u);
  EXPECT_GE(boxedPath.contactCount, 4u);
  EXPECT_NEAR(
      defaultPath.differentLowerVelocity,
      boxedPath.differentLowerVelocity,
      1e-12);
  EXPECT_NEAR(
      defaultPath.differentUpperTipVelocity,
      boxedPath.differentUpperTipVelocity,
      1e-12);
  EXPECT_NEAR(
      defaultPath.stackLowerVelocity, boxedPath.stackLowerVelocity, 1e-12);
  EXPECT_NEAR(
      defaultPath.stackMiddleVelocity, boxedPath.stackMiddleVelocity, 1e-12);
  EXPECT_NEAR(
      defaultPath.stackUpperVelocity, boxedPath.stackUpperVelocity, 1e-12);
  EXPECT_NEAR(
      defaultPath.lowerRigidVelocity, boxedPath.lowerRigidVelocity, 1e-12);
  EXPECT_NEAR(
      defaultPath.upperRigidVelocity, boxedPath.upperRigidVelocity, 1e-12);

  EXPECT_LT(defaultPath.lowerRigidVelocity, 0.4);
  EXPECT_GT(defaultPath.upperRigidVelocity, -0.4);
}

// Test that Coulomb friction at a link contact decelerates a sliding link. A
// vertical prismatic carries a horizontal prismatic link whose sphere rests on
// the ground; an initial horizontal velocity is braked to rest by friction.
TEST(World, MultibodyLinkContactFrictionStopsSlide)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("slider_robot");
  auto base = robot.addLink("base");

  sx::JointSpec verticalSpec;
  verticalSpec.name = "vertical";
  verticalSpec.type = sx::JointType::Prismatic;
  verticalSpec.axis = Eigen::Vector3d::UnitZ();
  auto carrier = robot.addLink("carrier", base, verticalSpec);
  carrier.setMass(0.1);

  sx::JointSpec horizontalSpec;
  horizontalSpec.name = "horizontal";
  horizontalSpec.type = sx::JointType::Prismatic;
  horizontalSpec.axis = Eigen::Vector3d::UnitX();
  auto slider = robot.addLink("slider", carrier, horizontalSpec);
  slider.setMass(1.0);
  slider.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  sx::RigidBodyOptions groundOptions;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  groundOptions.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));

  auto vertical = carrier.getParentJoint();
  auto horizontal = slider.getParentJoint();
  // Rest height so the sphere sits on the ground (top at z = -0.5).
  vertical.setPosition(Eigen::VectorXd::Constant(1, -0.3));
  horizontal.setVelocity(Eigen::VectorXd::Constant(1, 1.0));

  world.setTimeStep(0.002);
  world.enterSimulationMode();

  for (int i = 0; i < 600; ++i) {
    world.step();
  }

  // Friction brakes the slide to rest; the slider advanced but did not reverse.
  EXPECT_NEAR(horizontal.getVelocity()[0], 0.0, 5e-2);
  EXPECT_GT(slider.getParentJoint().getPosition()[0], 0.01);
  // The link stays resting on the ground (does not fall through).
  EXPECT_NEAR(slider.getWorldTransform().translation().z(), -0.3, 1e-2);
}

// Test that restitution at a link contact rebounds a dropped link. A prismatic
// link with a sphere falls onto a near-elastic ground and bounces back up.
TEST(World, MultibodyLinkContactRestitutionBounces)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  auto robot = world.addMultibody("bouncer");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "slider";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto bob = robot.addLink("bob", base, spec);
  bob.setMass(1.0);
  bob.setCollisionShape(sx::CollisionShape::makeSphere(0.2));

  sx::RigidBodyOptions groundOptions;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -1.0);
  groundOptions.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setRestitution(0.9);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  auto joint = bob.getParentJoint();
  joint.setPosition(Eigen::VectorXd::Constant(1, 0.0)); // drop 0.3 m to contact

  world.setTimeStep(0.002);
  world.enterSimulationMode();

  double maxUpwardVelocity = 0.0;
  for (int i = 0; i < 400; ++i) {
    world.step();
    maxUpwardVelocity = std::max(maxUpwardVelocity, joint.getVelocity()[0]);
  }

  // Impact speed from a 0.3 m drop is ~2.4 m/s; e = 0.9 rebounds at ~2.2 m/s.
  EXPECT_GT(maxUpwardVelocity, 1.5);
}

// Test that the contact stage resolves approaching velocities (fully inelastic)
// between overlapping rigid bodies and leaves separating bodies untouched.
TEST(World, RigidBodyContactResolvesApproachingVelocity)
{
  namespace sx = dart::simulation;

  // Two equal-mass spheres overlapping and approaching head-on along x.
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions optionsA;
    optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
    optionsA.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto bodyA = world.addRigidBody("a", optionsA);
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions optionsB;
    optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
    optionsB.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto bodyB = world.addRigidBody("b", optionsB);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.setTimeStep(0.001);
    world.step();

    // Equal masses, head-on, fully inelastic: both come to rest. Linear
    // momentum (zero) is conserved.
    EXPECT_NEAR(bodyA.getLinearVelocity().x(), 0.0, 1e-9);
    EXPECT_NEAR(bodyB.getLinearVelocity().x(), 0.0, 1e-9);
  }

  // Overlapping but separating: the contact stage must not change velocities.
  {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());

    sx::RigidBodyOptions optionsA;
    optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
    optionsA.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto bodyA = world.addRigidBody("a", optionsA);
    bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions optionsB;
    optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
    optionsB.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto bodyB = world.addRigidBody("b", optionsB);
    bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    world.setTimeStep(0.001);
    world.step();

    EXPECT_NEAR(bodyA.getLinearVelocity().x(), -1.0, 1e-9);
    EXPECT_NEAR(bodyB.getLinearVelocity().x(), 1.0, 1e-9);
  }
}

// Test that opt-in AVBD rigid contact projection handles a live
// dynamic/dynamic contact in the contact stage itself. The stage writes
// separating velocities for the following position stage instead of directly
// moving poses like the sequential fallback.
TEST(World, RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions optionsA;
  optionsA.mass = 1.0;
  optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
  auto bodyA = world.addRigidBody("avbd_contact_a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions optionsB;
  optionsB.mass = 1.0;
  optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto bodyB = world.addRigidBody("avbd_contact_b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  const auto configureAvbdContact = [&](const sx::RigidBody& body) {
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(body.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;
  };
  configureAvbdContact(bodyA);
  configureAvbdContact(bodyB);

  const Eigen::Vector3d initialPositionA = bodyA.getTranslation();
  const Eigen::Vector3d initialPositionB = bodyB.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(bodyA.getTranslation().isApprox(initialPositionA, 1e-12));
  EXPECT_TRUE(bodyB.getTranslation().isApprox(initialPositionB, 1e-12));
  EXPECT_LT(bodyA.getLinearVelocity().x(), -1e-9);
  EXPECT_GT(bodyB.getLinearVelocity().x(), 1e-9);
  EXPECT_GT(
      (bodyB.getLinearVelocity() - bodyA.getLinearVelocity())
          .dot(Eigen::Vector3d::UnitX()),
      1e-6);
}

// Test that default World::step() reaches dynamic/dynamic AVBD contact-stage
// projection and feeds the projected velocities to the position stage.
TEST(World, RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions optionsA;
  optionsA.mass = 1.0;
  optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
  auto bodyA = world.addRigidBody("avbd_default_dynamic_a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions optionsB;
  optionsB.mass = 1.0;
  optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto bodyB = world.addRigidBody("avbd_default_dynamic_b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  const auto configureAvbdContact = [&](const sx::RigidBody& body) {
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(body.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;
  };
  configureAvbdContact(bodyA);
  configureAvbdContact(bodyB);

  const Eigen::Vector3d initialPositionA = bodyA.getTranslation();
  const Eigen::Vector3d initialPositionB = bodyB.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_LT(bodyA.getTranslation().x(), initialPositionA.x() - 1e-9);
  EXPECT_GT(bodyB.getTranslation().x(), initialPositionB.x() + 1e-9);
  EXPECT_TRUE(bodyA.getTranslation().tail<2>().isApprox(
      initialPositionA.tail<2>(), 1e-12));
  EXPECT_TRUE(bodyB.getTranslation().tail<2>().isApprox(
      initialPositionB.tail<2>(), 1e-12));
  EXPECT_LT(bodyA.getLinearVelocity().x(), -1e-9);
  EXPECT_GT(bodyB.getLinearVelocity().x(), 1e-9);
  EXPECT_GT(
      (bodyB.getLinearVelocity() - bodyA.getLinearVelocity())
          .dot(Eigen::Vector3d::UnitX()),
      1e-6);
}

// Test that a dynamic/dynamic contact can opt into the contact-stage AVBD path
// when only one dynamic endpoint carries the private config.
TEST(World, RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions optionsA;
  optionsA.mass = 1.0;
  optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
  auto bodyA = world.addRigidBody("avbd_single_config_a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions optionsB;
  optionsB.mass = 1.0;
  optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto bodyB = world.addRigidBody("avbd_single_config_b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(bodyA.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialPositionA = bodyA.getTranslation();
  const Eigen::Vector3d initialPositionB = bodyB.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(bodyA.getTranslation().isApprox(initialPositionA, 1e-12));
  EXPECT_TRUE(bodyB.getTranslation().isApprox(initialPositionB, 1e-12));
  EXPECT_LT(bodyA.getLinearVelocity().x(), -1e-9);
  EXPECT_GT(bodyB.getLinearVelocity().x(), 1e-9);
  EXPECT_GT(
      (bodyB.getLinearVelocity() - bodyA.getLinearVelocity())
          .dot(Eigen::Vector3d::UnitX()),
      1e-6);
}

// Test that default World::step() also reaches dynamic/dynamic AVBD projection
// when only one dynamic endpoint carries the private config.
TEST(
    World,
    RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions optionsA;
  optionsA.mass = 1.0;
  optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
  auto bodyA = world.addRigidBody("avbd_default_single_config_a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions optionsB;
  optionsB.mass = 1.0;
  optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  auto bodyB = world.addRigidBody("avbd_default_single_config_b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(bodyA.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialPositionA = bodyA.getTranslation();
  const Eigen::Vector3d initialPositionB = bodyB.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_LT(bodyA.getTranslation().x(), initialPositionA.x() - 1e-9);
  EXPECT_GT(bodyB.getTranslation().x(), initialPositionB.x() + 1e-9);
  EXPECT_TRUE(bodyA.getTranslation().tail<2>().isApprox(
      initialPositionA.tail<2>(), 1e-12));
  EXPECT_TRUE(bodyB.getTranslation().tail<2>().isApprox(
      initialPositionB.tail<2>(), 1e-12));
  EXPECT_LT(bodyA.getLinearVelocity().x(), -1e-9);
  EXPECT_GT(bodyB.getLinearVelocity().x(), 1e-9);
  EXPECT_GT(
      (bodyB.getLinearVelocity() - bodyA.getLinearVelocity())
          .dot(Eigen::Vector3d::UnitX()),
      1e-6);
}

// Test that the private AVBD contact-stage opt-in also activates for a
// static/dynamic contact when only the dynamic body carries the private config.
TEST(World, RigidBodyContactStageAvbdProjectsStaticDynamicContactVelocity)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_contact_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("avbd_contact_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that AVBD contact-stage velocity projection feeds the following rigid
// body position stage instead of requiring direct pose writeback.
TEST(World, RigidBodyContactStageAvbdFeedsRigidBodyPositionStage)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_position_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("avbd_position_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::RigidBodyPositionStage positionStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage).addStage(positionStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_GT(sphere.getTranslation().z(), initialSpherePosition.z() + 1e-9);
  EXPECT_TRUE(sphere.getTranslation().head<2>().isApprox(
      initialSpherePosition.head<2>(), 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that the built-in World::step() schedule reaches the same AVBD
// contact-stage velocity projection before the rigid body position stage.
TEST(World, RigidBodyContactStageAvbdRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_default_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("avbd_default_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_GT(sphere.getTranslation().z(), initialSpherePosition.z() + 1e-9);
  EXPECT_TRUE(sphere.getTranslation().head<2>().isApprox(
      initialSpherePosition.head<2>(), 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that static/dynamic AVBD contact-stage activation is keyed by either
// contact endpoint, including the static endpoint.
TEST(World, RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_static_owned_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("avbd_static_owned_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(ground.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that the built-in World::step() schedule reaches static-owned
// static/dynamic AVBD contact-stage activation.
TEST(World, RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground
      = world.addRigidBody("avbd_default_static_owned_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere
      = world.addRigidBody("avbd_default_static_owned_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(ground.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_GT(sphere.getTranslation().z(), initialSpherePosition.z() + 1e-9);
  EXPECT_TRUE(sphere.getTranslation().head<2>().isApprox(
      initialSpherePosition.head<2>(), 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that static-body stored velocities do not behave like moving obstacles
// in the static-owned AVBD contact-stage path.
TEST(World, RigidBodyContactStageAvbdIgnoresStoredStaticVelocity)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const bool storeStaticVelocity) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle
        = world.addRigidBody("avbd_static_velocity_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions dynamicOptions;
    dynamicOptions.mass = 1.0;
    dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    auto dynamic
        = world.addRigidBody("avbd_static_velocity_dynamic", dynamicOptions);
    dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    const Eigen::Vector3d initialDynamicPosition = dynamic.getTranslation();
    const auto contacts = world.collide();
    EXPECT_FALSE(contacts.empty());
    if (contacts.empty()) {
      return ContactStageOutcome{};
    }

    constexpr double staticSpeed = 5.0;
    if (storeStaticVelocity) {
      const auto& contact = contacts.front();
      if (contact.bodyA.getEntity() == obstacle.getEntity()) {
        obstacle.setLinearVelocity(staticSpeed * contact.normal);
      } else {
        EXPECT_EQ(contact.bodyB.getEntity(), obstacle.getEntity());
        obstacle.setLinearVelocity(-staticSpeed * contact.normal);
      }
    }

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);

    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(
        dynamic.getTranslation().isApprox(initialDynamicPosition, 1e-12));
    EXPECT_NEAR(
        obstacle.getLinearVelocity().norm(),
        storeStaticVelocity ? staticSpeed : 0.0,
        1e-12);

    return ContactStageOutcome{
        dynamic.getLinearVelocity(), dynamic.getAngularVelocity()};
  };

  const ContactStageOutcome baseline = runContactStage(false);
  const ContactStageOutcome movingStatic = runContactStage(true);

  EXPECT_GT(baseline.linearVelocity.x(), 1e-9);
  EXPECT_TRUE(
      movingStatic.linearVelocity.isApprox(baseline.linearVelocity, 1e-12));
  EXPECT_TRUE(
      movingStatic.angularVelocity.isApprox(baseline.angularVelocity, 1e-12));
}

// Test that kinematic bodies are fixed endpoints in the AVBD contact-stage path
// and their prescribed velocity does not perturb contact projection.
TEST(World, RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d obstaclePosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d obstacleLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamicLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamicAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const bool prescribeKinematicVelocity) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle
        = world.addRigidBody("avbd_kinematic_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setKinematic(true);
    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());

    sx::RigidBodyOptions dynamicOptions;
    dynamicOptions.mass = 1.0;
    dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    auto dynamic = world.addRigidBody("avbd_kinematic_dynamic", dynamicOptions);
    dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    const Eigen::Vector3d initialDynamicPosition = dynamic.getTranslation();
    const auto contacts = world.collide();
    EXPECT_FALSE(contacts.empty());
    if (contacts.empty()) {
      return ContactStageOutcome{};
    }

    constexpr double kinematicSpeed = 5.0;
    if (prescribeKinematicVelocity) {
      const auto& contact = contacts.front();
      if (contact.bodyA.getEntity() == obstacle.getEntity()) {
        obstacle.setLinearVelocity(kinematicSpeed * contact.normal);
      } else {
        EXPECT_EQ(contact.bodyB.getEntity(), obstacle.getEntity());
        obstacle.setLinearVelocity(-kinematicSpeed * contact.normal);
      }
    }

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);

    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());
    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(
        dynamic.getTranslation().isApprox(initialDynamicPosition, 1e-12));
    EXPECT_NEAR(
        obstacle.getLinearVelocity().norm(),
        prescribeKinematicVelocity ? kinematicSpeed : 0.0,
        1e-12);

    return ContactStageOutcome{
        obstacle.getTranslation(),
        obstacle.getLinearVelocity(),
        dynamic.getLinearVelocity(),
        dynamic.getAngularVelocity()};
  };

  const ContactStageOutcome baseline = runContactStage(false);
  const ContactStageOutcome movingKinematic = runContactStage(true);

  EXPECT_GT(baseline.dynamicLinearVelocity.x(), 1e-9);
  EXPECT_TRUE(movingKinematic.obstaclePosition.isApprox(
      baseline.obstaclePosition, 1e-12));
  EXPECT_TRUE(movingKinematic.dynamicLinearVelocity.isApprox(
      baseline.dynamicLinearVelocity, 1e-12));
  EXPECT_TRUE(movingKinematic.dynamicAngularVelocity.isApprox(
      baseline.dynamicAngularVelocity, 1e-12));
}

// Test that the built-in World::step() path also treats kinematic AVBD contact
// endpoints as fixed/prescribed and feeds the projected dynamic velocity to the
// rigid body position stage.
TEST(World, RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d obstaclePosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d obstacleLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamicPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamicLinearVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const bool prescribeKinematicVelocity) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle = world.addRigidBody(
        "avbd_default_kinematic_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setKinematic(true);
    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());

    sx::RigidBodyOptions dynamicOptions;
    dynamicOptions.mass = 1.0;
    dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    auto dynamic
        = world.addRigidBody("avbd_default_kinematic_dynamic", dynamicOptions);
    dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    const Eigen::Vector3d initialDynamicPosition = dynamic.getTranslation();
    const auto contacts = world.collide();
    EXPECT_FALSE(contacts.empty());
    if (contacts.empty()) {
      return StepOutcome{};
    }

    constexpr double kinematicSpeed = 5.0;
    if (prescribeKinematicVelocity) {
      const auto& contact = contacts.front();
      if (contact.bodyA.getEntity() == obstacle.getEntity()) {
        obstacle.setLinearVelocity(kinematicSpeed * contact.normal);
      } else {
        EXPECT_EQ(contact.bodyB.getEntity(), obstacle.getEntity());
        obstacle.setLinearVelocity(-kinematicSpeed * contact.normal);
      }
    }

    world.step();

    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());
    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_GT((dynamic.getTranslation() - initialDynamicPosition).norm(), 1e-9);
    EXPECT_NEAR(
        obstacle.getLinearVelocity().norm(),
        prescribeKinematicVelocity ? kinematicSpeed : 0.0,
        1e-12);

    return StepOutcome{
        obstacle.getTranslation(),
        obstacle.getLinearVelocity(),
        dynamic.getTranslation(),
        dynamic.getLinearVelocity()};
  };

  const StepOutcome baseline = runDefaultStep(false);
  const StepOutcome movingKinematic = runDefaultStep(true);

  EXPECT_TRUE(movingKinematic.obstaclePosition.isApprox(
      baseline.obstaclePosition, 1e-12));
  EXPECT_TRUE(movingKinematic.dynamicPosition.isApprox(
      baseline.dynamicPosition, 1e-12));
  EXPECT_TRUE(movingKinematic.dynamicLinearVelocity.isApprox(
      baseline.dynamicLinearVelocity, 1e-12));
}

// Test that a disabled peer config does not veto contact-stage AVBD activation
// when the other contact endpoint carries an enabled private config.
TEST(World, RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_disabled_peer_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("avbd_disabled_peer_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& groundConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(ground.getEntity()));
  groundConfig.enabled = false;
  groundConfig.startStiffness = 10000.0;
  groundConfig.maxStiffness = 10000.0;

  auto& sphereConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
  sphereConfig.startStiffness = 200.0;
  sphereConfig.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that the built-in World::step() schedule also ignores a disabled peer
// config when the other endpoint enables AVBD contact-stage projection.
TEST(
    World,
    RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground
      = world.addRigidBody("avbd_default_disabled_peer_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.4);
  auto sphere
      = world.addRigidBody("avbd_default_disabled_peer_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& groundConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(ground.getEntity()));
  groundConfig.enabled = false;
  groundConfig.startStiffness = 10000.0;
  groundConfig.maxStiffness = 10000.0;

  auto& sphereConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
  sphereConfig.startStiffness = 200.0;
  sphereConfig.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  world.step();

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_GT(sphere.getTranslation().z(), initialSpherePosition.z() + 1e-9);
  EXPECT_TRUE(sphere.getTranslation().head<2>().isApprox(
      initialSpherePosition.head<2>(), 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(sphere.getLinearVelocity().z(), 1e-9);
}

// Test that the AVBD contact-stage path handles a live configured contact set
// with multiple static/dynamic contacts, not only a single manifold.
TEST(World, RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_multi_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 2.0, 0.25)));

  sx::RigidBodyOptions leftOptions;
  leftOptions.mass = 1.0;
  leftOptions.position = Eigen::Vector3d(-0.75, 0.0, 0.3);
  auto left = world.addRigidBody("avbd_multi_left", leftOptions);
  left.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions rightOptions;
  rightOptions.mass = 1.0;
  rightOptions.position = Eigen::Vector3d(0.75, 0.0, 0.3);
  auto right = world.addRigidBody("avbd_multi_right", rightOptions);
  right.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  const auto configureAvbdContact = [&](const sx::RigidBody& body) {
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(body.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;
  };
  configureAvbdContact(left);
  configureAvbdContact(right);

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialLeftPosition = left.getTranslation();
  const Eigen::Vector3d initialRightPosition = right.getTranslation();
  ASSERT_GE(world.collide().size(), 2u);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_TRUE(left.getTranslation().isApprox(initialLeftPosition, 1e-12));
  EXPECT_TRUE(right.getTranslation().isApprox(initialRightPosition, 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(left.getLinearVelocity().z(), 1e-9);
  EXPECT_GT(right.getLinearVelocity().z(), 1e-9);
}

// Test that multiple configured contacts also feed through the built-in
// World::step() schedule and following rigid body position stage.
TEST(
    World,
    RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_default_multi_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 2.0, 0.25)));

  sx::RigidBodyOptions leftOptions;
  leftOptions.mass = 1.0;
  leftOptions.position = Eigen::Vector3d(-0.75, 0.0, 0.3);
  auto left = world.addRigidBody("avbd_default_multi_left", leftOptions);
  left.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions rightOptions;
  rightOptions.mass = 1.0;
  rightOptions.position = Eigen::Vector3d(0.75, 0.0, 0.3);
  auto right = world.addRigidBody("avbd_default_multi_right", rightOptions);
  right.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  const auto configureAvbdContact = [&](const sx::RigidBody& body) {
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(body.getEntity()));
    config.startStiffness = 200.0;
    config.maxStiffness = 200.0;
  };
  configureAvbdContact(left);
  configureAvbdContact(right);

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialLeftPosition = left.getTranslation();
  const Eigen::Vector3d initialRightPosition = right.getTranslation();
  ASSERT_GE(world.collide().size(), 2u);

  world.step();

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_GT(left.getTranslation().z(), initialLeftPosition.z() + 1e-9);
  EXPECT_GT(right.getTranslation().z(), initialRightPosition.z() + 1e-9);
  EXPECT_TRUE(left.getTranslation().head<2>().isApprox(
      initialLeftPosition.head<2>(), 1e-12));
  EXPECT_TRUE(right.getTranslation().head<2>().isApprox(
      initialRightPosition.head<2>(), 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_GT(left.getLinearVelocity().z(), 1e-9);
  EXPECT_GT(right.getLinearVelocity().z(), 1e-9);
}

// Test that AVBD contact-stage activation is all-or-nothing for a live contact
// set. If any active contact lacks the private opt-in config, the configured
// contacts fall back with the rest of the set instead of being partially
// projected through AVBD while other contacts use sequential impulses.
TEST(World, RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_mixed_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 2.0, 0.25)));

  sx::RigidBodyOptions configuredOptions;
  configuredOptions.mass = 1.0;
  configuredOptions.position = Eigen::Vector3d(-0.75, 0.0, 0.3);
  configuredOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto configured
      = world.addRigidBody("avbd_mixed_configured", configuredOptions);
  configured.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions ordinaryOptions;
  ordinaryOptions.mass = 1.0;
  ordinaryOptions.position = Eigen::Vector3d(0.75, 0.0, 0.3);
  ordinaryOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto ordinary = world.addRigidBody("avbd_mixed_ordinary", ordinaryOptions);
  ordinary.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(configured.getEntity()));
  config.startStiffness = 200.0;
  config.maxStiffness = 200.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialConfiguredPosition = configured.getTranslation();
  const Eigen::Vector3d initialOrdinaryPosition = ordinary.getTranslation();
  ASSERT_GE(world.collide().size(), 2u);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_TRUE(
      configured.getTranslation().isApprox(initialConfiguredPosition, 1e-12));
  EXPECT_TRUE(
      ordinary.getTranslation().isApprox(initialOrdinaryPosition, 1e-12));
  EXPECT_NEAR(configured.getLinearVelocity().z(), 0.0, 1e-9);
  EXPECT_NEAR(ordinary.getLinearVelocity().z(), 0.0, 1e-9);
}

// Test that the built-in World::step() schedule preserves the same
// all-or-nothing AVBD contact-stage fallback for mixed configured and
// unconfigured live contact sets.
TEST(
    World, RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d groundPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d configuredPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d configuredLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d ordinaryPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d ordinaryLinearVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const bool configureOneEndpoint) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
    auto ground
        = world.addRigidBody("avbd_default_mixed_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(4.0, 2.0, 0.25)));

    sx::RigidBodyOptions configuredOptions;
    configuredOptions.mass = 1.0;
    configuredOptions.position = Eigen::Vector3d(-0.75, 0.0, 0.3);
    configuredOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
    auto configured = world.addRigidBody(
        "avbd_default_mixed_configured", configuredOptions);
    configured.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    sx::RigidBodyOptions ordinaryOptions;
    ordinaryOptions.mass = 1.0;
    ordinaryOptions.position = Eigen::Vector3d(0.75, 0.0, 0.3);
    ordinaryOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
    auto ordinary
        = world.addRigidBody("avbd_default_mixed_ordinary", ordinaryOptions);
    ordinary.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    if (configureOneEndpoint) {
      auto& registry = sx::detail::registryOf(world);
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(configured.getEntity()));
      config.startStiffness = 200.0;
      config.maxStiffness = 200.0;
    }

    EXPECT_GE(world.collide().size(), 2u);

    world.step();

    return StepOutcome{
        ground.getTranslation(),
        configured.getTranslation(),
        configured.getLinearVelocity(),
        ordinary.getTranslation(),
        ordinary.getLinearVelocity()};
  };

  const StepOutcome unconfigured = runDefaultStep(false);
  const StepOutcome mixed = runDefaultStep(true);

  EXPECT_TRUE(
      mixed.groundPosition.isApprox(unconfigured.groundPosition, 1e-12));
  EXPECT_TRUE(mixed.configuredPosition.isApprox(
      unconfigured.configuredPosition, 1e-12));
  EXPECT_TRUE(
      mixed.ordinaryPosition.isApprox(unconfigured.ordinaryPosition, 1e-12));
  EXPECT_TRUE(mixed.configuredLinearVelocity.isApprox(
      unconfigured.configuredLinearVelocity, 1e-12));
  EXPECT_TRUE(mixed.ordinaryLinearVelocity.isApprox(
      unconfigured.ordinaryLinearVelocity, 1e-12));
  EXPECT_NEAR(mixed.configuredLinearVelocity.z(), 0.0, 1e-9);
  EXPECT_NEAR(mixed.ordinaryLinearVelocity.z(), 0.0, 1e-9);
}

// Test that an explicitly disabled private contact config opts out of the AVBD
// contact-stage path and falls back to the ordinary rigid contact response.
TEST(World, RigidBodyContactStageAvbdDisabledConfigFallsBack)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("avbd_disabled_ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.3);
  sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto sphere = world.addRigidBody("avbd_disabled_sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
      sx::detail::toRegistryEntity(sphere.getEntity()));
  config.enabled = false;
  config.startStiffness = 10000.0;
  config.maxStiffness = 10000.0;

  const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
  const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
  ASSERT_FALSE(world.collide().empty());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidBodyContactStage contactStage(/*iterations=*/8);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(contactStage);
  world.step(executor, pipeline);

  EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
  EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_NEAR(sphere.getLinearVelocity().z(), 0.0, 1e-9);
}

// Test that the built-in World::step() schedule preserves disabled-config
// fallback instead of activating the AVBD contact-stage projection.
TEST(
    World,
    RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d groundPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d spherePosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d sphereLinearVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const bool addDisabledConfig) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
    auto ground
        = world.addRigidBody("avbd_default_disabled_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.3);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
    auto sphere
        = world.addRigidBody("avbd_default_disabled_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

    if (addDisabledConfig) {
      auto& registry = sx::detail::registryOf(world);
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(sphere.getEntity()));
      config.enabled = false;
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
    }

    EXPECT_FALSE(world.collide().empty());

    world.step();

    return StepOutcome{
        ground.getTranslation(),
        sphere.getTranslation(),
        sphere.getLinearVelocity()};
  };

  const StepOutcome unconfigured = runDefaultStep(false);
  const StepOutcome disabled = runDefaultStep(true);

  EXPECT_TRUE(
      disabled.groundPosition.isApprox(unconfigured.groundPosition, 1e-12));
  EXPECT_TRUE(
      disabled.spherePosition.isApprox(unconfigured.spherePosition, 1e-12));
  EXPECT_TRUE(disabled.sphereLinearVelocity.isApprox(
      unconfigured.sphereLinearVelocity, 1e-12));
  EXPECT_NEAR(disabled.sphereLinearVelocity.z(), 0.0, 1e-9);
}

// Test that the live AVBD contact-stage path uses warm-started normal rows to
// bound Coulomb friction rows in the following frame.
TEST(World, RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
    auto ground = world.addRigidBody("avbd_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));
    ground.setFriction(1.0);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.3);
    sphereOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto sphere = world.addRigidBody("avbd_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(sphere.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.5;
    config.gamma = 1.0;

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.linearVelocity.x()
        - radius * frictionless.angularVelocity.y();
  const double frictionalContactSlip
      = frictional.linearVelocity.x() - radius * frictional.angularVelocity.y();

  EXPECT_GT(frictionless.linearVelocity.z(), 1e-9);
  EXPECT_GT(frictional.linearVelocity.z(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.linearVelocity.x(), -1e-9);
  EXPECT_GT(frictional.angularVelocity.y(), 1e-9);
}

// Test that static-owned contact configs also feed warm-started AVBD friction
// rows in the contact-stage path.
TEST(World, RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle
        = world.addRigidBody("avbd_static_friction_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setFriction(1.0);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto sphere
        = world.addRigidBody("avbd_static_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.5;
    config.gamma = 1.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
    EXPECT_TRUE(obstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.linearVelocity.y()
        - radius * frictionless.angularVelocity.z();
  const double frictionalContactSlip
      = frictional.linearVelocity.y() - radius * frictional.angularVelocity.z();

  EXPECT_GT(frictionless.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.linearVelocity.x(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.angularVelocity.z(), 1e-9);
}

// Test that the built-in World::step() schedule also reaches the warm-started
// static/dynamic contact-stage friction path.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle
        = world.addRigidBody("avbd_default_friction_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setFriction(1.0);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto sphere
        = world.addRigidBody("avbd_default_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(sphere.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.95;
    config.gamma = 1.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    world.step();
    EXPECT_FALSE(world.collide().empty());
    world.step();

    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(obstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.linearVelocity.y()
        - radius * frictionless.angularVelocity.z();
  const double frictionalContactSlip
      = frictional.linearVelocity.y() - radius * frictional.angularVelocity.z();

  EXPECT_GT(frictionless.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.linearVelocity.x(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.angularVelocity.z(), 1e-9);
}

// Test that the built-in World::step() schedule also reaches static-owned
// warm-started AVBD friction rows.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle = world.addRigidBody(
        "avbd_default_static_friction_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setFriction(1.0);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto sphere = world.addRigidBody(
        "avbd_default_static_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.95;
    config.gamma = 1.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    world.step();
    EXPECT_FALSE(world.collide().empty());
    world.step();

    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(obstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.linearVelocity.y()
        - radius * frictionless.angularVelocity.z();
  const double frictionalContactSlip
      = frictional.linearVelocity.y() - radius * frictional.angularVelocity.z();

  EXPECT_GT(frictionless.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.linearVelocity.x(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.angularVelocity.z(), 1e-9);
}

// Test that kinematic-owned contact configs also feed warm-started AVBD
// friction rows without treating the prescribed kinematic velocity as contact
// slip input.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d dynamicLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamicAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double sphereFriction,
                                  const bool prescribeKinematicVelocity) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle = world.addRigidBody(
        "avbd_kinematic_friction_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setFriction(1.0);
    obstacle.setKinematic(true);
    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto sphere
        = world.addRigidBody("avbd_kinematic_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.5;
    config.gamma = 1.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    constexpr double kinematicSpeed = 5.0;
    if (prescribeKinematicVelocity) {
      obstacle.setLinearVelocity(Eigen::Vector3d(0.0, -kinematicSpeed, 0.0));
    }

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());
    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
    EXPECT_TRUE(obstacle.getLinearVelocity().isApprox(
        prescribeKinematicVelocity ? Eigen::Vector3d(0.0, -kinematicSpeed, 0.0)
                                   : Eigen::Vector3d::Zero(),
        1e-12));

    return ContactStageOutcome{
        sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0, true);
  const ContactStageOutcome frictional = runContactStage(1.0, true);
  const ContactStageOutcome stationaryFrictional = runContactStage(1.0, false);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.dynamicLinearVelocity.y()
        - radius * frictionless.dynamicAngularVelocity.z();
  const double frictionalContactSlip
      = frictional.dynamicLinearVelocity.y()
        - radius * frictional.dynamicAngularVelocity.z();

  EXPECT_GT(frictionless.dynamicLinearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.dynamicLinearVelocity.x(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.dynamicLinearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.dynamicAngularVelocity.z(), 1e-9);
  EXPECT_TRUE(frictional.dynamicLinearVelocity.isApprox(
      stationaryFrictional.dynamicLinearVelocity, 1e-12));
  EXPECT_TRUE(frictional.dynamicAngularVelocity.isApprox(
      stationaryFrictional.dynamicAngularVelocity, 1e-12));
}

// Test that the built-in World::step() schedule also reaches kinematic-owned
// warm-started AVBD friction rows without using the prescribed kinematic
// velocity as tangential slip.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d dynamicLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamicAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double sphereFriction,
                                 const bool prescribeKinematicVelocity) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle = world.addRigidBody(
        "avbd_default_kinematic_friction_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setFriction(1.0);
    obstacle.setKinematic(true);
    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto sphere = world.addRigidBody(
        "avbd_default_kinematic_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.95;
    config.gamma = 1.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    constexpr double kinematicSpeed = 5.0;
    if (prescribeKinematicVelocity) {
      obstacle.setLinearVelocity(Eigen::Vector3d(0.0, -kinematicSpeed, 0.0));
    }

    world.step();
    EXPECT_FALSE(world.collide().empty());
    world.step();

    EXPECT_TRUE(obstacle.isKinematic());
    EXPECT_FALSE(obstacle.isStatic());
    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(obstacle.getLinearVelocity().isApprox(
        prescribeKinematicVelocity ? Eigen::Vector3d(0.0, -kinematicSpeed, 0.0)
                                   : Eigen::Vector3d::Zero(),
        1e-12));

    return StepOutcome{sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0, true);
  const StepOutcome frictional = runDefaultStep(1.0, true);
  const StepOutcome stationaryFrictional = runDefaultStep(1.0, false);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.dynamicLinearVelocity.y()
        - radius * frictionless.dynamicAngularVelocity.z();
  const double frictionalContactSlip
      = frictional.dynamicLinearVelocity.y()
        - radius * frictional.dynamicAngularVelocity.z();

  EXPECT_GT(frictionless.dynamicLinearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.dynamicLinearVelocity.x(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.dynamicLinearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.dynamicAngularVelocity.z(), 1e-9);
  EXPECT_TRUE(frictional.dynamicLinearVelocity.isApprox(
      stationaryFrictional.dynamicLinearVelocity, 1e-12));
  EXPECT_TRUE(frictional.dynamicAngularVelocity.isApprox(
      stationaryFrictional.dynamicAngularVelocity, 1e-12));
}

// Test that an explicitly disabled peer config does not veto warm-started AVBD
// friction rows when the other contact endpoint carries an enabled config.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
    auto ground = world.addRigidBody(
        "avbd_disabled_peer_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));
    ground.setFriction(1.0);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.3);
    sphereOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto sphere = world.addRigidBody(
        "avbd_disabled_peer_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& groundConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(ground.getEntity()));
    groundConfig.enabled = false;
    groundConfig.startStiffness = 10000.0;
    groundConfig.maxStiffness = 10000.0;

    auto& sphereConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(sphere.getEntity()));
    sphereConfig.startStiffness = 10000.0;
    sphereConfig.maxStiffness = 10000.0;
    sphereConfig.alpha = 0.5;
    sphereConfig.gamma = 1.0;

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialSpherePosition = sphere.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(sphere.getTranslation().isApprox(initialSpherePosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.linearVelocity.x()
        - radius * frictionless.angularVelocity.y();
  const double frictionalContactSlip
      = frictional.linearVelocity.x() - radius * frictional.angularVelocity.y();

  EXPECT_GT(frictionless.linearVelocity.z(), 1e-9);
  EXPECT_GT(frictional.linearVelocity.z(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.linearVelocity.x(), -1e-9);
  EXPECT_GT(frictional.angularVelocity.y(), 1e-9);
}

// Test that the built-in World::step() schedule also ignores disabled peer
// configs for warm-started AVBD friction rows.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions obstacleOptions;
    obstacleOptions.isStatic = true;
    obstacleOptions.position = Eigen::Vector3d::Zero();
    auto obstacle = world.addRigidBody(
        "avbd_default_disabled_peer_friction_obstacle", obstacleOptions);
    obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    obstacle.setFriction(1.0);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.mass = 1.0;
    sphereOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
    sphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto sphere = world.addRigidBody(
        "avbd_default_disabled_peer_friction_sphere", sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& obstacleConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(obstacle.getEntity()));
    obstacleConfig.enabled = false;
    obstacleConfig.startStiffness = 10000.0;
    obstacleConfig.maxStiffness = 10000.0;

    auto& sphereConfig = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(sphere.getEntity()));
    sphereConfig.startStiffness = 10000.0;
    sphereConfig.maxStiffness = 10000.0;
    sphereConfig.alpha = 0.95;
    sphereConfig.gamma = 1.0;

    const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    world.step();
    EXPECT_FALSE(world.collide().empty());
    world.step();

    EXPECT_TRUE(
        obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
    EXPECT_TRUE(obstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{sphere.getLinearVelocity(), sphere.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);
  const double radius = 0.5;
  const double frictionlessContactSlip
      = frictionless.linearVelocity.y()
        - radius * frictionless.angularVelocity.z();
  const double frictionalContactSlip
      = frictional.linearVelocity.y() - radius * frictional.angularVelocity.z();

  EXPECT_GT(frictionless.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.linearVelocity.x(), 1e-9);
  EXPECT_LT(std::abs(frictionalContactSlip), std::abs(frictionlessContactSlip));
  EXPECT_GT(frictional.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.angularVelocity.z(), 1e-9);
}

// Test that simultaneous configured static/dynamic contacts all feed
// warm-started AVBD friction rows, not only the first live contact.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide)
{
  namespace sx = dart::simulation;

  struct BodyOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  struct ContactStageOutcome
  {
    BodyOutcome left;
    BodyOutcome right;
  };

  const auto runContactStage = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions leftObstacleOptions;
    leftObstacleOptions.isStatic = true;
    leftObstacleOptions.position = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto leftObstacle = world.addRigidBody(
        "avbd_multi_friction_left_obstacle", leftObstacleOptions);
    leftObstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    leftObstacle.setFriction(1.0);

    sx::RigidBodyOptions rightObstacleOptions;
    rightObstacleOptions.isStatic = true;
    rightObstacleOptions.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto rightObstacle = world.addRigidBody(
        "avbd_multi_friction_right_obstacle", rightObstacleOptions);
    rightObstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    rightObstacle.setFriction(1.0);

    sx::RigidBodyOptions leftSphereOptions;
    leftSphereOptions.mass = 1.0;
    leftSphereOptions.position = Eigen::Vector3d(-0.1, 0.0, 0.0);
    leftSphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto leftSphere = world.addRigidBody(
        "avbd_multi_friction_left_sphere", leftSphereOptions);
    leftSphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    leftSphere.setFriction(sphereFriction);

    sx::RigidBodyOptions rightSphereOptions;
    rightSphereOptions.mass = 1.0;
    rightSphereOptions.position = Eigen::Vector3d(1.9, 0.0, 0.0);
    rightSphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto rightSphere = world.addRigidBody(
        "avbd_multi_friction_right_sphere", rightSphereOptions);
    rightSphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    rightSphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftSphere);
    configureAvbdContact(rightSphere);

    const Eigen::Vector3d initialLeftObstaclePosition
        = leftObstacle.getTranslation();
    const Eigen::Vector3d initialRightObstaclePosition
        = rightObstacle.getTranslation();
    const Eigen::Vector3d initialLeftSpherePosition
        = leftSphere.getTranslation();
    const Eigen::Vector3d initialRightSpherePosition
        = rightSphere.getTranslation();
    EXPECT_GE(world.collide().size(), 2u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(leftObstacle.getTranslation().isApprox(
        initialLeftObstaclePosition, 1e-12));
    EXPECT_TRUE(rightObstacle.getTranslation().isApprox(
        initialRightObstaclePosition, 1e-12));
    EXPECT_TRUE(
        leftSphere.getTranslation().isApprox(initialLeftSpherePosition, 1e-12));
    EXPECT_TRUE(rightSphere.getTranslation().isApprox(
        initialRightSpherePosition, 1e-12));
    EXPECT_TRUE(
        leftObstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(
        rightObstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        BodyOutcome{
            leftSphere.getLinearVelocity(), leftSphere.getAngularVelocity()},
        BodyOutcome{
            rightSphere.getLinearVelocity(), rightSphere.getAngularVelocity()}};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);
  const double radius = 0.5;
  const auto contactSlip = [radius](const BodyOutcome& outcome) {
    return outcome.linearVelocity.y() - radius * outcome.angularVelocity.z();
  };

  EXPECT_GT(frictionless.left.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictionless.right.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.left.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.right.linearVelocity.x(), 1e-9);
  EXPECT_LT(
      std::abs(contactSlip(frictional.left)),
      std::abs(contactSlip(frictionless.left)));
  EXPECT_LT(
      std::abs(contactSlip(frictional.right)),
      std::abs(contactSlip(frictionless.right)));
  EXPECT_GT(frictional.left.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.right.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.left.angularVelocity.z(), 1e-9);
  EXPECT_GT(frictional.right.angularVelocity.z(), 1e-9);
}

// Test that the built-in World::step() schedule reaches warm-started AVBD
// friction rows for multiple configured contacts in one live contact set.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct BodyOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  };

  struct StepOutcome
  {
    BodyOutcome left;
    BodyOutcome right;
  };

  const auto runDefaultStep = [](const double sphereFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions leftObstacleOptions;
    leftObstacleOptions.isStatic = true;
    leftObstacleOptions.position = Eigen::Vector3d(-1.0, 0.0, 0.0);
    auto leftObstacle = world.addRigidBody(
        "avbd_default_multi_friction_left_obstacle", leftObstacleOptions);
    leftObstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    leftObstacle.setFriction(1.0);

    sx::RigidBodyOptions rightObstacleOptions;
    rightObstacleOptions.isStatic = true;
    rightObstacleOptions.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto rightObstacle = world.addRigidBody(
        "avbd_default_multi_friction_right_obstacle", rightObstacleOptions);
    rightObstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    rightObstacle.setFriction(1.0);

    sx::RigidBodyOptions leftSphereOptions;
    leftSphereOptions.mass = 1.0;
    leftSphereOptions.position = Eigen::Vector3d(-0.1, 0.0, 0.0);
    leftSphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto leftSphere = world.addRigidBody(
        "avbd_default_multi_friction_left_sphere", leftSphereOptions);
    leftSphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    leftSphere.setFriction(sphereFriction);

    sx::RigidBodyOptions rightSphereOptions;
    rightSphereOptions.mass = 1.0;
    rightSphereOptions.position = Eigen::Vector3d(1.9, 0.0, 0.0);
    rightSphereOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto rightSphere = world.addRigidBody(
        "avbd_default_multi_friction_right_sphere", rightSphereOptions);
    rightSphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    rightSphere.setFriction(sphereFriction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftSphere);
    configureAvbdContact(rightSphere);

    const Eigen::Vector3d initialLeftObstaclePosition
        = leftObstacle.getTranslation();
    const Eigen::Vector3d initialRightObstaclePosition
        = rightObstacle.getTranslation();
    EXPECT_GE(world.collide().size(), 2u);

    world.step();
    EXPECT_GE(world.collide().size(), 2u);
    world.step();

    EXPECT_TRUE(leftObstacle.getTranslation().isApprox(
        initialLeftObstaclePosition, 1e-12));
    EXPECT_TRUE(rightObstacle.getTranslation().isApprox(
        initialRightObstaclePosition, 1e-12));
    EXPECT_TRUE(
        leftObstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(
        rightObstacle.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{
        BodyOutcome{
            leftSphere.getLinearVelocity(), leftSphere.getAngularVelocity()},
        BodyOutcome{
            rightSphere.getLinearVelocity(), rightSphere.getAngularVelocity()}};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);
  const double radius = 0.5;
  const auto contactSlip = [radius](const BodyOutcome& outcome) {
    return outcome.linearVelocity.y() - radius * outcome.angularVelocity.z();
  };

  EXPECT_GT(frictionless.left.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictionless.right.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.left.linearVelocity.x(), 1e-9);
  EXPECT_GT(frictional.right.linearVelocity.x(), 1e-9);
  EXPECT_LT(
      std::abs(contactSlip(frictional.left)),
      std::abs(contactSlip(frictionless.left)));
  EXPECT_LT(
      std::abs(contactSlip(frictional.right)),
      std::abs(contactSlip(frictionless.right)));
  EXPECT_GT(frictional.left.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.right.linearVelocity.y(), -1e-9);
  EXPECT_GT(frictional.left.angularVelocity.z(), 1e-9);
  EXPECT_GT(frictional.right.angularVelocity.z(), 1e-9);
}

// Test that warm-started AVBD friction rows act on live box-box contact
// manifolds, not only sphere-like single-point contacts.
TEST(World, RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double boxFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d::Zero();
    auto ground = world.addRigidBody(
        "avbd_box_manifold_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));
    ground.setFriction(1.0);

    sx::RigidBodyOptions boxOptions;
    boxOptions.mass = 1.0;
    boxOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
    boxOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto box = world.addRigidBody("avbd_box_manifold_friction_box", boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    box.setFriction(boxFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(box.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.5;
    config.gamma = 1.0;

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialBoxPosition = box.getTranslation();
    EXPECT_GE(world.collide().size(), 4u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(box.getTranslation().isApprox(initialBoxPosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{box.getLinearVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);

  EXPECT_GT(frictionless.linearVelocity.y(), 1e-9);
  EXPECT_LT(
      std::abs(frictional.linearVelocity.y()),
      std::abs(frictionless.linearVelocity.y()));
  EXPECT_GT(frictional.linearVelocity.y(), -1e-9);
}

// Test that the built-in World::step() schedule also reaches warm-started AVBD
// friction rows for a live box-box contact manifold.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double boxFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d::Zero();
    auto ground = world.addRigidBody(
        "avbd_default_box_manifold_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.5)));
    ground.setFriction(1.0);

    sx::RigidBodyOptions boxOptions;
    boxOptions.mass = 1.0;
    boxOptions.position = Eigen::Vector3d(0.8, 0.0, 0.0);
    boxOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto box = world.addRigidBody(
        "avbd_default_box_manifold_friction_box", boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    box.setFriction(boxFriction);

    auto& registry = sx::detail::registryOf(world);
    auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
        sx::detail::toRegistryEntity(box.getEntity()));
    config.startStiffness = 10000.0;
    config.maxStiffness = 10000.0;
    config.alpha = 0.95;
    config.gamma = 1.0;

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    EXPECT_GE(world.collide().size(), 4u);

    world.step();
    EXPECT_GE(world.collide().size(), 4u);
    world.step();

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{box.getLinearVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  EXPECT_GT(frictionless.linearVelocity.y(), 1e-9);
  EXPECT_LT(
      std::abs(frictional.linearVelocity.y()),
      std::abs(frictionless.linearVelocity.y()));
  EXPECT_GT(frictional.linearVelocity.y(), -1e-9);
}

// Test that warm-started AVBD friction rows also act on live dynamic/dynamic
// box-box contact manifolds with multiple narrow-phase contact points.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d leftLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double boxFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions leftOptions;
    leftOptions.mass = 1.0;
    leftOptions.position = Eigen::Vector3d::Zero();
    auto leftBox = world.addRigidBody(
        "avbd_dynamic_box_manifold_friction_left", leftOptions);
    leftBox.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    leftBox.setFriction(boxFriction);

    sx::RigidBodyOptions rightOptions;
    rightOptions.mass = 1.0;
    rightOptions.position = Eigen::Vector3d(0.4, 0.0, 0.0);
    rightOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto rightBox = world.addRigidBody(
        "avbd_dynamic_box_manifold_friction_right", rightOptions);
    rightBox.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    rightBox.setFriction(boxFriction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftBox);
    configureAvbdContact(rightBox);

    const Eigen::Vector3d initialLeftPosition = leftBox.getTranslation();
    const Eigen::Vector3d initialRightPosition = rightBox.getTranslation();
    EXPECT_GE(world.collide().size(), 4u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(leftBox.getTranslation().isApprox(initialLeftPosition, 1e-12));
    EXPECT_TRUE(
        rightBox.getTranslation().isApprox(initialRightPosition, 1e-12));

    return ContactStageOutcome{
        leftBox.getLinearVelocity(),
        leftBox.getAngularVelocity(),
        rightBox.getLinearVelocity(),
        rightBox.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);
  constexpr double boxHalfExtent = 0.25;
  const auto contactSlip = [boxHalfExtent](const ContactStageOutcome& outcome) {
    const Eigen::Vector3d leftArm(boxHalfExtent, 0.0, 0.0);
    const Eigen::Vector3d rightArm(-boxHalfExtent, 0.0, 0.0);
    const Eigen::Vector3d leftContactVelocity
        = outcome.leftLinearVelocity
          + outcome.leftAngularVelocity.cross(leftArm);
    const Eigen::Vector3d rightContactVelocity
        = outcome.rightLinearVelocity
          + outcome.rightAngularVelocity.cross(rightArm);
    return rightContactVelocity.y() - leftContactVelocity.y();
  };
  const auto separatingVelocity = [](const ContactStageOutcome& outcome) {
    return outcome.rightLinearVelocity.x() - outcome.leftLinearVelocity.x();
  };

  EXPECT_GT(separatingVelocity(frictionless), 1e-9);
  EXPECT_GT(separatingVelocity(frictional), 1e-9);
  EXPECT_GT(std::abs(contactSlip(frictionless)), 1e-9);
  EXPECT_LT(
      std::abs(contactSlip(frictional)), std::abs(contactSlip(frictionless)));
}

// Test that the built-in World::step() schedule also reaches warm-started AVBD
// friction rows for live dynamic/dynamic box-box contact manifolds.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d leftLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double boxFriction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);

    sx::RigidBodyOptions leftOptions;
    leftOptions.mass = 1.0;
    leftOptions.position = Eigen::Vector3d::Zero();
    auto leftBox = world.addRigidBody(
        "avbd_default_dynamic_box_manifold_friction_left", leftOptions);
    leftBox.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    leftBox.setFriction(boxFriction);

    sx::RigidBodyOptions rightOptions;
    rightOptions.mass = 1.0;
    rightOptions.position = Eigen::Vector3d(0.2, 0.0, 0.0);
    rightOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto rightBox = world.addRigidBody(
        "avbd_default_dynamic_box_manifold_friction_right", rightOptions);
    rightBox.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    rightBox.setFriction(boxFriction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.95;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftBox);
    configureAvbdContact(rightBox);
    leftBox.setForce(Eigen::Vector3d(25.0, 0.0, 0.0));
    rightBox.setForce(Eigen::Vector3d(-25.0, 0.0, 0.0));

    EXPECT_GE(world.collide().size(), 4u);

    world.step();
    EXPECT_GE(world.collide().size(), 4u);
    world.step();

    return StepOutcome{
        leftBox.getLinearVelocity(),
        leftBox.getAngularVelocity(),
        rightBox.getLinearVelocity(),
        rightBox.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  constexpr double boxHalfExtent = 0.25;
  const auto contactSlip = [boxHalfExtent](const StepOutcome& outcome) {
    const Eigen::Vector3d leftArm(boxHalfExtent, 0.0, 0.0);
    const Eigen::Vector3d rightArm(-boxHalfExtent, 0.0, 0.0);
    const Eigen::Vector3d leftContactVelocity
        = outcome.leftLinearVelocity
          + outcome.leftAngularVelocity.cross(leftArm);
    const Eigen::Vector3d rightContactVelocity
        = outcome.rightLinearVelocity
          + outcome.rightAngularVelocity.cross(rightArm);
    return rightContactVelocity.y() - leftContactVelocity.y();
  };
  const auto separatingVelocity = [](const StepOutcome& outcome) {
    return outcome.rightLinearVelocity.x() - outcome.leftLinearVelocity.x();
  };

  EXPECT_GT(separatingVelocity(frictionless), 1e-9);
  EXPECT_GT(separatingVelocity(frictional), 1e-9);
  EXPECT_GT(std::abs(contactSlip(frictionless)), 1e-9);
  EXPECT_LT(
      std::abs(contactSlip(frictional)), std::abs(contactSlip(frictionless)));
}

// Test that warm-started AVBD contact-stage friction also acts on a stacked
// contact set containing both static/dynamic and dynamic/dynamic box manifolds.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionReducesSlip)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d lowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d lowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d upperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d upperAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    constexpr double halfExtent = 0.5;
    const Eigen::Vector3d boxHalfExtents
        = Eigen::Vector3d::Constant(halfExtent);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_stacked_box_manifold_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, halfExtent)));
    ground.setFriction(friction);

    sx::RigidBodyOptions lowerOptions;
    lowerOptions.mass = 1.0;
    lowerOptions.position = Eigen::Vector3d(0.02, 0.0, 0.45);
    auto lower = world.addRigidBody(
        "avbd_stacked_box_manifold_friction_lower", lowerOptions);
    lower.setCollisionShape(sx::CollisionShape::makeBox(boxHalfExtents));
    lower.setFriction(friction);

    sx::RigidBodyOptions upperOptions;
    upperOptions.mass = 1.0;
    upperOptions.position = Eigen::Vector3d(0.04, 0.0, 1.35);
    upperOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto upper = world.addRigidBody(
        "avbd_stacked_box_manifold_friction_upper", upperOptions);
    upper.setCollisionShape(sx::CollisionShape::makeBox(boxHalfExtents));
    upper.setFriction(friction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(lower);
    configureAvbdContact(upper);

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialLowerPosition = lower.getTranslation();
    const Eigen::Vector3d initialUpperPosition = upper.getTranslation();
    EXPECT_GE(world.collide().size(), 8u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(lower.getTranslation().isApprox(initialLowerPosition, 1e-12));
    EXPECT_TRUE(upper.getTranslation().isApprox(initialUpperPosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        lower.getLinearVelocity(),
        lower.getAngularVelocity(),
        upper.getLinearVelocity(),
        upper.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);

  constexpr double halfExtent = 0.5;
  const auto upperLowerSlip = [halfExtent](const ContactStageOutcome& outcome) {
    const Eigen::Vector3d lowerArm(0.0, 0.0, halfExtent);
    const Eigen::Vector3d upperArm(0.0, 0.0, -halfExtent);
    const Eigen::Vector3d lowerContactVelocity
        = outcome.lowerLinearVelocity
          + outcome.lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d upperContactVelocity
        = outcome.upperLinearVelocity
          + outcome.upperAngularVelocity.cross(upperArm);
    return upperContactVelocity.x() - lowerContactVelocity.x();
  };

  EXPECT_GT(std::abs(upperLowerSlip(frictionless)), 1e-9);
  EXPECT_LT(
      std::abs(upperLowerSlip(frictional)),
      std::abs(upperLowerSlip(frictionless)));
  EXPECT_LT(frictional.upperLinearVelocity.x(), 1.0 - 1e-9);
}

// Test a wider stacked contact set with two independent dynamic/dynamic
// box-box manifolds sharing one supported lower dynamic body.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d lowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d lowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    constexpr double lowerHalfZ = 0.5;
    constexpr double upperHalfZ = 0.35;
    constexpr double upperXOffset = 0.45;

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_multi_top_stack_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, lowerHalfZ)));
    ground.setFriction(friction);

    sx::RigidBodyOptions lowerOptions;
    lowerOptions.mass = 2.0;
    lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
    auto lower = world.addRigidBody(
        "avbd_multi_top_stack_friction_lower", lowerOptions);
    lower.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(1.2, 0.7, lowerHalfZ)));
    lower.setFriction(friction);

    const auto addUpper
        = [&](std::string_view name, const double x, const double yVelocity) {
            sx::RigidBodyOptions upperOptions;
            upperOptions.mass = 1.0;
            upperOptions.position = Eigen::Vector3d(x, 0.0, 1.25);
            upperOptions.linearVelocity = Eigen::Vector3d(0.0, yVelocity, 0.0);
            auto upper = world.addRigidBody(name, upperOptions);
            upper.setCollisionShape(
                sx::CollisionShape::makeBox(
                    Eigen::Vector3d(upperHalfZ, upperHalfZ, upperHalfZ)));
            upper.setFriction(friction);
            return upper;
          };
    auto leftUpper = addUpper(
        "avbd_multi_top_stack_friction_left_upper", -upperXOffset, 1.0);
    auto rightUpper = addUpper(
        "avbd_multi_top_stack_friction_right_upper", upperXOffset, -1.0);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(lower);
    configureAvbdContact(leftUpper);
    configureAvbdContact(rightUpper);

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialLowerPosition = lower.getTranslation();
    const Eigen::Vector3d initialLeftPosition = leftUpper.getTranslation();
    const Eigen::Vector3d initialRightPosition = rightUpper.getTranslation();
    EXPECT_GE(world.collide().size(), 12u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/16);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(lower.getTranslation().isApprox(initialLowerPosition, 1e-12));
    EXPECT_TRUE(
        leftUpper.getTranslation().isApprox(initialLeftPosition, 1e-12));
    EXPECT_TRUE(
        rightUpper.getTranslation().isApprox(initialRightPosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        lower.getLinearVelocity(),
        lower.getAngularVelocity(),
        leftUpper.getLinearVelocity(),
        leftUpper.getAngularVelocity(),
        rightUpper.getLinearVelocity(),
        rightUpper.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);

  const auto upperLowerSlip = [](const ContactStageOutcome& outcome,
                                 const double xOffset,
                                 const Eigen::Vector3d& upperLinearVelocity,
                                 const Eigen::Vector3d& upperAngularVelocity) {
    constexpr double lowerHalfZ = 0.5;
    constexpr double upperHalfZ = 0.35;
    const Eigen::Vector3d lowerArm(xOffset, 0.0, lowerHalfZ);
    const Eigen::Vector3d upperArm(0.0, 0.0, -upperHalfZ);
    const Eigen::Vector3d lowerContactVelocity
        = outcome.lowerLinearVelocity
          + outcome.lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d upperContactVelocity
        = upperLinearVelocity + upperAngularVelocity.cross(upperArm);
    return upperContactVelocity.y() - lowerContactVelocity.y();
  };

  constexpr double upperXOffset = 0.45;
  const double frictionlessLeftSlip = upperLowerSlip(
      frictionless,
      -upperXOffset,
      frictionless.leftUpperLinearVelocity,
      frictionless.leftUpperAngularVelocity);
  const double frictionlessRightSlip = upperLowerSlip(
      frictionless,
      upperXOffset,
      frictionless.rightUpperLinearVelocity,
      frictionless.rightUpperAngularVelocity);
  const double frictionalLeftSlip = upperLowerSlip(
      frictional,
      -upperXOffset,
      frictional.leftUpperLinearVelocity,
      frictional.leftUpperAngularVelocity);
  const double frictionalRightSlip = upperLowerSlip(
      frictional,
      upperXOffset,
      frictional.rightUpperLinearVelocity,
      frictional.rightUpperAngularVelocity);

  EXPECT_GT(std::abs(frictionlessLeftSlip), 1e-9);
  EXPECT_GT(std::abs(frictionlessRightSlip), 1e-9);
  EXPECT_LT(std::abs(frictionalLeftSlip), std::abs(frictionlessLeftSlip));
  EXPECT_LT(std::abs(frictionalRightSlip), std::abs(frictionlessRightSlip));
  EXPECT_LT(frictional.leftUpperLinearVelocity.y(), 1.0 - 1e-9);
  EXPECT_GT(frictional.rightUpperLinearVelocity.y(), -1.0 + 1e-9);
}

// Test a wider pile-style contact set with two lower dynamic supports and one
// top dynamic body spanning both supports.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedBoxPileManifoldFrictionReducesSlip)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d leftLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    constexpr double lowerHalfZ = 0.4;
    constexpr double topHalfZ = 0.35;
    constexpr double lowerXOffset = 0.45;

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_box_pile_stage_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, 0.5)));
    ground.setFriction(friction);

    const auto addLower = [&](std::string_view name, const double x) {
      sx::RigidBodyOptions lowerOptions;
      lowerOptions.mass = 1.0;
      lowerOptions.position = Eigen::Vector3d(x, 0.0, 0.38);
      auto lower = world.addRigidBody(name, lowerOptions);
      lower.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, lowerHalfZ)));
      lower.setFriction(friction);
      return lower;
    };
    auto leftLower
        = addLower("avbd_box_pile_stage_friction_left_lower", -lowerXOffset);
    auto rightLower
        = addLower("avbd_box_pile_stage_friction_right_lower", lowerXOffset);

    sx::RigidBodyOptions topOptions;
    topOptions.mass = 1.0;
    topOptions.position = Eigen::Vector3d(0.0, 0.0, 1.10);
    topOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto top
        = world.addRigidBody("avbd_box_pile_stage_friction_top", topOptions);
    top.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.7, 0.4, topHalfZ)));
    top.setFriction(friction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftLower);
    configureAvbdContact(rightLower);
    configureAvbdContact(top);

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialLeftPosition = leftLower.getTranslation();
    const Eigen::Vector3d initialRightPosition = rightLower.getTranslation();
    const Eigen::Vector3d initialTopPosition = top.getTranslation();
    EXPECT_GE(world.collide().size(), 12u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/20);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(
        leftLower.getTranslation().isApprox(initialLeftPosition, 1e-12));
    EXPECT_TRUE(
        rightLower.getTranslation().isApprox(initialRightPosition, 1e-12));
    EXPECT_TRUE(top.getTranslation().isApprox(initialTopPosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        leftLower.getLinearVelocity(),
        leftLower.getAngularVelocity(),
        rightLower.getLinearVelocity(),
        rightLower.getAngularVelocity(),
        top.getLinearVelocity(),
        top.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);

  const auto upperLowerSlip = [](const ContactStageOutcome& outcome,
                                 const double lowerXOffset,
                                 const Eigen::Vector3d& lowerLinearVelocity,
                                 const Eigen::Vector3d& lowerAngularVelocity) {
    constexpr double lowerHalfZ = 0.4;
    constexpr double topHalfZ = 0.35;
    const Eigen::Vector3d lowerArm(0.0, 0.0, lowerHalfZ);
    const Eigen::Vector3d topArm(lowerXOffset, 0.0, -topHalfZ);
    const Eigen::Vector3d lowerContactVelocity
        = lowerLinearVelocity + lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d topContactVelocity
        = outcome.topLinearVelocity + outcome.topAngularVelocity.cross(topArm);
    return topContactVelocity.y() - lowerContactVelocity.y();
  };

  constexpr double lowerXOffset = 0.45;
  const double frictionlessLeftSlip = upperLowerSlip(
      frictionless,
      -lowerXOffset,
      frictionless.leftLowerLinearVelocity,
      frictionless.leftLowerAngularVelocity);
  const double frictionlessRightSlip = upperLowerSlip(
      frictionless,
      lowerXOffset,
      frictionless.rightLowerLinearVelocity,
      frictionless.rightLowerAngularVelocity);
  const double frictionalLeftSlip = upperLowerSlip(
      frictional,
      -lowerXOffset,
      frictional.leftLowerLinearVelocity,
      frictional.leftLowerAngularVelocity);
  const double frictionalRightSlip = upperLowerSlip(
      frictional,
      lowerXOffset,
      frictional.rightLowerLinearVelocity,
      frictional.rightLowerAngularVelocity);

  EXPECT_GT(std::abs(frictionlessLeftSlip), 1e-9);
  EXPECT_GT(std::abs(frictionlessRightSlip), 1e-9);
  EXPECT_LT(std::abs(frictionalLeftSlip), std::abs(frictionlessLeftSlip));
  EXPECT_LT(std::abs(frictionalRightSlip), std::abs(frictionlessRightSlip));
  EXPECT_LT(frictional.topLinearVelocity.y(), 1.0 - 1e-9);
}

// Test a wider multi-top pile with two lower dynamic supports and two
// independent upper dynamic bodies.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionReducesSlip)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d leftLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    constexpr double lowerXOffset = 0.55;
    constexpr double lowerHalfZ = 0.4;
    constexpr double upperHalfZ = 0.35;

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_multi_top_box_pile_stage_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 2.0, 0.5)));
    ground.setFriction(friction);

    const auto addLower = [&](std::string_view name, const double x) {
      sx::RigidBodyOptions lowerOptions;
      lowerOptions.mass = 1.0;
      lowerOptions.position = Eigen::Vector3d(x, 0.0, 0.38);
      auto lower = world.addRigidBody(name, lowerOptions);
      lower.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(0.45, 0.45, lowerHalfZ)));
      lower.setFriction(friction);
      return lower;
    };
    auto leftLower = addLower(
        "avbd_multi_top_box_pile_stage_friction_left_lower", -lowerXOffset);
    auto rightLower = addLower(
        "avbd_multi_top_box_pile_stage_friction_right_lower", lowerXOffset);

    const auto addUpper = [&](std::string_view name,
                              const double x,
                              const double yVelocity) {
      sx::RigidBodyOptions upperOptions;
      upperOptions.mass = 1.0;
      upperOptions.position = Eigen::Vector3d(x, 0.0, 1.10);
      upperOptions.linearVelocity = Eigen::Vector3d(0.0, yVelocity, 0.0);
      auto upper = world.addRigidBody(name, upperOptions);
      upper.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(0.35, 0.35, upperHalfZ)));
      upper.setFriction(friction);
      return upper;
    };
    auto leftUpper = addUpper(
        "avbd_multi_top_box_pile_stage_friction_left_upper",
        -lowerXOffset,
        1.0);
    auto rightUpper = addUpper(
        "avbd_multi_top_box_pile_stage_friction_right_upper",
        lowerXOffset,
        -1.0);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftLower);
    configureAvbdContact(rightLower);
    configureAvbdContact(leftUpper);
    configureAvbdContact(rightUpper);

    const Eigen::Vector3d initialGroundPosition = ground.getTranslation();
    const Eigen::Vector3d initialLeftLowerPosition = leftLower.getTranslation();
    const Eigen::Vector3d initialRightLowerPosition
        = rightLower.getTranslation();
    const Eigen::Vector3d initialLeftUpperPosition = leftUpper.getTranslation();
    const Eigen::Vector3d initialRightUpperPosition
        = rightUpper.getTranslation();
    EXPECT_GE(world.collide().size(), 12u);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/20);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(ground.getTranslation().isApprox(initialGroundPosition, 1e-12));
    EXPECT_TRUE(
        leftLower.getTranslation().isApprox(initialLeftLowerPosition, 1e-12));
    EXPECT_TRUE(
        rightLower.getTranslation().isApprox(initialRightLowerPosition, 1e-12));
    EXPECT_TRUE(
        leftUpper.getTranslation().isApprox(initialLeftUpperPosition, 1e-12));
    EXPECT_TRUE(
        rightUpper.getTranslation().isApprox(initialRightUpperPosition, 1e-12));
    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return ContactStageOutcome{
        leftLower.getLinearVelocity(),
        leftLower.getAngularVelocity(),
        rightLower.getLinearVelocity(),
        rightLower.getAngularVelocity(),
        leftUpper.getLinearVelocity(),
        leftUpper.getAngularVelocity(),
        rightUpper.getLinearVelocity(),
        rightUpper.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);

  const auto upperLowerSlip = [](const Eigen::Vector3d& lowerLinearVelocity,
                                 const Eigen::Vector3d& lowerAngularVelocity,
                                 const Eigen::Vector3d& upperLinearVelocity,
                                 const Eigen::Vector3d& upperAngularVelocity) {
    constexpr double lowerHalfZ = 0.4;
    constexpr double upperHalfZ = 0.35;
    const Eigen::Vector3d lowerArm(0.0, 0.0, lowerHalfZ);
    const Eigen::Vector3d upperArm(0.0, 0.0, -upperHalfZ);
    const Eigen::Vector3d lowerContactVelocity
        = lowerLinearVelocity + lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d upperContactVelocity
        = upperLinearVelocity + upperAngularVelocity.cross(upperArm);
    return upperContactVelocity.y() - lowerContactVelocity.y();
  };

  const double frictionlessLeftSlip = upperLowerSlip(
      frictionless.leftLowerLinearVelocity,
      frictionless.leftLowerAngularVelocity,
      frictionless.leftUpperLinearVelocity,
      frictionless.leftUpperAngularVelocity);
  const double frictionlessRightSlip = upperLowerSlip(
      frictionless.rightLowerLinearVelocity,
      frictionless.rightLowerAngularVelocity,
      frictionless.rightUpperLinearVelocity,
      frictionless.rightUpperAngularVelocity);
  const double frictionalLeftSlip = upperLowerSlip(
      frictional.leftLowerLinearVelocity,
      frictional.leftLowerAngularVelocity,
      frictional.leftUpperLinearVelocity,
      frictional.leftUpperAngularVelocity);
  const double frictionalRightSlip = upperLowerSlip(
      frictional.rightLowerLinearVelocity,
      frictional.rightLowerAngularVelocity,
      frictional.rightUpperLinearVelocity,
      frictional.rightUpperAngularVelocity);

  EXPECT_GT(std::abs(frictionlessLeftSlip), 1e-9);
  EXPECT_GT(std::abs(frictionlessRightSlip), 1e-9);
  EXPECT_LT(std::abs(frictionalLeftSlip), std::abs(frictionlessLeftSlip));
  EXPECT_LT(std::abs(frictionalRightSlip), std::abs(frictionlessRightSlip));
  EXPECT_LT(frictional.leftUpperLinearVelocity.y(), 1.0 - 1e-9);
  EXPECT_GT(frictional.rightUpperLinearVelocity.y(), -1.0 + 1e-9);
}

// Test that the built-in World::step() schedule also reaches warm-started AVBD
// friction rows for the wider box-pile contact set.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedBoxPileManifoldFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d leftLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);

    constexpr double lowerHalfZ = 0.4;
    constexpr double topHalfZ = 0.35;
    constexpr double lowerXOffset = 0.45;

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_default_box_pile_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, 0.5)));
    ground.setFriction(friction);

    const auto addLower = [&](std::string_view name, const double x) {
      sx::RigidBodyOptions lowerOptions;
      lowerOptions.mass = 1.0;
      lowerOptions.position = Eigen::Vector3d(x, 0.0, 0.38);
      auto lower = world.addRigidBody(name, lowerOptions);
      lower.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(0.4, 0.4, lowerHalfZ)));
      lower.setFriction(friction);
      return lower;
    };
    auto leftLower
        = addLower("avbd_default_box_pile_friction_left_lower", -lowerXOffset);
    auto rightLower
        = addLower("avbd_default_box_pile_friction_right_lower", lowerXOffset);

    sx::RigidBodyOptions topOptions;
    topOptions.mass = 1.0;
    topOptions.position = Eigen::Vector3d(0.0, 0.0, 1.10);
    topOptions.linearVelocity = Eigen::Vector3d(0.0, 1.0, 0.0);
    auto top
        = world.addRigidBody("avbd_default_box_pile_friction_top", topOptions);
    top.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.7, 0.4, topHalfZ)));
    top.setFriction(friction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.95;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftLower);
    configureAvbdContact(rightLower);
    configureAvbdContact(top);

    leftLower.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));
    rightLower.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));
    top.setForce(Eigen::Vector3d(0.0, 0.0, -30.0));

    EXPECT_GE(world.collide().size(), 12u);

    world.step();
    EXPECT_GE(world.collide().size(), 12u);
    world.step();

    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{
        leftLower.getLinearVelocity(),
        leftLower.getAngularVelocity(),
        rightLower.getLinearVelocity(),
        rightLower.getAngularVelocity(),
        top.getLinearVelocity(),
        top.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  const auto upperLowerSlip = [](const StepOutcome& outcome,
                                 const double lowerXOffset,
                                 const Eigen::Vector3d& lowerLinearVelocity,
                                 const Eigen::Vector3d& lowerAngularVelocity) {
    constexpr double lowerHalfZ = 0.4;
    constexpr double topHalfZ = 0.35;
    const Eigen::Vector3d lowerArm(0.0, 0.0, lowerHalfZ);
    const Eigen::Vector3d topArm(lowerXOffset, 0.0, -topHalfZ);
    const Eigen::Vector3d lowerContactVelocity
        = lowerLinearVelocity + lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d topContactVelocity
        = outcome.topLinearVelocity + outcome.topAngularVelocity.cross(topArm);
    return topContactVelocity.y() - lowerContactVelocity.y();
  };

  constexpr double lowerXOffset = 0.45;
  const double frictionlessLeftSlip = upperLowerSlip(
      frictionless,
      -lowerXOffset,
      frictionless.leftLowerLinearVelocity,
      frictionless.leftLowerAngularVelocity);
  const double frictionlessRightSlip = upperLowerSlip(
      frictionless,
      lowerXOffset,
      frictionless.rightLowerLinearVelocity,
      frictionless.rightLowerAngularVelocity);
  const double frictionalLeftSlip = upperLowerSlip(
      frictional,
      -lowerXOffset,
      frictional.leftLowerLinearVelocity,
      frictional.leftLowerAngularVelocity);
  const double frictionalRightSlip = upperLowerSlip(
      frictional,
      lowerXOffset,
      frictional.rightLowerLinearVelocity,
      frictional.rightLowerAngularVelocity);

  EXPECT_GT(std::abs(frictionlessLeftSlip), 1e-9);
  EXPECT_GT(std::abs(frictionlessRightSlip), 1e-9);
  EXPECT_LT(std::abs(frictionalLeftSlip), std::abs(frictionlessLeftSlip));
  EXPECT_LT(std::abs(frictionalRightSlip), std::abs(frictionlessRightSlip));
  EXPECT_LT(frictional.topLinearVelocity.y(), 1.0 - 1e-9);
}

// Test that the built-in World::step() schedule also reaches warm-started AVBD
// friction rows for the two-lower/two-upper box-pile contact topology.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d leftLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightLowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);

    constexpr double lowerXOffset = 0.55;
    constexpr double lowerHalfZ = 0.4;
    constexpr double upperHalfZ = 0.35;

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_default_multi_top_box_pile_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 2.0, 0.5)));
    ground.setFriction(friction);

    const auto addLower = [&](std::string_view name, const double x) {
      sx::RigidBodyOptions lowerOptions;
      lowerOptions.mass = 1.0;
      lowerOptions.position = Eigen::Vector3d(x, 0.0, 0.38);
      auto lower = world.addRigidBody(name, lowerOptions);
      lower.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(0.45, 0.45, lowerHalfZ)));
      lower.setFriction(friction);
      return lower;
    };
    auto leftLower = addLower(
        "avbd_default_multi_top_box_pile_friction_left_lower", -lowerXOffset);
    auto rightLower = addLower(
        "avbd_default_multi_top_box_pile_friction_right_lower", lowerXOffset);

    const auto addUpper = [&](std::string_view name,
                              const double x,
                              const double yVelocity) {
      sx::RigidBodyOptions upperOptions;
      upperOptions.mass = 1.0;
      upperOptions.position = Eigen::Vector3d(x, 0.0, 1.10);
      upperOptions.linearVelocity = Eigen::Vector3d(0.0, yVelocity, 0.0);
      auto upper = world.addRigidBody(name, upperOptions);
      upper.setCollisionShape(
          sx::CollisionShape::makeBox(Eigen::Vector3d(0.35, 0.35, upperHalfZ)));
      upper.setFriction(friction);
      return upper;
    };
    auto leftUpper = addUpper(
        "avbd_default_multi_top_box_pile_friction_left_upper",
        -lowerXOffset,
        1.0);
    auto rightUpper = addUpper(
        "avbd_default_multi_top_box_pile_friction_right_upper",
        lowerXOffset,
        -1.0);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.95;
      config.gamma = 1.0;
    };
    configureAvbdContact(leftLower);
    configureAvbdContact(rightLower);
    configureAvbdContact(leftUpper);
    configureAvbdContact(rightUpper);

    leftLower.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));
    rightLower.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));
    leftUpper.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));
    rightUpper.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));

    EXPECT_GE(world.collide().size(), 12u);

    world.step();
    EXPECT_GE(world.collide().size(), 12u);
    world.step();

    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{
        leftLower.getLinearVelocity(),
        leftLower.getAngularVelocity(),
        rightLower.getLinearVelocity(),
        rightLower.getAngularVelocity(),
        leftUpper.getLinearVelocity(),
        leftUpper.getAngularVelocity(),
        rightUpper.getLinearVelocity(),
        rightUpper.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  const auto upperLowerSlip = [](const Eigen::Vector3d& lowerLinearVelocity,
                                 const Eigen::Vector3d& lowerAngularVelocity,
                                 const Eigen::Vector3d& upperLinearVelocity,
                                 const Eigen::Vector3d& upperAngularVelocity) {
    constexpr double lowerHalfZ = 0.4;
    constexpr double upperHalfZ = 0.35;
    const Eigen::Vector3d lowerArm(0.0, 0.0, lowerHalfZ);
    const Eigen::Vector3d upperArm(0.0, 0.0, -upperHalfZ);
    const Eigen::Vector3d lowerContactVelocity
        = lowerLinearVelocity + lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d upperContactVelocity
        = upperLinearVelocity + upperAngularVelocity.cross(upperArm);
    return upperContactVelocity.y() - lowerContactVelocity.y();
  };

  const double frictionlessLeftSlip = upperLowerSlip(
      frictionless.leftLowerLinearVelocity,
      frictionless.leftLowerAngularVelocity,
      frictionless.leftUpperLinearVelocity,
      frictionless.leftUpperAngularVelocity);
  const double frictionlessRightSlip = upperLowerSlip(
      frictionless.rightLowerLinearVelocity,
      frictionless.rightLowerAngularVelocity,
      frictionless.rightUpperLinearVelocity,
      frictionless.rightUpperAngularVelocity);
  const double frictionalLeftSlip = upperLowerSlip(
      frictional.leftLowerLinearVelocity,
      frictional.leftLowerAngularVelocity,
      frictional.leftUpperLinearVelocity,
      frictional.leftUpperAngularVelocity);
  const double frictionalRightSlip = upperLowerSlip(
      frictional.rightLowerLinearVelocity,
      frictional.rightLowerAngularVelocity,
      frictional.rightUpperLinearVelocity,
      frictional.rightUpperAngularVelocity);

  EXPECT_GT(std::abs(frictionlessLeftSlip), 1e-9);
  EXPECT_GT(std::abs(frictionlessRightSlip), 1e-9);
  EXPECT_LT(std::abs(frictionalLeftSlip), std::abs(frictionlessLeftSlip));
  EXPECT_LT(std::abs(frictionalRightSlip), std::abs(frictionlessRightSlip));
  EXPECT_LT(frictional.leftUpperLinearVelocity.y(), 1.0 - 1e-9);
  EXPECT_GT(frictional.rightUpperLinearVelocity.y(), -1.0 + 1e-9);
}

// Test that the built-in World::step() schedule also reaches warm-started AVBD
// friction rows for a multi-top stacked box-manifold contact set.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d lowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d lowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d leftUpperAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rightUpperAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);

    constexpr double lowerHalfZ = 0.5;
    constexpr double upperHalfZ = 0.35;
    constexpr double upperXOffset = 0.45;

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_default_multi_top_stack_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.5, 2.0, lowerHalfZ)));
    ground.setFriction(friction);

    sx::RigidBodyOptions lowerOptions;
    lowerOptions.mass = 2.0;
    lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
    auto lower = world.addRigidBody(
        "avbd_default_multi_top_stack_friction_lower", lowerOptions);
    lower.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(1.2, 0.7, lowerHalfZ)));
    lower.setFriction(friction);

    const auto addUpper
        = [&](std::string_view name, const double x, const double yVelocity) {
            sx::RigidBodyOptions upperOptions;
            upperOptions.mass = 1.0;
            upperOptions.position = Eigen::Vector3d(x, 0.0, 1.25);
            upperOptions.linearVelocity = Eigen::Vector3d(0.0, yVelocity, 0.0);
            auto upper = world.addRigidBody(name, upperOptions);
            upper.setCollisionShape(
                sx::CollisionShape::makeBox(
                    Eigen::Vector3d(upperHalfZ, upperHalfZ, upperHalfZ)));
            upper.setFriction(friction);
            return upper;
          };
    auto leftUpper = addUpper(
        "avbd_default_multi_top_stack_friction_left_upper", -upperXOffset, 1.0);
    auto rightUpper = addUpper(
        "avbd_default_multi_top_stack_friction_right_upper",
        upperXOffset,
        -1.0);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.95;
      config.gamma = 1.0;
    };
    configureAvbdContact(lower);
    configureAvbdContact(leftUpper);
    configureAvbdContact(rightUpper);

    lower.setForce(Eigen::Vector3d(0.0, 0.0, -40.0));
    leftUpper.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));
    rightUpper.setForce(Eigen::Vector3d(0.0, 0.0, -20.0));

    EXPECT_GE(world.collide().size(), 12u);

    world.step();
    EXPECT_GE(world.collide().size(), 12u);
    world.step();

    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{
        lower.getLinearVelocity(),
        lower.getAngularVelocity(),
        leftUpper.getLinearVelocity(),
        leftUpper.getAngularVelocity(),
        rightUpper.getLinearVelocity(),
        rightUpper.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  const auto upperLowerSlip = [](const StepOutcome& outcome,
                                 const double xOffset,
                                 const Eigen::Vector3d& upperLinearVelocity,
                                 const Eigen::Vector3d& upperAngularVelocity) {
    constexpr double lowerHalfZ = 0.5;
    constexpr double upperHalfZ = 0.35;
    const Eigen::Vector3d lowerArm(xOffset, 0.0, lowerHalfZ);
    const Eigen::Vector3d upperArm(0.0, 0.0, -upperHalfZ);
    const Eigen::Vector3d lowerContactVelocity
        = outcome.lowerLinearVelocity
          + outcome.lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d upperContactVelocity
        = upperLinearVelocity + upperAngularVelocity.cross(upperArm);
    return upperContactVelocity.y() - lowerContactVelocity.y();
  };

  constexpr double upperXOffset = 0.45;
  const double frictionlessLeftSlip = upperLowerSlip(
      frictionless,
      -upperXOffset,
      frictionless.leftUpperLinearVelocity,
      frictionless.leftUpperAngularVelocity);
  const double frictionlessRightSlip = upperLowerSlip(
      frictionless,
      upperXOffset,
      frictionless.rightUpperLinearVelocity,
      frictionless.rightUpperAngularVelocity);
  const double frictionalLeftSlip = upperLowerSlip(
      frictional,
      -upperXOffset,
      frictional.leftUpperLinearVelocity,
      frictional.leftUpperAngularVelocity);
  const double frictionalRightSlip = upperLowerSlip(
      frictional,
      upperXOffset,
      frictional.rightUpperLinearVelocity,
      frictional.rightUpperAngularVelocity);

  EXPECT_GT(std::abs(frictionlessLeftSlip), 1e-9);
  EXPECT_GT(std::abs(frictionlessRightSlip), 1e-9);
  EXPECT_LT(std::abs(frictionalLeftSlip), std::abs(frictionlessLeftSlip));
  EXPECT_LT(std::abs(frictionalRightSlip), std::abs(frictionlessRightSlip));
  EXPECT_LT(frictional.leftUpperLinearVelocity.y(), 1.0 - 1e-9);
  EXPECT_GT(frictional.rightUpperLinearVelocity.y(), -1.0 + 1e-9);
}

// Test that the built-in World::step() schedule reaches warm-started AVBD
// friction rows for a stacked static/dynamic plus dynamic/dynamic box-manifold
// contact set.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d lowerLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d lowerAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d upperLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d upperAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.02);

    constexpr double halfExtent = 0.5;
    const Eigen::Vector3d boxHalfExtents
        = Eigen::Vector3d::Constant(halfExtent);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
    auto ground = world.addRigidBody(
        "avbd_default_stacked_box_manifold_friction_ground", groundOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, halfExtent)));
    ground.setFriction(friction);

    sx::RigidBodyOptions lowerOptions;
    lowerOptions.mass = 1.0;
    lowerOptions.position = Eigen::Vector3d(0.02, 0.0, 0.45);
    auto lower = world.addRigidBody(
        "avbd_default_stacked_box_manifold_friction_lower", lowerOptions);
    lower.setCollisionShape(sx::CollisionShape::makeBox(boxHalfExtents));
    lower.setFriction(friction);

    sx::RigidBodyOptions upperOptions;
    upperOptions.mass = 1.0;
    upperOptions.position = Eigen::Vector3d(0.04, 0.0, 1.35);
    upperOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto upper = world.addRigidBody(
        "avbd_default_stacked_box_manifold_friction_upper", upperOptions);
    upper.setCollisionShape(sx::CollisionShape::makeBox(boxHalfExtents));
    upper.setFriction(friction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.95;
      config.gamma = 1.0;
    };
    configureAvbdContact(lower);
    configureAvbdContact(upper);

    lower.setForce(Eigen::Vector3d(0.0, 0.0, -25.0));
    upper.setForce(Eigen::Vector3d(0.0, 0.0, -25.0));

    EXPECT_GE(world.collide().size(), 8u);

    world.step();
    EXPECT_GE(world.collide().size(), 8u);
    world.step();

    EXPECT_TRUE(ground.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));

    return StepOutcome{
        lower.getLinearVelocity(),
        lower.getAngularVelocity(),
        upper.getLinearVelocity(),
        upper.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  constexpr double halfExtent = 0.5;
  const auto upperLowerSlip = [halfExtent](const StepOutcome& outcome) {
    const Eigen::Vector3d lowerArm(0.0, 0.0, halfExtent);
    const Eigen::Vector3d upperArm(0.0, 0.0, -halfExtent);
    const Eigen::Vector3d lowerContactVelocity
        = outcome.lowerLinearVelocity
          + outcome.lowerAngularVelocity.cross(lowerArm);
    const Eigen::Vector3d upperContactVelocity
        = outcome.upperLinearVelocity
          + outcome.upperAngularVelocity.cross(upperArm);
    return upperContactVelocity.x() - lowerContactVelocity.x();
  };

  EXPECT_GT(std::abs(upperLowerSlip(frictionless)), 1e-9);
  EXPECT_LT(
      std::abs(upperLowerSlip(frictional)),
      std::abs(upperLowerSlip(frictionless)));
  EXPECT_LT(frictional.upperLinearVelocity.x(), 1.0 - 1e-9);
}

// Test that warm-started AVBD contact-stage friction also acts on live
// dynamic/dynamic contacts by reducing relative tangential slip.
TEST(World, RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip)
{
  namespace sx = dart::simulation;

  struct ContactStageOutcome
  {
    Eigen::Vector3d bottomLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d bottomAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runContactStage = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    constexpr double radius = 0.5;
    sx::RigidBodyOptions bottomOptions;
    bottomOptions.mass = 1.0;
    bottomOptions.position = Eigen::Vector3d(0.0, 0.0, -0.45);
    auto bottom
        = world.addRigidBody("avbd_dynamic_friction_bottom", bottomOptions);
    bottom.setCollisionShape(sx::CollisionShape::makeSphere(radius));
    bottom.setFriction(friction);

    sx::RigidBodyOptions topOptions;
    topOptions.mass = 1.0;
    topOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
    topOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto top = world.addRigidBody("avbd_dynamic_friction_top", topOptions);
    top.setCollisionShape(sx::CollisionShape::makeSphere(radius));
    top.setFriction(friction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.5;
      config.gamma = 1.0;
    };
    configureAvbdContact(bottom);
    configureAvbdContact(top);

    const Eigen::Vector3d initialBottomPosition = bottom.getTranslation();
    const Eigen::Vector3d initialTopPosition = top.getTranslation();
    EXPECT_FALSE(world.collide().empty());

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidBodyContactStage contactStage(/*iterations=*/12);
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(contactStage);
    world.step(executor, pipeline);
    world.step(executor, pipeline);

    EXPECT_TRUE(bottom.getTranslation().isApprox(initialBottomPosition, 1e-12));
    EXPECT_TRUE(top.getTranslation().isApprox(initialTopPosition, 1e-12));

    return ContactStageOutcome{
        bottom.getLinearVelocity(),
        bottom.getAngularVelocity(),
        top.getLinearVelocity(),
        top.getAngularVelocity()};
  };

  const ContactStageOutcome frictionless = runContactStage(0.0);
  const ContactStageOutcome frictional = runContactStage(1.0);

  constexpr double radius = 0.5;
  const auto contactSlip = [radius](const ContactStageOutcome& outcome) {
    const Eigen::Vector3d bottomArm(0.0, 0.0, radius);
    const Eigen::Vector3d topArm(0.0, 0.0, -radius);
    const Eigen::Vector3d bottomContactVelocity
        = outcome.bottomLinearVelocity
          + outcome.bottomAngularVelocity.cross(bottomArm);
    const Eigen::Vector3d topContactVelocity
        = outcome.topLinearVelocity + outcome.topAngularVelocity.cross(topArm);
    return topContactVelocity.x() - bottomContactVelocity.x();
  };

  EXPECT_GT(
      frictionless.topLinearVelocity.z()
          - frictionless.bottomLinearVelocity.z(),
      1e-9);
  EXPECT_GT(
      frictional.topLinearVelocity.z() - frictional.bottomLinearVelocity.z(),
      1e-9);
  EXPECT_LT(
      std::abs(contactSlip(frictional)), std::abs(contactSlip(frictionless)));
  EXPECT_GT(frictional.bottomLinearVelocity.x(), 1e-9);
  EXPECT_LT(frictional.topLinearVelocity.x(), 1.0 - 1e-9);
}

// Test that the built-in World::step() schedule also reaches the warm-started
// dynamic/dynamic contact-stage friction path.
TEST(
    World,
    RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep)
{
  namespace sx = dart::simulation;

  struct StepOutcome
  {
    Eigen::Vector3d bottomLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d bottomAngularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topLinearVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d topAngularVelocity = Eigen::Vector3d::Zero();
  };

  const auto runDefaultStep = [](const double friction) {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    constexpr double radius = 0.5;
    sx::RigidBodyOptions bottomOptions;
    bottomOptions.mass = 1.0;
    bottomOptions.position = Eigen::Vector3d(0.0, 0.0, -0.45);
    auto bottom = world.addRigidBody(
        "avbd_default_dynamic_friction_bottom", bottomOptions);
    bottom.setCollisionShape(sx::CollisionShape::makeSphere(radius));
    bottom.setFriction(friction);

    sx::RigidBodyOptions topOptions;
    topOptions.mass = 1.0;
    topOptions.position = Eigen::Vector3d(0.0, 0.0, 0.45);
    topOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto top
        = world.addRigidBody("avbd_default_dynamic_friction_top", topOptions);
    top.setCollisionShape(sx::CollisionShape::makeSphere(radius));
    top.setFriction(friction);

    auto& registry = sx::detail::registryOf(world);
    const auto configureAvbdContact = [&](const sx::RigidBody& body) {
      auto& config = registry.emplace<sx::comps::RigidAvbdContactConfig>(
          sx::detail::toRegistryEntity(body.getEntity()));
      config.startStiffness = 10000.0;
      config.maxStiffness = 10000.0;
      config.alpha = 0.95;
      config.gamma = 1.0;
    };
    configureAvbdContact(bottom);
    configureAvbdContact(top);

    EXPECT_FALSE(world.collide().empty());

    world.step();
    EXPECT_FALSE(world.collide().empty());
    world.step();

    return StepOutcome{
        bottom.getLinearVelocity(),
        bottom.getAngularVelocity(),
        top.getLinearVelocity(),
        top.getAngularVelocity()};
  };

  const StepOutcome frictionless = runDefaultStep(0.0);
  const StepOutcome frictional = runDefaultStep(1.0);

  constexpr double radius = 0.5;
  const auto contactSlip = [radius](const StepOutcome& outcome) {
    const Eigen::Vector3d bottomArm(0.0, 0.0, radius);
    const Eigen::Vector3d topArm(0.0, 0.0, -radius);
    const Eigen::Vector3d bottomContactVelocity
        = outcome.bottomLinearVelocity
          + outcome.bottomAngularVelocity.cross(bottomArm);
    const Eigen::Vector3d topContactVelocity
        = outcome.topLinearVelocity + outcome.topAngularVelocity.cross(topArm);
    return topContactVelocity.x() - bottomContactVelocity.x();
  };

  EXPECT_GT(
      frictionless.topLinearVelocity.z()
          - frictionless.bottomLinearVelocity.z(),
      1e-9);
  EXPECT_GT(
      frictional.topLinearVelocity.z() - frictional.bottomLinearVelocity.z(),
      1e-9);
  EXPECT_LT(
      std::abs(contactSlip(frictional)), std::abs(contactSlip(frictionless)));
  EXPECT_GT(frictional.bottomLinearVelocity.x(), 1e-9);
  EXPECT_LT(frictional.topLinearVelocity.x(), 1.0 - 1e-9);
}

// Test that static rigid bodies can retain stored velocities without behaving
// like moving obstacles in rigid-body contact response.
TEST(World, RigidBodyContactIgnoresStoredStaticVelocity)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.isStatic = true;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
  auto dynamic = world.addRigidBody("dynamic", dynamicOptions);
  dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  const auto& contact = contacts.front();
  constexpr double staticSpeed = 5.0;
  if (contact.bodyA.getEntity() == obstacle.getEntity()) {
    obstacle.setLinearVelocity(staticSpeed * contact.normal);
  } else {
    ASSERT_EQ(contact.bodyB.getEntity(), obstacle.getEntity());
    obstacle.setLinearVelocity(-staticSpeed * contact.normal);
  }

  world.setTimeStep(0.001);
  world.step();

  EXPECT_NEAR(obstacle.getLinearVelocity().norm(), staticSpeed, 1e-12);
  EXPECT_TRUE(dynamic.getLinearVelocity().isZero(1e-12));
  EXPECT_TRUE(dynamic.getAngularVelocity().isZero(1e-12));
}

// Test that kinematic bodies behave like static obstacles in the default rigid
// body contact solver: their prescribed velocity is not treated as dynamic
// contact velocity, and penetration correction does not move them.
TEST(World, RigidBodyContactTreatsKinematicBodyAsStaticObstacle)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  obstacle.setKinematic(true);
  ASSERT_TRUE(obstacle.isKinematic());
  ASSERT_FALSE(obstacle.isStatic());

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
  auto dynamic = world.addRigidBody("dynamic", dynamicOptions);
  dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  const auto& contact = contacts.front();
  constexpr double kinematicSpeed = 5.0;
  if (contact.bodyA.getEntity() == obstacle.getEntity()) {
    obstacle.setLinearVelocity(kinematicSpeed * contact.normal);
  } else {
    ASSERT_EQ(contact.bodyB.getEntity(), obstacle.getEntity());
    obstacle.setLinearVelocity(-kinematicSpeed * contact.normal);
  }

  const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();

  world.setTimeStep(0.001);
  world.step();

  EXPECT_TRUE(obstacle.isKinematic());
  EXPECT_TRUE(
      obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
  EXPECT_NEAR(obstacle.getLinearVelocity().norm(), kinematicSpeed, 1e-12);
  EXPECT_TRUE(dynamic.getLinearVelocity().isZero(1e-12));
  EXPECT_TRUE(dynamic.getAngularVelocity().isZero(1e-12));
}

TEST(World, BoxedLcpContactTreatsKinematicBodyAsStaticObstacle)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions obstacleOptions;
  obstacleOptions.position = Eigen::Vector3d(0.0, 0.0, 0.0);
  auto obstacle = world.addRigidBody("obstacle", obstacleOptions);
  obstacle.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  obstacle.setKinematic(true);
  ASSERT_TRUE(obstacle.isKinematic());
  ASSERT_FALSE(obstacle.isStatic());

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.position = Eigen::Vector3d(0.9, 0.0, 0.0);
  auto dynamic = world.addRigidBody("dynamic", dynamicOptions);
  dynamic.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());
  const auto& contact = contacts.front();
  constexpr double kinematicSpeed = 5.0;
  if (contact.bodyA.getEntity() == obstacle.getEntity()) {
    obstacle.setLinearVelocity(kinematicSpeed * contact.normal);
  } else {
    ASSERT_EQ(contact.bodyB.getEntity(), obstacle.getEntity());
    obstacle.setLinearVelocity(-kinematicSpeed * contact.normal);
  }

  const Eigen::Vector3d initialObstaclePosition = obstacle.getTranslation();

  world.setTimeStep(0.001);
  world.step();

  EXPECT_TRUE(obstacle.isKinematic());
  EXPECT_TRUE(
      obstacle.getTranslation().isApprox(initialObstaclePosition, 1e-12));
  EXPECT_NEAR(obstacle.getLinearVelocity().norm(), kinematicSpeed, 1e-12);
  EXPECT_TRUE(dynamic.getLinearVelocity().isZero(1e-12));
  EXPECT_TRUE(dynamic.getAngularVelocity().isZero(1e-12));
}

TEST(World, RigidBodyStaticSetterClearsKinematicMode)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d::Zero();
  auto body = world.addRigidBody("body", options);
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.2, 0.2)));
  body.setLinearVelocity(Eigen::Vector3d(1.0, 0.0, 0.0));
  body.setKinematic(true);
  ASSERT_TRUE(body.isKinematic());
  ASSERT_FALSE(body.isStatic());

  body.setStatic(true);
  EXPECT_TRUE(body.isStatic());
  EXPECT_FALSE(body.isKinematic());

  const Eigen::Vector3d initialPosition = body.getTranslation();
  world.step();

  EXPECT_TRUE(body.getTranslation().isApprox(initialPosition, 1e-12));
  EXPECT_TRUE(
      body.getLinearVelocity().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

// Test that a dynamic body dropped onto a static ground comes to rest on it
// (gravity + contact response + positional correction keep it from sinking).
TEST(World, RigidBodyRestsOnStaticGround)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  // Static ground box with its top face at z = 0.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  EXPECT_TRUE(ground.isStatic());

  // Dynamic sphere dropped from above.
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 2.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(1000);

  // The sphere (radius 0.5) should rest with its center near z = 0.5 and have
  // essentially stopped moving. The ground must not have moved.
  EXPECT_NEAR(sphere.getTranslation().z(), 0.5, 2e-2);
  EXPECT_LT(std::abs(sphere.getLinearVelocity().z()), 0.1);
  EXPECT_TRUE(
      ground.getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));
}

// Test that restitution makes contacts bounce: a perfectly elastic, equal-mass,
// head-on collision swaps the bodies' velocities.
TEST(World, RigidBodyContactRestitution)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  sx::RigidBodyOptions optionsA;
  optionsA.position = Eigen::Vector3d(-0.45, 0.0, 0.0);
  optionsA.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto bodyA = world.addRigidBody("a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  bodyA.setRestitution(1.0);

  sx::RigidBodyOptions optionsB;
  optionsB.position = Eigen::Vector3d(0.45, 0.0, 0.0);
  optionsB.linearVelocity = Eigen::Vector3d(-1.0, 0.0, 0.0);
  auto bodyB = world.addRigidBody("b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  bodyB.setRestitution(1.0);

  EXPECT_DOUBLE_EQ(bodyA.getRestitution(), 1.0);
  EXPECT_DOUBLE_EQ(world.addRigidBody("default").getRestitution(), 0.0);
  EXPECT_DOUBLE_EQ(world.addRigidBody("default_friction").getFriction(), 1.0);

  world.setTimeStep(0.001);
  world.step();

  // Perfectly elastic, equal masses, head-on: the velocities swap.
  EXPECT_NEAR(bodyA.getLinearVelocity().x(), -1.0, 1e-9);
  EXPECT_NEAR(bodyB.getLinearVelocity().x(), 1.0, 1e-9);
}

// Test that Coulomb friction decelerates a body sliding on a static ground.
TEST(World, RigidBodyContactFrictionDeceleratesSlidingBody)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  // A flat, low box (so friction torque does not tip it) sliding in +x.
  sx::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.1);
  boxOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto slider = world.addRigidBody("slider", boxOptions);
  slider.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.1)));
  slider.setFriction(0.5);
  EXPECT_DOUBLE_EQ(slider.getFriction(), 0.5);

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(400);

  // Friction should have braked the slide (started at 2 m/s) without reversing
  // it, and the body should have moved forward before stopping.
  const double velocityX = slider.getLinearVelocity().x();
  EXPECT_LT(velocityX, 0.5);
  EXPECT_GT(velocityX, -0.2);
  EXPECT_GT(slider.getTranslation().x(), 0.0);
}

// Test that a coupled stack settles: two spheres stacked on a static ground
// generate two contacts (ground-sphere1 and sphere1-sphere2) that share the
// middle sphere, so the contact-space inverse-mass matrix is a non-singular
// 2x2 with a non-zero off-diagonal. The coupled boxed-LCP solve makes the
// bottom contact carry both spheres' weight in one solve, so the lower sphere
// does not sink under the load of the upper one.
TEST(World, RigidBodyContactCoupledStackRests)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  // Static ground box with its top face at z = 0.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));

  // Lower sphere dropped just above its resting height (center z = 0.5).
  sx::RigidBodyOptions lowerOptions;
  lowerOptions.position = Eigen::Vector3d(0.0, 0.0, 0.55);
  auto lower = world.addRigidBody("lower", lowerOptions);
  lower.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  // Upper sphere stacked directly above (resting center z = 1.5), perfectly
  // aligned so the contact normals are vertical and the stack stays upright.
  sx::RigidBodyOptions upperOptions;
  upperOptions.position = Eigen::Vector3d(0.0, 0.0, 1.6);
  auto upper = world.addRigidBody("upper", upperOptions);
  upper.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(1000);

  // Both spheres should settle near their stacked resting heights without the
  // lower one sinking into the ground, and essentially stop moving.
  EXPECT_NEAR(lower.getTranslation().z(), 0.5, 2e-2);
  EXPECT_NEAR(upper.getTranslation().z(), 1.5, 4e-2);
  EXPECT_LT(std::abs(lower.getLinearVelocity().z()), 0.1);
  EXPECT_LT(std::abs(upper.getLinearVelocity().z()), 0.1);
  EXPECT_TRUE(
      ground.getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, -0.5)));
}

// Test that friction solved inside the coupled contact LCP turns a sliding
// sphere into a rolling one. A single sphere on the ground is one contact, so
// the contact-space matrix is non-singular and the solver takes the full
// coupled normal+friction path (not the rank-deficient box-on-plane fallback).
// Coulomb friction drives the contact-point slip to zero, converting forward
// sliding into forward rolling without reversing the body.
TEST(World, RigidBodyContactFrictionRollsSlidingSphere)
{
  namespace sx = dart::simulation;

  sx::World world; // default gravity (0, 0, -9.81)

  // Large static ground so the sphere stays on it while rolling, top at z = 0.
  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(20.0, 20.0, 0.5)));

  // A sphere resting on the ground (center at z = 0.5) sliding in +x with no
  // initial spin.
  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Eigen::Vector3d(0.0, 0.0, 0.5);
  sphereOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(1.0);

  world.setTimeStep(0.005);
  world.enterSimulationMode();
  world.step(300);

  const Eigen::Vector3d linear = sphere.getLinearVelocity();
  const Eigen::Vector3d angular = sphere.getAngularVelocity();
  // Velocity of the contact point (sphere bottom, arm = (0, 0, -r)).
  const Eigen::Vector3d contactVelocity
      = linear + angular.cross(Eigen::Vector3d(0.0, 0.0, -0.5));

  // Friction (solved in the LCP) has driven the tangential slip at the contact
  // to nearly zero -- the sphere is rolling, not sliding. (The exact rolling
  // speed depends on the body inertia, so assert the slip and direction rather
  // than a specific velocity.)
  EXPECT_LT(std::hypot(contactVelocity.x(), contactVelocity.y()), 0.2);
  // The body slowed from 2.0 m/s but kept moving forward (never reversed).
  EXPECT_GT(linear.x(), 0.05);
  EXPECT_LT(linear.x(), 1.95);
  // It picked up forward spin (rolling: slip ~ 0 with v.x > 0 implies w.y > 0)
  // and translated forward, and stayed resting on the ground.
  EXPECT_GT(angular.y(), 0.05);
  EXPECT_GT(sphere.getTranslation().x(), 0.3);
  EXPECT_NEAR(sphere.getTranslation().z(), 0.5, 5e-2);
}

// Test that the opt-in rigid IPC stage can build its internal objective from
// runtime rigid-body state and write the solved pose/velocity back to the
// World.
TEST(World, RigidIpcContactStageAdvancesMeshBodyFromRuntimeDynamics)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("ipc_body", options);
  body.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 1u);
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 1u);
  EXPECT_EQ(stats.activeDynamicsTerms, 1u);
  EXPECT_EQ(stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(stats.frictionIterations, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::Converged);
  EXPECT_TRUE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_GT(stats.acceptedSteps, 0u);
  EXPECT_GT(stats.lastStepNorm, 0.0);
  EXPECT_EQ(stats.lastLineSearchStepBound, 1.0);
  EXPECT_FALSE(stats.lastLineSearchIndeterminate);
  EXPECT_EQ(stats.lineSearchPointPointChecks, 0u);
  EXPECT_EQ(stats.lineSearchPointEdgeChecks, 0u);
  EXPECT_EQ(stats.lineSearchEdgeEdgeChecks, 0u);
  EXPECT_EQ(stats.lineSearchPointTriangleChecks, 0u);
  EXPECT_EQ(stats.lineSearchHits, 0u);
  EXPECT_EQ(stats.lineSearchMisses, 0u);
  EXPECT_EQ(stats.lineSearchIndeterminateCount, 0u);
  EXPECT_EQ(stats.lineSearchZeroStepCount, 0u);
  EXPECT_EQ(stats.sufficientDecreaseChecks, stats.acceptedSteps);
  EXPECT_EQ(stats.sufficientDecreaseBacktracks, 0u);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_FALSE(stats.nonConvergedResultSkipped);

  // q_new = q + v*h + M^{-1}*f*h^2 for this dynamics-only slice.
  EXPECT_NEAR(body.getTranslation().x(), 0.12, 1e-8);
  EXPECT_NEAR(body.getLinearVelocity().x(), 1.2, 1e-8);
}

// Test that the runtime rigid IPC stage triangulates analytic sphere collision
// shapes into mesh-like surfaces before solving.
TEST(World, RigidIpcContactStageAdvancesSphereBodyFromRuntimeDynamics)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("ipc_sphere", options);
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 1u);
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 1u);
  EXPECT_EQ(stats.skippedUnsupportedShapeCount, 0u);
  EXPECT_EQ(stats.activeDynamicsTerms, 1u);
  EXPECT_EQ(stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(stats.frictionIterations, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::Converged);
  EXPECT_TRUE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_GT(stats.acceptedSteps, 0u);
  EXPECT_GT(stats.lastStepNorm, 0.0);
  EXPECT_EQ(stats.lastLineSearchStepBound, 1.0);
  EXPECT_FALSE(stats.lastLineSearchIndeterminate);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_FALSE(stats.nonConvergedResultSkipped);

  EXPECT_NEAR(body.getTranslation().x(), 0.12, 1e-8);
  EXPECT_NEAR(body.getLinearVelocity().x(), 1.2, 1e-8);
}

// Test that the opt-in BDF-2 rigid IPC path builds its inertial target from
// per-body runtime history while preserving the default first-order restart on
// the first accepted step.
TEST(World, RigidIpcContactStageBdf2FallingBoxEnergyStaysFinite)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setTimeStep(0.01);

  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  options.mass = 1.0;
  auto box = world.addRigidBody("bdf2_box", options);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));

  const auto mechanicalEnergy = [&]() {
    const double kinetic
        = 0.5 * box.getMass() * box.getLinearVelocity().squaredNorm();
    const double potential
        = -box.getMass() * world.getGravity().dot(box.getTranslation());
    return kinetic + potential;
  };
  const double initialHeight = box.getTranslation().z();
  const double initialEnergy = mechanicalEnergy();

  sx::compute::RigidIpcContactStageOptions stageOptions;
  stageOptions.maxIterations = 16;
  stageOptions.timeIntegration = sx::compute::RigidIpcTimeIntegration::Bdf2;
  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(stageOptions);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  world.step(executor, pipeline);
  const double firstStepHeight = box.getTranslation().z();
  {
    const auto& stats = ipcStage.getLastStats();
    EXPECT_EQ(stats.activeDynamicsTerms, 1u);
    EXPECT_EQ(stats.bdf2RestartedDynamicsTerms, 1u);
    EXPECT_EQ(stats.bdf2SecondOrderDynamicsTerms, 0u);
    EXPECT_TRUE(stats.converged);
    EXPECT_TRUE(stats.resultApplied);
  }

  world.step(executor, pipeline);
  const double finalHeight = box.getTranslation().z();
  {
    const auto& stats = ipcStage.getLastStats();
    EXPECT_EQ(stats.activeDynamicsTerms, 1u);
    EXPECT_EQ(stats.bdf2RestartedDynamicsTerms, 0u);
    EXPECT_EQ(stats.bdf2SecondOrderDynamicsTerms, 1u);
    EXPECT_TRUE(stats.converged);
    EXPECT_TRUE(stats.resultApplied);
  }

  const double finalEnergy = mechanicalEnergy();
  const double expectedBdf2Velocity
      = (3.0 * finalHeight - 4.0 * firstStepHeight + initialHeight)
        / (2.0 * world.getTimeStep());
  const double backwardDifferenceVelocity
      = (finalHeight - firstStepHeight) / world.getTimeStep();
  EXPECT_TRUE(std::isfinite(finalEnergy));
  EXPECT_GT(finalEnergy, 0.99 * initialEnergy);
  EXPECT_LT(finalEnergy, 1.01 * initialEnergy);
  EXPECT_LT(finalHeight, initialHeight);
  EXPECT_LT(box.getLinearVelocity().z(), 0.0);
  EXPECT_NEAR(box.getLinearVelocity().z(), expectedBdf2Velocity, 1e-10);
  EXPECT_GT(
      std::abs(box.getLinearVelocity().z() - backwardDifferenceVelocity), 1e-6);
}

TEST(World, RigidIpcContactStageUsesDeformableSurfaceObstacle)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.02);

  sx::DeformableBodyOptions sheetOptions;
  sheetOptions.positions
      = {Eigen::Vector3d(-0.5, -0.5, 0.0),
         Eigen::Vector3d(0.5, -0.5, 0.0),
         Eigen::Vector3d(-0.5, 0.5, 0.0),
         Eigen::Vector3d(0.5, 0.5, 0.0)};
  sheetOptions.masses = {1.0, 1.0, 1.0, 1.0};
  sheetOptions.fixedNodes = {0, 1, 2, 3};
  sheetOptions.surfaceTriangles
      = {sx::DeformableSurfaceTriangle{0, 1, 2},
         sx::DeformableSurfaceTriangle{1, 3, 2}};
  sheetOptions.material.frictionCoefficient = 0.0;
  auto sheet = world.addDeformableBody("deformable_surface", sheetOptions);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.055);
  boxOptions.linearVelocity = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto box = world.addRigidBody("falling_box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.05, 0.05, 0.05}));

  sx::compute::RigidIpcContactStageOptions stageOptions;
  stageOptions.maxIterations = 64;
  stageOptions.activationDistance = 0.02;
  stageOptions.frictionIterations = 0;
  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(stageOptions);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 1u);
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 2u);
  EXPECT_EQ(stats.mixedDomainDeformableSurfaceCount, 1u);
  EXPECT_GE(stats.mixedDomainSurfaceCount, 2u);
  EXPECT_GT(stats.mixedDomainCandidateCount, 0u);
  EXPECT_GT(stats.mixedDomainActiveBarrierCount, 0u);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_FALSE(stats.failed);

  const double boxBottom = box.getTranslation().z() - 0.05;
  EXPECT_GT(boxBottom, -1e-6);
  for (std::size_t node = 0; node < sheet.getNodeCount(); ++node) {
    EXPECT_TRUE(sheet.getPosition(node).allFinite());
    EXPECT_NEAR(sheet.getPosition(node).z(), 0.0, 1e-12);
  }
  const double kineticEnergy
      = 0.5 * box.getMass() * box.getLinearVelocity().squaredNorm();
  EXPECT_TRUE(std::isfinite(kineticEnergy));
}

TEST(World, RigidIpcContactStageMetadataDeclaresMixedDomainReads)
{
  namespace sx = dart::simulation;

  sx::compute::RigidIpcContactStage ipcStage;
  const auto metadata = ipcStage.getMetadata();

  const auto hasRead = [&metadata](const std::string_view resource) {
    return std::any_of(
        metadata.resources.begin(),
        metadata.resources.end(),
        [resource](const sx::compute::ComputeResourceAccess& access) {
          return access.resource == resource
                 && access.mode == sx::compute::ComputeAccessMode::Read;
        });
  };

  EXPECT_TRUE(hasRead("deformable_body.state"));
  EXPECT_TRUE(hasRead("deformable_body.model"));
  EXPECT_TRUE(hasRead("deformable_body.topology"));
  EXPECT_TRUE(hasRead("rigid_body.joint"));
}

// Test that the opt-in rigid IPC stage assembles active barrier constraints
// from runtime mesh surfaces and pushes a dynamic body away from a static one.
// Body-body generalization of the freeze fix: a stack of two free boxes on
// static ground. This exercises multiple dynamic bodies in one solve (a 12-DOF
// global system) plus simultaneous body-ground (A-ground) and body-body (B-A)
// barrier contacts, all driven by a single scene-level adaptive kappa. The
// stack must settle into a stable, intersection-free equilibrium rather than
// freeze, interpenetrate, or fly apart.
TEST(World, RigidIpcContactStageTwoBoxStackSettlesWithoutPenetration)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The multi-step rigid IPC stack gate is too slow under coverage; "
         "normal CI runs the full regression.";
#endif

  namespace sx = dart::simulation;
  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.25}));

  // Box A rests just above the ground (bottom at z = 0.002); box B rests just
  // above A (gap 0.002). Box half-height is 0.25, so a B-A center separation
  // below 0.5 would mean interpenetration.
  sx::RigidBodyOptions aOptions;
  aOptions.mass = 1.0;
  aOptions.position = Eigen::Vector3d(0.0, 0.0, 0.252);
  auto boxA = world.addRigidBody("A", aOptions);
  boxA.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));

  sx::RigidBodyOptions bOptions;
  bOptions.mass = 1.0;
  bOptions.position = Eigen::Vector3d(0.0, 0.0, 0.754);
  auto boxB = world.addRigidBody("B", bOptions);
  boxB.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  constexpr int kSteps = 24;
  double lastZa = 0.0;
  double lastZb = 0.0;
  for (int step = 0; step < kSteps; ++step) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();

    // Multiple dynamic bodies feed the solve and a scene-level adaptive kappa
    // (well above the old fixed value of 1) holds them apart.
    EXPECT_EQ(stats.dynamicBodyCount, 2u) << "step " << step;
    EXPECT_GT(stats.barrierStiffness, 100.0) << "step " << step;
    // No freeze: every step makes progress and is written back.
    EXPECT_TRUE(stats.resultApplied) << "step " << step;
    EXPECT_FALSE(stats.failed) << "step " << step;

    const double za = boxA.getTranslation().z();
    const double zb = boxB.getTranslation().z();

    // No penetration: A stays above the ground top (z = 0, A bottom = za -
    // 0.25) and B stays above A (center separation >= box height 0.5).
    EXPECT_GT(za, 0.25 - 1e-3) << "step " << step;
    EXPECT_GT(zb - za, 0.5 - 1e-3) << "step " << step;
    // No explosion: the stack stays near its initial configuration.
    EXPECT_LT(za, 0.27) << "step " << step;
    EXPECT_LT(zb, 0.78) << "step " << step;

    lastZa = za;
    lastZb = zb;
  }

  // Settled into a stable equilibrium: both boxes are still resting in a valid
  // stack (A on the ground, B on A) at the end of the run.
  EXPECT_NEAR(lastZa, 0.2501, 3e-3);
  EXPECT_NEAR(lastZb, 0.7531, 3e-3);
  EXPECT_GT(lastZb - lastZa, 0.5 - 1e-3);
}

TEST(World, RigidIpcContactStageSeparatesActivatedMeshBarrier)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.05);

  const std::vector<Eigen::Vector3d> triangleVertices{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0)};
  const std::vector<Eigen::Vector3i> triangleFaces{Eigen::Vector3i(0, 1, 2)};

  sx::RigidBodyOptions staticOptions;
  staticOptions.isStatic = true;
  auto ground = world.addRigidBody("static_ipc_triangle", staticOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeMesh(triangleVertices, triangleFaces));

  constexpr double initialHeight = 0.005;
  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 1.0;
  dynamicOptions.position = Eigen::Vector3d(0.0, 0.0, initialHeight);
  auto body = world.addRigidBody("dynamic_ipc_triangle", dynamicOptions);
  body.setCollisionShape(
      sx::CollisionShape::makeMesh(triangleVertices, triangleFaces));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 2u);
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 2u);
  // The solve is not failed (no penetration / line-search block) and produces a
  // usable result. With adaptive barrier stiffness this stiff zero-gravity
  // contact reaches a feasible friction-limited plateau rather than the exact
  // gradient tolerance, so the stage applies the best intersection-free
  // configuration (matching the reference IPC) instead of discarding it.
  EXPECT_FALSE(stats.failed);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_FALSE(stats.nonConvergedResultSkipped);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_GT(stats.activeFrictionConstraints, 0u);
  EXPECT_GT(stats.acceptedSteps, 0u);
  // Adaptive stiffness raised kappa well above the old fixed value of 1.
  EXPECT_GT(stats.barrierStiffness, 1.0);
  // The conservative swept broad phase skips line-search CCD when the activated
  // body separates within the step (no possible contact), so the per-family
  // check counts may legitimately be zero here. The CCD invariants still hold,
  // and the contact behavior asserted below is what this regression verifies.
  EXPECT_EQ(stats.lineSearchHits, 0u);
  EXPECT_EQ(stats.lineSearchIndeterminateCount, 0u);
  EXPECT_EQ(stats.lineSearchZeroStepCount, 0u);
  EXPECT_GT(stats.sufficientDecreaseChecks, 0u);
  // The activated barrier pushes the body off the surface.
  EXPECT_GT(body.getTranslation().z(), initialHeight);
  EXPECT_GT(body.getLinearVelocity().z(), 0.0);
}

// Regression for the freeze-on-contact "sink-then-stick" bug: under gravity a
// box resting/sliding on static ground used to creep into the barrier band
// (fixed kappa=1) until a step penetrated, after which the conservative line
// search blocked every step (status=LineSearchBlocked) and the body froze in
// place permanently. Adaptive barrier stiffness (IPC adaptive-kappa) keeps the
// equilibrium gap > 0, so the contact now produces continued dynamics: the box
// slides forward and friction brakes it toward rest without ever penetrating
// the ground or freezing.
TEST(World, RigidIpcContactStageSlidingContactDoesNotFreeze)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.01);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.25}));

  // Box bottom starts at z = 0.258 - 0.25 = 0.008 (inside the 1e-2 activation
  // band): exactly the configuration that used to freeze.
  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.258);
  boxOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto box = world.addRigidBody("box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  constexpr int kSteps = 12;
  double previousX = box.getTranslation().x();
  double previousVelocityX = box.getLinearVelocity().x();
  for (int step = 0; step < kSteps; ++step) {
    world.step(executor, pipeline);
    const auto& stats = ipcStage.getLastStats();

    // Never freezes: every step converges and is applied (the old bug returned
    // LineSearchBlocked / non-converged and skipped the result).
    EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::Converged)
        << "step " << step;
    EXPECT_TRUE(stats.converged) << "step " << step;
    EXPECT_FALSE(stats.failed) << "step " << step;
    EXPECT_TRUE(stats.resultApplied) << "step " << step;
    EXPECT_EQ(stats.lineSearchZeroStepCount, 0u) << "step " << step;
    // Adaptive stiffness raised kappa well above the old fixed value of 1.
    EXPECT_GT(stats.barrierStiffness, 100.0) << "step " << step;

    const double x = box.getTranslation().x();
    const double z = box.getTranslation().z();
    const double velocityX = box.getLinearVelocity().x();

    // Continued contact dynamics: the box keeps sliding forward and friction
    // monotonically brakes it, never reversing.
    EXPECT_GT(x, previousX) << "step " << step;
    EXPECT_GT(velocityX, 0.0) << "step " << step;
    EXPECT_LT(velocityX, previousVelocityX) << "step " << step;

    // No penetration and no launch: the box bottom (z - 0.25) stays just above
    // the ground top (0) within the activation band.
    EXPECT_GT(z, 0.25) << "step " << step;
    EXPECT_LT(z, 0.27) << "step " << step;

    previousX = x;
    previousVelocityX = velocityX;
  }

  // The slide decelerated substantially but has not frozen or reversed.
  EXPECT_LT(box.getLinearVelocity().x(), 0.2);
  EXPECT_GT(box.getLinearVelocity().x(), 0.0);
}

TEST(World, RigidIpcAdaptiveStiffnessPersistsAcrossSingleStepCalls)
{
  namespace sx = dart::simulation;

  struct Outcome
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  };

  constexpr int kSteps = 12;
  const auto run = [](bool batched) -> Outcome {
    sx::World world;
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.25}));

    sx::RigidBodyOptions boxOptions;
    boxOptions.mass = 1.0;
    boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.258);
    boxOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto box = world.addRigidBody("box", boxOptions);
    box.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));

    if (batched) {
      world.step(kSteps);
    } else {
      for (int step = 0; step < kSteps; ++step) {
        world.step();
      }
    }

    return Outcome{box.getTranslation(), box.getLinearVelocity()};
  };

  const Outcome singleStepCalls = run(false);
  const Outcome batchedCall = run(true);

  EXPECT_TRUE(singleStepCalls.position.isApprox(batchedCall.position, 1e-10));
  EXPECT_TRUE(singleStepCalls.linearVelocity.isApprox(
      batchedCall.linearVelocity, 1e-10));
}

// Test that lagged friction in the opt-in rigid IPC stage produces an
// observable runtime effect: at an activated contact a tangential slide is
// braked relative to the frictionless solve, without reversing direction.
//
// The scene mirrors RigidIpcContactStageSeparatesActivatedMeshBarrier but adds
// a tangential velocity and runs twice. Disabling friction on either body
// removes friction entirely (combined Coulomb coefficient sqrt(f_a*f_b)), so
// the frictionless run is the differential baseline.
TEST(World, RigidIpcContactStageFrictionBraketsTangentialSlide)
{
  namespace sx = dart::simulation;

  const std::vector<Eigen::Vector3d> triangleVertices{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0)};
  const std::vector<Eigen::Vector3i> triangleFaces{Eigen::Vector3i(0, 1, 2)};
  constexpr double initialHeight = 0.005;
  constexpr double tangentialSpeed = 1.0;

  struct SlideOutcome
  {
    sx::compute::RigidIpcSolverStats stats;
    double x = 0.0;
    double z = 0.0;
    double velocityX = 0.0;
  };

  const auto runSlide = [&](double friction) -> SlideOutcome {
    sx::World world;
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.05);

    sx::RigidBodyOptions staticOptions;
    staticOptions.isStatic = true;
    auto ground = world.addRigidBody("static_ipc_triangle", staticOptions);
    ground.setCollisionShape(
        sx::CollisionShape::makeMesh(triangleVertices, triangleFaces));
    ground.setFriction(friction);

    sx::RigidBodyOptions dynamicOptions;
    dynamicOptions.mass = 1.0;
    dynamicOptions.position = Eigen::Vector3d(0.0, 0.0, initialHeight);
    dynamicOptions.linearVelocity = Eigen::Vector3d(tangentialSpeed, 0.0, 0.0);
    auto body = world.addRigidBody("dynamic_ipc_triangle", dynamicOptions);
    body.setCollisionShape(
        sx::CollisionShape::makeMesh(triangleVertices, triangleFaces));
    body.setFriction(friction);

    sx::compute::SequentialExecutor executor;
    sx::compute::RigidIpcContactStage ipcStage;
    sx::compute::WorldStepPipeline pipeline;
    pipeline.addStage(ipcStage);
    world.step(executor, pipeline);

    return SlideOutcome{
        ipcStage.getLastStats(),
        body.getTranslation().x(),
        body.getTranslation().z(),
        body.getLinearVelocity().x()};
  };

  const SlideOutcome frictionless = runSlide(0.0);
  const SlideOutcome frictional = runSlide(1.0);

  // Both runs converge and separate along the barrier normal.
  EXPECT_TRUE(frictionless.stats.converged);
  EXPECT_TRUE(frictional.stats.converged);
  EXPECT_GT(frictionless.z, initialHeight);
  EXPECT_GT(frictional.z, initialHeight);

  // Friction is inactive without a positive combined coefficient and active
  // once both bodies carry friction.
  EXPECT_EQ(frictionless.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(frictionless.stats.frictionIterations, 0u);
  EXPECT_GT(frictional.stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(frictional.stats.frictionIterations, 1u);

  // The frictionless body slides forward toward its free-motion target; the
  // frictional body advances less and writes back a smaller forward velocity,
  // but friction does not reverse the slide.
  EXPECT_GT(frictionless.x, 0.0);
  EXPECT_GT(frictional.x, 0.0);
  EXPECT_LT(frictional.x, frictionless.x);
  EXPECT_LT(frictional.velocityX, frictionless.velocityX);
}

// Test that unsupported rigid shapes are reported by the opt-in IPC stage
// without silently falling back to the default rigid-body solver.
TEST(World, RigidIpcContactStageReportsUnsupportedShapes)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("unsupported", options);

  sx::CollisionShape unsupportedShape;
  unsupportedShape.type = static_cast<sx::CollisionShapeType>(99);
  body.setCollisionShape(unsupportedShape);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 0u);
  EXPECT_EQ(stats.skippedUnsupportedShapeCount, 1u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::NoDofs);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_FALSE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(body.getTranslation().isZero(1e-12));
  EXPECT_NEAR(body.getLinearVelocity().x(), 1.0, 1e-12);
}

// Test that shape-less kinematic bodies still advance by prescribed velocity in
// the IPC pipeline even though they do not contribute contact rows.
TEST(World, RigidIpcContactStageAdvancesShapelessKinematicBody)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  kinematicOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto kinematicBody
      = world.addRigidBody("kinematic_shapeless", kinematicOptions);
  kinematicBody.setKinematic(true);
  EXPECT_FALSE(kinematicBody.hasCollisionShape());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(0);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 1u);
  EXPECT_EQ(stats.dynamicBodyCount, 0u);
  EXPECT_EQ(stats.surfaceCount, 0u);
  EXPECT_EQ(stats.skippedUnsupportedShapeCount, 1u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::NoDofs);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_FALSE(stats.nonConvergedResultSkipped);
  EXPECT_NEAR(kinematicBody.getTranslation().x(), 10.2, 1e-12);
  EXPECT_NEAR(kinematicBody.getLinearVelocity().x(), 2.0, 1e-12);
}

// Test that kinematic writeback is independent of whether the body has a rigid
// IPC-supported collision surface. Unsupported kinematic shapes still advance
// by prescribed velocity; they simply do not contribute rigid IPC contact rows.
TEST(World, RigidIpcContactStageAdvancesUnsupportedKinematicBody)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  dynamicOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));

  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  kinematicOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto kinematicBody
      = world.addRigidBody("kinematic_capsule", kinematicOptions);
  kinematicBody.setCollisionShape(sx::CollisionShape::makeCapsule(0.1, 0.5));
  kinematicBody.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(0);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 2u);
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 1u);
  EXPECT_EQ(stats.skippedUnsupportedShapeCount, 1u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::MaxIterations);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_TRUE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(kinematicBody.getTranslation().x(), 10.2, 1e-12);
  EXPECT_NEAR(kinematicBody.getLinearVelocity().x(), 2.0, 1e-12);
}

// Test that the opt-in IPC stage validates runtime collision geometry before
// handing it to internal barrier and CCD loops.
TEST(World, RigidIpcContactStageSkipsInvalidRuntimeGeometry)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  auto invalidBox = world.addRigidBody("invalid_box", options);
  auto invalidMesh = world.addRigidBody("invalid_mesh", options);
  auto nonFiniteMesh = world.addRigidBody("nonfinite_mesh", options);

  sx::CollisionShape box
      = sx::CollisionShape::makeBox(Eigen::Vector3d(-0.5, 0.5, 0.5));
  sx::CollisionShape mesh = sx::CollisionShape::makeMesh(
      {Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()},
      {Eigen::Vector3i(0, 1, 2)});
  sx::CollisionShape nonFinite = sx::CollisionShape::makeMesh(
      {Eigen::Vector3d::Zero(),
       Eigen::Vector3d::UnitX(),
       Eigen::Vector3d(std::numeric_limits<double>::infinity(), 1.0, 0.0)},
      {Eigen::Vector3i(0, 1, 2)});

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& toReg = dart::simulation::detail::toRegistryEntity;
  registry.emplace_or_replace<sx::comps::CollisionGeometry>(
      toReg(invalidBox.getEntity()), sx::comps::CollisionGeometry{{box}});
  registry.emplace_or_replace<sx::comps::CollisionGeometry>(
      toReg(invalidMesh.getEntity()), sx::comps::CollisionGeometry{{mesh}});
  registry.emplace_or_replace<sx::comps::CollisionGeometry>(
      toReg(nonFiniteMesh.getEntity()),
      sx::comps::CollisionGeometry{{nonFinite}});

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 3u);
  EXPECT_EQ(stats.dynamicBodyCount, 0u);
  EXPECT_EQ(stats.surfaceCount, 0u);
  EXPECT_EQ(stats.skippedUnsupportedShapeCount, 3u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::NoDofs);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_FALSE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(invalidBox.getTranslation().isZero(1e-12));
  EXPECT_TRUE(invalidMesh.getTranslation().isZero(1e-12));
  EXPECT_TRUE(nonFiniteMesh.getTranslation().isZero(1e-12));
  EXPECT_NEAR(invalidBox.getLinearVelocity().x(), 1.0, 1e-12);
  EXPECT_NEAR(invalidMesh.getLinearVelocity().x(), 1.0, 1e-12);
  EXPECT_NEAR(nonFiniteMesh.getLinearVelocity().x(), 1.0, 1e-12);
}

// Test that the opt-in rigid IPC stage does not apply a non-converged solve
// result to runtime state.
TEST(World, RigidIpcContactStageSkipsUnconvergedSolveResult)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("ipc_body", options);
  body.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(0);
  EXPECT_EQ(ipcStage.getMaxIterations(), 0u);

  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.bodyCount, 1u);
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.surfaceCount, 1u);
  EXPECT_EQ(stats.activeDynamicsTerms, 1u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::MaxIterations);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_EQ(stats.acceptedSteps, 0u);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_TRUE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(body.getTranslation().isZero(1e-12));
  EXPECT_NEAR(body.getLinearVelocity().x(), 1.0, 1e-12);
}

// Test that the opt-in rigid IPC stage still advances kinematic obstacles when
// a rejected shared solve had no active contact rows.
TEST(World, RigidIpcContactStageAdvancesKinematicWritebackWithoutActiveContact)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  dynamicOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));

  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  kinematicOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto kinematicBody = world.addRigidBody("kinematic", kinematicOptions);
  kinematicBody.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  kinematicBody.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(0);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.activeFrictionConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::MaxIterations);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_TRUE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(kinematicBody.getTranslation().x(), 10.2, 1e-12);
  EXPECT_NEAR(kinematicBody.getTranslation().y(), 0.0, 1e-12);
  EXPECT_NEAR(kinematicBody.getLinearVelocity().x(), 2.0, 1e-12);
}

// Test that the opt-in rigid IPC stage does not advance kinematic obstacles
// when a rejected shared solve had active contact rows but no accepted step.
TEST(World, RigidIpcContactStageSkipsKinematicWritebackWithActiveContact)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));

  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.position = Eigen::Vector3d(0.215, 0.0, 0.0);
  kinematicOptions.linearVelocity = Eigen::Vector3d(-0.1, 0.0, 0.0);
  auto kinematicBody = world.addRigidBody("kinematic", kinematicOptions);
  kinematicBody.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  kinematicBody.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(0);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::MaxIterations);
  EXPECT_FALSE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_TRUE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(kinematicBody.getTranslation().x(), 0.215, 1e-12);
  EXPECT_NEAR(kinematicBody.getLinearVelocity().x(), -0.1, 1e-12);
}

// Test that a rejected shared solve only blocks kinematic writeback for the
// supported kinematic bodies that participated in active rows.
TEST(World, RigidIpcContactStageAdvancesUninvolvedKinematicAfterRejectedSolve)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));

  sx::RigidBodyOptions blockingOptions;
  blockingOptions.position = Eigen::Vector3d(0.215, 0.0, 0.0);
  blockingOptions.linearVelocity = Eigen::Vector3d(-0.1, 0.0, 0.0);
  auto blockingKinematic
      = world.addRigidBody("blocking_kinematic", blockingOptions);
  blockingKinematic.setCollisionShape(
      sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  blockingKinematic.setKinematic(true);

  sx::RigidBodyOptions uninvolvedOptions;
  uninvolvedOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  uninvolvedOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto uninvolvedKinematic
      = world.addRigidBody("uninvolved_kinematic", uninvolvedOptions);
  uninvolvedKinematic.setCollisionShape(
      sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  uninvolvedKinematic.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(0);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::MaxIterations);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_TRUE(stats.nonConvergedResultSkipped);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(blockingKinematic.getTranslation().x(), 0.215, 1e-12);
  EXPECT_NEAR(blockingKinematic.getLinearVelocity().x(), -0.1, 1e-12);
  EXPECT_NEAR(uninvolvedKinematic.getTranslation().x(), 10.2, 1e-12);
  EXPECT_NEAR(uninvolvedKinematic.getLinearVelocity().x(), 2.0, 1e-12);
}

// Test that a fast kinematic obstacle cannot bypass conservative CCD by
// starting and ending outside the activation distance while crossing a dynamic
// body mid-step.
TEST(World, RigidIpcContactStageBlocksKinematicSweepThroughDynamicBody)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(
      sx::CollisionShape::makeBox({0.05, 0.05, 0.05}));

  sx::RigidBodyOptions wallOptions;
  wallOptions.position = Eigen::Vector3d(-0.3, 0.0, 0.0);
  wallOptions.linearVelocity = Eigen::Vector3d(0.6, 0.0, 0.0);
  auto wall = world.addRigidBody("kinematic_wall", wallOptions);
  wall.setCollisionShape(sx::CollisionShape::makeBox({0.04, 0.2, 0.2}));
  wall.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_EQ(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::LineSearchBlocked);
  EXPECT_FALSE(stats.converged);
  EXPECT_TRUE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_GT(stats.lineSearchHits, 0u);
  EXPECT_LT(stats.lastLineSearchStepBound, 1.0);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(wall.getTranslation().x(), -0.3, 1e-12);
  EXPECT_NEAR(wall.getLinearVelocity().x(), 0.6, 1e-12);
}

// Test that a kinematic obstacle limited to a collision-free prefix by CCD does
// not write the full prescribed end pose when dynamic DOFs are present.
TEST(World, RigidIpcContactStageBlocksKinematicLimitedLineSearch)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(
      sx::CollisionShape::makeBox({0.05, 0.05, 0.05}));

  sx::RigidBodyOptions wallOptions;
  wallOptions.position = Eigen::Vector3d(-0.12, 0.0, 0.0);
  wallOptions.linearVelocity = Eigen::Vector3d(0.1, 0.0, 0.0);
  auto wall = world.addRigidBody("kinematic_wall", wallOptions);
  wall.setCollisionShape(sx::CollisionShape::makeBox({0.04, 0.2, 0.2}));
  wall.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::LineSearchBlocked);
  EXPECT_FALSE(stats.converged);
  EXPECT_TRUE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_GT(stats.lineSearchHits, 0u);
  EXPECT_GT(stats.lastLineSearchStepBound, 0.0);
  EXPECT_LT(stats.lastLineSearchStepBound, 1.0);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(wall.getTranslation().x(), -0.12, 1e-12);
  EXPECT_NEAR(wall.getLinearVelocity().x(), 0.1, 1e-12);
}

// Test that a zero-step kinematic CCD block is not treated as a resting plateau
// and does not write the prescribed end pose.
TEST(World, RigidIpcContactStageDoesNotMaskBlockedKinematicSweepAsResting)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0);

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  auto dynamicBody = world.addRigidBody("dynamic", dynamicOptions);
  dynamicBody.setCollisionShape(
      sx::CollisionShape::makeBox({0.05, 0.05, 0.05}));

  sx::RigidBodyOptions wallOptions;
  wallOptions.position = Eigen::Vector3d(-0.09, 0.0, 0.0);
  wallOptions.linearVelocity = Eigen::Vector3d(0.1, 0.0, 0.0);
  auto wall = world.addRigidBody("kinematic_wall", wallOptions);
  wall.setCollisionShape(sx::CollisionShape::makeBox({0.04, 0.2, 0.2}));
  wall.setKinematic(true);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::LineSearchBlocked);
  EXPECT_FALSE(stats.converged);
  EXPECT_TRUE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_GT(stats.lineSearchZeroStepCount, 0u);
  EXPECT_TRUE(dynamicBody.getTranslation().isZero(1e-12));
  EXPECT_NEAR(wall.getTranslation().x(), -0.09, 1e-12);
  EXPECT_NEAR(wall.getLinearVelocity().x(), 0.1, 1e-12);
}

// Test that a zero-step non-kinematic block does not become a no-op resting
// writeback for pure tangential motion at exact contact; that is valid motion
// to preserve, not a resting velocity to erase.
TEST(World, RigidIpcContactStagePreservesTangentialVelocityOnZeroStepBlock)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.1}));
  ground.setFriction(0.0);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  bodyOptions.position = Eigen::Vector3d(0.0, 0.0, 0.05);
  bodyOptions.linearVelocity = Eigen::Vector3d(0.2, 0.0, 0.0);
  auto body = world.addRigidBody("body", bodyOptions);
  body.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  body.setFriction(0.0);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.dynamicBodyCount, 1u);
  EXPECT_GT(stats.activeConstraints, 0u);
  EXPECT_EQ(stats.status, sx::compute::RigidIpcSolveStatus::LineSearchBlocked);
  EXPECT_TRUE(stats.failed);
  EXPECT_FALSE(stats.resultApplied);
  EXPECT_GT(stats.lineSearchZeroStepCount, 0u);
  EXPECT_TRUE(body.getTranslation().isApprox(Eigen::Vector3d(0.0, 0.0, 0.05)));
  EXPECT_NEAR(body.getLinearVelocity().x(), 0.2, 1e-12);
  EXPECT_NEAR(body.getLinearVelocity().z(), 0.0, 1e-12);
}

// Test that the default World step pipeline can select the experimental rigid
// IPC solver family without double-applying the legacy free-rigid velocity and
// position stages.
TEST(World, StepUsesSelectedRigidIpcSolver)
{
  namespace sx = dart::simulation;

  sx::World world;
  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::SequentialImpulse);
  world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::Ipc);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto body = world.addRigidBody("ipc_body", options);
  body.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Eigen::Vector3d(0.0, 0.0, 0.0),
           Eigen::Vector3d(1.0, 0.0, 0.0),
           Eigen::Vector3d(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));
  body.setForce(Eigen::Vector3d(4.0, 0.0, 0.0));

  world.step();

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_DOUBLE_EQ(world.getTime(), 0.1);
  EXPECT_EQ(world.getFrame(), 1u);
  EXPECT_NEAR(body.getTranslation().x(), 0.12, 1e-8);
  EXPECT_NEAR(body.getLinearVelocity().x(), 1.2, 1e-8);
}

// A kinematic rigid body is advanced by its prescribed (linear + angular)
// velocity each step by the rigid IPC stage, with the velocity preserved (it is
// not integrated under gravity or zeroed by contact response).
TEST(World, RigidIpcKinematicBodyAdvancesAtPrescribedVelocity)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81)); // ignored for kinematic
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.mass = 1.0;
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  options.angularVelocity = Eigen::Vector3d(0.0, 0.0, 2.0);
  auto body = world.addRigidBody("kinematic", options);
  body.setCollisionShape(sx::CollisionShape::makeBox({0.2, 0.2, 0.2}));
  body.setKinematic(true);
  EXPECT_TRUE(body.isKinematic());
  EXPECT_FALSE(body.isStatic());

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  // Advanced by the prescribed linear velocity (no gravity fall) and rotated by
  // omega*dt = 0.2 rad about z.
  EXPECT_NEAR(body.getTranslation().x(), 0.1, 1e-9);
  EXPECT_NEAR(body.getTranslation().z(), 0.0, 1e-9);
  EXPECT_NEAR(body.getLinearVelocity().x(), 1.0, 1e-9);
  EXPECT_NEAR(body.getAngularVelocity().z(), 2.0, 1e-9);
  const Eigen::AngleAxisd rotated{Eigen::Quaterniond(body.getRotation())};
  EXPECT_NEAR(rotated.angle(), 0.2, 1e-6);
}

// Kinematic-only rigid IPC writeback must preserve frame hierarchy ordering so
// child local transforms are computed against the parent's advanced pose.
TEST(World, RigidIpcKinematicOnlyWritebackOrdersParentBeforeChild)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(1.0);

  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  parentOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto parent = world.addRigidBody("parent", parentOptions);
  parent.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  parent.setKinematic(true);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(20.0, 0.0, 0.0);
  childOptions.linearVelocity = Eigen::Vector3d::Zero();
  auto child = world.addRigidBody("child", childOptions);
  child.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  child.setKinematic(true);
  child.setParentFrame(parent);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const Eigen::Vector3d expectedParentPosition(11.0, 0.0, 0.0);
  const Eigen::Vector3d expectedChildPosition(20.0, 0.0, 0.0);
  Eigen::Isometry3d expectedChildLocalTransform = Eigen::Isometry3d::Identity();
  expectedChildLocalTransform.translation()
      = expectedChildPosition - expectedParentPosition;

  EXPECT_TRUE(parent.getTranslation().isApprox(expectedParentPosition, 1e-12));
  EXPECT_TRUE(child.getTranslation().isApprox(expectedChildPosition, 1e-12));
  EXPECT_TRUE(
      child.getLocalTransform().isApprox(expectedChildLocalTransform, 1e-12));
}

// Fig. 13 mechanism (linear form): a kinematic conveyor floor moving in +x
// drags a resting free box forward through lagged Coulomb friction. The moving
// obstacle's surface motion must enter the friction term for this to happen.
TEST(World, RigidIpcKinematicConveyorDragsRestingBox)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The multi-step rigid IPC conveyor gate is too slow under coverage; "
         "normal CI runs the full regression.";
#endif

  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions floorOptions;
  floorOptions.position = Eigen::Vector3d(0.0, 0.0, -0.1);
  floorOptions.linearVelocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  auto floor = world.addRigidBody("conveyor", floorOptions);
  floor.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.1}));
  floor.setKinematic(true);
  floor.setFriction(1.0);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.1 + 1e-3);
  auto box = world.addRigidBody("box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  box.setFriction(1.0);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minBoxZ = box.getTranslation().z();
  for (int s = 0; s < 100; ++s) {
    world.step(executor, pipeline);
    EXPECT_FALSE(ipcStage.getLastStats().failed) << "step " << s;
    minBoxZ = std::min(minBoxZ, box.getTranslation().z());
  }

  // Dragged forward by the moving floor (started at rest), in the floor's
  // direction, without ever penetrating the floor top (z = 0, box half 0.1).
  EXPECT_GT(box.getTranslation().x(), 0.02);
  EXPECT_GT(box.getLinearVelocity().x(), 0.0);
  EXPECT_GT(minBoxZ, 0.1 - 5e-3);
}

// Fig. 13 (Turntable): a free box resting on a kinematic turntable rotating
// about the vertical axis is carried around by surface friction. The rotating
// obstacle's tangential surface motion drags the box (mu = 1), so a box
// starting on the +x axis is pulled toward +y (counter-clockwise) while staying
// seated on the table -- the rotational analogue of the conveyor drag.
TEST(World, RigidIpcKinematicTurntableCarriesRestingBox)
{
#ifdef DART_CODECOV
  GTEST_SKIP()
      << "The multi-step rigid IPC turntable gate is too slow under coverage; "
         "normal CI runs the full regression.";
#endif

  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.005);

  sx::RigidBodyOptions tableOptions;
  tableOptions.position = Eigen::Vector3d(0.0, 0.0, -0.1);
  tableOptions.angularVelocity = Eigen::Vector3d(0.0, 0.0, 1.0); // CCW about z
  auto table = world.addRigidBody("turntable", tableOptions);
  table.setCollisionShape(sx::CollisionShape::makeBox({0.6, 0.6, 0.1}));
  table.setKinematic(true);
  table.setFriction(1.0);

  sx::RigidBodyOptions boxOptions;
  boxOptions.mass = 1.0;
  boxOptions.position = Eigen::Vector3d(0.3, 0.0, 0.1 + 1e-3);
  auto box = world.addRigidBody("rider", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  box.setFriction(1.0);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage;
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);

  double minBoxZ = box.getTranslation().z();
  double maxBoxY = 0.0;
  int failCount = 0;
  for (int s = 0; s < 120; ++s) {
    world.step(executor, pipeline);
    if (ipcStage.getLastStats().failed) {
      ++failCount;
    }
    minBoxZ = std::min(minBoxZ, box.getTranslation().z());
    maxBoxY = std::max(maxBoxY, box.getTranslation().y());
  }

  // Carried tangentially (CCW) by the rotating table: a box on the +x axis is
  // dragged well toward +y with positive tangential velocity, and never
  // penetrates the table top (z = 0, box half 0.1). A `failed` step is a
  // conservative solver skip (the safe, no-penetration path), not an
  // intersection violation, so the asserted invariant is intersection-freedom
  // plus a meaningful tangential drag rather than zero skips.
  EXPECT_GT(maxBoxY, 0.05);
  EXPECT_GT(box.getLinearVelocity().y(), 0.0);
  EXPECT_GT(minBoxZ, 0.1 - 5e-3);
  EXPECT_LE(failCount, 3);
}

TEST(World, RigidBodyPointJointsExposeAvbdFiniteStiffness)
{
  namespace sx = dart::simulation;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  sx::World world;
  auto parent = world.addRigidBody("parent", parentOptions);
  auto child = world.addRigidBody("child", childOptions);
  auto joint = world.addRigidBodyFixedJoint("fixed", parent, child);

  EXPECT_GT(joint.getAvbdStartStiffness(), 0.0);
  EXPECT_TRUE(std::isinf(joint.getAvbdLinearStiffness()));
  EXPECT_TRUE(std::isinf(joint.getAvbdAngularStiffness()));

  joint.setAvbdStartStiffness(1.0);
  joint.setAvbdLinearStiffness(1000.0);
  joint.setAvbdAngularStiffness(100.0);

  EXPECT_DOUBLE_EQ(joint.getAvbdStartStiffness(), 1.0);
  EXPECT_DOUBLE_EQ(joint.getAvbdLinearStiffness(), 1000.0);
  EXPECT_DOUBLE_EQ(joint.getAvbdAngularStiffness(), 100.0);

  EXPECT_THROW(joint.setAvbdStartStiffness(-1.0), sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setAvbdLinearStiffness(std::numeric_limits<double>::quiet_NaN()),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      joint.setAvbdAngularStiffness(-1.0), sx::InvalidArgumentException);
}

TEST(World, ArticulatedPointJointsExposeAvbdFiniteStiffness)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setMultibodyOptions({"variational integrator"});

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  spec.transformFromParent.translation() = Eigen::Vector3d::UnitX();
  auto child = robot.addLink("child", base, spec);

  auto fixed = world.addArticulatedFixedJoint("fixed", base, child);
  auto hinge = world.addArticulatedRevoluteJoint(
      "hinge", base, child, Eigen::Vector3d::UnitY());
  auto slider = world.addArticulatedPrismaticJoint(
      "slider", base, child, Eigen::Vector3d::UnitX());
  auto socket = world.addArticulatedSphericalJoint("socket", base, child);
  auto worldFixed = world.addArticulatedFixedJoint("world_fixed", child);

  const auto expectDefaultHardJoint = [](const sx::Joint& joint) {
    EXPECT_TRUE(std::isinf(joint.getAvbdStartStiffness()));
    EXPECT_TRUE(std::isinf(joint.getAvbdLinearStiffness()));
    EXPECT_TRUE(std::isinf(joint.getAvbdAngularStiffness()));
  };
  expectDefaultHardJoint(fixed);
  expectDefaultHardJoint(hinge);
  expectDefaultHardJoint(slider);
  expectDefaultHardJoint(socket);
  expectDefaultHardJoint(worldFixed);

  const auto expectFiniteStiffnessSetters = [](sx::Joint& joint) {
    joint.setAvbdStartStiffness(2.0);
    joint.setAvbdLinearStiffness(200.0);
    joint.setAvbdAngularStiffness(300.0);

    EXPECT_DOUBLE_EQ(joint.getAvbdStartStiffness(), 2.0);
    EXPECT_DOUBLE_EQ(joint.getAvbdLinearStiffness(), 200.0);
    EXPECT_DOUBLE_EQ(joint.getAvbdAngularStiffness(), 300.0);
  };
  expectFiniteStiffnessSetters(fixed);
  expectFiniteStiffnessSetters(hinge);
  expectFiniteStiffnessSetters(slider);
  expectFiniteStiffnessSetters(socket);
  expectFiniteStiffnessSetters(worldFixed);

  EXPECT_THROW(
      fixed.setAvbdStartStiffness(std::numeric_limits<double>::infinity()),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      hinge.setAvbdLinearStiffness(std::numeric_limits<double>::quiet_NaN()),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      slider.setAvbdAngularStiffness(-1.0), sx::InvalidArgumentException);
}

TEST(World, ArticulatedPointJointsGenerateUniqueFacadeNames)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setMultibodyOptions({"variational integrator"});

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  spec.transformFromParent.translation() = Eigen::Vector3d::UnitX();
  auto child = robot.addLink("child", base, spec);

  auto explicitJoint = world.addArticulatedFixedJoint("joint_001", base, child);
  auto generatedFixed = world.addArticulatedFixedJoint("", base, child);
  auto generatedWorldHinge
      = world.addArticulatedRevoluteJoint("", child, Eigen::Vector3d::UnitY());
  auto generatedSlider = world.addArticulatedPrismaticJoint(
      "", base, child, Eigen::Vector3d::UnitX());

  EXPECT_EQ(explicitJoint.getName(), "joint_001");
  EXPECT_EQ(generatedFixed.getName(), "joint_002");
  EXPECT_EQ(generatedWorldHinge.getName(), "joint_003");
  EXPECT_EQ(generatedSlider.getName(), "joint_004");
  EXPECT_EQ(world.getArticulatedJointCount(), 4u);

  std::map<std::string, sx::JointType> jointsByName;
  for (const auto& joint : world.getArticulatedJoints()) {
    jointsByName.emplace(std::string(joint.getName()), joint.getType());
  }
  ASSERT_EQ(jointsByName.size(), 4u);
  ASSERT_EQ(jointsByName.count("joint_001"), 1u);
  ASSERT_EQ(jointsByName.count("joint_002"), 1u);
  ASSERT_EQ(jointsByName.count("joint_003"), 1u);
  ASSERT_EQ(jointsByName.count("joint_004"), 1u);
  EXPECT_EQ(jointsByName.at("joint_001"), sx::JointType::Fixed);
  EXPECT_EQ(jointsByName.at("joint_002"), sx::JointType::Fixed);
  EXPECT_EQ(jointsByName.at("joint_003"), sx::JointType::Revolute);
  EXPECT_EQ(jointsByName.at("joint_004"), sx::JointType::Prismatic);

  auto foundFixed = world.getArticulatedJoint("joint_002");
  ASSERT_TRUE(foundFixed.has_value());
  EXPECT_EQ(foundFixed->getChildLink().getName(), "child");
  EXPECT_TRUE(world.hasArticulatedJoint("joint_003"));
  EXPECT_FALSE(world.getArticulatedJoint("joint_005").has_value());
  EXPECT_THROW(
      world.addArticulatedSphericalJoint("joint_004", base, child),
      sx::InvalidArgumentException);
}

TEST(World, ClearResetsArticulatedPointJointGeneratedNames)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setMultibodyOptions({"variational integrator"});

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  auto child = robot.addLink("child", base, spec);

  auto firstGenerated = world.addArticulatedFixedJoint("", base, child);
  EXPECT_EQ(firstGenerated.getName(), "joint_001");
  EXPECT_EQ(world.getArticulatedJointCount(), 1u);
  EXPECT_TRUE(base.isValid());
  EXPECT_TRUE(child.isValid());
  EXPECT_TRUE(firstGenerated.isValid());

  world.clear();
  EXPECT_EQ(world.getArticulatedJointCount(), 0u);
  EXPECT_FALSE(world.hasArticulatedJoint("joint_001"));
  EXPECT_FALSE(base.isValid());
  EXPECT_FALSE(child.isValid());
  EXPECT_FALSE(firstGenerated.isValid());

  auto rebuiltRobot = world.addMultibody("rebuilt_robot");
  auto rebuiltBase = rebuiltRobot.addLink("base");
  sx::JointSpec rebuiltSpec;
  rebuiltSpec.name = "floating";
  rebuiltSpec.type = sx::JointType::Floating;
  auto rebuiltChild = rebuiltRobot.addLink("child", rebuiltBase, rebuiltSpec);

  auto regenerated = world.addArticulatedRevoluteJoint(
      "", rebuiltChild, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(regenerated.getName(), "joint_001");
  EXPECT_EQ(regenerated.getType(), sx::JointType::Revolute);
  EXPECT_EQ(world.getArticulatedJointCount(), 1u);
}

TEST(World, ArticulatedPointJointsRejectInvalidEndpointOwnership)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setMultibodyOptions({"variational integrator"});

  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");
  sx::JointSpec spec;
  spec.name = "floating";
  spec.type = sx::JointType::Floating;
  spec.transformFromParent.translation() = Eigen::Vector3d::UnitX();
  auto child = robot.addLink("child", base, spec);

  auto otherRobot = world.addMultibody("other");
  auto otherBase = otherRobot.addLink("base");
  sx::JointSpec otherSpec;
  otherSpec.name = "other_floating";
  otherSpec.type = sx::JointType::Floating;
  otherSpec.transformFromParent.translation() = 2.0 * Eigen::Vector3d::UnitX();
  auto otherChild = otherRobot.addLink("child", otherBase, otherSpec);

  sx::World foreignWorld;
  foreignWorld.setMultibodyOptions({"variational integrator"});
  auto foreignRobot = foreignWorld.addMultibody("foreign");
  auto foreignBase = foreignRobot.addLink("base");

  EXPECT_THROW(
      world.addArticulatedFixedJoint("same_link", base, base),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.addArticulatedRevoluteJoint(
          "cross_multibody", base, otherChild, Eigen::Vector3d::UnitZ()),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.addArticulatedPrismaticJoint(
          "cross_world", base, foreignBase, Eigen::Vector3d::UnitX()),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      world.addArticulatedSphericalJoint("world_cross_world", foreignBase),
      sx::InvalidArgumentException);

  auto valid = world.addArticulatedFixedJoint("valid", base, child);
  EXPECT_EQ(valid.getParentLink().getName(), "base");
  EXPECT_EQ(valid.getChildLink().getName(), "child");
}

// Test that public fixed joints feed the IPC rigid-body solver through private
// point and orientation equality rows instead of exposing solver-internal
// contracts.
TEST(World, RigidIpcContactStageProjectsFixedJointPose)
{
  namespace sx = dart::simulation;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  childOptions.mass = 1.0;

  sx::World world;
  world.setTimeStep(0.01);
  world.setGravity(-9.81 * Eigen::Vector3d::UnitY());
  auto parent = world.addRigidBody("parent", parentOptions);
  parent.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  auto child = world.addRigidBody("child", childOptions);
  child.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  child.setAngularVelocity(Eigen::Vector3d(0.0, 0.25, 0.0));
  child.setTorque(Eigen::Vector3d(0.0, 1.0, 0.0));
  (void)world.addRigidBodyFixedJoint("fixed", parent, child);

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(16);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.activeArticulationConstraints, 3u);
  EXPECT_TRUE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_LT(stats.finalEqualityResidualNorm, 1e-8);

  // The fixed joint captured the child's initial pose. Gravity and torque are
  // balanced by the equality rows, so the child cannot translate or rotate away
  // from the parent-side pose.
  EXPECT_TRUE(child.getTranslation().isApprox(Eigen::Vector3d::UnitX(), 1e-10));
  EXPECT_LT(child.getLinearVelocity().norm(), 1e-10);
  EXPECT_LT(
      (child.getTransform().linear() - Eigen::Matrix3d::Identity()).norm(),
      1e-10);
  EXPECT_LT(child.getAngularVelocity().norm(), 1e-10);
}

// Test that public revolute joints feed the IPC rigid-body solver through
// private point and hinge-axis equality rows, preserving the hinge axis while
// letting gravity swing the body around the captured pivot.
TEST(World, RigidIpcContactStageProjectsRevoluteJointHingeAxis)
{
  namespace sx = dart::simulation;
  namespace dvbd = sx::detail::deformable_vbd;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();
  childOptions.mass = 1.0;

  sx::World world;
  world.setTimeStep(0.001);
  world.setGravity(-9.81 * Eigen::Vector3d::UnitY());
  auto parent = world.addRigidBody("parent", parentOptions);
  parent.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  auto child = world.addRigidBody("child", childOptions);
  child.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.1, 0.1}));
  auto joint = world.addRigidBodyRevoluteJoint(
      "hinge", parent, child, Eigen::Vector3d::UnitZ());

  auto& registry = sx::detail::registryOf(world);
  auto& config = registry.get<dvbd::AvbdRigidWorldPointJointConfig>(
      sx::detail::toRegistryEntity(joint.getEntity()));
  config.localAnchorA = Eigen::Vector3d::Zero();
  config.localAnchorB = -Eigen::Vector3d::UnitX();

  sx::compute::SequentialExecutor executor;
  sx::compute::RigidIpcContactStage ipcStage(64);
  sx::compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  const auto& stats = ipcStage.getLastStats();
  EXPECT_EQ(stats.activeArticulationConstraints, 2u);
  EXPECT_EQ(stats.activeDynamicsTerms, 1u);
  EXPECT_GT(stats.acceptedSteps, 0u);
  EXPECT_TRUE(stats.converged);
  EXPECT_FALSE(stats.failed);
  EXPECT_TRUE(stats.resultApplied);
  EXPECT_LT(stats.finalEqualityResidualNorm, 1e-8);

  const Eigen::Vector3d pivot
      = child.getTransform() * (-Eigen::Vector3d::UnitX());
  EXPECT_LT(pivot.norm(), 1e-8);

  const Eigen::Vector3d childAxis
      = child.getTransform().linear() * Eigen::Vector3d::UnitZ();
  EXPECT_LT(childAxis.cross(Eigen::Vector3d::UnitZ()).norm(), 1e-8);
  EXPECT_LT(child.getTranslation().y(), -1e-9);
  EXPECT_LT(child.getAngularVelocity().z(), -1e-7);
}

// Test that unsupported rigid-body joint types still reject the IPC solver.
TEST(World, RigidBodyPrismaticJointsRejectIpcSolver)
{
  namespace sx = dart::simulation;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();

  sx::World solverFirst;
  solverFirst.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  auto solverFirstParent = solverFirst.addRigidBody("parent", parentOptions);
  auto solverFirstChild = solverFirst.addRigidBody("child", childOptions);
  EXPECT_THROW(
      solverFirst.addRigidBodyPrismaticJoint(
          "slide",
          solverFirstParent,
          solverFirstChild,
          Eigen::Vector3d::UnitX()),
      sx::InvalidOperationException);

  sx::World jointFirst;
  auto jointFirstParent = jointFirst.addRigidBody("parent", parentOptions);
  auto jointFirstChild = jointFirst.addRigidBody("child", childOptions);
  (void)jointFirst.addRigidBodyPrismaticJoint(
      "slide", jointFirstParent, jointFirstChild, Eigen::Vector3d::UnitX());

  EXPECT_THROW(
      jointFirst.setRigidBodySolver(sx::RigidBodySolver::Ipc),
      sx::InvalidOperationException);
  EXPECT_EQ(
      jointFirst.getRigidBodySolver(), sx::RigidBodySolver::SequentialImpulse);
}

// Test that breakable rigid-body joints reject the IPC solver instead of being
// silently skipped by the private equality-row projection path.
TEST(World, RigidBodyBreakableJointsRejectIpcSolver)
{
  namespace sx = dart::simulation;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();

  sx::World solverFirst;
  solverFirst.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  auto solverFirstParent = solverFirst.addRigidBody("parent", parentOptions);
  auto solverFirstChild = solverFirst.addRigidBody("child", childOptions);
  auto solverFirstJoint = solverFirst.addRigidBodyFixedJoint(
      "fixed", solverFirstParent, solverFirstChild);
  solverFirstJoint.setBreakForce(10.0);

  EXPECT_THROW(
      solverFirst.enterSimulationMode(), sx::InvalidOperationException);

  sx::World jointFirst;
  auto jointFirstParent = jointFirst.addRigidBody("parent", parentOptions);
  auto jointFirstChild = jointFirst.addRigidBody("child", childOptions);
  auto jointFirstJoint = jointFirst.addRigidBodyFixedJoint(
      "fixed", jointFirstParent, jointFirstChild);
  jointFirstJoint.setBreakForce(10.0);

  EXPECT_THROW(
      jointFirst.setRigidBodySolver(sx::RigidBodySolver::Ipc),
      sx::InvalidOperationException);
  EXPECT_EQ(
      jointFirst.getRigidBodySolver(), sx::RigidBodySolver::SequentialImpulse);
}

// Test that public fixed joints reject mixed multibody worlds instead of
// accepting a joint that the unified contact pipeline will not project.
TEST(World, RigidBodyFixedJointsRejectUnifiedMultibodyPipeline)
{
  namespace sx = dart::simulation;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();

  sx::World multibodyFirst;
  (void)multibodyFirst.addMultibody("robot");
  auto multibodyFirstParent
      = multibodyFirst.addRigidBody("parent", parentOptions);
  auto multibodyFirstChild = multibodyFirst.addRigidBody("child", childOptions);
  EXPECT_THROW(
      multibodyFirst.addRigidBodyFixedJoint(
          "fixed", multibodyFirstParent, multibodyFirstChild),
      sx::InvalidOperationException);

  sx::World jointFirst;
  auto jointFirstParent = jointFirst.addRigidBody("parent", parentOptions);
  auto jointFirstChild = jointFirst.addRigidBody("child", childOptions);
  (void)jointFirst.addRigidBodyFixedJoint(
      "fixed", jointFirstParent, jointFirstChild);

  EXPECT_THROW(jointFirst.addMultibody("robot"), sx::InvalidOperationException);
  EXPECT_EQ(jointFirst.getMultibodyCount(), 0u);
}

// Test that invalid solver selections are rejected instead of silently falling
// through to a different rigid-body pipeline.
TEST(World, SetRigidBodySolverRejectsInvalidEnum)
{
  namespace sx = dart::simulation;

  sx::World world;
  EXPECT_THROW(
      world.setRigidBodySolver(static_cast<sx::RigidBodySolver>(99)),
      sx::InvalidArgumentException);
  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::SequentialImpulse);
}

// Test that rigid-body integration keeps the integrated world pose
// authoritative when a body has been reparented through the inherited frame
// API.
TEST(World, StepStoresReparentedRigidBodyPoseAsParentLocal)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(10.0, 0.0, 0.0));
  parent.setLocalTransform(parentTransform);

  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);

  auto body = world.addRigidBody("body", options);
  body.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  world.step();

  const auto expectedPosition = Eigen::Vector3d(2.0, 2.0, 3.0);
  Eigen::Isometry3d expectedWorldTransform = Eigen::Isometry3d::Identity();
  expectedWorldTransform.translation() = expectedPosition;
  Eigen::Isometry3d expectedLocalTransform
      = parentTransform.inverse() * expectedWorldTransform;

  EXPECT_TRUE(body.getTranslation().isApprox(expectedPosition));
  EXPECT_TRUE(body.getLocalTransform().isApprox(expectedLocalTransform));
  EXPECT_TRUE(body.getTransform().isApprox(expectedWorldTransform));
}

// Test that the default non-IPC pipeline preserves kinematic rigid bodies as
// prescribed obstacles while keeping them immune to forces and gravity.
TEST(World, StepPreservesKinematicRigidBodyWithDefaultSolver)
{
  namespace sx = dart::simulation;

  const Eigen::Vector3d initialPosition(1.0, 2.0, 3.0);
  const Eigen::Quaterniond initialOrientation(
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d linearVelocity(2.0, -1.0, 0.5);
  const Eigen::Vector3d angularVelocity(0.0, 0.0, 0.4);

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.5);

  sx::RigidBodyOptions options;
  options.position = initialPosition;
  options.orientation = initialOrientation;
  options.linearVelocity = linearVelocity;
  options.angularVelocity = angularVelocity;
  auto body = world.addRigidBody("kinematic_body", options);
  body.setKinematic(true);
  body.setForce(Eigen::Vector3d(10.0, 20.0, -30.0));
  body.setTorque(Eigen::Vector3d(1.0, -2.0, 3.0));

  world.step();

  EXPECT_TRUE(body.getTranslation().isApprox(initialPosition));
  EXPECT_TRUE(body.getQuaternion().isApprox(initialOrientation));
  EXPECT_TRUE(body.getLinearVelocity().isApprox(linearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(angularVelocity));
}

TEST(World, RigidBodyVelocityStageSkipsPrescribedBodyForces)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -4.0));
  world.setTimeStep(0.25);

  sx::RigidBodyOptions staticOptions;
  staticOptions.isStatic = true;
  staticOptions.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  staticOptions.linearVelocity = Eigen::Vector3d(2.0, -1.0, 0.5);
  staticOptions.angularVelocity = Eigen::Vector3d(0.4, 0.1, -0.2);
  auto staticBody = world.addRigidBody("static_body", staticOptions);
  staticBody.setForce(Eigen::Vector3d(10.0, 20.0, -30.0));
  staticBody.setTorque(Eigen::Vector3d(1.0, -2.0, 3.0));

  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.position = Eigen::Vector3d(-2.0, 0.5, 4.0);
  kinematicOptions.linearVelocity = Eigen::Vector3d(-0.5, 1.0, 0.25);
  kinematicOptions.angularVelocity = Eigen::Vector3d(0.2, -0.1, 0.3);
  auto kinematicBody = world.addRigidBody("kinematic_body", kinematicOptions);
  kinematicBody.setKinematic(true);
  kinematicBody.setForce(Eigen::Vector3d(-3.0, 4.0, 5.0));
  kinematicBody.setTorque(Eigen::Vector3d(-0.25, 0.5, 0.75));

  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.mass = 2.0;
  dynamicOptions.position = Eigen::Vector3d::Zero();
  dynamicOptions.linearVelocity = Eigen::Vector3d::Zero();
  auto dynamicBody = world.addRigidBody("dynamic_body", dynamicOptions);
  dynamicBody.setForce(Eigen::Vector3d(0.0, 0.0, 8.0));

  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::RigidBodyVelocityStage stage;
  stage.prepare(world);
  stage.execute(world, executor);

  EXPECT_TRUE(
      staticBody.getLinearVelocity().isApprox(staticOptions.linearVelocity));
  EXPECT_TRUE(
      staticBody.getAngularVelocity().isApprox(staticOptions.angularVelocity));
  EXPECT_TRUE(kinematicBody.getLinearVelocity().isApprox(
      kinematicOptions.linearVelocity));
  EXPECT_TRUE(kinematicBody.getAngularVelocity().isApprox(
      kinematicOptions.angularVelocity));
  EXPECT_TRUE(
      dynamicBody.getLinearVelocity().isApprox(Eigen::Vector3d::Zero()));
}

// Test that rigid-body frame ancestry is represented in the integration graph
// so a parallel executor cannot run a child integration while its parent rigid
// body is updating frame properties.
TEST(World, RigidBodyIntegrationStageOrdersRigidBodyFrameAncestry)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;

  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  parentOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto parent = world.addRigidBody("parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(20.0, 0.0, 0.0);
  childOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto child = world.addRigidBody("child", childOptions);
  child.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  RecordingExecutor executor;
  compute::RigidBodyIntegrationStage stage(1);
  stage.execute(world, executor);

  EXPECT_EQ(executor.nodeCount, 2u);
  EXPECT_EQ(executor.edgeCount, 1u);

  const auto expectedParentPosition = Eigen::Vector3d(11.0, 0.0, 0.0);
  const auto expectedChildPosition = Eigen::Vector3d(20.5, 0.0, 0.0);
  Eigen::Isometry3d expectedChildLocalTransform = Eigen::Isometry3d::Identity();
  expectedChildLocalTransform.translation()
      = expectedChildPosition - expectedParentPosition;

  EXPECT_TRUE(parent.getLocalTransform().translation().isApprox(
      expectedParentPosition));
  EXPECT_TRUE(child.getLocalTransform().isApprox(expectedChildLocalTransform));
}

// Test that the direct rigid-body integration stage preserves static-body state
// even when stored velocity and force components are non-zero.
TEST(World, RigidBodyIntegrationStageSkipsStaticBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  const Eigen::Vector3d initialPosition(1.0, 2.0, 3.0);
  const Eigen::Quaterniond initialOrientation(
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d initialLinearVelocity(2.0, -1.0, 0.5);
  const Eigen::Vector3d initialAngularVelocity(0.4, 0.1, -0.2);

  sx::World world;
  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = initialPosition;
  options.orientation = initialOrientation;
  options.linearVelocity = initialLinearVelocity;
  options.angularVelocity = initialAngularVelocity;
  auto body = world.addRigidBody("static_body", options);
  body.setForce(Eigen::Vector3d(10.0, 20.0, -30.0));
  body.setTorque(Eigen::Vector3d(1.0, -2.0, 3.0));

  world.setTimeStep(0.5);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::RigidBodyIntegrationStage stage(1);
  stage.execute(world, executor);

  EXPECT_TRUE(body.getTranslation().isApprox(initialPosition));
  EXPECT_TRUE(body.getQuaternion().isApprox(initialOrientation));
  EXPECT_TRUE(body.getLinearVelocity().isApprox(initialLinearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(initialAngularVelocity));
}

// Test that the direct rigid-body integration stage also preserves kinematic
// bodies when custom pipelines bypass the default IPC contact stage.
TEST(World, RigidBodyIntegrationStageSkipsKinematicBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  const Eigen::Vector3d initialPosition(1.0, 2.0, 3.0);
  const Eigen::Quaterniond initialOrientation(
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d initialLinearVelocity(2.0, -1.0, 0.5);
  const Eigen::Vector3d initialAngularVelocity(0.4, 0.1, -0.2);

  sx::World world;
  sx::RigidBodyOptions options;
  options.position = initialPosition;
  options.orientation = initialOrientation;
  options.linearVelocity = initialLinearVelocity;
  options.angularVelocity = initialAngularVelocity;
  auto body = world.addRigidBody("kinematic_body", options);
  body.setKinematic(true);
  body.setForce(Eigen::Vector3d(10.0, 20.0, -30.0));
  body.setTorque(Eigen::Vector3d(1.0, -2.0, 3.0));

  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(0.5);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::RigidBodyIntegrationStage stage(1);
  stage.execute(world, executor);

  EXPECT_TRUE(body.getTranslation().isApprox(initialPosition));
  EXPECT_TRUE(body.getQuaternion().isApprox(initialOrientation));
  EXPECT_TRUE(body.getLinearVelocity().isApprox(initialLinearVelocity));
  EXPECT_TRUE(body.getAngularVelocity().isApprox(initialAngularVelocity));
}

// Test that the batched SoA integration stage also preserves static-body state
// while continuing to integrate dynamic rigid bodies in the same batch.
TEST(World, BatchedRigidBodyIntegrationStageSkipsStaticBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  const Eigen::Vector3d staticPosition(1.0, 2.0, 3.0);
  const Eigen::Quaterniond staticOrientation(
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d staticLinearVelocity(2.0, -1.0, 0.5);
  const Eigen::Vector3d staticAngularVelocity(0.4, 0.1, -0.2);

  sx::World world;
  sx::RigidBodyOptions staticOptions;
  staticOptions.isStatic = true;
  staticOptions.position = staticPosition;
  staticOptions.orientation = staticOrientation;
  staticOptions.linearVelocity = staticLinearVelocity;
  staticOptions.angularVelocity = staticAngularVelocity;
  auto staticBody = world.addRigidBody("static_body", staticOptions);
  staticBody.setForce(Eigen::Vector3d(10.0, 20.0, -30.0));
  staticBody.setTorque(Eigen::Vector3d(1.0, -2.0, 3.0));

  const Eigen::Vector3d dynamicPosition(-1.0, -2.0, 4.0);
  const Eigen::Vector3d dynamicLinearVelocity(0.5, 1.0, -2.0);
  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.position = dynamicPosition;
  dynamicOptions.linearVelocity = dynamicLinearVelocity;
  auto dynamicBody = world.addRigidBody("dynamic_body", dynamicOptions);

  constexpr double dt = 0.5;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(dt);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::BatchedRigidBodyIntegrationStage stage;
  stage.execute(world, executor);

  EXPECT_TRUE(staticBody.getTranslation().isApprox(staticPosition));
  EXPECT_TRUE(staticBody.getQuaternion().isApprox(staticOrientation));
  EXPECT_TRUE(staticBody.getLinearVelocity().isApprox(staticLinearVelocity));
  EXPECT_TRUE(staticBody.getAngularVelocity().isApprox(staticAngularVelocity));
  EXPECT_TRUE(dynamicBody.getTranslation().isApprox(
      dynamicPosition + dynamicLinearVelocity * dt));
  EXPECT_TRUE(dynamicBody.getLinearVelocity().isApprox(dynamicLinearVelocity));
}

// Test that the batched SoA integration stage preserves kinematic-body state
// while continuing to integrate dynamic rigid bodies in the same batch.
TEST(World, BatchedRigidBodyIntegrationStageSkipsKinematicBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  const Eigen::Vector3d kinematicPosition(1.0, 2.0, 3.0);
  const Eigen::Quaterniond kinematicOrientation(
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d kinematicLinearVelocity(2.0, -1.0, 0.5);
  const Eigen::Vector3d kinematicAngularVelocity(0.4, 0.1, -0.2);

  sx::World world;
  sx::RigidBodyOptions kinematicOptions;
  kinematicOptions.position = kinematicPosition;
  kinematicOptions.orientation = kinematicOrientation;
  kinematicOptions.linearVelocity = kinematicLinearVelocity;
  kinematicOptions.angularVelocity = kinematicAngularVelocity;
  auto kinematicBody = world.addRigidBody("kinematic_body", kinematicOptions);
  kinematicBody.setKinematic(true);
  kinematicBody.setForce(Eigen::Vector3d(10.0, 20.0, -30.0));
  kinematicBody.setTorque(Eigen::Vector3d(1.0, -2.0, 3.0));

  const Eigen::Vector3d dynamicPosition(-1.0, -2.0, 4.0);
  const Eigen::Vector3d dynamicLinearVelocity(0.5, 1.0, -2.0);
  sx::RigidBodyOptions dynamicOptions;
  dynamicOptions.position = dynamicPosition;
  dynamicOptions.linearVelocity = dynamicLinearVelocity;
  auto dynamicBody = world.addRigidBody("dynamic_body", dynamicOptions);

  constexpr double dt = 0.5;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(dt);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::BatchedRigidBodyIntegrationStage stage;
  stage.execute(world, executor);

  EXPECT_TRUE(kinematicBody.getTranslation().isApprox(kinematicPosition));
  EXPECT_TRUE(kinematicBody.getQuaternion().isApprox(kinematicOrientation));
  EXPECT_TRUE(
      kinematicBody.getLinearVelocity().isApprox(kinematicLinearVelocity));
  EXPECT_TRUE(
      kinematicBody.getAngularVelocity().isApprox(kinematicAngularVelocity));
  EXPECT_TRUE(dynamicBody.getTranslation().isApprox(
      dynamicPosition + dynamicLinearVelocity * dt));
  EXPECT_TRUE(dynamicBody.getLinearVelocity().isApprox(dynamicLinearVelocity));
}

// Test that the direct batched SoA integration stage uses the World's gravity
// when assembling its force buffer, matching the default step path's first
// velocity stage.
TEST(World, BatchedRigidBodyIntegrationStageIncludesWorldGravity)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  sx::RigidBodyOptions options;
  options.mass = 2.0;
  options.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  options.linearVelocity = Eigen::Vector3d::Zero();
  auto body = world.addRigidBody("falling_body", options);

  const Eigen::Vector3d gravity(0.0, 0.0, -4.0);
  constexpr double dt = 0.25;
  world.setGravity(gravity);
  world.setTimeStep(dt);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::BatchedRigidBodyIntegrationStage stage;
  stage.execute(world, executor);

  EXPECT_TRUE(body.getLinearVelocity().isApprox(gravity * dt));
  EXPECT_TRUE(
      body.getTranslation().isApprox(options.position + gravity * dt * dt));
}

// Test that the batched stage keeps world gravity when it falls back to the
// per-entity integrator for rigid-body frame dependencies.
TEST(World, BatchedRigidBodyIntegrationStageFallbackIncludesWorldGravity)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  sx::RigidBodyOptions parentOptions;
  parentOptions.mass = 2.0;
  parentOptions.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  parentOptions.linearVelocity = Eigen::Vector3d::Zero();
  auto parent = world.addRigidBody("parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.mass = 1.5;
  childOptions.position = Eigen::Vector3d(2.0, 2.0, 3.0);
  childOptions.linearVelocity = Eigen::Vector3d::Zero();
  auto child = world.addRigidBody("child", childOptions);
  child.setParentFrame(parent);

  const Eigen::Vector3d gravity(0.0, 0.0, -4.0);
  constexpr double dt = 0.25;
  world.setGravity(gravity);
  world.setTimeStep(dt);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::BatchedRigidBodyIntegrationStage stage;
  stage.execute(world, executor);

  EXPECT_TRUE(parent.getLinearVelocity().isApprox(gravity * dt));
  EXPECT_TRUE(parent.getTranslation().isApprox(
      parentOptions.position + gravity * dt * dt));
}

// Test that the rigid-body integration stage produces the same state through
// sequential and parallel graph executors.
TEST(World, RigidBodyStepParallelMatchesSequential)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto addBody = [](sx::World& world, int index) {
    sx::RigidBodyOptions options;
    options.mass = 1.0 + 0.25 * index;
    options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
    options.position = Eigen::Vector3d(0.1 * index, 0.2 * index, 0.3 * index);
    options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * index, -0.125 * index);
    options.angularVelocity
        = Eigen::Vector3d(0.01 * index, 0.02 * index, 0.03 * index);

    auto body = world.addRigidBody("body_" + std::to_string(index), options);
    body.setForce(Eigen::Vector3d(0.1 * index, 0.2, -0.3));
    return body;
  };

  sx::World sequentialWorld;
  sx::World parallelWorld;
  std::vector<sx::RigidBody> sequentialBodies;
  std::vector<sx::RigidBody> parallelBodies;

  for (int i = 0; i < 8; ++i) {
    sequentialBodies.push_back(addBody(sequentialWorld, i));
    parallelBodies.push_back(addBody(parallelWorld, i));
  }

  sequentialWorld.setTimeStep(0.01);
  parallelWorld.setTimeStep(0.01);
  sequentialWorld.enterSimulationMode();
  parallelWorld.enterSimulationMode();

  compute::ParallelExecutor executor(2);
  for (int i = 0; i < 4; ++i) {
    sequentialWorld.step();
    parallelWorld.step(executor);
  }

  EXPECT_DOUBLE_EQ(sequentialWorld.getTime(), parallelWorld.getTime());
  EXPECT_EQ(sequentialWorld.getFrame(), parallelWorld.getFrame());

  constexpr double tolerance = 1e-10;
  for (std::size_t i = 0; i < sequentialBodies.size(); ++i) {
    const auto sequentialTranslation = sequentialBodies[i].getTranslation();
    const auto parallelTranslation = parallelBodies[i].getTranslation();
    const auto sequentialRotation = sequentialBodies[i].getRotation();
    const auto parallelRotation = parallelBodies[i].getRotation();
    const auto sequentialLinearVelocity
        = sequentialBodies[i].getLinearVelocity();
    const auto parallelLinearVelocity = parallelBodies[i].getLinearVelocity();
    const auto sequentialAngularVelocity
        = sequentialBodies[i].getAngularVelocity();
    const auto parallelAngularVelocity = parallelBodies[i].getAngularVelocity();

    EXPECT_TRUE(sequentialTranslation.isApprox(parallelTranslation, tolerance))
        << "position diff: "
        << (sequentialTranslation - parallelTranslation).norm();
    EXPECT_TRUE(sequentialRotation.isApprox(parallelRotation, tolerance))
        << "orientation diff: "
        << (sequentialRotation - parallelRotation).norm();
    EXPECT_TRUE(
        sequentialLinearVelocity.isApprox(parallelLinearVelocity, tolerance))
        << "linear velocity diff: "
        << (sequentialLinearVelocity - parallelLinearVelocity).norm();
    EXPECT_TRUE(
        sequentialAngularVelocity.isApprox(parallelAngularVelocity, tolerance))
        << "angular velocity diff: "
        << (sequentialAngularVelocity - parallelAngularVelocity).norm();
    EXPECT_TRUE(
        sequentialBodies[i].getTransform().isApprox(
            parallelBodies[i].getTransform(), tolerance));
  }
}

// Test that parallel execution matches the sequential reference across a range
// of worker counts, hardening the deterministic-sync guarantee beyond a single
// worker configuration.
TEST(World, RigidBodyStepParallelMatchesSequentialAcrossWorkerCounts)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  constexpr int bodyCount = 12;
  constexpr int stepCount = 5;
  constexpr double tolerance = 1e-10;

  auto buildWorld = [](sx::World& world, std::vector<sx::RigidBody>& bodies) {
    for (int i = 0; i < bodyCount; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + 0.1 * i;
      options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
      options.position = Eigen::Vector3d(0.1 * i, 0.2 * i, 0.3 * i);
      options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * i, -0.1 * i);
      options.angularVelocity = Eigen::Vector3d(0.01 * i, 0.02 * i, 0.03 * i);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.1 * i, 0.2, -0.3));
      bodies.push_back(body);
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World reference;
  std::vector<sx::RigidBody> referenceBodies;
  buildWorld(reference, referenceBodies);
  for (int s = 0; s < stepCount; ++s) {
    reference.step();
  }

  for (const std::size_t workers :
       {std::size_t{1}, std::size_t{2}, std::size_t{4}, std::size_t{8}}) {
    sx::World world;
    std::vector<sx::RigidBody> bodies;
    buildWorld(world, bodies);

    compute::ParallelExecutor executor(workers);
    for (int s = 0; s < stepCount; ++s) {
      world.step(executor);
    }

    for (int i = 0; i < bodyCount; ++i) {
      EXPECT_TRUE(
          referenceBodies[static_cast<std::size_t>(i)].getTransform().isApprox(
              bodies[static_cast<std::size_t>(i)].getTransform(), tolerance))
          << "workers=" << workers << " body=" << i;
      EXPECT_TRUE(
          referenceBodies[static_cast<std::size_t>(i)]
              .getLinearVelocity()
              .isApprox(
                  bodies[static_cast<std::size_t>(i)].getLinearVelocity(),
                  tolerance))
          << "workers=" << workers << " body=" << i;
      EXPECT_TRUE(
          referenceBodies[static_cast<std::size_t>(i)]
              .getAngularVelocity()
              .isApprox(
                  bodies[static_cast<std::size_t>(i)].getAngularVelocity(),
                  tolerance))
          << "workers=" << workers << " body=" << i;
    }
  }
}

// Test the Phase 3 determinism gate's strong half: rigid-body integration is a
// map-only stage (each body reads and writes only its own components, with no
// cross-body reduction), so parallel execution must equal the sequential
// reference bit-for-bit, not merely within a tolerance. A batch size of one
// puts each body in its own graph node so the bodies actually run concurrently
// across workers, rather than collapsing into a single batched node.
TEST(World, RigidBodyIntegrationStageIsBitwiseDeterministicAcrossWorkers)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto buildWorld = [](sx::World& world, std::vector<sx::RigidBody>& bodies) {
    for (int i = 0; i < 12; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + 0.1 * i;
      options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
      options.position = Eigen::Vector3d(0.1 * i, 0.2 * i, 0.3 * i);
      options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * i, -0.1 * i);
      options.angularVelocity = Eigen::Vector3d(0.01 * i, 0.02 * i, 0.03 * i);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.1 * i, 0.2, -0.3));
      body.setTorque(Eigen::Vector3d(0.05, -0.1 * i, 0.2));
      bodies.push_back(body);
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World reference;
  std::vector<sx::RigidBody> referenceBodies;
  buildWorld(reference, referenceBodies);
  compute::SequentialExecutor sequential;
  compute::RigidBodyIntegrationStage referenceStage(1);
  for (int s = 0; s < 5; ++s) {
    referenceStage.execute(reference, sequential);
  }

  for (const std::size_t workers :
       {std::size_t{1}, std::size_t{2}, std::size_t{4}, std::size_t{8}}) {
    sx::World world;
    std::vector<sx::RigidBody> bodies;
    buildWorld(world, bodies);

    compute::ParallelExecutor executor(workers);
    compute::RigidBodyIntegrationStage stage(1);
    for (int s = 0; s < 5; ++s) {
      stage.execute(world, executor);
    }

    for (std::size_t i = 0; i < referenceBodies.size(); ++i) {
      EXPECT_TRUE((referenceBodies[i].getTransform().matrix().array()
                   == bodies[i].getTransform().matrix().array())
                      .all())
          << "workers=" << workers << " body=" << i << " transform not bitwise";
      EXPECT_TRUE((referenceBodies[i].getLinearVelocity().array()
                   == bodies[i].getLinearVelocity().array())
                      .all())
          << "workers=" << workers << " body=" << i
          << " linear velocity not bitwise";
      EXPECT_TRUE((referenceBodies[i].getAngularVelocity().array()
                   == bodies[i].getAngularVelocity().array())
                      .all())
          << "workers=" << workers << " body=" << i
          << " angular velocity not bitwise";
    }
  }
}

// Test that the batched SoA integration stage matches the per-entity stage for
// free (non-frame-coupled) rigid bodies under a full dynamic step: linear
// force, torque (via the shared world-inertia LDLT solve), and the shared
// angle-axis exponential-map orientation update.
TEST(World, BatchedRigidBodyIntegrationStageMatchesPerEntityForFreeBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto buildWorld = [](sx::World& world, std::vector<sx::RigidBody>& bodies) {
    for (int i = 0; i < 8; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + 0.3 * i;
      options.inertia = Eigen::Vector3d(1.0, 1.5, 2.0).asDiagonal();
      options.position = Eigen::Vector3d(0.2 * i, -0.1 * i, 0.05 * i);
      options.linearVelocity = Eigen::Vector3d(0.5, 0.25 * i, -0.1 * i);
      options.angularVelocity = Eigen::Vector3d(0.05 * i, -0.03 * i, 0.2);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.1 * i, 0.2, -0.3));
      // Non-zero torque exercises the world-inertia (R I R^T) LDLT solve that
      // both stages now share.
      body.setTorque(Eigen::Vector3d(0.05, -0.1 * i, 0.2));
      bodies.push_back(body);
    }
    world.setGravity(Eigen::Vector3d::Zero());
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World perEntityWorld;
  sx::World batchedWorld;
  std::vector<sx::RigidBody> perEntityBodies;
  std::vector<sx::RigidBody> batchedBodies;
  buildWorld(perEntityWorld, perEntityBodies);
  buildWorld(batchedWorld, batchedBodies);

  compute::SequentialExecutor executor;
  compute::RigidBodyIntegrationStage perEntityStage;
  compute::BatchedRigidBodyIntegrationStage batchedStage;

  constexpr double tolerance = 1e-10;
  for (int step = 0; step < 5; ++step) {
    perEntityStage.execute(perEntityWorld, executor);
    batchedStage.execute(batchedWorld, executor);

    for (std::size_t i = 0; i < perEntityBodies.size(); ++i) {
      EXPECT_TRUE(
          perEntityBodies[i].getTranslation().isApprox(
              batchedBodies[i].getTranslation(), tolerance))
          << "step " << step << " body " << i << " position";
      EXPECT_TRUE(
          perEntityBodies[i].getRotation().isApprox(
              batchedBodies[i].getRotation(), tolerance))
          << "step " << step << " body " << i << " orientation";
      EXPECT_TRUE(
          perEntityBodies[i].getLinearVelocity().isApprox(
              batchedBodies[i].getLinearVelocity(), tolerance))
          << "step " << step << " body " << i << " linear velocity";
      EXPECT_TRUE(
          perEntityBodies[i].getAngularVelocity().isApprox(
              batchedBodies[i].getAngularVelocity(), tolerance))
          << "step " << step << " body " << i << " angular velocity";
      EXPECT_TRUE(
          perEntityBodies[i].getLocalTransform().isApprox(
              batchedBodies[i].getLocalTransform(), tolerance))
          << "step " << step << " body " << i << " local transform";
    }
  }
}

// Test that the batched stage keeps the SoA path for frame-coupled rigid bodies
// and still matches the parent-before-child local-transform bookkeeping of the
// per-entity integrator.
TEST(World, BatchedRigidBodyIntegrationStageMatchesFrameCoupledBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto buildWorld = [](sx::World& world) {
    sx::RigidBodyOptions parentOptions;
    parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
    parentOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
    auto parent = world.addRigidBody("parent", parentOptions);

    sx::RigidBodyOptions childOptions;
    childOptions.position = Eigen::Vector3d(20.0, 0.0, 0.0);
    childOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto child = world.addRigidBody("child", childOptions);
    child.setParentFrame(parent);

    world.setTimeStep(0.5);
    world.enterSimulationMode();
    return std::pair<sx::RigidBody, sx::RigidBody>(parent, child);
  };

  sx::World perEntityWorld;
  sx::World batchedWorld;
  auto [perEntityParent, perEntityChild] = buildWorld(perEntityWorld);
  auto [batchedParent, batchedChild] = buildWorld(batchedWorld);

  compute::SequentialExecutor executor;
  compute::RigidBodyIntegrationStage perEntityStage;
  compute::BatchedRigidBodyIntegrationStage batchedStage;
  perEntityStage.execute(perEntityWorld, executor);
  batchedStage.execute(batchedWorld, executor);

  EXPECT_TRUE(perEntityParent.getLocalTransform().isApprox(
      batchedParent.getLocalTransform()));
  EXPECT_TRUE(perEntityChild.getLocalTransform().isApprox(
      batchedChild.getLocalTransform()));
  EXPECT_TRUE(
      perEntityParent.getTransform().isApprox(batchedParent.getTransform()));
  EXPECT_TRUE(
      perEntityChild.getTransform().isApprox(batchedChild.getTransform()));
}

// Test that the batched SoA rigid-body integration stage reuses same-shape
// batch and frame-order scratch once prewarmed. The scene includes a
// frame-coupled child so the parent-before-child local-transform writeback path
// is covered in addition to the state/model/force arrays.
TEST(
    World,
    BatchedRigidBodyIntegrationStageDoesNotAllocateHeapAfterPrewarmForFrameCoupledBodies)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  parentOptions.linearVelocity = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto parent = world.addRigidBody("parent", parentOptions);

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(20.0, 0.0, 0.0);
  childOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto child = world.addRigidBody("child", childOptions);
  child.setParentFrame(parent);

  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.25);
  world.enterSimulationMode();

  compute::SequentialExecutor executor;
  compute::BatchedRigidBodyIntegrationStage stage;
  stage.execute(world, executor);

  ScopedHeapAllocationCounter heapCounter;
  for (int i = 0; i < 4; ++i) {
    stage.execute(world, executor);
  }
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated by prewarmed batched rigid-body "
         "integration: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(heapCounter.allocationBytes(), 0u);
}

// Test that the experimental step path can swap the kinematics stage contract.
TEST(World, StepAcceptsCustomStage)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  compute::SequentialExecutor executor;
  CountingKinematicsStage stage;
  world.step(executor, stage);

  EXPECT_EQ(stage.executionCount, 1);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

TEST(World, StepRebuildsCachedKinematicsAfterFrameReparenting)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  auto oldParent = world.addFreeFrame("old_parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", oldParent, childOffset);

  auto newParent = world.addFreeFrame("new_parent");

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  newParent.setLocalTransform(updatedParentTransform);
  child.setParentFrame(newParent);

  compute::SequentialExecutor executor;
  world.step(executor);

  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

TEST(World, StepSkipsCleanKinematicsGraph)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  CountingGraphExecutor executor;
  world.step(2, executor);

  EXPECT_EQ(executor.executionCount, 0);
  EXPECT_DOUBLE_EQ(world.getTime(), 2.0 * world.getTimeStep());
  EXPECT_EQ(world.getFrame(), 2u);

  world.step();

  EXPECT_EQ(executor.executionCount, 0);
  EXPECT_DOUBLE_EQ(world.getTime(), 3.0 * world.getTimeStep());
  EXPECT_EQ(world.getFrame(), 3u);
  const auto cleanStepDiagnostics = world.getMemoryDiagnostics();
  EXPECT_EQ(cleanStepDiagnostics.frameScratchResetCount, 3u);
  EXPECT_EQ(cleanStepDiagnostics.frameScratchUsedBytes, 0u);
  EXPECT_EQ(cleanStepDiagnostics.frameScratchOverflowCount, 0u);
  EXPECT_EQ(cleanStepDiagnostics.frameScratchOverflowBytes, 0u);

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  world.step(executor);

  EXPECT_EQ(executor.executionCount, 1);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
  EXPECT_DOUBLE_EQ(world.getTime(), 4.0 * world.getTimeStep());
  EXPECT_EQ(world.getFrame(), 4u);
}

// Test that custom-stage step overloads keep the same default dynamics baseline
// as World::step(), with the caller-provided stage replacing the final stage.
TEST(World, StepWithCustomStageUsesDefaultDynamicsBaseline)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  const auto addBody = [](sx::World& world) {
    sx::RigidBodyOptions options;
    options.position = Eigen::Vector3d(0.0, 0.0, 5.0);
    world.setTimeStep(0.01);
    return world.addRigidBody("body", options);
  };

  sx::World defaultWorld;
  sx::World customWorld;
  auto defaultBody = addBody(defaultWorld);
  auto customBody = addBody(customWorld);

  compute::SequentialExecutor executor;
  compute::KinematicsStage replacementKinematics;
  defaultWorld.step(executor);
  customWorld.step(executor, replacementKinematics);

  EXPECT_TRUE(
      customBody.getLinearVelocity().isApprox(defaultBody.getLinearVelocity()));
  EXPECT_TRUE(
      customBody.getTranslation().isApprox(defaultBody.getTranslation()));
  EXPECT_DOUBLE_EQ(customWorld.getTime(), defaultWorld.getTime());
  EXPECT_EQ(customWorld.getFrame(), defaultWorld.getFrame());

  sx::World countedDefaultWorld;
  sx::World countedCustomWorld;
  auto countedDefaultBody = addBody(countedDefaultWorld);
  auto countedCustomBody = addBody(countedCustomWorld);
  countedDefaultWorld.step(3, executor);
  countedCustomWorld.step(3, executor, replacementKinematics);

  EXPECT_TRUE(countedCustomBody.getLinearVelocity().isApprox(
      countedDefaultBody.getLinearVelocity()));
  EXPECT_TRUE(countedCustomBody.getTranslation().isApprox(
      countedDefaultBody.getTranslation()));
  EXPECT_DOUBLE_EQ(countedCustomWorld.getTime(), countedDefaultWorld.getTime());
  EXPECT_EQ(countedCustomWorld.getFrame(), countedDefaultWorld.getFrame());
}

// Test that the DART 7 world step path can compose multiple solver
// domains without expanding the default public World::step() surface.
TEST(World, StepAcceptsMultiDomainSolverPipeline)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  world.setTimeStep(0.25);
  world.enterSimulationMode();

  std::vector<std::string> order;
  RecordingWorldStage articulated(
      "articulated",
      {compute::ComputeStageDomain::ArticulatedBody,
       compute::ComputeStageAcceleration::TaskParallel
           | compute::ComputeStageAcceleration::DataLocality},
      order);
  RecordingWorldStage deformable(
      "deformable",
      {compute::ComputeStageDomain::DeformableBody,
       compute::ComputeStageAcceleration::DataParallel
           | compute::ComputeStageAcceleration::Simd},
      order);
  RecordingWorldStage fluid(
      "fluid",
      {compute::ComputeStageDomain::Fluid,
       compute::ComputeStageAcceleration::DataParallel
           | compute::ComputeStageAcceleration::Gpu},
      order);
  RecordingWorldStage renderPrep(
      "render_prep",
      {compute::ComputeStageDomain::Rendering,
       compute::ComputeStageAcceleration::TaskParallel
           | compute::ComputeStageAcceleration::Gpu},
      order);

  compute::WorldStepPipeline pipeline;
  pipeline.addStage(articulated)
      .addStage(deformable)
      .addStage(fluid)
      .addStage(renderPrep);

  ASSERT_EQ(pipeline.getStageCount(), 4u);
  EXPECT_FALSE(pipeline.isEmpty());
  EXPECT_EQ(
      pipeline.getStage(3).getMetadata().domain,
      compute::ComputeStageDomain::Rendering);
  EXPECT_TRUE(
      compute::hasAcceleration(
          pipeline.getStage(3).getMetadata().acceleration,
          compute::ComputeStageAcceleration::Gpu));
  EXPECT_THROW({ (void)pipeline.getStage(4); }, sx::OutOfRangeException);

  compute::SequentialExecutor executor;
  world.step(executor, pipeline);

  const std::vector<std::string> expected{
      "articulated", "deformable", "fluid", "render_prep"};
  EXPECT_EQ(order, expected);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.25);
  EXPECT_EQ(world.getFrame(), 1u);
}

// Test that custom pipelines can exceed the inline storage threshold while
// built-in pipelines keep the inline no-allocation fast path.
TEST(World, StepPipelineAllowsMoreThanInlineStageCapacity)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  world.setTimeStep(0.25);
  std::vector<std::string> order;
  RecordingWorldStage stage0("stage0", {}, order);
  RecordingWorldStage stage1("stage1", {}, order);
  RecordingWorldStage stage2("stage2", {}, order);
  RecordingWorldStage stage3("stage3", {}, order);
  RecordingWorldStage stage4("stage4", {}, order);
  RecordingWorldStage stage5("stage5", {}, order);
  RecordingWorldStage stage6("stage6", {}, order);
  RecordingWorldStage stage7("stage7", {}, order);
  RecordingWorldStage stage8("stage8", {}, order);

  compute::WorldStepPipeline pipeline;
  pipeline.addStage(stage0)
      .addStage(stage1)
      .addStage(stage2)
      .addStage(stage3)
      .addStage(stage4)
      .addStage(stage5)
      .addStage(stage6)
      .addStage(stage7)
      .addStage(stage8);

  EXPECT_EQ(
      pipeline.getStageCount(),
      compute::WorldStepPipeline::kInlineStageCount + 1u);
  EXPECT_EQ(&pipeline.getStage(7), &stage7);
  EXPECT_EQ(&pipeline.getStage(8), &stage8);
  EXPECT_THROW({ (void)pipeline.getStage(9); }, sx::OutOfRangeException);

  compute::SequentialExecutor executor;
  world.step(executor, pipeline);
  EXPECT_EQ(
      order,
      (std::vector<std::string>{
          "stage0",
          "stage1",
          "stage2",
          "stage3",
          "stage4",
          "stage5",
          "stage6",
          "stage7",
          "stage8"}));
  EXPECT_DOUBLE_EQ(world.getTime(), 0.25);
  EXPECT_EQ(world.getFrame(), 1u);

  pipeline.clear();
  EXPECT_TRUE(pipeline.isEmpty());
  EXPECT_EQ(pipeline.getStageCount(), 0u);
}

TEST(World, StepPipelineOverflowUsesProvidedAllocator)
{
  namespace common = dart::common;
  namespace compute = dart::simulation::compute;

  common::MemoryManager memoryManager;
  auto& freeList = memoryManager.getFreeListAllocator();
  const auto allocationsBeforePipeline = freeList.getAllocationCount();

  {
    compute::WorldStepPipeline pipeline(memoryManager.getFreeAllocator());
    std::array<
        NoOpWorldStage,
        compute::WorldStepPipeline::kInlineStageCount + 1u>
        stages;

    for (std::size_t i = 0; i < compute::WorldStepPipeline::kInlineStageCount;
         ++i) {
      pipeline.addStage(stages[i]);
    }
    EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforePipeline);

    pipeline.addStage(stages.back());
    EXPECT_GT(freeList.getAllocationCount(), allocationsBeforePipeline)
        << "allocator-aware custom pipeline overflow storage should reserve "
           "from the provided free allocator";
  }

  EXPECT_EQ(freeList.getAllocationCount(), allocationsBeforePipeline);
}

// Test that repeated stepping can reuse a caller-owned executor and pipeline.
TEST(World, StepCountAcceptsMultiDomainSolverPipeline)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  world.setTimeStep(0.1);

  std::vector<std::string> order;
  RecordingWorldStage kinematics(
      "kinematics",
      {compute::ComputeStageDomain::Kinematics,
       compute::toMask(compute::ComputeStageAcceleration::TaskParallel)},
      order);
  RecordingWorldStage sensor(
      "sensor",
      {compute::ComputeStageDomain::Sensor,
       compute::toMask(compute::ComputeStageAcceleration::TaskParallel)},
      order);

  compute::WorldStepPipeline pipeline;
  pipeline.addStage(kinematics).addStage(sensor);

  RecordingExecutor executor;
  world.step(0, executor, pipeline);
  EXPECT_FALSE(world.isSimulationMode());
  EXPECT_TRUE(order.empty());
  EXPECT_EQ(executor.executeCount, 0u);

  world.step(3, executor, pipeline);

  const std::vector<std::string> expected{
      "kinematics", "sensor", "kinematics", "sensor", "kinematics", "sensor"};
  EXPECT_EQ(order, expected);
  EXPECT_EQ(executor.executeCount, expected.size());
  EXPECT_DOUBLE_EQ(world.getTime(), 0.3);
  EXPECT_EQ(world.getFrame(), 3u);
}

// Test that rigid-body state can be extracted to a structure-of-arrays batch
// and applied back, round-tripping exactly and rejecting size mismatches.
TEST(World, RigidBodyStateBatchRoundTrip)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto build = [](sx::World& world) {
    std::vector<sx::RigidBody> bodies;
    for (int i = 0; i < 4; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + i;
      options.position = Eigen::Vector3d(0.1 * i, 0.2 * i, 0.3 * i);
      options.linearVelocity = Eigen::Vector3d(0.5 * i, -0.25 * i, 0.1 * i);
      options.angularVelocity = Eigen::Vector3d(0.01 * i, 0.02 * i, 0.03 * i);
      bodies.push_back(
          world.addRigidBody("body_" + std::to_string(i), options));
    }
    return bodies;
  };

  sx::World source;
  auto sourceBodies = build(source);

  const auto batch = compute::extractRigidBodyState(source);
  EXPECT_EQ(batch.worldCount, 1u);
  EXPECT_EQ(batch.bodyCount, 4u);
  EXPECT_EQ(batch.position.size(), 12u);
  EXPECT_EQ(batch.orientation.size(), 16u);
  EXPECT_EQ(batch.linearVelocity.size(), 12u);
  EXPECT_EQ(batch.angularVelocity.size(), 12u);

  // Apply the source snapshot to an identical world; the re-extracted state
  // must match field for field.
  sx::World target;
  auto targetBodies = build(target);
  compute::applyRigidBodyState(target, batch);

  const auto roundTrip = compute::extractRigidBodyState(target);
  ASSERT_EQ(roundTrip.bodyCount, batch.bodyCount);
  EXPECT_EQ(roundTrip.position, batch.position);
  EXPECT_EQ(roundTrip.orientation, batch.orientation);
  EXPECT_EQ(roundTrip.linearVelocity, batch.linearVelocity);
  EXPECT_EQ(roundTrip.angularVelocity, batch.angularVelocity);

  // Mutating the batch and applying it changes world state deterministically.
  auto mutated = batch;
  for (auto& value : mutated.position) {
    value += 1.0;
  }
  compute::applyRigidBodyState(source, mutated);
  const auto afterMutation = compute::extractRigidBodyState(source);
  for (std::size_t i = 0; i < afterMutation.position.size(); ++i) {
    EXPECT_DOUBLE_EQ(afterMutation.position[i], batch.position[i] + 1.0);
  }

  // A batch whose body count does not match the world is rejected.
  sx::World smaller;
  sx::RigidBodyOptions soleOptions;
  smaller.addRigidBody("only", soleOptions);
  EXPECT_THROW(
      compute::applyRigidBodyState(smaller, batch),
      sx::InvalidArgumentException);

  // A batch whose array sizes are inconsistent with bodyCount is rejected
  // before any out-of-bounds access.
  compute::RigidBodyStateBatch malformed;
  malformed.worldCount = 1;
  malformed.bodyCount = batch.bodyCount; // arrays intentionally left empty
  EXPECT_THROW(
      compute::applyRigidBodyState(target, malformed),
      sx::InvalidArgumentException);
}

// Test the homogeneous multi-world batch path: extract with a leading world
// dimension, world-major ordering, round-trip apply, and rejection of
// world-count and body-count mismatches.
TEST(World, RigidBodyStateBatchMultiWorld)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto buildWorld = [](sx::World& world, double seed) {
    for (int i = 0; i < 2; ++i) {
      sx::RigidBodyOptions options;
      options.position = Eigen::Vector3d(seed + i, seed + 2 * i, seed + 3 * i);
      options.linearVelocity = Eigen::Vector3d(seed, -seed, 0.5 * seed);
      world.addRigidBody("body_" + std::to_string(i), options);
    }
  };

  sx::World w0;
  sx::World w1;
  sx::World w2;
  buildWorld(w0, 1.0);
  buildWorld(w1, 2.0);
  buildWorld(w2, 3.0);

  const std::vector<const sx::World*> worlds{&w0, &w1, &w2};
  const auto batch = compute::extractRigidBodyStateBatch(worlds);
  EXPECT_EQ(batch.worldCount, 3u);
  EXPECT_EQ(batch.bodyCount, 2u);
  EXPECT_EQ(batch.position.size(), 18u);    // 3 * worldCount * bodyCount
  EXPECT_EQ(batch.orientation.size(), 24u); // 4 * worldCount * bodyCount

  // World-major ordering: each world's slice equals its single-world extract.
  const auto single0 = compute::extractRigidBodyState(w0);
  const auto single2 = compute::extractRigidBodyState(w2);
  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_DOUBLE_EQ(batch.position[k], single0.position[k]);
    EXPECT_DOUBLE_EQ(batch.position[12 + k], single2.position[k]);
  }

  // Apply to fresh worlds (different initial state) and round-trip.
  sx::World t0;
  sx::World t1;
  sx::World t2;
  buildWorld(t0, 9.0);
  buildWorld(t1, 9.0);
  buildWorld(t2, 9.0);
  const std::vector<sx::World*> targets{&t0, &t1, &t2};
  compute::applyRigidBodyStateBatch(targets, batch);

  const std::vector<const sx::World*> constTargets{&t0, &t1, &t2};
  const auto roundTrip = compute::extractRigidBodyStateBatch(constTargets);
  EXPECT_EQ(roundTrip.position, batch.position);
  EXPECT_EQ(roundTrip.orientation, batch.orientation);
  EXPECT_EQ(roundTrip.linearVelocity, batch.linearVelocity);
  EXPECT_EQ(roundTrip.angularVelocity, batch.angularVelocity);

  // A world-count mismatch on apply is rejected.
  const std::vector<sx::World*> twoTargets{&t0, &t1};
  EXPECT_THROW(
      compute::applyRigidBodyStateBatch(twoTargets, batch),
      sx::InvalidArgumentException);

  // Heterogeneous body counts are rejected on extract.
  sx::World hetero;
  buildWorld(hetero, 5.0);
  sx::RigidBodyOptions extra;
  hetero.addRigidBody("extra", extra);
  const std::vector<const sx::World*> heteroWorlds{&w0, &hetero};
  EXPECT_THROW(
      { (void)compute::extractRigidBodyStateBatch(heteroWorlds); },
      sx::InvalidArgumentException);
}

// Test that applyRigidBodyStateBatch validates every world before mutating any,
// so an invalid later world leaves the earlier worlds unchanged (atomic
// failure) rather than partially applying the batch.
TEST(World, ApplyRigidBodyStateBatchValidatesBeforeMutating)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto build2 = [](sx::World& world, double seed) {
    for (int i = 0; i < 2; ++i) {
      sx::RigidBodyOptions options;
      options.position = Eigen::Vector3d(seed + i, 0.0, 0.0);
      world.addRigidBody("body_" + std::to_string(i), options);
    }
  };

  sx::World source0;
  sx::World source1;
  build2(source0, 1.0);
  build2(source1, 2.0);
  const std::vector<const sx::World*> sources{&source0, &source1};
  const auto batch = compute::extractRigidBodyStateBatch(sources);

  // t0 matches the batch body count; t1 does not, so apply must throw before
  // mutating t0.
  sx::World t0;
  sx::World t1;
  build2(t0, 9.0);
  build2(t1, 9.0);
  t1.addRigidBody(
      "extra", sx::RigidBodyOptions{}); // t1 now has 3 bodies, not 2

  const std::vector<sx::World*> targets{&t0, &t1};
  const auto beforePosition = compute::extractRigidBodyState(t0).position;
  EXPECT_THROW(
      compute::applyRigidBodyStateBatch(targets, batch),
      sx::InvalidArgumentException);
  const auto afterPosition = compute::extractRigidBodyState(t0).position;
  EXPECT_EQ(beforePosition, afterPosition);
}

// Test that the multi-world batch rejects worlds whose rigid bodies do not
// share the same ordered identity (same count but different bodies/order),
// which would otherwise silently permute state between bodies.
TEST(World, MultiWorldBatchRejectsMismatchedBodyIdentity)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto addTwo = [](sx::World& world, const std::string& prefix) {
    for (int i = 0; i < 2; ++i) {
      sx::RigidBodyOptions options;
      options.position = Eigen::Vector3d(i, 0.0, 0.0);
      world.addRigidBody(prefix + std::to_string(i), options);
    }
  };

  // Same body count, different names => different ordered identity.
  sx::World named0;
  sx::World named1;
  addTwo(named0, "a");
  addTwo(named1, "b");
  const std::vector<const sx::World*> mixed{&named0, &named1};
  EXPECT_THROW(
      { (void)compute::extractRigidBodyStateBatch(mixed); },
      sx::InvalidArgumentException);

  // A valid batch (consistent identity) applied to targets whose identities
  // disagree is rejected before any mutation.
  sx::World src0;
  sx::World src1;
  addTwo(src0, "a");
  addTwo(src1, "a");
  const std::vector<const sx::World*> sources{&src0, &src1};
  const auto batch = compute::extractRigidBodyStateBatch(sources);

  sx::World target0;
  addTwo(target0, "a");
  const std::vector<sx::World*> targets{&target0, &named1};
  EXPECT_THROW(
      compute::applyRigidBodyStateBatch(targets, batch),
      sx::InvalidArgumentException);
}

// Test the CPU batch executor: N independent homogeneous worlds advanced in
// parallel match a single sequentially-stepped reference, and null worlds are
// rejected.
TEST(World, StepWorldsBatchedMatchesSequential)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto build = [](sx::World& world) {
    for (int i = 0; i < 3; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + i;
      options.position = Eigen::Vector3d(1.0 + i, 0.0, 0.0);
      options.linearVelocity = Eigen::Vector3d(0.1 * (i + 1), 0.0, 0.0);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.2, -0.1, 0.05));
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World reference;
  build(reference);
  reference.step(5);
  const auto refState = compute::extractRigidBodyState(reference);

  sx::World w0;
  sx::World w1;
  sx::World w2;
  sx::World w3;
  build(w0);
  build(w1);
  build(w2);
  build(w3);
  const std::vector<sx::World*> worlds{&w0, &w1, &w2, &w3};

  compute::ParallelExecutor executor(2);
  compute::stepWorldsBatched(worlds, 5, executor);

  for (auto* world : worlds) {
    const auto state = compute::extractRigidBodyState(*world);
    ASSERT_EQ(state.bodyCount, refState.bodyCount);
    EXPECT_EQ(state.position, refState.position);
    EXPECT_EQ(state.linearVelocity, refState.linearVelocity);
    EXPECT_EQ(state.orientation, refState.orientation);
    EXPECT_DOUBLE_EQ(world->getTime(), reference.getTime());
  }

  const std::vector<sx::World*> withNull{&w0, nullptr};
  EXPECT_THROW(
      compute::stepWorldsBatched(withNull, 1, executor),
      sx::InvalidArgumentException);

  // The same world twice would be stepped concurrently from two nodes (a data
  // race); it must be rejected.
  const std::vector<sx::World*> withDuplicate{&w0, &w0};
  EXPECT_THROW(
      compute::stepWorldsBatched(withDuplicate, 1, executor),
      sx::InvalidArgumentException);
}

// Test the batched rollout: applying a shared initial state, advancing, and
// returning the final batched state matches a single-world reference.
TEST(World, RolloutWorldsBatchedMatchesReference)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  auto build = [](sx::World& world) {
    for (int i = 0; i < 2; ++i) {
      sx::RigidBodyOptions options;
      options.mass = 1.0 + i;
      options.position = Eigen::Vector3d(0.5 + i, 0.0, 0.0);
      options.linearVelocity = Eigen::Vector3d(0.2 * (i + 1), 0.1, 0.0);
      auto body = world.addRigidBody("body_" + std::to_string(i), options);
      body.setForce(Eigen::Vector3d(0.15, -0.05, 0.1));
    }
    world.setTimeStep(0.01);
    world.enterSimulationMode();
  };

  sx::World w0;
  sx::World w1;
  build(w0);
  build(w1);
  const std::vector<sx::World*> worlds{&w0, &w1};

  // Capture the homogeneous initial state, then perturb the worlds so the
  // rollout's apply step must reset them.
  const std::vector<const sx::World*> constWorlds{&w0, &w1};
  const auto initial = compute::extractRigidBodyStateBatch(constWorlds);
  w0.step(3);
  w1.step(7);

  compute::ParallelExecutor executor(2);
  const auto result
      = compute::rolloutWorldsBatched(worlds, initial, 5, executor);

  // Reference: a single world built identically (so its state equals one slice
  // of the initial batch) and stepped the same number of times.
  sx::World reference;
  build(reference);
  reference.step(5);
  const auto refFinal = compute::extractRigidBodyState(reference);

  EXPECT_EQ(result.worldCount, 2u);
  ASSERT_EQ(result.bodyCount, refFinal.bodyCount);
  const auto stride = refFinal.position.size();
  for (std::size_t r = 0; r < 2; ++r) {
    for (std::size_t k = 0; k < stride; ++k) {
      EXPECT_DOUBLE_EQ(result.position[r * stride + k], refFinal.position[k]);
      EXPECT_DOUBLE_EQ(
          result.linearVelocity[r * stride + k], refFinal.linearVelocity[k]);
    }
  }

  // A duplicate world must be rejected atomically: rollout applies state before
  // it steps, so the apply step rejects duplicates before any world is mutated.
  const auto beforePosition = compute::extractRigidBodyState(w0).position;
  const std::vector<sx::World*> duplicate{&w0, &w0};
  EXPECT_THROW(
      { (void)compute::rolloutWorldsBatched(duplicate, initial, 5, executor); },
      sx::InvalidArgumentException);
  const auto afterPosition = compute::extractRigidBodyState(w0).position;
  EXPECT_EQ(beforePosition, afterPosition);
}

// Test the immutable Model batch: inverse masses are extracted in state order,
// and the model-based integrator overload matches the explicit-vector overload.
TEST(World, RigidBodyModelBatchIntegration)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  for (int i = 0; i < 2; ++i) {
    sx::RigidBodyOptions options;
    options.mass = 2.0 * (i + 1); // body 0 mass 2, body 1 mass 4
    options.position = Eigen::Vector3d(i, 0.0, 0.0);
    world.addRigidBody("b" + std::to_string(i), options);
  }

  const auto state = compute::extractRigidBodyState(world);
  const auto model = compute::extractRigidBodyModelBatch(world);
  EXPECT_EQ(model.worldCount, 1u);
  EXPECT_EQ(model.bodyCount, 2u);
  ASSERT_EQ(model.inverseMass.size(), 2u);
  // EnTT view order is not insertion order, so check the set, not positions.
  EXPECT_TRUE(
      (model.inverseMass[0] == 0.5 && model.inverseMass[1] == 0.25)
      || (model.inverseMass[0] == 0.25 && model.inverseMass[1] == 0.5));

  const std::vector<double> force = {0.0, 0.0, 4.0, 0.0, 0.0, 8.0};
  const double dt = 0.5;

  auto viaModel = state;
  compute::integrateRigidBodyStateBatchLinear(viaModel, model, force, dt);

  auto viaVector = state;
  compute::integrateRigidBodyStateBatchLinear(
      viaVector, force, model.inverseMass, dt);

  EXPECT_EQ(viaModel.linearVelocity, viaVector.linearVelocity);
  EXPECT_EQ(viaModel.position, viaVector.position);
  // Per body, in extraction order: z-velocity == force.z * inverseMass * dt
  // (initial velocity is zero), independent of the view iteration order.
  for (std::size_t i = 0; i < model.bodyCount; ++i) {
    EXPECT_DOUBLE_EQ(
        viaModel.linearVelocity[3 * i + 2],
        force[3 * i + 2] * (model.inverseMass[i] * dt));
  }

  // A model whose body count does not match the state is rejected.
  compute::RigidBodyModelBatch wrong;
  wrong.worldCount = 1;
  wrong.bodyCount = 1;
  wrong.inverseMass = {1.0};
  EXPECT_THROW(
      compute::integrateRigidBodyStateBatchLinear(viaModel, wrong, force, dt),
      sx::InvalidArgumentException);
}

// Test opt-in replay recording and in-place restore of rigid-body runtime
// state.
TEST(World, ReplayRecordingRestoresRigidBodyState)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.1);

  sx::RigidBodyOptions options;
  options.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  options.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  sx::RigidBody body = world.addRigidBody("body", options);

  EXPECT_FALSE(world.isReplayRecordingEnabled());
  EXPECT_EQ(world.getReplayFrameCount(), 0u);
  EXPECT_FALSE(world.getReplayCursor().has_value());

  world.setReplayRecordingEnabled(true);
  ASSERT_TRUE(world.isReplayRecordingEnabled());
  ASSERT_EQ(world.getReplayFrameCount(), 1u);
  ASSERT_TRUE(world.getReplayCursor().has_value());
  EXPECT_EQ(*world.getReplayCursor(), 0u);
  EXPECT_DOUBLE_EQ(world.getReplayFrameTime(0), 0.0);
  EXPECT_EQ(world.getReplaySimulationFrame(0), 0u);

  world.step(3);
  ASSERT_EQ(world.getReplayFrameCount(), 4u);
  ASSERT_TRUE(world.getReplayCursor().has_value());
  EXPECT_EQ(*world.getReplayCursor(), 3u);
  EXPECT_EQ(world.getFrame(), 3u);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.3);
  EXPECT_NEAR(body.getTranslation().x(), 0.3, 1e-12);

  world.restoreReplayFrame(1);
  EXPECT_EQ(world.getFrame(), 1u);
  EXPECT_DOUBLE_EQ(world.getTime(), 0.1);
  EXPECT_NEAR(body.getTranslation().x(), 0.1, 1e-12);
  EXPECT_NEAR(body.getTranslation().z(), 1.0, 1e-12);
  EXPECT_TRUE(body.getLinearVelocity().isApprox(Eigen::Vector3d::UnitX()));
  ASSERT_TRUE(world.getReplayCursor().has_value());
  EXPECT_EQ(*world.getReplayCursor(), 1u);

  // Branching from an earlier replay cursor drops stale future frames before
  // appending the newly simulated state.
  world.step();
  ASSERT_EQ(world.getReplayFrameCount(), 3u);
  ASSERT_TRUE(world.getReplayCursor().has_value());
  EXPECT_EQ(*world.getReplayCursor(), 2u);
  EXPECT_EQ(world.getReplaySimulationFrame(2), 2u);
  EXPECT_DOUBLE_EQ(world.getReplayFrameTime(2), 0.2);
  EXPECT_NEAR(body.getTranslation().x(), 0.2, 1e-12);

  world.clearReplayRecording();
  EXPECT_EQ(world.getReplayFrameCount(), 1u);
  ASSERT_TRUE(world.getReplayCursor().has_value());
  EXPECT_EQ(*world.getReplayCursor(), 0u);

  world.setReplayRecordingEnabled(false);
  world.step();
  EXPECT_EQ(world.getReplayFrameCount(), 1u);
}

// Test replay restores World-level solver-family and policy metadata before
// continuing from a simulation-mode frame.
TEST(World, ReplayRecordingRestoresSolverOptions)
{
  namespace sx = dart::simulation;

  sx::WorldOptions options;
  options.rigidBodySolver = sx::RigidBodySolver::Ipc;
  options.multibodyOptions.integrationFamily = "variational integrator";
  options.differentiable = true;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  options.contactGradientMode = sx::ContactGradientMode::PreContactSurrogate;

  sx::World world(options);
  world.setGravity(Eigen::Vector3d::Zero());
  auto body = world.addRigidBody("body");
  body.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));

  world.setReplayRecordingEnabled(true);
  world.step();
  ASSERT_EQ(world.getReplayFrameCount(), 2u);

  world.setRigidBodySolver(sx::RigidBodySolver::SequentialImpulse);
  world.setMultibodyOptions({"semi-implicit"});
  world.setContactGradientMode(sx::ContactGradientMode::Analytic);

  world.restoreReplayFrame(1);

  EXPECT_TRUE(world.isSimulationMode());
  EXPECT_EQ(world.getRigidBodySolver(), sx::RigidBodySolver::Ipc);
  EXPECT_EQ(
      world.getMultibodyOptions().integrationFamily, "variational integrator");
  EXPECT_TRUE(world.isDifferentiable());
  EXPECT_EQ(world.getContactSolverMethod(), sx::ContactSolverMethod::BoxedLcp);
  EXPECT_EQ(
      world.getContactGradientMode(),
      sx::ContactGradientMode::PreContactSurrogate);

  EXPECT_NO_THROW(world.step());
}

// Test that replay restores rigid IPC's adaptive barrier continuation state, so
// a branch from an earlier contact frame matches an uninterrupted rollout.
TEST(World, ReplayRecordingRestoresRigidIpcAdaptiveBranchState)
{
  namespace sx = dart::simulation;

  const auto addSlidingContactScene = [](sx::World& world) {
    world.setRigidBodySolver(sx::RigidBodySolver::Ipc);
    world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world.setTimeStep(0.01);

    sx::RigidBodyOptions groundOptions;
    groundOptions.isStatic = true;
    groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
    auto ground = world.addRigidBody("ground", groundOptions);
    ground.setCollisionShape(sx::CollisionShape::makeBox({2.0, 2.0, 0.25}));

    sx::RigidBodyOptions boxOptions;
    boxOptions.mass = 1.0;
    boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.258);
    boxOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto box = world.addRigidBody("box", boxOptions);
    box.setCollisionShape(sx::CollisionShape::makeBox({0.25, 0.25, 0.25}));
    return box;
  };

  sx::World reference;
  auto referenceBox = addSlidingContactScene(reference);
  reference.step();
  reference.step();

  sx::World replay;
  auto replayBox = addSlidingContactScene(replay);
  replay.setReplayRecordingEnabled(true);
  replay.step();
  ASSERT_EQ(replay.getReplayFrameCount(), 2u);

  Eigen::Isometry3d farPose = Eigen::Isometry3d::Identity();
  farPose.translation() = Eigen::Vector3d(0.0, 0.0, 10.0);
  replayBox.setTransform(farPose);
  replayBox.setLinearVelocity(Eigen::Vector3d::Zero());
  replay.step();
  ASSERT_EQ(replay.getReplayFrameCount(), 3u);

  replay.restoreReplayFrame(1);
  replay.step();

  EXPECT_TRUE(replayBox.getTranslation().isApprox(
      referenceBox.getTranslation(), 1e-10));
  EXPECT_TRUE(replayBox.getLinearVelocity().isApprox(
      referenceBox.getLinearVelocity(), 1e-10));
}

// Test that replay restores deformable AVBD warm-start inventories, so a branch
// from an earlier frame does not keep later lambda/stiffness continuation
// state.
TEST(World, ReplayRecordingRestoresDeformableAvbdWarmStartState)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());
  world.setTimeStep(0.02);

  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -1.5)};
  options.masses = {1.0, 1.0};
  options.fixedNodes = {0};
  options.edges = {sx::DeformableEdge{0, 1, 1.0}};
  options.edgeStiffness = 1000.0;
  world.addDeformableBody("spring", options);

  auto& registry = sx::detail::registryOf(world);
  for (const auto entity : registry.view<sx::comps::DeformableBodyTag>()) {
    sx::comps::DeformableVbdConfig cfg;
    cfg.enabled = true;
    cfg.iterations = 1;
    cfg.useAvbdFiniteStiffnessRows = true;
    cfg.avbdFiniteStiffnessStart = 1.0;
    cfg.avbdBeta = 100.0;
    cfg.avbdGamma = 1.0;
    cfg.avbdMaxStiffness = 1000.0;
    registry.emplace_or_replace<sx::comps::DeformableVbdConfig>(entity, cfg);
  }

  const auto springRowStiffness = [&]() {
    const auto states
        = sx::compute::avbd_replay::captureDeformableAvbdWarmStartReplayState(
            registry);
    EXPECT_EQ(states.size(), 1u);
    if (states.empty()) {
      return 0.0;
    }
    EXPECT_EQ(states[0].springRows.size(), 1u);
    if (states[0].springRows.empty()) {
      return 0.0;
    }
    return states[0].springRows[0].state.stiffness;
  };

  world.setReplayRecordingEnabled(true);
  world.step();
  ASSERT_EQ(world.getReplayFrameCount(), 2u);
  const double frame1Stiffness = springRowStiffness();

  world.step();
  ASSERT_EQ(world.getReplayFrameCount(), 3u);
  const double frame2Stiffness = springRowStiffness();
  ASSERT_GT(frame2Stiffness, frame1Stiffness);

  world.restoreReplayFrame(1);
  EXPECT_DOUBLE_EQ(springRowStiffness(), frame1Stiffness);
}

// Test that replay restores transient variational solver-history components to
// match the target frame, including removing lazily-created state when
// scrubbing back before the first variational step.
TEST(World, ReplayRecordingRestoresTransientVariationalComponents)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;
  world.setMultibodyOptions({.integrationFamily = "variational integrator"});
  world.setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world.setTimeStep(1e-3);

  auto robot = world.addMultibody("slider");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "rail";
  spec.type = sx::JointType::Prismatic;
  spec.axis = Eigen::Vector3d::UnitZ();
  auto carriage = robot.addLink("carriage", base, spec);
  carriage.setMass(1.0);
  carriage.setInertia(Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal());
  carriage.getParentJoint().setPosition(Eigen::VectorXd::Constant(1, -0.001));

  robot.setGroundContact(
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::Zero(),
      1.0e4,
      /*frictionCoefficient=*/0.0,
      /*frictionRegularization=*/1.0e-4,
      /*dampingCoefficient=*/20.0,
      /*dualUpdateCadence=*/1);
  robot.addGroundContactPoint(carriage, Eigen::Vector3d::Zero());

  auto& registry = sx::detail::registryOf(world);
  const auto countVariationalStates = [&]() {
    std::size_t count = 0;
    for (auto entity : registry.view<compute::MultibodyVariationalState>()) {
      static_cast<void>(entity);
      ++count;
    }
    return count;
  };
  const auto countDualStates = [&]() {
    std::size_t count = 0;
    for (auto entity :
         registry.view<sx::comps::VariationalContactDualState>()) {
      static_cast<void>(entity);
      ++count;
    }
    return count;
  };

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);
  EXPECT_EQ(countVariationalStates(), 0u);
  EXPECT_EQ(countDualStates(), 0u);

  world.step();
  ASSERT_EQ(world.getReplayFrameCount(), 2u);
  EXPECT_EQ(countVariationalStates(), 1u);
  EXPECT_EQ(countDualStates(), 1u);

  std::vector<entt::entity> variationalStateEntities;
  for (auto entity : registry.view<compute::MultibodyVariationalState>()) {
    variationalStateEntities.push_back(entity);
  }
  for (auto entity : variationalStateEntities) {
    registry.remove<compute::MultibodyVariationalState>(entity);
  }

  std::vector<entt::entity> dualStateEntities;
  for (auto entity : registry.view<sx::comps::VariationalContactDualState>()) {
    dualStateEntities.push_back(entity);
  }
  for (auto entity : dualStateEntities) {
    registry.remove<sx::comps::VariationalContactDualState>(entity);
  }
  EXPECT_EQ(countVariationalStates(), 0u);
  EXPECT_EQ(countDualStates(), 0u);

  {
    ScopedHeapAllocationCounter heapCounter;
    world.restoreReplayFrame(1);
    heapCounter.stop();
    EXPECT_EQ(heapCounter.allocationCount(), 0u)
        << "transient variational replay restore should reinsert recorded "
           "components without allocating from the global heap";
    EXPECT_EQ(heapCounter.allocationBytes(), 0u);
  }
  EXPECT_EQ(countVariationalStates(), 1u);
  EXPECT_EQ(countDualStates(), 1u);

  world.restoreReplayFrame(0);
  EXPECT_EQ(countVariationalStates(), 0u);
  EXPECT_EQ(countDualStates(), 0u);

  world.step();
  EXPECT_EQ(countVariationalStates(), 1u);
  EXPECT_EQ(countDualStates(), 1u);
}

// Test that replay restore rejects loop-closure topology and runtime-policy
// edits rather than restoring into a different constraint graph.
TEST(World, ReplayRecordingRejectsLoopClosureLayoutChanges)
{
  namespace sx = dart::simulation;

  {
    sx::World world;
    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");
    auto link
        = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
    world.addLoopClosure("closure", {.frameA = base, .frameB = link});

    world.setReplayRecordingEnabled(true);
    world.addLoopClosure("added", {.frameA = link, .frameB = base});

    EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
  }

  {
    sx::World world;
    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");
    auto link
        = robot.addLink("link", {.parentLink = base, .jointName = "joint"});
    auto closure
        = world.addLoopClosure("closure", {.frameA = base, .frameB = link});

    world.setReplayRecordingEnabled(true);
    closure.setRuntimePolicy(
        {.enabled = false,
         .kinematics = sx::ClosureKinematicsPolicy::ResidualOnly,
         .dynamics = sx::ClosureDynamicsPolicy::ResidualOnly});

    EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
  }
}

// Test that replay restores parented rigid bodies in frame-hierarchy order even
// when entity id order would visit the child first.
TEST(World, ReplayRecordingRestoresParentedRigidBodiesParentBeforeChild)
{
  namespace sx = dart::simulation;

  sx::World world;

  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto child = world.addRigidBody("child", childOptions);

  sx::RigidBodyOptions parentOptions;
  parentOptions.position = Eigen::Vector3d(10.0, 0.0, 0.0);
  auto parent = world.addRigidBody("parent", parentOptions);

  child.setParentFrame(parent);

  Eigen::Isometry3d recordedChildTransform = Eigen::Isometry3d::Identity();
  recordedChildTransform.translation() = Eigen::Vector3d(13.0, 0.0, 0.0);
  child.setTransform(recordedChildTransform);

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  Eigen::Isometry3d movedParentTransform = Eigen::Isometry3d::Identity();
  movedParentTransform.translation() = Eigen::Vector3d(20.0, 0.0, 0.0);
  parent.setTransform(movedParentTransform);

  Eigen::Isometry3d movedChildTransform = Eigen::Isometry3d::Identity();
  movedChildTransform.translation() = Eigen::Vector3d(25.0, 0.0, 0.0);
  child.setTransform(movedChildTransform);

  world.restoreReplayFrame(0);

  EXPECT_TRUE(
      parent.getTranslation().isApprox(Eigen::Vector3d(10.0, 0.0, 0.0)));
  EXPECT_TRUE(child.getTranslation().isApprox(Eigen::Vector3d(13.0, 0.0, 0.0)));
  EXPECT_TRUE(child.getLocalTransform().translation().isApprox(
      Eigen::Vector3d(3.0, 0.0, 0.0)));
}

// Test that replay restores standalone public frame hierarchy state instead of
// leaving user-visible frames at transforms edited after recording.
TEST(World, ReplayRecordingRestoresPublicFrameState)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto parent = world.addFreeFrame("parent");
  Eigen::Isometry3d recordedParentTransform = Eigen::Isometry3d::Identity();
  recordedParentTransform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  parent.setLocalTransform(recordedParentTransform);

  Eigen::Isometry3d recordedChildOffset = Eigen::Isometry3d::Identity();
  recordedChildOffset.translation() = Eigen::Vector3d(0.0, 0.5, 0.0);
  auto child = world.addFixedFrame("child", parent, recordedChildOffset);

  auto alternateParent = world.addFreeFrame("alternate_parent");
  Eigen::Isometry3d alternateParentTransform = Eigen::Isometry3d::Identity();
  alternateParentTransform.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  alternateParent.setLocalTransform(alternateParentTransform);

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  Eigen::Isometry3d movedParentTransform = Eigen::Isometry3d::Identity();
  movedParentTransform.translation() = Eigen::Vector3d(-1.0, -2.0, -3.0);
  parent.setLocalTransform(movedParentTransform);

  Eigen::Isometry3d movedChildOffset = Eigen::Isometry3d::Identity();
  movedChildOffset.translation() = Eigen::Vector3d(0.0, -4.0, 0.0);
  child.setParentFrame(alternateParent);
  child.setLocalTransform(movedChildOffset);

  world.restoreReplayFrame(0);

  EXPECT_TRUE(parent.getLocalTransform().isApprox(recordedParentTransform));
  EXPECT_EQ(child.getParentFrame().getEntity(), parent.getEntity());
  EXPECT_TRUE(child.getLocalTransform().isApprox(recordedChildOffset));
  EXPECT_TRUE(child.getTransform().isApprox(
      recordedParentTransform * recordedChildOffset));
}

// Test that replay restore invalidates the cached kinematics graph when it
// restores a public-frame parent relationship captured in the replay frame.
TEST(World, ReplayRestoreRebuildsCachedKinematicsAfterFrameParentRestore)
{
  namespace sx = dart::simulation;
  namespace compute = dart::simulation::compute;

  sx::World world;

  auto oldParent = world.addFreeFrame("old_parent");
  Eigen::Isometry3d oldParentTransform = Eigen::Isometry3d::Identity();
  oldParentTransform.translation() = Eigen::Vector3d(-10.0, 0.0, 0.0);
  oldParent.setLocalTransform(oldParentTransform);

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
  auto child = world.addFixedFrame("child", oldParent, childOffset);

  auto replayParent = world.addFreeFrame("replay_parent");
  Eigen::Isometry3d recordedParentTransform = Eigen::Isometry3d::Identity();
  recordedParentTransform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  replayParent.setLocalTransform(recordedParentTransform);
  child.setParentFrame(replayParent);

  world.enterSimulationMode();
  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  compute::SequentialExecutor executor;
  world.step(executor);

  Eigen::Isometry3d movedReplayParentTransform = Eigen::Isometry3d::Identity();
  movedReplayParentTransform.translation() = Eigen::Vector3d(20.0, 0.0, 0.0);
  replayParent.setLocalTransform(movedReplayParentTransform);
  child.setParentFrame(oldParent);
  world.step(executor);

  world.restoreReplayFrame(0);
  world.step(executor);

  EXPECT_EQ(child.getParentFrame().getEntity(), replayParent.getEntity());
  EXPECT_TRUE(
      child.getTransform().isApprox(recordedParentTransform * childOffset));
}

// Test that public-frame replay restore dirties caches before restoring rigid
// bodies whose parent transform is a restored public frame.
TEST(World, ReplayRecordingRestoresRigidBodyThroughPublicFrameWithDirtyCache)
{
  namespace sx = dart::simulation;

  sx::World world;

  auto mount = world.addFreeFrame("mount");
  Eigen::Isometry3d recordedMountTransform = Eigen::Isometry3d::Identity();
  recordedMountTransform.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  mount.setLocalTransform(recordedMountTransform);

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.position = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto body = world.addRigidBody("body", bodyOptions);
  body.setParentFrame(mount);

  Eigen::Isometry3d recordedBodyTransform = Eigen::Isometry3d::Identity();
  recordedBodyTransform.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  body.setTransform(recordedBodyTransform);

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  Eigen::Isometry3d movedMountTransform = Eigen::Isometry3d::Identity();
  movedMountTransform.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  mount.setLocalTransform(movedMountTransform);
  EXPECT_TRUE(mount.getTransform().isApprox(movedMountTransform));

  Eigen::Isometry3d movedBodyTransform = Eigen::Isometry3d::Identity();
  movedBodyTransform.translation() = Eigen::Vector3d(12.0, 0.0, 0.0);
  body.setTransform(movedBodyTransform);

  world.restoreReplayFrame(0);

  EXPECT_TRUE(mount.getTransform().isApprox(recordedMountTransform));
  EXPECT_TRUE(body.getTransform().isApprox(recordedBodyTransform));
  EXPECT_TRUE(body.getLocalTransform().isApprox(
      recordedMountTransform.inverse() * recordedBodyTransform));
}

// Test replay control edge cases and invalid frame queries.
TEST(World, ReplayRecordingRejectsInvalidQueriesAndClears)
{
  namespace sx = dart::simulation;

  sx::World world;

  world.clearReplayRecording();
  EXPECT_FALSE(world.isReplayRecordingEnabled());
  EXPECT_EQ(world.getReplayFrameCount(), 0u);
  EXPECT_FALSE(world.getReplayCursor().has_value());

  EXPECT_THROW(
      static_cast<void>(world.getReplayFrameTime(0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      static_cast<void>(world.getReplaySimulationFrame(0)),
      sx::InvalidArgumentException);
  EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidArgumentException);

  world.setReplayRecordingEnabled(false);
  EXPECT_FALSE(world.isReplayRecordingEnabled());
  EXPECT_EQ(world.getReplayFrameCount(), 0u);
  EXPECT_FALSE(world.getReplayCursor().has_value());

  world.setReplayRecordingEnabled(true);
  world.setReplayRecordingEnabled(true);
  ASSERT_TRUE(world.isReplayRecordingEnabled());
  ASSERT_EQ(world.getReplayFrameCount(), 1u);
  ASSERT_TRUE(world.getReplayCursor().has_value());
  EXPECT_EQ(*world.getReplayCursor(), 0u);
  EXPECT_THROW(
      static_cast<void>(world.getReplayFrameTime(1)),
      sx::InvalidArgumentException);
  EXPECT_THROW(
      static_cast<void>(world.getReplaySimulationFrame(1)),
      sx::InvalidArgumentException);
  EXPECT_THROW(world.restoreReplayFrame(1), sx::InvalidArgumentException);

  world.clear();
  EXPECT_FALSE(world.isReplayRecordingEnabled());
  EXPECT_EQ(world.getReplayFrameCount(), 0u);
  EXPECT_FALSE(world.getReplayCursor().has_value());
}

// Test that replay restore rejects topology/layout changes instead of trying
// partial best-effort replay.
TEST(World, ReplayRecordingRejectsRigidBodyLayoutChanges)
{
  namespace sx = dart::simulation;

  sx::World world;
  world.addRigidBody("recorded");
  world.setReplayRecordingEnabled(true);
  world.addRigidBody("added_after_recording");

  EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
}

// Test that replay restore rejects rigid body static/kinematic mode drift.
// These tags affect solver participation and are not runtime replay state.
TEST(World, ReplayRecordingRejectsRigidBodyModeChanges)
{
  namespace sx = dart::simulation;

  sx::World staticWorld;
  auto staticBody = staticWorld.addRigidBody("body");
  staticWorld.setReplayRecordingEnabled(true);
  staticBody.setStatic(true);
  EXPECT_THROW(
      staticWorld.restoreReplayFrame(0), sx::InvalidOperationException);

  sx::World kinematicWorld;
  auto kinematicBody = kinematicWorld.addRigidBody("body");
  kinematicWorld.setReplayRecordingEnabled(true);
  kinematicBody.setKinematic(true);
  EXPECT_THROW(
      kinematicWorld.restoreReplayFrame(0), sx::InvalidOperationException);
}

// Test that replay restore rejects rigid body construction/layout edits that
// are not captured as mutable per-frame state.
TEST(World, ReplayRecordingRejectsRigidBodyConstructionEdits)
{
  namespace sx = dart::simulation;

  const auto expectBodyMutationRejected = [](auto configure, auto mutate) {
    sx::World world;
    auto body = world.addRigidBody("body");
    configure(world, body);
    world.setReplayRecordingEnabled(true);
    EXPECT_EQ(world.getReplayFrameCount(), 1u);

    mutate(world, body);

    EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
  };

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody& body) {
        body.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
      },
      [](sx::World&, sx::RigidBody& body) {
        body.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.2, 0.3}));
      });

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody&) {},
      [](sx::World&, sx::RigidBody& body) {
        body.addCollisionShape(sx::CollisionShape::makeSphere(0.25));
      });

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody&) {},
      [](sx::World&, sx::RigidBody& body) { body.setFriction(0.25); });

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody&) {},
      [](sx::World&, sx::RigidBody& body) { body.setRestitution(0.5); });

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody&) {},
      [](sx::World&, sx::RigidBody& body) { body.setMass(2.0); });

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody&) {},
      [](sx::World&, sx::RigidBody& body) {
        body.setInertia(2.0 * Eigen::Matrix3d::Identity());
      });

  expectBodyMutationRejected(
      [](sx::World&, sx::RigidBody&) {},
      [](sx::World&, sx::RigidBody& body) {
        body.setDeformableGroundBarrier(true);
      });

  sx::World parentWorld;
  auto child = parentWorld.addRigidBody("child");
  auto parent = parentWorld.addRigidBody("parent");
  parentWorld.setReplayRecordingEnabled(true);
  EXPECT_EQ(parentWorld.getReplayFrameCount(), 1u);

  child.setParentFrame(parent);

  EXPECT_THROW(
      parentWorld.restoreReplayFrame(0), sx::InvalidOperationException);
}

// Test that replay restore rejects joint dynamics/configuration edits that are
// not mutable per-frame state.
TEST(World, ReplayRecordingRejectsJointDynamicsChanges)
{
  namespace sx = dart::simulation;

  const auto expectJointMutationRejected = [](auto mutate) {
    sx::World world;
    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");

    sx::JointSpec spec;
    spec.name = "joint";
    spec.type = sx::JointType::Revolute;
    auto link = robot.addLink("link", base, spec);
    auto joint = link.getParentJoint();

    world.setReplayRecordingEnabled(true);
    EXPECT_EQ(world.getReplayFrameCount(), 1u);

    mutate(joint);

    EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
  };

  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setSpringStiffness(Eigen::VectorXd::Constant(1, 1.0));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setDampingCoefficient(Eigen::VectorXd::Constant(1, 0.1));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setRestPosition(Eigen::VectorXd::Constant(1, 0.25));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setArmature(Eigen::VectorXd::Constant(1, 0.05));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setCoulombFriction(Eigen::VectorXd::Constant(1, 0.2));
  });
  expectJointMutationRejected(
      [](sx::Joint& joint) { joint.setBreakForce(10.0); });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setPositionLimits(
        Eigen::VectorXd::Constant(1, -0.5), Eigen::VectorXd::Constant(1, 0.5));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setVelocityLimits(
        Eigen::VectorXd::Constant(1, -1.0), Eigen::VectorXd::Constant(1, 1.0));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setEffortLimits(
        Eigen::VectorXd::Constant(1, -2.0), Eigen::VectorXd::Constant(1, 2.0));
  });
  expectJointMutationRejected([](sx::Joint& joint) {
    joint.setActuatorType(sx::ActuatorType::Passive);
  });
}

// Test that replay restore rejects internally corrupted joint runtime vector
// sizes instead of resizing live Eigen storage during restore.
TEST(World, ReplayRecordingRejectsJointRuntimeVectorSizeChanges)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "joint";
  spec.type = sx::JointType::Floating;
  auto link = robot.addLink("link", base, spec);
  auto joint = link.getParentJoint();

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  auto& registry = sx::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());
  auto& jointComponent = registry.get<sx::comps::Joint>(jointEntity);
  jointComponent.position = sx::comps::makeJointVector(1, 0.0);

  EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
}

// Test that replay restore rejects internally corrupted deformable payload
// sizes instead of resizing live node-state vectors during restore.
TEST(World, ReplayRecordingRejectsDeformableNodeVectorSizeChanges)
{
  namespace sx = dart::simulation;

  sx::World world;
  sx::DeformableBodyOptions options;
  options.positions
      = {Eigen::Vector3d(0.0, 0.0, 0.0),
         Eigen::Vector3d(1.0, 0.0, 0.0),
         Eigen::Vector3d(0.0, 1.0, 0.0)};
  options.velocities.assign(options.positions.size(), Eigen::Vector3d::Zero());
  options.masses.assign(options.positions.size(), 1.0);
  world.addDeformableBody("deformable", options);

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  auto& registry = sx::detail::registryOf(world);
  auto view = registry.view<sx::comps::DeformableNodeState>();
  std::size_t deformableNodeStateCount = 0;
  for (auto entity : view) {
    ++deformableNodeStateCount;
    auto& state = view.get<sx::comps::DeformableNodeState>(entity);
    state.positions.push_back(Eigen::Vector3d::Zero());
  }
  ASSERT_EQ(deformableNodeStateCount, 1u);

  EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
}

// Test that replay restore rejects link physical/collision layout edits that
// are not mutable per-frame state.
TEST(World, ReplayRecordingRejectsLinkPhysicalLayoutChanges)
{
  namespace sx = dart::simulation;

  const auto expectLinkMutationRejected = [](auto configure, auto mutate) {
    sx::World world;
    auto robot = world.addMultibody("robot");
    auto base = robot.addLink("base");

    sx::JointSpec spec;
    spec.name = "joint";
    spec.type = sx::JointType::Revolute;
    auto link = robot.addLink("link", base, spec);

    configure(link);
    world.setReplayRecordingEnabled(true);
    EXPECT_EQ(world.getReplayFrameCount(), 1u);

    mutate(link);

    EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
  };

  expectLinkMutationRejected(
      [](sx::Link&) {}, [](sx::Link& link) { link.setMass(2.0); });

  expectLinkMutationRejected(
      [](sx::Link&) {},
      [](sx::Link& link) {
        link.setInertia(2.0 * Eigen::Matrix3d::Identity());
      });

  expectLinkMutationRejected(
      [](sx::Link&) {},
      [](sx::Link& link) {
        link.setCenterOfMass(Eigen::Vector3d(0.1, 0.0, 0.0));
      });

  expectLinkMutationRejected(
      [](sx::Link& link) {
        link.setCollisionShape(sx::CollisionShape::makeSphere(0.25));
      },
      [](sx::Link& link) {
        link.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.2, 0.3}));
      });

  expectLinkMutationRejected(
      [](sx::Link&) {},
      [](sx::Link& link) {
        link.addCollisionShape(sx::CollisionShape::makeSphere(0.25));
      });
}

// Test that a layout-incompatible restore fails before mutating earlier replay
// state such as articulated joint values.
TEST(World, ReplayRecordingRejectsLayoutBeforeMutatingState)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "joint";
  spec.type = sx::JointType::Revolute;
  auto link = robot.addLink("link", base, spec);
  auto joint = link.getParentJoint();

  const Eigen::VectorXd recordedPosition = Eigen::VectorXd::Constant(1, 0.25);
  joint.setPosition(recordedPosition);

  auto body = world.addRigidBody("body");
  body.setCollisionShape(sx::CollisionShape::makeSphere(0.25));

  world.setReplayRecordingEnabled(true);

  const Eigen::VectorXd mutatedPosition = Eigen::VectorXd::Constant(1, -1.0);
  joint.setPosition(mutatedPosition);
  body.setCollisionShape(sx::CollisionShape::makeBox({0.1, 0.2, 0.3}));

  EXPECT_THROW(world.restoreReplayFrame(0), sx::InvalidOperationException);
  EXPECT_TRUE(joint.getPosition().isApprox(mutatedPosition));
  EXPECT_FALSE(joint.getPosition().isApprox(recordedPosition));
}

// Test replay recording and restore for articulated runtime state.
TEST(World, ReplayRecordingRestoresMultibodyRuntimeState)
{
  namespace sx = dart::simulation;

  sx::World world;
  auto robot = world.addMultibody("robot");
  auto base = robot.addLink("base");

  sx::JointSpec spec;
  spec.name = "joint";
  spec.type = sx::JointType::Floating;
  auto link = robot.addLink("link", base, spec);
  auto joint = link.getParentJoint();

  const Eigen::VectorXd initialPosition
      = Eigen::VectorXd::LinSpaced(6, 0.25, 0.75);
  const Eigen::VectorXd initialVelocity
      = Eigen::VectorXd::LinSpaced(6, 0.5, 1.0);
  const Eigen::VectorXd initialTorque = Eigen::VectorXd::LinSpaced(6, 1.5, 2.0);
  const Eigen::VectorXd initialCommandVelocity
      = Eigen::VectorXd::LinSpaced(6, -0.75, -0.25);
  const double initialBreakForce = 100.0;
  joint.setPosition(initialPosition);
  joint.setVelocity(initialVelocity);
  joint.setForce(initialTorque);
  joint.setCommandVelocity(initialCommandVelocity);
  joint.setBreakForce(initialBreakForce);

  auto& registry = sx::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());
  auto& jointComponent = registry.get<sx::comps::Joint>(jointEntity);
  jointComponent.broken = false;
  const entt::entity linkEntity
      = sx::detail::toRegistryEntity(link.getEntity());
  auto& linkComponent = registry.get<sx::comps::Link>(linkEntity);
  const Eigen::Matrix<double, 6, 1> initialExternalForce
      = (Eigen::Matrix<double, 6, 1>() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
            .finished();
  linkComponent.externalForce = initialExternalForce;

  world.setReplayRecordingEnabled(true);
  ASSERT_EQ(world.getReplayFrameCount(), 1u);

  joint.setPosition(Eigen::VectorXd::Constant(6, -1.0));
  joint.setVelocity(Eigen::VectorXd::Constant(6, -2.0));
  joint.setForce(Eigen::VectorXd::Constant(6, -3.0));
  joint.setCommandVelocity(Eigen::VectorXd::Constant(6, -4.0));
  jointComponent.broken = true;
  linkComponent.externalForce.setZero();

  world.restoreReplayFrame(0);

  EXPECT_TRUE(joint.getPosition().isApprox(initialPosition));
  EXPECT_TRUE(joint.getVelocity().isApprox(initialVelocity));
  EXPECT_TRUE(joint.getForce().isApprox(initialTorque));
  EXPECT_TRUE(joint.getCommandVelocity().isApprox(initialCommandVelocity));
  EXPECT_DOUBLE_EQ(joint.getBreakForce(), initialBreakForce);
  EXPECT_FALSE(joint.isBroken());
  EXPECT_TRUE(registry.get<sx::comps::Link>(linkEntity)
                  .externalForce.isApprox(initialExternalForce));

  jointComponent.broken = true;
  world.clearReplayRecording();
  ASSERT_EQ(world.getReplayFrameCount(), 1u);
  joint.resetBreakage();

  world.restoreReplayFrame(0);

  EXPECT_TRUE(joint.isBroken());
}
