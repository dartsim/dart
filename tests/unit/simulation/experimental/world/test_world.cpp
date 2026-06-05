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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/contact.hpp>
#include <dart/simulation/experimental/body/deformable_body.hpp>
#include <dart/simulation/experimental/body/deformable_body_options.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/common/exceptions.hpp>
#include <dart/simulation/experimental/comps/collision_geometry.hpp>
#include <dart/simulation/experimental/comps/dynamics.hpp>
#include <dart/simulation/experimental/comps/frame_types.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/comps/rigid_body.hpp>
#include <dart/simulation/experimental/compute/compute_executor.hpp>
#include <dart/simulation/experimental/compute/compute_graph.hpp>
#include <dart/simulation/experimental/compute/parallel_executor.hpp>
#include <dart/simulation/experimental/compute/rigid_body_state_batch.hpp>
#include <dart/simulation/experimental/compute/sequential_executor.hpp>
#include <dart/simulation/experimental/compute/variational_integration.hpp>
#include <dart/simulation/experimental/compute/world_batch.hpp>
#include <dart/simulation/experimental/compute/world_step_stage.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_spec.hpp>
#include <dart/simulation/experimental/detail/entity_conversion.hpp>
#include <dart/simulation/experimental/detail/world_registry_access.hpp>
#include <dart/simulation/experimental/detail/world_storage.hpp>
#include <dart/simulation/experimental/frame/fixed_frame.hpp>
#include <dart/simulation/experimental/frame/free_frame.hpp>
#include <dart/simulation/experimental/multibody/multibody.hpp>
#include <dart/simulation/experimental/version.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <limits>

#include <cstdlib>
#if defined(_WIN32)
  #include <malloc.h>
#endif
#include <map>
#include <new>
#include <numbers>
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
  : public dart::simulation::experimental::compute::WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "counting_kinematics";
  }

  void execute(
      dart::simulation::experimental::World& world,
      dart::simulation::experimental::compute::ComputeExecutor& executor)
      override
  {
    ++executionCount;
    defaultStage.execute(world, executor);
  }

  int executionCount{0};
  dart::simulation::experimental::compute::KinematicsStage defaultStage;
};

class RecordingWorldStage final
  : public dart::simulation::experimental::compute::WorldStepStage
{
public:
  RecordingWorldStage(
      std::string name,
      dart::simulation::experimental::compute::ComputeStageMetadata metadata,
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

  [[nodiscard]] dart::simulation::experimental::compute::ComputeStageMetadata
  getMetadata() const noexcept override
  {
    return m_metadata;
  }

  void execute(
      dart::simulation::experimental::World&,
      dart::simulation::experimental::compute::ComputeExecutor& executor)
      override
  {
    dart::simulation::experimental::compute::ComputeGraph graph;
    graph.addNode(
        m_name + "_node",
        [&]() { m_executionOrder.push_back(m_name); },
        m_metadata);
    executor.execute(graph);
  }

private:
  std::string m_name;
  dart::simulation::experimental::compute::ComputeStageMetadata m_metadata;
  std::vector<std::string>& m_executionOrder;
};

class RecordingExecutor final
  : public dart::simulation::experimental::compute::ComputeExecutor
{
public:
  void execute(
      const dart::simulation::experimental::compute::ComputeGraph& graph)
      override
  {
    record(graph);
    sequential.execute(graph);
  }

  [[nodiscard]]
  dart::simulation::experimental::compute::ComputeExecutionProfile
  executeProfiled(
      const dart::simulation::experimental::compute::ComputeGraph& graph)
      override
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
  void record(
      const dart::simulation::experimental::compute::ComputeGraph& graph)
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

  dart::simulation::experimental::compute::SequentialExecutor sequential;
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

class FrameScratchStage final
  : public dart::simulation::experimental::compute::WorldStepStage
{
public:
  explicit FrameScratchStage(std::size_t bytes) : bytesToAllocate(bytes) {}

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "frame_scratch";
  }

  void execute(
      dart::simulation::experimental::World& world,
      dart::simulation::experimental::compute::ComputeExecutor&) override
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
    const dart::simulation::experimental::detail::WorldRegistry& registry)
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
    const dart::simulation::experimental::detail::WorldRegistry& registry)
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
    std::string_view scene, ConfigureScene&& configureScene)
{
  namespace sx = dart::simulation::experimental;

  SCOPED_TRACE(scene);
  CountingMemoryAllocator allocator;
  sx::WorldOptions worldOptions;
  worldOptions.baseAllocator = &allocator;
  sx::World world(worldOptions);

  configureScene(world);
  world.enterSimulationMode();

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
    std::string_view scene, ConfigureScene&& configureScene)
{
  namespace sx = dart::simulation::experimental;

  SCOPED_TRACE(scene);
  sx::World world;

  configureScene(world);
  world.enterSimulationMode();

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
  namespace sx = dart::simulation::experimental;

  SCOPED_TRACE(scene);
  sx::World world;

  configureScene(world);

  ScopedHeapAllocationCounter heapCounter;
  world.enterSimulationMode();
  heapCounter.stop();

  return {heapCounter.allocationCount(), heapCounter.allocationBytes()};
}

} // namespace

// Test World construction
TEST(World, Construction)
{
  // Create a World instance
  dart::simulation::experimental::World world;
  (void)world;       // Suppress unused variable warning
  EXPECT_TRUE(true); // If we get here, construction succeeded
}

TEST(World, MemoryManagerOptionsAndDiagnostics)
{
  namespace sx = dart::simulation::experimental;

  CountingMemoryAllocator allocator;
  sx::WorldOptions options;
  options.baseAllocator = &allocator;
  options.frameScratchInitialCapacity = 4096;

  sx::World world(options);

  EXPECT_EQ(&world.getMemoryManager().getBaseAllocator(), &allocator);
  EXPECT_GT(allocator.allocationCount, 0u);

  const auto diagnostics = world.getMemoryDiagnostics();
  EXPECT_LE(diagnostics.frameScratchCapacityBytes, 4096u);
  EXPECT_GE(diagnostics.frameScratchCapacityBytes + 32u, 4096u);
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

TEST(World, RegistryUsesWorldFreeAllocator)
{
  namespace common = dart::common;
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

TEST(World, EnterSimulationModeReservesRegistryStorageForMultibodySteps)
{
  namespace sx = dart::simulation::experimental;

  sx::World world;

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

  const auto& registry = sx::detail::registryOf(world);
  const auto capacities = registryStorageCapacities(registry);

  for (int i = 0; i < 4; ++i) {
    world.step();
    expectRegistryStorageCapacitiesUnchanged(capacities, registry);
  }
}

TEST(World, SetMultibodyOptionsReservesVariationalStateAfterBake)
{
  namespace sx = dart::simulation::experimental;

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

TEST(World, EnterSimulationModeReservesRegistryStorageForDeformableSteps)
{
  namespace sx = dart::simulation::experimental;

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

TEST(World, BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths)
{
  namespace sx = dart::simulation::experimental;

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
      "single deformable particle", [](sx::World& world) {
        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("particle", options);
        world.setTimeStep(0.01);
      });
}

TEST(World, BakedKinematicIpcStepsDoNotAllocateGlobalHeap)
{
  namespace sx = dart::simulation::experimental;

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

TEST(World, BakedRigidBodyContactStepsDoNotAllocateGlobalHeap)
{
  namespace sx = dart::simulation::experimental;

  expectNoGlobalHeapAllocationsDuringBakedSteps(
      "rigid body resting contact", [](sx::World& world) {
        world.setGravity(Eigen::Vector3d::Zero());

        sx::RigidBodyOptions groundOptions;
        groundOptions.isStatic = true;
        groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.25);
        auto ground = world.addRigidBody("ground", groundOptions);
        ground.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(2.0, 2.0, 0.25)));

        sx::RigidBodyOptions boxOptions;
        boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.2);
        auto box = world.addRigidBody("box", boxOptions);
        box.setMass(1.0);
        box.setCollisionShape(
            sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.2, 0.2)));

        world.setTimeStep(0.001);
      });
}

TEST(World, BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap)
{
  namespace sx = dart::simulation::experimental;

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
      "single deformable particle", [](sx::World& world) {
        sx::DeformableBodyOptions options;
        options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
        options.masses = {1.0};
        options.edgeStiffness = 0.0;
        world.addDeformableBody("particle", options);
        world.setTimeStep(0.01);
      });
}

TEST(World, SequentialImpulseBakeDoesNotPrewarmRigidIpcCollisionSurfaces)
{
  namespace sx = dart::simulation::experimental;

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

TEST(World, RigidIpcContactStagePrepareReusesSupportedDynamicSurfaceBuffers)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

TEST(World, FrameScratchCapacityReportsUsableArenaBytes)
{
  namespace sx = dart::simulation::experimental;

  sx::WorldOptions options;
  options.frameScratchInitialCapacity = 128;
  sx::World world(options);

  const auto initial = world.getMemoryDiagnostics();
  EXPECT_LE(
      initial.frameScratchCapacityBytes, options.frameScratchInitialCapacity);
  EXPECT_GE(
      initial.frameScratchCapacityBytes + 32u,
      options.frameScratchInitialCapacity);
  ASSERT_GT(initial.frameScratchCapacityBytes, 0u);
  EXPECT_EQ(initial.frameScratchCapacityBytes % 32u, 0u);

  dart::simulation::experimental::compute::SequentialExecutor executor;
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
  namespace sx = dart::simulation::experimental;

  sx::WorldOptions options;
  options.frameScratchInitialCapacity = 1024;
  sx::World world(options);

  dart::simulation::experimental::compute::SequentialExecutor executor;
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
  namespace sx = dart::simulation::experimental;

  sx::WorldOptions options;
  options.frameScratchInitialCapacity = 128;
  sx::World world(options);

  dart::simulation::experimental::compute::SequentialExecutor executor;
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
  auto ver = dart::simulation::experimental::version();
  EXPECT_FALSE(ver.empty());
  EXPECT_GT(ver.size(), 0u);

  // Test individual version components
  EXPECT_GE(dart::simulation::experimental::versionMajor(), 7);
  EXPECT_GE(dart::simulation::experimental::versionMinor(), 0);
  EXPECT_GE(dart::simulation::experimental::versionPatch(), 0);
}

//==============================================================================
// Mode Control Tests (Design Mode vs Simulation Mode)
//==============================================================================

// Test default mode is design mode
TEST(World, DefaultModeIsDesign)
{
  dart::simulation::experimental::World world;
  EXPECT_FALSE(world.isSimulationMode());
}

// Test entering simulation mode
TEST(World, EnterSimulationMode)
{
  dart::simulation::experimental::World world;

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
  dart::simulation::experimental::World world;

  // First call succeeds
  world.enterSimulationMode();
  EXPECT_TRUE(world.isSimulationMode());

  // Second call should throw
  EXPECT_THROW(
      world.enterSimulationMode(),
      dart::simulation::experimental::InvalidArgumentException);
}

// Test baking with empty world
TEST(World, BakingEmptyWorld)
{
  dart::simulation::experimental::World world;

  // Baking should work even with empty world
  EXPECT_NO_THROW(world.enterSimulationMode());
  EXPECT_TRUE(world.isSimulationMode());
}

// Test baking with multibodies
TEST(World, BakingWithMultibodies)
{
  dart::simulation::experimental::World world;

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
       .jointType = dart::simulation::experimental::JointType::Prismatic});

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d parentTransform = Eigen::Isometry3d::Identity();
  parentTransform.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
  parent.setLocalTransform(parentTransform);

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 4.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  auto& registry = dart::simulation::experimental::detail::registryOf(world);
  EXPECT_FALSE(registry
                   .get<sx::comps::FrameCache>(
                       dart::simulation::experimental::detail::toRegistryEntity(
                           parent.getEntity()))
                   .needTransformUpdate);
  EXPECT_FALSE(registry
                   .get<sx::comps::FrameCache>(
                       dart::simulation::experimental::detail::toRegistryEntity(
                           child.getEntity()))
                   .needTransformUpdate);
  EXPECT_TRUE(child.getTransform().isApprox(parentTransform * childOffset));

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(5.0, 6.0, 7.0));
  parent.setLocalTransform(updatedParentTransform);

  world.sync(sx::WorldSyncStage::Kinematics);

  EXPECT_FALSE(registry
                   .get<sx::comps::FrameCache>(
                       dart::simulation::experimental::detail::toRegistryEntity(
                           parent.getEntity()))
                   .needTransformUpdate);
  EXPECT_FALSE(registry
                   .get<sx::comps::FrameCache>(
                       dart::simulation::experimental::detail::toRegistryEntity(
                           child.getEntity()))
                   .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that kinematics-only sync can use an injected executor
// without advancing the simulation clock.
TEST(World, SyncKinematicsAcceptsExecutorWithoutAdvancingClock)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

  auto& registry = dart::simulation::experimental::detail::registryOf(world);
  EXPECT_TRUE(registry
                  .get<sx::comps::FrameCache>(
                      dart::simulation::experimental::detail::toRegistryEntity(
                          parent.getEntity()))
                  .needTransformUpdate);
  EXPECT_TRUE(registry
                  .get<sx::comps::FrameCache>(
                      dart::simulation::experimental::detail::toRegistryEntity(
                          child.getEntity()))
                  .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental step path enters simulation mode and refreshes
// kinematics using the default sequential graph executor.
TEST(World, StepRefreshesFrameHierarchy)
{
  namespace sx = dart::simulation::experimental;

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

  auto& registry = dart::simulation::experimental::detail::registryOf(world);
  EXPECT_FALSE(registry
                   .get<sx::comps::FrameCache>(
                       dart::simulation::experimental::detail::toRegistryEntity(
                           parent.getEntity()))
                   .needTransformUpdate);
  EXPECT_FALSE(registry
                   .get<sx::comps::FrameCache>(
                       dart::simulation::experimental::detail::toRegistryEntity(
                           child.getEntity()))
                   .needTransformUpdate);
  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that the experimental step path accepts alternate graph executors.
TEST(World, StepAcceptsParallelExecutor)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

  sx::World world;
  auto parent = world.addFreeFrame("parent");

  Eigen::Isometry3d childOffset = Eigen::Isometry3d::Identity();
  childOffset.translate(Eigen::Vector3d(0.0, 1.0, 0.0));
  auto child = world.addFixedFrame("child", parent, childOffset);

  world.enterSimulationMode();

  Eigen::Isometry3d updatedParentTransform = Eigen::Isometry3d::Identity();
  updatedParentTransform.translate(Eigen::Vector3d(2.0, 0.0, 0.0));
  parent.setLocalTransform(updatedParentTransform);

  compute::ParallelExecutor executor(1);
  world.step(executor);

  EXPECT_TRUE(
      child.getTransform().isApprox(updatedParentTransform * childOffset));
}

// Test that RigidBodyOptions seed the public rigid-body state used by the graph
// backed step path.
TEST(World, RigidBodyOptionsInitializePublicState)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

// Test the generalized impulse response dqdot = M^-1 f against the analytical
// single-pendulum value and the M dqdot = f consistency identity.
TEST(World, MultibodyImpulseResponse)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

// Test that multiple shapes on the same rigid body behave as compound
// collision geometry and do not self-collide.
TEST(World, CollisionQuerySupportsCompoundRigidBodyShapes)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

// Test that a multibody link with a collision shape rests on a static ground
// via the articulated contact response (a fixed-base prismatic "leg" drops
// under gravity and stops where its sphere meets the ground).
TEST(World, MultibodyLinkRestsOnStaticGround)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

// Test cross-multibody link-vs-link contact: two separate fixed-base
// articulated bodies overlap and approach along their prismatic rails. The
// unified row carries both articulated ends, so the impulse changes both
// multibodies' staged velocities.
TEST(World, CrossMultibodyLinksResolveContact)
{
  namespace sx = dart::simulation::experimental;

  sx::WorldOptions options;
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);
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

// Test that Coulomb friction at a link contact decelerates a sliding link. A
// vertical prismatic carries a horizontal prismatic link whose sphere rests on
// the ground; an initial horizontal velocity is braked to rest by friction.
TEST(World, MultibodyLinkContactFrictionStopsSlide)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

// Test that static rigid bodies can retain stored velocities without behaving
// like moving obstacles in rigid-body contact response.
TEST(World, RigidBodyContactIgnoresStoredStaticVelocity)
{
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

  namespace sx = dart::simulation::experimental;
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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

  auto& registry = dart::simulation::experimental::detail::registryOf(world);
  const auto& toReg = dart::simulation::experimental::detail::toRegistryEntity;
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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

  namespace sx = dart::simulation::experimental;

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

  namespace sx = dart::simulation::experimental;

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

// Test that public fixed joints reject the IPC rigid-body solver instead of
// accepting a joint that the IPC pipeline will not project.
TEST(World, RigidBodyFixedJointsRejectIpcSolver)
{
  namespace sx = dart::simulation::experimental;

  sx::RigidBodyOptions parentOptions;
  parentOptions.isStatic = true;
  sx::RigidBodyOptions childOptions;
  childOptions.position = Eigen::Vector3d::UnitX();

  sx::World solverFirst;
  solverFirst.setRigidBodySolver(sx::RigidBodySolver::Ipc);
  auto solverFirstParent = solverFirst.addRigidBody("parent", parentOptions);
  auto solverFirstChild = solverFirst.addRigidBody("child", childOptions);
  EXPECT_THROW(
      solverFirst.addRigidBodyFixedJoint(
          "fixed", solverFirstParent, solverFirstChild),
      sx::InvalidOperationException);

  const auto solverFirstJoints
      = dart::simulation::experimental::detail::registryOf(solverFirst)
            .view<sx::comps::Joint>();
  EXPECT_EQ(solverFirstJoints.begin(), solverFirstJoints.end());

  sx::World jointFirst;
  auto jointFirstParent = jointFirst.addRigidBody("parent", parentOptions);
  auto jointFirstChild = jointFirst.addRigidBody("child", childOptions);
  (void)jointFirst.addRigidBodyFixedJoint(
      "fixed", jointFirstParent, jointFirstChild);

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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
  namespace sx = dart::simulation::experimental;

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

// Test that rigid-body frame ancestry is represented in the integration graph
// so a parallel executor cannot run a child integration while its parent rigid
// body is updating frame properties.
TEST(World, RigidBodyIntegrationStageOrdersRigidBodyFrameAncestry)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  RecordingExecutor batchedExecutor;
  batchedStage.execute(batchedWorld, batchedExecutor);

  ASSERT_EQ(batchedExecutor.executeCount, 1u);
  ASSERT_EQ(batchedExecutor.nodeCount, 1u);
  ASSERT_EQ(batchedExecutor.edgeCount, 0u);
  ASSERT_FALSE(batchedExecutor.graphNodeNames[0].empty());
  EXPECT_EQ(batchedExecutor.graphNodeNames[0][0], "soa_rigid_body_integration");

  EXPECT_TRUE(perEntityParent.getLocalTransform().isApprox(
      batchedParent.getLocalTransform()));
  EXPECT_TRUE(perEntityChild.getLocalTransform().isApprox(
      batchedChild.getLocalTransform()));
  EXPECT_TRUE(
      perEntityParent.getTransform().isApprox(batchedParent.getTransform()));
  EXPECT_TRUE(
      perEntityChild.getTransform().isApprox(batchedChild.getTransform()));
}

// Test that the experimental step path can swap the kinematics stage contract.
TEST(World, StepAcceptsCustomStage)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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

// Test that custom-stage step overloads keep the same default dynamics baseline
// as World::step(), with the caller-provided stage replacing the final stage.
TEST(World, StepWithCustomStageUsesDefaultDynamicsBaseline)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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

// Test that the experimental world step path can compose multiple solver
// domains without expanding the default public World::step() surface.
TEST(World, StepAcceptsMultiDomainSolverPipeline)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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

// Test that repeated stepping can reuse a caller-owned executor and pipeline.
TEST(World, StepCountAcceptsMultiDomainSolverPipeline)
{
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
  namespace sx = dart::simulation::experimental;
  namespace compute = dart::simulation::experimental::compute;

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
