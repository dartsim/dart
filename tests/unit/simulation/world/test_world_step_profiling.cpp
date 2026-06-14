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

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/compute/parallel_executor.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

#include <atomic>

#include <cstdlib>
#if defined(_WIN32)
  #include <malloc.h>
#endif
#include <new>
#include <string>
#include <string_view>

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

class NoOpStage final : public dart::simulation::compute::WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "p";
  }

  void execute(
      dart::simulation::World&,
      dart::simulation::compute::ComputeExecutor&) override
  {
    ++executionCount;
  }

  std::size_t executionCount{0};
};

} // namespace

namespace sx = dart::simulation;

TEST(WorldStepProfileIntegration, DisabledByDefaultLeavesProfileEmpty)
{
  sx::World world;
  EXPECT_FALSE(world.isStepProfilingEnabled());

  world.step();

  EXPECT_TRUE(world.getLastStepProfile().isEmpty());
}

#if DART_BUILD_PROFILE

TEST(WorldStepProfileIntegration, EnabledStepRecordsPipelineStages)
{
  sx::World world;
  world.setStepProfilingEnabled(true);
  EXPECT_TRUE(world.isStepProfilingEnabled());

  world.step();

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  EXPECT_EQ(profile.stepCount, 1u);
  EXPECT_EQ(profile.stages.size(), 1u);

  // Empty worlds still refresh kinematics, but inactive solver domains should
  // not appear as placeholder stages.
  EXPECT_EQ(profile.getStage("rigid_body_velocity"), nullptr);
  EXPECT_EQ(profile.getStage("deformable_dynamics"), nullptr);
  ASSERT_NE(profile.getStage("kinematics"), nullptr);

  // Every recorded stage carries a name; wall time is non-negative.
  for (const auto& stage : profile.stages) {
    EXPECT_FALSE(stage.name.empty());
  }
  EXPECT_GE(profile.wallTime.count(), 0);
  EXPECT_GE(profile.wallTime, profile.totalStageTime());

  // The text summary is the AI-/human-facing surface; it names the stages.
  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("kinematics"), std::string::npos);
  EXPECT_NE(summary.find("World Step Profile"), std::string::npos);
}

TEST(WorldStepProfileIntegration, StaticOnlyRigidWorldSkipsRigidDynamicsStages)
{
  sx::World world;

  sx::RigidBodyOptions options;
  options.isStatic = true;
  options.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", options);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(5.0, 5.0, 0.5)));
  const Eigen::Vector3d initialTranslation = ground.getTranslation();

  world.setStepProfilingEnabled(true);
  world.step();

  EXPECT_EQ(world.getFrame(), 1u);
  EXPECT_DOUBLE_EQ(world.getTime(), world.getTimeStep());
  EXPECT_TRUE(ground.getTranslation().isApprox(initialTranslation));

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  EXPECT_EQ(profile.getStage("rigid_body_velocity"), nullptr);
  EXPECT_EQ(profile.getStage("rigid_body_contact"), nullptr);
  EXPECT_EQ(profile.getStage("rigid_body_position"), nullptr);
  ASSERT_NE(profile.getStage("kinematics"), nullptr);
}

TEST(WorldStepProfileIntegration, EnabledDeformableStepRecordsDomainStage)
{
  sx::World world;
  sx::DeformableBodyOptions options;
  options.positions = {Eigen::Vector3d(0.0, 0.0, 1.0)};
  options.masses = {1.0};
  options.edgeStiffness = 0.0;
  world.addDeformableBody("particle", options);
  world.setStepProfilingEnabled(true);

  world.step();

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  const auto* deformable = profile.getStage("deformable_dynamics");
  ASSERT_NE(deformable, nullptr);
  EXPECT_TRUE(
      sx::compute::hasAcceleration(
          deformable->acceleration,
          sx::compute::ComputeStageAcceleration::Gpu));
  ASSERT_NE(profile.getStage("kinematics"), nullptr);

  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("deformable_dynamics"), std::string::npos);
  EXPECT_NE(summary.find("gpu"), std::string::npos);
}

TEST(WorldStepProfileIntegration, CapturesNestedParallelGraphProfiles)
{
  sx::World world;
  sx::compute::ParallelExecutor executor(2);
  world.setStepProfilingEnabled(true);

  world.step(executor);

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());

  const auto* kinematics = profile.getStage("kinematics");
  ASSERT_NE(kinematics, nullptr);
  ASSERT_FALSE(kinematics->graphProfiles.empty());
  EXPECT_GE(kinematics->maxGraphWorkerCount(), 1u);
  EXPECT_LE(
      kinematics->maxGraphParallelism(), kinematics->maxGraphWorkerCount());
  EXPECT_NE(
      kinematics->acceleration
          & sx::compute::toMask(
              sx::compute::ComputeStageAcceleration::TaskParallel),
      0u);

  const auto summary = profile.toSummaryText();
  EXPECT_NE(summary.find("graph_profiles="), std::string::npos);
  EXPECT_NE(summary.find("max_workers="), std::string::npos);
}

TEST(WorldStepProfileIntegration, DisablingFreezesTheLastSnapshot)
{
  sx::World world;
  world.setStepProfilingEnabled(true);
  world.step();

  const std::size_t stageCount = world.getLastStepProfile().stages.size();
  ASSERT_GT(stageCount, 0u);

  // Once disabled, further steps must not touch the retained snapshot.
  world.setStepProfilingEnabled(false);
  for (int i = 0; i < 3; ++i) {
    world.step();
  }

  EXPECT_FALSE(world.isStepProfilingEnabled());
  EXPECT_EQ(world.getLastStepProfile().stages.size(), stageCount);
}

TEST(WorldStepProfileIntegration, ClearResetsProfilingStateAndSnapshot)
{
  sx::World world;
  world.setStepProfilingEnabled(true);
  world.step();

  ASSERT_FALSE(world.getLastStepProfile().isEmpty());

  world.clear();

  EXPECT_FALSE(world.isStepProfilingEnabled());
  EXPECT_TRUE(world.getLastStepProfile().isEmpty());
}

TEST(WorldStepProfileIntegration, WallTimeCoversReplayRecordingBookkeeping)
{
  sx::World world;
  world.setReplayRecordingEnabled(true);
  world.setStepProfilingEnabled(true);

  world.step();

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  EXPECT_GE(world.getReplayFrameCount(), 2u);
  EXPECT_GE(profile.wallTime, profile.totalStageTime());

  const std::string summary = profile.toSummaryText();
  EXPECT_NE(summary.find("(unattributed overhead)"), std::string::npos);
}

TEST(WorldStepProfileIntegration, MultiStepReflectsLastStep)
{
  sx::World world;
  world.setStepProfilingEnabled(true);

  world.step(5);

  const auto& profile = world.getLastStepProfile();
  ASSERT_FALSE(profile.isEmpty());
  // Snapshot semantics match the deformable diagnostics: the profile reflects
  // the most recent step, not an accumulation across the five.
  EXPECT_EQ(profile.stepCount, 1u);
}

TEST(WorldStepProfileIntegration, WarmedCustomPipelineProfileDoesNotAllocate)
{
  sx::World world;
  sx::compute::SequentialExecutor executor;
  sx::compute::WorldStepPipeline pipeline;
  NoOpStage stage;
  pipeline.addStage(stage);

  world.setStepProfilingEnabled(true);
  world.step(executor, pipeline);
  world.step(executor, pipeline);
  ASSERT_FALSE(world.getLastStepProfile().isEmpty());
  ASSERT_NE(world.getLastStepProfile().getStage("p"), nullptr);
  const std::size_t stageCapacity
      = world.getLastStepProfile().stages.capacity();

  ScopedHeapAllocationCounter heapCounter;
  world.step(executor, pipeline);
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated by warmed profiled custom World step: "
      << heapCounter.allocationBytes();
  EXPECT_EQ(world.getLastStepProfile().stages.capacity(), stageCapacity);
  EXPECT_EQ(stage.executionCount, 3u);
}

TEST(WorldStepProfileIntegration, WarmedNestedGraphProfileDoesNotAllocate)
{
  sx::World world;
  sx::RigidBodyOptions options;
  options.isStatic = true;
  world.addRigidBody("profiled_kinematics_node", options);

  sx::compute::ParallelExecutor executor(2);

  world.setStepProfilingEnabled(true);
  world.step(executor);
  world.step(executor);

  const auto* kinematics = world.getLastStepProfile().getStage("kinematics");
  ASSERT_NE(kinematics, nullptr);
  ASSERT_FALSE(kinematics->graphProfiles.empty());
  ASSERT_FALSE(kinematics->graphProfiles.front().nodes.empty());
  const std::size_t stageCapacity
      = world.getLastStepProfile().stages.capacity();
  const std::size_t graphProfileCapacity = kinematics->graphProfiles.capacity();
  const std::size_t nodeProfileCapacity
      = kinematics->graphProfiles.front().nodes.capacity();

  ScopedHeapAllocationCounter heapCounter;
  world.step(executor);
  heapCounter.stop();

  EXPECT_EQ(heapCounter.allocationCount(), 0u)
      << "global heap bytes allocated by warmed nested compute-graph "
         "profiling: "
      << heapCounter.allocationBytes();

  const auto* refreshedKinematics
      = world.getLastStepProfile().getStage("kinematics");
  ASSERT_NE(refreshedKinematics, nullptr);
  ASSERT_FALSE(refreshedKinematics->graphProfiles.empty());
  ASSERT_FALSE(refreshedKinematics->graphProfiles.front().nodes.empty());
  EXPECT_EQ(world.getLastStepProfile().stages.capacity(), stageCapacity);
  EXPECT_EQ(
      refreshedKinematics->graphProfiles.capacity(), graphProfileCapacity);
  EXPECT_EQ(
      refreshedKinematics->graphProfiles.front().nodes.capacity(),
      nodeProfileCapacity);
}

#else

TEST(WorldStepProfileIntegration, BuildProfileOffMakesRuntimeToggleANoop)
{
  sx::World world;
  world.setStepProfilingEnabled(true);

  EXPECT_FALSE(world.isStepProfilingEnabled());

  world.step();

  EXPECT_TRUE(world.getLastStepProfile().isEmpty());
}

#endif
