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

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/compute_graph.hpp"
#include "dart/simulation/compute/deformable_psd_backend.hpp"
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/world_kinematics_graph.hpp"
#include "dart/simulation/compute/world_step_stage.hpp"
#include "dart/simulation/world.hpp"

#include <dart/config.hpp>

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <memory>

#if DART_BUILD_PROFILE
  #include <chrono>
#endif

namespace dart::simulation::compute {

//==============================================================================
ComputeStageMetadata WorldStepStage::getMetadata() const noexcept
{
  return {};
}

//==============================================================================
void WorldStepStage::prepare(World& /*world*/)
{
  // Default no-op: stateless stages need no per-step preparation.
}

//==============================================================================
WorldStepPipeline::WorldStepPipeline() = default;

//==============================================================================
WorldStepPipeline::WorldStepPipeline(common::MemoryAllocator& allocator)
  : m_overflowStages(common::StlAllocator<WorldStepStage*>{allocator})
{
}

//==============================================================================
WorldStepPipeline& WorldStepPipeline::addStage(WorldStepStage& stage)
{
  if (m_stageCount >= m_stages.size()) {
    if (m_overflowStages.empty()) {
      m_overflowStages.reserve(m_stages.size());
    }

    m_overflowStages.push_back(&stage);
    ++m_stageCount;
    return *this;
  }

  m_stages[m_stageCount] = &stage;
  ++m_stageCount;
  return *this;
}

//==============================================================================
void WorldStepPipeline::clear() noexcept
{
  m_stageCount = 0;
  m_overflowStages.clear();
}

//==============================================================================
std::size_t WorldStepPipeline::getStageCount() const noexcept
{
  return m_stageCount;
}

//==============================================================================
bool WorldStepPipeline::isEmpty() const noexcept
{
  return m_stageCount == 0;
}

//==============================================================================
WorldStepStage& WorldStepPipeline::getStage(std::size_t index) const
{
  DART_SIMULATION_THROW_T_IF(
      index >= m_stageCount,
      OutOfRangeException,
      "World step pipeline stage index {} is out of range",
      index);

  if (index < m_stages.size()) {
    return *m_stages[index];
  }

  return *m_overflowStages[index - m_stages.size()];
}

//==============================================================================
void WorldStepPipeline::execute(World& world, ComputeExecutor& executor)
{
  for (std::size_t i = 0; i < m_stageCount; ++i) {
    getStage(i).execute(world, executor);
  }
}

#if DART_BUILD_PROFILE
namespace {

/// Executor adapter used only by World step profiling. It preserves the stage's
/// normal executor injection while capturing any nested compute graph profiles
/// the stage runs.
class WorldStepProfilingExecutor final : public ComputeExecutor
{
public:
  WorldStepProfilingExecutor(
      ComputeExecutor& delegate, WorldStepStageProfile& stageProfile)
    : m_delegate(delegate), m_stageProfile(stageProfile)
  {
  }

  ~WorldStepProfilingExecutor() override
  {
    m_stageProfile.graphProfiles.resize(m_nextGraphProfileIndex);
  }

  void execute(const ComputeGraph& graph) override
  {
    auto& profile = nextGraphProfile();
    m_delegate.executeProfiled(graph, profile);
  }

  [[nodiscard]] ComputeExecutionProfile executeProfiled(
      const ComputeGraph& graph) override
  {
    auto& profile = nextGraphProfile();
    m_delegate.executeProfiled(graph, profile);
    return profile;
  }

  void executeProfiled(
      const ComputeGraph& graph, ComputeExecutionProfile& profile) override
  {
    auto& storedProfile = nextGraphProfile();
    m_delegate.executeProfiled(graph, storedProfile);
    profile = storedProfile;
  }

  [[nodiscard]] std::size_t getWorkerCount() const override
  {
    return m_delegate.getWorkerCount();
  }

private:
  ComputeExecutionProfile& nextGraphProfile()
  {
    if (m_nextGraphProfileIndex < m_stageProfile.graphProfiles.size()) {
      return m_stageProfile.graphProfiles[m_nextGraphProfileIndex++];
    }

    m_stageProfile.graphProfiles.emplace_back();
    ++m_nextGraphProfileIndex;
    return m_stageProfile.graphProfiles.back();
  }

  ComputeExecutor& m_delegate;
  WorldStepStageProfile& m_stageProfile;
  std::size_t m_nextGraphProfileIndex = 0;
};

} // namespace
#endif

//==============================================================================
WorldStepProfile WorldStepPipeline::executeProfiled(
    World& world, ComputeExecutor& executor)
{
  WorldStepProfile profile;
  executeProfiled(world, executor, profile);
  return profile;
}

//==============================================================================
void WorldStepPipeline::executeProfiled(
    World& world, ComputeExecutor& executor, WorldStepProfile& profile)
{
#if DART_BUILD_PROFILE
  profile.stepCount = 1;
  profile.wallTime = {};
  profile.stages.resize(m_stageCount);

  const auto stepStart = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < m_stageCount; ++i) {
    auto& stage = getStage(i);
    const auto metadata = stage.getMetadata();

    auto& entry = profile.stages[i];
    entry.name = stage.getName();
    entry.domain = metadata.domain;
    entry.duration = {};
    entry.acceleration = metadata.acceleration;
    entry.acceleratedBackendEnabled = false;

    const bool stageCanUseGpuBackend
        = hasAcceleration(metadata.acceleration, ComputeStageAcceleration::Gpu);
    const bool acceleratedBackendBefore
        = stageCanUseGpuBackend && isDeformablePsdAccelerated();

    WorldStepProfilingExecutor profilingExecutor(executor, entry);
    const auto stageStart = std::chrono::steady_clock::now();
    stage.execute(world, profilingExecutor);
    const auto stageEnd = std::chrono::steady_clock::now();

    entry.duration = stageEnd - stageStart;
    entry.acceleratedBackendEnabled
        = acceleratedBackendBefore
          || (stageCanUseGpuBackend && isDeformablePsdAccelerated());
  }
  profile.wallTime = std::chrono::steady_clock::now() - stepStart;
#else
  profile.reset();
  execute(world, executor);
#endif
}

//==============================================================================
std::string_view KinematicsStage::getName() const noexcept
{
  return "kinematics";
}

//==============================================================================
KinematicsStage::KinematicsStage() : KinematicsStage(nullptr) {}

//==============================================================================
KinematicsStage::KinematicsStage(common::MemoryManager* memoryManager)
  : m_memoryManager(memoryManager),
    m_cachedGraph(nullptr, CachedGraphDeleter{memoryManager})
{
}

//==============================================================================
KinematicsStage::~KinematicsStage() = default;

//==============================================================================
void KinematicsStage::CachedGraphDeleter::operator()(
    WorldKinematicsGraph* graph) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, graph);
}

//==============================================================================
ComputeStageMetadata KinematicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::Kinematics,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality};
}

//==============================================================================
void KinematicsStage::prepare(World& world)
{
  if (m_cachedWorld != &world || !m_cachedGraph) {
    auto* graph
        = m_memoryManager != nullptr
              ? stage_detail::constructStageOwnedScratch<WorldKinematicsGraph>(
                    m_memoryManager, world, m_memoryManager->getFreeAllocator())
              : stage_detail::constructStageOwnedScratch<WorldKinematicsGraph>(
                    nullptr, world);
    m_cachedGraph = std::unique_ptr<WorldKinematicsGraph, CachedGraphDeleter>(
        graph, CachedGraphDeleter{m_memoryManager});
    m_cachedWorld = &world;
    return;
  }

  if (!m_cachedGraph->isTopologyCurrent()) {
    m_cachedGraph->rebuild();
  }
}

//==============================================================================
void KinematicsStage::execute(World& world, ComputeExecutor& executor)
{
  if (!m_cachedGraph || m_cachedWorld != &world || !world.isSimulationMode()
      || !m_cachedGraph->isTopologyCurrent()) {
    prepare(world);
  }
  m_cachedGraph->execute(executor);
}

} // namespace dart::simulation::compute
