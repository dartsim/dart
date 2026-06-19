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

#pragma once

#include <dart/simulation/compute/compute_stage_metadata.hpp>
#include <dart/simulation/compute/world_step_profile.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <dart/common/stl_allocator.hpp>

#include <array>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::common {
class MemoryAllocator;
} // namespace dart::common

namespace dart::simulation::compute {

/// Experimental contract for one stage in the World step pipeline.
class DART_SIMULATION_API WorldStepStage
{
public:
  virtual ~WorldStepStage() = default;

  WorldStepStage() = default;
  WorldStepStage(const WorldStepStage&) = delete;
  WorldStepStage& operator=(const WorldStepStage&) = delete;
  WorldStepStage(WorldStepStage&&) noexcept = default;
  WorldStepStage& operator=(WorldStepStage&&) noexcept = default;

  [[nodiscard]] virtual std::string_view getName() const noexcept = 0;
  [[nodiscard]] virtual ComputeStageMetadata getMetadata() const noexcept;
  virtual void execute(World& world, ComputeExecutor& executor) = 0;

  /// Optional pre-step preparation hook. The default is a no-op; a stage that
  /// caches per-step state overrides this. The built-in schedule calls
  /// `prepare()` on every scheduled stage, so a stateless stage needs no entry
  /// in any parallel "needs preparation" table.
  virtual void prepare(World& world);
};

/// Non-owning, ordered collection of stages for one DART 7 World step.
///
/// The pipeline is domain-neutral: rigid-body, articulated-body, deformable,
/// fluid, collision, constraint, control, sensor, and rendering-prep stages can
/// all use the same executor boundary and profiling surface.
class DART_SIMULATION_API WorldStepPipeline
{
public:
  WorldStepPipeline();
  explicit WorldStepPipeline(common::MemoryAllocator& allocator);
  ~WorldStepPipeline() = default;

  WorldStepPipeline(const WorldStepPipeline&) = delete;
  WorldStepPipeline& operator=(const WorldStepPipeline&) = delete;
  WorldStepPipeline(WorldStepPipeline&&) noexcept = default;
  WorldStepPipeline& operator=(WorldStepPipeline&&) noexcept = default;

  /// Number of stage pointers stored inline before custom pipelines spill over.
  static constexpr std::size_t kInlineStageCount = 8;

  WorldStepPipeline& addStage(WorldStepStage& stage);
  void clear() noexcept;

  [[nodiscard]] std::size_t getStageCount() const noexcept;
  [[nodiscard]] bool isEmpty() const noexcept;
  [[nodiscard]] WorldStepStage& getStage(std::size_t index) const;

  void execute(World& world, ComputeExecutor& executor);
  [[nodiscard]] WorldStepProfile executeProfiled(
      World& world, ComputeExecutor& executor);
  void executeProfiled(
      World& world, ComputeExecutor& executor, WorldStepProfile& profile);

private:
  std::array<WorldStepStage*, kInlineStageCount> m_stages{};
  std::vector<WorldStepStage*, common::StlAllocator<WorldStepStage*>>
      m_overflowStages;
  std::size_t m_stageCount = 0;
};

} // namespace dart::simulation::compute
