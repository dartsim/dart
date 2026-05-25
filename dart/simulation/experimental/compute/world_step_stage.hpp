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

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/compute/compute_stage_metadata.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::compute {

/// Experimental contract for one stage in the World step pipeline.
class DART_EXPERIMENTAL_API WorldStepStage
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
};

/// Non-owning, ordered collection of stages for one experimental World step.
///
/// The pipeline is domain-neutral: rigid-body, articulated-body, deformable,
/// fluid, collision, constraint, control, sensor, and rendering-prep stages can
/// all use the same executor boundary and profiling surface.
class DART_EXPERIMENTAL_API WorldStepPipeline
{
public:
  WorldStepPipeline() = default;
  ~WorldStepPipeline() = default;

  WorldStepPipeline(const WorldStepPipeline&) = delete;
  WorldStepPipeline& operator=(const WorldStepPipeline&) = delete;
  WorldStepPipeline(WorldStepPipeline&&) noexcept = default;
  WorldStepPipeline& operator=(WorldStepPipeline&&) noexcept = default;

  WorldStepPipeline& addStage(WorldStepStage& stage);
  void clear() noexcept;

  [[nodiscard]] std::size_t getStageCount() const noexcept;
  [[nodiscard]] bool isEmpty() const noexcept;
  [[nodiscard]] WorldStepStage& getStage(std::size_t index) const;

  void execute(World& world, ComputeExecutor& executor);

private:
  std::vector<WorldStepStage*> m_stages;
};

/// Default kinematics/cache update stage for the experimental World.
class DART_EXPERIMENTAL_API KinematicsStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Default unconstrained rigid-body integration stage for the experimental
/// World.
class DART_EXPERIMENTAL_API RigidBodyIntegrationStage final
  : public WorldStepStage
{
public:
  explicit RigidBodyIntegrationStage(std::size_t batchSize = 64);

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getBatchSize() const noexcept;

private:
  std::size_t m_batchSize;
};

/// Unconstrained rigid-body integration stage driven by the batched
/// structure-of-arrays path.
///
/// Instead of per-entity component access, this stage extracts a
/// `RigidBodyStateBatch` (plus the immutable `RigidBodyModelBatch`), runs the
/// scalar-generic SoA integrator, and applies the result back to the World. The
/// world-space dynamics are frame-independent, so for frame-coupled rigid
/// bodies (a rigid body parented to another rigid body), where the
/// local-transform bookkeeping must run parent-before-child, this stage defers
/// to `RigidBodyIntegrationStage`. It is the experimental seam through which
/// the later SIMD and device batch paths drive a live World step.
class DART_EXPERIMENTAL_API BatchedRigidBodyIntegrationStage final
  : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Updates free rigid-body velocities from the assembled transient force buffer
/// (persistent applied force/torque plus gravity) without advancing positions.
/// Pairs with RigidBodyPositionStage so a contact-resolution stage can run at
/// the velocity level in between.
class DART_EXPERIMENTAL_API RigidBodyVelocityStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Advances free rigid-body poses from their current velocities and refreshes
/// frame caches. Run after velocity and contact resolution.
class DART_EXPERIMENTAL_API RigidBodyPositionStage final : public WorldStepStage
{
public:
  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;
};

/// Resolves contacts between free rigid bodies with sequential normal impulses
/// (frictionless, fully inelastic). Static bodies (non-positive mass) act as
/// immovable. This is the first contact-solver slice; friction, restitution
/// tuning, joints/links, and an LCP formulation are future work.
class DART_EXPERIMENTAL_API RigidBodyContactStage final : public WorldStepStage
{
public:
  explicit RigidBodyContactStage(std::size_t iterations = 8);

  [[nodiscard]] std::string_view getName() const noexcept override;
  [[nodiscard]] ComputeStageMetadata getMetadata() const noexcept override;
  void execute(World& world, ComputeExecutor& executor) override;

  [[nodiscard]] std::size_t getIterations() const noexcept;

private:
  std::size_t m_iterations;
};

} // namespace dart::simulation::experimental::compute
