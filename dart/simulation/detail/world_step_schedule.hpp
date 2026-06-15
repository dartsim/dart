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

#include <dart/simulation/common/exceptions.hpp>

#include <array>

#include <cstddef>

namespace dart::simulation::detail {

/// Internal solver-family choice for the built-in `World::step()` schedule.
enum class BuiltInRigidBodySolverFamily
{
  SequentialImpulse,
  Ipc,
};

/// Internal multibody integration-family choice for the built-in schedule.
enum class BuiltInMultibodyIntegrationFamily
{
  SemiImplicit,
  Variational,
};

/// Concrete stage slots used by the built-in DART 7 `World::step()`.
///
/// This is intentionally an internal planning table, not a public solver API:
/// the facade continues to expose method families and value-object policies.
enum class BuiltInWorldStepStageSlot
{
  RigidBodyVelocity,
  RigidBodyContact,
  RigidBodyPosition,
  RigidIpcContact,
  MultibodyVelocity,
  MultibodyForwardDynamics,
  MultibodyPosition,
  MultibodyVariationalIntegration,
  UnifiedConstraint,
  DeformableDynamics,
  Kinematics,
};

/// The built-in schedule must fit the inline WorldStepPipeline storage.
inline constexpr std::size_t kMaxBuiltInWorldStepStageSlots = 8;

struct BuiltInWorldStepScheduleOptions
{
  BuiltInRigidBodySolverFamily rigidBodySolver
      = BuiltInRigidBodySolverFamily::SequentialImpulse;
  BuiltInMultibodyIntegrationFamily multibodyIntegration
      = BuiltInMultibodyIntegrationFamily::SemiImplicit;
  bool hasRigidBodies = false;
  bool hasMultibodyStructures = false;
  bool hasDeformableBodies = false;
  bool includeKinematics = true;
};

struct BuiltInWorldStepSchedule
{
  std::array<BuiltInWorldStepStageSlot, kMaxBuiltInWorldStepStageSlots> slots{};
  std::size_t count = 0;

  void add(BuiltInWorldStepStageSlot slot)
  {
    DART_SIMULATION_THROW_T_IF(
        count >= slots.size(),
        InvalidOperationException,
        "Built-in World step schedule exceeds inline stage capacity");
    slots[count++] = slot;
  }

  [[nodiscard]] constexpr const BuiltInWorldStepStageSlot* begin()
      const noexcept
  {
    return slots.data();
  }

  [[nodiscard]] constexpr const BuiltInWorldStepStageSlot* end() const noexcept
  {
    return slots.data() + count;
  }

  [[nodiscard]] constexpr bool contains(
      BuiltInWorldStepStageSlot slot) const noexcept
  {
    for (std::size_t i = 0; i < count; ++i) {
      if (slots[i] == slot) {
        return true;
      }
    }
    return false;
  }
};

// Note: which stages need per-step preparation is no longer a separate table.
// Each stage overrides the virtual `compute::WorldStepStage::prepare()` (a
// no-op by default), and the built-in schedule calls `prepare()` on every
// scheduled stage, so the prepare set cannot drift from the stage classes.

// --- Minimal per-family scheduling capabilities ---------------------------
//
// PLAN-091 WP-091.11 slice 3: rather than branching the schedule on the family
// enum directly, each built-in family declares the scheduling-relevant
// capabilities below, and `makeBuiltInWorldStepSchedule` derives stage
// inclusion from declared domain presence plus these capabilities. Adding a
// family means declaring its capabilities here instead of threading another
// hand-listed domain-presence branch through the derivation. The fuller
// capability matrix (supported joints/actuators/shapes, differentiability) is
// a documented non-goal of this packet.

/// Whether a rigid-body solver family advances the rigid-body domain through
/// the split velocity/contact/position pipeline (which fuses with a multibody
/// constraint solve when a multibody domain is present), rather than through a
/// single combined contact-and-advance stage.
[[nodiscard]] constexpr bool builtInRigidSolverUsesSplitPipeline(
    BuiltInRigidBodySolverFamily family) noexcept
{
  return family == BuiltInRigidBodySolverFamily::SequentialImpulse;
}

/// The single stage that owns rigid-body contact and advancement for families
/// that do not use the split pipeline.
[[nodiscard]] constexpr BuiltInWorldStepStageSlot builtInCombinedRigidStage(
    [[maybe_unused]] BuiltInRigidBodySolverFamily family) noexcept
{
  // Ipc is currently the only rigid family with a combined contact-and-advance
  // stage; the split pipeline (sequential impulse) never reaches here.
  return BuiltInWorldStepStageSlot::RigidIpcContact;
}

/// Whether a multibody integration family fuses with the rigid sequential-
/// impulse solve into the shared unified-constraint stage (splitting multibody
/// velocity/position around a shared constraint solve), rather than advancing
/// the multibody domain through a single standalone stage.
[[nodiscard]] constexpr bool builtInMultibodyFusesWithUnifiedConstraint(
    BuiltInMultibodyIntegrationFamily family) noexcept
{
  return family == BuiltInMultibodyIntegrationFamily::SemiImplicit;
}

/// The single stage that advances the multibody domain when the integration
/// family runs standalone (not fused into a unified-constraint solve).
[[nodiscard]] constexpr BuiltInWorldStepStageSlot
builtInStandaloneMultibodyStage(
    BuiltInMultibodyIntegrationFamily family) noexcept
{
  return family == BuiltInMultibodyIntegrationFamily::Variational
             ? BuiltInWorldStepStageSlot::MultibodyVariationalIntegration
             : BuiltInWorldStepStageSlot::MultibodyForwardDynamics;
}

[[nodiscard]] inline BuiltInWorldStepSchedule makeBuiltInWorldStepSchedule(
    const BuiltInWorldStepScheduleOptions& options)
{
  BuiltInWorldStepSchedule schedule;

  const bool hasRigid = options.hasRigidBodies;
  const bool hasMultibody = options.hasMultibodyStructures;
  const bool hasDeformable = options.hasDeformableBodies;

  if (builtInRigidSolverUsesSplitPipeline(options.rigidBodySolver)) {
    const bool fuseMultibody = hasMultibody
                               && builtInMultibodyFusesWithUnifiedConstraint(
                                   options.multibodyIntegration);

    if (fuseMultibody) {
      // Rigid sequential impulse and a fusing multibody family share one
      // velocity solve, a unified constraint solve, then a shared position
      // update.
      if (hasRigid) {
        schedule.add(BuiltInWorldStepStageSlot::RigidBodyVelocity);
      }
      schedule.add(BuiltInWorldStepStageSlot::MultibodyVelocity);
      schedule.add(BuiltInWorldStepStageSlot::UnifiedConstraint);
      if (hasRigid) {
        schedule.add(BuiltInWorldStepStageSlot::RigidBodyPosition);
      }
      schedule.add(BuiltInWorldStepStageSlot::MultibodyPosition);
      if (hasDeformable) {
        schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
      }
    } else {
      // Split rigid contact pipeline; a non-fusing multibody family (if any)
      // advances through its own standalone stage between contact resolution
      // and the rigid position update.
      if (hasRigid) {
        schedule.add(BuiltInWorldStepStageSlot::RigidBodyVelocity);
        schedule.add(BuiltInWorldStepStageSlot::RigidBodyContact);
      }
      if (hasMultibody) {
        schedule.add(
            builtInStandaloneMultibodyStage(options.multibodyIntegration));
      }
      if (hasDeformable) {
        schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
      }
      if (hasRigid) {
        schedule.add(BuiltInWorldStepStageSlot::RigidBodyPosition);
      }
    }
  } else {
    // Combined rigid contact-and-advance stage; the multibody domain (if any)
    // advances standalone afterward.
    if (hasRigid) {
      schedule.add(builtInCombinedRigidStage(options.rigidBodySolver));
    }
    if (hasMultibody) {
      schedule.add(
          builtInStandaloneMultibodyStage(options.multibodyIntegration));
    }
    if (hasDeformable) {
      schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
    }
  }

  if (options.includeKinematics) {
    schedule.add(BuiltInWorldStepStageSlot::Kinematics);
  }
  return schedule;
}

} // namespace dart::simulation::detail
