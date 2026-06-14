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

[[nodiscard]] inline BuiltInWorldStepSchedule makeBuiltInWorldStepSchedule(
    const BuiltInWorldStepScheduleOptions& options)
{
  BuiltInWorldStepSchedule schedule;
  const bool variationalSelected
      = options.multibodyIntegration
        == BuiltInMultibodyIntegrationFamily::Variational;

  switch (options.rigidBodySolver) {
    case BuiltInRigidBodySolverFamily::SequentialImpulse:
      if (options.hasMultibodyStructures && variationalSelected) {
        if (options.hasRigidBodies) {
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyVelocity);
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyContact);
        }
        schedule.add(
            BuiltInWorldStepStageSlot::MultibodyVariationalIntegration);
        if (options.hasDeformableBodies) {
          schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
        }
        if (options.hasRigidBodies) {
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyPosition);
        }
      } else if (options.hasMultibodyStructures) {
        if (options.hasRigidBodies) {
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyVelocity);
        }
        schedule.add(BuiltInWorldStepStageSlot::MultibodyVelocity);
        schedule.add(BuiltInWorldStepStageSlot::UnifiedConstraint);
        if (options.hasRigidBodies) {
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyPosition);
        }
        schedule.add(BuiltInWorldStepStageSlot::MultibodyPosition);
        if (options.hasDeformableBodies) {
          schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
        }
      } else {
        if (options.hasRigidBodies) {
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyVelocity);
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyContact);
        }
        if (options.hasDeformableBodies) {
          schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
        }
        if (options.hasRigidBodies) {
          schedule.add(BuiltInWorldStepStageSlot::RigidBodyPosition);
        }
      }
      break;
    case BuiltInRigidBodySolverFamily::Ipc:
      if (options.hasRigidBodies) {
        schedule.add(BuiltInWorldStepStageSlot::RigidIpcContact);
      }
      if (options.hasMultibodyStructures) {
        schedule.add(
            variationalSelected
                ? BuiltInWorldStepStageSlot::MultibodyVariationalIntegration
                : BuiltInWorldStepStageSlot::MultibodyForwardDynamics);
      }
      if (options.hasDeformableBodies) {
        schedule.add(BuiltInWorldStepStageSlot::DeformableDynamics);
      }
      break;
  }

  if (options.includeKinematics) {
    schedule.add(BuiltInWorldStepStageSlot::Kinematics);
  }
  return schedule;
}

} // namespace dart::simulation::detail
