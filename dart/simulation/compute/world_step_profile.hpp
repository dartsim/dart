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
#include <dart/simulation/compute/execution_profile.hpp>
#include <dart/simulation/export.hpp>

#include <Eigen/Core>

#include <chrono>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation::compute {

/// Wall-clock timing for one stage of a profiled World step.
struct DART_SIMULATION_API WorldStepStageProfile
{
  using Duration = std::chrono::steady_clock::duration;

  /// Stage name (see `WorldStepStage::getName`), e.g. "rigid_body_contact".
  std::string name;

  /// Broad domain the stage participates in (see
  /// `ComputeStageMetadata::domain`).
  ComputeStageDomain domain = ComputeStageDomain::Generic;

  /// Wall-clock time spent in this stage's `execute` during the step.
  Duration duration{};

  /// Acceleration opportunities advertised by the stage metadata.
  ComputeStageAccelerationMask acceleration = 0u;

  /// True when a backend-neutral accelerated implementation was active while
  /// this stage ran. The flag deliberately avoids naming a concrete device
  /// technology; stage metadata identifies whether GPU acceleration is a
  /// supported opportunity.
  bool acceleratedBackendEnabled = false;

  /// Profiles captured for compute graphs executed inside this stage. Empty
  /// when the stage does not execute compute graphs, or when profiling is not
  /// enabled.
  std::vector<ComputeExecutionProfile> graphProfiles;

  /// Sum of graph wall times captured inside this stage.
  [[nodiscard]] Duration totalGraphWallTime() const noexcept;

  /// Largest executor worker count among graph profiles captured in this stage.
  [[nodiscard]] std::size_t maxGraphWorkerCount() const noexcept;

  /// Largest observed parallelism among graph profiles captured in this stage.
  [[nodiscard]] std::size_t maxGraphParallelism() const noexcept;
};

/// Per-stage wall-clock profile of the most recent profiled World step.
///
/// This is the DART 7 World's text-first profiling surface: an opt-in,
/// allocation-light breakdown of where one `World::step` spent its time, stage
/// by stage. It is produced only when DART is built with `DART_BUILD_PROFILE`
/// and step profiling is enabled at runtime (`World::setStepProfilingEnabled`);
/// when the build option is off, the runtime toggle is a no-op and the hot step
/// path compiles back to the normal pipeline. The profile is intended for
/// non-GUI performance analysis -- including by AI agents -- through
/// `toSummaryText()` or the individual fields. It is empty before the first
/// profiled step.
///
/// The stage breakdown is the same domain-neutral pipeline surface the executor
/// runs (rigid-body, multibody, deformable, kinematics, ...), so the report is
/// stable across solver choices and reads the same from C++, Python, or a log.
struct DART_SIMULATION_API WorldStepProfile
{
  using Duration = std::chrono::steady_clock::duration;

  /// Number of steps captured into this snapshot. The `step` overloads
  /// overwrite the profile each step, so this is 1 after a single step and
  /// remains 1 after `step(count)` (the snapshot reflects the last step).
  std::size_t stepCount = 0;

  /// End-to-end wall time of the profiled step, including any per-stage
  /// scheduling overhead not attributed to an individual stage.
  Duration wallTime{};

  /// Per-stage timings, in pipeline execution order.
  std::vector<WorldStepStageProfile> stages;

  /// True before the first profiled step (no stages recorded).
  [[nodiscard]] bool isEmpty() const noexcept;

  /// Sum of all per-stage durations. `wallTime - totalStageTime()` is the
  /// unattributed step overhead (validation, scratch reset, bookkeeping).
  [[nodiscard]] Duration totalStageTime() const noexcept;

  /// Returns the stage profile with the given name, or nullptr if absent.
  [[nodiscard]] const WorldStepStageProfile* getStage(
      std::string_view name) const;

  /// Renders a compact, sorted, human- and agent-readable timing table.
  ///
  /// Stages are listed slowest-first with their domain, duration in
  /// milliseconds, and share of wall time. A trailing row reports the
  /// unattributed step overhead. Returns a short notice when empty.
  [[nodiscard]] std::string toSummaryText() const;

  /// Clears the profile back to the empty state.
  void reset() noexcept;
};

/// One finalize-time solver-resolution decision: which method family the World
/// requested for a domain, which it actually resolved, and why. When
/// `requested == resolved` the stage ran as requested; when they differ the
/// World substituted a method, and the substitution is recorded here instead of
/// happening silently.
struct DART_SIMULATION_API ResolvedConfigurationNote
{
  /// Domain the decision applies to, e.g. "rigid-body", "rigid-contact",
  /// "multibody".
  std::string domain;

  /// Requested method-family name.
  std::string requested;

  /// Resolved method-family name actually used by the step.
  std::string resolved;

  /// Reason the resolution differs from the request, or "as requested".
  std::string reason;

  /// True when the resolved family differs from the requested family.
  [[nodiscard]] bool isSubstitution() const noexcept
  {
    return requested != resolved;
  }
};

/// The per-domain solver families the World resolved at
/// `enterSimulationMode`, exposed alongside the step profile (see
/// `World::getResolvedConfiguration`). Every resolution is recorded with its
/// reason so a scene can no longer silently substitute a method.
///
/// PLAN-091 WP-091.11 slice 1: this records the resolved families (requested ==
/// resolved today); later slices classify the known silent substitutions
/// (for example AVBD rigid contact running sequential impulse) as recorded
/// decisions or strict-mode errors.
struct DART_SIMULATION_API ResolvedSolverConfiguration
{
  std::vector<ResolvedConfigurationNote> notes;

  [[nodiscard]] bool isEmpty() const noexcept
  {
    return notes.empty();
  }
  void reset() noexcept
  {
    notes.clear();
  }

  /// True if any domain resolved to a different family than requested.
  [[nodiscard]] bool hasSubstitution() const noexcept;

  /// Compact, human- and agent-readable table, one
  /// `domain: requested -> resolved (reason)` row per note.
  [[nodiscard]] std::string toSummaryText() const;
};

/// Solver-agnostic, per-domain physical quantities of one World at its current
/// state, computed by `World::computeStepMetrics()`.
///
/// This is a pure value object: it carries only plain scalars and `Vector3d`
/// momenta and names no solver, backend, registry, or component type. It is the
/// metrics surface PLAN-091 WP-091.24 builds its energy/momentum-conservation
/// and order-of-convergence validation on, and it reads the same from C++,
/// Python, or a log -- a snapshot of physical invariants that is stable across
/// solver choices. `computeStepMetrics()` is a read-only query: it inspects the
/// current World state (body mass, velocity, pose, inertia, gravity) without
/// running a step or mutating any state, so it never perturbs a trajectory.
///
/// Energy and momentum conventions match the rest of the simulation library:
///   - kinetic energy `0.5 m |v|^2 + 0.5 w^T I_world w` per rigid body, and the
///     multibody spatial form `0.5 V^T G V` (see
///     `compute::computeMultibodyMechanicalEnergy`);
///   - gravitational potential `- sum_i m_i g . x_i^com`, zero reference at the
///     world origin (consistent with `RigidBody::getPotentialEnergy` and the
///     variational integrator's energy diagnostic);
///   - linear momentum `sum_i m_i v_i`;
///   - angular momentum about the world origin
///     `sum_i (x_i^com x m_i v_i + I_world,i w_i)`.
struct DART_SIMULATION_API StepMetrics
{
  /// Total translational + rotational kinetic energy of every dynamic body.
  double kineticEnergy = 0.0;

  /// Total gravitational potential energy (zero reference at the world origin).
  double potentialEnergy = 0.0;

  /// `kineticEnergy + potentialEnergy`. For a passive (force-free) system this
  /// is the conserved mechanical energy the conservation gates track.
  double totalEnergy = 0.0;

  /// Total linear momentum `sum_i m_i v_i` (world frame).
  Eigen::Vector3d linearMomentum = Eigen::Vector3d::Zero();

  /// Total angular momentum about the world origin (world frame).
  Eigen::Vector3d angularMomentum = Eigen::Vector3d::Zero();

  /// Number of active rigid contacts at the current state.
  ///
  /// Populating this requires a narrow-phase contact pass, which mutates the
  /// World's collision-query cache and so cannot run from the read-only
  /// `computeStepMetrics()` query. This foundational WP-091.24 slice therefore
  /// leaves it at 0; a later slice adds an explicit contact-metrics path.
  std::size_t activeContactCount = 0;

  /// Maximum penetration depth over active rigid contacts (meters). Left at 0
  /// for the same read-only reason as `activeContactCount`.
  double maxPenetrationDepth = 0.0;

  /// RIQN/solver iterations performed on the last step, when the active solver
  /// exposes it; 0 otherwise. The default step profile records per-stage timing
  /// only, so this is 0 until a later slice threads solver-iteration data
  /// through the profile rather than inventing a value here.
  std::size_t lastStepIterations = 0;

  /// Final solver residual norm on the last step, when available; 0 otherwise
  /// (same provenance caveat as `lastStepIterations`).
  double lastStepResidual = 0.0;
};

} // namespace dart::simulation::compute
