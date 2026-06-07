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

#include <dart/simulation/multibody/multibody_options.hpp>

#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace dart::simulation {

/// Solver family used for free rigid-body dynamics in the default DART 7
/// World step pipeline.
enum class RigidBodySolver
{
  SequentialImpulse,
  Ipc,
};

/// Selects how the rigid-body contact stage resolves active contacts.
///
/// This is an explicit, documented opt-in. It is independent of the
/// differentiable flag: a non-differentiable world may use either method, and a
/// differentiable world defaults to the same `SequentialImpulse` path as any
/// other. The enum names the contact formulation, without exposing backend,
/// registry, ECS storage, or concrete solver-object types, so it is safe on the
/// public facade surface.
enum class ContactSolverMethod
{
  /// Sequential-impulse (Gauss-Seidel) normal+friction solve. The default and
  /// the long-standing path; unchanged by the LCP opt-in.
  SequentialImpulse,

  /// Boxed-LCP rigid-body contact solve via a pivoting (Dantzig-style) solver.
  /// The current rigid-body slice assembles one normal row plus two Coulomb
  /// tangent rows per active rigid-body contact (`findex`-coupled box bounds).
  /// Mixed articulated scenes use the unified constraint/contact path.
  BoxedLcp,
};

/// Selects how the differentiable contact stage produces its BACKWARD-pass
/// gradient. This is an opt-in refinement knob for DART 7
/// differentiable simulation (PLAN-110 WS5): it affects ONLY the gradient
/// (`World::getStepDerivatives()` / `applyStepVjp()`), never the forward step.
/// The forward result is identical for every mode.
///
/// The default `Analytic` reproduces exactly today's behavior. The other modes
/// trade exactness for a more useful search direction in degenerate / pre-
/// contact regimes; they are research-grade heuristics, NOT the true
/// derivative, and are documented as such below.
enum class ContactGradientMode
{
  /// The analytic contact gradient (the long-standing default): clamping /
  /// separating active-set classification with the reduced clamping Delassus
  /// inverse `A_CC⁻¹` and the sliding-friction upper-bound mapping. Restitution
  /// (a nonzero velocity bias) is differentiated here too. This is the true
  /// derivative of the forward step on the smooth side of every active-set
  /// boundary.
  Analytic,

  /// Complementarity-aware heuristic (after Nimble): a clamping contact whose
  /// analytic gradient is (near-)zero in the slide/approach direction (a
  /// saddle of the LCP) is RECLASSIFIED as separating for the backward pass so
  /// it yields a non-zero search direction. This is NOT the true derivative; it
  /// is a heuristic that trades correctness for a usable gradient at a
  /// degenerate active-set boundary.
  ComplementarityAware,

  /// Pre-contact surrogate (backward-only): a body that is approaching but not
  /// yet touching has an identically-zero analytic contact gradient. In this
  /// mode the backward pass synthesizes a distance-based surrogate gradient so
  /// `∂(loss)/∂(input)` is non-zero before contact and points toward contact.
  /// The forward step is unchanged. This is a surrogate, NOT the true
  /// derivative (which is genuinely zero before contact).
  PreContactSurrogate,
};

/// Opt-in automatic deactivation ("sleeping") policy.
///
/// Deactivation is disabled by default. When enabled, supported dynamic rigid
/// bodies and semi-implicit multibodies that remain below the configured speed
/// thresholds for `timeUntilSleep` may be frozen as a whole contact island
/// until a contact with an active dynamic body or an explicit user disturbance
/// wakes them. Gravity alone is not a wake disturbance.
struct DeactivationOptions
{
  /// Enable automatic deactivation. Defaults off to preserve exact legacy
  /// forward results and diagnostics.
  bool enabled = false;

  /// Rigid-body linear speed threshold in m/s.
  double linearSpeedThreshold = 1e-3;

  /// Rigid-body angular speed threshold in rad/s.
  double angularSpeedThreshold = 1e-3;

  /// Multibody generalized-speed threshold.
  double generalizedSpeedThreshold = 1e-3;

  /// Continuous quiet time required before a candidate island sleeps.
  double timeUntilSleep = 0.5;

  /// Hysteresis multiplier used by the wake-side speed band.
  double wakeThresholdScale = 2.0;

  /// Explicit force/torque/command magnitude above this value wakes a body.
  double disturbanceForceThreshold = 1e-9;
};

/// Construction-time options for a `World`.
///
/// `WorldOptions` is a plain value object: it carries the initial time step,
/// gravity, domain solver-family choices, whether the world opts in to
/// differentiable simulation, which contact-solver method the rigid-body
/// contact stage uses, and the root CPU allocator knobs for the World-owned
/// memory hierarchy. It does not expose any backend, stage, registry, or ECS
/// storage type, so it is safe as a public facade surface.
///
/// Differentiability is opt-in and defaults off. When `differentiable` is false
/// the step executes the identical non-differentiable code path with no extra
/// work; the flag does not alter the forward result.
struct WorldOptions
{
  /// Simulation time step in seconds (must be positive and finite).
  double timeStep = 0.001;

  /// Uniform gravitational acceleration applied to dynamic bodies.
  Eigen::Vector3d gravity{0.0, 0.0, -9.81};

  /// Free rigid-body solver family used by the built-in `World::step()`
  /// schedule. Defaults to sequential impulse.
  RigidBodySolver rigidBodySolver = RigidBodySolver::SequentialImpulse;

  /// Multibody domain method-family options used by the built-in
  /// `World::step()` schedule. Defaults to semi-implicit integration.
  MultibodyOptions multibodyOptions;

  /// Opt in to differentiable simulation. Default false (zero overhead).
  bool differentiable = false;

  /// Rigid-body contact resolution method. Defaults to `SequentialImpulse`.
  ContactSolverMethod contactSolverMethod
      = ContactSolverMethod::SequentialImpulse;

  /// Backward-pass contact gradient mode. Defaults to `Analytic` (today's
  /// behavior). Affects ONLY the gradient; the forward step is identical for
  /// every value. See `ContactGradientMode` for the per-mode semantics.
  ContactGradientMode contactGradientMode = ContactGradientMode::Analytic;

  /// When true, `enterSimulationMode` rejects (throws) a configuration whose
  /// resolved solver families substitute a method the World did not request
  /// (see `World::getResolvedConfiguration`). Default false records the
  /// substitution in the resolved-configuration report instead of erroring.
  /// PLAN-091 WP-091.11.
  bool strictSolverResolution = false;

  /// Automatic body deactivation policy. Disabled by default.
  DeactivationOptions deactivationOptions;

  /// Optional base allocator for the World's memory hierarchy. If null, the
  /// default DART memory allocator is used. The pointed-to allocator must
  /// outlive the World because the World-owned MemoryManager borrows it.
  common::MemoryAllocator* baseAllocator = nullptr;

  /// Initial bytes reserved for per-step frame scratch. The frame allocator may
  /// grow if a step exceeds this size; a later bake/build reserve path should
  /// size this high enough that steady-state simulation does not grow it.
  std::size_t frameScratchInitialCapacity = 65536;

  /// Initial bytes reserved for the World's free-list allocator. This is the
  /// persistent heap-like arena used by the World memory hierarchy.
  std::size_t freeListInitialAllocation = 1048576 /* 1 MB */;

  /// Whether the World's free-list allocator may grow after construction.
  /// `FixedCapacity` is intended for bake/build-sized worlds where runtime
  /// growth would violate a no-dynamic-allocation contract.
  common::FreeListAllocator::GrowthPolicy freeListGrowthPolicy
      = common::FreeListAllocator::GrowthPolicy::Expand;
};

} // namespace dart::simulation
