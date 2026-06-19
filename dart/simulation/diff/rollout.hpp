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

#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <Eigen/Core>

#include <vector>

#include <cstddef>

namespace dart::simulation::diff {

/// Reverse-mode gradient of a whole multi-step differentiable rollout.
///
/// Produced by `RolloutTrajectory::rolloutVjp`. Shapes mirror the rollout
/// inputs:
/// - `initialStateGrad` is `dL/dx_0`, size `2*ndof` (the gradient with respect
///   to the rollout's initial state),
/// - `controlGrads` has one entry per rolled-out step (`steps` entries); entry
///   `t` is `dL/du_t`, size `ndof`, the gradient with respect to the control
///   applied at step `t`.
struct DART_SIMULATION_API RolloutGradient
{
  Eigen::VectorXd initialStateGrad;          ///< dL/dx_0, size 2*ndof
  std::vector<Eigen::VectorXd> controlGrads; ///< dL/du_t, steps x ndof
};

/// Forward result and reverse-mode entry point for a multi-step differentiable
/// rollout.
///
/// This is the FRAMEWORK-NEUTRAL (no-torch) analogue of chaining the per-step
/// autograd `timestep` in a Python loop: it composes the per-step Jacobians of
/// `World::getStepDerivatives()` into a single trajectory, then walks the chain
/// backwards with `World::applyStepVjp` semantics.
///
/// `states` is the recorded trajectory `[x_0, x_1, ..., x_steps]`
/// (`steps + 1` state vectors, each size `2*ndof`); `x_0` is the initial state
/// passed to `rollout`, and `x_{t+1}` is the state after stepping with
/// `control[t]`. `stepDerivatives[t]` is `StepDerivatives` of the `t`-th step,
/// evaluated at the pre-step configuration `x_t` with control `control[t]`.
///
/// The value object owns its data (plain Eigen vectors/matrices) and carries no
/// World, registry, or backend reference, so it is safe to outlive the World
/// that produced it.
struct DART_SIMULATION_API RolloutTrajectory
{
  /// Recorded state trajectory `[x_0, ..., x_steps]`, `steps + 1` entries.
  std::vector<Eigen::VectorXd> states;

  /// Per-step explicit Jacobians, `steps` entries; `stepDerivatives[t]` is the
  /// Jacobian of step `t` (mapping `x_t, u_t -> x_{t+1}`).
  std::vector<StepDerivatives> stepDerivatives;

  /// Number of rolled-out steps (`states.size() - 1`).
  [[nodiscard]] std::size_t numSteps() const noexcept
  {
    return stepDerivatives.size();
  }

  /// Reverse-mode VJP of the whole rollout for an upstream gradient on the
  /// FINAL state only.
  ///
  /// Given `finalStateGrad = dL/dx_steps` (size `2*ndof`), this backpropagates
  /// through the chained per-step Jacobians and returns `{initialStateGrad,
  /// controlGrads}`. The reverse-mode chain (with adjoint `g`, initialized to
  /// `finalStateGrad`) is, for `t = steps-1 ... 0`:
  ///   - `controlGrads[t] = controlJacobianᵀ_t · g`,
  ///   - `g = stateJacobianᵀ_t · g`,
  /// and `initialStateGrad = g` after the loop.
  ///
  /// @throws InvalidArgumentException if `finalStateGrad` does not have the
  ///         final-state size `2*ndof`.
  [[nodiscard]] RolloutGradient rolloutVjp(
      const Eigen::VectorXd& finalStateGrad) const;

  /// Reverse-mode VJP of the whole rollout with PER-STEP state gradients.
  ///
  /// `stateGrads` carries an upstream gradient on each recorded state, indexed
  /// to match `states` (size `steps + 1`; `stateGrads[t] = dL/dx_t`). This is
  /// the general case where the loss reads intermediate states, not just the
  /// final one. The adjoint accumulates the local contribution at each state:
  /// starting from `g = stateGrads[steps]`, for `t = steps-1 ... 0`:
  ///   - `controlGrads[t] = controlJacobianᵀ_t · g`,
  ///   - `g = stateGrads[t] + stateJacobianᵀ_t · g`,
  /// and `initialStateGrad = g` after the loop. With only the final entry
  /// nonzero this reduces exactly to `rolloutVjp(finalStateGrad)`.
  ///
  /// @throws InvalidArgumentException if `stateGrads.size() != steps + 1` or
  /// any
  ///         entry does not have size `2*ndof`.
  [[nodiscard]] RolloutGradient rolloutVjpPerStep(
      const std::vector<Eigen::VectorXd>& stateGrads) const;
};

/// Run a FRAMEWORK-NEUTRAL differentiable multi-step rollout.
///
/// Forward pass: sets the initial state `x_0 = initialStateVector` on `world`,
/// then for `t` in `[0, steps)` sets `control[t]` as the world control, steps
/// the world once, captures the step's `StepDerivatives`, and records the new
/// state. The recorded trajectory and per-step Jacobians are returned in a
/// `RolloutTrajectory`, whose `rolloutVjp` performs the reverse-mode pass.
///
/// `controlSequence` is the row-major `steps x num_efforts` control matrix: row
/// `t` is the control `u_t` applied at step `t`. `world` is left at the final
/// rolled-out state `x_steps` on return.
/// The world's general state/control vector layout must match the active
/// differentiable family for the stepped scene (rigid-only and
/// single-multibody scenes satisfy this today).
///
/// @param world differentiable World (constructed with `differentiable=true`);
///        must have differentiable support compiled (`DART_BUILD_DIFF=ON`).
/// @param initialStateVector initial state `x_0`, size `2 *
/// world.getNumDofs()`.
/// @param controlSequence control matrix, size `steps x world.getNumEfforts()`.
/// @param steps number of steps to roll out (`>= 1`).
/// @returns the trajectory and per-step Jacobians; call `rolloutVjp` on it for
///          gradients.
/// @throws InvalidArgumentException if `steps == 0`, `initialStateVector` is
/// the
///         wrong size, or `controlSequence` is not `steps x num_efforts`.
/// @throws InvalidOperationException / NotImplementedException under the same
///         conditions as `World::getStepDerivatives()` (not differentiable, not
///         built, or an unsupported contact slice during a step).
[[nodiscard]] DART_SIMULATION_API RolloutTrajectory rollout(
    World& world,
    const Eigen::VectorXd& initialStateVector,
    const Eigen::MatrixXd& controlSequence,
    std::size_t steps);

} // namespace dart::simulation::diff
