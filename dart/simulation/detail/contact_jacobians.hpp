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

// @internal
//
// Implementation-detail header for opt-in differentiable simulation (PLAN-110
// WS2): the analytic CONTACT step Jacobian for rigid-body normal/friction
// contacts solved by the boxed-LCP path (`solveBoxedLcpContacts`), including
// contacts that excite the angular DOFs (lever arm not parallel to the normal)
// and multiple simultaneous contacts. It is NOT a
// public facade header: it takes the ECS/registry and contact types, which must
// never appear in a public signature. It lives under `detail/` so Doxygen and
// the API-boundary checks treat it as internal, and it is not part of the DART
// 7 public-facade promotion target. In-tree tests may include it directly; the
// entry point is marked DART_SIMULATION_API so the diff test can link it
// across the experimental .so (DART applies strict symbol visibility).

#include <dart/simulation/detail/world_registry_types.hpp>
#include <dart/simulation/diff/physical_parameter.hpp>
#include <dart/simulation/diff/step_derivatives.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Core>
#include <entt/entt.hpp>

#include <span>
#include <utility>
#include <vector>

namespace dart::simulation::detail {

/// Analytic CONTACT single-step Jacobian for a normal/friction rigid-body scene
/// resolved by the boxed-LCP path, evaluated at the current (pre-step) state.
///
/// The differentiated step is the boxed-LCP rigid-body sub-pipeline that
/// `World::step()` runs for a `ContactSolverMethod::BoxedLcp` world: integrate
/// the free (gravity/applied-force) velocity, solve the normal+friction LCP and
/// apply `Δv = M⁻¹ Jᵀ f`, project away residual penetration, then integrate the
/// (Euclidean) translational position from the post-contact velocity. This
/// slice covers the translational state of dynamic rigid bodies in contact with
/// prescribed rigid bodies (static or kinematic) or with each other.
///
/// The differentiated OUTPUT state is the body's translational `[pos; linvel]`,
/// but the LCP solve and the `Δv = M⁻¹ Jᵀ f` impulse map use the full 6-DOF
/// per-body dynamics: `M⁻¹` carries the world inertia and the angular rows of
/// `J` carry the screw-axis term `arm × dir` (arm = contact point − body
/// position). Contacts whose lever arm is NOT parallel to the normal (e.g. an
/// off-COM contact that excites rotation) and MULTIPLE simultaneous contacts
/// (e.g. a box resting on its corners) are therefore handled correctly: the
/// angular coupling enters the effective normal/friction mass in `A`, the
/// solved impulse, and the linear part of `Δv`. The translational output is
/// exact for the single step because the body's acquired angular velocity does
/// not feed back into the translational state within one step (position
/// advances by the linear velocity only); the orientation/angular-velocity
/// output is deliberately not assembled.
///
/// The state is `x = [q; q̇]` with `q` the stacked dynamic-body positions and
/// `q̇` their linear velocities (size `3 * dynamicBodyCount` each), and the
/// control `u = τ` the stacked applied forces. The constrained velocity is
/// `q̇' = q̇_free + M⁻¹ Jᵀ f`, so the contact-impulse contribution adds to the
/// contact-free velocity partials. The LCP-gradient core differentiates the
/// solved impulse analytically (the WS2 deliverable):
///
///   - The boxed-LCP path uses the convention `w = A f - b` with `b = -(J
///   v_free)
///     + bias`, so clamping rows (`f_i > tol`) satisfy `A_CC f_C = b_C`. By
///     implicit differentiation `∂f_C/∂z = A_CC⁻¹ (∂b_C/∂z − (∂A_CC/∂z) f_C)`;
///     separating rows contribute `∂f/∂z = 0`. `A_CC⁻¹` uses a rank-revealing
///     complete orthogonal decomposition to tolerate redundant contacts.
///   - The smooth geometric inputs (`∂A/∂z`, `∂b/∂z`, the free-velocity
///   partials,
///     and the position-update partials including the penetration projection)
///     are obtained by CENTRAL finite differencing the forward pieces over a
///     frozen active set (the registry body state is perturbed in place, the
///     pieces recomputed, and the state restored exactly, leaving the registry
///     unchanged on return). This FD-of-terms for the geometric inputs is the
///     interim approach (it matches WS1) and is distinct from, and non-circular
///     with, the FD-of-step used by the correctness checker.
///
/// The optional `mode` parameter (PLAN-110 WS5) selects the BACKWARD-pass
/// gradient refinement; it never changes the forward step. `Analytic` (the
/// default) is the true analytic gradient described above, including
/// restitution (a nonzero velocity bias differentiated through `∂b/∂z`).
/// `ComplementarityAware` reclassifies a saddle clamping contact as separating
/// to recover a non-zero search direction (a Nimble heuristic, not the true
/// derivative). `PreContactSurrogate` synthesizes a distance-based surrogate
/// gradient for a body that is approaching but not yet touching (also not the
/// true derivative, which is zero before contact). See `ContactGradientMode`.
///
/// @param registry  The world registry owning the rigid bodies (mutated and
///                   restored during finite differencing; unchanged on return).
/// @param contacts  Active contacts from `World::collide()` at the pre-step
///                   state (defines the frozen active set).
/// @param gravity   Uniform gravitational acceleration.
/// @param timeStep  Integration time step (> 0).
/// @param mode      Backward-pass gradient mode (default `Analytic`).
/// @return The assembled state and control Jacobians for the dynamic bodies'
///         translational state. Both matrices are empty when there is no
///         dynamic rigid body in scope. When there is no active normal
///         constraint the result equals the contact-free (free-fall) Jacobian
///         (except under `PreContactSurrogate`, which adds a surrogate block).
[[nodiscard]] DART_SIMULATION_API StepDerivatives contactStepDerivatives(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    ContactGradientMode mode = ContactGradientMode::Analytic);

/// One registered differentiable physical parameter: which rigid-body entity it
/// belongs to and which scalar parameter of that body to differentiate. This is
/// the detail-side, ECS-typed mirror of the public `PhysicalParameterSelector`;
/// it lives here (not in a public signature) so the registry/component types
/// stay internal.
using ParameterRegistration = std::pair<entt::entity, PhysicalParameter>;

/// Contact-aware single-step Jacobian as `contactStepDerivatives`, additionally
/// populating `StepDerivatives::parameterJacobian` with `∂x'/∂θ` for the given
/// registered physical parameters.
///
/// The columns of `parameterJacobian` are the partial derivatives of the
/// translational next state `x' = [pos; linvel]` with respect to the registered
/// scalar parameters, in registration order. A registration spans one or more
/// contiguous columns: `MASS` and `FRICTION` one column each, `INERTIA` three
/// (the diagonal principal moments Ixx, Iyy, Izz, in axis order). Each column
/// is computed by CENTRAL finite differencing the registered quantity (mass /
/// inertia diagonal entry in the registry's `MassProperties`, friction in the
/// `ContactMaterial`) and re-running the forward step pieces over the frozen
/// active set (recompute the smooth terms, re-solve the boxed LCP, re-assemble
/// the constrained next state), restoring the registry exactly on return. This
/// reuses the same FD-of-step mechanism as the state/control assembly and is
/// exact for the translational state of the supported slice. A parameter that
/// names a body outside the differentiated scene (no dynamic rigid body in
/// scope) contributes zero columns.
///
/// Supported: `PhysicalParameter::MASS`, `INERTIA`, `FRICTION`.
/// `CENTER_OF_MASS` is rejected upstream by `World::addDifferentiableParameter`
/// (the rigid-body step fixes the COM at the body origin, so its gradient is
/// identically zero).
///
/// @param registry   The world registry (mutated and restored during finite
///                    differencing; unchanged on return).
/// @param contacts   Active contacts (defines the frozen active set).
/// @param gravity    Uniform gravitational acceleration.
/// @param timeStep   Integration time step (> 0).
/// @param parameters The registered differentiable parameters, in registration
///                   order. When empty, `parameterJacobian` is left empty and
///                   the result equals `contactStepDerivatives`.
/// @param mode       Backward-pass gradient mode (default `Analytic`); applies
///                   to the state/control Jacobians exactly as in
///                   `contactStepDerivatives`. The parameter Jacobian is always
///                   the analytic FD-of-step column regardless of `mode`.
/// @return The state/control Jacobians (as `contactStepDerivatives`) plus the
///         parameter Jacobian. All matrices are empty when there is no dynamic
///         rigid body in scope.
[[nodiscard]] DART_SIMULATION_API StepDerivatives
contactStepDerivativesWithParameters(
    detail::WorldRegistry& registry,
    std::span<const Contact> contacts,
    const Eigen::Vector3d& gravity,
    double timeStep,
    const std::vector<ParameterRegistration>& parameters,
    ContactGradientMode mode = ContactGradientMode::Analytic);

} // namespace dart::simulation::detail
