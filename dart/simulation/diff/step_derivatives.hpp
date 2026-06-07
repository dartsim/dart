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

#include <dart/simulation/export.hpp>

#include <Eigen/Core>

namespace dart::simulation {

/// Explicit Jacobian blocks of a single differentiable simulation step.
///
/// The step maps the current state `x = [q; q̇]` and control `u = τ` to the next
/// state `x' = [q'; q̇']`. With `ndof` movable generalized coordinates, the
/// state vector has size `2 * ndof` and the control vector has size `ndof`.
///
/// - `stateJacobian` is `∂x'/∂x`, size `2*ndof x 2*ndof`, laid out as the block
///   matrix `[[∂q'/∂q, ∂q'/∂q̇], [∂q̇'/∂q, ∂q̇'/∂q̇]]`.
/// - `controlJacobian` is `∂x'/∂u`, size `2*ndof x ndof`, laid out as
///   `[[∂q'/∂τ], [∂q̇'/∂τ]]`.
/// - `parameterJacobian` is `∂x'/∂θ`, size `2*ndof x p`, where `θ` is the
/// vector
///   of `p` registered scalar physical parameters (mass / inertia / etc.) in
///   registration order. It is populated ONLY when one or more differentiable
///   parameters were registered on the World (via
///   `World::addDifferentiableParameter`); with no registered parameters it is
///   an empty matrix, so existing state/control-only consumers are unaffected.
///
/// This is a plain Eigen-typed value object with no solver, backend, or ECS
/// dependency. Derivatives are valid for the configuration at which they were
/// evaluated.
struct DART_SIMULATION_API StepDerivatives
{
  Eigen::MatrixXd stateJacobian;   ///< ∂x'/∂x, size 2*ndof x 2*ndof
  Eigen::MatrixXd controlJacobian; ///< ∂x'/∂u, size 2*ndof x ndof

  /// ∂x'/∂θ, size 2*ndof x (#registered scalar parameters). Empty when no
  /// differentiable parameter is registered. Always present (even in the
  /// DART_BUILD_DIFF=OFF build), where it stays empty.
  Eigen::MatrixXd parameterJacobian;
};

} // namespace dart::simulation
