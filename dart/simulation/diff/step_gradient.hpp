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

/// Reverse-mode (vector-Jacobian product) gradient of a single differentiable
/// simulation step.
///
/// Given an upstream gradient of a scalar loss with respect to the next state
/// `dL/dx'` (size `2 * ndof`), the step's reverse-mode rule pulls it back to
/// the inputs of the step:
///
/// - `state` is `dL/dx = stateJacobianᵀ · (dL/dx')`, size `2*ndof`, the
/// gradient
///   with respect to the current state `x = [q; q̇]`.
/// - `control` is `dL/du = controlJacobianᵀ · (dL/dx')`, size `ndof`, the
///   gradient with respect to the control `u = τ`.
///
/// This is a plain Eigen-typed value object with no solver, backend, or ECS
/// dependency. The vectors are valid for the configuration at which the step's
/// Jacobians were evaluated.
struct DART_SIMULATION_API StepGradient
{
  Eigen::VectorXd state;   ///< dL/dx = Jₓᵀ · (dL/dx'), size 2*ndof
  Eigen::VectorXd control; ///< dL/du = Jᵤᵀ · (dL/dx'), size ndof
};

} // namespace dart::simulation
