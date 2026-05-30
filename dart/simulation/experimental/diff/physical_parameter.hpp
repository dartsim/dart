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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <optional>

namespace dart::simulation::experimental {

/// A differentiable PHYSICAL PARAMETER of a rigid body.
///
/// Selecting a parameter for differentiation (via
/// `World::addDifferentiableParameter`) makes the most recent step's
/// `StepDerivatives::parameterJacobian` carry the partial derivative `∂x'/∂θ`
/// of the next state with respect to that parameter, alongside the
/// state/control Jacobians. This is the system-identification primitive: it
/// answers "how does the post-step state change when I nudge this body's mass /
/// inertia / friction by a little?".
///
/// Supported (PLAN-110 WS4): `MASS` (one column), `INERTIA` (three columns, the
/// diagonal principal moments), and `FRICTION` (one column) are implemented and
/// finite-difference verified. `CENTER_OF_MASS` is NOT supported for rigid
/// bodies: the rigid-body step assumes the center of mass at the body origin
/// (`MassProperties::localCenterOfMass` is unused outside the multibody path),
/// so its single-step gradient is identically zero; registering it throws
/// `NotImplementedException` from `World::addDifferentiableParameter`.
enum class PhysicalParameter
{
  /// The body's scalar mass. One differentiable scalar.
  MASS,
  /// The body's center of mass in the body frame. Not supported (the rigid-body
  /// step fixes the COM at the body origin, so the gradient is identically
  /// zero); registering it throws `NotImplementedException`.
  CENTER_OF_MASS,
  /// The body's rotational inertia tensor. Three differentiable scalars (the
  /// diagonal principal moments Ixx, Iyy, Izz, in axis order).
  INERTIA,
  /// The body's contact friction coefficient. One differentiable scalar
  /// (nonzero gradient only in a sliding/frictional contact).
  FRICTION,
};

/// Selects WHICH rigid-body parameter to differentiate, with optional bounds.
///
/// This is a plain value object over the PUBLIC `RigidBody` handle: it carries
/// no ECS/registry or component-storage types, so it is safe to expose across
/// the public facade. The optional bounds are advisory metadata for downstream
/// optimizers (e.g. projected gradient / system-ID solvers) that clamp the
/// parameter to a physically valid range; DART itself does not enforce them
/// when assembling the Jacobian.
struct DART_EXPERIMENTAL_API PhysicalParameterSelector
{
  /// The rigid body whose parameter is differentiated.
  RigidBody body;

  /// Which physical parameter of `body` to differentiate.
  PhysicalParameter parameter = PhysicalParameter::MASS;

  /// Optional inclusive lower bound for the parameter (advisory; not enforced
  /// by DART). For `MASS` this is a lower mass limit, etc.
  std::optional<double> lowerBound;

  /// Optional inclusive upper bound for the parameter (advisory; not enforced
  /// by DART).
  std::optional<double> upperBound;

  PhysicalParameterSelector(
      RigidBody body,
      PhysicalParameter parameter = PhysicalParameter::MASS,
      std::optional<double> lowerBound = std::nullopt,
      std::optional<double> upperBound = std::nullopt)
    : body(body),
      parameter(parameter),
      lowerBound(lowerBound),
      upperBound(upperBound)
  {
    // Nothing else to initialize.
  }
};

} // namespace dart::simulation::experimental
