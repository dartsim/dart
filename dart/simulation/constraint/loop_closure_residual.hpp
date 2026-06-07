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

/// Coordinate convention for loop-closure residual values.
enum class LoopClosureResidualCoordinates
{
  /// Linear residuals and rotation vectors are expressed in world coordinates.
  World,
};

/// Residual diagnostic for a loop closure at the current world state.
///
/// This value reports public residual diagnostics only. It does not expose
/// constraint rows, solver identifiers, backend storage, or force/impulse
/// estimates from a particular implementation.
struct DART_SIMULATION_API LoopClosureResidual
{
  /// Residual value for the closure family.
  ///
  /// Rigid closures use [linear_x, linear_y, linear_z, angular_x, angular_y,
  /// angular_z]. Point closures use the three linear coordinates. Distance
  /// closures use a single non-negative scalar distance.
  Eigen::VectorXd value;

  /// Euclidean norm of value.
  double norm = 0.0;

  /// Whether the closure is enabled for runtime participation.
  bool enabled = true;

  /// True when this closure is enabled and can participate in the active
  /// residual/projection/solve pipeline.
  bool active = true;

  /// Coordinate convention used by value.
  LoopClosureResidualCoordinates coordinates
      = LoopClosureResidualCoordinates::World;

  /// Current DART 7 diagnostics do not expose solved force or impulse data.
  bool forceAvailable = false;
};

} // namespace dart::simulation
