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

namespace dart::simulation {

/// Kinematics-stage participation for an explicit loop closure.
///
/// The values describe the public algorithmic intent. They do not expose the
/// internal projection method, solver rows, or backend implementation.
enum class ClosureKinematicsPolicy
{
  /// Track closure topology and allow residual diagnostics without projection.
  ResidualOnly,

  /// Project kinematic state toward the closure relation when the active
  /// pipeline includes a compatible projection stage.
  Project,
};

/// Dynamics-stage participation for an explicit loop closure.
///
/// The values describe runtime intent independently from the constraint solver
/// implementation chosen by a future dynamics pipeline.
enum class ClosureDynamicsPolicy
{
  /// Track closure topology and allow residual diagnostics without solving.
  ResidualOnly,

  /// Solve the closure relation when the active dynamics pipeline supports it.
  Solve,
};

/// Runtime participation policy for a loop closure.
///
/// LoopClosureSpec owns topology. LoopClosureRuntimePolicy owns whether that
/// topology is active for residual reporting, kinematic projection, or dynamic
/// solving. The current DART 7 implementation stores this policy as public
/// metadata; projection and solving stages are staged separately.
struct LoopClosureRuntimePolicy
{
  bool enabled = true;
  ClosureKinematicsPolicy kinematics = ClosureKinematicsPolicy::ResidualOnly;
  ClosureDynamicsPolicy dynamics = ClosureDynamicsPolicy::ResidualOnly;
};

} // namespace dart::simulation
