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

#include <dart/simulation/comps/component_category.hpp>
#include <dart/simulation/comps/dynamics.hpp>

#include <limits>
#include <optional>

#include <cstddef>

namespace dart::simulation::comps {

/// Tag marking entity as a RigidBody
///
/// Automatically serialized via DART_SIMULATION_TAG_COMPONENT macro.
/// **Internal Implementation Detail** - Not exposed in public API
struct RigidBodyTag
{
  DART_SIMULATION_TAG_COMPONENT(RigidBodyTag, "comps.RigidBodyTag");
};

/// Tag marking a rigid body as static (immovable): no gravity, no integration,
/// and treated as infinite mass by the contact solver.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct StaticBodyTag
{
  DART_SIMULATION_TAG_COMPONENT(StaticBodyTag, "comps.StaticBodyTag");
};

/// Tag marking a rigid body as kinematic (prescribed motion): the rigid IPC
/// contact stage advances it by its prescribed (linear/angular) velocity each
/// step and treats it as a moving obstacle -- contacting dynamic bodies cannot
/// penetrate it and are dragged by its surface friction -- but it receives no
/// contact or dynamics degrees of freedom and is unaffected by gravity or
/// collision response. Runtime-only configuration; intentionally not
/// serialized.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct KinematicBodyTag
{
  /// Optional world time at which fixture-replay kinematic motion stops.
  std::optional<double> maxTime;
};

/// Runtime-only trace of a kinematic body's realized motion during the current
/// world step. Rigid IPC writes this when it advances a kinematic body before
/// deformable dynamics so the deformable CCD stage can limit against the swept
/// current-step motion instead of only the final pose.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct KinematicBodyStepTrace
{
  std::size_t frame = 0u;
  Transform startTransform;
  Transform endTransform;
};

/// Internal opt-in configuration for the first AVBD rigid contact World slice.
///
/// This component is intentionally not surfaced through the public `World`
/// facade. When every rigid contact in the contact stage has at least one body
/// with an enabled config, supported free rigid-body contacts route through the
/// private 6-DOF AVBD point-pair row projection. Unsupported envelopes fall
/// back to the default sequential-impulse path.
struct RigidAvbdContactConfig
{
  bool enabled = true;
  double startStiffness = 1e5;
  double alpha = 0.0;
  double beta = 1000.0;
  double gamma = 0.99;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

} // namespace dart::simulation::comps
