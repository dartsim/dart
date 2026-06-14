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
#include <dart/simulation/constraint/loop_closure_family.hpp>
#include <dart/simulation/constraint/loop_closure_runtime_policy.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

namespace dart::simulation::comps {

/// Component storing an explicit loop-closure topology relation.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct LoopClosure
{
  DART_SIMULATION_STATE_COMPONENT(LoopClosure, "comps.LoopClosure");

  dart::simulation::LoopClosureFamily family
      = dart::simulation::LoopClosureFamily::Rigid;
  entt::entity frameA = entt::null;
  entt::entity frameB = entt::null;
  Eigen::Isometry3d offsetA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d offsetB = Eigen::Isometry3d::Identity();
  dart::simulation::LoopClosureRuntimePolicy runtimePolicy;

  /// Target separation for the `Distance` family (0 = coincident). Ignored by
  /// `Point`/`Rigid`.
  double distance = 0.0;

  static constexpr auto entityFields()
  {
    return std::tuple{&LoopClosure::frameA, &LoopClosure::frameB};
  }
};

} // namespace dart::simulation::comps
