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

#include <dart/simulation/experimental/constraint/loop_closure_family.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_residual.hpp>
#include <dart/simulation/experimental/constraint/loop_closure_runtime_policy.hpp>
#include <dart/simulation/experimental/entity.hpp>

#include <Eigen/Geometry>

#include <string_view>

namespace dart::simulation::experimental {

/// World-owned handle for a closed-chain topology relation.
///
/// The DART 7 experimental API stores loop closures as named topology objects
/// that can later participate in kinematic projection, residual diagnostics, or
/// dynamic solving. This handle exposes the stable semantic relation without
/// exposing solver rows or ECS storage.
class DART_EXPERIMENTAL_API LoopClosure
{
public:
  LoopClosure(Entity entity, World* world);

  [[nodiscard]] std::string_view getName() const;
  [[nodiscard]] LoopClosureFamily getFamily() const;
  [[nodiscard]] Frame getFrameA() const;
  [[nodiscard]] Frame getFrameB() const;
  [[nodiscard]] const Eigen::Isometry3d& getOffsetA() const;
  [[nodiscard]] const Eigen::Isometry3d& getOffsetB() const;
  [[nodiscard]] LoopClosureRuntimePolicy getRuntimePolicy() const;
  void setRuntimePolicy(const LoopClosureRuntimePolicy& policy);
  [[nodiscard]] LoopClosureResidual computeResidual() const;

  /// Get the opaque entity token.
  ///
  /// Internal code needing the raw ECS handle should call
  /// `detail::toRegistryEntity(closure.getEntity())`.
  [[nodiscard]] Entity getEntity() const;
  [[nodiscard]] World* getWorld() const;
  [[nodiscard]] bool isValid() const;

private:
  Entity m_entity; ///< Opaque entity token
  World* m_world;
};

} // namespace dart::simulation::experimental
