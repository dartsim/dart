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
 *     copyright notice, this list of conditions and/or other materials
 *     provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/experimental/body/deformable_body_options.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>

#include <string>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::experimental {

/// Public handle for an experimental deformable body.
///
/// The handle intentionally exposes node state and spring topology without
/// exposing the internal ECS entity, registry, or solver implementation.
class DART_EXPERIMENTAL_API DeformableBody
{
public:
  DeformableBody() = default;

  [[nodiscard]] bool isValid() const noexcept;
  [[nodiscard]] std::string getName() const;

  [[nodiscard]] std::size_t getNodeCount() const;
  [[nodiscard]] Eigen::Vector3d getPosition(std::size_t node) const;
  void setPosition(std::size_t node, const Eigen::Vector3d& position);
  [[nodiscard]] Eigen::Vector3d getVelocity(std::size_t node) const;
  void setVelocity(std::size_t node, const Eigen::Vector3d& velocity);
  [[nodiscard]] double getMass(std::size_t node) const;
  [[nodiscard]] bool isFixedNode(std::size_t node) const;

  [[nodiscard]] std::size_t getEdgeCount() const;
  [[nodiscard]] DeformableEdge getEdge(std::size_t edge) const;

private:
  friend class World;

  DeformableBody(std::uint32_t entityId, World* world);

  std::uint32_t m_entityId{0xffffffffu};
  World* m_world{nullptr};
};

} // namespace dart::simulation::experimental
