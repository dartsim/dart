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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
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

#include <dart/simulation/experimental/comps/component_category.hpp>

#include <Eigen/Core>

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::experimental::comps {

/// Tag marking an entity as a deformable body.
struct DeformableBodyTag
{
  DART_EXPERIMENTAL_TAG_COMPONENT(DeformableBodyTag);
};

/// Internal per-node state for a deformable body.
struct DeformableNodeState
{
  DART_EXPERIMENTAL_PROPERTY_COMPONENT(DeformableNodeState);

  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> previousPositions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
};

/// Internal spring edge connecting two deformable nodes.
struct DeformableSpringEdge
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;
  double restLength = 0.0;
};

/// Internal spring model for a deformable body.
struct DeformableSpringModel
{
  DART_EXPERIMENTAL_PROPERTY_COMPONENT(DeformableSpringModel);

  std::vector<DeformableSpringEdge> edges;
  double stiffness = 100.0;
  double damping = 0.0;
};

/// Transient scratch buffers reused by the default deformable solver.
///
/// These buffers are intentionally not serialized; they are resized lazily from
/// the live node state after loading or model edits.
struct DeformableSolverScratch
{
  std::vector<Eigen::Vector3d> inertialTargets;
  std::vector<Eigen::Vector3d> next;
  std::vector<Eigen::Vector3d> gradient;
  std::vector<Eigen::Vector3d> direction;
  std::vector<Eigen::Vector3d> candidate;
};

} // namespace dart::simulation::experimental::comps
