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

#include <Eigen/Core>

#include <vector>

#include <cstddef>

namespace dart::simulation::experimental {

/// Edge connecting two deformable point-mass nodes.
struct DeformableEdge
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;

  /// Rest length for the edge spring. Values <= 0 ask World::addDeformableBody
  /// to compute the rest length from the initial node positions.
  double restLength = -1.0;
};

/// Options for creating a DeformableBody.
///
/// This first experimental model is intentionally narrow: it represents a set
/// of point-mass nodes joined by distance springs. Contact and barrier tuning
/// are solver internals owned by the World step pipeline, not public body
/// options.
struct DeformableBodyOptions
{
  /// Initial world-space node positions. Must be non-empty and finite.
  std::vector<Eigen::Vector3d> positions;

  /// Initial world-space node velocities. If empty, all velocities are zero.
  /// Otherwise the size must match positions and all values must be finite.
  std::vector<Eigen::Vector3d> velocities;

  /// Per-node masses. If empty, all masses are one. Otherwise the size must
  /// match positions and all masses must be positive and finite.
  std::vector<double> masses;

  /// Distance-spring edges between nodes.
  std::vector<DeformableEdge> edges;

  /// Node indices eliminated from the solve and held fixed in world space.
  std::vector<std::size_t> fixedNodes;

  /// Distance-spring stiffness. Must be finite and non-negative.
  double edgeStiffness = 100.0;

  /// Simple velocity damping coefficient. Must be finite and non-negative.
  double damping = 0.0;
};

} // namespace dart::simulation::experimental
