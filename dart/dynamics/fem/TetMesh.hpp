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

#ifndef DART_DYNAMICS_FEM_TETMESH_HPP_
#define DART_DYNAMICS_FEM_TETMESH_HPP_

#include <dart/common/Memory.hpp>

#include <Eigen/Dense>

#include <vector>

#include <cstddef>

namespace dart {
namespace dynamics {
namespace fem {

/// Tetrahedral mesh with cached undeformed (rest) quantities.
///
/// The node ordering of every tetrahedron must be positively oriented, that is
/// det([x1 - x0, x2 - x0, x3 - x0]) must be greater than zero. A mesh holding a
/// degenerate or inverted tetrahedron is rejected instead of being silently
/// reordered, so an inverted input fails loudly rather than producing negative
/// element volumes and negative lumped masses downstream.
class TetMesh
{
public:
  /// Creates an empty mesh.
  TetMesh() = default;

  /// Creates a mesh from node rest positions and tetrahedron node indices.
  ///
  /// Throws std::invalid_argument when a node index is out of range or a
  /// tetrahedron has a non-positive rest volume.
  TetMesh(
      std::vector<Eigen::Vector3d> restPositions,
      common::aligned_vector<Eigen::Vector4i> tets);

  /// Creates a box centered on the local origin, subdivided into a regular grid
  /// of \c divisions cells, each split into six positively oriented
  /// tetrahedra. The tetrahedra tile the box exactly, so the total rest volume
  /// equals the product of the box dimensions.
  static TetMesh createBox(
      const Eigen::Vector3d& size, const Eigen::Vector3i& divisions);

  std::size_t getNumNodes() const;

  std::size_t getNumTets() const;

  bool isEmpty() const;

  const Eigen::Vector3d& getRestPosition(std::size_t nodeIndex) const;

  /// Returns the four node indices of one tetrahedron.
  Eigen::Vector4i getTet(std::size_t tetIndex) const;

  /// Returns the undeformed volume of one tetrahedron. Always positive.
  double getRestVolume(std::size_t tetIndex) const;

  /// Returns [x1 - x0, x2 - x0, x3 - x0]^-1 for one tetrahedron, the inverse
  /// undeformed shape matrix. The elastic-force work uses it to build
  /// deformation gradients as F = Ds * getInverseRestShape().
  const Eigen::Matrix3d& getInverseRestShape(std::size_t tetIndex) const;

  /// Returns the sum of every tetrahedron rest volume.
  double getTotalRestVolume() const;

private:
  /// Validates the topology and fills the cached rest quantities.
  void computeRestQuantities();

  std::vector<Eigen::Vector3d> mRestPositions;

  common::aligned_vector<Eigen::Vector4i> mTets;

  std::vector<double> mRestVolumes;

  std::vector<Eigen::Matrix3d> mInverseRestShapes;

  double mTotalRestVolume = 0.0;
};

} // namespace fem
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_FEM_TETMESH_HPP_
