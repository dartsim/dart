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

#ifndef DART_DYNAMICS_FEM_DEFORMABLEBODY_HPP_
#define DART_DYNAMICS_FEM_DEFORMABLEBODY_HPP_

#include <dart/dynamics/fem/TetMesh.hpp>

#include <Eigen/Dense>

#include <memory>
#include <vector>

#include <cstddef>

namespace dart {

namespace simulation {
class World;
} // namespace simulation

namespace dynamics {
namespace fem {

namespace detail {
class DeformableBodyImpl;
} // namespace detail

/// Material parameters of a volumetric FEM body.
///
/// Young's modulus and Poisson's ratio are carried but not yet consumed. The
/// elastic element forces that use them arrive with the constitutive-model
/// work; density and linear damping are already in use.
struct Material
{
  /// Mass per unit undeformed volume, in kg/m^3.
  double mDensity = 1000.0;

  /// Young's modulus, in Pa. Reserved for the elastic-force work.
  double mYoungsModulus = 1.0e5;

  /// Poisson's ratio. Reserved for the elastic-force work.
  double mPoissonRatio = 0.3;

  /// Viscous damping applied to each node, in N*s/m.
  double mLinearDamping = 0.0;
};

/// A volumetric deformable body driven by a tetrahedral mesh.
///
/// The body owns its node state and advances it once per simulation step. It
/// reaches the step loop through a constraint that the constraint solver
/// updates every solve but that never joins the LCP, so attaching a body adds
/// no cost to worlds that have none and does not perturb rigid-body results.
///
/// This first increment integrates gravity and linear damping only. Elastic
/// element forces, reduced coordinates, skeleton coupling, and collision are
/// separate follow-up increments.
class DeformableBody
{
public:
  /// Creates a body from a mesh and a material.
  ///
  /// Throws std::invalid_argument when the mesh is empty.
  static std::shared_ptr<DeformableBody> create(
      const TetMesh& mesh, const Material& material = Material());

  /// Detaches the body from its world, if attached, and destroys it.
  ~DeformableBody();

  DeformableBody(const DeformableBody&) = delete;
  DeformableBody& operator=(const DeformableBody&) = delete;

  const TetMesh& getMesh() const;

  const Material& getMaterial() const;

  std::size_t getNumNodes() const;

  Eigen::Vector3d getNodePosition(std::size_t nodeIndex) const;

  void setNodePosition(std::size_t nodeIndex, const Eigen::Vector3d& position);

  Eigen::Vector3d getNodeVelocity(std::size_t nodeIndex) const;

  void setNodeVelocity(std::size_t nodeIndex, const Eigen::Vector3d& velocity);

  /// Returns the lumped mass of one node. Each tetrahedron contributes a
  /// quarter of its own mass to each of its four nodes.
  double getNodeMass(std::size_t nodeIndex) const;

  /// Returns the sum of every lumped node mass, which equals the material
  /// density times the total rest volume of the mesh.
  double getTotalMass() const;

  /// Moves every node back to its rest position with zero velocity.
  void resetToRest();

  /// Embeds a surface in the volume mesh.
  ///
  /// Each surface vertex is bound to the tetrahedron that contains it using
  /// barycentric weights, so the surface follows the volume as it deforms. A
  /// vertex that lies outside every tetrahedron is bound to the closest one and
  /// extrapolates from it.
  void setEmbeddedSurface(const std::vector<Eigen::Vector3d>& restVertices);

  std::size_t getNumSurfaceVertices() const;

  /// Returns the current position of one embedded surface vertex, interpolated
  /// from the current node positions.
  Eigen::Vector3d getSurfaceVertexPosition(std::size_t vertexIndex) const;

  /// Attaches the body to a world so that it advances with every world step.
  /// Attaching to a new world detaches from the previous one.
  ///
  /// Throws std::invalid_argument when the world is null or has no constraint
  /// solver.
  void attachTo(const std::shared_ptr<simulation::World>& world);

  /// Stops the body from advancing and releases it from its world.
  void detach();

  bool isAttached() const;

private:
  DeformableBody(const TetMesh& mesh, const Material& material);

  std::unique_ptr<detail::DeformableBodyImpl> mImpl;
};

using DeformableBodyPtr = std::shared_ptr<DeformableBody>;

} // namespace fem
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_FEM_DEFORMABLEBODY_HPP_
