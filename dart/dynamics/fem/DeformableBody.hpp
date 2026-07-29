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

/// Constitutive model used to turn element deformation into elastic force.
enum class ElasticModel
{
  /// Small-strain linear elasticity. It measures strain directly from the
  /// deformation gradient, so a large rigid rotation registers as strain and
  /// produces spurious restoring forces. Useful for small deformation and as a
  /// reference, but not for characters that rotate.
  Linear,

  /// Co-rotated linear elasticity. The element's rotation is factored out
  /// before strain is measured, so a rigid rotation produces no force at all
  /// and large deformation stays well behaved. This is the default.
  Corotational,
};

/// Material parameters of a volumetric FEM body.
struct Material
{
  /// Mass per unit undeformed volume, in kg/m^3.
  double mDensity = 1000.0;

  /// Young's modulus, in Pa. Larger values make the body stiffer, and also
  /// tighten the largest stable explicit time step.
  double mYoungsModulus = 1.0e5;

  /// Poisson's ratio, which must lie in (-1, 0.5). It approaches
  /// incompressibility as it approaches 0.5, where the Lame parameter lambda
  /// diverges.
  double mPoissonRatio = 0.3;

  /// Constitutive model used for elastic element forces.
  ElasticModel mElasticModel = ElasticModel::Corotational;

  /// Mass-proportional viscous damping rate, in 1/s.
  ///
  /// The damping force on a node is -mDensity-weighted mass times this rate
  /// times velocity, so every node decays by the same factor and a body
  /// translating uniformly stays undeformed. It is integrated implicitly and is
  /// therefore stable for any non-negative rate and time step.
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

  /// Recomputes the elastic force on every node from the current node
  /// positions. Stepping the world does this automatically; call it directly to
  /// inspect forces in a configuration that has not been simulated.
  void computeElasticForces();

  /// Returns the elastic force on one node as of the last computation.
  Eigen::Vector3d getNodeForce(std::size_t nodeIndex) const;

  /// Returns the elastic potential energy stored in the current configuration.
  /// It is zero at rest and positive for any deformation.
  double getElasticEnergy() const;

  /// Holds a node at its current position, or releases it.
  ///
  /// A fixed node keeps zero velocity and ignores every force, which is how a
  /// body is anchored to the world without a coupling constraint.
  void setNodeFixed(std::size_t nodeIndex, bool fixed);

  bool isNodeFixed(std::size_t nodeIndex) const;

  /// Embeds a surface in the volume mesh.
  ///
  /// Each surface vertex is bound to the tetrahedron that contains it using
  /// barycentric weights, so the surface follows the volume as it deforms. A
  /// vertex that lies outside every tetrahedron is bound to the tetrahedron it
  /// violates least in barycentric terms and extrapolates from it. That measure
  /// is normalized by each tetrahedron's own dimensions rather than being a
  /// distance, so for a vertex well outside the mesh the chosen tetrahedron is
  /// not necessarily the nearest one and the extrapolation error is unbounded.
  void setEmbeddedSurface(const std::vector<Eigen::Vector3d>& restVertices);

  std::size_t getNumSurfaceVertices() const;

  /// Returns the current position of one embedded surface vertex, interpolated
  /// from the current node positions.
  Eigen::Vector3d getSurfaceVertexPosition(std::size_t vertexIndex) const;

  /// Returns the tetrahedron one embedded surface vertex is bound to.
  std::size_t getSurfaceVertexTet(std::size_t vertexIndex) const;

  /// Returns the barycentric weights binding one embedded surface vertex to its
  /// tetrahedron. Weights within [0, 1] mean the vertex lies inside that
  /// tetrahedron; a weight outside that range means it extrapolates.
  Eigen::Vector4d getSurfaceVertexWeights(std::size_t vertexIndex) const;

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
