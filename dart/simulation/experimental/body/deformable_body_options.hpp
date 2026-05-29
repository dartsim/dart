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

#include <limits>
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

/// Surface triangle over deformable body nodes.
struct DeformableSurfaceTriangle
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;
  std::size_t nodeC = 0;
};

/// Volumetric tetrahedron over deformable body nodes.
struct DeformableTetrahedron
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;
  std::size_t nodeC = 0;
  std::size_t nodeD = 0;
};

/// Material properties stored with a deformable body.
///
/// This mesh-state slice validates and serializes all fields, but only density
/// is used today, and only to assemble default lumped masses for tetrahedral
/// bodies when explicit masses are omitted. Elastic material models are added
/// in later PLAN-081 slices.
struct DeformableMaterialProperties
{
  /// Volumetric density used for default tetrahedral mass assembly.
  double density = 1.0;

  /// Young's modulus reserved for future elastic material models.
  double youngsModulus = 1.0e5;

  /// Poisson ratio reserved for future elastic material models.
  double poissonRatio = 0.3;

  /// Coulomb friction coefficient for contact against static ground barriers.
  /// Must be finite and non-negative. Zero (the default) disables friction, so
  /// existing contact behavior is unchanged unless friction is opted in.
  double frictionCoefficient = 0.0;
};

/// Scripted Dirichlet boundary condition over deformable nodes.
///
/// Active nodes are removed from the deformable solve while the time range is
/// active and follow a restartable linearized rigid motion around `center`.
/// This is boundary-control scaffolding for scene replay, not a public solver
/// callback mechanism.
struct DeformableDirichletBoundaryCondition
{
  /// Node indices controlled by this boundary condition.
  std::vector<std::size_t> nodes;

  /// Translation velocity in world units per second.
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();

  /// Angular velocity in radians per second.
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();

  /// World-space center used for angular velocity.
  Eigen::Vector3d center = Eigen::Vector3d::Zero();

  /// Inclusive start time in seconds.
  double startTime = 0.0;

  /// Exclusive end time in seconds. Infinity means no scheduled stop.
  double endTime = std::numeric_limits<double>::infinity();
};

/// Time-ranged Neumann-style nodal acceleration over deformable nodes.
///
/// IPC scene `NBC` records encode acceleration components (`ax ay az`). This
/// contact-free replay slice applies the vector directly as per-node external
/// acceleration. Traction work terms are added with later FEM material slices.
struct DeformableNeumannBoundaryCondition
{
  /// Node indices receiving the acceleration.
  std::vector<std::size_t> nodes;

  /// Nodal acceleration applied while the time range is active.
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

  /// Inclusive start time in seconds.
  double startTime = 0.0;

  /// Exclusive end time in seconds. Infinity means no scheduled stop.
  double endTime = std::numeric_limits<double>::infinity();
};

/// Options for creating a DeformableBody.
///
/// This experimental model currently steps point-mass nodes joined by distance
/// springs. Optional surface/tetrahedral topology and material properties are
/// stored as mesh-state scaffolding for later deformable solvers; they do not
/// enable FEM elasticity, mesh contact, CCD, projected Newton, or friction.
/// Contact and barrier tuning are solver internals owned by the World step
/// pipeline, not public body options.
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

  /// Optional surface topology for rendering and diagnostics.
  ///
  /// If empty and tetrahedra are provided, the boundary surface is derived from
  /// the tetrahedral topology. Surface-only bodies must provide explicit node
  /// masses until a shell mass model is introduced.
  std::vector<DeformableSurfaceTriangle> surfaceTriangles;

  /// Optional volumetric tetrahedral topology.
  ///
  /// When masses are omitted and tetrahedra are present, node masses are
  /// assembled as density * tetrahedron_volume / 4 per incident node.
  std::vector<DeformableTetrahedron> tetrahedra;

  /// Node indices eliminated from the solve and held fixed in world space.
  std::vector<std::size_t> fixedNodes;

  /// Optional scripted Dirichlet boundary conditions.
  std::vector<DeformableDirichletBoundaryCondition> dirichletBoundaryConditions;

  /// Optional time-ranged Neumann-style nodal accelerations.
  std::vector<DeformableNeumannBoundaryCondition> neumannBoundaryConditions;

  /// Material properties stored with the body. Only density affects this slice.
  DeformableMaterialProperties material;

  /// Distance-spring stiffness. Must be finite and non-negative.
  double edgeStiffness = 100.0;

  /// Simple velocity damping coefficient. Must be finite and non-negative.
  double damping = 0.0;
};

} // namespace dart::simulation::experimental
