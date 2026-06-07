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

#include <dart/simulation/comps/component_category.hpp>

#include <Eigen/Core>

#include <limits>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::io {
class SerializerRegistry;
} // namespace dart::simulation::io

namespace dart::simulation::comps {

/// Tag marking an entity as a deformable body.
struct DeformableBodyTag
{
  DART_SIMULATION_TAG_COMPONENT(DeformableBodyTag);
};

/// Internal per-node state for a deformable body.
struct DeformableNodeState
{
  DART_SIMULATION_PROPERTY_COMPONENT(DeformableNodeState);

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
  DART_SIMULATION_PROPERTY_COMPONENT(DeformableSpringModel);

  std::vector<DeformableSpringEdge> edges;
  double stiffness = 100.0;
  double damping = 0.0;
};

/// Internal surface triangle over deformable nodes.
struct DeformableSurfaceTriangle
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;
  std::size_t nodeC = 0;
};

/// Internal tetrahedron over deformable nodes.
struct DeformableTetrahedron
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;
  std::size_t nodeC = 0;
  std::size_t nodeD = 0;
};

/// Internal mesh topology and rest configuration for a deformable body.
struct DeformableMeshTopology
{
  DART_SIMULATION_PROPERTY_COMPONENT(DeformableMeshTopology);

  std::vector<Eigen::Vector3d> restPositions;
  std::vector<DeformableSurfaceTriangle> surfaceTriangles;
  std::vector<DeformableTetrahedron> tetrahedra;
  std::vector<double> tetrahedronRestVolumes;
};

/// Internal material properties for a deformable body.
struct DeformableMaterial
{
  DART_SIMULATION_PROPERTY_COMPONENT(DeformableMaterial);

  double density = 1.0;
  double youngsModulus = 1.0e5;
  double poissonRatio = 0.3;
  double frictionCoefficient = 0.0;
  bool useFiniteElementElasticity = false;
  // When finite-element elasticity is enabled, selects the fixed-corotational
  // material instead of the default stable neo-Hookean kernel. Ignored unless
  // useFiniteElementElasticity is true.
  bool useFixedCorotationalElasticity = false;
  // Opt in to IPC-style adaptive barrier stiffness (kappa) for this body's
  // ground/obstacle contact barriers, scaled per step from the mass/time-step
  // force balance. Off by default keeps the fixed kappa = 25.
  bool useAdaptiveBarrierStiffness = false;
  // Opt in to the iterative (conjugate-gradient) projected-Newton linear solve
  // for this body. The CG solve never factorizes, so its memory stays near
  // O(nnz) and it scales to large meshes; off by default uses the sparse
  // Cholesky direct solve (faster for small/medium meshes). Large meshes above
  // the direct-solve node cap always use CG regardless of this flag.
  bool useIterativeLinearSolver = false;
  // Opt in to the matrix-free CG path. This implies an iterative solve but
  // bypasses sparse Hessian assembly, using block Hessian-vector products and a
  // block-Jacobi preconditioner. Off by default keeps the existing sparse
  // Cholesky / sparse IC-CG behavior unchanged.
  bool useMatrixFreeLinearSolver = false;
};

/// Time-ranged scripted Dirichlet boundary region for deformable nodes.
struct DeformableDirichletBoundary
{
  std::vector<std::size_t> nodes;
  std::vector<Eigen::Vector3d> referencePositions;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
  double startTime = 0.0;
  double endTime = 0.0;
};

/// Time-ranged Neumann-style nodal acceleration region for deformable nodes.
struct DeformableNeumannBoundary
{
  std::vector<std::size_t> nodes;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double startTime = 0.0;
  double endTime = 0.0;
};

/// Internal scripted boundary-condition state for deformable scene replays.
struct DeformableBoundaryConditions
{
  DART_SIMULATION_PROPERTY_COMPONENT(DeformableBoundaryConditions);

  std::vector<DeformableDirichletBoundary> dirichlet;
  std::vector<DeformableNeumannBoundary> neumann;
};

/// Internal opt-in configuration selecting the Vertex Block Descent (VBD) inner
/// solver for a deformable body.
///
/// This component is intentionally not serialized and is not exposed through
/// the public facade: the public deformable stage stays algorithm-neutral, and
/// which inner solver runs is an internal/explicit-opt-in decision rather than
/// a leaked solver registry. When absent or `enabled == false`, the default
/// gradient-descent solver runs.
struct DeformableVbdConfig
{
  bool enabled = false;
  std::size_t iterations = 20;
  /// Stop sweeping early once the largest per-vertex update falls below this
  /// length (0 disables early termination).
  double convergenceDisplacement = 0.0;
  /// Over-relax each sweep with Chebyshev acceleration (same fixed point,
  /// faster convergence).
  bool useChebyshev = false;
  /// Estimated spectral radius in (0, 1) for the Chebyshev recurrence.
  double chebyshevRho = 0.95;
  /// Stiffness-proportional (Rayleigh) damping coefficient k_d (0 disables it).
  double rayleighDamping = 0.0;
  /// Worker threads for the colored sweep (1 = serial). With more than one
  /// thread the deterministic multithreaded driver runs, which does not apply
  /// Chebyshev over-relaxation or residual early termination (those run only on
  /// the serial path).
  unsigned int workerThreads = 1;
  /// Penalty stiffness k_c for static ground/obstacle half-space contact
  /// (0 disables ground contact, so the body falls freely past barriers). With
  /// a positive value the VBD solve keeps the body resting on the ground
  /// barrier set instead of routing the body to the default solver.
  double contactStiffness = 0.0;
  /// Internal AVBD slice flag: use warm-started augmented-Lagrangian
  /// contact-normal rows for static half-space contact when the current scene
  /// is inside the supported CPU mass-spring envelope. Unsupported cases fall
  /// back to the existing VBD penalty-contact path.
  bool useAvbdContactNormalRows = false;
  /// Internal AVBD slice flag: use warm-started augmented-Lagrangian
  /// self-contact normal rows for lagged point-triangle / edge-edge candidates
  /// when the current scene is inside the supported CPU mass-spring envelope.
  /// With positive material friction, matching tangent rows are generated for
  /// those same primitives. Unsupported cases fall back to the existing VBD
  /// self-contact penalty path.
  bool useAvbdSelfContactNormalRows = false;
  /// Internal AVBD slice flag: replace exact fixed-node skipping with
  /// warm-started hard point-attachment rows for pinned/scripted nodes when the
  /// current scene is inside the supported CPU mass-spring envelope.
  bool useAvbdAttachmentRows = false;
  /// Starting stiffness for AVBD hard point-attachment rows.
  double avbdAttachmentStiffness = 1000.0;
  /// Internal AVBD slice flag: use progressive finite-stiffness rows for
  /// deformable springs in the supported CPU mass-spring envelope.
  bool useAvbdFiniteStiffnessRows = false;
  /// Starting stiffness for AVBD finite-stiffness deformable rows. Spring rows
  /// use this in material stiffness units; tetrahedral material rows use it as
  /// a dimensionless multiplier on the Lamé parameters and cap at 1.0 unless
  /// avbdMaxStiffness is lower.
  double avbdFiniteStiffnessStart = 1.0;
  /// AVBD alpha regularization for pre-existing contact error.
  double avbdAlpha = 0.99;
  /// AVBD stiffness-growth coefficient for active rows.
  double avbdBeta = 1000.0;
  /// AVBD warm-start decay for active lambda/stiffness state.
  double avbdGamma = 0.99;
  /// Hard cap for AVBD row stiffness.
  double avbdMaxStiffness = std::numeric_limits<double>::infinity();
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
  std::vector<Eigen::Vector3d> previousStepPositions;
  std::vector<Eigen::Vector3d> externalAccelerations;
  std::vector<std::uint8_t> activeFixed;
  std::vector<std::uint8_t> activeDirichlet;
};

void registerDeformableBodySerializers(
    dart::simulation::io::SerializerRegistry& registry);

} // namespace dart::simulation::comps
