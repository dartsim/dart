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

#include "dart/simulation/experimental/world.hpp"

#include "dart/collision/native/collision_object.hpp"
#include "dart/collision/native/collision_world.hpp"
#include "dart/collision/native/contact_manifold.hpp"
#include "dart/collision/native/contact_point.hpp"
#include "dart/collision/native/shapes/shape.hpp"
#include "dart/collision/native/types.hpp"
#include "dart/simulation/experimental/body/contact.hpp"
#include "dart/simulation/experimental/body/deformable_body.hpp"
#include "dart/simulation/experimental/body/rigid_body.hpp"
#include "dart/simulation/experimental/common/ecs_utils.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/compute/multibody_dynamics.hpp"
#include "dart/simulation/experimental/compute/sequential_executor.hpp"
#include "dart/simulation/experimental/compute/variational_integration.hpp"
#include "dart/simulation/experimental/compute/world_kinematics_graph.hpp"
#include "dart/simulation/experimental/compute/world_step_stage.hpp"
#include "dart/simulation/experimental/constraint/loop_closure.hpp"
#include "dart/simulation/experimental/constraint/loop_closure_spec.hpp"
#include "dart/simulation/experimental/detail/deformable_vbd/rigid_world_contact.hpp"
#include "dart/simulation/experimental/detail/entity_conversion.hpp"
#include "dart/simulation/experimental/detail/world_registry_access.hpp"
#include "dart/simulation/experimental/detail/world_storage.hpp"
#include "dart/simulation/experimental/diff/physical_parameter.hpp"
#include "dart/simulation/experimental/diff/step_derivatives.hpp"
#include "dart/simulation/experimental/diff/step_gradient.hpp"
#include "dart/simulation/experimental/frame/fixed_frame.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/frame/free_frame.hpp"
#include "dart/simulation/experimental/io/binary_io.hpp"
#include "dart/simulation/experimental/io/serializer.hpp"
#include "dart/simulation/experimental/multibody/joint.hpp"
#include "dart/simulation/experimental/multibody/multibody.hpp"
#include "dart/simulation/experimental/world_options.hpp"

#ifdef DART_HAS_DIFF
  #include "dart/simulation/experimental/detail/contact_jacobians.hpp"
  #include "dart/simulation/experimental/detail/smooth_jacobians.hpp"
#endif

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <format>
#include <istream>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace {

template <typename... Components>
std::size_t countEntities(const entt::registry& registry)
{
  std::size_t count = 0;
  auto view = registry.view<Components...>();
  for (auto entity : view) {
    (void)entity;
    ++count;
  }
  return count;
}

template <typename Component>
bool hasEntityWithName(const entt::registry& registry, std::string_view name)
{
  auto view
      = registry.view<Component, dart::simulation::experimental::comps::Name>();
  for (auto entity : view) {
    const auto& info
        = view.template get<dart::simulation::experimental::comps::Name>(
            entity);
    if (info.name == name) {
      return true;
    }
  }
  return false;
}

} // namespace

namespace dart::simulation::experimental {

namespace ncol = dart::collision::native;

struct World::CollisionQueryCache
{
  struct Key
  {
    entt::entity entity;
    std::size_t shapeIndex;
    std::uint64_t geometryRevision;
    entt::entity multibody;
    bool isLink;

    bool operator==(const Key&) const = default;
  };

  struct ObjectEntry
  {
    entt::entity entity;
    entt::entity multibody;
    bool isLink;
    ncol::CollisionObject object;
  };

  void clear()
  {
    collisionWorld.clear();
    keys.clear();
    entries.clear();
    entryByObjectId.clear();
  }

  ncol::CollisionWorld collisionWorld;
  std::vector<Key> keys;
  std::vector<ObjectEntry> entries;
  std::vector<std::size_t> entryByObjectId;
};

namespace {

//==============================================================================
// Folds the full internal solver stats into the curated public diagnostics.
DeformableSolverDiagnostics makeDeformableSolverDiagnostics(
    const compute::DeformableSolverStats& stats)
{
  DeformableSolverDiagnostics diagnostics;
  diagnostics.bodyCount = stats.bodyCount;
  diagnostics.nodeCount = stats.nodeCount;
  diagnostics.edgeCount = stats.edgeCount;
  diagnostics.solverIterations = stats.solverIterations;
  diagnostics.objectiveEvaluations = stats.objectiveEvaluations;
  diagnostics.lineSearchTrials = stats.lineSearchTrials;
  diagnostics.projectedNewtonSteps = stats.projectedNewtonSteps;
  diagnostics.projectedNewtonFallbacks = stats.projectedNewtonFallbacks;
  diagnostics.projectedNewtonHessianNonZeros
      = stats.projectedNewtonHessianNonZeros;
  diagnostics.projectedNewtonHessianStorageBytes
      = stats.projectedNewtonHessianStorageBytes;
  diagnostics.projectedNewtonIterativeSolves
      = stats.projectedNewtonIterativeSolves;
  diagnostics.projectedNewtonMatrixFreeSolves
      = stats.projectedNewtonMatrixFreeSolves;
  diagnostics.projectedNewtonIterativeIterations
      = stats.projectedNewtonIterativeIterations;
  diagnostics.projectedNewtonIterativeMaxError
      = stats.projectedNewtonIterativeMaxError;
  diagnostics.selfContactBarrierActiveContacts
      = stats.selfContactBarrierActiveContacts;
  diagnostics.frictionDissipation = stats.frictionDissipation;
  diagnostics.minActiveContactDistance = stats.minActiveContactDistance;
  diagnostics.convergedActiveContactCount = stats.convergedActiveContactCount;
  return diagnostics;
}

//==============================================================================
void executeKinematicsGraph(World& world, compute::ComputeExecutor& executor)
{
  compute::WorldKinematicsGraph graph(world);
  graph.execute(executor);
}

//==============================================================================
bool isValidWorldSyncStage(WorldSyncStage stage)
{
  switch (stage) {
    case WorldSyncStage::Kinematics:
      return true;
  }

  return false;
}

//==============================================================================
bool isValidRigidBodySolver(RigidBodySolver solver)
{
  switch (solver) {
    case RigidBodySolver::SequentialImpulse:
    case RigidBodySolver::Ipc:
      return true;
  }

  return false;
}

//==============================================================================
bool hasMultibodyStructures(const World& world)
{
  const auto view = detail::registryOf(world).view<comps::MultibodyStructure>();
  return view.begin() != view.end();
}

//==============================================================================
bool isRigidBodyFixedJoint(
    const entt::registry& registry, const comps::Joint& joint)
{
  if (joint.type != comps::JointType::Fixed || joint.parentLink == entt::null
      || joint.childLink == entt::null || joint.parentLink == joint.childLink) {
    return false;
  }

  return registry.all_of<comps::RigidBodyTag>(joint.parentLink)
         && registry.all_of<comps::RigidBodyTag>(joint.childLink);
}

//==============================================================================
bool hasRigidBodyFixedJoints(const World& world)
{
  const auto& registry = detail::registryOf(world);
  const auto view = registry.view<comps::Joint>();
  for (auto entity : view) {
    (void)entity;
    const auto& joint = view.get<comps::Joint>(entity);
    if (isRigidBodyFixedJoint(registry, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
void validateRigidBodyFixedJointPipelineSupport(
    const World& world, RigidBodySolver solver)
{
  if (!hasRigidBodyFixedJoints(world)) {
    return;
  }

  if (solver == RigidBodySolver::Ipc) {
    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException,
        "Rigid-body fixed joints are not supported by the IPC rigid-body "
        "solver");
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      hasMultibodyStructures(world),
      InvalidOperationException,
      "Rigid-body fixed joints are not supported in worlds with multibody "
      "structures");
}

//==============================================================================
void validateLoopClosureKinematicsPolicySupport(const World& world)
{
  auto view = detail::registryOf(world).view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& closure = view.get<comps::LoopClosure>(entity);
    if (!closure.runtimePolicy.enabled
        || closure.runtimePolicy.kinematics
               == ClosureKinematicsPolicy::ResidualOnly) {
      continue;
    }

    const auto& name = view.get<comps::Name>(entity);
    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException,
        "LoopClosure '{}' requests kinematic projection, but the active "
        "pipeline does not include a loop-closure projection stage",
        name.name);
  }
}

//==============================================================================
// `variationalSelected` is passed in (an enum comparison the caller already
// has) rather than re-derived from a string here, so the per-step default path
// carries no configuration-string work. The variational integrator solves a
// supported subset of loop closures (see compute::bindVariationalLoopClosure);
// the semi-implicit pipeline has no loop-closure solving stage and rejects
// every Solve closure.
void validateLoopClosureDynamicsPolicySupport(
    const World& world, bool variationalSelected)
{
  auto view = detail::registryOf(world).view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& closure = view.get<comps::LoopClosure>(entity);
    if (!closure.runtimePolicy.enabled
        || closure.runtimePolicy.dynamics
               == ClosureDynamicsPolicy::ResidualOnly) {
      continue;
    }

    const auto& name = view.get<comps::Name>(entity);
    if (variationalSelected) {
      const auto binding = compute::bindVariationalLoopClosure(
          detail::registryOf(world), entity);
      DART_EXPERIMENTAL_THROW_T_IF(
          binding.status
              == compute::VariationalLoopClosureBinding::Status::Unsupported,
          InvalidOperationException,
          "LoopClosure '{}' cannot be solved by the variational integrator: {}",
          name.name,
          binding.reason);
      continue; // Supported: the variational stage will enforce it.
    }

    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException,
        "LoopClosure '{}' requests dynamic solving, but the active pipeline "
        "does not include a loop-closure solving stage",
        name.name);
  }
}

//==============================================================================
struct WorldStepPipelineStages
{
  compute::RigidBodyVelocityStage rigidBodyVelocity;
  compute::RigidBodyContactStage rigidBodyContact;
  compute::RigidBodyPositionStage rigidBodyPosition;
  compute::RigidIpcContactStage rigidIpcContact;
  compute::MultibodyVelocityStage multibodyVelocity;
  compute::MultibodyContactStage multibodyContact;
  compute::MultibodyPositionStage multibodyPosition;
  compute::MultibodyForwardDynamicsStage multibodyDynamics;
  compute::UnifiedConstraintStage unifiedConstraint;
  compute::MultibodyVariationalIntegrationStage multibodyVariational;
  compute::DeformableDynamicsStage deformableDynamics;
  compute::KinematicsStage kinematics;
  compute::WorldStepPipeline pipeline;

  compute::WorldStepPipeline& buildDefault(
      RigidBodySolver rigidBodySolver,
      bool variationalSelected,
      bool useUnifiedConstraints)
  {
    DART_EXPERIMENTAL_THROW_T_IF(
        !isValidRigidBodySolver(rigidBodySolver),
        InvalidArgumentException,
        "Rigid-body solver is invalid");

    switch (rigidBodySolver) {
      case RigidBodySolver::SequentialImpulse:
        if (variationalSelected) {
          appendVariationalDynamicsStages();
        } else if (useUnifiedConstraints) {
          appendSemiImplicitUnifiedStages();
        } else {
          appendSemiImplicitRigidBodyStages();
        }
        break;
      case RigidBodySolver::Ipc:
        appendIpcDynamicsStages(variationalSelected);
        break;
    }
    pipeline.addStage(kinematics);
    return pipeline;
  }

  compute::WorldStepPipeline& buildWithFinalStage(
      RigidBodySolver rigidBodySolver,
      bool variationalSelected,
      bool useUnifiedConstraints,
      compute::WorldStepStage& finalStage)
  {
    DART_EXPERIMENTAL_THROW_T_IF(
        !isValidRigidBodySolver(rigidBodySolver),
        InvalidArgumentException,
        "Rigid-body solver is invalid");

    switch (rigidBodySolver) {
      case RigidBodySolver::SequentialImpulse:
        if (variationalSelected) {
          appendVariationalDynamicsStages();
        } else if (useUnifiedConstraints) {
          appendSemiImplicitUnifiedStages();
        } else {
          appendSemiImplicitRigidBodyStages();
        }
        break;
      case RigidBodySolver::Ipc:
        appendIpcDynamicsStages(variationalSelected);
        break;
    }
    pipeline.addStage(finalStage);
    return pipeline;
  }

private:
  void appendVariationalDynamicsStages()
  {
    // Integrate rigid-body positions after the multibody stage so two-sided
    // link-vs-rigid-body contact impulses (applied to rigid-body velocities in
    // the multibody solve) take effect in the same step's pose update.
    pipeline.addStage(rigidBodyVelocity)
        .addStage(rigidBodyContact)
        .addStage(multibodyVariational)
        .addStage(deformableDynamics)
        .addStage(rigidBodyPosition);
  }

  void appendIpcDynamicsStages(bool variationalSelected)
  {
    compute::WorldStepStage& multibodyStage
        = variationalSelected
              ? static_cast<compute::WorldStepStage&>(multibodyVariational)
              : static_cast<compute::WorldStepStage&>(multibodyDynamics);
    pipeline.addStage(rigidIpcContact)
        .addStage(multibodyStage)
        .addStage(deformableDynamics);
  }

  void appendSemiImplicitRigidBodyStages()
  {
    pipeline.addStage(rigidBodyVelocity)
        .addStage(rigidBodyContact)
        .addStage(multibodyDynamics)
        .addStage(deformableDynamics)
        .addStage(rigidBodyPosition);
  }

  void appendSemiImplicitUnifiedStages()
  {
    // When articulated bodies are present, one coupled boxed-LCP over all
    // rigid-rigid and articulated link contacts replaces the separate
    // RigidBodyContactStage + MultibodyContactStage passes. Positions-last is
    // preserved (both velocity stages precede the solve, both position stages
    // follow), and the joint velocity-limit clamp stays post-solve in
    // MultibodyPositionStage.
    pipeline.addStage(rigidBodyVelocity)
        .addStage(multibodyVelocity)
        .addStage(unifiedConstraint)
        .addStage(rigidBodyPosition)
        .addStage(multibodyPosition)
        .addStage(deformableDynamics);
  }
};

//==============================================================================
bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

//==============================================================================
void validateFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !value.allFinite(),
      InvalidArgumentException,
      "RigidBodyOptions.{} must contain only finite values",
      fieldName);
}

//==============================================================================
bool isValidLoopClosureFamily(LoopClosureFamily family)
{
  switch (family) {
    case LoopClosureFamily::Rigid:
    case LoopClosureFamily::Point:
    case LoopClosureFamily::Distance:
      return true;
  }

  return false;
}

//==============================================================================
void validateLoopClosureOffset(
    const Eigen::Isometry3d& offset, std::string_view fieldName)
{
  constexpr double tolerance = 1e-9;

  DART_EXPERIMENTAL_THROW_T_IF(
      !offset.matrix().allFinite(),
      InvalidArgumentException,
      "LoopClosureSpec.{} must contain only finite values",
      fieldName);

  const auto& rotation = offset.linear();
  const double orthonormalError
      = (rotation * rotation.transpose() - Eigen::Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff();
  DART_EXPERIMENTAL_THROW_T_IF(
      orthonormalError > tolerance
          || std::abs(rotation.determinant() - 1.0) > tolerance,
      InvalidArgumentException,
      "LoopClosureSpec.{} rotation must be orthonormal",
      fieldName);
}

//==============================================================================
entt::entity resolveLoopClosureFrame(
    const World& world, const Frame& frame, std::string_view fieldName)
{
  if (frame.isWorld()) {
    return entt::null;
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !frame.isValid(),
      InvalidArgumentException,
      "LoopClosureSpec.{} is invalid or has been destroyed",
      fieldName);

  DART_EXPERIMENTAL_THROW_T_IF(
      frame.getWorld() != &world,
      InvalidArgumentException,
      "LoopClosureSpec.{} belongs to a different world",
      fieldName);

  return detail::toRegistryEntity(frame.getEntity());
}

//==============================================================================
void validateLoopClosureSpec(const World& world, const LoopClosureSpec& spec)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidLoopClosureFamily(spec.family),
      InvalidArgumentException,
      "LoopClosureSpec.family is invalid");

  const auto frameA = resolveLoopClosureFrame(world, spec.frameA, "frameA");
  const auto frameB = resolveLoopClosureFrame(world, spec.frameB, "frameB");
  DART_EXPERIMENTAL_THROW_T_IF(
      frameA == frameB,
      InvalidArgumentException,
      "LoopClosureSpec endpoints must be distinct frames");

  validateLoopClosureOffset(spec.offsetA, "offsetA");
  validateLoopClosureOffset(spec.offsetB, "offsetB");
}

//==============================================================================
void validateRigidBodyOptions(const RigidBodyOptions& options)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(options.mass) || options.mass <= 0.0,
      InvalidArgumentException,
      "RigidBodyOptions.mass must be positive and finite");

  DART_EXPERIMENTAL_THROW_T_IF(
      !isSymmetricPositiveDefinite(options.inertia),
      InvalidArgumentException,
      "RigidBodyOptions.inertia must be symmetric positive definite");

  validateFiniteVector(options.position, "position");
  validateFiniteVector(options.linearVelocity, "linearVelocity");
  validateFiniteVector(options.angularVelocity, "angularVelocity");

  const auto orientationNorm = options.orientation.norm();
  DART_EXPERIMENTAL_THROW_T_IF(
      !options.orientation.coeffs().allFinite()
          || !std::isfinite(orientationNorm) || orientationNorm <= 0.0,
      InvalidArgumentException,
      "RigidBodyOptions.orientation must be finite and non-zero");
}

//==============================================================================
void validateDeformableFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName, std::size_t index)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !value.allFinite(),
      InvalidArgumentException,
      "DeformableBodyOptions.{}[{}] must contain only finite values",
      fieldName,
      index);
}

//==============================================================================
std::array<std::size_t, 3> sortedFaceKey(
    std::size_t nodeA, std::size_t nodeB, std::size_t nodeC)
{
  std::array<std::size_t, 3> key{nodeA, nodeB, nodeC};
  std::ranges::sort(key);
  return key;
}

//==============================================================================
std::array<std::size_t, 4> sortedTetrahedronKey(
    std::size_t nodeA, std::size_t nodeB, std::size_t nodeC, std::size_t nodeD)
{
  std::array<std::size_t, 4> key{nodeA, nodeB, nodeC, nodeD};
  std::ranges::sort(key);
  return key;
}

//==============================================================================
bool hasRepeatedNodes(std::span<const std::size_t> nodes)
{
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    for (std::size_t j = i + 1; j < nodes.size(); ++j) {
      if (nodes[i] == nodes[j]) {
        return true;
      }
    }
  }
  return false;
}

//==============================================================================
double signedTetrahedronVolume(
    const std::vector<Eigen::Vector3d>& positions,
    const comps::DeformableTetrahedron& tetrahedron)
{
  const auto& a = positions[tetrahedron.nodeA];
  const auto& b = positions[tetrahedron.nodeB];
  const auto& c = positions[tetrahedron.nodeC];
  const auto& d = positions[tetrahedron.nodeD];
  return (b - a).cross(c - a).dot(d - a) / 6.0;
}

//==============================================================================
double surfaceTriangleAreaSquared(
    const std::vector<Eigen::Vector3d>& positions,
    const comps::DeformableSurfaceTriangle& triangle)
{
  const auto& a = positions[triangle.nodeA];
  const auto& b = positions[triangle.nodeB];
  const auto& c = positions[triangle.nodeC];
  return 0.25 * (b - a).cross(c - a).squaredNorm();
}

//==============================================================================
void validateDeformableMaterial(const DeformableMaterialProperties& material)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(material.density) || material.density <= 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material.density must be positive and finite");
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(material.youngsModulus) || material.youngsModulus <= 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material.youngsModulus must be positive and "
      "finite");
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(material.poissonRatio) || material.poissonRatio <= -1.0
          || material.poissonRatio >= 0.5,
      InvalidArgumentException,
      "DeformableBodyOptions.material.poissonRatio must be finite and in "
      "(-1, 0.5)");
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(material.frictionCoefficient)
          || material.frictionCoefficient < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.material.frictionCoefficient must be finite and "
      "non-negative");
}

//==============================================================================
struct PreparedDeformableBodyData
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> restPositions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<double> masses;
  std::vector<std::uint8_t> fixed;
  std::vector<comps::DeformableSpringEdge> edges;
  std::vector<comps::DeformableSurfaceTriangle> surfaceTriangles;
  std::vector<comps::DeformableTetrahedron> tetrahedra;
  std::vector<double> tetrahedronRestVolumes;
  comps::DeformableMaterial material;
  comps::DeformableBoundaryConditions boundaryConditions;
  double stiffness = 0.0;
  double damping = 0.0;
};

//==============================================================================
bool hasValidBoundaryEndTime(double value)
{
  return std::isfinite(value)
         || value == std::numeric_limits<double>::infinity();
}

//==============================================================================
bool boundaryRangesOverlap(
    double startA, double endA, double startB, double endB)
{
  return startA < endB && startB < endA;
}

//==============================================================================
void validateBoundaryTimeRange(
    double startTime, double endTime, std::string_view fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(startTime) || startTime < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.{} start time must be finite and non-negative",
      fieldName);
  DART_EXPERIMENTAL_THROW_T_IF(
      !hasValidBoundaryEndTime(endTime) || endTime < startTime,
      InvalidArgumentException,
      "DeformableBodyOptions.{} end time must be finite or infinity and must "
      "not precede the start time",
      fieldName);
}

//==============================================================================
void validateBoundaryNodes(
    std::span<const std::size_t> nodes,
    std::size_t nodeCount,
    std::string_view fieldName,
    std::size_t boundaryIndex)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      nodes.empty(),
      InvalidArgumentException,
      "DeformableBodyOptions.{}[{}] must reference at least one node",
      fieldName,
      boundaryIndex);
  std::set<std::size_t> uniqueNodes;
  for (const auto node : nodes) {
    DART_EXPERIMENTAL_THROW_T_IF(
        node >= nodeCount,
        InvalidArgumentException,
        "DeformableBodyOptions.{}[{}] references an out-of-range node",
        fieldName,
        boundaryIndex);
    DART_EXPERIMENTAL_THROW_T_IF(
        !uniqueNodes.insert(node).second,
        InvalidArgumentException,
        "DeformableBodyOptions.{}[{}] contains duplicate node {}",
        fieldName,
        boundaryIndex,
        node);
  }
}

//==============================================================================
void validateNoBoundaryConflicts(
    const DeformableBodyOptions& options,
    const std::vector<std::uint8_t>& fixed)
{
  for (std::size_t dirichletIndex = 0;
       dirichletIndex < options.dirichletBoundaryConditions.size();
       ++dirichletIndex) {
    const auto& dirichlet = options.dirichletBoundaryConditions[dirichletIndex];
    for (const auto node : dirichlet.nodes) {
      DART_EXPERIMENTAL_THROW_T_IF(
          fixed[node] != 0u,
          InvalidArgumentException,
          "DeformableBodyOptions.dirichletBoundaryConditions[{}] overlaps "
          "permanently fixed node {}",
          dirichletIndex,
          node);
    }

    for (std::size_t otherIndex = dirichletIndex + 1;
         otherIndex < options.dirichletBoundaryConditions.size();
         ++otherIndex) {
      const auto& other = options.dirichletBoundaryConditions[otherIndex];
      if (!boundaryRangesOverlap(
              dirichlet.startTime,
              dirichlet.endTime,
              other.startTime,
              other.endTime)) {
        continue;
      }
      for (const auto node : dirichlet.nodes) {
        DART_EXPERIMENTAL_THROW_T_IF(
            std::ranges::find(other.nodes, node) != other.nodes.end(),
            InvalidArgumentException,
            "DeformableBodyOptions.dirichletBoundaryConditions overlap on "
            "node {}",
            node);
      }
    }
  }

  for (std::size_t neumannIndex = 0;
       neumannIndex < options.neumannBoundaryConditions.size();
       ++neumannIndex) {
    const auto& neumann = options.neumannBoundaryConditions[neumannIndex];
    for (const auto node : neumann.nodes) {
      DART_EXPERIMENTAL_THROW_T_IF(
          fixed[node] != 0u,
          InvalidArgumentException,
          "DeformableBodyOptions.neumannBoundaryConditions[{}] overlaps "
          "permanently fixed node {}",
          neumannIndex,
          node);
    }

    for (const auto& dirichlet : options.dirichletBoundaryConditions) {
      if (!boundaryRangesOverlap(
              neumann.startTime,
              neumann.endTime,
              dirichlet.startTime,
              dirichlet.endTime)) {
        continue;
      }
      for (const auto node : neumann.nodes) {
        DART_EXPERIMENTAL_THROW_T_IF(
            std::ranges::find(dirichlet.nodes, node) != dirichlet.nodes.end(),
            InvalidArgumentException,
            "DeformableBodyOptions.neumannBoundaryConditions overlap active "
            "Dirichlet boundary node {}",
            node);
      }
    }
  }
}

//==============================================================================
std::vector<comps::DeformableSurfaceTriangle>
validateDeformableSurfaceTriangles(const DeformableBodyOptions& options)
{
  std::vector<comps::DeformableSurfaceTriangle> surfaceTriangles;
  surfaceTriangles.reserve(options.surfaceTriangles.size());

  std::set<std::array<std::size_t, 3>> uniqueFaces;
  for (std::size_t i = 0; i < options.surfaceTriangles.size(); ++i) {
    const auto& triangle = options.surfaceTriangles[i];
    const std::array<std::size_t, 3> nodes{
        triangle.nodeA, triangle.nodeB, triangle.nodeC};
    DART_EXPERIMENTAL_THROW_T_IF(
        hasRepeatedNodes(nodes),
        InvalidArgumentException,
        "DeformableBodyOptions.surfaceTriangles[{}] nodes must be distinct",
        i);
    for (const auto node : nodes) {
      DART_EXPERIMENTAL_THROW_T_IF(
          node >= options.positions.size(),
          InvalidArgumentException,
          "DeformableBodyOptions.surfaceTriangles[{}] references an "
          "out-of-range node",
          i);
    }

    comps::DeformableSurfaceTriangle internal{
        triangle.nodeA, triangle.nodeB, triangle.nodeC};
    DART_EXPERIMENTAL_THROW_T_IF(
        surfaceTriangleAreaSquared(options.positions, internal) <= 1e-24,
        InvalidArgumentException,
        "DeformableBodyOptions.surfaceTriangles[{}] is degenerate",
        i);

    const auto key
        = sortedFaceKey(internal.nodeA, internal.nodeB, internal.nodeC);
    DART_EXPERIMENTAL_THROW_T_IF(
        !uniqueFaces.insert(key).second,
        InvalidArgumentException,
        "DeformableBodyOptions.surfaceTriangles[{}] duplicates an existing "
        "face",
        i);
    surfaceTriangles.push_back(internal);
  }

  return surfaceTriangles;
}

//==============================================================================
void addBoundaryFace(
    std::map<
        std::array<std::size_t, 3>,
        std::pair<comps::DeformableSurfaceTriangle, std::size_t>>& faces,
    comps::DeformableSurfaceTriangle face)
{
  const auto key = sortedFaceKey(face.nodeA, face.nodeB, face.nodeC);
  auto [it, inserted] = faces.emplace(key, std::pair{face, 0u});
  ++it->second.second;
  DART_EXPERIMENTAL_THROW_T_IF(
      it->second.second > 2u,
      InvalidArgumentException,
      "DeformableBodyOptions.tetrahedra creates a nonmanifold surface face");
  if (!inserted && it->second.second == 2u) {
    it->second.first = face;
  }
}

//==============================================================================
std::vector<comps::DeformableSurfaceTriangle> deriveDeformableBoundarySurface(
    const std::vector<comps::DeformableTetrahedron>& tetrahedra)
{
  std::map<
      std::array<std::size_t, 3>,
      std::pair<comps::DeformableSurfaceTriangle, std::size_t>>
      faces;
  for (const auto& tet : tetrahedra) {
    addBoundaryFace(faces, {tet.nodeA, tet.nodeC, tet.nodeB});
    addBoundaryFace(faces, {tet.nodeA, tet.nodeB, tet.nodeD});
    addBoundaryFace(faces, {tet.nodeA, tet.nodeD, tet.nodeC});
    addBoundaryFace(faces, {tet.nodeB, tet.nodeC, tet.nodeD});
  }

  std::vector<comps::DeformableSurfaceTriangle> surfaceTriangles;
  for (const auto& [_, faceAndCount] : faces) {
    if (faceAndCount.second == 1u) {
      surfaceTriangles.push_back(faceAndCount.first);
    }
  }
  return surfaceTriangles;
}

//==============================================================================
PreparedDeformableBodyData prepareDeformableBodyOptions(
    const DeformableBodyOptions& options)
{
  const auto nodeCount = options.positions.size();
  DART_EXPERIMENTAL_THROW_T_IF(
      nodeCount == 0,
      InvalidArgumentException,
      "DeformableBodyOptions.positions must not be empty");

  PreparedDeformableBodyData data;
  data.positions = options.positions;
  data.restPositions = data.positions;
  data.velocities.assign(nodeCount, Eigen::Vector3d::Zero());
  data.masses.assign(nodeCount, 1.0);
  data.fixed.assign(nodeCount, 0u);
  data.stiffness = options.edgeStiffness;
  data.damping = options.damping;
  data.material.density = options.material.density;
  data.material.youngsModulus = options.material.youngsModulus;
  data.material.poissonRatio = options.material.poissonRatio;
  data.material.frictionCoefficient = options.material.frictionCoefficient;
  data.material.useFiniteElementElasticity
      = options.material.useFiniteElementElasticity;
  data.material.useFixedCorotationalElasticity
      = options.material.useFixedCorotationalElasticity;
  data.material.useAdaptiveBarrierStiffness
      = options.material.useAdaptiveBarrierStiffness;
  data.material.useIterativeLinearSolver
      = options.material.useIterativeLinearSolver;
  data.material.useMatrixFreeLinearSolver
      = options.material.useMatrixFreeLinearSolver;

  for (std::size_t i = 0; i < nodeCount; ++i) {
    validateDeformableFiniteVector(options.positions[i], "positions", i);
  }

  validateDeformableMaterial(options.material);

  std::set<std::array<std::size_t, 4>> uniqueTetrahedra;
  data.tetrahedra.reserve(options.tetrahedra.size());
  data.tetrahedronRestVolumes.reserve(options.tetrahedra.size());
  for (std::size_t i = 0; i < options.tetrahedra.size(); ++i) {
    const auto& tetrahedron = options.tetrahedra[i];
    const std::array<std::size_t, 4> nodes{
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD};
    DART_EXPERIMENTAL_THROW_T_IF(
        hasRepeatedNodes(nodes),
        InvalidArgumentException,
        "DeformableBodyOptions.tetrahedra[{}] nodes must be distinct",
        i);
    for (const auto node : nodes) {
      DART_EXPERIMENTAL_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "DeformableBodyOptions.tetrahedra[{}] references an out-of-range "
          "node",
          i);
    }

    const auto key = sortedTetrahedronKey(
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD);
    DART_EXPERIMENTAL_THROW_T_IF(
        !uniqueTetrahedra.insert(key).second,
        InvalidArgumentException,
        "DeformableBodyOptions.tetrahedra[{}] duplicates an existing "
        "tetrahedron",
        i);

    comps::DeformableTetrahedron internal{
        tetrahedron.nodeA,
        tetrahedron.nodeB,
        tetrahedron.nodeC,
        tetrahedron.nodeD};
    double volume = signedTetrahedronVolume(options.positions, internal);
    DART_EXPERIMENTAL_THROW_T_IF(
        !std::isfinite(volume) || std::abs(volume) <= 1e-18,
        InvalidArgumentException,
        "DeformableBodyOptions.tetrahedra[{}] has zero or nonfinite rest "
        "volume",
        i);
    if (volume < 0.0) {
      std::swap(internal.nodeC, internal.nodeD);
      volume = -volume;
    }

    data.tetrahedra.push_back(internal);
    data.tetrahedronRestVolumes.push_back(volume);
  }

  data.surfaceTriangles = validateDeformableSurfaceTriangles(options);
  if (data.surfaceTriangles.empty() && !data.tetrahedra.empty()) {
    data.surfaceTriangles = deriveDeformableBoundarySurface(data.tetrahedra);
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      options.masses.empty() && options.tetrahedra.empty()
          && !options.surfaceTriangles.empty(),
      InvalidArgumentException,
      "DeformableBodyOptions.surfaceTriangles require explicit masses when no "
      "tetrahedra are provided");

  DART_EXPERIMENTAL_THROW_T_IF(
      !options.velocities.empty() && options.velocities.size() != nodeCount,
      InvalidArgumentException,
      "DeformableBodyOptions.velocities must be empty or match positions");
  for (std::size_t i = 0; i < options.velocities.size(); ++i) {
    validateDeformableFiniteVector(options.velocities[i], "velocities", i);
    data.velocities[i] = options.velocities[i];
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !options.masses.empty() && options.masses.size() != nodeCount,
      InvalidArgumentException,
      "DeformableBodyOptions.masses must be empty or match positions");
  if (!options.masses.empty()) {
    for (std::size_t i = 0; i < options.masses.size(); ++i) {
      const double mass = options.masses[i];
      DART_EXPERIMENTAL_THROW_T_IF(
          !std::isfinite(mass) || mass <= 0.0,
          InvalidArgumentException,
          "DeformableBodyOptions.masses[{}] must be positive and finite",
          i);
      data.masses[i] = mass;
    }
  } else if (!data.tetrahedra.empty()) {
    data.masses.assign(nodeCount, 0.0);
    for (std::size_t i = 0; i < data.tetrahedra.size(); ++i) {
      const auto& tetrahedron = data.tetrahedra[i];
      const double nodeMass
          = options.material.density * data.tetrahedronRestVolumes[i] / 4.0;
      data.masses[tetrahedron.nodeA] += nodeMass;
      data.masses[tetrahedron.nodeB] += nodeMass;
      data.masses[tetrahedron.nodeC] += nodeMass;
      data.masses[tetrahedron.nodeD] += nodeMass;
    }

    for (std::size_t i = 0; i < data.masses.size(); ++i) {
      DART_EXPERIMENTAL_THROW_T_IF(
          !std::isfinite(data.masses[i]) || data.masses[i] <= 0.0,
          InvalidArgumentException,
          "DeformableBodyOptions.tetrahedra leave node {} without positive "
          "finite assembled mass",
          i);
    }
  }

  for (const auto fixedNode : options.fixedNodes) {
    DART_EXPERIMENTAL_THROW_T_IF(
        fixedNode >= nodeCount,
        InvalidArgumentException,
        "DeformableBodyOptions.fixedNodes contains out-of-range node {}",
        fixedNode);
    DART_EXPERIMENTAL_THROW_T_IF(
        data.fixed[fixedNode] != 0u,
        InvalidArgumentException,
        "DeformableBodyOptions.fixedNodes contains duplicate node {}",
        fixedNode);
    data.fixed[fixedNode] = 1u;
  }

  data.boundaryConditions.dirichlet.reserve(
      options.dirichletBoundaryConditions.size());
  for (std::size_t i = 0; i < options.dirichletBoundaryConditions.size(); ++i) {
    const auto& boundary = options.dirichletBoundaryConditions[i];
    validateBoundaryNodes(
        boundary.nodes, nodeCount, "dirichletBoundaryConditions", i);
    validateDeformableFiniteVector(
        boundary.linearVelocity,
        "dirichletBoundaryConditions.linearVelocity",
        i);
    validateDeformableFiniteVector(
        boundary.angularVelocity,
        "dirichletBoundaryConditions.angularVelocity",
        i);
    validateDeformableFiniteVector(
        boundary.center, "dirichletBoundaryConditions.center", i);
    validateBoundaryTimeRange(
        boundary.startTime, boundary.endTime, "dirichletBoundaryConditions");

    comps::DeformableDirichletBoundary internal;
    internal.nodes = boundary.nodes;
    internal.referencePositions.reserve(boundary.nodes.size());
    for (const auto node : boundary.nodes) {
      internal.referencePositions.push_back(data.restPositions[node]);
    }
    internal.center = boundary.center;
    internal.linearVelocity = boundary.linearVelocity;
    internal.angularVelocity = boundary.angularVelocity;
    internal.startTime = boundary.startTime;
    internal.endTime = boundary.endTime;
    data.boundaryConditions.dirichlet.push_back(std::move(internal));
  }

  data.boundaryConditions.neumann.reserve(
      options.neumannBoundaryConditions.size());
  for (std::size_t i = 0; i < options.neumannBoundaryConditions.size(); ++i) {
    const auto& boundary = options.neumannBoundaryConditions[i];
    validateBoundaryNodes(
        boundary.nodes, nodeCount, "neumannBoundaryConditions", i);
    validateDeformableFiniteVector(
        boundary.acceleration, "neumannBoundaryConditions.acceleration", i);
    validateBoundaryTimeRange(
        boundary.startTime, boundary.endTime, "neumannBoundaryConditions");

    comps::DeformableNeumannBoundary internal;
    internal.nodes = boundary.nodes;
    internal.acceleration = boundary.acceleration;
    internal.startTime = boundary.startTime;
    internal.endTime = boundary.endTime;
    data.boundaryConditions.neumann.push_back(std::move(internal));
  }

  validateNoBoundaryConflicts(options, data.fixed);

  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(options.edgeStiffness) || options.edgeStiffness < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.edgeStiffness must be finite and non-negative");
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(options.damping) || options.damping < 0.0,
      InvalidArgumentException,
      "DeformableBodyOptions.damping must be finite and non-negative");

  data.edges.reserve(options.edges.size());
  for (std::size_t i = 0; i < options.edges.size(); ++i) {
    const auto& edge = options.edges[i];
    DART_EXPERIMENTAL_THROW_T_IF(
        edge.nodeA >= nodeCount || edge.nodeB >= nodeCount,
        InvalidArgumentException,
        "DeformableBodyOptions.edges[{}] references an out-of-range node",
        i);
    DART_EXPERIMENTAL_THROW_T_IF(
        edge.nodeA == edge.nodeB,
        InvalidArgumentException,
        "DeformableBodyOptions.edges[{}] endpoints must be distinct",
        i);

    double restLength = edge.restLength;
    if (restLength <= 0.0) {
      restLength
          = (options.positions[edge.nodeB] - options.positions[edge.nodeA])
                .norm();
    }
    DART_EXPERIMENTAL_THROW_T_IF(
        !std::isfinite(restLength) || restLength <= 0.0,
        InvalidArgumentException,
        "DeformableBodyOptions.edges[{}].restLength must be positive and "
        "finite",
        i);

    data.edges.push_back(
        comps::DeformableSpringEdge{edge.nodeA, edge.nodeB, restLength});
  }

  return data;
}

//==============================================================================
Eigen::Quaterniond normalizeOrIdentity(const Eigen::Quaterniond& orientation)
{
  const auto norm = orientation.norm();
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }

  auto normalized = orientation;
  normalized.coeffs() /= norm;
  return normalized;
}

//==============================================================================
Eigen::Isometry3d toIsometry(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  transform.linear() = orientation.toRotationMatrix();
  return transform;
}

} // namespace

World::World() : m_storage(std::make_unique<detail::WorldStorage>())
{
  // Empty.
}

//==============================================================================
World::World(const WorldOptions& options)
  : m_storage(std::make_unique<detail::WorldStorage>()),
    m_gravity(options.gravity),
    m_timeStep(options.timeStep),
    m_differentiable(options.differentiable),
    m_contactSolverMethod(options.contactSolverMethod),
    m_contactGradientMode(options.contactGradientMode)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(options.timeStep) || options.timeStep <= 0.0,
      InvalidArgumentException,
      "WorldOptions.timeStep must be positive and finite");
  DART_EXPERIMENTAL_THROW_T_IF(
      !options.gravity.array().isFinite().all(),
      InvalidArgumentException,
      "WorldOptions.gravity must contain only finite coordinates");
}

//==============================================================================
World::~World() = default;

//==============================================================================
detail::WorldStorage& detail::storageOf(World& world)
{
  return *world.m_storage;
}

//==============================================================================
const detail::WorldStorage& detail::storageOf(const World& world)
{
  return *world.m_storage;
}

//==============================================================================
entt::registry& detail::registryOf(World& world)
{
  return detail::storageOf(world).registry;
}

//==============================================================================
const entt::registry& detail::registryOf(const World& world)
{
  return detail::storageOf(world).registry;
}

//==============================================================================
void World::clear()
{
  m_storage->registry.clear();
  m_simulationMode = false;
  m_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  m_rigidBodySolver = RigidBodySolver::SequentialImpulse;
  m_timeStep = 0.001;
  m_differentiable = false;
  m_contactSolverMethod = ContactSolverMethod::SequentialImpulse;
  m_contactGradientMode = ContactGradientMode::Analytic;
  resetRigidIpcAdaptiveBarrierStiffnessLowerBound();
  m_storage->stepDerivatives.reset();
  m_storage->differentiableParameters.clear();
  m_time = 0.0;
  m_frame = 0;
  m_freeFrameCounter = 0;
  m_fixedFrameCounter = 0;
  m_multibodyCounter = 0;
  m_loopClosureCounter = 0;
  m_rigidBodyCounter = 0;
  m_deformableBodyCounter = 0;
  m_linkCounter = 0;
  m_jointCounter = 0;
  if (m_collisionQueryCache) {
    m_collisionQueryCache->clear();
  }
}

//==============================================================================
void World::ensureDesignMode() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      m_simulationMode,
      InvalidOperationException,
      "World modifications are not allowed while in simulation mode");
}

//==============================================================================
FreeFrame World::addFreeFrame()
{
  return addFreeFrame("", Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name)
{
  return addFreeFrame(name, Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name, const Frame& parent)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      Eigen::Isometry3d::Identity(),
      &m_freeFrameCounter,
      "free_frame",
      false,
      actualName);

  return FreeFrame(entity, this);
}

//==============================================================================
FixedFrame World::addFixedFrame(std::string_view name, const Frame& parent)
{
  return addFixedFrame(name, parent, Eigen::Isometry3d::Identity());
}

//==============================================================================
FixedFrame World::addFixedFrame(
    std::string_view name, const Frame& parent, const Eigen::Isometry3d& offset)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  DART_EXPERIMENTAL_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "FixedFrame requires a non-empty name");

  DART_EXPERIMENTAL_THROW_T_IF(
      parentFrame.isWorld(),
      InvalidArgumentException,
      "FixedFrame cannot be attached directly to the world frame");

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      offset,
      &m_fixedFrameCounter,
      "fixed_frame",
      true,
      actualName);

  return FixedFrame(entity, this);
}

//==============================================================================
Entity World::createFrameEntity(
    std::string_view name,
    const Frame& parentFrame,
    const Eigen::Isometry3d& localTransform,
    std::size_t* autoNameCounter,
    std::string_view autoNamePrefix,
    bool isFixedFrame,
    std::string& outName)
{
  std::string actualName;
  if (name.empty()) {
    if (autoNameCounter) {
      actualName
          = std::format("{}_{:03d}", autoNamePrefix, ++(*autoNameCounter));
    } else {
      actualName = std::string(autoNamePrefix);
    }
  } else {
    actualName = std::string(name);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, actualName);
  m_storage->registry.emplace<comps::FrameTag>(entity);

  if (isFixedFrame) {
    m_storage->registry.emplace<comps::FixedFrameTag>(entity);
  } else {
    m_storage->registry.emplace<comps::FreeFrameTag>(entity);
  }

  auto& state = m_storage->registry.emplace<comps::FrameState>(entity);
  state.parentFrame = parentFrame.isWorld()
                          ? entt::null
                          : detail::toRegistryEntity(parentFrame.getEntity());

  auto& cache = m_storage->registry.emplace<comps::FrameCache>(entity);
  cache.worldTransform = Eigen::Isometry3d::Identity();
  cache.needTransformUpdate = true;

  if (isFixedFrame) {
    auto& props
        = m_storage->registry.emplace<comps::FixedFrameProperties>(entity);
    props.localTransform = localTransform;
  } else {
    auto& props
        = m_storage->registry.emplace<comps::FreeFrameProperties>(entity);
    props.localTransform = localTransform;
  }

  outName = actualName;
  return detail::fromRegistryEntity(entity);
}

//==============================================================================
Frame World::resolveParentFrame(const Frame& parent) const
{
  if (parent.isWorld()) {
    return Frame(Entity{}, const_cast<World*>(this));
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Parent frame is invalid or has been destroyed");

  DART_EXPERIMENTAL_THROW_T_IF(
      parent.getWorld() != this,
      InvalidArgumentException,
      "Parent frame belongs to a different world");

  return parent;
}

//==============================================================================
Multibody World::addMultibody(std::string_view name)
{
  ensureDesignMode();
  DART_EXPERIMENTAL_THROW_T_IF(
      hasRigidBodyFixedJoints(*this),
      InvalidOperationException,
      "Multibody structures are not supported in worlds with rigid-body "
      "fixed joints");

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName = std::format("multibody_{:03d}", ++m_multibodyCounter);
    } while (hasEntityWithName<comps::MultibodyTag>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::MultibodyTag>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "Multibody '{}' already exists",
        candidateName);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, candidateName);
  m_storage->registry.emplace<comps::MultibodyTag>(entity);
  m_storage->registry.emplace<comps::MultibodyStructure>(entity);

  return Multibody(detail::fromRegistryEntity(entity), this);
}

//==============================================================================
std::optional<Multibody> World::getMultibody(std::string_view name)
{
  auto view = m_storage->registry.view<comps::MultibodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return Multibody(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasMultibody(std::string_view name) const
{
  return hasEntityWithName<comps::MultibodyTag>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getMultibodyCount() const
{
  return countEntities<comps::MultibodyTag>(m_storage->registry);
}

//==============================================================================
LoopClosure World::addLoopClosure(
    std::string_view name, const LoopClosureSpec& spec)
{
  ensureDesignMode();
  validateLoopClosureSpec(*this, spec);

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName
          = std::format("loop_closure_{:03d}", ++m_loopClosureCounter);
    } while (hasEntityWithName<comps::LoopClosure>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::LoopClosure>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "LoopClosure '{}' already exists",
        candidateName);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, candidateName);

  auto& closure = m_storage->registry.emplace<comps::LoopClosure>(entity);
  closure.family = spec.family;
  closure.frameA = resolveLoopClosureFrame(*this, spec.frameA, "frameA");
  closure.frameB = resolveLoopClosureFrame(*this, spec.frameB, "frameB");
  closure.offsetA = spec.offsetA;
  closure.offsetB = spec.offsetB;
  closure.distance = spec.distance;

  return LoopClosure(detail::fromRegistryEntity(entity), this);
}

//==============================================================================
std::optional<LoopClosure> World::getLoopClosure(std::string_view name)
{
  auto view = m_storage->registry.view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return LoopClosure(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasLoopClosure(std::string_view name) const
{
  return hasEntityWithName<comps::LoopClosure>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getLoopClosureCount() const
{
  return countEntities<comps::LoopClosure>(m_storage->registry);
}

//==============================================================================
RigidBody World::addRigidBody(
    std::string_view name, const RigidBodyOptions& options)
{
  ensureDesignMode();

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName = std::format("rigid_body_{:03d}", ++m_rigidBodyCounter);
    } while (hasEntityWithName<comps::RigidBodyTag>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::RigidBodyTag>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "RigidBody '{}' already exists",
        candidateName);
  }

  validateRigidBodyOptions(options);

  Frame parent = Frame(Entity{}, this);
  const auto orientation = normalizeOrIdentity(options.orientation);
  const auto initialTransform = toIsometry(options.position, orientation);

  std::string actualName;
  const Entity entity = createFrameEntity(
      candidateName,
      parent,
      initialTransform,
      &m_rigidBodyCounter,
      "rigid_body",
      false,
      actualName);
  const auto enttEntity = detail::toRegistryEntity(entity);

  m_storage->registry.emplace<comps::RigidBodyTag>(enttEntity);

  auto& transform = m_storage->registry.emplace<comps::Transform>(enttEntity);
  transform.position = options.position;
  transform.orientation = orientation;

  auto& velocity = m_storage->registry.emplace<comps::Velocity>(enttEntity);
  velocity.linear = options.linearVelocity;
  velocity.angular = options.angularVelocity;

  auto& mass = m_storage->registry.emplace<comps::MassProperties>(enttEntity);
  mass.mass = options.mass;
  mass.inertia = options.inertia;

  m_storage->registry.emplace<comps::Force>(enttEntity);

  if (options.isStatic) {
    m_storage->registry.emplace<comps::StaticBodyTag>(enttEntity);
  }

  return RigidBody(entity, this);
}

//==============================================================================
Joint World::addRigidBodyFixedJoint(
    std::string_view name, const RigidBody& parent, const RigidBody& child)
{
  ensureDesignMode();

  DART_EXPERIMENTAL_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Fixed-joint parent rigid body is invalid or has been destroyed");
  DART_EXPERIMENTAL_THROW_T_IF(
      !child.isValid(),
      InvalidArgumentException,
      "Fixed-joint child rigid body is invalid or has been destroyed");
  DART_EXPERIMENTAL_THROW_T_IF(
      parent.getWorld() != this || child.getWorld() != this,
      InvalidArgumentException,
      "Fixed-joint rigid bodies must belong to this World");
  DART_EXPERIMENTAL_THROW_T_IF(
      parent.getEntity() == child.getEntity(),
      InvalidArgumentException,
      "Fixed-joint parent and child rigid bodies must be distinct");

  const entt::entity parentEntity
      = detail::toRegistryEntity(parent.getEntity());
  const entt::entity childEntity = detail::toRegistryEntity(child.getEntity());
  const bool parentIsRigidBody = m_storage->registry.all_of<
      comps::RigidBodyTag,
      comps::Transform,
      comps::MassProperties>(parentEntity);
  const bool childIsRigidBody = m_storage->registry.all_of<
      comps::RigidBodyTag,
      comps::Transform,
      comps::MassProperties>(childEntity);
  DART_EXPERIMENTAL_THROW_T_IF(
      !parentIsRigidBody || !childIsRigidBody,
      InvalidArgumentException,
      "Fixed-joint endpoints must be valid rigid bodies");
  DART_EXPERIMENTAL_THROW_T_IF(
      m_rigidBodySolver == RigidBodySolver::Ipc,
      InvalidOperationException,
      "Rigid-body fixed joints are not supported by the IPC rigid-body solver");
  DART_EXPERIMENTAL_THROW_T_IF(
      hasMultibodyStructures(*this),
      InvalidOperationException,
      "Rigid-body fixed joints are not supported in worlds with multibody "
      "structures");

  std::string actualName;
  if (name.empty()) {
    do {
      actualName = std::format("joint_{:03d}", ++m_jointCounter);
    } while (hasEntityWithName<comps::Joint>(m_storage->registry, actualName));
  } else {
    actualName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::Joint>(m_storage->registry, actualName),
        InvalidArgumentException,
        "Joint '{}' already exists",
        actualName);
  }

  const entt::entity jointEntity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(jointEntity, actualName);

  auto& joint = m_storage->registry.emplace<comps::Joint>(jointEntity);
  joint.type = comps::JointType::Fixed;
  joint.name = std::move(actualName);
  joint.parentLink = parentEntity;
  joint.childLink = childEntity;

  const Eigen::Index dof = static_cast<Eigen::Index>(joint.getDOF());
  joint.position = Eigen::VectorXd::Zero(dof);
  joint.velocity = Eigen::VectorXd::Zero(dof);
  joint.acceleration = Eigen::VectorXd::Zero(dof);
  joint.torque = Eigen::VectorXd::Zero(dof);
  joint.springStiffness = Eigen::VectorXd::Zero(dof);
  joint.dampingCoefficient = Eigen::VectorXd::Zero(dof);
  joint.restPosition = Eigen::VectorXd::Zero(dof);
  joint.armature = Eigen::VectorXd::Zero(dof);
  joint.coulombFriction = Eigen::VectorXd::Zero(dof);
  joint.commandVelocity = Eigen::VectorXd::Zero(dof);

  const double infinity = std::numeric_limits<double>::infinity();
  joint.limits.lower = Eigen::VectorXd::Constant(dof, -infinity);
  joint.limits.upper = Eigen::VectorXd::Constant(dof, infinity);
  joint.limits.velocityLower = Eigen::VectorXd::Constant(dof, -infinity);
  joint.limits.velocityUpper = Eigen::VectorXd::Constant(dof, infinity);
  joint.limits.effortLower = Eigen::VectorXd::Constant(dof, -infinity);
  joint.limits.effortUpper = Eigen::VectorXd::Constant(dof, infinity);

  const comps::RigidAvbdContactConfig defaultAvbdConfig;
  if (!detail::deformable_vbd::configureAvbdRigidWorldFixedJointFromCurrentPose(
          m_storage->registry,
          jointEntity,
          defaultAvbdConfig.startStiffness,
          defaultAvbdConfig.maxStiffness)) {
    m_storage->registry.destroy(jointEntity);
    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException,
        "Failed to configure fixed joint '{}' from current rigid-body poses",
        name);
  }

  return Joint(detail::fromRegistryEntity(jointEntity), this);
}

//==============================================================================
std::optional<Joint> World::getRigidBodyFixedJoint(std::string_view name)
{
  auto view = m_storage->registry.view<comps::Joint, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::Joint>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isRigidBodyFixedJoint(m_storage->registry, joint)) {
      return Joint(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasRigidBodyFixedJoint(std::string_view name) const
{
  const auto view = m_storage->registry.view<comps::Joint, comps::Name>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::Joint>(entity);
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name
        && isRigidBodyFixedJoint(m_storage->registry, joint)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
std::size_t World::getRigidBodyFixedJointCount() const
{
  std::size_t count = 0;
  const auto view = m_storage->registry.view<comps::Joint>();
  for (auto entity : view) {
    (void)entity;
    const auto& joint = view.get<comps::Joint>(entity);
    if (isRigidBodyFixedJoint(m_storage->registry, joint)) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
std::vector<Joint> World::getRigidBodyFixedJoints()
{
  std::vector<Joint> joints;
  joints.reserve(getRigidBodyFixedJointCount());
  const auto view = m_storage->registry.view<comps::Joint>();
  for (auto entity : view) {
    const auto& joint = view.get<comps::Joint>(entity);
    if (isRigidBodyFixedJoint(m_storage->registry, joint)) {
      joints.emplace_back(detail::fromRegistryEntity(entity), this);
    }
  }
  return joints;
}

//==============================================================================
std::optional<RigidBody> World::getRigidBody(std::string_view name)
{
  auto view = m_storage->registry.view<comps::RigidBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return RigidBody(detail::fromRegistryEntity(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasRigidBody(std::string_view name) const
{
  return hasEntityWithName<comps::RigidBodyTag>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getRigidBodyCount() const
{
  return countEntities<comps::RigidBodyTag>(m_storage->registry);
}

//==============================================================================
DeformableBody World::addDeformableBody(
    std::string_view name, const DeformableBodyOptions& options)
{
  ensureDesignMode();
  auto data = prepareDeformableBodyOptions(options);

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName
          = std::format("deformable_body_{:03d}", ++m_deformableBodyCounter);
    } while (hasEntityWithName<comps::DeformableBodyTag>(
        m_storage->registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::DeformableBodyTag>(
            m_storage->registry, candidateName),
        InvalidArgumentException,
        "DeformableBody '{}' already exists",
        candidateName);
  }

  auto entity = m_storage->registry.create();
  m_storage->registry.emplace<comps::Name>(entity, candidateName);
  m_storage->registry.emplace<comps::DeformableBodyTag>(entity);

  auto& state = m_storage->registry.emplace<comps::DeformableNodeState>(entity);
  state.positions = std::move(data.positions);
  state.previousPositions = state.positions;
  state.velocities = std::move(data.velocities);
  state.masses = std::move(data.masses);
  state.fixed = std::move(data.fixed);

  auto& model
      = m_storage->registry.emplace<comps::DeformableSpringModel>(entity);
  model.edges = std::move(data.edges);
  model.stiffness = data.stiffness;
  model.damping = data.damping;

  auto& topology
      = m_storage->registry.emplace<comps::DeformableMeshTopology>(entity);
  topology.restPositions = std::move(data.restPositions);
  topology.surfaceTriangles = std::move(data.surfaceTriangles);
  topology.tetrahedra = std::move(data.tetrahedra);
  topology.tetrahedronRestVolumes = std::move(data.tetrahedronRestVolumes);

  auto& material
      = m_storage->registry.emplace<comps::DeformableMaterial>(entity);
  material = data.material;

  if (!data.boundaryConditions.dirichlet.empty()
      || !data.boundaryConditions.neumann.empty()) {
    auto& boundaryConditions
        = m_storage->registry.emplace<comps::DeformableBoundaryConditions>(
            entity);
    boundaryConditions = std::move(data.boundaryConditions);
  }

  return DeformableBody(entt::to_integral(entity), this);
}

//==============================================================================
std::optional<DeformableBody> World::getDeformableBody(std::string_view name)
{
  auto view = m_storage->registry.view<comps::DeformableBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return DeformableBody(entt::to_integral(entity), this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasDeformableBody(std::string_view name) const
{
  return hasEntityWithName<comps::DeformableBodyTag>(m_storage->registry, name);
}

//==============================================================================
std::size_t World::getDeformableBodyCount() const
{
  return countEntities<comps::DeformableBodyTag>(m_storage->registry);
}

//==============================================================================
void World::configureDeformableSolver(
    std::string_view name, const DeformableSolverOptions& options)
{
  auto view = m_storage->registry.view<comps::DeformableBodyTag, comps::Name>();
  for (auto entity : view) {
    if (view.get<comps::Name>(entity).name != name) {
      continue;
    }
    // Translate the public, solver-agnostic options into the internal opt-in
    // inner-solver component. The mapping lives here (the World step pipeline
    // owns the concrete solver) so the public facade stays algorithm-neutral.
    m_storage->registry.emplace_or_replace<comps::DeformableVbdConfig>(
        entity,
        comps::DeformableVbdConfig{/*enabled=*/true,
                                   options.iterations,
                                   options.convergenceTolerance,
                                   options.useAcceleration,
                                   options.accelerationSpectralRadius,
                                   options.stiffnessDamping,
                                   options.workerThreads,
                                   options.groundContactStiffness});
    return;
  }
  DART_EXPERIMENTAL_THROW_T(
      InvalidArgumentException,
      "configureDeformableSolver: no deformable body named '{}'",
      name);
}

//==============================================================================
void World::enterSimulationMode()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      m_simulationMode,
      InvalidArgumentException,
      "World is already in simulation mode");

  validateLoopClosureKinematicsPolicySupport(*this);
  validateRigidBodyFixedJointPipelineSupport(*this, m_rigidBodySolver);
  m_simulationMode = true;

  // Initial bake so that cached transforms are up-to-date.
  updateKinematics();
  detail::deformable_vbd::configureAvbdRigidWorldFixedJointsFromCurrentPoses(
      m_storage->registry);
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& gravity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !gravity.array().isFinite().all(),
      InvalidArgumentException,
      "Gravity must contain only finite coordinates");

  m_gravity = gravity;
}

//==============================================================================
const Eigen::Vector3d& World::getGravity() const noexcept
{
  return m_gravity;
}

//==============================================================================
void World::setRigidBodySolver(RigidBodySolver solver)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidRigidBodySolver(solver),
      InvalidArgumentException,
      "Rigid-body solver is invalid");

  validateRigidBodyFixedJointPipelineSupport(*this, solver);
  m_rigidBodySolver = solver;
}

//==============================================================================
RigidBodySolver World::getRigidBodySolver() const noexcept
{
  return m_rigidBodySolver;
}

//==============================================================================
void World::setTimeStep(double timeStep)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(timeStep) || timeStep <= 0.0,
      InvalidArgumentException,
      "Time step must be positive and finite");

  m_timeStep = timeStep;
}

//==============================================================================
double World::getTimeStep() const noexcept
{
  return m_timeStep;
}

//==============================================================================
bool World::isDifferentiable() const noexcept
{
  return m_differentiable;
}

//==============================================================================
ContactSolverMethod World::getContactSolverMethod() const noexcept
{
  return m_contactSolverMethod;
}

//==============================================================================
ContactGradientMode World::getContactGradientMode() const noexcept
{
  return m_contactGradientMode;
}

//==============================================================================
void World::setContactGradientMode(ContactGradientMode mode) noexcept
{
  m_contactGradientMode = mode;
}

//==============================================================================
StepDerivatives World::getStepDerivatives() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_differentiable,
      InvalidOperationException,
      "World::getStepDerivatives() requires a differentiable World (construct "
      "with WorldOptions::differentiable set to true)");

#ifdef DART_HAS_DIFF
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_storage->stepDerivatives.has_value(),
      InvalidOperationException,
      "World::getStepDerivatives() has no derivatives yet; call step() first");
  return *m_storage->stepDerivatives;
#else
  DART_EXPERIMENTAL_THROW_T(
      InvalidOperationException,
      "World::getStepDerivatives() requires differentiable support to be built "
      "(enable the DART_BUILD_DIFF CMake option)");
#endif
}

//==============================================================================
StepGradient World::applyStepVjp(const Eigen::VectorXd& dLossDNextState) const
{
  // Reuse getStepDerivatives() for the not-differentiable / not-built /
  // no-derivatives-yet gating so applyStepVjp throws the identical errors. When
  // differentiable support is not compiled this call already throws, so the
  // explicit VJP below is only reached with valid cached Jacobians.
  const StepDerivatives derivatives = getStepDerivatives();

  DART_EXPERIMENTAL_THROW_T_IF(
      dLossDNextState.size() != derivatives.stateJacobian.rows(),
      InvalidArgumentException,
      "World::applyStepVjp(): dLossDNextState has size {} but the cached step "
      "Jacobian expects next-state size {}",
      dLossDNextState.size(),
      derivatives.stateJacobian.rows());

  StepGradient gradient;
  gradient.state = derivatives.stateJacobian.transpose() * dLossDNextState;
  gradient.control = derivatives.controlJacobian.transpose() * dLossDNextState;
  return gradient;
}

//==============================================================================
void World::addDifferentiableParameter(
    const PhysicalParameterSelector& selector)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_differentiable,
      InvalidOperationException,
      "World::addDifferentiableParameter() requires a differentiable World "
      "(construct with WorldOptions::differentiable set to true)");

#ifndef DART_HAS_DIFF
  (void)selector;
  DART_EXPERIMENTAL_THROW_T(
      InvalidOperationException,
      "World::addDifferentiableParameter() requires differentiable support to "
      "be built (enable the DART_BUILD_DIFF CMake option)");
#else
  DART_EXPERIMENTAL_THROW_T_IF(
      !selector.body.isValid() || selector.body.getWorld() != this,
      InvalidArgumentException,
      "World::addDifferentiableParameter(): the selector's body does not "
      "belong to this World");

  const auto entity = detail::toRegistryEntity(selector.body.getEntity());
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_storage->registry.all_of<comps::RigidBodyTag>(entity),
      InvalidArgumentException,
      "World::addDifferentiableParameter(): the selector's body is not a valid "
      "rigid body");

  // Supported parameters (PLAN-110 WS4): MASS (1 column), INERTIA (3 diagonal
  // principal-moment columns), and FRICTION (1 column). CENTER_OF_MASS stays
  // unsupported: the rigid-body forward step assumes the center of mass at the
  // body origin (`MassProperties::localCenterOfMass` is unused outside the
  // multibody path), so its single-step gradient is identically zero — there is
  // no meaningful Jacobian to assemble. Reject it with a clear
  // NotImplementedException rather than producing a vacuous zero column.
  DART_EXPERIMENTAL_THROW_T_IF(
      selector.parameter == PhysicalParameter::CENTER_OF_MASS,
      NotImplementedException,
      "World::addDifferentiableParameter(): PhysicalParameter::CENTER_OF_MASS "
      "is not supported for rigid bodies; the rigid-body step assumes the "
      "center of mass at the body origin, so the gradient is identically zero. "
      "Supported: MASS, INERTIA, FRICTION");

  m_storage->differentiableParameters.emplace_back(entity, selector.parameter);
#endif
}

//==============================================================================
void World::addDifferentiableParameter(
    const RigidBody& body, PhysicalParameter parameter)
{
  addDifferentiableParameter(PhysicalParameterSelector(body, parameter));
}

//==============================================================================
std::size_t World::getNumDifferentiableParameters() const noexcept
{
  return m_storage->differentiableParameters.size();
}

namespace {

// Collect dynamic (non-static) rigid bodies in registry iteration order. This
// is the same view and order the translational contact Jacobian uses, so the
// state/control vectors line up with getStepDerivatives()'s [q; q̇] layout.
std::vector<entt::entity> collectDynamicRigidBodies(
    const entt::registry& registry)
{
  std::vector<entt::entity> bodies;
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::Transform,
      comps::Velocity,
      comps::MassProperties,
      comps::Force>();
  for (const auto entity : view) {
    if (registry.all_of<comps::StaticBodyTag>(entity)) {
      continue;
    }
    bodies.push_back(entity);
  }
  return bodies;
}

} // namespace

//==============================================================================
std::size_t World::getNumDofs() const
{
  return 3 * collectDynamicRigidBodies(m_storage->registry).size();
}

//==============================================================================
std::size_t World::getNumEfforts() const
{
  return getNumDofs();
}

//==============================================================================
Eigen::VectorXd World::getStateVector() const
{
  const std::vector<entt::entity> bodies
      = collectDynamicRigidBodies(m_storage->registry);
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  Eigen::VectorXd state(2 * dofs);
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    const auto& transform
        = m_storage->registry.get<comps::Transform>(bodies[k]);
    const auto& velocity = m_storage->registry.get<comps::Velocity>(bodies[k]);
    state.segment<3>(base) = transform.position;
    state.segment<3>(dofs + base) = velocity.linear;
  }
  return state;
}

//==============================================================================
void World::setStateVector(const Eigen::VectorXd& state)
{
  const std::vector<entt::entity> bodies
      = collectDynamicRigidBodies(m_storage->registry);
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  DART_EXPERIMENTAL_THROW_T_IF(
      state.size() != 2 * dofs,
      InvalidArgumentException,
      "World::setStateVector(): expected size {} (= 2 * num_dofs) but got {}",
      2 * dofs,
      state.size());
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    auto& transform = m_storage->registry.get<comps::Transform>(bodies[k]);
    auto& velocity = m_storage->registry.get<comps::Velocity>(bodies[k]);
    transform.position = state.segment<3>(base);
    velocity.linear = state.segment<3>(dofs + base);
  }
}

//==============================================================================
Eigen::VectorXd World::getControlVector() const
{
  const std::vector<entt::entity> bodies
      = collectDynamicRigidBodies(m_storage->registry);
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  Eigen::VectorXd control(dofs);
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    const auto& force = m_storage->registry.get<comps::Force>(bodies[k]);
    control.segment<3>(base) = force.force;
  }
  return control;
}

//==============================================================================
void World::setControlVector(const Eigen::VectorXd& control)
{
  const std::vector<entt::entity> bodies
      = collectDynamicRigidBodies(m_storage->registry);
  const Eigen::Index dofs = 3 * static_cast<Eigen::Index>(bodies.size());
  DART_EXPERIMENTAL_THROW_T_IF(
      control.size() != dofs,
      InvalidArgumentException,
      "World::setControlVector(): expected size {} (= num_efforts) but got {}",
      dofs,
      control.size());
  for (std::size_t k = 0; k < bodies.size(); ++k) {
    const auto base = 3 * static_cast<Eigen::Index>(k);
    auto& force = m_storage->registry.get<comps::Force>(bodies[k]);
    force.force = control.segment<3>(base);
  }
}

//==============================================================================
void World::setTime(double time)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(time) || time < 0.0,
      InvalidArgumentException,
      "Time must be non-negative and finite");

  m_time = time;
}

//==============================================================================
double World::getTime() const noexcept
{
  return m_time;
}

//==============================================================================
std::size_t World::getFrame() const noexcept
{
  return m_frame;
}

//==============================================================================
void World::sync(WorldSyncStage stage)
{
  compute::SequentialExecutor executor;
  sync(stage, executor);
}

//==============================================================================
void World::sync(WorldSyncStage stage, compute::ComputeExecutor& executor)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "World::sync() requires simulation mode");

  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidWorldSyncStage(stage),
      InvalidArgumentException,
      "World::sync() stage is invalid");

  switch (stage) {
    case WorldSyncStage::Kinematics:
      validateLoopClosureKinematicsPolicySupport(*this);
      executeKinematicsGraph(*this, executor);
      return;
  }
}

//==============================================================================
void World::updateKinematics()
{
  sync(WorldSyncStage::Kinematics);
}

//==============================================================================
void World::updateKinematics(compute::ComputeExecutor& executor)
{
  sync(WorldSyncStage::Kinematics, executor);
}

//==============================================================================
void World::step()
{
  compute::SequentialExecutor executor;
  step(executor);
}

//==============================================================================
void World::step(std::size_t count)
{
  compute::SequentialExecutor executor;
  step(count, executor);
}

//==============================================================================
void World::setMultibodyOptions(const MultibodyOptions& options)
{
  const std::string& family = options.integrationFamily;
  if (family == "semi-implicit" || family == "semi-implicit Euler") {
    m_multibodyIntegrationMethod = MultibodyIntegrationMethod::SemiImplicit;
  } else if (family == "variational integrator" || family == "variational") {
    m_multibodyIntegrationMethod = MultibodyIntegrationMethod::Variational;
  } else {
    DART_EXPERIMENTAL_THROW_T(
        InvalidArgumentException,
        "Unknown multibody integrationFamily '{}'; supported method-family "
        "names are 'semi-implicit' and 'variational integrator'",
        family);
  }
}

//==============================================================================
MultibodyOptions World::getMultibodyOptions() const
{
  MultibodyOptions options;
  options.integrationFamily
      = m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational
            ? "variational integrator"
            : "semi-implicit";
  return options;
}

//==============================================================================
void World::step(compute::ComputeExecutor& executor)
{
  WorldStepPipelineStages stages;
  compute::WorldStepPipeline& pipeline = stages.buildDefault(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      hasMultibodyStructures(*this));
  step(executor, pipeline);
  m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
      stages.deformableDynamics.getLastStats());
}

//==============================================================================
void World::step(std::size_t count, compute::ComputeExecutor& executor)
{
  WorldStepPipelineStages stages;
  compute::WorldStepPipeline& pipeline = stages.buildDefault(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      hasMultibodyStructures(*this));
  step(count, executor, pipeline);
  m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
      stages.deformableDynamics.getLastStats());
}

//==============================================================================
void World::step(
    compute::ComputeExecutor& executor, compute::WorldStepStage& stage)
{
  WorldStepPipelineStages stages;
  compute::WorldStepPipeline& pipeline = stages.buildWithFinalStage(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      hasMultibodyStructures(*this),
      stage);
  step(executor, pipeline);
  m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
      stages.deformableDynamics.getLastStats());
}

//==============================================================================
void World::step(
    std::size_t count,
    compute::ComputeExecutor& executor,
    compute::WorldStepStage& stage)
{
  WorldStepPipelineStages stages;
  compute::WorldStepPipeline& pipeline = stages.buildWithFinalStage(
      m_rigidBodySolver,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational,
      hasMultibodyStructures(*this),
      stage);
  step(count, executor, pipeline);
  m_lastDeformableSolverDiagnostics = makeDeformableSolverDiagnostics(
      stages.deformableDynamics.getLastStats());
}

//==============================================================================
void World::step(
    compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline)
{
  validateLoopClosureKinematicsPolicySupport(*this);
  validateLoopClosureDynamicsPolicySupport(
      *this,
      m_multibodyIntegrationMethod == MultibodyIntegrationMethod::Variational);
  validateRigidBodyFixedJointPipelineSupport(*this, m_rigidBodySolver);

  if (!m_simulationMode) {
    enterSimulationMode();
  }

  // Differentiable opt-in: record the analytic contact-free step Jacobians at
  // the pre-step state before integration. This is a single predictable branch;
  // when off (the default) nothing extra runs and the forward result is
  // bitwise-identical.
  if (m_differentiable) {
    captureStepDerivatives();
  }

  pipeline.execute(*this, executor);

  m_time += m_timeStep;
  ++m_frame;
}

//==============================================================================
void World::captureStepDerivatives()
{
#ifdef DART_HAS_DIFF
  // Contact-aware path (PLAN-110 WS2): when the boxed-LCP contact solver is
  // selected, the differentiable step Jacobian must include the analytic
  // frictionless normal-contact gradient. Capture the active contacts at the
  // pre-step state from the same collide() source the BoxedLcp contact stage
  // consumes, validate they fall inside the WS2 slice's scope, then route
  // through detail::contactStepDerivatives(). When there are no active contacts
  // this reduces exactly to the contact-free (free-fall) Jacobian for the
  // dynamic rigid bodies; that is a mathematically exact reduction, not a
  // fallback.
  if (m_contactSolverMethod == ContactSolverMethod::BoxedLcp) {
    const std::vector<Contact> contacts = collide();

    // Scope guard: the contact gradient now covers Coulomb-friction rigid-body
    // contacts including those that excite the angular DOFs (lever arm not
    // parallel to the normal, e.g. an off-COM contact) and multiple
    // simultaneous contacts (e.g. a box resting on its corners). The full 6-DOF
    // Delassus solve and the screw-axis angular rows of J inside
    // detail::contactStepDerivatives() handle that coupling; the differentiated
    // output stays the body's translational [pos; linvel] (the single-step
    // angular state does not feed back into the translational output). The one
    // case still out of scope is an articulated-link (multibody) contact, which
    // the rigid-body contact assembly does not handle: reject it (rather than
    // silently returning a wrong matrix).
    for (const auto& contact : contacts) {
      const auto entityA = detail::toRegistryEntity(contact.bodyA.getEntity());
      const auto entityB = detail::toRegistryEntity(contact.bodyB.getEntity());

      const bool rigidA
          = m_storage->registry.all_of<comps::RigidBodyTag>(entityA);
      const bool rigidB
          = m_storage->registry.all_of<comps::RigidBodyTag>(entityB);
      DART_EXPERIMENTAL_THROW_T_IF(
          !rigidA || !rigidB,
          NotImplementedException,
          "World::getStepDerivatives(): differentiable contact gradient not "
          "yet supported for multibody/articulated-link contact; supported: "
          "rigid-body normal/friction contact (PLAN-110 WS2, incl. rotational "
          "and multi-contact)");
    }

    // When physical parameters are registered, additionally assemble the
    // parameter Jacobian ∂x'/∂θ alongside the state/control Jacobians;
    // otherwise skip the extra FD work and leave parameterJacobian empty.
    StepDerivatives contactDerivatives
        = m_storage->differentiableParameters.empty()
              ? detail::contactStepDerivatives(
                    m_storage->registry,
                    contacts,
                    m_gravity,
                    m_timeStep,
                    m_contactGradientMode)
              : detail::contactStepDerivativesWithParameters(
                    m_storage->registry,
                    contacts,
                    m_gravity,
                    m_timeStep,
                    m_storage->differentiableParameters,
                    m_contactGradientMode);
    // Non-empty only when a dynamic rigid body is in scope. When empty (e.g. a
    // pure multibody scene under BoxedLcp), fall through to the WS1 path below.
    if (contactDerivatives.stateJacobian.size() != 0) {
      m_storage->stepDerivatives = std::move(contactDerivatives);
      return;
    }
  }

  // WS1 covers a single contact-free multibody. The joint-type-keyed position
  // Jacobian in detail::contactFreeStepDerivatives covers fixed, revolute,
  // prismatic, screw, universal, planar, ball (Spherical), and free (Floating)
  // joints. Assemble tau from the joint efforts in construction (DOF) order and
  // compute the analytic Jacobian at the current (pre-step) state.
  auto view = m_storage->registry.view<comps::MultibodyStructure>();
  for (auto entity : view) {
    const auto& structure = view.get<comps::MultibodyStructure>(entity);

    Eigen::VectorXd tau;
    std::vector<double> torques;
    for (const auto linkEntity : structure.links) {
      const auto& link = m_storage->registry.get<comps::Link>(linkEntity);
      if (link.parentJoint == entt::null) {
        continue;
      }
      const auto& joint
          = m_storage->registry.get<comps::Joint>(link.parentJoint);
      for (Eigen::Index d = 0; d < joint.torque.size(); ++d) {
        torques.push_back(joint.torque[d]);
      }
    }
    if (torques.empty()) {
      continue;
    }
    tau = Eigen::Map<const Eigen::VectorXd>(
        torques.data(), static_cast<Eigen::Index>(torques.size()));

    m_storage->stepDerivatives = detail::contactFreeStepDerivatives(
        m_storage->registry, structure, m_gravity, m_timeStep, tau);
    return; // WS1: one multibody.
  }
#endif
}

//==============================================================================
double World::getRigidIpcAdaptiveBarrierStiffnessLowerBound() const noexcept
{
  return m_rigidIpcAdaptiveBarrierStiffnessLowerBound;
}

//==============================================================================
void World::setRigidIpcAdaptiveBarrierStiffnessLowerBound(
    const double value) noexcept
{
  m_rigidIpcAdaptiveBarrierStiffnessLowerBound
      = std::isfinite(value) && value > 0.0 ? value : 1.0;
}

//==============================================================================
void World::resetRigidIpcAdaptiveBarrierStiffnessLowerBound() noexcept
{
  m_rigidIpcAdaptiveBarrierStiffnessLowerBound = 1.0;
}

//==============================================================================
void World::step(
    std::size_t count,
    compute::ComputeExecutor& executor,
    compute::WorldStepPipeline& pipeline)
{
  for (std::size_t i = 0; i < count; ++i) {
    step(executor, pipeline);
  }
}

//==============================================================================
const DeformableSolverDiagnostics& World::getLastDeformableSolverDiagnostics()
    const
{
  return m_lastDeformableSolverDiagnostics;
}

//==============================================================================
std::vector<Contact> World::collide()
{
  return collide(CollisionQueryOptions{});
}

//==============================================================================
std::vector<Contact> World::collide(const CollisionQueryOptions& options)
{
  struct ShapeEntrySpec
  {
    CollisionQueryCache::Key key;
    const CollisionShape* shape;
    Eigen::Isometry3d pose;
  };
  std::vector<ShapeEntrySpec> specs;

  const auto findMultibodyOwningLink = [&](entt::entity linkEntity) {
    auto view = m_storage->registry.view<comps::MultibodyStructure>();
    for (auto multibody : view) {
      const auto& structure = view.get<comps::MultibodyStructure>(multibody);
      if (std::find(structure.links.begin(), structure.links.end(), linkEntity)
          != structure.links.end()) {
        return multibody;
      }
    }
    return static_cast<entt::entity>(entt::null);
  };

  const auto makeNativeShape =
      [](const CollisionShape& collisionShape) -> std::unique_ptr<ncol::Shape> {
    std::unique_ptr<ncol::Shape> shape;
    switch (collisionShape.type) {
      case CollisionShapeType::Sphere:
        shape = std::make_unique<ncol::SphereShape>(collisionShape.radius);
        break;
      case CollisionShapeType::Box:
        shape = std::make_unique<ncol::BoxShape>(collisionShape.halfExtents);
        break;
      case CollisionShapeType::Capsule:
        shape = std::make_unique<ncol::CapsuleShape>(
            collisionShape.radius, 2.0 * collisionShape.halfExtents.z());
        break;
      case CollisionShapeType::Cylinder:
        shape = std::make_unique<ncol::CylinderShape>(
            collisionShape.radius, 2.0 * collisionShape.halfExtents.z());
        break;
      case CollisionShapeType::Plane:
        shape = std::make_unique<ncol::PlaneShape>(
            collisionShape.normal, collisionShape.offset);
        break;
      case CollisionShapeType::Mesh:
        shape = std::make_unique<ncol::MeshShape>(
            collisionShape.vertices, collisionShape.triangles);
        break;
    }
    return shape;
  };

  const auto addSpecs = [&](entt::entity entity,
                            entt::entity multibody,
                            bool isLink,
                            const comps::CollisionGeometry& geometry,
                            const Eigen::Isometry3d& pose) {
    for (std::size_t i = 0; i < geometry.shapes.size(); ++i) {
      const auto& shape = geometry.shapes[i];
      specs.push_back(
          ShapeEntrySpec{
              CollisionQueryCache::Key{
                  entity, i, geometry.revision, multibody, isLink},
              &shape,
              pose * shape.localTransform});
    }
  };

  const auto includesPair = [&](const CollisionQueryCache::ObjectEntry& a,
                                const CollisionQueryCache::ObjectEntry& b) {
    if (a.entity == b.entity) {
      return false;
    }

    if (a.isLink && b.isLink) {
      return options.includeLinkPairs
             && (options.includeSameMultibodyLinkPairs
                 || a.multibody == entt::null || a.multibody != b.multibody);
    }

    if (a.isLink || b.isLink) {
      return options.includeRigidBodyLinkPairs;
    }

    return options.includeRigidBodyPairs;
  };

  // Rigid bodies pose their collision shapes from the rigid-body transform.
  auto rigidBodyView = m_storage->registry.view<
      comps::CollisionGeometry,
      comps::Transform,
      comps::RigidBodyTag>();
  for (auto entity : rigidBodyView) {
    const auto& geometry = rigidBodyView.get<comps::CollisionGeometry>(entity);
    const auto& transform = rigidBodyView.get<comps::Transform>(entity);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = transform.orientation.normalized().toRotationMatrix();
    pose.translation() = transform.position;
    addSpecs(entity, entt::null, false, geometry, pose);
  }

  // Multibody links pose their collision shapes through the frame accessor so
  // dirty joint-driven caches are refreshed before the query.
  auto linkView
      = m_storage->registry
            .view<comps::CollisionGeometry, comps::Link, comps::FrameCache>();
  for (auto entity : linkView) {
    const auto& geometry = linkView.get<comps::CollisionGeometry>(entity);
    const Link link(detail::fromRegistryEntity(entity), this);
    const entt::entity multibody = findMultibodyOwningLink(entity);
    addSpecs(entity, multibody, true, geometry, link.getWorldTransform());
  }

  if (!m_collisionQueryCache) {
    m_collisionQueryCache = std::make_unique<CollisionQueryCache>();
  }
  auto& cache = *m_collisionQueryCache;

  const bool rebuildCache = cache.keys.size() != specs.size()
                            || !std::equal(
                                specs.begin(),
                                specs.end(),
                                cache.keys.begin(),
                                [](const ShapeEntrySpec& spec,
                                   const CollisionQueryCache::Key& key) {
                                  return spec.key == key;
                                });

  if (rebuildCache) {
    cache.clear();
    cache.collisionWorld.reserveObjects(specs.size());
    cache.keys.reserve(specs.size());
    cache.entries.reserve(specs.size());
    for (const auto& spec : specs) {
      ncol::CollisionObject object = cache.collisionWorld.createObject(
          makeNativeShape(*spec.shape), spec.pose);
      const std::size_t entryIndex = cache.entries.size();
      const std::size_t objectId = object.getId();
      if (objectId >= cache.entryByObjectId.size()) {
        cache.entryByObjectId.resize(
            objectId + 1, std::numeric_limits<std::size_t>::max());
      }
      cache.entryByObjectId[objectId] = entryIndex;
      cache.keys.push_back(spec.key);
      cache.entries.push_back(
          CollisionQueryCache::ObjectEntry{
              spec.key.entity, spec.key.multibody, spec.key.isLink, object});
    }
  } else {
    for (std::size_t i = 0; i < specs.size(); ++i) {
      auto& object = cache.entries[i].object;
      object.setTransform(specs[i].pose);
      cache.collisionWorld.updateObject(object);
    }
  }

  // Broad-phase-pruned narrow-phase queries. Each candidate pair's bodies are
  // known here, so the contacts map back to the right experimental bodies
  // without relying on the result carrying object identity.
  const auto option = ncol::CollisionOption::fullContacts();
  const auto candidatePairs = cache.collisionWorld.buildBroadPhaseSnapshot();
  std::vector<Contact> contacts;
  for (const auto& pair : candidatePairs.pairs) {
    if (pair.first >= cache.entryByObjectId.size()
        || pair.second >= cache.entryByObjectId.size()) {
      continue;
    }
    const std::size_t i = cache.entryByObjectId[pair.first];
    const std::size_t j = cache.entryByObjectId[pair.second];
    if (i >= cache.entries.size() || j >= cache.entries.size()) {
      continue;
    }
    if (!includesPair(cache.entries[i], cache.entries[j])) {
      continue;
    }

    ncol::CollisionResult result;
    if (!cache.collisionWorld.collide(
            cache.entries[i].object, cache.entries[j].object, option, result)) {
      continue;
    }

    for (std::size_t k = 0; k < result.numContacts(); ++k) {
      const auto& point = result.getContact(k);
      // The native narrow phase reports the normal pointing from the second
      // object toward the first; the public Contact convention points from
      // bodyA (entries[i]) toward bodyB (entries[j]), so negate it.
      contacts.push_back(
          Contact{
              CollisionBody(
                  detail::fromRegistryEntity(cache.entries[i].entity), this),
              CollisionBody(
                  detail::fromRegistryEntity(cache.entries[j].entity), this),
              point.position,
              -point.normal,
              point.depth,
              specs[i].key.shapeIndex,
              specs[j].key.shapeIndex,
              specs[i].pose.inverse() * point.position,
              specs[j].pose.inverse() * point.position});
    }
  }

  return contacts;
}

//==============================================================================
void World::saveBinary(std::ostream& output) const
{
  io::writeFormatHeader(output);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().saveAllEntities(
      output, m_storage->registry, entityMap);

  const std::uint8_t simulationFlag = m_simulationMode ? 1 : 0;
  io::writePOD(output, simulationFlag);
  io::writePOD(output, m_freeFrameCounter);
  io::writePOD(output, m_fixedFrameCounter);
  io::writePOD(output, m_multibodyCounter);
  io::writePOD(output, m_rigidBodyCounter);
  io::writePOD(output, m_linkCounter);
  io::writePOD(output, m_jointCounter);
  io::writePOD(output, m_timeStep);
  io::writePOD(output, m_time);
  io::writePOD(output, m_frame);
  io::writePOD(output, m_gravity.x());
  io::writePOD(output, m_gravity.y());
  io::writePOD(output, m_gravity.z());
  io::writePOD(output, m_deformableBodyCounter);

  const std::uint8_t differentiableFlag = m_differentiable ? 1 : 0;
  io::writePOD(output, differentiableFlag);
}

//==============================================================================
void World::loadBinary(std::istream& input)
{
  clear();

  const auto formatVersion = io::readFormatHeader(input);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().loadAllEntities(
      input, m_storage->registry, entityMap, formatVersion);

  // World metadata (optional for forward-compatibility)
  if (input.peek() != std::char_traits<char>::eof()) {
    std::uint8_t simulationFlag = 0;
    io::readPOD(input, simulationFlag);
    m_simulationMode = simulationFlag != 0;

    io::readPOD(input, m_freeFrameCounter);
    io::readPOD(input, m_fixedFrameCounter);
    io::readPOD(input, m_multibodyCounter);
    io::readPOD(input, m_rigidBodyCounter);
    io::readPOD(input, m_linkCounter);
    io::readPOD(input, m_jointCounter);

    if (input.peek() != std::char_traits<char>::eof()) {
      io::readPOD(input, m_timeStep);
      io::readPOD(input, m_time);
      io::readPOD(input, m_frame);
    }

    if (formatVersion >= 2) {
      double gravityX = 0.0;
      double gravityY = 0.0;
      double gravityZ = 0.0;
      io::readPOD(input, gravityX);
      io::readPOD(input, gravityY);
      io::readPOD(input, gravityZ);
      m_gravity = Eigen::Vector3d(gravityX, gravityY, gravityZ);
    }

    if (input.peek() != std::char_traits<char>::eof()) {
      io::readPOD(input, m_deformableBodyCounter);
    }

    if (formatVersion >= 6 && input.peek() != std::char_traits<char>::eof()) {
      std::uint8_t differentiableFlag = 0;
      io::readPOD(input, differentiableFlag);
      m_differentiable = differentiableFlag != 0;
    }
  }

  // Ensure all frame entities have cache components (not serialized)
  auto frameView = m_storage->registry.view<comps::FrameTag>();
  for (auto entity : frameView) {
    if (!m_storage->registry.any_of<comps::FrameCache>(entity)) {
      auto& cache = m_storage->registry.emplace<comps::FrameCache>(entity);
      cache.worldTransform = Eigen::Isometry3d::Identity();
      cache.needTransformUpdate = true;
    } else {
      auto& cache = m_storage->registry.get<comps::FrameCache>(entity);
      cache.needTransformUpdate = true;
    }
  }

  resetCountersFromRegistry();

  if (m_simulationMode) {
    updateKinematics();
    detail::deformable_vbd::configureAvbdRigidWorldFixedJointsFromCurrentPoses(
        m_storage->registry);
  }
}

//==============================================================================
void World::resetCountersFromRegistry()
{
  m_freeFrameCounter = std::max(
      m_freeFrameCounter,
      countEntities<comps::FreeFrameTag>(m_storage->registry));
  m_fixedFrameCounter = std::max(
      m_fixedFrameCounter,
      countEntities<comps::FixedFrameTag>(m_storage->registry));
  m_multibodyCounter = std::max(
      m_multibodyCounter,
      countEntities<comps::MultibodyTag>(m_storage->registry));
  m_loopClosureCounter = std::max(
      m_loopClosureCounter,
      countEntities<comps::LoopClosure>(m_storage->registry));
  m_rigidBodyCounter = std::max(
      m_rigidBodyCounter,
      countEntities<comps::RigidBodyTag>(m_storage->registry));
  m_deformableBodyCounter = std::max(
      m_deformableBodyCounter,
      countEntities<comps::DeformableBodyTag>(m_storage->registry));
  m_linkCounter = std::max(
      m_linkCounter, countEntities<comps::Link>(m_storage->registry));
  m_jointCounter = std::max(
      m_jointCounter, countEntities<comps::Joint>(m_storage->registry));
}

} // namespace dart::simulation::experimental
