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

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/collision_geometry.hpp"
#include "dart/simulation/comps/deformable_body.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/frame_types.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/compute_executor.hpp"
#include "dart/simulation/compute/detail/deformable_avbd_replay_state.hpp"
#include "dart/simulation/compute/detail/stage_scratch.hpp"
#include "dart/simulation/compute/detail/world_step_stages.hpp"
#include "dart/simulation/detail/deformable_contact/barrier_kernel.hpp"
#include "dart/simulation/detail/deformable_contact/candidate_set.hpp"
#include "dart/simulation/detail/deformable_contact/continuous_collision_step.hpp"
#include "dart/simulation/detail/deformable_contact/tangent_stencil.hpp"
#include "dart/simulation/detail/deformable_elasticity/fem_tet_element.hpp"
#include "dart/simulation/detail/deformable_vbd/attachment_kernel.hpp"
#include "dart/simulation/detail/deformable_vbd/avbd_row_inventory.hpp"
#include "dart/simulation/detail/deformable_vbd/block_descent.hpp"
#include "dart/simulation/detail/deformable_vbd/parallel_block_descent.hpp"
#include "dart/simulation/detail/entity_conversion.hpp"
#include "dart/simulation/detail/newton_barrier/friction_kernel.hpp"
#include "dart/simulation/detail/newton_barrier/mixed_domain_coupling.hpp"
#include "dart/simulation/detail/newton_barrier/projected_newton.hpp"
#include "dart/simulation/detail/newton_barrier/psd_backend.hpp"
#include "dart/simulation/detail/newton_barrier/restitution_damping.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/world.hpp"

#include <dart/common/memory_manager.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <entt/entt.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart::simulation::compute {

namespace dc = dart::simulation::detail::deformable_contact;
namespace dvbd = dart::simulation::detail::deformable_vbd;
namespace fem = dart::simulation::detail::deformable_elasticity;
namespace nb = dart::simulation::detail::newton_barrier;
namespace sxdetail = dart::simulation::detail;

namespace {

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
std::optional<comps::Transform> collisionShapeWorldTransform(
    const comps::Transform& bodyTransform, const CollisionShape& shape)
{
  if (!bodyTransform.position.allFinite()
      || !bodyTransform.orientation.coeffs().allFinite()
      || !shape.localTransform.matrix().allFinite()) {
    return std::nullopt;
  }

  const Eigen::Quaterniond bodyOrientation
      = normalizeOrIdentity(bodyTransform.orientation);
  const Eigen::Quaterniond shapeOrientation(shape.localTransform.linear());

  comps::Transform result;
  result.position = bodyTransform.position
                    + bodyOrientation.toRotationMatrix()
                          * shape.localTransform.translation();
  result.orientation = bodyOrientation * shapeOrientation.normalized();
  result.orientation.normalize();
  return result;
}

//==============================================================================

} // namespace

//==============================================================================
/// Transient cached topology for the experimental Vertex Block Descent inner
/// solver: the spring and tetrahedron element lists plus the vertex-graph
/// coloring (over the union of springs and tets) and the per-element incident
/// adjacencies, rebuilt only when the element topology changes. Defined here
/// (not in comps) so the heavy kernel headers stay out of the component
/// headers; it is stored per body and never serialized.
struct DeformableVbdScratch
{
  template <typename Row>
  struct AvbdFrictionWarmStartRecord
  {
    using RowType = Row;

    dvbd::AvbdScalarRowKey key;
    Row row;
  };

  using ContactFeatureIdVector
      = std::vector<std::uint64_t, common::StlAllocator<std::uint64_t>>;
  using ContactPlaneAllocator = common::StlAllocator<dvbd::ContactPlane>;
  using ContactPlaneVector
      = std::vector<dvbd::ContactPlane, ContactPlaneAllocator>;
  using ByteVector
      = std::vector<std::uint8_t, common::StlAllocator<std::uint8_t>>;
  using Vector3Vector
      = std::vector<Eigen::Vector3d, common::StlAllocator<Eigen::Vector3d>>;
  using SurfaceTriangleVector = std::vector<
      comps::DeformableSurfaceTriangle,
      common::StlAllocator<comps::DeformableSurfaceTriangle>>;
  using AvbdDescriptorVector = std::vector<
      dvbd::AvbdScalarRowDescriptor,
      common::StlAllocator<dvbd::AvbdScalarRowDescriptor>>;

  template <typename Row>
  using AvbdFrictionWarmStartVector = std::vector<
      AvbdFrictionWarmStartRecord<Row>,
      common::StlAllocator<AvbdFrictionWarmStartRecord<Row>>>;
  template <typename Row>
  using AvbdRowVector = std::vector<Row, common::StlAllocator<Row>>;
  using SpringVector = std::
      vector<dvbd::SpringElement, common::StlAllocator<dvbd::SpringElement>>;
  using TetAllocator = common::StlAllocator<dvbd::TetMeshElement>;
  using TetVector = std::vector<dvbd::TetMeshElement, TetAllocator>;

  DeformableVbdScratch() = default;

  explicit DeformableVbdScratch(common::MemoryAllocator& allocator)
    : memoryAllocator(&allocator),
      springs(common::StlAllocator<dvbd::SpringElement>{allocator}),
      tets(common::StlAllocator<dvbd::TetMeshElement>{allocator}),
      cachedRestPositions(common::StlAllocator<Eigen::Vector3d>{allocator}),
      cachedSurfaceTriangles(
          common::StlAllocator<comps::DeformableSurfaceTriangle>{allocator}),
      coloring(allocator),
      springAdjacency(allocator),
      tetAdjacency(allocator),
      contactPlanes(common::StlAllocator<dvbd::ContactPlane>{allocator}),
      contactObjectIds(common::StlAllocator<std::uint64_t>{allocator}),
      contactFeatureIds(common::StlAllocator<std::uint64_t>{allocator}),
      avbdContactDescriptors(
          common::StlAllocator<dvbd::AvbdScalarRowDescriptor>{allocator}),
      avbdContactInventory(allocator),
      avbdContactRows(
          common::StlAllocator<dvbd::AvbdHalfSpaceContactRow>{allocator}),
      avbdFrictionDescriptors(
          common::StlAllocator<dvbd::AvbdScalarRowDescriptor>{allocator}),
      avbdFrictionInventory(allocator),
      avbdFrictionAnchors(allocator),
      avbdFrictionRows(
          common::StlAllocator<dvbd::AvbdHalfSpaceFrictionRow>{allocator}),
      previousAvbdFrictionWarmStarts(
          common::StlAllocator<
              AvbdFrictionWarmStartRecord<dvbd::AvbdHalfSpaceFrictionRow>>{
              allocator}),
      avbdSelfContactDescriptors(
          common::StlAllocator<dvbd::AvbdScalarRowDescriptor>{allocator}),
      avbdSelfContactInventory(allocator),
      avbdSelfContactRows(
          common::StlAllocator<dvbd::AvbdSelfContactNormalRow>{allocator}),
      avbdSelfContactFrictionDescriptors(
          common::StlAllocator<dvbd::AvbdScalarRowDescriptor>{allocator}),
      avbdSelfContactFrictionInventory(allocator),
      avbdSelfContactFrictionAnchors(allocator),
      avbdSelfContactFrictionRows(
          common::StlAllocator<dvbd::AvbdSelfContactFrictionRow>{allocator}),
      previousAvbdSelfContactFrictionWarmStarts(
          common::StlAllocator<
              AvbdFrictionWarmStartRecord<dvbd::AvbdSelfContactFrictionRow>>{
              allocator}),
      avbdAttachmentDescriptors(
          common::StlAllocator<dvbd::AvbdScalarRowDescriptor>{allocator}),
      avbdAttachmentInventory(allocator),
      avbdAttachmentRows(
          common::StlAllocator<dvbd::AvbdPointAttachmentRow>{allocator}),
      avbdSpringDescriptors(
          common::StlAllocator<dvbd::AvbdScalarRowDescriptor>{allocator}),
      avbdSpringInventory(allocator),
      avbdSpringRows(
          common::StlAllocator<dvbd::AvbdSpringFiniteStiffnessRow>{allocator}),
      avbdSolveFixed(common::StlAllocator<std::uint8_t>{allocator}),
      selfContactCandidates(allocator),
      selfContactSweepScratch(allocator),
      selfContactAdjacency(allocator),
      chebyshevTwoStepsBack(common::StlAllocator<Eigen::Vector3d>{allocator}),
      chebyshevBeforeSweep(common::StlAllocator<Eigen::Vector3d>{allocator})
  {
  }

  common::MemoryAllocator* memoryAllocator
      = &common::MemoryAllocator::GetDefault();
  SpringVector springs;
  TetVector tets;
  Vector3Vector cachedRestPositions;
  SurfaceTriangleVector cachedSurfaceTriangles;
  dvbd::VertexColoring coloring;
  dvbd::SpringAdjacency springAdjacency;
  dvbd::TetAdjacency tetAdjacency;
  // Per-vertex static ground-contact planes, rebuilt each step from the barrier
  // set at the warm-start position (lagged); a zero stiffness marks "no ground
  // under this vertex".
  ContactPlaneVector contactPlanes;
  // Stable static-contact feature IDs parallel to `contactPlanes`, used by the
  // AVBD row inventory so a vertex contact against ground, sphere, or box
  // features warm-starts only against the same static feature.
  ContactFeatureIdVector contactObjectIds;
  ContactFeatureIdVector contactFeatureIds;
  AvbdDescriptorVector avbdContactDescriptors;
  dvbd::AvbdScalarRowInventory avbdContactInventory;
  AvbdRowVector<dvbd::AvbdHalfSpaceContactRow> avbdContactRows;
  AvbdDescriptorVector avbdFrictionDescriptors;
  dvbd::AvbdScalarRowInventory avbdFrictionInventory;
  dvbd::AvbdHalfSpaceTangentAnchorInventory avbdFrictionAnchors;
  AvbdRowVector<dvbd::AvbdHalfSpaceFrictionRow> avbdFrictionRows;
  AvbdFrictionWarmStartVector<dvbd::AvbdHalfSpaceFrictionRow>
      previousAvbdFrictionWarmStarts;
  AvbdDescriptorVector avbdSelfContactDescriptors;
  dvbd::AvbdScalarRowInventory avbdSelfContactInventory;
  AvbdRowVector<dvbd::AvbdSelfContactNormalRow> avbdSelfContactRows;
  AvbdDescriptorVector avbdSelfContactFrictionDescriptors;
  dvbd::AvbdScalarRowInventory avbdSelfContactFrictionInventory;
  dvbd::AvbdSelfContactTangentAnchorInventory avbdSelfContactFrictionAnchors;
  AvbdRowVector<dvbd::AvbdSelfContactFrictionRow> avbdSelfContactFrictionRows;
  AvbdFrictionWarmStartVector<dvbd::AvbdSelfContactFrictionRow>
      previousAvbdSelfContactFrictionWarmStarts;
  AvbdDescriptorVector avbdAttachmentDescriptors;
  dvbd::AvbdScalarRowInventory avbdAttachmentInventory;
  AvbdRowVector<dvbd::AvbdPointAttachmentRow> avbdAttachmentRows;
  AvbdDescriptorVector avbdSpringDescriptors;
  dvbd::AvbdScalarRowInventory avbdSpringInventory;
  AvbdRowVector<dvbd::AvbdSpringFiniteStiffnessRow> avbdSpringRows;
  ByteVector avbdSolveFixed;
  // Self-contact candidate set + per-vertex incident lists, rebuilt each step
  // (lagged) from the body's swept start-to-warm-start surface motion.
  dc::ContactCandidateSet selfContactCandidates;
  dc::detail::ContactCandidateSweepScratch selfContactSweepScratch;
  dvbd::SelfContactAdjacency selfContactAdjacency;
  Vector3Vector chebyshevTwoStepsBack;
  Vector3Vector chebyshevBeforeSweep;
  std::size_t cachedNodeCount = 0;
  std::size_t cachedEdgeCount = 0;
  std::size_t cachedTetCount = 0;
  bool cachedUseFiniteElementElasticity = false;
  bool initialized = false;
};

//==============================================================================
namespace {

void clearDeformableAvbdWarmStartRows(DeformableVbdScratch& scratch)
{
  scratch.avbdContactDescriptors.clear();
  scratch.avbdContactInventory.records().clear();
  scratch.avbdContactRows.clear();
  scratch.avbdFrictionDescriptors.clear();
  scratch.avbdFrictionInventory.records().clear();
  scratch.avbdFrictionAnchors.clear();
  scratch.avbdFrictionRows.clear();
  scratch.previousAvbdFrictionWarmStarts.clear();
  scratch.avbdSelfContactDescriptors.clear();
  scratch.avbdSelfContactInventory.records().clear();
  scratch.avbdSelfContactRows.clear();
  scratch.avbdSelfContactFrictionDescriptors.clear();
  scratch.avbdSelfContactFrictionInventory.records().clear();
  scratch.avbdSelfContactFrictionAnchors.clear();
  scratch.avbdSelfContactFrictionRows.clear();
  scratch.previousAvbdSelfContactFrictionWarmStarts.clear();
  scratch.avbdAttachmentDescriptors.clear();
  scratch.avbdAttachmentInventory.records().clear();
  scratch.avbdAttachmentRows.clear();
  scratch.avbdSpringDescriptors.clear();
  scratch.avbdSpringInventory.records().clear();
  scratch.avbdSpringRows.clear();
  scratch.avbdSolveFixed.clear();
}

//==============================================================================
void clearDeformableAvbdMaterialWarmStartRows(DeformableVbdScratch& scratch)
{
  scratch.avbdSpringDescriptors.clear();
  scratch.avbdSpringInventory.records().clear();
  scratch.avbdSpringRows.clear();
}

//==============================================================================
void clearDeformableAvbdSelfContactWarmStart(DeformableVbdScratch& scratch)
{
  scratch.avbdSelfContactDescriptors.clear();
  scratch.avbdSelfContactInventory.records().clear();
  scratch.avbdSelfContactRows.clear();
  scratch.avbdSelfContactFrictionDescriptors.clear();
  scratch.avbdSelfContactFrictionInventory.records().clear();
  scratch.avbdSelfContactFrictionAnchors.clear();
  scratch.avbdSelfContactFrictionRows.clear();
  scratch.previousAvbdSelfContactFrictionWarmStarts.clear();
  scratch.selfContactCandidates.surfaceEdges.clear();
  scratch.selfContactCandidates.pointTriangleCandidates.clear();
  scratch.selfContactCandidates.edgeEdgeCandidates.clear();
  scratch.selfContactCandidates.stats = {};
  scratch.selfContactAdjacency.clear();
}

//==============================================================================
void clearDeformableAvbdAttachmentWarmStart(DeformableVbdScratch& scratch)
{
  scratch.avbdAttachmentDescriptors.clear();
  scratch.avbdAttachmentInventory.records().clear();
  scratch.avbdAttachmentRows.clear();
}

//==============================================================================
void rebuildAvbdFrictionWarmStartLookup(
    auto& lookup, const auto& descriptors, const auto& rows)
{
  lookup.clear();
  const std::size_t rowCount = std::min(descriptors.size(), rows.size());
  lookup.reserve(rowCount);
  for (std::size_t i = 0; i < rowCount; ++i) {
    lookup.push_back({descriptors[i].key, rows[i]});
  }
  std::sort(lookup.begin(), lookup.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.key < rhs.key;
  });
}

//==============================================================================
template <typename Lookup>
[[nodiscard]] const typename Lookup::value_type::RowType*
findAvbdFrictionWarmStartRow(
    const Lookup& lookup, const dvbd::AvbdScalarRowKey& key)
{
  using Row = typename Lookup::value_type::RowType;
  const auto match = std::lower_bound(
      lookup.begin(),
      lookup.end(),
      key,
      [](const auto& record, const dvbd::AvbdScalarRowKey& value) {
        return record.key < value;
      });
  if (match != lookup.end() && match->key == key) {
    return &match->row;
  }
  return static_cast<const Row*>(nullptr);
}

using AvbdTangentBasis = Eigen::Matrix<double, 3, 2>;

//==============================================================================
[[nodiscard]] bool validAvbdTangentBasis(const AvbdTangentBasis& basis)
{
  constexpr double tolerance = 1e-8;
  return basis.allFinite()
         && std::abs(basis.col(0).squaredNorm() - 1.0) <= tolerance
         && std::abs(basis.col(1).squaredNorm() - 1.0) <= tolerance
         && std::abs(basis.col(0).dot(basis.col(1))) <= tolerance;
}

//==============================================================================
[[nodiscard]] bool sameAvbdVector(
    const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
  return (lhs.array() == rhs.array()).all();
}

//==============================================================================
template <std::size_t Size>
[[nodiscard]] bool sameAvbdVectorArray(
    const std::array<Eigen::Vector3d, Size>& lhs,
    const std::array<Eigen::Vector3d, Size>& rhs)
{
  for (std::size_t i = 0; i < Size; ++i) {
    if (!sameAvbdVector(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
}

//==============================================================================
template <std::size_t Size>
[[nodiscard]] bool finiteAvbdVectorArray(
    const std::array<Eigen::Vector3d, Size>& vectors)
{
  for (const Eigen::Vector3d& vector : vectors) {
    if (!vector.allFinite()) {
      return false;
    }
  }
  return true;
}

//==============================================================================
template <typename Anchor>
[[nodiscard]] AvbdTangentBasis avbdAnchorBasis(const Anchor& anchor)
{
  AvbdTangentBasis basis;
  basis.col(0) = anchor.basis[0];
  basis.col(1) = anchor.basis[1];
  return basis;
}

//==============================================================================
template <typename Anchor>
void storeAvbdAnchorBasis(Anchor& anchor, const AvbdTangentBasis& basis)
{
  anchor.basis[0] = basis.col(0);
  anchor.basis[1] = basis.col(1);
}

//==============================================================================
template <typename Anchor>
[[nodiscard]] Eigen::Vector3d transportAvbdTangentDisplacement(
    const Anchor& anchor, const AvbdTangentBasis& currentBasis)
{
  const AvbdTangentBasis previousBasis = avbdAnchorBasis(anchor);
  if (!anchor.valid || !anchor.accumulatedDisplacement.allFinite()
      || !validAvbdTangentBasis(previousBasis)
      || !validAvbdTangentBasis(currentBasis)) {
    return Eigen::Vector3d::Zero();
  }
  return currentBasis
         * (previousBasis.transpose() * anchor.accumulatedDisplacement);
}

//==============================================================================
void persistAvbdHalfSpaceFrictionAnchors(
    const dvbd::AvbdScalarRowInventory& rowInventory,
    dvbd::AvbdHalfSpaceTangentAnchorInventory& anchorInventory,
    std::span<const dvbd::AvbdHalfSpaceFrictionRow> rows,
    std::span<const Eigen::Vector3d> positions)
{
  for (std::size_t pair = 0;
       pair < anchorInventory.size() && 2u * pair + 1u < rowInventory.size()
       && 2u * pair + 1u < rows.size();
       ++pair) {
    const std::size_t i = 2u * pair;
    auto expectedSecondKey = rowInventory[i].descriptor.key;
    expectedSecondKey.axis = 1u;
    const auto& first = rows[i];
    const auto& second = rows[i + 1u];
    AvbdTangentBasis basis;
    basis.col(0) = first.axis;
    basis.col(1) = second.axis;
    const bool validPair
        = rowInventory[i].descriptor.key.role
              == dvbd::AvbdScalarRowRole::FrictionTangent
          && rowInventory[i].descriptor.kind
                 == dvbd::AvbdScalarRowKind::HardConstraint
          && rowInventory[i + 1u].descriptor.kind
                 == dvbd::AvbdScalarRowKind::HardConstraint
          && rowInventory[i].descriptor.key.axis == 0u
          && rowInventory[i + 1u].descriptor.key == expectedSecondKey
          && anchorInventory[pair].key
                 == dvbd::makeAvbdDeformableTangentAnchorKey(
                     rowInventory[i].descriptor.key)
          && first.vertex == second.vertex && first.vertex < positions.size()
          && first.normalRow == second.normalRow
          && first.frictionCoefficient == second.frictionCoefficient
          && sameAvbdVector(first.stepStartPosition, second.stepStartPosition)
          && sameAvbdVector(
              first.accumulatedTangentialDisplacement,
              second.accumulatedTangentialDisplacement)
          && first.accumulatedTangentialDisplacement.allFinite()
          && first.sticking == second.sticking && validAvbdTangentBasis(basis);
    if (!validPair) {
      const auto key = anchorInventory[pair].key;
      anchorInventory[pair] = {};
      anchorInventory[pair].key = key;
      continue;
    }

    dvbd::AvbdHalfSpaceTangentAnchorState anchor;
    anchor.key = anchorInventory[pair].key;
    anchor.valid = true;
    anchor.sticking = first.sticking && second.sticking;
    anchor.vertex = first.vertex;
    storeAvbdAnchorBasis(anchor, basis);
    if (anchor.sticking) {
      const Eigen::Vector3d total
          = dvbd::avbdHalfSpaceFrictionTotalTangentialDisplacement(
              first, positions[first.vertex]);
      if (!total.allFinite()) {
        const auto key = anchorInventory[pair].key;
        anchorInventory[pair] = {};
        anchorInventory[pair].key = key;
        continue;
      }
      anchor.accumulatedDisplacement = basis * (basis.transpose() * total);
    }
    anchorInventory[pair] = anchor;
  }
}

//==============================================================================
void persistAvbdSelfContactFrictionAnchors(
    const dvbd::AvbdScalarRowInventory& rowInventory,
    dvbd::AvbdSelfContactTangentAnchorInventory& anchorInventory,
    std::span<const dvbd::AvbdSelfContactFrictionRow> rows,
    std::span<const Eigen::Vector3d> positions)
{
  for (std::size_t pair = 0;
       pair < anchorInventory.size() && 2u * pair + 1u < rowInventory.size()
       && 2u * pair + 1u < rows.size();
       ++pair) {
    const std::size_t i = 2u * pair;
    auto expectedSecondKey = rowInventory[i].descriptor.key;
    expectedSecondKey.axis = 1u;
    const auto& first = rows[i];
    const auto& second = rows[i + 1u];
    if (first.differentialSuspended && second.differentialSuspended) {
      // The owning normal primitive had no safe differential. The AVBD row
      // solve preserved the complete coupled continuation, so retain the
      // previously persisted material-point anchor byte-for-byte as well.
      continue;
    }
    const AvbdTangentBasis basis = dvbd::avbdSelfContactFrictionBasis(first);
    const bool validPair
        = rowInventory[i].descriptor.key.role
              == dvbd::AvbdScalarRowRole::FrictionTangent
          && rowInventory[i].descriptor.kind
                 == dvbd::AvbdScalarRowKind::HardConstraint
          && rowInventory[i + 1u].descriptor.kind
                 == dvbd::AvbdScalarRowKind::HardConstraint
          && rowInventory[i].descriptor.key.axis == 0u
          && rowInventory[i + 1u].descriptor.key == expectedSecondKey
          && anchorInventory[pair].key
                 == dvbd::makeAvbdDeformableTangentAnchorKey(
                     rowInventory[i].descriptor.key)
          && first.axis == 0u && second.axis == 1u
          && dvbd::avbdSelfContactSameFrictionPrimitive(first, second)
          && first.normalRow == second.normalRow
          && first.frictionCoefficient == second.frictionCoefficient
          && sameAvbdVectorArray(
              first.stepStartPositions, second.stepStartPositions)
          && sameAvbdVector(
              first.accumulatedTangentialDisplacement,
              second.accumulatedTangentialDisplacement)
          && first.accumulatedTangentialDisplacement.allFinite()
          && first.sticking == second.sticking && validAvbdTangentBasis(basis);
    if (!validPair) {
      const auto key = anchorInventory[pair].key;
      anchorInventory[pair] = {};
      anchorInventory[pair].key = key;
      continue;
    }

    dvbd::AvbdSelfContactTangentAnchorState anchor;
    anchor.key = anchorInventory[pair].key;
    anchor.valid = true;
    anchor.sticking = first.sticking && second.sticking;
    anchor.nodes = first.nodes;
    anchor.isEdgeEdge = first.isEdgeEdge;
    anchor.referencePositions = first.stepStartPositions;
    storeAvbdAnchorBasis(anchor, basis);
    if (anchor.sticking) {
      const Eigen::Vector3d total
          = dvbd::avbdSelfContactFrictionTotalTangentialDisplacement(
              first, positions);
      if (!total.allFinite()) {
        const auto key = anchorInventory[pair].key;
        anchorInventory[pair] = {};
        anchorInventory[pair].key = key;
        continue;
      }
      anchor.accumulatedDisplacement = basis * (basis.transpose() * total);
    }
    anchorInventory[pair] = anchor;
  }
}

} // namespace

namespace {

//==============================================================================
template <typename StateVector>
void captureDeformableAvbdWarmStartReplayStateInto(
    const detail::WorldRegistry& registry,
    common::MemoryAllocator& allocator,
    StateVector& states)
{
  using ReplayState = avbd_replay::DeformableAvbdWarmStartReplayState;
  const auto copyRowRecords = [&allocator](const auto& records) {
    ReplayState::RowVector copy(ReplayState::RowAllocator{allocator});
    copy.assign(records.begin(), records.end());
    return copy;
  };

  auto view = registry.view<DeformableVbdScratch>();
  std::size_t stateCount = 0;
  for (auto entity : view) {
    static_cast<void>(entity);
    ++stateCount;
  }
  states.reserve(stateCount);
  for (auto entity : view) {
    const auto& scratch = view.get<DeformableVbdScratch>(entity);
    ReplayState state(allocator);
    state.entity = entity;
    state.contactRows = copyRowRecords(scratch.avbdContactInventory.records());
    state.frictionRows
        = copyRowRecords(scratch.avbdFrictionInventory.records());
    state.frictionAnchors.assign(
        scratch.avbdFrictionAnchors.records().begin(),
        scratch.avbdFrictionAnchors.records().end());
    state.selfContactRows
        = copyRowRecords(scratch.avbdSelfContactInventory.records());
    state.selfContactFrictionRows
        = copyRowRecords(scratch.avbdSelfContactFrictionInventory.records());
    state.selfContactFrictionAnchors.assign(
        scratch.avbdSelfContactFrictionAnchors.records().begin(),
        scratch.avbdSelfContactFrictionAnchors.records().end());
    state.attachmentRows
        = copyRowRecords(scratch.avbdAttachmentInventory.records());
    state.springRows = copyRowRecords(scratch.avbdSpringInventory.records());
    states.push_back(std::move(state));
  }

  std::ranges::sort(states, [](const auto& lhs, const auto& rhs) {
    return static_cast<std::uint32_t>(lhs.entity)
           < static_cast<std::uint32_t>(rhs.entity);
  });
}

} // namespace

//==============================================================================
bool avbd_replay::hasDeformableAvbdWarmStartContinuationState(
    const detail::WorldRegistry& registry) noexcept
{
  for (const auto entity : registry.view<DeformableVbdScratch>()) {
    const auto& scratch = registry.get<DeformableVbdScratch>(entity);
    if (!scratch.avbdContactInventory.records().empty()
        || !scratch.avbdFrictionInventory.records().empty()
        || !scratch.avbdFrictionAnchors.records().empty()
        || !scratch.avbdSelfContactInventory.records().empty()
        || !scratch.avbdSelfContactFrictionInventory.records().empty()
        || !scratch.avbdSelfContactFrictionAnchors.records().empty()
        || !scratch.avbdAttachmentInventory.records().empty()
        || !scratch.avbdSpringInventory.records().empty()) {
      return true;
    }
  }
  return false;
}

//==============================================================================
std::vector<avbd_replay::DeformableAvbdWarmStartReplayState>
avbd_replay::captureDeformableAvbdWarmStartReplayState(
    const detail::WorldRegistry& registry)
{
  std::vector<avbd_replay::DeformableAvbdWarmStartReplayState> states;
  captureDeformableAvbdWarmStartReplayStateInto(
      registry, common::MemoryAllocator::GetDefault(), states);
  return states;
}

//==============================================================================
avbd_replay::AllocatedDeformableAvbdWarmStartReplayStates
avbd_replay::captureDeformableAvbdWarmStartReplayState(
    const detail::WorldRegistry& registry, common::MemoryAllocator& allocator)
{
  AllocatedDeformableAvbdWarmStartReplayStates states(
      common::StlAllocator<DeformableAvbdWarmStartReplayState>{allocator});
  captureDeformableAvbdWarmStartReplayStateInto(registry, allocator, states);
  return states;
}

//==============================================================================
void avbd_replay::restoreDeformableAvbdWarmStartReplayState(
    detail::WorldRegistry& registry,
    std::span<const avbd_replay::DeformableAvbdWarmStartReplayState>
        replayStates)
{
  restoreDeformableAvbdWarmStartReplayState(
      registry, replayStates, common::MemoryAllocator::GetDefault());
}

//==============================================================================
void avbd_replay::restoreDeformableAvbdWarmStartReplayState(
    detail::WorldRegistry& registry,
    std::span<const avbd_replay::DeformableAvbdWarmStartReplayState>
        replayStates,
    common::MemoryAllocator& allocator)
{
  // Replay states are produced by the internal capture path. Restore validates
  // pair layout and key identity here; the next friction-row projection also
  // validates finite anchor payload, basis, and live topology/stencil before
  // using it, cold-starting both tangent duals and the anchor on mismatch.
  const auto validFrictionAnchorPairs = [](const auto& rows,
                                           const auto& anchors) {
    if (rows.size() != 2u * anchors.size()) {
      return false;
    }
    for (std::size_t pair = 0; pair < anchors.size(); ++pair) {
      const auto& first = rows[2u * pair];
      const auto& second = rows[2u * pair + 1u];
      auto expectedSecondKey = first.descriptor.key;
      expectedSecondKey.axis = 1u;
      if (first.descriptor.key.role != dvbd::AvbdScalarRowRole::FrictionTangent
          || first.descriptor.kind != dvbd::AvbdScalarRowKind::HardConstraint
          || second.descriptor.kind != dvbd::AvbdScalarRowKind::HardConstraint
          || first.descriptor.key.axis != 0u
          || second.descriptor.key != expectedSecondKey
          || anchors[pair].key
                 != dvbd::makeAvbdDeformableTangentAnchorKey(
                     first.descriptor.key)) {
        return false;
      }
    }
    return true;
  };
  for (const auto& state : replayStates) {
    DART_SIMULATION_THROW_T_IF(
        !registry.valid(state.entity)
            || !registry.all_of<comps::DeformableBodyTag>(state.entity)
            || !validFrictionAnchorPairs(
                state.frictionRows, state.frictionAnchors)
            || !validFrictionAnchorPairs(
                state.selfContactFrictionRows,
                state.selfContactFrictionAnchors),
        InvalidOperationException,
        "Cannot restore replay frame: deformable AVBD entity or friction "
        "anchor layout changed");
  }

  for (auto entity : registry.view<DeformableVbdScratch>()) {
    clearDeformableAvbdWarmStartRows(
        registry.get<DeformableVbdScratch>(entity));
  }

  for (const auto& state : replayStates) {
    auto& scratch = registry.get_or_emplace<DeformableVbdScratch>(
        state.entity, allocator);
    clearDeformableAvbdWarmStartRows(scratch);
    const auto assignRowRecords
        = [](dvbd::AvbdScalarRowInventory& inventory, const auto& records) {
            inventory.records().assign(records.begin(), records.end());
          };
    assignRowRecords(scratch.avbdContactInventory, state.contactRows);
    assignRowRecords(scratch.avbdFrictionInventory, state.frictionRows);
    scratch.avbdFrictionAnchors.records().assign(
        state.frictionAnchors.begin(), state.frictionAnchors.end());
    assignRowRecords(scratch.avbdSelfContactInventory, state.selfContactRows);
    assignRowRecords(
        scratch.avbdSelfContactFrictionInventory,
        state.selfContactFrictionRows);
    scratch.avbdSelfContactFrictionAnchors.records().assign(
        state.selfContactFrictionAnchors.begin(),
        state.selfContactFrictionAnchors.end());
    assignRowRecords(scratch.avbdAttachmentInventory, state.attachmentRows);
    assignRowRecords(scratch.avbdSpringInventory, state.springRows);
  }
}

//==============================================================================
void avbd_replay::restoreDeformableAvbdWarmStartReplayStateNoAlloc(
    detail::WorldRegistry& registry,
    std::span<avbd_replay::DeformableAvbdWarmStartReplayState>
        replayStates) noexcept
{
  for (auto entity : registry.view<DeformableVbdScratch>()) {
    clearDeformableAvbdWarmStartRows(
        registry.get<DeformableVbdScratch>(entity));
  }

  for (auto& state : replayStates) {
    // This component existed when the rollback frame was captured. The normal
    // restore path clears or populates it but never removes it, so direct
    // access here cannot require EnTT storage growth.
    auto& scratch = registry.get<DeformableVbdScratch>(state.entity);
    scratch.avbdContactInventory.records() = std::move(state.contactRows);
    scratch.avbdFrictionInventory.records() = std::move(state.frictionRows);
    scratch.avbdFrictionAnchors.records() = std::move(state.frictionAnchors);
    scratch.avbdSelfContactInventory.records()
        = std::move(state.selfContactRows);
    scratch.avbdSelfContactFrictionInventory.records()
        = std::move(state.selfContactFrictionRows);
    scratch.avbdSelfContactFrictionAnchors.records()
        = std::move(state.selfContactFrictionAnchors);
    scratch.avbdAttachmentInventory.records() = std::move(state.attachmentRows);
    scratch.avbdSpringInventory.records() = std::move(state.springRows);
  }
}

namespace {

//==============================================================================
//==============================================================================
//==============================================================================
struct StaticGroundBarrier
{
  enum class Shape
  {
    Box,
    Sphere,
  };

  Shape shape = Shape::Box;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Zero();
  double radius = 0.0;
  double top = 0.0;
  std::uint64_t objectId = 0;
  std::uint64_t geometryRevision = 0;
};

//==============================================================================
// A static sphere obstacle that exerts a full radial clamped-log barrier force
// on nearby deformable nodes (unlike the vertical-only ground barrier above),
// so a deformable settles smoothly at ~d_hat against any side of the sphere.
struct SphereObstacleBarrier
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
  std::uint64_t objectId = 0;
  std::uint64_t geometryRevision = 0;
};

// A static (oriented) box opted in as a deformable obstacle exerts a
// clamped-log barrier on nearby deformable nodes along the outward surface
// normal -- the box analogue of the sphere obstacle barrier. Outside the box,
// the closest point on the box surface is found by clamping the node into the
// box's local frame, which handles face/edge/corner contact uniformly. Inside
// the box, the nearest exit face supplies the outward normal and penetration
// depth.
struct BoxObstacleBarrier
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Zero();
  std::uint64_t objectId = 0;
  std::uint64_t geometryRevision = 0;
};

//==============================================================================
// Geometry revisions are part of persistent AVBD row identity so a runtime
// shape edit cannot inherit the old constraint's multiplier or stiffness. Six
// low bits hold every box feature code and the seventh distinguishes the
// ground-height role from the full surface-obstacle role for dual-tag bodies.
std::uint64_t staticObstacleFeatureId(
    std::uint64_t geometryRevision,
    std::uint64_t featureCode = 0,
    bool surfaceObstacleRole = false)
{
  return (geometryRevision << 7u)
         | (surfaceObstacleRole ? std::uint64_t{0x40u} : std::uint64_t{0u})
         | (featureCode & 0x3fu);
}

// A static capsule (a segment swept by a radius -- a rod/wire) opted in as a
// deformable obstacle exerts a clamped-log barrier on nearby deformable nodes
// along the outward radial normal. The closest point on the capsule surface is
// found from the point-to-segment closest point, so the distance is
// |node - closestOnSegment| - radius. This is the codimensional (1D) obstacle
// analogue of the sphere (0D point) and box obstacles.
struct CapsuleObstacleBarrier
{
  Eigen::Vector3d pointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d pointB = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

//==============================================================================
// A lagged self-contact friction contact: the four stencil nodes, the lagged
// normal-force magnitude, and the tangent projection (2x12) that maps the
// stacked four-node displacement to tangential relative motion. Computed once
// per outer iteration (standard IPC lagging) at the current iterate.
struct SelfContactFrictionContact
{
  std::array<std::size_t, 4> nodes{};
  double normalForce = 0.0;
  dc::Matrix2x12d projection = dc::Matrix2x12d::Zero();
};

//==============================================================================
struct ProjectedNewtonMatrixFreeBlock3
{
  std::size_t rowNode = 0;
  std::size_t colNode = 0;
  Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
};

using ProjectedNewtonIntVector = std::vector<int, common::StlAllocator<int>>;
using ProjectedNewtonTripletVector = std::vector<
    Eigen::Triplet<double>,
    common::StlAllocator<Eigen::Triplet<double>>>;
using ProjectedNewtonScalarVector
    = std::vector<double, common::StlAllocator<double>>;
using ProjectedNewtonEdgeNodeVector = std::vector<
    std::array<std::size_t, 2>,
    common::StlAllocator<std::array<std::size_t, 2>>>;
using ProjectedNewtonTetNodeVector = std::vector<
    std::array<std::size_t, 4>,
    common::StlAllocator<std::array<std::size_t, 4>>>;
using ProjectedNewtonMatrixFreeBlockVector = std::vector<
    ProjectedNewtonMatrixFreeBlock3,
    common::StlAllocator<ProjectedNewtonMatrixFreeBlock3>>;
using ProjectedNewtonMatrix3Vector
    = std::vector<Eigen::Matrix3d, common::StlAllocator<Eigen::Matrix3d>>;
using ProjectedNewtonFemRestShapeVector
    = std::vector<fem::TetRestShape, common::StlAllocator<fem::TetRestShape>>;
using ProjectedNewtonVector3Vector
    = std::vector<Eigen::Vector3d, common::StlAllocator<Eigen::Vector3d>>;
using ProjectedNewtonFrictionContactVector = std::vector<
    SelfContactFrictionContact,
    common::StlAllocator<SelfContactFrictionContact>>;

constexpr Eigen::Index kProjectedNewtonDenseDirectDofCap = 128;

//==============================================================================
bool usesProjectedNewtonSparseIterativePath(
    Eigen::Index dim, const comps::DeformableMaterial& material)
{
  return dim > 0 && !material.useMatrixFreeLinearSolver
         && (material.useIterativeLinearSolver
             || dim > kProjectedNewtonDenseDirectDofCap);
}

//==============================================================================
struct DeformableContactSolverScratch
{
  using SurfaceTriangleVector = std::vector<
      DeformableSurfaceTriangle,
      common::StlAllocator<DeformableSurfaceTriangle>>;
  using SurfaceContactPointMaskVector
      = std::vector<std::uint8_t, common::StlAllocator<std::uint8_t>>;
  using SurfaceEdgeVector
      = std::vector<dc::SurfaceEdge, common::StlAllocator<dc::SurfaceEdge>>;
  using SizeVector
      = std::vector<std::size_t, common::StlAllocator<std::size_t>>;
  using SweepItemVector = std::vector<
      dc::detail::SweepItem,
      common::StlAllocator<dc::detail::SweepItem>>;
  using SweepLinkVector
      = std::vector<std::size_t, common::StlAllocator<std::size_t>>;

  DeformableContactSolverScratch() = default;

  explicit DeformableContactSolverScratch(common::MemoryAllocator& allocator)
    : surfaceTriangles(
          common::StlAllocator<DeformableSurfaceTriangle>{allocator}),
      surfaceContactPointMask(common::StlAllocator<std::uint8_t>{allocator}),
      surfaceEdgeDegrees(common::StlAllocator<std::size_t>{allocator}),
      candidates(allocator),
      barrierCandidates(allocator),
      sweepScratch(allocator),
      interBodyCurrentEdges(common::StlAllocator<dc::SurfaceEdge>{allocator}),
      interBodyCurrentPointItems(
          common::StlAllocator<dc::detail::SweepItem>{allocator}),
      interBodyObstaclePointItems(
          common::StlAllocator<dc::detail::SweepItem>{allocator}),
      interBodyCurrentTriangleItems(
          common::StlAllocator<dc::detail::SweepItem>{allocator}),
      interBodyObstacleTriangleItems(
          common::StlAllocator<dc::detail::SweepItem>{allocator}),
      interBodyCurrentEdgeItems(
          common::StlAllocator<dc::detail::SweepItem>{allocator}),
      interBodyObstacleEdgeItems(
          common::StlAllocator<dc::detail::SweepItem>{allocator}),
      interBodySweepLinks(common::StlAllocator<std::size_t>{allocator}),
      newtonPatternOuter(common::StlAllocator<int>{allocator}),
      newtonPatternInner(common::StlAllocator<int>{allocator}),
      femRestShapes(common::StlAllocator<fem::TetRestShape>{allocator}),
      cachedFemTetNodes(
          common::StlAllocator<std::array<std::size_t, 4>>{allocator}),
      cachedFemRestPositions(common::StlAllocator<Eigen::Vector3d>{allocator}),
      projectedNewtonTriplets(
          common::StlAllocator<Eigen::Triplet<double>>{allocator}),
      projectedNewtonEdgeBlocks(common::StlAllocator<double>{allocator}),
      projectedNewtonEdgeBlockNodes(
          common::StlAllocator<std::array<std::size_t, 2>>{allocator}),
      projectedNewtonTetBlocks(common::StlAllocator<double>{allocator}),
      projectedNewtonTetBlockNodes(
          common::StlAllocator<std::array<std::size_t, 4>>{allocator}),
      projectedNewtonBarrierBlocks(common::StlAllocator<double>{allocator}),
      projectedNewtonBarrierBlockNodes(
          common::StlAllocator<std::array<std::size_t, 4>>{allocator}),
      projectedNewtonMatrixFreeBlocks(
          common::StlAllocator<ProjectedNewtonMatrixFreeBlock3>{allocator}),
      projectedNewtonMatrixFreeDiagonalBlocks(
          common::StlAllocator<Eigen::Matrix3d>{allocator}),
      projectedNewtonMatrixFreeInverseDiagonalBlocks(
          common::StlAllocator<Eigen::Matrix3d>{allocator}),
      groundFrictionNormalForce(common::StlAllocator<double>{allocator}),
      groundFrictionNormalDirection(
          common::StlAllocator<Eigen::Vector3d>{allocator}),
      selfContactFrictionContacts(
          common::StlAllocator<SelfContactFrictionContact>{allocator})
  {
  }

  SurfaceTriangleVector surfaceTriangles;
  SurfaceContactPointMaskVector surfaceContactPointMask;
  SizeVector surfaceEdgeDegrees;
  dc::ContactCandidateSet candidates;
  // Self-contact barrier active set, assembled once per outer solver iteration
  // at the current positions (within the barrier activation distance d_hat).
  dc::ContactCandidateSet barrierCandidates;
  dc::detail::ContactCandidateSweepScratch sweepScratch;
  SurfaceEdgeVector interBodyCurrentEdges;
  SweepItemVector interBodyCurrentPointItems;
  SweepItemVector interBodyObstaclePointItems;
  SweepItemVector interBodyCurrentTriangleItems;
  SweepItemVector interBodyObstacleTriangleItems;
  SweepItemVector interBodyCurrentEdgeItems;
  SweepItemVector interBodyObstacleEdgeItems;
  SweepLinkVector interBodySweepLinks;

  // Persistent projected-Newton sparse Hessian storage. The cached column/row
  // index arrays describe the assembled Hessian pattern so sparse iterative
  // rows can refill values after bake without rebuilding Eigen storage.
  ProjectedNewtonIntVector newtonPatternOuter;
  ProjectedNewtonIntVector newtonPatternInner;
  bool hessianPatternValid = false;

  // Cached per-tetrahedron FEM rest shapes (inverse rest edge matrix + rest
  // volume), plus the exact inputs that produced them. Replay restoration and
  // internal model tooling can replace a topology without changing its size,
  // so a count-only cache key would silently retain the old material model.
  ProjectedNewtonFemRestShapeVector femRestShapes;
  ProjectedNewtonTetNodeVector cachedFemTetNodes;
  ProjectedNewtonVector3Vector cachedFemRestPositions;

  Eigen::VectorXd projectedNewtonRhs;
  Eigen::VectorXd projectedNewtonSolution;
  Eigen::MatrixXd projectedNewtonDenseHessian;
  Eigen::LDLT<Eigen::MatrixXd> projectedNewtonDenseLdlt;
  Eigen::SparseMatrix<double> projectedNewtonHessian;
  ProjectedNewtonTripletVector projectedNewtonTriplets;
  ProjectedNewtonScalarVector projectedNewtonEdgeBlocks;
  ProjectedNewtonEdgeNodeVector projectedNewtonEdgeBlockNodes;
  ProjectedNewtonScalarVector projectedNewtonTetBlocks;
  ProjectedNewtonTetNodeVector projectedNewtonTetBlockNodes;
  ProjectedNewtonScalarVector projectedNewtonBarrierBlocks;
  ProjectedNewtonTetNodeVector projectedNewtonBarrierBlockNodes;
  ProjectedNewtonMatrixFreeBlockVector projectedNewtonMatrixFreeBlocks;
  ProjectedNewtonMatrix3Vector projectedNewtonMatrixFreeDiagonalBlocks;
  ProjectedNewtonMatrix3Vector projectedNewtonMatrixFreeInverseDiagonalBlocks;
  Eigen::VectorXd projectedNewtonMatrixFreeResidual;
  Eigen::VectorXd projectedNewtonMatrixFreePreconditionedResidual;
  Eigen::VectorXd projectedNewtonMatrixFreeDirection;
  Eigen::VectorXd projectedNewtonMatrixFreeHessianDirection;
  Eigen::VectorXd projectedNewtonIterativeInverseDiagonal;

  ProjectedNewtonScalarVector groundFrictionNormalForce;
  ProjectedNewtonVector3Vector groundFrictionNormalDirection;
  ProjectedNewtonFrictionContactVector selfContactFrictionContacts;

  // Construction policy and deterministic baked capacities. One combined
  // budget covers point-triangle plus edge-edge candidates for every builder.
  std::size_t requestedSurfaceCandidateCapacity = 0u;
  std::size_t resolvedSurfaceCandidateCapacity = 0u;
  std::size_t selfSurfaceCandidateCapacity = 0u;
  /// True only for the automatic policy of a topology whose exact valid-pair
  /// bound exceeds the reserve budget: builds may then exceed the resolved
  /// capacity (allocating) instead of failing closed.
  bool surfaceCandidateGrowthAllowed = false;
  std::size_t bakedSurfaceCandidateCount = 0u;
  bool surfaceCandidateCapacityPrepared = false;
};

//==============================================================================
bool tryAssembleProjectedNewtonHessianFromCachedPattern(
    DeformableContactSolverScratch& scratch, Eigen::Index dim)
{
  auto& hessian = scratch.projectedNewtonHessian;
  const auto& triplets = scratch.projectedNewtonTriplets;
  const auto& cachedOuter = scratch.newtonPatternOuter;
  const auto& cachedInner = scratch.newtonPatternInner;
  if (!scratch.hessianPatternValid || hessian.rows() != dim
      || hessian.cols() != dim || !hessian.isCompressed()
      || cachedOuter.size() != static_cast<std::size_t>(dim + 1)
      || hessian.nonZeros() != static_cast<Eigen::Index>(cachedInner.size())) {
    return false;
  }

  using SparseMatrix = Eigen::SparseMatrix<double>;
  using StorageIndex = SparseMatrix::StorageIndex;
  const StorageIndex* outer = hessian.outerIndexPtr();
  const StorageIndex* inner = hessian.innerIndexPtr();
  double* values = hessian.valuePtr();
  if (outer == nullptr || inner == nullptr || values == nullptr) {
    return false;
  }

  for (Eigen::Index col = 0; col <= dim; ++col) {
    if (outer[col] != cachedOuter[static_cast<std::size_t>(col)]) {
      return false;
    }
  }
  for (Eigen::Index entry = 0; entry < hessian.nonZeros(); ++entry) {
    if (inner[entry] != cachedInner[static_cast<std::size_t>(entry)]) {
      return false;
    }
  }

  std::fill(values, values + hessian.nonZeros(), 0.0);
  for (const auto& triplet : triplets) {
    const Eigen::Index row = triplet.row();
    const Eigen::Index col = triplet.col();
    if (row < 0 || row >= dim || col < 0 || col >= dim) {
      return false;
    }
    const auto begin = outer[col];
    const auto end = outer[col + 1];
    const StorageIndex rowIndex = static_cast<StorageIndex>(row);
    const StorageIndex* it
        = std::lower_bound(inner + begin, inner + end, rowIndex);
    if (it == inner + end || *it != rowIndex) {
      return false;
    }
    values[it - inner] += triplet.value();
  }

  return true;
}

//==============================================================================
void cacheProjectedNewtonHessianPattern(DeformableContactSolverScratch& scratch)
{
  auto& hessian = scratch.projectedNewtonHessian;
  if (!hessian.isCompressed()) {
    hessian.makeCompressed();
  }

  const auto cols = hessian.cols();
  const auto nnz = hessian.nonZeros();
  const int* outer = hessian.outerIndexPtr();
  const int* inner = hessian.innerIndexPtr();
  if (outer == nullptr || inner == nullptr || cols < 0 || nnz < 0) {
    scratch.hessianPatternValid = false;
    return;
  }

  scratch.newtonPatternOuter.assign(outer, outer + cols + 1);
  scratch.newtonPatternInner.assign(inner, inner + nnz);
  scratch.hessianPatternValid = true;
}

//==============================================================================
struct ProjectedNewtonMatrixFreeHessian
{
  ProjectedNewtonMatrixFreeHessian(
      ProjectedNewtonMatrixFreeBlockVector& blockStorage,
      ProjectedNewtonMatrix3Vector& diagonalStorage,
      ProjectedNewtonMatrix3Vector& inverseDiagonalStorage)
    : blocks(blockStorage),
      diagonalBlocks(diagonalStorage),
      inverseDiagonalBlocks(inverseDiagonalStorage)
  {
  }

  void reset(std::size_t nodes)
  {
    nodeCount = nodes;
    blocks.clear();
    diagonalBlocks.resize(nodeCount);
    for (auto& block : diagonalBlocks) {
      block.setZero();
    }
  }

  void addBlock3(
      std::size_t rowNode, std::size_t colNode, const Eigen::Matrix3d& block)
  {
    blocks.push_back({rowNode, colNode, block});
    if (rowNode == colNode) {
      diagonalBlocks[rowNode] += block;
    }
  }

  void multiplyInto(const Eigen::VectorXd& x, Eigen::VectorXd& y) const
  {
    y.resize(static_cast<Eigen::Index>(3 * nodeCount));
    y.setZero();
    for (const auto& entry : blocks) {
      y.segment<3>(static_cast<Eigen::Index>(3 * entry.rowNode))
          += entry.block
             * x.segment<3>(static_cast<Eigen::Index>(3 * entry.colNode));
    }
  }

  [[nodiscard]] bool factorBlockJacobi()
  {
    inverseDiagonalBlocks.resize(nodeCount);
    for (std::size_t i = 0; i < nodeCount; ++i) {
      Eigen::LLT<Eigen::Matrix3d> llt(diagonalBlocks[i]);
      if (llt.info() != Eigen::Success) {
        return false;
      }
      inverseDiagonalBlocks[i] = llt.solve(Eigen::Matrix3d::Identity()).eval();
      if (!inverseDiagonalBlocks[i].allFinite()) {
        return false;
      }
    }
    return true;
  }

  void applyPreconditionerInto(
      const Eigen::VectorXd& residual, Eigen::VectorXd& z) const
  {
    z.resize(static_cast<Eigen::Index>(3 * nodeCount));
    for (std::size_t i = 0; i < nodeCount; ++i) {
      z.segment<3>(static_cast<Eigen::Index>(3 * i))
          = inverseDiagonalBlocks[i]
            * residual.segment<3>(static_cast<Eigen::Index>(3 * i));
    }
  }

  std::size_t nodeCount = 0;
  ProjectedNewtonMatrixFreeBlockVector& blocks;
  ProjectedNewtonMatrix3Vector& diagonalBlocks;
  ProjectedNewtonMatrix3Vector& inverseDiagonalBlocks;
};

//==============================================================================
bool solveMatrixFreeConjugateGradient(
    ProjectedNewtonMatrixFreeHessian& hessian,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution,
    Eigen::VectorXd& residual,
    Eigen::VectorXd& z,
    Eigen::VectorXd& direction,
    Eigen::VectorXd& hessianDirection,
    std::size_t& iterations,
    double& relativeResidual)
{
  constexpr double kTolerance = 1e-8;
  const Eigen::Index maxIterations = 2 * rhs.size();
  solution.resize(rhs.size());
  solution.setZero();
  iterations = 0;
  relativeResidual = 0.0;

  if (!hessian.factorBlockJacobi()) {
    return false;
  }

  const double rhsNorm = rhs.norm();
  if (rhsNorm == 0.0) {
    return true;
  }

  residual.resize(rhs.size());
  residual = rhs;
  hessian.applyPreconditionerInto(residual, z);
  direction.resize(rhs.size());
  direction = z;
  double rz = residual.dot(z);
  if (!std::isfinite(rz) || rz <= 0.0 || !direction.allFinite()) {
    return false;
  }

  for (Eigen::Index iter = 0; iter < maxIterations; ++iter) {
    hessian.multiplyInto(direction, hessianDirection);
    const double curvature = direction.dot(hessianDirection);
    if (!std::isfinite(curvature) || curvature <= 0.0) {
      return false;
    }

    const double alpha = rz / curvature;
    solution += alpha * direction;
    residual -= alpha * hessianDirection;
    if (!solution.allFinite() || !residual.allFinite()) {
      return false;
    }

    relativeResidual = residual.norm() / rhsNorm;
    iterations = static_cast<std::size_t>(iter + 1);
    if (relativeResidual <= kTolerance) {
      return true;
    }

    hessian.applyPreconditionerInto(residual, z);
    const double nextRz = residual.dot(z);
    if (!std::isfinite(nextRz) || nextRz <= 0.0 || !z.allFinite()) {
      return false;
    }
    const double beta = nextRz / rz;
    direction *= beta;
    direction += z;
    if (!direction.allFinite()) {
      return false;
    }
    rz = nextRz;
  }

  return false;
}

//==============================================================================
bool solveSparseJacobiConjugateGradient(
    const Eigen::SparseMatrix<double>& hessian,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution,
    Eigen::VectorXd& residual,
    Eigen::VectorXd& z,
    Eigen::VectorXd& direction,
    Eigen::VectorXd& hessianDirection,
    Eigen::VectorXd& inverseDiagonal,
    std::size_t& iterations,
    double& relativeResidual)
{
  constexpr double kTolerance = 1e-8;
  const Eigen::Index dim = rhs.size();
  const Eigen::Index maxIterations = 2 * dim;
  solution.resize(dim);
  solution.setZero();
  residual.resize(dim);
  z.resize(dim);
  direction.resize(dim);
  hessianDirection.resize(dim);
  inverseDiagonal.resize(dim);
  inverseDiagonal.setZero();
  iterations = 0;
  relativeResidual = 0.0;

  for (Eigen::Index outer = 0; outer < hessian.outerSize(); ++outer) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(hessian, outer); it;
         ++it) {
      if (it.row() == it.col()) {
        inverseDiagonal[it.row()] = it.value();
      }
    }
  }
  for (Eigen::Index i = 0; i < dim; ++i) {
    const double diagonal = inverseDiagonal[i];
    if (!std::isfinite(diagonal) || diagonal <= 0.0) {
      return false;
    }
    inverseDiagonal[i] = 1.0 / diagonal;
  }

  const double rhsNorm = rhs.norm();
  if (rhsNorm == 0.0) {
    return true;
  }

  residual = rhs;
  z = residual.cwiseProduct(inverseDiagonal);
  direction = z;
  double rz = residual.dot(z);
  if (!std::isfinite(rz) || rz <= 0.0 || !direction.allFinite()) {
    return false;
  }

  for (Eigen::Index iter = 0; iter < maxIterations; ++iter) {
    hessianDirection.setZero();
    for (Eigen::Index outer = 0; outer < hessian.outerSize(); ++outer) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(hessian, outer); it;
           ++it) {
        hessianDirection[it.row()] += it.value() * direction[it.col()];
      }
    }

    const double curvature = direction.dot(hessianDirection);
    if (!std::isfinite(curvature) || curvature <= 0.0) {
      return false;
    }

    const double alpha = rz / curvature;
    solution += alpha * direction;
    residual -= alpha * hessianDirection;
    if (!solution.allFinite() || !residual.allFinite()) {
      return false;
    }

    relativeResidual = residual.norm() / rhsNorm;
    iterations = static_cast<std::size_t>(iter + 1);
    if (relativeResidual <= kTolerance) {
      return true;
    }

    z = residual.cwiseProduct(inverseDiagonal);
    const double nextRz = residual.dot(z);
    if (!std::isfinite(nextRz) || nextRz <= 0.0 || !z.allFinite()) {
      return false;
    }
    const double beta = nextRz / rz;
    direction *= beta;
    direction += z;
    if (!direction.allFinite()) {
      return false;
    }
    rz = nextRz;
  }

  return false;
}

//==============================================================================
struct SurfaceContactSnapshot
{
  using PositionVector
      = std::vector<Eigen::Vector3d, common::StlAllocator<Eigen::Vector3d>>;
  using TriangleVector = std::vector<
      DeformableSurfaceTriangle,
      common::StlAllocator<DeformableSurfaceTriangle>>;
  using ContactPointMaskVector
      = std::vector<std::uint8_t, common::StlAllocator<std::uint8_t>>;
  using EdgeVector
      = std::vector<dc::SurfaceEdge, common::StlAllocator<dc::SurfaceEdge>>;

  SurfaceContactSnapshot() = default;

  explicit SurfaceContactSnapshot(common::MemoryAllocator& allocator)
    : positions(common::StlAllocator<Eigen::Vector3d>{allocator}),
      surfaceTriangles(
          common::StlAllocator<DeformableSurfaceTriangle>{allocator}),
      surfaceContactPointMask(common::StlAllocator<std::uint8_t>{allocator}),
      surfaceEdges(common::StlAllocator<dc::SurfaceEdge>{allocator})
  {
  }

  entt::entity entity = entt::null;
  PositionVector positions;
  TriangleVector surfaceTriangles;
  ContactPointMaskVector surfaceContactPointMask;
  EdgeVector surfaceEdges;
};

//==============================================================================
SurfaceContactSnapshot& nextSurfaceContactSnapshot(
    auto& snapshots,
    std::size_t& count,
    common::MemoryAllocator* payloadAllocator = nullptr)
{
  if (count == snapshots.size()) {
    if (payloadAllocator != nullptr) {
      snapshots.emplace_back(*payloadAllocator);
    } else {
      snapshots.emplace_back();
    }
  }
  return snapshots[count++];
}

//==============================================================================
std::span<const SurfaceContactSnapshot> activeSurfaceContactSnapshots(
    const auto& snapshots, std::size_t count)
{
  return std::span<const SurfaceContactSnapshot>(snapshots.data(), count);
}

//==============================================================================
struct InterBodySurfaceContactResult
{
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;
  std::size_t pointTrianglePairCapacity = 0;
  std::size_t edgeEdgePairCapacity = 0;
  std::size_t pointTriangleCandidateCount = 0;
  std::size_t edgeEdgeCandidateCount = 0;
  std::size_t capacityOverflowCount = 0;
  dc::ContinuousCollisionStepStats stats;
};

//==============================================================================
double surfaceContactMinSeparation()
{
  return 1e-4;
}

//==============================================================================
double surfaceContactTolerance()
{
  return 1e-6;
}

//==============================================================================
dc::ContactCandidateOptions makeSurfaceContactCandidateOptions(
    std::size_t candidateCapacity = std::numeric_limits<std::size_t>::max(),
    std::span<const std::uint8_t> pointMask = {},
    bool allowCapacityGrowth = false)
{
  dc::ContactCandidateOptions options;
  options.activationDistance
      = surfaceContactMinSeparation() + surfaceContactTolerance();
  options.exactDistanceFilter = true;
  options.excludeIncidentPointTriangles = true;
  options.excludeAdjacentEdges = true;
  options.pointMask = pointMask;
  options.candidateCapacity = candidateCapacity;
  options.allowCapacityGrowth = allowCapacityGrowth;
  return options;
}

//==============================================================================
dc::ContinuousCollisionStepOptions makeSurfaceContactCcdOptions()
{
  dc::ContinuousCollisionStepOptions options;
  options.minSeparation = surfaceContactMinSeparation();
  options.tolerance = surfaceContactTolerance();
  options.maxIterations = 128;
  return options;
}

//==============================================================================
void copySurfaceContactTopology(
    std::span<const comps::DeformableSurfaceTriangle> source,
    std::size_t nodeCount,
    bool restrictPointsToReferencedSurfaceNodes,
    auto& surfaceTriangles,
    auto& surfaceContactPointMask)
{
  surfaceTriangles.clear();
  surfaceTriangles.reserve(source.size());
  if (restrictPointsToReferencedSurfaceNodes) {
    surfaceContactPointMask.assign(nodeCount, 0u);
  } else {
    surfaceContactPointMask.clear();
  }

  for (const auto& triangle : source) {
    surfaceTriangles.push_back(
        DeformableSurfaceTriangle{
            triangle.nodeA, triangle.nodeB, triangle.nodeC});
    if (restrictPointsToReferencedSurfaceNodes) {
      surfaceContactPointMask[triangle.nodeA] = 1u;
      surfaceContactPointMask[triangle.nodeB] = 1u;
      surfaceContactPointMask[triangle.nodeC] = 1u;
    }
  }
}

//==============================================================================
void syncSurfaceContactTopology(
    std::span<const comps::DeformableSurfaceTriangle> source,
    std::size_t nodeCount,
    bool restrictPointsToReferencedSurfaceNodes,
    DeformableContactSolverScratch& scratch)
{
  copySurfaceContactTopology(
      source,
      nodeCount,
      restrictPointsToReferencedSurfaceNodes,
      scratch.surfaceTriangles,
      scratch.surfaceContactPointMask);
}

//==============================================================================
void filterSurfaceContactPointCandidates(
    dc::ContactCandidateSet& candidates,
    std::span<const std::uint8_t> pointMask)
{
  if (pointMask.empty()) {
    return;
  }

  auto& pointTriangleCandidates = candidates.pointTriangleCandidates;
  pointTriangleCandidates.erase(
      std::remove_if(
          pointTriangleCandidates.begin(),
          pointTriangleCandidates.end(),
          [&](const dc::PointTriangleCandidate& candidate) {
            return candidate.point >= pointMask.size()
                   || pointMask[candidate.point] == 0u;
          }),
      pointTriangleCandidates.end());
  candidates.stats.pointTriangleCandidateCount = pointTriangleCandidates.size();
}

//==============================================================================
bool surfaceContactPointAllowed(
    const std::size_t point, std::span<const std::uint8_t> pointMask)
{
  return pointMask.empty()
         || (point < pointMask.size() && pointMask[point] != 0u);
}

//==============================================================================
std::size_t countSurfaceContactCandidatePoints(
    const std::size_t pointCount, std::span<const std::uint8_t> pointMask)
{
  if (pointMask.empty()) {
    return pointCount;
  }

  const std::size_t limit = std::min(pointCount, pointMask.size());
  return static_cast<std::size_t>(std::count_if(
      pointMask.begin(),
      pointMask.begin() + limit,
      [](const std::uint8_t value) { return value != 0u; }));
}

//==============================================================================
std::size_t checkedSurfaceCandidateAdd(
    std::size_t lhs, std::size_t rhs, std::string_view context)
{
  DART_SIMULATION_THROW_T_IF(
      lhs > std::numeric_limits<std::size_t>::max() - rhs,
      InvalidArgumentException,
      "Deformable surface contact capacity arithmetic overflow ({})",
      context);
  return lhs + rhs;
}

//==============================================================================
std::size_t checkedSurfaceCandidateMultiply(
    std::size_t lhs, std::size_t rhs, std::string_view context)
{
  DART_SIMULATION_THROW_T_IF(
      lhs != 0u && rhs > std::numeric_limits<std::size_t>::max() / lhs,
      InvalidArgumentException,
      "Deformable surface contact capacity arithmetic overflow ({})",
      context);
  return lhs * rhs;
}

//==============================================================================
std::size_t checkedSurfaceCandidateChooseTwo(
    std::size_t count, std::string_view context)
{
  if (count < 2u) {
    return 0u;
  }
  const std::size_t even = (count % 2u == 0u) ? count : count - 1u;
  const std::size_t other = (count % 2u == 0u) ? count - 1u : count;
  return checkedSurfaceCandidateMultiply(even / 2u, other, context);
}

//==============================================================================
struct SurfaceCandidateTopologyCounts
{
  std::size_t points = 0u;
  std::size_t triangles = 0u;
  std::size_t edges = 0u;
  std::size_t validSelfPairs = 0u;
};

//==============================================================================
SurfaceCandidateTopologyCounts surfaceCandidateTopologyCounts(
    std::size_t nodeCount,
    std::span<const DeformableSurfaceTriangle> triangles,
    std::span<const std::uint8_t> pointMask,
    std::span<const dc::SurfaceEdge> edges,
    DeformableContactSolverScratch::SizeVector& edgeDegrees)
{
  SurfaceCandidateTopologyCounts counts;
  counts.points = countSurfaceContactCandidatePoints(nodeCount, pointMask);
  counts.triangles = triangles.size();
  counts.edges = edges.size();

  const std::size_t rawPointTriangle = checkedSurfaceCandidateMultiply(
      counts.points, counts.triangles, "self point-triangle pairs");
  const std::size_t incidentPointTriangle = checkedSurfaceCandidateMultiply(
      3u, counts.triangles, "incident point-triangle pairs");
  const std::size_t validPointTriangle
      = rawPointTriangle >= incidentPointTriangle
            ? rawPointTriangle - incidentPointTriangle
            : 0u;

  edgeDegrees.assign(nodeCount, 0u);
  for (const dc::SurfaceEdge& edge : edges) {
    DART_SIMULATION_THROW_T_IF(
        edge.nodeA >= nodeCount || edge.nodeB >= nodeCount,
        InvalidArgumentException,
        "Deformable surface edge references an out-of-range node");
    ++edgeDegrees[edge.nodeA];
    ++edgeDegrees[edge.nodeB];
  }
  std::size_t adjacentEdgePairs = 0u;
  for (const std::size_t degree : edgeDegrees) {
    adjacentEdgePairs = checkedSurfaceCandidateAdd(
        adjacentEdgePairs,
        checkedSurfaceCandidateChooseTwo(degree, "adjacent edge pairs"),
        "adjacent edge-pair sum");
  }
  const std::size_t rawEdgePairs
      = checkedSurfaceCandidateChooseTwo(counts.edges, "self edge pairs");
  const std::size_t validEdgePairs = rawEdgePairs >= adjacentEdgePairs
                                         ? rawEdgePairs - adjacentEdgePairs
                                         : 0u;
  counts.validSelfPairs = checkedSurfaceCandidateAdd(
      validPointTriangle, validEdgePairs, "valid self-contact pairs");
  return counts;
}

//==============================================================================
std::size_t interSurfaceCandidatePairBound(
    const SurfaceCandidateTopologyCounts& current,
    const SurfaceCandidateTopologyCounts& obstacle)
{
  const std::size_t currentPointsObstacleTriangles
      = checkedSurfaceCandidateMultiply(
          current.points,
          obstacle.triangles,
          "inter-surface point-triangle pairs");
  const std::size_t obstaclePointsCurrentTriangles
      = checkedSurfaceCandidateMultiply(
          obstacle.points,
          current.triangles,
          "inter-surface reverse point-triangle pairs");
  const std::size_t edgePairs = checkedSurfaceCandidateMultiply(
      current.edges, obstacle.edges, "inter-surface edge pairs");
  return checkedSurfaceCandidateAdd(
      checkedSurfaceCandidateAdd(
          currentPointsObstacleTriangles,
          obstaclePointsCurrentTriangles,
          "inter-surface point-triangle sum"),
      edgePairs,
      "inter-surface combined pairs");
}

//==============================================================================
// Largest exact valid-pair bound that the automatic policy reserves up front.
// Below this, a frozen topology can never overflow its reserved capacity, so
// steps stay allocation-free and fail-closed without any per-scene tuning.
// Above it, the automatic policy reserves the budget and lets builds grow
// (allocating and counting the overflow) rather than failing a valid scene
// mid-step; explicit capacities remain strict. The budget also bounds the
// projected-Newton triplet reserve, which scales with 144 entries per
// candidate, so it is kept well below the candidate vectors' own cost.
constexpr std::size_t kAutomaticSurfaceCandidateReserveBudget = 65536u;

struct AutomaticSurfaceCandidatePolicy
{
  std::size_t capacity = 1u;
  bool growthAllowed = false;
};

//==============================================================================
AutomaticSurfaceCandidatePolicy automaticSurfaceCandidateCapacity(
    std::size_t theoreticalValidPairBound, std::size_t observedCandidateCount)
{
  AutomaticSurfaceCandidatePolicy policy;
  if (theoreticalValidPairBound <= kAutomaticSurfaceCandidateReserveBudget) {
    // Candidate builders are skipped for topology with no valid pair. Keeping
    // a one-entry floor avoids encoding "unbounded" and "empty" with the same
    // zero sentinel while retaining negligible storage for point-only bodies.
    policy.capacity = std::max<std::size_t>(
        1u, std::max(theoreticalValidPairBound, observedCandidateCount));
    policy.growthAllowed = false;
    return policy;
  }
  policy.capacity = std::max(
      kAutomaticSurfaceCandidateReserveBudget, observedCandidateCount);
  policy.growthAllowed = true;
  return policy;
}

//==============================================================================
std::size_t candidatePairCapacity(
    const std::size_t pointTrianglePairCapacity,
    const std::size_t edgeEdgePairCapacity)
{
  return checkedSurfaceCandidateAdd(
      pointTrianglePairCapacity,
      edgeEdgePairCapacity,
      "candidate-pair diagnostics");
}

//==============================================================================
std::size_t candidateRejectedPairCount(
    const std::size_t pairCapacity,
    const std::size_t pointTriangleCandidateCount,
    const std::size_t edgeEdgeCandidateCount)
{
  const std::size_t candidateCount = checkedSurfaceCandidateAdd(
      pointTriangleCandidateCount,
      edgeEdgeCandidateCount,
      "emitted candidate diagnostics");
  return pairCapacity > candidateCount ? pairCapacity - candidateCount : 0u;
}

//==============================================================================
void accumulateCandidateFilterPressure(
    std::size_t& pairCapacityCounter,
    std::size_t& rejectedPairsCounter,
    const std::size_t pointTrianglePairCapacity,
    const std::size_t edgeEdgePairCapacity,
    const std::size_t pointTriangleCandidateCount,
    const std::size_t edgeEdgeCandidateCount)
{
  const std::size_t pairCapacity
      = candidatePairCapacity(pointTrianglePairCapacity, edgeEdgePairCapacity);
  pairCapacityCounter = checkedSurfaceCandidateAdd(
      pairCapacityCounter, pairCapacity, "candidate-pair diagnostics");
  rejectedPairsCounter = checkedSurfaceCandidateAdd(
      rejectedPairsCounter,
      candidateRejectedPairCount(
          pairCapacity, pointTriangleCandidateCount, edgeEdgeCandidateCount),
      "rejected candidate-pair diagnostics");
}

//==============================================================================
void recordSurfaceCandidateCount(
    DeformableSolverStats& stats,
    std::size_t pointTriangleCount,
    std::size_t edgeEdgeCount,
    std::size_t capacityOverflowCount)
{
  stats.surfaceContactCandidateCountPeak = std::max(
      stats.surfaceContactCandidateCountPeak,
      checkedSurfaceCandidateAdd(
          pointTriangleCount,
          edgeEdgeCount,
          "runtime emitted surface candidates"));
  stats.surfaceContactCandidateOverflowCount = checkedSurfaceCandidateAdd(
      stats.surfaceContactCandidateOverflowCount,
      capacityOverflowCount,
      "runtime surface candidate overflow");
}

//==============================================================================
void considerInterBodyContactResult(
    InterBodySurfaceContactResult& aggregate,
    const dc::ContinuousCollisionStepResult& candidate)
{
  nb::accumulateLineSearchStats(aggregate.stats, candidate.stats);
  aggregate.indeterminate = aggregate.indeterminate || candidate.indeterminate;
  if (candidate.indeterminate) {
    aggregate.stepBound = 0.0;
    return;
  }

  if (!candidate.hit) {
    return;
  }

  if (!aggregate.hit || candidate.stepBound < aggregate.stepBound) {
    aggregate.hit = true;
    aggregate.stepBound = candidate.stepBound;
  }
}

//==============================================================================
void requireInterSurfaceCandidateCapacity(
    InterBodySurfaceContactResult& aggregate,
    const dc::ContactCandidateOptions& options)
{
  const std::size_t pointTriangle = aggregate.pointTriangleCandidateCount;
  const std::size_t edgeEdge = aggregate.edgeEdgeCandidateCount;
  const bool overflow
      = pointTriangle > std::numeric_limits<std::size_t>::max() - edgeEdge;
  const std::size_t emittedCount = overflow
                                       ? std::numeric_limits<std::size_t>::max()
                                       : pointTriangle + edgeEdge;
  const std::size_t requestedCount
      = emittedCount == std::numeric_limits<std::size_t>::max()
            ? emittedCount
            : emittedCount + 1u;
  if (!overflow && emittedCount >= options.candidateCapacity
      && options.allowCapacityGrowth) {
    ++aggregate.capacityOverflowCount;
    return;
  }
  DART_SIMULATION_THROW_T_IF(
      overflow || emittedCount >= options.candidateCapacity,
      InvalidOperationException,
      "Deformable surface contact candidate capacity {} exceeded by "
      "requested count {}",
      options.candidateCapacity,
      requestedCount);
}

//==============================================================================
void buildPointSweepItems(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const std::uint8_t> pointMask,
    double margin,
    auto& items)
{
  items.clear();
  items.reserve(positionsStart.size());
  for (std::size_t point = 0; point < positionsStart.size(); ++point) {
    if (!surfaceContactPointAllowed(point, pointMask)) {
      continue;
    }
    items.push_back(
        dc::detail::SweepItem{
            point,
            dc::detail::makeSweptPointAabb(
                positionsStart[point], positionsEnd[point], margin)});
  }
}

//==============================================================================
void buildTriangleSweepItems(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    double margin,
    auto& items)
{
  items.clear();
  items.reserve(triangles.size());
  for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
    const auto& t = triangles[triangle];
    items.push_back(
        dc::detail::SweepItem{
            triangle,
            dc::detail::makeSweptTriangleAabb(
                positionsStart[t.nodeA],
                positionsEnd[t.nodeA],
                positionsStart[t.nodeB],
                positionsEnd[t.nodeB],
                positionsStart[t.nodeC],
                positionsEnd[t.nodeC],
                margin)});
  }
}

//==============================================================================
void buildEdgeSweepItems(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const dc::SurfaceEdge> edges,
    double margin,
    auto& items)
{
  items.clear();
  items.reserve(edges.size());
  for (std::size_t edge = 0; edge < edges.size(); ++edge) {
    const auto& e = edges[edge];
    items.push_back(
        dc::detail::SweepItem{
            edge,
            dc::detail::makeSweptSegmentAabb(
                positionsStart[e.nodeA],
                positionsEnd[e.nodeA],
                positionsStart[e.nodeB],
                positionsEnd[e.nodeB],
                margin)});
  }
}

//==============================================================================
InterBodySurfaceContactResult interBodySurfaceContactStepBound(
    std::span<const Eigen::Vector3d> currentStart,
    std::span<const Eigen::Vector3d> currentEnd,
    std::span<const DeformableSurfaceTriangle> currentTriangles,
    std::span<const std::uint8_t> currentPointMask,
    std::span<const dc::SurfaceEdge> currentEdges,
    const SurfaceContactSnapshot& obstacle,
    const dc::ContactCandidateOptions& candidateOptions,
    const dc::ContinuousCollisionStepOptions& ccdOptions,
    DeformableContactSolverScratch& scratch)
{
  InterBodySurfaceContactResult aggregate;
  const double margin
      = 0.5 * dc::detail::nonnegativeActivationDistance(candidateOptions);

  buildPointSweepItems(
      currentStart,
      currentEnd,
      currentPointMask,
      margin,
      scratch.interBodyCurrentPointItems);
  buildPointSweepItems(
      obstacle.positions,
      obstacle.positions,
      obstacle.surfaceContactPointMask,
      margin,
      scratch.interBodyObstaclePointItems);
  buildTriangleSweepItems(
      currentStart,
      currentEnd,
      currentTriangles,
      margin,
      scratch.interBodyCurrentTriangleItems);
  buildTriangleSweepItems(
      obstacle.positions,
      obstacle.positions,
      obstacle.surfaceTriangles,
      margin,
      scratch.interBodyObstacleTriangleItems);
  aggregate.pointTrianglePairCapacity = checkedSurfaceCandidateAdd(
      checkedSurfaceCandidateMultiply(
          scratch.interBodyCurrentPointItems.size(),
          scratch.interBodyObstacleTriangleItems.size(),
          "inter-surface current-point pairs"),
      checkedSurfaceCandidateMultiply(
          scratch.interBodyObstaclePointItems.size(),
          scratch.interBodyCurrentTriangleItems.size(),
          "inter-surface obstacle-point pairs"),
      "inter-surface point-triangle pair diagnostics");

  dc::detail::visitSweepPairs(
      scratch.interBodyCurrentPointItems,
      scratch.interBodyObstacleTriangleItems,
      [&](const std::size_t point, const std::size_t triangleIndex) {
        requireInterSurfaceCandidateCapacity(aggregate, candidateOptions);
        ++aggregate.pointTriangleCandidateCount;
        const auto& triangle = obstacle.surfaceTriangles[triangleIndex];
        const auto result = dc::pointTriangleStepBound(
            currentStart[point],
            currentEnd[point],
            obstacle.positions[triangle.nodeA],
            obstacle.positions[triangle.nodeA],
            obstacle.positions[triangle.nodeB],
            obstacle.positions[triangle.nodeB],
            obstacle.positions[triangle.nodeC],
            obstacle.positions[triangle.nodeC],
            ccdOptions);
        considerInterBodyContactResult(aggregate, result);
      },
      scratch.interBodySweepLinks);

  dc::detail::visitSweepPairs(
      scratch.interBodyObstaclePointItems,
      scratch.interBodyCurrentTriangleItems,
      [&](const std::size_t point, const std::size_t triangleIndex) {
        requireInterSurfaceCandidateCapacity(aggregate, candidateOptions);
        ++aggregate.pointTriangleCandidateCount;
        const auto& triangle = currentTriangles[triangleIndex];
        const auto result = dc::pointTriangleStepBound(
            obstacle.positions[point],
            obstacle.positions[point],
            currentStart[triangle.nodeA],
            currentEnd[triangle.nodeA],
            currentStart[triangle.nodeB],
            currentEnd[triangle.nodeB],
            currentStart[triangle.nodeC],
            currentEnd[triangle.nodeC],
            ccdOptions);
        considerInterBodyContactResult(aggregate, result);
      },
      scratch.interBodySweepLinks);

  buildEdgeSweepItems(
      currentStart,
      currentEnd,
      currentEdges,
      margin,
      scratch.interBodyCurrentEdgeItems);
  buildEdgeSweepItems(
      obstacle.positions,
      obstacle.positions,
      obstacle.surfaceEdges,
      margin,
      scratch.interBodyObstacleEdgeItems);
  aggregate.edgeEdgePairCapacity = checkedSurfaceCandidateMultiply(
      scratch.interBodyCurrentEdgeItems.size(),
      scratch.interBodyObstacleEdgeItems.size(),
      "inter-surface edge pairs");
  dc::detail::visitSweepPairs(
      scratch.interBodyCurrentEdgeItems,
      scratch.interBodyObstacleEdgeItems,
      [&](const std::size_t currentEdge, const std::size_t obstacleEdge) {
        requireInterSurfaceCandidateCapacity(aggregate, candidateOptions);
        ++aggregate.edgeEdgeCandidateCount;
        const auto& a = currentEdges[currentEdge];
        const auto& b = obstacle.surfaceEdges[obstacleEdge];
        const auto result = dc::edgeEdgeStepBound(
            currentStart[a.nodeA],
            currentEnd[a.nodeA],
            currentStart[a.nodeB],
            currentEnd[a.nodeB],
            obstacle.positions[b.nodeA],
            obstacle.positions[b.nodeA],
            obstacle.positions[b.nodeB],
            obstacle.positions[b.nodeB],
            ccdOptions);
        considerInterBodyContactResult(aggregate, result);
      },
      scratch.interBodySweepLinks);

  return aggregate;
}

//==============================================================================
void fillStaticBoxSurfaceCcdSnapshot(
    SurfaceContactSnapshot& snapshot,
    entt::entity entity,
    const Eigen::Vector3d& halfExtents,
    const comps::Transform& transform)
{
  snapshot.entity = entity;
  snapshot.positions.clear();
  snapshot.positions.reserve(8);
  snapshot.surfaceTriangles.clear();
  snapshot.surfaceContactPointMask.clear();
  snapshot.surfaceEdges.clear();

  const Eigen::Matrix3d rotation
      = normalizeOrIdentity(transform.orientation).toRotationMatrix();
  const std::array<Eigen::Vector3d, 8> localVertices{
      Eigen::Vector3d(-halfExtents.x(), -halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), -halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(-halfExtents.x(), halfExtents.y(), -halfExtents.z()),
      Eigen::Vector3d(-halfExtents.x(), -halfExtents.y(), halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), -halfExtents.y(), halfExtents.z()),
      Eigen::Vector3d(halfExtents.x(), halfExtents.y(), halfExtents.z()),
      Eigen::Vector3d(-halfExtents.x(), halfExtents.y(), halfExtents.z())};
  for (const auto& local : localVertices) {
    snapshot.positions.push_back(transform.position + rotation * local);
  }

  constexpr std::array<std::array<std::size_t, 3>, 12> kBoxTriangles{{
      {{0, 3, 1}},
      {{1, 3, 2}},
      {{4, 5, 7}},
      {{5, 6, 7}},
      {{0, 1, 4}},
      {{1, 5, 4}},
      {{2, 3, 6}},
      {{3, 7, 6}},
      {{0, 4, 3}},
      {{3, 4, 7}},
      {{1, 2, 5}},
      {{2, 6, 5}},
  }};
  snapshot.surfaceTriangles.reserve(kBoxTriangles.size());
  for (const auto& triangle : kBoxTriangles) {
    snapshot.surfaceTriangles.push_back(
        DeformableSurfaceTriangle{triangle[0], triangle[1], triangle[2]});
  }

  constexpr std::array<std::array<std::size_t, 2>, 12> kBoxEdges{{
      {{0, 1}},
      {{1, 2}},
      {{2, 3}},
      {{0, 3}},
      {{4, 5}},
      {{5, 6}},
      {{6, 7}},
      {{4, 7}},
      {{0, 4}},
      {{1, 5}},
      {{2, 6}},
      {{3, 7}},
  }};
  snapshot.surfaceEdges.reserve(kBoxEdges.size());
  for (const auto& edge : kBoxEdges) {
    snapshot.surfaceEdges.push_back(
        dc::detail::makeSurfaceEdge(edge[0], edge[1]));
  }
}

//==============================================================================
// A static sphere has no flat faces to triangulate, so it is tessellated into a
// UV-sphere polyhedron that CIRCUMSCRIBES the true sphere: the vertices sit at
// a slightly larger radius so every flat face stays outside the analytic
// surface. That keeps the conservative no-penetration guarantee (a deformable
// is stopped at the polyhedron, never inside the real sphere), and the existing
// point-triangle / edge-edge CCD reducers consume the snapshot unchanged. The
// outward inflation is the modest over-conservatism this introduces (smaller
// with finer tessellation), analogous to the box supersampling's thin-corner
// over-coverage.
void fillStaticSphereSurfaceCcdSnapshot(
    SurfaceContactSnapshot& snapshot,
    entt::entity entity,
    double radius,
    const comps::Transform& transform)
{
  constexpr int kLongitude = 16; // segments around the equator
  constexpr int kLatitude = 8;   // bands from pole to pole
  constexpr double kPi = 3.14159265358979323846;
  constexpr std::size_t kNorthPole = 0;

  const double dTheta = 2.0 * kPi / static_cast<double>(kLongitude);
  const double dPhi = kPi / static_cast<double>(kLatitude);
  // Inflate the vertex radius so the flat faces (chords) circumscribe the
  // sphere: a cell's circumscribed cap has angular radius ~0.5 * the cell
  // diagonal, and a face at vertex radius r sits at r*cos(that) from the
  // center, so r = radius / cos(angularRadius) puts every face at >= radius. A
  // small safety factor absorbs rounding.
  const double angularRadius = 0.5 * std::sqrt(dTheta * dTheta + dPhi * dPhi);
  const double r = radius * (1.001 / std::cos(angularRadius));

  const Eigen::Matrix3d rotation
      = normalizeOrIdentity(transform.orientation).toRotationMatrix();
  const auto worldPoint = [&](const Eigen::Vector3d& local) {
    return Eigen::Vector3d(transform.position + rotation * local);
  };

  snapshot.entity = entity;
  snapshot.positions.clear();
  snapshot.surfaceTriangles.clear();
  snapshot.surfaceContactPointMask.clear();
  snapshot.surfaceEdges.clear();

  // Vertices: north pole, (kLatitude - 1) interior rings of kLongitude vertices
  // each, then the south pole.
  snapshot.positions.push_back(worldPoint(Eigen::Vector3d(0.0, 0.0, r)));
  for (int i = 1; i < kLatitude; ++i) {
    const double phi = dPhi * static_cast<double>(i);
    const double z = r * std::cos(phi);
    const double ringRadius = r * std::sin(phi);
    for (int j = 0; j < kLongitude; ++j) {
      const double theta = dTheta * static_cast<double>(j);
      snapshot.positions.push_back(worldPoint(
          Eigen::Vector3d(
              ringRadius * std::cos(theta), ringRadius * std::sin(theta), z)));
    }
  }
  const auto southPole = snapshot.positions.size();
  snapshot.positions.push_back(worldPoint(Eigen::Vector3d(0.0, 0.0, -r)));

  // Vertex index for column j (wrapping) of interior ring in [1, kLatitude -
  // 1].
  const auto ringVertex = [&](int ring, int j) {
    const int col = ((j % kLongitude) + kLongitude) % kLongitude;
    return static_cast<std::size_t>(1 + (ring - 1) * kLongitude + col);
  };

  for (int j = 0; j < kLongitude; ++j) {
    snapshot.surfaceTriangles.push_back(
        DeformableSurfaceTriangle{
            kNorthPole, ringVertex(1, j), ringVertex(1, j + 1)});
  }
  for (int i = 1; i < kLatitude - 1; ++i) {
    for (int j = 0; j < kLongitude; ++j) {
      const auto a = ringVertex(i, j);
      const auto b = ringVertex(i, j + 1);
      const auto c = ringVertex(i + 1, j);
      const auto d = ringVertex(i + 1, j + 1);
      snapshot.surfaceTriangles.push_back(DeformableSurfaceTriangle{a, c, b});
      snapshot.surfaceTriangles.push_back(DeformableSurfaceTriangle{b, c, d});
    }
  }
  for (int j = 0; j < kLongitude; ++j) {
    snapshot.surfaceTriangles.push_back(
        DeformableSurfaceTriangle{
            southPole,
            ringVertex(kLatitude - 1, j + 1),
            ringVertex(kLatitude - 1, j)});
  }

  // Unlike a box (whose triangulation diagonals are not physical edges), every
  // sphere tessellation edge lies on the surface, so the full unique edge set
  // is the correct edge-edge CCD input.
  dc::buildUniqueSurfaceEdges(snapshot.surfaceTriangles, snapshot.surfaceEdges);
}

//==============================================================================
bool isCurrentPoseRigidSurfaceCcdObstacle(
    const detail::WorldRegistry& registry, const entt::entity entity)
{
  return registry.all_of<comps::StaticBodyTag>(entity)
         || registry.all_of<comps::KinematicBodyTag>(entity);
}

//==============================================================================
bool hasCurrentKinematicStepTrace(const World& world, const entt::entity entity)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto* trace = registry.try_get<comps::KinematicBodyStepTrace>(entity);
  return trace != nullptr && trace->frame == world.getFrame();
}

//==============================================================================
void collectStaticRigidSurfaceCcdObstaclesInto(
    const World& world,
    DeformableSolverStats& stats,
    auto& snapshots,
    std::size_t& snapshotCount,
    common::MemoryAllocator* payloadAllocator = nullptr)
{
  ++stats.staticRigidSurfaceCcdSnapshotBuilds;
  snapshotCount = 0;

  const auto& registry = dart::simulation::detail::registryOf(world);
  // Barrier-only obstacles keep their contact barrier but are excluded from the
  // CCD limiter, so a deformable can slide tangentially against them (and be
  // decelerated by friction) instead of having its whole step over-limited.
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>(entt::exclude<comps::DeformableObstacleNoCcdTag>);

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    // Current-frame kinematic boxes move to the swept moving collector; other
    // supported shapes stay as current-pose snapshots until they gain swept
    // snapshot support.
    if (hasCurrentKinematicStepTrace(world, entity)
        && shape->type == CollisionShapeType::Box) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (!shapeTransform.has_value()) {
      continue;
    }

    if (shape->type == CollisionShapeType::Box) {
      if (!shape->halfExtents.allFinite()
          || (shape->halfExtents.array() <= 0.0).any()) {
        continue;
      }
      auto& snapshot = nextSurfaceContactSnapshot(
          snapshots, snapshotCount, payloadAllocator);
      fillStaticBoxSurfaceCcdSnapshot(
          snapshot, entity, shape->halfExtents, *shapeTransform);
      ++stats.staticRigidSurfaceCcdBoxCount;
      stats.staticRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.staticRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
    } else if (shape->type == CollisionShapeType::Sphere) {
      if (!std::isfinite(shape->radius) || shape->radius <= 0.0) {
        continue;
      }
      auto& snapshot = nextSurfaceContactSnapshot(
          snapshots, snapshotCount, payloadAllocator);
      fillStaticSphereSurfaceCcdSnapshot(
          snapshot, entity, shape->radius, *shapeTransform);
      ++stats.staticRigidSurfaceCcdSphereCount;
      stats.staticRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.staticRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
    } else {
      continue;
    }
  }
}

//==============================================================================
// Predict a free rigid body's end-of-step transform from its current pose and
// velocity WITHOUT mutating any component. Mirrors integrateRigidBodyPosition
// exactly so the deformable stage limits against the SAME motion that
// RigidBodyPositionStage will later apply at stage 5.
comps::Transform predictRigidBodyEndTransform(
    const comps::Transform& transform,
    const comps::Velocity& velocity,
    const double timeStep)
{
  comps::Transform end = transform;
  end.position = transform.position + velocity.linear * timeStep;

  auto orientation = normalizeOrIdentity(transform.orientation);
  const double angularSpeed = velocity.angular.norm();
  if (angularSpeed > 0.0 && std::isfinite(angularSpeed)) {
    const auto rotation = Eigen::AngleAxisd(
        angularSpeed * timeStep, velocity.angular.normalized());
    orientation = rotation * orientation;
    orientation.normalize();
  }
  end.orientation = normalizeOrIdentity(orientation);
  return end;
}

//==============================================================================
comps::Transform predictKinematicRigidSurfaceCcdEndTransform(
    const World& world,
    const detail::WorldRegistry& registry,
    const entt::entity entity,
    const comps::Transform& transform,
    const comps::Velocity& velocity)
{
  double timeStep = world.getTimeStep();
  if (const auto* tag = registry.try_get<comps::KinematicBodyTag>(entity);
      tag != nullptr && tag->maxTime.has_value()
      && std::isfinite(*tag->maxTime)) {
    const double remainingTime = *tag->maxTime - world.getTime();
    if (!(remainingTime > 0.0)) {
      timeStep = 0.0;
    } else {
      timeStep = std::min(timeStep, remainingTime);
    }
  }
  return predictRigidBodyEndTransform(transform, velocity, timeStep);
}

//==============================================================================
// Number of intermediate static box poses that tile a moving obstacle's swept
// motion. Below the cap, consecutive sample centers are one box min-half-extent
// apart so the sampled boxes overlap with margin. The count is capped so a
// pathologically fast obstacle cannot explode the work; past the cap the
// residual spacing grows, and the caller restores overlap by inflating the
// sampled boxes (see collectMovingRigidSurfaceCcdObstacles), so the swept
// corridor stays covered rather than developing tunnelable gaps.
std::size_t movingRigidSurfaceCcdSampleCount(
    const Eigen::Vector3d& halfExtents,
    const comps::Transform& startTransform,
    const comps::Transform& endTransform,
    const Eigen::Vector3d& angularVelocity,
    const double timeStep)
{
  constexpr std::size_t kMaxSamples = 64;
  const double minHalfExtent = halfExtents.minCoeff();
  if (!(minHalfExtent > 0.0)) {
    return 2;
  }

  const double linearMotion
      = (endTransform.position - startTransform.position).norm();
  const double radius = halfExtents.norm();
  const double angularMotion = radius * angularVelocity.norm() * timeStep;
  const double motion = linearMotion + angularMotion;
  if (!std::isfinite(motion) || motion <= 0.0) {
    return 2;
  }

  const auto segments
      = static_cast<std::size_t>(std::ceil(motion / minHalfExtent));
  const std::size_t samples = std::min(kMaxSamples, segments + 1);
  return std::max<std::size_t>(samples, 2);
}

//==============================================================================
// Collect conservative static box snapshots that tile each MOVING rigid box
// obstacle's swept motion over the step. Each snapshot is an ordinary static
// pose, so the existing static-style limiter and
// interBodySurfaceContactStepBound handle them unchanged. Free-rigid obstacle
// end poses are predicted from velocity because deformable dynamics runs before
// RigidBodyPositionStage. Kinematic obstacle end poses come from the current
// step trace written by rigid IPC, so deformables are limited against the
// realized start->current motion instead of a future frame.
//
// This treats the swept volume as a static blocker for the step (timing-
// agnostic): it is more conservative than a timing-aware sweep for fast
// obstacles. At IPC-scale time steps the per-step motion is small, so few
// samples closely approximate the true contact surface. Consecutive sample
// boxes are kept overlapping so the deformable cannot tunnel between them: when
// the sample count hits its cap and the natural spacing would open gaps, the
// sampled boxes are isotropically inflated by half the residual spacing, which
// keeps the real obstacle inside each sample and the corridor covered (at the
// cost of extra over-conservatism). Coverage of the swept hull is exact along
// the box min axis for axis-aligned motion; for diagonal motion a bounded,
// half-extent-scale thin-corner under-coverage remains, inherent to all box
// supersampling.
void collectMovingRigidSurfaceCcdObstaclesInto(
    const World& world,
    const double timeStep,
    DeformableSolverStats& stats,
    auto& snapshots,
    std::size_t& snapshotCount,
    const bool primeKinematicWithoutCurrentTrace = false,
    common::MemoryAllocator* payloadAllocator = nullptr)
{
  ++stats.movingRigidSurfaceCcdSnapshotBuilds;
  snapshotCount = 0;

  const auto& registry = dart::simulation::detail::registryOf(world);
  // Moving obstacles are free rigid bodies integrated by RigidBodyPositionStage
  // or kinematic bodies already advanced by the rigid IPC stage. Static bodies
  // stay in collectStaticRigidSurfaceCcdObstacles. Kinematic bodies only enter
  // this moving set when rigid IPC left a current-frame trace; otherwise they
  // remain current-pose obstacles.
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform,
      comps::Velocity>(entt::exclude<comps::StaticBodyTag>);

  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto& velocity = view.get<comps::Velocity>(entity);
    const bool isKinematic = registry.all_of<comps::KinematicBodyTag>(entity);
    const auto* trace = registry.try_get<comps::KinematicBodyStepTrace>(entity);
    if (shape->type != CollisionShapeType::Box
        || !shape->halfExtents.allFinite()
        || (shape->halfExtents.array() <= 0.0).any()
        || !velocity.linear.allFinite() || !velocity.angular.allFinite()) {
      continue;
    }
    comps::Transform startTransform = transform;
    comps::Transform endTransform;
    if (isKinematic) {
      if (trace != nullptr && trace->frame == world.getFrame()) {
        startTransform = trace->startTransform;
        endTransform = trace->endTransform;
      } else if (primeKinematicWithoutCurrentTrace) {
        endTransform = predictKinematicRigidSurfaceCcdEndTransform(
            world, registry, entity, transform, velocity);
      } else {
        continue;
      }
    } else {
      endTransform
          = predictRigidBodyEndTransform(transform, velocity, timeStep);
    }
    const auto startShapeTransform
        = collisionShapeWorldTransform(startTransform, *shape);
    if (!startShapeTransform.has_value()) {
      continue;
    }

    const auto endShapeTransform
        = collisionShapeWorldTransform(endTransform, *shape);
    if (!endShapeTransform.has_value()) {
      continue;
    }
    const auto& halfExtents = shape->halfExtents;
    const std::size_t samples = movingRigidSurfaceCcdSampleCount(
        halfExtents,
        *startShapeTransform,
        *endShapeTransform,
        velocity.angular,
        timeStep);

    // Keep consecutive sample boxes overlapping. The natural (uncapped) spacing
    // is one box min-half-extent, so the boxes overlap with margin; once the
    // sample count is capped the residual spacing can exceed the box min
    // dimension, so inflate each sampled box isotropically by half the residual
    // spacing to bridge the gap. inflation is 0 in the common (uncapped) case.
    const double minHalfExtent = halfExtents.minCoeff();
    const double linearMotion
        = (endShapeTransform->position - startShapeTransform->position).norm();
    const double angularMotion
        = halfExtents.norm() * velocity.angular.norm() * timeStep;
    const double motion = linearMotion + angularMotion;
    const double spacing = (samples > 1 && std::isfinite(motion))
                               ? motion / static_cast<double>(samples - 1)
                               : 0.0;
    const double inflation = std::max(0.0, 0.5 * spacing - minHalfExtent);
    const Eigen::Vector3d sampleHalfExtents
        = halfExtents + Eigen::Vector3d::Constant(inflation);
    if (inflation > 0.0) {
      ++stats.movingRigidSurfaceCcdInflatedBoxCount;
    }

    const Eigen::Quaterniond startOrientation
        = normalizeOrIdentity(startShapeTransform->orientation);
    const Eigen::Quaterniond endOrientation
        = normalizeOrIdentity(endShapeTransform->orientation);

    for (std::size_t k = 0; k < samples; ++k) {
      // samples is always >= 2, so the denominator is never zero.
      const double fraction
          = static_cast<double>(k) / static_cast<double>(samples - 1);
      comps::Transform sampleTransform;
      sampleTransform.position = startShapeTransform->position
                                 + fraction
                                       * (endShapeTransform->position
                                          - startShapeTransform->position);
      sampleTransform.orientation
          = startOrientation.slerp(fraction, endOrientation);

      auto& snapshot = nextSurfaceContactSnapshot(
          snapshots, snapshotCount, payloadAllocator);
      fillStaticBoxSurfaceCcdSnapshot(
          snapshot, entity, sampleHalfExtents, sampleTransform);
      stats.movingRigidSurfaceCcdTriangleCount
          += snapshot.surfaceTriangles.size();
      stats.movingRigidSurfaceCcdEdgeCount += snapshot.surfaceEdges.size();
      ++stats.movingRigidSurfaceCcdSampleCount;
    }
    ++stats.movingRigidSurfaceCcdBoxCount;
  }
}

//==============================================================================
// A static-ground contact sample under a deformable node: the supporting
// surface height (`top`, world z directly below/above the node) and the
// geometric outward surface normal there (unit, upward hemisphere). The barrier
// itself is a vertical height-field penalty (force along +z), but friction
// resolves its tangent plane against this true normal, so a sphere or tilted
// box gives a tilted tangent plane rather than a hardcoded xy plane.
struct StaticGroundContact
{
  double top = 0.0;
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  std::uint64_t objectId = 0;
  std::uint64_t featureId = 0;
};

//==============================================================================
// Vertical ray-march of the +z line through the node's (x, y) against a
// (possibly rotated) box, returning the exit height and that exit face's
// outward world normal. The exit face is whichever slab bound the +z ray leaves
// last; its local outward normal points along sign(direction) of the binding
// axis (toward the upper hemisphere). For an axis-aligned box this is the +z
// top face with normal +z, recovering the flat-ground case exactly.
std::optional<StaticGroundContact> boxContactAt(
    const StaticGroundBarrier& barrier, const Eigen::Vector3d& position)
{
  constexpr double tolerance = 1e-12;
  const Eigen::Vector3d worldAtZero(position.x(), position.y(), 0.0);
  const Eigen::Vector3d localAtZero
      = barrier.rotation.transpose() * (worldAtZero - barrier.center);
  const Eigen::Vector3d localVertical
      = barrier.rotation.transpose() * Eigen::Vector3d::UnitZ();

  double minZ = -std::numeric_limits<double>::infinity();
  double maxZ = std::numeric_limits<double>::infinity();
  int bindingAxis = -1;
  double bindingSign = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double extent = barrier.halfExtents[axis];
    const double origin = localAtZero[axis];
    const double direction = localVertical[axis];
    if (std::abs(direction) <= tolerance) {
      if (origin < -extent - tolerance || origin > extent + tolerance) {
        return std::nullopt;
      }
      continue;
    }

    double intervalMin = (-extent - origin) / direction;
    double intervalMax = (extent - origin) / direction;
    if (intervalMin > intervalMax) {
      std::swap(intervalMin, intervalMax);
    }

    minZ = std::max(minZ, intervalMin);
    if (intervalMax < maxZ) {
      maxZ = intervalMax;
      bindingAxis = axis;
      // The exit face on this axis is the one the +z ray leaves through; its
      // local outward normal points along the sign of the ray direction.
      bindingSign = (direction > 0.0) ? 1.0 : -1.0;
    }
    if (minZ > maxZ + tolerance) {
      return std::nullopt;
    }
  }

  if (!std::isfinite(maxZ) || bindingAxis < 0) {
    return std::nullopt;
  }

  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  localNormal[bindingAxis] = bindingSign;
  Eigen::Vector3d worldNormal = barrier.rotation * localNormal;
  if (worldNormal.z() < 0.0) {
    worldNormal = -worldNormal;
  }
  const double norm = worldNormal.norm();

  StaticGroundContact contact;
  contact.top = maxZ;
  contact.normal = (norm > tolerance)
                       ? Eigen::Vector3d(worldNormal / norm)
                       : Eigen::Vector3d(Eigen::Vector3d::UnitZ());
  contact.objectId = barrier.objectId;
  const Eigen::Vector3d localContact = localAtZero + maxZ * localVertical;
  contact.featureId = staticObstacleFeatureId(
      barrier.geometryRevision,
      dvbd::avbdBoxContactFeatureCode(localContact, barrier.halfExtents));
  return contact;
}

//==============================================================================
void collectStaticGroundBarriersInto(const World& world, auto& barriers)
{
  barriers.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::StaticBodyTag,
      comps::DeformableGroundBarrierTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (!shapeTransform.has_value()) {
      continue;
    }

    switch (shape->type) {
      case CollisionShapeType::Sphere: {
        const double radius = shape->radius;
        if (!(radius > 0.0) || !std::isfinite(radius)) {
          break;
        }

        StaticGroundBarrier barrier;
        barrier.shape = StaticGroundBarrier::Shape::Sphere;
        barrier.center = shapeTransform->position;
        barrier.radius = radius;
        barrier.top = shapeTransform->position.z() + radius;
        barrier.objectId
            = static_cast<std::uint64_t>(entt::to_integral(entity));
        barrier.geometryRevision = geometry.revision;
        barriers.push_back(barrier);
        break;
      }
      case CollisionShapeType::Box: {
        if (!shape->halfExtents.allFinite()
            || (shape->halfExtents.array() <= 0.0).any()) {
          break;
        }

        StaticGroundBarrier barrier;
        barrier.shape = StaticGroundBarrier::Shape::Box;
        barrier.center = shapeTransform->position;
        barrier.rotation = normalizeOrIdentity(shapeTransform->orientation)
                               .toRotationMatrix();
        barrier.halfExtents = shape->halfExtents;
        barrier.objectId
            = static_cast<std::uint64_t>(entt::to_integral(entity));
        barrier.geometryRevision = geometry.revision;
        barriers.push_back(barrier);
        break;
      }
      case CollisionShapeType::Capsule:
      case CollisionShapeType::Cylinder:
      case CollisionShapeType::Plane:
        break;
      case CollisionShapeType::Mesh:
        break;
    }
  }
}

//==============================================================================
// Static rigid SPHERE bodies opted in as deformable surface-CCD obstacles also
// exert a full radial barrier force on nearby deformable nodes. They are
// collected by center + radius (the smooth radial barrier needs no
// tessellation). Boxes opted in as surface-CCD obstacles are skipped here --
// their barrier force is a later increment -- and the surface CCD limiter
// remains the conservative no-penetration gate.
void collectSphereObstacleBarriersInto(const World& world, auto& obstacles)
{
  obstacles.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (shape->type != CollisionShapeType::Sphere || !(shape->radius > 0.0)
        || !std::isfinite(shape->radius) || !shapeTransform.has_value()) {
      continue;
    }
    obstacles.push_back(
        SphereObstacleBarrier{
            shapeTransform->position,
            shape->radius,
            static_cast<std::uint64_t>(entt::to_integral(entity)),
            geometry.revision});
  }
}

//==============================================================================
void collectBoxObstacleBarriersInto(const World& world, auto& obstacles)
{
  obstacles.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    if (shape->type != CollisionShapeType::Box
        || !shape->halfExtents.allFinite()
        || (shape->halfExtents.array() <= 0.0).any()
        || !shapeTransform.has_value()) {
      continue;
    }
    BoxObstacleBarrier obstacle;
    obstacle.center = shapeTransform->position;
    obstacle.rotation
        = normalizeOrIdentity(shapeTransform->orientation).toRotationMatrix();
    obstacle.halfExtents = shape->halfExtents;
    obstacle.objectId = static_cast<std::uint64_t>(entt::to_integral(entity));
    obstacle.geometryRevision = geometry.revision;
    obstacles.push_back(obstacle);
  }
}

//==============================================================================
void collectCapsuleObstacleBarriersInto(const World& world, auto& obstacles)
{
  obstacles.clear();

  const auto& registry = dart::simulation::detail::registryOf(world);
  auto view = registry.view<
      comps::RigidBodyTag,
      comps::DeformableSurfaceCcdObstacleTag,
      comps::CollisionGeometry,
      comps::Transform>();

  for (const auto entity : view) {
    if (!isCurrentPoseRigidSurfaceCcdObstacle(registry, entity)) {
      continue;
    }

    const auto& geometry = view.get<comps::CollisionGeometry>(entity);
    const auto* shape = geometry.getPrimaryShape();
    if (shape == nullptr) {
      continue;
    }
    const auto& transform = view.get<comps::Transform>(entity);
    const auto shapeTransform = collisionShapeWorldTransform(transform, *shape);
    const double radius = shape->radius;
    const double halfHeight = shape->halfExtents.z();
    if (shape->type != CollisionShapeType::Capsule || !(radius > 0.0)
        || !std::isfinite(radius) || !(halfHeight > 0.0)
        || !std::isfinite(halfHeight) || !shapeTransform.has_value()) {
      continue;
    }
    // The capsule axis is the body z axis; map its two segment endpoints into
    // the world frame.
    const Eigen::Vector3d axis
        = normalizeOrIdentity(shapeTransform->orientation)
              .toRotationMatrix()
              .col(2);
    CapsuleObstacleBarrier obstacle;
    obstacle.pointA = shapeTransform->position - halfHeight * axis;
    obstacle.pointB = shapeTransform->position + halfHeight * axis;
    obstacle.radius = radius;
    obstacles.push_back(obstacle);
  }
}

//==============================================================================
// The supporting static-ground contact under a node: the highest barrier
// surface across all barriers at the node's (x, y), together with that
// surface's geometric normal. Mirrors the max-top selection of the legacy
// height query but also carries the normal for friction's tangent basis.
std::optional<StaticGroundContact> staticGroundContactAt(
    const Eigen::Vector3d& position,
    std::span<const StaticGroundBarrier> barriers)
{
  std::optional<StaticGroundContact> best;
  for (const auto& barrier : barriers) {
    std::optional<StaticGroundContact> candidate;
    switch (barrier.shape) {
      case StaticGroundBarrier::Shape::Box:
        candidate = boxContactAt(barrier, position);
        break;
      case StaticGroundBarrier::Shape::Sphere: {
        const Eigen::Vector2d offset
            = position.head<2>() - barrier.center.head<2>();
        const double radiusSquared = barrier.radius * barrier.radius;
        const double planarDistanceSquared = offset.squaredNorm();
        if (planarDistanceSquared <= radiusSquared) {
          const double height
              = std::sqrt(radiusSquared - planarDistanceSquared);
          StaticGroundContact contact;
          contact.top = barrier.center.z() + height;
          // The contact point sits on the sphere directly above the node's
          // (x, y); its outward normal is radial, tilting away from +z toward
          // the rim. |(offset.x, offset.y, height)| == radius by construction.
          const Eigen::Vector3d radial(offset.x(), offset.y(), height);
          const double norm = radial.norm();
          contact.normal = (norm > 1e-12)
                               ? Eigen::Vector3d(radial / norm)
                               : Eigen::Vector3d(Eigen::Vector3d::UnitZ());
          contact.objectId = barrier.objectId;
          contact.featureId = staticObstacleFeatureId(barrier.geometryRevision);
          candidate = contact;
        }
        break;
      }
    }

    if (candidate.has_value() && std::isfinite(candidate->top)) {
      if (!best.has_value() || candidate->top > best->top) {
        best = candidate;
      }
    }
  }

  return best;
}

//==============================================================================
std::optional<double> staticGroundTopAt(
    const Eigen::Vector3d& position,
    std::span<const StaticGroundBarrier> barriers)
{
  const auto contact = staticGroundContactAt(position, barriers);
  if (!contact.has_value()) {
    return std::nullopt;
  }
  return contact->top;
}

//==============================================================================
double minimumStaticGroundHeight(double groundTop)
{
  constexpr double clearance = 1e-4;
  return groundTop + clearance;
}

//==============================================================================
double staticGroundBarrierCcdClearanceTolerance()
{
  return 1e-12;
}

//==============================================================================
double cross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
{
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

//==============================================================================
struct TimeInterval
{
  double begin{0.0};
  double end{1.0};
};

//==============================================================================
struct ProjectedBoxFootprint
{
  std::array<Eigen::Vector2d, 16> points{};
  std::size_t size = 0;

  void pushBack(const Eigen::Vector2d& point)
  {
    points[size++] = point;
  }

  void popBack()
  {
    --size;
  }

  [[nodiscard]] Eigen::Vector2d& back()
  {
    return points[size - 1];
  }

  [[nodiscard]] const Eigen::Vector2d& back() const
  {
    return points[size - 1];
  }

  [[nodiscard]] std::span<const Eigen::Vector2d> span() const
  {
    return std::span<const Eigen::Vector2d>(points.data(), size);
  }
};

//==============================================================================
ProjectedBoxFootprint projectedBoxFootprint(const StaticGroundBarrier& barrier)
{
  ProjectedBoxFootprint points;
  for (const double xSign : {-1.0, 1.0}) {
    for (const double ySign : {-1.0, 1.0}) {
      for (const double zSign : {-1.0, 1.0}) {
        const Eigen::Vector3d local(
            xSign * barrier.halfExtents.x(),
            ySign * barrier.halfExtents.y(),
            zSign * barrier.halfExtents.z());
        const Eigen::Vector3d world = barrier.center + barrier.rotation * local;
        points.pushBack(world.head<2>());
      }
    }
  }

  constexpr double tolerance = 1e-12;
  std::sort(
      points.points.begin(),
      points.points.begin() + static_cast<std::ptrdiff_t>(points.size),
      [](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        return std::tie(lhs.x(), lhs.y()) < std::tie(rhs.x(), rhs.y());
      });
  const auto uniqueEnd = std::unique(
      points.points.begin(),
      points.points.begin() + static_cast<std::ptrdiff_t>(points.size),
      [](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        return (lhs - rhs).squaredNorm() <= tolerance * tolerance;
      });
  points.size = static_cast<std::size_t>(
      std::distance(points.points.begin(), uniqueEnd));

  if (points.size <= 2) {
    return points;
  }

  ProjectedBoxFootprint hull;
  for (std::size_t i = 0; i < points.size; ++i) {
    const auto& point = points.points[i];
    while (hull.size >= 2
           && cross2d(
                  hull.back() - hull.points[hull.size - 2], point - hull.back())
                  <= tolerance) {
      hull.popBack();
    }
    hull.pushBack(point);
  }

  const auto lowerSize = hull.size;
  for (std::size_t i = points.size - 2; i < points.size; --i) {
    const auto& point = points.points[i];
    while (hull.size > lowerSize
           && cross2d(
                  hull.back() - hull.points[hull.size - 2], point - hull.back())
                  <= tolerance) {
      hull.popBack();
    }
    hull.pushBack(point);
    if (i == 0) {
      break;
    }
  }
  if (hull.size > 0) {
    hull.popBack();
  }

  return hull;
}

//==============================================================================
std::optional<TimeInterval> clipSegmentToConvexFootprint(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    std::span<const Eigen::Vector2d> footprint)
{
  if (footprint.size() < 3) {
    return std::nullopt;
  }

  constexpr double tolerance = 1e-12;
  const Eigen::Vector2d direction = end - start;
  double intervalBegin = 0.0;
  double intervalEnd = 1.0;

  for (std::size_t i = 0; i < footprint.size(); ++i) {
    const Eigen::Vector2d& a = footprint[i];
    const Eigen::Vector2d& b = footprint[(i + 1) % footprint.size()];
    const Eigen::Vector2d edge = b - a;
    const double valueAtStart = cross2d(edge, start - a);
    const double slope = cross2d(edge, direction);

    if (std::abs(slope) <= tolerance) {
      if (valueAtStart < -tolerance) {
        return std::nullopt;
      }
      continue;
    }

    const double boundaryT = (-tolerance - valueAtStart) / slope;
    if (slope > 0.0) {
      intervalBegin = std::max(intervalBegin, boundaryT);
    } else {
      intervalEnd = std::min(intervalEnd, boundaryT);
    }

    if (intervalBegin > intervalEnd) {
      return std::nullopt;
    }
  }

  return TimeInterval{
      std::clamp(intervalBegin, 0.0, 1.0), std::clamp(intervalEnd, 0.0, 1.0)};
}

//==============================================================================
std::optional<TimeInterval> sphereFootprintInterval(
    const StaticGroundBarrier& barrier,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  const Eigen::Vector2d origin = start.head<2>() - barrier.center.head<2>();
  const Eigen::Vector2d direction = end.head<2>() - start.head<2>();
  const double a = direction.squaredNorm();
  const double b = 2.0 * origin.dot(direction);
  const double c = origin.squaredNorm() - barrier.radius * barrier.radius;
  constexpr double tolerance = 1e-14;

  if (a <= tolerance) {
    if (c <= tolerance) {
      return TimeInterval{0.0, 1.0};
    }
    return std::nullopt;
  }

  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < -tolerance) {
    return std::nullopt;
  }

  const double root = std::sqrt(std::max(0.0, discriminant));
  double intervalBegin = (-b - root) / (2.0 * a);
  double intervalEnd = (-b + root) / (2.0 * a);
  if (intervalBegin > intervalEnd) {
    std::swap(intervalBegin, intervalEnd);
  }

  intervalBegin = std::max(intervalBegin, 0.0);
  intervalEnd = std::min(intervalEnd, 1.0);
  if (intervalBegin > intervalEnd) {
    return std::nullopt;
  }

  return TimeInterval{intervalBegin, intervalEnd};
}

//==============================================================================
std::optional<TimeInterval> staticGroundBarrierFootprintInterval(
    const StaticGroundBarrier& barrier,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end)
{
  switch (barrier.shape) {
    case StaticGroundBarrier::Shape::Box: {
      const auto footprint = projectedBoxFootprint(barrier);
      return clipSegmentToConvexFootprint(
          start.head<2>(), end.head<2>(), footprint.span());
    }
    case StaticGroundBarrier::Shape::Sphere:
      return sphereFootprintInterval(barrier, start, end);
  }

  return std::nullopt;
}

//==============================================================================
std::optional<double> staticGroundClearanceAt(
    const Eigen::Vector3d& position,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats)
{
  ++stats.staticGroundBarrierCcdSampleChecks;
  const auto groundTop = staticGroundTopAt(position, barriers);
  if (!groundTop.has_value()) {
    return std::nullopt;
  }
  return position.z() - minimumStaticGroundHeight(*groundTop);
}

//==============================================================================
Eigen::Vector3d interpolatePoint(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end, double t)
{
  return start + t * (end - start);
}

//==============================================================================
bool isStaticGroundBarrierCcdHit(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double t,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats)
{
  const auto clearance = staticGroundClearanceAt(
      interpolatePoint(start, end, t), barriers, stats);
  return clearance.has_value()
         && *clearance < -staticGroundBarrierCcdClearanceTolerance();
}

//==============================================================================
std::optional<double> verticalStaticGroundBarrierStepBound(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats)
{
  const auto startClearance = staticGroundClearanceAt(start, barriers, stats);
  const auto endClearance = staticGroundClearanceAt(end, barriers, stats);
  const double tolerance = staticGroundBarrierCcdClearanceTolerance();
  if (startClearance.has_value() && *startClearance < -tolerance) {
    return 0.0;
  }
  if (!endClearance.has_value() || *endClearance >= -tolerance) {
    return std::nullopt;
  }
  if (!startClearance.has_value()) {
    return std::nullopt;
  }

  const double denominator = *startClearance - *endClearance;
  if (!(denominator > 0.0) || !std::isfinite(denominator)) {
    return std::nullopt;
  }

  return std::clamp(*startClearance / denominator, 0.0, 1.0);
}

//==============================================================================
std::optional<double> firstStaticGroundBarrierHitInInterval(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const TimeInterval& interval,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats)
{
  const double tolerance = staticGroundBarrierCcdClearanceTolerance();
  if (isStaticGroundBarrierCcdHit(
          start, end, interval.begin, barriers, stats)) {
    return interval.begin;
  }
  if (interval.end <= interval.begin) {
    return std::nullopt;
  }

  constexpr int minimizationIterations = 48;
  double lo = interval.begin;
  double hi = interval.end;
  const auto clearanceOrInfinity = [&](double t) {
    const auto clearance = staticGroundClearanceAt(
        interpolatePoint(start, end, t), barriers, stats);
    return clearance.has_value() ? *clearance
                                 : std::numeric_limits<double>::infinity();
  };

  for (int iteration = 0; iteration < minimizationIterations; ++iteration) {
    const double third = (hi - lo) / 3.0;
    const double midA = lo + third;
    const double midB = hi - third;
    if (clearanceOrInfinity(midA) < clearanceOrInfinity(midB)) {
      hi = midB;
    } else {
      lo = midA;
    }
  }

  double hitT = 0.5 * (lo + hi);
  const auto minClearance = staticGroundClearanceAt(
      interpolatePoint(start, end, hitT), barriers, stats);
  if (!minClearance.has_value() || *minClearance >= -tolerance) {
    return std::nullopt;
  }

  constexpr int bisectionIterations = 32;
  lo = interval.begin;
  hi = hitT;
  for (int iteration = 0; iteration < bisectionIterations; ++iteration) {
    const double mid = 0.5 * (lo + hi);
    if (isStaticGroundBarrierCcdHit(start, end, mid, barriers, stats)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }

  return std::clamp(hi, 0.0, 1.0);
}

//==============================================================================
std::optional<double> continuousStaticGroundBarrierStepBound(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats)
{
  if (isStaticGroundBarrierCcdHit(start, end, 0.0, barriers, stats)) {
    return 0.0;
  }

  std::optional<double> stepBound;
  for (const auto& barrier : barriers) {
    const auto interval
        = staticGroundBarrierFootprintInterval(barrier, start, end);
    if (!interval.has_value()) {
      continue;
    }

    const auto hit = firstStaticGroundBarrierHitInInterval(
        start, end, *interval, barriers, stats);
    if (hit.has_value()) {
      stepBound = stepBound.has_value() ? std::min(*stepBound, *hit) : hit;
    }
  }

  return stepBound;
}

//==============================================================================
std::optional<double> staticGroundBarrierStepBound(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats)
{
  constexpr double planarTolerance = 1e-14;
  if ((end.head<2>() - start.head<2>()).squaredNorm()
      <= planarTolerance * planarTolerance) {
    return verticalStaticGroundBarrierStepBound(start, end, barriers, stats);
  }
  return continuousStaticGroundBarrierStepBound(start, end, barriers, stats);
}

//==============================================================================
double staticGroundBarrierActivationDistance()
{
  return 2e-2;
}

//==============================================================================
// The default (fixed) clamped-log barrier stiffness kappa shared by the ground,
// obstacle, and self-contact barriers.
constexpr double kDefaultBarrierStiffness = 25.0;

//==============================================================================
// IPC-style adaptive barrier stiffness (kappa) for a body, opted in per body.
//
// The clamped-log barrier's curvature contribution to the Hessian scales like
// kappa / d_hat^2, while a node's inertial stiffness is mass / dt^2. Balancing
// the two (so the barrier resolves contact in few iterations regardless of the
// mass/stiffness ratio) gives kappa ~ (mass / dt^2) * d_hat^2. For a unit nodal
// mass at dt = 1/250 and d_hat = 2e-2 this evaluates to exactly the historical
// fixed kappa = 25, so the fixed value is just this balance at unit mass; the
// adaptive form generalizes it, stiffening the barrier for heavier/faster
// bodies. The result is floored at the fixed default (never softer, so contact
// robustness never regresses) and capped to avoid an ill-conditioned Hessian.
double adaptiveBarrierStiffness(
    double maxNodalMass, double timeStep, double activationDistance)
{
  if (!(maxNodalMass > 0.0) || !(timeStep > 0.0) || !std::isfinite(maxNodalMass)
      || !std::isfinite(timeStep)) {
    return kDefaultBarrierStiffness;
  }
  constexpr double kMaxBarrierStiffness = 1.0e6;
  const double balanced = (maxNodalMass / (timeStep * timeStep))
                          * activationDistance * activationDistance;
  return std::clamp(balanced, kDefaultBarrierStiffness, kMaxBarrierStiffness);
}

//==============================================================================
bool satisfiesStaticGroundBarrier(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers)
{
  if (barriers.empty()) {
    return true;
  }

  for (std::size_t i = 0; i < positions.size(); ++i) {
    const auto groundTop = staticGroundTopAt(positions[i], barriers);
    if (fixed[i] == 0u && groundTop.has_value()
        && positions[i].z() < minimumStaticGroundHeight(*groundTop)) {
      return false;
    }
  }
  return true;
}

//==============================================================================
void makeInitialPositionsFeasible(
    std::span<Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats* stats)
{
  if (barriers.empty()) {
    return;
  }

  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] == 0u) {
      const auto groundTop = staticGroundTopAt(positions[i], barriers);
      if (groundTop.has_value()) {
        const double previousZ = positions[i].z();
        positions[i].z()
            = std::max(positions[i].z(), minimumStaticGroundHeight(*groundTop));
        if (stats != nullptr && positions[i].z() != previousZ) {
          ++stats->initialProjectionCount;
        }
      }
    }
  }
}

//==============================================================================
double addStaticGroundBarrierEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers,
    comps::DeformableSolverScratch::Vector3Vector* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (barriers.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    const auto groundTop = staticGroundTopAt(positions[i], barriers);
    if (!groundTop.has_value()) {
      continue;
    }

    const double distance = positions[i].z() - *groundTop;
    if (distance <= 0.0 || !std::isfinite(distance)) {
      return std::numeric_limits<double>::infinity();
    }
    if (distance >= activationDistance) {
      continue;
    }

    const double normalizedDistance = distance / activationDistance;
    const double distanceOffset = distance - activationDistance;
    energy += -barrierScale * distanceOffset * distanceOffset
              * std::log(normalizedDistance);

    if (gradient != nullptr) {
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      (*gradient)[i].z() += derivative;
    }
  }

  return energy;
}

//==============================================================================
// Full radial clamped-log barrier for static sphere obstacles: each free node
// within the activation band of a sphere's surface is pushed out along the
// outward radial normal, so a deformable settles smoothly at ~d_hat against any
// side of the sphere (a true 3D contact force, unlike the vertical-only ground
// barrier). Energy + gradient only -- the projected-Newton Hessian, box and
// codimensional obstacles are later increments; the line search on the
// barrier-inclusive energy keeps nodes outside because the clamped-log energy
// diverges as the distance approaches zero.
double addSphereObstacleBarrierEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const SphereObstacleBarrier> obstacles,
    comps::DeformableSolverScratch::Vector3Vector* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (obstacles.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    for (const auto& obstacle : obstacles) {
      const Eigen::Vector3d offset = positions[i] - obstacle.center;
      const double centerDistance = offset.norm();
      const double distance = centerDistance - obstacle.radius;
      if (distance <= 0.0 || !std::isfinite(distance)) {
        return std::numeric_limits<double>::infinity();
      }
      if (distance >= activationDistance || centerDistance <= 0.0) {
        continue;
      }

      const double normalizedDistance = distance / activationDistance;
      const double distanceOffset = distance - activationDistance;
      energy += -barrierScale * distanceOffset * distanceOffset
                * std::log(normalizedDistance);

      if (gradient != nullptr) {
        const double derivative
            = -barrierScale
              * (2.0 * distanceOffset * std::log(normalizedDistance)
                 + distanceOffset * distanceOffset / distance);
        // dDistance/dx is the outward radial unit normal.
        (*gradient)[i] += derivative * (offset / centerDistance);
      }
    }
  }

  return energy;
}

//==============================================================================
// Signed closest-surface distance from a node to an oriented box obstacle, with
// the outward world-frame surface normal set for outside, on-surface, and
// inside nodes. Outside the box, the node is clamped into the box's local
// frame; |local - clamp(local)| is the distance to the surface, uniform across
// face, edge, and corner contact. Inside the box, the nearest exit face
// provides the outward normal and the returned distance is negative penetration
// depth.
double boxObstacleSurfaceDistance(
    const Eigen::Vector3d& position,
    const BoxObstacleBarrier& obstacle,
    Eigen::Vector3d& outwardNormal)
{
  const Eigen::Vector3d local
      = obstacle.rotation.transpose() * (position - obstacle.center);
  const Eigen::Vector3d clamped
      = local.cwiseMax(-obstacle.halfExtents).cwiseMin(obstacle.halfExtents);
  const Eigen::Vector3d delta = local - clamped;
  const double distance = delta.norm();
  if (distance > 0.0 && std::isfinite(distance)) {
    outwardNormal = obstacle.rotation * (delta / distance);
    return distance;
  }

  Eigen::Index nearestAxis = 0;
  double nearestMargin = std::numeric_limits<double>::infinity();
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const double margin = obstacle.halfExtents[axis] - std::abs(local[axis]);
    if (margin < nearestMargin) {
      nearestMargin = margin;
      nearestAxis = axis;
    }
  }
  Eigen::Vector3d localNormal = Eigen::Vector3d::Zero();
  localNormal[nearestAxis] = local[nearestAxis] >= 0.0 ? 1.0 : -1.0;
  outwardNormal = obstacle.rotation * localNormal;
  return -nearestMargin;
}

//==============================================================================
// Closest-surface distance from a node to a capsule obstacle, with the outward
// radial surface normal set when positive. The distance is the point-to-segment
// axis distance minus the radius; the normal points from the closest axis point
// to the node.
double capsuleObstacleSurfaceDistance(
    const Eigen::Vector3d& position,
    const CapsuleObstacleBarrier& obstacle,
    Eigen::Vector3d& outwardNormal)
{
  const Eigen::Vector3d axis = obstacle.pointB - obstacle.pointA;
  const double axisLengthSq = axis.squaredNorm();
  double t = 0.0;
  if (axisLengthSq > 0.0) {
    t = (position - obstacle.pointA).dot(axis) / axisLengthSq;
    t = std::clamp(t, 0.0, 1.0);
  }
  const Eigen::Vector3d closest = obstacle.pointA + t * axis;
  const Eigen::Vector3d delta = position - closest;
  const double axisDistance = delta.norm();
  if (axisDistance > 0.0 && std::isfinite(axisDistance)) {
    outwardNormal = delta / axisDistance;
  }
  return axisDistance - obstacle.radius;
}

//==============================================================================
// Adds the clamped-log barrier energy/gradient pushing free deformable nodes
// out of the activation band of a capsule obstacle, along the outward radial
// normal. The capsule analogue of addBoxObstacleBarrierEnergy.
double addCapsuleObstacleBarrierEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const CapsuleObstacleBarrier> obstacles,
    comps::DeformableSolverScratch::Vector3Vector* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (obstacles.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = capsuleObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || !std::isfinite(distance)) {
        // On or inside the capsule: a penetration the barrier forbids.
        return std::numeric_limits<double>::infinity();
      }
      if (distance >= activationDistance) {
        continue;
      }

      const double normalizedDistance = distance / activationDistance;
      const double distanceOffset = distance - activationDistance;
      energy += -barrierScale * distanceOffset * distanceOffset
                * std::log(normalizedDistance);

      if (gradient != nullptr) {
        const double derivative
            = -barrierScale
              * (2.0 * distanceOffset * std::log(normalizedDistance)
                 + distanceOffset * distanceOffset / distance);
        (*gradient)[i] += derivative * normal;
      }
    }
  }

  return energy;
}

//==============================================================================
// Adds the clamped-log barrier energy/gradient pushing free deformable nodes
// out of the activation band of an oriented box obstacle, along the outward
// surface normal. The box analogue of addSphereObstacleBarrierEnergy.
double addBoxObstacleBarrierEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const BoxObstacleBarrier> obstacles,
    comps::DeformableSolverScratch::Vector3Vector* gradient,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (obstacles.empty()) {
    return 0.0;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = barrierStiffness;

  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = boxObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || !std::isfinite(distance)) {
        // On or inside the box: a penetration the barrier forbids.
        return std::numeric_limits<double>::infinity();
      }
      if (distance >= activationDistance) {
        continue;
      }

      const double normalizedDistance = distance / activationDistance;
      const double distanceOffset = distance - activationDistance;
      energy += -barrierScale * distanceOffset * distanceOffset
                * std::log(normalizedDistance);

      if (gradient != nullptr) {
        const double derivative
            = -barrierScale
              * (2.0 * distanceOffset * std::log(normalizedDistance)
                 + distanceOffset * distanceOffset / distance);
        (*gradient)[i] += derivative * normal;
      }
    }
  }

  return energy;
}

//==============================================================================
// Tangential speed below which static-ground friction smoothly vanishes (the
// IPC mollifier velocity threshold epsv). Scaled by the time step to a
// displacement radius.
double staticGroundFrictionVelocityThreshold()
{
  return 1e-3;
}

//==============================================================================
// Compute the lagged static-ground normal force magnitude per node at the
// current iterate: the upward barrier force |dB/dz| for nodes inside the
// activation band, zero otherwise. Friction lags this across the inner line
// search (standard IPC), so it is evaluated once per outer iteration.
void computeStaticGroundNormalForces(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers,
    auto& normalForce,
    auto& normalDirection)
{
  normalForce.assign(positions.size(), 0.0);
  normalDirection.assign(positions.size(), Eigen::Vector3d::UnitZ());
  if (barriers.empty()) {
    return;
  }

  const double activationDistance = staticGroundBarrierActivationDistance();
  // The lagged friction normal-force estimate uses the base barrier stiffness
  // (adaptive kappa scales the contact barriers, not this approximate force).
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    const auto contact = staticGroundContactAt(positions[i], barriers);
    if (!contact.has_value()) {
      continue;
    }
    const double distance = positions[i].z() - contact->top;
    if (distance <= 0.0 || distance >= activationDistance
        || !std::isfinite(distance)) {
      continue;
    }
    const double distanceOffset = distance - activationDistance;
    const double normalizedDistance = distance / activationDistance;
    const double derivative
        = -barrierScale
          * (2.0 * distanceOffset * std::log(normalizedDistance)
             + distanceOffset * distanceOffset / distance);
    // derivative = dB/dz < 0 (repulsive); the upward normal force is its
    // magnitude. The barrier acts vertically (height field); the geometric
    // surface normal only shapes friction's tangent plane below.
    normalForce[i] = -derivative;
    normalDirection[i] = contact->normal;
  }
}

//==============================================================================
// Merges the capsule obstacle barrier's per-node radial normal force and
// direction into the friction normal-force arrays (populated first by
// computeStaticGroundNormalForces). The dominant (largest-force) contact per
// node wins, so a node resting on a capsule rod gets the rod's radial normal
// for its friction tangent plane. The capsule obstacle is barrier-only (no
// surface CCD), so tangential sliding is unconstrained and friction is
// effective.
void addCapsuleObstacleNormalForces(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const CapsuleObstacleBarrier> obstacles,
    auto& normalForce,
    auto& normalDirection)
{
  if (obstacles.empty()) {
    return;
  }
  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = capsuleObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || distance >= activationDistance
          || !std::isfinite(distance)) {
        continue;
      }
      const double distanceOffset = distance - activationDistance;
      const double normalizedDistance = distance / activationDistance;
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      const double force = -derivative;
      if (force > normalForce[i]) {
        normalForce[i] = force;
        normalDirection[i] = normal;
      }
    }
  }
}

//==============================================================================
// Merges the sphere obstacle barrier's per-node radial normal force/direction
// into the friction normal-force arrays (dominant contact per node wins). Used
// for friction against barrier-only sphere obstacles.
void addSphereObstacleNormalForces(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const SphereObstacleBarrier> obstacles,
    auto& normalForce,
    auto& normalDirection)
{
  if (obstacles.empty()) {
    return;
  }
  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (const auto& obstacle : obstacles) {
      const Eigen::Vector3d offset = positions[i] - obstacle.center;
      const double centerDistance = offset.norm();
      const double distance = centerDistance - obstacle.radius;
      if (distance <= 0.0 || distance >= activationDistance
          || centerDistance <= 0.0 || !std::isfinite(distance)) {
        continue;
      }
      const double distanceOffset = distance - activationDistance;
      const double normalizedDistance = distance / activationDistance;
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      const double force = -derivative;
      if (force > normalForce[i]) {
        normalForce[i] = force;
        normalDirection[i] = offset / centerDistance;
      }
    }
  }
}

//==============================================================================
// Merges the box obstacle barrier's per-node surface normal force/direction
// into the friction normal-force arrays (dominant contact per node wins). Used
// for friction against barrier-only box obstacles.
void addBoxObstacleNormalForces(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const BoxObstacleBarrier> obstacles,
    auto& normalForce,
    auto& normalDirection)
{
  if (obstacles.empty()) {
    return;
  }
  const double activationDistance = staticGroundBarrierActivationDistance();
  const double barrierScale = kDefaultBarrierStiffness;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (const auto& obstacle : obstacles) {
      Eigen::Vector3d normal;
      const double distance
          = boxObstacleSurfaceDistance(positions[i], obstacle, normal);
      if (distance <= 0.0 || distance >= activationDistance
          || !std::isfinite(distance)) {
        continue;
      }
      const double distanceOffset = distance - activationDistance;
      const double normalizedDistance = distance / activationDistance;
      const double derivative
          = -barrierScale
            * (2.0 * distanceOffset * std::log(normalizedDistance)
               + distanceOffset * distanceOffset / distance);
      const double force = -derivative;
      if (force > normalForce[i]) {
        normalForce[i] = force;
        normalDirection[i] = normal;
      }
    }
  }
}

//==============================================================================
// Lagged smoothed Coulomb friction inputs for static-ground contact. The
// per-node normal force and the step-start positions are fixed across the inner
// line search; the tangential displacement uses the candidate positions.
struct GroundFrictionInputs
{
  double coefficient = 0.0; // mu
  double epsilon = 0.0;     // epsv * timeStep (mollifier displacement radius)
  std::span<const Eigen::Vector3d> stepStartPositions;
  std::span<const double> laggedNormalForce;
  // Per-node geometric ground normal at the lagged contact (unit, upward). When
  // empty the tangent plane defaults to xy (flat ground), preserving the legacy
  // behavior exactly.
  std::span<const Eigen::Vector3d> laggedNormalDirection;
};

//==============================================================================
// IPC smoothed-friction mollifier (Li et al. 2020): f0 is the friction
// potential profile and f1 = f0' the force profile. Both are C1 with f1 -> 1
// (kinetic) for tangential displacement beyond the threshold and a smooth ramp
// to zero below it.
double frictionF0(double y, double epsilon)
{
  if (y >= epsilon) {
    return y;
  }
  return y * y / epsilon - y * y * y / (3.0 * epsilon * epsilon)
         + epsilon / 3.0;
}

double frictionF1(double y, double epsilon)
{
  if (y >= epsilon) {
    return 1.0;
  }
  return 2.0 * y / epsilon - y * y / (epsilon * epsilon);
}

//==============================================================================
// Add the lagged smoothed Coulomb friction energy/gradient for static-ground
// contact. The tangent plane is the plane orthogonal to the lagged geometric
// ground normal n: the friction opposes the node's step displacement projected
// into that plane, u_T = (I - n n^T) (x - x_start). For flat/box-top ground
// n = +z and u_T is the xy displacement, recovering the legacy behavior; a
// sphere or tilted box gives a tilted tangent plane. The force magnitude
// saturates at mu * normalForce (kinetic) and ramps smoothly to zero at rest,
// so there is no division by zero.
double addGroundFrictionEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    const GroundFrictionInputs& friction,
    comps::DeformableSolverScratch::Vector3Vector* gradient)
{
  if (friction.coefficient <= 0.0 || friction.epsilon <= 0.0
      || friction.stepStartPositions.empty()
      || friction.laggedNormalForce.empty()) {
    return 0.0;
  }

  const auto start = friction.stepStartPositions;
  const auto normalForce = friction.laggedNormalForce;
  const auto normalDirection = friction.laggedNormalDirection;
  double energy = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u || normalForce[i] <= 0.0) {
      continue;
    }
    const Eigen::Vector3d n = (i < normalDirection.size())
                                  ? normalDirection[i]
                                  : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d u = positions[i] - start[i];
    const Eigen::Vector3d tangent = u - n.dot(u) * n;
    const double y = tangent.norm();
    const double scale = friction.coefficient * normalForce[i];
    energy += scale * frictionF0(y, friction.epsilon);
    if (gradient != nullptr) {
      // grad = mu * normalForce * f1(y) * u_T / ||u_T||, which lies in the
      // tangent plane. As y -> 0, f1(y)/y -> 2/epsilon, so the force vanishes
      // smoothly at rest.
      constexpr double tiny = 1e-12;
      const double ratio = (y > tiny) ? frictionF1(y, friction.epsilon) / y
                                      : 2.0 / friction.epsilon;
      (*gradient)[i] += scale * ratio * tangent;
    }
  }
  return energy;
}

//==============================================================================
// Activation distance d_hat for the self-contact barrier. The barrier is active
// only when a contact pair's distance is below this value.
double selfContactBarrierActivationDistance()
{
  return 2e-2;
}

// Fixed barrier stiffness (kappa). Adaptive stiffness is a later slice.
double selfContactBarrierStiffness()
{
  return 1e5;
}

// Inputs for the IPC clamped-log self-contact barrier energy term. Null/zero
// fields disable the term, preserving the contact-free objective exactly.
struct SelfContactBarrierInputs
{
  const dc::ContactCandidateSet* candidates = nullptr;
  std::span<const DeformableSurfaceTriangle> triangles;
  double squaredActivationDistance = 0.0;
  double stiffness = 0.0;
};

// Adds the IPC clamped-log barrier energy/gradient over the active self-contact
// candidate set (point-triangle and edge-edge). The barrier kernel already
// returns position-space derivatives (a 12-vector over the four primitive
// points), so each contact's gradient is scattered into its four nodes. This
// produces smooth repulsive contact forces; the CCD limiters remain the hard
// no-penetration guarantee.
double addSelfContactBarrierEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    const SelfContactBarrierInputs& inputs,
    std::size_t* activeContacts,
    comps::DeformableSolverScratch::Vector3Vector* gradient)
{
  if (inputs.candidates == nullptr || inputs.triangles.empty()
      || inputs.stiffness <= 0.0 || !(inputs.squaredActivationDistance > 0.0)) {
    return 0.0;
  }

  const auto& candidates = *inputs.candidates;
  const auto triangles = inputs.triangles;
  double energy = 0.0;

  const auto scatter = [&](const dc::PrimitiveBarrierResult& result,
                           const std::array<std::size_t, 4>& nodes) {
    energy += result.value;
    if (activeContacts != nullptr) {
      ++(*activeContacts);
    }
    if (gradient == nullptr) {
      return;
    }
    for (int k = 0; k < 4; ++k) {
      if (fixed[nodes[k]] != 0u) {
        continue;
      }
      (*gradient)[nodes[k]] += result.gradient.segment<3>(3 * k);
    }
  };

  for (const auto& candidate : candidates.pointTriangleCandidates) {
    const auto& triangle = triangles[candidate.triangle];
    const auto result = dc::pointTriangleBarrier(
        positions[candidate.point],
        positions[triangle.nodeA],
        positions[triangle.nodeB],
        positions[triangle.nodeC],
        inputs.squaredActivationDistance,
        inputs.stiffness);
    if (!result.active) {
      continue;
    }
    scatter(
        result,
        {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC});
  }

  // Edge-edge uses the plain (non-mollified) barrier. The IPC edge-edge
  // mollifier that smooths nearly-parallel configurations is intentionally
  // deferred: it pairs with the projected-Newton slice that also needs the
  // mollified Hessian. The plain barrier is finite and safe here; the CCD
  // limiter remains the hard no-penetration gate.
  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    const auto result = dc::edgeEdgeBarrier(
        positions[edgeA.nodeA],
        positions[edgeA.nodeB],
        positions[edgeB.nodeA],
        positions[edgeB.nodeB],
        inputs.squaredActivationDistance,
        inputs.stiffness);
    if (!result.active) {
      continue;
    }
    scatter(result, {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB});
  }

  return energy;
}

struct SelfContactFrictionInputs
{
  double coefficient = 0.0; // mu
  double epsilon = 0.0;     // epsv * timeStep
  std::span<const Eigen::Vector3d> stepStartPositions;
  std::span<const SelfContactFrictionContact> contacts;
};

// Assemble the lagged self-contact friction set from the active point-triangle
// and edge-edge barrier candidates at the current iterate. The lagged normal
// force is the barrier force magnitude on the primitive's first feature (the
// point node, or the net force on the first edge), and the tangent projection
// comes from the matching point-triangle / edge-edge tangent stencil. The
// downstream friction energy/gradient/Hessian are generic over the four-node
// stencil, so both contact types share them.
void buildSelfContactFrictionContacts(
    std::span<const Eigen::Vector3d> positions,
    const SelfContactBarrierInputs& barrier,
    auto& contacts)
{
  contacts.clear();
  if (barrier.candidates == nullptr || barrier.triangles.empty()
      || barrier.stiffness <= 0.0
      || !(barrier.squaredActivationDistance > 0.0)) {
    return;
  }

  const auto& candidates = *barrier.candidates;
  const auto triangles = barrier.triangles;
  for (const auto& candidate : candidates.pointTriangleCandidates) {
    const auto& triangle = triangles[candidate.triangle];
    const auto& p = positions[candidate.point];
    const auto& a = positions[triangle.nodeA];
    const auto& b = positions[triangle.nodeB];
    const auto& c = positions[triangle.nodeC];
    const auto result = dc::pointTriangleBarrier(
        p, a, b, c, barrier.squaredActivationDistance, barrier.stiffness);
    if (!result.active) {
      continue;
    }
    SelfContactFrictionContact contact;
    contact.nodes
        = {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC};
    contact.normalForce = result.gradient.template head<3>().norm();
    contact.projection = dc::pointTriangleTangentStencil(p, a, b, c).projection;
    contacts.push_back(contact);
  }

  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    const auto& a0 = positions[edgeA.nodeA];
    const auto& a1 = positions[edgeA.nodeB];
    const auto& b0 = positions[edgeB.nodeA];
    const auto& b1 = positions[edgeB.nodeB];
    const auto result = dc::edgeEdgeBarrier(
        a0, a1, b0, b1, barrier.squaredActivationDistance, barrier.stiffness);
    if (!result.active) {
      continue;
    }
    SelfContactFrictionContact contact;
    contact.nodes = {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB};
    // Lagged normal force = net barrier force on edge A (its two endpoints).
    contact.normalForce = (result.gradient.template head<3>()
                           + result.gradient.template segment<3>(3))
                              .norm();
    contact.projection = dc::edgeEdgeTangentStencil(a0, a1, b0, b1).projection;
    contacts.push_back(contact);
  }
}

// Add lagged smoothed Coulomb self-contact friction energy/gradient over the
// active point-triangle contacts. The tangential relative displacement is
// projection * (stacked four-node displacement over the step); the IPC f0/f1
// mollifier gives a C1 force opposing it (saturating at mu * normalForce). The
// lagged friction Hessian is a later increment, like the ground-friction path's
// first cut; the line search on this energy still ensures descent.
double addSelfContactFrictionEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    const SelfContactFrictionInputs& friction,
    comps::DeformableSolverScratch::Vector3Vector* gradient)
{
  if (friction.coefficient <= 0.0 || friction.epsilon <= 0.0
      || friction.stepStartPositions.empty() || friction.contacts.empty()) {
    return 0.0;
  }

  const auto start = friction.stepStartPositions;
  double energy = 0.0;
  for (const auto& contact : friction.contacts) {
    if (contact.normalForce <= 0.0) {
      continue;
    }
    Eigen::Matrix<double, 12, 1> displacement;
    for (int k = 0; k < 4; ++k) {
      displacement.segment<3>(3 * k)
          = positions[contact.nodes[k]] - start[contact.nodes[k]];
    }
    const Eigen::Vector2d tangent = contact.projection * displacement;
    const double y = tangent.norm();
    const double scale = friction.coefficient * contact.normalForce;
    energy += scale * frictionF0(y, friction.epsilon);
    if (gradient != nullptr) {
      constexpr double tiny = 1e-12;
      const double ratio = (y > tiny) ? frictionF1(y, friction.epsilon) / y
                                      : 2.0 / friction.epsilon;
      const Eigen::Matrix<double, 12, 1> g
          = scale * ratio * (contact.projection.transpose() * tangent);
      for (int k = 0; k < 4; ++k) {
        if (fixed[contact.nodes[k]] != 0u) {
          continue;
        }
        (*gradient)[contact.nodes[k]] += g.segment<3>(3 * k);
      }
    }
  }
  return energy;
}

//==============================================================================
// Friction diagnostics at the converged iterate, over both static-ground and
// self-contact friction. Accumulates the IPC Coulomb dissipation
// mu * lambda * f1(y) * y (force times tangential slip; equal to mu * lambda *
// y in the kinetic regime and ramped smoothly to zero at rest by the mollifier)
// and the count of contacts carrying a nonzero lagged normal force. Evaluated
// once per step outside the line-search hot path, mirroring the slip measures
// the friction energy uses: u_T = (I - n n^T)(x - x_start) for ground contact
// and projection * (stacked four-node displacement) for self-contact.
void accumulateFrictionDiagnostics(
    std::span<const Eigen::Vector3d> positions,
    std::span<const Eigen::Vector3d> stepStart,
    std::span<const std::uint8_t> fixed,
    const double frictionCoefficient,
    const double epsilon,
    const auto& groundNormalForce,
    const auto& groundNormalDirection,
    const auto& selfContacts,
    double& dissipation,
    std::size_t& activeContacts)
{
  if (frictionCoefficient <= 0.0 || epsilon <= 0.0) {
    return;
  }

  for (std::size_t i = 0; i < groundNormalForce.size() && i < positions.size();
       ++i) {
    if (fixed[i] != 0u || groundNormalForce[i] <= 0.0) {
      continue;
    }
    const Eigen::Vector3d n = (i < groundNormalDirection.size())
                                  ? groundNormalDirection[i]
                                  : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d u = positions[i] - stepStart[i];
    const double y = (u - n.dot(u) * n).norm();
    const auto contribution = nb::frictionWorkContribution(
        y, frictionCoefficient * groundNormalForce[i], epsilon);
    if (contribution.active) {
      dissipation += contribution.work;
      ++activeContacts;
    }
  }

  for (const auto& contact : selfContacts) {
    if (contact.normalForce <= 0.0) {
      continue;
    }
    Eigen::Matrix<double, 12, 1> displacement;
    for (int k = 0; k < 4; ++k) {
      displacement.segment<3>(3 * k)
          = positions[contact.nodes[k]] - stepStart[contact.nodes[k]];
    }
    const double y = (contact.projection * displacement).norm();
    const auto contribution = nb::frictionWorkContribution(
        y, frictionCoefficient * contact.normalForce, epsilon);
    if (contribution.active) {
      dissipation += contribution.work;
      ++activeContacts;
    }
  }
}

//==============================================================================
// Closest-approach diagnostic over the active self-contact barrier set at the
// converged iterate. Each candidate's point-triangle / edge-edge squared
// distance is recomputed at the terminal positions; candidates within the
// activation band (squared distance < d_hat^2) form the active set, and the
// smallest distance among them is the IPC intersection-free "minimum distance"
// statistic. Returns the active-set size and writes the closest distance (0
// when the set is empty). Read once after the outer loop, not on the
// line-search hot path.
std::size_t accumulateContactDistanceDiagnostics(
    std::span<const Eigen::Vector3d> positions,
    const SelfContactBarrierInputs& barrier,
    double& outMinDistance)
{
  outMinDistance = 0.0;
  if (barrier.candidates == nullptr || barrier.triangles.empty()
      || !(barrier.squaredActivationDistance > 0.0)) {
    return 0;
  }

  const auto& candidates = *barrier.candidates;
  const auto triangles = barrier.triangles;
  double minSquared = std::numeric_limits<double>::infinity();
  std::size_t activeContacts = 0;

  for (const auto& candidate : candidates.pointTriangleCandidates) {
    const auto& triangle = triangles[candidate.triangle];
    const double squaredDistance = dc::pointTriangleSquaredDistance(
                                       positions[candidate.point],
                                       positions[triangle.nodeA],
                                       positions[triangle.nodeB],
                                       positions[triangle.nodeC])
                                       .squaredDistance;
    if (squaredDistance >= barrier.squaredActivationDistance) {
      continue;
    }
    ++activeContacts;
    minSquared = std::min(minSquared, squaredDistance);
  }

  for (const auto& candidate : candidates.edgeEdgeCandidates) {
    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    const double squaredDistance = dc::edgeEdgeSquaredDistance(
                                       positions[edgeA.nodeA],
                                       positions[edgeA.nodeB],
                                       positions[edgeB.nodeA],
                                       positions[edgeB.nodeB])
                                       .squaredDistance;
    if (squaredDistance >= barrier.squaredActivationDistance) {
      continue;
    }
    ++activeContacts;
    minSquared = std::min(minSquared, squaredDistance);
  }

  if (activeContacts > 0 && std::isfinite(minSquared)) {
    outMinDistance = std::sqrt(std::max(0.0, minSquared));
  }
  return activeContacts;
}

//==============================================================================
// Stable neo-Hookean FEM elasticity inputs. When non-null (opt-in via
// DeformableMaterial.useFiniteElementElasticity), each tetrahedron contributes
// a volumetric strain energy/gradient/Hessian instead of the mass-spring edge
// model. Null preserves the spring objective exactly.
struct FemElasticityInputs
{
  std::span<const comps::DeformableTetrahedron> tetrahedra;
  std::span<const fem::TetRestShape> restShapes;
  fem::LameParameters lame;
  // Selects the isotropic material: false (default) is the inversion-robust
  // stable neo-Hookean kernel; true is fixed-corotational (the IPC paper's
  // other material), opt-in via
  // DeformableMaterial.useFixedCorotationalElasticity.
  bool fixedCorotational = false;
};

// Dispatches one tetrahedron to the configured FEM material kernel. Both
// kernels share the TetElementResult shape, so the rest of the assembly is
// identical.
inline fem::TetElementResult evaluateFemTetElement(
    const FemElasticityInputs& inputs,
    std::span<const Eigen::Vector3d> positions,
    const comps::DeformableTetrahedron& tet,
    const fem::TetRestShape& rest,
    const bool computeHessian)
{
  if (inputs.fixedCorotational) {
    return fem::evaluateFixedCorotationalTet(
        positions[tet.nodeA],
        positions[tet.nodeB],
        positions[tet.nodeC],
        positions[tet.nodeD],
        rest,
        inputs.lame,
        computeHessian);
  }
  return fem::evaluateStableNeoHookeanTet(
      positions[tet.nodeA],
      positions[tet.nodeB],
      positions[tet.nodeC],
      positions[tet.nodeD],
      rest,
      inputs.lame,
      computeHessian);
}

// Adds the stable neo-Hookean strain energy/gradient over every valid
// tetrahedron, scattering each element's 12-vector force gradient into its four
// nodes (fixed nodes receive no gradient, like the spring term).
double addFemElasticityEnergy(
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    const FemElasticityInputs& inputs,
    comps::DeformableSolverScratch::Vector3Vector* gradient)
{
  if (inputs.tetrahedra.empty() || inputs.restShapes.empty()) {
    return 0.0;
  }
  const auto tets = inputs.tetrahedra;
  const auto rests = inputs.restShapes;
  const std::size_t count = std::min(tets.size(), rests.size());
  double energy = 0.0;
  for (std::size_t t = 0; t < count; ++t) {
    const auto& tet = tets[t];
    const fem::TetElementResult element = evaluateFemTetElement(
        inputs, positions, tet, rests[t], /*computeHessian=*/false);
    if (!element.valid) {
      continue;
    }
    energy += element.energy;
    if (gradient != nullptr) {
      const std::array<std::size_t, 4> nodes
          = {tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD};
      for (int i = 0; i < 4; ++i) {
        if (fixed[nodes[i]] == 0u) {
          (*gradient)[nodes[i]] += element.gradient.segment<3>(3 * i);
        }
      }
    }
  }
  return energy;
}

double evaluateDeformableObjective(
    const comps::DeformableNodeModel& nodeModel,
    const comps::DeformableSpringModel& model,
    std::span<const Eigen::Vector3d> positions,
    std::span<const Eigen::Vector3d> inertialTargets,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers,
    std::span<const SphereObstacleBarrier> sphereObstacles,
    std::span<const BoxObstacleBarrier> boxObstacles,
    std::span<const CapsuleObstacleBarrier> capsuleObstacles,
    double timeStep,
    comps::DeformableSolverScratch::Vector3Vector* gradient,
    const SelfContactBarrierInputs* contactBarrier = nullptr,
    std::size_t* barrierActiveContacts = nullptr,
    const GroundFrictionInputs* groundFriction = nullptr,
    const SelfContactFrictionInputs* selfContactFriction = nullptr,
    const FemElasticityInputs* femElasticity = nullptr,
    double barrierStiffness = kDefaultBarrierStiffness)
{
  if (gradient != nullptr) {
    if (gradient->size() != positions.size()) {
      gradient->resize(positions.size());
    }
    std::fill(gradient->begin(), gradient->end(), Eigen::Vector3d::Zero());
  }

  double energy = 0.0;
  const double invDt2 = 1.0 / (timeStep * timeStep);
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (fixed[i] != 0u) {
      continue;
    }

    const Eigen::Vector3d delta = positions[i] - inertialTargets[i];
    energy += 0.5 * nodeModel.masses[i] * invDt2 * delta.squaredNorm();
    if (gradient != nullptr) {
      (*gradient)[i] += nodeModel.masses[i] * invDt2 * delta;
    }
  }

  constexpr double minLength = 1e-12;
  for (const auto& edge : model.edges) {
    const Eigen::Vector3d delta = positions[edge.nodeB] - positions[edge.nodeA];
    const double length = delta.norm();
    if (length <= minLength || !std::isfinite(length)) {
      continue;
    }

    const Eigen::Vector3d direction = delta / length;
    const double stretch = length - edge.restLength;
    energy += 0.5 * model.stiffness * stretch * stretch;
    if (gradient != nullptr) {
      const Eigen::Vector3d springGradient
          = model.stiffness * stretch * direction;
      if (fixed[edge.nodeA] == 0u) {
        (*gradient)[edge.nodeA] -= springGradient;
      }
      if (fixed[edge.nodeB] == 0u) {
        (*gradient)[edge.nodeB] += springGradient;
      }
    }
  }

  if (femElasticity != nullptr) {
    energy
        += addFemElasticityEnergy(positions, fixed, *femElasticity, gradient);
  }
  energy += addStaticGroundBarrierEnergy(
      positions, fixed, barriers, gradient, barrierStiffness);
  energy += addSphereObstacleBarrierEnergy(
      positions, fixed, sphereObstacles, gradient, barrierStiffness);
  energy += addBoxObstacleBarrierEnergy(
      positions, fixed, boxObstacles, gradient, barrierStiffness);
  energy += addCapsuleObstacleBarrierEnergy(
      positions, fixed, capsuleObstacles, gradient, barrierStiffness);
  if (contactBarrier != nullptr) {
    energy += addSelfContactBarrierEnergy(
        positions, fixed, *contactBarrier, barrierActiveContacts, gradient);
  }
  if (groundFriction != nullptr) {
    energy
        += addGroundFrictionEnergy(positions, fixed, *groundFriction, gradient);
  }
  if (selfContactFriction != nullptr) {
    energy += addSelfContactFrictionEnergy(
        positions, fixed, *selfContactFriction, gradient);
  }
  return energy;
}

//==============================================================================
double gradientNormSquared(
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed)
{
  double normSquared = 0.0;
  for (std::size_t i = 0; i < gradient.size(); ++i) {
    if (fixed[i] == 0u) {
      normSquared += gradient[i].squaredNorm();
    }
  }
  return normSquared;
}

//==============================================================================
double buildLineSearchCandidate(
    std::span<const Eigen::Vector3d> current,
    std::span<const Eigen::Vector3d> direction,
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed,
    double step,
    comps::DeformableSolverScratch::Vector3Vector& candidate)
{
  double directionalDerivative = 0.0;
  for (std::size_t i = 0; i < current.size(); ++i) {
    candidate[i] = current[i];
    if (fixed[i] == 0u) {
      candidate[i] += step * direction[i];
      directionalDerivative += gradient[i].dot(candidate[i] - current[i]);
    }
  }
  return directionalDerivative;
}

//==============================================================================
bool applySurfaceContactCcdLimit(
    std::span<const Eigen::Vector3d> current,
    std::span<const Eigen::Vector3d> direction,
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    comps::DeformableSolverScratch::Vector3Vector& candidate,
    double& directionalDerivative)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return true;
  }

  ++stats.surfaceContactCandidateBuilds;
  dc::buildMotionAwareContactCandidatesSweep(
      current,
      candidate,
      contactScratch.surfaceTriangles,
      makeSurfaceContactCandidateOptions(
          contactScratch.selfSurfaceCandidateCapacity,
          contactScratch.surfaceContactPointMask,
          contactScratch.surfaceCandidateGrowthAllowed),
      contactScratch.candidates,
      contactScratch.sweepScratch);
  filterSurfaceContactPointCandidates(
      contactScratch.candidates, contactScratch.surfaceContactPointMask);
  recordSurfaceCandidateCount(
      stats,
      contactScratch.candidates.pointTriangleCandidates.size(),
      contactScratch.candidates.edgeEdgeCandidates.size(),
      contactScratch.candidates.stats.capacityOverflowCount);
  const std::size_t pointTrianglePairCapacity = checkedSurfaceCandidateMultiply(
      countSurfaceContactCandidatePoints(
          contactScratch.candidates.stats.pointCount,
          contactScratch.surfaceContactPointMask),
      contactScratch.candidates.stats.triangleCount,
      "self point-triangle pair diagnostics");
  const std::size_t edgeEdgePairCapacity = checkedSurfaceCandidateMultiply(
      contactScratch.candidates.stats.edgeCount,
      contactScratch.candidates.stats.edgeCount,
      "self edge-edge pair diagnostics");
  stats.surfaceContactPointTriangleCandidates
      += contactScratch.candidates.pointTriangleCandidates.size();
  stats.surfaceContactEdgeEdgeCandidates
      += contactScratch.candidates.edgeEdgeCandidates.size();
  accumulateCandidateFilterPressure(
      stats.surfaceContactCandidatePairCapacity,
      stats.surfaceContactCandidateRejectedPairs,
      pointTrianglePairCapacity,
      edgeEdgePairCapacity,
      contactScratch.candidates.pointTriangleCandidates.size(),
      contactScratch.candidates.edgeEdgeCandidates.size());

  const auto result = dc::contactCandidateStepBound(
      current,
      candidate,
      contactScratch.surfaceTriangles,
      contactScratch.candidates,
      makeSurfaceContactCcdOptions());
  stats.surfaceContactCcdPointTriangleChecks
      += result.stats.pointTriangleChecks;
  stats.surfaceContactCcdEdgeEdgeChecks += result.stats.edgeEdgeChecks;
  stats.surfaceContactCcdHits += result.stats.hits;
  stats.surfaceContactCcdMisses += result.stats.misses;
  stats.surfaceContactCcdIndeterminateCount += result.stats.indeterminate;
  stats.surfaceContactCcdZeroStepCount += result.stats.zeroStepCount;

  if (!result.allowsPositiveStep()) {
    return false;
  }

  if (!result.hit) {
    return true;
  }

  const double safeFraction
      = nb::makeInteriorLineSearchStepScale(result.stepBound);
  if (safeFraction <= 0.0) {
    return false;
  }

  step *= safeFraction;
  ++stats.surfaceContactCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
bool applyInterBodySurfaceContactCcdLimit(
    entt::entity entity,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const Eigen::Vector3d> current,
    std::span<const Eigen::Vector3d> direction,
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    comps::DeformableSolverScratch::Vector3Vector& candidate,
    double& directionalDerivative)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return true;
  }

  bool hasObstacleSurface = false;
  for (const auto& snapshot : surfaceSnapshots) {
    if (snapshot.entity != entity && !snapshot.surfaceTriangles.empty()) {
      hasObstacleSurface = true;
      break;
    }
  }
  if (!hasObstacleSurface) {
    return true;
  }

  dc::buildUniqueSurfaceEdges(
      contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);

  ++stats.interBodySurfaceContactCandidateBuilds;
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;

  for (const auto& snapshot : surfaceSnapshots) {
    if (snapshot.entity == entity || snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        current,
        candidate,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        makeSurfaceContactCandidateOptions(
            contactScratch.resolvedSurfaceCandidateCapacity,
            {},
            contactScratch.surfaceCandidateGrowthAllowed),
        makeSurfaceContactCcdOptions(),
        contactScratch);

    stats.interBodySurfaceContactPointTriangleCandidates
        += result.pointTriangleCandidateCount;
    stats.interBodySurfaceContactEdgeEdgeCandidates
        += result.edgeEdgeCandidateCount;
    recordSurfaceCandidateCount(
        stats,
        result.pointTriangleCandidateCount,
        result.edgeEdgeCandidateCount,
        result.capacityOverflowCount);
    accumulateCandidateFilterPressure(
        stats.interBodySurfaceContactCandidatePairCapacity,
        stats.interBodySurfaceContactCandidateRejectedPairs,
        result.pointTrianglePairCapacity,
        result.edgeEdgePairCapacity,
        result.pointTriangleCandidateCount,
        result.edgeEdgeCandidateCount);
    stats.interBodySurfaceContactCcdPointTriangleChecks
        += result.stats.pointTriangleChecks;
    stats.interBodySurfaceContactCcdEdgeEdgeChecks
        += result.stats.edgeEdgeChecks;
    stats.interBodySurfaceContactCcdHits += result.stats.hits;
    stats.interBodySurfaceContactCcdMisses += result.stats.misses;
    stats.interBodySurfaceContactCcdIndeterminateCount
        += result.stats.indeterminate;
    stats.interBodySurfaceContactCcdZeroStepCount += result.stats.zeroStepCount;

    indeterminate = indeterminate || result.indeterminate;
    if (result.indeterminate) {
      stepBound = 0.0;
    }
    if (result.hit && (!hit || result.stepBound < stepBound)) {
      hit = true;
      stepBound = result.stepBound;
    }
  }

  if (indeterminate) {
    return false;
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    return false;
  }

  step *= safeFraction;
  ++stats.interBodySurfaceContactCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
// Apply the self-surface CCD limiter to a fully assembled candidate
// displacement. VBD uses this after its block solve so fast same-body surface
// crossings keep the same no-tunneling limit as the default line-search path.
bool applySurfaceContactCcdCandidateLimit(
    std::span<const Eigen::Vector3d> current,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    comps::DeformableSolverScratch& scratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return true;
  }

  const std::size_t nodeCount = current.size();
  scratch.direction.resize(nodeCount);
  scratch.gradient.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.candidate = scratch.next;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.direction[i] = scratch.candidate[i] - current[i];
  }

  double step = 1.0;
  double directionalDerivative = 0.0;
  const bool accepted = applySurfaceContactCcdLimit(
      current,
      scratch.direction,
      scratch.gradient,
      fixed,
      contactScratch,
      stats,
      step,
      scratch.candidate,
      directionalDerivative);

  if (accepted) {
    scratch.next = scratch.candidate;
    return true;
  }

  scratch.next.assign(current.begin(), current.end());
  return false;
}

//==============================================================================
// Apply the inter-body deformable-surface CCD limiter to a fully assembled
// candidate displacement. VBD uses this after its block solve so opt-in
// surface bodies keep the same no-tunneling limit the default line-search path
// applies between deformable bodies.
bool applyInterBodySurfaceContactCcdCandidateLimit(
    entt::entity entity,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const Eigen::Vector3d> current,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    comps::DeformableSolverScratch& scratch)
{
  if (surfaceSnapshots.empty()) {
    return true;
  }

  const std::size_t nodeCount = current.size();
  scratch.direction.resize(nodeCount);
  scratch.gradient.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.candidate = scratch.next;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.direction[i] = scratch.candidate[i] - current[i];
  }

  double step = 1.0;
  double directionalDerivative = 0.0;
  const bool accepted = applyInterBodySurfaceContactCcdLimit(
      entity,
      surfaceSnapshots,
      current,
      scratch.direction,
      scratch.gradient,
      fixed,
      contactScratch,
      stats,
      step,
      scratch.candidate,
      directionalDerivative);

  if (accepted) {
    scratch.next = scratch.candidate;
    return true;
  }

  scratch.next.assign(current.begin(), current.end());
  return false;
}

//==============================================================================
bool applyStaticRigidSurfaceCcdLimit(
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const Eigen::Vector3d> current,
    std::span<const Eigen::Vector3d> direction,
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    comps::DeformableSolverScratch::Vector3Vector& candidate,
    double& directionalDerivative)
{
  if (rigidSurfaceSnapshots.empty()) {
    return true;
  }

  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);
  }

  ++stats.staticRigidSurfaceCcdCandidateBuilds;
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;

  for (const auto& snapshot : rigidSurfaceSnapshots) {
    if (snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        current,
        candidate,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        makeSurfaceContactCandidateOptions(
            contactScratch.resolvedSurfaceCandidateCapacity,
            {},
            contactScratch.surfaceCandidateGrowthAllowed),
        makeSurfaceContactCcdOptions(),
        contactScratch);

    stats.staticRigidSurfaceCcdPointTriangleCandidates
        += result.pointTriangleCandidateCount;
    stats.staticRigidSurfaceCcdEdgeEdgeCandidates
        += result.edgeEdgeCandidateCount;
    recordSurfaceCandidateCount(
        stats,
        result.pointTriangleCandidateCount,
        result.edgeEdgeCandidateCount,
        result.capacityOverflowCount);
    accumulateCandidateFilterPressure(
        stats.staticRigidSurfaceCcdCandidatePairCapacity,
        stats.staticRigidSurfaceCcdCandidateRejectedPairs,
        result.pointTrianglePairCapacity,
        result.edgeEdgePairCapacity,
        result.pointTriangleCandidateCount,
        result.edgeEdgeCandidateCount);
    stats.staticRigidSurfaceCcdPointTriangleChecks
        += result.stats.pointTriangleChecks;
    stats.staticRigidSurfaceCcdEdgeEdgeChecks += result.stats.edgeEdgeChecks;
    stats.staticRigidSurfaceCcdHits += result.stats.hits;
    stats.staticRigidSurfaceCcdMisses += result.stats.misses;
    stats.staticRigidSurfaceCcdIndeterminateCount += result.stats.indeterminate;
    stats.staticRigidSurfaceCcdZeroStepCount += result.stats.zeroStepCount;

    indeterminate = indeterminate || result.indeterminate;
    if (result.indeterminate) {
      stepBound = 0.0;
    }
    if (result.hit && (!hit || result.stepBound < stepBound)) {
      hit = true;
      stepBound = result.stepBound;
    }
  }

  if (indeterminate) {
    return false;
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    ++stats.staticRigidSurfaceCcdZeroStepCount;
    return false;
  }

  step *= safeFraction;
  ++stats.staticRigidSurfaceCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
// Apply the static rigid-surface CCD limiter to a fully assembled candidate
// displacement, such as the World VBD path's post-solve iterate. This reuses
// the default solver's step-bound machinery without coupling VBD to line-search
// gradients.
bool applyStaticRigidSurfaceCcdCandidateLimit(
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const Eigen::Vector3d> current,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    comps::DeformableSolverScratch& scratch)
{
  if (rigidSurfaceSnapshots.empty()) {
    return true;
  }

  const std::size_t nodeCount = current.size();
  scratch.direction.resize(nodeCount);
  scratch.gradient.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.candidate = scratch.next;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.direction[i] = scratch.candidate[i] - current[i];
  }

  double step = 1.0;
  double directionalDerivative = 0.0;
  const bool accepted = applyStaticRigidSurfaceCcdLimit(
      rigidSurfaceSnapshots,
      current,
      scratch.direction,
      scratch.gradient,
      fixed,
      contactScratch,
      stats,
      step,
      scratch.candidate,
      directionalDerivative);

  if (accepted) {
    scratch.next = scratch.candidate;
    return true;
  }

  scratch.next.assign(current.begin(), current.end());
  return false;
}

//==============================================================================
// Conservative CCD limiter against MOVING rigid box obstacles. The snapshots
// are static poses sampled along the obstacle's predicted swept motion, so this
// reuses exactly the static-obstacle step-bound path; only the stat family
// differs. Limiting against every sampled pose keeps the deformable out of the
// obstacle's swept corridor.
bool applyMovingRigidSurfaceCcdLimit(
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    std::span<const Eigen::Vector3d> current,
    std::span<const Eigen::Vector3d> direction,
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed,
    DeformableContactSolverScratch& contactScratch,
    DeformableSolverStats& stats,
    double& step,
    comps::DeformableSolverScratch::Vector3Vector& candidate,
    double& directionalDerivative)
{
  if (movingRigidSurfaceSnapshots.empty()) {
    return true;
  }

  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);
  }

  ++stats.movingRigidSurfaceCcdCandidateBuilds;
  bool hit = false;
  bool indeterminate = false;
  double stepBound = 1.0;

  for (const auto& snapshot : movingRigidSurfaceSnapshots) {
    if (snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        current,
        candidate,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        makeSurfaceContactCandidateOptions(
            contactScratch.resolvedSurfaceCandidateCapacity,
            {},
            contactScratch.surfaceCandidateGrowthAllowed),
        makeSurfaceContactCcdOptions(),
        contactScratch);

    stats.movingRigidSurfaceCcdPointTriangleCandidates
        += result.pointTriangleCandidateCount;
    stats.movingRigidSurfaceCcdEdgeEdgeCandidates
        += result.edgeEdgeCandidateCount;
    recordSurfaceCandidateCount(
        stats,
        result.pointTriangleCandidateCount,
        result.edgeEdgeCandidateCount,
        result.capacityOverflowCount);
    accumulateCandidateFilterPressure(
        stats.movingRigidSurfaceCcdCandidatePairCapacity,
        stats.movingRigidSurfaceCcdCandidateRejectedPairs,
        result.pointTrianglePairCapacity,
        result.edgeEdgePairCapacity,
        result.pointTriangleCandidateCount,
        result.edgeEdgeCandidateCount);
    stats.movingRigidSurfaceCcdPointTriangleChecks
        += result.stats.pointTriangleChecks;
    stats.movingRigidSurfaceCcdEdgeEdgeChecks += result.stats.edgeEdgeChecks;
    stats.movingRigidSurfaceCcdHits += result.stats.hits;
    stats.movingRigidSurfaceCcdMisses += result.stats.misses;
    stats.movingRigidSurfaceCcdIndeterminateCount += result.stats.indeterminate;
    stats.movingRigidSurfaceCcdZeroStepCount += result.stats.zeroStepCount;

    indeterminate = indeterminate || result.indeterminate;
    if (result.indeterminate) {
      stepBound = 0.0;
    }
    if (result.hit && (!hit || result.stepBound < stepBound)) {
      hit = true;
      stepBound = result.stepBound;
    }
  }

  if (indeterminate) {
    return false;
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    ++stats.movingRigidSurfaceCcdZeroStepCount;
    return false;
  }

  step *= safeFraction;
  ++stats.movingRigidSurfaceCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
bool applyStaticGroundBarrierCcdLimit(
    std::span<const Eigen::Vector3d> current,
    std::span<const Eigen::Vector3d> direction,
    std::span<const Eigen::Vector3d> gradient,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers,
    DeformableSolverStats& stats,
    double& step,
    comps::DeformableSolverScratch::Vector3Vector& candidate,
    double& directionalDerivative)
{
  if (barriers.empty()) {
    return true;
  }

  bool hit = false;
  double stepBound = 1.0;
  for (std::size_t node = 0; node < current.size(); ++node) {
    if (fixed[node] != 0u) {
      continue;
    }

    ++stats.staticGroundBarrierCcdNodeChecks;
    const auto nodeStepBound = staticGroundBarrierStepBound(
        current[node], candidate[node], barriers, stats);
    if (!nodeStepBound.has_value()) {
      continue;
    }

    ++stats.staticGroundBarrierCcdHits;
    if (*nodeStepBound <= 0.0) {
      ++stats.staticGroundBarrierCcdZeroStepCount;
      return false;
    }
    hit = true;
    stepBound = std::min(stepBound, *nodeStepBound);
  }

  if (!hit) {
    return true;
  }

  const double safeFraction = nb::makeInteriorLineSearchStepScale(stepBound);
  if (safeFraction <= 0.0) {
    ++stats.staticGroundBarrierCcdZeroStepCount;
    return false;
  }

  step *= safeFraction;
  ++stats.staticGroundBarrierCcdLimitedSteps;
  directionalDerivative = buildLineSearchCandidate(
      current, direction, gradient, fixed, step, candidate);
  return true;
}

//==============================================================================
bool isBoundaryActiveAtStepStart(double time, double start, double end)
{
  // Contact-free scene controls intentionally use step-start sampling. Later
  // force-work slices can add partial-step integration when needed.
  return time >= start && time < end;
}

//==============================================================================
double elapsedBoundaryTime(
    double time, double timeStep, double start, double end)
{
  const double nextTime = std::min(time + timeStep, end);
  return std::max(0.0, nextTime - start);
}

//==============================================================================
Eigen::Vector3d boundaryVelocity(
    const comps::DeformableDirichletBoundary& boundary,
    const Eigen::Vector3d& referencePosition)
{
  return boundary.linearVelocity
         + boundary.angularVelocity.cross(referencePosition - boundary.center);
}

//==============================================================================
void reserveDeformableSolverScratch(
    const comps::DeformableNodeState& state,
    comps::DeformableSolverScratch& scratch)
{
  const auto nodeCount = state.positions.size();
  scratch.inertialTargets.reserve(nodeCount);
  scratch.next.reserve(nodeCount);
  scratch.gradient.reserve(nodeCount);
  scratch.direction.reserve(nodeCount);
  scratch.candidate.reserve(nodeCount);
  scratch.previousStepPositions.reserve(nodeCount);
  scratch.previousAttachmentTargets.reserve(nodeCount);
  scratch.stepStartPositions.reserve(nodeCount);
  scratch.stepStartVelocities.reserve(nodeCount);
  scratch.stepAttachmentTargets.reserve(nodeCount);
  scratch.externalAccelerations.reserve(nodeCount);
  scratch.activeFixed.reserve(nodeCount);
  scratch.activeDirichlet.reserve(nodeCount);
  scratch.countedDirichlet.reserve(nodeCount);
  scratch.countedNeumann.reserve(nodeCount);
}

//==============================================================================
void reserveSurfaceContactCandidateScratch(
    std::size_t nodeCount, DeformableContactSolverScratch& scratch)
{
  const std::size_t triangleCount = scratch.surfaceTriangles.size();
  const std::size_t edgeCapacity = checkedSurfaceCandidateMultiply(
      3u, triangleCount, "unique surface edges");

  // Only topology-fixed arrays have a useful linear upper bound. Candidate
  // counts are pose-dependent and can be quadratic; multiplying topology by
  // a broad heuristic here consumed hundreds of megabytes for separated
  // paper-scale surfaces. The prepare-time candidate build below establishes
  // capacity from observed work through the World allocator instead.
  scratch.candidates.surfaceEdges.reserve(edgeCapacity);
  scratch.barrierCandidates.surfaceEdges.reserve(edgeCapacity);
  scratch.sweepScratch.pointItems.reserve(nodeCount);
  scratch.sweepScratch.triangleItems.reserve(triangleCount);
  scratch.sweepScratch.edgeItems.reserve(edgeCapacity);
  scratch.sweepScratch.sweepLinks.reserve(
      std::max(nodeCount, std::max(triangleCount, edgeCapacity)));
}

//==============================================================================
void reserveVbdSelfContactCandidateScratch(
    std::size_t nodeCount,
    std::size_t triangleCount,
    std::size_t candidateCapacity,
    DeformableVbdScratch& scratch)
{
  const std::size_t edgeCapacity = checkedSurfaceCandidateMultiply(
      3u, triangleCount, "VBD unique surface edges");

  scratch.selfContactCandidates.surfaceEdges.reserve(edgeCapacity);
  scratch.selfContactCandidates.pointTriangleCandidates.reserve(
      candidateCapacity);
  scratch.selfContactCandidates.edgeEdgeCandidates.reserve(candidateCapacity);
  scratch.selfContactSweepScratch.pointItems.reserve(nodeCount);
  scratch.selfContactSweepScratch.triangleItems.reserve(triangleCount);
  scratch.selfContactSweepScratch.edgeItems.reserve(edgeCapacity);
  scratch.selfContactSweepScratch.sweepLinks.reserve(
      std::max(nodeCount, std::max(triangleCount, edgeCapacity)));
  scratch.selfContactAdjacency.reserve(nodeCount, candidateCapacity);
}

//==============================================================================
void reserveVbdChebyshevScratch(
    std::size_t nodeCount, DeformableVbdScratch& scratch)
{
  scratch.chebyshevTwoStepsBack.reserve(nodeCount);
  scratch.chebyshevBeforeSweep.reserve(nodeCount);
}

//==============================================================================
void reserveDeformableFrictionScratch(
    std::size_t nodeCount, DeformableContactSolverScratch& scratch)
{
  scratch.groundFrictionNormalForce.reserve(nodeCount);
  scratch.groundFrictionNormalDirection.reserve(nodeCount);
  scratch.selfContactFrictionContacts.reserve(
      scratch.selfSurfaceCandidateCapacity);
}

//==============================================================================
// hasStaticObstacleBarriers must mirror the step-time gate on the static
// ground/sphere/box/capsule barrier and lagged ground-friction terms, so the
// baked sparse pattern reserves a node-local block exactly when the step can
// assemble one.
void reserveProjectedNewtonScratch(
    std::size_t nodeCount,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    const comps::DeformableMaterial* material,
    bool hasStaticObstacleBarriers,
    DeformableContactSolverScratch& scratch)
{
  const std::size_t dimension = checkedSurfaceCandidateMultiply(
      3u, nodeCount, "projected-Newton degrees of freedom");
  DART_SIMULATION_THROW_T_IF(
      dimension
          > static_cast<std::size_t>(std::numeric_limits<Eigen::Index>::max()),
      InvalidArgumentException,
      "Projected-Newton dimension exceeds Eigen::Index");
  const auto dim = static_cast<Eigen::Index>(dimension);
  const bool useMatrixFree
      = material != nullptr && material->useMatrixFreeLinearSolver;
  const bool useSparseIterative
      = material != nullptr
        && usesProjectedNewtonSparseIterativePath(dim, *material);
  const bool useDenseDirect = !useMatrixFree && !useSparseIterative && dim > 0
                              && dim <= kProjectedNewtonDenseDirectDofCap;
  const std::size_t femTetCount
      = material != nullptr && material->useFiniteElementElasticity
            ? topology.tetrahedra.size()
            : 0u;
  scratch.projectedNewtonRhs.resize(dim);
  scratch.projectedNewtonSolution.resize(dim);
  if (useDenseDirect) {
    scratch.projectedNewtonDenseHessian.resize(dim, dim);
    scratch.projectedNewtonDenseHessian.setIdentity();
    scratch.projectedNewtonDenseLdlt.compute(
        scratch.projectedNewtonDenseHessian);
    scratch.projectedNewtonDenseHessian.setZero();
  } else {
    // Above the direct-solve cap, retaining a dense 3N-by-3N matrix would turn
    // the otherwise sparse/iterative path into quadratic memory (7.2 GB for
    // 10k nodes). Keep no dense storage when that path cannot execute.
    scratch.projectedNewtonDenseHessian.resize(0, 0);
  }

  const std::size_t barrierCandidateCount
      = scratch.selfSurfaceCandidateCapacity;
  std::size_t tripletEstimate = checkedSurfaceCandidateMultiply(
      39u, nodeCount, "projected-Newton node triplets");
  tripletEstimate = checkedSurfaceCandidateAdd(
      tripletEstimate,
      checkedSurfaceCandidateMultiply(
          36u, model.edges.size(), "projected-Newton spring triplets"),
      "projected-Newton triplets");
  tripletEstimate = checkedSurfaceCandidateAdd(
      tripletEstimate,
      checkedSurfaceCandidateMultiply(
          144u, femTetCount, "projected-Newton tet triplets"),
      "projected-Newton triplets");
  tripletEstimate = checkedSurfaceCandidateAdd(
      tripletEstimate,
      checkedSurfaceCandidateMultiply(
          144u, barrierCandidateCount, "projected-Newton barrier triplets"),
      "projected-Newton triplets");
  std::size_t matrixFreeBlockEstimate = checkedSurfaceCandidateMultiply(
      4u, nodeCount, "projected-Newton node blocks");
  matrixFreeBlockEstimate = checkedSurfaceCandidateAdd(
      matrixFreeBlockEstimate,
      checkedSurfaceCandidateMultiply(
          4u, model.edges.size(), "projected-Newton spring blocks"),
      "projected-Newton matrix-free blocks");
  matrixFreeBlockEstimate = checkedSurfaceCandidateAdd(
      matrixFreeBlockEstimate,
      checkedSurfaceCandidateMultiply(
          16u, femTetCount, "projected-Newton tet blocks"),
      "projected-Newton matrix-free blocks");
  matrixFreeBlockEstimate = checkedSurfaceCandidateAdd(
      matrixFreeBlockEstimate,
      checkedSurfaceCandidateMultiply(
          16u, barrierCandidateCount, "projected-Newton barrier blocks"),
      "projected-Newton matrix-free blocks");
  if (!useMatrixFree) {
    scratch.projectedNewtonTriplets.reserve(tripletEstimate);
  }
  if (useSparseIterative) {
    scratch.projectedNewtonHessian.resize(dim, dim);
    scratch.projectedNewtonHessian.reserve(
        static_cast<Eigen::Index>(tripletEstimate));
    scratch.newtonPatternOuter.reserve(static_cast<std::size_t>(dim + 1));
    scratch.newtonPatternInner.reserve(tripletEstimate);
    scratch.projectedNewtonIterativeInverseDiagonal.resize(dim);
  }
  scratch.projectedNewtonEdgeBlocks.reserve(checkedSurfaceCandidateMultiply(
      36u, model.edges.size(), "projected-Newton spring block entries"));
  scratch.projectedNewtonEdgeBlockNodes.reserve(model.edges.size());
  scratch.projectedNewtonTetBlocks.reserve(checkedSurfaceCandidateMultiply(
      144u, femTetCount, "projected-Newton tet block entries"));
  scratch.projectedNewtonTetBlockNodes.reserve(femTetCount);
  scratch.projectedNewtonBarrierBlocks.reserve(checkedSurfaceCandidateMultiply(
      144u, barrierCandidateCount, "projected-Newton barrier block entries"));
  scratch.projectedNewtonBarrierBlockNodes.reserve(barrierCandidateCount);
  if (useMatrixFree) {
    scratch.projectedNewtonMatrixFreeBlocks.reserve(matrixFreeBlockEstimate);
    scratch.projectedNewtonMatrixFreeDiagonalBlocks.reserve(nodeCount);
    scratch.projectedNewtonMatrixFreeDiagonalBlocks.resize(nodeCount);
    scratch.projectedNewtonMatrixFreeInverseDiagonalBlocks.reserve(nodeCount);
    scratch.projectedNewtonMatrixFreeInverseDiagonalBlocks.resize(nodeCount);
  }
  if (useMatrixFree || useSparseIterative) {
    // Both iterative implementations share these retained CG work vectors.
    // The sparse path must size them during prepare just as the matrix-free
    // path does, or its first post-bake solve allocates lazily.
    scratch.projectedNewtonMatrixFreeResidual.resize(dim);
    scratch.projectedNewtonMatrixFreePreconditionedResidual.resize(dim);
    scratch.projectedNewtonMatrixFreeDirection.resize(dim);
    scratch.projectedNewtonMatrixFreeHessianDirection.resize(dim);
  }

  if (!useSparseIterative || nodeCount == 0u) {
    return;
  }

  auto& triplets = scratch.projectedNewtonTriplets;
  triplets.clear();
  const auto addNodeDiagonalPattern = [&](std::size_t node) {
    if (node >= nodeCount) {
      return;
    }
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Index dof = static_cast<Eigen::Index>(3 * node + axis);
      triplets.emplace_back(dof, dof, 1.0);
    }
  };
  const auto addNodeBlockPattern = [&](std::span<const std::size_t> nodes) {
    for (const std::size_t rowNode : nodes) {
      if (rowNode >= nodeCount) {
        continue;
      }
      for (const std::size_t colNode : nodes) {
        if (colNode >= nodeCount) {
          continue;
        }
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            const Eigen::Index row = static_cast<Eigen::Index>(3 * rowNode + r);
            const Eigen::Index col = static_cast<Eigen::Index>(3 * colNode + c);
            triplets.emplace_back(row, col, row == col ? 1.0 : 0.0);
          }
        }
      }
    }
  };

  // Inertia alone is strictly diagonal, so a node reached by no other term
  // contributes exactly its three diagonal entries and the pattern stays free
  // of explicit zeros. Once the scene carries a static ground or
  // sphere/box/capsule obstacle, every free node can instead receive a dense
  // node-local 3x3 block: the barrier curvatures assemble as max(0, B'') n n^T
  // and the lagged ground friction as a tangent-plane block, both with
  // non-zero off-diagonals for any non-axis-aligned normal (and the flat
  // ground barrier still emits its full 3x3 stencil around block(2, 2)).
  // Omitting those off-diagonals would leave them outside the cached pattern,
  // so the allocation-free assembly below would reject every step in which a
  // node touches an obstacle -- contacts already present at bake included --
  // and permanently fall back to steepest descent.
  for (std::size_t node = 0; node < nodeCount; ++node) {
    if (hasStaticObstacleBarriers) {
      const std::array<std::size_t, 1> nodes = {node};
      addNodeBlockPattern(nodes);
    } else {
      addNodeDiagonalPattern(node);
    }
  }
  for (const auto& edge : model.edges) {
    const std::array<std::size_t, 2> nodes = {edge.nodeA, edge.nodeB};
    addNodeBlockPattern(nodes);
  }
  if (femTetCount != 0u) {
    for (const auto& tet : topology.tetrahedra) {
      const std::array<std::size_t, 4> nodes
          = {tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD};
      addNodeBlockPattern(nodes);
    }
  }
  for (const auto& candidate :
       scratch.barrierCandidates.pointTriangleCandidates) {
    if (candidate.triangle >= topology.surfaceTriangles.size()) {
      continue;
    }
    const auto& triangle = topology.surfaceTriangles[candidate.triangle];
    const std::array<std::size_t, 4> nodes
        = {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC};
    addNodeBlockPattern(nodes);
  }
  for (const auto& candidate : scratch.barrierCandidates.edgeEdgeCandidates) {
    if (candidate.edgeA >= scratch.barrierCandidates.surfaceEdges.size()
        || candidate.edgeB >= scratch.barrierCandidates.surfaceEdges.size()) {
      continue;
    }
    const auto& edgeA = scratch.barrierCandidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = scratch.barrierCandidates.surfaceEdges[candidate.edgeB];
    const std::array<std::size_t, 4> nodes
        = {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB};
    addNodeBlockPattern(nodes);
  }

  scratch.projectedNewtonHessian.resize(dim, dim);
  scratch.projectedNewtonHessian.setFromTriplets(
      triplets.begin(), triplets.end());
  scratch.projectedNewtonHessian.makeCompressed();
  cacheProjectedNewtonHessianPattern(scratch);
}

//==============================================================================
void syncFemRestShapeScratch(
    std::size_t nodeCount,
    const comps::DeformableMeshTopology& topology,
    const comps::DeformableMaterial& material,
    DeformableContactSolverScratch& scratch,
    bool forceInputCheck = false)
{
  if (!material.useFiniteElementElasticity || topology.tetrahedra.empty()
      || topology.restPositions.size() != nodeCount) {
    if (forceInputCheck
        && (!scratch.femRestShapes.empty() || !scratch.cachedFemTetNodes.empty()
            || !scratch.cachedFemRestPositions.empty())) {
      scratch.femRestShapes.clear();
      scratch.cachedFemTetNodes.clear();
      scratch.cachedFemRestPositions.clear();
      scratch.hessianPatternValid = false;
    }
    return;
  }

  if (!forceInputCheck
      && scratch.femRestShapes.size() == topology.tetrahedra.size()) {
    return;
  }

  bool connectivityChanged
      = scratch.cachedFemTetNodes.size() != topology.tetrahedra.size();
  if (!connectivityChanged) {
    for (std::size_t i = 0; i < topology.tetrahedra.size(); ++i) {
      const auto& tet = topology.tetrahedra[i];
      if (scratch.cachedFemTetNodes[i]
          != std::array<std::size_t, 4>{
              tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD}) {
        connectivityChanged = true;
        break;
      }
    }
  }

  if (forceInputCheck
      && scratch.femRestShapes.size() == topology.tetrahedra.size()
      && scratch.cachedFemTetNodes.size() == topology.tetrahedra.size()
      && scratch.cachedFemRestPositions.size()
             == topology.restPositions.size()) {
    bool inputsMatch = true;
    for (std::size_t i = 0; i < topology.tetrahedra.size(); ++i) {
      const auto& tet = topology.tetrahedra[i];
      inputsMatch = inputsMatch
                    && scratch.cachedFemTetNodes[i]
                           == std::array<std::size_t, 4>{
                               tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD};
    }
    for (std::size_t i = 0; inputsMatch && i < topology.restPositions.size();
         ++i) {
      inputsMatch = topology.restPositions[i].isApprox(
          scratch.cachedFemRestPositions[i], 0.0);
    }
    if (inputsMatch) {
      return;
    }
  }

  scratch.femRestShapes.clear();
  scratch.cachedFemTetNodes.clear();
  scratch.femRestShapes.reserve(topology.tetrahedra.size());
  scratch.cachedFemTetNodes.reserve(topology.tetrahedra.size());
  for (const auto& tet : topology.tetrahedra) {
    scratch.cachedFemTetNodes.push_back(
        {tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD});
    scratch.femRestShapes.push_back(
        fem::makeTetRestShape(
            topology.restPositions[tet.nodeA],
            topology.restPositions[tet.nodeB],
            topology.restPositions[tet.nodeC],
            topology.restPositions[tet.nodeD]));
  }
  scratch.cachedFemRestPositions.assign(
      topology.restPositions.begin(), topology.restPositions.end());
  if (connectivityChanged) {
    scratch.hessianPatternValid = false;
  }
}

//==============================================================================
void prepareDeformableBoundaryConditions(
    const comps::DeformableNodeState& state,
    const comps::DeformableNodeModel& nodeModel,
    const comps::DeformableBoundaryConditions* boundaryConditions,
    double time,
    double timeStep,
    comps::DeformableSolverScratch& scratch,
    DeformableSolverStats& stats)
{
  const auto nodeCount = state.positions.size();
  reserveDeformableSolverScratch(state, scratch);
  scratch.previousStepPositions.assign(
      state.positions.begin(), state.positions.end());
  scratch.previousAttachmentTargets.assign(
      state.attachmentTargets.begin(), state.attachmentTargets.end());
  scratch.stepStartPositions.assign(
      state.positions.begin(), state.positions.end());
  scratch.stepStartVelocities.assign(
      state.velocities.begin(), state.velocities.end());
  scratch.stepAttachmentTargets.assign(
      state.attachmentTargets.begin(), state.attachmentTargets.end());
  scratch.externalAccelerations.assign(nodeCount, Eigen::Vector3d::Zero());
  scratch.activeFixed.assign(nodeModel.fixed.begin(), nodeModel.fixed.end());
  scratch.activeDirichlet.assign(nodeCount, 0u);

  if (boundaryConditions == nullptr) {
    return;
  }

  scratch.countedDirichlet.assign(nodeCount, 0u);
  for (const auto& boundary : boundaryConditions->dirichlet) {
    if (!isBoundaryActiveAtStepStart(
            time, boundary.startTime, boundary.endTime)) {
      continue;
    }

    DART_SIMULATION_THROW_T_IF(
        boundary.nodes.size() != boundary.referencePositions.size(),
        InvalidArgumentException,
        "Serialized deformable Dirichlet boundary has mismatched node and "
        "reference-position counts");
    const double elapsed = elapsedBoundaryTime(
        time, timeStep, boundary.startTime, boundary.endTime);
    for (std::size_t i = 0; i < boundary.nodes.size(); ++i) {
      const auto node = boundary.nodes[i];
      DART_SIMULATION_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "Serialized deformable Dirichlet boundary references out-of-range "
          "node {}",
          node);

      const auto velocity
          = boundaryVelocity(boundary, boundary.referencePositions[i]);
      scratch.stepStartPositions[node]
          = boundary.referencePositions[i] + elapsed * velocity;
      scratch.stepStartVelocities[node] = velocity;
      scratch.stepAttachmentTargets[node] = scratch.stepStartPositions[node];
      scratch.activeFixed[node] = 1u;
      scratch.activeDirichlet[node] = 1u;
      if (scratch.countedDirichlet[node] == 0u) {
        scratch.countedDirichlet[node] = 1u;
        ++stats.activeDirichletNodeCount;
      }
    }
  }

  scratch.countedNeumann.assign(nodeCount, 0u);
  for (const auto& boundary : boundaryConditions->neumann) {
    if (!isBoundaryActiveAtStepStart(
            time, boundary.startTime, boundary.endTime)) {
      continue;
    }

    for (const auto node : boundary.nodes) {
      DART_SIMULATION_THROW_T_IF(
          node >= nodeCount,
          InvalidArgumentException,
          "Serialized deformable Neumann boundary references out-of-range "
          "node {}",
          node);
      if (scratch.activeFixed[node] != 0u) {
        continue;
      }

      scratch.externalAccelerations[node] += boundary.acceleration;
      if (scratch.countedNeumann[node] == 0u) {
        scratch.countedNeumann[node] = 1u;
        ++stats.activeNeumannNodeCount;
      }
    }
  }
}

inline constexpr std::uint32_t kAvbdSelfContactPointTriangleRow = 0;
inline constexpr std::uint32_t kAvbdSelfContactEdgeEdgeRow = 1;

//==============================================================================
void syncVbdTopologyScratch(
    std::size_t nodeCount,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    bool useFiniteElementElasticity,
    DeformableVbdScratch& vbdScratch,
    bool forceInputCheck = false)
{
  const bool countsMatch
      = vbdScratch.initialized && vbdScratch.cachedNodeCount == nodeCount
        && vbdScratch.cachedEdgeCount == model.edges.size()
        && vbdScratch.cachedTetCount == topology.tetrahedra.size()
        && vbdScratch.cachedUseFiniteElementElasticity
               == useFiniteElementElasticity;
  if (countsMatch && !forceInputCheck) {
    return;
  }

  if (countsMatch && forceInputCheck) {
    bool springsMatch = vbdScratch.springs.size() == model.edges.size();
    for (std::size_t i = 0; springsMatch && i < model.edges.size(); ++i) {
      const auto& edge = model.edges[i];
      const auto& cached = vbdScratch.springs[i];
      springsMatch = cached.a == static_cast<std::uint32_t>(edge.nodeA)
                     && cached.b == static_cast<std::uint32_t>(edge.nodeB)
                     && cached.restLength == edge.restLength;
    }

    bool restPositionsMatch = vbdScratch.cachedRestPositions.size()
                              == topology.restPositions.size();
    for (std::size_t i = 0;
         restPositionsMatch && i < topology.restPositions.size();
         ++i) {
      restPositionsMatch = topology.restPositions[i].isApprox(
          vbdScratch.cachedRestPositions[i], 0.0);
    }

    bool surfaceTrianglesMatch = vbdScratch.cachedSurfaceTriangles.size()
                                 == topology.surfaceTriangles.size();
    for (std::size_t i = 0;
         surfaceTrianglesMatch && i < topology.surfaceTriangles.size();
         ++i) {
      const auto& triangle = topology.surfaceTriangles[i];
      const auto& cached = vbdScratch.cachedSurfaceTriangles[i];
      surfaceTrianglesMatch = cached.nodeA == triangle.nodeA
                              && cached.nodeB == triangle.nodeB
                              && cached.nodeC == triangle.nodeC;
    }

    bool tetsMatch = true;
    if (useFiniteElementElasticity) {
      tetsMatch = vbdScratch.tets.size() == topology.tetrahedra.size();
      for (std::size_t i = 0; tetsMatch && i < topology.tetrahedra.size();
           ++i) {
        const auto& tet = topology.tetrahedra[i];
        tetsMatch = vbdScratch.tets[i].vertices
                    == std::array<std::uint32_t, 4>{
                        static_cast<std::uint32_t>(tet.nodeA),
                        static_cast<std::uint32_t>(tet.nodeB),
                        static_cast<std::uint32_t>(tet.nodeC),
                        static_cast<std::uint32_t>(tet.nodeD)};
      }
    }
    if (springsMatch && restPositionsMatch && surfaceTrianglesMatch
        && tetsMatch) {
      return;
    }

    if (!surfaceTrianglesMatch) {
      clearDeformableAvbdSelfContactWarmStart(vbdScratch);
    }
    if (!restPositionsMatch) {
      clearDeformableAvbdAttachmentWarmStart(vbdScratch);
    }
  }

  if (vbdScratch.initialized && !countsMatch) {
    clearDeformableAvbdWarmStartRows(vbdScratch);
    clearDeformableAvbdSelfContactWarmStart(vbdScratch);
  } else if (vbdScratch.initialized) {
    clearDeformableAvbdMaterialWarmStartRows(vbdScratch);
  }
  vbdScratch.springs.clear();
  vbdScratch.springs.reserve(model.edges.size());
  for (const auto& edge : model.edges) {
    vbdScratch.springs.push_back(
        {static_cast<std::uint32_t>(edge.nodeA),
         static_cast<std::uint32_t>(edge.nodeB),
         edge.restLength});
  }

  vbdScratch.tets.clear();
  if (useFiniteElementElasticity) {
    vbdScratch.tets.reserve(topology.tetrahedra.size());
    for (const auto& tet : topology.tetrahedra) {
      const std::array<std::uint32_t, 4> vertices
          = {static_cast<std::uint32_t>(tet.nodeA),
             static_cast<std::uint32_t>(tet.nodeB),
             static_cast<std::uint32_t>(tet.nodeC),
             static_cast<std::uint32_t>(tet.nodeD)};
      const dvbd::TetRestShape rest = dvbd::makeTetRestShape(
          {topology.restPositions[tet.nodeA],
           topology.restPositions[tet.nodeB],
           topology.restPositions[tet.nodeC],
           topology.restPositions[tet.nodeD]});
      vbdScratch.tets.push_back({vertices, rest});
    }
  }
  vbdScratch.cachedRestPositions.assign(
      topology.restPositions.begin(), topology.restPositions.end());
  vbdScratch.cachedSurfaceTriangles.assign(
      topology.surfaceTriangles.begin(), topology.surfaceTriangles.end());

  auto& allocator = *vbdScratch.memoryAllocator;
  vbdScratch.coloring = dvbd::colorDeformable(
      nodeCount, vbdScratch.springs, vbdScratch.tets, allocator);
  vbdScratch.springAdjacency
      = dvbd::SpringAdjacency::build(nodeCount, vbdScratch.springs, allocator);
  vbdScratch.tetAdjacency
      = dvbd::TetAdjacency::build(nodeCount, vbdScratch.tets, allocator);
  vbdScratch.cachedNodeCount = nodeCount;
  vbdScratch.cachedEdgeCount = model.edges.size();
  vbdScratch.cachedTetCount = topology.tetrahedra.size();
  vbdScratch.cachedUseFiniteElementElasticity = useFiniteElementElasticity;
  vbdScratch.initialized = true;
}

//==============================================================================
void primeVbdStaticContactScratch(
    std::size_t nodeCount,
    std::span<const StaticGroundBarrier> barriers,
    std::span<const SphereObstacleBarrier> sphereObstacles,
    std::span<const BoxObstacleBarrier> boxObstacles,
    const comps::DeformableVbdConfig& config,
    DeformableVbdScratch& vbdScratch)
{
  const bool anyStaticContact
      = !barriers.empty() || !sphereObstacles.empty() || !boxObstacles.empty();
  if (config.contactStiffness <= 0.0 || !anyStaticContact) {
    return;
  }

  vbdScratch.contactPlanes.assign(nodeCount, dvbd::ContactPlane{});
  vbdScratch.contactObjectIds.assign(nodeCount, 0);
  vbdScratch.contactFeatureIds.assign(nodeCount, 0);
}

//==============================================================================
/// Solve one implicit-Euler step for a deformable body with the
/// Vertex Block Descent inner solver: graph-colored Gauss-Seidel block
/// coordinate descent on the same variational objective the default solver
/// minimizes, warm-started at the inertial target. Handles both distance
/// springs and volumetric tetrahedra, with the World path routing tet
/// elasticity through the shared FEM kernels so the body's material choice is
/// honored. Fills `scratch.next`; the caller's write-back updates
/// positions/velocities. With `config.contactStiffness > 0` it also resolves
/// static ground/obstacle half-space contact (penalty + optional Coulomb
/// friction) plus lagged surface self-contact normal/friction penalties. Moving
/// rigid-surface CCD and static capsule obstacle barriers still fall back to
/// the default solver.
void runVbdDeformableSolve(
    entt::entity entity,
    std::span<const Eigen::Vector3d> stepPositions,
    std::span<const Eigen::Vector3d> attachmentTargets,
    const comps::DeformableNodeModel& nodeModel,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    comps::DeformableSolverScratch& scratch,
    DeformableVbdScratch& vbdScratch,
    double timeStep,
    double youngsModulus,
    double poissonRatio,
    bool useFiniteElementElasticity,
    bool useFixedCorotationalTets,
    std::span<const StaticGroundBarrier> barriers,
    std::span<const SphereObstacleBarrier> sphereObstacles,
    std::span<const BoxObstacleBarrier> boxObstacles,
    std::span<const DeformableSurfaceTriangle> surfaceTriangles,
    std::span<const std::uint8_t> surfaceContactPointMask,
    std::size_t surfaceCandidateCapacity,
    bool surfaceCandidateGrowthAllowed,
    double frictionCoeff,
    const comps::DeformableVbdConfig& config,
    ComputeExecutor& executor,
    DeformableSolverStats& stats)
{
  const std::size_t nodeCount = scratch.next.size();

  syncVbdTopologyScratch(
      nodeCount, model, topology, useFiniteElementElasticity, vbdScratch);

  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (scratch.activeFixed[i] == 0u) {
      scratch.next[i] = scratch.inertialTargets[i];
    }
  }

  // Build the per-vertex static-contact planes from the barrier + obstacle set
  // at the warm-start position (lagged for the step). Each free vertex gets the
  // single most-binding (smallest-gap) linearized half-space among the z-up
  // ground barriers and the static sphere/box obstacles it is near: a z-up
  // plane at the ground top, or the tangent plane at the closest sphere/box
  // surface point along the outward surface normal. The half-space penalty (and
  // Coulomb friction) act only on penetration, so an armed-but-separated plane
  // is a no-op; vertices off every constraint get a zero-stiffness (inactive)
  // plane and fall freely. One plane per vertex keeps the driver contract
  // unchanged; a vertex pressed into ground and an obstacle at once resolves to
  // the nearer constraint and recovers over steps.
  std::span<const dvbd::ContactPlane> contactPlanes;
  bool hasActiveContactPlanes = false;
  const bool anyStaticContact
      = !barriers.empty() || !sphereObstacles.empty() || !boxObstacles.empty();
  if (config.contactStiffness > 0.0 && anyStaticContact) {
    const double band = staticGroundBarrierActivationDistance();
    vbdScratch.contactPlanes.assign(nodeCount, dvbd::ContactPlane{});
    vbdScratch.contactObjectIds.assign(nodeCount, 0);
    vbdScratch.contactFeatureIds.assign(nodeCount, 0);
    for (std::size_t i = 0; i < nodeCount; ++i) {
      dvbd::ContactPlane& plane = vbdScratch.contactPlanes[i];
      plane.normal = Eigen::Vector3d::UnitZ();
      plane.offset = 0.0;
      plane.stiffness = 0.0;
      // Model-fixed nodes remain exact pins unless the supported mass-spring
      // AVBD route promotes them to compliant attachment rows. Include those
      // potential attachment nodes in contact collection now. If another
      // feature later forces the legacy VBD route, its fixed mask still makes
      // these extra planes inert.
      const bool mayUseAttachmentRow
          = config.useAvbdAttachmentRows && vbdScratch.tets.empty();
      if (scratch.activeFixed[i] != 0u && !mayUseAttachmentRow) {
        continue;
      }
      const Eigen::Vector3d& position = scratch.next[i];
      double bestGap = std::numeric_limits<double>::infinity();
      Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();
      double bestOffset = 0.0;
      std::uint64_t bestObjectId = 0;
      std::uint64_t bestFeatureId = 0;
      bool found = false;

      const auto groundContact = staticGroundContactAt(position, barriers);
      if (groundContact.has_value()) {
        const double gap = position.z() - groundContact->top;
        if (gap < band && gap < bestGap) {
          bestGap = gap;
          bestNormal = Eigen::Vector3d::UnitZ();
          bestOffset = groundContact->top;
          bestObjectId = groundContact->objectId;
          bestFeatureId = groundContact->featureId;
          found = true;
        }
      }

      for (std::size_t s = 0; s < sphereObstacles.size(); ++s) {
        const SphereObstacleBarrier& sphere = sphereObstacles[s];
        const Eigen::Vector3d offset = position - sphere.center;
        const double centerDistance = offset.norm();
        if (!std::isfinite(centerDistance)) {
          continue;
        }
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        if (centerDistance > 0.0) {
          normal = offset / centerDistance;
        }
        const double surfaceDistance = centerDistance - sphere.radius;
        if (surfaceDistance >= band) {
          continue;
        }
        const double planeOffset = normal.dot(sphere.center) + sphere.radius;
        const double gap = normal.dot(position) - planeOffset;
        if (gap < bestGap) {
          bestGap = gap;
          bestNormal = normal;
          bestOffset = planeOffset;
          bestObjectId = sphere.objectId;
          bestFeatureId = staticObstacleFeatureId(
              sphere.geometryRevision,
              /*featureCode=*/0,
              /*surfaceObstacleRole=*/true);
          found = true;
        }
      }

      for (std::size_t b = 0; b < boxObstacles.size(); ++b) {
        const BoxObstacleBarrier& box = boxObstacles[b];
        Eigen::Vector3d normal;
        const double surfaceDistance
            = boxObstacleSurfaceDistance(position, box, normal);
        if (!std::isfinite(surfaceDistance) || surfaceDistance >= band) {
          continue;
        }
        const Eigen::Vector3d surfacePoint
            = position - surfaceDistance * normal;
        const double planeOffset = normal.dot(surfacePoint);
        const double gap = normal.dot(position) - planeOffset;
        if (gap < bestGap) {
          const Eigen::Vector3d localPosition
              = box.rotation.transpose() * (position - box.center);
          const std::uint64_t featureCode
              = dvbd::avbdBoxContactFeatureCode(localPosition, box.halfExtents);
          bestGap = gap;
          bestNormal = normal;
          bestOffset = planeOffset;
          bestObjectId = box.objectId;
          bestFeatureId = staticObstacleFeatureId(
              box.geometryRevision,
              featureCode,
              /*surfaceObstacleRole=*/true);
          found = true;
        }
      }

      if (found) {
        plane.normal = bestNormal;
        plane.offset = bestOffset;
        plane.stiffness = config.contactStiffness;
        vbdScratch.contactObjectIds[i] = bestObjectId;
        vbdScratch.contactFeatureIds[i] = bestFeatureId;
        hasActiveContactPlanes = true;
      }
    }
    contactPlanes = vbdScratch.contactPlanes;
  }

  dvbd::LameParameters lame{};
  if (!vbdScratch.tets.empty()) {
    // World tetrahedral elasticity is explicitly FEM-opt-in. Fixed
    // corotation consumes the physical Lame pair; the shared stable
    // Neo-Hookean log-barrier consumes its own Smith reparameterization. The
    // detached low-level VBD kernel still exposes the no-log Smith energy, but
    // the World path must not evaluate that material domain for FEM-opt-out
    // bodies whose tetrahedra are intentionally inactive.
    DART_SIMULATION_THROW_T_IF(
        !useFiniteElementElasticity,
        InvalidArgumentException,
        "Internal VBD tetrahedral topology requires finite-element elasticity");
    const fem::LameParameters femLame = fem::tetMaterialParameters(
        youngsModulus, poissonRatio, useFixedCorotationalTets);
    lame = {femLame.mu, femLame.lambda};
  }

  dvbd::BlockDescentOptions options;
  options.iterations = config.iterations;
  options.clampSpringHessian = true;
  options.convergenceDisplacement = config.convergenceDisplacement;
  options.useChebyshev = config.useChebyshev;
  options.chebyshevRho = config.chebyshevRho;
  options.rayleighDamping = config.rayleighDamping;
  // Route tetrahedra through the shared FEM elasticity kernels so a VBD body
  // applies the same hyperelastic material (Stable Neo-Hookean or
  // fixed-corotational) the default solver would, instead of the VBD-local
  // Stable Neo-Hookean copy.
  options.useFemTetKernel = useFiniteElementElasticity;
  options.useFixedCorotationalTets = useFixedCorotationalTets;
  // Build the lagged self-contact candidate set + per-vertex incident lists
  // from the body's swept start-to-warm-start surface motion. The IPC
  // clamped-log point-triangle / edge-edge barrier then enters each free
  // vertex's block during the colored sweeps, so a VBD surface resists folding
  // onto itself. Self-contact uses its own IPC barrier stiffness, independent
  // of the static ground/obstacle penalty stiffness; convex bodies that do not
  // fold produce no candidates and pay nothing. The post-solve self CCD limit
  // below remains the no-tunneling backstop for contacts that cross and land
  // outside the barrier band.
  const dvbd::SelfContactAdjacency* selfContact = nullptr;
  if (surfaceTriangles.size() >= 2) {
    const double dHat = selfContactBarrierActivationDistance();
    // Find candidates across the full barrier activation band (not just the
    // tight CCD min-separation the default solver uses), so VBD's lagged
    // penalty barrier engages and decelerates an approaching surface before it
    // can cross
    // -- VBD has no CCD line-search backstop of its own.
    dc::ContactCandidateOptions candidateOptions;
    candidateOptions.activationDistance = dHat;
    candidateOptions.exactDistanceFilter = true;
    candidateOptions.excludeIncidentPointTriangles = true;
    candidateOptions.excludeAdjacentEdges = true;
    candidateOptions.pointMask = surfaceContactPointMask;
    candidateOptions.candidateCapacity = surfaceCandidateCapacity;
    candidateOptions.allowCapacityGrowth = surfaceCandidateGrowthAllowed;
    dc::buildMotionAwareContactCandidatesSweep(
        stepPositions,
        scratch.next,
        surfaceTriangles,
        candidateOptions,
        vbdScratch.selfContactCandidates,
        vbdScratch.selfContactSweepScratch);
    filterSurfaceContactPointCandidates(
        vbdScratch.selfContactCandidates, surfaceContactPointMask);
    recordSurfaceCandidateCount(
        stats,
        vbdScratch.selfContactCandidates.pointTriangleCandidates.size(),
        vbdScratch.selfContactCandidates.edgeEdgeCandidates.size(),
        vbdScratch.selfContactCandidates.stats.capacityOverflowCount);
    vbdScratch.selfContactAdjacency.rebuild(
        nodeCount,
        vbdScratch.selfContactCandidates,
        surfaceTriangles,
        dHat * dHat,
        selfContactBarrierStiffness());
    if (!vbdScratch.selfContactCandidates.pointTriangleCandidates.empty()
        || !vbdScratch.selfContactCandidates.edgeEdgeCandidates.empty()) {
      selfContact = &vbdScratch.selfContactAdjacency;
    }
  }

  bool hasFixedNodes = false;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (scratch.activeFixed[i] != 0u) {
      hasFixedNodes = true;
      break;
    }
  }
  const bool hasRequestedAvbdContactNormalRows
      = config.useAvbdContactNormalRows && hasActiveContactPlanes;
  const bool hasRequestedAvbdAttachmentRows
      = config.useAvbdAttachmentRows && hasFixedNodes;
  const bool hasRequestedAvbdFiniteStiffnessRows
      = config.useAvbdFiniteStiffnessRows && !vbdScratch.springs.empty();
  DART_SIMULATION_THROW_T_IF(
      config.useAvbdFiniteStiffnessRows && !vbdScratch.tets.empty(),
      InvalidArgumentException,
      "AVBD finite-stiffness rows are unsupported for tetrahedral materials; "
      "no independently rampable Equation-16 row formulation is implemented; "
      "use spring constraints or disable the finite-stiffness row option");
  const bool useAvbdFrictionRows = config.useAvbdContactNormalRows
                                   && hasActiveContactPlanes
                                   && frictionCoeff > 0.0;
  const bool useAvbdSelfContactRows
      = config.useAvbdSelfContactNormalRows && selfContact != nullptr;
  const bool hasRequestedAvbdMassSpringRows
      = hasRequestedAvbdContactNormalRows || useAvbdSelfContactRows
        || hasRequestedAvbdAttachmentRows
        || hasRequestedAvbdFiniteStiffnessRows;
  const bool useAvbdSelfContactFrictionRows
      = useAvbdSelfContactRows && frictionCoeff > 0.0;
  const bool hasUnsupportedAvbdFrictionSource
      = frictionCoeff > 0.0
        && ((hasActiveContactPlanes && !useAvbdFrictionRows)
            || (selfContact != nullptr && !useAvbdSelfContactFrictionRows));
  const bool canUseAvbdMassSpringRows
      = hasRequestedAvbdMassSpringRows
        && vbdScratch.tets.empty()
        // A combined AVBD solve must represent every active contact family.
        // Otherwise attachment/finite-stiffness opt-ins would silently drop
        // the already-enabled legacy VBD static-contact penalty.
        && (!hasActiveContactPlanes || config.useAvbdContactNormalRows)
        && !hasUnsupportedAvbdFrictionSource
        && (selfContact == nullptr || useAvbdSelfContactRows)
        && !options.useChebyshev && options.rayleighDamping <= 0.0;
  const bool canUseAvbdTetSelfContactRows
      = useAvbdSelfContactRows && !hasRequestedAvbdContactNormalRows
        && !hasRequestedAvbdAttachmentRows && contactPlanes.empty()
        && vbdScratch.springs.empty() && !vbdScratch.tets.empty()
        && !hasUnsupportedAvbdFrictionSource && !options.useChebyshev
        && options.rayleighDamping <= 0.0;

  dvbd::BlockDescentStats result;
  const auto projectAvbdSelfContactFrictionWarmStarts =
      [](dvbd::AvbdScalarRowInventory& inventory,
         dvbd::AvbdSelfContactTangentAnchorInventory& anchors,
         auto& rows,
         const auto& previousRows) {
        for (std::size_t i = 0; i + 1 < inventory.size() && i + 1 < rows.size();
             i += 2) {
          const std::size_t pair = i / 2u;
          dvbd::AvbdScalarRowRecord& firstRecord = inventory[i];
          dvbd::AvbdScalarRowRecord& secondRecord = inventory[i + 1];
          dvbd::AvbdSelfContactFrictionRow& firstRow = rows[i];
          dvbd::AvbdSelfContactFrictionRow& secondRow = rows[i + 1];
          auto expectedSecondKey = firstRecord.descriptor.key;
          expectedSecondKey.axis = 1;
          if (pair >= anchors.size()
              || firstRecord.descriptor.key.role
                     != dvbd::AvbdScalarRowRole::FrictionTangent
              || firstRecord.descriptor.kind
                     != dvbd::AvbdScalarRowKind::HardConstraint
              || secondRecord.descriptor.kind
                     != dvbd::AvbdScalarRowKind::HardConstraint
              || firstRecord.descriptor.key.axis != 0
              || secondRecord.descriptor.key != expectedSecondKey
              || firstRow.axis != 0 || secondRow.axis != 1
              || !dvbd::avbdSelfContactSameFrictionPrimitive(
                  firstRow, secondRow)
              || firstRow.normalRow != secondRow.normalRow
              || firstRow.frictionCoefficient != secondRow.frictionCoefficient
              || !sameAvbdVectorArray(
                  firstRow.stepStartPositions, secondRow.stepStartPositions)) {
            firstRow.state.lambda = 0.0;
            secondRow.state.lambda = 0.0;
            firstRow.accumulatedTangentialDisplacement.setZero();
            secondRow.accumulatedTangentialDisplacement.setZero();
            firstRow.sticking = false;
            secondRow.sticking = false;
            firstRecord.state.lambda = 0.0;
            secondRecord.state.lambda = 0.0;
            if (pair < anchors.size()) {
              const auto key = anchors[pair].key;
              anchors[pair] = {};
              anchors[pair].key = key;
            }
            continue;
          }

          const auto* previousFirst = findAvbdFrictionWarmStartRow(
              previousRows, firstRecord.descriptor.key);
          const auto* previousSecond = findAvbdFrictionWarmStartRow(
              previousRows, secondRecord.descriptor.key);
          const dvbd::AvbdSelfContactTangentAnchorState& anchor = anchors[pair];
          const AvbdTangentBasis currentBasis
              = dvbd::avbdSelfContactFrictionBasis(firstRow);
          const bool anchorMatches
              = anchor.valid
                && anchor.key
                       == dvbd::makeAvbdDeformableTangentAnchorKey(
                           firstRecord.descriptor.key)
                && anchor.nodes == firstRow.nodes
                && anchor.isEdgeEdge == firstRow.isEdgeEdge
                && anchor.accumulatedDisplacement.allFinite()
                && finiteAvbdVectorArray(anchor.referencePositions)
                && validAvbdTangentBasis(avbdAnchorBasis(anchor))
                && validAvbdTangentBasis(currentBasis);
          if (!anchorMatches) {
            firstRow.state.lambda = 0.0;
            secondRow.state.lambda = 0.0;
            firstRow.accumulatedTangentialDisplacement.setZero();
            secondRow.accumulatedTangentialDisplacement.setZero();
            firstRow.sticking = false;
            secondRow.sticking = false;
            firstRecord.state.lambda = 0.0;
            secondRecord.state.lambda = 0.0;
            const auto key = anchors[pair].key;
            anchors[pair] = {};
            anchors[pair].key = key;
            continue;
          }

          dvbd::AvbdSelfContactFrictionRow replayFirst;
          dvbd::AvbdSelfContactFrictionRow replaySecond;
          if (previousFirst == nullptr && previousSecond == nullptr) {
            replayFirst.nodes = anchor.nodes;
            replaySecond.nodes = anchor.nodes;
            replayFirst.stepStartPositions = anchor.referencePositions;
            replaySecond.stepStartPositions = anchor.referencePositions;
            replayFirst.isEdgeEdge = anchor.isEdgeEdge;
            replaySecond.isEdgeEdge = anchor.isEdgeEdge;
            replayFirst.axis = 0u;
            replaySecond.axis = 1u;
            previousFirst = &replayFirst;
            previousSecond = &replaySecond;
          } else if (
              previousFirst == nullptr || previousSecond == nullptr
              || previousFirst->axis != 0u || previousSecond->axis != 1u
              || !dvbd::avbdSelfContactSameFrictionPrimitive(
                  *previousFirst, *previousSecond)
              || previousFirst->nodes != anchor.nodes
              || previousFirst->isEdgeEdge != anchor.isEdgeEdge
              || !sameAvbdVectorArray(
                  previousFirst->stepStartPositions, anchor.referencePositions)
              || !sameAvbdVectorArray(
                  previousSecond->stepStartPositions,
                  anchor.referencePositions)) {
            firstRow.state.lambda = 0.0;
            secondRow.state.lambda = 0.0;
            firstRow.accumulatedTangentialDisplacement.setZero();
            secondRow.accumulatedTangentialDisplacement.setZero();
            firstRow.sticking = false;
            secondRow.sticking = false;
            firstRecord.state.lambda = 0.0;
            secondRecord.state.lambda = 0.0;
            const auto key = anchors[pair].key;
            anchors[pair] = {};
            anchors[pair].key = key;
            continue;
          }

          const AvbdTangentBasis previousBasis
              = dvbd::avbdSelfContactFrictionBasis(*previousFirst);
          if (!validAvbdTangentBasis(previousBasis)
              || !previousBasis.isApprox(avbdAnchorBasis(anchor), 1e-12)) {
            firstRow.state.lambda = 0.0;
            secondRow.state.lambda = 0.0;
            firstRow.accumulatedTangentialDisplacement.setZero();
            secondRow.accumulatedTangentialDisplacement.setZero();
            firstRow.sticking = false;
            secondRow.sticking = false;
            firstRecord.state.lambda = 0.0;
            secondRecord.state.lambda = 0.0;
            const auto key = anchors[pair].key;
            anchors[pair] = {};
            anchors[pair].key = key;
            continue;
          }

          const Eigen::Vector2d projected
              = dvbd::projectAvbdSelfContactFrictionDualToTangentPair(
                  firstRow.state.lambda,
                  secondRow.state.lambda,
                  *previousFirst,
                  *previousSecond,
                  firstRow,
                  secondRow);
          firstRow.state.lambda = projected.x();
          secondRow.state.lambda = projected.y();
          const Eigen::Vector3d transported
              = transportAvbdTangentDisplacement(anchor, currentBasis);
          firstRow.accumulatedTangentialDisplacement = transported;
          secondRow.accumulatedTangentialDisplacement = transported;
          firstRow.sticking = anchor.sticking;
          secondRow.sticking = anchor.sticking;
          firstRow.previousConstraintValue
              = dvbd::avbdSelfContactFrictionConstraintValue(
                  firstRow, firstRow.stepStartPositions);
          secondRow.previousConstraintValue
              = dvbd::avbdSelfContactFrictionConstraintValue(
                  secondRow, secondRow.stepStartPositions);
          dvbd::setAvbdSelfContactFrictionTangentPairForceLimit(
              firstRow,
              secondRow,
              dvbd::avbdSelfContactFrictionPairForceLimit(firstRow, secondRow));
        }
      };
  if (canUseAvbdMassSpringRows) {
    const auto bodyId = static_cast<std::uint64_t>(entt::to_integral(entity));

    vbdScratch.avbdSelfContactDescriptors.clear();
    if (useAvbdSelfContactRows) {
      vbdScratch.avbdSelfContactDescriptors.reserve(
          vbdScratch.selfContactCandidates.pointTriangleCandidates.size()
          + vbdScratch.selfContactCandidates.edgeEdgeCandidates.size());
      for (const dc::PointTriangleCandidate& candidate :
           vbdScratch.selfContactCandidates.pointTriangleCandidates) {
        // One indexing contract with SelfContactAdjacency::rebuild: it numbers
        // SelfContactEntry::constraint densely over the candidates whose
        // indices resolve, and those numbers address this row array. Admitting
        // a candidate the adjacency skipped would shift every later row out
        // from under its entries, and would dereference the out-of-range
        // triangle when the row's stencil is assigned below.
        if (!dvbd::isSelfContactPointTriangleCandidateInRange(
                candidate, surfaceTriangles.size())) {
          continue;
        }
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.point);
        descriptor.key.featureB
            = static_cast<std::uint64_t>(candidate.triangle);
        descriptor.key.row = kAvbdSelfContactPointTriangleRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
      for (const dc::EdgeEdgeCandidate& candidate :
           vbdScratch.selfContactCandidates.edgeEdgeCandidates) {
        // Same one indexing contract as the point-triangle loop above.
        if (!dvbd::isSelfContactEdgeEdgeCandidateInRange(
                candidate,
                vbdScratch.selfContactCandidates.surfaceEdges.size())) {
          continue;
        }
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.edgeA);
        descriptor.key.featureB = static_cast<std::uint64_t>(candidate.edgeB);
        descriptor.key.row = kAvbdSelfContactEdgeEdgeRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
    }

    vbdScratch.avbdContactDescriptors.clear();
    if (config.useAvbdContactNormalRows && !contactPlanes.empty()) {
      for (std::size_t i = 0; i < nodeCount; ++i) {
        const dvbd::ContactPlane& plane = contactPlanes[i];
        if (plane.stiffness <= 0.0) {
          continue;
        }
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::ContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = vbdScratch.contactObjectIds[i];
        descriptor.key.featureA = static_cast<std::uint64_t>(i);
        descriptor.key.featureB = vbdScratch.contactFeatureIds[i];
        descriptor.key.row = 0;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = plane.stiffness;
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdContactDescriptors.push_back(descriptor);
      }
    }

    vbdScratch.avbdSolveFixed.assign(
        scratch.activeFixed.begin(), scratch.activeFixed.end());
    vbdScratch.avbdAttachmentDescriptors.clear();
    if (config.useAvbdAttachmentRows) {
      for (std::size_t i = 0; i < nodeCount; ++i) {
        if (scratch.activeFixed[i] == 0u) {
          continue;
        }

        vbdScratch.avbdSolveFixed[i] = 0u;
        scratch.next[i] = scratch.inertialTargets[i];

        for (std::uint8_t axis = 0; axis < 3; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::Attachment;
          descriptor.key.objectA = bodyId;
          descriptor.key.featureA = static_cast<std::uint64_t>(i);
          descriptor.key.row = 0;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.startStiffness = config.avbdAttachmentStiffness;
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdAttachmentDescriptors.push_back(descriptor);
        }
      }
    }

    dvbd::AvbdRowWarmStartOptions warmStartOptions;
    warmStartOptions.alpha = config.avbdAlpha;
    warmStartOptions.gamma = config.avbdGamma;
    warmStartOptions.maxStiffness = config.avbdMaxStiffness;
    vbdScratch.avbdSelfContactInventory.reserve(
        vbdScratch.avbdSelfContactDescriptors.size());
    vbdScratch.avbdSelfContactInventory.syncActiveRows(
        vbdScratch.avbdSelfContactDescriptors, warmStartOptions);
    vbdScratch.avbdContactInventory.reserve(
        vbdScratch.avbdContactDescriptors.size());
    vbdScratch.avbdContactInventory.syncActiveRows(
        vbdScratch.avbdContactDescriptors, warmStartOptions);
    vbdScratch.avbdAttachmentInventory.reserve(
        vbdScratch.avbdAttachmentDescriptors.size());
    vbdScratch.avbdAttachmentInventory.syncActiveRows(
        vbdScratch.avbdAttachmentDescriptors, warmStartOptions);

    const auto assignSelfContactPrimitive
        = [&](const dvbd::AvbdScalarRowDescriptor& descriptor, auto& row) {
            if (descriptor.key.row == kAvbdSelfContactPointTriangleRow) {
              const auto point
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto triangleIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const DeformableSurfaceTriangle& triangle
                  = surfaceTriangles[triangleIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(point),
                     static_cast<std::uint32_t>(triangle.nodeA),
                     static_cast<std::uint32_t>(triangle.nodeB),
                     static_cast<std::uint32_t>(triangle.nodeC)};
              row.isEdgeEdge = false;
            } else {
              const auto edgeAIndex
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto edgeBIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const dc::SurfaceEdge& edgeA
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeAIndex];
              const dc::SurfaceEdge& edgeB
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeBIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(edgeA.nodeA),
                     static_cast<std::uint32_t>(edgeA.nodeB),
                     static_cast<std::uint32_t>(edgeB.nodeA),
                     static_cast<std::uint32_t>(edgeB.nodeB)};
              row.isEdgeEdge = true;
            }
          };

    vbdScratch.avbdSelfContactRows.clear();
    vbdScratch.avbdSelfContactRows.reserve(
        vbdScratch.avbdSelfContactInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactInventory.records()) {
      dvbd::AvbdSelfContactNormalRow row;
      row.state = record.state;
      row.squaredActivationDistance
          = vbdScratch.selfContactAdjacency.squaredActivationDistance;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      row.previousConstraintValue
          = dvbd::avbdSelfContactNormalConstraintValue(row, stepPositions);
      vbdScratch.avbdSelfContactRows.push_back(row);
    }

    rebuildAvbdFrictionWarmStartLookup(
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts,
        vbdScratch.avbdSelfContactFrictionDescriptors,
        vbdScratch.avbdSelfContactFrictionRows);
    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    if (useAvbdSelfContactFrictionRows) {
      vbdScratch.avbdSelfContactFrictionDescriptors.reserve(
          2 * vbdScratch.avbdSelfContactRows.size());
      for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
        const dvbd::AvbdScalarRowRecord& normalRecord
            = vbdScratch.avbdSelfContactInventory[i];
        const dvbd::AvbdSelfContactNormalRow& normalRow
            = vbdScratch.avbdSelfContactRows[i];
        // This bound only projects the incoming warm start. The block driver
        // refreshes the pair from the matching trial/current normal force on
        // every sweep, including a newly active contact's first sweep.
        const double warmStartNormalForce
            = std::max(0.0, normalRow.state.lambda);
        const double forceLimit = frictionCoeff * warmStartNormalForce;
        for (std::uint8_t axis = 0; axis < 2; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::FrictionTangent;
          descriptor.key.objectA = normalRecord.descriptor.key.objectA;
          descriptor.key.objectB = normalRecord.descriptor.key.objectB;
          descriptor.key.featureA = normalRecord.descriptor.key.featureA;
          descriptor.key.featureB = normalRecord.descriptor.key.featureB;
          descriptor.key.row = normalRecord.descriptor.key.row;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.bounds = dvbd::avbdFrictionTangentBounds(forceLimit);
          descriptor.startStiffness = selfContactBarrierStiffness();
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdSelfContactFrictionDescriptors.push_back(descriptor);
        }
      }
    }
    vbdScratch.avbdSelfContactFrictionInventory.reserve(
        vbdScratch.avbdSelfContactFrictionDescriptors.size());
    vbdScratch.avbdSelfContactFrictionInventory.syncActiveRows(
        vbdScratch.avbdSelfContactFrictionDescriptors, warmStartOptions);
    const std::size_t selfContactFrictionPairCount
        = vbdScratch.avbdSelfContactFrictionDescriptors.size() / 2u;
    vbdScratch.avbdSelfContactFrictionAnchors.reserve(
        selfContactFrictionPairCount);
    vbdScratch.avbdSelfContactFrictionAnchors.syncActiveKeysByIndex(
        selfContactFrictionPairCount, [&](std::size_t pair) {
          return dvbd::makeAvbdDeformableTangentAnchorKey(
              vbdScratch.avbdSelfContactFrictionDescriptors[2u * pair].key);
        });

    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdSelfContactFrictionRows.reserve(
        vbdScratch.avbdSelfContactFrictionInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactFrictionInventory.records()) {
      dvbd::AvbdSelfContactFrictionRow row;
      row.state = record.state;
      row.axis = record.descriptor.key.axis;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      for (std::uint8_t i = 0; i < 4; ++i) {
        row.stepStartPositions[i] = stepPositions[row.nodes[i]];
      }
      row.previousConstraintValue
          = dvbd::avbdSelfContactFrictionConstraintValue(row, stepPositions);
      vbdScratch.avbdSelfContactFrictionRows.push_back(row);
    }
    for (std::size_t i = 0;
         i + 1 < vbdScratch.avbdSelfContactFrictionRows.size();
         i += 2) {
      const std::size_t normalRow = i / 2u;
      vbdScratch.avbdSelfContactFrictionRows[i].normalRow = normalRow;
      vbdScratch.avbdSelfContactFrictionRows[i + 1u].normalRow = normalRow;
      vbdScratch.avbdSelfContactFrictionRows[i].frictionCoefficient
          = frictionCoeff;
      vbdScratch.avbdSelfContactFrictionRows[i + 1u].frictionCoefficient
          = frictionCoeff;
    }
    projectAvbdSelfContactFrictionWarmStarts(
        vbdScratch.avbdSelfContactFrictionInventory,
        vbdScratch.avbdSelfContactFrictionAnchors,
        vbdScratch.avbdSelfContactFrictionRows,
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts);

    vbdScratch.avbdContactRows.clear();
    vbdScratch.avbdContactRows.reserve(vbdScratch.avbdContactInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdContactInventory.records()) {
      const auto vertex
          = static_cast<std::uint32_t>(record.descriptor.key.featureA);
      const dvbd::ContactPlane& plane = contactPlanes[vertex];
      vbdScratch.avbdContactRows.push_back(
          dvbd::AvbdHalfSpaceContactRow{
              vertex,
              plane,
              record.state,
              dvbd::avbdHalfSpaceContactConstraintValue(
                  stepPositions[vertex], plane),
              record.descriptor.bounds});
    }

    rebuildAvbdFrictionWarmStartLookup(
        vbdScratch.previousAvbdFrictionWarmStarts,
        vbdScratch.avbdFrictionDescriptors,
        vbdScratch.avbdFrictionRows);
    vbdScratch.avbdFrictionDescriptors.clear();
    if (useAvbdFrictionRows) {
      vbdScratch.avbdFrictionDescriptors.reserve(
          2 * vbdScratch.avbdContactRows.size());
      for (std::size_t i = 0; i < vbdScratch.avbdContactRows.size(); ++i) {
        const dvbd::AvbdHalfSpaceContactRow& contactRow
            = vbdScratch.avbdContactRows[i];
        const std::uint32_t vertex = contactRow.vertex;
        // This bound only projects the incoming warm start. The block driver
        // refreshes the pair from the matching trial/current normal force on
        // every sweep, including a newly active contact's first sweep.
        const double warmStartNormalForce
            = std::max(0.0, contactRow.state.lambda);
        const double forceLimit = frictionCoeff * warmStartNormalForce;
        for (std::uint8_t axis = 0; axis < 2; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::FrictionTangent;
          descriptor.key.objectA = bodyId;
          descriptor.key.objectB = vbdScratch.contactObjectIds[vertex];
          descriptor.key.featureA = static_cast<std::uint64_t>(vertex);
          descriptor.key.featureB = vbdScratch.contactFeatureIds[vertex];
          descriptor.key.row = 0;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.bounds = dvbd::avbdFrictionTangentBounds(forceLimit);
          descriptor.startStiffness = contactRow.plane.stiffness;
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdFrictionDescriptors.push_back(descriptor);
        }
      }
    }
    vbdScratch.avbdFrictionInventory.reserve(
        vbdScratch.avbdFrictionDescriptors.size());
    vbdScratch.avbdFrictionInventory.syncActiveRows(
        vbdScratch.avbdFrictionDescriptors, warmStartOptions);
    const std::size_t frictionPairCount
        = vbdScratch.avbdFrictionDescriptors.size() / 2u;
    vbdScratch.avbdFrictionAnchors.reserve(frictionPairCount);
    vbdScratch.avbdFrictionAnchors.syncActiveKeysByIndex(
        frictionPairCount, [&](std::size_t pair) {
          return dvbd::makeAvbdDeformableTangentAnchorKey(
              vbdScratch.avbdFrictionDescriptors[2u * pair].key);
        });
    for (std::size_t i = 0; i + 1 < vbdScratch.avbdFrictionInventory.size();
         i += 2) {
      const std::size_t pair = i / 2u;
      dvbd::AvbdScalarRowRecord& firstRecord
          = vbdScratch.avbdFrictionInventory[i];
      dvbd::AvbdScalarRowRecord& secondRecord
          = vbdScratch.avbdFrictionInventory[i + 1];
      auto expectedSecondKey = firstRecord.descriptor.key;
      expectedSecondKey.axis = 1;
      if (pair >= vbdScratch.avbdFrictionAnchors.size()
          || firstRecord.descriptor.key.role
                 != dvbd::AvbdScalarRowRole::FrictionTangent
          || firstRecord.descriptor.kind
                 != dvbd::AvbdScalarRowKind::HardConstraint
          || secondRecord.descriptor.kind
                 != dvbd::AvbdScalarRowKind::HardConstraint
          || firstRecord.descriptor.key.axis != 0
          || secondRecord.descriptor.key != expectedSecondKey) {
        firstRecord.state.lambda = 0.0;
        secondRecord.state.lambda = 0.0;
        if (pair < vbdScratch.avbdFrictionAnchors.size()) {
          const auto key = vbdScratch.avbdFrictionAnchors[pair].key;
          vbdScratch.avbdFrictionAnchors[pair] = {};
          vbdScratch.avbdFrictionAnchors[pair].key = key;
        }
        continue;
      }

      const auto vertex
          = static_cast<std::uint32_t>(firstRecord.descriptor.key.featureA);
      const dvbd::ContactPlane& plane = contactPlanes[vertex];
      const dc::Matrix3x2d basis
          = dc::detail::fallbackBasisFromNormal(plane.normal);
      dvbd::AvbdHalfSpaceTangentAnchorState& anchor
          = vbdScratch.avbdFrictionAnchors[pair];
      const bool anchorMatches
          = anchor.valid && anchor.vertex == vertex
            && anchor.key
                   == dvbd::makeAvbdDeformableTangentAnchorKey(
                       firstRecord.descriptor.key)
            && anchor.accumulatedDisplacement.allFinite()
            && validAvbdTangentBasis(avbdAnchorBasis(anchor))
            && validAvbdTangentBasis(basis);
      if (!anchorMatches) {
        firstRecord.state.lambda = 0.0;
        secondRecord.state.lambda = 0.0;
        const auto key = vbdScratch.avbdFrictionAnchors[pair].key;
        vbdScratch.avbdFrictionAnchors[pair] = {};
        vbdScratch.avbdFrictionAnchors[pair].key = key;
        continue;
      }
      const Eigen::Vector2d projected
          = dvbd::projectAvbdFrictionDualToTangentPair(
              firstRecord.state.lambda,
              secondRecord.state.lambda,
              anchor.basis[0],
              anchor.basis[1],
              basis.col(0),
              basis.col(1));
      firstRecord.state.lambda = projected.x();
      secondRecord.state.lambda = projected.y();
      const double forceLimit = std::min(
          -firstRecord.descriptor.bounds.lower,
          firstRecord.descriptor.bounds.upper);
      (void)dvbd::projectAvbdHalfSpaceFrictionWarmStartToLiveCone(
          firstRecord.state, secondRecord.state, anchor, forceLimit);
    }

    vbdScratch.avbdFrictionRows.clear();
    vbdScratch.avbdFrictionRows.reserve(
        vbdScratch.avbdFrictionInventory.size());
    for (std::size_t rowIndex = 0;
         rowIndex < vbdScratch.avbdFrictionInventory.size();
         ++rowIndex) {
      const dvbd::AvbdScalarRowRecord& record
          = vbdScratch.avbdFrictionInventory[rowIndex];
      const auto vertex
          = static_cast<std::uint32_t>(record.descriptor.key.featureA);
      const std::uint8_t axisId = record.descriptor.key.axis;
      const dvbd::ContactPlane& plane = contactPlanes[vertex];
      const dc::Matrix3x2d basis
          = dc::detail::fallbackBasisFromNormal(plane.normal);
      const Eigen::Vector3d axis = basis.col(axisId < 2 ? axisId : 0);
      dvbd::AvbdHalfSpaceFrictionRow row{
          vertex,
          stepPositions[vertex],
          axis,
          record.state,
          0.0,
          record.descriptor.bounds};
      const std::size_t pair = rowIndex / 2u;
      if (pair < vbdScratch.avbdFrictionAnchors.size()) {
        const auto& anchor = vbdScratch.avbdFrictionAnchors[pair];
        if (anchor.valid && anchor.vertex == vertex
            && anchor.key
                   == dvbd::makeAvbdDeformableTangentAnchorKey(
                       record.descriptor.key)
            && validAvbdTangentBasis(avbdAnchorBasis(anchor))
            && validAvbdTangentBasis(basis)) {
          row.accumulatedTangentialDisplacement
              = transportAvbdTangentDisplacement(anchor, basis);
          row.sticking = anchor.sticking;
        }
      }
      row.normalRow = rowIndex / 2u;
      row.frictionCoefficient = frictionCoeff;
      row.previousConstraintValue = dvbd::avbdHalfSpaceFrictionConstraintValue(
          stepPositions[vertex],
          row.stepStartPosition,
          row.axis,
          row.accumulatedTangentialDisplacement);
      vbdScratch.avbdFrictionRows.push_back(row);
    }

    const bool hasDirichletMask = scratch.activeDirichlet.size() == nodeCount;
    vbdScratch.avbdAttachmentRows.clear();
    vbdScratch.avbdAttachmentRows.reserve(
        vbdScratch.avbdAttachmentInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdAttachmentInventory.records()) {
      const auto vertex
          = static_cast<std::uint32_t>(record.descriptor.key.featureA);
      const std::uint8_t axisId = record.descriptor.key.axis;
      const Eigen::Vector3d axis = dvbd::canonicalAvbdAttachmentAxis(axisId);
      const bool isScriptedDirichlet
          = hasDirichletMask && scratch.activeDirichlet[vertex] != 0u;
      const Eigen::Vector3d& target = attachmentTargets[vertex];
      const Eigen::Vector3d& previousTarget
          = isScriptedDirichlet ? scratch.previousAttachmentTargets[vertex]
                                : target;
      vbdScratch.avbdAttachmentRows.push_back(
          dvbd::AvbdPointAttachmentRow{
              vertex,
              target,
              axis,
              record.state,
              dvbd::avbdPointAttachmentConstraintValue(
                  scratch.previousStepPositions[vertex], previousTarget, axis),
              record.descriptor.bounds});
    }

    vbdScratch.avbdSpringDescriptors.clear();
    if (config.useAvbdFiniteStiffnessRows) {
      vbdScratch.avbdSpringDescriptors.reserve(vbdScratch.springs.size());
      for (std::size_t i = 0; i < vbdScratch.springs.size(); ++i) {
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::DeformableSpring;
        descriptor.key.objectA = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(i);
        descriptor.kind = dvbd::AvbdScalarRowKind::FiniteStiffness;
        descriptor.startStiffness = config.avbdFiniteStiffnessStart;
        descriptor.materialStiffness = model.stiffness;
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSpringDescriptors.push_back(descriptor);
      }
    }
    vbdScratch.avbdSpringInventory.reserve(
        vbdScratch.avbdSpringDescriptors.size());
    vbdScratch.avbdSpringInventory.syncActiveRows(
        vbdScratch.avbdSpringDescriptors, warmStartOptions);

    vbdScratch.avbdSpringRows.clear();
    vbdScratch.avbdSpringRows.reserve(vbdScratch.avbdSpringInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSpringInventory.records()) {
      vbdScratch.avbdSpringRows.push_back(
          dvbd::AvbdSpringFiniteStiffnessRow{
              static_cast<std::uint32_t>(record.descriptor.key.featureA),
              record.state,
              dvbd::maxAvbdDescriptorStiffness(
                  record.descriptor, warmStartOptions)});
    }

    dvbd::AvbdHalfSpaceContactOptions contactOptions;
    contactOptions.alpha = config.avbdAlpha;
    contactOptions.beta = config.avbdBeta;
    contactOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdPointAttachmentOptions attachmentOptions;
    attachmentOptions.alpha = config.avbdAlpha;
    attachmentOptions.beta = config.avbdBeta;
    attachmentOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSpringFiniteStiffnessOptions springOptions;
    springOptions.beta = config.avbdBeta;
    springOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdHalfSpaceFrictionOptions frictionOptions;
    frictionOptions.alpha = config.avbdAlpha;
    frictionOptions.beta = config.avbdBeta;
    frictionOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSelfContactNormalOptions selfContactOptions;
    selfContactOptions.alpha = config.avbdAlpha;
    selfContactOptions.beta = config.avbdBeta;
    selfContactOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;
    selfContactFrictionOptions.alpha = config.avbdAlpha;
    selfContactFrictionOptions.beta = config.avbdBeta;
    selfContactFrictionOptions.maxStiffness = config.avbdMaxStiffness;
    result = dvbd::blockDescentMassSpringAvbdRows(
        scratch.next,
        nodeModel.masses,
        vbdScratch.avbdSolveFixed,
        scratch.inertialTargets,
        vbdScratch.springs,
        model.stiffness,
        timeStep,
        vbdScratch.avbdContactRows,
        vbdScratch.avbdAttachmentRows,
        vbdScratch.avbdSpringRows,
        vbdScratch.coloring,
        vbdScratch.springAdjacency,
        options,
        contactOptions,
        attachmentOptions,
        springOptions,
        &vbdScratch.avbdFrictionRows,
        &frictionOptions,
        &vbdScratch.avbdSelfContactRows,
        useAvbdSelfContactRows ? selfContact : nullptr,
        &selfContactOptions,
        useAvbdSelfContactFrictionRows ? &vbdScratch.avbdSelfContactFrictionRows
                                       : nullptr,
        useAvbdSelfContactFrictionRows ? &selfContactFrictionOptions : nullptr,
        &executor);

    for (std::size_t i = 0; i < vbdScratch.avbdContactRows.size(); ++i) {
      vbdScratch.avbdContactInventory[i].state
          = vbdScratch.avbdContactRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdAttachmentRows.size(); ++i) {
      vbdScratch.avbdAttachmentInventory[i].state
          = vbdScratch.avbdAttachmentRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSpringRows.size(); ++i) {
      vbdScratch.avbdSpringInventory[i].state
          = vbdScratch.avbdSpringRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdFrictionRows.size(); ++i) {
      vbdScratch.avbdFrictionInventory[i].state
          = vbdScratch.avbdFrictionRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
      vbdScratch.avbdSelfContactInventory[i].state
          = vbdScratch.avbdSelfContactRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactFrictionRows.size();
         ++i) {
      vbdScratch.avbdSelfContactFrictionInventory[i].state
          = vbdScratch.avbdSelfContactFrictionRows[i].state;
    }
    persistAvbdHalfSpaceFrictionAnchors(
        vbdScratch.avbdFrictionInventory,
        vbdScratch.avbdFrictionAnchors,
        vbdScratch.avbdFrictionRows,
        scratch.next);
    persistAvbdSelfContactFrictionAnchors(
        vbdScratch.avbdSelfContactFrictionInventory,
        vbdScratch.avbdSelfContactFrictionAnchors,
        vbdScratch.avbdSelfContactFrictionRows,
        scratch.next);
    stats.vbdAvbdContactNormalRows += vbdScratch.avbdContactRows.size();
    stats.vbdAvbdSelfContactNormalRows += vbdScratch.avbdSelfContactRows.size();
    stats.vbdAvbdFrictionTangentRows
        += vbdScratch.avbdFrictionRows.size()
           + vbdScratch.avbdSelfContactFrictionRows.size();
    stats.vbdAvbdAttachmentRows += vbdScratch.avbdAttachmentRows.size();
    stats.vbdAvbdFiniteStiffnessRows += vbdScratch.avbdSpringRows.size();
  } else if (canUseAvbdTetSelfContactRows) {
    const auto bodyId = static_cast<std::uint64_t>(entt::to_integral(entity));

    vbdScratch.avbdContactDescriptors.clear();
    vbdScratch.avbdContactRows.clear();
    vbdScratch.avbdContactInventory.records().clear();
    vbdScratch.avbdFrictionDescriptors.clear();
    vbdScratch.avbdFrictionRows.clear();
    vbdScratch.avbdFrictionInventory.records().clear();
    vbdScratch.avbdFrictionAnchors.clear();
    vbdScratch.avbdSelfContactDescriptors.clear();
    vbdScratch.avbdSelfContactRows.clear();
    rebuildAvbdFrictionWarmStartLookup(
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts,
        vbdScratch.avbdSelfContactFrictionDescriptors,
        vbdScratch.avbdSelfContactFrictionRows);
    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdAttachmentDescriptors.clear();
    vbdScratch.avbdAttachmentRows.clear();
    vbdScratch.avbdAttachmentInventory.records().clear();
    vbdScratch.avbdSpringDescriptors.clear();
    vbdScratch.avbdSpringRows.clear();
    vbdScratch.avbdSpringInventory.records().clear();
    vbdScratch.avbdSolveFixed.clear();

    dvbd::AvbdRowWarmStartOptions hardRowWarmStartOptions;
    hardRowWarmStartOptions.alpha = config.avbdAlpha;
    hardRowWarmStartOptions.gamma = config.avbdGamma;
    hardRowWarmStartOptions.maxStiffness = config.avbdMaxStiffness;

    vbdScratch.avbdSelfContactDescriptors.clear();
    if (useAvbdSelfContactRows) {
      vbdScratch.avbdSelfContactDescriptors.reserve(
          vbdScratch.selfContactCandidates.pointTriangleCandidates.size()
          + vbdScratch.selfContactCandidates.edgeEdgeCandidates.size());
      for (const dc::PointTriangleCandidate& candidate :
           vbdScratch.selfContactCandidates.pointTriangleCandidates) {
        // One indexing contract with SelfContactAdjacency::rebuild: it numbers
        // SelfContactEntry::constraint densely over the candidates whose
        // indices resolve, and those numbers address this row array. Admitting
        // a candidate the adjacency skipped would shift every later row out
        // from under its entries, and would dereference the out-of-range
        // triangle when the row's stencil is assigned below.
        if (!dvbd::isSelfContactPointTriangleCandidateInRange(
                candidate, surfaceTriangles.size())) {
          continue;
        }
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.point);
        descriptor.key.featureB
            = static_cast<std::uint64_t>(candidate.triangle);
        descriptor.key.row = kAvbdSelfContactPointTriangleRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
      for (const dc::EdgeEdgeCandidate& candidate :
           vbdScratch.selfContactCandidates.edgeEdgeCandidates) {
        // Same one indexing contract as the point-triangle loop above.
        if (!dvbd::isSelfContactEdgeEdgeCandidateInRange(
                candidate,
                vbdScratch.selfContactCandidates.surfaceEdges.size())) {
          continue;
        }
        dvbd::AvbdScalarRowDescriptor descriptor;
        descriptor.key.role = dvbd::AvbdScalarRowRole::SelfContactNormal;
        descriptor.key.objectA = bodyId;
        descriptor.key.objectB = bodyId;
        descriptor.key.featureA = static_cast<std::uint64_t>(candidate.edgeA);
        descriptor.key.featureB = static_cast<std::uint64_t>(candidate.edgeB);
        descriptor.key.row = kAvbdSelfContactEdgeEdgeRow;
        descriptor.key.axis = 0;
        descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
        descriptor.bounds = dvbd::avbdContactNormalBounds();
        descriptor.startStiffness = selfContactBarrierStiffness();
        descriptor.maxStiffness = config.avbdMaxStiffness;
        vbdScratch.avbdSelfContactDescriptors.push_back(descriptor);
      }
    }
    vbdScratch.avbdSelfContactInventory.reserve(
        vbdScratch.avbdSelfContactDescriptors.size());
    vbdScratch.avbdSelfContactInventory.syncActiveRows(
        vbdScratch.avbdSelfContactDescriptors, hardRowWarmStartOptions);

    const auto assignSelfContactPrimitive
        = [&](const dvbd::AvbdScalarRowDescriptor& descriptor, auto& row) {
            if (descriptor.key.row == kAvbdSelfContactPointTriangleRow) {
              const auto point
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto triangleIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const DeformableSurfaceTriangle& triangle
                  = surfaceTriangles[triangleIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(point),
                     static_cast<std::uint32_t>(triangle.nodeA),
                     static_cast<std::uint32_t>(triangle.nodeB),
                     static_cast<std::uint32_t>(triangle.nodeC)};
              row.isEdgeEdge = false;
            } else {
              const auto edgeAIndex
                  = static_cast<std::size_t>(descriptor.key.featureA);
              const auto edgeBIndex
                  = static_cast<std::size_t>(descriptor.key.featureB);
              const dc::SurfaceEdge& edgeA
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeAIndex];
              const dc::SurfaceEdge& edgeB
                  = vbdScratch.selfContactCandidates.surfaceEdges[edgeBIndex];
              row.nodes
                  = {static_cast<std::uint32_t>(edgeA.nodeA),
                     static_cast<std::uint32_t>(edgeA.nodeB),
                     static_cast<std::uint32_t>(edgeB.nodeA),
                     static_cast<std::uint32_t>(edgeB.nodeB)};
              row.isEdgeEdge = true;
            }
          };

    vbdScratch.avbdSelfContactRows.clear();
    vbdScratch.avbdSelfContactRows.reserve(
        vbdScratch.avbdSelfContactInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactInventory.records()) {
      dvbd::AvbdSelfContactNormalRow row;
      row.state = record.state;
      row.squaredActivationDistance
          = vbdScratch.selfContactAdjacency.squaredActivationDistance;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      row.previousConstraintValue
          = dvbd::avbdSelfContactNormalConstraintValue(row, stepPositions);
      vbdScratch.avbdSelfContactRows.push_back(row);
    }

    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    if (useAvbdSelfContactFrictionRows) {
      vbdScratch.avbdSelfContactFrictionDescriptors.reserve(
          2 * vbdScratch.avbdSelfContactRows.size());
      for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
        const dvbd::AvbdScalarRowRecord& normalRecord
            = vbdScratch.avbdSelfContactInventory[i];
        const dvbd::AvbdSelfContactNormalRow& normalRow
            = vbdScratch.avbdSelfContactRows[i];
        // This bound only projects the incoming warm start. The block driver
        // refreshes the pair from the matching trial/current normal force on
        // every sweep, including a newly active contact's first sweep.
        const double warmStartNormalForce
            = std::max(0.0, normalRow.state.lambda);
        const double forceLimit = frictionCoeff * warmStartNormalForce;
        for (std::uint8_t axis = 0; axis < 2; ++axis) {
          dvbd::AvbdScalarRowDescriptor descriptor;
          descriptor.key.role = dvbd::AvbdScalarRowRole::FrictionTangent;
          descriptor.key.objectA = normalRecord.descriptor.key.objectA;
          descriptor.key.objectB = normalRecord.descriptor.key.objectB;
          descriptor.key.featureA = normalRecord.descriptor.key.featureA;
          descriptor.key.featureB = normalRecord.descriptor.key.featureB;
          descriptor.key.row = normalRecord.descriptor.key.row;
          descriptor.key.axis = axis;
          descriptor.kind = dvbd::AvbdScalarRowKind::HardConstraint;
          descriptor.bounds = dvbd::avbdFrictionTangentBounds(forceLimit);
          descriptor.startStiffness = selfContactBarrierStiffness();
          descriptor.maxStiffness = config.avbdMaxStiffness;
          vbdScratch.avbdSelfContactFrictionDescriptors.push_back(descriptor);
        }
      }
    }
    vbdScratch.avbdSelfContactFrictionInventory.reserve(
        vbdScratch.avbdSelfContactFrictionDescriptors.size());
    vbdScratch.avbdSelfContactFrictionInventory.syncActiveRows(
        vbdScratch.avbdSelfContactFrictionDescriptors, hardRowWarmStartOptions);
    const std::size_t selfContactFrictionPairCount
        = vbdScratch.avbdSelfContactFrictionDescriptors.size() / 2u;
    vbdScratch.avbdSelfContactFrictionAnchors.reserve(
        selfContactFrictionPairCount);
    vbdScratch.avbdSelfContactFrictionAnchors.syncActiveKeysByIndex(
        selfContactFrictionPairCount, [&](std::size_t pair) {
          return dvbd::makeAvbdDeformableTangentAnchorKey(
              vbdScratch.avbdSelfContactFrictionDescriptors[2u * pair].key);
        });

    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdSelfContactFrictionRows.reserve(
        vbdScratch.avbdSelfContactFrictionInventory.size());
    for (const dvbd::AvbdScalarRowRecord& record :
         vbdScratch.avbdSelfContactFrictionInventory.records()) {
      dvbd::AvbdSelfContactFrictionRow row;
      row.state = record.state;
      row.axis = record.descriptor.key.axis;
      row.bounds = record.descriptor.bounds;
      assignSelfContactPrimitive(record.descriptor, row);
      for (std::uint8_t i = 0; i < 4; ++i) {
        row.stepStartPositions[i] = stepPositions[row.nodes[i]];
      }
      row.previousConstraintValue
          = dvbd::avbdSelfContactFrictionConstraintValue(row, stepPositions);
      vbdScratch.avbdSelfContactFrictionRows.push_back(row);
    }
    for (std::size_t i = 0;
         i + 1 < vbdScratch.avbdSelfContactFrictionRows.size();
         i += 2) {
      const std::size_t normalRow = i / 2u;
      vbdScratch.avbdSelfContactFrictionRows[i].normalRow = normalRow;
      vbdScratch.avbdSelfContactFrictionRows[i + 1u].normalRow = normalRow;
      vbdScratch.avbdSelfContactFrictionRows[i].frictionCoefficient
          = frictionCoeff;
      vbdScratch.avbdSelfContactFrictionRows[i + 1u].frictionCoefficient
          = frictionCoeff;
    }
    projectAvbdSelfContactFrictionWarmStarts(
        vbdScratch.avbdSelfContactFrictionInventory,
        vbdScratch.avbdSelfContactFrictionAnchors,
        vbdScratch.avbdSelfContactFrictionRows,
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts);

    dvbd::AvbdSelfContactNormalOptions selfContactOptions;
    selfContactOptions.alpha = config.avbdAlpha;
    selfContactOptions.beta = config.avbdBeta;
    selfContactOptions.maxStiffness = config.avbdMaxStiffness;
    dvbd::AvbdSelfContactFrictionOptions selfContactFrictionOptions;
    selfContactFrictionOptions.alpha = config.avbdAlpha;
    selfContactFrictionOptions.beta = config.avbdBeta;
    selfContactFrictionOptions.maxStiffness = config.avbdMaxStiffness;
    result = dvbd::blockDescentTetMeshAvbdSelfContact(
        scratch.next,
        nodeModel.masses,
        scratch.activeFixed,
        scratch.inertialTargets,
        vbdScratch.tets,
        lame.mu,
        lame.lambda,
        timeStep,
        vbdScratch.coloring,
        vbdScratch.tetAdjacency,
        options,
        selfContact,
        useAvbdSelfContactRows ? &vbdScratch.avbdSelfContactRows : nullptr,
        useAvbdSelfContactRows ? &selfContactOptions : nullptr,
        useAvbdSelfContactFrictionRows ? &vbdScratch.avbdSelfContactFrictionRows
                                       : nullptr,
        useAvbdSelfContactFrictionRows ? &selfContactFrictionOptions : nullptr,
        &executor);

    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactRows.size(); ++i) {
      vbdScratch.avbdSelfContactInventory[i].state
          = vbdScratch.avbdSelfContactRows[i].state;
    }
    for (std::size_t i = 0; i < vbdScratch.avbdSelfContactFrictionRows.size();
         ++i) {
      vbdScratch.avbdSelfContactFrictionInventory[i].state
          = vbdScratch.avbdSelfContactFrictionRows[i].state;
    }
    persistAvbdSelfContactFrictionAnchors(
        vbdScratch.avbdSelfContactFrictionInventory,
        vbdScratch.avbdSelfContactFrictionAnchors,
        vbdScratch.avbdSelfContactFrictionRows,
        scratch.next);
    stats.vbdAvbdSelfContactNormalRows += vbdScratch.avbdSelfContactRows.size();
    stats.vbdAvbdFrictionTangentRows
        += vbdScratch.avbdSelfContactFrictionRows.size();
  } else {
    vbdScratch.avbdContactDescriptors.clear();
    vbdScratch.avbdContactRows.clear();
    vbdScratch.avbdContactInventory.records().clear();
    vbdScratch.avbdFrictionDescriptors.clear();
    vbdScratch.avbdFrictionRows.clear();
    vbdScratch.avbdFrictionInventory.records().clear();
    vbdScratch.avbdFrictionAnchors.clear();
    vbdScratch.avbdSelfContactDescriptors.clear();
    vbdScratch.avbdSelfContactRows.clear();
    vbdScratch.avbdSelfContactInventory.records().clear();
    vbdScratch.avbdSelfContactFrictionDescriptors.clear();
    vbdScratch.avbdSelfContactFrictionRows.clear();
    vbdScratch.avbdSelfContactFrictionInventory.records().clear();
    vbdScratch.avbdSelfContactFrictionAnchors.clear();
    vbdScratch.avbdAttachmentDescriptors.clear();
    vbdScratch.avbdAttachmentRows.clear();
    vbdScratch.avbdAttachmentInventory.records().clear();
    vbdScratch.avbdSpringDescriptors.clear();
    vbdScratch.avbdSpringRows.clear();
    vbdScratch.avbdSpringInventory.records().clear();
    vbdScratch.avbdSolveFixed.clear();

    // stepPositions holds x^t after scripted controls but before solve, so it
    // is the Rayleigh displacement reference.
    // parallelBlockDescentDeformable falls back to the full-featured serial
    // driver when the injected executor has a single worker.
    result = dvbd::parallelBlockDescentDeformable(
        scratch.next,
        nodeModel.masses,
        scratch.activeFixed,
        scratch.inertialTargets,
        vbdScratch.springs,
        model.stiffness,
        vbdScratch.springAdjacency,
        vbdScratch.tets,
        lame.mu,
        lame.lambda,
        vbdScratch.tetAdjacency,
        timeStep,
        vbdScratch.coloring,
        options,
        executor,
        stepPositions,
        contactPlanes,
        frictionCoeff,
        selfContact,
        &vbdScratch.chebyshevTwoStepsBack,
        &vbdScratch.chebyshevBeforeSweep);
  }

  ++stats.vbdBodyCount;
  stats.vbdSweeps += result.iterations;
  stats.vbdVertexUpdates += result.vertexUpdates;
  stats.vbdResidualNormSquared = result.finalResidualNormSquared;
}

//==============================================================================
// Assemble the projected-Newton Hessian (inertia + spring + self-contact
// barrier + static ground barrier) with per-element PSD projection and solve
// H d = -gradient for the search direction. Built-in DART 7 World steps use
// retained dense LDLT scratch below the dense cap and sparse CG above it; Eigen
// sparse direct factorization is intentionally kept out of the simulation loop
// because its numeric storage uses malloc-family allocation. Returns false so
// the caller falls back to mass-scaled steepest descent when the linear solve
// does not produce a finite descent candidate. The per-element barrier/spring
// eigen-decompositions and sparse assembly are data-parallel GPU candidates.
bool computeProjectedNewtonDirection(
    const comps::DeformableNodeModel& nodeModel,
    const comps::DeformableSpringModel& model,
    std::span<const Eigen::Vector3d> positions,
    std::span<const std::uint8_t> fixed,
    std::span<const StaticGroundBarrier> barriers,
    std::span<const SphereObstacleBarrier> sphereObstacles,
    std::span<const BoxObstacleBarrier> boxObstacles,
    std::span<const CapsuleObstacleBarrier> capsuleObstacles,
    const SelfContactBarrierInputs* contactBarrier,
    const GroundFrictionInputs* groundFriction,
    const SelfContactFrictionInputs* selfContactFriction,
    const FemElasticityInputs* femElasticity,
    const double timeStep,
    std::span<const Eigen::Vector3d> gradient,
    comps::DeformableSolverScratch::Vector3Vector& direction,
    DeformableContactSolverScratch& solverCache,
    DeformableSolverStats& stats,
    double barrierStiffness = kDefaultBarrierStiffness,
    bool useIterativeSolver = false,
    bool useMatrixFreeSolver = false)
{
  const std::size_t nodeCount = positions.size();
  // The retained dense direct solve is used only below the DART-owned dense
  // scratch cap. Above that cap, the default path is sparse CG over retained
  // sparse Hessian storage so the first post-bake step remains allocation-free.
  // The explicit matrix-free opt-in goes one step further and bypasses sparse
  // Hessian assembly, using local block Hessian-vector products. The hard cap
  // is a runaway bound only.
  constexpr std::size_t kMaxIterativeNodes = 1000000;
  if (nodeCount == 0 || nodeCount > kMaxIterativeNodes) {
    return false;
  }

  const auto dim = static_cast<Eigen::Index>(3 * nodeCount);
  const bool solveMatrixFree = useMatrixFreeSolver;
  const bool solveDenseDirect = !solveMatrixFree && !useIterativeSolver
                                && dim <= kProjectedNewtonDenseDirectDofCap;
  const bool solveIteratively = !solveMatrixFree && !solveDenseDirect;
  const double invDt2 = 1.0 / (timeStep * timeStep);

  // Right-hand side: -gradient on free DOFs, zero on pinned (fixed) DOFs.
  Eigen::VectorXd& rhs = solverCache.projectedNewtonRhs;
  rhs.resize(dim);
  rhs.setZero();
  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (fixed[i] != 0u) {
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      rhs(static_cast<Eigen::Index>(3 * i + k)) = -gradient[i][k];
    }
  }

  // Triplet assembly. Each fixed DOF is pinned with a lone unit diagonal and a
  // zero right-hand side; only the free-free entries of each element block are
  // emitted (a principal submatrix of a PSD block stays PSD), which decouples
  // pinned DOFs from the free system exactly as zeroing a dense row/column
  // would. Eigen sums duplicate triplets, giving standard additive assembly.
  // The reserve below is a best-effort capacity hint only (it over-counts edges
  // touching fixed nodes and omits ground-barrier diagonals); setFromTriplets
  // sizes the matrix exactly from the actual triplet list regardless.
  const std::size_t activeContactCount
      = contactBarrier != nullptr && contactBarrier->candidates != nullptr
            ? contactBarrier->candidates->pointTriangleCandidates.size()
                  + contactBarrier->candidates->edgeEdgeCandidates.size()
            : 0u;
  const std::size_t femTetCount
      = femElasticity != nullptr ? femElasticity->tetrahedra.size() : 0u;
  const std::size_t tripletEstimate
      = 3 * nodeCount + 36 * model.edges.size() + 144 * femTetCount
        + 144 * activeContactCount + 36 * nodeCount;
  auto& triplets = solverCache.projectedNewtonTriplets;
  const std::size_t barrierBlockEstimate = 144 * activeContactCount;
  if ((!solveMatrixFree && tripletEstimate > triplets.capacity())
      || barrierBlockEstimate
             > solverCache.projectedNewtonBarrierBlocks.capacity()
      || activeContactCount
             > solverCache.projectedNewtonBarrierBlockNodes.capacity()) {
    // The baked scratch owns a finite active-set capacity. A late contact may
    // exceed it, but growing retained vectors in the simulation loop would
    // violate the post-prepare allocation contract. Fall back to the caller's
    // mass-scaled descent direction for this iteration; the contact gradient
    // and line search still enforce the barrier.
    return false;
  }
  triplets.clear();
  ProjectedNewtonMatrixFreeHessian matrixFreeHessian(
      solverCache.projectedNewtonMatrixFreeBlocks,
      solverCache.projectedNewtonMatrixFreeDiagonalBlocks,
      solverCache.projectedNewtonMatrixFreeInverseDiagonalBlocks);
  matrixFreeHessian.reset(solveMatrixFree ? nodeCount : 0);
  if (!solveMatrixFree) {
    triplets.reserve(tripletEstimate);
  } else {
    const std::size_t matrixFreeBlockEstimate
        = 4 * nodeCount + 4 * model.edges.size()
          + (femElasticity != nullptr && !femElasticity->tetrahedra.empty()
                 ? 16 * femElasticity->tetrahedra.size()
                 : 0u)
          + 16 * activeContactCount;
    matrixFreeHessian.blocks.reserve(matrixFreeBlockEstimate);
  }

  const auto isFree = [&](std::size_t node) {
    return fixed[node] == 0u;
  };
  const auto addBlock3 = [&](std::size_t nodeRow,
                             std::size_t nodeCol,
                             const Eigen::Matrix3d& block) {
    if (!isFree(nodeRow) || !isFree(nodeCol)) {
      return;
    }
    if (solveMatrixFree) {
      matrixFreeHessian.addBlock3(nodeRow, nodeCol, block);
      return;
    }
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        triplets.emplace_back(
            static_cast<Eigen::Index>(3 * nodeRow + r),
            static_cast<Eigen::Index>(3 * nodeCol + c),
            block(r, c));
      }
    }
  };
  const auto addDiagonalBlock3 = [&](std::size_t node, double diagonal) {
    if (solveMatrixFree) {
      matrixFreeHessian.addBlock3(
          node, node, diagonal * Eigen::Matrix3d::Identity());
      return;
    }
    for (int r = 0; r < 3; ++r) {
      triplets.emplace_back(
          static_cast<Eigen::Index>(3 * node + r),
          static_cast<Eigen::Index>(3 * node + r),
          diagonal);
    }
  };

  // Inertia: block diagonal, positive definite for free nodes. Fixed DOFs get
  // a unit diagonal so the global matrix stays positive definite.
  for (std::size_t i = 0; i < nodeCount; ++i) {
    const double diagonal = isFree(i) ? nodeModel.masses[i] * invDt2 : 1.0;
    addDiagonalBlock3(i, diagonal);
  }

  // Spring stretch Hessian per edge, PSD-projected over its 6x6 block. The
  // per-element projections are collected into one packed batch and projected
  // through the pluggable PSD backend (CPU by default, optional GPU offload),
  // then scattered. Batching is what lets a data-parallel backend amortize the
  // per-block eigensolves; the CPU backend is bit-identical to the previous
  // inline per-block projection.
  constexpr double minLength = 1e-12;
  constexpr std::size_t kEdgeBlockEntries = 36; // 6x6
  auto& edgeBlocks = solverCache.projectedNewtonEdgeBlocks;
  auto& edgeBlockNodes = solverCache.projectedNewtonEdgeBlockNodes;
  edgeBlocks.clear();
  edgeBlockNodes.clear();
  edgeBlocks.reserve(kEdgeBlockEntries * model.edges.size());
  edgeBlockNodes.reserve(model.edges.size());
  for (const auto& edge : model.edges) {
    const Eigen::Vector3d delta = positions[edge.nodeB] - positions[edge.nodeA];
    const double length = delta.norm();
    if (length <= minLength || !std::isfinite(length)) {
      continue;
    }
    const Eigen::Vector3d dir = delta / length;
    const Eigen::Matrix3d projection = dir * dir.transpose();
    const double scale = std::max(0.0, 1.0 - edge.restLength / length);
    const Eigen::Matrix3d block
        = model.stiffness
          * (projection + scale * (Eigen::Matrix3d::Identity() - projection));
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> edgeHessian;
    edgeHessian.block<3, 3>(0, 0) = block;
    edgeHessian.block<3, 3>(3, 3) = block;
    edgeHessian.block<3, 3>(0, 3) = -block;
    edgeHessian.block<3, 3>(3, 0) = -block;
    const std::size_t offset = edgeBlocks.size();
    edgeBlocks.resize(offset + kEdgeBlockEntries);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        edgeBlocks.data() + offset) = edgeHessian;
    edgeBlockNodes.push_back({edge.nodeA, edge.nodeB});
  }
  nb::projectSymmetricBlocksToPsd(edgeBlocks.data(), 6, edgeBlockNodes.size());
  for (std::size_t b = 0; b < edgeBlockNodes.size(); ++b) {
    const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
        edgeHessian(edgeBlocks.data() + b * kEdgeBlockEntries);
    const auto& nodes = edgeBlockNodes[b];
    for (int bi = 0; bi < 2; ++bi) {
      for (int bj = 0; bj < 2; ++bj) {
        addBlock3(
            nodes[bi], nodes[bj], edgeHessian.block<3, 3>(3 * bi, 3 * bj));
      }
    }
  }

  // Stable neo-Hookean FEM elasticity Hessian per tetrahedron, PSD-projected
  // over its 12x12 block through the same batched seam as the spring and
  // barrier blocks. Null femElasticity (the default mass-spring path) skips
  // this entirely, so spring bodies assemble exactly as before.
  if (femElasticity != nullptr && !femElasticity->tetrahedra.empty()
      && !femElasticity->restShapes.empty()) {
    const auto tets = femElasticity->tetrahedra;
    const auto rests = femElasticity->restShapes;
    const std::size_t tetCount = std::min(tets.size(), rests.size());
    constexpr std::size_t kTetBlockEntries = 144; // 12x12
    auto& tetBlocks = solverCache.projectedNewtonTetBlocks;
    auto& tetBlockNodes = solverCache.projectedNewtonTetBlockNodes;
    tetBlocks.clear();
    tetBlockNodes.clear();
    tetBlocks.reserve(kTetBlockEntries * tetCount);
    tetBlockNodes.reserve(tetCount);
    for (std::size_t t = 0; t < tetCount; ++t) {
      const auto& tet = tets[t];
      const fem::TetElementResult element = evaluateFemTetElement(
          *femElasticity, positions, tet, rests[t], /*computeHessian=*/true);
      if (!element.valid) {
        continue;
      }
      const std::size_t offset = tetBlocks.size();
      tetBlocks.resize(offset + kTetBlockEntries);
      Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(
          tetBlocks.data() + offset) = element.hessian;
      tetBlockNodes.push_back({tet.nodeA, tet.nodeB, tet.nodeC, tet.nodeD});
    }
    nb::projectSymmetricBlocksToPsd(tetBlocks.data(), 12, tetBlockNodes.size());
    for (std::size_t b = 0; b < tetBlockNodes.size(); ++b) {
      const Eigen::Map<const Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>
          projected(tetBlocks.data() + b * kTetBlockEntries);
      const auto& nodes = tetBlockNodes[b];
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              nodes[bi], nodes[bj], projected.block<3, 3>(3 * bi, 3 * bj));
        }
      }
    }
  }

  // Self-contact barrier Hessian per active contact, PSD-projected over 12x12.
  // Active point-triangle and edge-edge blocks are collected into one packed
  // batch, projected through the pluggable PSD backend (CPU by default,
  // optional GPU offload), then scattered -- the same batching seam the spring
  // blocks use. The CPU backend is bit-identical to the previous inline
  // per-block projection.
  if (contactBarrier != nullptr && contactBarrier->candidates != nullptr
      && !contactBarrier->triangles.empty() && contactBarrier->stiffness > 0.0
      && contactBarrier->squaredActivationDistance > 0.0) {
    const auto& candidates = *contactBarrier->candidates;
    const auto triangles = contactBarrier->triangles;
    const double sqAct = contactBarrier->squaredActivationDistance;
    const double kappa = contactBarrier->stiffness;
    constexpr std::size_t kBarrierBlockEntries = 144; // 12x12
    auto& barrierBlocks = solverCache.projectedNewtonBarrierBlocks;
    auto& barrierBlockNodes = solverCache.projectedNewtonBarrierBlockNodes;
    barrierBlocks.clear();
    barrierBlockNodes.clear();
    const auto collect12 = [&](const dc::Matrix12d& blockHessian,
                               const std::array<std::size_t, 4>& nodes) {
      const std::size_t offset = barrierBlocks.size();
      barrierBlocks.resize(offset + kBarrierBlockEntries);
      Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>(
          barrierBlocks.data() + offset) = blockHessian;
      barrierBlockNodes.push_back(nodes);
    };
    for (const auto& candidate : candidates.pointTriangleCandidates) {
      const auto& triangle = triangles[candidate.triangle];
      const auto result = dc::pointTriangleBarrier(
          positions[candidate.point],
          positions[triangle.nodeA],
          positions[triangle.nodeB],
          positions[triangle.nodeC],
          sqAct,
          kappa);
      if (!result.active) {
        continue;
      }
      collect12(
          result.hessian,
          {candidate.point, triangle.nodeA, triangle.nodeB, triangle.nodeC});
    }
    for (const auto& candidate : candidates.edgeEdgeCandidates) {
      const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
      const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
      const auto result = dc::edgeEdgeBarrier(
          positions[edgeA.nodeA],
          positions[edgeA.nodeB],
          positions[edgeB.nodeA],
          positions[edgeB.nodeB],
          sqAct,
          kappa);
      if (!result.active) {
        continue;
      }
      collect12(
          result.hessian, {edgeA.nodeA, edgeA.nodeB, edgeB.nodeA, edgeB.nodeB});
    }
    nb::projectSymmetricBlocksToPsd(
        barrierBlocks.data(), 12, barrierBlockNodes.size());
    for (std::size_t b = 0; b < barrierBlockNodes.size(); ++b) {
      const Eigen::Map<const Eigen::Matrix<double, 12, 12, Eigen::RowMajor>>
          projected(barrierBlocks.data() + b * kBarrierBlockEntries);
      const auto& nodes = barrierBlockNodes[b];
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              nodes[bi], nodes[bj], projected.block<3, 3>(3 * bi, 3 * bj));
        }
      }
    }
  }

  // Static ground barrier Hessian (vertical scalar per active free node).
  if (!barriers.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      const auto groundTop = staticGroundTopAt(positions[i], barriers);
      if (!groundTop.has_value()) {
        continue;
      }
      const double d = positions[i].z() - *groundTop;
      if (d <= 0.0 || d >= activationDistance || !std::isfinite(d)) {
        continue;
      }
      const double offset = d - activationDistance;
      const double logRatio = std::log(d / activationDistance);
      const double second
          = -barrierScale
            * (2.0 * logRatio + 4.0 * offset / d - (offset * offset) / (d * d));
      Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
      block(2, 2) = std::max(0.0, second);
      addBlock3(i, i, block);
    }
  }

  // Static sphere obstacle barrier Hessian. The full per-node barrier Hessian
  // is B''(d) n n^T + (B'(d)/|x-c|)(I - n n^T) with n the outward radial
  // normal; its tangential eigenvalues B'(d)/|x-c| are non-positive (the
  // barrier force pulls the node radially out), so the PSD projection keeps
  // only the rank-1 radial term max(0, B''(d)) n n^T -- the sphere analogue of
  // the vertical ground barrier curvature, now along the radial normal.
  if (!sphereObstacles.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      for (const auto& obstacle : sphereObstacles) {
        const Eigen::Vector3d offset = positions[i] - obstacle.center;
        const double centerDistance = offset.norm();
        const double d = centerDistance - obstacle.radius;
        if (d <= 0.0 || d >= activationDistance || centerDistance <= 0.0
            || !std::isfinite(d)) {
          continue;
        }
        const double distanceOffset = d - activationDistance;
        const double logRatio = std::log(d / activationDistance);
        const double second = -barrierScale
                              * (2.0 * logRatio + 4.0 * distanceOffset / d
                                 - (distanceOffset * distanceOffset) / (d * d));
        const double curvature = std::max(0.0, second);
        if (curvature <= 0.0) {
          continue;
        }
        const Eigen::Vector3d normal = offset / centerDistance;
        addBlock3(i, i, curvature * (normal * normal.transpose()));
      }
    }
  }

  // Static box obstacle barrier Hessian: the rank-1 radial curvature
  // max(0, B''(d)) n n^T along the outward box-surface normal, the box analogue
  // of the sphere obstacle barrier Hessian above (the tangential eigenvalues
  // are again non-positive and drop out under PSD projection).
  if (!boxObstacles.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      for (const auto& obstacle : boxObstacles) {
        Eigen::Vector3d normal;
        const double d
            = boxObstacleSurfaceDistance(positions[i], obstacle, normal);
        if (d <= 0.0 || d >= activationDistance || !std::isfinite(d)) {
          continue;
        }
        const double distanceOffset = d - activationDistance;
        const double logRatio = std::log(d / activationDistance);
        const double second = -barrierScale
                              * (2.0 * logRatio + 4.0 * distanceOffset / d
                                 - (distanceOffset * distanceOffset) / (d * d));
        const double curvature = std::max(0.0, second);
        if (curvature <= 0.0) {
          continue;
        }
        addBlock3(i, i, curvature * (normal * normal.transpose()));
      }
    }
  }

  // Capsule obstacle barrier Hessian: the same rank-1 radial block as the
  // sphere/box obstacle barriers (curvature * n n^T along the outward radial
  // normal; the tangential eigenvalues are non-positive and drop out under PSD
  // projection).
  if (!capsuleObstacles.empty()) {
    const double activationDistance = staticGroundBarrierActivationDistance();
    const double barrierScale = barrierStiffness;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i)) {
        continue;
      }
      for (const auto& obstacle : capsuleObstacles) {
        Eigen::Vector3d normal;
        const double d
            = capsuleObstacleSurfaceDistance(positions[i], obstacle, normal);
        if (d <= 0.0 || d >= activationDistance || !std::isfinite(d)) {
          continue;
        }
        const double distanceOffset = d - activationDistance;
        const double logRatio = std::log(d / activationDistance);
        const double second = -barrierScale
                              * (2.0 * logRatio + 4.0 * distanceOffset / d
                                 - (distanceOffset * distanceOffset) / (d * d));
        const double curvature = std::max(0.0, second);
        if (curvature <= 0.0) {
          continue;
        }
        addBlock3(i, i, curvature * (normal * normal.transpose()));
      }
    }
  }

  // Lagged ground-friction Hessian: a 3x3 tangent-plane block per node with an
  // active friction normal force. With P = I - n n^T the tangent projector onto
  // the plane orthogonal to the lagged ground normal n, T = u_T/||u_T|| the
  // unit slip direction (in that plane), the block is scale *
  // [ (f1/||u_T||) (P - T T^T) + f1' T T^T ]. Both coefficients are
  // non-negative and (P - T T^T), T T^T are PSD with ranges inside the tangent
  // plane, so the block is positive semidefinite by construction (no projection
  // needed) and -> scale * (2/eps) P isotropically as ||u_T|| -> 0. For flat
  // ground n = +z, P = diag(1, 1, 0) and this reduces to the xy 2x2 block.
  if (groundFriction != nullptr && groundFriction->coefficient > 0.0
      && groundFriction->epsilon > 0.0
      && !groundFriction->stepStartPositions.empty()
      && !groundFriction->laggedNormalForce.empty()) {
    const auto start = groundFriction->stepStartPositions;
    const auto normalForce = groundFriction->laggedNormalForce;
    const auto normalDirection = groundFriction->laggedNormalDirection;
    const double epsilon = groundFriction->epsilon;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (!isFree(i) || normalForce[i] <= 0.0) {
        continue;
      }
      const Eigen::Vector3d n = (i < normalDirection.size())
                                    ? normalDirection[i]
                                    : Eigen::Vector3d::UnitZ();
      const Eigen::Matrix3d projector
          = Eigen::Matrix3d::Identity() - n * n.transpose();
      const Eigen::Vector3d u = positions[i] - start[i];
      const Eigen::Vector3d tangent = u - n.dot(u) * n;
      const double y = tangent.norm();
      const double scale = groundFriction->coefficient * normalForce[i];
      constexpr double tiny = 1e-12;
      Eigen::Matrix3d block;
      if (y > tiny) {
        const double f1OverY = frictionF1(y, epsilon) / y;
        const double f1Prime
            = (y < epsilon) ? (2.0 / epsilon - 2.0 * y / (epsilon * epsilon))
                            : 0.0;
        const Eigen::Vector3d t = tangent / y;
        const Eigen::Matrix3d tt = t * t.transpose();
        block = scale * (f1OverY * (projector - tt) + f1Prime * tt);
      } else {
        block = scale * (2.0 / epsilon) * projector;
      }
      addBlock3(i, i, block);
    }
  }

  // Lagged self-contact friction Hessian (point-triangle): a PSD 12x12 block
  // per active contact, projection^T * H_2x2 * projection, scattered to the
  // four stencil nodes (free-free only). Mirrors the ground-friction tangent
  // Hessian through the contact's tangent projection; non-negative coefficients
  // keep H_2x2 positive semidefinite, so the assembled block is PSD by
  // construction.
  if (selfContactFriction != nullptr && selfContactFriction->coefficient > 0.0
      && selfContactFriction->epsilon > 0.0
      && !selfContactFriction->stepStartPositions.empty()
      && !selfContactFriction->contacts.empty()) {
    const auto start = selfContactFriction->stepStartPositions;
    const double epsilon = selfContactFriction->epsilon;
    for (const auto& contact : selfContactFriction->contacts) {
      if (contact.normalForce <= 0.0) {
        continue;
      }
      Eigen::Matrix<double, 12, 1> displacement;
      for (int k = 0; k < 4; ++k) {
        displacement.segment<3>(3 * k)
            = positions[contact.nodes[k]] - start[contact.nodes[k]];
      }
      const Eigen::Vector2d tangent = contact.projection * displacement;
      const double y = tangent.norm();
      const double scale
          = selfContactFriction->coefficient * contact.normalForce;
      constexpr double tiny = 1e-12;
      double f1OverY = 2.0 / epsilon;
      double f1Prime = 2.0 / epsilon;
      double tx = 0.0;
      double ty = 0.0;
      if (y > tiny) {
        f1OverY = frictionF1(y, epsilon) / y;
        f1Prime = (y < epsilon)
                      ? (2.0 / epsilon - 2.0 * y / (epsilon * epsilon))
                      : 0.0;
        tx = tangent.x() / y;
        ty = tangent.y() / y;
      }
      Eigen::Matrix2d tangentHessian;
      tangentHessian(0, 0) = f1OverY * (1.0 - tx * tx) + f1Prime * tx * tx;
      tangentHessian(0, 1) = (f1Prime - f1OverY) * tx * ty;
      tangentHessian(1, 0) = tangentHessian(0, 1);
      tangentHessian(1, 1) = f1OverY * (1.0 - ty * ty) + f1Prime * ty * ty;
      const Eigen::Matrix<double, 12, 12> block
          = scale * contact.projection.transpose() * tangentHessian
            * contact.projection;
      for (int bi = 0; bi < 4; ++bi) {
        for (int bj = 0; bj < 4; ++bj) {
          addBlock3(
              contact.nodes[bi],
              contact.nodes[bj],
              block.block<3, 3>(3 * bi, 3 * bj));
        }
      }
    }
  }

  Eigen::SparseMatrix<double>& hessian = solverCache.projectedNewtonHessian;
  if (!solveMatrixFree && !solveDenseDirect) {
    if (!tryAssembleProjectedNewtonHessianFromCachedPattern(solverCache, dim)) {
      // Eigen's sparse storage is not World-allocator-backed. Rebuilding a
      // changed contact pattern here would call the raw heap after prepare;
      // let the caller use its allocation-free descent fallback instead.
      return false;
    }
    const auto hessianNonZeros = static_cast<std::size_t>(hessian.nonZeros());
    const auto hessianCols = static_cast<std::size_t>(hessian.cols());
    using SparseStorageIndex = Eigen::SparseMatrix<double>::StorageIndex;
    const std::size_t hessianStorageBytes
        = hessianNonZeros * (sizeof(double) + sizeof(SparseStorageIndex))
          + (hessianCols + 1u) * sizeof(SparseStorageIndex);
    stats.projectedNewtonHessianNonZeros
        = std::max(stats.projectedNewtonHessianNonZeros, hessianNonZeros);
    stats.projectedNewtonHessianStorageBytes = std::max(
        stats.projectedNewtonHessianStorageBytes, hessianStorageBytes);
  }

  Eigen::VectorXd& solution = solverCache.projectedNewtonSolution;
  if (solveMatrixFree) {
    std::size_t cgIterations = 0;
    double cgError = 0.0;
    if (!solveMatrixFreeConjugateGradient(
            matrixFreeHessian,
            rhs,
            solution,
            solverCache.projectedNewtonMatrixFreeResidual,
            solverCache.projectedNewtonMatrixFreePreconditionedResidual,
            solverCache.projectedNewtonMatrixFreeDirection,
            solverCache.projectedNewtonMatrixFreeHessianDirection,
            cgIterations,
            cgError)
        || !solution.allFinite()) {
      solverCache.hessianPatternValid = false;
      return false;
    }
    ++stats.projectedNewtonIterativeSolves;
    ++stats.projectedNewtonMatrixFreeSolves;
    stats.projectedNewtonIterativeIterations += cgIterations;
    if (std::isfinite(cgError)) {
      stats.projectedNewtonIterativeMaxError
          = std::max(stats.projectedNewtonIterativeMaxError, cgError);
    }
    solverCache.hessianPatternValid = false;
  } else if (solveIteratively) {
    // Iterative sparse path: use a Jacobi-preconditioned CG loop over the
    // assembled Hessian while reusing the projected-Newton scratch vectors.
    // This keeps the opt-in sparse iterative solver inside the World allocator
    // boundary; non-convergence falls back to mass-scaled steepest descent just
    // as direct factorization failures do below.
    std::size_t cgIterations = 0;
    double cgError = 0.0;
    if (!solveSparseJacobiConjugateGradient(
            hessian,
            rhs,
            solution,
            solverCache.projectedNewtonMatrixFreeResidual,
            solverCache.projectedNewtonMatrixFreePreconditionedResidual,
            solverCache.projectedNewtonMatrixFreeDirection,
            solverCache.projectedNewtonMatrixFreeHessianDirection,
            solverCache.projectedNewtonIterativeInverseDiagonal,
            cgIterations,
            cgError)
        || !solution.allFinite()) {
      return false;
    }
    ++stats.projectedNewtonIterativeSolves;
    stats.projectedNewtonIterativeIterations += cgIterations;
    if (std::isfinite(cgError)) {
      stats.projectedNewtonIterativeMaxError
          = std::max(stats.projectedNewtonIterativeMaxError, cgError);
    }
  } else if (solveDenseDirect) {
    auto& denseHessian = solverCache.projectedNewtonDenseHessian;
    denseHessian.resize(dim, dim);
    denseHessian.setZero();
    for (const auto& triplet : triplets) {
      denseHessian(triplet.row(), triplet.col()) += triplet.value();
    }

    auto& ldlt = solverCache.projectedNewtonDenseLdlt;
    ldlt.compute(denseHessian);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      solverCache.hessianPatternValid = false;
      return false;
    }
    solution.resize(dim);
    solution = rhs;
    ldlt.solveInPlace(solution);
    if (ldlt.info() != Eigen::Success || !solution.allFinite()) {
      solverCache.hessianPatternValid = false;
      return false;
    }
    solverCache.hessianPatternValid = false;
  }

  direction.resize(nodeCount);
  for (std::size_t i = 0; i < nodeCount; ++i) {
    if (fixed[i] != 0u) {
      direction[i].setZero();
    } else {
      direction[i] = solution.segment<3>(static_cast<Eigen::Index>(3 * i));
    }
  }
  return true;
}

//==============================================================================
void advanceDeformableBody(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    const comps::DeformableNodeModel& nodeModel,
    const comps::DeformableSpringModel& model,
    const comps::DeformableMeshTopology& topology,
    comps::DeformableSolverScratch& scratch,
    DeformableContactSolverScratch& contactScratch,
    DeformableVbdScratch& vbdScratch,
    const comps::DeformableVbdConfig* vbdConfig,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    const Eigen::Vector3d& gravity,
    double timeStep,
    std::span<const StaticGroundBarrier> barriers,
    std::span<const SphereObstacleBarrier> sphereObstacles,
    std::span<const BoxObstacleBarrier> boxObstacles,
    std::span<const CapsuleObstacleBarrier> capsuleObstacles,
    const comps::DeformableMaterial& material,
    ComputeExecutor& executor,
    DeformableSolverStats& stats)
{
  const auto nodeCount = scratch.stepStartPositions.size();
  if (nodeCount == 0) {
    return;
  }

  const double frictionCoefficient = material.frictionCoefficient;

  stats.surfaceContactCandidateCapacityRequested = std::max(
      stats.surfaceContactCandidateCapacityRequested,
      contactScratch.requestedSurfaceCandidateCapacity);
  stats.surfaceContactCandidateCapacityResolved = std::max(
      stats.surfaceContactCandidateCapacityResolved,
      contactScratch.resolvedSurfaceCandidateCapacity);

  // Opt-in stable neo-Hookean FEM elasticity. Each tetrahedron's rest shape is
  // computed once and cached in the per-entity scratch (the rest configuration
  // never changes), then reused every step; the inputs are passed by pointer
  // into the objective and projected-Newton assembly, replacing the mass-spring
  // edge model for this body. Null (the default) leaves the spring path
  // byte-identical.
  FemElasticityInputs femElasticity;
  const FemElasticityInputs* femElasticityPtr = nullptr;
  if (material.useFiniteElementElasticity && !topology.tetrahedra.empty()
      && topology.restPositions.size() == nodeCount) {
    syncFemRestShapeScratch(nodeCount, topology, material, contactScratch);
    femElasticity.tetrahedra = topology.tetrahedra;
    femElasticity.restShapes = std::span<const fem::TetRestShape>(
        contactScratch.femRestShapes.data(),
        contactScratch.femRestShapes.size());
    femElasticity.lame = fem::tetMaterialParameters(
        material.youngsModulus,
        material.poissonRatio,
        material.useFixedCorotationalElasticity);
    femElasticity.fixedCorotational = material.useFixedCorotationalElasticity;
    femElasticityPtr = &femElasticity;
  }

  // Per-step barrier stiffness (kappa). Opt-in adaptive scaling balances the
  // barrier against the heaviest free node's inertial stiffness; off it is the
  // fixed default, so every existing scene is byte-identical.
  double barrierStiffness = kDefaultBarrierStiffness;
  if (material.useAdaptiveBarrierStiffness) {
    double maxNodalMass = 0.0;
    for (std::size_t i = 0; i < nodeCount; ++i) {
      maxNodalMass = std::max(maxNodalMass, nodeModel.masses[i]);
    }
    barrierStiffness = adaptiveBarrierStiffness(
        maxNodalMass, timeStep, staticGroundBarrierActivationDistance());
  }

  // Opt this body into the iterative (conjugate-gradient) Newton linear solve.
  // Systems above the retained dense-direct cap take the iterative path by
  // default; the flag forces it for any size so callers can trade direct-solve
  // speed for the factorization-free memory profile.
  const bool useIterativeSolver = material.useIterativeLinearSolver;
  const bool useMatrixFreeSolver = material.useMatrixFreeLinearSolver;

  stats.nodeCount += nodeCount;
  stats.edgeCount += model.edges.size();
  const bool requiresVbd = vbdConfig != nullptr && vbdConfig->enabled
                           && vbdConfig->requireVbdExecution;
  syncSurfaceContactTopology(
      topology.surfaceTriangles,
      nodeCount,
      !topology.tetrahedra.empty(),
      contactScratch);
  // These arrays belong exclusively to the projected-Newton path. Required
  // public VBD must not lazily allocate unused fallback-solver storage on its
  // first post-bake step.
  if (!requiresVbd) {
    reserveDeformableFrictionScratch(nodeCount, contactScratch);
  }

  scratch.inertialTargets.resize(nodeCount);
  scratch.next.resize(nodeCount);
  scratch.gradient.resize(nodeCount);
  scratch.direction.resize(nodeCount);
  scratch.candidate.resize(nodeCount);
  if (scratch.activeFixed.size() != nodeCount) {
    scratch.activeFixed.assign(nodeModel.fixed.begin(), nodeModel.fixed.end());
  }
  if (scratch.externalAccelerations.size() != nodeCount) {
    scratch.externalAccelerations.assign(nodeCount, Eigen::Vector3d::Zero());
  }
  if (scratch.previousStepPositions.size() != nodeCount) {
    scratch.previousStepPositions.assign(
        state.positions.begin(), state.positions.end());
  }

  const double dampingScale = 1.0 / (1.0 + model.damping * timeStep);
  const Eigen::Vector3d gravityStep = gravity * timeStep * timeStep;
  for (std::size_t i = 0; i < nodeCount; ++i) {
    scratch.inertialTargets[i] = scratch.stepStartPositions[i];
    scratch.next[i] = scratch.stepStartPositions[i];
    if (scratch.activeFixed[i] == 0u) {
      scratch.inertialTargets[i]
          += timeStep * dampingScale * scratch.stepStartVelocities[i]
             + gravityStep
             + timeStep * timeStep * scratch.externalAccelerations[i];
    }
  }

  const bool publicVbdIgnoresRigidObstacles
      = vbdConfig != nullptr && vbdConfig->enabled
        && vbdConfig->requireVbdExecution && vbdConfig->contactStiffness <= 0.0;
  if (!publicVbdIgnoresRigidObstacles) {
    makeInitialPositionsFeasible(
        scratch.next, scratch.activeFixed, barriers, &stats);
  }

  // VBD handles static ground barriers and static sphere/box obstacles itself
  // (lagged per-vertex half-space penalty contact + Coulomb friction) when
  // contactStiffness > 0. Static rigid-surface CCD obstacles are always sphere
  // or box bodies (collectStaticRigidSurfaceCcdObstacles skips other shapes),
  // i.e. exactly the obstacles VBD handles via those barriers, so they no
  // longer force the body onto the default solver; after the VBD solve, the
  // shared self-surface and static rigid-surface CCD limiters still clip fast
  // crossings before write-back. The VBD candidate is also clipped by the
  // shared inter-body deformable-surface CCD limiter. VBD still cannot honor
  // *moving* rigid-surface CCD when contact is enabled. A required public VBD
  // selection with zero contact stiffness deliberately ignores all rigid
  // obstacles, matching the public option contract. Surface triangles no
  // longer disqualify VBD:
  // runVbdDeformableSolve builds a lagged VT/EE self-contact candidate set and
  // adds the normal barrier blocks during the colored sweeps. A body with
  // static contacts but no VBD contact stiffness falls back to the default
  // solver so it still rests on / collides with them. The default-solver fast
  // path below keeps the stricter surface check so non-VBD bodies still get
  // self-contact.
  const bool movingRigidSurfaceFree = movingRigidSurfaceSnapshots.empty();
  const bool anyStaticContact = !barriers.empty() || !sphereObstacles.empty()
                                || !boxObstacles.empty()
                                || !rigidSurfaceSnapshots.empty();
  const bool vbdHandlesStaticContacts
      = !anyStaticContact
        || (vbdConfig != nullptr
            && (vbdConfig->contactStiffness > 0.0
                || vbdConfig->requireVbdExecution));
  // VBD does not yet handle static capsule-rod obstacles, so a body near one
  // falls back to the default solver (which does).
  const bool contactFree = movingRigidSurfaceFree && !anyStaticContact
                           && capsuleObstacles.empty()
                           && contactScratch.surfaceTriangles.empty();
  if (vbdConfig != nullptr && vbdConfig->enabled
      && vbdConfig->requireVbdExecution && vbdConfig->contactStiffness > 0.0) {
    DART_SIMULATION_THROW_T_IF(
        !movingRigidSurfaceFree,
        InvalidArgumentException,
        "Public deformable VBD selection cannot resolve moving rigid-surface "
        "contact; remove the moving surface or select the default deformable "
        "solver");
    DART_SIMULATION_THROW_T_IF(
        !capsuleObstacles.empty(),
        InvalidArgumentException,
        "Public deformable VBD selection cannot resolve static capsule "
        "obstacles; remove the capsule or select the default deformable "
        "solver");
  }
  if (vbdConfig != nullptr && vbdConfig->enabled
      && (publicVbdIgnoresRigidObstacles
          || (movingRigidSurfaceFree && capsuleObstacles.empty()))
      && vbdHandlesStaticContacts) {
    runVbdDeformableSolve(
        entity,
        scratch.stepStartPositions,
        scratch.stepAttachmentTargets,
        nodeModel,
        model,
        topology,
        scratch,
        vbdScratch,
        timeStep,
        material.youngsModulus,
        material.poissonRatio,
        material.useFiniteElementElasticity,
        material.useFiniteElementElasticity
            && material.useFixedCorotationalElasticity,
        barriers,
        sphereObstacles,
        boxObstacles,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.selfSurfaceCandidateCapacity,
        contactScratch.surfaceCandidateGrowthAllowed,
        frictionCoefficient,
        *vbdConfig,
        executor,
        stats);
    const std::span<const std::uint8_t> vbdCcdFixed
        = vbdScratch.avbdSolveFixed.size() == nodeCount
              ? std::span<const std::uint8_t>(vbdScratch.avbdSolveFixed)
              : std::span<const std::uint8_t>(scratch.activeFixed);
    applySurfaceContactCcdCandidateLimit(
        scratch.stepStartPositions,
        vbdCcdFixed,
        contactScratch,
        stats,
        scratch);
    applyInterBodySurfaceContactCcdCandidateLimit(
        entity,
        surfaceSnapshots,
        scratch.stepStartPositions,
        vbdCcdFixed,
        contactScratch,
        stats,
        scratch);
    if (vbdConfig->contactStiffness > 0.0) {
      applyStaticRigidSurfaceCcdCandidateLimit(
          rigidSurfaceSnapshots,
          scratch.stepStartPositions,
          vbdCcdFixed,
          contactScratch,
          stats,
          scratch);
    }
  } else if (
      model.edges.empty() && contactFree && femElasticityPtr == nullptr) {
    for (std::size_t i = 0; i < nodeCount; ++i) {
      if (scratch.activeFixed[i] == 0u) {
        scratch.next[i] = scratch.inertialTargets[i];
      }
    }
  } else {
    constexpr std::size_t maxIterations = 64;
    constexpr std::size_t maxLineSearchIterations = 16;
    constexpr double gradientTolerance = 1e-9;
    constexpr double armijo = nb::kDefaultSufficientDecreaseFactor;
    constexpr double minStep = 1e-12;
    const double backtrackingScale
        = nb::sanitizeBacktrackingScale(nb::kDefaultBacktrackingScale);

    double lastGradSquared = 0.0;
    double lastAcceptedStepInfinityNorm = 0.0;
    // Whether the solve left the loop via an early break (converged, stalled,
    // or non-finite energy) rather than exhausting the iteration cap. It gates
    // the terminal-residual recompute below.
    bool brokeEarly = false;
    auto& groundFrictionNormalForce = contactScratch.groundFrictionNormalForce;
    auto& groundFrictionNormalDirection
        = contactScratch.groundFrictionNormalDirection;
    auto& selfContactFrictionContacts
        = contactScratch.selfContactFrictionContacts;
    const double frictionEpsilon
        = staticGroundFrictionVelocityThreshold() * timeStep;
    for (std::size_t iteration = 0; iteration < maxIterations; ++iteration) {
      ++stats.solverIterations;
      ++stats.objectiveEvaluations;

      // Assemble the self-contact barrier active set at the current positions,
      // held fixed for this outer iteration (standard IPC). Skip when the body
      // has no surface mesh (point-mass bodies have no self-contact).
      SelfContactBarrierInputs contactBarrier;
      if (!contactScratch.surfaceTriangles.empty()) {
        const double dHat = selfContactBarrierActivationDistance();
        dc::ContactCandidateOptions barrierOptions;
        barrierOptions.activationDistance = dHat;
        barrierOptions.exactDistanceFilter = true;
        barrierOptions.excludeIncidentPointTriangles = true;
        barrierOptions.excludeAdjacentEdges = true;
        barrierOptions.pointMask = contactScratch.surfaceContactPointMask;
        barrierOptions.candidateCapacity
            = contactScratch.selfSurfaceCandidateCapacity;
        barrierOptions.allowCapacityGrowth
            = contactScratch.surfaceCandidateGrowthAllowed;
        dc::buildContactCandidatesSweep(
            scratch.next,
            contactScratch.surfaceTriangles,
            barrierOptions,
            contactScratch.barrierCandidates,
            contactScratch.sweepScratch);
        // Restrict point-triangle barrier candidates to surface-referenced
        // nodes (same mask the CCD path uses), so volumetric interior nodes do
        // not receive spurious barrier forces against their own shell.
        filterSurfaceContactPointCandidates(
            contactScratch.barrierCandidates,
            contactScratch.surfaceContactPointMask);
        recordSurfaceCandidateCount(
            stats,
            contactScratch.barrierCandidates.pointTriangleCandidates.size(),
            contactScratch.barrierCandidates.edgeEdgeCandidates.size(),
            contactScratch.barrierCandidates.stats.capacityOverflowCount);
        ++stats.selfContactBarrierCandidateBuilds;
        contactBarrier.candidates = &contactScratch.barrierCandidates;
        contactBarrier.triangles = contactScratch.surfaceTriangles;
        contactBarrier.squaredActivationDistance = dHat * dHat;
        contactBarrier.stiffness = selfContactBarrierStiffness();
      }

      // Lagged smoothed Coulomb friction for static-ground contact: the normal
      // force is sampled once per outer iteration (held fixed through the inner
      // line search), opposing each contacting node's tangential displacement
      // over the step. With mu == 0 or no ground contact this is a no-op.
      GroundFrictionInputs groundFriction;
      if (frictionCoefficient > 0.0
          && (!barriers.empty() || !sphereObstacles.empty()
              || !boxObstacles.empty() || !capsuleObstacles.empty())) {
        computeStaticGroundNormalForces(
            scratch.next,
            scratch.activeFixed,
            barriers,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addSphereObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            sphereObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addBoxObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            boxObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addCapsuleObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            capsuleObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        groundFriction.coefficient = frictionCoefficient;
        groundFriction.epsilon = frictionEpsilon;
        groundFriction.stepStartPositions = scratch.previousStepPositions;
        groundFriction.laggedNormalForce = std::span<const double>(
            groundFrictionNormalForce.data(), groundFrictionNormalForce.size());
        groundFriction.laggedNormalDirection = std::span<const Eigen::Vector3d>(
            groundFrictionNormalDirection.data(),
            groundFrictionNormalDirection.size());
      }

      // Lagged smoothed self-contact friction over the active point-triangle
      // barrier set, sampled once per outer iteration. No-op without friction
      // or a surface mesh.
      SelfContactFrictionInputs selfContactFriction;
      if (frictionCoefficient > 0.0 && contactBarrier.candidates != nullptr) {
        buildSelfContactFrictionContacts(
            scratch.next, contactBarrier, selfContactFrictionContacts);
        selfContactFriction.coefficient = frictionCoefficient;
        selfContactFriction.epsilon = frictionEpsilon;
        selfContactFriction.stepStartPositions = scratch.previousStepPositions;
        selfContactFriction.contacts
            = std::span<const SelfContactFrictionContact>(
                selfContactFrictionContacts.data(),
                selfContactFrictionContacts.size());
      }

      // Snapshot the cumulative active-contact counter so the per-iteration
      // active-set size can be recovered as the delta this objective evaluation
      // adds. Only this outer-iteration evaluation passes the counter (the
      // line-search and terminal evaluations pass nullptr), so the delta is
      // exactly the active barrier set at this iteration for this body.
      const std::size_t activeContactsBeforeEval
          = stats.selfContactBarrierActiveContacts;
      const double energy = evaluateDeformableObjective(
          nodeModel,
          model,
          scratch.next,
          scratch.inertialTargets,
          scratch.activeFixed,
          barriers,
          sphereObstacles,
          boxObstacles,
          capsuleObstacles,
          timeStep,
          &scratch.gradient,
          &contactBarrier,
          &stats.selfContactBarrierActiveContacts,
          &groundFriction,
          &selfContactFriction,
          femElasticityPtr,
          barrierStiffness);
      // Peak single-iteration active-set size across the step's bodies (the
      // IPC Fig. 23 "max contacts per step" axis).
      stats.maxActiveContactCount = std::max(
          stats.maxActiveContactCount,
          stats.selfContactBarrierActiveContacts - activeContactsBeforeEval);
      if (!std::isfinite(energy)) {
        brokeEarly = true;
        break;
      }

      const double gradSquared
          = gradientNormSquared(scratch.gradient, scratch.activeFixed);
      lastGradSquared = gradSquared;
      if (nb::projectedNewtonSquaredResidualConverged(
              gradSquared, gradientTolerance)) {
        brokeEarly = true;
        break;
      }

      // Mass-scaled steepest-descent direction: the graceful fallback used when
      // the dense Newton solve is skipped/fails, or when the Newton line search
      // cannot make progress.
      const auto fillSteepestDescentDirection = [&]() {
        for (std::size_t i = 0; i < nodeCount; ++i) {
          if (scratch.activeFixed[i] == 0u) {
            scratch.direction[i] = -(timeStep * timeStep / nodeModel.masses[i])
                                   * scratch.gradient[i];
          } else {
            scratch.direction[i].setZero();
          }
        }
      };

      // Armijo backtracking line search along scratch.direction, enforcing
      // every CCD limiter and the static-ground barrier. Commits the accepted
      // step into scratch.next and returns true on success.
      const auto runLineSearch = [&]() -> bool {
        double step = 1.0;
        for (std::size_t ls = 0; ls < maxLineSearchIterations; ++ls) {
          ++stats.lineSearchTrials;
          double directionalDerivative = buildLineSearchCandidate(
              scratch.next,
              scratch.direction,
              scratch.gradient,
              scratch.activeFixed,
              step,
              scratch.candidate);

          if (applySurfaceContactCcdLimit(
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyInterBodySurfaceContactCcdLimit(
                  entity,
                  surfaceSnapshots,
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyStaticRigidSurfaceCcdLimit(
                  rigidSurfaceSnapshots,
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyMovingRigidSurfaceCcdLimit(
                  movingRigidSurfaceSnapshots,
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  contactScratch,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && applyStaticGroundBarrierCcdLimit(
                  scratch.next,
                  scratch.direction,
                  scratch.gradient,
                  scratch.activeFixed,
                  barriers,
                  stats,
                  step,
                  scratch.candidate,
                  directionalDerivative)
              && directionalDerivative < -1e-24
              && satisfiesStaticGroundBarrier(
                  scratch.candidate, scratch.activeFixed, barriers)) {
            ++stats.objectiveEvaluations;
            const double candidateEnergy = evaluateDeformableObjective(
                nodeModel,
                model,
                scratch.candidate,
                scratch.inertialTargets,
                scratch.activeFixed,
                barriers,
                sphereObstacles,
                boxObstacles,
                capsuleObstacles,
                timeStep,
                nullptr,
                &contactBarrier,
                nullptr,
                &groundFriction,
                &selfContactFriction,
                femElasticityPtr,
                barrierStiffness);
            if (nb::satisfiesSufficientDecrease(
                    energy, candidateEnergy, directionalDerivative, armijo)) {
              // Record the accepted step's infinity norm (the actual per-node
              // position change). It is the converged-ness measure for stiff
              // barrier problems: it shrinks to ~0 at equilibrium even while
              // the raw gradient norm stays large because the barrier Hessian
              // is near-singular. scratch.candidate holds the new positions and
              // scratch.next the prior iterate (the swap follows).
              double stepInfinityNorm = 0.0;
              for (std::size_t i = 0; i < nodeCount; ++i) {
                if (scratch.activeFixed[i] != 0u) {
                  continue;
                }
                stepInfinityNorm = std::max(
                    stepInfinityNorm,
                    (scratch.candidate[i] - scratch.next[i])
                        .cwiseAbs()
                        .maxCoeff());
              }
              lastAcceptedStepInfinityNorm = stepInfinityNorm;
              std::swap(scratch.next, scratch.candidate);
              ++stats.acceptedLineSearchSteps;
              return true;
            }
          }

          ++stats.rejectedLineSearchCandidates;
          step *= backtrackingScale;
          if (step < minStep) {
            break;
          }
        }
        return false;
      };

      // Projected-Newton search direction (sparse PD Hessian solve); falls back
      // to mass-scaled steepest descent if the sparse solve is skipped or
      // fails. Newton lets the stiff barrier term converge cleanly.
      const bool newtonDirection = computeProjectedNewtonDirection(
          nodeModel,
          model,
          scratch.next,
          scratch.activeFixed,
          barriers,
          sphereObstacles,
          boxObstacles,
          capsuleObstacles,
          &contactBarrier,
          &groundFriction,
          &selfContactFriction,
          femElasticityPtr,
          timeStep,
          scratch.gradient,
          scratch.direction,
          contactScratch,
          stats,
          barrierStiffness,
          useIterativeSolver,
          useMatrixFreeSolver);
      if (newtonDirection) {
        ++stats.projectedNewtonSteps;
      } else {
        ++stats.projectedNewtonFallbacks;
        fillSteepestDescentDirection();
      }

      bool accepted = runLineSearch();
      if (!accepted && newtonDirection) {
        // The Newton direction was finite but the line search could not make
        // progress (an ill-conditioned barrier Hessian can round the
        // directional derivative to ~0). Degrade gracefully to mass-scaled
        // steepest descent within this iteration instead of stalling the solve.
        ++stats.projectedNewtonFallbacks;
        fillSteepestDescentDirection();
        accepted = runLineSearch();
      }

      if (!accepted) {
        brokeEarly = true;
        break;
      }
    }
    // If the solve ran to the iteration cap, the final accepted line-search
    // step changed scratch.next *after* lastGradSquared was recorded at the top
    // of that iteration, so the stored residual is the pre-final-step value.
    // Recompute the gradient at the terminal iterate -- rebuilding the
    // self-contact barrier active set and the lagged ground / self-contact
    // friction inputs there so the residual matches the in-loop objective --
    // and use it, so the convergence diagnostic reports the residual at solve
    // termination.
    if (!brokeEarly) {
      SelfContactBarrierInputs terminalBarrier;
      if (!contactScratch.surfaceTriangles.empty()) {
        const double dHat = selfContactBarrierActivationDistance();
        dc::ContactCandidateOptions barrierOptions;
        barrierOptions.activationDistance = dHat;
        barrierOptions.exactDistanceFilter = true;
        barrierOptions.excludeIncidentPointTriangles = true;
        barrierOptions.excludeAdjacentEdges = true;
        barrierOptions.pointMask = contactScratch.surfaceContactPointMask;
        barrierOptions.candidateCapacity
            = contactScratch.selfSurfaceCandidateCapacity;
        barrierOptions.allowCapacityGrowth
            = contactScratch.surfaceCandidateGrowthAllowed;
        dc::buildContactCandidatesSweep(
            scratch.next,
            contactScratch.surfaceTriangles,
            barrierOptions,
            contactScratch.barrierCandidates,
            contactScratch.sweepScratch);
        filterSurfaceContactPointCandidates(
            contactScratch.barrierCandidates,
            contactScratch.surfaceContactPointMask);
        recordSurfaceCandidateCount(
            stats,
            contactScratch.barrierCandidates.pointTriangleCandidates.size(),
            contactScratch.barrierCandidates.edgeEdgeCandidates.size(),
            contactScratch.barrierCandidates.stats.capacityOverflowCount);
        terminalBarrier.candidates = &contactScratch.barrierCandidates;
        terminalBarrier.triangles = contactScratch.surfaceTriangles;
        terminalBarrier.squaredActivationDistance = dHat * dHat;
        terminalBarrier.stiffness = selfContactBarrierStiffness();
      }
      GroundFrictionInputs terminalGroundFriction;
      if (frictionCoefficient > 0.0
          && (!barriers.empty() || !sphereObstacles.empty()
              || !boxObstacles.empty() || !capsuleObstacles.empty())) {
        computeStaticGroundNormalForces(
            scratch.next,
            scratch.activeFixed,
            barriers,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addSphereObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            sphereObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addBoxObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            boxObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        addCapsuleObstacleNormalForces(
            scratch.next,
            scratch.activeFixed,
            capsuleObstacles,
            groundFrictionNormalForce,
            groundFrictionNormalDirection);
        terminalGroundFriction.coefficient = frictionCoefficient;
        terminalGroundFriction.epsilon = frictionEpsilon;
        terminalGroundFriction.stepStartPositions
            = scratch.previousStepPositions;
        terminalGroundFriction.laggedNormalForce = std::span<const double>(
            groundFrictionNormalForce.data(), groundFrictionNormalForce.size());
        terminalGroundFriction.laggedNormalDirection
            = std::span<const Eigen::Vector3d>(
                groundFrictionNormalDirection.data(),
                groundFrictionNormalDirection.size());
      }
      SelfContactFrictionInputs terminalSelfContactFriction;
      if (frictionCoefficient > 0.0 && terminalBarrier.candidates != nullptr) {
        buildSelfContactFrictionContacts(
            scratch.next, terminalBarrier, selfContactFrictionContacts);
        terminalSelfContactFriction.coefficient = frictionCoefficient;
        terminalSelfContactFriction.epsilon = frictionEpsilon;
        terminalSelfContactFriction.stepStartPositions
            = scratch.previousStepPositions;
        terminalSelfContactFriction.contacts
            = std::span<const SelfContactFrictionContact>(
                selfContactFrictionContacts.data(),
                selfContactFrictionContacts.size());
      }
      const double terminalEnergy = evaluateDeformableObjective(
          nodeModel,
          model,
          scratch.next,
          scratch.inertialTargets,
          scratch.activeFixed,
          barriers,
          sphereObstacles,
          boxObstacles,
          capsuleObstacles,
          timeStep,
          &scratch.gradient,
          &terminalBarrier,
          nullptr,
          &terminalGroundFriction,
          &terminalSelfContactFriction,
          femElasticityPtr,
          barrierStiffness);
      if (std::isfinite(terminalEnergy)) {
        lastGradSquared
            = gradientNormSquared(scratch.gradient, scratch.activeFixed);
      }
    }
    // Record the worst-case solve residual across the step's bodies (the
    // gradient norm at termination), a convergence diagnostic for the
    // benchmark statistics.
    stats.finalGradientResidualNorm = std::max(
        stats.finalGradientResidualNorm,
        nb::projectedNewtonResidualNormFromSquared(lastGradSquared));
    // Converged-ness measure that stays meaningful for stiff barrier problems:
    // the worst-case last accepted step (infinity norm) across the step's
    // bodies. It tends to zero at equilibrium even when the gradient residual
    // stays large because the barrier Hessian is near-singular, so it is the
    // honest companion to finalGradientResidualNorm.
    stats.finalStepInfinityNorm
        = std::max(stats.finalStepInfinityNorm, lastAcceptedStepInfinityNorm);

    // Friction dissipation/active-contact diagnostics at the converged iterate,
    // using the final lagged ground normals and self-contact friction set.
    accumulateFrictionDiagnostics(
        scratch.next,
        scratch.previousStepPositions,
        scratch.activeFixed,
        frictionCoefficient,
        frictionEpsilon,
        groundFrictionNormalForce,
        groundFrictionNormalDirection,
        selfContactFrictionContacts,
        stats.frictionDissipation,
        stats.activeFrictionContacts);

    // Closest-approach (minimum) distance over the active self-contact barrier
    // set at the converged iterate -- the IPC intersection-free statistic. The
    // barrier candidate buffer holds the terminal active set: rebuilt at
    // scratch.next above when the solve hit the iteration cap, otherwise the
    // last in-loop set at the converged iterate (the final accepted step was
    // sub-tolerance, so the iterate barely moved), mirroring how the friction
    // diagnostics reuse their lagged sets.
    if (!contactScratch.surfaceTriangles.empty()) {
      const double dHat = selfContactBarrierActivationDistance();
      SelfContactBarrierInputs diagnosticBarrier;
      diagnosticBarrier.candidates = &contactScratch.barrierCandidates;
      diagnosticBarrier.triangles = contactScratch.surfaceTriangles;
      diagnosticBarrier.squaredActivationDistance = dHat * dHat;
      diagnosticBarrier.stiffness = selfContactBarrierStiffness();
      double bodyMinContactDistance = 0.0;
      const std::size_t bodyActiveContacts
          = accumulateContactDistanceDiagnostics(
              scratch.next, diagnosticBarrier, bodyMinContactDistance);
      if (bodyActiveContacts > 0) {
        // Fold across bodies: sum the active counts, take the minimum closest
        // approach. Gate the min on the running count (not on a zero-distance
        // sentinel) so the first contacting body always seeds it.
        if (stats.convergedActiveContactCount == 0) {
          stats.minActiveContactDistance = bodyMinContactDistance;
        } else {
          stats.minActiveContactDistance = std::min(
              stats.minActiveContactDistance, bodyMinContactDistance);
        }
        stats.convergedActiveContactCount += bodyActiveContacts;
      }
    }
  }
}

//==============================================================================
//==============================================================================

} // namespace

//==============================================================================
struct DeformableDynamicsStage::Scratch
{
  Scratch() = default;

  explicit Scratch(common::MemoryAllocator& allocator)
    : payloadAllocator(&allocator),
      barriers(common::StlAllocator<StaticGroundBarrier>{allocator}),
      sphereObstacles(common::StlAllocator<SphereObstacleBarrier>{allocator}),
      boxObstacles(common::StlAllocator<BoxObstacleBarrier>{allocator}),
      capsuleObstacles(common::StlAllocator<CapsuleObstacleBarrier>{allocator}),
      surfaceSnapshots(common::StlAllocator<SurfaceContactSnapshot>{allocator}),
      rigidSurfaceSnapshots(
          common::StlAllocator<SurfaceContactSnapshot>{allocator}),
      movingRigidSurfaceSnapshots(
          common::StlAllocator<SurfaceContactSnapshot>{allocator})
  {
  }

  common::MemoryAllocator* payloadAllocator = nullptr;
  std::vector<StaticGroundBarrier, common::StlAllocator<StaticGroundBarrier>>
      barriers;
  std::
      vector<SphereObstacleBarrier, common::StlAllocator<SphereObstacleBarrier>>
          sphereObstacles;
  std::vector<BoxObstacleBarrier, common::StlAllocator<BoxObstacleBarrier>>
      boxObstacles;
  std::vector<
      CapsuleObstacleBarrier,
      common::StlAllocator<CapsuleObstacleBarrier>>
      capsuleObstacles;
  std::vector<
      SurfaceContactSnapshot,
      common::StlAllocator<SurfaceContactSnapshot>>
      surfaceSnapshots;
  std::vector<
      SurfaceContactSnapshot,
      common::StlAllocator<SurfaceContactSnapshot>>
      rigidSurfaceSnapshots;
  std::vector<
      SurfaceContactSnapshot,
      common::StlAllocator<SurfaceContactSnapshot>>
      movingRigidSurfaceSnapshots;
  std::size_t surfaceSnapshotCount = 0;
  std::size_t rigidSurfaceSnapshotCount = 0;
  std::size_t movingRigidSurfaceSnapshotCount = 0;
};

//==============================================================================
namespace {

//==============================================================================
bool everyExecutableDeformableBodyIgnoresRigidObstacles(
    const detail::WorldRegistry& registry)
{
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableSpringModel,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial>();
  if (view.begin() == view.end()) {
    return false;
  }

  for (const auto entity : view) {
    const auto* config = registry.try_get<comps::DeformableVbdConfig>(entity);
    if (config == nullptr || !config->enabled || !config->requireVbdExecution
        || config->contactStiffness != 0.0) {
      return false;
    }
  }
  return true;
}

//==============================================================================
void clearRigidObstacleScratch(auto& scratch)
{
  scratch.barriers.clear();
  scratch.sphereObstacles.clear();
  scratch.boxObstacles.clear();
  scratch.capsuleObstacles.clear();
  scratch.rigidSurfaceSnapshotCount = 0;
  scratch.movingRigidSurfaceSnapshotCount = 0;
}

} // namespace

//==============================================================================
//==============================================================================
//==============================================================================
DeformableDynamicsStage::DeformableDynamicsStage()
  : DeformableDynamicsStage(nullptr)
{
}

//==============================================================================
DeformableDynamicsStage::DeformableDynamicsStage(
    common::MemoryManager* memoryManager)
  : m_memoryManager(memoryManager),
    m_scratch(
        memoryManager != nullptr
            ? stage_detail::constructStageOwnedScratch<Scratch>(
                  memoryManager, memoryManager->getFreeAllocator())
            : stage_detail::constructStageOwnedScratch<Scratch>(nullptr),
        ScratchDeleter{memoryManager})
{
}

//==============================================================================
DeformableDynamicsStage::~DeformableDynamicsStage() = default;

//==============================================================================
void DeformableDynamicsStage::ScratchDeleter::operator()(
    Scratch* scratch) const noexcept
{
  stage_detail::destroyStageOwnedScratch(memoryManager, scratch);
}

//==============================================================================
std::string_view DeformableDynamicsStage::getName() const noexcept
{
  return "deformable_dynamics";
}

//==============================================================================
ComputeStageMetadata DeformableDynamicsStage::getMetadata() const noexcept
{
  return {
      ComputeStageDomain::DeformableBody,
      ComputeStageAcceleration::TaskParallel
          | ComputeStageAcceleration::DataLocality
          | ComputeStageAcceleration::Gpu,
      {{"deformable_body.state", ComputeAccessMode::ReadWrite},
       {"deformable_body.model", ComputeAccessMode::Read},
       {"deformable_body.topology", ComputeAccessMode::Read},
       {"deformable_body.boundary_conditions", ComputeAccessMode::Read},
       {"rigid_body.kinematic_step_trace", ComputeAccessMode::Read},
       {"static_collision_geometry", ComputeAccessMode::Read}}};
}

//==============================================================================
void DeformableDynamicsStage::preflight(World& world)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto view = registry.view<comps::DeformableBodyTag>();
  for (const entt::entity entity : view) {
    const auto* state = registry.try_get<comps::DeformableNodeState>(entity);
    const auto* nodeModel
        = registry.try_get<comps::DeformableNodeModel>(entity);
    const auto* springModel
        = registry.try_get<comps::DeformableSpringModel>(entity);
    const auto* topology
        = registry.try_get<comps::DeformableMeshTopology>(entity);
    const auto* material = registry.try_get<comps::DeformableMaterial>(entity);
    const auto* config
        = registry.try_get<comps::DeformableContactConfig>(entity);
    const auto* solverScratch
        = registry.try_get<comps::DeformableSolverScratch>(entity);
    const auto* scratch
        = registry.try_get<DeformableContactSolverScratch>(entity);
    DART_SIMULATION_THROW_T_IF(
        state == nullptr || nodeModel == nullptr || springModel == nullptr
            || topology == nullptr || material == nullptr || config == nullptr
            || solverScratch == nullptr || scratch == nullptr
            || !scratch->surfaceCandidateCapacityPrepared,
        InvalidOperationException,
        "Executable deformable layout or surface contact capacity was not "
        "prepared before step");
    const std::size_t nodeCount = state->positions.size();
    DART_SIMULATION_THROW_T_IF(
        nodeCount == 0u || state->previousPositions.size() != nodeCount
            || state->velocities.size() != nodeCount
            || state->attachmentTargets.size() != nodeCount
            || nodeModel->masses.size() != nodeCount
            || nodeModel->fixed.size() != nodeCount
            || topology->restPositions.size() != nodeCount
            || topology->tetrahedronRestVolumes.size()
                   != topology->tetrahedra.size()
            || solverScratch->previousStepPositions.capacity() < nodeCount
            || solverScratch->stepStartPositions.capacity() < nodeCount
            || solverScratch->stepStartVelocities.capacity() < nodeCount
            || solverScratch->stepAttachmentTargets.capacity() < nodeCount,
        InvalidOperationException,
        "Deformable body state/model sizes changed after bake");
    const auto nodeOutOfRange = [nodeCount](std::size_t node) {
      return node >= nodeCount;
    };
    for (const auto& edge : springModel->edges) {
      DART_SIMULATION_THROW_T_IF(
          nodeOutOfRange(edge.nodeA) || nodeOutOfRange(edge.nodeB),
          InvalidOperationException,
          "Deformable spring topology changed after bake");
    }
    for (const auto& triangle : topology->surfaceTriangles) {
      DART_SIMULATION_THROW_T_IF(
          nodeOutOfRange(triangle.nodeA) || nodeOutOfRange(triangle.nodeB)
              || nodeOutOfRange(triangle.nodeC),
          InvalidOperationException,
          "Deformable surface topology changed after bake");
    }
    for (const auto& tet : topology->tetrahedra) {
      DART_SIMULATION_THROW_T_IF(
          nodeOutOfRange(tet.nodeA) || nodeOutOfRange(tet.nodeB)
              || nodeOutOfRange(tet.nodeC) || nodeOutOfRange(tet.nodeD),
          InvalidOperationException,
          "Deformable tetrahedral topology changed after bake");
    }
    DART_SIMULATION_THROW_T_IF(
        config->surfaceCandidateCapacity
            != scratch->requestedSurfaceCandidateCapacity,
        InvalidOperationException,
        "Deformable surface contact capacity policy changed after bake");
    DART_SIMULATION_THROW_T_IF(
        scratch->resolvedSurfaceCandidateCapacity == 0u
            || scratch->selfSurfaceCandidateCapacity == 0u
            || scratch->bakedSurfaceCandidateCount
                   > scratch->resolvedSurfaceCandidateCapacity
            || scratch->selfSurfaceCandidateCapacity
                   > scratch->resolvedSurfaceCandidateCapacity,
        InvalidOperationException,
        "Deformable surface contact capacity cache is invalid");
  }
}

namespace {

//==============================================================================
std::size_t combinedSurfaceCandidateCount(const dc::ContactCandidateSet& set)
{
  return checkedSurfaceCandidateAdd(
      set.pointTriangleCandidates.size(),
      set.edgeEdgeCandidates.size(),
      "combined emitted candidates");
}

//==============================================================================
std::size_t primeSurfaceContactCandidateScratch(
    const comps::DeformableNodeState& state,
    std::size_t candidateCapacity,
    DeformableContactSolverScratch& contactScratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    return 0u;
  }

  reserveSurfaceContactCandidateScratch(state.positions.size(), contactScratch);

  dc::buildMotionAwareContactCandidatesSweep(
      state.positions,
      state.positions,
      contactScratch.surfaceTriangles,
      makeSurfaceContactCandidateOptions(
          candidateCapacity,
          contactScratch.surfaceContactPointMask,
          contactScratch.surfaceCandidateGrowthAllowed),
      contactScratch.candidates,
      contactScratch.sweepScratch);
  filterSurfaceContactPointCandidates(
      contactScratch.candidates, contactScratch.surfaceContactPointMask);

  dc::ContactCandidateOptions barrierOptions;
  barrierOptions.activationDistance = selfContactBarrierActivationDistance();
  barrierOptions.exactDistanceFilter = true;
  barrierOptions.excludeIncidentPointTriangles = true;
  barrierOptions.excludeAdjacentEdges = true;
  barrierOptions.pointMask = contactScratch.surfaceContactPointMask;
  barrierOptions.candidateCapacity = candidateCapacity;
  barrierOptions.allowCapacityGrowth
      = contactScratch.surfaceCandidateGrowthAllowed;
  dc::buildContactCandidatesSweep(
      state.positions,
      contactScratch.surfaceTriangles,
      barrierOptions,
      contactScratch.barrierCandidates,
      contactScratch.sweepScratch);
  filterSurfaceContactPointCandidates(
      contactScratch.barrierCandidates, contactScratch.surfaceContactPointMask);
  return std::max(
      combinedSurfaceCandidateCount(contactScratch.candidates),
      combinedSurfaceCandidateCount(contactScratch.barrierCandidates));
}

//==============================================================================
void collectDeformableSurfaceSnapshotsInto(
    const sxdetail::WorldRegistry& registry,
    auto& snapshots,
    std::size_t& snapshotCount,
    common::MemoryAllocator* payloadAllocator = nullptr,
    bool usePreparedStepState = false)
{
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableMeshTopology>();

  snapshotCount = 0;
  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    if (topology.surfaceTriangles.empty()) {
      continue;
    }

    auto& snapshot = nextSurfaceContactSnapshot(
        snapshots, snapshotCount, payloadAllocator);
    snapshot.entity = entity;
    const auto* solverScratch
        = registry.try_get<comps::DeformableSolverScratch>(entity);
    const bool hasPreparedStepState
        = usePreparedStepState && solverScratch != nullptr
          && solverScratch->stepStartPositions.size() == state.positions.size();
    if (hasPreparedStepState) {
      snapshot.positions.assign(
          solverScratch->stepStartPositions.begin(),
          solverScratch->stepStartPositions.end());
    } else {
      snapshot.positions.assign(state.positions.begin(), state.positions.end());
    }
    copySurfaceContactTopology(
        topology.surfaceTriangles,
        state.positions.size(),
        !topology.tetrahedra.empty(),
        snapshot.surfaceTriangles,
        snapshot.surfaceContactPointMask);
    dc::buildUniqueSurfaceEdges(
        snapshot.surfaceTriangles, snapshot.surfaceEdges);
  }
}

//==============================================================================
std::size_t primeSurfaceContactSnapshotSweepScratch(
    std::span<const Eigen::Vector3d> positions,
    std::span<const SurfaceContactSnapshot> snapshots,
    std::size_t candidateCapacity,
    DeformableContactSolverScratch& contactScratch)
{
  std::size_t observed = 0u;
  const auto candidateOptions
      = makeSurfaceContactCandidateOptions(candidateCapacity);
  const auto ccdOptions = makeSurfaceContactCcdOptions();
  for (const auto& snapshot : snapshots) {
    if (snapshot.surfaceTriangles.empty()) {
      continue;
    }

    const auto result = interBodySurfaceContactStepBound(
        positions,
        positions,
        contactScratch.surfaceTriangles,
        contactScratch.surfaceContactPointMask,
        contactScratch.interBodyCurrentEdges,
        snapshot,
        candidateOptions,
        ccdOptions,
        contactScratch);
    observed = std::max(
        observed,
        checkedSurfaceCandidateAdd(
            result.pointTriangleCandidateCount,
            result.edgeEdgeCandidateCount,
            "baked inter-surface candidates"));
  }
  return observed;
}

//==============================================================================
std::size_t primeInterBodySurfaceContactScratch(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    std::size_t candidateCapacity,
    DeformableContactSolverScratch& contactScratch)
{
  std::size_t observed = 0u;
  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);

    const auto candidateOptions
        = makeSurfaceContactCandidateOptions(candidateCapacity);
    const auto ccdOptions = makeSurfaceContactCcdOptions();
    for (const auto& snapshot : surfaceSnapshots) {
      if (snapshot.entity == entity || snapshot.surfaceTriangles.empty()) {
        continue;
      }

      const auto result = interBodySurfaceContactStepBound(
          state.positions,
          state.positions,
          contactScratch.surfaceTriangles,
          contactScratch.surfaceContactPointMask,
          contactScratch.interBodyCurrentEdges,
          snapshot,
          candidateOptions,
          ccdOptions,
          contactScratch);
      observed = std::max(
          observed,
          checkedSurfaceCandidateAdd(
              result.pointTriangleCandidateCount,
              result.edgeEdgeCandidateCount,
              "baked deformable inter-body candidates"));
    }
  }

  observed = std::max(
      observed,
      primeSurfaceContactSnapshotSweepScratch(
          state.positions,
          rigidSurfaceSnapshots,
          candidateCapacity,
          contactScratch));
  observed = std::max(
      observed,
      primeSurfaceContactSnapshotSweepScratch(
          state.positions,
          movingRigidSurfaceSnapshots,
          candidateCapacity,
          contactScratch));
  return observed;
}

//==============================================================================
std::size_t conservativeSurfaceCandidateBakeBound(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    DeformableContactSolverScratch& contactScratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);
  }
  const SurfaceCandidateTopologyCounts current = surfaceCandidateTopologyCounts(
      state.positions.size(),
      contactScratch.surfaceTriangles,
      contactScratch.surfaceContactPointMask,
      contactScratch.interBodyCurrentEdges,
      contactScratch.surfaceEdgeDegrees);
  std::size_t bound = current.validSelfPairs;
  const auto absorb = [&](auto snapshots, bool excludeSelf) {
    for (const SurfaceContactSnapshot& snapshot : snapshots) {
      if (snapshot.surfaceTriangles.empty()
          || (excludeSelf && snapshot.entity == entity)) {
        continue;
      }
      const SurfaceCandidateTopologyCounts obstacle
          = surfaceCandidateTopologyCounts(
              snapshot.positions.size(),
              snapshot.surfaceTriangles,
              snapshot.surfaceContactPointMask,
              snapshot.surfaceEdges,
              contactScratch.surfaceEdgeDegrees);
      bound
          = std::max(bound, interSurfaceCandidatePairBound(current, obstacle));
    }
  };
  absorb(surfaceSnapshots, true);
  absorb(rigidSurfaceSnapshots, false);
  absorb(movingRigidSurfaceSnapshots, false);
  return std::max<std::size_t>(1u, bound);
}

//==============================================================================
void resolveSurfaceContactCandidateCapacity(
    entt::entity entity,
    const comps::DeformableNodeState& state,
    const comps::DeformableContactConfig& config,
    std::span<const SurfaceContactSnapshot> surfaceSnapshots,
    std::span<const SurfaceContactSnapshot> rigidSurfaceSnapshots,
    std::span<const SurfaceContactSnapshot> movingRigidSurfaceSnapshots,
    DeformableContactSolverScratch& contactScratch)
{
  if (contactScratch.surfaceTriangles.empty()) {
    contactScratch.interBodyCurrentEdges.clear();
  } else {
    dc::buildUniqueSurfaceEdges(
        contactScratch.surfaceTriangles, contactScratch.interBodyCurrentEdges);
  }

  const SurfaceCandidateTopologyCounts current = surfaceCandidateTopologyCounts(
      state.positions.size(),
      contactScratch.surfaceTriangles,
      contactScratch.surfaceContactPointMask,
      contactScratch.interBodyCurrentEdges,
      contactScratch.surfaceEdgeDegrees);
  std::size_t theoreticalValidPairBound = current.validSelfPairs;

  const auto absorbSnapshots = [&](auto snapshots, bool excludeSelf) {
    for (const SurfaceContactSnapshot& snapshot : snapshots) {
      if (snapshot.surfaceTriangles.empty()
          || (excludeSelf && snapshot.entity == entity)) {
        continue;
      }
      const SurfaceCandidateTopologyCounts obstacle
          = surfaceCandidateTopologyCounts(
              snapshot.positions.size(),
              snapshot.surfaceTriangles,
              snapshot.surfaceContactPointMask,
              snapshot.surfaceEdges,
              contactScratch.surfaceEdgeDegrees);
      theoreticalValidPairBound = std::max(
          theoreticalValidPairBound,
          interSurfaceCandidatePairBound(current, obstacle));
    }
  };
  absorbSnapshots(surfaceSnapshots, true);
  absorbSnapshots(rigidSurfaceSnapshots, false);
  absorbSnapshots(movingRigidSurfaceSnapshots, false);

  const std::size_t requested = config.surfaceCandidateCapacity;
  const AutomaticSurfaceCandidatePolicy automaticPolicy
      = automaticSurfaceCandidateCapacity(
          theoreticalValidPairBound, contactScratch.bakedSurfaceCandidateCount);
  const std::size_t resolved
      = requested != 0u ? requested : automaticPolicy.capacity;
  const bool growthAllowed = requested == 0u && automaticPolicy.growthAllowed;
  DART_SIMULATION_THROW_T_IF(
      contactScratch.bakedSurfaceCandidateCount > resolved,
      InvalidOperationException,
      "Deformable surface contact candidate capacity {} is smaller than the "
      "baked active set {}",
      resolved,
      contactScratch.bakedSurfaceCandidateCount);

  contactScratch.requestedSurfaceCandidateCapacity = requested;
  contactScratch.resolvedSurfaceCandidateCapacity = resolved;
  contactScratch.surfaceCandidateGrowthAllowed = growthAllowed;
  contactScratch.selfSurfaceCandidateCapacity
      = std::min(resolved, std::max<std::size_t>(1u, current.validSelfPairs));
  contactScratch.surfaceCandidateCapacityPrepared = true;

  const std::size_t selfCapacity = contactScratch.selfSurfaceCandidateCapacity;
  contactScratch.candidates.pointTriangleCandidates.reserve(selfCapacity);
  contactScratch.candidates.edgeEdgeCandidates.reserve(selfCapacity);
  contactScratch.barrierCandidates.pointTriangleCandidates.reserve(
      selfCapacity);
  contactScratch.barrierCandidates.edgeEdgeCandidates.reserve(selfCapacity);
}

} // namespace

//==============================================================================
void DeformableDynamicsStage::prepare(World& world)
{
  if (m_scratch == nullptr) {
    m_scratch = std::unique_ptr<Scratch, ScratchDeleter>(
        m_memoryManager != nullptr
            ? stage_detail::constructStageOwnedScratch<Scratch>(
                  m_memoryManager, m_memoryManager->getFreeAllocator())
            : stage_detail::constructStageOwnedScratch<Scratch>(nullptr),
        ScratchDeleter{m_memoryManager});
  }

  auto& registry = dart::simulation::detail::registryOf(world);
  auto& worldFreeAllocator = world.getMemoryManager().getFreeAllocator();
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableMeshTopology>();

  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& nodeModel = registry.get<comps::DeformableNodeModel>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    auto& solverScratch
        = registry.get_or_emplace<comps::DeformableSolverScratch>(
            entity, worldFreeAllocator);
    reserveDeformableSolverScratch(state, solverScratch);
    solverScratch.previousStepPositions.assign(
        state.positions.begin(), state.positions.end());
    solverScratch.externalAccelerations.assign(
        state.positions.size(), Eigen::Vector3d::Zero());
    solverScratch.activeFixed.assign(
        nodeModel.fixed.begin(), nodeModel.fixed.end());
    solverScratch.activeDirichlet.assign(state.positions.size(), 0u);

    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(
            entity, worldFreeAllocator);
    const auto* vbdConfig
        = registry.try_get<comps::DeformableVbdConfig>(entity);
    const bool requiresVbd = vbdConfig != nullptr && vbdConfig->enabled
                             && vbdConfig->requireVbdExecution;
    syncSurfaceContactTopology(
        topology.surfaceTriangles,
        state.positions.size(),
        !topology.tetrahedra.empty(),
        contactScratch);
    contactScratch.bakedSurfaceCandidateCount = 0u;
    const auto* material = registry.try_get<comps::DeformableMaterial>(entity);
    if (!requiresVbd && material != nullptr) {
      syncFemRestShapeScratch(
          state.positions.size(), topology, *material, contactScratch, true);
    }
    (void)registry.get_or_emplace<DeformableVbdScratch>(
        entity, worldFreeAllocator);
  }

  auto& scratch = *m_scratch;
  DeformableSolverStats stats;
  if (everyExecutableDeformableBodyIgnoresRigidObstacles(registry)) {
    clearRigidObstacleScratch(scratch);
  } else {
    collectStaticGroundBarriersInto(world, scratch.barriers);
    collectSphereObstacleBarriersInto(world, scratch.sphereObstacles);
    collectBoxObstacleBarriersInto(world, scratch.boxObstacles);
    collectCapsuleObstacleBarriersInto(world, scratch.capsuleObstacles);
    collectStaticRigidSurfaceCcdObstaclesInto(
        world,
        stats,
        scratch.rigidSurfaceSnapshots,
        scratch.rigidSurfaceSnapshotCount,
        scratch.payloadAllocator);
    collectMovingRigidSurfaceCcdObstaclesInto(
        world,
        world.getTimeStep(),
        stats,
        scratch.movingRigidSurfaceSnapshots,
        scratch.movingRigidSurfaceSnapshotCount,
        true,
        scratch.payloadAllocator);
  }
  collectDeformableSurfaceSnapshotsInto(
      registry,
      scratch.surfaceSnapshots,
      scratch.surfaceSnapshotCount,
      scratch.payloadAllocator);

  // Mirrors the step-time gate on the static barrier and lagged ground-friction
  // terms (see the GroundFrictionInputs setup in the solve): when any of these
  // is present a free node can take a dense node-local 3x3 Hessian block, so
  // the baked sparse pattern has to reserve one.
  const bool hasStaticObstacleBarriers
      = !scratch.barriers.empty() || !scratch.sphereObstacles.empty()
        || !scratch.boxObstacles.empty() || !scratch.capsuleObstacles.empty();
  const auto surfaceSnapshots = activeSurfaceContactSnapshots(
      scratch.surfaceSnapshots, scratch.surfaceSnapshotCount);
  const auto rigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      scratch.rigidSurfaceSnapshots, scratch.rigidSurfaceSnapshotCount);
  const auto movingRigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      scratch.movingRigidSurfaceSnapshots,
      scratch.movingRigidSurfaceSnapshotCount);
  for (const auto entity : view) {
    const auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(
            entity, worldFreeAllocator);
    const auto& contactConfig
        = registry.get_or_emplace<comps::DeformableContactConfig>(entity);
    const std::size_t conservativeBakeBound
        = conservativeSurfaceCandidateBakeBound(
            entity,
            state,
            surfaceSnapshots,
            rigidSurfaceSnapshots,
            movingRigidSurfaceSnapshots,
            contactScratch);
    const std::size_t bakeCapacity
        = contactConfig.surfaceCandidateCapacity != 0u
              ? contactConfig.surfaceCandidateCapacity
              : conservativeBakeBound;
    contactScratch.bakedSurfaceCandidateCount
        = primeSurfaceContactCandidateScratch(
            state, bakeCapacity, contactScratch);
    contactScratch.bakedSurfaceCandidateCount = std::max(
        contactScratch.bakedSurfaceCandidateCount,
        primeInterBodySurfaceContactScratch(
            entity,
            state,
            surfaceSnapshots,
            rigidSurfaceSnapshots,
            movingRigidSurfaceSnapshots,
            bakeCapacity,
            contactScratch));
    resolveSurfaceContactCandidateCapacity(
        entity,
        state,
        contactConfig,
        surfaceSnapshots,
        rigidSurfaceSnapshots,
        movingRigidSurfaceSnapshots,
        contactScratch);
    auto* vbdConfig = registry.try_get<comps::DeformableVbdConfig>(entity);
    const auto* model = registry.try_get<comps::DeformableSpringModel>(entity);
    const bool requiresVbd = vbdConfig != nullptr && vbdConfig->enabled
                             && vbdConfig->requireVbdExecution;
    if (!requiresVbd) {
      reserveDeformableFrictionScratch(state.positions.size(), contactScratch);
      if (model != nullptr) {
        const auto* projectedNewtonMaterial
            = registry.try_get<comps::DeformableMaterial>(entity);
        // Rest-shape synchronization can invalidate the sparse pattern when
        // tet connectivity changes. Reserve after the contact cap resolves so
        // delayed active sets stay inside retained block/triplet storage.
        reserveProjectedNewtonScratch(
            state.positions.size(),
            *model,
            topology,
            projectedNewtonMaterial,
            hasStaticObstacleBarriers,
            contactScratch);
      }
    }
    const bool publicVbdIgnoresRigidObstacles
        = vbdConfig != nullptr && vbdConfig->enabled
          && vbdConfig->requireVbdExecution
          && vbdConfig->contactStiffness <= 0.0;
    if (vbdConfig != nullptr && vbdConfig->enabled && model != nullptr
        && (publicVbdIgnoresRigidObstacles
            || (scratch.capsuleObstacles.empty()
                && movingRigidSurfaceSnapshots.empty()))) {
      auto& vbdScratch = registry.get_or_emplace<DeformableVbdScratch>(
          entity, worldFreeAllocator);
      const auto* material
          = registry.try_get<comps::DeformableMaterial>(entity);
      syncVbdTopologyScratch(
          state.positions.size(),
          *model,
          topology,
          material != nullptr && material->useFiniteElementElasticity,
          vbdScratch,
          true);
      // Both Chebyshev extrapolation and deterministic convergence checking
      // retain a full pre-sweep position vector. Reserve it during bake so a
      // positive public convergence tolerance cannot allocate in World::step.
      if (vbdConfig->useChebyshev || vbdConfig->convergenceDisplacement > 0.0) {
        reserveVbdChebyshevScratch(state.positions.size(), vbdScratch);
      }
      primeVbdStaticContactScratch(
          state.positions.size(),
          scratch.barriers,
          scratch.sphereObstacles,
          scratch.boxObstacles,
          *vbdConfig,
          vbdScratch);
      if (contactScratch.surfaceTriangles.size() >= 2) {
        reserveVbdSelfContactCandidateScratch(
            state.positions.size(),
            contactScratch.surfaceTriangles.size(),
            contactScratch.selfSurfaceCandidateCapacity,
            vbdScratch);
      }
      if (vbdConfig->useAvbdContactNormalRows
          && vbdConfig->contactStiffness > 0.0) {
        const std::size_t contactRowCapacity = state.positions.size();
        vbdScratch.avbdSolveFixed.reserve(state.positions.size());
        vbdScratch.avbdContactDescriptors.reserve(contactRowCapacity);
        vbdScratch.avbdContactInventory.reserve(contactRowCapacity);
        vbdScratch.avbdContactRows.reserve(contactRowCapacity);
        const std::size_t frictionRowCapacity = checkedSurfaceCandidateMultiply(
            2u, contactRowCapacity, "static AVBD friction rows");
        vbdScratch.avbdFrictionDescriptors.reserve(frictionRowCapacity);
        vbdScratch.avbdFrictionInventory.reserve(frictionRowCapacity);
        vbdScratch.avbdFrictionAnchors.reserve(contactRowCapacity);
        vbdScratch.avbdFrictionRows.reserve(frictionRowCapacity);
        vbdScratch.previousAvbdFrictionWarmStarts.reserve(frictionRowCapacity);
      }
      if (vbdConfig->useAvbdSelfContactNormalRows
          && contactScratch.surfaceTriangles.size() >= 2) {
        vbdScratch.avbdSolveFixed.reserve(state.positions.size());

        const double dHat = selfContactBarrierActivationDistance();
        dc::ContactCandidateOptions candidateOptions;
        candidateOptions.activationDistance = dHat;
        candidateOptions.exactDistanceFilter = true;
        candidateOptions.excludeIncidentPointTriangles = true;
        candidateOptions.excludeAdjacentEdges = true;
        candidateOptions.pointMask = contactScratch.surfaceContactPointMask;
        candidateOptions.candidateCapacity
            = contactScratch.selfSurfaceCandidateCapacity;
        candidateOptions.allowCapacityGrowth
            = contactScratch.surfaceCandidateGrowthAllowed;
        dc::buildMotionAwareContactCandidatesSweep(
            state.positions,
            state.positions,
            contactScratch.surfaceTriangles,
            candidateOptions,
            vbdScratch.selfContactCandidates,
            vbdScratch.selfContactSweepScratch);
        filterSurfaceContactPointCandidates(
            vbdScratch.selfContactCandidates,
            contactScratch.surfaceContactPointMask);
        vbdScratch.selfContactAdjacency.rebuild(
            state.positions.size(),
            vbdScratch.selfContactCandidates,
            contactScratch.surfaceTriangles,
            dHat * dHat,
            selfContactBarrierStiffness());

        const std::size_t selfContactRowCapacity
            = contactScratch.selfSurfaceCandidateCapacity;
        vbdScratch.avbdSelfContactDescriptors.reserve(selfContactRowCapacity);
        vbdScratch.avbdSelfContactInventory.reserve(selfContactRowCapacity);
        vbdScratch.avbdSelfContactRows.reserve(selfContactRowCapacity);
        const std::size_t frictionRowCapacity = checkedSurfaceCandidateMultiply(
            2u, selfContactRowCapacity, "self-contact AVBD friction rows");
        vbdScratch.avbdSelfContactFrictionDescriptors.reserve(
            frictionRowCapacity);
        vbdScratch.avbdSelfContactFrictionInventory.reserve(
            frictionRowCapacity);
        vbdScratch.avbdSelfContactFrictionAnchors.reserve(
            selfContactRowCapacity);
        vbdScratch.avbdSelfContactFrictionRows.reserve(frictionRowCapacity);
        vbdScratch.previousAvbdSelfContactFrictionWarmStarts.reserve(
            frictionRowCapacity);
      }
    }
  }
}

//==============================================================================
void DeformableDynamicsStage::execute(World& world, ComputeExecutor& executor)
{
  m_lastStats.reset();

  if (m_scratch == nullptr) {
    m_scratch = std::unique_ptr<Scratch, ScratchDeleter>(
        m_memoryManager != nullptr
            ? stage_detail::constructStageOwnedScratch<Scratch>(
                  m_memoryManager, m_memoryManager->getFreeAllocator())
            : stage_detail::constructStageOwnedScratch<Scratch>(nullptr),
        ScratchDeleter{m_memoryManager});
  }

  auto& registry = dart::simulation::detail::registryOf(world);
  auto& worldFreeAllocator = world.getMemoryManager().getFreeAllocator();
  auto view = registry.view<
      comps::DeformableBodyTag,
      comps::DeformableNodeState,
      comps::DeformableSpringModel,
      comps::DeformableMeshTopology,
      comps::DeformableMaterial>();
  if (view.begin() == view.end()) {
    return;
  }

  auto& stageScratch = *m_scratch;
  const auto timeStep = world.getTimeStep();
  if (everyExecutableDeformableBodyIgnoresRigidObstacles(registry)) {
    clearRigidObstacleScratch(stageScratch);
  } else {
    collectStaticGroundBarriersInto(world, stageScratch.barriers);
    collectSphereObstacleBarriersInto(world, stageScratch.sphereObstacles);
    collectBoxObstacleBarriersInto(world, stageScratch.boxObstacles);
    collectCapsuleObstacleBarriersInto(world, stageScratch.capsuleObstacles);
    collectStaticRigidSurfaceCcdObstaclesInto(
        world,
        m_lastStats,
        stageScratch.rigidSurfaceSnapshots,
        stageScratch.rigidSurfaceSnapshotCount,
        stageScratch.payloadAllocator);
    collectMovingRigidSurfaceCcdObstaclesInto(
        world,
        timeStep,
        m_lastStats,
        stageScratch.movingRigidSurfaceSnapshots,
        stageScratch.movingRigidSurfaceSnapshotCount,
        false,
        stageScratch.payloadAllocator);
  }
  const auto gravity = world.getGravity();
  const auto& barriers = stageScratch.barriers;
  const auto& sphereObstacles = stageScratch.sphereObstacles;
  const auto& boxObstacles = stageScratch.boxObstacles;
  const auto& capsuleObstacles = stageScratch.capsuleObstacles;
  const auto rigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      stageScratch.rigidSurfaceSnapshots,
      stageScratch.rigidSurfaceSnapshotCount);
  const auto movingRigidSurfaceSnapshots = activeSurfaceContactSnapshots(
      stageScratch.movingRigidSurfaceSnapshots,
      stageScratch.movingRigidSurfaceSnapshotCount);
  m_lastStats.staticGroundBarrierCount = barriers.size();

  for (const auto entity : view) {
    auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& nodeModel = registry.get<comps::DeformableNodeModel>(entity);
    auto& scratch = registry.get_or_emplace<comps::DeformableSolverScratch>(
        entity, worldFreeAllocator);
    const auto* boundaryConditions
        = registry.try_get<comps::DeformableBoundaryConditions>(entity);
    ++m_lastStats.bodyCount;
    prepareDeformableBoundaryConditions(
        state,
        nodeModel,
        boundaryConditions,
        world.getTime(),
        timeStep,
        scratch,
        m_lastStats);
  }

  collectDeformableSurfaceSnapshotsInto(
      registry,
      stageScratch.surfaceSnapshots,
      stageScratch.surfaceSnapshotCount,
      stageScratch.payloadAllocator,
      true);
  const auto surfaceSnapshots = activeSurfaceContactSnapshots(
      stageScratch.surfaceSnapshots, stageScratch.surfaceSnapshotCount);

  for (const auto entity : view) {
    auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& nodeModel = registry.get<comps::DeformableNodeModel>(entity);
    const auto& model = view.get<comps::DeformableSpringModel>(entity);
    const auto& topology = view.get<comps::DeformableMeshTopology>(entity);
    const auto& material = view.get<comps::DeformableMaterial>(entity);
    auto& scratch = registry.get_or_emplace<comps::DeformableSolverScratch>(
        entity, worldFreeAllocator);
    auto& contactScratch
        = registry.get_or_emplace<DeformableContactSolverScratch>(
            entity, worldFreeAllocator);
    auto& vbdScratch = registry.get_or_emplace<DeformableVbdScratch>(
        entity, worldFreeAllocator);
    const auto* vbdConfig
        = registry.try_get<comps::DeformableVbdConfig>(entity);
    advanceDeformableBody(
        entity,
        state,
        nodeModel,
        model,
        topology,
        scratch,
        contactScratch,
        vbdScratch,
        vbdConfig,
        surfaceSnapshots,
        rigidSurfaceSnapshots,
        movingRigidSurfaceSnapshots,
        gravity,
        timeStep,
        barriers,
        sphereObstacles,
        boxObstacles,
        capsuleObstacles,
        material,
        executor,
        m_lastStats);
  }

  // Transaction commit: every body has now solved successfully into retained
  // scratch. This loop performs only fixed-size element assignments, so a cap
  // overflow or invalid body encountered above cannot leave an earlier
  // deformable body partially advanced.
  for (const auto entity : view) {
    auto& state = view.get<comps::DeformableNodeState>(entity);
    const auto& scratch = registry.get<comps::DeformableSolverScratch>(entity);
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
      const Eigen::Vector3d previous = scratch.previousStepPositions[i];
      state.previousPositions[i] = previous;
      state.positions[i] = scratch.next[i];
      state.velocities[i] = (state.positions[i] - previous) / timeStep;
      state.attachmentTargets[i] = scratch.stepAttachmentTargets[i];
    }
  }
}

//==============================================================================
const DeformableSolverStats& DeformableDynamicsStage::getLastStats()
    const noexcept
{
  return m_lastStats;
}

//==============================================================================
void reserveDeformableDynamicsRegistryStorage(
    detail::WorldRegistry& registry,
    std::size_t deformableBodyCount,
    common::MemoryAllocator& allocator)
{
  auto& contactScratchStorage
      = registry.storage<DeformableContactSolverScratch>();
  auto& vbdScratchStorage = registry.storage<DeformableVbdScratch>();
  contactScratchStorage.reserve(deformableBodyCount);
  vbdScratchStorage.reserve(deformableBodyCount);

  if (deformableBodyCount == 0u) {
    return;
  }

  auto view = registry.view<comps::DeformableBodyTag>();
  for (auto entity : view) {
    if (!registry.all_of<DeformableContactSolverScratch>(entity)) {
      registry.emplace<DeformableContactSolverScratch>(entity, allocator);
      registry.remove<DeformableContactSolverScratch>(entity);
    }
    if (!registry.all_of<DeformableVbdScratch>(entity)) {
      registry.emplace<DeformableVbdScratch>(entity, allocator);
      registry.remove<DeformableVbdScratch>(entity);
    }
  }
}

} // namespace dart::simulation::compute
