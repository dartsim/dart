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

#include <dart/simulation/detail/deformable_contact/candidate_set.hpp>
#include <dart/simulation/detail/newton_barrier/articulation_constraint.hpp>
#include <dart/simulation/detail/newton_barrier/change_of_variable.hpp>
#include <dart/simulation/detail/newton_barrier/friction_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/projected_newton.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>
#include <dart/simulation/detail/newton_barrier/tangent_stencil.hpp>
#include <dart/simulation/detail/rigid_ipc_barrier.hpp>

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include <cassert>
#include <cmath>

namespace dart::simulation::detail {

namespace {

constexpr double kRotationDerivativeStep = 1e-6;
constexpr double kRotationHessianStep = 1e-4;

using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
namespace dcd = deformable_contact::detail;

struct SurfaceEdge
{
  std::size_t vertexA = 0;
  std::size_t vertexB = 0;

  [[nodiscard]] friend bool operator==(
      const SurfaceEdge& lhs, const SurfaceEdge& rhs) = default;

  [[nodiscard]] friend bool operator<(
      const SurfaceEdge& lhs, const SurfaceEdge& rhs)
  {
    return std::tie(lhs.vertexA, lhs.vertexB)
           < std::tie(rhs.vertexA, rhs.vertexB);
  }
};

struct PointTransformDerivatives
{
  Matrix3x6d jacobian = Matrix3x6d::Zero();
  std::array<Matrix6d, 3> hessians{
      Matrix6d::Zero(), Matrix6d::Zero(), Matrix6d::Zero()};
};

RigidIpcVector6d poseToVector(const RigidIpcPose& pose)
{
  RigidIpcVector6d vector;
  vector.head<3>() = pose.position;
  vector.tail<3>() = pose.rotation;
  return vector;
}

Eigen::Vector3d transformWithRotation(
    const Eigen::Vector3d& localPoint, const Eigen::Vector3d& rotation)
{
  return rigidIpcRotationVectorToMatrix(rotation) * localPoint;
}

SurfaceEdge makeSurfaceEdge(std::size_t a, std::size_t b)
{
  if (b < a) {
    std::swap(a, b);
  }
  return SurfaceEdge{a, b};
}

std::size_t checkedTriangleVertex(
    const Eigen::Vector3i& triangle,
    const int index,
    const std::size_t vertexCount)
{
  assert(index >= 0 && index < 3);
  const int vertex = triangle[index];
  static_cast<void>(vertexCount);
  assert(vertex >= 0);
  assert(static_cast<std::size_t>(vertex) < vertexCount);
  return static_cast<std::size_t>(vertex);
}

template <typename EdgeVector>
void buildSurfaceEdges(const RigidIpcBarrierSurface& surface, EdgeVector& edges)
{
  edges.clear();
  edges.reserve(3 * surface.triangles.size());
  for (const Eigen::Vector3i& triangle : surface.triangles) {
    const std::size_t a
        = checkedTriangleVertex(triangle, 0, surface.vertices.size());
    const std::size_t b
        = checkedTriangleVertex(triangle, 1, surface.vertices.size());
    const std::size_t c
        = checkedTriangleVertex(triangle, 2, surface.vertices.size());
    edges.push_back(makeSurfaceEdge(a, b));
    edges.push_back(makeSurfaceEdge(b, c));
    edges.push_back(makeSurfaceEdge(c, a));
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
}

/// World-space axis-aligned bounding box of a surface's transformed vertices.
struct SurfaceWorldAabb
{
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  bool valid = false;
};

dart::common::MemoryAllocator& allocatorOrDefault(
    dart::common::MemoryAllocator* allocator)
{
  return allocator ? *allocator : dart::common::MemoryAllocator::GetDefault();
}

template <typename SurfaceVector, typename SourceSurfaces>
void assignSurfacesPreservingStorage(
    SurfaceVector& target,
    const SourceSurfaces& source,
    dart::common::MemoryAllocator* allocator)
{
  const std::size_t size = source.size();
  if (target.size() > size) {
    target.resize(size);
  }
  while (target.size() < size) {
    if (allocator != nullptr) {
      target.emplace_back(*allocator);
    } else {
      target.emplace_back();
    }
  }
  for (std::size_t i = 0; i < size; ++i) {
    target[i] = source[i];
  }
}

template <typename TripletVector>
bool tryAssembleSparseFromExistingPattern(
    Eigen::SparseMatrix<double>& matrix,
    Eigen::Index rows,
    Eigen::Index cols,
    const TripletVector& triplets)
{
  if (matrix.rows() != rows || matrix.cols() != cols
      || !matrix.isCompressed()) {
    return false;
  }

  using SparseMatrix = Eigen::SparseMatrix<double>;
  using StorageIndex = SparseMatrix::StorageIndex;
  const StorageIndex* outer = matrix.outerIndexPtr();
  const StorageIndex* inner = matrix.innerIndexPtr();
  double* values = matrix.valuePtr();
  if (outer == nullptr || inner == nullptr || values == nullptr) {
    return triplets.empty() && matrix.nonZeros() == 0;
  }

  std::fill(values, values + matrix.nonZeros(), 0.0);
  for (const auto& triplet : triplets) {
    const Eigen::Index row = triplet.row();
    const Eigen::Index col = triplet.col();
    if (row < 0 || row >= rows || col < 0 || col >= cols) {
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

template <typename TripletVector>
void assembleSparseFromTripletsReusingPattern(
    Eigen::SparseMatrix<double>& matrix,
    Eigen::Index rows,
    Eigen::Index cols,
    const TripletVector& triplets)
{
  if (tryAssembleSparseFromExistingPattern(matrix, rows, cols, triplets)) {
    return;
  }

  matrix.resize(rows, cols);
  matrix.setZero();
  matrix.reserve(static_cast<Eigen::Index>(triplets.size()));
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
}

using SurfaceEdgeAllocator = dart::common::StlAllocator<SurfaceEdge>;
using SurfaceEdgeVector = std::vector<SurfaceEdge, SurfaceEdgeAllocator>;
using SurfaceEdgeVectorAllocator
    = dart::common::StlAllocator<SurfaceEdgeVector>;
using SurfaceWorldAabbAllocator = dart::common::StlAllocator<SurfaceWorldAabb>;
using SurfaceMarginAllocator = dart::common::StlAllocator<double>;
using SweepItemAllocator = dart::common::StlAllocator<dcd::SweepItem>;
using CandidatePair = std::pair<std::size_t, std::size_t>;
using CandidatePairAllocator = dart::common::StlAllocator<CandidatePair>;
using BarrierTriplet = Eigen::Triplet<double>;
using BarrierTripletAllocator = dart::common::StlAllocator<BarrierTriplet>;

void reserveDenseSparsePattern(
    Eigen::SparseMatrix<double>& matrix,
    const Eigen::Index rows,
    const Eigen::Index cols,
    std::vector<BarrierTriplet, BarrierTripletAllocator>& triplets)
{
  matrix.resize(rows, cols);
  if (rows <= 0 || cols <= 0) {
    matrix.setZero();
    matrix.makeCompressed();
    return;
  }

  const auto entryCount
      = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  triplets.clear();
  triplets.reserve(entryCount);
  for (Eigen::Index col = 0; col < cols; ++col) {
    for (Eigen::Index row = 0; row < rows; ++row) {
      triplets.emplace_back(row, col, 1.0);
    }
  }

  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
  std::fill(matrix.valuePtr(), matrix.valuePtr() + matrix.nonZeros(), 0.0);
}

void reserveRigidIpcAssemblySparsePatterns(
    RigidIpcBarrierAssembly& assembly,
    const Eigen::Index dofs,
    const Eigen::Index equalityRows,
    std::vector<BarrierTriplet, BarrierTripletAllocator>& triplets)
{
  if (assembly.gradient.size() != dofs) {
    assembly.gradient.resize(dofs);
  }
  reserveDenseSparsePattern(assembly.hessian, dofs, dofs, triplets);

  if (assembly.equalityResidual.size() != equalityRows) {
    assembly.equalityResidual.resize(equalityRows);
  }
  reserveDenseSparsePattern(
      assembly.equalityJacobian, equalityRows, dofs, triplets);
}

struct RigidIpcSurfacePairScratch
{
  static constexpr std::size_t kSmallSurfaceEdgeCapacity = 8u;

  explicit RigidIpcSurfacePairScratch(
      dart::common::MemoryAllocator* allocator = nullptr)
    : memoryAllocator(allocator),
      surfaceEdges(SurfaceEdgeVectorAllocator{allocatorOrDefault(allocator)}),
      surfaceAabbs(SurfaceWorldAabbAllocator{allocatorOrDefault(allocator)}),
      surfaceMargins(SurfaceMarginAllocator{allocatorOrDefault(allocator)}),
      motionBounds(SurfaceMarginAllocator{allocatorOrDefault(allocator)}),
      sweepItems(SweepItemAllocator{allocatorOrDefault(allocator)}),
      candidatePairs(CandidatePairAllocator{allocatorOrDefault(allocator)})
  {
  }

  void resizeSurfaceEdges(std::size_t size)
  {
    if (size <= kSmallSurfaceEdgeCapacity) {
      useSmallSurfaceEdges = true;
      surfaceEdgeCount = size;
      for (std::size_t i = 0; i < size; ++i) {
        if (!smallSurfaceEdges[i].has_value()) {
          if (memoryAllocator != nullptr) {
            smallSurfaceEdges[i].emplace(
                SurfaceEdgeAllocator{*memoryAllocator});
          } else {
            smallSurfaceEdges[i].emplace();
          }
        }
      }
      return;
    }

    useSmallSurfaceEdges = false;
    surfaceEdgeCount = size;
    if (surfaceEdges.size() > size) {
      surfaceEdges.resize(size);
    }
    if (surfaceEdges.size() >= size) {
      return;
    }

    surfaceEdges.reserve(size);
    while (surfaceEdges.size() < size) {
      if (memoryAllocator != nullptr) {
        surfaceEdges.emplace_back(SurfaceEdgeAllocator{*memoryAllocator});
      } else {
        surfaceEdges.emplace_back();
      }
    }
  }

  SurfaceEdgeVector& surfaceEdgesAt(std::size_t index)
  {
    assert(index < surfaceEdgeCount);
    if (useSmallSurfaceEdges) {
      assert(index < kSmallSurfaceEdgeCapacity);
      assert(smallSurfaceEdges[index].has_value());
      return *smallSurfaceEdges[index];
    }
    return surfaceEdges[index];
  }

  dart::common::MemoryAllocator* memoryAllocator = nullptr;
  bool useSmallSurfaceEdges = true;
  std::size_t surfaceEdgeCount = 0u;
  std::array<std::optional<SurfaceEdgeVector>, kSmallSurfaceEdgeCapacity>
      smallSurfaceEdges;
  std::vector<SurfaceEdgeVector, SurfaceEdgeVectorAllocator> surfaceEdges;
  std::vector<SurfaceWorldAabb, SurfaceWorldAabbAllocator> surfaceAabbs;
  std::vector<double, SurfaceMarginAllocator> surfaceMargins;
  std::vector<double, SurfaceMarginAllocator> motionBounds;
  std::vector<dcd::SweepItem, SweepItemAllocator> sweepItems;
  std::vector<CandidatePair, CandidatePairAllocator> candidatePairs;
};

struct RigidIpcAssemblyScratch
{
  explicit RigidIpcAssemblyScratch(
      dart::common::MemoryAllocator* allocator = nullptr)
    : pairs(allocator),
      barrierTriplets(BarrierTripletAllocator{allocatorOrDefault(allocator)}),
      articulationResiduals(
          SurfaceMarginAllocator{allocatorOrDefault(allocator)}),
      articulationJacobianTriplets(
          BarrierTripletAllocator{allocatorOrDefault(allocator)}),
      energyAssembly(allocatorOrDefault(allocator)),
      unitBarrierAssembly(allocatorOrDefault(allocator)),
      laggedAssembly(allocatorOrDefault(allocator)),
      candidateAssembly(allocatorOrDefault(allocator))
  {
  }

  RigidIpcSurfacePairScratch pairs;
  std::vector<BarrierTriplet, BarrierTripletAllocator> barrierTriplets;
  std::vector<double, SurfaceMarginAllocator> articulationResiduals;
  std::vector<BarrierTriplet, BarrierTripletAllocator>
      articulationJacobianTriplets;
  RigidIpcBarrierAssembly energyAssembly;
  RigidIpcBarrierAssembly unitBarrierAssembly;
  RigidIpcBarrierAssembly laggedAssembly;
  RigidIpcBarrierAssembly candidateAssembly;
  Eigen::VectorXd gradBarrier;
};

struct RigidIpcLineSearchScratch
{
  explicit RigidIpcLineSearchScratch(
      dart::common::MemoryAllocator* allocator = nullptr)
    : pairs(allocator)
  {
  }

  RigidIpcSurfacePairScratch pairs;
};

} // namespace

struct RigidIpcProjectedNewtonSolveScratchWorkspace
{
  explicit RigidIpcProjectedNewtonSolveScratchWorkspace(
      dart::common::MemoryAllocator* allocator = nullptr)
    : assemblyScratch(allocator), lineSearchScratch(allocator)
  {
  }

  RigidIpcAssemblyScratch assemblyScratch;
  RigidIpcLineSearchScratch lineSearchScratch;
  Eigen::MatrixXd denseHessian;
  Eigen::VectorXd rawStep;
  Eigen::LDLT<Eigen::MatrixXd> denseHessianLdlt;
  Eigen::MatrixXd equalityDenseJacobian;
  Eigen::MatrixXd equalityKktMatrix;
  Eigen::VectorXd equalityKktRhs;
  Eigen::VectorXd equalityKktSolution;
};

namespace {

SurfaceWorldAabb computeSurfaceWorldAabb(const RigidIpcBarrierSurface& surface)
{
  SurfaceWorldAabb aabb;
  if (surface.vertices.empty()) {
    return aabb;
  }
  Eigen::Vector3d lo
      = transformRigidIpcPoint(surface.vertices[0], surface.pose);
  Eigen::Vector3d hi = lo;
  for (std::size_t i = 1; i < surface.vertices.size(); ++i) {
    const Eigen::Vector3d p
        = transformRigidIpcPoint(surface.vertices[i], surface.pose);
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  aabb.min = lo;
  aabb.max = hi;
  aabb.valid = true;
  return aabb;
}

/// Lower bound on the squared distance between two surfaces.
///
/// Each surface lies inside its world AABB, so the AABB-AABB gap never exceeds
/// the true minimum surface distance. A pair whose value reaches the squared
/// activation distance therefore cannot host an active barrier (the kernel is
/// inactive once `squaredDistance >= squaredActivationDistance`) and can be
/// skipped without changing the assembled system.
double surfaceAabbSquaredDistance(
    const SurfaceWorldAabb& a, const SurfaceWorldAabb& b)
{
  const Eigen::Vector3d gap
      = (a.min - b.max).cwiseMax(b.min - a.max).cwiseMax(0.0);
  return gap.squaredNorm();
}

/// Conservative bound on how far any of a surface's points can move over the
/// step, reusing the curved-CCD trajectory speed bound. Every world point of
/// the surface stays within this distance of its start position for all t in
/// [0, 1], so a pair whose start AABBs are farther apart than the sum of their
/// motion bounds (plus the minimum separation) cannot reach contact during the
/// step.
double surfaceMotionBound(
    const RigidIpcBarrierSurface& start, const RigidIpcBarrierSurface& end)
{
  double bound = 0.0;
  for (const Eigen::Vector3d& localVertex : start.vertices) {
    bound = std::max(
        bound,
        rigidIpcPointTrajectorySpeedBound(localVertex, start.pose, end.pose));
  }
  return bound;
}

/// Enumerate candidate surface pairs with a sort-and-sweep broad phase, reusing
/// the deformable IPC sweep utilities (PLAN-082 Workstream 8: shared IPC
/// primitives). Each surface's world AABB is expanded by a per-surface margin;
/// the sweep returns every pair whose expanded AABBs overlap, which is a
/// superset of the pairs the exact distance/reach cull keeps, so callers must
/// still apply that exact cull. This replaces the all-pairs O(N^2) enumeration
/// with O(N log N + overlapping pairs) for large scenes. Surfaces with empty
/// AABBs carry no primitives and are omitted (they contribute no constraints).
template <typename SweepItemVector, typename CandidatePairVector>
void collectCandidateSurfacePairs(
    std::span<const SurfaceWorldAabb> aabbs,
    std::span<const double> margins,
    SweepItemVector& items,
    CandidatePairVector& pairs)
{
  items.clear();
  items.reserve(aabbs.size());
  for (std::size_t body = 0; body < aabbs.size(); ++body) {
    if (!aabbs[body].valid) {
      continue;
    }
    dcd::CandidateAabb aabb;
    aabb.min = aabbs[body].min;
    aabb.max = aabbs[body].max;
    aabb.expand(margins[body]);
    items.push_back(dcd::SweepItem{body, aabb});
  }

  pairs.clear();
  dcd::visitSelfSweepPairs(
      items, [&](std::size_t a, std::size_t b) { pairs.emplace_back(a, b); });
}

PointTransformDerivatives pointTransformDerivatives(
    const Eigen::Vector3d& localPoint, const RigidIpcPose& pose)
{
  PointTransformDerivatives derivatives;
  derivatives.jacobian.leftCols<3>() = Eigen::Matrix3d::Identity();

  const Eigen::Vector3d rotation = pose.rotation;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = rotation;
    Eigen::Vector3d minus = rotation;
    plus[axis] += kRotationDerivativeStep;
    minus[axis] -= kRotationDerivativeStep;
    derivatives.jacobian.col(3 + axis)
        = (transformWithRotation(localPoint, plus)
           - transformWithRotation(localPoint, minus))
          / (2.0 * kRotationDerivativeStep);
  }

  const Eigen::Vector3d center = transformWithRotation(localPoint, rotation);
  for (int row = 0; row < 3; ++row) {
    for (int col = row; col < 3; ++col) {
      Eigen::Vector3d rowPlus = rotation;
      Eigen::Vector3d rowMinus = rotation;
      rowPlus[row] += kRotationHessianStep;
      rowMinus[row] -= kRotationHessianStep;

      Eigen::Vector3d secondDerivative;
      if (row == col) {
        secondDerivative
            = (transformWithRotation(localPoint, rowPlus) - 2.0 * center
               + transformWithRotation(localPoint, rowMinus))
              / (kRotationHessianStep * kRotationHessianStep);
      } else {
        Eigen::Vector3d pp = rotation;
        Eigen::Vector3d pm = rotation;
        Eigen::Vector3d mp = rotation;
        Eigen::Vector3d mm = rotation;
        pp[row] += kRotationHessianStep;
        pp[col] += kRotationHessianStep;
        pm[row] += kRotationHessianStep;
        pm[col] -= kRotationHessianStep;
        mp[row] -= kRotationHessianStep;
        mp[col] += kRotationHessianStep;
        mm[row] -= kRotationHessianStep;
        mm[col] -= kRotationHessianStep;
        secondDerivative
            = (transformWithRotation(localPoint, pp)
               - transformWithRotation(localPoint, pm)
               - transformWithRotation(localPoint, mp)
               + transformWithRotation(localPoint, mm))
              / (4.0 * kRotationHessianStep * kRotationHessianStep);
      }

      for (int coord = 0; coord < 3; ++coord) {
        derivatives.hessians[coord](3 + row, 3 + col) = secondDerivative[coord];
        derivatives.hessians[coord](3 + col, 3 + row) = secondDerivative[coord];
      }
    }
  }

  return derivatives;
}

Matrix3x6d directionTransformJacobian(
    const Eigen::Vector3d& localDirection, const RigidIpcPose& pose)
{
  Matrix3x6d jacobian = Matrix3x6d::Zero();
  const Eigen::Vector3d rotation = pose.rotation;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = rotation;
    Eigen::Vector3d minus = rotation;
    plus[axis] += kRotationDerivativeStep;
    minus[axis] -= kRotationDerivativeStep;
    jacobian.col(3 + axis) = (transformWithRotation(localDirection, plus)
                              - transformWithRotation(localDirection, minus))
                             / (2.0 * kRotationDerivativeStep);
  }
  return jacobian;
}

Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d& vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return matrix;
}

template <int Columns>
RigidIpcFrictionPotentialResult computeProjectedFrictionPotential(
    const Eigen::Matrix<double, 2, Columns>& projection,
    const Eigen::Matrix<double, Columns, 1>& displacement,
    const RigidIpcFrictionOptions& options)
{
  RigidIpcFrictionPotentialResult result;
  const double weight = options.coefficient * options.laggedNormalForce;
  const auto potential = newton_barrier::projectedFrictionPotential<Columns>(
      projection, displacement, weight, options.staticFrictionDisplacement);
  if (!potential.active) {
    return result;
  }

  result.value = potential.value;
  result.work = potential.work;
  result.gradient.template head<Columns>() = potential.gradient;
  result.hessian.template topLeftCorner<Columns, Columns>() = potential.hessian;
  result.tangentialDisplacement = potential.tangentialDisplacement;
  result.tangentialDisplacementNorm = potential.tangentialDisplacementNorm;
  result.weight = potential.weight;
  result.active = true;
  result.dynamicBranch = potential.dynamicBranch;
  result.hessian
      = 0.5 * (result.hessian + RigidIpcMatrix12d(result.hessian.transpose()));
  return result;
}

RigidIpcReducedFrictionResult chainFrictionPotentialToReducedCoordinates(
    const RigidIpcFrictionPotentialResult& potential,
    const std::array<PointTransformDerivatives, 4>& derivatives,
    const std::array<int, 4>& bodyIds,
    const bool projectHessianToPsd)
{
  RigidIpcReducedFrictionResult result;
  result.value = potential.value;
  result.potential = potential;
  result.active = potential.active;

  if (!potential.active) {
    return result;
  }

  RigidIpcMatrix12d jacobian = RigidIpcMatrix12d::Zero();
  for (int vertex = 0; vertex < 4; ++vertex) {
    jacobian.block<3, 6>(3 * vertex, 6 * bodyIds[vertex])
        = derivatives[vertex].jacobian;
  }

  result.gradient = jacobian.transpose() * potential.gradient;
  result.hessian = jacobian.transpose() * potential.hessian * jacobian;

  for (int vertex = 0; vertex < 4; ++vertex) {
    const int bodyOffset = 6 * bodyIds[vertex];
    for (int coord = 0; coord < 3; ++coord) {
      result.hessian.block<6, 6>(bodyOffset, bodyOffset)
          += potential.gradient[3 * vertex + coord]
             * derivatives[vertex].hessians[coord];
    }
  }

  result.hessian
      = 0.5 * (result.hessian + RigidIpcMatrix12d(result.hessian.transpose()));
  if (projectHessianToPsd) {
    result.hessian
        = newton_barrier::projectSymmetricMatrixToPsd<12>(result.hessian);
  }
  return result;
}

RigidIpcReducedBarrierResult chainPrimitiveBarrierToReducedCoordinates(
    const RigidIpcPrimitiveBarrierResult& primitive,
    const std::array<PointTransformDerivatives, 4>& derivatives,
    const std::array<int, 4>& bodyIds,
    const RigidIpcBarrierOptions& options)
{
  RigidIpcReducedBarrierResult result;
  result.value = primitive.value;
  result.primitive = primitive;
  result.active = primitive.active;

  if (!primitive.active) {
    return result;
  }

  RigidIpcMatrix12d jacobian = RigidIpcMatrix12d::Zero();
  for (int vertex = 0; vertex < 4; ++vertex) {
    jacobian.block<3, 6>(3 * vertex, 6 * bodyIds[vertex])
        = derivatives[vertex].jacobian;
  }

  result.gradient = jacobian.transpose() * primitive.gradient;
  result.hessian = jacobian.transpose() * primitive.hessian * jacobian;

  for (int vertex = 0; vertex < 4; ++vertex) {
    const int bodyOffset = 6 * bodyIds[vertex];
    for (int coord = 0; coord < 3; ++coord) {
      result.hessian.block<6, 6>(bodyOffset, bodyOffset)
          += primitive.gradient[3 * vertex + coord]
             * derivatives[vertex].hessians[coord];
    }
  }

  result.hessian
      = 0.5 * (result.hessian + RigidIpcMatrix12d(result.hessian.transpose()));
  if (options.projectReducedHessianToPsd) {
    result.hessian
        = newton_barrier::projectSymmetricMatrixToPsd<12>(result.hessian);
  }
  return result;
}

void initializeGlobalAssembly(
    std::span<const RigidIpcBarrierSurface> surfaces,
    RigidIpcBarrierAssembly& assembly,
    bool preserveEqualityStorage = false)
{
  assembly.value = 0.0;
  assembly.activeConstraints.clear();
  assembly.activeFrictionConstraints.clear();
  assembly.activeArticulationConstraints.clear();
  assembly.activeDynamicsTerms = 0u;

  std::size_t dofCount = 0;
  assembly.bodyDofOffsets.assign(
      surfaces.size(), RigidIpcBarrierAssembly::npos);
  for (std::size_t body = 0; body < surfaces.size(); ++body) {
    if (!surfaces[body].dynamic) {
      continue;
    }
    assembly.bodyDofOffsets[body] = dofCount;
    dofCount += 6;
  }

  assembly.gradient
      = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dofCount));
  const auto dofs = static_cast<Eigen::Index>(dofCount);
  if (assembly.hessian.rows() != dofs || assembly.hessian.cols() != dofs) {
    assembly.hessian.resize(dofs, dofs);
  }
  if (!preserveEqualityStorage) {
    assembly.equalityResidual.resize(0);
  }
  if (assembly.equalityJacobian.cols() != dofs) {
    assembly.equalityJacobian.resize(0, dofs);
  }
}

template <typename TripletVector>
void scatterReducedBarrier(
    RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const std::size_t bodyA,
    const std::size_t bodyB,
    const RigidIpcReducedBarrierResult& reduced)
{
  if (!reduced.active) {
    return;
  }

  assembly.value += reduced.value;

  const std::array<std::size_t, 2> offsets{
      assembly.bodyDofOffsets[bodyA], assembly.bodyDofOffsets[bodyB]};
  for (int localBody = 0; localBody < 2; ++localBody) {
    const std::size_t offset = offsets[localBody];
    if (offset == RigidIpcBarrierAssembly::npos) {
      continue;
    }
    assembly.gradient.segment<6>(static_cast<Eigen::Index>(offset))
        += reduced.gradient.segment<6>(6 * localBody);
  }

  constexpr double kEntryTolerance = 1e-15;
  for (int localRowBody = 0; localRowBody < 2; ++localRowBody) {
    const std::size_t rowOffset = offsets[localRowBody];
    if (rowOffset == RigidIpcBarrierAssembly::npos) {
      continue;
    }
    for (int localColBody = 0; localColBody < 2; ++localColBody) {
      const std::size_t colOffset = offsets[localColBody];
      if (colOffset == RigidIpcBarrierAssembly::npos) {
        continue;
      }
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          const double value
              = reduced.hessian(6 * localRowBody + row, 6 * localColBody + col);
          if (std::abs(value) <= kEntryTolerance) {
            continue;
          }
          triplets.emplace_back(
              static_cast<Eigen::Index>(rowOffset + row),
              static_cast<Eigen::Index>(colOffset + col),
              value);
        }
      }
    }
  }
}

template <typename TripletVector>
void scatterReducedFriction(
    RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const std::size_t bodyA,
    const std::size_t bodyB,
    const RigidIpcReducedFrictionResult& reduced)
{
  if (!reduced.active) {
    return;
  }

  assembly.value += reduced.value;

  const std::array<std::size_t, 2> offsets{
      assembly.bodyDofOffsets[bodyA], assembly.bodyDofOffsets[bodyB]};
  for (int localBody = 0; localBody < 2; ++localBody) {
    const std::size_t offset = offsets[localBody];
    if (offset == RigidIpcBarrierAssembly::npos) {
      continue;
    }
    assembly.gradient.segment<6>(static_cast<Eigen::Index>(offset))
        += reduced.gradient.segment<6>(6 * localBody);
  }

  constexpr double kEntryTolerance = 1e-15;
  for (int localRowBody = 0; localRowBody < 2; ++localRowBody) {
    const std::size_t rowOffset = offsets[localRowBody];
    if (rowOffset == RigidIpcBarrierAssembly::npos) {
      continue;
    }
    for (int localColBody = 0; localColBody < 2; ++localColBody) {
      const std::size_t colOffset = offsets[localColBody];
      if (colOffset == RigidIpcBarrierAssembly::npos) {
        continue;
      }
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          const double value
              = reduced.hessian(6 * localRowBody + row, 6 * localColBody + col);
          if (std::abs(value) <= kEntryTolerance) {
            continue;
          }
          triplets.emplace_back(
              static_cast<Eigen::Index>(rowOffset + row),
              static_cast<Eigen::Index>(colOffset + col),
              value);
        }
      }
    }
  }
}

template <typename TripletVector>
void addActiveConstraint(
    RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const RigidIpcBarrierPrimitive primitive,
    const std::size_t bodyA,
    const std::size_t bodyB,
    const std::array<std::size_t, 4>& vertices,
    const RigidIpcReducedBarrierResult& reduced)
{
  if (!reduced.active) {
    return;
  }

  scatterReducedBarrier(assembly, triplets, bodyA, bodyB, reduced);
  assembly.activeConstraints.push_back(
      RigidIpcBarrierConstraint{primitive, bodyA, bodyB, vertices, reduced});
}

template <typename TripletVector>
void addActiveFrictionConstraint(
    RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const RigidIpcBarrierPrimitive primitive,
    const std::size_t bodyA,
    const std::size_t bodyB,
    const std::array<std::size_t, 4>& vertices,
    const RigidIpcReducedFrictionResult& reduced,
    const double coefficient,
    const double laggedNormalForce)
{
  if (!reduced.active) {
    return;
  }

  scatterReducedFriction(assembly, triplets, bodyA, bodyB, reduced);
  assembly.activeFrictionConstraints.push_back(
      RigidIpcFrictionConstraint{
          primitive,
          bodyA,
          bodyB,
          vertices,
          reduced,
          coefficient,
          laggedNormalForce});
}

double combinedFrictionCoefficient(
    const RigidIpcBarrierSurface& a,
    const RigidIpcBarrierSurface& b,
    const RigidIpcFrictionOptions& options)
{
  if (!(options.coefficient > 0.0) || !std::isfinite(options.coefficient)) {
    return 0.0;
  }
  if (!(a.frictionCoefficient > 0.0) || !std::isfinite(a.frictionCoefficient)
      || !(b.frictionCoefficient > 0.0)
      || !std::isfinite(b.frictionCoefficient)) {
    return 0.0;
  }
  return options.coefficient
         * std::sqrt(a.frictionCoefficient * b.frictionCoefficient);
}

double estimateLaggedNormalForce(
    const RigidIpcPrimitiveBarrierResult& primitive)
{
  if (!primitive.active || !primitive.gradient.allFinite()) {
    return 0.0;
  }

  double force = 0.0;
  for (int vertex = 0; vertex < 4; ++vertex) {
    const double candidate = primitive.gradient.segment<3>(3 * vertex).norm();
    if (std::isfinite(candidate)) {
      force = std::max(force, candidate);
    }
  }
  return force;
}

RigidIpcReducedFrictionResult evaluateFrictionConstraint(
    const std::span<const RigidIpcBarrierSurface> surfaces,
    const std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    const RigidIpcBarrierConstraint& constraint,
    const RigidIpcFrictionOptions& baseOptions,
    const double coefficient,
    const double laggedNormalForce)
{
  RigidIpcFrictionOptions options = baseOptions;
  options.coefficient = coefficient;
  options.laggedNormalForce = laggedNormalForce;

  const auto npos = RigidIpcBarrierAssembly::npos;
  if (constraint.bodyA >= surfaces.size() || constraint.bodyB >= surfaces.size()
      || constraint.bodyA >= laggedSurfaces.size()
      || constraint.bodyB >= laggedSurfaces.size()) {
    return {};
  }

  const RigidIpcBarrierSurface& a = surfaces[constraint.bodyA];
  const RigidIpcBarrierSurface& b = surfaces[constraint.bodyB];
  const RigidIpcBarrierSurface& laggedA = laggedSurfaces[constraint.bodyA];
  const RigidIpcBarrierSurface& laggedB = laggedSurfaces[constraint.bodyB];

  const auto vertex = [](const RigidIpcBarrierSurface& surface,
                         const std::size_t index) -> const Eigen::Vector3d& {
    assert(index < surface.vertices.size());
    return surface.vertices[index];
  };

  switch (constraint.primitive) {
    case RigidIpcBarrierPrimitive::VertexVertex:
      if (constraint.vertices[0] == npos || constraint.vertices[1] == npos) {
        return {};
      }
      return rigidIpcPointPointReducedFrictionPotential(
          vertex(a, constraint.vertices[0]),
          laggedA.pose,
          a.pose,
          vertex(b, constraint.vertices[1]),
          laggedB.pose,
          b.pose,
          options);
    case RigidIpcBarrierPrimitive::EdgeVertex:
      if (constraint.vertices[0] == npos || constraint.vertices[1] == npos
          || constraint.vertices[2] == npos) {
        return {};
      }
      return rigidIpcPointEdgeReducedFrictionPotential(
          vertex(a, constraint.vertices[0]),
          laggedA.pose,
          a.pose,
          vertex(b, constraint.vertices[1]),
          vertex(b, constraint.vertices[2]),
          laggedB.pose,
          b.pose,
          options);
    case RigidIpcBarrierPrimitive::EdgeEdge:
      if (constraint.vertices[0] == npos || constraint.vertices[1] == npos
          || constraint.vertices[2] == npos || constraint.vertices[3] == npos) {
        return {};
      }
      return rigidIpcEdgeEdgeReducedFrictionPotential(
          vertex(a, constraint.vertices[0]),
          vertex(a, constraint.vertices[1]),
          laggedA.pose,
          a.pose,
          vertex(b, constraint.vertices[2]),
          vertex(b, constraint.vertices[3]),
          laggedB.pose,
          b.pose,
          options);
    case RigidIpcBarrierPrimitive::FaceVertex:
      if (constraint.vertices[0] == npos || constraint.vertices[1] == npos
          || constraint.vertices[2] == npos || constraint.vertices[3] == npos) {
        return {};
      }
      return rigidIpcPointTriangleReducedFrictionPotential(
          vertex(a, constraint.vertices[0]),
          laggedA.pose,
          a.pose,
          vertex(b, constraint.vertices[1]),
          vertex(b, constraint.vertices[2]),
          vertex(b, constraint.vertices[3]),
          laggedB.pose,
          b.pose,
          options);
  }

  return {};
}

template <typename TripletVector>
void addLaggedFrictionTerms(
    RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const std::span<const RigidIpcBarrierSurface> surfaces,
    const std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    const RigidIpcFrictionOptions& frictionOptions,
    const RigidIpcBarrierAssembly& laggedAssembly)
{
  if (laggedSurfaces.size() != surfaces.size()
      || !(frictionOptions.coefficient > 0.0)
      || !std::isfinite(frictionOptions.coefficient)
      || !(frictionOptions.staticFrictionDisplacement > 0.0)
      || !std::isfinite(frictionOptions.staticFrictionDisplacement)) {
    return;
  }

  for (const RigidIpcBarrierConstraint& constraint :
       laggedAssembly.activeConstraints) {
    const double coefficient = combinedFrictionCoefficient(
        laggedSurfaces[constraint.bodyA],
        laggedSurfaces[constraint.bodyB],
        frictionOptions);
    const double laggedNormalForce
        = estimateLaggedNormalForce(constraint.reduced.primitive);
    if (!(coefficient > 0.0) || !(laggedNormalForce > 0.0)
        || !std::isfinite(laggedNormalForce)) {
      continue;
    }

    const RigidIpcReducedFrictionResult reduced = evaluateFrictionConstraint(
        surfaces,
        laggedSurfaces,
        constraint,
        frictionOptions,
        coefficient,
        laggedNormalForce);
    addActiveFrictionConstraint(
        assembly,
        triplets,
        constraint.primitive,
        constraint.bodyA,
        constraint.bodyB,
        constraint.vertices,
        reduced,
        coefficient,
        laggedNormalForce);
  }
}

template <typename TripletVector>
void addBodyDynamicsTerm(
    RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const std::size_t body,
    const RigidIpcBarrierSurface& surface,
    const RigidIpcBodyDynamicsTerm& term)
{
  if (!term.active || !surface.dynamic) {
    return;
  }

  const std::size_t offset = assembly.bodyDofOffsets[body];
  if (offset == RigidIpcBarrierAssembly::npos) {
    return;
  }

  const RigidIpcVector6d weights
      = term.diagonalWeights.array().max(0.0).matrix();
  const RigidIpcVector6d q = poseToVector(surface.pose);
  const RigidIpcVector6d target = poseToVector(term.targetPose);
  const RigidIpcVector6d residual = q - target;

  assembly.value += 0.5 * weights.dot(residual.cwiseProduct(residual))
                    - term.generalizedForce.dot(q);
  assembly.gradient.segment<6>(static_cast<Eigen::Index>(offset))
      += weights.cwiseProduct(residual) - term.generalizedForce;

  constexpr double kEntryTolerance = 1e-15;
  for (int dof = 0; dof < 6; ++dof) {
    if (weights[dof] <= kEntryTolerance) {
      continue;
    }
    triplets.emplace_back(
        static_cast<Eigen::Index>(offset + dof),
        static_cast<Eigen::Index>(offset + dof),
        weights[dof]);
  }
  ++assembly.activeDynamicsTerms;
}

template <typename TripletVector>
void scatterArticulationJacobianBlock(
    const RigidIpcBarrierAssembly& assembly,
    TripletVector& triplets,
    const std::size_t rowOffset,
    const std::size_t body,
    const Matrix3x6d& jacobian)
{
  if (body >= assembly.bodyDofOffsets.size()) {
    return;
  }
  const std::size_t colOffset = assembly.bodyDofOffsets[body];
  if (colOffset == RigidIpcBarrierAssembly::npos) {
    return;
  }

  constexpr double kEntryTolerance = 1e-15;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 6; ++col) {
      const double value = jacobian(row, col);
      if (std::abs(value) <= kEntryTolerance) {
        continue;
      }
      triplets.emplace_back(
          static_cast<Eigen::Index>(rowOffset + row),
          static_cast<Eigen::Index>(colOffset + col),
          value);
    }
  }
}

template <typename ResidualVector, typename TripletVector>
void addArticulationConstraintRows(
    RigidIpcBarrierAssembly& assembly,
    ResidualVector& residuals,
    TripletVector& jacobianTriplets,
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcArticulationConstraintInput& input)
{
  if (!input.active || input.bodyA >= surfaces.size()
      || input.bodyB >= surfaces.size()
      || (!surfaces[input.bodyA].dynamic && !surfaces[input.bodyB].dynamic)
      || !input.localPointA.allFinite() || !input.localPointB.allFinite()
      || !input.localAxisA.allFinite() || !input.localAxisB.allFinite()) {
    return;
  }

  const RigidIpcBarrierSurface& surfaceA = surfaces[input.bodyA];
  const RigidIpcBarrierSurface& surfaceB = surfaces[input.bodyB];
  const std::size_t rowOffset = residuals.size();

  Matrix3x6d jacobianA = Matrix3x6d::Zero();
  Matrix3x6d jacobianB = Matrix3x6d::Zero();
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  switch (input.type) {
    case RigidIpcArticulationConstraintType::PointConnection: {
      const Eigen::Vector3d pointA
          = transformRigidIpcPoint(input.localPointA, surfaceA.pose);
      const Eigen::Vector3d pointB
          = transformRigidIpcPoint(input.localPointB, surfaceB.pose);
      const auto pointConstraint
          = newton_barrier::pointConnectionConstraint(pointA, pointB);
      residual = pointConstraint.residual;
      jacobianA = pointTransformDerivatives(input.localPointA, surfaceA.pose)
                      .jacobian;
      jacobianB = -pointTransformDerivatives(input.localPointB, surfaceB.pose)
                       .jacobian;
      break;
    }
    case RigidIpcArticulationConstraintType::HingeAxis: {
      const Eigen::Vector3d axisA
          = transformWithRotation(input.localAxisA, surfaceA.pose.rotation);
      const Eigen::Vector3d axisB
          = transformWithRotation(input.localAxisB, surfaceB.pose.rotation);
      const auto hingeConstraint
          = newton_barrier::hingeAxisConstraint(axisA, axisB);
      residual = hingeConstraint.residual;
      const Eigen::Matrix3d skewA = crossProductMatrix(axisA);
      const Eigen::Matrix3d skewB = crossProductMatrix(axisB);
      jacobianA = -skewB
                  * directionTransformJacobian(input.localAxisA, surfaceA.pose);
      jacobianB
          = skewA * directionTransformJacobian(input.localAxisB, surfaceB.pose);
      break;
    }
  }

  if (!residual.allFinite() || !jacobianA.allFinite()
      || !jacobianB.allFinite()) {
    return;
  }

  for (int row = 0; row < 3; ++row) {
    residuals.push_back(residual[row]);
  }
  scatterArticulationJacobianBlock(
      assembly, jacobianTriplets, rowOffset, input.bodyA, jacobianA);
  scatterArticulationJacobianBlock(
      assembly, jacobianTriplets, rowOffset, input.bodyB, jacobianB);
  assembly.activeArticulationConstraints.push_back(
      RigidIpcArticulationConstraint{
          input.type, input.bodyA, input.bodyB, residual});
}

template <typename ResidualVector, typename TripletVector>
void addArticulationConstraintRows(
    RigidIpcBarrierAssembly& assembly,
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcArticulationConstraintInput> constraints,
    ResidualVector& residuals,
    TripletVector& jacobianTriplets)
{
  if (constraints.empty()) {
    assembly.equalityResidual.resize(0);
    assembly.equalityJacobian.resize(0, assembly.gradient.size());
    return;
  }

  residuals.clear();
  residuals.reserve(3 * constraints.size());
  jacobianTriplets.clear();
  jacobianTriplets.reserve(12 * constraints.size());
  for (const RigidIpcArticulationConstraintInput& constraint : constraints) {
    addArticulationConstraintRows(
        assembly, residuals, jacobianTriplets, surfaces, constraint);
  }

  assembly.equalityResidual.resize(static_cast<Eigen::Index>(residuals.size()));
  assembly.equalityResidual.setZero();
  for (std::size_t row = 0; row < residuals.size(); ++row) {
    assembly.equalityResidual[static_cast<Eigen::Index>(row)] = residuals[row];
  }
  assembleSparseFromTripletsReusingPattern(
      assembly.equalityJacobian,
      assembly.equalityResidual.size(),
      assembly.gradient.size(),
      jacobianTriplets);
}

void assertMatchingSurfaceTopology(
    const RigidIpcBarrierSurface& start, const RigidIpcBarrierSurface& end)
{
  static_cast<void>(end);
  assert(start.body == end.body);
  assert(start.dynamic == end.dynamic);
  assert(start.vertices.size() == end.vertices.size());
  assert(start.triangles.size() == end.triangles.size());
  for (std::size_t i = 0; i < start.triangles.size(); ++i) {
    assert(start.triangles[i] == end.triangles[i]);
  }
}

void recordLineSearchCandidate(
    RigidIpcLineSearchResult& aggregate,
    const collision::native::CcdPrimitiveResult& candidate,
    const RigidIpcBarrierPrimitive primitive,
    const std::size_t bodyA,
    const std::size_t bodyB,
    const std::array<std::size_t, 4>& vertices)
{
  const auto outcome
      = newton_barrier::recordLineSearchCcdOutcome(aggregate.stats, candidate);
  if (outcome.hit) {
    const double candidateStep = outcome.stepBound;
    if (!aggregate.limited || candidateStep < aggregate.stepBound) {
      aggregate.limited = true;
      aggregate.stepBound = candidateStep;
      aggregate.limitingPrimitive = primitive;
      aggregate.bodyA = bodyA;
      aggregate.bodyB = bodyB;
      aggregate.vertices = vertices;
    }
    return;
  }

  if (outcome.indeterminate) {
    // The ACCD could not prove a definitive hit/miss within its budget, but
    // each conservative advance it took was a provably contact-free sub-step,
    // so it did prove the pair separated over [0, timeOfImpact]. Use that as a
    // conservative, provably-safe positive step bound (limiting like a hit)
    // rather than freezing the entire solve with a zero step. This keeps the
    // intersection-free guarantee (the bound is a proven lower bound on the
    // true TOI) while letting dense resting contacts -- where a couple of tight
    // pairs never fully resolve -- still advance. Only if no safe progress was
    // proven (timeOfImpact == 0) do we fall back to the blocking zero step.
    const double safeTime = outcome.stepBound;
    if (safeTime <= 0.0) {
      aggregate.indeterminate = true;
      aggregate.limited = true;
      aggregate.stepBound = 0.0;
      aggregate.limitingPrimitive = primitive;
      aggregate.bodyA = bodyA;
      aggregate.bodyB = bodyB;
      aggregate.vertices = vertices;
      return;
    }
    if (!aggregate.limited || safeTime < aggregate.stepBound) {
      aggregate.limited = true;
      aggregate.stepBound = safeTime;
      aggregate.limitingPrimitive = primitive;
      aggregate.bodyA = bodyA;
      aggregate.bodyB = bodyB;
      aggregate.vertices = vertices;
    }
    return;
  }
}

template <int MaxSize>
bool solveStackLinearSystemInPlace(
    Eigen::Matrix<double, MaxSize, MaxSize>& matrix,
    Eigen::Matrix<double, MaxSize, 1>& rhs,
    const Eigen::Index size,
    Eigen::Matrix<double, MaxSize, 1>& solution)
{
  constexpr double kPivotTolerance = 1e-12;
  for (Eigen::Index col = 0; col < size; ++col) {
    Eigen::Index pivot = col;
    double pivotAbs = std::abs(matrix(col, col));
    for (Eigen::Index row = col + 1; row < size; ++row) {
      const double candidate = std::abs(matrix(row, col));
      if (candidate > pivotAbs) {
        pivotAbs = candidate;
        pivot = row;
      }
    }
    if (!(pivotAbs > kPivotTolerance) || !std::isfinite(pivotAbs)) {
      return false;
    }
    if (pivot != col) {
      matrix.row(col).swap(matrix.row(pivot));
      std::swap(rhs[col], rhs[pivot]);
    }

    for (Eigen::Index row = col + 1; row < size; ++row) {
      const double factor = matrix(row, col) / matrix(col, col);
      matrix(row, col) = 0.0;
      for (Eigen::Index c = col + 1; c < size; ++c) {
        matrix(row, c) -= factor * matrix(col, c);
      }
      rhs[row] -= factor * rhs[col];
    }
  }

  solution.setZero();
  for (Eigen::Index row = size; row-- > 0;) {
    double value = rhs[row];
    for (Eigen::Index col = row + 1; col < size; ++col) {
      value -= matrix(row, col) * solution[col];
    }
    const double diagonal = matrix(row, row);
    if (!(std::abs(diagonal) > kPivotTolerance) || !std::isfinite(diagonal)) {
      return false;
    }
    solution[row] = value / diagonal;
  }
  return solution.head(size).allFinite();
}

bool solveDynamicLinearSystemInPlace(
    Eigen::MatrixXd& matrix,
    Eigen::VectorXd& rhs,
    const Eigen::Index size,
    Eigen::VectorXd& solution)
{
  constexpr double kPivotTolerance = 1e-12;
  for (Eigen::Index col = 0; col < size; ++col) {
    Eigen::Index pivot = col;
    double pivotAbs = std::abs(matrix(col, col));
    for (Eigen::Index row = col + 1; row < size; ++row) {
      const double candidate = std::abs(matrix(row, col));
      if (candidate > pivotAbs) {
        pivotAbs = candidate;
        pivot = row;
      }
    }
    if (!(pivotAbs > kPivotTolerance) || !std::isfinite(pivotAbs)) {
      return false;
    }
    if (pivot != col) {
      matrix.row(col).swap(matrix.row(pivot));
      std::swap(rhs[col], rhs[pivot]);
    }

    for (Eigen::Index row = col + 1; row < size; ++row) {
      const double factor = matrix(row, col) / matrix(col, col);
      matrix(row, col) = 0.0;
      for (Eigen::Index c = col + 1; c < size; ++c) {
        matrix(row, c) -= factor * matrix(col, c);
      }
      rhs[row] -= factor * rhs[col];
    }
  }

  solution.resize(size);
  solution.setZero();
  for (Eigen::Index row = size; row-- > 0;) {
    double value = rhs[row];
    for (Eigen::Index col = row + 1; col < size; ++col) {
      value -= matrix(row, col) * solution[col];
    }
    const double diagonal = matrix(row, row);
    if (!(std::abs(diagonal) > kPivotTolerance) || !std::isfinite(diagonal)) {
      return false;
    }
    solution[row] = value / diagonal;
  }
  return solution.head(size).allFinite();
}

bool solveEqualityConstrainedKktInto(
    const RigidIpcBarrierAssembly& assembly,
    const Eigen::MatrixXd& denseHessian,
    RigidIpcProjectedNewtonSolveScratchWorkspace& workspace,
    Eigen::VectorXd& rawStep)
{
  const Eigen::Index dofs = assembly.gradient.size();
  const Eigen::Index equalityRows = assembly.equalityResidual.size();
  const Eigen::Index kktSize = dofs + equalityRows;
  constexpr Eigen::Index kMaxStackKktSize = 24;
  if (kktSize <= kMaxStackKktSize) {
    Eigen::Matrix<double, kMaxStackKktSize, kMaxStackKktSize> kktMatrix;
    Eigen::Matrix<double, kMaxStackKktSize, 1> kktRhs;
    kktMatrix.setIdentity();
    kktMatrix.topLeftCorner(kktSize, kktSize).setZero();
    kktMatrix.topLeftCorner(dofs, dofs) = denseHessian;

    const auto& sparseJacobian = assembly.equalityJacobian;
    for (int outer = 0; outer < sparseJacobian.outerSize(); ++outer) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(sparseJacobian, outer);
           it;
           ++it) {
        kktMatrix(it.col(), dofs + it.row()) = it.value();
        kktMatrix(dofs + it.row(), it.col()) = it.value();
      }
    }

    kktRhs.setZero();
    kktRhs.head(dofs) = -assembly.gradient;
    kktRhs.segment(dofs, equalityRows) = -assembly.equalityResidual;

    Eigen::Matrix<double, kMaxStackKktSize, 1> kktSolution;
    if (!solveStackLinearSystemInPlace(
            kktMatrix, kktRhs, kktSize, kktSolution)) {
      return false;
    }
    rawStep.resize(dofs);
    rawStep = kktSolution.head(dofs);
    return true;
  }

  auto& denseJacobian = workspace.equalityDenseJacobian;
  denseJacobian.resize(equalityRows, dofs);
  denseJacobian.setZero();
  const auto& sparseJacobian = assembly.equalityJacobian;
  for (int outer = 0; outer < sparseJacobian.outerSize(); ++outer) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sparseJacobian, outer);
         it;
         ++it) {
      denseJacobian(it.row(), it.col()) = it.value();
    }
  }

  auto& kktMatrix = workspace.equalityKktMatrix;
  kktMatrix.resize(kktSize, kktSize);
  kktMatrix.setZero();
  kktMatrix.topLeftCorner(dofs, dofs) = denseHessian;
  kktMatrix.topRightCorner(dofs, equalityRows) = denseJacobian.transpose();
  kktMatrix.bottomLeftCorner(equalityRows, dofs) = denseJacobian;

  auto& kktRhs = workspace.equalityKktRhs;
  kktRhs.resize(kktSize);
  kktRhs.head(dofs) = -assembly.gradient;
  kktRhs.tail(equalityRows) = -assembly.equalityResidual;

  auto& kktSolution = workspace.equalityKktSolution;
  if (!solveDynamicLinearSystemInPlace(
          kktMatrix, kktRhs, kktSize, kktSolution)) {
    return false;
  }

  rawStep.resize(dofs);
  rawStep = kktSolution.head(dofs);
  return true;
}

RigidIpcProjectedNewtonStep& computeRigidIpcProjectedNewtonStepInto(
    const RigidIpcBarrierAssembly& assembly,
    const RigidIpcLineSearchResult* lineSearch,
    const RigidIpcProjectedNewtonOptions& options,
    RigidIpcProjectedNewtonStep& result,
    dart::common::MemoryAllocator* allocator = nullptr,
    RigidIpcProjectedNewtonSolveScratchWorkspace* workspace = nullptr)
{
  assert(assembly.hessian.rows() == assembly.gradient.size());
  assert(assembly.hessian.cols() == assembly.gradient.size());
  assert(assembly.equalityJacobian.cols() == assembly.gradient.size());
  assert(assembly.equalityJacobian.rows() == assembly.equalityResidual.size());

  result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
  result.success = false;
  result.converged = false;
  result.lineSearchBlocked = false;
  result.stats = RigidIpcProjectedNewtonStats{};

  const Eigen::Index dofs = assembly.gradient.size();
  const Eigen::Index equalityRows = assembly.equalityResidual.size();
  const bool hasEqualityRows = equalityRows > 0;
  result.delta.resize(dofs);
  result.delta.setZero();
  result.stats.dofs = static_cast<std::size_t>(dofs);
  result.stats.gradientNorm = assembly.gradient.norm();

  if (dofs == 0) {
    result.status = RigidIpcProjectedNewtonStatus::NoDofs;
    result.success = true;
    result.converged = true;
    return result;
  }

  if (!assembly.gradient.allFinite()
      || (hasEqualityRows && !assembly.equalityResidual.allFinite())) {
    result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
    return result;
  }

  if (!hasEqualityRows
      && newton_barrier::projectedNewtonResidualConverged(
          result.stats.gradientNorm, options.gradientTolerance)) {
    result.status = RigidIpcProjectedNewtonStatus::Converged;
    result.success = true;
    result.converged = true;
    return result;
  }

  const double regularization = std::max(0.0, options.hessianRegularization);
  result.stats.hessianRegularization = regularization;

  Eigen::MatrixXd localDenseHessian;
  Eigen::VectorXd localRawStep;
  Eigen::LDLT<Eigen::MatrixXd> localDenseHessianLdlt;
  Eigen::MatrixXd& denseHessian
      = workspace != nullptr ? workspace->denseHessian : localDenseHessian;
  Eigen::VectorXd& rawStep
      = workspace != nullptr ? workspace->rawStep : localRawStep;
  Eigen::LDLT<Eigen::MatrixXd>& denseHessianLdlt
      = workspace != nullptr ? workspace->denseHessianLdlt
                             : localDenseHessianLdlt;

  denseHessian.resize(dofs, dofs);
  denseHessian = assembly.hessian;
  denseHessian.diagonal().array() += regularization;
  if (!denseHessian.allFinite()) {
    result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
    return result;
  }

  if (hasEqualityRows && workspace != nullptr) {
    if (!solveEqualityConstrainedKktInto(
            assembly, denseHessian, *workspace, rawStep)) {
      result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
      return result;
    }
  } else if (hasEqualityRows) {
    dart::common::MemoryAllocator& changeOfVariableAllocator
        = allocator ? *allocator : dart::common::MemoryAllocator::GetDefault();
    const auto changeOfVariable = newton_barrier::makeEqualityChangeOfVariable(
        assembly.equalityJacobian,
        assembly.equalityResidual,
        changeOfVariableAllocator);
    if (!changeOfVariable.valid) {
      result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
      return result;
    }

    rawStep = newton_barrier::solveEqualityConstrainedQuadraticReduced(
        denseHessian, assembly.gradient, changeOfVariable);
    if (!rawStep.allFinite()) {
      result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
      return result;
    }
  } else {
    denseHessianLdlt.compute(denseHessian);
    if (denseHessianLdlt.info() != Eigen::Success
        || !denseHessianLdlt.isPositive()) {
      result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
      return result;
    }

    rawStep.resize(dofs);
    rawStep = -assembly.gradient;
    denseHessianLdlt.solveInPlace(rawStep);
    if (denseHessianLdlt.info() != Eigen::Success) {
      result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
      return result;
    }
  }
  if (!rawStep.allFinite()) {
    result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
    return result;
  }

  result.stats.rawStepNorm = rawStep.norm();
  double stepScale = 1.0;
  if (std::isfinite(options.maxStepNorm) && options.maxStepNorm >= 0.0
      && result.stats.rawStepNorm > options.maxStepNorm
      && result.stats.rawStepNorm > 0.0) {
    stepScale = options.maxStepNorm / result.stats.rawStepNorm;
  }

  if (lineSearch != nullptr) {
    result.stats.usedLineSearch = true;
    if (!lineSearch->allowsPositiveStep()) {
      result.status = RigidIpcProjectedNewtonStatus::LineSearchBlocked;
      result.lineSearchBlocked = true;
      result.stats.stepScale = 0.0;
      result.stats.lineSearchLimited = lineSearch->limited;
      return result;
    }

    if (lineSearch->limited) {
      result.stats.lineSearchLimited = true;
      const double safeBound = newton_barrier::makeLineSearchStepScale(
          lineSearch->stepBound, options.lineSearchSafetyScale);
      if (safeBound <= 0.0) {
        result.status = RigidIpcProjectedNewtonStatus::LineSearchBlocked;
        result.lineSearchBlocked = true;
        result.stats.stepScale = 0.0;
        return result;
      }
      stepScale = std::min(stepScale, safeBound);
    }
  }

  result.delta = stepScale * rawStep;
  result.stats.stepScale = stepScale;
  result.stats.stepNorm = result.delta.norm();
  result.stats.gradientDotStep = assembly.gradient.dot(result.delta.asEigen());
  if ((!hasEqualityRows && !(result.stats.gradientDotStep < 0.0))
      || !std::isfinite(result.stats.gradientDotStep)) {
    result.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
    result.delta.setZero();
    result.stats.stepNorm = 0.0;
    result.stats.stepScale = 0.0;
    result.stats.gradientDotStep = 0.0;
    return result;
  }

  result.status = RigidIpcProjectedNewtonStatus::DescentStep;
  result.success = true;
  return result;
}

RigidIpcProjectedNewtonStep computeRigidIpcProjectedNewtonStepImpl(
    const RigidIpcBarrierAssembly& assembly,
    const RigidIpcLineSearchResult* lineSearch,
    const RigidIpcProjectedNewtonOptions& options,
    dart::common::MemoryAllocator* allocator = nullptr)
{
  RigidIpcProjectedNewtonStep result;
  computeRigidIpcProjectedNewtonStepInto(
      assembly, lineSearch, options, result, allocator);
  return result;
}

template <typename Derived>
void applyRigidIpcNewtonDelta(
    auto& surfaces,
    const RigidIpcBarrierAssembly& assembly,
    const Eigen::MatrixBase<Derived>& delta)
{
  assert(assembly.bodyDofOffsets.size() == surfaces.size());
  for (std::size_t body = 0; body < surfaces.size(); ++body) {
    const std::size_t offset = assembly.bodyDofOffsets[body];
    if (offset == RigidIpcBarrierAssembly::npos) {
      continue;
    }
    assert(static_cast<Eigen::Index>(offset + 6) <= delta.size());
    surfaces[body].pose.position
        += delta.template segment<3>(static_cast<Eigen::Index>(offset));
    surfaces[body].pose.rotation
        += delta.template segment<3>(static_cast<Eigen::Index>(offset + 3));
  }
}

void scaleRigidIpcNewtonStep(
    RigidIpcProjectedNewtonStep& step, const double scale)
{
  assert(scale >= 0.0);
  step.delta *= scale;
  step.stats.stepScale *= scale;
  step.stats.stepNorm *= scale;
  step.stats.gradientDotStep *= scale;
}

void recordSolveAssemblyStats(
    RigidIpcProjectedNewtonSolveResult& result, const bool initial)
{
  if (initial) {
    result.stats.initialValue = result.assembly.value;
    result.stats.initialGradientNorm = result.assembly.gradient.norm();
    result.stats.initialEqualityResidualNorm
        = result.assembly.equalityResidual.norm();
  }
  result.stats.finalValue = result.assembly.value;
  result.stats.finalGradientNorm = result.assembly.gradient.norm();
  result.stats.finalEqualityResidualNorm
      = result.assembly.equalityResidual.norm();
  result.stats.finalMomentumBalance = std::max(
      result.stats.finalGradientNorm, result.stats.finalEqualityResidualNorm);
  result.stats.activeArticulationConstraints
      = result.assembly.activeArticulationConstraints.size();
  result.stats.activeFrictionConstraints
      = result.assembly.activeFrictionConstraints.size();
}

void recordSolveLineSearchStats(RigidIpcProjectedNewtonSolveResult& result)
{
  result.stats.lineSearchPointPointChecks
      += result.lineSearch.stats.pointPointChecks;
  result.stats.lineSearchPointEdgeChecks
      += result.lineSearch.stats.pointEdgeChecks;
  result.stats.lineSearchEdgeEdgeChecks
      += result.lineSearch.stats.edgeEdgeChecks;
  result.stats.lineSearchPointTriangleChecks
      += result.lineSearch.stats.pointTriangleChecks;
  result.stats.lineSearchHits += result.lineSearch.stats.hits;
  result.stats.lineSearchMisses += result.lineSearch.stats.misses;
  result.stats.lineSearchIndeterminateCount
      += result.lineSearch.stats.indeterminate;
  result.stats.lineSearchZeroStepCount += result.lineSearch.stats.zeroStepCount;
}

} // namespace

static RigidIpcBarrierAssembly assembleRigidIpcBarrierSystemWithScratch(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcBarrierOptions& options,
    RigidIpcAssemblyScratch& scratch);

static void assembleRigidIpcBarrierSystemWithScratchInto(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcBarrierOptions& options,
    RigidIpcAssemblyScratch& scratch,
    RigidIpcBarrierAssembly& assembly,
    bool finalizeHessian = true,
    bool preserveEqualityStorage = false);

static RigidIpcBarrierAssembly assembleRigidIpcObjectiveSystemWithScratch(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions,
    RigidIpcAssemblyScratch& scratch);

static void assembleRigidIpcObjectiveSystemWithScratchInto(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions,
    RigidIpcAssemblyScratch& scratch,
    RigidIpcBarrierAssembly& assembly);

static RigidIpcLineSearchResult computeRigidIpcLineSearchStepBoundWithScratch(
    std::span<const RigidIpcBarrierSurface> startSurfaces,
    std::span<const RigidIpcBarrierSurface> endSurfaces,
    const RigidIpcLineSearchOptions& options,
    RigidIpcLineSearchScratch& scratch);

RigidIpcBodyDynamicsTerm makeRigidIpcBodyDynamicsTerm(
    const RigidIpcBodyDynamicsState& state, const double timeStep)
{
  RigidIpcBodyDynamicsTerm term;
  term.targetPose = state.pose;

  const bool validState
      = state.active && std::isfinite(timeStep) && timeStep > 0.0
        && std::isfinite(state.mass) && state.mass > 0.0
        && state.pose.position.allFinite() && state.pose.rotation.allFinite()
        && state.velocity.allFinite() && state.inertia.allFinite()
        && state.generalizedForce.allFinite();
  if (!validState) {
    return term;
  }

  const double inverseTimeStepSquared = 1.0 / (timeStep * timeStep);
  if (!std::isfinite(inverseTimeStepSquared)) {
    return term;
  }

  term.active = true;
  term.targetPose.position
      = state.pose.position + timeStep * state.velocity.head<3>();
  term.targetPose.rotation
      = state.pose.rotation + timeStep * state.velocity.tail<3>();
  term.diagonalWeights.head<3>().setConstant(
      state.mass * inverseTimeStepSquared);
  term.diagonalWeights.tail<3>()
      = (state.inertia.diagonal().array().max(0.0) * inverseTimeStepSquared)
            .matrix();
  term.generalizedForce = state.generalizedForce;
  return term;
}

RigidIpcPrimitiveBarrierResult rigidIpcPointTriangleBarrierAtTime(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePoseStart,
    const RigidIpcPose& trianglePoseEnd,
    const double time,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d pointWorld
      = transformRigidIpcPoint(point, pointPoseStart, pointPoseEnd, time);
  const Eigen::Vector3d triangleAWorld = transformRigidIpcPoint(
      triangleA, trianglePoseStart, trianglePoseEnd, time);
  const Eigen::Vector3d triangleBWorld = transformRigidIpcPoint(
      triangleB, trianglePoseStart, trianglePoseEnd, time);
  const Eigen::Vector3d triangleCWorld = transformRigidIpcPoint(
      triangleC, trianglePoseStart, trianglePoseEnd, time);

  return newton_barrier::pointTriangleBarrier(
      pointWorld,
      triangleAWorld,
      triangleBWorld,
      triangleCWorld,
      options.squaredActivationDistance,
      options.stiffness);
}

RigidIpcPrimitiveBarrierResult rigidIpcPointEdgeBarrierAtTime(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPoseStart,
    const RigidIpcPose& pointPoseEnd,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePoseStart,
    const RigidIpcPose& edgePoseEnd,
    const double time,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d pointWorld
      = transformRigidIpcPoint(point, pointPoseStart, pointPoseEnd, time);
  const Eigen::Vector3d edgeAWorld
      = transformRigidIpcPoint(edgeA, edgePoseStart, edgePoseEnd, time);
  const Eigen::Vector3d edgeBWorld
      = transformRigidIpcPoint(edgeB, edgePoseStart, edgePoseEnd, time);

  return newton_barrier::pointEdgeBarrier(
      pointWorld,
      edgeAWorld,
      edgeBWorld,
      options.squaredActivationDistance,
      options.stiffness);
}

RigidIpcReducedBarrierResult rigidIpcPointTriangleReducedBarrier(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& trianglePose,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d pointWorld = transformRigidIpcPoint(point, pointPose);
  const Eigen::Vector3d triangleAWorld
      = transformRigidIpcPoint(triangleA, trianglePose);
  const Eigen::Vector3d triangleBWorld
      = transformRigidIpcPoint(triangleB, trianglePose);
  const Eigen::Vector3d triangleCWorld
      = transformRigidIpcPoint(triangleC, trianglePose);
  const RigidIpcPrimitiveBarrierResult primitive
      = newton_barrier::pointTriangleBarrier(
          pointWorld,
          triangleAWorld,
          triangleBWorld,
          triangleCWorld,
          options.squaredActivationDistance,
          options.stiffness);

  return chainPrimitiveBarrierToReducedCoordinates(
      primitive,
      {pointTransformDerivatives(point, pointPose),
       pointTransformDerivatives(triangleA, trianglePose),
       pointTransformDerivatives(triangleB, trianglePose),
       pointTransformDerivatives(triangleC, trianglePose)},
      {0, 1, 1, 1},
      options);
}

RigidIpcReducedBarrierResult rigidIpcPointEdgeReducedBarrier(
    const Eigen::Vector3d& point,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& edgePose,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d pointWorld = transformRigidIpcPoint(point, pointPose);
  const Eigen::Vector3d edgeAWorld = transformRigidIpcPoint(edgeA, edgePose);
  const Eigen::Vector3d edgeBWorld = transformRigidIpcPoint(edgeB, edgePose);
  const RigidIpcPrimitiveBarrierResult primitive
      = newton_barrier::pointEdgeBarrier(
          pointWorld,
          edgeAWorld,
          edgeBWorld,
          options.squaredActivationDistance,
          options.stiffness);

  return chainPrimitiveBarrierToReducedCoordinates(
      primitive,
      {pointTransformDerivatives(point, pointPose),
       pointTransformDerivatives(edgeA, edgePose),
       pointTransformDerivatives(edgeB, edgePose),
       PointTransformDerivatives{}},
      {0, 1, 1, 0},
      options);
}

RigidIpcReducedBarrierResult rigidIpcEdgeEdgeReducedBarrier(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPose,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPose,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d edgeA0World = transformRigidIpcPoint(edgeA0, edgeAPose);
  const Eigen::Vector3d edgeA1World = transformRigidIpcPoint(edgeA1, edgeAPose);
  const Eigen::Vector3d edgeB0World = transformRigidIpcPoint(edgeB0, edgeBPose);
  const Eigen::Vector3d edgeB1World = transformRigidIpcPoint(edgeB1, edgeBPose);
  const RigidIpcPrimitiveBarrierResult primitive
      = newton_barrier::edgeEdgeBarrier(
          edgeA0World,
          edgeA1World,
          edgeB0World,
          edgeB1World,
          options.squaredActivationDistance,
          options.stiffness);

  return chainPrimitiveBarrierToReducedCoordinates(
      primitive,
      {pointTransformDerivatives(edgeA0, edgeAPose),
       pointTransformDerivatives(edgeA1, edgeAPose),
       pointTransformDerivatives(edgeB0, edgeBPose),
       pointTransformDerivatives(edgeB1, edgeBPose)},
      {0, 0, 1, 1},
      options);
}

RigidIpcReducedBarrierResult rigidIpcPointPointReducedBarrier(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& pointAPose,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& pointBPose,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d pointAWorld
      = transformRigidIpcPoint(pointA, pointAPose);
  const Eigen::Vector3d pointBWorld
      = transformRigidIpcPoint(pointB, pointBPose);
  const RigidIpcPrimitiveBarrierResult primitive
      = newton_barrier::pointPointBarrier(
          pointAWorld,
          pointBWorld,
          options.squaredActivationDistance,
          options.stiffness);

  return chainPrimitiveBarrierToReducedCoordinates(
      primitive,
      {pointTransformDerivatives(pointA, pointAPose),
       pointTransformDerivatives(pointB, pointBPose),
       PointTransformDerivatives{},
       PointTransformDerivatives{}},
      {0, 1, 0, 0},
      options);
}

RigidIpcFrictionPotentialResult rigidIpcPointPointFrictionPotential(
    const Eigen::Vector3d& laggedPointA,
    const Eigen::Vector3d& laggedPointB,
    const Eigen::Vector3d& pointA,
    const Eigen::Vector3d& pointB,
    const RigidIpcFrictionOptions& options)
{
  RigidIpcFrictionPotentialResult result;
  if (!laggedPointA.allFinite() || !laggedPointB.allFinite()
      || !pointA.allFinite() || !pointB.allFinite()) {
    return result;
  }

  const auto stencil
      = newton_barrier::pointPointTangentStencil(laggedPointA, laggedPointB);
  Eigen::Matrix<double, 6, 1> displacement;
  displacement.head<3>() = pointA - laggedPointA;
  displacement.tail<3>() = pointB - laggedPointB;
  return computeProjectedFrictionPotential<6>(
      stencil.projection, displacement, options);
}

RigidIpcFrictionPotentialResult rigidIpcPointEdgeFrictionPotential(
    const Eigen::Vector3d& laggedPoint,
    const Eigen::Vector3d& laggedEdgeA,
    const Eigen::Vector3d& laggedEdgeB,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcFrictionOptions& options)
{
  RigidIpcFrictionPotentialResult result;
  if (!laggedPoint.allFinite() || !laggedEdgeA.allFinite()
      || !laggedEdgeB.allFinite() || !point.allFinite() || !edgeA.allFinite()
      || !edgeB.allFinite()) {
    return result;
  }

  const auto stencil = newton_barrier::pointEdgeTangentStencil(
      laggedPoint, laggedEdgeA, laggedEdgeB);
  Eigen::Matrix<double, 9, 1> displacement;
  displacement.segment<3>(0) = point - laggedPoint;
  displacement.segment<3>(3) = edgeA - laggedEdgeA;
  displacement.segment<3>(6) = edgeB - laggedEdgeB;
  return computeProjectedFrictionPotential<9>(
      stencil.projection, displacement, options);
}

RigidIpcFrictionPotentialResult rigidIpcEdgeEdgeFrictionPotential(
    const Eigen::Vector3d& laggedEdgeA0,
    const Eigen::Vector3d& laggedEdgeA1,
    const Eigen::Vector3d& laggedEdgeB0,
    const Eigen::Vector3d& laggedEdgeB1,
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcFrictionOptions& options)
{
  RigidIpcFrictionPotentialResult result;
  if (!laggedEdgeA0.allFinite() || !laggedEdgeA1.allFinite()
      || !laggedEdgeB0.allFinite() || !laggedEdgeB1.allFinite()
      || !edgeA0.allFinite() || !edgeA1.allFinite() || !edgeB0.allFinite()
      || !edgeB1.allFinite()) {
    return result;
  }

  const auto stencil = newton_barrier::edgeEdgeTangentStencil(
      laggedEdgeA0, laggedEdgeA1, laggedEdgeB0, laggedEdgeB1);
  RigidIpcVector12d displacement;
  displacement.segment<3>(0) = edgeA0 - laggedEdgeA0;
  displacement.segment<3>(3) = edgeA1 - laggedEdgeA1;
  displacement.segment<3>(6) = edgeB0 - laggedEdgeB0;
  displacement.segment<3>(9) = edgeB1 - laggedEdgeB1;
  return computeProjectedFrictionPotential<12>(
      stencil.projection, displacement, options);
}

RigidIpcFrictionPotentialResult rigidIpcPointTriangleFrictionPotential(
    const Eigen::Vector3d& laggedPoint,
    const Eigen::Vector3d& laggedTriangleA,
    const Eigen::Vector3d& laggedTriangleB,
    const Eigen::Vector3d& laggedTriangleC,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcFrictionOptions& options)
{
  RigidIpcFrictionPotentialResult result;
  if (!laggedPoint.allFinite() || !laggedTriangleA.allFinite()
      || !laggedTriangleB.allFinite() || !laggedTriangleC.allFinite()
      || !point.allFinite() || !triangleA.allFinite() || !triangleB.allFinite()
      || !triangleC.allFinite()) {
    return result;
  }

  const auto stencil = newton_barrier::pointTriangleTangentStencil(
      laggedPoint, laggedTriangleA, laggedTriangleB, laggedTriangleC);
  RigidIpcVector12d displacement;
  displacement.segment<3>(0) = point - laggedPoint;
  displacement.segment<3>(3) = triangleA - laggedTriangleA;
  displacement.segment<3>(6) = triangleB - laggedTriangleB;
  displacement.segment<3>(9) = triangleC - laggedTriangleC;
  return computeProjectedFrictionPotential<12>(
      stencil.projection, displacement, options);
}

RigidIpcReducedFrictionResult rigidIpcPointPointReducedFrictionPotential(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& laggedPointAPose,
    const RigidIpcPose& pointAPose,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& laggedPointBPose,
    const RigidIpcPose& pointBPose,
    const RigidIpcFrictionOptions& options)
{
  const Eigen::Vector3d laggedPointAWorld
      = transformRigidIpcPoint(pointA, laggedPointAPose);
  const Eigen::Vector3d laggedPointBWorld
      = transformRigidIpcPoint(pointB, laggedPointBPose);
  const Eigen::Vector3d pointAWorld
      = transformRigidIpcPoint(pointA, pointAPose);
  const Eigen::Vector3d pointBWorld
      = transformRigidIpcPoint(pointB, pointBPose);
  const RigidIpcFrictionPotentialResult potential
      = rigidIpcPointPointFrictionPotential(
          laggedPointAWorld,
          laggedPointBWorld,
          pointAWorld,
          pointBWorld,
          options);

  return chainFrictionPotentialToReducedCoordinates(
      potential,
      {pointTransformDerivatives(pointA, pointAPose),
       pointTransformDerivatives(pointB, pointBPose),
       PointTransformDerivatives{},
       PointTransformDerivatives{}},
      {0, 1, 0, 0},
      options.projectReducedHessianToPsd);
}

RigidIpcReducedFrictionResult rigidIpcPointEdgeReducedFrictionPotential(
    const Eigen::Vector3d& point,
    const RigidIpcPose& laggedPointPose,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& edgeA,
    const Eigen::Vector3d& edgeB,
    const RigidIpcPose& laggedEdgePose,
    const RigidIpcPose& edgePose,
    const RigidIpcFrictionOptions& options)
{
  const Eigen::Vector3d laggedPointWorld
      = transformRigidIpcPoint(point, laggedPointPose);
  const Eigen::Vector3d laggedEdgeAWorld
      = transformRigidIpcPoint(edgeA, laggedEdgePose);
  const Eigen::Vector3d laggedEdgeBWorld
      = transformRigidIpcPoint(edgeB, laggedEdgePose);
  const Eigen::Vector3d pointWorld = transformRigidIpcPoint(point, pointPose);
  const Eigen::Vector3d edgeAWorld = transformRigidIpcPoint(edgeA, edgePose);
  const Eigen::Vector3d edgeBWorld = transformRigidIpcPoint(edgeB, edgePose);
  const RigidIpcFrictionPotentialResult potential
      = rigidIpcPointEdgeFrictionPotential(
          laggedPointWorld,
          laggedEdgeAWorld,
          laggedEdgeBWorld,
          pointWorld,
          edgeAWorld,
          edgeBWorld,
          options);

  return chainFrictionPotentialToReducedCoordinates(
      potential,
      {pointTransformDerivatives(point, pointPose),
       pointTransformDerivatives(edgeA, edgePose),
       pointTransformDerivatives(edgeB, edgePose),
       PointTransformDerivatives{}},
      {0, 1, 1, 0},
      options.projectReducedHessianToPsd);
}

RigidIpcReducedFrictionResult rigidIpcEdgeEdgeReducedFrictionPotential(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& laggedEdgeAPose,
    const RigidIpcPose& edgeAPose,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& laggedEdgeBPose,
    const RigidIpcPose& edgeBPose,
    const RigidIpcFrictionOptions& options)
{
  const Eigen::Vector3d laggedEdgeA0World
      = transformRigidIpcPoint(edgeA0, laggedEdgeAPose);
  const Eigen::Vector3d laggedEdgeA1World
      = transformRigidIpcPoint(edgeA1, laggedEdgeAPose);
  const Eigen::Vector3d laggedEdgeB0World
      = transformRigidIpcPoint(edgeB0, laggedEdgeBPose);
  const Eigen::Vector3d laggedEdgeB1World
      = transformRigidIpcPoint(edgeB1, laggedEdgeBPose);
  const Eigen::Vector3d edgeA0World = transformRigidIpcPoint(edgeA0, edgeAPose);
  const Eigen::Vector3d edgeA1World = transformRigidIpcPoint(edgeA1, edgeAPose);
  const Eigen::Vector3d edgeB0World = transformRigidIpcPoint(edgeB0, edgeBPose);
  const Eigen::Vector3d edgeB1World = transformRigidIpcPoint(edgeB1, edgeBPose);
  const RigidIpcFrictionPotentialResult potential
      = rigidIpcEdgeEdgeFrictionPotential(
          laggedEdgeA0World,
          laggedEdgeA1World,
          laggedEdgeB0World,
          laggedEdgeB1World,
          edgeA0World,
          edgeA1World,
          edgeB0World,
          edgeB1World,
          options);

  return chainFrictionPotentialToReducedCoordinates(
      potential,
      {pointTransformDerivatives(edgeA0, edgeAPose),
       pointTransformDerivatives(edgeA1, edgeAPose),
       pointTransformDerivatives(edgeB0, edgeBPose),
       pointTransformDerivatives(edgeB1, edgeBPose)},
      {0, 0, 1, 1},
      options.projectReducedHessianToPsd);
}

RigidIpcReducedFrictionResult rigidIpcPointTriangleReducedFrictionPotential(
    const Eigen::Vector3d& point,
    const RigidIpcPose& laggedPointPose,
    const RigidIpcPose& pointPose,
    const Eigen::Vector3d& triangleA,
    const Eigen::Vector3d& triangleB,
    const Eigen::Vector3d& triangleC,
    const RigidIpcPose& laggedTrianglePose,
    const RigidIpcPose& trianglePose,
    const RigidIpcFrictionOptions& options)
{
  const Eigen::Vector3d laggedPointWorld
      = transformRigidIpcPoint(point, laggedPointPose);
  const Eigen::Vector3d laggedTriangleAWorld
      = transformRigidIpcPoint(triangleA, laggedTrianglePose);
  const Eigen::Vector3d laggedTriangleBWorld
      = transformRigidIpcPoint(triangleB, laggedTrianglePose);
  const Eigen::Vector3d laggedTriangleCWorld
      = transformRigidIpcPoint(triangleC, laggedTrianglePose);
  const Eigen::Vector3d pointWorld = transformRigidIpcPoint(point, pointPose);
  const Eigen::Vector3d triangleAWorld
      = transformRigidIpcPoint(triangleA, trianglePose);
  const Eigen::Vector3d triangleBWorld
      = transformRigidIpcPoint(triangleB, trianglePose);
  const Eigen::Vector3d triangleCWorld
      = transformRigidIpcPoint(triangleC, trianglePose);
  const RigidIpcFrictionPotentialResult potential
      = rigidIpcPointTriangleFrictionPotential(
          laggedPointWorld,
          laggedTriangleAWorld,
          laggedTriangleBWorld,
          laggedTriangleCWorld,
          pointWorld,
          triangleAWorld,
          triangleBWorld,
          triangleCWorld,
          options);

  return chainFrictionPotentialToReducedCoordinates(
      potential,
      {pointTransformDerivatives(point, pointPose),
       pointTransformDerivatives(triangleA, trianglePose),
       pointTransformDerivatives(triangleB, trianglePose),
       pointTransformDerivatives(triangleC, trianglePose)},
      {0, 1, 1, 1},
      options.projectReducedHessianToPsd);
}

RigidIpcPrimitiveBarrierResult rigidIpcEdgeEdgeBarrierAtTime(
    const Eigen::Vector3d& edgeA0,
    const Eigen::Vector3d& edgeA1,
    const RigidIpcPose& edgeAPoseStart,
    const RigidIpcPose& edgeAPoseEnd,
    const Eigen::Vector3d& edgeB0,
    const Eigen::Vector3d& edgeB1,
    const RigidIpcPose& edgeBPoseStart,
    const RigidIpcPose& edgeBPoseEnd,
    const double time,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d edgeA0World
      = transformRigidIpcPoint(edgeA0, edgeAPoseStart, edgeAPoseEnd, time);
  const Eigen::Vector3d edgeA1World
      = transformRigidIpcPoint(edgeA1, edgeAPoseStart, edgeAPoseEnd, time);
  const Eigen::Vector3d edgeB0World
      = transformRigidIpcPoint(edgeB0, edgeBPoseStart, edgeBPoseEnd, time);
  const Eigen::Vector3d edgeB1World
      = transformRigidIpcPoint(edgeB1, edgeBPoseStart, edgeBPoseEnd, time);

  return newton_barrier::edgeEdgeBarrier(
      edgeA0World,
      edgeA1World,
      edgeB0World,
      edgeB1World,
      options.squaredActivationDistance,
      options.stiffness);
}

RigidIpcPrimitiveBarrierResult rigidIpcPointPointBarrierAtTime(
    const Eigen::Vector3d& pointA,
    const RigidIpcPose& pointAPoseStart,
    const RigidIpcPose& pointAPoseEnd,
    const Eigen::Vector3d& pointB,
    const RigidIpcPose& pointBPoseStart,
    const RigidIpcPose& pointBPoseEnd,
    const double time,
    const RigidIpcBarrierOptions& options)
{
  const Eigen::Vector3d pointAWorld
      = transformRigidIpcPoint(pointA, pointAPoseStart, pointAPoseEnd, time);
  const Eigen::Vector3d pointBWorld
      = transformRigidIpcPoint(pointB, pointBPoseStart, pointBPoseEnd, time);

  return newton_barrier::pointPointBarrier(
      pointAWorld,
      pointBWorld,
      options.squaredActivationDistance,
      options.stiffness);
}

RigidIpcBarrierAssembly assembleRigidIpcBarrierSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcBarrierOptions& options)
{
  RigidIpcAssemblyScratch scratch;
  return assembleRigidIpcBarrierSystemWithScratch(surfaces, options, scratch);
}

static RigidIpcBarrierAssembly assembleRigidIpcBarrierSystemWithScratch(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcBarrierOptions& options,
    RigidIpcAssemblyScratch& scratch)
{
  RigidIpcBarrierAssembly assembly(
      allocatorOrDefault(scratch.pairs.memoryAllocator));
  assembleRigidIpcBarrierSystemWithScratchInto(
      surfaces, options, scratch, assembly);
  return assembly;
}

static void assembleRigidIpcBarrierSystemWithScratchInto(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcBarrierOptions& options,
    RigidIpcAssemblyScratch& scratch,
    RigidIpcBarrierAssembly& assembly,
    bool finalizeHessian,
    bool preserveEqualityStorage)
{
  initializeGlobalAssembly(surfaces, assembly, preserveEqualityStorage);

  scratch.pairs.resizeSurfaceEdges(surfaces.size());
  auto& surfaceAabbs = scratch.pairs.surfaceAabbs;
  surfaceAabbs.clear();
  surfaceAabbs.reserve(surfaces.size());
  for (std::size_t body = 0; body < surfaces.size(); ++body) {
    buildSurfaceEdges(surfaces[body], scratch.pairs.surfaceEdgesAt(body));
    surfaceAabbs.push_back(computeSurfaceWorldAabb(surfaces[body]));
  }

  auto& triplets = scratch.barrierTriplets;
  triplets.clear();
  const double activationDistance
      = std::sqrt(std::max(0.0, options.squaredActivationDistance));
  auto& surfaceMargins = scratch.pairs.surfaceMargins;
  surfaceMargins.assign(surfaces.size(), 0.5 * activationDistance);
  collectCandidateSurfacePairs(
      surfaceAabbs,
      surfaceMargins,
      scratch.pairs.sweepItems,
      scratch.pairs.candidatePairs);
  const auto& candidatePairs = scratch.pairs.candidatePairs;
  for (const auto& candidatePair : candidatePairs) {
    const std::size_t bodyA = candidatePair.first;
    const std::size_t bodyB = candidatePair.second;
    const RigidIpcBarrierSurface& a = surfaces[bodyA];
    {
      const RigidIpcBarrierSurface& b = surfaces[bodyB];
      if (!a.dynamic && !b.dynamic) {
        continue;
      }

      // Exact broad-phase cull on the sweep candidates: skip surface pairs
      // whose world AABBs are already at or beyond the activation distance,
      // where every primitive barrier is inactive. This is behavior-preserving
      // (the AABB gap lower bounds the true surface distance) and, with the
      // sort-and-sweep candidate enumeration above, keeps the assembled system
      // identical to the former all-pairs loop.
      if (surfaceAabbs[bodyA].valid && surfaceAabbs[bodyB].valid
          && surfaceAabbSquaredDistance(
                 surfaceAabbs[bodyA], surfaceAabbs[bodyB])
                 >= options.squaredActivationDistance) {
        continue;
      }

      for (std::size_t pointA = 0; pointA < a.vertices.size(); ++pointA) {
        for (std::size_t pointB = 0; pointB < b.vertices.size(); ++pointB) {
          const auto reduced = rigidIpcPointPointReducedBarrier(
              a.vertices[pointA], a.pose, b.vertices[pointB], b.pose, options);
          addActiveConstraint(
              assembly,
              triplets,
              RigidIpcBarrierPrimitive::VertexVertex,
              bodyA,
              bodyB,
              {pointA,
               pointB,
               RigidIpcBarrierAssembly::npos,
               RigidIpcBarrierAssembly::npos},
              reduced);
        }
      }

      for (std::size_t pointA = 0; pointA < a.vertices.size(); ++pointA) {
        for (const SurfaceEdge& edgeB : scratch.pairs.surfaceEdgesAt(bodyB)) {
          const auto reduced = rigidIpcPointEdgeReducedBarrier(
              a.vertices[pointA],
              a.pose,
              b.vertices[edgeB.vertexA],
              b.vertices[edgeB.vertexB],
              b.pose,
              options);
          addActiveConstraint(
              assembly,
              triplets,
              RigidIpcBarrierPrimitive::EdgeVertex,
              bodyA,
              bodyB,
              {pointA,
               edgeB.vertexA,
               edgeB.vertexB,
               RigidIpcBarrierAssembly::npos},
              reduced);
        }
      }
      for (std::size_t pointB = 0; pointB < b.vertices.size(); ++pointB) {
        for (const SurfaceEdge& edgeA : scratch.pairs.surfaceEdgesAt(bodyA)) {
          const auto reduced = rigidIpcPointEdgeReducedBarrier(
              b.vertices[pointB],
              b.pose,
              a.vertices[edgeA.vertexA],
              a.vertices[edgeA.vertexB],
              a.pose,
              options);
          addActiveConstraint(
              assembly,
              triplets,
              RigidIpcBarrierPrimitive::EdgeVertex,
              bodyB,
              bodyA,
              {pointB,
               edgeA.vertexA,
               edgeA.vertexB,
               RigidIpcBarrierAssembly::npos},
              reduced);
        }
      }

      for (const SurfaceEdge& edgeA : scratch.pairs.surfaceEdgesAt(bodyA)) {
        for (const SurfaceEdge& edgeB : scratch.pairs.surfaceEdgesAt(bodyB)) {
          const auto reduced = rigidIpcEdgeEdgeReducedBarrier(
              a.vertices[edgeA.vertexA],
              a.vertices[edgeA.vertexB],
              a.pose,
              b.vertices[edgeB.vertexA],
              b.vertices[edgeB.vertexB],
              b.pose,
              options);
          addActiveConstraint(
              assembly,
              triplets,
              RigidIpcBarrierPrimitive::EdgeEdge,
              bodyA,
              bodyB,
              {edgeA.vertexA, edgeA.vertexB, edgeB.vertexA, edgeB.vertexB},
              reduced);
        }
      }

      for (std::size_t pointA = 0; pointA < a.vertices.size(); ++pointA) {
        for (const Eigen::Vector3i& triangleB : b.triangles) {
          const std::size_t triangleB0
              = checkedTriangleVertex(triangleB, 0, b.vertices.size());
          const std::size_t triangleB1
              = checkedTriangleVertex(triangleB, 1, b.vertices.size());
          const std::size_t triangleB2
              = checkedTriangleVertex(triangleB, 2, b.vertices.size());
          const auto reduced = rigidIpcPointTriangleReducedBarrier(
              a.vertices[pointA],
              a.pose,
              b.vertices[triangleB0],
              b.vertices[triangleB1],
              b.vertices[triangleB2],
              b.pose,
              options);
          addActiveConstraint(
              assembly,
              triplets,
              RigidIpcBarrierPrimitive::FaceVertex,
              bodyA,
              bodyB,
              {pointA, triangleB0, triangleB1, triangleB2},
              reduced);
        }
      }
      for (std::size_t pointB = 0; pointB < b.vertices.size(); ++pointB) {
        for (const Eigen::Vector3i& triangleA : a.triangles) {
          const std::size_t triangleA0
              = checkedTriangleVertex(triangleA, 0, a.vertices.size());
          const std::size_t triangleA1
              = checkedTriangleVertex(triangleA, 1, a.vertices.size());
          const std::size_t triangleA2
              = checkedTriangleVertex(triangleA, 2, a.vertices.size());
          const auto reduced = rigidIpcPointTriangleReducedBarrier(
              b.vertices[pointB],
              b.pose,
              a.vertices[triangleA0],
              a.vertices[triangleA1],
              a.vertices[triangleA2],
              a.pose,
              options);
          addActiveConstraint(
              assembly,
              triplets,
              RigidIpcBarrierPrimitive::FaceVertex,
              bodyB,
              bodyA,
              {pointB, triangleA0, triangleA1, triangleA2},
              reduced);
        }
      }
    }
  }

  if (finalizeHessian) {
    assembleSparseFromTripletsReusingPattern(
        assembly.hessian,
        assembly.hessian.rows(),
        assembly.hessian.cols(),
        triplets);
  }
}

RigidIpcBarrierAssembly assembleRigidIpcObjectiveSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    const RigidIpcBarrierOptions& options)
{
  RigidIpcAssemblyScratch scratch;
  return assembleRigidIpcObjectiveSystemWithScratch(
      surfaces,
      std::span<const RigidIpcBarrierSurface>{},
      dynamicsTerms,
      std::span<const RigidIpcArticulationConstraintInput>{},
      options,
      RigidIpcFrictionOptions{},
      scratch);
}

RigidIpcBarrierAssembly assembleRigidIpcObjectiveSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions)
{
  RigidIpcAssemblyScratch scratch;
  return assembleRigidIpcObjectiveSystemWithScratch(
      surfaces,
      laggedSurfaces,
      dynamicsTerms,
      std::span<const RigidIpcArticulationConstraintInput>{},
      barrierOptions,
      frictionOptions,
      scratch);
}

RigidIpcBarrierAssembly assembleRigidIpcObjectiveSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions)
{
  RigidIpcAssemblyScratch scratch;
  return assembleRigidIpcObjectiveSystemWithScratch(
      surfaces,
      laggedSurfaces,
      dynamicsTerms,
      articulationConstraints,
      barrierOptions,
      frictionOptions,
      scratch);
}

static RigidIpcBarrierAssembly assembleRigidIpcObjectiveSystemWithScratch(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions,
    RigidIpcAssemblyScratch& scratch)
{
  RigidIpcBarrierAssembly assembly(
      allocatorOrDefault(scratch.pairs.memoryAllocator));
  assembleRigidIpcObjectiveSystemWithScratchInto(
      surfaces,
      laggedSurfaces,
      dynamicsTerms,
      articulationConstraints,
      barrierOptions,
      frictionOptions,
      scratch,
      assembly);
  return assembly;
}

static void assembleRigidIpcObjectiveSystemWithScratchInto(
    std::span<const RigidIpcBarrierSurface> surfaces,
    std::span<const RigidIpcBarrierSurface> laggedSurfaces,
    std::span<const RigidIpcBodyDynamicsTerm> dynamicsTerms,
    std::span<const RigidIpcArticulationConstraintInput>
        articulationConstraints,
    const RigidIpcBarrierOptions& barrierOptions,
    const RigidIpcFrictionOptions& frictionOptions,
    RigidIpcAssemblyScratch& scratch,
    RigidIpcBarrierAssembly& assembly)
{
  const bool hasArticulationConstraints = !articulationConstraints.empty();
  assembleRigidIpcBarrierSystemWithScratchInto(
      surfaces,
      barrierOptions,
      scratch,
      assembly,
      false,
      hasArticulationConstraints);
  assert(dynamicsTerms.size() <= surfaces.size());

  const bool useLaggedFriction
      = laggedSurfaces.size() == surfaces.size()
        && frictionOptions.coefficient > 0.0
        && std::isfinite(frictionOptions.coefficient)
        && frictionOptions.staticFrictionDisplacement > 0.0
        && std::isfinite(frictionOptions.staticFrictionDisplacement);
  auto& laggedAssembly = scratch.laggedAssembly;
  if (useLaggedFriction) {
    assembleRigidIpcBarrierSystemWithScratchInto(
        laggedSurfaces, barrierOptions, scratch, laggedAssembly);
  }

  const std::size_t termCount = std::min(dynamicsTerms.size(), surfaces.size());
  auto& triplets = scratch.barrierTriplets;
  triplets.reserve(triplets.size() + 6 * termCount);
  if (useLaggedFriction) {
    addLaggedFrictionTerms(
        assembly,
        triplets,
        surfaces,
        laggedSurfaces,
        frictionOptions,
        laggedAssembly);
  }
  for (std::size_t body = 0; body < termCount; ++body) {
    addBodyDynamicsTerm(
        assembly, triplets, body, surfaces[body], dynamicsTerms[body]);
  }
  addArticulationConstraintRows(
      assembly,
      surfaces,
      articulationConstraints,
      scratch.articulationResiduals,
      scratch.articulationJacobianTriplets);

  assembleSparseFromTripletsReusingPattern(
      assembly.hessian,
      assembly.hessian.rows(),
      assembly.hessian.cols(),
      triplets);
}

RigidIpcLineSearchResult computeRigidIpcLineSearchStepBound(
    std::span<const RigidIpcBarrierSurface> startSurfaces,
    std::span<const RigidIpcBarrierSurface> endSurfaces,
    const RigidIpcLineSearchOptions& options)
{
  RigidIpcLineSearchScratch scratch;
  return computeRigidIpcLineSearchStepBoundWithScratch(
      startSurfaces, endSurfaces, options, scratch);
}

static RigidIpcLineSearchResult computeRigidIpcLineSearchStepBoundWithScratch(
    std::span<const RigidIpcBarrierSurface> startSurfaces,
    std::span<const RigidIpcBarrierSurface> endSurfaces,
    const RigidIpcLineSearchOptions& options,
    RigidIpcLineSearchScratch& scratch)
{
  assert(startSurfaces.size() == endSurfaces.size());

  RigidIpcLineSearchResult result;
  const collision::native::CcdOption ccdOption
      = newton_barrier::makeLineSearchCcdOption(options);

  scratch.pairs.resizeSurfaceEdges(startSurfaces.size());
  auto& startAabbs = scratch.pairs.surfaceAabbs;
  startAabbs.clear();
  startAabbs.reserve(startSurfaces.size());
  auto& motionBounds = scratch.pairs.motionBounds;
  motionBounds.clear();
  motionBounds.reserve(startSurfaces.size());
  for (std::size_t body = 0; body < startSurfaces.size(); ++body) {
    assertMatchingSurfaceTopology(startSurfaces[body], endSurfaces[body]);
    buildSurfaceEdges(startSurfaces[body], scratch.pairs.surfaceEdgesAt(body));
    startAabbs.push_back(computeSurfaceWorldAabb(startSurfaces[body]));
    motionBounds.push_back(
        surfaceMotionBound(startSurfaces[body], endSurfaces[body]));
  }

  const double minSeparation = std::max(0.0, options.minSeparation);
  // curvedAccdAdvance reports a hit once the clearance reaches convergeAbs, so
  // the effective contact threshold is minSeparation + convergeAbs. The cull
  // reach must include this term or it could skip a pair whose true distance
  // lands within the tolerance band of the activation threshold.
  const double convergeAbs = std::max(ccdOption.tolerance, 1e-12);

  auto& surfaceMargins = scratch.pairs.surfaceMargins;
  surfaceMargins.resize(startSurfaces.size());
  for (std::size_t body = 0; body < startSurfaces.size(); ++body) {
    surfaceMargins[body]
        = motionBounds[body] + 0.5 * (minSeparation + convergeAbs);
  }
  collectCandidateSurfacePairs(
      startAabbs,
      surfaceMargins,
      scratch.pairs.sweepItems,
      scratch.pairs.candidatePairs);
  const auto& candidatePairs = scratch.pairs.candidatePairs;
  for (const auto& candidatePair : candidatePairs) {
    const std::size_t bodyA = candidatePair.first;
    const std::size_t bodyB = candidatePair.second;
    const RigidIpcBarrierSurface& a0 = startSurfaces[bodyA];
    const RigidIpcBarrierSurface& a1 = endSurfaces[bodyA];
    {
      const RigidIpcBarrierSurface& b0 = startSurfaces[bodyB];
      const RigidIpcBarrierSurface& b1 = endSurfaces[bodyB];
      if (!a0.dynamic && !b0.dynamic) {
        continue;
      }

      // Exact swept broad-phase cull on the sweep candidates. Each surface
      // stays within its motion bound of its start AABB over the whole step, so
      // if the start AABBs are farther apart than the combined motion bounds
      // plus the minimum separation and CCD convergence tolerance, no primitive
      // pair can reach contact. This reuses the same curved-trajectory speed
      // bound as the per-primitive CCD, so it never skips a pair the CCD below
      // would limit.
      if (startAabbs[bodyA].valid && startAabbs[bodyB].valid) {
        const double reach = motionBounds[bodyA] + motionBounds[bodyB]
                             + minSeparation + convergeAbs;
        if (surfaceAabbSquaredDistance(startAabbs[bodyA], startAabbs[bodyB])
            > reach * reach) {
          continue;
        }
      }

      for (std::size_t pointA = 0; pointA < a0.vertices.size(); ++pointA) {
        for (std::size_t pointB = 0; pointB < b0.vertices.size(); ++pointB) {
          collision::native::CcdPrimitiveResult ccdResult;
          ++result.stats.pointPointChecks;
          const bool hit = rigidIpcPointPointCcd(
              a0.vertices[pointA],
              a0.pose,
              a1.pose,
              b0.vertices[pointB],
              b0.pose,
              b1.pose,
              ccdOption,
              ccdResult);
          static_cast<void>(hit);
          recordLineSearchCandidate(
              result,
              ccdResult,
              RigidIpcBarrierPrimitive::VertexVertex,
              bodyA,
              bodyB,
              {pointA,
               pointB,
               RigidIpcBarrierAssembly::npos,
               RigidIpcBarrierAssembly::npos});
        }
      }

      for (std::size_t pointA = 0; pointA < a0.vertices.size(); ++pointA) {
        for (const SurfaceEdge& edgeB : scratch.pairs.surfaceEdgesAt(bodyB)) {
          collision::native::CcdPrimitiveResult ccdResult;
          ++result.stats.pointEdgeChecks;
          const bool hit = rigidIpcPointEdgeCcd(
              a0.vertices[pointA],
              a0.pose,
              a1.pose,
              b0.vertices[edgeB.vertexA],
              b0.vertices[edgeB.vertexB],
              b0.pose,
              b1.pose,
              ccdOption,
              ccdResult);
          static_cast<void>(hit);
          recordLineSearchCandidate(
              result,
              ccdResult,
              RigidIpcBarrierPrimitive::EdgeVertex,
              bodyA,
              bodyB,
              {pointA,
               edgeB.vertexA,
               edgeB.vertexB,
               RigidIpcBarrierAssembly::npos});
        }
      }
      for (std::size_t pointB = 0; pointB < b0.vertices.size(); ++pointB) {
        for (const SurfaceEdge& edgeA : scratch.pairs.surfaceEdgesAt(bodyA)) {
          collision::native::CcdPrimitiveResult ccdResult;
          ++result.stats.pointEdgeChecks;
          const bool hit = rigidIpcPointEdgeCcd(
              b0.vertices[pointB],
              b0.pose,
              b1.pose,
              a0.vertices[edgeA.vertexA],
              a0.vertices[edgeA.vertexB],
              a0.pose,
              a1.pose,
              ccdOption,
              ccdResult);
          static_cast<void>(hit);
          recordLineSearchCandidate(
              result,
              ccdResult,
              RigidIpcBarrierPrimitive::EdgeVertex,
              bodyB,
              bodyA,
              {pointB,
               edgeA.vertexA,
               edgeA.vertexB,
               RigidIpcBarrierAssembly::npos});
        }
      }

      for (const SurfaceEdge& edgeA : scratch.pairs.surfaceEdgesAt(bodyA)) {
        for (const SurfaceEdge& edgeB : scratch.pairs.surfaceEdgesAt(bodyB)) {
          collision::native::CcdPrimitiveResult ccdResult;
          ++result.stats.edgeEdgeChecks;
          const bool hit = rigidIpcEdgeEdgeCcd(
              a0.vertices[edgeA.vertexA],
              a0.vertices[edgeA.vertexB],
              a0.pose,
              a1.pose,
              b0.vertices[edgeB.vertexA],
              b0.vertices[edgeB.vertexB],
              b0.pose,
              b1.pose,
              ccdOption,
              ccdResult);
          static_cast<void>(hit);
          recordLineSearchCandidate(
              result,
              ccdResult,
              RigidIpcBarrierPrimitive::EdgeEdge,
              bodyA,
              bodyB,
              {edgeA.vertexA, edgeA.vertexB, edgeB.vertexA, edgeB.vertexB});
        }
      }

      for (std::size_t pointA = 0; pointA < a0.vertices.size(); ++pointA) {
        for (const Eigen::Vector3i& triangleB : b0.triangles) {
          const std::size_t triangleB0
              = checkedTriangleVertex(triangleB, 0, b0.vertices.size());
          const std::size_t triangleB1
              = checkedTriangleVertex(triangleB, 1, b0.vertices.size());
          const std::size_t triangleB2
              = checkedTriangleVertex(triangleB, 2, b0.vertices.size());
          collision::native::CcdPrimitiveResult ccdResult;
          ++result.stats.pointTriangleChecks;
          const bool hit = rigidIpcPointTriangleCcd(
              a0.vertices[pointA],
              a0.pose,
              a1.pose,
              b0.vertices[triangleB0],
              b0.vertices[triangleB1],
              b0.vertices[triangleB2],
              b0.pose,
              b1.pose,
              ccdOption,
              ccdResult);
          static_cast<void>(hit);
          recordLineSearchCandidate(
              result,
              ccdResult,
              RigidIpcBarrierPrimitive::FaceVertex,
              bodyA,
              bodyB,
              {pointA, triangleB0, triangleB1, triangleB2});
        }
      }
      for (std::size_t pointB = 0; pointB < b0.vertices.size(); ++pointB) {
        for (const Eigen::Vector3i& triangleA : a0.triangles) {
          const std::size_t triangleA0
              = checkedTriangleVertex(triangleA, 0, a0.vertices.size());
          const std::size_t triangleA1
              = checkedTriangleVertex(triangleA, 1, a0.vertices.size());
          const std::size_t triangleA2
              = checkedTriangleVertex(triangleA, 2, a0.vertices.size());
          collision::native::CcdPrimitiveResult ccdResult;
          ++result.stats.pointTriangleChecks;
          const bool hit = rigidIpcPointTriangleCcd(
              b0.vertices[pointB],
              b0.pose,
              b1.pose,
              a0.vertices[triangleA0],
              a0.vertices[triangleA1],
              a0.vertices[triangleA2],
              a0.pose,
              a1.pose,
              ccdOption,
              ccdResult);
          static_cast<void>(hit);
          recordLineSearchCandidate(
              result,
              ccdResult,
              RigidIpcBarrierPrimitive::FaceVertex,
              bodyB,
              bodyA,
              {pointB, triangleA0, triangleA1, triangleA2});
        }
      }
    }
  }

  return result;
}

RigidIpcProjectedNewtonStep computeRigidIpcProjectedNewtonStep(
    const RigidIpcBarrierAssembly& assembly,
    const RigidIpcProjectedNewtonOptions& options)
{
  return computeRigidIpcProjectedNewtonStepImpl(assembly, nullptr, options);
}

RigidIpcProjectedNewtonStep computeRigidIpcProjectedNewtonStep(
    const RigidIpcBarrierAssembly& assembly,
    const RigidIpcLineSearchResult& lineSearch,
    const RigidIpcProjectedNewtonOptions& options)
{
  return computeRigidIpcProjectedNewtonStepImpl(assembly, &lineSearch, options);
}

double computeInitialRigidIpcBarrierStiffness(
    const double bboxDiagonal,
    const double squaredActivationDistance,
    const double averageMass,
    const Eigen::VectorXd& gradEnergy,
    const Eigen::VectorXd& gradBarrier,
    const double minStiffnessScale,
    double& maxStiffness)
{
  maxStiffness = std::numeric_limits<double>::infinity();
  if (!(bboxDiagonal > 0.0) || !std::isfinite(bboxDiagonal)
      || !(averageMass > 0.0) || !std::isfinite(averageMass)
      || !(minStiffnessScale > 0.0) || !std::isfinite(minStiffnessScale)
      || !(squaredActivationDistance > 0.0)
      || !std::isfinite(squaredActivationDistance)) {
    return 1.0;
  }

  // Reference `initial_barrier_stiffness` with dmin = 0, evaluated on DART's
  // squared-distance clamped-log barrier. `d0` is a tiny probe distance; the
  // second derivative is taken with respect to squared distance.
  const double d0 = 1e-8 * bboxDiagonal;
  double d0Squared = d0 * d0;
  // Reference clamp (adaptive_stiffness.cpp, dmin = 0 specialization): if the
  // probe distance lands outside the activation band the barrier is inactive
  // there (second derivative 0), so pull it to an interior point. Without this
  // a grossly mis-scaled world (bbox diagonal >= 1e6 * dhat) would otherwise
  // fall back to kappa = 1.
  if (d0Squared >= squaredActivationDistance) {
    d0Squared = 0.5 * squaredActivationDistance;
  }
  const auto probe = newton_barrier::c2ClampedLogBarrier(
      d0Squared, squaredActivationDistance);
  const double minStiffnessRaw = 4.0 * d0Squared * probe.secondDerivative;
  if (!(minStiffnessRaw > 0.0) || !std::isfinite(minStiffnessRaw)) {
    return 1.0;
  }

  const double minStiffness = minStiffnessScale * averageMass / minStiffnessRaw;
  if (!(minStiffness > 0.0) || !std::isfinite(minStiffness)) {
    return 1.0;
  }
  maxStiffness = 100.0 * minStiffness;

  double kappa = 1.0;
  const double gradBarrierSquaredNorm = gradBarrier.squaredNorm();
  if (gradBarrierSquaredNorm > 0.0 && gradBarrier.size() == gradEnergy.size()) {
    // If negative, the clamp below pins it to kappa_min anyway.
    kappa = -gradBarrier.dot(gradEnergy) / gradBarrierSquaredNorm;
  }
  if (!std::isfinite(kappa)) {
    kappa = minStiffness;
  }

  return std::clamp(kappa, minStiffness, maxStiffness);
}

double updateRigidIpcBarrierStiffness(
    const double prevMinSquaredDistance,
    const double minSquaredDistance,
    const double maxStiffness,
    const double currentStiffness,
    const double bboxDiagonal,
    const double dhatEpsilonScale)
{
  if (!std::isfinite(currentStiffness) || !std::isfinite(maxStiffness)
      || !std::isfinite(bboxDiagonal) || !std::isfinite(dhatEpsilonScale)
      || !std::isfinite(minSquaredDistance)
      || !std::isfinite(prevMinSquaredDistance)) {
    return currentStiffness;
  }

  // Is the barrier having a difficulty pushing the bodies apart?
  double dhatEpsilon = dhatEpsilonScale * bboxDiagonal;
  dhatEpsilon *= dhatEpsilon;
  if (prevMinSquaredDistance < dhatEpsilon && minSquaredDistance < dhatEpsilon
      && minSquaredDistance < prevMinSquaredDistance) {
    return std::min(maxStiffness, 2.0 * currentStiffness);
  }
  return currentStiffness;
}

namespace {

// Smallest active-constraint squared distance in an assembly, or +inf if none.
double minActiveSquaredDistance(const RigidIpcBarrierAssembly& assembly)
{
  double minSquaredDistance = std::numeric_limits<double>::infinity();
  for (const auto& constraint : assembly.activeConstraints) {
    minSquaredDistance = std::min(
        minSquaredDistance, constraint.reduced.primitive.squaredDistance);
  }
  return minSquaredDistance;
}

} // namespace

RigidIpcProjectedNewtonSolveScratch::RigidIpcProjectedNewtonSolveScratch(
    dart::common::MemoryAllocator& allocator)
  : memoryAllocator(&allocator),
    laggedSurfaces(SurfaceAllocator{allocator}),
    lineSearchStartSurfaces(SurfaceAllocator{allocator}),
    candidateSurfaces(SurfaceAllocator{allocator}),
    acceptedSurfaces(SurfaceAllocator{allocator}),
    bestDecreasingSurfaces(SurfaceAllocator{allocator}),
    step(allocator),
    workspace(allocator.construct<RigidIpcProjectedNewtonSolveScratchWorkspace>(
        &allocator))
{
}

RigidIpcProjectedNewtonSolveScratch::~RigidIpcProjectedNewtonSolveScratch()
{
  if (memoryAllocator != nullptr && workspace != nullptr) {
    memoryAllocator->destroy(workspace);
  }
}

RigidIpcProjectedNewtonSolveScratch::RigidIpcProjectedNewtonSolveScratch(
    RigidIpcProjectedNewtonSolveScratch&& other) noexcept
  : memoryAllocator(other.memoryAllocator),
    laggedSurfaces(std::move(other.laggedSurfaces)),
    lineSearchStartSurfaces(std::move(other.lineSearchStartSurfaces)),
    candidateSurfaces(std::move(other.candidateSurfaces)),
    acceptedSurfaces(std::move(other.acceptedSurfaces)),
    bestDecreasingSurfaces(std::move(other.bestDecreasingSurfaces)),
    step(std::move(other.step)),
    workspace(std::exchange(other.workspace, nullptr))
{
  other.memoryAllocator = nullptr;
}

RigidIpcProjectedNewtonSolveScratch&
RigidIpcProjectedNewtonSolveScratch::operator=(
    RigidIpcProjectedNewtonSolveScratch&& other) noexcept
{
  if (this == &other) {
    return *this;
  }

  if (memoryAllocator != nullptr && workspace != nullptr) {
    memoryAllocator->destroy(workspace);
  }

  memoryAllocator = other.memoryAllocator;
  laggedSurfaces = std::move(other.laggedSurfaces);
  lineSearchStartSurfaces = std::move(other.lineSearchStartSurfaces);
  candidateSurfaces = std::move(other.candidateSurfaces);
  acceptedSurfaces = std::move(other.acceptedSurfaces);
  bestDecreasingSurfaces = std::move(other.bestDecreasingSurfaces);
  step = std::move(other.step);
  workspace = std::exchange(other.workspace, nullptr);
  other.memoryAllocator = nullptr;
  return *this;
}

void reserveRigidIpcProjectedNewtonSolveScratchForSameShape(
    std::span<const RigidIpcBarrierSurface> surfaces,
    RigidIpcProjectedNewtonSolveResult& result,
    RigidIpcProjectedNewtonSolveScratch& scratch)
{
  Eigen::Index dofs = 0;
  for (const RigidIpcBarrierSurface& surface : surfaces) {
    if (surface.dynamic) {
      dofs += 6;
    }
  }
  const Eigen::Index equalityRows = result.assembly.equalityResidual.size();

  std::vector<BarrierTriplet, BarrierTripletAllocator> localTriplets{
      BarrierTripletAllocator{allocatorOrDefault(scratch.memoryAllocator)}};
  auto& triplets = scratch.workspace != nullptr
                       ? scratch.workspace->assemblyScratch.barrierTriplets
                       : localTriplets;
  reserveRigidIpcAssemblySparsePatterns(
      result.assembly, dofs, equalityRows, triplets);

  if (scratch.workspace == nullptr) {
    return;
  }

  auto& workspace = *scratch.workspace;
  reserveRigidIpcAssemblySparsePatterns(
      workspace.assemblyScratch.energyAssembly, dofs, equalityRows, triplets);
  reserveRigidIpcAssemblySparsePatterns(
      workspace.assemblyScratch.unitBarrierAssembly,
      dofs,
      equalityRows,
      triplets);
  reserveRigidIpcAssemblySparsePatterns(
      workspace.assemblyScratch.laggedAssembly, dofs, equalityRows, triplets);
  reserveRigidIpcAssemblySparsePatterns(
      workspace.assemblyScratch.candidateAssembly,
      dofs,
      equalityRows,
      triplets);

  workspace.denseHessian.resize(dofs, dofs);
  workspace.denseHessian.setIdentity();
  workspace.denseHessianLdlt.compute(workspace.denseHessian);
  workspace.denseHessian.setZero();
  workspace.rawStep.resize(dofs);

  if (equalityRows > 0) {
    workspace.equalityDenseJacobian.resize(equalityRows, dofs);
    workspace.equalityDenseJacobian.setZero();
    const Eigen::Index kktSize = dofs + equalityRows;
    workspace.equalityKktMatrix.resize(kktSize, kktSize);
    workspace.equalityKktMatrix.setIdentity();
    workspace.equalityKktRhs.resize(kktSize);
    workspace.equalityKktSolution.resize(kktSize);
    workspace.equalityKktMatrix.setZero();
  }
}

RigidIpcProjectedNewtonSolveResult solveRigidIpcProjectedNewtonBarrierSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcProjectedNewtonSolveOptions& options)
{
  RigidIpcProjectedNewtonSolveResult result;
  RigidIpcProjectedNewtonSolveScratch scratch;
  solveRigidIpcProjectedNewtonBarrierSystem(surfaces, options, result, scratch);
  return result;
}

void solveRigidIpcProjectedNewtonBarrierSystem(
    std::span<const RigidIpcBarrierSurface> surfaces,
    const RigidIpcProjectedNewtonSolveOptions& options,
    RigidIpcProjectedNewtonSolveResult& result,
    RigidIpcProjectedNewtonSolveScratch& scratch)
{
  result.status = RigidIpcProjectedNewtonSolveStatus::MaxIterations;
  result.converged = false;
  result.failed = false;
  result.assembly.value = 0.0;
  result.assembly.activeDynamicsTerms = 0u;
  result.assembly.activeConstraints.clear();
  result.assembly.activeFrictionConstraints.clear();
  result.assembly.activeArticulationConstraints.clear();
  result.lineSearch = RigidIpcLineSearchResult{};
  result.lastStep.status = RigidIpcProjectedNewtonStatus::FactorizationFailed;
  result.lastStep.success = false;
  result.lastStep.converged = false;
  result.lastStep.lineSearchBlocked = false;
  result.lastStep.stats = RigidIpcProjectedNewtonStats{};
  result.stats = RigidIpcProjectedNewtonSolveStats{};
  assignSurfacesPreservingStorage(
      result.surfaces, surfaces, scratch.memoryAllocator);

  auto& laggedSurfaces = scratch.laggedSurfaces;
  auto& lineSearchStartSurfaces = scratch.lineSearchStartSurfaces;
  auto& candidateSurfaces = scratch.candidateSurfaces;
  auto& acceptedSurfaces = scratch.acceptedSurfaces;
  auto& bestDecreasingSurfaces = scratch.bestDecreasingSurfaces;
  assignSurfacesPreservingStorage(
      laggedSurfaces, surfaces, scratch.memoryAllocator);
  lineSearchStartSurfaces.reserve(surfaces.size());
  candidateSurfaces.reserve(surfaces.size());
  acceptedSurfaces.reserve(surfaces.size());
  bestDecreasingSurfaces.reserve(surfaces.size());
  std::optional<RigidIpcAssemblyScratch> localAssemblyScratch;
  std::optional<RigidIpcLineSearchScratch> localLineSearchScratch;
  if (scratch.workspace == nullptr) {
    localAssemblyScratch.emplace(scratch.memoryAllocator);
    localLineSearchScratch.emplace(scratch.memoryAllocator);
  }
  RigidIpcAssemblyScratch& assemblyScratch
      = scratch.workspace != nullptr ? scratch.workspace->assemblyScratch
                                     : *localAssemblyScratch;
  RigidIpcLineSearchScratch& lineSearchScratch
      = scratch.workspace != nullptr ? scratch.workspace->lineSearchScratch
                                     : *localLineSearchScratch;

  // Kinematic (prescribed-motion) obstacles advance from their start pose to
  // their end pose over the step. `result.surfaces`/`laggedSurfaces` hold the
  // END pose (the implicit configuration the barrier and dynamics see); the lag
  // used by lagged friction is overridden to the START pose so a moving
  // obstacle drags contacting dynamic bodies, and the line search below sweeps
  // the start->end motion. Every override is gated on `hasKinematic`, so scenes
  // with no kinematic obstacle take the exact original code path.
  bool hasKinematic = false;
  for (const auto& surface : surfaces) {
    if (surface.kinematic) {
      hasKinematic = true;
      break;
    }
  }
  const auto applyKinematicLag = [&surfaces](auto& lag) {
    for (std::size_t i = 0; i < lag.size(); ++i) {
      if (surfaces[i].kinematic) {
        lag[i].pose = surfaces[i].kinematicStartPose;
      }
    }
  };
  if (hasKinematic) {
    applyKinematicLag(laggedSurfaces);
  }

  const double stepTolerance = std::max(0.0, options.stepTolerance);
  const double frictionConvergenceTolerance
      = std::max(0.0, options.frictionConvergenceTolerance);

  RigidIpcFrictionOptions frictionOptions = options.friction;
  const bool useLaggedFriction
      = options.frictionIterations > 0u && frictionOptions.coefficient > 0.0
        && std::isfinite(frictionOptions.coefficient)
        && frictionOptions.staticFrictionDisplacement > 0.0
        && std::isfinite(frictionOptions.staticFrictionDisplacement);
  if (!useLaggedFriction) {
    frictionOptions.coefficient = 0.0;
    frictionOptions.laggedNormalForce = 0.0;
  }
  const std::size_t frictionIterationCount
      = useLaggedFriction ? options.frictionIterations : 1u;

  // Barrier stiffness (kappa) used by every assembly in this solve. With
  // adaptive stiffness disabled this stays the caller-provided fixed value;
  // otherwise it is initialized from the IPC adaptive-kappa scheme and may be
  // increased per-iteration when the barrier struggles to separate.
  RigidIpcBarrierOptions barrierOptions = options.barrier;
  const RigidIpcAdaptiveStiffnessOptions& adaptive = options.adaptiveStiffness;
  double maxBarrierStiffness = std::numeric_limits<double>::infinity();
  double prevMinSquaredDistance = std::numeric_limits<double>::infinity();
  if (adaptive.enabled) {
    // Separate the inertial-energy gradient from the unit-barrier gradient at
    // the start-of-step configuration (both evaluated frictionless, matching
    // the reference `init_solve`). The barrier gradient is linear in kappa for
    // a fixed configuration, so the unit-barrier gradient is the difference.
    RigidIpcBarrierOptions energyOnly = options.barrier;
    energyOnly.stiffness = 0.0;
    RigidIpcBarrierOptions unitBarrier = options.barrier;
    unitBarrier.stiffness = 1.0;
    const RigidIpcFrictionOptions noFriction;
    auto& energyAssembly = assemblyScratch.energyAssembly;
    assembleRigidIpcObjectiveSystemWithScratchInto(
        result.surfaces,
        laggedSurfaces,
        options.dynamicsTerms,
        options.articulationConstraints,
        energyOnly,
        noFriction,
        assemblyScratch,
        energyAssembly);
    auto& unitBarrierAssembly = assemblyScratch.unitBarrierAssembly;
    assembleRigidIpcObjectiveSystemWithScratchInto(
        result.surfaces,
        laggedSurfaces,
        options.dynamicsTerms,
        options.articulationConstraints,
        unitBarrier,
        noFriction,
        assemblyScratch,
        unitBarrierAssembly);
    auto& gradBarrier = assemblyScratch.gradBarrier;
    gradBarrier.resize(unitBarrierAssembly.gradient.size());
    gradBarrier = unitBarrierAssembly.gradient;
    if (gradBarrier.size() == energyAssembly.gradient.size()) {
      gradBarrier -= energyAssembly.gradient;
    }
    barrierOptions.stiffness = computeInitialRigidIpcBarrierStiffness(
        adaptive.bboxDiagonal,
        options.barrier.squaredActivationDistance,
        adaptive.averageMass,
        energyAssembly.gradient,
        gradBarrier,
        adaptive.minStiffnessScale,
        maxBarrierStiffness);
    if (std::isfinite(options.barrier.stiffness)
        && options.barrier.stiffness > barrierOptions.stiffness) {
      barrierOptions.stiffness
          = std::min(options.barrier.stiffness, maxBarrierStiffness);
    }
    prevMinSquaredDistance = minActiveSquaredDistance(unitBarrierAssembly);
  }
  result.stats.barrierStiffness = barrierOptions.stiffness;

  // Newton options with an effective gradient tolerance that includes the
  // relative floor (filled once the initial gradient norm is known below).
  RigidIpcProjectedNewtonOptions newtonOptions = options.newton;

  const auto makeKinematicLineSearchStart = [&]() {
    assignSurfacesPreservingStorage(
        lineSearchStartSurfaces, result.surfaces, scratch.memoryAllocator);
    for (std::size_t i = 0; i < lineSearchStartSurfaces.size(); ++i) {
      if (surfaces[i].kinematic) {
        lineSearchStartSurfaces[i].pose = surfaces[i].kinematicStartPose;
      }
    }
    return lineSearchStartSurfaces;
  };
  const auto checkConvergedKinematicSweep = [&]() {
    if (!hasKinematic || !options.useLineSearch) {
      return true;
    }
    result.lineSearch = computeRigidIpcLineSearchStepBoundWithScratch(
        makeKinematicLineSearchStart(),
        result.surfaces,
        options.lineSearch,
        lineSearchScratch);
    recordSolveLineSearchStats(result);
    if (result.lineSearch.limited) {
      ++result.stats.lineSearchLimitedSteps;
    }
    if (!result.lineSearch.allowsFullStep()) {
      result.status = RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked;
      result.failed = true;
      return false;
    }
    return true;
  };
  const auto kinematicLineSearchLimitBlocksAcceptedState
      = [&](const RigidIpcLineSearchResult& lineSearch) {
          if (!lineSearch.limited || !(lineSearch.stepBound < 1.0)) {
            return false;
          }
          const bool bodyAIsKinematic = lineSearch.bodyA < surfaces.size()
                                        && surfaces[lineSearch.bodyA].kinematic;
          const bool bodyBIsKinematic = lineSearch.bodyB < surfaces.size()
                                        && surfaces[lineSearch.bodyB].kinematic;
          return bodyAIsKinematic || bodyBIsKinematic;
        };
  const auto kinematicCandidateAllowsFullStep
      = [&](const auto& candidateSurfaces) {
          if (!hasKinematic || !options.useLineSearch) {
            return true;
          }
          result.lineSearch = computeRigidIpcLineSearchStepBoundWithScratch(
              makeKinematicLineSearchStart(),
              candidateSurfaces,
              options.lineSearch,
              lineSearchScratch);
          recordSolveLineSearchStats(result);
          if (result.lineSearch.limited) {
            ++result.stats.lineSearchLimitedSteps;
          }
          return result.lineSearch.allowsFullStep();
        };

  for (std::size_t frictionIteration = 0;
       frictionIteration < frictionIterationCount;
       ++frictionIteration) {
    result.converged = false;

    for (std::size_t iteration = 0; iteration <= options.maxIterations;
         ++iteration) {
      assembleRigidIpcObjectiveSystemWithScratchInto(
          result.surfaces,
          laggedSurfaces,
          options.dynamicsTerms,
          options.articulationConstraints,
          barrierOptions,
          frictionOptions,
          assemblyScratch,
          result.assembly);
      if (adaptive.enabled) {
        // Post-(previous-)step adaptive kappa update. The current assembly
        // reflects the surfaces produced by the prior accepted step, so its
        // closest pair is the post-step minimum distance.
        const double minSquaredDistance
            = minActiveSquaredDistance(result.assembly);
        const double updatedStiffness = updateRigidIpcBarrierStiffness(
            prevMinSquaredDistance,
            minSquaredDistance,
            maxBarrierStiffness,
            barrierOptions.stiffness,
            adaptive.bboxDiagonal,
            adaptive.dhatEpsilonScale);
        prevMinSquaredDistance = minSquaredDistance;
        if (updatedStiffness != barrierOptions.stiffness) {
          barrierOptions.stiffness = updatedStiffness;
          result.stats.barrierStiffness = updatedStiffness;
          ++result.stats.barrierStiffnessIncreases;
          assembleRigidIpcObjectiveSystemWithScratchInto(
              result.surfaces,
              laggedSurfaces,
              options.dynamicsTerms,
              options.articulationConstraints,
              barrierOptions,
              frictionOptions,
              assemblyScratch,
              result.assembly);
        }
      }
      recordSolveAssemblyStats(
          result, frictionIteration == 0u && iteration == 0u);
      if (frictionIteration == 0u && iteration == 0u) {
        newtonOptions.gradientTolerance
            = newton_barrier::projectedNewtonEffectiveGradientTolerance(
                options.newton.gradientTolerance,
                options.newton.relativeGradientTolerance,
                result.stats.initialGradientNorm);
      }

      RigidIpcProjectedNewtonStep& step
          = computeRigidIpcProjectedNewtonStepInto(
              result.assembly,
              nullptr,
              newtonOptions,
              scratch.step,
              scratch.memoryAllocator,
              scratch.workspace);
      result.lastStep = step;

      if (step.status == RigidIpcProjectedNewtonStatus::NoDofs) {
        if (!checkConvergedKinematicSweep()) {
          return;
        }
        result.status = RigidIpcProjectedNewtonSolveStatus::NoDofs;
        result.converged = true;
        return;
      }
      if (step.converged) {
        if (!checkConvergedKinematicSweep()) {
          return;
        }
        result.status = RigidIpcProjectedNewtonSolveStatus::Converged;
        result.converged = true;
        break;
      }
      if (!step.success) {
        result.status = RigidIpcProjectedNewtonSolveStatus::FactorizationFailed;
        result.failed = true;
        return;
      }
      if (iteration == options.maxIterations) {
        result.status = RigidIpcProjectedNewtonSolveStatus::MaxIterations;
        return;
      }

      if (options.useLineSearch) {
        assignSurfacesPreservingStorage(
            candidateSurfaces, result.surfaces, scratch.memoryAllocator);
        applyRigidIpcNewtonDelta(
            candidateSurfaces, result.assembly, step.delta.asEigen());
        // The Newton delta only moves dynamic surfaces, so candidateSurfaces
        // holds each kinematic obstacle at its END pose. Sweep from a start
        // configuration with kinematic obstacles at their START pose so the
        // conservative CCD checks the obstacle's start->end motion
        // (anti-tunneling against a moving obstacle); dynamic surfaces keep
        // their current iterate.
        if (hasKinematic) {
          result.lineSearch = computeRigidIpcLineSearchStepBoundWithScratch(
              makeKinematicLineSearchStart(),
              candidateSurfaces,
              options.lineSearch,
              lineSearchScratch);
        } else {
          result.lineSearch = computeRigidIpcLineSearchStepBoundWithScratch(
              result.surfaces,
              candidateSurfaces,
              options.lineSearch,
              lineSearchScratch);
        }
        recordSolveLineSearchStats(result);
        if (result.lineSearch.limited) {
          ++result.stats.lineSearchLimitedSteps;
        }
        if (hasKinematic
            && kinematicLineSearchLimitBlocksAcceptedState(result.lineSearch)) {
          result.status = RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked;
          result.failed = true;
          return;
        }
        computeRigidIpcProjectedNewtonStepInto(
            result.assembly,
            &result.lineSearch,
            newtonOptions,
            step,
            scratch.memoryAllocator,
            scratch.workspace);
        result.lastStep = step;
        if (step.lineSearchBlocked) {
          if (adaptive.enabled && iteration < options.maxIterations
              && barrierOptions.stiffness < maxBarrierStiffness) {
            const double updatedStiffness
                = std::min(maxBarrierStiffness, 2.0 * barrierOptions.stiffness);
            if (updatedStiffness > barrierOptions.stiffness
                && std::isfinite(updatedStiffness)) {
              barrierOptions.stiffness = updatedStiffness;
              result.stats.barrierStiffness = updatedStiffness;
              ++result.stats.barrierStiffnessIncreases;
              continue;
            }
          }
          result.status = RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked;
          result.failed = true;
          return;
        }
        if (!step.success) {
          result.status
              = RigidIpcProjectedNewtonSolveStatus::FactorizationFailed;
          result.failed = true;
          return;
        }
      }

      bool acceptedCandidate = false;
      if (newtonOptions.useSufficientDecreaseLineSearch
          && result.assembly.equalityResidual.size() == 0) {
        const double currentValue = result.assembly.value;
        const double directionalDerivative = step.stats.gradientDotStep;
        if (!std::isfinite(currentValue)
            || !std::isfinite(directionalDerivative)
            || !(directionalDerivative < 0.0)) {
          result.status
              = RigidIpcProjectedNewtonSolveStatus::FactorizationFailed;
          result.failed = true;
          return;
        }

        const double sufficientDecreaseFactor
            = newton_barrier::sanitizeSufficientDecreaseFactor(
                newtonOptions.sufficientDecreaseFactor);
        const double backtrackingScale
            = newton_barrier::sanitizeBacktrackingScale(
                newtonOptions.backtrackingScale);

        double trialScale = 1.0;
        acceptedSurfaces.clear();
        bestDecreasingSurfaces.clear();
        bool hasDecreasingCandidate = false;
        bool hasUnsafeKinematicCandidate = false;
        double bestDecreasingValue = currentValue;
        double bestDecreasingScale = 1.0;
        for (std::size_t backtrack = 0;
             backtrack <= newtonOptions.maxBacktrackingIterations;
             ++backtrack) {
          assignSurfacesPreservingStorage(
              candidateSurfaces, result.surfaces, scratch.memoryAllocator);
          applyRigidIpcNewtonDelta(
              candidateSurfaces,
              result.assembly,
              trialScale * step.delta.asEigen());
          if (!kinematicCandidateAllowsFullStep(candidateSurfaces)) {
            hasUnsafeKinematicCandidate = true;
            if (backtrack == newtonOptions.maxBacktrackingIterations) {
              break;
            }
            trialScale *= backtrackingScale;
            ++result.stats.sufficientDecreaseBacktracks;
            continue;
          }
          auto& candidateAssembly = assemblyScratch.candidateAssembly;
          assembleRigidIpcObjectiveSystemWithScratchInto(
              candidateSurfaces,
              laggedSurfaces,
              options.dynamicsTerms,
              options.articulationConstraints,
              barrierOptions,
              frictionOptions,
              assemblyScratch,
              candidateAssembly);
          ++result.stats.sufficientDecreaseChecks;

          if (std::isfinite(candidateAssembly.value)
              && candidateAssembly.value < bestDecreasingValue) {
            bestDecreasingValue = candidateAssembly.value;
            bestDecreasingScale = trialScale;
            assignSurfacesPreservingStorage(
                bestDecreasingSurfaces,
                candidateSurfaces,
                scratch.memoryAllocator);
            hasDecreasingCandidate = true;
          }
          if (newton_barrier::satisfiesSufficientDecrease(
                  currentValue,
                  candidateAssembly.value,
                  trialScale * directionalDerivative,
                  sufficientDecreaseFactor)) {
            assignSurfacesPreservingStorage(
                acceptedSurfaces, candidateSurfaces, scratch.memoryAllocator);
            acceptedCandidate = true;
            break;
          }

          if (backtrack == newtonOptions.maxBacktrackingIterations) {
            break;
          }
          trialScale *= backtrackingScale;
          ++result.stats.sufficientDecreaseBacktracks;
        }

        if (!acceptedCandidate) {
          if (hasDecreasingCandidate) {
            // Lagged friction and active-set changes can make Armijo too strict
            // for the finite budget even when a feasible candidate lowers the
            // assembled objective. Keep that progress instead of reporting a
            // line-search failure; per-candidate CCD already filtered unsafe
            // kinematic backtracking states.
            assignSurfacesPreservingStorage(
                acceptedSurfaces,
                bestDecreasingSurfaces,
                scratch.memoryAllocator);
            trialScale = bestDecreasingScale;
            acceptedCandidate = true;
          } else if (hasUnsafeKinematicCandidate) {
            result.status
                = RigidIpcProjectedNewtonSolveStatus::LineSearchBlocked;
            result.failed = true;
            return;
          } else {
            result.status = RigidIpcProjectedNewtonSolveStatus::MaxIterations;
            return;
          }
        }

        if (trialScale < 1.0) {
          scaleRigidIpcNewtonStep(step, trialScale);
          result.lastStep = step;
        }
        assignSurfacesPreservingStorage(
            result.surfaces, acceptedSurfaces, scratch.memoryAllocator);
      }

      if (!acceptedCandidate) {
        applyRigidIpcNewtonDelta(
            result.surfaces, result.assembly, step.delta.asEigen());
      }
      ++result.stats.acceptedSteps;
      ++result.stats.iterations;
      result.stats.lastStepNorm = step.stats.stepNorm;
      if (step.stats.stepNorm <= stepTolerance) {
        assembleRigidIpcObjectiveSystemWithScratchInto(
            result.surfaces,
            laggedSurfaces,
            options.dynamicsTerms,
            options.articulationConstraints,
            barrierOptions,
            frictionOptions,
            assemblyScratch,
            result.assembly);
        recordSolveAssemblyStats(result, false);
        if (result.assembly.equalityResidual.size() > 0
            && result.assembly.equalityResidual.norm()
                   > std::max(0.0, options.equalityTolerance)) {
          result.status = RigidIpcProjectedNewtonSolveStatus::MaxIterations;
          return;
        }
        result.status = RigidIpcProjectedNewtonSolveStatus::Converged;
        result.converged = true;
        break;
      }
    }

    if (!result.converged || !useLaggedFriction) {
      return;
    }

    assignSurfacesPreservingStorage(
        laggedSurfaces, result.surfaces, scratch.memoryAllocator);
    if (hasKinematic) {
      applyKinematicLag(laggedSurfaces);
    }
    assembleRigidIpcObjectiveSystemWithScratchInto(
        result.surfaces,
        laggedSurfaces,
        options.dynamicsTerms,
        options.articulationConstraints,
        barrierOptions,
        frictionOptions,
        assemblyScratch,
        result.assembly);
    recordSolveAssemblyStats(result, false);
    if (result.assembly.activeFrictionConstraints.empty()) {
      result.status = RigidIpcProjectedNewtonSolveStatus::Converged;
      result.converged = true;
      return;
    }

    ++result.stats.frictionIterations;
    if (frictionIteration + 1u == frictionIterationCount
        || (frictionConvergenceTolerance > 0.0
            && result.stats.finalMomentumBalance
                   <= frictionConvergenceTolerance)) {
      result.status = RigidIpcProjectedNewtonSolveStatus::Converged;
      result.converged = true;
      return;
    }
  }

  result.status = RigidIpcProjectedNewtonSolveStatus::MaxIterations;
}

} // namespace dart::simulation::detail
