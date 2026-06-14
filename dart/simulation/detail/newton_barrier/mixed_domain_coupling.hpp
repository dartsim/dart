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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND
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

#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/line_search.hpp>
#include <dart/simulation/detail/newton_barrier/primitive_distance.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <limits>
#include <span>
#include <tuple>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::newton_barrier {

enum class MixedDomainType
{
  Rigid,
  Deformable,
  Affine,
  Particle,
  Rod,
  Shell,
  Codimensional,
};

enum class MixedDomainPrimitive
{
  PointPoint,
  PointEdge,
  EdgeEdge,
  PointTriangle,
};

enum class MixedDomainOracleOwner
{
  RigidIpc,
  DeformableIpc,
  AffineBodyDynamics,
  MixedNewtonBarrier,
};

struct MixedDomainEdge
{
  std::size_t a = 0;
  std::size_t b = 0;

  [[nodiscard]] friend bool operator==(
      const MixedDomainEdge& lhs, const MixedDomainEdge& rhs) = default;

  [[nodiscard]] friend bool operator<(
      const MixedDomainEdge& lhs, const MixedDomainEdge& rhs)
  {
    return std::tie(lhs.a, lhs.b) < std::tie(rhs.a, rhs.b);
  }
};

struct MixedDomainSurface
{
  using VertexAllocator = dart::common::StlAllocator<Eigen::Vector3d>;
  using EdgeAllocator = dart::common::StlAllocator<MixedDomainEdge>;
  using TriangleAllocator = dart::common::StlAllocator<Eigen::Vector3i>;
  using VertexVector = std::vector<Eigen::Vector3d, VertexAllocator>;
  using EdgeVector = std::vector<MixedDomainEdge, EdgeAllocator>;
  using TriangleVector = std::vector<Eigen::Vector3i, TriangleAllocator>;

  MixedDomainSurface() = default;

  explicit MixedDomainSurface(dart::common::MemoryAllocator& allocator)
    : startVertices(VertexAllocator{allocator}),
      endVertices(VertexAllocator{allocator}),
      edges(EdgeAllocator{allocator}),
      triangles(TriangleAllocator{allocator})
  {
  }

  MixedDomainType domain = MixedDomainType::Rigid;
  std::size_t domainInstance = 0;
  bool active = true;
  bool dynamic = true;
  double frictionCoefficient = 0.0;
  VertexVector startVertices;
  VertexVector endVertices;
  EdgeVector edges;
  TriangleVector triangles;
};

struct MixedDomainCandidateOptions
{
  double activationDistance = 0.0;
  bool exactDistanceFilter = true;
  bool includePointPoint = true;
  bool includePointEdge = true;
  bool includeEdgeEdge = true;
  bool includePointTriangle = true;
};

struct MixedDomainContactCandidate
{
  MixedDomainPrimitive primitive = MixedDomainPrimitive::PointPoint;
  std::size_t surfaceA = 0;
  std::size_t surfaceB = 0;
  std::array<std::size_t, 4> vertices{
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max(),
      std::numeric_limits<std::size_t>::max()};
  double squaredDistance = 0.0;
  bool dynamicPair = true;
  MixedDomainOracleOwner oracleOwner
      = MixedDomainOracleOwner::MixedNewtonBarrier;
};

struct MixedDomainCandidateStats
{
  std::size_t surfaceCount = 0;
  std::size_t activeSurfaceCount = 0;
  std::size_t dynamicSurfaceCount = 0;
  std::array<std::size_t, 7> domainCounts{};
  std::size_t broadPhasePairCount = 0;
  std::size_t exactDistanceCheckCount = 0;
  std::size_t staticStaticRejectCount = 0;
  std::size_t pointPointCandidateCount = 0;
  std::size_t pointEdgeCandidateCount = 0;
  std::size_t edgeEdgeCandidateCount = 0;
  std::size_t pointTriangleCandidateCount = 0;
};

struct MixedDomainCandidateSet
{
  using CandidateAllocator
      = dart::common::StlAllocator<MixedDomainContactCandidate>;
  using CandidateVector
      = std::vector<MixedDomainContactCandidate, CandidateAllocator>;

  MixedDomainCandidateSet() = default;

  explicit MixedDomainCandidateSet(dart::common::MemoryAllocator& allocator)
    : candidates(CandidateAllocator{allocator})
  {
  }

  CandidateVector candidates;
  MixedDomainCandidateStats stats;
};

struct MixedDomainBarrierDiagnostics
{
  bool finite = true;
  double value = 0.0;
  double minSquaredDistance = std::numeric_limits<double>::infinity();
  double maxFrictionCoefficient = 0.0;
  std::size_t activeBarrierCount = 0;
  std::size_t candidateCount = 0;
  LineSearchStats lineSearchStats;
};

struct MixedDomainLinearCcdResult
{
  bool limited = false;
  bool indeterminate = false;
  double stepBound = 1.0;
  std::size_t limitingCandidate = std::numeric_limits<std::size_t>::max();
  LineSearchStats stats;

  [[nodiscard]] bool allowsPositiveStep() const noexcept
  {
    return allowsPositiveLineSearchStep(stepBound, indeterminate);
  }
};

namespace detail {

//==============================================================================
inline std::size_t domainIndex(const MixedDomainType domain)
{
  return static_cast<std::size_t>(domain);
}

//==============================================================================
inline MixedDomainEdge makeMixedDomainEdge(std::size_t a, std::size_t b)
{
  if (b < a) {
    std::swap(a, b);
  }
  return MixedDomainEdge{a, b};
}

//==============================================================================
inline bool validVertexIndex(
    const std::size_t index, const MixedDomainSurface& surface)
{
  return index < surface.endVertices.size()
         && index < surface.startVertices.size();
}

//==============================================================================
inline MixedDomainSurface::EdgeVector collectSurfaceEdges(
    const MixedDomainSurface& surface,
    dart::common::MemoryAllocator* allocator = nullptr)
{
  MixedDomainSurface::EdgeVector edges{
      allocator != nullptr ? MixedDomainSurface::EdgeAllocator{*allocator}
                           : MixedDomainSurface::EdgeAllocator{}};
  edges.assign(surface.edges.begin(), surface.edges.end());
  for (MixedDomainEdge& edge : edges) {
    edge = makeMixedDomainEdge(edge.a, edge.b);
  }
  for (const Eigen::Vector3i& triangle : surface.triangles) {
    if (triangle.minCoeff() < 0) {
      continue;
    }
    const std::size_t a = static_cast<std::size_t>(triangle[0]);
    const std::size_t b = static_cast<std::size_t>(triangle[1]);
    const std::size_t c = static_cast<std::size_t>(triangle[2]);
    if (validVertexIndex(a, surface) && validVertexIndex(b, surface)
        && validVertexIndex(c, surface)) {
      edges.push_back(makeMixedDomainEdge(a, b));
      edges.push_back(makeMixedDomainEdge(b, c));
      edges.push_back(makeMixedDomainEdge(c, a));
    }
  }
  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

struct MixedDomainAabb
{
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  bool valid = false;

  void expand(const double margin)
  {
    const Eigen::Vector3d delta
        = Eigen::Vector3d::Constant(std::max(0.0, margin));
    min -= delta;
    max += delta;
  }

  [[nodiscard]] bool overlaps(const MixedDomainAabb& other) const
  {
    return valid && other.valid && min.x() <= other.max.x()
           && max.x() >= other.min.x() && min.y() <= other.max.y()
           && max.y() >= other.min.y() && min.z() <= other.max.z()
           && max.z() >= other.min.z();
  }
};

//==============================================================================
inline MixedDomainAabb makeSweptSurfaceAabb(
    const MixedDomainSurface& surface, const double margin)
{
  MixedDomainAabb aabb;
  if (!surface.active
      || surface.startVertices.size() != surface.endVertices.size()
      || surface.startVertices.empty()) {
    return aabb;
  }

  Eigen::Vector3d lo
      = surface.startVertices.front().cwiseMin(surface.endVertices.front());
  Eigen::Vector3d hi
      = surface.startVertices.front().cwiseMax(surface.endVertices.front());
  for (std::size_t i = 1; i < surface.startVertices.size(); ++i) {
    lo = lo.cwiseMin(surface.startVertices[i]).cwiseMin(surface.endVertices[i]);
    hi = hi.cwiseMax(surface.startVertices[i]).cwiseMax(surface.endVertices[i]);
  }
  aabb.min = lo;
  aabb.max = hi;
  aabb.valid = true;
  aabb.expand(margin);
  return aabb;
}

//==============================================================================
inline bool withinActivationDistance(
    const double squaredDistance, const double activationDistance)
{
  if (!std::isfinite(squaredDistance)) {
    return false;
  }
  const double distance = std::max(0.0, activationDistance);
  const double threshold = distance * distance;
  const double tolerance = kRelativeEpsilon * std::max(1.0, threshold);
  return squaredDistance <= threshold + tolerance;
}

//==============================================================================
inline MixedDomainOracleOwner sameDomainOracleOwner(
    const MixedDomainType domain)
{
  switch (domain) {
    case MixedDomainType::Rigid:
      return MixedDomainOracleOwner::RigidIpc;
    case MixedDomainType::Affine:
      return MixedDomainOracleOwner::AffineBodyDynamics;
    case MixedDomainType::Deformable:
    case MixedDomainType::Particle:
    case MixedDomainType::Rod:
    case MixedDomainType::Shell:
    case MixedDomainType::Codimensional:
      return MixedDomainOracleOwner::DeformableIpc;
  }
  return MixedDomainOracleOwner::MixedNewtonBarrier;
}

//==============================================================================
inline void pushCandidate(
    MixedDomainCandidateSet& set, const MixedDomainContactCandidate& candidate)
{
  set.candidates.push_back(candidate);
  switch (candidate.primitive) {
    case MixedDomainPrimitive::PointPoint:
      ++set.stats.pointPointCandidateCount;
      break;
    case MixedDomainPrimitive::PointEdge:
      ++set.stats.pointEdgeCandidateCount;
      break;
    case MixedDomainPrimitive::EdgeEdge:
      ++set.stats.edgeEdgeCandidateCount;
      break;
    case MixedDomainPrimitive::PointTriangle:
      ++set.stats.pointTriangleCandidateCount;
      break;
  }
}

//==============================================================================
inline std::uint64_t hashCombine(std::uint64_t seed, const std::uint64_t value)
{
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

} // namespace detail

//==============================================================================
inline MixedDomainOracleOwner mixedDomainOracleOwner(
    const MixedDomainType domainA, const MixedDomainType domainB)
{
  if (domainA == domainB) {
    return detail::sameDomainOracleOwner(domainA);
  }
  return MixedDomainOracleOwner::MixedNewtonBarrier;
}

//==============================================================================
inline MixedDomainSurface makeMixedDomainSurface(
    const MixedDomainType domain,
    const std::size_t domainInstance,
    std::vector<Eigen::Vector3d> vertices,
    std::vector<Eigen::Vector3i> triangles = {},
    std::vector<MixedDomainEdge> edges = {},
    const bool dynamic = true)
{
  MixedDomainSurface surface;
  surface.domain = domain;
  surface.domainInstance = domainInstance;
  surface.dynamic = dynamic;
  surface.startVertices.assign(vertices.begin(), vertices.end());
  surface.endVertices.assign(vertices.begin(), vertices.end());
  surface.triangles.assign(triangles.begin(), triangles.end());
  surface.edges.assign(edges.begin(), edges.end());
  return surface;
}

//==============================================================================
inline void setMixedDomainEndVertices(
    MixedDomainSurface& surface, std::vector<Eigen::Vector3d> vertices)
{
  if (vertices.size() == surface.startVertices.size()) {
    surface.endVertices.assign(vertices.begin(), vertices.end());
  }
}

//==============================================================================
inline MixedDomainCandidateSet buildMixedDomainContactCandidates(
    std::span<const MixedDomainSurface> surfaces,
    const MixedDomainCandidateOptions& options = {},
    dart::common::MemoryAllocator* allocator = nullptr)
{
  MixedDomainCandidateSet set = allocator != nullptr
                                    ? MixedDomainCandidateSet{*allocator}
                                    : MixedDomainCandidateSet{};
  set.stats.surfaceCount = surfaces.size();

  using EdgeVector = MixedDomainSurface::EdgeVector;
  using EdgeVectorAllocator = dart::common::StlAllocator<EdgeVector>;
  using EdgeVectorList = std::vector<EdgeVector, EdgeVectorAllocator>;
  using AabbAllocator = dart::common::StlAllocator<detail::MixedDomainAabb>;
  using AabbVector = std::vector<detail::MixedDomainAabb, AabbAllocator>;

  EdgeVectorList edges{
      allocator != nullptr ? EdgeVectorAllocator{*allocator}
                           : EdgeVectorAllocator{}};
  edges.reserve(surfaces.size());
  AabbVector aabbs{
      allocator != nullptr ? AabbAllocator{*allocator} : AabbAllocator{}};
  aabbs.reserve(surfaces.size());
  const double activationDistance = std::max(0.0, options.activationDistance);

  for (const MixedDomainSurface& surface : surfaces) {
    if (surface.active) {
      ++set.stats.activeSurfaceCount;
      if (surface.dynamic) {
        ++set.stats.dynamicSurfaceCount;
      }
      ++set.stats.domainCounts[detail::domainIndex(surface.domain)];
    }
    edges.push_back(detail::collectSurfaceEdges(surface, allocator));
    aabbs.push_back(detail::makeSweptSurfaceAabb(surface, activationDistance));
  }

  const auto maybePush = [&](MixedDomainContactCandidate candidate) {
    if (options.exactDistanceFilter
        && !detail::withinActivationDistance(
            candidate.squaredDistance, activationDistance)) {
      return;
    }
    detail::pushCandidate(set, candidate);
  };

  for (std::size_t a = 0; a < surfaces.size(); ++a) {
    for (std::size_t b = a + 1; b < surfaces.size(); ++b) {
      const MixedDomainSurface& surfaceA = surfaces[a];
      const MixedDomainSurface& surfaceB = surfaces[b];
      if (!surfaceA.active || !surfaceB.active) {
        continue;
      }
      if (!surfaceA.dynamic && !surfaceB.dynamic) {
        ++set.stats.staticStaticRejectCount;
        continue;
      }
      if (!aabbs[a].overlaps(aabbs[b])) {
        continue;
      }
      ++set.stats.broadPhasePairCount;

      const bool dynamicPair = surfaceA.dynamic || surfaceB.dynamic;
      const MixedDomainOracleOwner oracleOwner
          = mixedDomainOracleOwner(surfaceA.domain, surfaceB.domain);

      if (options.includePointPoint) {
        for (std::size_t vertexA = 0; vertexA < surfaceA.endVertices.size();
             ++vertexA) {
          for (std::size_t vertexB = 0; vertexB < surfaceB.endVertices.size();
               ++vertexB) {
            ++set.stats.exactDistanceCheckCount;
            const double squaredDistance = pointPointSquaredDistance(
                surfaceA.endVertices[vertexA], surfaceB.endVertices[vertexB]);
            maybePush(
                MixedDomainContactCandidate{
                    MixedDomainPrimitive::PointPoint,
                    a,
                    b,
                    {vertexA,
                     vertexB,
                     std::numeric_limits<std::size_t>::max(),
                     std::numeric_limits<std::size_t>::max()},
                    squaredDistance,
                    dynamicPair,
                    oracleOwner});
          }
        }
      }

      if (options.includePointEdge) {
        for (std::size_t vertexA = 0; vertexA < surfaceA.endVertices.size();
             ++vertexA) {
          for (const MixedDomainEdge& edgeB : edges[b]) {
            if (!detail::validVertexIndex(edgeB.a, surfaceB)
                || !detail::validVertexIndex(edgeB.b, surfaceB)) {
              continue;
            }
            ++set.stats.exactDistanceCheckCount;
            const auto distance = pointEdgeSquaredDistance(
                surfaceA.endVertices[vertexA],
                surfaceB.endVertices[edgeB.a],
                surfaceB.endVertices[edgeB.b]);
            maybePush(
                MixedDomainContactCandidate{
                    MixedDomainPrimitive::PointEdge,
                    a,
                    b,
                    {vertexA,
                     edgeB.a,
                     edgeB.b,
                     std::numeric_limits<std::size_t>::max()},
                    distance.squaredDistance,
                    dynamicPair,
                    oracleOwner});
          }
        }
        for (std::size_t vertexB = 0; vertexB < surfaceB.endVertices.size();
             ++vertexB) {
          for (const MixedDomainEdge& edgeA : edges[a]) {
            if (!detail::validVertexIndex(edgeA.a, surfaceA)
                || !detail::validVertexIndex(edgeA.b, surfaceA)) {
              continue;
            }
            ++set.stats.exactDistanceCheckCount;
            const auto distance = pointEdgeSquaredDistance(
                surfaceB.endVertices[vertexB],
                surfaceA.endVertices[edgeA.a],
                surfaceA.endVertices[edgeA.b]);
            maybePush(
                MixedDomainContactCandidate{
                    MixedDomainPrimitive::PointEdge,
                    b,
                    a,
                    {vertexB,
                     edgeA.a,
                     edgeA.b,
                     std::numeric_limits<std::size_t>::max()},
                    distance.squaredDistance,
                    dynamicPair,
                    oracleOwner});
          }
        }
      }

      if (options.includeEdgeEdge) {
        for (const MixedDomainEdge& edgeA : edges[a]) {
          if (!detail::validVertexIndex(edgeA.a, surfaceA)
              || !detail::validVertexIndex(edgeA.b, surfaceA)) {
            continue;
          }
          for (const MixedDomainEdge& edgeB : edges[b]) {
            if (!detail::validVertexIndex(edgeB.a, surfaceB)
                || !detail::validVertexIndex(edgeB.b, surfaceB)) {
              continue;
            }
            ++set.stats.exactDistanceCheckCount;
            const auto distance = edgeEdgeSquaredDistance(
                surfaceA.endVertices[edgeA.a],
                surfaceA.endVertices[edgeA.b],
                surfaceB.endVertices[edgeB.a],
                surfaceB.endVertices[edgeB.b]);
            maybePush(
                MixedDomainContactCandidate{
                    MixedDomainPrimitive::EdgeEdge,
                    a,
                    b,
                    {edgeA.a, edgeA.b, edgeB.a, edgeB.b},
                    distance.squaredDistance,
                    dynamicPair,
                    oracleOwner});
          }
        }
      }

      if (options.includePointTriangle) {
        for (std::size_t vertexA = 0; vertexA < surfaceA.endVertices.size();
             ++vertexA) {
          for (std::size_t triangleB = 0; triangleB < surfaceB.triangles.size();
               ++triangleB) {
            const Eigen::Vector3i tri = surfaceB.triangles[triangleB];
            if (tri.minCoeff() < 0) {
              continue;
            }
            const std::size_t tb0 = static_cast<std::size_t>(tri[0]);
            const std::size_t tb1 = static_cast<std::size_t>(tri[1]);
            const std::size_t tb2 = static_cast<std::size_t>(tri[2]);
            if (!detail::validVertexIndex(tb0, surfaceB)
                || !detail::validVertexIndex(tb1, surfaceB)
                || !detail::validVertexIndex(tb2, surfaceB)) {
              continue;
            }
            ++set.stats.exactDistanceCheckCount;
            const auto distance = pointTriangleSquaredDistance(
                surfaceA.endVertices[vertexA],
                surfaceB.endVertices[tb0],
                surfaceB.endVertices[tb1],
                surfaceB.endVertices[tb2]);
            maybePush(
                MixedDomainContactCandidate{
                    MixedDomainPrimitive::PointTriangle,
                    a,
                    b,
                    {vertexA, tb0, tb1, tb2},
                    distance.squaredDistance,
                    dynamicPair,
                    oracleOwner});
          }
        }
        for (std::size_t vertexB = 0; vertexB < surfaceB.endVertices.size();
             ++vertexB) {
          for (std::size_t triangleA = 0; triangleA < surfaceA.triangles.size();
               ++triangleA) {
            const Eigen::Vector3i tri = surfaceA.triangles[triangleA];
            if (tri.minCoeff() < 0) {
              continue;
            }
            const std::size_t ta0 = static_cast<std::size_t>(tri[0]);
            const std::size_t ta1 = static_cast<std::size_t>(tri[1]);
            const std::size_t ta2 = static_cast<std::size_t>(tri[2]);
            if (!detail::validVertexIndex(ta0, surfaceA)
                || !detail::validVertexIndex(ta1, surfaceA)
                || !detail::validVertexIndex(ta2, surfaceA)) {
              continue;
            }
            ++set.stats.exactDistanceCheckCount;
            const auto distance = pointTriangleSquaredDistance(
                surfaceB.endVertices[vertexB],
                surfaceA.endVertices[ta0],
                surfaceA.endVertices[ta1],
                surfaceA.endVertices[ta2]);
            maybePush(
                MixedDomainContactCandidate{
                    MixedDomainPrimitive::PointTriangle,
                    b,
                    a,
                    {vertexB, ta0, ta1, ta2},
                    distance.squaredDistance,
                    dynamicPair,
                    oracleOwner});
          }
        }
      }
    }
  }

  std::sort(
      set.candidates.begin(),
      set.candidates.end(),
      [](const auto& lhs, const auto& rhs) {
        return std::tie(lhs.primitive, lhs.surfaceA, lhs.surfaceB, lhs.vertices)
               < std::tie(
                   rhs.primitive, rhs.surfaceA, rhs.surfaceB, rhs.vertices);
      });
  return set;
}

//==============================================================================
inline double mixedDomainEffectiveFrictionCoefficient(
    const MixedDomainSurface& a, const MixedDomainSurface& b)
{
  const auto sanitize = [](const double coefficient) {
    return std::isfinite(coefficient) ? std::max(0.0, coefficient) : 0.0;
  };
  return std::max(
      sanitize(a.frictionCoefficient), sanitize(b.frictionCoefficient));
}

//==============================================================================
inline MixedDomainBarrierDiagnostics evaluateMixedDomainBarrierDiagnostics(
    std::span<const MixedDomainSurface> surfaces,
    const MixedDomainCandidateSet& candidateSet,
    const double activationDistance,
    const double stiffness = 1.0)
{
  MixedDomainBarrierDiagnostics diagnostics;
  diagnostics.candidateCount = candidateSet.candidates.size();
  const double squaredActivation
      = std::max(0.0, activationDistance) * std::max(0.0, activationDistance);

  for (const MixedDomainContactCandidate& candidate : candidateSet.candidates) {
    if (candidate.surfaceA >= surfaces.size()
        || candidate.surfaceB >= surfaces.size()) {
      diagnostics.finite = false;
      continue;
    }
    const MixedDomainSurface& a = surfaces[candidate.surfaceA];
    const MixedDomainSurface& b = surfaces[candidate.surfaceB];
    diagnostics.maxFrictionCoefficient = std::max(
        diagnostics.maxFrictionCoefficient,
        mixedDomainEffectiveFrictionCoefficient(a, b));
    diagnostics.minSquaredDistance
        = std::min(diagnostics.minSquaredDistance, candidate.squaredDistance);

    PrimitiveBarrierResult barrier;
    switch (candidate.primitive) {
      case MixedDomainPrimitive::PointPoint:
        if (detail::validVertexIndex(candidate.vertices[0], a)
            && detail::validVertexIndex(candidate.vertices[1], b)) {
          barrier = pointPointBarrier(
              a.endVertices[candidate.vertices[0]],
              b.endVertices[candidate.vertices[1]],
              squaredActivation,
              stiffness);
        }
        break;
      case MixedDomainPrimitive::PointEdge:
        if (detail::validVertexIndex(candidate.vertices[0], a)
            && detail::validVertexIndex(candidate.vertices[1], b)
            && detail::validVertexIndex(candidate.vertices[2], b)) {
          barrier = pointEdgeBarrier(
              a.endVertices[candidate.vertices[0]],
              b.endVertices[candidate.vertices[1]],
              b.endVertices[candidate.vertices[2]],
              squaredActivation,
              stiffness);
        }
        break;
      case MixedDomainPrimitive::EdgeEdge:
        if (detail::validVertexIndex(candidate.vertices[0], a)
            && detail::validVertexIndex(candidate.vertices[1], a)
            && detail::validVertexIndex(candidate.vertices[2], b)
            && detail::validVertexIndex(candidate.vertices[3], b)) {
          barrier = edgeEdgeBarrier(
              a.endVertices[candidate.vertices[0]],
              a.endVertices[candidate.vertices[1]],
              b.endVertices[candidate.vertices[2]],
              b.endVertices[candidate.vertices[3]],
              squaredActivation,
              stiffness);
        }
        break;
      case MixedDomainPrimitive::PointTriangle:
        if (detail::validVertexIndex(candidate.vertices[0], a)
            && detail::validVertexIndex(candidate.vertices[1], b)
            && detail::validVertexIndex(candidate.vertices[2], b)
            && detail::validVertexIndex(candidate.vertices[3], b)) {
          barrier = pointTriangleBarrier(
              a.endVertices[candidate.vertices[0]],
              b.endVertices[candidate.vertices[1]],
              b.endVertices[candidate.vertices[2]],
              b.endVertices[candidate.vertices[3]],
              squaredActivation,
              stiffness);
        }
        break;
    }

    if (barrier.active) {
      ++diagnostics.activeBarrierCount;
      diagnostics.value += barrier.value;
    }
    diagnostics.finite = diagnostics.finite && std::isfinite(diagnostics.value)
                         && std::isfinite(candidate.squaredDistance);
  }

  if (candidateSet.candidates.empty()) {
    diagnostics.minSquaredDistance = 0.0;
  }
  return diagnostics;
}

//==============================================================================
inline MixedDomainLinearCcdResult mixedDomainPointPointLinearCcd(
    std::span<const MixedDomainSurface> surfaces,
    const MixedDomainCandidateSet& candidateSet,
    const LineSearchOptions& options = {})
{
  MixedDomainLinearCcdResult result;
  const double minSeparation = std::max(0.0, options.minSeparation);
  const double minSeparationSquared = minSeparation * minSeparation;

  for (std::size_t i = 0; i < candidateSet.candidates.size(); ++i) {
    const MixedDomainContactCandidate& candidate = candidateSet.candidates[i];
    if (candidate.primitive != MixedDomainPrimitive::PointPoint) {
      continue;
    }
    ++result.stats.pointPointChecks;
    if (candidate.surfaceA >= surfaces.size()
        || candidate.surfaceB >= surfaces.size()) {
      ++result.stats.indeterminate;
      result.indeterminate = true;
      continue;
    }
    const MixedDomainSurface& surfaceA = surfaces[candidate.surfaceA];
    const MixedDomainSurface& surfaceB = surfaces[candidate.surfaceB];
    if (!detail::validVertexIndex(candidate.vertices[0], surfaceA)
        || !detail::validVertexIndex(candidate.vertices[1], surfaceB)) {
      ++result.stats.indeterminate;
      result.indeterminate = true;
      continue;
    }

    const Eigen::Vector3d r0 = surfaceA.startVertices[candidate.vertices[0]]
                               - surfaceB.startVertices[candidate.vertices[1]];
    const Eigen::Vector3d r1 = surfaceA.endVertices[candidate.vertices[0]]
                               - surfaceB.endVertices[candidate.vertices[1]];
    if (!r0.allFinite() || !r1.allFinite()) {
      ++result.stats.indeterminate;
      result.indeterminate = true;
      continue;
    }

    if (r0.squaredNorm() <= minSeparationSquared) {
      const double step = recordLineSearchHit(result.stats, 0.0);
      result.limited = true;
      result.stepBound = std::min(result.stepBound, step);
      result.limitingCandidate = i;
      continue;
    }

    const Eigen::Vector3d velocity = r1 - r0;
    const double a = velocity.squaredNorm();
    const double b = 2.0 * r0.dot(velocity);
    const double c = r0.squaredNorm() - minSeparationSquared;
    if (a <= std::numeric_limits<double>::min()) {
      ++result.stats.misses;
      continue;
    }

    const double discriminant = b * b - 4.0 * a * c;
    if (!(discriminant >= 0.0) || !std::isfinite(discriminant)) {
      ++result.stats.misses;
      continue;
    }

    const double t = (-b - std::sqrt(discriminant)) / (2.0 * a);
    if (t >= 0.0 && t <= 1.0) {
      const double step = recordLineSearchHit(result.stats, t);
      if (!result.limited || step < result.stepBound) {
        result.limited = true;
        result.stepBound = step;
        result.limitingCandidate = i;
      }
    } else {
      ++result.stats.misses;
    }
  }

  return result;
}

//==============================================================================
inline std::uint64_t mixedDomainCandidateKey(
    const MixedDomainContactCandidate& candidate)
{
  std::uint64_t seed = 1469598103934665603ULL;
  seed = detail::hashCombine(
      seed, static_cast<std::uint64_t>(candidate.primitive));
  seed = detail::hashCombine(seed, candidate.surfaceA);
  seed = detail::hashCombine(seed, candidate.surfaceB);
  for (const std::size_t vertex : candidate.vertices) {
    seed = detail::hashCombine(seed, vertex);
  }
  return seed;
}

//==============================================================================
inline std::vector<std::uint64_t> mixedDomainCandidateKeys(
    const MixedDomainCandidateSet& candidateSet)
{
  std::vector<std::uint64_t> keys;
  keys.reserve(candidateSet.candidates.size());
  for (const MixedDomainContactCandidate& candidate : candidateSet.candidates) {
    keys.push_back(mixedDomainCandidateKey(candidate));
  }
  return keys;
}

} // namespace dart::simulation::detail::newton_barrier
