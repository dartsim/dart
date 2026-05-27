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

#include <dart/simulation/experimental/body/deformable_body_options.hpp>
#include <dart/simulation/experimental/detail/deformable_contact/primitive_distance.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <span>
#include <tuple>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::detail::deformable_contact {

struct SurfaceEdge
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;

  [[nodiscard]] friend bool operator==(
      const SurfaceEdge& lhs, const SurfaceEdge& rhs)
      = default;

  [[nodiscard]] friend bool operator<(
      const SurfaceEdge& lhs, const SurfaceEdge& rhs)
  {
    return std::tie(lhs.nodeA, lhs.nodeB) < std::tie(rhs.nodeA, rhs.nodeB);
  }
};

struct PointTriangleCandidate
{
  std::size_t point = 0;
  std::size_t triangle = 0;
  double squaredDistance = 0.0;

  [[nodiscard]] friend bool operator<(
      const PointTriangleCandidate& lhs, const PointTriangleCandidate& rhs)
  {
    return std::tie(lhs.point, lhs.triangle)
           < std::tie(rhs.point, rhs.triangle);
  }
};

struct EdgeEdgeCandidate
{
  std::size_t edgeA = 0;
  std::size_t edgeB = 0;
  double squaredDistance = 0.0;

  [[nodiscard]] friend bool operator<(
      const EdgeEdgeCandidate& lhs, const EdgeEdgeCandidate& rhs)
  {
    return std::tie(lhs.edgeA, lhs.edgeB) < std::tie(rhs.edgeA, rhs.edgeB);
  }
};

struct ContactCandidateOptions
{
  double activationDistance = 0.0;
  bool exactDistanceFilter = true;
  bool excludeIncidentPointTriangles = true;
  bool excludeAdjacentEdges = true;
};

struct ContactCandidateStats
{
  std::size_t pointCount = 0;
  std::size_t triangleCount = 0;
  std::size_t edgeCount = 0;
  std::size_t broadPhaseOverlapCount = 0;
  std::size_t exactDistanceCheckCount = 0;
  std::size_t incidentPointTriangleRejectCount = 0;
  std::size_t adjacentEdgeEdgeRejectCount = 0;
  std::size_t pointTriangleCandidateCount = 0;
  std::size_t edgeEdgeCandidateCount = 0;
};

struct ContactCandidateSet
{
  std::vector<SurfaceEdge> surfaceEdges;
  std::vector<PointTriangleCandidate> pointTriangleCandidates;
  std::vector<EdgeEdgeCandidate> edgeEdgeCandidates;
  ContactCandidateStats stats;
};

namespace detail {

struct CandidateAabb
{
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();

  [[nodiscard]] bool overlaps(const CandidateAabb& other) const
  {
    return min.x() <= other.max.x() && max.x() >= other.min.x()
           && min.y() <= other.max.y() && max.y() >= other.min.y()
           && min.z() <= other.max.z() && max.z() >= other.min.z();
  }

  void expand(const double margin)
  {
    const Eigen::Vector3d delta
        = Eigen::Vector3d::Constant(std::max(0.0, margin));
    min -= delta;
    max += delta;
  }
};

struct SweepItem
{
  std::size_t id = 0;
  CandidateAabb aabb;
};

//==============================================================================
inline double nonnegativeActivationDistance(
    const ContactCandidateOptions& options)
{
  return std::max(0.0, options.activationDistance);
}

//==============================================================================
inline bool withinActivationDistance(
    const double squaredDistance, const double activationDistance)
{
  const double distance = std::max(0.0, activationDistance);
  const double threshold = distance * distance;
  const double tolerance = kRelativeEpsilon * std::max(1.0, threshold);
  return squaredDistance <= threshold + tolerance;
}

//==============================================================================
inline SurfaceEdge makeSurfaceEdge(std::size_t a, std::size_t b)
{
  if (b < a) {
    std::swap(a, b);
  }
  return SurfaceEdge{a, b};
}

//==============================================================================
inline CandidateAabb makePointAabb(
    const Eigen::Vector3d& point, const double margin)
{
  CandidateAabb aabb;
  aabb.min = point;
  aabb.max = point;
  aabb.expand(margin);
  return aabb;
}

//==============================================================================
inline CandidateAabb makeSegmentAabb(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b, const double margin)
{
  CandidateAabb aabb;
  aabb.min = a.cwiseMin(b);
  aabb.max = a.cwiseMax(b);
  aabb.expand(margin);
  return aabb;
}

//==============================================================================
inline CandidateAabb makeTriangleAabb(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const double margin)
{
  CandidateAabb aabb;
  aabb.min = a.cwiseMin(b).cwiseMin(c);
  aabb.max = a.cwiseMax(b).cwiseMax(c);
  aabb.expand(margin);
  return aabb;
}

//==============================================================================
inline void sortSweepItems(std::vector<SweepItem>& items)
{
  std::sort(
      items.begin(),
      items.end(),
      [](const SweepItem& lhs, const SweepItem& rhs) {
        return std::tie(lhs.aabb.min.x(), lhs.aabb.max.x(), lhs.id)
               < std::tie(rhs.aabb.min.x(), rhs.aabb.max.x(), rhs.id);
      });
}

//==============================================================================
template <typename Visitor>
void visitSweepPairs(
    std::vector<SweepItem> lhsItems,
    std::vector<SweepItem> rhsItems,
    Visitor&& visitor)
{
  sortSweepItems(lhsItems);
  sortSweepItems(rhsItems);

  for (const auto& lhs : lhsItems) {
    for (const auto& rhs : rhsItems) {
      if (rhs.aabb.min.x() > lhs.aabb.max.x()) {
        break;
      }
      if (rhs.aabb.max.x() < lhs.aabb.min.x()) {
        continue;
      }
      if (lhs.aabb.overlaps(rhs.aabb)) {
        visitor(lhs.id, rhs.id);
      }
    }
  }
}

//==============================================================================
template <typename Visitor>
void visitSelfSweepPairs(std::vector<SweepItem> items, Visitor&& visitor)
{
  sortSweepItems(items);

  for (std::size_t i = 0; i < items.size(); ++i) {
    for (std::size_t j = i + 1; j < items.size(); ++j) {
      if (items[j].aabb.min.x() > items[i].aabb.max.x()) {
        break;
      }
      if (items[i].aabb.overlaps(items[j].aabb)) {
        visitor(items[i].id, items[j].id);
      }
    }
  }
}

//==============================================================================
inline bool pointIsIncidentToTriangle(
    const std::size_t point, const DeformableSurfaceTriangle& triangle)
{
  return point == triangle.nodeA || point == triangle.nodeB
         || point == triangle.nodeC;
}

//==============================================================================
inline bool edgesAreAdjacent(const SurfaceEdge& a, const SurfaceEdge& b)
{
  return a.nodeA == b.nodeA || a.nodeA == b.nodeB || a.nodeB == b.nodeA
         || a.nodeB == b.nodeB;
}

//==============================================================================
inline void sortAndDedupe(std::vector<PointTriangleCandidate>& candidates)
{
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(
      std::unique(
          candidates.begin(),
          candidates.end(),
          [](const PointTriangleCandidate& lhs,
             const PointTriangleCandidate& rhs) {
            return lhs.point == rhs.point && lhs.triangle == rhs.triangle;
          }),
      candidates.end());
}

//==============================================================================
inline void sortAndDedupe(std::vector<EdgeEdgeCandidate>& candidates)
{
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(
      std::unique(
          candidates.begin(),
          candidates.end(),
          [](const EdgeEdgeCandidate& lhs, const EdgeEdgeCandidate& rhs) {
            return lhs.edgeA == rhs.edgeA && lhs.edgeB == rhs.edgeB;
          }),
      candidates.end());
}

//==============================================================================
inline void maybeAddPointTriangleCandidate(
    ContactCandidateSet& candidates,
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const std::size_t point,
    const std::size_t triangleIndex,
    const ContactCandidateOptions& options)
{
  const auto& triangle = triangles[triangleIndex];
  if (options.excludeIncidentPointTriangles
      && pointIsIncidentToTriangle(point, triangle)) {
    ++candidates.stats.incidentPointTriangleRejectCount;
    return;
  }

  const double squaredDistance = pointTriangleSquaredDistance(
                                     positions[point],
                                     positions[triangle.nodeA],
                                     positions[triangle.nodeB],
                                     positions[triangle.nodeC])
                                     .squaredDistance;
  if (options.exactDistanceFilter) {
    ++candidates.stats.exactDistanceCheckCount;
    if (!withinActivationDistance(
            squaredDistance, options.activationDistance)) {
      return;
    }
  }

  candidates.pointTriangleCandidates.push_back(
      PointTriangleCandidate{point, triangleIndex, squaredDistance});
}

//==============================================================================
inline void maybeAddEdgeEdgeCandidate(
    ContactCandidateSet& candidates,
    std::span<const Eigen::Vector3d> positions,
    std::size_t edgeA,
    std::size_t edgeB,
    const ContactCandidateOptions& options)
{
  if (edgeB < edgeA) {
    std::swap(edgeA, edgeB);
  }

  const auto& a = candidates.surfaceEdges[edgeA];
  const auto& b = candidates.surfaceEdges[edgeB];
  if (options.excludeAdjacentEdges && edgesAreAdjacent(a, b)) {
    ++candidates.stats.adjacentEdgeEdgeRejectCount;
    return;
  }

  const double squaredDistance = edgeEdgeSquaredDistance(
                                     positions[a.nodeA],
                                     positions[a.nodeB],
                                     positions[b.nodeA],
                                     positions[b.nodeB])
                                     .squaredDistance;
  if (options.exactDistanceFilter) {
    ++candidates.stats.exactDistanceCheckCount;
    if (!withinActivationDistance(
            squaredDistance, options.activationDistance)) {
      return;
    }
  }

  candidates.edgeEdgeCandidates.push_back(
      EdgeEdgeCandidate{edgeA, edgeB, squaredDistance});
}

//==============================================================================
inline void finishCandidateSet(ContactCandidateSet& candidates)
{
  sortAndDedupe(candidates.pointTriangleCandidates);
  sortAndDedupe(candidates.edgeEdgeCandidates);
  candidates.stats.pointTriangleCandidateCount
      = candidates.pointTriangleCandidates.size();
  candidates.stats.edgeEdgeCandidateCount
      = candidates.edgeEdgeCandidates.size();
}

} // namespace detail

//==============================================================================
inline std::vector<SurfaceEdge> makeUniqueSurfaceEdges(
    std::span<const DeformableSurfaceTriangle> triangles)
{
  std::vector<SurfaceEdge> edges;
  edges.reserve(3 * triangles.size());

  for (const auto& triangle : triangles) {
    edges.push_back(detail::makeSurfaceEdge(triangle.nodeA, triangle.nodeB));
    edges.push_back(detail::makeSurfaceEdge(triangle.nodeB, triangle.nodeC));
    edges.push_back(detail::makeSurfaceEdge(triangle.nodeC, triangle.nodeA));
  }

  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

//==============================================================================
inline ContactCandidateSet buildContactCandidatesBruteForce(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options)
{
  ContactCandidateSet candidates;
  candidates.surfaceEdges = makeUniqueSurfaceEdges(triangles);
  candidates.stats.pointCount = positions.size();
  candidates.stats.triangleCount = triangles.size();
  candidates.stats.edgeCount = candidates.surfaceEdges.size();

  for (std::size_t point = 0; point < positions.size(); ++point) {
    for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
      detail::maybeAddPointTriangleCandidate(
          candidates, positions, triangles, point, triangle, options);
    }
  }

  for (std::size_t edgeA = 0; edgeA < candidates.surfaceEdges.size(); ++edgeA) {
    for (std::size_t edgeB = edgeA + 1; edgeB < candidates.surfaceEdges.size();
         ++edgeB) {
      detail::maybeAddEdgeEdgeCandidate(
          candidates, positions, edgeA, edgeB, options);
    }
  }

  detail::finishCandidateSet(candidates);
  return candidates;
}

//==============================================================================
inline ContactCandidateSet buildContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options)
{
  ContactCandidateSet candidates;
  candidates.surfaceEdges = makeUniqueSurfaceEdges(triangles);
  candidates.stats.pointCount = positions.size();
  candidates.stats.triangleCount = triangles.size();
  candidates.stats.edgeCount = candidates.surfaceEdges.size();

  const double margin = 0.5 * detail::nonnegativeActivationDistance(options);

  std::vector<detail::SweepItem> pointItems;
  pointItems.reserve(positions.size());
  for (std::size_t point = 0; point < positions.size(); ++point) {
    pointItems.push_back(
        detail::SweepItem{
            point, detail::makePointAabb(positions[point], margin)});
  }

  std::vector<detail::SweepItem> triangleItems;
  triangleItems.reserve(triangles.size());
  for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
    const auto& t = triangles[triangle];
    triangleItems.push_back(
        detail::SweepItem{
            triangle,
            detail::makeTriangleAabb(
                positions[t.nodeA],
                positions[t.nodeB],
                positions[t.nodeC],
                margin)});
  }

  detail::visitSweepPairs(
      std::move(pointItems),
      std::move(triangleItems),
      [&](const std::size_t point, const std::size_t triangle) {
        ++candidates.stats.broadPhaseOverlapCount;
        detail::maybeAddPointTriangleCandidate(
            candidates, positions, triangles, point, triangle, options);
      });

  std::vector<detail::SweepItem> edgeItems;
  edgeItems.reserve(candidates.surfaceEdges.size());
  for (std::size_t edge = 0; edge < candidates.surfaceEdges.size(); ++edge) {
    const auto& e = candidates.surfaceEdges[edge];
    edgeItems.push_back(
        detail::SweepItem{
            edge,
            detail::makeSegmentAabb(
                positions[e.nodeA], positions[e.nodeB], margin)});
  }

  detail::visitSelfSweepPairs(
      std::move(edgeItems),
      [&](const std::size_t edgeA, const std::size_t edgeB) {
        ++candidates.stats.broadPhaseOverlapCount;
        detail::maybeAddEdgeEdgeCandidate(
            candidates, positions, edgeA, edgeB, options);
      });

  detail::finishCandidateSet(candidates);
  return candidates;
}

} // namespace dart::simulation::experimental::detail::deformable_contact
