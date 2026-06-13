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

#include <dart/simulation/body/deformable_body_options.hpp>
#include <dart/simulation/detail/deformable_contact/primitive_distance.hpp>

#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include <cassert>
#include <cstddef>

namespace dart::simulation::detail::deformable_contact {

struct SurfaceEdge
{
  std::size_t nodeA = 0;
  std::size_t nodeB = 0;

  [[nodiscard]] friend bool operator==(
      const SurfaceEdge& lhs, const SurfaceEdge& rhs) = default;

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
  /// Static builders store the current-pose squared distance. Motion-aware
  /// builders store the minimum endpoint squared distance as representative
  /// metadata; inclusion comes from swept-AABB overlap, not endpoint distance.
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
  /// Static builders store the current-pose squared distance. Motion-aware
  /// builders store the minimum endpoint squared distance as representative
  /// metadata; inclusion comes from swept-AABB overlap, not endpoint distance.
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
  using SurfaceEdgeAllocator = ::dart::common::StlAllocator<SurfaceEdge>;
  using PointTriangleAllocator
      = ::dart::common::StlAllocator<PointTriangleCandidate>;
  using EdgeEdgeAllocator = ::dart::common::StlAllocator<EdgeEdgeCandidate>;
  using SurfaceEdgeVector = std::vector<SurfaceEdge, SurfaceEdgeAllocator>;
  using PointTriangleVector
      = std::vector<PointTriangleCandidate, PointTriangleAllocator>;
  using EdgeEdgeVector = std::vector<EdgeEdgeCandidate, EdgeEdgeAllocator>;

  ContactCandidateSet() = default;

  explicit ContactCandidateSet(::dart::common::MemoryAllocator& allocator)
    : surfaceEdges(SurfaceEdgeAllocator{allocator}),
      pointTriangleCandidates(PointTriangleAllocator{allocator}),
      edgeEdgeCandidates(EdgeEdgeAllocator{allocator})
  {
  }

  SurfaceEdgeVector surfaceEdges;
  PointTriangleVector pointTriangleCandidates;
  EdgeEdgeVector edgeEdgeCandidates;
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

struct ContactCandidateSweepScratch
{
  using SweepItemAllocator = ::dart::common::StlAllocator<SweepItem>;
  using SweepLinkAllocator = ::dart::common::StlAllocator<std::size_t>;
  using SweepItemVector = std::vector<SweepItem, SweepItemAllocator>;
  using SweepLinkVector = std::vector<std::size_t, SweepLinkAllocator>;

  ContactCandidateSweepScratch() = default;

  explicit ContactCandidateSweepScratch(
      ::dart::common::MemoryAllocator& allocator)
    : pointItems(SweepItemAllocator{allocator}),
      triangleItems(SweepItemAllocator{allocator}),
      edgeItems(SweepItemAllocator{allocator}),
      sweepLinks(SweepLinkAllocator{allocator})
  {
  }

  explicit ContactCandidateSweepScratch(
      const ContactCandidateSet::SurfaceEdgeAllocator& allocator)
    : pointItems(SweepItemAllocator{allocator}),
      triangleItems(SweepItemAllocator{allocator}),
      edgeItems(SweepItemAllocator{allocator}),
      sweepLinks(SweepLinkAllocator{allocator})
  {
  }

  SweepItemVector pointItems;
  SweepItemVector triangleItems;
  SweepItemVector edgeItems;
  // Reusable next-index live list for the cross-set sweep-and-prune traversal
  // (sized to the right-hand-side item count per call).
  SweepLinkVector sweepLinks;
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
inline void clearContactCandidateSet(ContactCandidateSet& candidates)
{
  candidates.surfaceEdges.clear();
  candidates.pointTriangleCandidates.clear();
  candidates.edgeEdgeCandidates.clear();
  candidates.stats = ContactCandidateStats{};
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
inline CandidateAabb makeSweptPointAabb(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const double margin)
{
  CandidateAabb aabb;
  aabb.min = start.cwiseMin(end);
  aabb.max = start.cwiseMax(end);
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
inline CandidateAabb makeSweptSegmentAabb(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const double margin)
{
  CandidateAabb aabb;
  aabb.min = aStart.cwiseMin(aEnd).cwiseMin(bStart).cwiseMin(bEnd);
  aabb.max = aStart.cwiseMax(aEnd).cwiseMax(bStart).cwiseMax(bEnd);
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
inline CandidateAabb makeSweptTriangleAabb(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const double margin)
{
  CandidateAabb aabb;
  aabb.min = aStart.cwiseMin(aEnd)
                 .cwiseMin(bStart)
                 .cwiseMin(bEnd)
                 .cwiseMin(cStart)
                 .cwiseMin(cEnd);
  aabb.max = aStart.cwiseMax(aEnd)
                 .cwiseMax(bStart)
                 .cwiseMax(bEnd)
                 .cwiseMax(cStart)
                 .cwiseMax(cEnd);
  aabb.expand(margin);
  return aabb;
}

//==============================================================================
inline void sortSweepItems(std::span<SweepItem> items)
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
// Cross-set broad-phase sweep ("sweep and prune"). For each lhs in ascending
// min-x order, visits every rhs whose AABB overlaps it, in ascending rhs min-x
// order. This is a full active-set sweep: an rhs whose max-x has fallen behind
// the sweep line (max-x < lhs min-x, so it can never overlap this or a later
// lhs since lhs min-x only grows) is unlinked from a live list, so a long-lived
// early interval no longer forces the expired intervals behind it to be
// rescanned for every lhs. The visited (lhs, rhs) sequence is identical to a
// naive nested scan -- only the skipped non-overlapping work differs.
// `linkScratch` is reusable next-index storage; it is resized to the rhs count.
template <typename Visitor, typename LinkScratch>
void visitSweepPairs(
    std::span<SweepItem> lhsItems,
    std::span<SweepItem> rhsItems,
    Visitor&& visitor,
    LinkScratch& linkScratch)
{
  sortSweepItems(lhsItems);
  sortSweepItems(rhsItems);

  // `count` doubles as the one-past-last "end" sentinel for the live list;
  // `kNoPrev` marks "the current item is the live head" during a walk.
  const std::size_t count = rhsItems.size();
  constexpr std::size_t kNoPrev = std::numeric_limits<std::size_t>::max();
  linkScratch.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    linkScratch[i] = i + 1; // next live index; the last entry points at `count`
  }
  std::size_t head = 0; // when count == 0 this equals the end sentinel

  for (const auto& lhs : lhsItems) {
    std::size_t prev = kNoPrev;
    std::size_t cur = head;
    while (cur != count) {
      const auto& rhs = rhsItems[cur];
      if (rhs.aabb.min.x() > lhs.aabb.max.x()) {
        break; // every remaining rhs starts after this lhs ends
      }
      if (rhs.aabb.max.x() < lhs.aabb.min.x()) {
        // Expired for this and every later lhs: unlink from the live list so it
        // is never revisited.
        const std::size_t next = linkScratch[cur];
        if (prev == kNoPrev) {
          head = next;
        } else {
          linkScratch[prev] = next;
        }
        cur = next;
        continue;
      }
      if (lhs.aabb.overlaps(rhs.aabb)) {
        visitor(lhs.id, rhs.id);
      }
      prev = cur;
      cur = linkScratch[cur];
    }
  }
}

//==============================================================================
// Convenience overload that owns the live-list scratch, for tests and
// standalone callers; hot paths pass reusable scratch via the overload above.
template <typename Visitor>
void visitSweepPairs(
    std::span<SweepItem> lhsItems,
    std::span<SweepItem> rhsItems,
    Visitor&& visitor)
{
  std::vector<std::size_t> linkScratch;
  visitSweepPairs(
      lhsItems, rhsItems, std::forward<Visitor>(visitor), linkScratch);
}

//==============================================================================
template <typename Visitor>
void visitSelfSweepPairs(std::span<SweepItem> items, Visitor&& visitor)
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
template <typename Allocator>
inline void sortAndDedupe(
    std::vector<PointTriangleCandidate, Allocator>& candidates)
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
template <typename Allocator>
inline void sortAndDedupe(std::vector<EdgeEdgeCandidate, Allocator>& candidates)
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
inline double endpointPointTriangleSquaredDistance(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const std::size_t point,
    const std::size_t triangleIndex)
{
  const auto& triangle = triangles[triangleIndex];
  const double startDistance = pointTriangleSquaredDistance(
                                   positionsStart[point],
                                   positionsStart[triangle.nodeA],
                                   positionsStart[triangle.nodeB],
                                   positionsStart[triangle.nodeC])
                                   .squaredDistance;
  const double endDistance = pointTriangleSquaredDistance(
                                 positionsEnd[point],
                                 positionsEnd[triangle.nodeA],
                                 positionsEnd[triangle.nodeB],
                                 positionsEnd[triangle.nodeC])
                                 .squaredDistance;
  return std::min(startDistance, endDistance);
}

//==============================================================================
inline void maybeAddMotionAwarePointTriangleCandidate(
    ContactCandidateSet& candidates,
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
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

  const double squaredDistance = endpointPointTriangleSquaredDistance(
      positionsStart, positionsEnd, triangles, point, triangleIndex);
  candidates.stats.exactDistanceCheckCount += 2;

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
inline double endpointEdgeEdgeSquaredDistance(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const SurfaceEdge> surfaceEdges,
    const std::size_t edgeA,
    const std::size_t edgeB)
{
  const auto& a = surfaceEdges[edgeA];
  const auto& b = surfaceEdges[edgeB];
  const double startDistance = edgeEdgeSquaredDistance(
                                   positionsStart[a.nodeA],
                                   positionsStart[a.nodeB],
                                   positionsStart[b.nodeA],
                                   positionsStart[b.nodeB])
                                   .squaredDistance;
  const double endDistance = edgeEdgeSquaredDistance(
                                 positionsEnd[a.nodeA],
                                 positionsEnd[a.nodeB],
                                 positionsEnd[b.nodeA],
                                 positionsEnd[b.nodeB])
                                 .squaredDistance;
  return std::min(startDistance, endDistance);
}

//==============================================================================
inline void maybeAddMotionAwareEdgeEdgeCandidate(
    ContactCandidateSet& candidates,
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
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

  const double squaredDistance = endpointEdgeEdgeSquaredDistance(
      positionsStart, positionsEnd, candidates.surfaceEdges, edgeA, edgeB);
  candidates.stats.exactDistanceCheckCount += 2;

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
template <typename EdgeVector>
inline void buildUniqueSurfaceEdges(
    std::span<const DeformableSurfaceTriangle> triangles, EdgeVector& edges)
{
  edges.clear();
  edges.reserve(3 * triangles.size());

  for (const auto& triangle : triangles) {
    edges.push_back(detail::makeSurfaceEdge(triangle.nodeA, triangle.nodeB));
    edges.push_back(detail::makeSurfaceEdge(triangle.nodeB, triangle.nodeC));
    edges.push_back(detail::makeSurfaceEdge(triangle.nodeC, triangle.nodeA));
  }

  std::sort(edges.begin(), edges.end());
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
}

//==============================================================================
inline std::vector<SurfaceEdge> makeUniqueSurfaceEdges(
    std::span<const DeformableSurfaceTriangle> triangles)
{
  std::vector<SurfaceEdge> edges;
  buildUniqueSurfaceEdges(triangles, edges);
  return edges;
}

//==============================================================================
inline void buildContactCandidatesBruteForce(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options,
    ContactCandidateSet& candidates)
{
  detail::clearContactCandidateSet(candidates);
  buildUniqueSurfaceEdges(triangles, candidates.surfaceEdges);
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
}

//==============================================================================
inline ContactCandidateSet buildContactCandidatesBruteForce(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options)
{
  ContactCandidateSet candidates;
  buildContactCandidatesBruteForce(positions, triangles, options, candidates);
  return candidates;
}

//==============================================================================
inline void buildContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options,
    ContactCandidateSet& candidates,
    detail::ContactCandidateSweepScratch& scratch)
{
  detail::clearContactCandidateSet(candidates);
  buildUniqueSurfaceEdges(triangles, candidates.surfaceEdges);
  candidates.stats.pointCount = positions.size();
  candidates.stats.triangleCount = triangles.size();
  candidates.stats.edgeCount = candidates.surfaceEdges.size();

  const double margin = 0.5 * detail::nonnegativeActivationDistance(options);

  scratch.pointItems.clear();
  scratch.pointItems.reserve(positions.size());
  for (std::size_t point = 0; point < positions.size(); ++point) {
    scratch.pointItems.push_back(
        detail::SweepItem{
            point, detail::makePointAabb(positions[point], margin)});
  }

  scratch.triangleItems.clear();
  scratch.triangleItems.reserve(triangles.size());
  for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
    const auto& t = triangles[triangle];
    scratch.triangleItems.push_back(
        detail::SweepItem{
            triangle,
            detail::makeTriangleAabb(
                positions[t.nodeA],
                positions[t.nodeB],
                positions[t.nodeC],
                margin)});
  }

  detail::visitSweepPairs(
      scratch.pointItems,
      scratch.triangleItems,
      [&](const std::size_t point, const std::size_t triangle) {
        ++candidates.stats.broadPhaseOverlapCount;
        detail::maybeAddPointTriangleCandidate(
            candidates, positions, triangles, point, triangle, options);
      },
      scratch.sweepLinks);

  scratch.edgeItems.clear();
  scratch.edgeItems.reserve(candidates.surfaceEdges.size());
  for (std::size_t edge = 0; edge < candidates.surfaceEdges.size(); ++edge) {
    const auto& e = candidates.surfaceEdges[edge];
    scratch.edgeItems.push_back(
        detail::SweepItem{
            edge,
            detail::makeSegmentAabb(
                positions[e.nodeA], positions[e.nodeB], margin)});
  }

  detail::visitSelfSweepPairs(
      scratch.edgeItems, [&](const std::size_t edgeA, const std::size_t edgeB) {
        ++candidates.stats.broadPhaseOverlapCount;
        detail::maybeAddEdgeEdgeCandidate(
            candidates, positions, edgeA, edgeB, options);
      });

  detail::finishCandidateSet(candidates);
}

//==============================================================================
inline void buildContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options,
    ContactCandidateSet& candidates)
{
  detail::ContactCandidateSweepScratch scratch(
      candidates.surfaceEdges.get_allocator());
  buildContactCandidatesSweep(
      positions, triangles, options, candidates, scratch);
}

//==============================================================================
inline ContactCandidateSet buildContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positions,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options)
{
  ContactCandidateSet candidates;
  buildContactCandidatesSweep(positions, triangles, options, candidates);
  return candidates;
}

//==============================================================================
/// Builds a conservative motion-aware contact candidate set by sweeping
/// primitive AABBs over the start and end positions.
///
/// Unlike the static builders, this function does not reject candidates whose
/// endpoint distances exceed `activationDistance`; a fast primitive can be far
/// at both endpoints while crossing another primitive during the step. The
/// candidate `squaredDistance` fields therefore store the minimum endpoint
/// squared distance only as representative metadata. Conservative CCD remains
/// the downstream safety gate.
inline void buildMotionAwareContactCandidatesBruteForce(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options,
    ContactCandidateSet& candidates)
{
  assert(positionsStart.size() == positionsEnd.size());

  detail::clearContactCandidateSet(candidates);
  buildUniqueSurfaceEdges(triangles, candidates.surfaceEdges);
  candidates.stats.pointCount = positionsStart.size();
  candidates.stats.triangleCount = triangles.size();
  candidates.stats.edgeCount = candidates.surfaceEdges.size();

  const double margin = 0.5 * detail::nonnegativeActivationDistance(options);

  for (std::size_t point = 0; point < positionsStart.size(); ++point) {
    const auto pointAabb = detail::makeSweptPointAabb(
        positionsStart[point], positionsEnd[point], margin);
    for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
      const auto& t = triangles[triangle];
      const auto triangleAabb = detail::makeSweptTriangleAabb(
          positionsStart[t.nodeA],
          positionsEnd[t.nodeA],
          positionsStart[t.nodeB],
          positionsEnd[t.nodeB],
          positionsStart[t.nodeC],
          positionsEnd[t.nodeC],
          margin);
      if (!pointAabb.overlaps(triangleAabb)) {
        continue;
      }

      ++candidates.stats.broadPhaseOverlapCount;
      detail::maybeAddMotionAwarePointTriangleCandidate(
          candidates,
          positionsStart,
          positionsEnd,
          triangles,
          point,
          triangle,
          options);
    }
  }

  for (std::size_t edgeA = 0; edgeA < candidates.surfaceEdges.size(); ++edgeA) {
    const auto& a = candidates.surfaceEdges[edgeA];
    const auto edgeAabb = detail::makeSweptSegmentAabb(
        positionsStart[a.nodeA],
        positionsEnd[a.nodeA],
        positionsStart[a.nodeB],
        positionsEnd[a.nodeB],
        margin);
    for (std::size_t edgeB = edgeA + 1; edgeB < candidates.surfaceEdges.size();
         ++edgeB) {
      const auto& b = candidates.surfaceEdges[edgeB];
      const auto edgeBAabb = detail::makeSweptSegmentAabb(
          positionsStart[b.nodeA],
          positionsEnd[b.nodeA],
          positionsStart[b.nodeB],
          positionsEnd[b.nodeB],
          margin);
      if (!edgeAabb.overlaps(edgeBAabb)) {
        continue;
      }

      ++candidates.stats.broadPhaseOverlapCount;
      detail::maybeAddMotionAwareEdgeEdgeCandidate(
          candidates, positionsStart, positionsEnd, edgeA, edgeB, options);
    }
  }

  detail::finishCandidateSet(candidates);
}

//==============================================================================
inline ContactCandidateSet buildMotionAwareContactCandidatesBruteForce(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options)
{
  ContactCandidateSet candidates;
  buildMotionAwareContactCandidatesBruteForce(
      positionsStart, positionsEnd, triangles, options, candidates);
  return candidates;
}

//==============================================================================
/// Builds a conservative motion-aware contact candidate set with swept-AABB
/// broad-phase pruning over the start and end positions.
///
/// The `exactDistanceFilter` flag is intentionally not used for endpoint
/// rejection here. The builder records endpoint distances as metadata only so
/// fast crossings are not culled before the conservative CCD line-search gate.
inline void buildMotionAwareContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options,
    ContactCandidateSet& candidates,
    detail::ContactCandidateSweepScratch& scratch)
{
  assert(positionsStart.size() == positionsEnd.size());

  detail::clearContactCandidateSet(candidates);
  buildUniqueSurfaceEdges(triangles, candidates.surfaceEdges);
  candidates.stats.pointCount = positionsStart.size();
  candidates.stats.triangleCount = triangles.size();
  candidates.stats.edgeCount = candidates.surfaceEdges.size();

  const double margin = 0.5 * detail::nonnegativeActivationDistance(options);

  scratch.pointItems.clear();
  scratch.pointItems.reserve(positionsStart.size());
  for (std::size_t point = 0; point < positionsStart.size(); ++point) {
    scratch.pointItems.push_back(
        detail::SweepItem{
            point,
            detail::makeSweptPointAabb(
                positionsStart[point], positionsEnd[point], margin)});
  }

  scratch.triangleItems.clear();
  scratch.triangleItems.reserve(triangles.size());
  for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle) {
    const auto& t = triangles[triangle];
    scratch.triangleItems.push_back(
        detail::SweepItem{
            triangle,
            detail::makeSweptTriangleAabb(
                positionsStart[t.nodeA],
                positionsEnd[t.nodeA],
                positionsStart[t.nodeB],
                positionsEnd[t.nodeB],
                positionsStart[t.nodeC],
                positionsEnd[t.nodeC],
                margin)});
  }

  detail::visitSweepPairs(
      scratch.pointItems,
      scratch.triangleItems,
      [&](const std::size_t point, const std::size_t triangle) {
        ++candidates.stats.broadPhaseOverlapCount;
        detail::maybeAddMotionAwarePointTriangleCandidate(
            candidates,
            positionsStart,
            positionsEnd,
            triangles,
            point,
            triangle,
            options);
      },
      scratch.sweepLinks);

  scratch.edgeItems.clear();
  scratch.edgeItems.reserve(candidates.surfaceEdges.size());
  for (std::size_t edge = 0; edge < candidates.surfaceEdges.size(); ++edge) {
    const auto& e = candidates.surfaceEdges[edge];
    scratch.edgeItems.push_back(
        detail::SweepItem{
            edge,
            detail::makeSweptSegmentAabb(
                positionsStart[e.nodeA],
                positionsEnd[e.nodeA],
                positionsStart[e.nodeB],
                positionsEnd[e.nodeB],
                margin)});
  }

  detail::visitSelfSweepPairs(
      scratch.edgeItems, [&](const std::size_t edgeA, const std::size_t edgeB) {
        ++candidates.stats.broadPhaseOverlapCount;
        detail::maybeAddMotionAwareEdgeEdgeCandidate(
            candidates, positionsStart, positionsEnd, edgeA, edgeB, options);
      });

  detail::finishCandidateSet(candidates);
}

//==============================================================================
inline void buildMotionAwareContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options,
    ContactCandidateSet& candidates)
{
  detail::ContactCandidateSweepScratch scratch(
      candidates.surfaceEdges.get_allocator());
  buildMotionAwareContactCandidatesSweep(
      positionsStart, positionsEnd, triangles, options, candidates, scratch);
}

//==============================================================================
inline ContactCandidateSet buildMotionAwareContactCandidatesSweep(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateOptions& options)
{
  ContactCandidateSet candidates;
  buildMotionAwareContactCandidatesSweep(
      positionsStart, positionsEnd, triangles, options, candidates);
  return candidates;
}

} // namespace dart::simulation::detail::deformable_contact
