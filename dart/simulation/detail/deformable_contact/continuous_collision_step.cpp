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

#include <dart/simulation/detail/deformable_contact/continuous_collision_step.hpp>

#include <dart/collision/native/narrow_phase/primitive_ccd.hpp>

#include <algorithm>

#include <cassert>

namespace dart::simulation::detail::deformable_contact {
namespace {

//==============================================================================
collision::native::CcdOption makeNativeOption(
    const ContinuousCollisionStepOptions& options)
{
  collision::native::CcdOption native;
  native.minSeparation = std::max(0.0, options.minSeparation);
  native.tolerance = std::max(0.0, options.tolerance);
  native.maxIterations = std::max(1, options.maxIterations);
  native.advancement = collision::native::CcdAdvancement::Conservative;
  return native;
}

//==============================================================================
void recordHit(
    ContinuousCollisionStepResult& result,
    const double timeOfImpact,
    const ContinuousCollisionPrimitive primitive)
{
  result.hit = true;
  result.stepBound = std::clamp(timeOfImpact, 0.0, 1.0);
  result.limitingPrimitive = primitive;
  ++result.stats.hits;
  if (result.stepBound <= 0.0) {
    ++result.stats.zeroStepCount;
  }
}

//==============================================================================
void accumulateStats(
    ContinuousCollisionStepStats& total,
    const ContinuousCollisionStepStats& addend)
{
  total.pointTriangleChecks += addend.pointTriangleChecks;
  total.edgeEdgeChecks += addend.edgeEdgeChecks;
  total.hits += addend.hits;
  total.misses += addend.misses;
  total.indeterminate += addend.indeterminate;
  total.zeroStepCount += addend.zeroStepCount;
}

//==============================================================================
void considerCandidate(
    ContinuousCollisionStepResult& aggregate,
    const ContinuousCollisionStepResult& candidate,
    const std::size_t candidateIndex)
{
  accumulateStats(aggregate.stats, candidate.stats);
  if (!candidate.hit) {
    aggregate.indeterminate
        = aggregate.indeterminate || candidate.indeterminate;
    if (candidate.indeterminate) {
      aggregate.stepBound = 0.0;
    }
    return;
  }

  const bool improves
      = !aggregate.hit || candidate.stepBound < aggregate.stepBound;
  if (!improves) {
    return;
  }

  aggregate.hit = true;
  aggregate.stepBound = candidate.stepBound;
  aggregate.limitingPrimitive = candidate.limitingPrimitive;
  aggregate.limitingCandidate = candidateIndex;
}

} // namespace

//==============================================================================
ContinuousCollisionStepResult pointTriangleStepBound(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const ContinuousCollisionStepOptions& options)
{
  ContinuousCollisionStepResult result;
  ++result.stats.pointTriangleChecks;

  collision::native::CcdPrimitiveResult nativeResult;
  const bool hit = collision::native::pointTriangleCcd(
      pointStart,
      pointEnd,
      aStart,
      aEnd,
      bStart,
      bEnd,
      cStart,
      cEnd,
      makeNativeOption(options),
      nativeResult);

  if (hit) {
    recordHit(
        result,
        nativeResult.timeOfImpact,
        ContinuousCollisionPrimitive::PointTriangle);
  } else if (
      nativeResult.status
      == collision::native::CcdPrimitiveStatus::Indeterminate) {
    result.indeterminate = true;
    result.stepBound = 0.0;
    ++result.stats.indeterminate;
  } else {
    ++result.stats.misses;
  }

  return result;
}

//==============================================================================
ContinuousCollisionStepResult edgeEdgeStepBound(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const ContinuousCollisionStepOptions& options)
{
  ContinuousCollisionStepResult result;
  ++result.stats.edgeEdgeChecks;

  collision::native::CcdPrimitiveResult nativeResult;
  const bool hit = collision::native::edgeEdgeCcd(
      aStart,
      aEnd,
      bStart,
      bEnd,
      cStart,
      cEnd,
      dStart,
      dEnd,
      makeNativeOption(options),
      nativeResult);

  if (hit) {
    recordHit(
        result,
        nativeResult.timeOfImpact,
        ContinuousCollisionPrimitive::EdgeEdge);
  } else if (
      nativeResult.status
      == collision::native::CcdPrimitiveStatus::Indeterminate) {
    result.indeterminate = true;
    result.stepBound = 0.0;
    ++result.stats.indeterminate;
  } else {
    ++result.stats.misses;
  }

  return result;
}

//==============================================================================
ContinuousCollisionStepResult contactCandidateStepBound(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateSet& candidates,
    const ContinuousCollisionStepOptions& options)
{
  assert(positionsStart.size() == positionsEnd.size());

  ContinuousCollisionStepResult aggregate;

  for (std::size_t candidateIndex = 0;
       candidateIndex < candidates.pointTriangleCandidates.size();
       ++candidateIndex) {
    const auto& candidate = candidates.pointTriangleCandidates[candidateIndex];
    assert(candidate.point < positionsStart.size());
    assert(candidate.triangle < triangles.size());

    const auto& triangle = triangles[candidate.triangle];
    assert(triangle.nodeA < positionsStart.size());
    assert(triangle.nodeB < positionsStart.size());
    assert(triangle.nodeC < positionsStart.size());

    const auto result = pointTriangleStepBound(
        positionsStart[candidate.point],
        positionsEnd[candidate.point],
        positionsStart[triangle.nodeA],
        positionsEnd[triangle.nodeA],
        positionsStart[triangle.nodeB],
        positionsEnd[triangle.nodeB],
        positionsStart[triangle.nodeC],
        positionsEnd[triangle.nodeC],
        options);
    considerCandidate(aggregate, result, candidateIndex);
  }

  for (std::size_t candidateIndex = 0;
       candidateIndex < candidates.edgeEdgeCandidates.size();
       ++candidateIndex) {
    const auto& candidate = candidates.edgeEdgeCandidates[candidateIndex];
    assert(candidate.edgeA < candidates.surfaceEdges.size());
    assert(candidate.edgeB < candidates.surfaceEdges.size());

    const auto& edgeA = candidates.surfaceEdges[candidate.edgeA];
    const auto& edgeB = candidates.surfaceEdges[candidate.edgeB];
    assert(edgeA.nodeA < positionsStart.size());
    assert(edgeA.nodeB < positionsStart.size());
    assert(edgeB.nodeA < positionsStart.size());
    assert(edgeB.nodeB < positionsStart.size());

    const auto result = edgeEdgeStepBound(
        positionsStart[edgeA.nodeA],
        positionsEnd[edgeA.nodeA],
        positionsStart[edgeA.nodeB],
        positionsEnd[edgeA.nodeB],
        positionsStart[edgeB.nodeA],
        positionsEnd[edgeB.nodeA],
        positionsStart[edgeB.nodeB],
        positionsEnd[edgeB.nodeB],
        options);
    considerCandidate(aggregate, result, candidateIndex);
  }

  return aggregate;
}

} // namespace dart::simulation::detail::deformable_contact
