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

#pragma once

#include <dart/simulation/experimental/detail/deformable_contact/candidate_set.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>

#include <limits>
#include <span>

#include <cstddef>

namespace dart::simulation::experimental::detail::deformable_contact {

enum class ContinuousCollisionPrimitive
{
  None,
  PointTriangle,
  EdgeEdge,
};

struct ContinuousCollisionStepOptions
{
  double minSeparation = 0.0;
  double tolerance = 1e-6;
  int maxIterations = 64;
};

struct ContinuousCollisionStepStats
{
  std::size_t pointTriangleChecks = 0;
  std::size_t edgeEdgeChecks = 0;
  std::size_t hits = 0;
  std::size_t misses = 0;
  std::size_t zeroStepCount = 0;
};

struct ContinuousCollisionStepResult
{
  bool hit = false;
  double stepBound = 1.0;
  ContinuousCollisionPrimitive limitingPrimitive
      = ContinuousCollisionPrimitive::None;
  std::size_t limitingCandidate = std::numeric_limits<std::size_t>::max();
  ContinuousCollisionStepStats stats;
};

/// Conservative step bound for one moving point-triangle pair.
///
/// Each primitive follows a linear trajectory from the start position to the
/// end position over normalized step alpha in [0, 1]. A returned hit means
/// `stepBound` is the largest alpha this internal helper is willing to pass to
/// a future line search; it is conservative with respect to the native CCD
/// primitive query and returns 0 when the pair starts inside `minSeparation`.
[[nodiscard]] DART_EXPERIMENTAL_API ContinuousCollisionStepResult
pointTriangleStepBound(
    const Eigen::Vector3d& pointStart,
    const Eigen::Vector3d& pointEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const ContinuousCollisionStepOptions& options = {});

/// Conservative step bound for one moving edge-edge pair.
[[nodiscard]] DART_EXPERIMENTAL_API ContinuousCollisionStepResult
edgeEdgeStepBound(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const ContinuousCollisionStepOptions& options = {});

/// Conservative minimum step bound over an already assembled contact candidate
/// set.
///
/// This consumes the current static candidate set only. It intentionally does
/// not provide motion-aware broad-phase culling yet; that remains a separate
/// solver-integration slice.
[[nodiscard]] DART_EXPERIMENTAL_API ContinuousCollisionStepResult
contactCandidateStepBound(
    std::span<const Eigen::Vector3d> positionsStart,
    std::span<const Eigen::Vector3d> positionsEnd,
    std::span<const DeformableSurfaceTriangle> triangles,
    const ContactCandidateSet& candidates,
    const ContinuousCollisionStepOptions& options = {});

} // namespace dart::simulation::experimental::detail::deformable_contact
