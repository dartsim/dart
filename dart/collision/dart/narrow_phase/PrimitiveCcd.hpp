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

#pragma once

// Primitive-level continuous collision detection for independently moving
// vertices (point-triangle and edge-edge). These are the queries IPC-class and
// deformable-body solvers consume: each vertex follows a linear trajectory
// x(t) = x0 + t (x1 - x0) over the step t in [0, 1], and the query returns a
// conservative time of impact. Rigid meshes are the special case where all
// vertices of a body share one rigid motion sampled at the endpoints.
//
// The default solver is additive conservative advancement (ACCD): it returns a
// time of impact that never overshoots the true first contact, which is the
// safety contract a barrier method requires. An exact coplanarity-cubic solver
// is also provided for validation/diagnostics; it is exact in real arithmetic
// but not conservative under floating point, so it must not be used as a
// runtime default. See docs/dev_tasks/continuous_collision_detection/.

#include <dart/collision/dart/Export.hpp>
#include <dart/collision/dart/Types.hpp>

#include <Eigen/Core>

namespace dart::collision::native {

//==============================================================================
// Additive conservative advancement (ACCD) - conservative, gap-aware default
//==============================================================================

/// Continuous collision between a moving point and a moving triangle.
///
/// The point moves pStart -> pEnd; the triangle vertices move aStart -> aEnd,
/// bStart -> bEnd, cStart -> cEnd. Reports a hit if the point closes to within
/// option.minSeparation of the triangle for some t in [0, 1];
/// result.timeOfImpact is a conservative lower bound on that time.
[[nodiscard]] DART_COLLISION_NATIVE_API bool pointTriangleCcd(
    const Eigen::Vector3d& pStart,
    const Eigen::Vector3d& pEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result);

/// Continuous collision between two moving line segments (edges).
///
/// Edge A spans (aStart..aEnd) -> ... moving as aStart->aEnd' is encoded per
/// endpoint: edge A endpoints are a (aStart at t0, aEnd at t1) and b; edge B
/// endpoints are c and d. Reports a hit if the edges close to within
/// option.minSeparation for some t in [0, 1]; result.timeOfImpact is a
/// conservative lower bound on that time.
[[nodiscard]] DART_COLLISION_NATIVE_API bool edgeEdgeCcd(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result);

//==============================================================================
// Exact coplanarity-cubic solver - VALIDATION ONLY (not conservative under FP)
//==============================================================================

/// Exact point-triangle time of impact via the coplanarity cubic
/// (Provot/Bridson). Exact in real arithmetic but not conservative under
/// floating point; provided for cross-validation and diagnostics, never as a
/// runtime default. Returns the earliest validated coplanarity time in [0, 1].
[[nodiscard]] DART_COLLISION_NATIVE_API bool pointTriangleCcdExact(
    const Eigen::Vector3d& pStart,
    const Eigen::Vector3d& pEnd,
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result);

/// Exact edge-edge time of impact via the coplanarity cubic (Provot/Bridson).
/// Validation only; see pointTriangleCcdExact.
[[nodiscard]] DART_COLLISION_NATIVE_API bool edgeEdgeCcdExact(
    const Eigen::Vector3d& aStart,
    const Eigen::Vector3d& aEnd,
    const Eigen::Vector3d& bStart,
    const Eigen::Vector3d& bEnd,
    const Eigen::Vector3d& cStart,
    const Eigen::Vector3d& cEnd,
    const Eigen::Vector3d& dStart,
    const Eigen::Vector3d& dEnd,
    const CcdOption& option,
    CcdPrimitiveResult& result);

} // namespace dart::collision::native
