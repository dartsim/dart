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

#ifndef DART_CONSTRAINT_DETAIL_FRICTIONCONE_HPP_
#define DART_CONSTRAINT_DETAIL_FRICTIONCONE_HPP_

#include <Eigen/Core>

#include <cstdint>

namespace dart::constraint::detail {

/// Internal building blocks; these types carry no API stability promise.
enum class FrictionConeLaw
{
  Ellipse,
  Box
};

struct FrictionCone
{
  Eigen::Vector2d mu = Eigen::Vector2d::Ones();
  FrictionConeLaw law = FrictionConeLaw::Ellipse;
};

struct LocalSolveResult
{
  Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
  bool certified = false;
  std::uint64_t numLocalFallbacks = 0;
  std::uint64_t numQpSolves = 0;
  /// Normal shift and regularization are in the caller's objective units.
  double normalShift = 0.0;
  /// Certificate and returned velocity use H + regularization * I.
  double regularization = 0.0;
};

/// Solve min 0.5 * lambda.transpose() * H * lambda + c.dot(lambda) in K.
/// H must be symmetric positive semidefinite. A singular block receives
/// 1e-12 * trace(H) on its diagonal (1e-12 for an all-zero block).
/// A shift below the double range is raised to the smallest positive double.
/// Extreme objectives use a lossless common power-of-two scale. Unrepresentable
/// scaling or regularized caller diagonals return certified=false.
/// Invalid inputs return certified=false.
/// All numeric paths use fixed-size storage and a deterministic iteration
/// order.
LocalSolveResult solveConeQp(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const FrictionCone& cone);

/// Solve the non-associated law by Illinois iteration on the normal shift.
/// A previous shift may be supplied as a warm start. Local fallback counts
/// include every cone QP used by the root finder.
LocalSolveResult solveExactContact(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const FrictionCone& cone,
    double normalShift = 0.0);

/// Euclidean projection (not radial clipping) onto the selected cone.
Eigen::Vector3d projectCone(
    const Eigen::Vector3d& impulse, const FrictionCone& cone);

/// Return velocity + B(velocity), including B_box for the box law.
Eigen::Vector3d deSaxce(
    const Eigen::Vector3d& velocity, const FrictionCone& cone);

/// Primal infeasibility in normal-impulse units; zero-axis tangents must be
/// zero.
double coneViolation(const Eigen::Vector3d& impulse, const FrictionCone& cone);

/// a * ||lambda - projection(lambda - shiftedVelocity/a)|| in velocity units.
/// The associated ablation uses the unshifted velocity.
double contactViolation(
    const Eigen::Vector3d& impulse,
    const Eigen::Vector3d& velocity,
    double maxDiagonal,
    const FrictionCone& cone,
    bool associated = false);

/// Relative primal, dual and complementarity certificate. H must be symmetric
/// positive semidefinite; singular blocks are accepted without regularization.
/// H and c are normalized by their common maximum absolute coefficient, and
/// the impulse scale uses max(|lambda|, max|c| / max|H|) when H is nonzero.
/// The dual scale uses |H|*|lambda| + |c|, so cancellation is judged relative
/// to the problem data. Call with the effective regularized matrix of a solve.
bool coneQpCertificate(
    const Eigen::Matrix3d& H,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& impulse,
    const FrictionCone& cone,
    double tolerance = 1e-10);

} // namespace dart::constraint::detail

#endif // DART_CONSTRAINT_DETAIL_FRICTIONCONE_HPP_
