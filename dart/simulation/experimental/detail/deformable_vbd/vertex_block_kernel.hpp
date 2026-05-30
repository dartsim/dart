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

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>

#include <cmath>

namespace dart::simulation::experimental::detail::deformable_vbd {

/// Per-vertex local quantities for one Vertex Block Descent (VBD) update.
///
/// VBD minimizes the variational implicit-Euler objective
///   G(x) = sum_i (m_i / (2 h^2)) ||x_i - y_i||^2 + E_elastic(x)
/// by sweeping one vertex block at a time. For a vertex `i`, holding all other
/// vertices fixed, it takes one regularized Newton step on `G` restricted to
/// `x_i` using the local force and Hessian
///   f_i = -dG/dx_i,   H_i = d^2 G / dx_i^2,   x_i += H_i^{-1} f_i.
///
/// `force` accumulates `f_i` (the negative gradient, i.e. the physical force on
/// the vertex). `hessian` accumulates `H_i`. The inertia term plus
/// PSD-projected element blocks keep `H_i` symmetric positive-definite, so the
/// fixed-size 3x3 block solve is always well posed. This struct is the internal
/// accumulator only; it is not part of the public solver surface.
struct VertexBlock
{
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();

  void reset() noexcept
  {
    force.setZero();
    hessian.setZero();
  }
};

/// Smallest spring length treated as a usable direction. Matches the guard used
/// by the existing deformable objective so degenerate edges are skipped rather
/// than producing an undefined direction.
inline constexpr double kMinSpringLength = 1e-12;

//==============================================================================
/// Add the variational implicit-Euler inertia term for a vertex to `block`.
///
/// G_inertia(x_i) = (m / (2 h^2)) ||x_i - y||^2, so
///   f_i += -(m / h^2)(x_i - y),   H_i += (m / h^2) I_3.
///
/// The added Hessian contribution is positive-definite for `mass > 0` and
/// `timeStep > 0`, which anchors the local solve.
inline void addInertiaTerm(
    VertexBlock& block,
    double mass,
    double timeStep,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& inertialTarget)
{
  const double invDt2 = 1.0 / (timeStep * timeStep);
  const double coeff = mass * invDt2;
  block.force.noalias() -= coeff * (position - inertialTarget);
  block.hessian.diagonal().array() += coeff;
}

//==============================================================================
/// Add the distance-spring elastic term for the edge (`self`, `other`) to
/// `block`, accumulating the force on `self` and `self`'s 3x3 Hessian block.
///
/// For E = (k/2)(l - L)^2 with `delta = other - self`, `l = ||delta||`,
/// `n = delta / l`:
///   f_self += k (l - L) n,
///   H_self += k [ n n^T + (1 - L/l)(I - n n^T) ].
///
/// When `clampToPsd` is true (the VBD robustness choice), the transverse
/// (geometric-stiffness) factor `(1 - L/l)` is clamped to be non-negative so
/// that the returned block is positive semidefinite even under compression
/// (`l < L`); the rank-1 `n n^T` term always carries the positive `k` factor.
/// When false, the exact (possibly indefinite) energy Hessian is added, which
/// is what finite-difference derivative checks compare against.
///
/// Degenerate edges (`l <= kMinSpringLength`) contribute nothing, matching the
/// existing deformable objective's direction guard.
inline void addSpringTerm(
    VertexBlock& block,
    double stiffness,
    double restLength,
    const Eigen::Vector3d& self,
    const Eigen::Vector3d& other,
    bool clampToPsd = true)
{
  const Eigen::Vector3d delta = other - self;
  const double length = delta.norm();
  if (length <= kMinSpringLength || !std::isfinite(length)) {
    return;
  }

  const Eigen::Vector3d n = delta / length;
  const double stretch = length - restLength;

  // Force on `self`: pulls toward `other` when stretched (stretch > 0).
  block.force.noalias() += stiffness * stretch * n;

  // Energy Hessian block for `self`. The longitudinal term k n n^T is always
  // positive; the transverse term carries the (1 - L/l) geometric-stiffness
  // factor, which is negative under compression and is clamped for the PSD
  // block.
  double transverse = 1.0 - restLength / length;
  if (clampToPsd) {
    transverse = std::max(0.0, transverse);
  }
  const Eigen::Matrix3d nnT = n * n.transpose();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  block.hessian.noalias() += stiffness * (nnT + transverse * (identity - nnT));
}

//==============================================================================
/// Solve the regularized 3x3 block Newton step `delta = H^{-1} f`.
///
/// `regularization >= 0` adds `regularization * I` to the Hessian before the
/// solve to keep it well conditioned (a Levenberg-Marquardt-style damping that
/// also shortens the step). Returns the zero step if the Hessian is not usable
/// (non-finite, non-positive-definite, or a non-finite solution), so a failed
/// block solve leaves the vertex unchanged rather than diverging.
inline Eigen::Vector3d solveVertexBlock(
    const VertexBlock& block, double regularization = 0.0)
{
  Eigen::Matrix3d hessian = block.hessian;
  if (regularization > 0.0) {
    hessian.diagonal().array() += regularization;
  }

  if (!hessian.allFinite() || !block.force.allFinite()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::LDLT<Eigen::Matrix3d> ldlt(hessian);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d delta = ldlt.solve(block.force);
  if (!delta.allFinite()) {
    return Eigen::Vector3d::Zero();
  }
  return delta;
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
