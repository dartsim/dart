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

#include <dart/simulation/experimental/detail/deformable_vbd/vertex_block_kernel.hpp>

#include <Eigen/Core>

namespace dart::simulation::experimental::detail::deformable_vbd {

/// A static half-space contact plane `{x : normal . x >= offset}` (the free
/// side). A vertex penetrates when `normal . x < offset`.
struct ContactPlane
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
  double offset = 0.0;
  double stiffness = 0.0;
};

//==============================================================================
/// Add the VBD penalty-contact term for a vertex against a static half-space to
/// `block`. The contact energy is `E_c = (k_c / 2) d^2` with penetration depth
/// `d = max(0, offset - normal . x)`, so contact is active only while the
/// vertex is below the plane:
///   f += k_c d normal,   H += k_c normal normal^T.
/// The Hessian contribution is the rank-1 positive-semidefinite `k_c n n^T`,
/// keeping the per-vertex block positive-definite when combined with inertia.
/// `normal` is assumed unit length.
inline void addHalfSpacePenaltyContact(
    VertexBlock& block,
    const Eigen::Vector3d& position,
    const ContactPlane& plane)
{
  const double gap = plane.normal.dot(position) - plane.offset;
  if (gap >= 0.0 || plane.stiffness <= 0.0) {
    return;
  }
  const double penetration = -gap;
  block.force.noalias() += plane.stiffness * penetration * plane.normal;
  block.hessian.noalias()
      += plane.stiffness * (plane.normal * plane.normal.transpose());
}

} // namespace dart::simulation::experimental::detail::deformable_vbd
