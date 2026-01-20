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

#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/dynamics/spatial_math.hpp>
#include <dart/simulation/experimental/export.hpp>

namespace dart::simulation::experimental::dynamics {

/// Compute motion subspace matrix S for a Fixed joint (0-DOF)
/// Returns empty matrix (6x0)
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace computeFixedMotionSubspace();

/// Compute motion subspace matrix S for a Revolute joint (1-DOF)
/// S = [axis; 0] (pure rotation)
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace1
computeRevoluteMotionSubspace(const Eigen::Vector3d& axis);

/// Compute motion subspace matrix S for a Prismatic joint (1-DOF)
/// S = [0; axis] (pure translation)
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace1
computePrismaticMotionSubspace(const Eigen::Vector3d& axis);

/// Compute motion subspace matrix S for a Screw joint (1-DOF)
/// S = [axis; pitch * axis] (coupled rotation + translation)
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace1
computeScrewMotionSubspace(const Eigen::Vector3d& axis, double pitch);

/// Compute motion subspace matrix S for a Universal joint (2-DOF)
/// S = [axis1, R(angle1)*axis2; 0, 0]
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace2
computeUniversalMotionSubspace(
    const Eigen::Vector3d& axis1,
    const Eigen::Vector3d& axis2InParentFrame,
    double angle1ForAxis2Rotation);

/// Compute motion subspace matrix S for a Ball joint (3-DOF)
/// S = [I_3x3; 0_3x3] (3 rotational DOFs)
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace3 computeBallMotionSubspace();

/// Compute motion subspace matrix S for a Planar joint (3-DOF)
/// S has 2 in-plane translations + 1 rotation about normal
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace3 computePlanarMotionSubspace(
    const Eigen::Vector3d& planeNormal, const Eigen::Vector3d& inPlaneAxis);

/// Compute motion subspace matrix S for a Free joint (6-DOF)
/// S = I_6x6 (full 6-DOF motion)
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace6 computeFreeMotionSubspace();

/// Compute motion subspace matrix S based on joint type
/// Dispatches to type-specific functions
[[nodiscard]] DART_EXPERIMENTAL_API MotionSubspace
computeMotionSubspace(const comps::Joint& joint);

} // namespace dart::simulation::experimental::dynamics
