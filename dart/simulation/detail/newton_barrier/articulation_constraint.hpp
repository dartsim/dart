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

#include <Eigen/Core>

namespace dart::simulation::detail::newton_barrier {

struct PointConnectionConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
  double squaredNorm = 0.0;
};

struct FixedPointConstraintResult
{
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
  double squaredNorm = 0.0;
};

//==============================================================================
[[nodiscard]] inline PointConnectionConstraintResult pointConnectionConstraint(
    const Eigen::Vector3d& pointA, const Eigen::Vector3d& pointB)
{
  PointConnectionConstraintResult result;
  result.residual = pointA - pointB;
  result.jacobian.leftCols<3>().setIdentity();
  result.jacobian.rightCols<3>() = -Eigen::Matrix3d::Identity();
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

//==============================================================================
[[nodiscard]] inline FixedPointConstraintResult fixedPointConstraint(
    const Eigen::Vector3d& point, const Eigen::Vector3d& target)
{
  FixedPointConstraintResult result;
  result.residual = point - target;
  result.jacobian.setIdentity();
  result.squaredNorm = result.residual.squaredNorm();
  return result;
}

} // namespace dart::simulation::detail::newton_barrier
