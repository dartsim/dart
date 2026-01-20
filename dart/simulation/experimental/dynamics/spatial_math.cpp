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

#include <dart/simulation/experimental/dynamics/spatial_math.hpp>

namespace dart::simulation::experimental::dynamics {

SpatialInertia makeSpatialInertia(
    double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia)
{
  SpatialInertia I = SpatialInertia::Zero();

  const Eigen::Matrix3d comSkew = math::makeSkewSymmetric(com);
  const Eigen::Matrix3d mComSkewSq = mass * comSkew * comSkew.transpose();

  I.topLeftCorner<3, 3>() = inertia - mComSkewSq;
  I.topRightCorner<3, 3>() = mass * comSkew.transpose();
  I.bottomLeftCorner<3, 3>() = mass * comSkew;
  I.bottomRightCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();

  return I;
}

SpatialInertia makePointMassInertia(
    double mass, const Eigen::Vector3d& position)
{
  return makeSpatialInertia(mass, position, Eigen::Matrix3d::Zero());
}

} // namespace dart::simulation::experimental::dynamics
