/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
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

#include <cmath>

namespace dart::simulation::detail::newton_barrier {

struct SmoothFrictionNormResult
{
  double value = 0.0;
  double firstDerivative = 0.0;
  double secondDerivative = 0.0;
  bool dynamicBranch = false;
};

template <int Columns>
struct FrictionPotentialResult
{
  double value = 0.0;
  Eigen::Matrix<double, Columns, 1> gradient
      = Eigen::Matrix<double, Columns, 1>::Zero();
  Eigen::Matrix<double, Columns, Columns> hessian
      = Eigen::Matrix<double, Columns, Columns>::Zero();
  Eigen::Vector2d tangentialDisplacement = Eigen::Vector2d::Zero();
  double tangentialDisplacementNorm = 0.0;
  double weight = 0.0;
  bool active = false;
  bool dynamicBranch = false;
};

//==============================================================================
inline SmoothFrictionNormResult smoothFrictionNorm(
    const double norm, const double staticDisplacement)
{
  SmoothFrictionNormResult result;
  if (!(norm >= 0.0) || !std::isfinite(norm) || !(staticDisplacement > 0.0)
      || !std::isfinite(staticDisplacement)) {
    return result;
  }

  if (norm > staticDisplacement) {
    result.value = norm;
    result.firstDerivative = 1.0;
    result.secondDerivative = 0.0;
    result.dynamicBranch = true;
    return result;
  }

  const double invEps = 1.0 / staticDisplacement;
  const double invEpsSquared = invEps * invEps;
  result.value = norm * norm * (1.0 - norm * invEps / 3.0) * invEps
                 + staticDisplacement / 3.0;
  result.firstDerivative = 2.0 * norm * invEps - norm * norm * invEpsSquared;
  result.secondDerivative = 2.0 * invEps - 2.0 * norm * invEpsSquared;
  return result;
}

//==============================================================================
template <int Columns>
inline FrictionPotentialResult<Columns> projectedFrictionPotential(
    const Eigen::Matrix<double, 2, Columns>& projection,
    const Eigen::Matrix<double, Columns, 1>& displacement,
    const double weight,
    const double staticDisplacement)
{
  FrictionPotentialResult<Columns> result;
  if (!projection.allFinite() || !displacement.allFinite() || !(weight > 0.0)
      || !std::isfinite(weight) || !(staticDisplacement > 0.0)
      || !std::isfinite(staticDisplacement)) {
    return result;
  }

  result.tangentialDisplacement = projection * displacement;
  result.tangentialDisplacementNorm = result.tangentialDisplacement.norm();

  const SmoothFrictionNormResult smooth = smoothFrictionNorm(
      result.tangentialDisplacementNorm, staticDisplacement);
  result.weight = weight;
  result.value = weight * smooth.value;
  result.active = true;
  result.dynamicBranch = smooth.dynamicBranch;

  Eigen::Vector2d tangentGradient = Eigen::Vector2d::Zero();
  Eigen::Matrix2d tangentHessian = Eigen::Matrix2d::Zero();
  if (result.tangentialDisplacementNorm > 0.0) {
    const Eigen::Vector2d direction
        = result.tangentialDisplacement / result.tangentialDisplacementNorm;
    tangentGradient = smooth.firstDerivative * direction;
    tangentHessian
        = (smooth.firstDerivative / result.tangentialDisplacementNorm)
              * Eigen::Matrix2d::Identity()
          + (smooth.secondDerivative
             - smooth.firstDerivative / result.tangentialDisplacementNorm)
                * (direction * direction.transpose());
  } else {
    tangentHessian = smooth.secondDerivative * Eigen::Matrix2d::Identity();
  }

  result.gradient = weight * projection.transpose() * tangentGradient;
  result.hessian
      = weight * projection.transpose() * tangentHessian * projection;
  result.hessian = 0.5
                   * (result.hessian
                      + Eigen::Matrix<double, Columns, Columns>(
                          result.hessian.transpose()));
  return result;
}

} // namespace dart::simulation::detail::newton_barrier
