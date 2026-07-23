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

#ifndef DART_MATH_DETAIL_COULOMBCONE_HPP_
#define DART_MATH_DETAIL_COULOMBCONE_HPP_

#include <dart/common/Macros.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart {
namespace math {
namespace detail {

/// Returns true if a coefficient is suitable for contact-friction cone math.
inline bool isValidCoulombConeCoefficient(double coefficient)
{
  return std::isfinite(coefficient) && coefficient >= 0.0;
}

/// Scaling used to report exact-Coulomb residual terms dimensionlessly.
struct CoulombConeResidualScales
{
  double reactionScale = 1.0;
  double velocityScale = 1.0;
};

/// Primal, dual, and complementarity residuals for the exact Coulomb law.
struct CoulombConeResidual
{
  double primalFeasibility = 0.0;
  double dualFeasibility = 0.0;
  double complementarity = 0.0;
  double value = 0.0;
  Eigen::Index worstPrimalContact = -1;
  Eigen::Index worstDualContact = -1;
  Eigen::Index worstComplementarityContact = -1;
};

/// Returns true if a residual scale is finite and strictly positive.
inline bool isValidCoulombConeResidualScale(double scale)
{
  return std::isfinite(scale) && scale > 0.0;
}

inline CoulombConeResidual makeInvalidCoulombConeResidual()
{
  const double infinity = std::numeric_limits<double>::infinity();
  CoulombConeResidual residual;
  residual.primalFeasibility = infinity;
  residual.dualFeasibility = infinity;
  residual.complementarity = infinity;
  residual.value = infinity;
  return residual;
}

/// Returns true when every scalar residual component is finite.
inline bool isFiniteCoulombConeResidual(const CoulombConeResidual& residual)
{
  return std::isfinite(residual.primalFeasibility)
         && std::isfinite(residual.dualFeasibility)
         && std::isfinite(residual.complementarity)
         && std::isfinite(residual.value);
}

/// Project a normal-first contact vector onto the Coulomb cone.
///
/// The vector layout is `[normal, tangent_1, tangent_2]`, matching DART's
/// contact-constraint row order. The cone is
/// `{ x | x_n >= 0, ||x_t|| <= coefficient * x_n }`.
inline Eigen::Vector3d projectCoulombConeNormalFirst(
    const Eigen::Vector3d& value, double coefficient)
{
  if (!value.allFinite() || !isValidCoulombConeCoefficient(coefficient)) {
    DART_ASSERT(false && "Invalid Coulomb cone projection input.");
    return Eigen::Vector3d::Zero();
  }

  const double normal = value[0];
  const Eigen::Vector2d tangent = value.tail<2>();
  const double tangentNorm = tangent.norm();

  if (coefficient == 0.0) {
    return Eigen::Vector3d{
        (std::max)(normal, 0.0),
        0.0,
        0.0,
    };
  }

  if (tangentNorm <= coefficient * normal) {
    return value;
  }

  if (normal <= -coefficient * tangentNorm) {
    return Eigen::Vector3d::Zero();
  }

  const double coefficientSquared = coefficient * coefficient;
  const double projectedNormal
      = (normal + coefficient * tangentNorm) / (1.0 + coefficientSquared);
  const double projectedTangentNorm = coefficient * projectedNormal;

  Eigen::Vector3d projected = Eigen::Vector3d::Zero();
  projected[0] = projectedNormal;
  if (tangentNorm > 0.0) {
    projected.tail<2>() = (projectedTangentNorm / tangentNorm) * tangent;
  }
  return projected;
}

/// Project a normal-first contact vector onto the dual Coulomb cone.
///
/// For a positive friction coefficient, the dual cone is the Coulomb cone with
/// reciprocal coefficient. For zero friction, the dual is the normal half-space
/// `{ y | y_n >= 0 }` with unconstrained tangential components.
inline Eigen::Vector3d projectCoulombDualConeNormalFirst(
    const Eigen::Vector3d& value, double coefficient)
{
  if (!value.allFinite() || !isValidCoulombConeCoefficient(coefficient)) {
    DART_ASSERT(false && "Invalid Coulomb dual-cone projection input.");
    return Eigen::Vector3d::Zero();
  }

  if (coefficient == 0.0) {
    Eigen::Vector3d projected = value;
    projected[0] = (std::max)(projected[0], 0.0);
    return projected;
  }

  const double dualCoefficient = 1.0 / coefficient;
  if (!std::isfinite(dualCoefficient)) {
    Eigen::Vector3d projected = value;
    projected[0] = (std::max)(projected[0], 0.0);
    return projected;
  }

  return projectCoulombConeNormalFirst(value, dualCoefficient);
}

/// Compute the dimensionless exact-Coulomb residual for one contact.
///
/// The augmented velocity is the De Saxce-Feng velocity
/// `v + mu * ||v_t|| * e_n`. Residual terms are reported as normalized
/// distances to the primal cone, dual cone, and complementarity gap.
inline CoulombConeResidual computeCoulombConeResidualNormalFirst(
    const Eigen::Vector3d& reaction,
    const Eigen::Vector3d& augmentedVelocity,
    double coefficient,
    const CoulombConeResidualScales& scales = CoulombConeResidualScales())
{
  if (!reaction.allFinite() || !augmentedVelocity.allFinite()
      || !isValidCoulombConeCoefficient(coefficient)
      || !isValidCoulombConeResidualScale(scales.reactionScale)
      || !isValidCoulombConeResidualScale(scales.velocityScale)) {
    DART_ASSERT(false && "Invalid Coulomb cone residual input.");
    return makeInvalidCoulombConeResidual();
  }

  const Eigen::Vector3d primalProjection
      = projectCoulombConeNormalFirst(reaction, coefficient);
  const Eigen::Vector3d dualProjection
      = projectCoulombDualConeNormalFirst(augmentedVelocity, coefficient);

  CoulombConeResidual residual;
  residual.primalFeasibility
      = (reaction - primalProjection).norm() / scales.reactionScale;
  residual.dualFeasibility
      = (augmentedVelocity - dualProjection).norm() / scales.velocityScale;
  residual.complementarity = std::abs(augmentedVelocity.dot(reaction))
                             / (scales.reactionScale * scales.velocityScale);
  residual.value = (std::max)(
      {residual.primalFeasibility,
       residual.dualFeasibility,
       residual.complementarity});
  residual.worstPrimalContact = 0;
  residual.worstDualContact = 0;
  residual.worstComplementarityContact = 0;
  return residual;
}

/// Compute the dimensionless exact-Coulomb residual over a product of cones.
///
/// `reactions` and `augmentedVelocities` store one normal-first contact triple
/// per coefficient. The primal and dual terms are normalized Euclidean product
/// cone distances; the complementarity term is the normalized global gap.
inline CoulombConeResidual computeCoulombConeResidualNormalFirst(
    const Eigen::Ref<const Eigen::VectorXd>& reactions,
    const Eigen::Ref<const Eigen::VectorXd>& augmentedVelocities,
    const Eigen::Ref<const Eigen::VectorXd>& coefficients,
    const CoulombConeResidualScales& scales = CoulombConeResidualScales())
{
  if (reactions.size() != augmentedVelocities.size()
      || reactions.size() != 3 * coefficients.size() || !reactions.allFinite()
      || !augmentedVelocities.allFinite() || !coefficients.allFinite()
      || (coefficients.array() < 0.0).any()
      || !isValidCoulombConeResidualScale(scales.reactionScale)
      || !isValidCoulombConeResidualScale(scales.velocityScale)) {
    DART_ASSERT(false && "Invalid Coulomb cone product residual input.");
    return makeInvalidCoulombConeResidual();
  }

  double primalDistanceSquared = 0.0;
  double dualDistanceSquared = 0.0;
  double complementarityGap = 0.0;
  double worstPrimalDistanceSquared = -1.0;
  double worstDualDistanceSquared = -1.0;
  double worstComplementarityContribution = -1.0;
  CoulombConeResidual residual;

  for (Eigen::Index i = 0; i < coefficients.size(); ++i) {
    const Eigen::Index offset = 3 * i;
    const Eigen::Vector3d reaction = reactions.segment<3>(offset);
    const Eigen::Vector3d augmentedVelocity
        = augmentedVelocities.segment<3>(offset);
    const double coefficient = coefficients[i];

    const Eigen::Vector3d primalProjection
        = projectCoulombConeNormalFirst(reaction, coefficient);
    const Eigen::Vector3d dualProjection
        = projectCoulombDualConeNormalFirst(augmentedVelocity, coefficient);

    const double primalContribution
        = (reaction - primalProjection).squaredNorm();
    const double dualContribution
        = (augmentedVelocity - dualProjection).squaredNorm();
    const double complementarityContribution = augmentedVelocity.dot(reaction);

    primalDistanceSquared += primalContribution;
    dualDistanceSquared += dualContribution;
    complementarityGap += complementarityContribution;

    if (primalContribution > worstPrimalDistanceSquared) {
      worstPrimalDistanceSquared = primalContribution;
      residual.worstPrimalContact = i;
    }
    if (dualContribution > worstDualDistanceSquared) {
      worstDualDistanceSquared = dualContribution;
      residual.worstDualContact = i;
    }
    const double absoluteComplementarityContribution
        = std::abs(complementarityContribution);
    if (absoluteComplementarityContribution
        > worstComplementarityContribution) {
      worstComplementarityContribution = absoluteComplementarityContribution;
      residual.worstComplementarityContact = i;
    }
  }

  residual.primalFeasibility
      = std::sqrt(primalDistanceSquared) / scales.reactionScale;
  residual.dualFeasibility
      = std::sqrt(dualDistanceSquared) / scales.velocityScale;
  residual.complementarity = std::abs(complementarityGap)
                             / (scales.reactionScale * scales.velocityScale);
  residual.value = (std::max)(
      {residual.primalFeasibility,
       residual.dualFeasibility,
       residual.complementarity});
  return residual;
}

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_COULOMBCONE_HPP_
