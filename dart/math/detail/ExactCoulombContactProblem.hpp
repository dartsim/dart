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

#ifndef DART_MATH_DETAIL_EXACTCOULOMBCONTACTPROBLEM_HPP_
#define DART_MATH_DETAIL_EXACTCOULOMBCONTACTPROBLEM_HPP_

#include <dart/math/detail/CoulombCone.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart {
namespace math {
namespace detail {

/// Internal reduced contact-space problem for exact Coulomb friction.
///
/// The problem stores only the data that is independent of how the Delassus
/// operator `W = J M^-1 J^T` is applied. Callers provide `W` as a matrix-free
/// operator to keep this reusable for dense tests, legacy DART 6 assembly, and
/// future contact kernels without committing to a public solver API.
struct ExactCoulombContactProblem
{
  /// Free contact-space velocity in normal-first triples.
  Eigen::VectorXd freeVelocity;

  /// One non-negative friction coefficient per contact.
  Eigen::VectorXd coefficients;

  Eigen::Index getContactCount() const
  {
    return coefficients.size();
  }

  Eigen::Index getDimension() const
  {
    return 3 * getContactCount();
  }
};

/// Returns true when a reduced exact-Coulomb contact problem is well formed.
inline bool isValidExactCoulombContactProblem(
    const ExactCoulombContactProblem& problem)
{
  return problem.freeVelocity.size() == problem.getDimension()
         && problem.freeVelocity.allFinite() && problem.coefficients.allFinite()
         && (problem.coefficients.array() >= 0.0).all();
}

/// Compute the De Saxce-Feng augmented velocity for normal-first triples.
///
/// `augmentedVelocity = velocity + mu * ||velocity_t|| * e_n` per contact.
inline bool computeExactCoulombAugmentedVelocityNormalFirst(
    const Eigen::Ref<const Eigen::VectorXd>& velocity,
    const Eigen::Ref<const Eigen::VectorXd>& coefficients,
    Eigen::Ref<Eigen::VectorXd> augmentedVelocity)
{
  if (velocity.size() != augmentedVelocity.size()
      || velocity.size() != 3 * coefficients.size() || !velocity.allFinite()
      || !coefficients.allFinite() || (coefficients.array() < 0.0).any()) {
    DART_ASSERT(false && "Invalid exact-Coulomb augmented velocity input.");
    return false;
  }

  augmentedVelocity = velocity;
  for (Eigen::Index i = 0; i < coefficients.size(); ++i) {
    const Eigen::Index offset = 3 * i;
    augmentedVelocity[offset]
        += coefficients[i] * velocity.segment<2>(offset + 1).norm();
  }
  return true;
}

/// Return the De Saxce-Feng augmented velocity for normal-first triples.
inline Eigen::VectorXd computeExactCoulombAugmentedVelocityNormalFirst(
    const Eigen::Ref<const Eigen::VectorXd>& velocity,
    const Eigen::Ref<const Eigen::VectorXd>& coefficients)
{
  Eigen::VectorXd augmentedVelocity(velocity.size());
  if (!computeExactCoulombAugmentedVelocityNormalFirst(
          velocity, coefficients, augmentedVelocity)) {
    augmentedVelocity.resize(0);
  }
  return augmentedVelocity;
}

/// Compute contact velocity `v(lambda) = W lambda + v_free`.
///
/// `applyDelassus(input, output)` must write `W * input` into `output`.
template <typename DelassusOperator>
bool computeExactCoulombContactVelocityNormalFirst(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const DelassusOperator& applyDelassus,
    Eigen::Ref<Eigen::VectorXd> velocity)
{
  if (!isValidExactCoulombContactProblem(problem)
      || reaction.size() != problem.getDimension()
      || velocity.size() != problem.getDimension() || !reaction.allFinite()) {
    DART_ASSERT(false && "Invalid exact-Coulomb contact velocity input.");
    return false;
  }

  applyDelassus(reaction, velocity);
  if (!velocity.allFinite()) {
    DART_ASSERT(
        false && "Exact-Coulomb Delassus operator returned non-finite values.");
    return false;
  }

  velocity += problem.freeVelocity;
  return velocity.allFinite();
}

/// Return contact velocity `v(lambda) = W lambda + v_free`.
template <typename DelassusOperator>
Eigen::VectorXd computeExactCoulombContactVelocityNormalFirst(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const DelassusOperator& applyDelassus)
{
  Eigen::VectorXd velocity(problem.getDimension());
  if (!computeExactCoulombContactVelocityNormalFirst(
          problem, reaction, applyDelassus, velocity)) {
    velocity.resize(0);
  }
  return velocity;
}

/// Compute the exact-Coulomb residual for `lambda` using matrix-free `W`.
template <typename DelassusOperator>
CoulombConeResidual computeExactCoulombContactResidualNormalFirst(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const DelassusOperator& applyDelassus,
    const CoulombConeResidualScales& scales = CoulombConeResidualScales())
{
  Eigen::VectorXd velocity(problem.getDimension());
  if (!computeExactCoulombContactVelocityNormalFirst(
          problem, reaction, applyDelassus, velocity)) {
    return makeInvalidCoulombConeResidual();
  }

  Eigen::VectorXd augmentedVelocity(problem.getDimension());
  if (!computeExactCoulombAugmentedVelocityNormalFirst(
          velocity, problem.coefficients, augmentedVelocity)) {
    return makeInvalidCoulombConeResidual();
  }

  return computeCoulombConeResidualNormalFirst(
      reaction, augmentedVelocity, problem.coefficients, scales);
}

/// Compute the unscaled natural-map residual used by the pinned author code.
///
/// This returns
/// `||lambda - project_K(lambda - v_tilde(lambda))||_2` over the product of
/// normal-first Coulomb cones. It is distinct from DART's dimensionless
/// primal/dual/complementarity convergence residual above.
template <typename DelassusOperator>
double computeExactCoulombNaturalMapResidualNormalFirst(
    const ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const DelassusOperator& applyDelassus)
{
  Eigen::VectorXd velocity(problem.getDimension());
  if (!computeExactCoulombContactVelocityNormalFirst(
          problem, reaction, applyDelassus, velocity)) {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::VectorXd augmentedVelocity(problem.getDimension());
  if (!computeExactCoulombAugmentedVelocityNormalFirst(
          velocity, problem.coefficients, augmentedVelocity)) {
    return std::numeric_limits<double>::infinity();
  }

  double residualSquared = 0.0;
  for (Eigen::Index contact = 0; contact < problem.getContactCount();
       ++contact) {
    const Eigen::Index offset = 3 * contact;
    const Eigen::Vector3d localReaction = reaction.segment<3>(offset);
    const Eigen::Vector3d projected = projectCoulombConeNormalFirst(
        localReaction - augmentedVelocity.segment<3>(offset),
        problem.coefficients[contact]);
    residualSquared += (localReaction - projected).squaredNorm();
  }

  return std::sqrt(residualSquared);
}

/// Estimate the largest Delassus eigenvalue with deterministic power iteration.
///
/// The FBF paper uses this value to choose a safe base step size. The operator
/// is expected to be symmetric positive semidefinite; negative Rayleigh
/// estimates are clamped to zero so tiny numerical symmetry errors do not leak
/// into step-size code.
template <typename DelassusOperator>
double estimateLargestExactCoulombDelassusEigenvalue(
    const ExactCoulombContactProblem& problem,
    const DelassusOperator& applyDelassus,
    int iterations = 10)
{
  if (!isValidExactCoulombContactProblem(problem) || iterations <= 0) {
    DART_ASSERT(false && "Invalid exact-Coulomb eigenvalue estimate input.");
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Index dimension = problem.getDimension();
  if (dimension == 0) {
    return 0.0;
  }

  Eigen::VectorXd vector = Eigen::VectorXd::Ones(dimension);
  vector.normalize();

  Eigen::VectorXd product(dimension);
  for (int i = 0; i < iterations; ++i) {
    applyDelassus(vector, product);
    if (!product.allFinite()) {
      DART_ASSERT(
          false
          && "Exact-Coulomb Delassus operator returned non-finite values.");
      return std::numeric_limits<double>::infinity();
    }

    const double norm = product.norm();
    if (norm == 0.0) {
      // The deterministic all-ones seed can lie in the nullspace of a nonzero
      // PSD operator (for example, a graph Laplacian). Fall back to canonical
      // basis columns so a nonzero operator cannot be mistaken for the zero
      // operator. This path leaves the historical estimate unchanged whenever
      // the original seed has a nonzero image.
      bool foundNonzeroColumn = false;
      for (Eigen::Index column = 0; column < dimension; ++column) {
        vector.setZero();
        vector[column] = 1.0;
        applyDelassus(vector, product);
        if (!product.allFinite()) {
          DART_ASSERT(
              false
              && "Exact-Coulomb Delassus operator returned non-finite values.");
          return std::numeric_limits<double>::infinity();
        }

        const double columnNorm = product.norm();
        if (columnNorm > 0.0) {
          vector = product / columnNorm;
          foundNonzeroColumn = true;
          break;
        }
      }
      if (!foundNonzeroColumn) {
        return 0.0;
      }
      continue;
    }
    vector = product / norm;
  }

  applyDelassus(vector, product);
  if (!product.allFinite()) {
    DART_ASSERT(
        false && "Exact-Coulomb Delassus operator returned non-finite values.");
    return std::numeric_limits<double>::infinity();
  }

  const double rayleigh = vector.dot(product);
  if (!std::isfinite(rayleigh)) {
    return std::numeric_limits<double>::infinity();
  }
  return (std::max)(rayleigh, 0.0);
}

} // namespace detail
} // namespace math
} // namespace dart

#endif // DART_MATH_DETAIL_EXACTCOULOMBCONTACTPROBLEM_HPP_
