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

#ifndef DART_CONSTRAINT_DETAIL_EXACTCOULOMBCONSTRAINTADAPTER_HPP_
#define DART_CONSTRAINT_DETAIL_EXACTCOULOMBCONSTRAINTADAPTER_HPP_

#include <dart/constraint/ConstraintBase.hpp>

#include <dart/math/detail/ExactCoulombContactProblem.hpp>
#include <dart/math/detail/ExactCoulombFbfSolver.hpp>

#include <dart/common/Macros.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart {
namespace constraint {
namespace detail {

/// Status for adapting DART 6 constraint rows to exact-Coulomb contact space.
enum class ExactCoulombConstraintBuildStatus
{
  Success,
  EmptyInput,
  NullConstraint,
  InvalidOptions,
  UnsupportedDimension,
  UnsupportedBounds,
  UnsupportedFrictionCoupling,
  UnsupportedAnisotropicFriction,
  NonFiniteData,
};

/// Options for the DART 6 constraint-row exact-Coulomb adapter.
struct ExactCoulombConstraintBuildOptions
{
  /// Inverse time step forwarded to ConstraintBase::getInformation().
  double invTimeStep = 1.0;

  /// Whether getVelocityChange() should include DART's CFM/slip terms.
  ///
  /// The exact-Coulomb paper's reduced operator is `J M^-1 J^T`, so the
  /// default is false. This option exists only to compare against the legacy
  /// boxed-LCP assembly while the integration path is being staged.
  bool includeConstraintRegularization = false;

  /// Constraint solve phase forwarded to ConstraintBase::getInformation().
  ConstraintPhase phase = ConstraintPhase::Velocity;

  /// Split-impulse flag forwarded to ConstraintBase::getInformation().
  bool useSplitImpulse = false;

  /// Absolute tolerance for isotropic friction-row validation.
  double frictionTolerance = 1e-12;

  /// Seed zero row guesses from local/global Delassus estimates.
  ///
  /// DART contact constraints currently report a zero initial `x` for contact
  /// rows. This deterministic dense cold start first solves each local 3D
  /// contact block approximately, then compares that seed against a projected
  /// full-Delassus linear seed and keeps the lower residual. It preserves
  /// explicit nonzero row guesses and later solver-level warm starts. The
  /// historical option name is kept because the seed still falls back to the
  /// normal Delassus diagonal when the richer block solve is not usable.
  bool seedNormalImpulseFromDiagonal = true;

  /// Assemble the dense Delassus snapshot with DART impulse tests.
  ///
  /// Callers that can assemble the snapshot from a cheaper supported route,
  /// such as the exact-Coulomb contact-row operator, disable this to skip
  /// the impulse tests and then fill `delassus` and run the dense seed
  /// themselves. When disabled, `seedNormalImpulseFromDiagonal` is deferred
  /// with the assembly instead of running on an empty snapshot.
  bool assembleDenseDelassus = true;
};

/// Adapted exact-Coulomb contact problem plus a dense Delassus snapshot.
struct ExactCoulombConstraintProblem
{
  ExactCoulombConstraintBuildStatus status
      = ExactCoulombConstraintBuildStatus::EmptyInput;
  math::detail::ExactCoulombContactProblem contactProblem;
  Eigen::MatrixXd delassus;
  Eigen::VectorXd initialGuess;
  std::vector<ConstraintBase*> constraints;
};

inline bool isValidExactCoulombConstraintBuildOptions(
    const ExactCoulombConstraintBuildOptions& options)
{
  return std::isfinite(options.invTimeStep) && options.invTimeStep > 0.0
         && std::isfinite(options.frictionTolerance)
         && options.frictionTolerance >= 0.0;
}

inline bool equalsWithinExactCoulombConstraintTolerance(
    double lhs, double rhs, double tolerance)
{
  return std::abs(lhs - rhs) <= tolerance;
}

inline bool validateExactCoulombContactRows(
    const double* lo,
    const double* hi,
    const int* findex,
    double frictionTolerance,
    double& coefficient,
    ExactCoulombConstraintBuildStatus& status)
{
  if (lo == nullptr || hi == nullptr || findex == nullptr
      || !std::isfinite(lo[0]) || !std::isfinite(lo[1]) || !std::isfinite(lo[2])
      || std::isnan(hi[0]) || !std::isfinite(hi[1]) || !std::isfinite(hi[2])) {
    status = ExactCoulombConstraintBuildStatus::NonFiniteData;
    return false;
  }

  if (!equalsWithinExactCoulombConstraintTolerance(
          lo[0], 0.0, frictionTolerance)
      || hi[0] <= 0.0) {
    status = ExactCoulombConstraintBuildStatus::UnsupportedBounds;
    return false;
  }

  if (findex[0] != -1 || findex[1] != 0 || findex[2] != 0) {
    status = ExactCoulombConstraintBuildStatus::UnsupportedFrictionCoupling;
    return false;
  }

  const double primary = hi[1];
  const double secondary = hi[2];
  if (primary < 0.0 || secondary < 0.0
      || !equalsWithinExactCoulombConstraintTolerance(
          lo[1], -primary, frictionTolerance)
      || !equalsWithinExactCoulombConstraintTolerance(
          lo[2], -secondary, frictionTolerance)) {
    status = ExactCoulombConstraintBuildStatus::UnsupportedBounds;
    return false;
  }

  if (!equalsWithinExactCoulombConstraintTolerance(
          primary, secondary, frictionTolerance)) {
    status = ExactCoulombConstraintBuildStatus::UnsupportedAnisotropicFriction;
    return false;
  }

  coefficient = primary;
  return true;
}

inline Eigen::Vector3d computeExactCoulombLocalDiagonalBlockSeed(
    const Eigen::Vector3d& freeVelocity,
    double coefficient,
    const Eigen::Matrix3d& delassusBlock)
{
  Eigen::Vector3d guess = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d localDelassus
      = 0.5 * (delassusBlock + delassusBlock.transpose());
  if (localDelassus.allFinite() && freeVelocity.allFinite()) {
    Eigen::LDLT<Eigen::Matrix3d> factorization(localDelassus);
    if (factorization.info() == Eigen::Success) {
      const Eigen::Vector3d localGuess = factorization.solve(-freeVelocity);
      if (localGuess.allFinite()) {
        return math::detail::projectCoulombConeNormalFirst(
            localGuess, coefficient);
      }
    }
  }

  const double normalDiagonal = delassusBlock(0, 0);
  const double freeNormal = freeVelocity[0];
  if (std::isfinite(normalDiagonal) && normalDiagonal > 0.0
      && std::isfinite(freeNormal)) {
    guess[0] = std::max(0.0, -freeNormal / normalDiagonal);
  }

  for (Eigen::Index tangent = 1; tangent < 3; ++tangent) {
    const double diagonal = delassusBlock(tangent, tangent);
    const double freeTangent = freeVelocity[tangent];
    if (std::isfinite(diagonal) && diagonal > 0.0
        && std::isfinite(freeTangent)) {
      guess[tangent] = -freeTangent / diagonal;
    }
  }

  return math::detail::projectCoulombConeNormalFirst(guess, coefficient);
}

inline Eigen::Vector3d computeExactCoulombLocalDiagonalBlockSeed(
    const ExactCoulombConstraintProblem& problem, Eigen::Index contact)
{
  const Eigen::Index offset = 3 * contact;
  return computeExactCoulombLocalDiagonalBlockSeed(
      problem.contactProblem.freeVelocity.segment<3>(offset),
      problem.contactProblem.coefficients[contact],
      problem.delassus.block<3, 3>(offset, offset));
}

inline bool computeExactCoulombProjectedGlobalDelassusSeed(
    const ExactCoulombConstraintProblem& problem,
    Eigen::Ref<Eigen::VectorXd> seed)
{
  if (seed.size() != problem.contactProblem.getDimension()) {
    return false;
  }

  const Eigen::MatrixXd symmetricDelassus
      = 0.5 * (problem.delassus + problem.delassus.transpose());
  if (!symmetricDelassus.allFinite()) {
    return false;
  }

  Eigen::LDLT<Eigen::MatrixXd> factorization(symmetricDelassus);
  if (factorization.info() != Eigen::Success) {
    return false;
  }

  const Eigen::VectorXd unconstrained
      = factorization.solve(-problem.contactProblem.freeVelocity);
  if (!unconstrained.allFinite()) {
    return false;
  }

  for (Eigen::Index contact = 0;
       contact < problem.contactProblem.getContactCount();
       ++contact) {
    const Eigen::Index offset = 3 * contact;
    seed.segment<3>(offset) = math::detail::projectCoulombConeNormalFirst(
        unconstrained.segment<3>(offset),
        problem.contactProblem.coefficients[contact]);
  }
  return seed.allFinite();
}

template <typename DelassusOperator>
inline double computeExactCoulombConstraintSeedResidual(
    const ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    const DelassusOperator& applyDelassus)
{
  return math::detail::computeExactCoulombContactResidualNormalFirst(
             problem.contactProblem, seed, applyDelassus)
      .value;
}

inline double computeExactCoulombConstraintSeedResidual(
    const ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& seed)
{
  const auto applyDenseDelassus
      = [&problem](
            const Eigen::Ref<const Eigen::VectorXd>& input,
            Eigen::Ref<Eigen::VectorXd> output) {
          output.noalias() = problem.delassus * input;
        };

  return computeExactCoulombConstraintSeedResidual(
      problem, seed, applyDenseDelassus);
}

inline void seedExactCoulombImpulseFromDelassus(
    ExactCoulombConstraintProblem& problem)
{
  const Eigen::Index contactCount = problem.contactProblem.getContactCount();
  std::vector<bool> canUseGlobalSeed(static_cast<std::size_t>(contactCount));

  for (Eigen::Index contact = 0; contact < contactCount; ++contact) {
    const Eigen::Index offset = 3 * contact;
    Eigen::Vector3d guess = problem.initialGuess.segment<3>(offset);
    if (!guess.allFinite()) {
      continue;
    }

    if (guess.norm() > 0.0) {
      problem.initialGuess.segment<3>(offset)
          = math::detail::projectCoulombConeNormalFirst(
              guess, problem.contactProblem.coefficients[contact]);
      canUseGlobalSeed[static_cast<std::size_t>(contact)] = false;
      continue;
    }

    canUseGlobalSeed[static_cast<std::size_t>(contact)] = true;
    problem.initialGuess.segment<3>(offset)
        = computeExactCoulombLocalDiagonalBlockSeed(problem, contact);
  }

  Eigen::VectorXd globalSeed(problem.contactProblem.getDimension());
  if (!computeExactCoulombProjectedGlobalDelassusSeed(problem, globalSeed)) {
    return;
  }

  Eigen::VectorXd candidate = problem.initialGuess;
  for (Eigen::Index contact = 0; contact < contactCount; ++contact) {
    if (!canUseGlobalSeed[static_cast<std::size_t>(contact)]) {
      continue;
    }

    const Eigen::Index offset = 3 * contact;
    candidate.segment<3>(offset) = globalSeed.segment<3>(offset);
  }

  const double currentResidual = computeExactCoulombConstraintSeedResidual(
      problem, problem.initialGuess);
  const double candidateResidual
      = computeExactCoulombConstraintSeedResidual(problem, candidate);
  if (std::isfinite(candidateResidual)
      && (!std::isfinite(currentResidual)
          || candidateResidual < currentResidual)) {
    problem.initialGuess = candidate;
  }
}

/// Seed zero contact-row guesses using operator-extracted local Delassus
/// blocks.
///
/// This keeps explicit nonzero row guesses, but projects them onto the product
/// Coulomb cone. Unlike the dense helper above, this intentionally avoids the
/// dense full-Delassus global seed so matrix-free benchmark rows can separate
/// product/diagonal-block staging from dense prototype recovery paths.
template <typename DelassusOperator>
inline bool seedExactCoulombImpulseFromDelassusOperator(
    ExactCoulombConstraintProblem& problem,
    const std::vector<Eigen::Matrix3d>& diagonalBlocks,
    const DelassusOperator& applyDelassus)
{
  const Eigen::Index contactCount = problem.contactProblem.getContactCount();
  if (problem.initialGuess.size() != problem.contactProblem.getDimension()
      || !problem.contactProblem.freeVelocity.allFinite()
      || !problem.contactProblem.coefficients.allFinite()) {
    DART_ASSERT(false && "Invalid exact-Coulomb operator seed input.");
    return false;
  }

  if (diagonalBlocks.size() != static_cast<std::size_t>(contactCount)) {
    DART_ASSERT(false && "Invalid exact-Coulomb diagonal-block output.");
    return false;
  }

  for (Eigen::Index contact = 0; contact < contactCount; ++contact) {
    const Eigen::Index offset = 3 * contact;
    Eigen::Vector3d guess = problem.initialGuess.segment<3>(offset);
    if (!guess.allFinite()) {
      return false;
    }

    if (guess.norm() > 0.0) {
      problem.initialGuess.segment<3>(offset)
          = math::detail::projectCoulombConeNormalFirst(
              guess, problem.contactProblem.coefficients[contact]);
      continue;
    }

    problem.initialGuess.segment<3>(offset)
        = computeExactCoulombLocalDiagonalBlockSeed(
            problem.contactProblem.freeVelocity.segment<3>(offset),
            problem.contactProblem.coefficients[contact],
            diagonalBlocks[static_cast<std::size_t>(contact)]);
  }

  return problem.initialGuess.allFinite()
         && std::isfinite(computeExactCoulombConstraintSeedResidual(
             problem, problem.initialGuess, applyDelassus));
}

/// Extract local Delassus blocks through an operator and seed from them.
template <typename DelassusOperator>
inline bool seedExactCoulombImpulseFromDelassusOperator(
    ExactCoulombConstraintProblem& problem,
    const DelassusOperator& applyDelassus)
{
  std::vector<Eigen::Matrix3d> diagonalBlocks;
  if (!math::detail::computeExactCoulombDelassusDiagonalBlocksNormalFirst(
          problem.contactProblem, applyDelassus, diagonalBlocks)) {
    return false;
  }

  return seedExactCoulombImpulseFromDelassusOperator(
      problem, diagonalBlocks, applyDelassus);
}

/// Apply the adapted contact-space Delassus operator without reading the dense
/// snapshot.
///
/// This uses the same impulse-test contract as the dense adapter and the
/// legacy boxed-LCP assembly. It is useful for validating and staging the
/// matrix-free exact-FBF route while the dense snapshot is still retained for
/// cold-start and prototype polish diagnostics.
inline bool applyExactCoulombConstraintDelassus(
    const ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& input,
    Eigen::Ref<Eigen::VectorXd> output,
    bool includeConstraintRegularization = false)
{
  const Eigen::Index dimension = problem.contactProblem.getDimension();
  if (problem.status != ExactCoulombConstraintBuildStatus::Success
      || input.size() != dimension || output.size() != dimension
      || input.size()
             != 3 * static_cast<Eigen::Index>(problem.constraints.size())
      || !input.allFinite()) {
    DART_ASSERT(false && "Invalid exact-Coulomb Delassus application input.");
    return false;
  }

  output.setZero();

  Eigen::Vector3d velocityChange;
  for (std::size_t i = 0; i < problem.constraints.size(); ++i) {
    ConstraintBase* source = problem.constraints[i];
    if (source == nullptr) {
      DART_ASSERT(false && "Null exact-Coulomb adapted constraint.");
      output.setConstant(std::numeric_limits<double>::quiet_NaN());
      return false;
    }

    const Eigen::Index sourceOffset = static_cast<Eigen::Index>(3u * i);

    source->excite();
    for (std::size_t localRow = 0; localRow < 3u; ++localRow) {
      const Eigen::Index row
          = sourceOffset + static_cast<Eigen::Index>(localRow);
      source->applyUnitImpulse(localRow);

      source->getVelocityChange(
          velocityChange.data(), includeConstraintRegularization);
      if (!velocityChange.allFinite()) {
        source->unexcite();
        output.setConstant(std::numeric_limits<double>::quiet_NaN());
        return false;
      }
      output[row] += velocityChange.dot(input.segment<3>(sourceOffset));

      for (std::size_t k = i + 1; k < problem.constraints.size(); ++k) {
        ConstraintBase* target = problem.constraints[k];
        if (target == nullptr) {
          source->unexcite();
          DART_ASSERT(false && "Null exact-Coulomb adapted constraint.");
          output.setConstant(std::numeric_limits<double>::quiet_NaN());
          return false;
        }

        const Eigen::Index targetOffset = static_cast<Eigen::Index>(3u * k);
        target->getVelocityChange(velocityChange.data(), false);
        if (!velocityChange.allFinite()) {
          source->unexcite();
          output.setConstant(std::numeric_limits<double>::quiet_NaN());
          return false;
        }

        output[row] += velocityChange.dot(input.segment<3>(targetOffset));
        output.segment<3>(targetOffset) += velocityChange * input[row];
      }
    }
    source->unexcite();
  }

  return output.allFinite();
}

/// Assemble the dense Delassus snapshot with DART impulse tests.
///
/// This is the same impulse-test mechanism used by BoxedLcpConstraintSolver.
/// It requires `result.constraints` and the problem dimensions to be already
/// populated, writes into `result.delassus`, and mirrors the upper triangle.
inline bool assembleExactCoulombConstraintDelassusByImpulseTests(
    ExactCoulombConstraintProblem& result, bool includeConstraintRegularization)
{
  const std::size_t contactCount = result.constraints.size();
  const Eigen::Index dimension = static_cast<Eigen::Index>(3u * contactCount);
  result.delassus.setZero(dimension, dimension);

  Eigen::Vector3d velocityChange;
  for (std::size_t i = 0; i < contactCount; ++i) {
    ConstraintBase* source = result.constraints[i];
    const Eigen::Index sourceOffset = static_cast<Eigen::Index>(3u * i);

    source->excite();
    for (std::size_t localRow = 0; localRow < 3u; ++localRow) {
      const Eigen::Index row
          = sourceOffset + static_cast<Eigen::Index>(localRow);
      source->applyUnitImpulse(localRow);

      source->getVelocityChange(
          velocityChange.data(), includeConstraintRegularization);
      if (!velocityChange.allFinite()) {
        source->unexcite();
        return false;
      }
      result.delassus.block<1, 3>(row, sourceOffset)
          = velocityChange.transpose();

      for (std::size_t k = i + 1; k < contactCount; ++k) {
        const Eigen::Index targetOffset = static_cast<Eigen::Index>(3u * k);
        result.constraints[k]->getVelocityChange(velocityChange.data(), false);
        if (!velocityChange.allFinite()) {
          source->unexcite();
          return false;
        }
        result.delassus.block<1, 3>(row, targetOffset)
            = velocityChange.transpose();
      }
    }
    source->unexcite();
  }

  for (Eigen::Index row = 1; row < dimension; ++row) {
    for (Eigen::Index col = 0; col < row; ++col) {
      result.delassus(row, col) = result.delassus(col, row);
    }
  }

  return true;
}

/// Build an exact-Coulomb problem from existing DART 6 contact-style rows.
///
/// The adapter accepts only three-row isotropic contact constraints whose two
/// tangential rows are ODE-style friction rows coupled to the local normal row.
/// It converts DART's boxed-LCP right-hand side `A lambda = b + w` into the
/// exact-Coulomb free velocity `v_free = -b`, and assembles `W` with the same
/// impulse-test mechanism used by BoxedLcpConstraintSolver.
inline ExactCoulombConstraintProblem buildExactCoulombConstraintProblem(
    const std::vector<ConstraintBase*>& constraints,
    const ExactCoulombConstraintBuildOptions& options
    = ExactCoulombConstraintBuildOptions())
{
  ExactCoulombConstraintProblem result;

  if (!isValidExactCoulombConstraintBuildOptions(options)) {
    result.status = ExactCoulombConstraintBuildStatus::InvalidOptions;
    return result;
  }

  if (constraints.empty()) {
    result.status = ExactCoulombConstraintBuildStatus::EmptyInput;
    return result;
  }

  const std::size_t contactCount = constraints.size();
  const Eigen::Index dimension = static_cast<Eigen::Index>(3u * contactCount);
  result.contactProblem.freeVelocity.resize(dimension);
  result.contactProblem.coefficients.resize(
      static_cast<Eigen::Index>(contactCount));
  // The impulse-test assembler or an external supported route sizes and fills
  // the snapshot; keep it empty until then so accidental use fails loudly.
  result.delassus.resize(0, 0);
  result.initialGuess.resize(dimension);
  result.constraints = constraints;

  std::vector<double> x(static_cast<std::size_t>(dimension), 0.0);
  std::vector<double> lo(static_cast<std::size_t>(dimension), 0.0);
  std::vector<double> hi(static_cast<std::size_t>(dimension), 0.0);
  std::vector<double> b(static_cast<std::size_t>(dimension), 0.0);
  std::vector<double> w(static_cast<std::size_t>(dimension), 0.0);
  std::vector<int> findex(static_cast<std::size_t>(dimension), -1);

  ConstraintInfo info;
  info.invTimeStep = options.invTimeStep;
  info.phase = options.phase;
  info.useSplitImpulse = options.useSplitImpulse;

  for (std::size_t i = 0; i < contactCount; ++i) {
    ConstraintBase* constraint = constraints[i];
    if (constraint == nullptr) {
      result.status = ExactCoulombConstraintBuildStatus::NullConstraint;
      return result;
    }

    if (constraint->getDimension() != 3u) {
      result.status = ExactCoulombConstraintBuildStatus::UnsupportedDimension;
      return result;
    }

    const std::size_t offset = 3u * i;
    info.x = x.data() + offset;
    info.lo = lo.data() + offset;
    info.hi = hi.data() + offset;
    info.b = b.data() + offset;
    info.w = w.data() + offset;
    info.findex = findex.data() + offset;

    constraint->getInformation(&info);

    const Eigen::Map<const Eigen::Vector3d> rhs(b.data() + offset);
    const Eigen::Map<const Eigen::Vector3d> initial(x.data() + offset);
    if (!rhs.allFinite() || !initial.allFinite()) {
      result.status = ExactCoulombConstraintBuildStatus::NonFiniteData;
      return result;
    }

    double coefficient = 0.0;
    if (!validateExactCoulombContactRows(
            lo.data() + offset,
            hi.data() + offset,
            findex.data() + offset,
            options.frictionTolerance,
            coefficient,
            result.status)) {
      return result;
    }

    result.contactProblem.freeVelocity.segment<3>(
        static_cast<Eigen::Index>(offset))
        = -rhs;
    result.contactProblem.coefficients[static_cast<Eigen::Index>(i)]
        = coefficient;
    result.initialGuess.segment<3>(static_cast<Eigen::Index>(offset)) = initial;
  }

  if (options.assembleDenseDelassus) {
    if (!assembleExactCoulombConstraintDelassusByImpulseTests(
            result, options.includeConstraintRegularization)) {
      result.status = ExactCoulombConstraintBuildStatus::NonFiniteData;
      return result;
    }

    if (options.seedNormalImpulseFromDiagonal) {
      seedExactCoulombImpulseFromDelassus(result);
    }
  }

  if (!math::detail::isValidExactCoulombContactProblem(result.contactProblem)
      || !result.delassus.allFinite() || !result.initialGuess.allFinite()) {
    result.status = ExactCoulombConstraintBuildStatus::NonFiniteData;
    return result;
  }

  result.status = ExactCoulombConstraintBuildStatus::Success;
  return result;
}

/// Apply exact-Coulomb reaction triples back through the original constraints.
inline bool applyExactCoulombConstraintImpulses(
    const ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction)
{
  if (problem.status != ExactCoulombConstraintBuildStatus::Success
      || reaction.size() != problem.contactProblem.getDimension()
      || reaction.size()
             != 3 * static_cast<Eigen::Index>(problem.constraints.size())
      || !reaction.allFinite()) {
    DART_ASSERT(false && "Invalid exact-Coulomb constraint impulse input.");
    return false;
  }

  for (std::size_t i = 0; i < problem.constraints.size(); ++i) {
    ConstraintBase* constraint = problem.constraints[i];
    if (constraint == nullptr) {
      DART_ASSERT(false && "Null exact-Coulomb adapted constraint.");
      return false;
    }

    Eigen::Vector3d localReaction
        = reaction.segment<3>(static_cast<Eigen::Index>(3u * i));
    constraint->applyImpulse(localReaction.data());
    constraint->excite();
  }

  return true;
}

} // namespace detail
} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_DETAIL_EXACTCOULOMBCONSTRAINTADAPTER_HPP_
