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

#include <dart/math/lcp/lcp_types.hpp>

#include <dart/common/macros.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <cmath>

namespace dart::math::detail {

inline double projectToBounds(double value, double lo, double hi)
{
  const bool hasLo = std::isfinite(lo);
  const bool hasUpper = std::isfinite(hi);

  if (hasLo && hasUpper) {
    if (lo <= hi) {
      return std::clamp(value, lo, hi);
    }

    return std::min(std::max(value, lo), hi);
  }

  if (hasLo) {
    return std::max(value, lo);
  }

  if (hasUpper) {
    return std::min(value, hi);
  }

  return value;
}

inline bool validateProblem(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    std::string* message = nullptr)
{
  const bool dimensionMismatch
      = (A.rows() != A.cols()) || (A.rows() != b.size())
        || (lo.size() != b.size()) || (hi.size() != b.size())
        || (findex.size() != b.size());
  if (dimensionMismatch) {
    if (message) {
      *message = "Matrix/vector dimensions inconsistent";
    }
    return false;
  }

  if (!A.allFinite()) {
    if (message) {
      *message = "Matrix contains non-finite values";
    }
    return false;
  }

  if (!b.allFinite()) {
    if (message) {
      *message = "Vector b contains non-finite values";
    }
    return false;
  }

  const Eigen::Index n = b.size();
  for (Eigen::Index i = 0; i < n; ++i) {
    const int ref = findex[i];
    if (ref >= n) {
      if (message) {
        *message = "Friction index entry out of range";
      }
      return false;
    }

    if (ref == i) {
      if (message) {
        *message = "Friction index entry cannot reference itself";
      }
      return false;
    }

    if (std::isnan(lo[i]) || std::isnan(hi[i])) {
      if (message) {
        *message = "Bounds contain NaN";
      }
      return false;
    }

    if (lo[i] == std::numeric_limits<double>::infinity()
        || hi[i] == -std::numeric_limits<double>::infinity()) {
      if (message) {
        *message = "Bounds have invalid infinity direction";
      }
      return false;
    }

    if (ref >= 0 && !std::isfinite(hi[i])) {
      if (message) {
        *message = "Friction coefficient (hi) must be finite";
      }
      return false;
    }

    if (ref >= 0 && hi[i] < 0.0) {
      if (message) {
        *message = "Friction coefficient (hi) must be non-negative";
      }
      return false;
    }

    if (std::isfinite(lo[i]) && std::isfinite(hi[i]) && lo[i] > hi[i]) {
      if (message) {
        *message = "Lower bound exceeds upper bound";
      }
      return false;
    }
  }

  return true;
}

inline bool validateProblem(
    const LcpProblem& problem, std::string* message = nullptr)
{
  const std::string problemMessage = problem.getValidationMessage();
  if (problemMessage.empty()) {
    return true;
  }

  if (message) {
    *message = problemMessage;
  }
  return false;
}

inline bool computeEffectiveBounds(
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    const Eigen::VectorXd& x,
    Eigen::VectorXd& loEff,
    Eigen::VectorXd& hiEff,
    std::string* message = nullptr)
{
  const Eigen::Index n = x.size();

  if (lo.size() != n || hi.size() != n || findex.size() != n) {
    if (message) {
      *message = "Invalid LCP problem dimensions";
    }
    return false;
  }

  loEff = lo;
  hiEff = hi;

  for (Eigen::Index i = 0; i < n; ++i) {
    const int ref = findex[i];
    if (ref < 0) {
      continue;
    }

    if (ref >= n) {
      if (message) {
        *message = "Invalid friction index entry";
      }
      return false;
    }

    if (ref == i) {
      if (message) {
        *message = "Invalid friction index entry: self reference";
      }
      return false;
    }

    const double scale = x[ref];
    const double mu = hi[i];
    if (!std::isfinite(scale)) {
      if (message) {
        *message = "Invalid friction index reference value";
      }
      return false;
    }

    if (!std::isfinite(mu)) {
      if (message) {
        *message = "Invalid friction coefficient (hi) for friction index entry";
      }
      return false;
    }

    if (mu < 0.0) {
      if (message) {
        *message = "Invalid negative friction coefficient (hi)";
      }
      return false;
    }

    const double bound = mu * std::abs(scale);
    loEff[i] = -bound;
    hiEff[i] = bound;
  }

  return true;
}

inline double naturalResidualInfinityNorm(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi)
{
  const Eigen::Index n = x.size();
  DART_ASSERT(w.size() == n);
  DART_ASSERT(lo.size() == n);
  DART_ASSERT(hi.size() == n);

  double maxResidual = 0.0;

  for (Eigen::Index i = 0; i < n; ++i) {
    const double projected = projectToBounds(x[i] - w[i], lo[i], hi[i]);

    const double residual = x[i] - projected;
    maxResidual = std::max(maxResidual, std::abs(residual));
  }

  return maxResidual;
}

inline double complementarityInfinityNorm(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double tol)
{
  const Eigen::Index n = x.size();
  DART_ASSERT(w.size() == n);
  DART_ASSERT(lo.size() == n);
  DART_ASSERT(hi.size() == n);

  double maxViolation = 0.0;

  for (Eigen::Index i = 0; i < n; ++i) {
    const double xi = x[i];
    const double wi = w[i];

    double violation = 0.0;
    if (std::isfinite(lo[i]) && xi < lo[i] - tol) {
      violation = std::max(violation, lo[i] - xi);
    }
    if (std::isfinite(hi[i]) && xi > hi[i] + tol) {
      violation = std::max(violation, xi - hi[i]);
    }

    const bool hasLo = std::isfinite(lo[i]);
    const bool hasUpper = std::isfinite(hi[i]);
    const bool atLo = hasLo && std::abs(xi - lo[i]) <= tol;
    const bool atHi = hasUpper && std::abs(xi - hi[i]) <= tol;
    const bool fixed = atLo && atHi;
    const bool interior = (!atLo && !atHi) && (!hasLo || xi > lo[i] + tol)
                          && (!hasUpper || xi < hi[i] - tol);

    if (fixed) {
      // A collapsed interval has no feasible motion, so its normal cone admits
      // any gradient value once the variable is at the fixed bound.
      continue;
    } else if (atLo) {
      violation = std::max(violation, std::max(0.0, -wi));
    } else if (atHi) {
      violation = std::max(violation, std::max(0.0, wi));
    } else if (interior) {
      violation = std::max(violation, std::abs(wi));
    }

    maxViolation = std::max(maxViolation, violation);
  }

  return maxViolation;
}

inline bool validateSolution(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double tol,
    std::string* message = nullptr)
{
  const Eigen::Index n = x.size();
  if (w.size() != n || lo.size() != n || hi.size() != n) {
    if (message) {
      *message = "Solution validation failed: dimension mismatch";
    }
    return false;
  }

  if (!x.allFinite() || !w.allFinite()) {
    if (message) {
      *message = "Solution validation failed: non-finite values";
    }
    return false;
  }

  for (Eigen::Index i = 0; i < n; ++i) {
    const double xi = x[i];
    const double wi = w[i];

    if (std::isfinite(lo[i]) && xi < lo[i] - tol) {
      if (message) {
        *message = "Solution validation failed: lower bound violation";
      }
      return false;
    }
    if (std::isfinite(hi[i]) && xi > hi[i] + tol) {
      if (message) {
        *message = "Solution validation failed: upper bound violation";
      }
      return false;
    }

    const bool hasLo = std::isfinite(lo[i]);
    const bool hasUpper = std::isfinite(hi[i]);
    const bool atLo = hasLo && std::abs(xi - lo[i]) <= tol;
    const bool atHi = hasUpper && std::abs(xi - hi[i]) <= tol;
    const bool fixed = atLo && atHi;
    const bool interior = (!atLo && !atHi) && (!hasLo || xi > lo[i] + tol)
                          && (!hasUpper || xi < hi[i] - tol);

    if (fixed) {
      // A collapsed interval has no feasible motion, so its normal cone admits
      // any gradient value once the variable is at the fixed bound.
      continue;
    } else if (atLo) {
      if (wi < -tol) {
        if (message) {
          *message = "Solution validation failed: w must be non-negative at lo";
        }
        return false;
      }
    } else if (atHi) {
      if (wi > tol) {
        if (message) {
          *message = "Solution validation failed: w must be non-positive at hi";
        }
        return false;
      }
    } else if (interior) {
      if (std::abs(wi) > tol) {
        if (message) {
          *message = "Solution validation failed: w must be zero in interior";
        }
        return false;
      }
    }
  }

  return true;
}

inline bool acceptStrictInteriorStandardCandidate(
    const LcpProblem& problem,
    double interiorTolerance,
    double validationTolerance,
    Eigen::VectorXd candidate,
    Eigen::VectorXd& x,
    Eigen::VectorXd* wOut)
{
  if (!candidate.allFinite()) {
    return false;
  }

  const double strictInteriorTol = std::max(0.0, interiorTolerance);
  if (candidate.minCoeff() <= strictInteriorTol) {
    return false;
  }

  Eigen::VectorXd w = problem.A * candidate - problem.b;
  if (!validateSolution(
          candidate, w, problem.lo, problem.hi, validationTolerance)) {
    return false;
  }

  x = std::move(candidate);
  if (wOut) {
    *wOut = std::move(w);
  }
  return true;
}

inline bool trySolveStrictInteriorStandardLcp(
    const LcpProblem& problem,
    double interiorTolerance,
    double validationTolerance,
    Eigen::VectorXd& x,
    Eigen::VectorXd* wOut = nullptr)
{
  if (!problem.isStandardLcp(interiorTolerance)) {
    return false;
  }

  return acceptStrictInteriorStandardCandidate(
      problem,
      interiorTolerance,
      validationTolerance,
      problem.A.partialPivLu().solve(problem.b),
      x,
      wOut);
}

inline bool trySolveStrictInteriorStandardLcpLltFirst(
    const LcpProblem& problem,
    double interiorTolerance,
    double validationTolerance,
    Eigen::VectorXd& x,
    Eigen::VectorXd* wOut = nullptr)
{
  if (!problem.isStandardLcp(interiorTolerance)) {
    return false;
  }

  const Eigen::LLT<Eigen::MatrixXd> llt(problem.A);
  if (llt.info() == Eigen::Success
      && acceptStrictInteriorStandardCandidate(
          problem,
          interiorTolerance,
          validationTolerance,
          llt.solve(problem.b),
          x,
          wOut)) {
    return true;
  }

  return acceptStrictInteriorStandardCandidate(
      problem,
      interiorTolerance,
      validationTolerance,
      problem.A.partialPivLu().solve(problem.b),
      x,
      wOut);
}

inline bool trySolveProjectedActiveSetBoxedLcp(
    const LcpProblem& problem,
    double activeSetTolerance,
    double validationTolerance,
    Eigen::VectorXd& x,
    Eigen::VectorXd* wOut = nullptr)
{
  if (!problem.isBoxedLcp()) {
    return false;
  }

  const Eigen::Index n = problem.size();
  const auto solveDense = [](const Eigen::MatrixXd& A,
                             const Eigen::VectorXd& b,
                             bool lltFirst) -> Eigen::VectorXd {
    if (lltFirst) {
      const Eigen::LLT<Eigen::MatrixXd> llt(A);
      if (llt.info() == Eigen::Success) {
        Eigen::VectorXd candidate = llt.solve(b);
        if (candidate.allFinite()) {
          return candidate;
        }
      }
    }
    return A.partialPivLu().solve(b);
  };

  const auto tryCandidate = [&](bool lltFirst) -> bool {
    Eigen::VectorXd unconstrained = solveDense(problem.A, problem.b, lltFirst);
    if (!unconstrained.allFinite()) {
      return false;
    }

    const double activeTol = std::max(0.0, activeSetTolerance);
    Eigen::VectorXd candidate = Eigen::VectorXd::Zero(n);
    std::vector<Eigen::Index> freeRows;
    freeRows.reserve(static_cast<std::size_t>(n));

    for (Eigen::Index i = 0; i < n; ++i) {
      const double lo = problem.lo[i];
      const double hi = problem.hi[i];
      const double value = unconstrained[i];
      const bool hasLo = std::isfinite(lo);
      const bool upperBoundFinite = std::isfinite(hi);
      const bool fixed = hasLo && upperBoundFinite
                         && std::abs(hi - lo) <= validationTolerance;

      if (fixed) {
        candidate[i] = 0.5 * (lo + hi);
      } else if (hasLo && value <= lo + activeTol) {
        candidate[i] = lo;
      } else if (upperBoundFinite && value >= hi - activeTol) {
        candidate[i] = hi;
      } else {
        freeRows.push_back(i);
      }
    }

    if (freeRows.size() == static_cast<std::size_t>(n)) {
      candidate = std::move(unconstrained);
    } else if (!freeRows.empty()) {
      const Eigen::Index freeCount = static_cast<Eigen::Index>(freeRows.size());
      Eigen::MatrixXd reducedA(freeCount, freeCount);
      Eigen::VectorXd reducedB(freeCount);

      for (Eigen::Index r = 0; r < freeCount; ++r) {
        const Eigen::Index row = freeRows[static_cast<std::size_t>(r)];
        double rhs = problem.b[row];
        for (Eigen::Index c = 0; c < n; ++c) {
          rhs -= problem.A(row, c) * candidate[c];
        }

        for (Eigen::Index c = 0; c < freeCount; ++c) {
          const Eigen::Index col = freeRows[static_cast<std::size_t>(c)];
          reducedA(r, c) = problem.A(row, col);
          rhs += problem.A(row, col) * candidate[col];
        }

        reducedB[r] = rhs;
      }

      Eigen::VectorXd reducedX = solveDense(reducedA, reducedB, lltFirst);
      if (!reducedX.allFinite()) {
        return false;
      }

      for (Eigen::Index i = 0; i < freeCount; ++i) {
        candidate[freeRows[static_cast<std::size_t>(i)]] = reducedX[i];
      }
    }

    Eigen::VectorXd w = problem.A * candidate - problem.b;
    if (!validateSolution(
            candidate, w, problem.lo, problem.hi, validationTolerance)) {
      return false;
    }

    x = std::move(candidate);
    if (wOut) {
      *wOut = std::move(w);
    }
    return true;
  };

  return tryCandidate(true) || tryCandidate(false);
}

inline bool trySolveInteriorFrictionIndexLcp(
    const LcpProblem& problem,
    double interiorTolerance,
    double validationTolerance,
    Eigen::VectorXd& x,
    Eigen::VectorXd* wOut = nullptr)
{
  if (!problem.hasFrictionIndex()) {
    return false;
  }

  const auto tryCandidate = [&](Eigen::VectorXd candidate) -> bool {
    if (!candidate.allFinite()) {
      return false;
    }

    Eigen::VectorXd loEff;
    Eigen::VectorXd hiEff;
    if (!computeEffectiveBounds(
            problem.lo, problem.hi, problem.findex, candidate, loEff, hiEff)) {
      return false;
    }

    const double strictInteriorTol = std::max(0.0, interiorTolerance);
    for (Eigen::Index i = 0; i < candidate.size(); ++i) {
      if (std::isfinite(loEff[i])
          && candidate[i] <= loEff[i] + strictInteriorTol) {
        return false;
      }
      if (std::isfinite(hiEff[i])
          && candidate[i] >= hiEff[i] - strictInteriorTol) {
        return false;
      }
    }

    Eigen::VectorXd w = problem.A * candidate - problem.b;
    if (!validateSolution(candidate, w, loEff, hiEff, validationTolerance)) {
      return false;
    }

    x = std::move(candidate);
    if (wOut) {
      *wOut = std::move(w);
    }
    return true;
  };

  const Eigen::LLT<Eigen::MatrixXd> llt(problem.A);
  if (llt.info() == Eigen::Success && tryCandidate(llt.solve(problem.b))) {
    return true;
  }

  return tryCandidate(problem.A.partialPivLu().solve(problem.b));
}

} // namespace dart::math::detail
