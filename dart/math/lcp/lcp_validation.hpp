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

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <string>

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

template <
    typename ADerived,
    typename BDerived,
    typename LoDerived,
    typename HiDerived,
    typename FindexDerived>
inline bool validateProblemView(
    const Eigen::MatrixBase<ADerived>& A,
    const Eigen::MatrixBase<BDerived>& b,
    const Eigen::MatrixBase<LoDerived>& lo,
    const Eigen::MatrixBase<HiDerived>& hi,
    const Eigen::MatrixBase<FindexDerived>& findex,
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

    if (ref >= 0 && !std::isfinite(hi[i])) {
      if (message) {
        *message = "Friction coefficient (hi) must be finite";
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
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    std::string* message = nullptr)
{
  return validateProblemView(A, b, lo, hi, findex, message);
}

inline bool validateProblem(
    const LcpProblem& problem, std::string* message = nullptr)
{
  return validateProblem(
      problem.A, problem.b, problem.lo, problem.hi, problem.findex, message);
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
    const double mu = std::abs(hi[i]);
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

    const double bound = mu * std::abs(scale);
    loEff[i] = -bound;
    hiEff[i] = bound;
  }

  return true;
}

template <
    typename LoDerived,
    typename HiDerived,
    typename FindexDerived,
    typename XDerived,
    typename LoEffDerived,
    typename HiEffDerived>
inline bool computeEffectiveBoundsInto(
    const Eigen::MatrixBase<LoDerived>& lo,
    const Eigen::MatrixBase<HiDerived>& hi,
    const Eigen::MatrixBase<FindexDerived>& findex,
    const Eigen::MatrixBase<XDerived>& x,
    Eigen::MatrixBase<LoEffDerived>& loEff,
    Eigen::MatrixBase<HiEffDerived>& hiEff,
    std::string* message = nullptr)
{
  const Eigen::Index n = x.size();
  auto& loEffOut = loEff.derived();
  auto& hiEffOut = hiEff.derived();

  if (lo.size() != n || hi.size() != n || findex.size() != n
      || loEffOut.size() != n || hiEffOut.size() != n) {
    if (message) {
      *message = "Invalid LCP problem dimensions";
    }
    return false;
  }

  loEffOut = lo;
  hiEffOut = hi;

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
    const double mu = std::abs(hi[i]);
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

    const double bound = mu * std::abs(scale);
    loEffOut[i] = -bound;
    hiEffOut[i] = bound;
  }

  return true;
}

template <
    typename XDerived,
    typename WDerived,
    typename LoDerived,
    typename HiDerived>
inline double naturalResidualInfinityNormView(
    const Eigen::MatrixBase<XDerived>& x,
    const Eigen::MatrixBase<WDerived>& w,
    const Eigen::MatrixBase<LoDerived>& lo,
    const Eigen::MatrixBase<HiDerived>& hi)
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

inline double naturalResidualInfinityNorm(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi)
{
  return naturalResidualInfinityNormView(x, w, lo, hi);
}

template <
    typename XDerived,
    typename WDerived,
    typename LoDerived,
    typename HiDerived>
inline double complementarityInfinityNormView(
    const Eigen::MatrixBase<XDerived>& x,
    const Eigen::MatrixBase<WDerived>& w,
    const Eigen::MatrixBase<LoDerived>& lo,
    const Eigen::MatrixBase<HiDerived>& hi,
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

inline double complementarityInfinityNorm(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double tol)
{
  return complementarityInfinityNormView(x, w, lo, hi, tol);
}

template <
    typename XDerived,
    typename WDerived,
    typename LoDerived,
    typename HiDerived>
inline bool validateSolutionView(
    const Eigen::MatrixBase<XDerived>& x,
    const Eigen::MatrixBase<WDerived>& w,
    const Eigen::MatrixBase<LoDerived>& lo,
    const Eigen::MatrixBase<HiDerived>& hi,
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

inline bool validateSolution(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& w,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    double tol,
    std::string* message = nullptr)
{
  return validateSolutionView(x, w, lo, hi, tol, message);
}

} // namespace dart::math::detail
