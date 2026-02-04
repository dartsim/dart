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

#include <limits>
#include <string>

#include <cmath>

namespace dart::math::detail {

inline bool validateProblem(
    const LcpProblem& problem, std::string* message = nullptr)
{
  const auto& A = problem.A;
  const auto& b = problem.b;
  const auto& lo = problem.lo;
  const auto& hi = problem.hi;
  const auto& findex = problem.findex;

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
    double projected = x[i] - w[i];
    if (std::isfinite(lo[i])) {
      projected = std::max(projected, lo[i]);
    }
    if (std::isfinite(hi[i])) {
      projected = std::min(projected, hi[i]);
    }

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
      violation = std::max(violation, std::abs(wi));
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
      if (std::abs(wi) > tol) {
        if (message) {
          *message = "Solution validation failed: fixed variable residual";
        }
        return false;
      }
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

} // namespace dart::math::detail
