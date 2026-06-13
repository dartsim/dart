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

#include "dart/math/NonNegativeLeastSquares.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <vector>

namespace dart {
namespace math {

//==============================================================================
bool solveNonNegativeLeastSquares(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    double tolerance,
    std::size_t maxIterations)
{
  const Eigen::Index m = A.rows();
  const Eigen::Index n = A.cols();

  x.setZero(n);

  if (b.size() != m) {
    return false;
  }

  if (!A.allFinite() || !b.allFinite()) {
    return false;
  }

  if (n == 0 || m == 0) {
    return true;
  }

  const double tol = tolerance > 0.0
                         ? tolerance
                         : 10.0 * std::numeric_limits<double>::epsilon()
                               * static_cast<double>(std::max(m, n))
                               * A.cwiseAbs().maxCoeff()
                               * std::max(1.0, b.cwiseAbs().maxCoeff());

  // The cap counts inner iterations (including alpha-step removals), which
  // is stricter than the classic 3n outer-iteration guidance, so leave
  // generous headroom; the cap only exists as an anti-cycling backstop.
  const std::size_t maxIters
      = maxIterations > 0 ? maxIterations : 10 * static_cast<std::size_t>(n);

  std::vector<bool> passive(static_cast<std::size_t>(n), false);
  std::vector<Eigen::Index> passiveIndices;
  passiveIndices.reserve(static_cast<std::size_t>(n));

  // Dual vector of the nonnegativity constraints: w = A^T * (b - A * x).
  Eigen::VectorXd w = A.transpose() * b;

  Eigen::MatrixXd passiveA;
  Eigen::VectorXd z;

  std::size_t iterations = 0;
  while (true) {
    // Select the most violated dual among the active (clamped) variables.
    Eigen::Index candidate = -1;
    double maxDual = tol;
    for (Eigen::Index i = 0; i < n; ++i) {
      if (!passive[static_cast<std::size_t>(i)] && w[i] > maxDual) {
        maxDual = w[i];
        candidate = i;
      }
    }

    // Optimal: every active variable has a non-positive dual.
    if (candidate < 0) {
      return true;
    }

    passive[static_cast<std::size_t>(candidate)] = true;
    passiveIndices.push_back(candidate);

    while (true) {
      if (++iterations > maxIters) {
        return false;
      }

      // Unconstrained least squares restricted to the passive columns.
      const Eigen::Index numPassive
          = static_cast<Eigen::Index>(passiveIndices.size());
      passiveA.resize(m, numPassive);
      for (Eigen::Index k = 0; k < numPassive; ++k) {
        passiveA.col(k) = A.col(passiveIndices[static_cast<std::size_t>(k)]);
      }
      z = passiveA.colPivHouseholderQr().solve(b);

      if ((z.array() > 0.0).all()) {
        x.setZero();
        for (Eigen::Index k = 0; k < numPassive; ++k) {
          x[passiveIndices[static_cast<std::size_t>(k)]] = z[k];
        }
        break;
      }

      // Step from x toward z just far enough to keep feasibility, then drop
      // every variable that reached zero from the passive set.
      double alpha = 1.0;
      for (Eigen::Index k = 0; k < numPassive; ++k) {
        if (z[k] <= 0.0) {
          const double xk = x[passiveIndices[static_cast<std::size_t>(k)]];
          const double denominator = xk - z[k];
          // Guard the 0/0 case (xk == 0 with z exactly 0 on a rank-deficient
          // passive set): the correct step is 0.
          const double step = denominator > 0.0 ? xk / denominator : 0.0;
          alpha = std::min(alpha, step);
        }
      }

      for (Eigen::Index k = 0; k < numPassive; ++k) {
        const Eigen::Index idx = passiveIndices[static_cast<std::size_t>(k)];
        x[idx] += alpha * (z[k] - x[idx]);
      }

      const double dropTol = std::numeric_limits<double>::epsilon()
                             * std::max(1.0, x.maxCoeff());
      for (auto it = passiveIndices.begin(); it != passiveIndices.end();) {
        if (x[*it] <= dropTol) {
          x[*it] = 0.0;
          passive[static_cast<std::size_t>(*it)] = false;
          it = passiveIndices.erase(it);
        } else {
          ++it;
        }
      }

      if (passiveIndices.empty()) {
        break;
      }
    }

    w.noalias() = A.transpose() * (b - A * x);
  }
}

} // namespace math
} // namespace dart
