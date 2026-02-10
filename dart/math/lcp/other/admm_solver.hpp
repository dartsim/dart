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

#include <dart/math/lcp/lcp_solver.hpp>

namespace dart::math {

/// ADMM (Alternating Direction Method of Multipliers) solver for boxed LCP.
///
/// Splits the problem into a linear solve + box projection, iterating:
///   x = solve((A + rho*I), rho*z - y + b)
///   z = clamp(x + y/rho, lo, hi)
///   y = y + rho*(x - z)
///
/// Features adaptive rho based on primal/dual residual balancing (OSQP-style).
///
/// References:
/// - Carpentier et al. "From Compliant to Rigid Contact Simulation" (RSS 2024)
/// - Stellato et al. "OSQP: An Operator Splitting Solver" (2020)
class DART_API AdmmSolver : public LcpSolver
{
public:
  struct Parameters
  {
    double rhoInit{1.0};
    double muProx{1e-9};
    double adaptiveRhoTolerance{5.0};
    bool adaptiveRho{true};
  };

  AdmmSolver();
  ~AdmmSolver() override = default;

  using LcpSolver::solve;

  LcpResult solve(
      const LcpProblem& problem,
      Eigen::VectorXd& x,
      const LcpOptions& options) override;

  void setParameters(const Parameters& params);
  const Parameters& getParameters() const;

  std::string getName() const override;
  std::string getCategory() const override;

private:
  Parameters mParameters;
};

} // namespace dart::math
