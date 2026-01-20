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

#ifndef DART_MATH_LCP_PIVOTING_LEMKESOLVER_HPP_
#define DART_MATH_LCP_PIVOTING_LEMKESOLVER_HPP_

#include <dart/math/lcp/lcp_solver.hpp>

#include <dart/Export.hpp>

namespace dart {
namespace math {

//==============================================================================
/// Lemke's complementary pivot algorithm for LCP
///
/// This solver uses Lemke's algorithm with an artificial variable to solve
/// standard LCPs. For boxed LCPs (including friction-index coupling), it
/// delegates to the boxed-capable pivoting solver implementation.
///
/// Reference:
///   Lemke, C. E. (1965). "Bimatrix equilibrium points and mathematical
///   programming". Management Science, 11(7), 681-689.
///
/// Properties:
///   - Time Complexity: O(n^4) worst case
///   - Space Complexity: O(n^2)
///   - Convergence: Exact solution (finite termination)
///   - Matrix Requirements: None (general LCP)
///
/// Problem formulation:
///   Find z such that:
///     Mz = q + w
///     z >= 0, w >= 0
///     z^T * w = 0
class DART_API LemkeSolver : public LcpSolver
{
public:
  /// Constructor
  LemkeSolver();
  ~LemkeSolver() override = default;

  using LcpSolver::solve;

  LcpResult solve(
      const LcpProblem& problem,
      Eigen::VectorXd& x,
      const LcpOptions& options) override;

  // Documentation inherited
  std::string getName() const override;

  // Documentation inherited
  std::string getCategory() const override;
};

} // namespace math
} // namespace dart

#endif // DART_MATH_LCP_PIVOTING_LEMKESOLVER_HPP_
