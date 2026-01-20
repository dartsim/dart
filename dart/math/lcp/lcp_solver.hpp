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

#ifndef DART_MATH_LCP_LCPSOLVER_HPP_
#define DART_MATH_LCP_LCPSOLVER_HPP_

#include <dart/math/lcp/lcp_types.hpp>

#include <dart/Export.hpp>

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <string>

namespace dart {
namespace math {

//==============================================================================
class DART_API LcpSolver
{
public:
  /// Virtual destructor
  virtual ~LcpSolver() = default;

  virtual LcpResult solve(
      const LcpProblem& problem, Eigen::VectorXd& x, const LcpOptions& options)
      = 0;

  /// Get the solver name
  ///
  /// @return Name of the solver (e.g., "Lemke", "Dantzig")
  virtual std::string getName() const = 0;

  /// Get the solver category
  ///
  /// @return Category name (e.g., "Pivoting", "Projection", "Newton")
  virtual std::string getCategory() const = 0;

  /// Get default options for this solver
  ///
  /// @return Default LcpOptions
  virtual const LcpOptions& getDefaultOptions() const;

  /// Set default options for this solver
  ///
  /// @param[in] options New default options
  virtual void setDefaultOptions(const LcpOptions& options);

protected:
  /// Default options for this solver instance
  LcpOptions mDefaultOptions;
};

//==============================================================================
/// Shared pointer type for LcpSolver
using LcpSolverPtr = std::shared_ptr<LcpSolver>;

//==============================================================================
/// Factory function type for creating solvers
using LcpSolverFactory = std::function<LcpSolverPtr()>;

} // namespace math
} // namespace dart

#endif // DART_MATH_LCP_LCPSOLVER_HPP_
