/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_
#define DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_

#include <dart/config.hpp>

#include <dart/math/lcp/LcpTypes.hpp>

#include <dart/common/Castable.hpp>

#include <Eigen/Core>

#include <string>

namespace dart {
namespace constraint {

class BoxedLcpSolver : public common::Castable<BoxedLcpSolver>
{
public:
  /// Destructor
  virtual ~BoxedLcpSolver() = default;

  /// Returns the type (legacy identifier kept for compatibility)
  virtual const std::string& getType() const = 0;

  /// Default options for this solver
  virtual math::LcpOptions getDefaultOptions() const;
  virtual void setDefaultOptions(const math::LcpOptions& options);

  /// Solve the boxed LCP using Eigen-friendly inputs.
  virtual math::LcpResult solve(
      const Eigen::MatrixXd& A,
      const Eigen::VectorXd& b,
      const Eigen::VectorXd& lo,
      const Eigen::VectorXd& hi,
      const Eigen::VectorXi& findex,
      Eigen::VectorXd& x,
      const math::LcpOptions& options)
      = 0;

#if DART_BUILD_MODE_DEBUG
  virtual bool canSolve(const Eigen::MatrixXd& A) = 0;
#endif

protected:
  math::LcpOptions mDefaultOptions;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_
