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

#ifndef DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_
#define DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_

#include <dart/common/castable.hpp>
#include <dart/common/deprecated.hpp>
#include <dart/common/diagnostics.hpp>

#include <string>
#include <string_view>

namespace dart {
namespace constraint {

/// Deprecated; will be removed in DART 8.0. Use dart::math::LcpSolver.
class DART_DEPRECATED("8.0") BoxedLcpSolver
  : public common::Castable<BoxedLcpSolver>
{
public:
  virtual ~BoxedLcpSolver() = default;

  /// Note: kept for gz-physics compatibility; prefer getTypeView().
  [[deprecated(
      "Use getTypeView() for string_view access.")]] virtual const std::string&
  getType() const
      = 0;

  // TODO(DART 8): Rename getTypeView() to getType() and drop the compatibility
  // string-returning overload.
  DART_SUPPRESS_DEPRECATED_BEGIN
  [[nodiscard]] std::string_view getTypeView() const
  {
    return getType();
  }
  DART_SUPPRESS_DEPRECATED_END

  virtual bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination = false)
      = 0;

  virtual bool canSolve(int n, const double* A) = 0;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_
