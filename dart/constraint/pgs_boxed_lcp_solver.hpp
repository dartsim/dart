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

#ifndef DART_CONSTRAINT_PGSBOXEDLCPSOLVER_HPP_
#define DART_CONSTRAINT_PGSBOXEDLCPSOLVER_HPP_

#include <dart/constraint/boxed_lcp_solver.hpp>

#include <dart/Export.hpp>

namespace dart {
namespace constraint {

/// Deprecated; will be removed in DART 8.0. Use dart::math::LcpSolver.
class DART_API DART_DEPRECATED("8.0") PgsBoxedLcpSolver : public BoxedLcpSolver
{
public:
  struct DART_API Option
  {
    int mMaxIteration;
    double mDeltaXThreshold;
    double mRelativeDeltaXTolerance;
    double mEpsilonForDivision;
    bool mRandomizeConstraintOrder;

    Option(
        int maxIteration = 30,
        double deltaXTolerance = 1e-6,
        double relativeDeltaXTolerance = 1e-3,
        double epsilonForDivision = 1e-9,
        bool randomizeConstraintOrder = false);
  };

  [[deprecated("Use getTypeView() for string_view access.")]] const std::string&
  getType() const override;

  static const std::string& getStaticType();

  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination) override;

  bool canSolve(int n, const double* A) override;

  void setOption(const Option& option);

  const Option& getOption() const;

protected:
  Option mOption;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_PGSBOXEDLCPSOLVER_HPP_
