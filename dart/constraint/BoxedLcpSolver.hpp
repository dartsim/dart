/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/common/Castable.hpp"

#include <Eigen/Core>

#include <string>

namespace dart {
namespace constraint {

class BoxedLcpSolver : public common::Castable<BoxedLcpSolver>
{
public:
  /// Destructor
  virtual ~BoxedLcpSolver() = default;

  /// Returns the type
  virtual const std::string& getType() const = 0;

  /// Solves constriant impulses for a constrained group. The LCP formulation
  /// setting that this function solve is A*x = b + w where each x[i], w[i]
  /// satisfies one of
  ///   (1) x = lo, w >= 0
  ///   (2) x = hi, w <= 0
  ///   (3) lo < x < hi, w = 0
  ///
  /// \param[in] n Dimension of constraints.
  /// \param[in] A A term of the LCP formulation.
  /// \param[in] x x term of the LCP formulation.
  /// \param[in] b b term of the LCP formulation.
  /// \param[in] nub Number of the first unbounded constraints.
  /// \param[in] lo Lower bound of x where it's restricted to be lo <= 0.
  /// \param[in] hi Upper bound of x where it's enforced to be hi >= 0.
  /// \param[in] findex Indices to corresponding normal contact constraint. Set
  /// the index to itself (e.g., findex[k] = k) for normal contacts or
  /// non-contact constraints. For friction constraint, set the cooresponding
  /// normal contact constraint.
  /// \param[in] earlyTermination Set true to return false as soon as the solver
  /// find the solution doesn't exist. Otherwise, the solver will continue to
  /// push hard to solve the problem using some hacks.
  ///
  /// \return Success.
  // Note: The function signature is ODE specific for now. Consider changing
  // this to Eigen friendly version once own Dantzig LCP solver is available.
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

#ifndef NDEBUG
  virtual bool canSolve(int n, const double* A) = 0;
#endif
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_
