/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <string>
#include <Eigen/Core>

namespace dart {
namespace constraint {

class BoxedLcpSolver
{
public:
  /// Destructor
  virtual ~BoxedLcpSolver() = default;

  /// Returns the type
  virtual const std::string& getType() const = 0;

  /// Get true if this solver and the template parameter (a solver class) are
  /// the same type. This function is a syntactic sugar, which is identical to:
  /// \code getType() == ShapeType::getStaticType() \endcode.
  ///
  /// Example code:
  /// \code
  /// auto shape = bodyNode->getShapeNode(0)->getShape();
  /// if (shape->is<BoxShape>())
  ///   std::cout << "The shape type is box!\n";
  /// \endcode
  ///
  /// \sa getType()
  template <typename BoxedLcpSolverT>
  bool is() const;

  /// Solves constriant impulses for a constrained group
  // Note: The function signature is ODE specific for now. Consider changing
  // this to Eigen friendly version once own Dantzig LCP solver is available.
  virtual void solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex)
      = 0;

#ifndef NDEBUG
  virtual bool canSolve(int n, const double* A) = 0;
#endif
};

} // namespace constraint
} // namespace dart

#include "dart/constraint/detail/BoxedLcpSolver-impl.hpp"

#endif // DART_CONSTRAINT_BOXEDLCPSOLVER_HPP_
