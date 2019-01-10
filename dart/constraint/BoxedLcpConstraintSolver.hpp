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

#ifndef DART_CONSTRAINT_BOXEDLCPCONSTRAINTSOLVER_HPP_
#define DART_CONSTRAINT_BOXEDLCPCONSTRAINTSOLVER_HPP_

#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/constraint/SmartPointer.hpp"

namespace dart {
namespace constraint {

class BoxedLcpConstraintSolver : public ConstraintSolver
{
public:
  /// Constructor
  BoxedLcpConstraintSolver(
      double timeStep, BoxedLcpSolverPtr boxedLcpSolver = nullptr);

  /// Sets boxed LCP (BLCP) solver
  void setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver);

  /// Returns boxed LCP (BLCP) solver
  ConstBoxedLcpSolverPtr getBoxedLcpSolver() const;

protected:
  // Documentation inherited.
  void solveConstrainedGroup(ConstrainedGroup& group) override;

  /// Boxed LCP solver
  BoxedLcpSolverPtr mBoxedLcpSolver;

  /// Cache data for boxed LCP formulation
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mA;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mX;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mB;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mW;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mLo;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mHi;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXi mFIndex;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXi mOffset;

#ifndef NDEBUG
private:
  /// Return true if the matrix is symmetric
  bool isSymmetric(std::size_t n, double* A);

  /// Return true if the diagonla block of matrix is symmetric
  bool isSymmetric(
      std::size_t n, double* A, std::size_t begin, std::size_t end);

  /// Print debug information
  void print(
      std::size_t n,
      double* A,
      double* x,
      double* lo,
      double* hi,
      double* b,
      double* w,
      int* findex);
#endif
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_BOXEDLCPCONSTRAINTSOLVER_HPP_
