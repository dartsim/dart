/*
 * Copyright (c) 2011-2021, The DART development contributors
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
  ///
  /// \param[in] timeStep Simulation time step
  /// \param[in] boxedLcpSolver The primary boxed LCP solver. When nullptr is
  /// passed, Dantzig solver will be used.
  /// \param[in] secondaryBoxedLcpSolver The secondary boxed-LCP solver. When
  /// nullptr is passed, PGS solver will be used. This is to make the default
  /// solver setting to be Dantzig + PGS. In order to disable use of secondary
  /// solver, call setSecondaryBoxedLcpSolver(nullptr) explicitly.
  ///
  /// \deprecated Deprecated in DART 6.8. Please use other constructors that
  /// doesn't take timespte. Timestep should be set by the owner of this solver
  /// such as dart::simulation::World when the solver added.
  DART_DEPRECATED(6.8)
  BoxedLcpConstraintSolver(
      double timeStep,
      BoxedLcpSolverPtr boxedLcpSolver = nullptr,
      BoxedLcpSolverPtr secondaryBoxedLcpSolver = nullptr);

  /// Constructos with default primary and secondary LCP solvers, which are
  /// Dantzig and PGS, respectively.
  BoxedLcpConstraintSolver();

  /// Constructors with specific primary LCP solver.
  ///
  /// \param[in] boxedLcpSolver The primary boxed LCP solver. When nullptr is
  /// passed, which is discouraged, Dantzig solver will be used.
  BoxedLcpConstraintSolver(BoxedLcpSolverPtr boxedLcpSolver);

  /// Constructs with specific primary and secondary LCP solvers.
  ///
  /// \param[in] boxedLcpSolver The primary boxed LCP solver. When nullptr is
  /// passed, which is discouraged, Dantzig solver will be used.
  /// \param[in] secondaryBoxedLcpSolver The secondary boxed-LCP solver. Pass
  /// nullptr to disable using secondary LCP solver.
  BoxedLcpConstraintSolver(
      BoxedLcpSolverPtr boxedLcpSolver,
      BoxedLcpSolverPtr secondaryBoxedLcpSolver);

  /// Sets boxed LCP (BLCP) solver
  ///
  /// \param[in] lcpSolver The primary boxed LCP solver. When nullptr is
  /// passed, Dantzig solver will be used.
  void setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver);

  /// Returns boxed LCP (BLCP) solver
  ConstBoxedLcpSolverPtr getBoxedLcpSolver() const;

  /// Sets boxed LCP (BLCP) solver that is used when the primary solver failed
  void setSecondaryBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver);

  /// Returns boxed LCP (BLCP) solver that is used when the primary solver
  /// failed
  ConstBoxedLcpSolverPtr getSecondaryBoxedLcpSolver() const;

protected:
  // Documentation inherited.
  void solveConstrainedGroup(ConstrainedGroup& group) override;

  /// Boxed LCP solver
  BoxedLcpSolverPtr mBoxedLcpSolver;
  // TODO(JS): Hold as unique_ptr because there is no reason to share. Make this
  // change in DART 7 because it's API breaking change.

  /// Boxed LCP solver to be used when the primary solver failed
  BoxedLcpSolverPtr mSecondaryBoxedLcpSolver;
  // TODO(JS): Hold as unique_ptr because there is no reason to share. Make this
  // change in DART 7 because it's API breaking change.

  /// Cache data for boxed LCP formulation
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mA;

  /// Cache data for boxed LCP formulation
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      mABackup;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mX;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mXBackup;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mB;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mBBackup;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mW;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mLo;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mLoBackup;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mHi;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXd mHiBackup;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXi mFIndex;

  /// Cache data for boxed LCP formulation
  Eigen::VectorXi mFIndexBackup;

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
