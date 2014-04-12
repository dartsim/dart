/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_CONSTRAINT_COMMUNITY_H_
#define DART_CONSTRAINT_COMMUNITY_H_

#include <vector>
#include <Eigen/Dense>

namespace dart {

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace constraint {

class ODELcp;
class Constraint;
class ConstraintSolver;

//==============================================================================
/// \brief class ConstrainedGroup is a set of skeletons interacting each
///        other by various kinds of constraints.
/// \sa class ConstraintSolver
class ConstrainedGroup
{
public:
  //----------------------------------------------------------------------------
  // Constructor / Desctructor
  //----------------------------------------------------------------------------
  /// \brief Default contructor
  explicit ConstrainedGroup(ConstraintSolver* _solver);

  /// \brief Default destructor
  virtual ~ConstrainedGroup();

  //----------------------------------------------------------------------------
  // Setting
  //----------------------------------------------------------------------------
  /// \brief Add constraint
  void addConstraint(Constraint* _constraint);

  /// \brief Remove constraint
  void removeConstraint(Constraint* _constraint);

  /// \brief Remove all constraints
  void removeAllConstraints();

  /// \brief Get total dimension of contraints in this group
  int getTotalDimension() const;

  //----------------------------------------------------------------------------
  // Solving
  //----------------------------------------------------------------------------
  // TODO(JS): Pass option
  /// \brief Solve constraints and store the results of constraint impulse to
  ///        each skeleton.
  bool solve();

  dynamics::Skeleton* mRootSkeleton;

protected:
  /// \brief List of constraints
  std::vector<Constraint*> mConstraints;

  /// \brief Constraint solver
  ConstraintSolver* mConstraintSolver;

private:
  /// \brief Check if _constraint is contained
  bool containConstraint(Constraint* _constraint) const;

  /// \brief Check if _constraint is contained and, if so, add the constraint
  bool checkAndAddConstraint(Constraint* _constraint);

  /// \brief
  void fillLCPTermsODE(ODELcp* _lcp);

//  /// \brief
//  void fillLCPTermsLemke(const LCPTermsODE& _lcp);

//  /// \brief
//  void fillLCPTermsPGS(const LCPTermsODE& _lcp);

  // TODO(JS): more solvers
  /// \brief
  bool solveODE(ODELcp* _lcp);

//  bool _solveLemke();

//  bool _solvePGS();

  void applyODE(ODELcp* _lcp);

//  // Matrices to pass to solver
//  Eigen::MatrixXd mA;
//  Eigen::VectorXd mQBar;
//  Eigen::VectorXd mX;


};

} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_COMMUNITY_H_

