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

#include "dart/constraint/ConstrainedGroup.h"

#include <iostream>
#include <vector>

#include "dart/common/Console.h"
#include "dart/lcpsolver/LCPSolver.h"
#include "dart/lcpsolver/Lemke.h"
#include "dart/lcpsolver/lcp.h"
#include "dart/constraint/Constraint.h"
#include "dart/constraint/ConstraintSolver.h"

namespace dart {
namespace constraint {

using namespace lcpsolver;

//==============================================================================
ConstrainedGroup::ConstrainedGroup(ConstraintSolver* _solver)
  : mConstraintSolver(_solver)
{
  assert(_solver);
}

//==============================================================================
ConstrainedGroup::~ConstrainedGroup()
{
}

//==============================================================================
void ConstrainedGroup::addConstraint(Constraint* _constraint)
{
  assert(_constraint != NULL && "Null constraint pointer is now allowed.");
  assert(_containConstraint(_constraint) == false
         && "Don't try to add same constraint multiple times into Community.");

  mConstraints.push_back(_constraint);
}

//==============================================================================
void ConstrainedGroup::removeConstraint(Constraint* _constraint)
{
  assert(_constraint != NULL && "Null constraint pointer is now allowed.");
  assert(_containConstraint(_constraint) == true
         && "Don't try to remove a constraint not contained in Community.");

  mConstraints.erase(
        remove(mConstraints.begin(), mConstraints.end(), _constraint),
        mConstraints.end());
}

//==============================================================================
void ConstrainedGroup::removeAllConstraints()
{
//  dtwarn << "ConstrainedGroup::removeAllConstraints(): "
//         << "Not implemented." << std::endl;

  // TODO(JS): Temporary implementation
//  for (int i = 0; i < mConstraints.size(); ++i)
//  {
//    delete mConstraints[i];
//  }

  mConstraints.clear();
}

//==============================================================================
bool ConstrainedGroup::solve()
{
  // If there is no constraint, then just return true.
  if (mConstraints.size() == 0)
    return true;

//  for (std::vector<Constraint*>::iterator it = mConstraints)

  // Build LCP terms by aggregating them from constraints
  ODELcp lcp(getTotalDimension());

  // Fill LCP terms
  lcp.invTimestep = 1.0 / mConstraintSolver->getTimeStep();
  _fillLCPTermsODE(&lcp);

  //////////////////////////////////////////////////////////////////////////////
//  dtmsg << "Before solve" << std::endl;
//  lcp.print();
  //////////////////////////////////////////////////////////////////////////////

  // Solve LCP
  bool result = _solveODE(&lcp);

//  dtmsg << "After solve" << std::endl;
//  lcp.print();

  // Apply impulse
  _applyODE(&lcp);

  return result;
}

//==============================================================================
bool ConstrainedGroup::_containConstraint(Constraint* _constraint) const
{
//  std::cout << "CommunityTEST::_containConstraint(): Not implemented."
//            << std::endl;

  return false;
}

//==============================================================================
bool ConstrainedGroup::_checkAndAddConstraint(Constraint* _constraint)
{
  std::cout << "CommunityTEST::_checkAndAddConstraint(): Not implemented."
            << std::endl;

  return false;
}

//==============================================================================
void ConstrainedGroup::_fillLCPTermsODE(ODELcp* _lcp)
{
  _lcp->clear();

  // Compute offset indices
  int* offsetIndex = new int[_lcp->dim];
  offsetIndex[0] = 0;
  for (int i = 1; i < mConstraints.size(); ++i)
  {
    assert(mConstraints[i - 1]->getDimension() > 0);
    offsetIndex[i] = offsetIndex[i - 1] + mConstraints[i - 1]->getDimension();
//    std::cout << "offsetIndex[" << i << "]: " << offsetIndex[i] << std::endl;
  }

  // For each constraint
  Constraint* constraint;
  for (int i = 0; i < mConstraints.size(); ++i)
  {
    constraint = mConstraints[i];

    // Update constraint
    constraint->update();

    // Fill vectors: lo, hi, b, w
    constraint->fillLcpOde(_lcp, offsetIndex[i]);

    // Fill a matrix by impulse tests: A
    constraint->excite();
    for (int j = 0; j < constraint->getDimension(); ++j)
    {
      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

//      _lcp->print();
//      std::cout << std::endl;

      mConstraints[i]->getVelocityChange(
            _lcp->A, _lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[i], true);

      // Fill upper triangle blocks of A matrix
      for (int k = i + 1; k < mConstraints.size(); ++k)
      {
        int index = _lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[k];
        mConstraints[k]->getVelocityChange(_lcp->A, index, false);

//        std::cout << "index: " << index << std::endl;

//        _lcp->print();
//        std::cout << std::endl;
      }

      assert(_lcp->checkSymmetric2(j));

//      std::cout << "idx: " << _lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[i] + j << std::endl;

//      if (j == 0)

//      std::cout << "A: " << _lcp->A[_lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[i] + j] << std::endl;
//      std::cout << "index: " << _lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[i] + j << std::endl;

//      _lcp->print();
//      std::cout << std::endl;

//      std::cout << "A: " << _lcp->A[_lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[i] + j] << std::endl;

      // Filling symmetric part of A matrix
      for (int k = 0; k < i; ++k)
      {
        for (int l = 0; l < mConstraints[k]->getDimension(); ++l)
        {
          int index1 = _lcp->nSkip * (offsetIndex[i] + j) + offsetIndex[k] + l;
          int index2 = _lcp->nSkip * (offsetIndex[k] + l) + offsetIndex[i] + j;

          _lcp->A[index1] = _lcp->A[index2];

//          std::cout << "index1: " << index1 << std::endl;
//          std::cout << "index2: " << index2 << std::endl;
//          _lcp->print();
//          std::cout << std::endl;
        }
      }

//      _lcp->print();
//      std::cout << std::endl;

    }
    constraint->unexcite();
  }

  assert(_lcp->checkSymmetric());

  delete[] offsetIndex;
}

//==============================================================================
bool ConstrainedGroup::_solveODE(ODELcp* _lcp)
{
//  for (int i = 0; i < _lcp->dim; ++i)
//    std::cout << "_lcp->lb[" << i << "]: " << _lcp->lb[i] << std::endl;
//  for (int i = 0; i < _lcp->dim; ++i)
//    std::cout << "_lcp->ub[" << i << "]: " << _lcp->ub[i] << std::endl;

//  for (int i = 0; i < _lcp->dim; ++i)
//    std::cout << "_lcp->x[" << i << "]: " << _lcp->x[i] << std::endl;

  // Solve LCP using ODE's Dantzig algorithm
  dSolveLCP(_lcp->dim,
            _lcp->A,
            _lcp->x,
            _lcp->b,
            _lcp->w,
            0,
            _lcp->lb,
            _lcp->ub,
            _lcp->frictionIndex);

//  std::cout << "FINISHED ----" << std::endl;

//  for (int i = 0; i < _lcp->dim; ++i)
//    std::cout << "_lcp->lb[" << i << "]: " << _lcp->lb[i] << std::endl;
//  for (int i = 0; i < _lcp->dim; ++i)
//    std::cout << "_lcp->ub[" << i << "]: " << _lcp->ub[i] << std::endl;

//  for (int i = 0; i < _lcp->dim; ++i)
//    std::cout << "_lcp->x[" << i << "]: " << _lcp->x[i] << std::endl;

  // TODO(JS): Do we need to return boolean?
  return true;
}

//==============================================================================
void ConstrainedGroup::_applyODE(ODELcp* _lcp)
{
  // Compute offset indices
  int* offsetIndex = new int[_lcp->dim];
  offsetIndex[0] = 0;
  for (int i = 1; i < mConstraints.size(); ++i)
    offsetIndex[i] = offsetIndex[i - 1] + mConstraints[i - 1]->getDimension();

  // Apply constraint impulses
  for (int i = 0; i < mConstraints.size(); ++i)
  {
    mConstraints[i]->applyConstraintImpulse(_lcp->x, offsetIndex[i]);
    mConstraints[i]->excite();
  }

  delete[] offsetIndex;
}

//==============================================================================
int ConstrainedGroup::getTotalDimension() const
{
  int totalDim = 0;

  for (int i = 0; i < mConstraints.size(); ++i)
    totalDim += mConstraints[i]->getDimension();

  return totalDim;
}

}  // namespace constraint
}  // namespace dart
