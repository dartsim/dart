/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/constraint/DantzigSolver.h"

#include "dart/constraint/Constraint.h"
#include "dart/constraint/ConstrainedGroup.h"
#include "dart/lcpsolver/LCPSolver.h"
#include "dart/lcpsolver/Lemke.h"
#include "dart/lcpsolver/lcp.h"

namespace dart {
namespace constraint {

//==============================================================================
DantzigSolver::DantzigSolver() : LCPSolver()
{
}

//==============================================================================
DantzigSolver::~DantzigSolver()
{
}

//==============================================================================
void DantzigSolver::solve(ConstrainedGroup* _group)
{
  // If there is no constraint, then just return true.
  size_t numConstraints = _group->getNumConstraints();
  if (numConstraints == 0)
    return;

  // Build LCP terms by aggregating them from constraints
  size_t n = _group->getTotalDimension();
  int nSkip = dPAD(n);
  double* A = new double[n * nSkip];
  double* b = new double[n];
  double* x = new double[n];
  double* w = new double[n];
  double* lo = new double[n];
  double* hi = new double[n];
  int* findex = new int[n];

  ConstraintInfo totalInfo;

  // Compute offset indices
  size_t* offsetIndex = new size_t[totalInfo.dim];
  offsetIndex[0] = 0;
  for (size_t i = 1; i < numConstraints; ++i)
  {
    Constraint* constraint = _group->getConstraint(i - 1);
    assert(constraint->getDimension() > 0);
    offsetIndex[i] = offsetIndex[i - 1] + constraint->getDimension();
//    std::cout << "offsetIndex[" << i << "]: " << offsetIndex[i] << std::endl;
  }

  // For each constraint
  ConstraintInfo partialInfo;
  Constraint* constraint;
  for (int i = 0; i < numConstraints; ++i)
  {
    constraint = _group->getConstraint(i);

    partialInfo.lo            = totalInfo.lo            + offsetIndex[i];
    partialInfo.hi            = totalInfo.hi            + offsetIndex[i];
    partialInfo.b             = totalInfo.b             + offsetIndex[i];
    partialInfo.findex = totalInfo.findex + offsetIndex[i];
    partialInfo.w             = totalInfo.w             + offsetIndex[i];

    // Fill vectors: lo, hi, b, w
    constraint->getLCPVectors(&partialInfo);

    // Fill a matrix by impulse tests: A
    constraint->excite();
    for (int j = 0; j < constraint->getDimension(); ++j)
    {
      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

      constraint->getVelocityChange(
            totalInfo.A, totalInfo.nSkip * (offsetIndex[i] + j) + offsetIndex[i], true);

      // Fill upper triangle blocks of A matrix
      for (int k = i + 1; k < numConstraints; ++k)
      {
        int index = totalInfo.nSkip * (offsetIndex[i] + j) + offsetIndex[k];
        _group->getConstraint(k)->getVelocityChange(totalInfo.A, index, false);
      }

      assert(totalInfo.checkSymmetric2(j));

      // Filling symmetric part of A matrix
      for (int k = 0; k < i; ++k)
      {
        for (int l = 0; l < _group->getConstraint(k)->getDimension(); ++l)
        {
          int index1 = totalInfo.nSkip * (offsetIndex[i] + j) + offsetIndex[k] + l;
          int index2 = totalInfo.nSkip * (offsetIndex[k] + l) + offsetIndex[i] + j;

          totalInfo.A[index1] = totalInfo.A[index2];
        }
      }
    }
    constraint->unexcite();
  }

  assert(totalInfo.checkSymmetric());

  // Solve LCP using ODE's Dantzig algorithm
  dSolveLCP(totalInfo.dim,
            totalInfo.A,
            totalInfo.x,
            totalInfo.b,
            totalInfo.w,
            0,
            totalInfo.lo,
            totalInfo.hi,
            totalInfo.findex);

  // Apply constraint impulses
  for (int i = 0; i < numConstraints; ++i)
  {
    constraint = _group->getConstraint(i);
    constraint->applyConstraintImpulse(totalInfo.x + offsetIndex[i]);
    constraint->excite();
  }

  delete[] A;
  delete[] b;
  delete[] x;
  delete[] w;
  delete[] lo;
  delete[] hi;
  delete[] findex;
  delete[] offsetIndex;
}

}  // namespace constraint
}  // namespace dart
