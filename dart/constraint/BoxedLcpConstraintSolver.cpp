/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"

#include <cassert>
#ifndef NDEBUG
#include <iomanip>
#include <iostream>
#endif

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/common/Console.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/lcpsolver/Lemke.hpp"

namespace dart {
namespace constraint {

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    double timeStep, BoxedLcpSolverPtr boxedLcpSolver)
  : ConstraintSolver(timeStep), mBoxedLcpSolver(std::move(boxedLcpSolver))
{
  if (!mBoxedLcpSolver)
    mBoxedLcpSolver = std::make_shared<DantzigBoxedLcpSolver>();
}

//==============================================================================
void BoxedLcpConstraintSolver::setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver)
{
  if (!lcpSolver)
  {
    dtwarn << "[BoxedLcpConstraintSolver::setBoxedLcpSolver] "
           << "nullptr for boxed LCP solver is not allowed.";
    return;
  }

  mBoxedLcpSolver = std::move(lcpSolver);
}

//==============================================================================
ConstBoxedLcpSolverPtr BoxedLcpConstraintSolver::getBoxedLcpSolver() const
{
  return mBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::solveConstrainedGroup(ConstrainedGroup& group)
{
  solveConstrainedGroup2(group);
  return;

  // Build LCP terms by aggregating them from constraints
  const std::size_t numConstraints = group.getNumConstraints();
  const std::size_t n = group.getTotalDimension();

  // If there is no constraint, then just return.
  if (0u == n)
    return;

  const int nSkip = dPAD(n);
#ifdef NDEBUG // release
  mA.resize(n, nSkip);
#else // debug
  mA.setZero(n, nSkip);
#endif
  mX.resize(n);
  mB.resize(n);
  mW.setZero(n); // set w to 0
  mLo.resize(n);
  mHi.resize(n);
  mFIndex.setConstant(n, -1); // set findex to -1

  // Compute offset indices
  mConstraintOffset.resize(numConstraints);
  mConstraintOffset[0] = 0;
  for (std::size_t i = 1; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = group.getConstraint(i - 1);
    assert(constraint->getDimension() > 0);
    mConstraintOffset[i]
        = mConstraintOffset[i - 1] + constraint->getDimension();
  }

  // For each constraint
  ConstraintInfo constInfo;
  constInfo.invTimeStep = 1.0 / mTimeStep;
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = group.getConstraint(i);

    constInfo.x = mX.data() + mConstraintOffset[i];
    constInfo.lo = mLo.data() + mConstraintOffset[i];
    constInfo.hi = mHi.data() + mConstraintOffset[i];
    constInfo.b = mB.data() + mConstraintOffset[i];
    constInfo.findex = mFIndex.data() + mConstraintOffset[i];
    constInfo.w = mW.data() + mConstraintOffset[i];

    // Fill vectors: lo, hi, b, w
    constraint->getInformation(&constInfo);

    // Fill a matrix by impulse tests: A
    constraint->excite();
    for (std::size_t j = 0; j < constraint->getDimension(); ++j)
    {
      // Adjust findex for global index
      if (mFIndex[mConstraintOffset[i] + j] >= 0)
        mFIndex[mConstraintOffset[i] + j] += mConstraintOffset[i];

      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

      // Fill upper triangle blocks of A matrix
      int index = nSkip * (mConstraintOffset[i] + j) + mConstraintOffset[i];
      constraint->getVelocityChange(mA.data() + index, true);
      for (std::size_t k = i + 1; k < numConstraints; ++k)
      {
        index = nSkip * (mConstraintOffset[i] + j) + mConstraintOffset[k];
        group.getConstraint(k)->getVelocityChange(mA.data() + index, false);
      }

      // Filling symmetric part of A matrix
      for (std::size_t k = 0; k < i; ++k)
      {
        const int indexI = mConstraintOffset[i] + j;
        for (std::size_t l = 0; l < group.getConstraint(k)->getDimension(); ++l)
        {
          const int indexJ = mConstraintOffset[k] + l;
          mA(indexI, indexJ) = mA(indexJ, indexI);
        }
      }
    }

    assert(
        isSymmetric(
            n,
            mA.data(),
            mConstraintOffset[i],
            mConstraintOffset[i] + constraint->getDimension() - 1));

    constraint->unexcite();
  }

  assert(isSymmetric(n, mA.data()));

  // Print LCP formulation
  //  dtdbg << "Before solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Solve LCP using ODE's Dantzig algorithm
  assert(mBoxedLcpSolver);
  mBoxedLcpSolver->solve(
      n,
      mA.data(),
      mX.data(),
      mB.data(),
      0,
      mLo.data(),
      mHi.data(),
      mFIndex.data());

  // Print LCP formulation
  //  dtdbg << "After solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Apply constraint impulses
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = group.getConstraint(i);
    constraint->applyImpulse(mX.data() + mConstraintOffset[i]);
    constraint->excite();
  }
}

//==============================================================================
void BoxedLcpConstraintSolver::solveConstrainedGroup2(ConstrainedGroup& group)
{
  // Sum of all the constraint dimensions
  const std::size_t n = group.computeTotalDimension();

  // If there is no constraint, then just return.
  if (0u == n)
    return;

  // Build LCP terms by aggregating them from constraints
  const std::size_t numConstraints = group.getNumConstraints();

  int totalConstraintDim = 0;
  for (std::size_t i = 0; i < numConstraints; ++i)
    totalConstraintDim += group.getConstraint(i)->getDimension();
  // TODO(JS): Consider having these values as precomputed in ConstrainedGroup

  const int nSkip = dPAD(n);
#ifdef NDEBUG // release
  mA.resize(n, nSkip);
#else // debug
  mA.setZero(n, nSkip);
#endif
  mX.resize(n);
  mB.resize(n);
  mW.setZero(n); // set w to 0
  mLo.resize(n);
  mHi.resize(n);
  mFIndex.setConstant(n, -1); // set findex to -1

  mSkeletonNumDofsOffsetMap.clear();
  mSkeletonNumDofsOffsetMap.reserve(mSkeletons.size());
  std::size_t skeletonNumDofsOffset = 0;
  for (const dynamics::SkeletonPtr& skel : mSkeletons)
  {
    mSkeletonNumDofsOffsetMap[skel.get()] = skeletonNumDofsOffset;
    skeletonNumDofsOffset += skel->getNumDofs();
  }

  // Compute offset indices
  mConstraintOffset.resize(numConstraints);
  mConstraintOffset[0] = 0;
  for (std::size_t i = 1; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = group.getConstraint(i - 1);
    assert(constraint->getDimension() > 0);
    mConstraintOffset[i]
        = mConstraintOffset[i - 1] + constraint->getDimension();
  }

  mJ.setZero(totalConstraintDim, n);
  mInvMJtran.setZero(n, totalConstraintDim);

  // For each constraint
  ConstraintInfo constInfo;
  constInfo.invTimeStep = 1.0 / mTimeStep;
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = group.getConstraint(i);

    constInfo.x = mX.data() + mConstraintOffset[i];
    constInfo.lo = mLo.data() + mConstraintOffset[i];
    constInfo.hi = mHi.data() + mConstraintOffset[i];
    constInfo.b = mB.data() + mConstraintOffset[i];
    constInfo.findex = mFIndex.data() + mConstraintOffset[i];
    constInfo.w = mW.data() + mConstraintOffset[i];

    // Fill vectors: lo, hi, b, w
    constraint->getInformation(&constInfo);

    // Fill a matrix by impulse tests: A
    constraint->excite();
    for (std::size_t j = 0; j < constraint->getDimension(); ++j)
    {
      // Adjust findex for global index
      if (mFIndex[mConstraintOffset[i] + j] >= 0)
        mFIndex[mConstraintOffset[i] + j] += mConstraintOffset[i];
    }

    for (std::size_t j = 0u; j < constraint->getNumSkeletons(); ++j)
    {
      dynamics::Skeleton* skel = constraint->getSkeleton(j);
      const std::size_t skelNumDofsIndex = mSkeletonNumDofsOffsetMap[skel];
      const std::size_t size = skel->getNumDofs();

      mJ.block(
          mConstraintOffset[i],
          skelNumDofsIndex,
          constraint->getDimension(),
          size)
          = constraint->getJacobian(j);

      mInvMJtran.block(
          skelNumDofsIndex,
          mConstraintOffset[i],
          size,
          constraint->getDimension())
          = constraint->getUnitResponses(j);
    }

//    assert(
//        isSymmetric(
//            numTotalConstDim,
//            mA.data(),
//            mConstraintOffset[i],
//            mConstraintOffset[i] + constraint->getDimension() - 1));

    constraint->unexcite();
  }

  mA.noalias() = mJ * mInvMJtran;

  assert(isSymmetric(n, mA.data(), 1e-2));

  // Print LCP formulation
  //  dtdbg << "Before solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Solve LCP using ODE's Dantzig algorithm
  assert(mBoxedLcpSolver);
  mBoxedLcpSolver->solve(
      n,
      mA.data(),
      mX.data(),
      mB.data(),
      0,
      mLo.data(),
      mHi.data(),
      mFIndex.data());

  // Print LCP formulation
  //  dtdbg << "After solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Apply constraint impulses
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = group.getConstraint(i);
    constraint->applyImpulse(mX.data() + mConstraintOffset[i]);
    constraint->excite();
  }
}

//==============================================================================
#ifndef NDEBUG
bool BoxedLcpConstraintSolver::isSymmetric(std::size_t n, double* A, double tol)
{
  std::size_t nSkip = dPAD(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    for (std::size_t j = 0; j < n; ++j)
    {
      if (std::abs(A[nSkip * i + j] - A[nSkip * j + i]) > tol)
      {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < n; ++k)
        {
          for (std::size_t l = 0; l < nSkip; ++l)
          {
            std::cout << std::setw(8) << std::setprecision(4)
                      << A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << A[nSkip * j + i]
                  << std::endl;
        std::cout << "The difference: "
                  << std::abs(A[nSkip * i + j] - A[nSkip * j + i]) << "\n";
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
bool BoxedLcpConstraintSolver::isSymmetric(
    std::size_t n, double* A, std::size_t begin, std::size_t end)
{
  std::size_t nSkip = dPAD(n);
  for (std::size_t i = begin; i <= end; ++i)
  {
    for (std::size_t j = begin; j <= end; ++j)
    {
      if (std::abs(A[nSkip * i + j] - A[nSkip * j + i]) > 1e-6)
      {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < n; ++k)
        {
          for (std::size_t l = 0; l < nSkip; ++l)
          {
            std::cout << std::setprecision(4) << A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
void BoxedLcpConstraintSolver::print(
    std::size_t n,
    double* A,
    double* x,
    double* /*lo*/,
    double* /*hi*/,
    double* b,
    double* w,
    int* findex)
{
  std::size_t nSkip = dPAD(n);
  std::cout << "A: " << std::endl;
  for (std::size_t i = 0; i < n; ++i)
  {
    for (std::size_t j = 0; j < nSkip; ++j)
    {
      std::cout << std::setprecision(4) << A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "b: ";
  for (std::size_t i = 0; i < n; ++i)
  {
    std::cout << std::setprecision(4) << b[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "w: ";
  for (std::size_t i = 0; i < n; ++i)
  {
    std::cout << w[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "x: ";
  for (std::size_t i = 0; i < n; ++i)
  {
    std::cout << x[i] << " ";
  }
  std::cout << std::endl;

  //  std::cout << "lb: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    std::cout << lb[i] << " ";
  //  }
  //  std::cout << std::endl;

  //  std::cout << "ub: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    std::cout << ub[i] << " ";
  //  }
  //  std::cout << std::endl;

  std::cout << "frictionIndex: ";
  for (std::size_t i = 0; i < n; ++i)
  {
    std::cout << findex[i] << " ";
  }
  std::cout << std::endl;

  double* Ax = new double[n];

  for (std::size_t i = 0; i < n; ++i)
  {
    Ax[i] = 0.0;
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    for (std::size_t j = 0; j < n; ++j)
    {
      Ax[i] += A[i * nSkip + j] * x[j];
    }
  }

  std::cout << "Ax   : ";
  for (std::size_t i = 0; i < n; ++i)
  {
    std::cout << Ax[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "b + w: ";
  for (std::size_t i = 0; i < n; ++i)
  {
    std::cout << b[i] + w[i] << " ";
  }
  std::cout << std::endl;

  delete[] Ax;
}
#endif

} // namespace constraint
} // namespace dart
