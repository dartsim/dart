/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/dynamics/PgsBoxedLcpSolver.hpp"

#include "dart/external/odelcpsolver/matrix.h"
#include "dart/external/odelcpsolver/misc.h"
#include "dart/math/Constants.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstring>

#define PGS_EPSILON 10e-9

namespace dart {
namespace dynamics {

//==============================================================================
PgsBoxedLcpSolver::Option::Option(
    int maxIteration,
    double deltaXTolerance,
    double relativeDeltaXTolerance,
    double epsilonForDivision,
    bool randomizeConstraintOrder)
  : mMaxIteration(maxIteration),
    mDeltaXThreshold(deltaXTolerance),
    mRelativeDeltaXTolerance(relativeDeltaXTolerance),
    mEpsilonForDivision(epsilonForDivision),
    mRandomizeConstraintOrder(randomizeConstraintOrder)
{
  // Do nothing
}

//==============================================================================
const std::string& PgsBoxedLcpSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& PgsBoxedLcpSolver::getStaticType()
{
  static const std::string type = "PgsBoxedLcpSolver";
  return type;
}

//==============================================================================
bool PgsBoxedLcpSolver::solve(
    int n,
    double* A,
    double* x,
    double* b,
    int nub,
    double* lo,
    double* hi,
    int* findex,
    bool /*earlyTermination*/)
{
  const int nskip = dPAD(n);

  // If all the variables are unbounded then we can just factor, solve, and
  // return.R
  if (nub >= n)
  {
    mCacheD.resize(n);
    std::fill(mCacheD.begin(), mCacheD.end(), 0);

    external::ode::dFactorLDLT(A, mCacheD.data(), n, nskip);
    external::ode::dSolveLDLT(A, mCacheD.data(), b, n, nskip);
    std::memcpy(x, b, n * sizeof(double));

    return true;
  }

  mCacheOrder.clear();
  mCacheOrder.reserve(n);

  bool possibleToTerminate = true;
  for (int i = 0; i < n; ++i)
  {
    // mOrderCacheing
    if (A[nskip * i + i] < mOption.mEpsilonForDivision)
    {
      x[i] = 0.0;
      continue;
    }

    mCacheOrder.push_back(i);

    // Initial loop
    const double* A_ptr = A + nskip * i;
    const double old_x = x[i];

    double new_x = b[i];

    for (int j = 0; j < i; ++j)
      new_x -= A_ptr[j] * x[j];

    for (int j = i + 1; j < n; ++j)
      new_x -= A_ptr[j] * x[j];

    new_x /= A[nskip * i + i];

    if (findex[i] >= 0)
    {
      const double hi_tmp = hi[i] * x[findex[i]];
      const double lo_tmp = -hi_tmp;

      if (new_x > hi_tmp)
        x[i] = hi_tmp;
      else if (new_x < lo_tmp)
        x[i] = lo_tmp;
      else
        x[i] = new_x;
    }
    else
    {
      if (new_x > hi[i])
        x[i] = hi[i];
      else if (new_x < lo[i])
        x[i] = lo[i];
      else
        x[i] = new_x;
    }

    // Test
    if (possibleToTerminate)
    {
      const double deltaX = std::abs(x[i] - old_x);
      if (deltaX > mOption.mDeltaXThreshold)
        possibleToTerminate = false;
    }
  }

  if (possibleToTerminate)
  {
    return true;
  }

  // Normalizing
  for (const auto& index : mCacheOrder)
  {
    const double dummy = 1.0 / A[nskip * index + index];
    b[index] *= dummy;
    for (int j = 0; j < n; ++j)
      A[nskip * index + j] *= dummy;
  }

  for (int iter = 1; iter < mOption.mMaxIteration; ++iter)
  {
    if (mOption.mRandomizeConstraintOrder)
    {
      if ((iter & 7) == 0)
      {
        for (std::size_t i = 1; i < mCacheOrder.size(); ++i)
        {
          const int tmp = mCacheOrder[i];
          const int swapi = external::ode::dRandInt(i + 1);
          mCacheOrder[i] = mCacheOrder[swapi];
          mCacheOrder[swapi] = tmp;
        }
      }
    }

    possibleToTerminate = true;

    // Single loop
    for (const auto& index : mCacheOrder)
    {
      const double* A_ptr = A + nskip * index;
      double new_x = b[index];
      const double old_x = x[index];

      for (int j = 0; j < index; j++)
        new_x -= A_ptr[j] * x[j];

      for (int j = index + 1; j < n; j++)
        new_x -= A_ptr[j] * x[j];

      if (findex[index] >= 0)
      {
        const double hi_tmp = hi[index] * x[findex[index]];
        const double lo_tmp = -hi_tmp;

        if (new_x > hi_tmp)
          x[index] = hi_tmp;
        else if (new_x < lo_tmp)
          x[index] = lo_tmp;
        else
          x[index] = new_x;
      }
      else
      {
        if (new_x > hi[index])
          x[index] = hi[index];
        else if (new_x < lo[index])
          x[index] = lo[index];
        else
          x[index] = new_x;
      }

      if (possibleToTerminate
          && std::abs(x[index]) > mOption.mEpsilonForDivision)
      {
        const double relativeDeltaX = std::abs((x[index] - old_x) / x[index]);
        if (relativeDeltaX > mOption.mRelativeDeltaXTolerance)
          possibleToTerminate = false;
      }
    }

    if (possibleToTerminate)
      break;
  }

  return possibleToTerminate;
}

#ifndef NDEBUG
//==============================================================================
bool PgsBoxedLcpSolver::canSolve(int n, const double* A)
{
  const int nskip = dPAD(n);

  // Return false if A has zero-diagonal or A is nonsymmetric matrix
  for (auto i = 0; i < n; ++i)
  {
    if (A[nskip * i + i] < PGS_EPSILON)
      return false;

    for (auto j = 0; j < n; ++j)
    {
      if (std::abs(A[nskip * i + j] - A[nskip * j + i]) > PGS_EPSILON)
        return false;
    }
  }

  return true;
}
#endif

//==============================================================================
void PgsBoxedLcpSolver::setOption(const PgsBoxedLcpSolver::Option& option)
{
  mOption = option;
}

//==============================================================================
const PgsBoxedLcpSolver::Option& PgsBoxedLcpSolver::getOption() const
{
  return mOption;
}

} // namespace dynamics
} // namespace dart
