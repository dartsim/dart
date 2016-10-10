/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/constraint/PGSLCPSolver.hpp"

#ifndef NDEBUG
#include <iomanip>
#include <iostream>
#endif

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/common/Console.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/lcpsolver/Lemke.hpp"

namespace dart {
namespace constraint {

//==============================================================================
PGSLCPSolver::PGSLCPSolver(double _timestep) : LCPSolver(_timestep)
{
}

//==============================================================================
PGSLCPSolver::~PGSLCPSolver()
{
}

//==============================================================================
void PGSLCPSolver::solve(ConstrainedGroup* _group)
{
  // If there is no constraint, then just return true.
  std::size_t numConstraints = _group->getNumConstraints();
  if (numConstraints == 0)
    return;

  // Build LCP terms by aggregating them from constraints
  std::size_t n = _group->getTotalDimension();
  int nSkip = dPAD(n);
  double* A = new double[n * nSkip];
  double* x = new double[n];
  double* b = new double[n];
  double* w = new double[n];
  double* lo = new double[n];
  double* hi = new double[n];
  int* findex = new int[n];

  // Set w to 0 and findex to -1
#ifndef NDEBUG
  std::memset(A, 0.0, n * nSkip * sizeof(double));
#endif
  std::memset(w, 0.0, n * sizeof(double));
  std::memset(findex, -1, n * sizeof(int));

  // Compute offset indices
  std::size_t* offset = new std::size_t[n];
  offset[0] = 0;
  //  std::cout << "offset[" << 0 << "]: " << offset[0] << std::endl;
  for (std::size_t i = 1; i < numConstraints; ++i)
  {
    const ConstraintBasePtr&  constraint = _group->getConstraint(i - 1);
    assert(constraint->getDimension() > 0);
    offset[i] = offset[i - 1] + constraint->getDimension();
    //    std::cout << "offset[" << i << "]: " << offset[i] << std::endl;
  }

  // For each constraint
  ConstraintInfo constInfo;
  constInfo.invTimeStep = 1.0 / mTimeStep;
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = _group->getConstraint(i);

    constInfo.x      = x      + offset[i];
    constInfo.lo     = lo     + offset[i];
    constInfo.hi     = hi     + offset[i];
    constInfo.b      = b      + offset[i];
    constInfo.findex = findex + offset[i];
    constInfo.w      = w      + offset[i];

    // Fill vectors: lo, hi, b, w
    constraint->getInformation(&constInfo);

    // Fill a matrix by impulse tests: A
    constraint->excite();
    for (std::size_t j = 0; j < constraint->getDimension(); ++j)
    {
      // Adjust findex for global index
      if (findex[offset[i] + j] >= 0)
        findex[offset[i] + j] += offset[i];

      // Apply impulse for mipulse test
      constraint->applyUnitImpulse(j);

      // Fill upper triangle blocks of A matrix
      int index = nSkip * (offset[i] + j) + offset[i];
      constraint->getVelocityChange(A + index, true);
      for (std::size_t k = i + 1; k < numConstraints; ++k)
      {
        index = nSkip * (offset[i] + j) + offset[k];
        _group->getConstraint(k)->getVelocityChange(A + index, false);
      }

      // Filling symmetric part of A matrix
      for (std::size_t k = 0; k < i; ++k)
      {
        for (std::size_t l = 0; l < _group->getConstraint(k)->getDimension(); ++l)
        {
          int index1 = nSkip * (offset[i] + j) + offset[k] + l;
          int index2 = nSkip * (offset[k] + l) + offset[i] + j;

          A[index1] = A[index2];
        }
      }
    }

    assert(isSymmetric(n, A, offset[i],
                       offset[i] + constraint->getDimension() - 1));

    constraint->unexcite();
  }

  assert(isSymmetric(n, A));

  // Print LCP formulation
  //  dtdbg << "Before solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Solve LCP using ODE's Dantzig algorithm
//  dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);
  PGSOption option;
  option.setDefault();
  solvePGS(n, nSkip, 0, A, x, b, lo, hi, findex, &option);

  // Print LCP formulation
  //  dtdbg << "After solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Apply constraint impulses
  for (std::size_t i = 0; i < numConstraints; ++i)
  {
    const ConstraintBasePtr& constraint = _group->getConstraint(i);
    constraint->applyImpulse(x + offset[i]);
    constraint->excite();
  }

  delete[] offset;

  delete[] A;
  delete[] x;
  delete[] b;
  delete[] w;
  delete[] lo;
  delete[] hi;
  delete[] findex;
}

//==============================================================================
#ifndef NDEBUG
bool PGSLCPSolver::isSymmetric(std::size_t _n, double* _A)
{
  std::size_t nSkip = dPAD(_n);
  for (std::size_t i = 0; i < _n; ++i)
  {
    for (std::size_t j = 0; j < _n; ++j)
    {
      if (std::abs(_A[nSkip * i + j] - _A[nSkip * j + i]) > 1e-6)
      {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < _n; ++k)
        {
          for (std::size_t l = 0; l < nSkip; ++l)
          {
            std::cout << std::setprecision(4) << _A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << _A[nSkip * i + j] << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << _A[nSkip * j + i] << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
bool PGSLCPSolver::isSymmetric(std::size_t _n, double* _A,
                                   std::size_t _begin, std::size_t _end)
{
  std::size_t nSkip = dPAD(_n);
  for (std::size_t i = _begin; i <= _end; ++i)
  {
    for (std::size_t j = _begin; j <= _end; ++j)
    {
      if (std::abs(_A[nSkip * i + j] - _A[nSkip * j + i]) > 1e-6)
      {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < _n; ++k)
        {
          for (std::size_t l = 0; l < nSkip; ++l)
          {
            std::cout << std::setprecision(4) << _A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << _A[nSkip * i + j] << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << _A[nSkip * j + i] << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
void PGSLCPSolver::print(std::size_t _n, double* _A, double* _x,
                         double* /*lo*/, double* /*hi*/, double* b,
                         double* w, int* findex)
{
  std::size_t nSkip = dPAD(_n);
  std::cout << "A: " << std::endl;
  for (std::size_t i = 0; i < _n; ++i)
  {
    for (std::size_t j = 0; j < nSkip; ++j)
    {
      std::cout << std::setprecision(4) << _A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "b: ";
  for (std::size_t i = 0; i < _n; ++i)
  {
    std::cout << std::setprecision(4) << b[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "w: ";
  for (std::size_t i = 0; i < _n; ++i)
  {
    std::cout << w[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "x: ";
  for (std::size_t i = 0; i < _n; ++i)
  {
    std::cout << _x[i] << " ";
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
  for (std::size_t i = 0; i < _n; ++i)
  {
    std::cout << findex[i] << " ";
  }
  std::cout << std::endl;

  double* Ax  = new double[_n];

  for (std::size_t i = 0; i < _n; ++i)
  {
    Ax[i] = 0.0;
  }

  for (std::size_t i = 0; i < _n; ++i)
  {
    for (std::size_t j = 0; j < _n; ++j)
    {
      Ax[i] += _A[i * nSkip + j] * _x[j];
    }
  }

  std::cout << "Ax   : ";
  for (std::size_t i = 0; i < _n; ++i)
  {
    std::cout << Ax[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "b + w: ";
  for (std::size_t i = 0; i < _n; ++i)
  {
    std::cout << b[i] + w[i] << " ";
  }
  std::cout << std::endl;

  delete[] Ax;
}
#endif

bool solvePGS(int n, int nskip, int /*nub*/, double * A, double * x, double * b,
              double * lo, double * hi, int * findex, PGSOption * option)
{
  // LDLT solver will work !!!
  //if (nub == n)
  //{
  //	return LDLTSolver(n,nskip,A,x,b)
  //}

  int i, j, iter, idx, n_new;
  bool sentinel;
  double old_x, new_x, hi_tmp, lo_tmp, dummy, ea;
  double * A_ptr;
  double one_minus_sor_w = 1.0 - (option->sor_w);

  //--- ORDERING & SCALING & INITIAL LOOP & Test
  int* order = new int[n];

  n_new = 0;
  sentinel = true;
  for (i = 0 ; i < n ; i++)
  {
    // ORDERING
    if ( A[nskip*i + i] < option->eps_div )
    {
      x[i] = 0.0;
      continue;
    }
    order[n_new++] = i;

    // INITIAL LOOP
    A_ptr = A + nskip*i;
    new_x = b[i];
    old_x = x[i];

    for (j = 0 ; j < i ; j++)
      new_x -= A_ptr[j]*x[j];
    for (j = i + 1 ; j < n ; j++)
      new_x -= A_ptr[j]*x[j];

    new_x = new_x/A[nskip*i + i];

    if (findex[i] >= 0)	// friction index
    {
      hi_tmp = hi[i] * x[findex[i]];
      lo_tmp = -hi_tmp;

      if (new_x > hi_tmp)
        x[i] = hi_tmp;
      else if (new_x < lo_tmp)
        x[i] = lo_tmp;
      else
        x[i] = new_x;
    }
    else					// no friction index
    {
      if (new_x > hi[i])
        x[i] = hi[i];
      else if (new_x < lo[i])
        x[i] = lo[i];
      else
        x[i] = new_x;
    }

    // TEST
    if (sentinel)
    {
      ea = std::abs(x[i] - old_x);
      if (ea > option->eps_res)
        sentinel = false;
    }
  }
  if (sentinel)
  {
    delete[] order;
    return true;
  }

  // SCALING
  for (i = 0 ; i < n_new ; i++)
  {
    idx = order[i];

    dummy = 1.0/A[nskip*idx + idx];  // diagonal element
    b[idx] *= dummy;
    for (j = 0 ; j < n ; j++)
      A[nskip*idx + j] *= dummy;
  }

  //--- ITERATION LOOP
  for (iter = 1 ; iter < option->itermax ; iter++)
  {
    //--- RANDOMLY_REORDER_CONSTRAINTS
#if LCP_PGS_RANDOMLY_REORDER_CONSTRAINTS
    if ((iter & 7)==0)
    {
      int tmp, swapi;
      for (i = 1 ; i < n_new ; i++)
      {
        tmp = order[i];
        swapi = dRandInt(i+1);
        order[i] = order[swapi];
        order[swapi] = tmp;
      }
    }
#endif

    sentinel = true;

    //-- ONE LOOP
    for (i = 0 ; i < n_new ; i++)
    {
      idx = order[i];

      A_ptr = A + nskip*idx;
      new_x = b[idx];
      old_x = x[idx];

      for (j = 0 ; j < idx ; j++)
        new_x -= A_ptr[j]*x[j];
      for (j = idx + 1 ; j < n ; j++)
        new_x -= A_ptr[j]*x[j];

      new_x = (option->sor_w * new_x) + (one_minus_sor_w * old_x);

      if (findex[idx] >= 0)	// friction index
      {
        hi_tmp = hi[idx] * x[findex[idx]];
        lo_tmp = -hi_tmp;

        if (new_x > hi_tmp)
          x[idx] = hi_tmp;
        else if (new_x < lo_tmp)
          x[idx] = lo_tmp;
        else
          x[idx] = new_x;
      }
      else					// no friction index
      {
        if (new_x > hi[idx])
          x[idx] = hi[idx];
        else if (new_x < lo[idx])
          x[idx] = lo[idx];
        else
          x[idx] = new_x;
      }

      if ( sentinel && std::abs(x[idx]) > option->eps_div)
      {
        ea = std::abs((x[idx] - old_x)/x[idx]);
        if (ea > option->eps_ea)
          sentinel = false;
      }
    }

    if (sentinel)
      break;
  }
  delete[] order;
  return sentinel;
}

#define LCP_PGS_RANDOMLY_REORDER_CONSTRAINTS		1
#define LCP_PGS_OPTION_DEFAULT_ITERMAX				30
#define LCP_PGS_OPTION_DEFAULT_SOR_W				0.9
#define LCP_PGS_OPTION_DEFAULT_EPS_EA				1E-3
#define LCP_PGS_OPTION_DEFAULT_EPS_RESIDUAL			1E-6
#define LCP_PGS_OPTION_DEFAULT_EPS_DIVIDE			1E-9

void PGSOption::setDefault()
{
  itermax = LCP_PGS_OPTION_DEFAULT_ITERMAX;
  sor_w = LCP_PGS_OPTION_DEFAULT_SOR_W;
  eps_ea = LCP_PGS_OPTION_DEFAULT_EPS_EA;
  eps_res = LCP_PGS_OPTION_DEFAULT_EPS_RESIDUAL;
  eps_div = LCP_PGS_OPTION_DEFAULT_EPS_DIVIDE;
}

}  // namespace constraint
}  // namespace dart
