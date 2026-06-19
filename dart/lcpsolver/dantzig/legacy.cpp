/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include "dart/lcpsolver/dantzig/error.h"
#include "dart/lcpsolver/dantzig/lcp.h"
#include "dart/lcpsolver/dantzig/matrix.h"
#include "dart/lcpsolver/dantzig/misc.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#undef dSetZero
#undef dSetValue
#undef dDot
#undef dMultiply0
#undef dMultiply1
#undef dMultiply2
#undef dFactorCholesky
#undef dSolveCholesky
#undef dInvertPDMatrix
#undef dIsPositiveDefinite
#undef dFactorLDLT
#undef dSolveL1
#undef dSolveL1T
#undef dVectorScale
#undef dSolveLDLT
#undef dLDLTAddTL
#undef dLDLTRemove
#undef dRemoveRowCol

namespace dart {
namespace external {
namespace ode {

namespace dantzig = ::dart::lcpsolver::dantzig;

using dMessageFunction = dantzig::dMessageFunction;
using dReal = dantzig::dReal;

namespace {

void printMessage(int num, const char* msg1, const char* msg2, va_list ap)
{
  std::fflush(stderr);
  std::fflush(stdout);
  if (num)
    std::fprintf(stderr, "\n%s %d: ", msg1, num);
  else
    std::fprintf(stderr, "\n%s: ", msg1);
  std::vfprintf(stderr, msg2, ap);
  std::fprintf(stderr, "\n");
  std::fflush(stderr);
}

} // namespace

void dSetErrorHandler(dMessageFunction* fn)
{
  dantzig::dSetErrorHandler(fn);
}

void dSetDebugHandler(dMessageFunction* fn)
{
  dantzig::dSetDebugHandler(fn);
}

void dSetMessageHandler(dMessageFunction* fn)
{
  dantzig::dSetMessageHandler(fn);
}

dMessageFunction* dGetErrorHandler()
{
  return dantzig::dGetErrorHandler();
}

dMessageFunction* dGetDebugHandler()
{
  return dantzig::dGetDebugHandler();
}

dMessageFunction* dGetMessageHandler()
{
  return dantzig::dGetMessageHandler();
}

void dError(int num, const char* msg, ...)
{
  va_list ap;
  va_start(ap, msg);
  if (dMessageFunction* handler = dantzig::dGetErrorHandler())
    handler(num, msg, ap);
  else
    printMessage(num, "ODE Error", msg, ap);
  va_end(ap);
  std::exit(1);
}

void dDebug(int num, const char* msg, ...)
{
  va_list ap;
  va_start(ap, msg);
  if (dMessageFunction* handler = dantzig::dGetDebugHandler())
    handler(num, msg, ap);
  else
    printMessage(num, "ODE INTERNAL ERROR", msg, ap);
  va_end(ap);
  std::abort();
}

void dMessage(int num, const char* msg, ...)
{
  va_list ap;
  va_start(ap, msg);
  if (dMessageFunction* handler = dantzig::dGetMessageHandler())
    handler(num, msg, ap);
  else
    printMessage(num, "ODE Message", msg, ap);
  va_end(ap);
}

void _dSetZero(dReal* a, size_t n)
{
  dantzig::_dSetZero(a, n);
}

void _dSetValue(dReal* a, size_t n, dReal value)
{
  dantzig::_dSetValue(a, n, value);
}

dReal _dDot(const dReal* a, const dReal* b, int n)
{
  return dantzig::_dDot(a, b, n);
}

void _dMultiply0(
    dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  dantzig::_dMultiply0(A, B, C, p, q, r);
}

void _dMultiply1(
    dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  dantzig::_dMultiply1(A, B, C, p, q, r);
}

void _dMultiply2(
    dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  dantzig::_dMultiply2(A, B, C, p, q, r);
}

int _dFactorCholesky(dReal* A, int n, void* tmpbuf)
{
  return dantzig::_dFactorCholesky(A, n, tmpbuf);
}

void _dSolveCholesky(const dReal* L, dReal* b, int n, void* tmpbuf)
{
  dantzig::_dSolveCholesky(L, b, n, tmpbuf);
}

int _dInvertPDMatrix(const dReal* A, dReal* Ainv, int n, void* tmpbuf)
{
  return dantzig::_dInvertPDMatrix(A, Ainv, n, tmpbuf);
}

int _dIsPositiveDefinite(const dReal* A, int n, void* tmpbuf)
{
  return dantzig::_dIsPositiveDefinite(A, n, tmpbuf);
}

void _dFactorLDLT(dReal* A, dReal* d, int n, int nskip)
{
  dantzig::_dFactorLDLT(A, d, n, nskip);
}

void _dSolveL1(const dReal* L, dReal* b, int n, int nskip)
{
  dantzig::_dSolveL1(L, b, n, nskip);
}

void _dSolveL1T(const dReal* L, dReal* b, int n, int nskip)
{
  dantzig::_dSolveL1T(L, b, n, nskip);
}

void _dVectorScale(dReal* a, const dReal* d, int n)
{
  dantzig::_dVectorScale(a, d, n);
}

void _dSolveLDLT(const dReal* L, const dReal* d, dReal* b, int n, int nskip)
{
  dantzig::_dSolveLDLT(L, d, b, n, nskip);
}

void _dLDLTAddTL(
    dReal* L, dReal* d, const dReal* a, int n, int nskip, void* tmpbuf)
{
  dantzig::_dLDLTAddTL(L, d, a, n, nskip, tmpbuf);
}

void _dLDLTRemove(
    dReal** A,
    const int* p,
    dReal* L,
    dReal* d,
    int n1,
    int n2,
    int r,
    int nskip,
    void* tmpbuf)
{
  dantzig::_dLDLTRemove(A, p, L, d, n1, n2, r, nskip, tmpbuf);
}

void _dRemoveRowCol(dReal* A, int n, int nskip, int r)
{
  dantzig::_dRemoveRowCol(A, n, nskip, r);
}

void dSetZero(dReal* a, int n)
{
  dantzig::dSetZero(a, n);
}

void dSetValue(dReal* a, int n, dReal value)
{
  dantzig::dSetValue(a, n, value);
}

dReal dDot(const dReal* a, const dReal* b, int n)
{
  return dantzig::dDot(a, b, n);
}

void dMultiply0(
    dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  dantzig::dMultiply0(A, B, C, p, q, r);
}

void dMultiply1(
    dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  dantzig::dMultiply1(A, B, C, p, q, r);
}

void dMultiply2(
    dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  dantzig::dMultiply2(A, B, C, p, q, r);
}

int dFactorCholesky(dReal* A, int n)
{
  return dantzig::dFactorCholesky(A, n);
}

void dSolveCholesky(const dReal* L, dReal* b, int n)
{
  dantzig::dSolveCholesky(L, b, n);
}

int dInvertPDMatrix(const dReal* A, dReal* Ainv, int n)
{
  return dantzig::dInvertPDMatrix(A, Ainv, n);
}

int dIsPositiveDefinite(const dReal* A, int n)
{
  return dantzig::dIsPositiveDefinite(A, n);
}

void dFactorLDLT(dReal* A, dReal* d, int n, int nskip)
{
  dantzig::dFactorLDLT(A, d, n, nskip);
}

void dSolveL1(const dReal* L, dReal* b, int n, int nskip)
{
  dantzig::dSolveL1(L, b, n, nskip);
}

void dSolveL1T(const dReal* L, dReal* b, int n, int nskip)
{
  dantzig::dSolveL1T(L, b, n, nskip);
}

void dVectorScale(dReal* a, const dReal* d, int n)
{
  dantzig::dVectorScale(a, d, n);
}

void dSolveLDLT(const dReal* L, const dReal* d, dReal* b, int n, int nskip)
{
  dantzig::dSolveLDLT(L, d, b, n, nskip);
}

void dLDLTAddTL(dReal* L, dReal* d, const dReal* a, int n, int nskip)
{
  dantzig::dLDLTAddTL(L, d, a, n, nskip);
}

void dLDLTRemove(
    dReal** A,
    const int* p,
    dReal* L,
    dReal* d,
    int n1,
    int n2,
    int r,
    int nskip)
{
  dantzig::dLDLTRemove(A, p, L, d, n1, n2, r, nskip);
}

void dRemoveRowCol(dReal* A, int n, int nskip, int r)
{
  dantzig::dRemoveRowCol(A, n, nskip, r);
}

int dTestRand()
{
  return dantzig::dTestRand();
}

unsigned long dRand()
{
  return dantzig::dRand();
}

unsigned long dRandGetSeed()
{
  return dantzig::dRandGetSeed();
}

void dRandSetSeed(unsigned long s)
{
  dantzig::dRandSetSeed(s);
}

int dRandInt(int n)
{
  return dantzig::dRandInt(n);
}

dReal dRandReal()
{
  return dantzig::dRandReal();
}

void dPrintMatrix(const dReal* A, int n, int m, char* fmt, FILE* f)
{
  dantzig::dPrintMatrix(A, n, m, fmt, f);
}

void dMakeRandomVector(dReal* A, int n, dReal range)
{
  dantzig::dMakeRandomVector(A, n, range);
}

void dMakeRandomMatrix(dReal* A, int n, int m, dReal range)
{
  dantzig::dMakeRandomMatrix(A, n, m, range);
}

void dClearUpperTriangle(dReal* A, int n)
{
  dantzig::dClearUpperTriangle(A, n);
}

dReal dMaxDifference(const dReal* A, const dReal* B, int n, int m)
{
  return dantzig::dMaxDifference(A, B, n, m);
}

dReal dMaxDifferenceLowerTriangle(const dReal* A, const dReal* B, int n)
{
  return dantzig::dMaxDifferenceLowerTriangle(A, B, n);
}

bool dSolveLCP(
    int n,
    dReal* A,
    dReal* x,
    dReal* b,
    dReal* w,
    int nub,
    dReal* lo,
    dReal* hi,
    int* findex,
    bool earlyTermination)
{
  return dantzig::dSolveLCP(
      n, A, x, b, w, nub, lo, hi, findex, earlyTermination);
}

size_t dEstimateSolveLCPMemoryReq(int n, bool outer_w_avail)
{
  return dantzig::dEstimateSolveLCPMemoryReq(n, outer_w_avail);
}

int dTestSolveLCP()
{
  return dantzig::dTestSolveLCP();
}

} // namespace ode
} // namespace external
} // namespace dart
