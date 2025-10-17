/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/lcpsolver/dantzig/matrix.h"

namespace dart::lcpsolver {

int dFactorCholesky(dReal* A, int n, void* tmpbuf)
{
  return dFactorCholesky<dReal>(A, n, tmpbuf);
}

void dSolveCholesky(const dReal* L, dReal* b, int n, void* tmpbuf)
{
  dSolveCholesky<dReal>(L, b, n, tmpbuf);
}

void dLDLTAddTL(
    dReal* L, dReal* d, const dReal* a, int n, int nskip, void* tmpbuf)
{
  dLDLTAddTL<dReal>(L, d, a, n, nskip, tmpbuf);
}

void dRemoveRowCol(dReal* A, int n, int nskip, int r)
{
  dRemoveRowCol<dReal>(A, n, nskip, r);
}

void dFactorLDLT(dReal* A, dReal* d, int n, int nskip)
{
  dFactorLDLT<dReal>(A, d, n, nskip);
}

void dSolveLDLT(const dReal* L, const dReal* d, dReal* b, int n, int nskip)
{
  dSolveLDLT<dReal>(L, d, b, n, nskip);
}

void dSolveL1(const dReal* L, dReal* b, int n, int nskip)
{
  dSolveL1<dReal>(L, b, n, nskip);
}

void dSolveL1T(const dReal* L, dReal* b, int n, int nskip)
{
  dSolveL1T<dReal>(L, b, n, nskip);
}

void dLDLTRemove(
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
  dLDLTRemove<dReal>(A, p, L, d, n1, n2, r, nskip, tmpbuf);
}

} // namespace dart::lcpsolver
