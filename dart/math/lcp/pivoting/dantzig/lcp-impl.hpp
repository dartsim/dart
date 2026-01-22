/*
 * Copyright (c) 2011, The DART development contributors
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

/*
 * This file contains code derived from Open Dynamics Engine (ODE).
 * Original copyright notice:
 *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file LICENSE-BSD.TXT.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.
 */

#ifndef DART_MATH_LCP_DANTZIG_LCP_IMPL_HPP_
#define DART_MATH_LCP_DANTZIG_LCP_IMPL_HPP_

#include "dart/common/macros.hpp"
#include "dart/math/lcp/pivoting/dantzig/matrix.hpp"
#include "dart/math/lcp/pivoting/dantzig/misc.hpp"
#include "dart/math/lcp/pivoting/dantzig/pivot_matrix.hpp"

#define ROWPTRS // Keep for compatibility (but now uses PivotMatrix internally)
#define AROW(i) (m_A[i])

#define NUB_OPTIMIZATIONS

namespace dart::math {

//***************************************************************************
// Template implementations
//***************************************************************************

// swap row/column i1 with i2 in the n*n matrix A. the leading dimension of
// A is nskip. this only references and swaps the lower triangle.
// if `do_fast_row_swaps' is nonzero and row pointers are being used, then
// rows will be swapped by exchanging row pointers. otherwise the data will
// be copied.

template <typename Scalar>
static void swapRowsAndCols(
    PivotMatrix<Scalar>& A,
    int n,
    int i1,
    int i2,
    int nskip,
    int do_fast_row_swaps)
{
  DART_ASSERT(
      n > 0 && i1 >= 0 && i2 >= 0 && i1 < n && i2 < n && nskip >= n && i1 < i2);

#ifdef ROWPTRS
  DART_UNUSED(nskip);
  Scalar* A_i1 = A[i1];
  Scalar* A_i2 = A[i2];
  for (int i = i1 + 1; i < i2; ++i) {
    Scalar* A_i_i1 = A[i] + i1;
    A_i1[i] = *A_i_i1;
    *A_i_i1 = A_i2[i];
  }
  A_i1[i2] = A_i1[i1];
  A_i1[i1] = A_i2[i1];
  A_i2[i1] = A_i2[i2];
  // swap rows using PivotMatrix's O(1) pointer swapping
  if (do_fast_row_swaps) {
    A.swapRows(i1, i2);
  } else {
    // Only swap till i2 column to match A plain storage variant.
    for (int k = 0; k <= i2; ++k) {
      Scalar tmp = A_i1[k];
      A_i1[k] = A_i2[k];
      A_i2[k] = tmp;
    }
  }
  // swap columns the hard way
  for (int j = i2 + 1; j < n; ++j) {
    Scalar* A_j = A[j];
    Scalar tmp = A_j[i1];
    A_j[i1] = A_j[i2];
    A_j[i2] = tmp;
  }
#else
  Scalar* A_i1 = A + i1 * nskip;
  Scalar* A_i2 = A + i2 * nskip;
  for (int k = 0; k < i1; ++k) {
    Scalar tmp = A_i1[k];
    A_i1[k] = A_i2[k];
    A_i2[k] = tmp;
  }
  Scalar* A_i = A_i1 + nskip;
  for (int i = i1 + 1; i < i2; A_i += nskip, ++i) {
    Scalar tmp = A_i2[i];
    A_i2[i] = A_i[i1];
    A_i[i1] = tmp;
  }
  {
    Scalar tmp = A_i1[i1];
    A_i1[i1] = A_i2[i2];
    A_i2[i2] = tmp;
  }
  Scalar* A_j = A_i2 + nskip;
  for (int j = i2 + 1; j < n; A_j += nskip, ++j) {
    Scalar tmp = A_j[i1];
    A_j[i1] = A_j[i2];
    A_j[i2] = tmp;
  }
#endif
}

// swap two indexes in the n*n LCP problem. i1 must be <= i2.

template <typename Scalar>
static void swapProblem(
    PivotMatrix<Scalar>& A,
    Scalar* x,
    Scalar* b,
    Scalar* w,
    Scalar* lo,
    Scalar* hi,
    int* p,
    bool* state,
    int* findex,
    int n,
    int i1,
    int i2,
    int nskip,
    int do_fast_row_swaps)
{
  Scalar tmpr;
  int tmpi;
  bool tmpb;
  DART_ASSERT(
      n > 0 && i1 >= 0 && i2 >= 0 && i1 < n && i2 < n && nskip >= n
      && i1 <= i2);
  if (i1 == i2) {
    return;
  }

  swapRowsAndCols(A, n, i1, i2, nskip, do_fast_row_swaps);

  tmpr = x[i1];
  x[i1] = x[i2];
  x[i2] = tmpr;

  tmpr = b[i1];
  b[i1] = b[i2];
  b[i2] = tmpr;

  tmpr = w[i1];
  w[i1] = w[i2];
  w[i2] = tmpr;

  tmpr = lo[i1];
  lo[i1] = lo[i2];
  lo[i2] = tmpr;

  tmpr = hi[i1];
  hi[i1] = hi[i2];
  hi[i2] = tmpr;

  tmpi = p[i1];
  p[i1] = p[i2];
  p[i2] = tmpi;

  tmpb = state[i1];
  state[i1] = state[i2];
  state[i2] = tmpb;

  if (findex) {
    tmpi = findex[i1];
    findex[i1] = findex[i2];
    findex[i2] = tmpi;
  }
}

// for debugging - check that L,d is the factorization of A[C,C].
// A[C,C] has size nC*nC and leading dimension nskip.
// L has size nC*nC and leading dimension nskip.
// d has size nC.

#ifdef DEBUG_LCP

template <typename Scalar>
static void checkFactorization(
    PivotMatrix<Scalar>& A, Scalar* _L, Scalar* _d, int nC, int* C, int nskip)
{
  if (nC == 0) {
    return;
  }

  // get A1=A, copy the lower triangle to the upper triangle, get A2=A[C,C]
  dMatrix A1(nC, nC);
  for (int i = 0; i < nC; ++i) {
    for (int j = 0; j <= i; ++j) {
      A1(i, j) = A1(j, i) = A[i][j];
    }
  }
  dMatrix A2 = A1.select(nC, C, nC, C);

  // printf ("A1=\n"); A1.print(); printf ("\n");
  // printf ("A2=\n"); A2.print(); printf ("\n");

  // compute A3 = L*D*L'
  dMatrix L(nC, nC, _L, nskip, 1);
  dMatrix D(nC, nC);
  for (int i = 0; i < nC; ++i) {
    D(i, i) = 1 / _d[i];
  }
  L.clearUpperTriangle();
  for (int i = 0; i < nC; ++i) {
    L(i, i) = 1;
  }
  dMatrix A3 = L * D * L.transpose();

  // printf ("L=\n"); L.print(); printf ("\n");
  // printf ("D=\n"); D.print(); printf ("\n");
  // printf ("A3=\n"); A2.print(); printf ("\n");

  // compare A2 and A3
  Scalar diff = A2.maxDifference(A3);
  DART_DEBUG_IF(diff > 1e-8, "L*D*L' check, maximum difference = {:.6e}", diff);
}

#endif

// for debugging

#ifdef DEBUG_LCP

static void checkPermutations(int i, int n, int nC, int nN, int* p, int* C)
{
  int j, k;
  DART_ASSERT(nC >= 0 && nN >= 0 && (nC + nN) == i && i < n);
  for (k = 0; k < i; k++) {
    DART_ASSERT(p[k] >= 0 && p[k] < i);
  }
  for (k = i; k < n; k++) {
    DART_ASSERT(p[k] == k);
  }
  for (j = 0; j < nC; j++) {
    int C_is_bad = 1;
    for (k = 0; k < nC; k++) {
      if (C[k] == j) {
        C_is_bad = 0;
      }
    }
    DART_ASSERT(C_is_bad == 0);
  }
}

#endif

//***************************************************************************
// dLCP manipulator object. this represents an n*n LCP problem.
//
// two index sets C and N are kept. each set holds a subset of
// the variable indexes 0..n-1. an index can only be in one set.
// initially both sets are empty.
//
// the index set C is special: solutions to A(C,C)\A(C,i) can be generated.

//***************************************************************************
// fast implementation of dLCP. see the above definition of dLCP for
// interface comments.
//
// `p' records the permutation of A,x,b,w,etc. p is initially 1:n and is
// permuted as the other vectors/matrices are permuted.
//
// A,x,b,w,lo,hi,state,findex,p,c are permuted such that sets C,N have
// contiguous indexes. the don't-care indexes follow N.
//
// an L*D*L' factorization is maintained of A(C,C), and whenever indexes are
// added or removed from the set C the factorization is updated.
// thus L*D*L'=A[C,C], i.e. a permuted top left nC*nC submatrix of A.
// the leading dimension of the matrix L is always `nskip'.
//
// at the start there may be other indexes that are unbounded but are not
// included in `nub'. dLCP will permute the matrix so that absolutely all
// unbounded vectors are at the start. thus there may be some initial
// permutation.
//
// the algorithms here assume certain patterns, particularly with respect to
// index transfer.

#ifdef dLCP_FAST

// Phase 13: Fully templated LCP class
template <typename Scalar>
struct LCP
{
  const int m_n;
  const int m_nskip;
  int m_nub;
  int m_nC, m_nN;           // size of each index set
  PivotMatrix<Scalar>& m_A; // A rows (PivotMatrix reference)
  Scalar *const m_x,
      *const m_b, *const m_w, *const m_lo,
                                  *const m_hi; // permuted LCP problem data
  Scalar *const m_L, *const m_d;               // L*D*L' factorization of set C
  Scalar *const m_Dell, *const m_ell, *const m_tmp;
  bool* const m_state;
  int *const m_findex, *const m_p, *const m_C;

  LCP(int _n,
      int _nskip,
      int _nub,
      PivotMatrix<Scalar>& _A,
      Scalar* _x,
      Scalar* _b,
      Scalar* _w,
      Scalar* _lo,
      Scalar* _hi,
      Scalar* _L,
      Scalar* _d,
      Scalar* _Dell,
      Scalar* _ell,
      Scalar* _tmp,
      bool* _state,
      int* _findex,
      int* _p,
      int* _C);
  int getNub() const
  {
    return m_nub;
  }
  void transfer_i_to_C(int i);
  void transfer_i_to_N(int /*i*/)
  {
    m_nN++;
  } // because we can assume C and N span 1:i-1
  void transfer_i_from_N_to_C(int i);
  void transfer_i_from_C_to_N(int i, void* tmpbuf);
  static size_t estimate_transfer_i_from_C_to_N_mem_req(int nC, int nskip)
  {
    return dEstimateLDLTRemoveTmpbufSize<Scalar>(nC, nskip);
  }
  int numC() const
  {
    return m_nC;
  }
  int numN() const
  {
    return m_nN;
  }
  int indexC(int i) const
  {
    return i;
  }
  int indexN(int i) const
  {
    return i + m_nC;
  }
  Scalar Aii(int i) const
  {
    return m_A[i][i];
  }
  Scalar AiC_times_qC(int i, Scalar* q) const
  {
    return Dot(m_A[i], q, m_nC);
  }
  Scalar AiN_times_qN(int i, Scalar* q) const
  {
    return Dot(m_A[i] + m_nC, q + m_nC, m_nN);
  }
  void pN_equals_ANC_times_qC(Scalar* p, Scalar* q);
  void pN_plusequals_ANi(Scalar* p, int i, int sign = 1);
  void pC_plusequals_s_times_qC(Scalar* p, Scalar s, Scalar* q);
  void pN_plusequals_s_times_qN(Scalar* p, Scalar s, Scalar* q);
  void solve1(Scalar* a, int i, int dir = 1, int only_transfer = 0);
  void unpermute();
};

// No type alias needed - use LCP<Scalar> directly

template <typename Scalar>
LCP<Scalar>::LCP(
    int _n,
    int _nskip,
    int _nub,
    PivotMatrix<Scalar>& _A,
    Scalar* _x,
    Scalar* _b,
    Scalar* _w,
    Scalar* _lo,
    Scalar* _hi,
    Scalar* _L,
    Scalar* _d,
    Scalar* _Dell,
    Scalar* _ell,
    Scalar* _tmp,
    bool* _state,
    int* _findex,
    int* _p,
    int* _C)
  : m_n(_n),
    m_nskip(_nskip),
    m_nub(_nub),
    m_nC(0),
    m_nN(0),
    m_A(_A),
    m_x(_x),
    m_b(_b),
    m_w(_w),
    m_lo(_lo),
    m_hi(_hi),
    m_L(_L),
    m_d(_d),
    m_Dell(_Dell),
    m_ell(_ell),
    m_tmp(_tmp),
    m_state(_state),
    m_findex(_findex),
    m_p(_p),
    m_C(_C)
{
  {
    SetZero(m_x, m_n);
  }

  {
    int* p = m_p;
    const int n = m_n;
    for (int k = 0; k < n; ++k) {
      p[k] = k; // initially unpermuted
    }
  }

  /*
  // for testing, we can do some random swaps in the area i > nub
  {
    const int n = m_n;
    const int nub = m_nub;
    if (nub < n) {
    for (int k=0; k<100; k++) {
      int i1,i2;
      do {
        i1 = Random::randInt(n-nub)+nub;
        i2 = Random::randInt(n-nub)+nub;
      }
      while (i1 > i2);
      //printf ("--> %d %d\n",i1,i2);
      swapProblem
  (m_A,m_x,m_b,m_w,m_lo,m_hi,m_p,m_state,m_findex,n,i1,i2,m_nskip,0);
    }
  }
  */

  // permute the problem so that *all* the unbounded variables are at the
  // start, i.e. look for unbounded variables not included in `nub'. we can
  // potentially push up `nub' this way and get a bigger initial factorization.
  // note that when we swap rows/cols here we must not just swap row pointers,
  // as the initial factorization relies on the data being all in one chunk.
  // variables that have findex >= 0 are *not* considered to be unbounded even
  // if lo=-inf and hi=inf - this is because these limits may change during the
  // solution process.

  {
    int* findex = m_findex;
    Scalar *lo = m_lo, *hi = m_hi;
    const int n = m_n;
    const Scalar inf = ScalarTraits<Scalar>::inf();
    for (int k = m_nub; k < n; ++k) {
      if (findex && findex[k] >= 0) {
        continue;
      }
      if (lo[k] == -inf && hi[k] == inf) {
        swapProblem(
            m_A,
            m_x,
            m_b,
            m_w,
            lo,
            hi,
            m_p,
            m_state,
            findex,
            n,
            m_nub,
            k,
            m_nskip,
            0);
        m_nub++;
      }
    }
  }

  // if there are unbounded variables at the start, factorize A up to that
  // point and solve for x. this puts all indexes 0..nub-1 into C.
  if (m_nub > 0) {
    const int nub = m_nub;
    {
      Scalar* Lrow = m_L;
      const int nskip = m_nskip;
      for (int j = 0; j < nub; Lrow += nskip, ++j) {
        memcpy(Lrow, AROW(j), (j + 1) * sizeof(Scalar));
      }
    }
    dFactorLDLT(m_L, m_d, nub, m_nskip);
    memcpy(m_x, m_b, nub * sizeof(Scalar));
    dSolveLDLT(m_L, m_d, m_x, nub, m_nskip);
    SetZero(m_w, nub);
    {
      int* C = m_C;
      for (int k = 0; k < nub; ++k) {
        C[k] = k;
      }
    }
    m_nC = nub;
  }

  // permute the indexes > nub such that all findex variables are at the end
  if (m_findex) {
    const int nub = m_nub;
    int* findex = m_findex;
    int num_at_end = 0;
    for (int k = m_n - 1; k >= nub; k--) {
      if (findex[k] >= 0) {
        swapProblem(
            m_A,
            m_x,
            m_b,
            m_w,
            m_lo,
            m_hi,
            m_p,
            m_state,
            findex,
            m_n,
            k,
            m_n - 1 - num_at_end,
            m_nskip,
            1);
        num_at_end++;
      }
    }
  }

  // print info about indexes
  /*
  {
    const int n = m_n;
    const int nub = m_nub;
    for (int k=0; k<n; k++) {
      if (k<nub) printf ("C");
      else if (m_lo[k]==-inf && m_hi[k]==inf) printf ("c");
      else printf (".");
    }
    printf ("\n");
  }
  */
}

template <typename Scalar>
void LCP<Scalar>::transfer_i_to_C(int i)
{
  if (m_nC > 0) {
    // ell,Dell were computed by solve1(). note, ell = D \ L1solve (L,A(i,C))
    {
      const int nC = m_nC;
      Scalar *const Ltgt = m_L + nC * m_nskip, *ell = m_ell;
      for (int j = 0; j < nC; ++j) {
        Ltgt[j] = ell[j];
      }
    }
    const int nC = m_nC;
    m_d[nC] = reciprocal(AROW(i)[i] - Dot(m_ell, m_Dell, nC));
  } else {
    m_d[0] = reciprocal(AROW(i)[i]);
  }

  swapProblem(
      m_A,
      m_x,
      m_b,
      m_w,
      m_lo,
      m_hi,
      m_p,
      m_state,
      m_findex,
      m_n,
      m_nC,
      i,
      m_nskip,
      1);

  const int nC = m_nC;
  m_C[nC] = nC;
  m_nC = nC + 1; // nC value is outdated after this line

  #ifdef DEBUG_LCP
  checkFactorization(m_A, m_L, m_d, m_nC, m_C, m_nskip);
  if (i < (m_n - 1)) {
    checkPermutations(i + 1, m_n, m_nC, m_nN, m_p, m_C);
  }
  #endif
}

template <typename Scalar>
void LCP<Scalar>::transfer_i_from_N_to_C(int i)
{
  {
    if (m_nC > 0) {
      {
        Scalar* const aptr = AROW(i);
        Scalar* Dell = m_Dell;
        const int* C = m_C;
  #ifdef NUB_OPTIMIZATIONS
        // if nub>0, initial part of aptr unpermuted
        const int nub = m_nub;
        int j = 0;
        for (; j < nub; ++j) {
          Dell[j] = aptr[j];
        }
        const int nC = m_nC;
        for (; j < nC; ++j) {
          Dell[j] = aptr[C[j]];
        }
  #else
        const int nC = m_nC;
        for (int j = 0; j < nC; ++j) {
          Dell[j] = aptr[C[j]];
        }
  #endif
      }
      dSolveL1(m_L, m_Dell, m_nC, m_nskip);
      {
        const int nC = m_nC;
        Scalar* const Ltgt = m_L + nC * m_nskip;
        Scalar *ell = m_ell, *Dell = m_Dell, *d = m_d;
        for (int j = 0; j < nC; ++j) {
          Ltgt[j] = ell[j] = Dell[j] * d[j];
        }
      }
      const int nC = m_nC;
      m_d[nC] = reciprocal(AROW(i)[i] - Dot(m_ell, m_Dell, nC));
    } else {
      m_d[0] = reciprocal(AROW(i)[i]);
    }

    swapProblem(
        m_A,
        m_x,
        m_b,
        m_w,
        m_lo,
        m_hi,
        m_p,
        m_state,
        m_findex,
        m_n,
        m_nC,
        i,
        m_nskip,
        1);

    const int nC = m_nC;
    m_C[nC] = nC;
    m_nN--;
    m_nC = nC + 1; // nC value is outdated after this line
  }

    // @@@ TO DO LATER
    // if we just finish here then we'll go back and re-solve for
    // delta_x. but actually we can be more efficient and incrementally
    // update delta_x here. but if we do this, we won't have ell and Dell
    // to use in updating the factorization later.

  #ifdef DEBUG_LCP
  checkFactorization(m_A, m_L, m_d, m_nC, m_C, m_nskip);
  #endif
}

template <typename Scalar>
void LCP<Scalar>::transfer_i_from_C_to_N(int i, void* tmpbuf)
{
  {
    int* C = m_C;
    // remove a row/column from the factorization, and adjust the
    // indexes (black magic!)
    int last_idx = -1;
    const int nC = m_nC;
    int j = 0;
    for (; j < nC; ++j) {
      if (C[j] == nC - 1) {
        last_idx = j;
      }
      if (C[j] == i) {
        dLDLTRemove(
            m_A.rowPointers(), C, m_L, m_d, m_n, nC, j, m_nskip, tmpbuf);
        int k;
        if (last_idx == -1) {
          for (k = j + 1; k < nC; ++k) {
            if (C[k] == nC - 1) {
              break;
            }
          }
          DART_ASSERT(k < nC);
        } else {
          k = last_idx;
        }
        C[k] = C[j];
        if (j < (nC - 1)) {
          memmove(C + j, C + j + 1, (nC - j - 1) * sizeof(int));
        }
        break;
      }
    }
    DART_ASSERT(j < nC);

    swapProblem(
        m_A,
        m_x,
        m_b,
        m_w,
        m_lo,
        m_hi,
        m_p,
        m_state,
        m_findex,
        m_n,
        i,
        nC - 1,
        m_nskip,
        1);

    m_nN++;
    m_nC = nC - 1; // nC value is outdated after this line
  }

  #ifdef DEBUG_LCP
  checkFactorization(m_A, m_L, m_d, m_nC, m_C, m_nskip);
  #endif
}

template <typename Scalar>
void LCP<Scalar>::pN_equals_ANC_times_qC(Scalar* p, Scalar* q)
{
  // we could try to make this matrix-vector multiplication faster using
  // outer product matrix tricks, e.g. with the dMultidotX() functions.
  // but i tried it and it actually made things slower on random 100x100
  // problems because of the overhead involved. so we'll stick with the
  // simple method for now.
  const int nC = m_nC;
  Scalar* ptgt = p + nC;
  const int nN = m_nN;
  for (int i = 0; i < nN; ++i) {
    ptgt[i] = Dot(AROW(i + nC), q, nC);
  }
}

template <typename Scalar>
void LCP<Scalar>::pN_plusequals_ANi(Scalar* p, int i, int sign)
{
  const int nC = m_nC;
  Scalar* aptr = AROW(i) + nC;
  Scalar* ptgt = p + nC;
  if (sign > 0) {
    const int nN = m_nN;
    for (int j = 0; j < nN; ++j) {
      ptgt[j] += aptr[j];
    }
  } else {
    const int nN = m_nN;
    for (int j = 0; j < nN; ++j) {
      ptgt[j] -= aptr[j];
    }
  }
}

template <typename Scalar>
void LCP<Scalar>::pC_plusequals_s_times_qC(Scalar* p, Scalar s, Scalar* q)
{
  const int nC = m_nC;
  for (int i = 0; i < nC; ++i) {
    p[i] += s * q[i];
  }
}

template <typename Scalar>
void LCP<Scalar>::pN_plusequals_s_times_qN(Scalar* p, Scalar s, Scalar* q)
{
  const int nC = m_nC;
  Scalar *ptgt = p + nC, *qsrc = q + nC;
  const int nN = m_nN;
  for (int i = 0; i < nN; ++i) {
    ptgt[i] += s * qsrc[i];
  }
}

template <typename Scalar>
void LCP<Scalar>::solve1(Scalar* a, int i, int dir, int only_transfer)
{
  // the `Dell' and `ell' that are computed here are saved. if index i is
  // later added to the factorization then they can be reused.
  //
  // @@@ question: do we need to solve for entire delta_x??? yes, but
  //     only if an x goes below 0 during the step.

  if (m_nC > 0) {
    {
      Scalar* Dell = m_Dell;
      int* C = m_C;
      Scalar* aptr = AROW(i);
  #ifdef NUB_OPTIMIZATIONS
      // if nub>0, initial part of aptr[] is guaranteed unpermuted
      const int nub = m_nub;
      int j = 0;
      for (; j < nub; ++j) {
        Dell[j] = aptr[j];
      }
      const int nC = m_nC;
      for (; j < nC; ++j) {
        Dell[j] = aptr[C[j]];
      }
  #else
      const int nC = m_nC;
      for (int j = 0; j < nC; ++j) {
        Dell[j] = aptr[C[j]];
      }
  #endif
    }
    dSolveL1(m_L, m_Dell, m_nC, m_nskip);
    {
      Scalar *ell = m_ell, *Dell = m_Dell, *d = m_d;
      const int nC = m_nC;
      for (int j = 0; j < nC; ++j) {
        ell[j] = Dell[j] * d[j];
      }
    }

    if (!only_transfer) {
      Scalar *tmp = m_tmp, *ell = m_ell;
      {
        const int nC = m_nC;
        for (int j = 0; j < nC; ++j) {
          tmp[j] = ell[j];
        }
      }
      dSolveL1T(m_L, tmp, m_nC, m_nskip);
      if (dir > 0) {
        int* C = m_C;
        Scalar* tmp = m_tmp;
        const int nC = m_nC;
        for (int j = 0; j < nC; ++j) {
          a[C[j]] = -tmp[j];
        }
      } else {
        int* C = m_C;
        Scalar* tmp = m_tmp;
        const int nC = m_nC;
        for (int j = 0; j < nC; ++j) {
          a[C[j]] = tmp[j];
        }
      }
    }
  }
}

template <typename Scalar>
void LCP<Scalar>::unpermute()
{
  // now we have to un-permute x and w
  {
    memcpy(m_tmp, m_x, m_n * sizeof(Scalar));
    Scalar *x = m_x, *tmp = m_tmp;
    const int* p = m_p;
    const int n = m_n;
    for (int j = 0; j < n; ++j) {
      x[p[j]] = tmp[j];
    }
  }
  {
    memcpy(m_tmp, m_w, m_n * sizeof(Scalar));
    Scalar *w = m_w, *tmp = m_tmp;
    const int* p = m_p;
    const int n = m_n;
    for (int j = 0; j < n; ++j) {
      w[p[j]] = tmp[j];
    }
  }
}

#endif // dLCP_FAST

} // namespace dart::math

#endif // DART_MATH_LCP_DANTZIG_LCP_IMPL_HPP_
