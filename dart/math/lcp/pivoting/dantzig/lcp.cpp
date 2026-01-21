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

/*


THE ALGORITHM
-------------

solve A*x = b+w, with x and w subject to certain LCP conditions.
each x(i),w(i) must lie on one of the three line segments in the following
diagram. each line segment corresponds to one index set :

     w(i)
     /|\      |           :
      |       |           :
      |       |i in N     :
  w>0 |       |state[i]=0 :
      |       |           :
      |       |           :  i in C
  w=0 +       +-----------------------+
      |                   :           |
      |                   :           |
  w<0 |                   :           |i in N
      |                   :           |state[i]=1
      |                   :           |
      |                   :           |
      +-------|-----------|-----------|----------> x(i)
             lo           0           hi

the Dantzig algorithm proceeds as follows:
  for i=1:n
    * if (x(i),w(i)) is not on the line, push x(i) and w(i) positive or
      negative towards the line. as this is done, the other (x(j),w(j))
      for j<i are constrained to be on the line. if any (x,w) reaches the
      end of a line segment then it is switched between index sets.
    * i is added to the appropriate index set depending on what line segment
      it hits.

we restrict lo(i) <= 0 and hi(i) >= 0. this makes the algorithm a bit
simpler, because the starting point for x(i),w(i) is always on the dotted
line x=0 and x will only ever increase in one direction, so it can only hit
two out of the three line segments.


NOTES
-----

this is an implementation of "lcp_dantzig2_ldlt.m" and "lcp_dantzig_lohi.m".
the implementation is split into an LCP problem object (dLCP) and an LCP
driver function. most optimization occurs in the dLCP object.

a naive implementation of the algorithm requires either a lot of data motion
or a lot of permutation-array lookup, because we are constantly re-ordering
rows and columns. to avoid this and make a more optimized algorithm, a
non-trivial data structure is used to represent the matrix A (this is
implemented in the fast version of the dLCP object).

during execution of this algorithm, some indexes in A are clamped (set C),
some are non-clamped (set N), and some are "don't care" (where x=0).
A,x,b,w (and other problem vectors) are permuted such that the clamped
indexes are first, the unclamped indexes are next, and the don't-care
indexes are last. this permutation is recorded in the array `p'.
initially p = 0..n-1, and as the rows and columns of A,x,b,w are swapped,
the corresponding elements of p are swapped.

because the C and N elements are grouped together in the rows of A, we can do
lots of work with a fast dot product function. if A,x,etc were not permuted
and we only had a permutation array, then those dot products would be much
slower as we would have a permutation array lookup in some inner loops.

A is accessed through an array of row pointers, so that element (i,j) of the
permuted matrix is A[i][j]. this makes row swapping fast. for column swapping
we still have to actually move the data.

during execution of this algorithm we maintain an L*D*L' factorization of
the clamped submatrix of A (call it `AC') which is the top left nC*nC
submatrix of A. there are two ways we could arrange the rows/columns in AC.

(1) AC is always permuted such that L*D*L' = AC. this causes a problem
when a row/column is removed from C, because then all the rows/columns of A
between the deleted index and the end of C need to be rotated downward.
this results in a lot of data motion and slows things down.
(2) L*D*L' is actually a factorization of a *permutation* of AC (which is
itself a permutation of the underlying A). this is what we do - the
permutation is recorded in the vector C. call this permutation A[C,C].
when a row/column is removed from C, all we have to do is swap two
rows/columns and manipulate C.

*/

// Define macros BEFORE including headers so they're available
// LCP debugging (mostly for fast dLCP) - this slows things down a lot
// #define DEBUG_LCP

#define dLCP_FAST // use fast dLCP object

#define ROWPTRS // Keep for compatibility (but now uses PivotMatrix internally)
#define AROW(i) (m_A[i])

#define NUB_OPTIMIZATIONS

#include "dart/math/lcp/pivoting/dantzig/lcp.hpp"

#include "dart/common/macros.hpp"
#include "dart/math/lcp/pivoting/dantzig/matrix.hpp"
#include "dart/math/lcp/pivoting/dantzig/misc.hpp"

#include <memory>
#include <vector>

//***************************************************************************
// code generation parameters

namespace dart::math {

//***************************************************************************
// an optimized Dantzig LCP driver routine for the lo-hi LCP problem.

template <typename Scalar>
bool SolveLCP(
    int n,
    Scalar* A,
    Scalar* x,
    Scalar* b,
    Scalar* outer_w /*=nullptr*/,
    int nub,
    Scalar* lo,
    Scalar* hi,
    int* findex,
    bool earlyTermination)
{
  DART_ASSERT(n > 0 && A && x && b && lo && hi && nub >= 0 && nub <= n);
  const Scalar kInfinity = ScalarTraits<Scalar>::inf();
#ifndef NDEBUG
  {
    // check restrictions on lo and hi
    for (int k = 0; k < n; ++k) {
      DART_ASSERT(lo[k] <= 0 && hi[k] >= 0);
    }
  }
#endif

  // if all the variables are unbounded then we can just factor, solve,
  // and return
  if (nub >= n) {
    const int nskip = padding(n);
    auto d = std::make_unique<Scalar[]>(n);
    SetZero(d.get(), n);

    std::unique_ptr<Scalar[]> A_storage;
    Scalar* A_work = nullptr;

    if (n != nskip) {
      A_storage.reset(new Scalar[n * nskip]);
      for (int i = 0; i < n; ++i) {
        memcpy(&A_storage[i * nskip], &A[i * nskip], n * sizeof(Scalar));
      }
      A_work = A_storage.get();
    } else {
      A_work = A;
    }

    dFactorLDLT(A_work, d.get(), n, nskip);
    dSolveLDLT(A_work, d.get(), b, n, nskip);
    memcpy(x, b, n * sizeof(Scalar));

    return true;
  }

  const int nskip = padding(n);

  std::unique_ptr<Scalar[]> A_storage;
  Scalar* A_work = nullptr;

  if (n != nskip) {
    A_storage.reset(new Scalar[n * nskip]);
    for (int i = 0; i < n; ++i) {
      memcpy(&A_storage[i * nskip], &A[i * nskip], n * sizeof(Scalar));
    }
    A_work = A_storage.get();
  } else {
    A_work = A;
  }

  // Create PivotMatrix from work array
  PivotMatrix<Scalar> A_pivot(n, n, A_work, nskip);

  auto L = std::make_unique<Scalar[]>(n * nskip);
  auto d = std::make_unique<Scalar[]>(n);
  std::unique_ptr<Scalar[]> wStorage;
  Scalar* w = outer_w;
  if (!w) {
    wStorage.reset(new Scalar[n]);
    w = wStorage.get();
  }
  auto delta_w = std::make_unique<Scalar[]>(n);
  auto delta_x = std::make_unique<Scalar[]>(n);
  auto Dell = std::make_unique<Scalar[]>(n);
  auto ell = std::make_unique<Scalar[]>(n);
  auto p = std::make_unique<int[]>(n);
  auto C = std::make_unique<int[]>(n);
  auto state = std::make_unique<bool[]>(n);

  // create LCP object. note that tmp is set to delta_w to save space, this
  // optimization relies on knowledge of how tmp is used, so be careful!
  LCP<Scalar> lcp(
      n,
      nskip,
      nub,
      A_pivot,
      x,
      b,
      w,
      lo,
      hi,
      L.get(),
      d.get(),
      Dell.get(),
      ell.get(),
      delta_w.get(),
      state.get(),
      findex,
      p.get(),
      C.get());
  int adj_nub = lcp.getNub();

  // loop over all indexes adj_nub..n-1. for index i, if x(i),w(i) satisfy the
  // LCP conditions then i is added to the appropriate index set. otherwise
  // x(i),w(i) is driven either +ve or -ve to force it to the valid region.
  // as we drive x(i), x(C) is also adjusted to keep w(C) at zero.
  // while driving x(i) we maintain the LCP conditions on the other variables
  // 0..i-1. we do this by watching out for other x(i),w(i) values going
  // outside the valid region, and then switching them between index sets
  // when that happens.

  bool hit_first_friction_index = false;
  for (int i = adj_nub; i < n; ++i) {
    bool s_error = false;
    // the index i is the driving index and indexes i+1..n-1 are "dont care",
    // i.e. when we make changes to the system those x's will be zero and we
    // don't care what happens to those w's. in other words, we only consider
    // an (i+1)*(i+1) sub-problem of A*x=b+w.

    // if we've hit the first friction index, we have to compute the lo and
    // hi values based on the values of x already computed. we have been
    // permuting the indexes, so the values stored in the findex vector are
    // no longer valid. thus we have to temporarily unpermute the x vector.
    // for the purposes of this computation, 0*infinity = 0 ... so if the
    // contact constraint's normal force is 0, there should be no tangential
    // force applied.

    if (!hit_first_friction_index && findex && findex[i] >= 0) {
      // un-permute x into delta_w, which is not being used at the moment
      for (int j = 0; j < n; ++j) {
        delta_w[p[j]] = x[j];
      }

      // set lo and hi values
      for (int k = i; k < n; ++k) {
        Scalar wfk = delta_w[findex[k]];
        if (wfk == 0) {
          hi[k] = 0;
          lo[k] = 0;
        } else {
          hi[k] = std::fabs(hi[k] * wfk);
          lo[k] = -hi[k];
        }
      }
      hit_first_friction_index = true;
    }

    // thus far we have not even been computing the w values for indexes
    // greater than i, so compute w[i] now.
    w[i] = lcp.AiC_times_qC(i, x) + lcp.AiN_times_qN(i, x) - b[i];

    // if lo=hi=0 (which can happen for tangential friction when normals are
    // 0) then the index will be assigned to set N with some state. however,
    // set C's line has zero size, so the index will always remain in set N.
    // with the "normal" switching logic, if w changed sign then the index
    // would have to switch to set C and then back to set N with an inverted
    // state. this is pointless, and also computationally expensive. to
    // prevent this from happening, we use the rule that indexes with lo=hi=0
    // will never be checked for set changes. this means that the state for
    // these indexes may be incorrect, but that doesn't matter.

    // see if x(i),w(i) is in a valid region
    if (lo[i] == 0 && w[i] >= 0) {
      lcp.transfer_i_to_N(i);
      state[i] = false;
    } else if (hi[i] == 0 && w[i] <= 0) {
      lcp.transfer_i_to_N(i);
      state[i] = true;
    } else if (w[i] == 0) {
      // this is a degenerate case. by the time we get to this test we know
      // that lo != 0, which means that lo < 0 as lo is not allowed to be +ve,
      // and similarly that hi > 0. this means that the line segment
      // corresponding to set C is at least finite in extent, and we are on it.
      // NOTE: we must call lcp.solve1() before lcp.transfer_i_to_C()
      lcp.solve1(delta_x.get(), i, 0, 1);

      lcp.transfer_i_to_C(i);
    } else {
      // we must push x(i) and w(i)
      for (;;) {
        int dir;
        Scalar dirf;
        // find direction to push on x(i)
        if (w[i] <= 0) {
          dir = 1;
          dirf = static_cast<Scalar>(1.0);
        } else {
          dir = -1;
          dirf = static_cast<Scalar>(-1.0);
        }

        // compute: delta_x(C) = -dir*A(C,C)\A(C,i)
        lcp.solve1(delta_x.get(), i, dir);

        // note that delta_x[i] = dirf, but we won't bother to set it

        // compute: delta_w = A*delta_x ... note we only care about
        // delta_w(N) and delta_w(i), the rest is ignored
        lcp.pN_equals_ANC_times_qC(delta_w.get(), delta_x.get());
        lcp.pN_plusequals_ANi(delta_w.get(), i, dir);
        delta_w[i] = lcp.AiC_times_qC(i, delta_x.get()) + lcp.Aii(i) * dirf;

        // find largest step we can take (size=s), either to drive x(i),w(i)
        // to the valid LCP region or to drive an already-valid variable
        // outside the valid region.

        int cmd = 1; // index switching command
        int si = 0;  // si = index to switch if cmd>3
        Scalar s = -w[i] / delta_w[i];
        if (dir > 0) {
          if (hi[i] < kInfinity) {
            Scalar s2 = (hi[i] - x[i])
                        * dirf; // was (hi[i]-x[i])/dirf	// step to x(i)=hi(i)
            if (s2 < s) {
              s = s2;
              cmd = 3;
            }
          }
        } else {
          if (lo[i] > -kInfinity) {
            Scalar s2 = (lo[i] - x[i])
                        * dirf; // was (lo[i]-x[i])/dirf	// step to x(i)=lo(i)
            if (s2 < s) {
              s = s2;
              cmd = 2;
            }
          }
        }

        {
          const int numN = lcp.numN();
          for (int k = 0; k < numN; ++k) {
            const int indexN_k = lcp.indexN(k);
            if (!state[indexN_k] ? delta_w[indexN_k] < 0
                                 : delta_w[indexN_k] > 0) {
              // don't bother checking if lo=hi=0
              if (lo[indexN_k] == 0 && hi[indexN_k] == 0) {
                continue;
              }
              Scalar s2 = -w[indexN_k] / delta_w[indexN_k];
              if (s2 < s) {
                s = s2;
                cmd = 4;
                si = indexN_k;
              }
            }
          }
        }

        {
          const int numC = lcp.numC();
          for (int k = adj_nub; k < numC; ++k) {
            const int indexC_k = lcp.indexC(k);
            if (delta_x[indexC_k] < 0 && lo[indexC_k] > -kInfinity) {
              Scalar s2 = (lo[indexC_k] - x[indexC_k]) / delta_x[indexC_k];
              if (s2 < s) {
                s = s2;
                cmd = 5;
                si = indexC_k;
              }
            }
            if (delta_x[indexC_k] > 0 && hi[indexC_k] < kInfinity) {
              Scalar s2 = (hi[indexC_k] - x[indexC_k]) / delta_x[indexC_k];
              if (s2 < s) {
                s = s2;
                cmd = 6;
                si = indexC_k;
              }
            }
          }
        }

        // static char* cmdstring[8] = {0,"->C","->NL","->NH","N->C",
        //			     "C->NL","C->NH"};
        // printf ("cmd=%d (%s), si=%d\n",cmd,cmdstring[cmd],(cmd>3) ? si : i);

        // if s <= 0 then we've got a problem. if we just keep going then
        // we're going to get stuck in an infinite loop. instead, just cross
        // our fingers and exit with the current solution.
        if (s <= static_cast<Scalar>(0.0)) {
          if (earlyTermination) {
            return false;
          }

          // We shouldn't be overly aggressive about printing this warning,
          // because sometimes it gets spammed if s is just a tiny bit beneath
          // 0.0. Rely on the *_ONCE helper so we only emit the first instance.
          DART_WARN_ONCE_IF(
              s < static_cast<Scalar>(-1e-6),
              "LCP internal error, s <= 0 (s={:.4e})",
              s);

          if (i < n) {
            SetZero(x + i, n - i);
            SetZero(w + i, n - i);
          }
          s_error = true;
          break;
        }

        // apply x = x + s * delta_x
        lcp.pC_plusequals_s_times_qC(x, s, delta_x.get());
        x[i] += s * dirf;

        // apply w = w + s * delta_w
        lcp.pN_plusequals_s_times_qN(w, s, delta_w.get());
        w[i] += s * delta_w[i];

        // void *tmpbuf;
        // switch indexes between sets if necessary
        switch (cmd) {
          case 1: // done
            w[i] = 0;
            lcp.transfer_i_to_C(i);
            break;
          case 2: // done
            x[i] = lo[i];
            state[i] = false;
            lcp.transfer_i_to_N(i);
            break;
          case 3: // done
            x[i] = hi[i];
            state[i] = true;
            lcp.transfer_i_to_N(i);
            break;
          case 4: // keep going
            w[si] = 0;
            lcp.transfer_i_from_N_to_C(si);
            break;
          case 5: // keep going
            x[si] = lo[si];
            state[si] = false;
            lcp.transfer_i_from_C_to_N(si, nullptr);
            break;
          case 6: // keep going
            x[si] = hi[si];
            state[si] = true;
            lcp.transfer_i_from_C_to_N(si, nullptr);
            break;
        }

        if (cmd <= 3) {
          break;
        }
      } // for (;;)
    } // else

    if (s_error) {
      break;
    }
  } // for (int i=adj_nub; i<n; ++i)

  lcp.unpermute();

  return true;
}

// Explicit template instantiations
template DART_API bool SolveLCP<float>(
    int n,
    float* A,
    float* x,
    float* b,
    float* w,
    int nub,
    float* lo,
    float* hi,
    int* findex,
    bool earlyTermination);

template DART_API bool SolveLCP<double>(
    int n,
    double* A,
    double* x,
    double* b,
    double* w,
    int nub,
    double* lo,
    double* hi,
    int* findex,
    bool earlyTermination);

} // namespace dart::math

// Note: Template implementations are in lcp-impl.hpp (included via lcp.h)
