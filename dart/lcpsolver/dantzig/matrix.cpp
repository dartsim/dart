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

//#include "config.h"
#ifdef __APPLE__
  #include <malloc/malloc.h>
#else
  #include <malloc.h>
#endif

#include "dart/lcpsolver/dantzig/common.h"
#include "dart/lcpsolver/dantzig/matrix.h"

#include <algorithm>

#ifndef EFFICIENT_ALIGNMENT
  #define EFFICIENT_ALIGNMENT 16
#endif

/* utility */

/* round something up to be a multiple of the EFFICIENT_ALIGNMENT */

#define dEFFICIENT_SIZE(x)                                                     \
  (((x) + (EFFICIENT_ALIGNMENT - 1)) & ~((size_t)(EFFICIENT_ALIGNMENT - 1)))
#define dEFFICIENT_PTR(p) ((void*)dEFFICIENT_SIZE((size_t)(p)))
#define dOFFSET_EFFICIENTLY(p, b) ((void*)((size_t)(p) + dEFFICIENT_SIZE(b)))

/* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
 * up to 15 bytes per allocation, depending on what alloca() returns.
 */

#define dALLOCA16(n)                                                           \
  ((char*)dEFFICIENT_PTR(alloca((n) + (EFFICIENT_ALIGNMENT - 1))))

// misc defines
#define ALLOCA dALLOCA16

namespace dart::lcpsolver {

void dMultiply0(dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  Multiply0(A, B, C, p, q, r);
}

void dMultiply1(dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  Multiply1(A, B, C, p, q, r);
}

void dMultiply2(dReal* A, const dReal* B, const dReal* C, int p, int q, int r)
{
  Multiply2(A, B, C, p, q, r);
}

int dFactorCholesky(dReal* A, int n, void* tmpbuf /*[n]*/)
{
  DART_ASSERT(n > 0 && A);
  bool failure = false;
  const int nskip = padding(n);
  dReal* recip = tmpbuf ? (dReal*)tmpbuf : (dReal*)ALLOCA(n * sizeof(dReal));
  dReal* aa = A;
  for (int i = 0; i < n; aa += nskip, ++i) {
    dReal* cc = aa;
    {
      const dReal* bb = A;
      for (int j = 0; j < i; bb += nskip, ++cc, ++j) {
        dReal sum = *cc;
        const dReal *a = aa, *b = bb, *bend = bb + j;
        for (; b != bend; ++a, ++b) {
          sum -= (*a) * (*b);
        }
        *cc = sum * recip[j];
      }
    }
    {
      dReal sum = *cc;
      dReal *a = aa, *aend = aa + i;
      for (; a != aend; ++a) {
        sum -= (*a) * (*a);
      }
      if (sum <= REAL(0.0)) {
        failure = true;
        break;
      }
      dReal sumsqrt = std::sqrt(sum);
      *cc = sumsqrt;
      recip[i] = reciprocal(sumsqrt);
    }
  }
  return failure ? 0 : 1;
}

void dSolveCholesky(const dReal* L, dReal* b, int n, void* tmpbuf /*[n]*/)
{
  DART_ASSERT(n > 0 && L && b);
  const int nskip = padding(n);
  dReal* y = tmpbuf ? (dReal*)tmpbuf : (dReal*)ALLOCA(n * sizeof(dReal));
  {
    const dReal* ll = L;
    for (int i = 0; i < n; ll += nskip, ++i) {
      dReal sum = REAL(0.0);
      for (int k = 0; k < i; ++k) {
        sum += ll[k] * y[k];
      }
      DART_ASSERT(ll[i] != dReal(0.0));
      y[i] = (b[i] - sum) / ll[i];
    }
  }
  {
    const dReal* ll = L + (n - 1) * (nskip + 1);
    for (int i = n - 1; i >= 0; ll -= nskip + 1, --i) {
      dReal sum = REAL(0.0);
      const dReal* l = ll + nskip;
      for (int k = i + 1; k < n; l += nskip, ++k) {
        sum += (*l) * b[k];
      }
      DART_ASSERT(*ll != dReal(0.0));
      b[i] = (y[i] - sum) / (*ll);
    }
  }
}

int dInvertPDMatrix(
    const dReal* A, dReal* Ainv, int n, void* tmpbuf /*[nskip*(n+2)]*/)
{
  DART_ASSERT(n > 0 && A && Ainv);
  bool success = false;
  size_t FactorCholesky_size = dEstimateFactorCholeskyTmpbufSize(n);
  size_t SolveCholesky_size = dEstimateSolveCholeskyTmpbufSize(n);
  size_t MaxCholesky_size = FactorCholesky_size > SolveCholesky_size
                                ? FactorCholesky_size
                                : SolveCholesky_size;
  DART_ASSERT(MaxCholesky_size % sizeof(dReal) == 0);
  const int nskip = padding(n);
  const int nskip_mul_n = nskip * n;
  dReal* tmp
      = tmpbuf ? (dReal*)tmpbuf
               : (dReal*)ALLOCA(
                   MaxCholesky_size + (nskip + nskip_mul_n) * sizeof(dReal));
  dReal* X = (dReal*)((char*)tmp + MaxCholesky_size);
  dReal* L = X + nskip;
  memcpy(L, A, nskip_mul_n * sizeof(dReal));
  if (dFactorCholesky(L, n, tmp)) {
    SetZero(Ainv, nskip_mul_n); // make sure all padding elements set to 0
    dReal *aa = Ainv, *xi = X, *xiend = X + n;
    for (; xi != xiend; ++aa, ++xi) {
      SetZero(X, n);
      *xi = REAL(1.0);
      dSolveCholesky(L, X, n, tmp);
      dReal* a = aa;
      const dReal *x = X, *xend = X + n;
      for (; x != xend; a += nskip, ++x) {
        *a = *x;
      }
    }
    success = true;
  }
  return success ? 1 : 0;
}

int dIsPositiveDefinite(const dReal* A, int n, void* tmpbuf /*[nskip*(n+1)]*/)
{
  DART_ASSERT(n > 0 && A);
  size_t FactorCholesky_size = dEstimateFactorCholeskyTmpbufSize(n);
  DART_ASSERT(FactorCholesky_size % sizeof(dReal) == 0);
  const int nskip = padding(n);
  const int nskip_mul_n = nskip * n;
  dReal* tmp
      = tmpbuf
            ? (dReal*)tmpbuf
            : (dReal*)ALLOCA(FactorCholesky_size + nskip_mul_n * sizeof(dReal));
  dReal* Acopy = (dReal*)((char*)tmp + FactorCholesky_size);
  memcpy(Acopy, A, nskip_mul_n * sizeof(dReal));
  return dFactorCholesky(Acopy, n, tmp);
}

void dVectorScale(dReal* a, const dReal* d, int n)
{
  VectorScale(a, d, n);
}

void dSolveLDLT(const dReal* L, const dReal* d, dReal* b, int n, int nskip)
{
  DART_ASSERT(L && d && b && n > 0 && nskip >= n);
  dSolveL1(L, b, n, nskip);
  dVectorScale(b, d, n);
  dSolveL1T(L, b, n, nskip);
}

void dLDLTAddTL(
    dReal* L,
    dReal* d,
    const dReal* a,
    int n,
    int nskip,
    void* tmpbuf /*[2*nskip]*/)
{
  DART_ASSERT(L && d && a && n > 0 && nskip >= n);

  if (n < 2)
    return;
  dReal* W1
      = tmpbuf ? (dReal*)tmpbuf : (dReal*)ALLOCA((2 * nskip) * sizeof(dReal));
  dReal* W2 = W1 + nskip;

  W1[0] = REAL(0.0);
  W2[0] = REAL(0.0);
  for (int j = 1; j < n; ++j) {
    W1[j] = W2[j] = (dReal)(a[j] * M_SQRT1_2);
  }
  dReal W11 = (dReal)((REAL(0.5) * a[0] + 1) * M_SQRT1_2);
  dReal W21 = (dReal)((REAL(0.5) * a[0] - 1) * M_SQRT1_2);

  dReal alpha1 = REAL(1.0);
  dReal alpha2 = REAL(1.0);

  {
    dReal dee = d[0];
    dReal alphanew = alpha1 + (W11 * W11) * dee;
    DART_ASSERT(alphanew != dReal(0.0));
    dee /= alphanew;
    dReal gamma1 = W11 * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - (W21 * W21) * dee;
    dee /= alphanew;
    // dReal gamma2 = W21 * dee;
    alpha2 = alphanew;
    dReal k1 = REAL(1.0) - W21 * gamma1;
    dReal k2 = W21 * gamma1 * W11 - W21;
    dReal* ll = L + nskip;
    for (int p = 1; p < n; ll += nskip, ++p) {
      dReal Wp = W1[p];
      dReal ell = *ll;
      W1[p] = Wp - W11 * ell;
      W2[p] = k1 * Wp + k2 * ell;
    }
  }

  dReal* ll = L + (nskip + 1);
  for (int j = 1; j < n; ll += nskip + 1, ++j) {
    dReal k1 = W1[j];
    dReal k2 = W2[j];

    dReal dee = d[j];
    dReal alphanew = alpha1 + (k1 * k1) * dee;
    DART_ASSERT(alphanew != dReal(0.0));
    dee /= alphanew;
    dReal gamma1 = k1 * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - (k2 * k2) * dee;
    dee /= alphanew;
    dReal gamma2 = k2 * dee;
    dee *= alpha2;
    d[j] = dee;
    alpha2 = alphanew;

    dReal* l = ll + nskip;
    for (int p = j + 1; p < n; l += nskip, ++p) {
      dReal ell = *l;
      dReal Wp = W1[p] - k1 * ell;
      ell += gamma1 * Wp;
      W1[p] = Wp;
      Wp = W2[p] - k2 * ell;
      ell -= gamma2 * Wp;
      W2[p] = Wp;
      *l = ell;
    }
  }
}

// macros for dLDLTRemove() for accessing A - either access the matrix
// directly or access it via row pointers. we are only supposed to reference
// the lower triangle of A (it is symmetric), but indexes i and j come from
// permutation vectors so they are not predictable. so do a test on the
// indexes - this should not slow things down too much, as we don't do this
// in an inner loop.

#define _GETA(i, j) (A[i][j])
//#define _GETA(i,j) (A[(i)*nskip+(j)])
#define GETA(i, j) ((i > j) ? _GETA(i, j) : _GETA(j, i))

void dLDLTRemove(
    dReal** A,
    const int* p,
    dReal* L,
    dReal* d,
    int n1,
    int n2,
    int r,
    int nskip,
    void* tmpbuf /*n2 + 2*nskip*/)
{
  DART_ASSERT(
      A && p && L && d && n1 > 0 && n2 > 0 && r >= 0 && r < n2 && n1 >= n2
      && nskip >= n1);
  for (int i = 0; i < n2; ++i) {
    DART_ASSERT(p[i] >= 0 && p[i] < n1);
  }
  DART_UNUSED(n1); // Only used in assertions above

  if (r == n2 - 1) {
    return; // deleting last row/col is easy
  } else {
    size_t LDLTAddTL_size = dEstimateLDLTAddTLTmpbufSize(nskip);
    DART_ASSERT(LDLTAddTL_size % sizeof(dReal) == 0);
    dReal* tmp = tmpbuf ? (dReal*)tmpbuf
                        : (dReal*)ALLOCA(LDLTAddTL_size + n2 * sizeof(dReal));
    if (r == 0) {
      dReal* a = (dReal*)((char*)tmp + LDLTAddTL_size);
      const int p_0 = p[0];
      for (int i = 0; i < n2; ++i) {
        a[i] = -GETA(p[i], p_0);
      }
      a[0] += REAL(1.0);
      dLDLTAddTL(L, d, a, n2, nskip, tmp);
    } else {
      dReal* t = (dReal*)((char*)tmp + LDLTAddTL_size);
      {
        dReal* Lcurr = L + r * nskip;
        for (int i = 0; i < r; ++Lcurr, ++i) {
          DART_ASSERT(d[i] != dReal(0.0));
          t[i] = *Lcurr / d[i];
        }
      }
      dReal* a = t + r;
      {
        dReal* Lcurr = L + r * nskip;
        const int *pp_r = p + r, p_r = *pp_r;
        const int n2_minus_r = n2 - r;
        for (int i = 0; i < n2_minus_r; Lcurr += nskip, ++i) {
          a[i] = dDot(Lcurr, t, r) - GETA(pp_r[i], p_r);
        }
      }
      a[0] += REAL(1.0);
      dLDLTAddTL(L + r * nskip + r, d + r, a, n2 - r, nskip, tmp);
    }
  }

  // snip out row/column r from L and d
  dRemoveRowCol(L, n2, nskip, r);
  if (r < (n2 - 1))
    memmove(d + r, d + r + 1, (n2 - r - 1) * sizeof(dReal));
}

void dRemoveRowCol(dReal* A, int n, int nskip, int r)
{
  DART_ASSERT(A && n > 0 && nskip >= n && r >= 0 && r < n);
  if (r >= n - 1)
    return;
  if (r > 0) {
    {
      const size_t move_size = (n - r - 1) * sizeof(dReal);
      dReal* Adst = A + r;
      for (int i = 0; i < r; Adst += nskip, ++i) {
        dReal* Asrc = Adst + 1;
        memmove(Adst, Asrc, move_size);
      }
    }
    {
      const size_t cpy_size = r * sizeof(dReal);
      dReal* Adst = A + r * nskip;
      for (int i = r; i < (n - 1); ++i) {
        dReal* Asrc = Adst + nskip;
        memcpy(Adst, Asrc, cpy_size);
        Adst = Asrc;
      }
    }
  }
  {
    const size_t cpy_size = (n - r - 1) * sizeof(dReal);
    dReal* Adst = A + r * (nskip + 1);
    for (int i = r; i < (n - 1); ++i) {
      dReal* Asrc = Adst + (nskip + 1);
      memcpy(Adst, Asrc, cpy_size);
      Adst = Asrc - 1;
    }
  }
}

//==============================================================================
// Fast optimized implementations (originally in fast*.cpp files)
//==============================================================================

// From fastdot.cpp
dReal _dDot(const dReal* a, const dReal* b, int n)
{
  dReal p0, q0, m0, p1, q1, m1, sum;
  sum = 0;
  n -= 2;
  while (n >= 0) {
    p0 = a[0];
    q0 = b[0];
    m0 = p0 * q0;
    p1 = a[1];
    q1 = b[1];
    m1 = p1 * q1;
    sum += m0;
    sum += m1;
    a += 2;
    b += 2;
    n -= 2;
  }
  n += 2;
  while (n > 0) {
    sum += (*a) * (*b);
    a++;
    b++;
    n--;
  }
  return sum;
}

#undef dDot

dReal dDot(const dReal* a, const dReal* b, int n)
{
  return _dDot(a, b, n);
}

// From fastldlt.cpp
static void dSolveL1_1(const dReal* L, dReal* B, int n, int lskip1)
{
  dReal Z11, m11, Z21, m21, p1, q1, p2, *ex;
  const dReal* ell;
  int i, j;
  for (i = 0; i < n; i += 2) {
    Z11 = 0;
    Z21 = 0;
    ell = L + i * lskip1;
    ex = B;
    for (j = i - 2; j >= 0; j -= 2) {
      p1 = ell[0];
      q1 = ex[0];
      m11 = p1 * q1;
      p2 = ell[lskip1];
      m21 = p2 * q1;
      Z11 += m11;
      Z21 += m21;
      p1 = ell[1];
      q1 = ex[1];
      m11 = p1 * q1;
      p2 = ell[1 + lskip1];
      m21 = p2 * q1;
      ell += 2;
      ex += 2;
      Z11 += m11;
      Z21 += m21;
    }
    j += 2;
    for (; j > 0; j--) {
      p1 = ell[0];
      q1 = ex[0];
      m11 = p1 * q1;
      p2 = ell[lskip1];
      m21 = p2 * q1;
      ell += 1;
      ex += 1;
      Z11 += m11;
      Z21 += m21;
    }
    Z11 = ex[0] - Z11;
    ex[0] = Z11;
    p1 = ell[lskip1];
    Z21 = ex[1] - Z21 - p1 * Z11;
    ex[1] = Z21;
  }
}

static void dSolveL1_2(const dReal* L, dReal* B, int n, int lskip1)
{
  dReal Z11, m11, Z12, m12, Z21, m21, Z22, m22, p1, q1, p2, q2, *ex;
  const dReal* ell;
  int i, j;
  for (i = 0; i < n; i += 2) {
    Z11 = 0;
    Z12 = 0;
    Z21 = 0;
    Z22 = 0;
    ell = L + i * lskip1;
    ex = B;
    for (j = i - 2; j >= 0; j -= 2) {
      p1 = ell[0];
      q1 = ex[0];
      m11 = p1 * q1;
      q2 = ex[lskip1];
      m12 = p1 * q2;
      p2 = ell[lskip1];
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z12 += m12;
      Z21 += m21;
      Z22 += m22;
      p1 = ell[1];
      q1 = ex[1];
      m11 = p1 * q1;
      q2 = ex[1 + lskip1];
      m12 = p1 * q2;
      p2 = ell[1 + lskip1];
      m21 = p2 * q1;
      m22 = p2 * q2;
      ell += 2;
      ex += 2;
      Z11 += m11;
      Z12 += m12;
      Z21 += m21;
      Z22 += m22;
    }
    j += 2;
    for (; j > 0; j--) {
      p1 = ell[0];
      q1 = ex[0];
      m11 = p1 * q1;
      q2 = ex[lskip1];
      m12 = p1 * q2;
      p2 = ell[lskip1];
      m21 = p2 * q1;
      m22 = p2 * q2;
      ell += 1;
      ex += 1;
      Z11 += m11;
      Z12 += m12;
      Z21 += m21;
      Z22 += m22;
    }
    Z11 = ex[0] - Z11;
    ex[0] = Z11;
    Z12 = ex[lskip1] - Z12;
    ex[lskip1] = Z12;
    p1 = ell[lskip1];
    Z21 = ex[1] - Z21 - p1 * Z11;
    ex[1] = Z21;
    Z22 = ex[1 + lskip1] - Z22 - p1 * Z12;
    ex[1 + lskip1] = Z22;
  }
}

void _dFactorLDLT(dReal* A, dReal* d, int n, int nskip1)
{
  int i, j;
  dReal sum, *ell, *dee, dd, p1, p2, q1, q2, Z11, m11, Z21, m21, Z22, m22;
  if (n < 1)
    return;

  for (i = 0; i <= n - 2; i += 2) {
    dSolveL1_2(A, A + i * nskip1, i, nskip1);
    Z11 = 0;
    Z21 = 0;
    Z22 = 0;
    ell = A + i * nskip1;
    dee = d;
    for (j = i - 6; j >= 0; j -= 6) {
      p1 = ell[0];
      p2 = ell[nskip1];
      dd = dee[0];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[0] = q1;
      ell[nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      p1 = ell[1];
      p2 = ell[1 + nskip1];
      dd = dee[1];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[1] = q1;
      ell[1 + nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      p1 = ell[2];
      p2 = ell[2 + nskip1];
      dd = dee[2];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[2] = q1;
      ell[2 + nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      p1 = ell[3];
      p2 = ell[3 + nskip1];
      dd = dee[3];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[3] = q1;
      ell[3 + nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      p1 = ell[4];
      p2 = ell[4 + nskip1];
      dd = dee[4];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[4] = q1;
      ell[4 + nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      p1 = ell[5];
      p2 = ell[5 + nskip1];
      dd = dee[5];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[5] = q1;
      ell[5 + nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      ell += 6;
      dee += 6;
    }
    j += 6;
    for (; j > 0; j--) {
      p1 = ell[0];
      p2 = ell[nskip1];
      dd = dee[0];
      q1 = p1 * dd;
      q2 = p2 * dd;
      ell[0] = q1;
      ell[nskip1] = q2;
      m11 = p1 * q1;
      m21 = p2 * q1;
      m22 = p2 * q2;
      Z11 += m11;
      Z21 += m21;
      Z22 += m22;
      ell++;
      dee++;
    }
    Z11 = ell[0] - Z11;
    Z21 = ell[nskip1] - Z21;
    Z22 = ell[1 + nskip1] - Z22;
    dee = d + i;
    dee[0] = reciprocal(Z11);
    sum = 0;
    q1 = Z21;
    q2 = q1 * dee[0];
    Z21 = q2;
    sum += q1 * q2;
    dee[1] = reciprocal(Z22 - sum);
    ell[nskip1] = Z21;
  }
  switch (n - i) {
    case 0:
      break;

    case 1:
      dSolveL1_1(A, A + i * nskip1, i, nskip1);
      Z11 = 0;
      ell = A + i * nskip1;
      dee = d;
      for (j = i - 6; j >= 0; j -= 6) {
        p1 = ell[0];
        dd = dee[0];
        q1 = p1 * dd;
        ell[0] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        p1 = ell[1];
        dd = dee[1];
        q1 = p1 * dd;
        ell[1] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        p1 = ell[2];
        dd = dee[2];
        q1 = p1 * dd;
        ell[2] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        p1 = ell[3];
        dd = dee[3];
        q1 = p1 * dd;
        ell[3] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        p1 = ell[4];
        dd = dee[4];
        q1 = p1 * dd;
        ell[4] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        p1 = ell[5];
        dd = dee[5];
        q1 = p1 * dd;
        ell[5] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        ell += 6;
        dee += 6;
      }
      j += 6;
      for (; j > 0; j--) {
        p1 = ell[0];
        dd = dee[0];
        q1 = p1 * dd;
        ell[0] = q1;
        m11 = p1 * q1;
        Z11 += m11;
        ell++;
        dee++;
      }
      Z11 = ell[0] - Z11;
      dee = d + i;
      dee[0] = reciprocal(Z11);
      break;
  }
}

#undef dFactorLDLT

void dFactorLDLT(dReal* A, dReal* d, int n, int nskip1)
{
  _dFactorLDLT(A, d, n, nskip1);
}

// From fastlsolve.cpp
void _dSolveL1(const dReal* L, dReal* B, int n, int lskip1)
{
  dReal Z11, Z21, Z31, Z41, p1, q1, p2, p3, p4, *ex;
  const dReal* ell;
  int lskip2, lskip3, i, j;
  lskip2 = 2 * lskip1;
  lskip3 = 3 * lskip1;
  for (i = 0; i <= n - 4; i += 4) {
    Z11 = 0;
    Z21 = 0;
    Z31 = 0;
    Z41 = 0;
    ell = L + i * lskip1;
    ex = B;
    for (j = i - 12; j >= 0; j -= 12) {
      p1 = ell[0];
      q1 = ex[0];
      p2 = ell[lskip1];
      p3 = ell[lskip2];
      p4 = ell[lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[1];
      q1 = ex[1];
      p2 = ell[1 + lskip1];
      p3 = ell[1 + lskip2];
      p4 = ell[1 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[2];
      q1 = ex[2];
      p2 = ell[2 + lskip1];
      p3 = ell[2 + lskip2];
      p4 = ell[2 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[3];
      q1 = ex[3];
      p2 = ell[3 + lskip1];
      p3 = ell[3 + lskip2];
      p4 = ell[3 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[4];
      q1 = ex[4];
      p2 = ell[4 + lskip1];
      p3 = ell[4 + lskip2];
      p4 = ell[4 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[5];
      q1 = ex[5];
      p2 = ell[5 + lskip1];
      p3 = ell[5 + lskip2];
      p4 = ell[5 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[6];
      q1 = ex[6];
      p2 = ell[6 + lskip1];
      p3 = ell[6 + lskip2];
      p4 = ell[6 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[7];
      q1 = ex[7];
      p2 = ell[7 + lskip1];
      p3 = ell[7 + lskip2];
      p4 = ell[7 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[8];
      q1 = ex[8];
      p2 = ell[8 + lskip1];
      p3 = ell[8 + lskip2];
      p4 = ell[8 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[9];
      q1 = ex[9];
      p2 = ell[9 + lskip1];
      p3 = ell[9 + lskip2];
      p4 = ell[9 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[10];
      q1 = ex[10];
      p2 = ell[10 + lskip1];
      p3 = ell[10 + lskip2];
      p4 = ell[10 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      p1 = ell[11];
      q1 = ex[11];
      p2 = ell[11 + lskip1];
      p3 = ell[11 + lskip2];
      p4 = ell[11 + lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      ell += 12;
      ex += 12;
    }
    j += 12;
    for (; j > 0; j--) {
      p1 = ell[0];
      q1 = ex[0];
      p2 = ell[lskip1];
      p3 = ell[lskip2];
      p4 = ell[lskip3];
      Z11 += p1 * q1;
      Z21 += p2 * q1;
      Z31 += p3 * q1;
      Z41 += p4 * q1;
      ell += 1;
      ex += 1;
    }
    Z11 = ex[0] - Z11;
    ex[0] = Z11;
    p1 = ell[lskip1];
    Z21 = ex[1] - Z21 - p1 * Z11;
    ex[1] = Z21;
    p1 = ell[lskip2];
    p2 = ell[1 + lskip2];
    Z31 = ex[2] - Z31 - p1 * Z11 - p2 * Z21;
    ex[2] = Z31;
    p1 = ell[lskip3];
    p2 = ell[1 + lskip3];
    p3 = ell[2 + lskip3];
    Z41 = ex[3] - Z41 - p1 * Z11 - p2 * Z21 - p3 * Z31;
    ex[3] = Z41;
  }
  for (; i < n; i++) {
    Z11 = 0;
    ell = L + i * lskip1;
    ex = B;
    for (j = i - 12; j >= 0; j -= 12) {
      p1 = ell[0];
      q1 = ex[0];
      Z11 += p1 * q1;
      p1 = ell[1];
      q1 = ex[1];
      Z11 += p1 * q1;
      p1 = ell[2];
      q1 = ex[2];
      Z11 += p1 * q1;
      p1 = ell[3];
      q1 = ex[3];
      Z11 += p1 * q1;
      p1 = ell[4];
      q1 = ex[4];
      Z11 += p1 * q1;
      p1 = ell[5];
      q1 = ex[5];
      Z11 += p1 * q1;
      p1 = ell[6];
      q1 = ex[6];
      Z11 += p1 * q1;
      p1 = ell[7];
      q1 = ex[7];
      Z11 += p1 * q1;
      p1 = ell[8];
      q1 = ex[8];
      Z11 += p1 * q1;
      p1 = ell[9];
      q1 = ex[9];
      Z11 += p1 * q1;
      p1 = ell[10];
      q1 = ex[10];
      Z11 += p1 * q1;
      p1 = ell[11];
      q1 = ex[11];
      Z11 += p1 * q1;
      ell += 12;
      ex += 12;
    }
    j += 12;
    for (; j > 0; j--) {
      p1 = ell[0];
      q1 = ex[0];
      Z11 += p1 * q1;
      ell += 1;
      ex += 1;
    }
    Z11 = ex[0] - Z11;
    ex[0] = Z11;
  }
}

#undef dSolveL1

void dSolveL1(const dReal* L, dReal* B, int n, int lskip1)
{
  _dSolveL1(L, B, n, lskip1);
}

// From fastltsolve.cpp
void _dSolveL1T(const dReal* L, dReal* B, int n, int lskip1)
{
  dReal Z11, m11, Z21, m21, Z31, m31, Z41, m41, p1, q1, p2, p3, p4, *ex;
  const dReal* ell;
  int lskip2, i, j;
  L = L + (n - 1) * (lskip1 + 1);
  B = B + n - 1;
  lskip1 = -lskip1;
  lskip2 = 2 * lskip1;
  for (i = 0; i <= n - 4; i += 4) {
    Z11 = 0;
    Z21 = 0;
    Z31 = 0;
    Z41 = 0;
    ell = L - i;
    ex = B;
    for (j = i - 4; j >= 0; j -= 4) {
      p1 = ell[0];
      q1 = ex[0];
      p2 = ell[-1];
      p3 = ell[-2];
      p4 = ell[-3];
      m11 = p1 * q1;
      m21 = p2 * q1;
      m31 = p3 * q1;
      m41 = p4 * q1;
      ell += lskip1;
      Z11 += m11;
      Z21 += m21;
      Z31 += m31;
      Z41 += m41;
      p1 = ell[0];
      q1 = ex[-1];
      p2 = ell[-1];
      p3 = ell[-2];
      p4 = ell[-3];
      m11 = p1 * q1;
      m21 = p2 * q1;
      m31 = p3 * q1;
      m41 = p4 * q1;
      ell += lskip1;
      Z11 += m11;
      Z21 += m21;
      Z31 += m31;
      Z41 += m41;
      p1 = ell[0];
      q1 = ex[-2];
      p2 = ell[-1];
      p3 = ell[-2];
      p4 = ell[-3];
      m11 = p1 * q1;
      m21 = p2 * q1;
      m31 = p3 * q1;
      m41 = p4 * q1;
      ell += lskip1;
      Z11 += m11;
      Z21 += m21;
      Z31 += m31;
      Z41 += m41;
      p1 = ell[0];
      q1 = ex[-3];
      p2 = ell[-1];
      p3 = ell[-2];
      p4 = ell[-3];
      m11 = p1 * q1;
      m21 = p2 * q1;
      m31 = p3 * q1;
      m41 = p4 * q1;
      ell += lskip1;
      ex -= 4;
      Z11 += m11;
      Z21 += m21;
      Z31 += m31;
      Z41 += m41;
    }
    j += 4;
    for (; j > 0; j--) {
      p1 = ell[0];
      q1 = ex[0];
      p2 = ell[-1];
      p3 = ell[-2];
      p4 = ell[-3];
      m11 = p1 * q1;
      m21 = p2 * q1;
      m31 = p3 * q1;
      m41 = p4 * q1;
      ell += lskip1;
      ex -= 1;
      Z11 += m11;
      Z21 += m21;
      Z31 += m31;
      Z41 += m41;
    }
    Z11 = ex[0] - Z11;
    ex[0] = Z11;
    p1 = ell[-1];
    Z21 = ex[-1] - Z21 - p1 * Z11;
    ex[-1] = Z21;
    p1 = ell[-2];
    p2 = ell[-2 + lskip1];
    Z31 = ex[-2] - Z31 - p1 * Z11 - p2 * Z21;
    ex[-2] = Z31;
    p1 = ell[-3];
    p2 = ell[-3 + lskip1];
    p3 = ell[-3 + lskip2];
    Z41 = ex[-3] - Z41 - p1 * Z11 - p2 * Z21 - p3 * Z31;
    ex[-3] = Z41;
  }
  for (; i < n; i++) {
    Z11 = 0;
    ell = L - i;
    ex = B;
    for (j = i - 4; j >= 0; j -= 4) {
      p1 = ell[0];
      q1 = ex[0];
      m11 = p1 * q1;
      ell += lskip1;
      Z11 += m11;
      p1 = ell[0];
      q1 = ex[-1];
      m11 = p1 * q1;
      ell += lskip1;
      Z11 += m11;
      p1 = ell[0];
      q1 = ex[-2];
      m11 = p1 * q1;
      ell += lskip1;
      Z11 += m11;
      p1 = ell[0];
      q1 = ex[-3];
      m11 = p1 * q1;
      ell += lskip1;
      ex -= 4;
      Z11 += m11;
    }
    j += 4;
    for (; j > 0; j--) {
      p1 = ell[0];
      q1 = ex[0];
      m11 = p1 * q1;
      ell += lskip1;
      ex -= 1;
      Z11 += m11;
    }
    Z11 = ex[0] - Z11;
    ex[0] = Z11;
  }
}

#undef dSolveL1T

void dSolveL1T(const dReal* L, dReal* B, int n, int lskip1)
{
  _dSolveL1T(L, B, n, lskip1);
}

// No wrapper functions needed anymore - single API with optional tmpbuf
// parameters

// Explicit template instantiations for float and double
template void SetZero<float>(float* a, size_t n);
template void SetZero<double>(double* a, size_t n);

template void SetValue<float>(float* a, size_t n, float value);
template void SetValue<double>(double* a, size_t n, double value);

template float Dot<float>(const float* a, const float* b, int n);
template double Dot<double>(const double* a, const double* b, int n);

template void Multiply0<float>(
    float* A, const float* B, const float* C, int p, int q, int r);
template void Multiply0<double>(
    double* A, const double* B, const double* C, int p, int q, int r);

template void Multiply1<float>(
    float* A, const float* B, const float* C, int p, int q, int r);
template void Multiply1<double>(
    double* A, const double* B, const double* C, int p, int q, int r);

template void Multiply2<float>(
    float* A, const float* B, const float* C, int p, int q, int r);
template void Multiply2<double>(
    double* A, const double* B, const double* C, int p, int q, int r);

template void VectorScale<float>(float* a, const float* d, int n);
template void VectorScale<double>(double* a, const double* d, int n);

template void CopyVector<float>(float* dst, const float* src, int n);
template void CopyVector<double>(double* dst, const double* src, int n);

template void VectorAdd<float>(float* a, const float* b, int n);
template void VectorAdd<double>(double* a, const double* b, int n);

template void VectorSubtract<float>(float* a, const float* b, int n);
template void VectorSubtract<double>(double* a, const double* b, int n);

template void VectorNegate<float>(float* a, int n);
template void VectorNegate<double>(double* a, int n);

} // namespace dart::lcpsolver
