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

#pragma once

#include "dart/common/macros.hpp"
#include "dart/math/lcp/pivoting/dantzig/common.hpp"

#include <vector>

namespace dart::math {

template <typename Scalar>
void dMultiply0(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  Multiply0(A, B, C, p, q, r);
}

template <typename Scalar>
void dMultiply1(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  Multiply1(A, B, C, p, q, r);
}

template <typename Scalar>
void dMultiply2(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  Multiply2(A, B, C, p, q, r);
}

template <typename Scalar>
int dFactorCholesky(Scalar* A, int n, void* tmpbuf /*[n]*/)
{
  DART_ASSERT(n > 0 && A);
  bool failure = false;
  const int nskip = padding(n);
  // Use heap allocation instead of alloca() to avoid stack corruption on arm64
  std::vector<Scalar> recip_storage;
  Scalar* recip;
  if (tmpbuf) {
    recip = static_cast<Scalar*>(tmpbuf);
  } else {
    recip_storage.resize(n);
    recip = recip_storage.data();
  }
  Scalar* aa = A;
  for (int i = 0; i < n; aa += nskip, ++i) {
    Scalar* cc = aa;
    {
      const Scalar* bb = A;
      for (int j = 0; j < i; bb += nskip, ++cc, ++j) {
        Scalar sum = *cc;
        const Scalar *a = aa, *b = bb, *bend = bb + j;
        for (; b != bend; ++a, ++b) {
          sum -= (*a) * (*b);
        }
        *cc = sum * recip[j];
      }
    }
    {
      Scalar sum = *cc;
      Scalar *a = aa, *aend = aa + i;
      for (; a != aend; ++a) {
        sum -= (*a) * (*a);
      }
      if (sum <= static_cast<Scalar>(0.0)) {
        failure = true;
        break;
      }
      Scalar sumsqrt = std::sqrt(sum);
      *cc = sumsqrt;
      recip[i] = reciprocal(sumsqrt);
    }
  }
  return failure ? 0 : 1;
}

template <typename Scalar>
void dSolveCholesky(const Scalar* L, Scalar* b, int n, void* tmpbuf /*[n]*/)
{
  DART_ASSERT(n > 0 && L && b);
  const int nskip = padding(n);
  std::vector<Scalar> y_storage;
  Scalar* y;
  if (tmpbuf) {
    y = static_cast<Scalar*>(tmpbuf);
  } else {
    y_storage.resize(n);
    y = y_storage.data();
  }
  {
    const Scalar* ll = L;
    for (int i = 0; i < n; ll += nskip, ++i) {
      Scalar sum = static_cast<Scalar>(0.0);
      for (int k = 0; k < i; ++k) {
        sum += ll[k] * y[k];
      }
      DART_ASSERT(ll[i] != Scalar(0.0));
      y[i] = (b[i] - sum) / ll[i];
    }
  }
  {
    const Scalar* ll = L + (n - 1) * (nskip + 1);
    for (int i = n - 1; i >= 0; ll -= nskip + 1, --i) {
      Scalar sum = static_cast<Scalar>(0.0);
      const Scalar* l = ll + nskip;
      for (int k = i + 1; k < n; l += nskip, ++k) {
        sum += (*l) * b[k];
      }
      DART_ASSERT(*ll != Scalar(0.0));
      b[i] = (y[i] - sum) / (*ll);
    }
  }
}

template <typename Scalar>
int dInvertPDMatrix(
    const Scalar* A, Scalar* Ainv, int n, void* tmpbuf /*[nskip*(n+2)]*/)
{
  DART_ASSERT(n > 0 && A && Ainv);
  bool success = false;
  size_t FactorCholesky_size = dEstimateFactorCholeskyTmpbufSize<Scalar>(n);
  size_t SolveCholesky_size = dEstimateSolveCholeskyTmpbufSize<Scalar>(n);
  size_t MaxCholesky_size = FactorCholesky_size > SolveCholesky_size
                                ? FactorCholesky_size
                                : SolveCholesky_size;
  DART_ASSERT(MaxCholesky_size % sizeof(Scalar) == 0);
  const int nskip = padding(n);
  const int nskip_mul_n = nskip * n;
  // Compute sizes in Scalar units for proper alignment on ARM64
  const size_t MaxCholesky_scalars = MaxCholesky_size / sizeof(Scalar);
  const size_t total_scalars = MaxCholesky_scalars + nskip + nskip_mul_n;
  std::vector<Scalar> tmp_storage;
  Scalar* tmp;
  if (tmpbuf) {
    tmp = static_cast<Scalar*>(tmpbuf);
  } else {
    tmp_storage.resize(total_scalars);
    tmp = tmp_storage.data();
  }
  Scalar* X = tmp + MaxCholesky_scalars;
  Scalar* L = X + nskip;
  memcpy(L, A, nskip_mul_n * sizeof(Scalar));
  if (dFactorCholesky(L, n, tmp)) {
    SetZero(Ainv, nskip_mul_n);
    Scalar *aa = Ainv, *xi = X, *xiend = X + n;
    for (; xi != xiend; ++aa, ++xi) {
      SetZero(X, n);
      *xi = static_cast<Scalar>(1.0);
      dSolveCholesky(L, X, n, tmp);
      Scalar* a = aa;
      const Scalar *x = X, *xend = X + n;
      for (; x != xend; a += nskip, ++x) {
        *a = *x;
      }
    }
    success = true;
  }
  return success ? 1 : 0;
}

template <typename Scalar>
int dIsPositiveDefinite(const Scalar* A, int n, void* tmpbuf /*[nskip*(n+1)]*/)
{
  DART_ASSERT(n > 0 && A);
  size_t FactorCholesky_size = dEstimateFactorCholeskyTmpbufSize<Scalar>(n);
  DART_ASSERT(FactorCholesky_size % sizeof(Scalar) == 0);
  const int nskip = padding(n);
  const int nskip_mul_n = nskip * n;
  const size_t FactorCholesky_scalars = FactorCholesky_size / sizeof(Scalar);
  const size_t total_scalars = FactorCholesky_scalars + nskip_mul_n;
  std::vector<Scalar> tmp_storage;
  Scalar* tmp;
  if (tmpbuf) {
    tmp = static_cast<Scalar*>(tmpbuf);
  } else {
    tmp_storage.resize(total_scalars);
    tmp = tmp_storage.data();
  }
  Scalar* Acopy = tmp + FactorCholesky_scalars;
  memcpy(Acopy, A, nskip_mul_n * sizeof(Scalar));
  return dFactorCholesky(Acopy, n, tmp);
}

template <typename Scalar>
void dVectorScale(Scalar* a, const Scalar* d, int n)
{
  VectorScale(a, d, n);
}

template <typename Scalar>
void dSolveLDLT(const Scalar* L, const Scalar* d, Scalar* b, int n, int nskip)
{
  SolveL1(L, b, n, nskip);
  for (int i = 0; i < n; ++i)
    b[i] *= d[i];
  SolveL1T(L, b, n, nskip);
}

template <typename Scalar>
void dLDLTAddTL(
    Scalar* L,
    Scalar* d,
    const Scalar* a,
    int n,
    int nskip,
    void* tmpbuf /*[2*nskip]*/)
{
  DART_ASSERT(L && d && a && n > 0 && nskip >= n);

  if (n < 2)
    return;
  std::vector<Scalar> W_storage;
  Scalar* W1;
  if (tmpbuf) {
    W1 = static_cast<Scalar*>(tmpbuf);
  } else {
    W_storage.resize(2 * nskip);
    W1 = W_storage.data();
  }
  Scalar* W2 = W1 + nskip;
  const Scalar sqrtHalf
      = ScalarTraits<Scalar>::reciprocalSqrt(static_cast<Scalar>(2));

  W1[0] = static_cast<Scalar>(0.0);
  W2[0] = static_cast<Scalar>(0.0);
  for (int j = 1; j < n; ++j) {
    W1[j] = W2[j] = static_cast<Scalar>(a[j] * sqrtHalf);
  }
  Scalar W11
      = static_cast<Scalar>((static_cast<Scalar>(0.5) * a[0] + 1) * sqrtHalf);
  Scalar W21
      = static_cast<Scalar>((static_cast<Scalar>(0.5) * a[0] - 1) * sqrtHalf);

  Scalar alpha1 = static_cast<Scalar>(1.0);
  Scalar alpha2 = static_cast<Scalar>(1.0);

  {
    Scalar dee = d[0];
    Scalar alphanew = alpha1 + (W11 * W11) * dee;
    DART_ASSERT(alphanew != Scalar(0.0));
    dee /= alphanew;
    Scalar gamma1 = W11 * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - (W21 * W21) * dee;
    dee /= alphanew;
    alpha2 = alphanew;
    Scalar k1 = static_cast<Scalar>(1.0) - W21 * gamma1;
    Scalar k2 = W21 * gamma1 * W11 - W21;
    Scalar* ll = L + nskip;
    for (int p = 1; p < n; ll += nskip, ++p) {
      Scalar Wp = W1[p];
      Scalar ell = *ll;
      W1[p] = Wp - W11 * ell;
      W2[p] = k1 * Wp + k2 * ell;
    }
  }

  Scalar* ll = L + (nskip + 1);
  for (int j = 1; j < n; ll += nskip + 1, ++j) {
    Scalar k1 = W1[j];
    Scalar k2 = W2[j];

    Scalar dee = d[j];
    Scalar alphanew = alpha1 + (k1 * k1) * dee;
    DART_ASSERT(alphanew != Scalar(0.0));
    dee /= alphanew;
    Scalar gamma1 = k1 * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - (k2 * k2) * dee;
    dee /= alphanew;
    Scalar gamma2 = k2 * dee;
    dee *= alpha2;
    d[j] = dee;
    alpha2 = alphanew;

    Scalar* l = ll + nskip;
    for (int p = j + 1; p < n; l += nskip, ++p) {
      Scalar ell = *l;
      Scalar Wp = W1[p] - k1 * ell;
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
// #define _GETA(i,j) (A[(i)*nskip+(j)])
#define GETA(i, j) ((i > j) ? _GETA(i, j) : _GETA(j, i))

template <typename Scalar>
void dLDLTRemove(
    Scalar** A,
    const int* p,
    Scalar* L,
    Scalar* d,
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
  DART_UNUSED(n1);

  if (r == n2 - 1) {
    return;
  } else {
    size_t LDLTAddTL_size = dEstimateLDLTAddTLTmpbufSize<Scalar>(nskip);
    DART_ASSERT(LDLTAddTL_size % sizeof(Scalar) == 0);
    const size_t LDLTAddTL_scalars = LDLTAddTL_size / sizeof(Scalar);
    const size_t total_scalars = LDLTAddTL_scalars + n2;
    std::vector<Scalar> tmp_storage;
    Scalar* tmp;
    if (tmpbuf) {
      tmp = static_cast<Scalar*>(tmpbuf);
    } else {
      tmp_storage.resize(total_scalars);
      tmp = tmp_storage.data();
    }
    if (r == 0) {
      Scalar* a = tmp + LDLTAddTL_scalars;
      const int p_0 = p[0];
      for (int i = 0; i < n2; ++i) {
        a[i] = -GETA(p[i], p_0);
      }
      a[0] += static_cast<Scalar>(1.0);
      dLDLTAddTL(L, d, a, n2, nskip, tmp);
    } else {
      Scalar* t = tmp + LDLTAddTL_scalars;
      {
        Scalar* Lcurr = L + r * nskip;
        for (int i = 0; i < r; ++Lcurr, ++i) {
          DART_ASSERT(d[i] != Scalar(0.0));
          t[i] = *Lcurr / d[i];
        }
      }
      Scalar* a = t + r;
      {
        Scalar* Lcurr = L + r * nskip;
        const int *pp_r = p + r, p_r = *pp_r;
        const int n2_minus_r = n2 - r;
        for (int i = 0; i < n2_minus_r; Lcurr += nskip, ++i) {
          a[i] = Dot(Lcurr, t, r) - GETA(pp_r[i], p_r);
        }
      }
      a[0] += static_cast<Scalar>(1.0);
      dLDLTAddTL(L + r * nskip + r, d + r, a, n2 - r, nskip, tmp);
    }
  }

  RemoveRowCol(L, n2, nskip, r);
  if (r < (n2 - 1))
    memmove(d + r, d + r + 1, (n2 - r - 1) * sizeof(Scalar));
}

template <typename Scalar>
void dRemoveRowCol(Scalar* A, int n, int nskip, int r)
{
  DART_ASSERT(A && n > 0 && nskip >= n && r >= 0 && r < n);
  if (r >= n - 1)
    return;
  if (r > 0) {
    {
      const size_t move_size = (n - r - 1) * sizeof(Scalar);
      Scalar* Adst = A + r;
      for (int i = 0; i < r; Adst += nskip, ++i) {
        Scalar* Asrc = Adst + 1;
        memmove(Adst, Asrc, move_size);
      }
    }
    {
      const size_t cpy_size = r * sizeof(Scalar);
      Scalar* Adst = A + r * nskip;
      for (int i = r; i < (n - 1); ++i) {
        Scalar* Asrc = Adst + nskip;
        memcpy(Adst, Asrc, cpy_size);
        Adst = Asrc;
      }
    }
  }
  {
    const size_t cpy_size = (n - r - 1) * sizeof(Scalar);
    Scalar* Adst = A + r * (nskip + 1);
    for (int i = r; i < (n - 1); ++i) {
      Scalar* Asrc = Adst + (nskip + 1);
      memcpy(Adst, Asrc, cpy_size);
      Adst = Asrc - 1;
    }
  }
}

//==============================================================================
// Fast optimized implementations (originally in fast*.cpp files)
//==============================================================================

// From fastdot.cpp
template <typename Scalar>
Scalar _dDot(const Scalar* a, const Scalar* b, int n)
{
  Scalar p0, q0, m0, p1, q1, m1, sum;
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

// Phase 11: Eigen-optimized dot product (uses SIMD)
// Benchmarking shows this can be faster for n >= 10-20
template <typename Scalar>
Scalar _DotEigen(const Scalar* a, const Scalar* b, int n)
{
  Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> va(a, n);
  Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> vb(b, n);
  return va.dot(vb);
}

#undef dDot

// Phase 13: Fully templated dot product implementation
// Select implementation based on size threshold
// Small vectors: use original (better for tiny sizes)
// Large vectors: use Eigen SIMD (better for n >= threshold)
template <typename Scalar>
Scalar Dot(const Scalar* a, const Scalar* b, int n)
{
  // Threshold determined by benchmarking: Eigen wins for n >= 20
  // For smaller sizes, the original hand-rolled code is competitive
  constexpr int EIGEN_THRESHOLD = 20;

  if (n >= EIGEN_THRESHOLD) {
    return _DotEigen<Scalar>(a, b, n);
  } else {
    return _dDot(a, b, n);
  }
}

//==============================================================================
// Hand-optimized LDLT factorization and triangular solvers from ODE
//==============================================================================
// These are performance-critical paths (~30-50% of solve time) with:
// - Loop unrolling (2×2, 4×4, 6×6, 12×12 blocks)
// - Cache-optimized memory access patterns
// - Carefully tuned for modern CPUs
//
// Migrated from ODE baseline in Phase 13.5:
// - fastldlt.cpp → _dFactorLDLT
// - fastlsolve.cpp → _dSolveL1
// - fastltsolve.cpp → _dSolveL1T
//==============================================================================

// Helper function for _dFactorLDLT (solves L*X=B with 1 RHS, 2×2 blocks)
template <typename Scalar>
static void dSolveL1_1(const Scalar* L, Scalar* B, int n, int lskip1)
{
  Scalar Z11, m11, Z21, m21, p1, q1, p2, *ex;
  const Scalar* ell;
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

// Helper function for _dFactorLDLT (solves L*X=B with 2 RHS, 2×2 blocks)
template <typename Scalar>
static void dSolveL1_2(const Scalar* L, Scalar* B, int n, int lskip1)
{
  Scalar Z11, m11, Z12, m12, Z21, m21, Z22, m22, p1, q1, p2, q2, *ex;
  const Scalar* ell;
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

// LDLT factorization with 2×2 and 6×6 blocking for cache efficiency
template <typename Scalar>
void _dFactorLDLT(Scalar* A, Scalar* d, int n, int nskip1)
{
  int i, j;
  Scalar sum, *ell, *dee, dd, p1, p2, q1, q2, Z11, m11, Z21, m21, Z22, m22;
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

// Solve L*X=B with 4×4 and 12×12 blocking (L is lower triangular with 1s on
// diagonal)
template <typename Scalar>
void _dSolveL1(const Scalar* L, Scalar* B, int n, int lskip1)
{
  Scalar Z11, Z21, Z31, Z41, p1, q1, p2, p3, p4, *ex;
  const Scalar* ell;
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

// Solve L^T * X=B with 4×4 blocking (L transpose, lower triangular with 1s on
// diagonal)
template <typename Scalar>
void _dSolveL1T(const Scalar* L, Scalar* B, int n, int lskip1)
{
  Scalar Z11, m11, Z21, m21, Z31, m31, Z41, m41, p1, q1, p2, p3, p4, *ex;
  const Scalar* ell;
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

//==============================================================================
// Non-template wrappers for backward compatibility
//==============================================================================

template <typename Scalar>
void dFactorLDLT(Scalar* A, Scalar* d, int n, int nskip)
{
  _dFactorLDLT(A, d, n, nskip);
}

template <typename Scalar>
void dSolveL1(const Scalar* L, Scalar* b, int n, int nskip)
{
  _dSolveL1(L, b, n, nskip);
}

template <typename Scalar>
void dSolveL1T(const Scalar* L, Scalar* b, int n, int nskip)
{
  _dSolveL1T(L, b, n, nskip);
}

//==============================================================================
} // namespace dart::math
