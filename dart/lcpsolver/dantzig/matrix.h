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

/// @file matrix.h
/// @brief Optimized vector and matrix operations for LCP solver

#pragma once

#include "dart/lcpsolver/dantzig/common.h"

#include <Eigen/Core>

#include <algorithm>

namespace dart::lcpsolver {

/// Get the dot product of two n×1 vectors
/// @param a First vector
/// @param b Second vector
/// @param n Vector size (if n <= 0, returns zero)
/// @return Dot product a·b
dReal dDot(const dReal* a, const dReal* b, int n);

/* get the dot products of (a0,b), (a1,b), etc and return them in outsum.
 * all vectors are n*1. if n <= 0 then zeroes will be returned (in which case
 * the input vectors need not be valid). this function is somewhat faster
 * than calling dDot() for all of the combinations separately.
 */

/* NOT INCLUDED in the library for now.
void dMultidot2 (const dReal *a0, const dReal *a1,
     const dReal *b, dReal *outsum, int n);
*/

/* matrix multiplication. all matrices are stored in standard row format.
 * the digit refers to the argument that is transposed:
 *   0:   A = B  * C   (sizes: A:p*r B:p*q C:q*r)
 *   1:   A = B' * C   (sizes: A:p*r B:q*p C:q*r)
 *   2:   A = B  * C'  (sizes: A:p*r B:p*q C:r*q)
 * case 1,2 are equivalent to saying that the operation is A=B*C but
 * B or C are stored in standard column format.
 */

void dMultiply0(dReal* A, const dReal* B, const dReal* C, int p, int q, int r);
void dMultiply1(dReal* A, const dReal* B, const dReal* C, int p, int q, int r);
void dMultiply2(dReal* A, const dReal* B, const dReal* C, int p, int q, int r);

/* do an in-place cholesky decomposition on the lower triangle of the n*n
 * symmetric matrix A (which is stored by rows). the resulting lower triangle
 * will be such that L*L'=A. return 1 on success and 0 on failure (on failure
 * the matrix is not positive definite).
 * @param tmpbuf Optional temporary buffer for performance (can be nullptr)
 */

int dFactorCholesky(dReal* A, int n, void* tmpbuf = nullptr);

/* solve for x: L*L'*x = b, and put the result back into x.
 * L is size n*n, b is size n*1. only the lower triangle of L is considered.
 * @param tmpbuf Optional temporary buffer for performance (can be nullptr)
 */

void dSolveCholesky(const dReal* L, dReal* b, int n, void* tmpbuf = nullptr);

/* compute the inverse of the n*n positive definite matrix A and put it in
 * Ainv. this is not especially fast. this returns 1 on success (A was
 * positive definite) or 0 on failure (not PD).
 * @param tmpbuf Optional temporary buffer for performance (can be nullptr)
 */

int dInvertPDMatrix(const dReal* A, dReal* Ainv, int n, void* tmpbuf = nullptr);

/* check whether an n*n matrix A is positive definite, return 1/0 (yes/no).
 * positive definite means that x'*A*x > 0 for any x. this performs a
 * cholesky decomposition of A. if the decomposition fails then the matrix
 * is not positive definite. A is stored by rows. A is not altered.
 * @param tmpbuf Optional temporary buffer for performance (can be nullptr)
 */

int dIsPositiveDefinite(const dReal* A, int n, void* tmpbuf = nullptr);

/* factorize a matrix A into L*D*L', where L is lower triangular with ones on
 * the diagonal, and D is diagonal.
 * A is an n*n matrix stored by rows, with a leading dimension of n rounded
 * up to 4. L is written into the strict lower triangle of A (the ones are not
 * written) and the reciprocal of the diagonal elements of D are written into
 * d.
 */
void dFactorLDLT(dReal* A, dReal* d, int n, int nskip);

/* solve L*x=b, where L is n*n lower triangular with ones on the diagonal,
 * and x,b are n*1. b is overwritten with x.
 * the leading dimension of L is `nskip'.
 */
void dSolveL1(const dReal* L, dReal* b, int n, int nskip);

/* solve L'*x=b, where L is n*n lower triangular with ones on the diagonal,
 * and x,b are n*1. b is overwritten with x.
 * the leading dimension of L is `nskip'.
 */
void dSolveL1T(const dReal* L, dReal* b, int n, int nskip);

/* in matlab syntax: a(1:n) = a(1:n) .* d(1:n) */

void dVectorScale(dReal* a, const dReal* d, int n);

/* given `L', a n*n lower triangular matrix with ones on the diagonal,
 * and `d', a n*1 vector of the reciprocal diagonal elements of an n*n matrix
 * D, solve L*D*L'*x=b where x,b are n*1. x overwrites b.
 * the leading dimension of L is `nskip'.
 */

void dSolveLDLT(const dReal* L, const dReal* d, dReal* b, int n, int nskip);

/* given an L*D*L' factorization of an n*n matrix A, return the updated
 * factorization L2*D2*L2' of A plus the following "top left" matrix:
 *
 *    [ b a' ]     <-- b is a[0]
 *    [ a 0  ]     <-- a is a[1..n-1]
 *
 *   - L has size n*n, its leading dimension is nskip. L is lower triangular
 *     with ones on the diagonal. only the lower triangle of L is referenced.
 *   - d has size n. d contains the reciprocal diagonal elements of D.
 *   - a has size n.
 * the result is written into L, except that the left column of L and d[0]
 * are not actually modified. see ldltaddTL.m for further comments.
 * @param tmpbuf Optional temporary buffer for performance (can be nullptr)
 */
void dLDLTAddTL(
    dReal* L,
    dReal* d,
    const dReal* a,
    int n,
    int nskip,
    void* tmpbuf = nullptr);

/* given an L*D*L' factorization of a permuted matrix A, produce a new
 * factorization for row and column `r' removed.
 *   - A has size n1*n1, its leading dimension in nskip. A is symmetric and
 *     positive definite. only the lower triangle of A is referenced.
 *     A itself may actually be an array of row pointers.
 *   - L has size n2*n2, its leading dimension in nskip. L is lower triangular
 *     with ones on the diagonal. only the lower triangle of L is referenced.
 *   - d has size n2. d contains the reciprocal diagonal elements of D.
 *   - p is a permutation vector. it contains n2 indexes into A. each index
 *     must be in the range 0..n1-1.
 *   - r is the row/column of L to remove.
 * the new L will be written within the old L, i.e. will have the same leading
 * dimension. the last row and column of L, and the last element of d, are
 * undefined on exit.
 *
 * a fast O(n^2) algorithm is used. see ldltremove.m for further comments.
 * @param tmpbuf Optional temporary buffer for performance (can be nullptr)
 */
void dLDLTRemove(
    dReal** A,
    const int* p,
    dReal* L,
    dReal* d,
    int n1,
    int n2,
    int r,
    int nskip,
    void* tmpbuf = nullptr);

/* given an n*n matrix A (with leading dimension nskip), remove the r'th row
 * and column by moving elements. the new matrix will have the same leading
 * dimension. the last row and column of A are untouched on exit.
 */
void dRemoveRowCol(dReal* A, int n, int nskip, int r);

/// Memory size estimation functions for temporary buffer allocation
inline constexpr size_t dEstimateFactorCholeskyTmpbufSize(int n)
{
  return padding(n) * sizeof(dReal);
}

inline constexpr size_t dEstimateSolveCholeskyTmpbufSize(int n)
{
  return padding(n) * sizeof(dReal);
}

inline constexpr size_t dEstimateInvertPDMatrixTmpbufSize(int n)
{
  size_t FactorCholesky_size = dEstimateFactorCholeskyTmpbufSize(n);
  size_t SolveCholesky_size = dEstimateSolveCholeskyTmpbufSize(n);
  size_t MaxCholesky_size = FactorCholesky_size > SolveCholesky_size
                                ? FactorCholesky_size
                                : SolveCholesky_size;
  return padding(n) * (n + 1) * sizeof(dReal) + MaxCholesky_size;
}

inline constexpr size_t dEstimateIsPositiveDefiniteTmpbufSize(int n)
{
  return padding(n) * n * sizeof(dReal) + dEstimateFactorCholeskyTmpbufSize(n);
}

inline constexpr size_t dEstimateLDLTAddTLTmpbufSize(int nskip)
{
  return nskip * 2 * sizeof(dReal);
}

inline constexpr size_t dEstimateLDLTRemoveTmpbufSize(int n2, int nskip)
{
  return n2 * sizeof(dReal) + dEstimateLDLTAddTLTmpbufSize(nskip);
}

//==============================================================================
// Template implementations for type-safe matrix operations
//==============================================================================

/// Set a vector/matrix of size n to all zeros (template version)
/// @param a Pointer to array
/// @param n Number of elements
template <typename Scalar>
inline void SetZero(Scalar* a, size_t n)
{
  DART_ASSERT(a);
  std::fill(a, a + n, Scalar(0));
}

/// Set an Eigen vector/matrix to all zeros (Eigen version)
/// @param a Eigen matrix/vector
template <typename Derived>
inline void SetZero(Eigen::MatrixBase<Derived>& a)
{
  a.setZero();
}

/// Set a vector/matrix of size n to a specific value (template version)
/// @param a Pointer to array
/// @param n Number of elements
/// @param value Value to set
template <typename Scalar>
inline void SetValue(Scalar* a, size_t n, Scalar value)
{
  DART_ASSERT(a);
  std::fill(a, a + n, value);
}

/// Set an Eigen vector/matrix to a specific value (Eigen version)
/// @param a Eigen matrix/vector
/// @param value Value to set
template <typename Derived>
inline void SetValue(
    Eigen::MatrixBase<Derived>& a, typename Derived::Scalar value)
{
  a.setConstant(value);
}

/// Get the dot product of two vectors (Eigen version)
/// @param a First vector
/// @param b Second vector
/// @return Dot product a·b
template <typename Derived1, typename Derived2>
inline typename Derived1::Scalar Dot(
    const Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b)
{
  return a.dot(b);
}

/// @deprecated Use Eigen version - will be removed
/// Get the dot product of two n×1 vectors (pointer version)
template <typename Scalar>
inline Scalar Dot(const Scalar* a, const Scalar* b, int n)
{
  DART_ASSERT(a && b);
  Scalar sum = Scalar(0);
  for (int i = 0; i < n; ++i) {
    sum += a[i] * b[i];
  }
  return sum;
}

/// Matrix multiplication: A = B * C (template version)
/// @param A Output matrix (p×r)
/// @param B Input matrix (p×q)
/// @param C Input matrix (q×r)
/// @param p Number of rows in B
/// @param q Number of columns in B / rows in C
/// @param r Number of columns in C
template <typename Scalar>
inline void Multiply0(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  DART_ASSERT(A && B && C && p > 0 && q > 0 && r > 0);
  const int qskip = padding(q);
  const int rskip = padding(r);
  Scalar* aa = A;
  const Scalar* bb = B;
  for (int i = p; i; aa += rskip, bb += qskip, --i) {
    Scalar* a = aa;
    const Scalar *cc = C, *ccend = C + r;
    for (; cc != ccend; ++a, ++cc) {
      Scalar sum = Scalar(0);
      const Scalar* c = cc;
      const Scalar *b = bb, *bend = bb + q;
      for (; b != bend; c += rskip, ++b) {
        sum += (*b) * (*c);
      }
      (*a) = sum;
    }
  }
}

/// Matrix multiplication: A = B' * C (template version)
/// B is transposed
/// @param A Output matrix (p×r)
/// @param B Input matrix stored in column format (q×p)
/// @param C Input matrix (q×r)
/// @param p Number of columns in B (rows in output)
/// @param q Number of rows in B / rows in C
/// @param r Number of columns in C
template <typename Scalar>
inline void Multiply1(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  DART_ASSERT(A && B && C && p > 0 && q > 0 && r > 0);
  const int pskip = padding(p);
  const int rskip = padding(r);
  Scalar* aa = A;
  const Scalar *bb = B, *bbend = B + p;
  for (; bb != bbend; aa += rskip, ++bb) {
    Scalar* a = aa;
    const Scalar *cc = C, *ccend = C + r;
    for (; cc != ccend; ++a, ++cc) {
      Scalar sum = Scalar(0);
      const Scalar *b = bb, *c = cc;
      for (int k = q; k; b += pskip, c += rskip, --k) {
        sum += (*b) * (*c);
      }
      (*a) = sum;
    }
  }
}

/// Matrix multiplication: A = B * C' (template version)
/// C is transposed
/// @param A Output matrix (p×r)
/// @param B Input matrix (p×q)
/// @param C Input matrix stored in column format (r×q)
/// @param p Number of rows in B
/// @param q Number of columns in B / columns in C
/// @param r Number of rows in C (columns in output)
template <typename Scalar>
inline void Multiply2(
    Scalar* A, const Scalar* B, const Scalar* C, int p, int q, int r)
{
  DART_ASSERT(A && B && C && p > 0 && q > 0 && r > 0);
  const int rskip = padding(r);
  const int qskip = padding(q);
  Scalar* aa = A;
  const Scalar* bb = B;
  for (int i = p; i; aa += rskip, bb += qskip, --i) {
    Scalar *a = aa, *aend = aa + r;
    const Scalar* cc = C;
    for (; a != aend; cc += qskip, ++a) {
      Scalar sum = Scalar(0);
      const Scalar *b = bb, *c = cc, *cend = cc + q;
      for (; c != cend; ++b, ++c) {
        sum += (*b) * (*c);
      }
      (*a) = sum;
    }
  }
}

/// Element-wise vector scaling: a[i] = a[i] * d[i] (Eigen version)
/// @param a Vector to scale (modified in place)
/// @param d Scaling factors
template <typename Derived1, typename Derived2>
inline void VectorScale(
    Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& d)
{
  a.array() *= d.array();
}

/// @deprecated Use Eigen version - will be removed
/// Element-wise vector scaling: a[i] = a[i] * d[i] (pointer version)
template <typename Scalar>
inline void VectorScale(Scalar* a, const Scalar* d, int n)
{
  DART_ASSERT(a && d && n >= 0);
  for (int i = 0; i < n; ++i) {
    a[i] *= d[i];
  }
}

/// Copy vector src to dst (Eigen version)
/// @param dst Destination vector
/// @param src Source vector
template <typename Derived1, typename Derived2>
inline void CopyVector(
    Eigen::MatrixBase<Derived1>& dst, const Eigen::MatrixBase<Derived2>& src)
{
  dst = src;
}

/// @deprecated Use Eigen version - will be removed
/// Copy n elements from src to dst (pointer version)
template <typename Scalar>
inline void CopyVector(Scalar* dst, const Scalar* src, int n)
{
  DART_ASSERT(dst && src && n >= 0);
  for (int i = 0; i < n; ++i) {
    dst[i] = src[i];
  }
}

/// Add vector b to vector a: a = a + b (Eigen version)
/// @param a First vector (modified in place)
/// @param b Second vector
template <typename Derived1, typename Derived2>
inline void VectorAdd(
    Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b)
{
  a += b;
}

/// @deprecated Use Eigen version - will be removed
/// Add vector b to vector a: a = a + b (pointer version)
template <typename Scalar>
inline void VectorAdd(Scalar* a, const Scalar* b, int n)
{
  DART_ASSERT(a && b && n >= 0);
  for (int i = 0; i < n; ++i) {
    a[i] += b[i];
  }
}

/// Subtract vector b from vector a: a = a - b (Eigen version)
/// @param a First vector (modified in place)
/// @param b Second vector
template <typename Derived1, typename Derived2>
inline void VectorSubtract(
    Eigen::MatrixBase<Derived1>& a, const Eigen::MatrixBase<Derived2>& b)
{
  a -= b;
}

/// @deprecated Use Eigen version - will be removed
/// Subtract vector b from vector a: a = a - b (pointer version)
template <typename Scalar>
inline void VectorSubtract(Scalar* a, const Scalar* b, int n)
{
  DART_ASSERT(a && b && n >= 0);
  for (int i = 0; i < n; ++i) {
    a[i] -= b[i];
  }
}

/// Negate vector: a = -a (Eigen version)
/// @param a Vector to negate (modified in place)
template <typename Derived>
inline void VectorNegate(Eigen::MatrixBase<Derived>& a)
{
  a = -a;
}

/// @deprecated Use Eigen version - will be removed
/// Negate vector: a = -a (pointer version)
template <typename Scalar>
inline void VectorNegate(Scalar* a, int n)
{
  DART_ASSERT(a && n >= 0);
  for (int i = 0; i < n; ++i) {
    a[i] = -a[i];
  }
}

/// Factorize matrix as L*D*L' (template version - currently calls dReal
/// version)
/// @param A Matrix to factorize (n×n), modified in place
/// @param d Output diagonal reciprocal elements
/// @param n Matrix size
/// @param nskip Leading dimension of A
template <typename Scalar>
inline void FactorLDLT(Scalar* A, Scalar* d, int n, int nskip)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "FactorLDLT currently only supports dReal type");
  dFactorLDLT(A, d, n, nskip);
}

/// Solve L*x=b where L is lower triangular (template version - currently calls
/// dReal version)
/// @param L Lower triangular matrix (n×n) with ones on diagonal
/// @param b Right-hand side vector (modified in place with solution)
/// @param n Matrix size
/// @param nskip Leading dimension of L
template <typename Scalar>
inline void SolveL1(const Scalar* L, Scalar* b, int n, int nskip)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "SolveL1 currently only supports dReal type");
  dSolveL1(L, b, n, nskip);
}

/// Solve L'*x=b where L is lower triangular (template version - currently calls
/// dReal version)
/// @param L Lower triangular matrix (n×n) with ones on diagonal
/// @param b Right-hand side vector (modified in place with solution)
/// @param n Matrix size
/// @param nskip Leading dimension of L
template <typename Scalar>
inline void SolveL1T(const Scalar* L, Scalar* b, int n, int nskip)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "SolveL1T currently only supports dReal type");
  dSolveL1T(L, b, n, nskip);
}

/// Solve L*D*L'*x=b (template version - currently calls dReal version)
/// @param L Lower triangular matrix (n×n) with ones on diagonal
/// @param d Diagonal reciprocal elements
/// @param b Right-hand side vector (modified in place with solution)
/// @param n Matrix size
/// @param nskip Leading dimension of L
template <typename Scalar>
inline void SolveLDLT(
    const Scalar* L, const Scalar* d, Scalar* b, int n, int nskip)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "SolveLDLT currently only supports dReal type");
  dSolveLDLT(L, d, b, n, nskip);
}

/// Add top-left matrix to L*D*L' factorization (template version - currently
/// calls dReal version)
/// @param L Lower triangular matrix factor (modified in place)
/// @param d Diagonal reciprocal elements (modified in place)
/// @param a Input vector for top-left addition
/// @param n Matrix size
/// @param nskip Leading dimension of L
/// @param tmpbuf Optional temporary buffer (can be nullptr)
template <typename Scalar>
inline void LDLTAddTL(
    Scalar* L,
    Scalar* d,
    const Scalar* a,
    int n,
    int nskip,
    void* tmpbuf = nullptr)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "LDLTAddTL currently only supports dReal type");
  dLDLTAddTL(L, d, a, n, nskip, tmpbuf);
}

/// Remove row/column from L*D*L' factorization (template version - currently
/// calls dReal version)
/// @param A Original matrix (array of row pointers)
/// @param p Permutation vector
/// @param L Lower triangular factor (modified in place)
/// @param d Diagonal reciprocal elements (modified in place)
/// @param n1 Size of original matrix A
/// @param n2 Current size of factorization
/// @param r Row/column to remove
/// @param nskip Leading dimension
/// @param tmpbuf Optional temporary buffer (can be nullptr)
template <typename Scalar>
inline void LDLTRemove(
    Scalar** A,
    const int* p,
    Scalar* L,
    Scalar* d,
    int n1,
    int n2,
    int r,
    int nskip,
    void* tmpbuf = nullptr)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "LDLTRemove currently only supports dReal type");
  dLDLTRemove(A, p, L, d, n1, n2, r, nskip, tmpbuf);
}

/// Remove row/column from matrix (template version - currently calls dReal
/// version)
/// @param A Matrix to modify (n×n)
/// @param n Matrix size
/// @param nskip Leading dimension
/// @param r Row/column to remove
template <typename Scalar>
inline void RemoveRowCol(Scalar* A, int n, int nskip, int r)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "RemoveRowCol currently only supports dReal type");
  dRemoveRowCol(A, n, nskip, r);
}

/// Cholesky factorization (template version - currently calls dReal version)
/// @param A Matrix to factorize (n×n), modified in place with L
/// @param n Matrix size
/// @param tmpbuf Optional temporary buffer (can be nullptr)
/// @return 1 on success, 0 on failure (not positive definite)
template <typename Scalar>
inline int FactorCholesky(Scalar* A, int n, void* tmpbuf = nullptr)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "FactorCholesky currently only supports dReal type");
  return dFactorCholesky(A, n, tmpbuf);
}

/// Solve L*L'*x=b using Cholesky factor (template version - currently calls
/// dReal version)
/// @param L Lower triangular Cholesky factor (n×n)
/// @param b Right-hand side vector (modified in place with solution)
/// @param n Matrix size
/// @param tmpbuf Optional temporary buffer (can be nullptr)
template <typename Scalar>
inline void SolveCholesky(
    const Scalar* L, Scalar* b, int n, void* tmpbuf = nullptr)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "SolveCholesky currently only supports dReal type");
  dSolveCholesky(L, b, n, tmpbuf);
}

/// Invert positive definite matrix (template version - currently calls dReal
/// version)
/// @param A Input matrix (n×n)
/// @param Ainv Output inverse matrix (n×n)
/// @param n Matrix size
/// @param tmpbuf Optional temporary buffer (can be nullptr)
/// @return 1 on success, 0 on failure (not positive definite)
template <typename Scalar>
inline int InvertPDMatrix(
    const Scalar* A, Scalar* Ainv, int n, void* tmpbuf = nullptr)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "InvertPDMatrix currently only supports dReal type");
  return dInvertPDMatrix(A, Ainv, n, tmpbuf);
}

/// Check if matrix is positive definite (template version - currently calls
/// dReal version)
/// @param A Input matrix (n×n), not modified
/// @param n Matrix size
/// @param tmpbuf Optional temporary buffer (can be nullptr)
/// @return 1 if positive definite, 0 otherwise
template <typename Scalar>
inline int IsPositiveDefinite(const Scalar* A, int n, void* tmpbuf = nullptr)
{
  static_assert(
      std::is_same<Scalar, dReal>::value,
      "IsPositiveDefinite currently only supports dReal type");
  return dIsPositiveDefinite(A, n, tmpbuf);
}

} // namespace dart::lcpsolver
