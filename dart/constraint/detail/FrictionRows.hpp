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

#ifndef DART_CONSTRAINT_DETAIL_FRICTIONROWS_HPP_
#define DART_CONSTRAINT_DETAIL_FRICTIONROWS_HPP_

#include <dart/constraint/detail/FrictionCone.hpp>

#include <array>
#include <limits>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::constraint::detail {

/// Internal row description; no API stability promise. Indices refer to the
/// group's original, read-only LCP arrays (A x = b + w).
struct FrictionContactRows
{
  int normalRow = -1;
  std::array<int, 2> tangentRows{{-1, -1}};
  FrictionCone cone;
  std::size_t rowOffset = 0;
  std::size_t rowCount = 0;
  /// Malformed coupled components retain all rows and use their original
  /// lo/hi/findex with normal-first PGS semantics, rather than a cone QP.
  bool usePgs = false;
};

/// Caller-owned scratch: capacity is reused on subsequent classifications.
/// contactRows stores every row of each component in ascending row order;
/// scalarRows and the contact ranges partition the input without omissions.
struct FrictionRowClassification
{
  std::vector<FrictionContactRows> contacts;
  std::vector<int> contactRows;
  std::vector<int> scalarRows;
  std::size_t numBoxContacts = 0;

  // Linear-time graph traversal scratch; no unordered iteration or recursion.
  std::vector<int> parents;
  std::vector<int> roots;
  std::vector<int> path;
  std::vector<int> componentContacts;
  std::vector<std::size_t> componentSizes;
};

/// Classify clean normal/tangent stars, and retain all other coupled components
/// as PGS boxes. Accept findex=-1 and self indices for uncoupled rows. Invalid
/// indices return false; callers must reject that problem rather than omit
/// rows. With boxForAnisotropic, |mu1-mu2| > 1e-9 max(mu1,mu2) selects the box
/// law. This changes capacity by up to sqrt(2) at 45 degrees across the switch.
/// A minor semi-axis <= 1e-6 of the major is set to zero (1-D friction).
/// Reserving the output/scratch vectors for n rows avoids all later allocation.
bool classifyFrictionRows(
    int n,
    const double* lo,
    const double* hi,
    const int* findex,
    FrictionRowClassification& result,
    bool boxForAnisotropic = true);

/// w_row = sum_{j<n} A(row, j) x_j - b_row (row-major, stride padded).
inline double rowVelocity(
    int row,
    int n,
    int stride,
    const double* A,
    const double* x,
    const double* b)
{
  double velocity = -b[row];
  for (int j = 0; j < n; ++j)
    velocity += A[std::size_t(row) * stride + j] * x[j];
  return velocity;
}

/// PGS bounds, including coupling to the current normal. Keeps PGS's clamp
/// order; a NaN bound returns NaN.
inline double projectRow(
    int row,
    double value,
    const double* x,
    const double* lo,
    const double* hi,
    const int* findex)
{
  double lower = lo[row];
  double upper = hi[row];
  if (findex[row] >= 0 && findex[row] != row) {
    upper = hi[row] * x[findex[row]];
    lower = -upper;
  }
  if (std::isnan(lower) || std::isnan(upper))
    return std::numeric_limits<double>::quiet_NaN();
  // Preserve PGS's clamp order even for a malformed coupled component.
  return value > upper ? upper : value < lower ? lower : value;
}

/// Diagonal-scaled natural-map violation; a non-positive diagonal uses 1.
/// A non-finite velocity or result returns infinity.
inline double scalarViolation(
    int row,
    double velocity,
    double diagonal,
    const double* x,
    const double* lo,
    const double* hi,
    const int* findex)
{
  if (!std::isfinite(velocity))
    return std::numeric_limits<double>::infinity();
  // A zero row still has complementarity conditions; a unit step tests those
  // without dividing by zero. Positive diagonals use the specified scaling.
  const double scale = diagonal > 0.0 ? diagonal : 1.0;
  const double argument = x[row] - velocity / scale;
  if (!std::isfinite(argument))
    return std::numeric_limits<double>::infinity();
  const double violation
      = scale * std::abs(x[row] - projectRow(row, argument, x, lo, hi, findex));
  return std::isfinite(violation) ? violation
                                  : std::numeric_limits<double>::infinity();
}

inline Eigen::Vector3d contactImpulse(
    const FrictionContactRows& contact, const double* x)
{
  Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
  impulse[0] = x[contact.normalRow];
  for (int axis = 0; axis < 2; ++axis)
    if (contact.tangentRows[axis] >= 0)
      impulse[axis + 1] = x[contact.tangentRows[axis]];
  return impulse;
}

inline std::array<int, 3> contactIndices(const FrictionContactRows& contact)
{
  return {{contact.normalRow, contact.tangentRows[0], contact.tangentRows[1]}};
}

/// Project scalar rows and each contact in place. PGS components project the
/// normal first; cone contacts use Euclidean projection only when infeasible.
/// Adds local QP fallbacks; false on non-finite or uncertified results.
bool projectIterate(
    double* x,
    const double* lo,
    const double* hi,
    const int* findex,
    const FrictionRowClassification& classification,
    std::uint64_t& numLocalFallbacks);

/// Largest law violation on the assembled LCP [m/s]. Cone contacts use their
/// largest positive diagonal, or 1. If non-null, velocities receives every
/// row velocity.
double lawViolation(
    int n,
    int stride,
    const double* A,
    const double* x,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    const FrictionRowClassification& classification,
    bool associated,
    double* velocities = nullptr);

/// FNV-1a hash of the unpadded A entries, b, lo, hi and findex.
std::uint64_t lcpFingerprint(
    int n,
    int stride,
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex);

/// Finite LCP matrix, symmetric within 1e-10 relative, nonnegative diagonal.
bool canSolveFrictionLcp(int n, const double* A);

} // namespace dart::constraint::detail

#endif // DART_CONSTRAINT_DETAIL_FRICTIONROWS_HPP_
