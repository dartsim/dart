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

#include <dart/constraint/detail/FrictionRows.hpp>

#include <dart/lcpsolver/dantzig/DantzigCommon.hpp>

#include <algorithm>

#include <cmath>
#include <cstring>

namespace dart::constraint::detail {

//==============================================================================
bool classifyFrictionRows(
    int n,
    const double* lo,
    const double* hi,
    const int* findex,
    FrictionRowClassification& result,
    bool boxForAnisotropic)
{
  result.contacts.clear();
  result.contactRows.clear();
  result.scalarRows.clear();
  result.numBoxContacts = 0;
  if (n < 0 || (n > 0 && (!lo || !hi || !findex)))
    return false;

  result.parents.resize(n);
  result.roots.assign(n, -1);
  result.componentContacts.assign(n, -1);
  result.componentSizes.assign(n, 0);
  result.path.clear();
  for (int i = 0; i < n; ++i) {
    if (findex[i] < -1 || findex[i] >= n)
      return false;
    result.parents[i] = findex[i] == i ? -1 : findex[i];
  }

  // Each row has at most one parent. Walk each edge once, including cycles;
  // assigning the completed path also absorbs chains into their component.
  for (int i = 0; i < n; ++i) {
    if (result.roots[i] >= 0)
      continue;
    int row = i;
    result.path.clear();
    while (result.roots[row] == -1) {
      result.roots[row] = -2;
      result.path.push_back(row);
      if (result.parents[row] < 0)
        break;
      row = result.parents[row];
    }
    const int root = result.roots[row] >= 0 ? result.roots[row] : row;
    for (int visited : result.path)
      result.roots[visited] = root;
  }
  for (int root : result.roots)
    ++result.componentSizes[root];

  // Components are emitted in first-row order. A non-normal root, chain,
  // cycle, finite cap, asymmetric bound or >2 tangents goes to PGS intact.
  std::size_t totalRows = 0;
  for (int i = 0; i < n; ++i) {
    const int root = result.roots[i];
    if (result.componentSizes[root] == 1) {
      result.scalarRows.push_back(i);
      continue;
    }
    if (result.componentContacts[root] >= 0)
      continue;
    FrictionContactRows contact;
    contact.normalRow = root;
    contact.rowOffset = totalRows;
    contact.rowCount = result.componentSizes[root];
    contact.usePgs = contact.rowCount > 3 || result.parents[root] >= 0
                     || lo[root] != 0 || !std::isinf(hi[root]) || hi[root] < 0;
    totalRows += contact.rowCount;
    result.componentContacts[root] = static_cast<int>(result.contacts.size());
    result.contacts.push_back(contact);
  }
  result.contactRows.resize(totalRows);
  // Reuse componentSizes as write cursors after sizes have been recorded.
  std::fill(result.componentSizes.begin(), result.componentSizes.end(), 0);
  for (int i = 0; i < n; ++i) {
    const int root = result.roots[i];
    const int index = result.componentContacts[root];
    if (index < 0)
      continue;
    auto& contact = result.contacts[index];
    const std::size_t cursor = result.componentSizes[root]++;
    result.contactRows[contact.rowOffset + cursor] = i;
    if (i == root)
      continue;
    // Coefficients the cone laws cannot represent stay with PGS.
    const bool cleanTangent = result.parents[i] == root && std::isfinite(hi[i])
                              && hi[i] <= kMaxFrictionCoefficient && hi[i] >= 0
                              && lo[i] == -hi[i];
    contact.usePgs |= !cleanTangent;
    // Tangent axes follow their row order even if the normal comes later.
    const int axis = contact.tangentRows[0] < 0 ? 0 : 1;
    if (contact.tangentRows[axis] < 0) {
      contact.tangentRows[axis] = i;
      contact.cone.mu[axis] = hi[i];
    }
  }
  for (auto& contact : result.contacts) {
    if (contact.rowCount == 2)
      contact.cone.mu[1] = 0;
    const double major = contact.cone.mu.maxCoeff();
    const double minor = contact.cone.mu.minCoeff();
    const double anisotropyThreshold = 1e-9 * major;
    const double reductionThreshold = 1e-6 * major;
    // An unrepresentable threshold must not change the friction law or axes.
    contact.usePgs
        |= !std::isfinite(anisotropyThreshold)
           || !std::isfinite(reductionThreshold)
           || (major > 0
               && (anisotropyThreshold == 0 || reductionThreshold == 0));
    contact.cone.law
        = contact.usePgs
                  || (boxForAnisotropic && major - minor > anisotropyThreshold)
              ? FrictionConeLaw::Box
              : FrictionConeLaw::Ellipse;
    if (!contact.usePgs && minor <= reductionThreshold) {
      if (contact.cone.mu[0] <= contact.cone.mu[1])
        contact.cone.mu[0] = 0;
      else
        contact.cone.mu[1] = 0;
    }
    if (contact.cone.law == FrictionConeLaw::Box)
      ++result.numBoxContacts;
  }
  return true;
}

//==============================================================================
bool projectIterate(
    double* x,
    const double* lo,
    const double* hi,
    const int* findex,
    const FrictionRowClassification& classification,
    std::uint64_t& numLocalFallbacks)
{
  for (int row : classification.scalarRows) {
    x[row] = projectRow(row, x[row], x, lo, hi, findex);
    if (!std::isfinite(x[row]))
      return false;
  }
  for (const auto& contact : classification.contacts) {
    if (contact.usePgs) {
      x[contact.normalRow] = projectRow(
          contact.normalRow, x[contact.normalRow], x, lo, hi, findex);
      if (!std::isfinite(x[contact.normalRow]))
        return false;
      for (std::size_t j = 0; j < contact.rowCount; ++j) {
        const int tangent = classification.contactRows[contact.rowOffset + j];
        if (tangent == contact.normalRow)
          continue;
        x[tangent] = projectRow(tangent, x[tangent], x, lo, hi, findex);
        if (!std::isfinite(x[tangent]))
          return false;
      }
    } else {
      // Cached box impulses may lie outside the newly selected circle. The
      // initial best iterate must already satisfy the selected cone at a cap.
      const auto impulse = contactImpulse(contact, x);
      if (detail::coneViolation(impulse, contact.cone) <= 0.0)
        continue;
      const auto result = detail::solveConeQp(
          Eigen::Matrix3d::Identity(), -impulse, contact.cone);
      numLocalFallbacks += result.numLocalFallbacks;
      if (!result.certified || !result.impulse.allFinite())
        return false;
      const auto indices = contactIndices(contact);
      for (int axis = 0; axis < 3; ++axis)
        if (indices[axis] >= 0)
          x[indices[axis]] = result.impulse[axis];
    }
  }
  return true;
}

//==============================================================================
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
    double* velocities)
{
  // Local solves may shift singular blocks; convergence and best-iterate
  // selection must judge the unregularized LCP assembled by the caller.
  double largest = 0.0;
  for (int row : classification.scalarRows) {
    const double velocity = rowVelocity(row, n, stride, A, x, b);
    if (velocities)
      velocities[row] = velocity;
    largest = std::max(
        largest,
        scalarViolation(
            row,
            velocity,
            A[std::size_t(row) * stride + row],
            x,
            lo,
            hi,
            findex));
  }
  for (const auto& contact : classification.contacts) {
    if (contact.usePgs) {
      for (std::size_t j = 0; j < contact.rowCount; ++j) {
        const int row = classification.contactRows[contact.rowOffset + j];
        const double velocity = rowVelocity(row, n, stride, A, x, b);
        if (velocities)
          velocities[row] = velocity;
        largest = std::max(
            largest,
            scalarViolation(
                row,
                velocity,
                A[std::size_t(row) * stride + row],
                x,
                lo,
                hi,
                findex));
      }
      continue;
    }
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double maxDiagonal = 0.0;
    const auto indices = contactIndices(contact);
    for (int axis = 0; axis < 3; ++axis) {
      const int row = indices[axis];
      if (row < 0)
        continue;
      velocity[axis] = rowVelocity(row, n, stride, A, x, b);
      if (velocities)
        velocities[row] = velocity[axis];
      maxDiagonal = std::max(maxDiagonal, A[std::size_t(row) * stride + row]);
    }
    largest = std::max(
        largest,
        detail::contactViolation(
            contactImpulse(contact, x),
            velocity,
            maxDiagonal > 0.0 ? maxDiagonal : 1.0,
            contact.cone,
            associated));
  }
  return largest;
}

//==============================================================================
std::uint64_t lcpFingerprint(
    int n,
    int stride,
    const double* A,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex)
{
  std::uint64_t hash = 14695981039346656037ull;
  const auto append = [&](double value) {
    std::uint64_t bits;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    hash = (hash ^ bits) * 1099511628211ull;
  };
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j)
      append(A[std::size_t(i) * stride + j]);
    append(b[i]);
    append(lo[i]);
    append(hi[i]);
    hash = (hash ^ std::uint64_t(findex[i])) * 1099511628211ull;
  }
  return hash;
}

//==============================================================================
bool canSolveFrictionLcp(int n, const double* A)
{
  if (n < 0 || n > std::numeric_limits<int>::max() - 3 || (n > 0 && !A))
    return false;
  const int stride = lcpsolver::dantzig::padding(n);
  for (int i = 0; i < n; ++i) {
    if (A[std::size_t(i) * stride + i] < 0.0)
      return false;
    for (int j = 0; j < n; ++j) {
      const double a = A[std::size_t(i) * stride + j];
      const double transpose = A[std::size_t(j) * stride + i];
      if (!std::isfinite(a)
          || std::abs(a - transpose)
                 > 1e-10 * std::max({1.0, std::abs(a), std::abs(transpose)}))
        return false;
    }
  }
  return true;
}

} // namespace dart::constraint::detail
