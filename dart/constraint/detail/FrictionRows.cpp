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

#include <algorithm>

#include <cmath>

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
    const bool cleanTangent = result.parents[i] == root && std::isfinite(hi[i])
                              && hi[i] >= 0 && lo[i] == -hi[i];
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
    contact.cone.law
        = contact.usePgs || (boxForAnisotropic && major - minor > 1e-9 * major)
              ? FrictionConeLaw::Box
              : FrictionConeLaw::Ellipse;
    if (!contact.usePgs && minor <= 1e-6 * major) {
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

} // namespace dart::constraint::detail
