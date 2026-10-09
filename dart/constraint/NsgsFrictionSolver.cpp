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

#include <dart/constraint/NsgsFrictionSolver.hpp>
#include <dart/constraint/detail/ContactWarmStartCache.hpp>
#include <dart/constraint/detail/FrictionRows.hpp>

#include <dart/lcpsolver/dantzig/DantzigCommon.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include <cassert>
#include <cmath>

namespace dart {
namespace constraint {
namespace {

using detail::FrictionConeLaw;

struct NsgsThreadScratch
{
  detail::FrictionRowClassification classification;
  std::vector<int> rowContacts;
  std::vector<double> best;
  std::vector<double> normalShifts;
};

NsgsThreadScratch& nsgsThreadScratch()
{
  static thread_local NsgsThreadScratch scratch;
  return scratch;
}

bool updateRow(
    int row,
    int n,
    int stride,
    const double* A,
    double* x,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex)
{
  const double diagonal = A[std::size_t(row) * stride + row];
  const double value
      = diagonal > 0.0
            ? x[row] - detail::rowVelocity(row, n, stride, A, x, b) / diagonal
            : x[row];
  if (!std::isfinite(value))
    return false;
  x[row] = detail::projectRow(row, value, x, lo, hi, findex);
  return std::isfinite(x[row]);
}

bool sweep(
    int n,
    int stride,
    const double* A,
    double* x,
    const double* b,
    const double* lo,
    const double* hi,
    const int* findex,
    NsgsThreadScratch& scratch,
    NsgsFrictionSolver::Law law,
    FrictionSolveStats& stats)
{
  const auto& classification = scratch.classification;
  for (int row = 0; row < n; ++row) {
    const int index = scratch.rowContacts[row];
    if (index < 0) {
      if (!updateRow(row, n, stride, A, x, b, lo, hi, findex))
        return false;
      continue;
    }
    const auto& contact = classification.contacts[index];
    if (row != classification.contactRows[contact.rowOffset])
      continue;
    if (contact.usePgs
        || (contact.cone.law == FrictionConeLaw::Box
            && law != NsgsFrictionSolver::Law::Associated)) {
      if (!updateRow(contact.normalRow, n, stride, A, x, b, lo, hi, findex))
        return false;
      for (std::size_t j = 0; j < contact.rowCount; ++j) {
        const int tangent = classification.contactRows[contact.rowOffset + j];
        if (tangent == contact.normalRow)
          continue;
        if (contact.usePgs) {
          if (!updateRow(tangent, n, stride, A, x, b, lo, hi, findex))
            return false;
        } else {
          const int axis = tangent == contact.tangentRows[0] ? 0 : 1;
          const double diagonal = A[std::size_t(tangent) * stride + tangent];
          const double value
              = diagonal > 0.0
                    ? x[tangent]
                          - detail::rowVelocity(tangent, n, stride, A, x, b)
                                / diagonal
                    : x[tangent];
          const double bound = contact.cone.mu[axis] * x[contact.normalRow];
          if (!std::isfinite(value) || !std::isfinite(bound))
            return false;
          // Use the classified axis, including the 1-D cutoff, rather than
          // the original coefficient that classification deliberately drops.
          x[tangent] = std::clamp(value, -bound, bound);
        }
      }
    } else {
      const auto indices = detail::contactIndices(contact);
      Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
      Eigen::Vector3d freeVelocity = Eigen::Vector3d::Zero();
      const Eigen::Vector3d oldImpulse = detail::contactImpulse(contact, x);
      for (int axis = 0; axis < 3; ++axis) {
        const int localRow = indices[axis];
        if (localRow < 0)
          continue;
        freeVelocity[axis] = detail::rowVelocity(localRow, n, stride, A, x, b);
        for (int column = 0; column < 3; ++column)
          if (indices[column] >= 0)
            block(axis, column)
                = A[std::size_t(localRow) * stride + indices[column]];
        freeVelocity[axis] -= block.row(axis).dot(oldImpulse);
      }
      // An absent tangent is constrained to zero. Supply its inert diagonal
      // without making a nonsingular 2-row contact require regularization.
      for (int axis = 1; axis < 3; ++axis)
        if (indices[axis] < 0)
          block(axis, axis) = std::max(block.diagonal().maxCoeff(), 1e-12);
      const auto result
          = law == NsgsFrictionSolver::Law::Associated
                ? detail::solveConeQp(block, freeVelocity, contact.cone)
                : detail::solveExactContact(
                    block,
                    freeVelocity,
                    contact.cone,
                    scratch.normalShifts[index]);
      stats.numLocalFallbacks += result.numLocalFallbacks;
      if (!result.certified || !result.impulse.allFinite())
        return false;
      scratch.normalShifts[index] = result.normalShift;
      for (int axis = 0; axis < 3; ++axis)
        if (indices[axis] >= 0)
          x[indices[axis]] = result.impulse[axis];
    }
    if (!contact.usePgs) {
      assert(
          detail::coneViolation(
              detail::contactImpulse(contact, x), contact.cone)
          <= 1e-9 * (1.0 + std::abs(x[contact.normalRow])));
    }
  }
  return true;
}

} // namespace

//==============================================================================
NsgsFrictionSolver::NsgsFrictionSolver() : NsgsFrictionSolver(Options()) {}

//==============================================================================
NsgsFrictionSolver::NsgsFrictionSolver(const Options& options)
  : mOptions(options)
{
}

//==============================================================================
const std::string& NsgsFrictionSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& NsgsFrictionSolver::getStaticType()
{
  static const std::string type = "NsgsFrictionSolver";
  return type;
}

//==============================================================================
bool NsgsFrictionSolver::solve(
    int n,
    double* A,
    double* x,
    double* b,
    int /*nub*/,
    double* lo,
    double* hi,
    int* findex,
    bool earlyTermination)
{
  auto& refinementSolver = detail::contactWarmStartRefinementSolver();
  const bool requireRefinement
      = refinementSolver == this && mOptions.maxSweeps > 0;
  refinementSolver = nullptr;
  FrictionSolveStats stats;
  stats.numSolves = 1;
  const auto finish = [&](bool success, bool converged, double finalViolation) {
    detail::contactWarmStartSolveResult()
        = {this, success, converged, finalViolation};
    stats.numConverged = success && converged;
    stats.numAcceptedAtCap = success && !converged;
    stats.numFailed = !success;
    stats.maxViolation = success ? finalViolation : 0.0;
    accumulateStats(stats);
    return success;
  };
  const double infinity = std::numeric_limits<double>::infinity();
  if (n < 0 || n > std::numeric_limits<int>::max() - 3
      || (n > 0 && (!A || !x || !b || !lo || !hi || !findex))
      || mOptions.maxSweeps < 0 || !std::isfinite(mOptions.tolerance)
      || mOptions.tolerance < 0.0
      || (mOptions.law != Law::Coulomb && mOptions.law != Law::Associated
          && mOptions.law != Law::Box))
    return finish(false, false, infinity);
  if (n == 0)
    return finish(true, true, 0.0);
  const int stride = lcpsolver::dantzig::padding(n);
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(x[i]) || !std::isfinite(b[i]) || std::isnan(lo[i])
        || std::isnan(hi[i]) || lo[i] > hi[i] || findex[i] == i
        || A[std::size_t(i) * stride + i] < 0.0)
      return finish(false, false, infinity);
    for (int j = 0; j < n; ++j)
      if (!std::isfinite(A[std::size_t(i) * stride + j]))
        return finish(false, false, infinity);
  }
#ifndef NDEBUG
  const auto fingerprint
      = detail::lcpFingerprint(n, stride, A, b, lo, hi, findex);
#endif
  auto& scratch = nsgsThreadScratch();
  auto& classification = scratch.classification;
  if (!detail::classifyFrictionRows(
          n, lo, hi, findex, classification, mOptions.boxForAnisotropic))
    return finish(false, false, infinity);
  if (mOptions.law == Law::Box) {
    for (auto& contact : classification.contacts)
      contact.cone.law = FrictionConeLaw::Box;
    classification.numBoxContacts = classification.contacts.size();
  }
  stats.numContacts = classification.contacts.size();
  stats.numBoxContacts = classification.numBoxContacts;
  scratch.rowContacts.assign(n, -1);
  for (std::size_t c = 0; c < classification.contacts.size(); ++c) {
    const auto& contact = classification.contacts[c];
    for (std::size_t j = 0; j < contact.rowCount; ++j)
      scratch.rowContacts[classification.contactRows[contact.rowOffset + j]]
          = c;
  }
  scratch.normalShifts.assign(classification.contacts.size(), 0.0);
  if (!detail::projectIterate(
          x, lo, hi, findex, classification, stats.numLocalFallbacks))
    return finish(false, false, infinity);
  scratch.best.assign(x, x + n);
  double bestViolation = detail::lawViolation(
      n,
      stride,
      A,
      x,
      b,
      lo,
      hi,
      findex,
      classification,
      mOptions.law == Law::Associated);
  const double startingViolation = bestViolation;
  if (!std::isfinite(bestViolation))
    return finish(false, false, infinity);
  if (bestViolation <= mOptions.tolerance && !requireRefinement)
    return finish(true, true, bestViolation);
  bool producedNoWorse
      = mOptions.maxSweeps == 0
        || (requireRefinement && bestViolation <= mOptions.tolerance);
  bool failed = false;
  bool converged = false;
  for (int iteration = 0; iteration < mOptions.maxSweeps; ++iteration) {
    ++stats.numIterations;
    if (!sweep(
            n, stride, A, x, b, lo, hi, findex, scratch, mOptions.law, stats)) {
      failed = true;
      break;
    }
    // Use the actual complete iterate, rather than a mixture of pre-update
    // block velocities, for stopping and best-iterate selection.
    const double currentViolation = detail::lawViolation(
        n,
        stride,
        A,
        x,
        b,
        lo,
        hi,
        findex,
        classification,
        mOptions.law == Law::Associated);
    if (!std::isfinite(currentViolation)) {
      failed = true;
      break;
    }
    producedNoWorse |= currentViolation <= startingViolation;
    if (currentViolation < bestViolation) {
      bestViolation = currentViolation;
      std::copy(x, x + n, scratch.best.begin());
    }
    // A refinement must not discard an already certified best guess.
    if ((requireRefinement ? bestViolation : currentViolation)
        <= mOptions.tolerance) {
      converged = true;
      break;
    }
  }
  std::copy(scratch.best.begin(), scratch.best.end(), x);
#ifndef NDEBUG
  assert(
      fingerprint == detail::lcpFingerprint(n, stride, A, b, lo, hi, findex));
#endif
  return finish(
      !earlyTermination || (!failed && producedNoWorse),
      converged,
      bestViolation);
}

//==============================================================================
void NsgsFrictionSolver::setOptions(const Options& options)
{
  mOptions = options;
}

//==============================================================================
const NsgsFrictionSolver::Options& NsgsFrictionSolver::getOptions() const
{
  return mOptions;
}

//==============================================================================
void NsgsFrictionSolver::reserve(std::size_t numRows)
{
  if (numRows > std::size_t(std::numeric_limits<int>::max() - 3))
    throw std::length_error("NSGS scratch row count exceeds padded stride");
  auto& scratch = nsgsThreadScratch();
  scratch.rowContacts.reserve(numRows);
  scratch.best.reserve(numRows);
  scratch.normalShifts.reserve(numRows);
  auto& classification = scratch.classification;
  classification.contacts.reserve(numRows);
  classification.contactRows.reserve(numRows);
  classification.scalarRows.reserve(numRows);
  classification.parents.reserve(numRows);
  classification.roots.reserve(numRows);
  classification.path.reserve(numRows);
  classification.componentContacts.reserve(numRows);
  classification.componentSizes.reserve(numRows);
}

//==============================================================================
void NsgsFrictionSolver::accumulateStats(const FrictionSolveStats& stats)
{
  constexpr auto order = std::memory_order_relaxed;
  mNumSolves.fetch_add(stats.numSolves, order);
  mNumConverged.fetch_add(stats.numConverged, order);
  mNumAcceptedAtCap.fetch_add(stats.numAcceptedAtCap, order);
  mNumFailed.fetch_add(stats.numFailed, order);
  mNumContacts.fetch_add(stats.numContacts, order);
  mNumBoxContacts.fetch_add(stats.numBoxContacts, order);
  mNumLocalFallbacks.fetch_add(stats.numLocalFallbacks, order);
  mNumIterations.fetch_add(stats.numIterations, order);
  double previous = mMaxViolation.load(order);
  while (previous < stats.maxViolation
         && !mMaxViolation.compare_exchange_weak(
             previous, stats.maxViolation, order, order)) {
  }
}

//==============================================================================
FrictionSolveStats NsgsFrictionSolver::getStats() const
{
  constexpr auto order = std::memory_order_relaxed;
  FrictionSolveStats stats;
  stats.numSolves = mNumSolves.load(order);
  stats.numConverged = mNumConverged.load(order);
  stats.numAcceptedAtCap = mNumAcceptedAtCap.load(order);
  stats.numFailed = mNumFailed.load(order);
  stats.numContacts = mNumContacts.load(order);
  stats.numBoxContacts = mNumBoxContacts.load(order);
  stats.numLocalFallbacks = mNumLocalFallbacks.load(order);
  stats.numIterations = mNumIterations.load(order);
  stats.maxViolation = mMaxViolation.load(order);
  return stats;
}

//==============================================================================
void NsgsFrictionSolver::resetStats()
{
  constexpr auto order = std::memory_order_relaxed;
  mNumSolves.store(0, order);
  mNumConverged.store(0, order);
  mNumAcceptedAtCap.store(0, order);
  mNumFailed.store(0, order);
  mNumContacts.store(0, order);
  mNumBoxContacts.store(0, order);
  mNumLocalFallbacks.store(0, order);
  mNumIterations.store(0, order);
  mMaxViolation.store(0.0, order);
}

} // namespace constraint
} // namespace dart
