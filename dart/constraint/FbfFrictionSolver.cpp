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

#include <dart/constraint/FbfFrictionSolver.hpp>
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

constexpr double kFbfRatioLimit = 0.9;
constexpr double kFbfShrinkFactor = 0.7;
constexpr int kFbfMaxShrinks = 20;
constexpr double kFbfInnerFloor = 1e-12;
constexpr double kFbfInnerTargetCap = 100.0;

struct FbfThreadScratch
{
  detail::FrictionRowClassification classification;
  std::vector<int> rowContacts;
  std::vector<double> best, warm, y, velocities, shifts, trialShifts;
};
FbfThreadScratch& fbfThreadScratch()
{
  static thread_local FbfThreadScratch s;
  return s;
}
struct FbfTerms
{
  int n, stride;
  const double *A, *b, *lo, *hi;
  const int* findex;
};

// sqrt(max row sum) * sqrt(max column sum) of |A| over the cone contacts'
// present tangent rows T; |A_T|_2 <= sqrt(|A_T|_1 |A_T|_inf).
double fbfTangentRowBound(
    const FbfTerms& t, const detail::FrictionRowClassification& cls)
{
  double rows = 0.0, columns = 0.0;
  for (const auto& c : cls.contacts)
    if (!c.usePgs)
      for (int r : c.tangentRows)
        if (r >= 0) {
          double sum = 0.0;
          for (int j = 0; j < t.n; ++j)
            sum += std::abs(t.A[std::size_t(r) * t.stride + j]);
          rows = std::max(rows, sum);
        }
  for (int j = 0; j < t.n; ++j) {
    double sum = 0.0;
    for (const auto& c : cls.contacts)
      if (!c.usePgs)
        for (int r : c.tangentRows)
          if (r >= 0)
            sum += std::abs(t.A[std::size_t(r) * t.stride + j]);
    columns = std::max(columns, sum);
  }
  return std::sqrt(rows) * std::sqrt(columns);
}

// Overflow-safe |a - b|; 0 only if a == b componentwise; inf if a difference
// overflows.
double fbfDistance(const double* a, const double* b, std::size_t size)
{
  double scale = 0.0;
  for (std::size_t i = 0; i < size; ++i)
    scale = std::max(scale, std::abs(a[i] - b[i]));
  if (scale == 0.0 || !std::isfinite(scale))
    return scale;
  double sum = 0.0;
  for (std::size_t i = 0; i < size; ++i) {
    const double d = (a[i] - b[i]) / scale;
    sum += d * d;
  }
  return scale * std::sqrt(sum);
}

double fbfCoupling(const detail::FrictionContactRows& c, const double* w)
{
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  for (int a = 0; a < 2; ++a)
    if (c.tangentRows[a] >= 0)
      t[a + 1] = w[c.tangentRows[a]];
  return detail::deSaxce(t, c.cone)[0];
}

bool fbfUpdateRow(
    const FbfTerms& t,
    int row,
    double* z,
    const double* center,
    double invGamma)
{
  double gradient = detail::rowVelocity(row, t.n, t.stride, t.A, z, t.b);
  if (invGamma > 0.0)
    gradient += invGamma * (z[row] - center[row]);
  const double diagonal = t.A[std::size_t(row) * t.stride + row] + invGamma;
  const double value = diagonal > 0.0 ? z[row] - gradient / diagonal : z[row];
  if (!std::isfinite(value))
    return false;
  z[row] = detail::projectRow(row, value, z, t.lo, t.hi, t.findex);
  return std::isfinite(z[row]);
}

bool fbfSweep(
    const FbfTerms& t,
    double* z,
    const double* center,
    double invGamma,
    const double* shifts,
    const FbfThreadScratch& s,
    FrictionSolveStats& stats)
{
  const auto& cls = s.classification;
  for (int row = 0; row < t.n; ++row) {
    const int index = s.rowContacts[row];
    if (index < 0) {
      if (!fbfUpdateRow(t, row, z, center, invGamma))
        return false;
      continue;
    }
    const auto& c = cls.contacts[index];
    if (row != cls.contactRows[c.rowOffset])
      continue;
    if (c.usePgs) {
      if (!fbfUpdateRow(t, c.normalRow, z, center, invGamma))
        return false;
      for (std::size_t j = 0; j < c.rowCount; ++j) {
        const int other = cls.contactRows[c.rowOffset + j];
        if (other != c.normalRow
            && !fbfUpdateRow(t, other, z, center, invGamma))
          return false;
      }
      continue;
    }
    const auto idx = detail::contactIndices(c);
    Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    const Eigen::Vector3d old = detail::contactImpulse(c, z);
    for (int a = 0; a < 3; ++a) {
      if (idx[a] < 0)
        continue;
      q[a] = detail::rowVelocity(idx[a], t.n, t.stride, t.A, z, t.b);
      for (int col = 0; col < 3; ++col)
        if (idx[col] >= 0)
          block(a, col) = t.A[std::size_t(idx[a]) * t.stride + idx[col]];
      q[a] -= block.row(a).dot(old);
    }
    if (invGamma > 0.0)
      for (int a = 0; a < 3; ++a)
        if (idx[a] >= 0) {
          block(a, a) += invGamma;
          q[a] -= invGamma * center[idx[a]];
        }
    if (shifts)
      q[0] += shifts[index];
    for (int a = 1; a < 3; ++a)
      if (idx[a] < 0)
        block(a, a) = std::max(block.diagonal().maxCoeff(), 1e-12);
    const auto r = detail::solveConeQp(block, q, c.cone);
    stats.numLocalFallbacks += r.numLocalFallbacks;
    if (!r.certified || !r.impulse.allFinite())
      return false;
    for (int a = 0; a < 3; ++a)
      if (idx[a] >= 0)
        z[idx[a]] = r.impulse[a];
    assert(
        detail::coneViolation(detail::contactImpulse(c, z), c.cone)
        <= 1e-9 * (1.0 + std::abs(z[c.normalRow])));
  }
  return true;
}

double fbfInnerResidual(
    const FbfTerms& t,
    const double* z,
    const double* center,
    double invGamma,
    const double* shifts,
    FbfThreadScratch& s)
{
  double* w = s.velocities.data();
  for (int i = 0; i < t.n; ++i)
    w[i] = detail::rowVelocity(i, t.n, t.stride, t.A, z, t.b);
  const auto scalar = [&](int row) {
    return detail::scalarViolation(
        row,
        w[row] + invGamma * (z[row] - center[row]),
        t.A[std::size_t(row) * t.stride + row] + invGamma,
        z,
        t.lo,
        t.hi,
        t.findex);
  };
  double largest = 0.0;
  for (int row : s.classification.scalarRows)
    largest = std::max(largest, scalar(row));
  for (std::size_t ci = 0; ci < s.classification.contacts.size(); ++ci) {
    const auto& c = s.classification.contacts[ci];
    if (c.usePgs) {
      for (std::size_t j = 0; j < c.rowCount; ++j)
        largest = std::max(
            largest, scalar(s.classification.contactRows[c.rowOffset + j]));
      continue;
    }
    const auto idx = detail::contactIndices(c);
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    double maxDiagonal = 0.0;
    for (int a = 0; a < 3; ++a) {
      if (idx[a] < 0)
        continue;
      u[a] = w[idx[a]] + invGamma * (z[idx[a]] - center[idx[a]]);
      maxDiagonal
          = std::max(maxDiagonal, t.A[std::size_t(idx[a]) * t.stride + idx[a]]);
    }
    u[0] += shifts[ci];
    largest = std::max(
        largest,
        detail::contactViolation(
            detail::contactImpulse(c, z),
            u,
            maxDiagonal + invGamma,
            c.cone,
            true));
  }
  return largest;
}

} // namespace

//==============================================================================
FbfFrictionSolver::FbfFrictionSolver() : FbfFrictionSolver(Options()) {}

//==============================================================================
FbfFrictionSolver::FbfFrictionSolver(const Options& options) : mOptions(options)
{
}

//==============================================================================
const std::string& FbfFrictionSolver::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& FbfFrictionSolver::getStaticType()
{
  static const std::string type = "FbfFrictionSolver";
  return type;
}

//==============================================================================
bool FbfFrictionSolver::solve(
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
  const Options o = mOptions;
  FrictionSolveStats stats;
  stats.numSolves = 1;
  const auto finish = [&](bool success, bool converged, double finalViolation) {
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
      || o.maxOuterIterations < 0 || !std::isfinite(o.tolerance)
      || o.tolerance < 0.0 || !std::isfinite(o.stepScale) || o.stepScale <= 0.0
      || o.maxInnerSweeps < 1 || !std::isfinite(o.innerToleranceFactor)
      || o.innerToleranceFactor < 0.0)
    return finish(false, false, infinity);
  if (n == 0)
    return finish(true, true, 0.0);
  const int stride = lcpsolver::dantzig::padding(n);
  double L = 0.0;
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(x[i]) || !std::isfinite(b[i]) || std::isnan(lo[i])
        || std::isnan(hi[i]) || lo[i] > hi[i] || findex[i] == i
        || A[std::size_t(i) * stride + i] < 0.0)
      return finish(false, false, infinity);
    double rowSum = 0.0;
    for (int j = 0; j < n; ++j) {
      const double a = A[std::size_t(i) * stride + j];
      if (!std::isfinite(a))
        return finish(false, false, infinity);
      rowSum += std::abs(a);
    }
    L = std::max(L, rowSum);
  }
#ifndef NDEBUG
  const auto fingerprint
      = detail::lcpFingerprint(n, stride, A, b, lo, hi, findex);
#endif
  auto& s = fbfThreadScratch();
  auto& cls = s.classification;
  if (!detail::classifyFrictionRows(
          n, lo, hi, findex, cls, o.boxForAnisotropic))
    return finish(false, false, infinity);
  stats.numContacts = cls.contacts.size();
  stats.numBoxContacts = cls.numBoxContacts;
  s.rowContacts.assign(n, -1);
  for (std::size_t c = 0; c < cls.contacts.size(); ++c) {
    const auto& contact = cls.contacts[c];
    for (std::size_t j = 0; j < contact.rowCount; ++j)
      s.rowContacts[cls.contactRows[contact.rowOffset + j]] = c;
  }
  double muEff = 0.0;
  for (const auto& c : cls.contacts)
    if (!c.usePgs)
      muEff = std::max(
          muEff,
          c.cone.law == detail::FrictionConeLaw::Box
              ? std::hypot(c.cone.mu[0], c.cone.mu[1])
              : c.cone.mu.maxCoeff());
  const bool proximal = muEff > 0.0 && L > 0.0;
  const FbfTerms t{n, stride, A, b, lo, hi, findex};
  double gamma = 0.0;
  double invGamma = 0.0;
  if (proximal) {
    const double LT = fbfTangentRowBound(t, cls);
    gamma = o.stepScale / (muEff * (LT > 0.0 ? LT : L));
    invGamma = 1.0 / gamma;
    if (!(std::isfinite(gamma) && gamma > 0.0 && std::isfinite(invGamma)))
      return finish(false, false, infinity);
  }
  s.velocities.resize(n);
  if (!detail::projectIterate(x, lo, hi, findex, cls, stats.numLocalFallbacks))
    return finish(false, false, infinity);
  s.best.assign(x, x + n);
  const double start = detail::lawViolation(
      n, stride, A, x, b, lo, hi, findex, cls, false, s.velocities.data());
  if (!std::isfinite(start))
    return finish(false, false, infinity);
  if (start <= o.tolerance)
    return finish(true, true, start);
  double bestViolation = start;
  bool producedNoWorse = o.maxOuterIterations == 0;
  bool failed = false;
  bool converged = false;
  const auto record = [&](double v) {
    // Residual changes within tolerance do not establish divergence.
    producedNoWorse |= v - start <= o.tolerance;
    if (v < bestViolation) {
      bestViolation = v;
      std::copy(x, x + n, s.best.begin());
    }
    converged = v <= o.tolerance;
  };
  if (!proximal) {
    for (int k = 0; k < o.maxOuterIterations; ++k) {
      ++stats.numIterations;
      ++stats.numInnerIterations;
      if (!fbfSweep(t, x, nullptr, 0.0, nullptr, s, stats)) {
        failed = true;
        break;
      }
      const double v = detail::lawViolation(
          n, stride, A, x, b, lo, hi, findex, cls, false);
      if (!std::isfinite(v)) {
        failed = true;
        break;
      }
      record(v);
      if (converged)
        break;
    }
  } else {
    const std::size_t m = cls.contacts.size();
    s.shifts.assign(m, 0.0);
    s.trialShifts.assign(m, 0.0);
    s.warm.assign(x, x + n);
    s.y.resize(n);
    double current = start;
    for (int k = 0; k < o.maxOuterIterations; ++k) {
      ++stats.numIterations;
      for (std::size_t c = 0; c < m; ++c)
        if (!cls.contacts[c].usePgs) {
          s.shifts[c] = fbfCoupling(cls.contacts[c], s.velocities.data());
          failed |= !std::isfinite(s.shifts[c]);
        }
      if (failed)
        break;
      const double target = std::max(
          kFbfInnerFloor,
          o.innerToleranceFactor
              * std::min(current, kFbfInnerTargetCap * o.tolerance));
      bool accepted = false;
      for (int trial = 0;; ++trial) {
        std::copy(s.warm.begin(), s.warm.end(), s.y.begin());
        double residual;
        int sweeps = 0;
        do {
          ++stats.numInnerIterations;
          ++sweeps;
          residual
              = fbfSweep(t, s.y.data(), x, invGamma, s.shifts.data(), s, stats)
                    ? fbfInnerResidual(
                        t, s.y.data(), x, invGamma, s.shifts.data(), s)
                    : std::numeric_limits<double>::quiet_NaN();
        } while (std::isfinite(residual) && residual > target
                 && sweeps < o.maxInnerSweeps);
        if (!std::isfinite(residual)) {
          failed = true;
          break;
        }
        if (residual > target)
          ++stats.numInnerCaps;
        for (std::size_t c = 0; c < m; ++c)
          if (!cls.contacts[c].usePgs) {
            s.trialShifts[c]
                = fbfCoupling(cls.contacts[c], s.velocities.data());
            failed |= !std::isfinite(s.trialShifts[c]);
          }
        if (failed)
          break;
        const double shiftChange
            = fbfDistance(s.trialShifts.data(), s.shifts.data(), m);
        const double step = fbfDistance(s.y.data(), x, n);
        if (!std::isfinite(shiftChange) || !std::isfinite(step)) {
          failed = true;
          break;
        }
        const double ratio = step == 0.0 ? 0.0 : gamma * (shiftChange / step);
        if (ratio <= kFbfRatioLimit) {
          accepted = true;
          break;
        }
        if (trial == kFbfMaxShrinks)
          break;
        gamma *= kFbfShrinkFactor;
        invGamma = 1.0 / gamma;
        ++stats.numStepShrinks;
      }
      if (failed)
        break;
      if (!accepted) {
        failed = true;
        break;
      }
      std::copy(s.y.begin(), s.y.end(), s.warm.begin());
      std::copy(s.y.begin(), s.y.end(), x);
      for (std::size_t c = 0; c < m; ++c)
        if (!cls.contacts[c].usePgs)
          x[cls.contacts[c].normalRow]
              -= gamma * (s.trialShifts[c] - s.shifts[c]);
      if (!detail::projectIterate(
              x, lo, hi, findex, cls, stats.numLocalFallbacks)) {
        failed = true;
        break;
      }
      current = detail::lawViolation(
          n, stride, A, x, b, lo, hi, findex, cls, false, s.velocities.data());
      if (!std::isfinite(current)) {
        failed = true;
        break;
      }
      record(current);
      if (converged)
        break;
    }
  }
  std::copy(s.best.begin(), s.best.end(), x);
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
void FbfFrictionSolver::setOptions(const Options& options)
{
  mOptions = options;
}

//==============================================================================
const FbfFrictionSolver::Options& FbfFrictionSolver::getOptions() const
{
  return mOptions;
}

//==============================================================================
void FbfFrictionSolver::reserve(std::size_t numRows)
{
  if (numRows > std::size_t(std::numeric_limits<int>::max() - 3))
    throw std::length_error("FBF scratch row count exceeds padded stride");
  auto& scratch = fbfThreadScratch();
  scratch.rowContacts.reserve(numRows);
  scratch.best.reserve(numRows);
  scratch.warm.reserve(numRows);
  scratch.y.reserve(numRows);
  scratch.velocities.reserve(numRows);
  scratch.shifts.reserve(numRows);
  scratch.trialShifts.reserve(numRows);
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
void FbfFrictionSolver::accumulateStats(const FrictionSolveStats& stats)
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
  mNumInnerIterations.fetch_add(stats.numInnerIterations, order);
  mNumStepShrinks.fetch_add(stats.numStepShrinks, order);
  mNumInnerCaps.fetch_add(stats.numInnerCaps, order);
  double previous = mMaxViolation.load(order);
  while (previous < stats.maxViolation
         && !mMaxViolation.compare_exchange_weak(
             previous, stats.maxViolation, order, order)) {
  }
}

//==============================================================================
FrictionSolveStats FbfFrictionSolver::getStats() const
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
  stats.numInnerIterations = mNumInnerIterations.load(order);
  stats.numStepShrinks = mNumStepShrinks.load(order);
  stats.numInnerCaps = mNumInnerCaps.load(order);
  stats.maxViolation = mMaxViolation.load(order);
  return stats;
}

//==============================================================================
void FbfFrictionSolver::resetStats()
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
  mNumInnerIterations.store(0, order);
  mNumStepShrinks.store(0, order);
  mNumInnerCaps.store(0, order);
  mMaxViolation.store(0.0, order);
}

} // namespace constraint
} // namespace dart
