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

#include "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"

#include "dart/common/Console.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/detail/ExactCoulombContactRowOperator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <cstdint>

#if defined(__linux__)
  #include <sched.h>
#endif

namespace dart {
namespace constraint {
namespace {

constexpr std::size_t kMinParallelContactRowContacts = 128u;
constexpr double kWarmStartMinNormalDot = 0.9;

struct ExactCoulombBodyLocalContactFeature
{
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  bool available = false;
};

struct ExactCoulombBodyLocalContactFeatures
{
  ExactCoulombBodyLocalContactFeature sideA;
  ExactCoulombBodyLocalContactFeature sideB;
};

struct ExactCoulombContactIdentityMatch
{
  bool matched = false;
  bool flipped = false;
  double distance = std::numeric_limits<double>::infinity();
};

ExactCoulombBodyLocalContactFeature makeBodyLocalContactFeature(
    const dynamics::BodyNode* body,
    const Eigen::Vector3d& worldPoint,
    const Eigen::Vector3d& outwardWorldNormal)
{
  ExactCoulombBodyLocalContactFeature feature;
  if (body == nullptr || !worldPoint.allFinite()
      || !outwardWorldNormal.allFinite()) {
    return feature;
  }

  const Eigen::Isometry3d& worldTransform = body->getWorldTransform();
  if (!worldTransform.matrix().allFinite())
    return feature;

  feature.point = worldTransform.inverse() * worldPoint;
  feature.normal = worldTransform.linear().transpose() * outwardWorldNormal;
  const double normalNorm = feature.normal.norm();
  if (!feature.point.allFinite() || !std::isfinite(normalNorm)
      || normalNorm <= std::numeric_limits<double>::epsilon()) {
    return feature;
  }

  feature.normal /= normalNorm;
  feature.available = true;
  return feature;
}

ExactCoulombBodyLocalContactFeatures makeBodyLocalContactFeatures(
    const detail::ExactCoulombContactRowOperator::ContactFrame& frame)
{
  ExactCoulombBodyLocalContactFeatures features;
  features.sideA
      = makeBodyLocalContactFeature(frame.bodyA, frame.point, frame.normal);
  features.sideB
      = makeBodyLocalContactFeature(frame.bodyB, frame.point, -frame.normal);
  return features;
}

bool hasConsistentWarmStartNormal(
    const Eigen::Vector3d& first, const Eigen::Vector3d& second)
{
  if (!first.allFinite() || !second.allFinite())
    return false;

  const double firstNorm = first.norm();
  const double secondNorm = second.norm();
  if (!std::isfinite(firstNorm) || !std::isfinite(secondNorm)
      || firstNorm <= std::numeric_limits<double>::epsilon()
      || secondNorm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  return first.dot(second) >= kWarmStartMinNormalDot * firstNorm * secondNorm;
}

template <typename ContactRecord>
ExactCoulombContactIdentityMatch matchExactCoulombContactIdentity(
    const ContactRecord& record,
    const detail::ExactCoulombContactRowOperator::ContactFrame& frame,
    const ExactCoulombBodyLocalContactFeatures& currentFeatures,
    double matchDistance)
{
  ExactCoulombContactIdentityMatch match;
  const bool sameOrder
      = record.bodyA == frame.bodyA && record.bodyB == frame.bodyB;
  const bool flippedOrder
      = record.bodyA == frame.bodyB && record.bodyB == frame.bodyA;
  if (!sameOrder && !flippedOrder)
    return match;

  match.flipped = !sameOrder;
  bool hasComparableLocalFeature = false;
  const auto considerLocalFeature = [&](const auto& current,
                                        bool recordAvailable,
                                        const Eigen::Vector3d& recordPoint,
                                        const Eigen::Vector3d& recordNormal) {
    if (!current.available || !recordAvailable)
      return;

    hasComparableLocalFeature = true;
    const double distance = (recordPoint - current.point).norm();
    if (!std::isfinite(distance) || distance > matchDistance
        || !hasConsistentWarmStartNormal(recordNormal, current.normal)) {
      return;
    }

    match.matched = true;
    match.distance = std::min(match.distance, distance);
  };

  if (sameOrder) {
    considerLocalFeature(
        currentFeatures.sideA,
        record.hasLocalFeatureA,
        record.localPointA,
        record.localNormalA);
    considerLocalFeature(
        currentFeatures.sideB,
        record.hasLocalFeatureB,
        record.localPointB,
        record.localNormalB);
  } else {
    considerLocalFeature(
        currentFeatures.sideA,
        record.hasLocalFeatureB,
        record.localPointB,
        record.localNormalB);
    considerLocalFeature(
        currentFeatures.sideB,
        record.hasLocalFeatureA,
        record.localPointA,
        record.localNormalA);
  }

  // A local feature is authoritative whenever one is available. Falling back
  // to the world point despite a local mismatch would recreate the rotating
  // patch aliasing that this cache is intended to avoid.
  if (hasComparableLocalFeature)
    return match;

  const Eigen::Vector3d expectedWorldNormal
      = sameOrder ? record.worldNormal : -record.worldNormal;
  const double worldDistance = (record.point - frame.point).norm();
  if (std::isfinite(worldDistance) && worldDistance <= matchDistance
      && hasConsistentWarmStartNormal(expectedWorldNormal, frame.normal)) {
    match.matched = true;
    match.distance = worldDistance;
  }
  return match;
}

bool hasConsistentWarmStartNormal(
    const Eigen::Vector3d& first,
    const Eigen::Vector3d& second,
    double minimumCosine)
{
  if (!first.allFinite() || !second.allFinite()
      || !std::isfinite(minimumCosine)) {
    return false;
  }

  const double firstNorm = first.norm();
  const double secondNorm = second.norm();
  if (!std::isfinite(firstNorm) || !std::isfinite(secondNorm)
      || firstNorm <= std::numeric_limits<double>::epsilon()
      || secondNorm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  return first.dot(second) >= minimumCosine * firstNorm * secondNorm;
}

template <typename ContactRecord>
ExactCoulombContactIdentityMatch matchExactCoulombContactIdentity(
    const ContactRecord& record,
    const detail::ExactCoulombContactRowOperator::ContactFrame& frame,
    const ExactCoulombBodyLocalContactFeatures& currentFeatures,
    double matchDistance,
    const ExactCoulombFbfCrossStepPolicyOptions& policy)
{
  if (policy.warmStartMatchMode
          == ExactCoulombFbfWarmStartMatchMode::EitherBodyLocalFeature
      && policy.warmStartNormalCosine == kWarmStartMinNormalDot
      && !policy.useStrictWarmStartMatchDistance) {
    return matchExactCoulombContactIdentity(
        record, frame, currentFeatures, matchDistance);
  }

  ExactCoulombContactIdentityMatch match;
  const bool sameOrder
      = record.bodyA == frame.bodyA && record.bodyB == frame.bodyB;
  const bool flippedOrder
      = record.bodyA == frame.bodyB && record.bodyB == frame.bodyA;
  if (!sameOrder
      && (policy.warmStartMatchMode
              == ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature
          || !flippedOrder)) {
    return match;
  }

  match.flipped = !sameOrder;
  const auto isWithinMatchDistance = [&policy, matchDistance](double distance) {
    return policy.useStrictWarmStartMatchDistance ? distance < matchDistance
                                                  : distance <= matchDistance;
  };

  if (policy.warmStartMatchMode
      == ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature) {
    if (!currentFeatures.sideB.available || !record.hasLocalFeatureB)
      return match;

    const double distance
        = (record.localPointB - currentFeatures.sideB.point).norm();
    if (std::isfinite(distance) && isWithinMatchDistance(distance)
        && hasConsistentWarmStartNormal(
            record.localNormalB,
            currentFeatures.sideB.normal,
            policy.warmStartNormalCosine)) {
      match.matched = true;
      match.distance = distance;
    }
    return match;
  }

  bool hasComparableLocalFeature = false;
  const auto considerLocalFeature = [&](const auto& current,
                                        bool recordAvailable,
                                        const Eigen::Vector3d& recordPoint,
                                        const Eigen::Vector3d& recordNormal) {
    if (!current.available || !recordAvailable)
      return;

    hasComparableLocalFeature = true;
    const double distance = (recordPoint - current.point).norm();
    if (!std::isfinite(distance) || !isWithinMatchDistance(distance)
        || !hasConsistentWarmStartNormal(
            recordNormal, current.normal, policy.warmStartNormalCosine)) {
      return;
    }

    match.matched = true;
    match.distance = std::min(match.distance, distance);
  };

  if (sameOrder) {
    considerLocalFeature(
        currentFeatures.sideA,
        record.hasLocalFeatureA,
        record.localPointA,
        record.localNormalA);
    considerLocalFeature(
        currentFeatures.sideB,
        record.hasLocalFeatureB,
        record.localPointB,
        record.localNormalB);
  } else {
    considerLocalFeature(
        currentFeatures.sideA,
        record.hasLocalFeatureB,
        record.localPointB,
        record.localNormalB);
    considerLocalFeature(
        currentFeatures.sideB,
        record.hasLocalFeatureA,
        record.localPointA,
        record.localNormalA);
  }
  if (hasComparableLocalFeature)
    return match;

  const Eigen::Vector3d expectedWorldNormal
      = sameOrder ? record.worldNormal : -record.worldNormal;
  const double worldDistance = (record.point - frame.point).norm();
  if (std::isfinite(worldDistance) && isWithinMatchDistance(worldDistance)
      && hasConsistentWarmStartNormal(
          expectedWorldNormal, frame.normal, policy.warmStartNormalCosine)) {
    match.matched = true;
    match.distance = worldDistance;
  }
  return match;
}

struct ExactCoulombContactRowParallelEvidence
{
  std::size_t products = 0u;
  std::size_t parallelProducts = 0u;
  std::size_t maxParticipants = 0u;
  std::size_t totalProducts = 0u;
  std::size_t totalParallelProducts = 0u;
  std::size_t maxParticipantsEver = 0u;
  std::vector<int> logicalCpuIds;
  std::vector<int> logicalCpuIdsEver;
  std::vector<int> maxPhaseLogicalCpuIds;
  std::vector<int> maxPhaseLogicalCpuIdsEver;
};

struct ExactCoulombColoredBlockGaussSeidelState
{
  bool enabled = false;
  bool participantAffinityEnabled = false;
  bool used = false;
  std::size_t solves = 0u;
  std::size_t dispatches = 0u;
  std::size_t maxParticipants = 0u;
  std::size_t manifoldCount = 0u;
  std::size_t colorCount = 0u;
  std::size_t maxManifoldsPerColor = 0u;
  std::vector<int> logicalCpuIds;
  std::vector<int> maxPhaseLogicalCpuIds;
};

struct ExactCoulombPolicyState
{
  ExactCoulombFbfCrossStepPolicyOptions crossStepOptions;
  ExactCoulombFbfSourceContinuationOptions sourceContinuationOptions;
  double lastInitialResidual = std::numeric_limits<double>::quiet_NaN();
  double lastInitialNaturalMapResidual
      = std::numeric_limits<double>::quiet_NaN();
  double lastNaturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  double lastUncappedInitialStepSize = std::numeric_limits<double>::quiet_NaN();
  bool lastWarmStartStepSizeCapApplied = false;
  std::size_t warmStartStepSizeCaps = 0u;
  std::size_t unconvergedCacheSkips = 0u;
  bool lastSourceContinuationActive = false;
  bool lastLineSearchShrinkCapReached = false;
  int lastLineSearchShrinkCapCount = 0;
  double lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  std::size_t plateausAccepted = 0u;
  std::size_t lineSearchShrinkCaps = 0u;
};

struct ExactCoulombLastFailureState
{
  ExactCoulombFbfConstraintSolverStatus status
      = ExactCoulombFbfConstraintSolverStatus::NotRun;
  detail::ExactCoulombConstraintBuildStatus buildStatus
      = detail::ExactCoulombConstraintBuildStatus::EmptyInput;
  std::size_t contactCount = 0u;
};

bool isPreferredExactCoulombCpuPhaseSet(
    const std::vector<int>& candidate, const std::vector<int>& current)
{
  if (candidate.empty())
    return false;
  if (candidate.size() != current.size())
    return candidate.size() > current.size();
  return std::lexicographical_compare(
      candidate.begin(), candidate.end(), current.begin(), current.end());
}

class ExactCoulombCpuRecorder
{
public:
  void beginPhase(std::uint64_t phase, std::size_t maxParticipants)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mActivePhase != 0u) {
      // Parallel phases are synchronous and must not overlap. Fail closed if
      // that contract is ever violated instead of mixing residency evidence.
      mActivePhase = 0u;
      mActivePhaseLogicalCpuIds.clear();
      return;
    }
    mActivePhase = phase;
    mActivePhaseLogicalCpuIds.clear();
    mActivePhaseLogicalCpuIds.reserve(maxParticipants);
  }

  void observe(std::uint64_t phase)
  {
#if defined(__linux__)
    // Every pool participant executes at least one indexed callback. Observe
    // each participating thread once per scatter or gather phase so evidence
    // reflects actual callback residency without serializing every row.
    static thread_local std::uint64_t lastObservedPhase = 0u;
    if (lastObservedPhase == phase)
      return;
    lastObservedPhase = phase;

    const int logicalCpu = ::sched_getcpu();
    if (logicalCpu < 0)
      return;

    std::lock_guard<std::mutex> lock(mMutex);
    mLogicalCpuIds.insert(logicalCpu);
    if (mActivePhase != phase)
      return;
    if (std::find(
            mActivePhaseLogicalCpuIds.begin(),
            mActivePhaseLogicalCpuIds.end(),
            logicalCpu)
        == mActivePhaseLogicalCpuIds.end()) {
      mActivePhaseLogicalCpuIds.push_back(logicalCpu);
    }
#else
    static_cast<void>(phase);
#endif
  }

  void finishPhase(std::uint64_t phase, std::size_t participants)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mActivePhase != phase) {
      mActivePhase = 0u;
      mActivePhaseLogicalCpuIds.clear();
      return;
    }

    std::sort(
        mActivePhaseLogicalCpuIds.begin(), mActivePhaseLogicalCpuIds.end());
    // A one-participant phase is serial even when the overall row product's
    // other phase uses the pool. Discard impossible over-counts rather than
    // turning inconsistent diagnostics into evidence.
    if (participants > 1u && mActivePhaseLogicalCpuIds.size() <= participants
        && isPreferredExactCoulombCpuPhaseSet(
            mActivePhaseLogicalCpuIds, mMaxPhaseLogicalCpuIds)) {
      mMaxPhaseLogicalCpuIds = mActivePhaseLogicalCpuIds;
    }
    mActivePhase = 0u;
    mActivePhaseLogicalCpuIds.clear();
  }

  std::vector<int> getLogicalCpuIds() const
  {
    std::lock_guard<std::mutex> lock(mMutex);
    return {mLogicalCpuIds.begin(), mLogicalCpuIds.end()};
  }

  std::vector<int> getMaxPhaseLogicalCpuIds() const
  {
    std::lock_guard<std::mutex> lock(mMutex);
    return mMaxPhaseLogicalCpuIds;
  }

private:
  mutable std::mutex mMutex;
  std::set<int> mLogicalCpuIds;
  std::uint64_t mActivePhase = 0u;
  std::vector<int> mActivePhaseLogicalCpuIds;
  std::vector<int> mMaxPhaseLogicalCpuIds;
};

class ExactCoulombScopedParticipantAffinity
{
public:
  ExactCoulombScopedParticipantAffinity(
      bool enabled, std::size_t participant, std::size_t participantCount)
  {
#if defined(__linux__)
    if (!enabled || participant >= participantCount
        || ::sched_getaffinity(0, sizeof(mOriginal), &mOriginal) != 0) {
      return;
    }

    std::vector<int> allowedCpus;
    allowedCpus.reserve(participantCount);
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
      if (CPU_ISSET(cpu, &mOriginal))
        allowedCpus.push_back(cpu);
    }
    // Apply per-participant pinning only for an explicit N-CPU process mask.
    // A broader inherited mask retains normal scheduler behavior.
    if (allowedCpus.size() != participantCount)
      return;

    cpu_set_t target;
    CPU_ZERO(&target);
    CPU_SET(allowedCpus[participant], &target);
    mChanged = ::sched_setaffinity(0, sizeof(target), &target) == 0;
#else
    static_cast<void>(enabled);
    static_cast<void>(participant);
    static_cast<void>(participantCount);
#endif
  }

  ~ExactCoulombScopedParticipantAffinity()
  {
#if defined(__linux__)
    if (mChanged
        && ::sched_setaffinity(0, sizeof(mOriginal), &mOriginal) != 0) {
      dtwarn << "[ExactCoulombFbfConstraintSolver] Failed to restore a "
                "colored-BGS participant affinity mask.\n";
    }
#endif
  }

  ExactCoulombScopedParticipantAffinity(
      const ExactCoulombScopedParticipantAffinity&)
      = delete;
  ExactCoulombScopedParticipantAffinity& operator=(
      const ExactCoulombScopedParticipantAffinity&)
      = delete;

private:
#if defined(__linux__)
  cpu_set_t mOriginal{};
  bool mChanged = false;
#endif
};

std::uint64_t nextExactCoulombCpuObservationPhase()
{
  static std::atomic<std::uint64_t> nextPhase{1u};
  const std::uint64_t phase
      = nextPhase.fetch_add(1u, std::memory_order_relaxed);
  // Wrapping would require roughly one phase per nanosecond for centuries.
  // Reserve zero as the thread-local "not observed" sentinel regardless.
  return phase == 0u ? nextPhase.fetch_add(1u, std::memory_order_relaxed)
                     : phase;
}

std::mutex& exactCoulombContactRowParallelEvidenceMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<
    const ExactCoulombFbfConstraintSolver*,
    ExactCoulombContactRowParallelEvidence>&
exactCoulombContactRowParallelEvidenceBySolver()
{
  static auto* evidence = new std::unordered_map<
      const ExactCoulombFbfConstraintSolver*,
      ExactCoulombContactRowParallelEvidence>;
  return *evidence;
}

ExactCoulombContactRowParallelEvidence getContactRowParallelEvidence(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombContactRowParallelEvidenceMutex());
  const auto& evidenceBySolver
      = exactCoulombContactRowParallelEvidenceBySolver();
  const auto evidence = evidenceBySolver.find(solver);
  return evidence == evidenceBySolver.end()
             ? ExactCoulombContactRowParallelEvidence{}
             : evidence->second;
}

void resetLastContactRowParallelEvidence(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombContactRowParallelEvidenceMutex());
  auto& evidence = exactCoulombContactRowParallelEvidenceBySolver()[solver];
  evidence.products = 0u;
  evidence.parallelProducts = 0u;
  evidence.maxParticipants = 0u;
  evidence.logicalCpuIds.clear();
  evidence.maxPhaseLogicalCpuIds.clear();
}

void recordContactRowParallelEvidence(
    const ExactCoulombFbfConstraintSolver* solver,
    const ExactCoulombContactRowParallelEvidence& latest)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombContactRowParallelEvidenceMutex());
  auto& evidence = exactCoulombContactRowParallelEvidenceBySolver()[solver];
  evidence.products = latest.products;
  evidence.parallelProducts = latest.parallelProducts;
  evidence.maxParticipants = latest.maxParticipants;
  evidence.logicalCpuIds = latest.logicalCpuIds;
  evidence.maxPhaseLogicalCpuIds = latest.maxPhaseLogicalCpuIds;
  evidence.totalProducts += latest.products;
  evidence.totalParallelProducts += latest.parallelProducts;
  evidence.maxParticipantsEver
      = std::max(evidence.maxParticipantsEver, latest.maxParticipants);
  std::vector<int> logicalCpuIdsEver;
  logicalCpuIdsEver.reserve(
      evidence.logicalCpuIdsEver.size() + latest.logicalCpuIds.size());
  std::set_union(
      evidence.logicalCpuIdsEver.begin(),
      evidence.logicalCpuIdsEver.end(),
      latest.logicalCpuIds.begin(),
      latest.logicalCpuIds.end(),
      std::back_inserter(logicalCpuIdsEver));
  evidence.logicalCpuIdsEver = std::move(logicalCpuIdsEver);
  if (isPreferredExactCoulombCpuPhaseSet(
          latest.maxPhaseLogicalCpuIds, evidence.maxPhaseLogicalCpuIdsEver)) {
    evidence.maxPhaseLogicalCpuIdsEver = latest.maxPhaseLogicalCpuIds;
  }
}

void eraseContactRowParallelEvidence(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombContactRowParallelEvidenceMutex());
  exactCoulombContactRowParallelEvidenceBySolver().erase(solver);
}

std::mutex& exactCoulombColoredBlockGaussSeidelStateMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<
    const ExactCoulombFbfConstraintSolver*,
    ExactCoulombColoredBlockGaussSeidelState>&
exactCoulombColoredBlockGaussSeidelStateBySolver()
{
  static auto* states = new std::unordered_map<
      const ExactCoulombFbfConstraintSolver*,
      ExactCoulombColoredBlockGaussSeidelState>;
  return *states;
}

ExactCoulombColoredBlockGaussSeidelState
getExactCoulombColoredBlockGaussSeidelState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombColoredBlockGaussSeidelStateMutex());
  const auto& states = exactCoulombColoredBlockGaussSeidelStateBySolver();
  const auto state = states.find(solver);
  return state == states.end() ? ExactCoulombColoredBlockGaussSeidelState{}
                               : state->second;
}

void setExactCoulombColoredBlockGaussSeidelStateEnabled(
    const ExactCoulombFbfConstraintSolver* solver, bool enabled)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombColoredBlockGaussSeidelStateMutex());
  exactCoulombColoredBlockGaussSeidelStateBySolver()[solver].enabled = enabled;
}

void setExactCoulombColoredBlockGaussSeidelParticipantAffinityStateEnabled(
    const ExactCoulombFbfConstraintSolver* solver, bool enabled)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombColoredBlockGaussSeidelStateMutex());
  exactCoulombColoredBlockGaussSeidelStateBySolver()[solver]
      .participantAffinityEnabled
      = enabled;
}

void resetLastExactCoulombColoredBlockGaussSeidelState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombColoredBlockGaussSeidelStateMutex());
  auto& state = exactCoulombColoredBlockGaussSeidelStateBySolver()[solver];
  state.used = false;
  state.solves = 0u;
  state.dispatches = 0u;
  state.maxParticipants = 0u;
  state.manifoldCount = 0u;
  state.colorCount = 0u;
  state.maxManifoldsPerColor = 0u;
  state.logicalCpuIds.clear();
  state.maxPhaseLogicalCpuIds.clear();
}

void recordExactCoulombColoredBlockGaussSeidelState(
    const ExactCoulombFbfConstraintSolver* solver,
    const ExactCoulombColoredBlockGaussSeidelState& latest)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombColoredBlockGaussSeidelStateMutex());
  auto& state = exactCoulombColoredBlockGaussSeidelStateBySolver()[solver];
  state.used = latest.used;
  state.solves = latest.solves;
  state.dispatches = latest.dispatches;
  state.maxParticipants = latest.maxParticipants;
  state.manifoldCount = latest.manifoldCount;
  state.colorCount = latest.colorCount;
  state.maxManifoldsPerColor = latest.maxManifoldsPerColor;
  state.logicalCpuIds = latest.logicalCpuIds;
  state.maxPhaseLogicalCpuIds = latest.maxPhaseLogicalCpuIds;
}

void eraseExactCoulombColoredBlockGaussSeidelState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombColoredBlockGaussSeidelStateMutex());
  exactCoulombColoredBlockGaussSeidelStateBySolver().erase(solver);
}

std::mutex& exactCoulombPostCorrectionProjectionStateMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<const ExactCoulombFbfConstraintSolver*, bool>&
exactCoulombPostCorrectionProjectionStateBySolver()
{
  static auto* states
      = new std::unordered_map<const ExactCoulombFbfConstraintSolver*, bool>;
  return *states;
}

bool getExactCoulombPostCorrectionProjectionState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombPostCorrectionProjectionStateMutex());
  const auto& states = exactCoulombPostCorrectionProjectionStateBySolver();
  const auto state = states.find(solver);
  return state == states.end() ? true : state->second;
}

void setExactCoulombPostCorrectionProjectionState(
    const ExactCoulombFbfConstraintSolver* solver, bool enabled)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombPostCorrectionProjectionStateMutex());
  exactCoulombPostCorrectionProjectionStateBySolver()[solver] = enabled;
}

void resetExactCoulombPostCorrectionProjectionState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  setExactCoulombPostCorrectionProjectionState(solver, true);
}

void eraseExactCoulombPostCorrectionProjectionState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombPostCorrectionProjectionStateMutex());
  exactCoulombPostCorrectionProjectionStateBySolver().erase(solver);
}

std::mutex& exactCoulombSourceInnerInitializationStateMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<const ExactCoulombFbfConstraintSolver*, bool>&
exactCoulombSourceInnerInitializationStateBySolver()
{
  static auto* states
      = new std::unordered_map<const ExactCoulombFbfConstraintSolver*, bool>;
  return *states;
}

bool getExactCoulombSourceInnerInitializationState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombSourceInnerInitializationStateMutex());
  const auto& states = exactCoulombSourceInnerInitializationStateBySolver();
  const auto state = states.find(solver);
  return state == states.end() ? false : state->second;
}

void setExactCoulombSourceInnerInitializationState(
    const ExactCoulombFbfConstraintSolver* solver, bool enabled)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombSourceInnerInitializationStateMutex());
  exactCoulombSourceInnerInitializationStateBySolver()[solver] = enabled;
}

void resetExactCoulombSourceInnerInitializationState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  setExactCoulombSourceInnerInitializationState(solver, false);
}

void eraseExactCoulombSourceInnerInitializationState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(
      exactCoulombSourceInnerInitializationStateMutex());
  exactCoulombSourceInnerInitializationStateBySolver().erase(solver);
}

std::mutex& exactCoulombLastFailureStateMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<
    const ExactCoulombFbfConstraintSolver*,
    ExactCoulombLastFailureState>&
exactCoulombLastFailureStateBySolver()
{
  static auto* states = new std::unordered_map<
      const ExactCoulombFbfConstraintSolver*,
      ExactCoulombLastFailureState>;
  return *states;
}

ExactCoulombLastFailureState getExactCoulombLastFailureState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombLastFailureStateMutex());
  const auto& states = exactCoulombLastFailureStateBySolver();
  const auto state = states.find(solver);
  return state == states.end() ? ExactCoulombLastFailureState{} : state->second;
}

void resetExactCoulombLastFailureState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombLastFailureStateMutex());
  exactCoulombLastFailureStateBySolver()[solver]
      = ExactCoulombLastFailureState{};
}

void recordExactCoulombLastFailureState(
    const ExactCoulombFbfConstraintSolver* solver,
    ExactCoulombFbfConstraintSolverStatus status,
    detail::ExactCoulombConstraintBuildStatus buildStatus,
    std::size_t contactCount)
{
  std::lock_guard<std::mutex> lock(exactCoulombLastFailureStateMutex());
  auto& state = exactCoulombLastFailureStateBySolver()[solver];
  state.status = status;
  state.buildStatus = buildStatus;
  state.contactCount = contactCount;
}

void eraseExactCoulombLastFailureState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombLastFailureStateMutex());
  exactCoulombLastFailureStateBySolver().erase(solver);
}

std::mutex& exactCoulombPolicyStateMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<
    const ExactCoulombFbfConstraintSolver*,
    ExactCoulombPolicyState>&
exactCoulombPolicyStateBySolver()
{
  static auto* states = new std::unordered_map<
      const ExactCoulombFbfConstraintSolver*,
      ExactCoulombPolicyState>;
  return *states;
}

ExactCoulombPolicyState getExactCoulombPolicyState(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  const auto& states = exactCoulombPolicyStateBySolver();
  const auto state = states.find(solver);
  return state == states.end() ? ExactCoulombPolicyState{} : state->second;
}

void resetExactCoulombPolicyState(const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  exactCoulombPolicyStateBySolver()[solver] = ExactCoulombPolicyState{};
}

void setExactCoulombCrossStepPolicyStateOptions(
    const ExactCoulombFbfConstraintSolver* solver,
    const ExactCoulombFbfCrossStepPolicyOptions& options)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  auto& state = exactCoulombPolicyStateBySolver()[solver];
  state.crossStepOptions = options;
  state.lastInitialResidual = std::numeric_limits<double>::quiet_NaN();
  state.lastInitialNaturalMapResidual
      = std::numeric_limits<double>::quiet_NaN();
  state.lastNaturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  state.lastUncappedInitialStepSize = std::numeric_limits<double>::quiet_NaN();
  state.lastWarmStartStepSizeCapApplied = false;
  state.warmStartStepSizeCaps = 0u;
  state.unconvergedCacheSkips = 0u;
}

void setExactCoulombSourceContinuationStateOptions(
    const ExactCoulombFbfConstraintSolver* solver,
    const ExactCoulombFbfSourceContinuationOptions& options)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  auto& state = exactCoulombPolicyStateBySolver()[solver];
  state.sourceContinuationOptions = options;
  state.lastSourceContinuationActive = false;
  state.lastLineSearchShrinkCapReached = false;
  state.lastLineSearchShrinkCapCount = 0;
  state.lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  state.plateausAccepted = 0u;
  state.lineSearchShrinkCaps = 0u;
}

ExactCoulombPolicyState beginExactCoulombPolicyAttempt(
    const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  auto& state = exactCoulombPolicyStateBySolver()[solver];
  state.lastInitialResidual = std::numeric_limits<double>::quiet_NaN();
  state.lastInitialNaturalMapResidual
      = std::numeric_limits<double>::quiet_NaN();
  state.lastNaturalMapResidual = std::numeric_limits<double>::quiet_NaN();
  state.lastUncappedInitialStepSize = std::numeric_limits<double>::quiet_NaN();
  state.lastWarmStartStepSizeCapApplied = false;
  state.lastSourceContinuationActive = false;
  state.lastLineSearchShrinkCapReached = false;
  state.lastLineSearchShrinkCapCount = 0;
  state.lastInnerSolveStepSize = std::numeric_limits<double>::quiet_NaN();
  return state;
}

void recordExactCoulombPolicyAttempt(
    const ExactCoulombFbfConstraintSolver* solver,
    const math::detail::ExactCoulombFbfResult& result,
    bool unconvergedCacheSkipped,
    bool sourceContinuationActive,
    bool plateauAccepted)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  auto& state = exactCoulombPolicyStateBySolver()[solver];
  state.lastInitialResidual = result.initialResidual.value;
  state.lastInitialNaturalMapResidual = result.initialNaturalMapResidual;
  state.lastNaturalMapResidual = result.naturalMapResidual;
  state.lastUncappedInitialStepSize = result.uncappedInitialStepSize;
  state.lastWarmStartStepSizeCapApplied = result.initialStepSizeCapApplied;
  state.lastSourceContinuationActive = sourceContinuationActive;
  state.lastLineSearchShrinkCapReached = result.lineSearchShrinkCapReached;
  state.lastLineSearchShrinkCapCount = result.lineSearchShrinkCapCount;
  state.lastInnerSolveStepSize = result.lastInnerSolveStepSize;
  if (result.initialStepSizeCapApplied)
    ++state.warmStartStepSizeCaps;
  if (unconvergedCacheSkipped)
    ++state.unconvergedCacheSkips;
  if (result.lineSearchShrinkCapCount > 0) {
    state.lineSearchShrinkCaps
        += static_cast<std::size_t>(result.lineSearchShrinkCapCount);
  }
  if (plateauAccepted)
    ++state.plateausAccepted;
}

void eraseExactCoulombPolicyState(const ExactCoulombFbfConstraintSolver* solver)
{
  std::lock_guard<std::mutex> lock(exactCoulombPolicyStateMutex());
  exactCoulombPolicyStateBySolver().erase(solver);
}

bool isValidExactCoulombSourceContinuationOptions(
    const ExactCoulombFbfSourceContinuationOptions& options)
{
  return options.residualCheckInterval > 0 && options.plateauPatience >= 0
         && std::isfinite(options.plateauRelativeTolerance)
         && options.plateauRelativeTolerance >= 0.0
         && options.stepSizeBacktrackLimit > 0
         && std::isfinite(options.couplingVariationSkipThreshold)
         && options.couplingVariationSkipThreshold >= 0.0;
}

bool isValidExactCoulombCrossStepPolicyOptions(
    const ExactCoulombFbfCrossStepPolicyOptions& options)
{
  const bool validMatchMode
      = options.warmStartMatchMode
            == ExactCoulombFbfWarmStartMatchMode::EitherBodyLocalFeature
        || options.warmStartMatchMode
               == ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature;
  const bool hasWarmStartStepSizeCap
      = std::isfinite(options.warmStartResidualThreshold)
        && std::isfinite(options.warmStartStepSizeCap);
  const bool validWarmStartStepSizeCap
      = (std::isnan(options.warmStartResidualThreshold)
         && std::isnan(options.warmStartStepSizeCap))
        || (hasWarmStartStepSizeCap && options.warmStartResidualThreshold >= 0.0
            && options.warmStartStepSizeCap > 0.0);
  const bool validMinimumStepSize = std::isnan(options.minimumStepSize)
                                    || (std::isfinite(options.minimumStepSize)
                                        && options.minimumStepSize > 0.0);
  const bool validMaximumStepSize = std::isnan(options.maximumStepSize)
                                    || (std::isfinite(options.maximumStepSize)
                                        && options.maximumStepSize > 0.0);

  return validMatchMode && std::isfinite(options.warmStartNormalCosine)
         && options.warmStartNormalCosine >= -1.0
         && options.warmStartNormalCosine <= 1.0
         && options.warmStartMaxAge >= -1
         && std::isfinite(options.persistentStepSizeSafeBoundScale)
         && options.persistentStepSizeSafeBoundScale > 0.0
         && validMinimumStepSize && validMaximumStepSize
         && (std::isnan(options.minimumStepSize)
             || std::isnan(options.maximumStepSize)
             || options.minimumStepSize <= options.maximumStepSize)
         && validWarmStartStepSizeCap;
}

bool isValidExactCoulombOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  const bool validInitialStep = std::isnan(options.initialStepSize)
                                || (std::isfinite(options.initialStepSize)
                                    && options.initialStepSize > 0.0);
  const bool validLocalSolver
      = options.innerLocalSolver
            == ExactCoulombFbfLocalBlockSolver::InverseEuclideanProjection
        || options.innerLocalSolver
               == ExactCoulombFbfLocalBlockSolver::ExactMetricProjection
        || options.innerLocalSolver
               == ExactCoulombFbfLocalBlockSolver::ProjectedGradient;
  const bool validProjectedGradientOptions
      = options.innerLocalSolver
            != ExactCoulombFbfLocalBlockSolver::ProjectedGradient
        || (options.innerLocalIterations > 0
            && std::isfinite(options.innerLocalTolerance)
            && options.innerLocalTolerance >= 0.0
            && std::isfinite(options.innerDiagonalRegularization)
            && options.innerDiagonalRegularization >= 0.0);

  return options.maxOuterIterations >= 0 && std::isfinite(options.tolerance)
         && options.tolerance >= 0.0 && validInitialStep
         && std::isfinite(options.stepSizeRecoveryGrowthFactor)
         && options.stepSizeRecoveryGrowthFactor >= 1.0
         && std::isfinite(options.stepSizeScale) && options.stepSizeScale > 0.0
         && std::isfinite(options.outerRelaxation)
         && options.outerRelaxation > 0.0
         && std::isfinite(options.couplingVariationTolerance)
         && options.couplingVariationTolerance > 0.0
         && std::isfinite(options.shrinkFactor) && options.shrinkFactor > 0.0
         && options.shrinkFactor < 1.0 && options.maxStepShrinkIterations >= 0
         && options.spectralIterations > 0 && options.innerMaxSweeps >= 0
         && validLocalSolver && validProjectedGradientOptions
         && std::isfinite(options.innerTolerance)
         && options.innerTolerance >= 0.0
         && options.projectedGradientMaxIterations >= 0
         && std::isfinite(options.projectedGradientTolerance)
         && options.projectedGradientTolerance >= 0.0
         && options.denseResidualPolishIterations >= 0
         && options.denseResidualPolishLineSearchIterations >= 0
         && std::isfinite(options.denseResidualPolishRegularization)
         && options.denseResidualPolishRegularization >= 0.0
         && options.maxResidualHistorySamples >= 0
         && options.maxResidualHistoryRecords >= 0;
}

math::detail::CoulombConeResidual computeExactCoulombDenseResidual(
    const detail::ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    const math::detail::CoulombConeResidualScales& scales)
{
  const auto applyDelassus = [&problem](
                                 const Eigen::Ref<const Eigen::VectorXd>& input,
                                 Eigen::Ref<Eigen::VectorXd> output) {
    output.noalias() = problem.delassus * input;
  };

  return math::detail::computeExactCoulombContactResidualNormalFirst(
      problem.contactProblem, reaction, applyDelassus, scales);
}

bool computeExactCoulombDenseDualCorrection(
    const detail::ExactCoulombConstraintProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    Eigen::Ref<Eigen::VectorXd> correction)
{
  const Eigen::Index dimension = problem.contactProblem.getDimension();
  if (reaction.size() != dimension || correction.size() != dimension
      || !reaction.allFinite()) {
    return false;
  }

  Eigen::VectorXd velocity = problem.delassus * reaction;
  if (!velocity.allFinite()) {
    return false;
  }
  velocity += problem.contactProblem.freeVelocity;

  Eigen::VectorXd augmentedVelocity(dimension);
  if (!math::detail::computeExactCoulombAugmentedVelocityNormalFirst(
          velocity, problem.contactProblem.coefficients, augmentedVelocity)) {
    return false;
  }

  correction.setZero();
  for (Eigen::Index contact = 0;
       contact < problem.contactProblem.getContactCount();
       ++contact) {
    const Eigen::Index offset = 3 * contact;
    const Eigen::Vector3d augmented = augmentedVelocity.segment<3>(offset);
    const Eigen::Vector3d projected
        = math::detail::projectCoulombDualConeNormalFirst(
            augmented, problem.contactProblem.coefficients[contact]);
    correction.segment<3>(offset) = projected - augmented;
  }

  return correction.allFinite();
}

math::detail::ExactCoulombFbfOptions makeFbfOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  math::detail::ExactCoulombFbfOptions fbfOptions;
  fbfOptions.maxOuterIterations = options.maxOuterIterations;
  fbfOptions.tolerance = options.tolerance;
  fbfOptions.initialStepSize = options.initialStepSize;
  fbfOptions.capInitialStepSizeAtSafeBound
      = options.capInitialStepSizeAtSafeBound;
  fbfOptions.stepSizeScale = options.stepSizeScale;
  fbfOptions.outerRelaxation = options.outerRelaxation;
  fbfOptions.couplingVariationTolerance = options.couplingVariationTolerance;
  fbfOptions.shrinkFactor = options.shrinkFactor;
  fbfOptions.maxStepShrinkIterations = options.maxStepShrinkIterations;
  fbfOptions.enableAdaptiveStepSize = options.enableAdaptiveStepSize;
  fbfOptions.spectralIterations = options.spectralIterations;
  fbfOptions.maxResidualHistorySamples = options.maxResidualHistorySamples;
  return fbfOptions;
}

void applyExactCoulombCrossStepPolicy(
    const ExactCoulombFbfCrossStepPolicyOptions& policy,
    math::detail::ExactCoulombFbfOptions& options)
{
  options.explicitStepSizeSafeBoundScale
      = policy.persistentStepSizeSafeBoundScale;
  options.minimumStepSize = policy.minimumStepSize;
  options.maximumStepSize = policy.maximumStepSize;
  options.initialResidualStepSizeCapThreshold
      = policy.warmStartResidualThreshold;
  options.initialResidualStepSizeCap = policy.warmStartStepSizeCap;
  options.useNaturalMapResidualForInitialStepSizeCap
      = std::isfinite(policy.warmStartResidualThreshold)
        || policy.requireResidualImprovementForUnconvergedCacheSave;
}

void applyExactCoulombSourceContinuationPolicy(
    const ExactCoulombFbfSourceContinuationOptions& policy,
    math::detail::ExactCoulombFbfOptions& options)
{
  if (!policy.enabled)
    return;

  options.residualCheckInterval = policy.residualCheckInterval;
  options.plateauPatience = policy.plateauPatience;
  options.plateauRelativeTolerance = policy.plateauRelativeTolerance;
  options.useStrictToleranceComparison = true;
  options.useNaturalMapResidualForInitialConvergence = true;
  options.reportNonFiniteValuesSeparately = true;
  options.couplingVariationSkipThreshold
      = policy.couplingVariationSkipThreshold;
  options.maxStepShrinkIterations = policy.stepSizeBacktrackLimit;
  options.acceptStepSizeAfterFinalShrink = true;
}

math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions
makeFrozenConeOptions(const ExactCoulombFbfConstraintSolverOptions& options)
{
  math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions frozenOptions;
  frozenOptions.maxSweeps = options.innerMaxSweeps;
  switch (options.innerLocalSolver) {
    case ExactCoulombFbfLocalBlockSolver::InverseEuclideanProjection:
      frozenOptions.localSolver = math::detail::
          ExactCoulombFrozenConeLocalSolver::InverseEuclideanProjection;
      break;
    case ExactCoulombFbfLocalBlockSolver::ExactMetricProjection:
      frozenOptions.localSolver = math::detail::
          ExactCoulombFrozenConeLocalSolver::ExactMetricProjection;
      break;
    case ExactCoulombFbfLocalBlockSolver::ProjectedGradient:
      frozenOptions.localSolver
          = math::detail::ExactCoulombFrozenConeLocalSolver::ProjectedGradient;
      break;
  }
  frozenOptions.runFixedSweeps = options.runFixedInnerSweeps;
  frozenOptions.localIterations = options.innerLocalIterations;
  frozenOptions.tolerance = options.innerTolerance;
  frozenOptions.localTolerance = options.innerLocalTolerance;
  frozenOptions.diagonalRegularization = options.innerDiagonalRegularization;
  return frozenOptions;
}

math::detail::ExactCoulombFrozenConeOptions makeProjectedGradientOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  math::detail::ExactCoulombFrozenConeOptions projectedOptions;
  projectedOptions.maxIterations = options.projectedGradientMaxIterations;
  projectedOptions.tolerance = options.projectedGradientTolerance;
  projectedOptions.spectralIterations = options.spectralIterations;
  return projectedOptions;
}

class ExactCoulombColoredBlockGaussSeidelBarrier
{
public:
  explicit ExactCoulombColoredBlockGaussSeidelBarrier(
      std::size_t participantCount)
    : mParticipantCount(participantCount)
  {
    DART_ASSERT(participantCount > 0u);
  }

  void wait()
  {
    const std::size_t generation = mGeneration.load(std::memory_order_acquire);
    if (mArrivals.fetch_add(1u, std::memory_order_acq_rel) + 1u
        == mParticipantCount) {
      {
        // Pair the generation change with the waiters' predicate mutex. This
        // prevents a notify from racing between a failed predicate check and
        // the condition-variable wait.
        std::lock_guard<std::mutex> lock(mMutex);
        mArrivals.store(0u, std::memory_order_relaxed);
        mGeneration.fetch_add(1u, std::memory_order_release);
      }
      mCondition.notify_all();
      return;
    }

    // Colored contact work is intentionally fine grained. A short bounded
    // spin avoids a kernel sleep/wakeup on the normal balanced path, while
    // the predicate-locked condition-variable fallback remains available
    // when a participant is descheduled or heavily imbalanced.
    constexpr std::size_t kSpinCount = 16384u;
    for (std::size_t spin = 0u; spin < kSpinCount; ++spin) {
      if (mGeneration.load(std::memory_order_acquire) != generation)
        return;
    }

    std::unique_lock<std::mutex> lock(mMutex);
    mCondition.wait(lock, [&] {
      return mGeneration.load(std::memory_order_acquire) != generation;
    });
  }

private:
  const std::size_t mParticipantCount;
  std::atomic<std::size_t> mArrivals{0u};
  std::atomic<std::size_t> mGeneration{0u};
  std::mutex mMutex;
  std::condition_variable mCondition;
};

// Keeps the solver's persistent constraint-pool participants inside one
// exact-FBF attempt. The outer pool dispatch happens once; each frozen-cone
// solve then publishes a short-lived job to the already resident workers.
// This avoids one pool dispatch per FBF iteration while retaining a single
// owner for the pool and a deterministic participant index.
class ExactCoulombColoredBlockGaussSeidelExecutor
{
public:
  explicit ExactCoulombColoredBlockGaussSeidelExecutor(
      std::size_t participantCount)
    : mParticipantCount(participantCount), mStartBarrier(participantCount)
  {
    DART_ASSERT(participantCount > 0u);
  }

  template <typename Work>
  std::size_t run(Work& work)
  {
    if (mParticipantCount == 1u) {
      work(0u);
      return 1u;
    }

    {
      std::lock_guard<std::mutex> lock(mMutex);
      DART_ASSERT(!mJobActive);
      mJobActive = true;
      mJobCallable = static_cast<void*>(std::addressof(work));
      mJobInvoker = [](void* callable, std::size_t workerIndex) {
        (*static_cast<Work*>(callable))(workerIndex);
      };
      mCompletedWorkers = 0u;
      ++mJobGeneration;
    }
    mJobCondition.notify_all();

    work(0u);

    {
      std::unique_lock<std::mutex> lock(mMutex);
      mDoneCondition.wait(
          lock, [this] { return mCompletedWorkers + 1u == mParticipantCount; });
      mJobActive = false;
      mJobCallable = nullptr;
      mJobInvoker = nullptr;
    }
    return mParticipantCount;
  }

  template <typename MainWork>
  void runMain(MainWork& work)
  {
    mStartBarrier.wait();
    work();
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = true;
      ++mJobGeneration;
    }
    mJobCondition.notify_all();
  }

  void runWorker(std::size_t workerIndex)
  {
    mStartBarrier.wait();
    std::size_t observedGeneration = 0u;
    while (true) {
      void* callable = nullptr;
      JobInvoker invoker = nullptr;
      {
        std::unique_lock<std::mutex> lock(mMutex);
        mJobCondition.wait(lock, [&] {
          return mStop || observedGeneration != mJobGeneration;
        });
        if (mStop)
          return;

        observedGeneration = mJobGeneration;
        DART_ASSERT(mJobActive);
        callable = mJobCallable;
        invoker = mJobInvoker;
      }

      DART_ASSERT(callable != nullptr);
      DART_ASSERT(invoker != nullptr);
      invoker(callable, workerIndex);

      {
        std::lock_guard<std::mutex> lock(mMutex);
        ++mCompletedWorkers;
        if (mCompletedWorkers + 1u == mParticipantCount)
          mDoneCondition.notify_one();
      }
    }
  }

private:
  using JobInvoker = void (*)(void*, std::size_t);

  const std::size_t mParticipantCount;
  ExactCoulombColoredBlockGaussSeidelBarrier mStartBarrier;
  std::mutex mMutex;
  std::condition_variable mJobCondition;
  std::condition_variable mDoneCondition;
  bool mStop = false;
  bool mJobActive = false;
  std::size_t mJobGeneration = 0u;
  std::size_t mCompletedWorkers = 0u;
  void* mJobCallable = nullptr;
  JobInvoker mJobInvoker = nullptr;
};

bool isValidExactCoulombColoredBlockGaussSeidelSchedule(
    const detail::ExactCoulombContactRowOperator::
        ColoredBlockGaussSeidelSchedule& schedule,
    Eigen::Index contactCount)
{
  if (contactCount <= 0 || schedule.manifolds.empty() || schedule.colors.empty()
      || !schedule.hasUsableParallelism()) {
    return false;
  }

  std::vector<unsigned char> seenContacts(
      static_cast<std::size_t>(contactCount), 0u);
  for (const auto& manifold : schedule.manifolds) {
    if (manifold.contacts.empty()
        || !std::is_sorted(manifold.contacts.begin(), manifold.contacts.end())
        || !std::is_sorted(
            manifold.writeContacts.begin(), manifold.writeContacts.end())
        || std::adjacent_find(
               manifold.writeContacts.begin(), manifold.writeContacts.end())
               != manifold.writeContacts.end()) {
      return false;
    }
    for (const Eigen::Index contact : manifold.contacts) {
      if (contact < 0 || contact >= contactCount
          || seenContacts[static_cast<std::size_t>(contact)] != 0u) {
        return false;
      }
      seenContacts[static_cast<std::size_t>(contact)] = 1u;
    }
    for (const Eigen::Index contact : manifold.writeContacts) {
      if (contact < 0 || contact >= contactCount)
        return false;
    }
  }
  if (std::find(seenContacts.begin(), seenContacts.end(), 0u)
      != seenContacts.end()) {
    return false;
  }

  std::vector<unsigned char> seenManifolds(schedule.manifolds.size(), 0u);
  for (const auto& color : schedule.colors) {
    if (color.empty())
      return false;
    for (std::size_t first = 0u; first < color.size(); ++first) {
      const std::size_t manifoldIndex = color[first];
      if (manifoldIndex >= schedule.manifolds.size()
          || seenManifolds[manifoldIndex] != 0u) {
        return false;
      }
      seenManifolds[manifoldIndex] = 1u;

      const auto& firstWrites = schedule.manifolds[manifoldIndex].writeContacts;
      for (std::size_t second = 0u; second < first; ++second) {
        const auto& secondWrites
            = schedule.manifolds[color[second]].writeContacts;
        std::vector<Eigen::Index> intersection;
        std::set_intersection(
            firstWrites.begin(),
            firstWrites.end(),
            secondWrites.begin(),
            secondWrites.end(),
            std::back_inserter(intersection));
        if (!intersection.empty())
          return false;
      }
    }
  }
  return std::find(seenManifolds.begin(), seenManifolds.end(), 0u)
         == seenManifolds.end();
}

template <
    typename SerialDelassusOperator,
    typename DelassusBlockColumnOperator,
    typename ParallelForWorkers>
math::detail::ExactCoulombFrozenConeResult
solveExactCoulombFrozenConeColoredBlockGaussSeidel(
    const math::detail::ExactCoulombContactProblem& problem,
    const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
    const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
    double stepSizeGamma,
    const SerialDelassusOperator& applyDelassusSerial,
    const DelassusBlockColumnOperator& accumulateDelassusBlockColumns,
    const detail::ExactCoulombContactRowOperator::
        ColoredBlockGaussSeidelSchedule& schedule,
    std::size_t workerCount,
    ParallelForWorkers& parallelForWorkers,
    const math::detail::ExactCoulombFrozenConeBlockGaussSeidelOptions& options,
    std::size_t& participants)
{
  using LocalSolver = math::detail::ExactCoulombFrozenConeLocalSolver;
  using Status = math::detail::ExactCoulombFrozenConeStatus;

  math::detail::ExactCoulombFrozenConeResult result;
  result.reaction.resize(problem.getDimension());
  participants = 0u;

  if (!math::detail::isValidExactCoulombContactProblem(problem)
      || !math::detail::isValidExactCoulombFrozenConeBlockGaussSeidelOptions(
          options)
      || referenceReaction.size() != problem.getDimension()
      || frozenCoupling.size() != problem.getDimension()
      || !referenceReaction.allFinite() || !frozenCoupling.allFinite()
      || !std::isfinite(stepSizeGamma) || stepSizeGamma <= 0.0
      || workerCount == 0u
      || !isValidExactCoulombColoredBlockGaussSeidelSchedule(
          schedule, problem.getContactCount())) {
    result.status = Status::InvalidInput;
    return result;
  }

  const bool hasInitialReaction = options.initialReaction != nullptr;
  if (hasInitialReaction
      && (options.initialReaction->size() != problem.getDimension()
          || !options.initialReaction->allFinite())) {
    result.status = Status::InvalidInput;
    return result;
  }
  if (options.projectInitialReaction) {
    const bool projected
        = hasInitialReaction
              ? math::detail::projectExactCoulombReactionNormalFirst(
                  *options.initialReaction,
                  problem.coefficients,
                  result.reaction)
              : math::detail::projectExactCoulombReactionNormalFirst(
                  referenceReaction, problem.coefficients, result.reaction);
    if (!projected) {
      result.status = Status::InvalidInput;
      return result;
    }
  } else {
    if (hasInitialReaction)
      result.reaction = *options.initialReaction;
    else
      result.reaction = referenceReaction;
  }

  const Eigen::Index dimension = problem.getDimension();
  const Eigen::Index contactCount = problem.getContactCount();
  const auto* diagonalBlocks = options.cachedDiagonalBlocks;
  if (diagonalBlocks == nullptr
      || diagonalBlocks->size() != static_cast<std::size_t>(contactCount)) {
    // The constraint layer always supplies the row operator's cached blocks.
    // Fail closed instead of issuing full products from inside this path.
    result.status = Status::InvalidInput;
    return result;
  }

  const double inverseGamma = 1.0 / stepSizeGamma;
  if (!std::isfinite(inverseGamma)) {
    result.status = Status::InvalidInput;
    return result;
  }

  math::detail::ExactCoulombFrozenConeBlockGaussSeidelWorkspace
      localSystemWorkspace;
  auto* systemWorkspace = options.workspace == nullptr ? &localSystemWorkspace
                                                       : options.workspace;
  if (!systemWorkspace->prepareLocalSystems(
          *diagonalBlocks,
          problem.coefficients,
          stepSizeGamma,
          options.localSolver,
          options.diagonalRegularization)) {
    result.status = Status::InvalidInput;
    return result;
  }
  const auto& preparedLocalHessians = systemWorkspace->getLocalHessians();
  const auto& preparedLocalInverses = systemWorkspace->getLocalInverses();
  const auto& preparedLocalConeFactorizations
      = systemWorkspace->getLocalConeFactorizations();
  const auto& preparedLocalSteps = systemWorkspace->getLocalSteps();
  if (options.localSolver == LocalSolver::ProjectedGradient)
    result.stepSize = systemWorkspace->getMinimumLocalStep();

  Eigen::VectorXd localDelassusReaction;
  if (options.workspace == nullptr)
    localDelassusReaction.resize(dimension);
  Eigen::VectorXd& delassusReaction
      = options.workspace == nullptr
            ? localDelassusReaction
            : options.workspace->getDelassusReactionScratch(dimension);
  applyDelassusSerial(result.reaction, delassusReaction);
  if (!delassusReaction.allFinite()) {
    result.status = Status::InvalidInput;
    return result;
  }

  constexpr int kDelassusRefreshSweepInterval = 16;
  std::vector<double> contactDeltaSquared(
      static_cast<std::size_t>(contactCount), 0.0);
  std::vector<unsigned char> workerUpdated(workerCount, 0u);
  std::atomic<Eigen::Index> lowestFailedContact{contactCount};
  ExactCoulombColoredBlockGaussSeidelBarrier barrier(workerCount);
  bool stop = false;
  result.status = Status::MaxIterations;

  const auto recordFailure = [&lowestFailedContact](Eigen::Index contact) {
    Eigen::Index observed = lowestFailedContact.load(std::memory_order_relaxed);
    while (contact < observed
           && !lowestFailedContact.compare_exchange_weak(
               observed,
               contact,
               std::memory_order_relaxed,
               std::memory_order_relaxed)) {
      // Keep the smallest original contact index.
    }
  };

  auto runWorker = [&](std::size_t workerIndex) {
    for (int sweep = 0; sweep < options.maxSweeps; ++sweep) {
      if (workerIndex == 0u) {
        std::fill(contactDeltaSquared.begin(), contactDeltaSquared.end(), 0.0);
        std::fill(workerUpdated.begin(), workerUpdated.end(), 0u);
        lowestFailedContact.store(contactCount, std::memory_order_relaxed);
      }
      barrier.wait();

      for (const auto& color : schedule.colors) {
        for (std::size_t colorSlot = workerIndex; colorSlot < color.size();
             colorSlot += workerCount) {
          const auto& manifold = schedule.manifolds[color[colorSlot]];
          bool manifoldFailed = false;
          for (const Eigen::Index contact : manifold.contacts) {
            const Eigen::Index offset = 3 * contact;
            const std::size_t blockIndex = static_cast<std::size_t>(contact);
            const Eigen::Matrix3d& localHessian
                = preparedLocalHessians[blockIndex];

            const Eigen::Vector3d previousBlock
                = result.reaction.segment<3>(offset);
            const Eigen::Vector3d previousGradient
                = delassusReaction.segment<3>(offset)
                  + problem.freeVelocity.segment<3>(offset)
                  + frozenCoupling.segment<3>(offset)
                  + inverseGamma
                        * (previousBlock
                           - referenceReaction.segment<3>(offset));
            if (!previousGradient.allFinite()) {
              recordFailure(contact);
              manifoldFailed = true;
              break;
            }

            const Eigen::Vector3d localLinearTerm
                = previousGradient - localHessian * previousBlock;
            Eigen::Vector3d block;
            bool localSolved = false;
            if (options.localSolver
                == LocalSolver::InverseEuclideanProjection) {
              localSolved = math::detail::
                  solveExactCoulombLocalConeInverseEuclideanProjection(
                      preparedLocalInverses[blockIndex],
                      localLinearTerm,
                      problem.coefficients[contact],
                      block);
            } else if (
                options.localSolver == LocalSolver::ExactMetricProjection) {
              localSolved = math::detail::solveExactCoulombLocalConeQuadratic(
                  preparedLocalConeFactorizations[blockIndex],
                  localLinearTerm,
                  block);
            } else {
              block = previousBlock;
              const double localStep = preparedLocalSteps[blockIndex];
              for (int localIteration = 0;
                   localIteration < options.localIterations;
                   ++localIteration) {
                const Eigen::Vector3d localGradient
                    = previousGradient + localHessian * (block - previousBlock);
                const Eigen::Vector3d candidate
                    = block - localStep * localGradient;
                const Eigen::Vector3d nextBlock
                    = math::detail::projectCoulombConeNormalFirst(
                        candidate, problem.coefficients[contact]);
                const double localChange
                    = (nextBlock - block).norm() / (1.0 + nextBlock.norm());
                block = nextBlock;
                if (localChange <= options.localTolerance)
                  break;
              }
              localSolved = block.allFinite();
            }
            if (!localSolved) {
              recordFailure(contact);
              manifoldFailed = true;
              break;
            }

            const Eigen::Vector3d delta = block - previousBlock;
            if (delta.norm() == 0.0)
              continue;

            result.reaction.segment<3>(offset) = block;
            accumulateDelassusBlockColumns(contact, delta, delassusReaction);
            workerUpdated[workerIndex] = 1u;
            contactDeltaSquared[blockIndex] = delta.squaredNorm();
          }
          if (manifoldFailed)
            continue;
        }
        barrier.wait();
        if (lowestFailedContact.load(std::memory_order_relaxed)
            < contactCount) {
          if (workerIndex == 0u)
            result.status = Status::InvalidInput;
          return;
        }
      }

      if (workerIndex == 0u) {
        double sweepChangeSquared = 0.0;
        for (const double deltaSquared : contactDeltaSquared)
          sweepChangeSquared += deltaSquared;

        const bool sweepUpdated
            = std::find(workerUpdated.begin(), workerUpdated.end(), 1u)
              != workerUpdated.end();
        result.fixedPointError
            = std::sqrt(sweepChangeSquared) / (1.0 + result.reaction.norm());
        result.iterations = sweep + 1;
        if (!options.runFixedSweeps
            && result.fixedPointError <= options.tolerance) {
          result.status = Status::Success;
          stop = true;
        } else if (
            sweepUpdated && (sweep + 1) % kDelassusRefreshSweepInterval == 0) {
          // This callback already owns the persistent constraint pool. Keep
          // the refresh on worker zero's serial row operator to avoid nested
          // pool dispatch while the remaining workers wait at the barrier.
          applyDelassusSerial(result.reaction, delassusReaction);
          if (!delassusReaction.allFinite()) {
            result.status = Status::InvalidInput;
            stop = true;
          }
        }
      }
      barrier.wait();
      if (stop)
        return;
    }
  };

  participants = parallelForWorkers(workerCount, runWorker);
  if (participants != workerCount) {
    result.status = Status::InvalidInput;
    return result;
  }
  return result;
}

} // namespace

//==============================================================================
ExactCoulombFbfConstraintSolver::ExactCoulombFbfConstraintSolver()
{
  resetLastContactRowParallelEvidence(this);
  resetLastExactCoulombColoredBlockGaussSeidelState(this);
  resetExactCoulombPostCorrectionProjectionState(this);
  resetExactCoulombSourceInnerInitializationState(this);
  resetExactCoulombLastFailureState(this);
  resetExactCoulombPolicyState(this);
}

//==============================================================================
ExactCoulombFbfConstraintSolver::ExactCoulombFbfConstraintSolver(
    const ExactCoulombFbfConstraintSolverOptions& options)
  : mExactCoulombOptions(options)
{
  resetLastContactRowParallelEvidence(this);
  resetLastExactCoulombColoredBlockGaussSeidelState(this);
  resetExactCoulombPostCorrectionProjectionState(this);
  resetExactCoulombSourceInnerInitializationState(this);
  resetExactCoulombLastFailureState(this);
  resetExactCoulombPolicyState(this);
}

//==============================================================================
ExactCoulombFbfConstraintSolver::~ExactCoulombFbfConstraintSolver()
{
  eraseContactRowParallelEvidence(this);
  eraseExactCoulombColoredBlockGaussSeidelState(this);
  eraseExactCoulombPostCorrectionProjectionState(this);
  eraseExactCoulombSourceInnerInitializationState(this);
  eraseExactCoulombLastFailureState(this);
  eraseExactCoulombPolicyState(this);
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::setExactCoulombOptions(
    const ExactCoulombFbfConstraintSolverOptions& options)
{
  if (!isValidExactCoulombOptions(options)) {
    dtwarn << "[ExactCoulombFbfConstraintSolver] Ignoring invalid exact "
           << "Coulomb FBF options.\n";
    return;
  }

  mExactCoulombOptions = options;
  // A new option bundle may change the operator, safe-step estimate, or
  // recovery policy. Keep reaction warm starts, but do not carry gamma state
  // across that configuration boundary.
  clearExactCoulombPersistentStepSize();
  if (!mExactCoulombOptions.enableWarmStart) {
    clearExactCoulombWarmStart();
  }
}

//==============================================================================
const ExactCoulombFbfConstraintSolverOptions&
ExactCoulombFbfConstraintSolver::getExactCoulombOptions() const
{
  return mExactCoulombOptions;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::setExactCoulombCrossStepPolicyOptions(
    const ExactCoulombFbfCrossStepPolicyOptions& options)
{
  if (!isValidExactCoulombCrossStepPolicyOptions(options)) {
    dtwarn << "[ExactCoulombFbfConstraintSolver] Ignoring invalid exact "
           << "Coulomb cross-step policy options.\n";
    return;
  }

  setExactCoulombCrossStepPolicyStateOptions(this, options);
  // Contact identity and cache-retention rules can change across this boundary.
  // Do not reinterpret reactions or gamma tokens created by another policy.
  clearExactCoulombWarmStart();
}

//==============================================================================
ExactCoulombFbfCrossStepPolicyOptions
ExactCoulombFbfConstraintSolver::getExactCoulombCrossStepPolicyOptions() const
{
  return getExactCoulombPolicyState(this).crossStepOptions;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::setExactCoulombSourceContinuationOptions(
    const ExactCoulombFbfSourceContinuationOptions& options)
{
  if (!isValidExactCoulombSourceContinuationOptions(options)) {
    dtwarn << "[ExactCoulombFbfConstraintSolver] Ignoring invalid exact "
           << "Coulomb source-continuation options.\n";
    return;
  }

  setExactCoulombSourceContinuationStateOptions(this, options);
  // The policy changes accepted solver outcomes, residual sampling, and gamma
  // evolution. Never reinterpret cross-step state created under another loop.
  clearExactCoulombWarmStart();
}

//==============================================================================
ExactCoulombFbfSourceContinuationOptions
ExactCoulombFbfConstraintSolver::getExactCoulombSourceContinuationOptions()
    const
{
  return getExactCoulombPolicyState(this).sourceContinuationOptions;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombSourceContinuationActive() const
{
  return getExactCoulombPolicyState(this).lastSourceContinuationActive;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    setExactCoulombPostCorrectionProjectionEnabled(bool enabled)
{
  if (getExactCoulombPostCorrectionProjectionState(this) == enabled)
    return;

  setExactCoulombPostCorrectionProjectionState(this, enabled);
  clearExactCoulombPersistentStepSize();
  clearExactCoulombWarmStart();
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getExactCoulombPostCorrectionProjectionEnabled() const
{
  return getExactCoulombPostCorrectionProjectionState(this);
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    setExactCoulombSourceInnerInitializationEnabled(bool enabled)
{
  if (getExactCoulombSourceInnerInitializationState(this) == enabled)
    return;

  setExactCoulombSourceInnerInitializationState(this, enabled);
  clearExactCoulombPersistentStepSize();
  clearExactCoulombWarmStart();
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getExactCoulombSourceInnerInitializationEnabled() const
{
  return getExactCoulombSourceInnerInitializationState(this);
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::setFromOtherConstraintSolver(
    const ConstraintSolver& other)
{
  BoxedLcpConstraintSolver::setFromOtherConstraintSolver(other);

  if (const auto* exact
      = dynamic_cast<const ExactCoulombFbfConstraintSolver*>(&other)) {
    setExactCoulombOptions(exact->getExactCoulombOptions());
    setExactCoulombCrossStepPolicyOptions(
        exact->getExactCoulombCrossStepPolicyOptions());
    setExactCoulombSourceContinuationOptions(
        exact->getExactCoulombSourceContinuationOptions());
    setExactCoulombPostCorrectionProjectionEnabled(
        exact->getExactCoulombPostCorrectionProjectionEnabled());
    setExactCoulombSourceInnerInitializationEnabled(
        exact->getExactCoulombSourceInnerInitializationEnabled());
    setExactCoulombColoredBlockGaussSeidelEnabled(
        exact->getExactCoulombColoredBlockGaussSeidelEnabled());
    setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
        exact
            ->getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled());
  } else {
    setExactCoulombCrossStepPolicyOptions(
        ExactCoulombFbfCrossStepPolicyOptions{});
    setExactCoulombSourceContinuationOptions(
        ExactCoulombFbfSourceContinuationOptions{});
    setExactCoulombPostCorrectionProjectionEnabled(true);
    setExactCoulombSourceInnerInitializationEnabled(false);
    setExactCoulombColoredBlockGaussSeidelEnabled(false);
    setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(false);
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    setExactCoulombColoredBlockGaussSeidelEnabled(bool enabled)
{
  setExactCoulombColoredBlockGaussSeidelStateEnabled(this, enabled);
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getExactCoulombColoredBlockGaussSeidelEnabled() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).enabled;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    setExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled(
        bool enabled)
{
  setExactCoulombColoredBlockGaussSeidelParticipantAffinityStateEnabled(
      this, enabled);
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getExactCoulombColoredBlockGaussSeidelParticipantAffinityEnabled() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this)
      .participantAffinityEnabled;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelUsed() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).used;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelSolves() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).solves;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelDispatches() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).dispatches;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelParticipants() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).maxParticipants;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelManifolds() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).manifoldCount;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelColors() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).colorCount;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelMaxManifoldsPerColor() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).maxManifoldsPerColor;
}

//==============================================================================
std::vector<int> ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelLogicalCpuIds() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this).logicalCpuIds;
}

//==============================================================================
std::vector<int> ExactCoulombFbfConstraintSolver::
    getLastExactCoulombColoredBlockGaussSeidelMaxPhaseLogicalCpuIds() const
{
  return getExactCoulombColoredBlockGaussSeidelState(this)
      .maxPhaseLogicalCpuIds;
}

//==============================================================================
ExactCoulombFbfConstraintSolverStatus
ExactCoulombFbfConstraintSolver::getLastExactCoulombStatus() const
{
  return mLastExactCoulombStatus;
}

//==============================================================================
detail::ExactCoulombConstraintBuildStatus
ExactCoulombFbfConstraintSolver::getLastExactCoulombBuildStatus() const
{
  return mLastExactCoulombBuildStatus;
}

//==============================================================================
math::detail::ExactCoulombFbfStatus
ExactCoulombFbfConstraintSolver::getLastExactCoulombFbfStatus() const
{
  return mLastExactCoulombFbfStatus;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombResidual() const
{
  return mLastExactCoulombResidual;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombBestResidual() const
{
  return mLastExactCoulombBestResidual;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastExactCoulombBestIteration() const
{
  return mLastExactCoulombBestIteration;
}

//==============================================================================
const math::detail::CoulombConeResidual&
ExactCoulombFbfConstraintSolver::getLastExactCoulombResidualDetails() const
{
  return mLastExactCoulombResidualDetails;
}

const std::vector<math::detail::ExactCoulombFbfResidualSample>&
ExactCoulombFbfConstraintSolver::getLastExactCoulombResidualHistory() const
{
  return mLastExactCoulombResidualHistory;
}

//==============================================================================
const std::vector<ExactCoulombFbfResidualHistoryRecord>&
ExactCoulombFbfConstraintSolver::getExactCoulombResidualHistoryRecords() const
{
  return mExactCoulombResidualHistoryRecords;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::clearExactCoulombResidualHistoryRecords()
{
  mExactCoulombResidualHistoryRecords.clear();
  mResidualHistoryRecordSequence = 0u;
}

//==============================================================================
ExactCoulombFbfConstraintSolverStatus
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombStatus() const
{
  return getExactCoulombLastFailureState(this).status;
}

//==============================================================================
detail::ExactCoulombConstraintBuildStatus
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombBuildStatus() const
{
  return getExactCoulombLastFailureState(this).buildStatus;
}

//==============================================================================
math::detail::ExactCoulombFbfStatus
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombFbfStatus() const
{
  return mLastFailedExactCoulombFbfStatus;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombResidual()
    const
{
  return mLastFailedExactCoulombResidual;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombBestResidual()
    const
{
  return mLastFailedExactCoulombBestResidual;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombBestIteration()
    const
{
  return mLastFailedExactCoulombBestIteration;
}

//==============================================================================
const math::detail::CoulombConeResidual&
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombResidualDetails()
    const
{
  return mLastFailedExactCoulombResidualDetails;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombContactCount() const
{
  return getExactCoulombLastFailureState(this).contactCount;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombIterations() const
{
  return mLastFailedExactCoulombIterations;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastExactCoulombIterations() const
{
  return mLastExactCoulombIterations;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombStepSize() const
{
  return mLastExactCoulombStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombInnerSolveStepSize()
    const
{
  return getExactCoulombPolicyState(this).lastInnerSolveStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombSafeStepSize() const
{
  return mLastExactCoulombSafeStepSize;
}

//==============================================================================
double
ExactCoulombFbfConstraintSolver::getLastExactCoulombCouplingVariationRatio()
    const
{
  return mLastExactCoulombCouplingVariationRatio;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastExactCoulombShrinkIterations() const
{
  return mLastExactCoulombShrinkIterations;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombLineSearchShrinkCapReached() const
{
  return getExactCoulombPolicyState(this).lastLineSearchShrinkCapReached;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::
    getLastExactCoulombLineSearchShrinkCapCount() const
{
  return getExactCoulombPolicyState(this).lastLineSearchShrinkCapCount;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombStepSize()
    const
{
  return mLastFailedExactCoulombStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombSafeStepSize()
    const
{
  return mLastFailedExactCoulombSafeStepSize;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::
    getLastFailedExactCoulombCouplingVariationRatio() const
{
  return mLastFailedExactCoulombCouplingVariationRatio;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getLastFailedExactCoulombShrinkIterations()
    const
{
  return mLastFailedExactCoulombShrinkIterations;
}

//==============================================================================
int ExactCoulombFbfConstraintSolver::getMaxExactCoulombIterations() const
{
  return mMaxExactCoulombIterations;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getTotalExactCoulombIterations()
    const
{
  return mTotalExactCoulombIterations;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombProjectedGradientRetryUsed() const
{
  return mLastExactCoulombProjectedGradientRetryUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombDenseResidualPolishUsed() const
{
  return mLastExactCoulombDenseResidualPolishUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombMatrixFreeDelassusOperatorUsed() const
{
  return mLastExactCoulombMatrixFreeDelassusOperatorUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombContactRowOperatorUsed() const
{
  return mLastExactCoulombContactRowOperatorUsed;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getLastExactCoulombContactRowDelassusProducts()
    const
{
  return getContactRowParallelEvidence(this).products;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getLastExactCoulombParallelContactRowDelassusProducts() const
{
  return getContactRowParallelEvidence(this).parallelProducts;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getLastExactCoulombMaxContactRowParticipants()
    const
{
  return getContactRowParallelEvidence(this).maxParticipants;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombContactRowDelassusProducts()
    const
{
  return getContactRowParallelEvidence(this).totalProducts;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::
    getNumExactCoulombParallelContactRowDelassusProducts() const
{
  return getContactRowParallelEvidence(this).totalParallelProducts;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getMaxExactCoulombContactRowParticipants()
    const
{
  return getContactRowParallelEvidence(this).maxParticipantsEver;
}

//==============================================================================
std::vector<int>
ExactCoulombFbfConstraintSolver::getLastExactCoulombContactRowLogicalCpuIds()
    const
{
  return getContactRowParallelEvidence(this).logicalCpuIds;
}

//==============================================================================
std::vector<int>
ExactCoulombFbfConstraintSolver::getExactCoulombContactRowLogicalCpuIds() const
{
  return getContactRowParallelEvidence(this).logicalCpuIdsEver;
}

//==============================================================================
std::vector<int> ExactCoulombFbfConstraintSolver::
    getLastExactCoulombMaxPhaseContactRowLogicalCpuIds() const
{
  return getContactRowParallelEvidence(this).maxPhaseLogicalCpuIds;
}

//==============================================================================
std::vector<int> ExactCoulombFbfConstraintSolver::
    getMaxExactCoulombPhaseContactRowLogicalCpuIds() const
{
  return getContactRowParallelEvidence(this).maxPhaseLogicalCpuIdsEver;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombDenseContactRowSnapshotAssembled() const
{
  return mLastExactCoulombDenseContactRowSnapshotAssembled;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombMatrixFreeDelassusSeedUsed() const
{
  return mLastExactCoulombMatrixFreeDelassusSeedUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::getLastExactCoulombWarmStartUsed() const
{
  return mLastExactCoulombWarmStartUsed;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombPersistentStepSizeUsed() const
{
  return mLastExactCoulombPersistentStepSizeUsed;
}

//==============================================================================
double
ExactCoulombFbfConstraintSolver::getLastExactCoulombPersistentStepSizeRequest()
    const
{
  return mLastExactCoulombPersistentStepSizeRequest;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getLastExactCoulombWarmStartMatchedContacts()
    const
{
  return mLastExactCoulombWarmStartMatchedContacts;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombInitialResidual()
    const
{
  return getExactCoulombPolicyState(this).lastInitialResidual;
}

//==============================================================================
double
ExactCoulombFbfConstraintSolver::getLastExactCoulombInitialNaturalMapResidual()
    const
{
  return getExactCoulombPolicyState(this).lastInitialNaturalMapResidual;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getLastExactCoulombNaturalMapResidual()
    const
{
  return getExactCoulombPolicyState(this).lastNaturalMapResidual;
}

//==============================================================================
double
ExactCoulombFbfConstraintSolver::getLastExactCoulombUncappedInitialStepSize()
    const
{
  return getExactCoulombPolicyState(this).lastUncappedInitialStepSize;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::
    getLastExactCoulombWarmStartStepSizeCapApplied() const
{
  return getExactCoulombPolicyState(this).lastWarmStartStepSizeCapApplied;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombWarmStartStepSizeCaps() const
{
  return getExactCoulombPolicyState(this).warmStartStepSizeCaps;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombUnconvergedCacheSkips() const
{
  return getExactCoulombPolicyState(this).unconvergedCacheSkips;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombSolves() const
{
  return mNumExactCoulombSolves;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombAttempts() const
{
  return mNumExactCoulombAttempts;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombMaxIterationsAccepted() const
{
  return mNumExactCoulombMaxIterationsAccepted;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombPlateausAccepted() const
{
  return getExactCoulombPolicyState(this).plateausAccepted;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombLineSearchShrinkCaps() const
{
  return getExactCoulombPolicyState(this).lineSearchShrinkCaps;
}

//==============================================================================
double ExactCoulombFbfConstraintSolver::getWorstExactCoulombResidual() const
{
  return mWorstExactCoulombResidual;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumBoxedLcpFallbacks() const
{
  return mNumBoxedLcpFallbacks;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombFailures() const
{
  return mNumExactCoulombFailures;
}

//==============================================================================
std::size_t ExactCoulombFbfConstraintSolver::getNumExactCoulombWarmStarts()
    const
{
  return mNumExactCoulombWarmStarts;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombProjectedGradientRetries()
    const
{
  return mNumExactCoulombProjectedGradientRetries;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombPersistentStepSizeRetries()
    const
{
  return mNumExactCoulombPersistentStepSizeRetries;
}

//==============================================================================
std::size_t
ExactCoulombFbfConstraintSolver::getNumExactCoulombDenseResidualPolishes() const
{
  return mNumExactCoulombDenseResidualPolishes;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::solveConstrainedGroup(
    ConstrainedGroup& group)
{
  if (trySolveExactCoulombConstrainedGroup(group)) {
    return;
  }

  if (!mExactCoulombOptions.fallbackToBoxedLcp) {
    return;
  }

  ++mNumBoxedLcpFallbacks;
  mLastExactCoulombStatus
      = ExactCoulombFbfConstraintSolverStatus::BoxedLcpFallback;
  BoxedLcpConstraintSolver::solveConstrainedGroup(group);
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::trySolveExactCoulombConstrainedGroup(
    ConstrainedGroup& group)
{
  ExactCoulombContactRowParallelEvidence contactRowParallelEvidence;
  ExactCoulombCpuRecorder contactRowCpuRecorder;
  ExactCoulombCpuRecorder coloredBlockGaussSeidelCpuRecorder;
  resetLastContactRowParallelEvidence(this);
  resetLastExactCoulombColoredBlockGaussSeidelState(this);
  const ExactCoulombPolicyState policyState
      = beginExactCoulombPolicyAttempt(this);
  ExactCoulombColoredBlockGaussSeidelState coloredBlockGaussSeidelState
      = getExactCoulombColoredBlockGaussSeidelState(this);
  const ExactCoulombFbfCrossStepPolicyOptions crossStepPolicy
      = policyState.crossStepOptions;
  const ExactCoulombFbfSourceContinuationOptions sourceContinuationPolicy
      = policyState.sourceContinuationOptions;
  ++mNumExactCoulombAttempts;
  mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::NotRun;
  mLastExactCoulombBuildStatus
      = detail::ExactCoulombConstraintBuildStatus::EmptyInput;
  mLastExactCoulombFbfStatus
      = math::detail::ExactCoulombFbfStatus::InvalidInput;
  mLastExactCoulombResidual = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombResidualDetails
      = math::detail::makeInvalidCoulombConeResidual();
  mLastExactCoulombBestResidual = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombBestIteration = 0;
  mLastExactCoulombResidualHistory.clear();
  mLastExactCoulombIterations = 0;
  mLastExactCoulombStepSize = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombSafeStepSize = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombCouplingVariationRatio
      = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombShrinkIterations = 0;
  mLastExactCoulombProjectedGradientRetryUsed = false;
  mLastExactCoulombDenseResidualPolishUsed = false;
  mLastExactCoulombMatrixFreeDelassusOperatorUsed = false;
  mLastExactCoulombContactRowOperatorUsed = false;
  mLastExactCoulombDenseContactRowSnapshotAssembled = false;
  mLastExactCoulombMatrixFreeDelassusSeedUsed = false;
  mLastExactCoulombWarmStartUsed = false;
  mLastExactCoulombPersistentStepSizeUsed = false;
  mLastExactCoulombPersistentStepSizeRequest
      = std::numeric_limits<double>::quiet_NaN();
  mLastExactCoulombWarmStartMatchedContacts = 0u;

  if (!isValidExactCoulombOptions(mExactCoulombOptions)) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::InvalidOptions;
    clearExactCoulombWarmStart();
    recordLastFailedExactCoulombAttempt(
        math::detail::ExactCoulombFbfStatus::InvalidInput,
        mLastExactCoulombResidualDetails,
        mLastExactCoulombIterations);
    ++mNumExactCoulombFailures;
    return false;
  }

  if (!std::isfinite(mTimeStep) || mTimeStep <= 0.0) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::InvalidOptions;
    clearExactCoulombWarmStart();
    recordLastFailedExactCoulombAttempt(
        math::detail::ExactCoulombFbfStatus::InvalidInput,
        mLastExactCoulombResidualDetails,
        mLastExactCoulombIterations);
    ++mNumExactCoulombFailures;
    return false;
  }

  std::vector<ConstraintBase*> constraints;
  constraints.reserve(group.getNumConstraints());
  for (std::size_t i = 0u; i < group.getNumConstraints(); ++i) {
    constraints.push_back(group.getConstraint(i).get());
  }

  detail::ExactCoulombConstraintBuildOptions buildOptions;
  buildOptions.invTimeStep = 1.0 / mTimeStep;
  // Honor the solver's split-impulse setting: with split impulse enabled the
  // velocity-phase right-hand side excludes DART's ERP position-correction
  // bias (position recovery runs as a separate pseudo-impulse phase), which
  // matches the paper's formulation where the exact friction solve sees pure
  // dynamics.
  buildOptions.useSplitImpulse = isSplitImpulseEnabled();
  buildOptions.includeConstraintRegularization
      = mExactCoulombOptions.includeConstraintRegularization;
  const bool useMatrixFreeDelassusSeed
      = mExactCoulombOptions.seedNormalImpulseFromDiagonal
        && mExactCoulombOptions.useMatrixFreeDelassusSeed;
  buildOptions.seedNormalImpulseFromDiagonal
      = mExactCoulombOptions.seedNormalImpulseFromDiagonal
        && !useMatrixFreeDelassusSeed;

  // The scratch-backed contact-row operator represents the pure reduced
  // operator `J M^-1 J^T`, so CFM/slip-regularized staging keeps the
  // impulse-test assembly route.
  const bool attemptContactRowOperator
      = mExactCoulombOptions.useContactRowDelassusOperator
        && !mExactCoulombOptions.includeConstraintRegularization;
  const bool deferDenseSeed
      = buildOptions.seedNormalImpulseFromDiagonal && attemptContactRowOperator;
  buildOptions.assembleDenseDelassus = !attemptContactRowOperator;
  buildOptions.seedNormalImpulseFromDiagonal
      = buildOptions.seedNormalImpulseFromDiagonal && !deferDenseSeed;

  auto problem
      = detail::buildExactCoulombConstraintProblem(constraints, buildOptions);
  mLastExactCoulombBuildStatus = problem.status;
  if (problem.status != detail::ExactCoulombConstraintBuildStatus::Success) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::UnsupportedProblem;
    invalidateExactCoulombPersistentStepSizeForGroup(constraints);
    invalidateExactCoulombWarmStartPointerCache();
    recordLastFailedExactCoulombAttempt(
        math::detail::ExactCoulombFbfStatus::InvalidInput,
        mLastExactCoulombResidualDetails,
        mLastExactCoulombIterations);
    ++mNumExactCoulombFailures;
    return false;
  }

  detail::ExactCoulombContactRowOperator contactRowOperator;
  if (attemptContactRowOperator) {
    bool rowOperatorReady = contactRowOperator.build(constraints)
                            && contactRowOperator.getDimension()
                                   == problem.contactProblem.getDimension();
    if (rowOperatorReady
        && mExactCoulombOptions.assembleDenseContactRowSnapshot) {
      contactRowOperator.assembleDense(problem.delassus);
      rowOperatorReady = problem.delassus.allFinite();
      mLastExactCoulombDenseContactRowSnapshotAssembled = rowOperatorReady;
    }

    if (!rowOperatorReady) {
      // Unsupported or degenerate row group: restore the general
      // impulse-test snapshot so every downstream path behaves as before.
      contactRowOperator.clear();
      if (!detail::assembleExactCoulombConstraintDelassusByImpulseTests(
              problem, mExactCoulombOptions.includeConstraintRegularization)) {
        mLastExactCoulombBuildStatus
            = detail::ExactCoulombConstraintBuildStatus::NonFiniteData;
        mLastExactCoulombStatus
            = ExactCoulombFbfConstraintSolverStatus::UnsupportedProblem;
        invalidateExactCoulombPersistentStepSizeForGroup(constraints);
        invalidateExactCoulombWarmStartPointerCache();
        recordLastFailedExactCoulombAttempt(
            math::detail::ExactCoulombFbfStatus::InvalidInput,
            mLastExactCoulombResidualDetails,
            mLastExactCoulombIterations);
        ++mNumExactCoulombFailures;
        return false;
      }
    }

    const Eigen::Index problemDimension = problem.contactProblem.getDimension();
    bool hasDeferredDenseSeedSnapshot
        = rowOperatorReady
          && mExactCoulombOptions.assembleDenseContactRowSnapshot;
    if (!rowOperatorReady) {
      hasDeferredDenseSeedSnapshot
          = problem.delassus.rows() == problemDimension
            && problem.delassus.cols() == problemDimension
            && problem.delassus.allFinite();
    }
    if (deferDenseSeed && hasDeferredDenseSeedSnapshot) {
      // A contact-row attempt defers the builder's dense seed. Restore that
      // cold-start behavior both when the row route assembles its requested
      // snapshot and when an unsupported row group falls back to the general
      // impulse-test snapshot.
      detail::seedExactCoulombImpulseFromDelassus(problem);
    }

    mLastExactCoulombContactRowOperatorUsed = rowOperatorReady;
  }

  const bool contactRowOperatorActive = mLastExactCoulombContactRowOperatorUsed;
  const auto& coloredBlockGaussSeidelSchedule
      = contactRowOperator.getColoredBlockGaussSeidelSchedule();
  const bool useColoredBlockGaussSeidel
      = coloredBlockGaussSeidelState.enabled && contactRowOperatorActive
        && coloredBlockGaussSeidelSchedule.hasUsableParallelism();
  if (useColoredBlockGaussSeidel) {
    coloredBlockGaussSeidelState.manifoldCount
        = coloredBlockGaussSeidelSchedule.manifolds.size();
    coloredBlockGaussSeidelState.colorCount
        = coloredBlockGaussSeidelSchedule.colors.size();
    for (const auto& color : coloredBlockGaussSeidelSchedule.colors) {
      coloredBlockGaussSeidelState.maxManifoldsPerColor = std::max(
          coloredBlockGaussSeidelState.maxManifoldsPerColor, color.size());
    }
  }
  // Reuse the conservative crossover already used by the release branch's
  // persistent constraint pool. Small exact fixtures and every dense or
  // unsupported route retain their historical serial execution.
  const bool useParallelContactRowDelassus
      = contactRowOperatorActive
        && !mExactCoulombOptions.assembleDenseContactRowSnapshot
        && !useColoredBlockGaussSeidel
        && contactRowOperator.getContactCount()
               >= kMinParallelContactRowContacts
        && getNumSimulationThreads() > 1u;
  mLastExactCoulombMatrixFreeDelassusOperatorUsed
      = mExactCoulombOptions.useMatrixFreeDelassusOperator
        && !contactRowOperatorActive;

  const auto applyDelassus = [this,
                              &problem,
                              &contactRowOperator,
                              contactRowOperatorActive,
                              useParallelContactRowDelassus,
                              &contactRowParallelEvidence,
                              &contactRowCpuRecorder](
                                 const Eigen::Ref<const Eigen::VectorXd>& input,
                                 Eigen::Ref<Eigen::VectorXd> output) {
    if (contactRowOperatorActive) {
      ++contactRowParallelEvidence.products;
      std::size_t participants = 1u;
      if (useParallelContactRowDelassus) {
        auto parallelFor = [this, &contactRowCpuRecorder](
                               std::size_t count, auto& work) {
          const std::uint64_t observationPhase
              = nextExactCoulombCpuObservationPhase();
          contactRowCpuRecorder.beginPhase(
              observationPhase,
              std::min<std::size_t>(count, getNumSimulationThreads()));
          auto observedWork = [&work, &contactRowCpuRecorder, observationPhase](
                                  std::size_t index) {
            contactRowCpuRecorder.observe(observationPhase);
            work(index);
          };
          using Work = std::decay_t<decltype(observedWork)>;
          const std::size_t participants = parallelForConstraintWork(
              count,
              std::addressof(observedWork),
              [](void* context, std::size_t index) {
                (*static_cast<Work*>(context))(index);
              });
          contactRowCpuRecorder.finishPhase(observationPhase, participants);
          return participants;
        };
        participants
            = contactRowOperator.applyParallel(input, output, parallelFor);
      } else {
        contactRowOperator.apply(input, output);
      }

      contactRowParallelEvidence.maxParticipants
          = std::max(contactRowParallelEvidence.maxParticipants, participants);
      if (participants > 1u)
        ++contactRowParallelEvidence.parallelProducts;
      return;
    }

    if (mExactCoulombOptions.useMatrixFreeDelassusOperator) {
      if (!detail::applyExactCoulombConstraintDelassus(
              problem,
              input,
              output,
              mExactCoulombOptions.includeConstraintRegularization)) {
        output.setConstant(std::numeric_limits<double>::quiet_NaN());
      }
      return;
    }

    output.noalias() = problem.delassus * input;
  };

  const auto applyContactRowDelassusSerial
      = [&contactRowOperator, &contactRowParallelEvidence](
            const Eigen::Ref<const Eigen::VectorXd>& input,
            Eigen::Ref<Eigen::VectorXd> output) {
          ++contactRowParallelEvidence.products;
          contactRowParallelEvidence.maxParticipants = std::max(
              contactRowParallelEvidence.maxParticipants, std::size_t{1u});
          contactRowOperator.apply(input, output);
        };

  // The Delassus operator is fixed for one group solve, so extract the
  // per-contact 3x3 diagonal blocks once for every frozen-cone subproblem
  // instead of once per FBF outer iteration.
  const Eigen::Index frozenConeContactCount
      = problem.contactProblem.getContactCount();
  std::vector<Eigen::Matrix3d> frozenConeDiagonalBlocks;
  bool frozenConeDiagonalBlocksValid = false;
  if (contactRowOperatorActive) {
    frozenConeDiagonalBlocks.resize(
        static_cast<std::size_t>(frozenConeContactCount));
    for (Eigen::Index contact = 0; contact < frozenConeContactCount;
         ++contact) {
      frozenConeDiagonalBlocks[static_cast<std::size_t>(contact)]
          = contactRowOperator.diagonalBlock(contact);
    }
    frozenConeDiagonalBlocksValid = true;
  } else if (mLastExactCoulombMatrixFreeDelassusOperatorUsed) {
    frozenConeDiagonalBlocksValid
        = math::detail::computeExactCoulombDelassusDiagonalBlocksNormalFirst(
            problem.contactProblem, applyDelassus, frozenConeDiagonalBlocks);
  } else {
    frozenConeDiagonalBlocks.resize(
        static_cast<std::size_t>(frozenConeContactCount));
    for (Eigen::Index contact = 0; contact < frozenConeContactCount;
         ++contact) {
      frozenConeDiagonalBlocks[static_cast<std::size_t>(contact)]
          = problem.delassus.block<3, 3>(3 * contact, 3 * contact);
    }
    frozenConeDiagonalBlocksValid = true;
  }

  if (useMatrixFreeDelassusSeed && frozenConeDiagonalBlocksValid) {
    mLastExactCoulombMatrixFreeDelassusSeedUsed
        = detail::seedExactCoulombImpulseFromDelassusOperator(
            problem, frozenConeDiagonalBlocks, applyDelassus);
  }

  if (tryApplyExactCoulombWarmStart(constraints, problem)) {
    mLastExactCoulombMatrixFreeDelassusSeedUsed = false;
    mLastExactCoulombWarmStartUsed = true;
    ++mNumExactCoulombWarmStarts;
  }

  // Apply `accumulator += W.middleCols(3 * contact, 3) * delta` for the
  // incremental inner block Gauss-Seidel updates. The dense snapshot route
  // reads the columns directly; the opt-in constraint-row route stays on
  // full sparse-delta products until a scratch-backed operator lands.
  Eigen::VectorXd blockColumnBasis
      = Eigen::VectorXd::Zero(problem.contactProblem.getDimension());
  Eigen::VectorXd blockColumnProduct(problem.contactProblem.getDimension());
  const auto accumulateDelassusBlockColumns
      = [this,
         &problem,
         &applyDelassus,
         &contactRowOperator,
         contactRowOperatorActive,
         &blockColumnBasis,
         &blockColumnProduct](
            Eigen::Index contact,
            const Eigen::Vector3d& delta,
            Eigen::Ref<Eigen::VectorXd> accumulator) {
          if (contactRowOperatorActive) {
            contactRowOperator.accumulateBlockColumns(
                contact, delta, accumulator);
            return;
          }

          if (mExactCoulombOptions.useMatrixFreeDelassusOperator) {
            blockColumnBasis.segment<3>(3 * contact) = delta;
            applyDelassus(blockColumnBasis, blockColumnProduct);
            accumulator += blockColumnProduct;
            blockColumnBasis.segment<3>(3 * contact).setZero();
            return;
          }

          accumulator.noalias()
              += problem.delassus.middleCols<3>(3 * contact) * delta;
        };

  auto frozenOptions = makeFrozenConeOptions(mExactCoulombOptions);
  const bool sourceInnerInitializationEnabled
      = getExactCoulombSourceInnerInitializationState(this);
  frozenOptions.projectInitialReaction = !sourceInnerInitializationEnabled;
  math::detail::ExactCoulombFrozenConeBlockGaussSeidelWorkspace
      frozenConeWorkspace;
  frozenOptions.workspace = &frozenConeWorkspace;
  if (frozenConeDiagonalBlocksValid) {
    frozenOptions.cachedDiagonalBlocks = &frozenConeDiagonalBlocks;
  }
  ExactCoulombColoredBlockGaussSeidelExecutor* activeColoredExecutor = nullptr;
  const auto solveFrozenCone
      = [this,
         &applyDelassus,
         &applyContactRowDelassusSerial,
         &accumulateDelassusBlockColumns,
         &coloredBlockGaussSeidelSchedule,
         &activeColoredExecutor,
         useColoredBlockGaussSeidel,
         &coloredBlockGaussSeidelState,
         &frozenOptions,
         acceptInnerMaxIterations
         = mExactCoulombOptions.acceptInnerMaxIterations](
            const auto& innerProblem,
            const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
            const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
            double stepSizeGamma,
            const Eigen::VectorXd& initialReaction,
            Eigen::Ref<Eigen::VectorXd> output) {
          frozenOptions.initialReaction = &initialReaction;
          math::detail::ExactCoulombFrozenConeResult innerResult;
          if (useColoredBlockGaussSeidel) {
            coloredBlockGaussSeidelState.used = true;
            ++coloredBlockGaussSeidelState.solves;
            const std::size_t requestedWorkers = getNumSimulationThreads();
            auto runWorkers
                = [&activeColoredExecutor](std::size_t count, auto& work) {
                    if (count == 1u) {
                      work(0u);
                      return std::size_t{1u};
                    }
                    if (activeColoredExecutor == nullptr)
                      return std::size_t{0u};
                    return activeColoredExecutor->run(work);
                  };
            std::size_t participants = 0u;
            innerResult = solveExactCoulombFrozenConeColoredBlockGaussSeidel(
                innerProblem,
                referenceReaction,
                frozenCoupling,
                stepSizeGamma,
                applyContactRowDelassusSerial,
                accumulateDelassusBlockColumns,
                coloredBlockGaussSeidelSchedule,
                requestedWorkers,
                runWorkers,
                frozenOptions,
                participants);
            coloredBlockGaussSeidelState.maxParticipants = std::max(
                coloredBlockGaussSeidelState.maxParticipants, participants);
          } else {
            innerResult
                = math::detail::solveExactCoulombFrozenConeBlockGaussSeidel(
                    innerProblem,
                    referenceReaction,
                    frozenCoupling,
                    stepSizeGamma,
                    applyDelassus,
                    accumulateDelassusBlockColumns,
                    frozenOptions);
          }
          output = innerResult.reaction;
          if (innerResult.status
              == math::detail::ExactCoulombFrozenConeStatus::Success) {
            return true;
          }

          return acceptInnerMaxIterations
                 && innerResult.status
                        == math::detail::ExactCoulombFrozenConeStatus::
                            MaxIterations
                 && innerResult.iterations > 0 && output.allFinite();
        };

  auto fbfOptions = makeFbfOptions(mExactCoulombOptions);
  applyExactCoulombCrossStepPolicy(crossStepPolicy, fbfOptions);
  applyExactCoulombSourceContinuationPolicy(
      sourceContinuationPolicy, fbfOptions);
  fbfOptions.projectAfterCorrection
      = getExactCoulombPostCorrectionProjectionState(this);
  fbfOptions.restartInnerFromCurrentOuterReaction
      = sourceInnerInitializationEnabled;
  double persistedStepSize = std::numeric_limits<double>::quiet_NaN();
  std::size_t persistedNoRejectionSolves = 0u;
  const bool persistentStepSizeMatched
      = mExactCoulombOptions.enableWarmStart
        && mExactCoulombOptions.enableStepSizePersistence
        && mExactCoulombOptions.enableAdaptiveStepSize
        && std::isnan(mExactCoulombOptions.initialStepSize)
        && tryGetExactCoulombPersistentStepSize(
            constraints, persistedStepSize, persistedNoRejectionSolves);
  if (persistentStepSizeMatched) {
    double requestedStepSize = persistedStepSize;
    if (persistedNoRejectionSolves > 0u) {
      requestedStepSize *= mExactCoulombOptions.stepSizeRecoveryGrowthFactor;
    }
    if (std::isfinite(requestedStepSize) && requestedStepSize > 0.0) {
      // The math solver recomputes gamma_safe for this group and caps this
      // request there. This layer never guesses or caches the spectral bound.
      fbfOptions.initialStepSize = requestedStepSize;
      mLastExactCoulombPersistentStepSizeUsed = true;
      mLastExactCoulombPersistentStepSizeRequest = requestedStepSize;
    }
  }

  const auto runFbf = [this,
                       &problem,
                       &applyDelassus,
                       &solveFrozenCone,
                       &activeColoredExecutor,
                       &coloredBlockGaussSeidelCpuRecorder,
                       useColoredBlockGaussSeidel,
                       &coloredBlockGaussSeidelState](
                          const Eigen::VectorXd& initialReaction,
                          const math::detail::ExactCoulombFbfOptions& options) {
    if (!useColoredBlockGaussSeidel || getNumSimulationThreads() <= 1u) {
      return math::detail::solveExactCoulombFbf(
          problem.contactProblem,
          initialReaction,
          applyDelassus,
          solveFrozenCone,
          options);
    }

    const std::size_t requestedParticipants = getNumSimulationThreads();
    ExactCoulombColoredBlockGaussSeidelExecutor executor(requestedParticipants);
    const std::uint64_t observationPhase
        = nextExactCoulombCpuObservationPhase();
    coloredBlockGaussSeidelCpuRecorder.beginPhase(
        observationPhase, requestedParticipants);
    math::detail::ExactCoulombFbfResult result;
    auto mainWork = [&] {
      activeColoredExecutor = &executor;
      result = math::detail::solveExactCoulombFbf(
          problem.contactProblem,
          initialReaction,
          applyDelassus,
          solveFrozenCone,
          options);
      activeColoredExecutor = nullptr;
    };
    auto participantWork = [&](std::size_t participant) {
      ExactCoulombScopedParticipantAffinity scopedAffinity(
          coloredBlockGaussSeidelState.participantAffinityEnabled,
          participant,
          requestedParticipants);
      coloredBlockGaussSeidelCpuRecorder.observe(observationPhase);
      if (participant == 0u)
        executor.runMain(mainWork);
      else
        executor.runWorker(participant);
    };
    using Work = std::decay_t<decltype(participantWork)>;
    const std::size_t participants = parallelForConstraintWork(
        requestedParticipants,
        std::addressof(participantWork),
        [](void* context, std::size_t participant) {
          (*static_cast<Work*>(context))(participant);
        });
    coloredBlockGaussSeidelCpuRecorder.finishPhase(
        observationPhase, participants);
    DART_ASSERT(participants == requestedParticipants);
    ++coloredBlockGaussSeidelState.dispatches;
    coloredBlockGaussSeidelState.maxParticipants
        = std::max(coloredBlockGaussSeidelState.maxParticipants, participants);
    return result;
  };

  auto solution = runFbf(problem.initialGuess, fbfOptions);
  bool hadAnyStepSizeRejection = solution.shrinkIterations > 0;
  const auto accountFbfAttempt
      = [this](const math::detail::ExactCoulombFbfResult& attempt) {
          if (attempt.iterations <= 0)
            return;
          mMaxExactCoulombIterations
              = std::max(mMaxExactCoulombIterations, attempt.iterations);
          mTotalExactCoulombIterations
              += static_cast<std::size_t>(attempt.iterations);
        };
  accountFbfAttempt(solution);

  const auto isAcceptableUnconvergedResult =
      [this, &problem, &sourceContinuationPolicy](
          const math::detail::ExactCoulombFbfResult& candidate) {
        const bool sourceStatusAccepted
            = sourceContinuationPolicy.enabled
              && (candidate.status
                      == math::detail::ExactCoulombFbfStatus::Plateau
                  || candidate.status
                         == math::detail::ExactCoulombFbfStatus::MaxIterations);
        const bool legacyMaxIterationsAccepted
            = !sourceContinuationPolicy.enabled
              && mExactCoulombOptions.acceptOuterMaxIterations
              && candidate.status
                     == math::detail::ExactCoulombFbfStatus::MaxIterations;
        return (sourceStatusAccepted || legacyMaxIterationsAccepted)
               && candidate.reaction.size()
                      == problem.contactProblem.getDimension()
               && candidate.reaction.allFinite()
               && math::detail::isFiniteCoulombConeResidual(candidate.residual)
               && (!sourceContinuationPolicy.enabled
                   || std::isfinite(candidate.naturalMapResidual));
      };
  const auto candidateRank =
      [&isAcceptableUnconvergedResult](
          const math::detail::ExactCoulombFbfResult& candidate) {
        if (candidate.status == math::detail::ExactCoulombFbfStatus::Success) {
          return 3;
        }
        if (isAcceptableUnconvergedResult(candidate))
          return 2;
        if ((candidate.status
                 == math::detail::ExactCoulombFbfStatus::MaxIterations
             || candidate.status
                    == math::detail::ExactCoulombFbfStatus::Plateau)
            && candidate.reaction.allFinite()
            && math::detail::isFiniteCoulombConeResidual(candidate.residual)) {
          return 1;
        }
        return 0;
      };
  const auto candidateIsBetter =
      [&candidateRank](
          const math::detail::ExactCoulombFbfResult& candidate,
          const math::detail::ExactCoulombFbfResult& current) {
        const int candidateStatusRank = candidateRank(candidate);
        const int currentStatusRank = candidateRank(current);
        const bool residualBetter
            = std::isfinite(candidate.residual.value)
              && (!std::isfinite(current.residual.value)
                  || candidate.residual.value <= current.residual.value);
        return candidateStatusRank > currentStatusRank
               || (candidateStatusRank == currentStatusRank && residualBetter);
      };

  bool retriedWithoutPersistentStepSize = false;
  if (solution.status != math::detail::ExactCoulombFbfStatus::Success
      && !isAcceptableUnconvergedResult(solution)
      && !sourceContinuationPolicy.enabled
      && mLastExactCoulombPersistentStepSizeUsed) {
    // A persisted gamma is only an optimization hint. In particular, an
    // automatically scaled gamma from the prior step is capped at the fresh
    // unscaled safe bound when reused. If that conservative hint fails, retry
    // once with this step's normal automatic policy before changing contact
    // laws through the boxed-LCP fallback.
    retriedWithoutPersistentStepSize = true;
    ++mNumExactCoulombPersistentStepSizeRetries;
    const bool hasBestReaction
        = solution.bestReaction.size() == problem.initialGuess.size()
          && solution.bestReaction.allFinite()
          && std::isfinite(solution.bestResidual.value)
          && (!std::isfinite(solution.residual.value)
              || solution.bestResidual.value <= solution.residual.value);
    const Eigen::VectorXd& retryInitialReaction
        = hasBestReaction
              ? solution.bestReaction
              : (solution.reaction.size() == problem.initialGuess.size()
                         && solution.reaction.allFinite()
                     ? solution.reaction
                     : problem.initialGuess);

    auto automaticFbfOptions = makeFbfOptions(mExactCoulombOptions);
    applyExactCoulombCrossStepPolicy(crossStepPolicy, automaticFbfOptions);
    applyExactCoulombSourceContinuationPolicy(
        sourceContinuationPolicy, automaticFbfOptions);
    automaticFbfOptions.projectAfterCorrection
        = getExactCoulombPostCorrectionProjectionState(this);
    automaticFbfOptions.restartInnerFromCurrentOuterReaction
        = sourceInnerInitializationEnabled;
    automaticFbfOptions.useAutomaticResidualScales = false;
    automaticFbfOptions.residualScales = solution.residualScales;
    const auto automaticSolution
        = runFbf(retryInitialReaction, automaticFbfOptions);
    hadAnyStepSizeRejection
        = hadAnyStepSizeRejection || automaticSolution.shrinkIterations > 0;
    accountFbfAttempt(automaticSolution);
    if (candidateIsBetter(automaticSolution, solution)) {
      solution = automaticSolution;
    }

    // Any later projected-gradient retry should also avoid the gamma hint
    // that this solve just proved unproductive.
    fbfOptions.initialStepSize = std::numeric_limits<double>::quiet_NaN();
  }

  if (solution.status != math::detail::ExactCoulombFbfStatus::Success
      && !isAcceptableUnconvergedResult(solution)
      && !sourceContinuationPolicy.enabled
      && mExactCoulombOptions.enableProjectedGradientRetry) {
    mLastExactCoulombProjectedGradientRetryUsed = true;
    ++mNumExactCoulombProjectedGradientRetries;
    const auto projectedOptions
        = makeProjectedGradientOptions(mExactCoulombOptions);
    // The Delassus spectral estimate is deterministic and fixed for one group
    // solve, so compute it once for the retry instead of once per projected
    // frozen-cone subproblem.
    double projectedSpectralRadius = std::numeric_limits<double>::quiet_NaN();
    if (std::isnan(projectedOptions.stepSize)) {
      projectedSpectralRadius
          = math::detail::estimateLargestExactCoulombDelassusEigenvalue(
              problem.contactProblem,
              applyDelassus,
              projectedOptions.spectralIterations);
    }
    const auto solveProjectedFrozenCone
        = [&applyDelassus, &projectedOptions, projectedSpectralRadius](
              const auto& innerProblem,
              const Eigen::Ref<const Eigen::VectorXd>& referenceReaction,
              const Eigen::Ref<const Eigen::VectorXd>& frozenCoupling,
              double stepSizeGamma,
              Eigen::Ref<Eigen::VectorXd> output) {
            auto localOptions = projectedOptions;
            if (std::isnan(localOptions.stepSize)
                && std::isfinite(projectedSpectralRadius)
                && std::isfinite(stepSizeGamma) && stepSizeGamma > 0.0) {
              const double lipschitz
                  = projectedSpectralRadius + 1.0 / stepSizeGamma;
              if (std::isfinite(lipschitz) && lipschitz > 0.0) {
                localOptions.stepSize = 1.0 / lipschitz;
              }
            }

            const auto innerResult
                = math::detail::solveExactCoulombFrozenConeProjectedGradient(
                    innerProblem,
                    referenceReaction,
                    frozenCoupling,
                    stepSizeGamma,
                    applyDelassus,
                    localOptions);
            output = innerResult.reaction;
            return innerResult.status
                   == math::detail::ExactCoulombFrozenConeStatus::Success;
          };

    const bool hasBestReaction
        = solution.bestReaction.size() == problem.initialGuess.size()
          && solution.bestReaction.allFinite()
          && std::isfinite(solution.bestResidual.value)
          && (!std::isfinite(solution.residual.value)
              || solution.bestResidual.value <= solution.residual.value);
    const Eigen::VectorXd& retryInitialReaction
        = hasBestReaction
              ? solution.bestReaction
              : (solution.reaction.size() == problem.initialGuess.size()
                         && solution.reaction.allFinite()
                     ? solution.reaction
                     : problem.initialGuess);

    auto retryFbfOptions = fbfOptions;
    retryFbfOptions.useAutomaticResidualScales = false;
    retryFbfOptions.residualScales = solution.residualScales;
    const auto projectedSolution = math::detail::solveExactCoulombFbf(
        problem.contactProblem,
        retryInitialReaction,
        applyDelassus,
        solveProjectedFrozenCone,
        retryFbfOptions);
    hadAnyStepSizeRejection
        = hadAnyStepSizeRejection || projectedSolution.shrinkIterations > 0;
    accountFbfAttempt(projectedSolution);
    if (candidateIsBetter(projectedSolution, solution)) {
      solution = projectedSolution;
    }
  }

  const bool unconvergedResultAccepted
      = isAcceptableUnconvergedResult(solution);
  const bool maxIterationsAccepted
      = unconvergedResultAccepted
        && solution.status
               == math::detail::ExactCoulombFbfStatus::MaxIterations;
  const bool plateauAccepted
      = unconvergedResultAccepted
        && solution.status == math::detail::ExactCoulombFbfStatus::Plateau;
  if (solution.status != math::detail::ExactCoulombFbfStatus::Success
      && !unconvergedResultAccepted && !sourceContinuationPolicy.enabled) {
    bool densePolishAvailable = true;
    const Eigen::Index dimension = problem.contactProblem.getDimension();
    if (mExactCoulombOptions.enableDenseResidualPolish
        && contactRowOperatorActive
        && (problem.delassus.rows() != dimension
            || problem.delassus.cols() != dimension)) {
      contactRowOperator.assembleDense(problem.delassus);
      densePolishAvailable = problem.delassus.rows() == dimension
                             && problem.delassus.cols() == dimension
                             && problem.delassus.allFinite();
      mLastExactCoulombDenseContactRowSnapshotAssembled = densePolishAvailable;
    }
    if (densePolishAvailable)
      tryPolishFailedExactCoulombSolution(problem, solution);
  }

  if ((fbfOptions.useNaturalMapResidualForInitialStepSizeCap
       || sourceContinuationPolicy.enabled)
      && solution.reaction.size() == problem.contactProblem.getDimension()
      && solution.reaction.allFinite()) {
    solution.naturalMapResidual
        = math::detail::computeExactCoulombNaturalMapResidualNormalFirst(
            problem.contactProblem, solution.reaction, applyDelassus);
  }

  contactRowParallelEvidence.logicalCpuIds
      = contactRowCpuRecorder.getLogicalCpuIds();
  contactRowParallelEvidence.maxPhaseLogicalCpuIds
      = contactRowCpuRecorder.getMaxPhaseLogicalCpuIds();
  coloredBlockGaussSeidelState.logicalCpuIds
      = coloredBlockGaussSeidelCpuRecorder.getLogicalCpuIds();
  coloredBlockGaussSeidelState.maxPhaseLogicalCpuIds
      = coloredBlockGaussSeidelCpuRecorder.getMaxPhaseLogicalCpuIds();
  recordContactRowParallelEvidence(this, contactRowParallelEvidence);
  recordExactCoulombColoredBlockGaussSeidelState(
      this, coloredBlockGaussSeidelState);

  mLastExactCoulombFbfStatus = solution.status;
  mLastExactCoulombResidual = solution.residual.value;
  mLastExactCoulombResidualDetails = solution.residual;
  mLastExactCoulombBestResidual = solution.bestResidual.value;
  mLastExactCoulombBestIteration = solution.bestIteration;
  mLastExactCoulombResidualHistory = solution.residualHistory;
  mLastExactCoulombIterations = solution.iterations;
  mLastExactCoulombStepSize = solution.stepSize;
  mLastExactCoulombSafeStepSize = solution.safeStepSize;
  mLastExactCoulombCouplingVariationRatio = solution.couplingVariationRatio;
  mLastExactCoulombShrinkIterations = solution.shrinkIterations;
  if (std::isfinite(solution.residual.value)
      && (!std::isfinite(mWorstExactCoulombResidual)
          || solution.residual.value > mWorstExactCoulombResidual)) {
    mWorstExactCoulombResidual = solution.residual.value;
  }
  if (solution.status != math::detail::ExactCoulombFbfStatus::Success
      && !unconvergedResultAccepted) {
    mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::FbfFailed;
    recordExactCoulombPolicyAttempt(
        this, solution, false, sourceContinuationPolicy.enabled, false);
    recordExactCoulombResidualHistory(
        static_cast<std::size_t>(problem.contactProblem.getContactCount()),
        mLastExactCoulombStatus,
        solution,
        sourceContinuationPolicy.enabled);
    invalidateExactCoulombPersistentStepSizeForGroup(constraints);
    invalidateExactCoulombWarmStartPointerCache();
    recordLastFailedExactCoulombAttempt(
        solution.status,
        solution.residual,
        solution.iterations,
        solution.stepSize,
        solution.safeStepSize,
        solution.couplingVariationRatio,
        solution.shrinkIterations,
        solution.bestResidual.value,
        solution.bestIteration);
    recordExactCoulombLastFailureState(
        this,
        mLastExactCoulombStatus,
        mLastExactCoulombBuildStatus,
        static_cast<std::size_t>(problem.contactProblem.getContactCount()));
    ++mNumExactCoulombFailures;
    return false;
  }

  if (!detail::applyExactCoulombConstraintImpulses(
          problem, solution.reaction)) {
    mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::FbfFailed;
    recordExactCoulombPolicyAttempt(
        this, solution, false, sourceContinuationPolicy.enabled, false);
    recordExactCoulombResidualHistory(
        static_cast<std::size_t>(problem.contactProblem.getContactCount()),
        mLastExactCoulombStatus,
        solution,
        sourceContinuationPolicy.enabled);
    invalidateExactCoulombPersistentStepSizeForGroup(constraints);
    invalidateExactCoulombWarmStartPointerCache();
    recordLastFailedExactCoulombAttempt(
        solution.status,
        solution.residual,
        solution.iterations,
        solution.stepSize,
        solution.safeStepSize,
        solution.couplingVariationRatio,
        solution.shrinkIterations,
        solution.bestResidual.value,
        solution.bestIteration);
    recordExactCoulombLastFailureState(
        this,
        mLastExactCoulombStatus,
        mLastExactCoulombBuildStatus,
        static_cast<std::size_t>(problem.contactProblem.getContactCount()));
    ++mNumExactCoulombFailures;
    return false;
  }

  double stepSizeToPersist = std::numeric_limits<double>::quiet_NaN();
  std::size_t consecutiveNoRejectionSolves = 0u;
  const bool persistentStepSizeEnabled
      = mExactCoulombOptions.enableWarmStart
        && mExactCoulombOptions.enableStepSizePersistence
        && mExactCoulombOptions.enableAdaptiveStepSize
        && std::isnan(mExactCoulombOptions.initialStepSize)
        && !retriedWithoutPersistentStepSize;
  if (persistentStepSizeEnabled && std::isfinite(solution.stepSize)
      && solution.stepSize > 0.0) {
    const bool persistUncappedStepSize
        = crossStepPolicy.persistUncappedStepSizeOnWarmStartCap
          && solution.initialStepSizeCapApplied
          && std::isfinite(solution.uncappedInitialStepSize)
          && solution.uncappedInitialStepSize > 0.0;
    stepSizeToPersist = persistUncappedStepSize
                            ? solution.uncappedInitialStepSize
                            : solution.stepSize;
    if (!hadAnyStepSizeRejection) {
      const std::size_t priorNoRejectionSolves
          = persistentStepSizeMatched ? persistedNoRejectionSolves : 0u;
      if (priorNoRejectionSolves < std::numeric_limits<std::size_t>::max()) {
        consecutiveNoRejectionSolves = priorNoRejectionSolves + 1u;
      } else {
        consecutiveNoRejectionSolves = priorNoRejectionSolves;
      }
    }
  }
  const bool unconvergedCacheSkipped
      = unconvergedResultAccepted
        && crossStepPolicy.requireResidualImprovementForUnconvergedCacheSave
        && !(
            std::isfinite(solution.initialNaturalMapResidual)
            && std::isfinite(solution.naturalMapResidual)
            && solution.naturalMapResidual
                   < solution.initialNaturalMapResidual);
  updateExactCoulombWarmStart(
      constraints,
      solution.reaction,
      stepSizeToPersist,
      consecutiveNoRejectionSolves,
      !unconvergedCacheSkipped);
  ++mNumExactCoulombSolves;
  if (maxIterationsAccepted)
    ++mNumExactCoulombMaxIterationsAccepted;
  if (plateauAccepted) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::PlateauAccepted;
  } else if (maxIterationsAccepted) {
    mLastExactCoulombStatus
        = ExactCoulombFbfConstraintSolverStatus::MaxIterationsAccepted;
  } else {
    mLastExactCoulombStatus = ExactCoulombFbfConstraintSolverStatus::Success;
  }
  recordExactCoulombPolicyAttempt(
      this,
      solution,
      unconvergedCacheSkipped,
      sourceContinuationPolicy.enabled,
      plateauAccepted);
  recordExactCoulombResidualHistory(
      static_cast<std::size_t>(problem.contactProblem.getContactCount()),
      mLastExactCoulombStatus,
      solution,
      sourceContinuationPolicy.enabled);
  return true;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::tryPolishFailedExactCoulombSolution(
    const detail::ExactCoulombConstraintProblem& problem,
    math::detail::ExactCoulombFbfResult& solution)
{
  if (!mExactCoulombOptions.enableDenseResidualPolish
      || mExactCoulombOptions.denseResidualPolishIterations <= 0
      || solution.residual.value <= mExactCoulombOptions.tolerance) {
    return;
  }

  const Eigen::Index dimension = problem.contactProblem.getDimension();
  Eigen::VectorXd reaction;
  if (solution.bestReaction.size() == dimension
      && solution.bestReaction.allFinite()) {
    reaction = solution.bestReaction;
  } else {
    reaction = solution.reaction;
  }
  if (reaction.size() != dimension || !reaction.allFinite()) {
    return;
  }

  if (!math::detail::projectExactCoulombReactionNormalFirst(
          reaction, problem.contactProblem.coefficients, reaction)) {
    return;
  }

  Eigen::MatrixXd symmetricDelassus
      = 0.5 * (problem.delassus + problem.delassus.transpose());
  if (!symmetricDelassus.allFinite()) {
    return;
  }

  const double maxDiagonal = symmetricDelassus.diagonal().cwiseAbs().maxCoeff();
  const double regularization
      = mExactCoulombOptions.denseResidualPolishRegularization
        * (std::max)(1.0, maxDiagonal);
  symmetricDelassus.diagonal().array() += regularization;

  Eigen::LDLT<Eigen::MatrixXd> factorization(symmetricDelassus);
  if (factorization.info() != Eigen::Success) {
    return;
  }

  auto bestResidual = computeExactCoulombDenseResidual(
      problem, reaction, solution.residualScales);
  if (!std::isfinite(bestResidual.value)
      || (std::isfinite(solution.bestResidual.value)
          && solution.bestResidual.value < bestResidual.value)) {
    bestResidual = solution.bestResidual;
  }

  bool attempted = false;
  Eigen::VectorXd dualCorrection(dimension);
  Eigen::VectorXd direction(dimension);
  Eigen::VectorXd candidate(dimension);
  Eigen::VectorXd localDirection = Eigen::VectorXd::Zero(dimension);
  for (int iteration = 0;
       iteration < mExactCoulombOptions.denseResidualPolishIterations;
       ++iteration) {
    if (!computeExactCoulombDenseDualCorrection(
            problem, reaction, dualCorrection)) {
      break;
    }

    if (dualCorrection.norm() == 0.0) {
      break;
    }

    direction = factorization.solve(dualCorrection);
    if (!direction.allFinite()) {
      break;
    }

    attempted = true;
    bool accepted = false;
    double step = 1.0;
    for (int lineSearch = 0;
         lineSearch
         <= mExactCoulombOptions.denseResidualPolishLineSearchIterations;
         ++lineSearch) {
      candidate = reaction + step * direction;
      if (!math::detail::projectExactCoulombReactionNormalFirst(
              candidate, problem.contactProblem.coefficients, candidate)) {
        break;
      }

      const auto candidateResidual = computeExactCoulombDenseResidual(
          problem, candidate, solution.residualScales);
      if (std::isfinite(candidateResidual.value)
          && candidateResidual.value < bestResidual.value) {
        reaction = candidate;
        bestResidual = candidateResidual;
        accepted = true;
        break;
      }

      step *= 0.5;
    }

    if (!accepted || bestResidual.value <= mExactCoulombOptions.tolerance) {
      break;
    }
  }

  for (int iteration = 0;
       iteration < mExactCoulombOptions.denseResidualPolishIterations
       && bestResidual.value > mExactCoulombOptions.tolerance;
       ++iteration) {
    const Eigen::Index contact = bestResidual.worstDualContact;
    if (contact < 0 || contact >= problem.contactProblem.getContactCount()) {
      break;
    }

    if (!computeExactCoulombDenseDualCorrection(
            problem, reaction, dualCorrection)) {
      break;
    }

    const Eigen::Index offset = 3 * contact;
    const Eigen::Vector3d localCorrection = dualCorrection.segment<3>(offset);
    if (localCorrection.norm() == 0.0) {
      break;
    }

    const Eigen::Matrix3d localBlock
        = symmetricDelassus.block<3, 3>(offset, offset);
    Eigen::LDLT<Eigen::Matrix3d> localFactorization(localBlock);
    if (localFactorization.info() != Eigen::Success) {
      break;
    }

    const Eigen::Vector3d localStep = localFactorization.solve(localCorrection);
    if (!localStep.allFinite()) {
      break;
    }

    localDirection.setZero();
    localDirection.segment<3>(offset) = localStep;
    attempted = true;

    bool accepted = false;
    double step = 1.0;
    for (int lineSearch = 0;
         lineSearch
         <= mExactCoulombOptions.denseResidualPolishLineSearchIterations;
         ++lineSearch) {
      candidate = reaction + step * localDirection;
      if (!math::detail::projectExactCoulombReactionNormalFirst(
              candidate, problem.contactProblem.coefficients, candidate)) {
        break;
      }

      const auto candidateResidual = computeExactCoulombDenseResidual(
          problem, candidate, solution.residualScales);
      if (std::isfinite(candidateResidual.value)
          && candidateResidual.value < bestResidual.value) {
        reaction = candidate;
        bestResidual = candidateResidual;
        accepted = true;
        break;
      }

      step *= 0.5;
    }

    if (!accepted) {
      break;
    }
  }

  if (!attempted) {
    return;
  }

  mLastExactCoulombDenseResidualPolishUsed = true;
  ++mNumExactCoulombDenseResidualPolishes;

  if (std::isfinite(bestResidual.value)
      && bestResidual.value < solution.bestResidual.value) {
    solution.bestResidual = bestResidual;
    solution.bestReaction = reaction;
    solution.bestIteration = solution.iterations;
  }

  if (std::isfinite(bestResidual.value)
      && bestResidual.value < solution.residual.value) {
    solution.reaction = reaction;
    solution.residual = bestResidual;
  }

  if (solution.residual.value <= mExactCoulombOptions.tolerance) {
    solution.status = math::detail::ExactCoulombFbfStatus::Success;
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::recordLastFailedExactCoulombAttempt(
    math::detail::ExactCoulombFbfStatus status,
    const math::detail::CoulombConeResidual& residual,
    int iterations,
    double stepSize,
    double safeStepSize,
    double couplingVariationRatio,
    int shrinkIterations,
    double bestResidual,
    int bestIteration)
{
  mLastFailedExactCoulombFbfStatus = status;
  mLastFailedExactCoulombResidual = residual.value;
  mLastFailedExactCoulombResidualDetails = residual;
  mLastFailedExactCoulombBestResidual = bestResidual;
  mLastFailedExactCoulombBestIteration = bestIteration;
  mLastFailedExactCoulombIterations = iterations;
  mLastFailedExactCoulombStepSize = stepSize;
  mLastFailedExactCoulombSafeStepSize = safeStepSize;
  mLastFailedExactCoulombCouplingVariationRatio = couplingVariationRatio;
  mLastFailedExactCoulombShrinkIterations = shrinkIterations;
  recordExactCoulombLastFailureState(
      this, mLastExactCoulombStatus, mLastExactCoulombBuildStatus, 0u);
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::tryApplyExactCoulombWarmStart(
    const std::vector<ConstraintBase*>& constraints,
    detail::ExactCoulombConstraintProblem& problem)
{
  if (!mExactCoulombOptions.enableWarmStart) {
    return false;
  }

  // ContactConstraint instances are pooled and reset between World steps, so
  // an identical pointer sequence says nothing about contact identity. If the
  // group exposes any physical contact frame, always use the manifold path
  // below. The pointer cache remains only for adapter-compatible synthetic
  // constraints that cannot expose a world-space contact identity at all.
  const bool hasPhysicalContactFrame = std::any_of(
      constraints.begin(),
      constraints.end(),
      [](const ConstraintBase* constraint) {
        detail::ExactCoulombContactRowOperator::ContactFrame frame;
        return detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraint, frame);
      });

  // Compatibility path for non-ContactConstraint adapters only.
  const bool pointerSequenceMatches
      = !hasPhysicalContactFrame && mHasExactCoulombWarmStart
        && mWarmStartConstraints.size() == constraints.size()
        && mWarmStartReaction.size() == problem.initialGuess.size()
        && mWarmStartReaction.allFinite();
  if (pointerSequenceMatches) {
    bool identical = true;
    for (std::size_t i = 0u; i < constraints.size(); ++i) {
      if (mWarmStartConstraints[i] != constraints[i]) {
        identical = false;
        break;
      }
    }

    if (identical) {
      Eigen::VectorXd projected(problem.initialGuess.size());
      if (math::detail::projectExactCoulombReactionNormalFirst(
              mWarmStartReaction,
              problem.contactProblem.coefficients,
              projected)) {
        problem.initialGuess = projected;
        mLastExactCoulombWarmStartMatchedContacts = constraints.size();
        return true;
      }
    }
  }

  // Manifold path: contact constraints are recreated every `World` step, so
  // match cached contacts by body pair and stable body-local feature, then
  // re-project the cached reaction onto the current frames.
  const auto policy = getExactCoulombPolicyState(this).crossStepOptions;
  const double matchDistance = mExactCoulombOptions.warmStartMatchDistance;
  if (mWarmStartContactRecords.empty() || !std::isfinite(matchDistance)
      || matchDistance <= 0.0) {
    return false;
  }

  std::vector<bool> recordUsed(mWarmStartContactRecords.size(), false);
  std::size_t matchedContacts = 0u;
  for (std::size_t i = 0u; i < constraints.size(); ++i) {
    detail::ExactCoulombContactRowOperator::ContactFrame frame;
    if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraints[i], frame)) {
      continue;
    }
    const auto currentFeatures = makeBodyLocalContactFeatures(frame);

    int bestRecord = -1;
    double bestDistance = std::numeric_limits<double>::infinity();
    bool bestFlipped = false;
    for (std::size_t r = 0u; r < mWarmStartContactRecords.size(); ++r) {
      if (recordUsed[r]) {
        continue;
      }

      const auto& candidate = mWarmStartContactRecords[r];
      if (!candidate.reactionValid
          || (policy.warmStartMaxAge >= 0
              && candidate.age > policy.warmStartMaxAge)) {
        continue;
      }
      const auto identity = matchExactCoulombContactIdentity(
          candidate, frame, currentFeatures, matchDistance, policy);
      if (!identity.matched || identity.distance >= bestDistance) {
        continue;
      }

      bestDistance = identity.distance;
      bestRecord = static_cast<int>(r);
      bestFlipped = identity.flipped;
    }

    if (bestRecord < 0) {
      continue;
    }

    recordUsed[static_cast<std::size_t>(bestRecord)] = true;
    const auto& record
        = mWarmStartContactRecords[static_cast<std::size_t>(bestRecord)];
    Eigen::Vector3d worldImpulse;
    if (policy.warmStartMatchMode
            == ExactCoulombFbfWarmStartMatchMode::OrderedBodyBLocalFeature
        && record.hasLocalImpulseB && frame.bodyB != nullptr) {
      worldImpulse
          = frame.bodyB->getWorldTransform().linear() * record.localImpulseB;
    } else {
      worldImpulse = bestFlipped ? Eigen::Vector3d(-record.worldImpulse)
                                 : record.worldImpulse;
    }
    const Eigen::Index offset = static_cast<Eigen::Index>(3u * i);
    Eigen::Vector3d localReaction(
        worldImpulse.dot(frame.normal),
        worldImpulse.dot(frame.tangent1),
        worldImpulse.dot(frame.tangent2));
    if (!localReaction.allFinite()) {
      continue;
    }

    problem.initialGuess.segment<3>(offset)
        = math::detail::projectCoulombConeNormalFirst(
            localReaction,
            problem.contactProblem.coefficients[static_cast<Eigen::Index>(i)]);
    ++matchedContacts;
  }

  if (matchedContacts == 0u) {
    return false;
  }

  mLastExactCoulombWarmStartMatchedContacts = matchedContacts;
  return true;
}

//==============================================================================
bool ExactCoulombFbfConstraintSolver::tryGetExactCoulombPersistentStepSize(
    const std::vector<ConstraintBase*>& constraints,
    double& stepSize,
    std::size_t& consecutiveNoRejectionSolves) const
{
  stepSize = std::numeric_limits<double>::quiet_NaN();
  consecutiveNoRejectionSolves = 0u;

  // ContactConstraint instances are pooled and reset between World steps.
  // Never let their pointer identity bypass the complete manifold match
  // below. Retain pointer-based persistence only for adapter-compatible
  // synthetic constraints that cannot expose a physical contact frame.
  const bool hasPhysicalContactFrame = std::any_of(
      constraints.begin(),
      constraints.end(),
      [](const ConstraintBase* constraint) {
        detail::ExactCoulombContactRowOperator::ContactFrame frame;
        return detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraint, frame);
      });
  if (!hasPhysicalContactFrame && mHasExactCoulombWarmStart
      && mWarmStartConstraints.size() == constraints.size()
      && std::isfinite(mWarmStartStepSize) && mWarmStartStepSize > 0.0) {
    bool identical = true;
    for (std::size_t i = 0u; i < constraints.size(); ++i) {
      if (mWarmStartConstraints[i] != constraints[i]) {
        identical = false;
        break;
      }
    }
    if (identical) {
      stepSize = mWarmStartStepSize;
      consecutiveNoRejectionSolves = mWarmStartConsecutiveNoRejectionSolves;
      return true;
    }
  }

  // Cross-step constraints are recreated. Match the complete contact group by
  // its body-pair/body-local-feature manifold, and require every record to
  // carry the same group token. Exact group matching avoids leaking one
  // island's gamma into another island that happens to share a nearby point.
  const auto policy = getExactCoulombPolicyState(this).crossStepOptions;
  const double matchDistance = mExactCoulombOptions.warmStartMatchDistance;
  if (constraints.empty() || mWarmStartContactRecords.empty()
      || !std::isfinite(matchDistance) || matchDistance <= 0.0) {
    return false;
  }

  std::vector<bool> recordUsed(mWarmStartContactRecords.size(), false);
  std::size_t matchedGroupId = 0u;
  for (const ConstraintBase* constraint : constraints) {
    detail::ExactCoulombContactRowOperator::ContactFrame frame;
    if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraint, frame)) {
      return false;
    }
    const auto currentFeatures = makeBodyLocalContactFeatures(frame);

    int bestRecord = -1;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t r = 0u; r < mWarmStartContactRecords.size(); ++r) {
      if (recordUsed[r]) {
        continue;
      }

      const auto& record = mWarmStartContactRecords[r];
      if (record.stepSizeGroupId == 0u || !std::isfinite(record.stepSize)
          || record.stepSize <= 0.0) {
        continue;
      }

      const auto identity = matchExactCoulombContactIdentity(
          record, frame, currentFeatures, matchDistance, policy);
      if (!identity.matched || identity.distance >= bestDistance)
        continue;

      bestDistance = identity.distance;
      bestRecord = static_cast<int>(r);
    }

    if (bestRecord < 0) {
      return false;
    }

    const std::size_t recordIndex = static_cast<std::size_t>(bestRecord);
    recordUsed[recordIndex] = true;
    const auto& record = mWarmStartContactRecords[recordIndex];
    if (matchedGroupId == 0u) {
      matchedGroupId = record.stepSizeGroupId;
      stepSize = record.stepSize;
      consecutiveNoRejectionSolves = record.consecutiveNoRejectionSolves;
    } else if (
        record.stepSizeGroupId != matchedGroupId || record.stepSize != stepSize
        || record.consecutiveNoRejectionSolves
               != consecutiveNoRejectionSolves) {
      return false;
    }
  }

  const auto storedGroupSize = static_cast<std::size_t>(std::count_if(
      mWarmStartContactRecords.begin(),
      mWarmStartContactRecords.end(),
      [matchedGroupId](const ExactCoulombWarmStartContactRecord& record) {
        return record.stepSizeGroupId == matchedGroupId;
      }));
  return matchedGroupId != 0u && storedGroupSize == constraints.size();
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::updateExactCoulombWarmStart(
    const std::vector<ConstraintBase*>& constraints,
    const Eigen::Ref<const Eigen::VectorXd>& reaction,
    double stepSize,
    std::size_t consecutiveNoRejectionSolves,
    bool updateReactionCache)
{
  if (!mExactCoulombOptions.enableWarmStart || !reaction.allFinite()) {
    clearExactCoulombWarmStart();
    return;
  }

  const auto policy = getExactCoulombPolicyState(this).crossStepOptions;
  if (updateReactionCache) {
    mWarmStartConstraints = constraints;
    mWarmStartReaction = reaction;
    mHasExactCoulombWarmStart = true;
  }
  mWarmStartStepSize = stepSize;
  mWarmStartConsecutiveNoRejectionSolves = consecutiveNoRejectionSolves;

  if (!updateReactionCache) {
    const double matchDistance = mExactCoulombOptions.warmStartMatchDistance;
    if (constraints.empty() || mWarmStartContactRecords.empty()
        || !std::isfinite(matchDistance) || matchDistance <= 0.0) {
      return;
    }

    std::vector<bool> recordUsed(mWarmStartContactRecords.size(), false);
    std::vector<std::size_t> matchedRecords;
    matchedRecords.reserve(constraints.size());
    for (const ConstraintBase* constraint : constraints) {
      detail::ExactCoulombContactRowOperator::ContactFrame frame;
      if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
              constraint, frame)) {
        return;
      }
      const auto currentFeatures = makeBodyLocalContactFeatures(frame);

      int bestRecord = -1;
      double bestDistance = std::numeric_limits<double>::infinity();
      for (std::size_t r = 0u; r < mWarmStartContactRecords.size(); ++r) {
        if (recordUsed[r])
          continue;

        const auto identity = matchExactCoulombContactIdentity(
            mWarmStartContactRecords[r],
            frame,
            currentFeatures,
            matchDistance,
            policy);
        if (!identity.matched || identity.distance >= bestDistance)
          continue;

        bestDistance = identity.distance;
        bestRecord = static_cast<int>(r);
      }
      if (bestRecord < 0)
        return;

      const std::size_t recordIndex = static_cast<std::size_t>(bestRecord);
      recordUsed[recordIndex] = true;
      matchedRecords.push_back(recordIndex);
    }

    const std::size_t stepSizeGroupId = mNextExactCoulombStepSizeGroupId++;
    if (mNextExactCoulombStepSizeGroupId == 0u)
      mNextExactCoulombStepSizeGroupId = 1u;
    for (const std::size_t recordIndex : matchedRecords) {
      auto& record = mWarmStartContactRecords[recordIndex];
      record.stepSizeGroupId = stepSizeGroupId;
      record.stepSize = stepSize;
      record.consecutiveNoRejectionSolves = consecutiveNoRejectionSolves;
      if (record.age < std::numeric_limits<int>::max())
        ++record.age;
      if (policy.warmStartMaxAge >= 0 && record.age > policy.warmStartMaxAge) {
        record.reactionValid = false;
      }
    }
    return;
  }

  // Store body-local contact features and world-space reactions for
  // cross-step manifold matching. One `World` step can solve several groups
  // (contact islands) through this solver, so the record store is keyed by
  // body pair: a group's update replaces only records for its own pairs and
  // leaves other islands' records intact. Skip the manifold update when any
  // frame is unavailable so adapter-compatible synthetic constraints keep
  // their pointer-based cache without creating incomplete manifold records.
  std::vector<ExactCoulombWarmStartContactRecord> groupRecords;
  groupRecords.reserve(constraints.size());
  const std::size_t stepSizeGroupId = mNextExactCoulombStepSizeGroupId++;
  if (mNextExactCoulombStepSizeGroupId == 0u) {
    mNextExactCoulombStepSizeGroupId = 1u;
  }
  for (std::size_t i = 0u; i < constraints.size(); ++i) {
    detail::ExactCoulombContactRowOperator::ContactFrame frame;
    if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraints[i], frame)) {
      mWarmStartContactRecords.clear();
      return;
    }

    const Eigen::Index offset = static_cast<Eigen::Index>(3u * i);
    ExactCoulombWarmStartContactRecord record;
    record.bodyA = frame.bodyA;
    record.bodyB = frame.bodyB;
    record.point = frame.point;
    record.worldNormal = frame.normal;
    const auto localFeatures = makeBodyLocalContactFeatures(frame);
    record.localPointA = localFeatures.sideA.point;
    record.localPointB = localFeatures.sideB.point;
    record.localNormalA = localFeatures.sideA.normal;
    record.localNormalB = localFeatures.sideB.normal;
    record.hasLocalFeatureA = localFeatures.sideA.available;
    record.hasLocalFeatureB = localFeatures.sideB.available;
    record.worldImpulse = reaction[offset] * frame.normal
                          + reaction[offset + 1] * frame.tangent1
                          + reaction[offset + 2] * frame.tangent2;
    if (frame.bodyB != nullptr
        && frame.bodyB->getWorldTransform().matrix().allFinite()) {
      record.localImpulseB
          = frame.bodyB->getWorldTransform().linear().transpose()
            * record.worldImpulse;
      record.hasLocalImpulseB = record.localImpulseB.allFinite();
    }
    record.stepSizeGroupId = stepSizeGroupId;
    record.stepSize = stepSize;
    record.consecutiveNoRejectionSolves = consecutiveNoRejectionSolves;
    groupRecords.push_back(record);
  }

  const auto groupOwnsPair = [&groupRecords,
                              &policy](const ExactCoulombWarmStartContactRecord&
                                           record) {
    for (const auto& groupRecord : groupRecords) {
      const bool sameOrder = groupRecord.bodyA == record.bodyA
                             && groupRecord.bodyB == record.bodyB;
      const bool flippedOrder = groupRecord.bodyA == record.bodyB
                                && groupRecord.bodyB == record.bodyA;
      if (sameOrder
          || (policy.warmStartMatchMode
                  == ExactCoulombFbfWarmStartMatchMode::EitherBodyLocalFeature
              && flippedOrder)) {
        return true;
      }
    }
    return false;
  };
  mWarmStartContactRecords.erase(
      std::remove_if(
          mWarmStartContactRecords.begin(),
          mWarmStartContactRecords.end(),
          groupOwnsPair),
      mWarmStartContactRecords.end());
  mWarmStartContactRecords.insert(
      mWarmStartContactRecords.end(), groupRecords.begin(), groupRecords.end());

  // Bound the store so pathological scenes cannot grow it without limit.
  constexpr std::size_t kMaxWarmStartContactRecords = 4096u;
  if (mWarmStartContactRecords.size() > kMaxWarmStartContactRecords) {
    mWarmStartContactRecords.erase(
        mWarmStartContactRecords.begin(),
        mWarmStartContactRecords.begin()
            + static_cast<std::ptrdiff_t>(
                mWarmStartContactRecords.size() - kMaxWarmStartContactRecords));
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::clearExactCoulombWarmStart()
{
  mHasExactCoulombWarmStart = false;
  mWarmStartConstraints.clear();
  mWarmStartReaction.resize(0);
  mWarmStartContactRecords.clear();
  clearExactCoulombPersistentStepSize();
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::clearExactCoulombPersistentStepSize()
{
  mWarmStartStepSize = std::numeric_limits<double>::quiet_NaN();
  mWarmStartConsecutiveNoRejectionSolves = 0u;
  for (auto& record : mWarmStartContactRecords) {
    record.stepSizeGroupId = 0u;
    record.stepSize = std::numeric_limits<double>::quiet_NaN();
    record.consecutiveNoRejectionSolves = 0u;
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    invalidateExactCoulombPersistentStepSizeForGroup(
        const std::vector<ConstraintBase*>& constraints)
{
  const double matchDistance = mExactCoulombOptions.warmStartMatchDistance;
  const auto policy = getExactCoulombPolicyState(this).crossStepOptions;
  if (constraints.empty() || mWarmStartContactRecords.empty()
      || !std::isfinite(matchDistance) || matchDistance <= 0.0) {
    return;
  }

  // A group's contact count or ordering may have changed on the failing solve.
  // Match failed contacts one-to-one with the same body-local identity used by
  // reaction and gamma reuse, then invalidate all matched prior group tokens.
  // The reactions themselves remain useful cone-projected cold seeds.
  std::vector<std::size_t> failedGroupIds;
  failedGroupIds.reserve(constraints.size());
  std::vector<bool> recordUsed(mWarmStartContactRecords.size(), false);
  for (const ConstraintBase* constraint : constraints) {
    detail::ExactCoulombContactRowOperator::ContactFrame frame;
    if (!detail::ExactCoulombContactRowOperator::extractContactFrame(
            constraint, frame)) {
      continue;
    }
    const auto currentFeatures = makeBodyLocalContactFeatures(frame);

    int bestRecord = -1;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t r = 0u; r < mWarmStartContactRecords.size(); ++r) {
      if (recordUsed[r])
        continue;

      const auto& record = mWarmStartContactRecords[r];
      if (record.stepSizeGroupId == 0u)
        continue;

      const auto identity = matchExactCoulombContactIdentity(
          record, frame, currentFeatures, matchDistance, policy);
      if (!identity.matched || identity.distance >= bestDistance)
        continue;

      bestDistance = identity.distance;
      bestRecord = static_cast<int>(r);
    }

    if (bestRecord < 0)
      continue;

    const std::size_t recordIndex = static_cast<std::size_t>(bestRecord);
    recordUsed[recordIndex] = true;
    const std::size_t stepSizeGroupId
        = mWarmStartContactRecords[recordIndex].stepSizeGroupId;
    if (stepSizeGroupId != 0u
        && std::find(
               failedGroupIds.begin(), failedGroupIds.end(), stepSizeGroupId)
               == failedGroupIds.end()) {
      failedGroupIds.push_back(stepSizeGroupId);
    }
  }

  if (failedGroupIds.empty()) {
    return;
  }

  for (auto& record : mWarmStartContactRecords) {
    if (std::find(
            failedGroupIds.begin(),
            failedGroupIds.end(),
            record.stepSizeGroupId)
        == failedGroupIds.end()) {
      continue;
    }

    record.stepSizeGroupId = 0u;
    record.stepSize = std::numeric_limits<double>::quiet_NaN();
    record.consecutiveNoRejectionSolves = 0u;
  }
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::
    invalidateExactCoulombWarmStartPointerCache()
{
  // One solve failure invalidates only the adapter pointer cache.
  // Body-local manifold reaction records remain safe seeds: warm starts are
  // cone-projected initial guesses, so a stale reaction can only cost
  // convergence speed, never correctness. The failing group's persisted gamma
  // is invalidated separately while other islands retain theirs.
  mHasExactCoulombWarmStart = false;
  mWarmStartConstraints.clear();
  mWarmStartReaction.resize(0);
  mWarmStartStepSize = std::numeric_limits<double>::quiet_NaN();
  mWarmStartConsecutiveNoRejectionSolves = 0u;
}

//==============================================================================
void ExactCoulombFbfConstraintSolver::recordExactCoulombResidualHistory(
    std::size_t contactCount,
    ExactCoulombFbfConstraintSolverStatus status,
    const math::detail::ExactCoulombFbfResult& solution,
    bool sourceContinuationActive)
{
  if (mExactCoulombOptions.maxResidualHistoryRecords <= 0
      || solution.residualHistory.empty()) {
    return;
  }

  const auto maxRecords = static_cast<std::size_t>(
      mExactCoulombOptions.maxResidualHistoryRecords);
  if (mExactCoulombResidualHistoryRecords.size() >= maxRecords) {
    mExactCoulombResidualHistoryRecords.erase(
        mExactCoulombResidualHistoryRecords.begin());
  }

  ExactCoulombFbfResidualHistoryRecord record;
  record.solveIndex = mResidualHistoryRecordSequence++;
  record.contactCount = contactCount;
  record.status = status;
  record.fbfStatus = solution.status;
  record.iterations = solution.iterations;
  record.finalResidual = solution.residual.value;
  record.finalNaturalMapResidual = solution.naturalMapResidual;
  record.plateauReferenceNaturalMapResidual
      = solution.plateauReferenceNaturalMapResidual;
  record.plateauRelativeImprovement = solution.plateauRelativeImprovement;
  record.shrinkIterations = solution.shrinkIterations;
  record.lineSearchShrinkCapCount = solution.lineSearchShrinkCapCount;
  record.lastInnerSolveStepSize = solution.lastInnerSolveStepSize;
  record.sourceContinuationActive = sourceContinuationActive;
  record.samples = solution.residualHistory;
  mExactCoulombResidualHistoryRecords.push_back(record);
}

} // namespace constraint
} // namespace dart
