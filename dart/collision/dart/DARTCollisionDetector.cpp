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

#include "dart/collision/dart/DARTCollisionDetector.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/collision/DistanceFilter.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/collision/dart/PersistentManifoldCache.hpp"
#include "dart/collision/dart/SoftCollision.hpp"
#include "dart/collision/dart/detail/DARTCollisionDetectorAccessor.hpp"
#include "dart/collision/dart/detail/DARTCollisionGroupEngineData.hpp"
#include "dart/collision/dart/detail/DARTCollisionObjectEngineData.hpp"
#include "dart/collision/dart/detail/FourPointPlanarCollision.hpp"
#include "dart/collision/dart/narrow_phase/NarrowPhase.hpp"
#include "dart/collision/dart/shapes/Shape.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Observer.hpp"
#include "dart/common/Profile.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace dart {
namespace collision {

static_assert(
    sizeof(DARTCollisionDetector) == sizeof(CollisionDetector),
    "DARTCollisionDetector must not add state to its exported ABI layout");
static_assert(
    alignof(DARTCollisionDetector) == alignof(CollisionDetector),
    "DARTCollisionDetector must preserve its exported ABI alignment");

//==============================================================================
class CollisionThreadPool
{
public:
  CollisionThreadPool() = default;

  ~CollisionThreadPool()
  {
    setWorkerCount(0u);
  }

  void setWorkerCount(std::size_t workerCount)
  {
    std::lock_guard<std::mutex> submitLock(mSubmitMutex);

    if (workerCount == mWorkers.size())
      return;

    stopWorkers();
    if (workerCount == 0u)
      return;

    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = false;
    }

    mWorkers.reserve(workerCount);
    for (std::size_t i = 0u; i < workerCount; ++i)
      mWorkers.emplace_back([this] { workerLoop(); });
  }

  template <typename Func>
  void parallelFor(std::size_t count, std::size_t numThreads, Func&& func)
  {
    if (count == 0u)
      return;

    std::lock_guard<std::mutex> submitLock(mSubmitMutex);

    const std::size_t totalParticipants = std::min<std::size_t>(
        std::min<std::size_t>(numThreads, count), mWorkers.size() + 1u);
    if (totalParticipants <= 1u) {
      for (std::size_t i = 0u; i < count; ++i)
        func(i);
      return;
    }

    const std::size_t chunkSize
        = (count + totalParticipants - 1u) / totalParticipants;
    const std::size_t workerCount = totalParticipants - 1u;
    using Function = typename std::remove_reference<Func>::type;

    {
      std::lock_guard<std::mutex> lock(mMutex);
      mTaskActive = true;
      mTaskCallable = static_cast<void*>(std::addressof(func));
      mTaskInvoker = [](void* callable, std::size_t begin, std::size_t end) {
        auto& task = *static_cast<Function*>(callable);
        for (std::size_t i = begin; i < end; ++i)
          task(i);
      };
      mTaskCount = count;
      mTaskChunkSize = chunkSize;
      mWorkerLimit = totalParticipants;
      mNextWorkerIndex = 1u;
      mActiveWorkerCount = workerCount;
      ++mTaskGeneration;
    }

    mTaskCv.notify_all();

    const std::size_t mainEnd = std::min<std::size_t>(count, chunkSize);
    for (std::size_t i = 0u; i < mainEnd; ++i)
      func(i);

    {
      std::unique_lock<std::mutex> lock(mMutex);
      mDoneCv.wait(lock, [this] { return mActiveWorkerCount == 0u; });
      mTaskActive = false;
      mTaskCallable = nullptr;
      mTaskInvoker = nullptr;
      mWorkerLimit = 1u;
    }
  }

private:
  using TaskInvoker = void (*)(void*, std::size_t, std::size_t);

  void stopWorkers()
  {
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mStop = true;
      ++mTaskGeneration;
    }

    mTaskCv.notify_all();

    for (auto& worker : mWorkers) {
      if (worker.joinable())
        worker.join();
    }
    mWorkers.clear();

    std::lock_guard<std::mutex> lock(mMutex);
    mStop = false;
    mTaskActive = false;
    mTaskCallable = nullptr;
    mTaskInvoker = nullptr;
    mActiveWorkerCount = 0u;
    mWorkerLimit = 1u;
    mNextWorkerIndex = 1u;
  }

  void workerLoop()
  {
    std::size_t observedGeneration = 0u;

    while (true) {
      void* callable = nullptr;
      TaskInvoker invoker = nullptr;
      std::size_t begin = 0u;
      std::size_t end = 0u;

      {
        std::unique_lock<std::mutex> lock(mMutex);
        mTaskCv.wait(lock, [&] {
          return mStop || observedGeneration != mTaskGeneration;
        });

        if (mStop)
          return;

        observedGeneration = mTaskGeneration;
        if (!mTaskActive || mNextWorkerIndex >= mWorkerLimit)
          continue;

        const std::size_t workerIndex = mNextWorkerIndex++;
        begin = workerIndex * mTaskChunkSize;
        end = std::min<std::size_t>(mTaskCount, begin + mTaskChunkSize);
        callable = mTaskCallable;
        invoker = mTaskInvoker;
      }

      if (begin < end)
        invoker(callable, begin, end);

      {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mActiveWorkerCount > 0u)
          --mActiveWorkerCount;
        if (mActiveWorkerCount == 0u)
          mDoneCv.notify_one();
      }
    }
  }

  std::vector<std::thread> mWorkers;
  std::mutex mSubmitMutex;
  std::mutex mMutex;
  std::condition_variable mTaskCv;
  std::condition_variable mDoneCv;
  bool mStop{false};
  bool mTaskActive{false};
  std::size_t mTaskGeneration{0u};
  std::size_t mTaskCount{0u};
  std::size_t mTaskChunkSize{0u};
  std::size_t mWorkerLimit{1u};
  std::size_t mNextWorkerIndex{1u};
  std::size_t mActiveWorkerCount{0u};
  void* mTaskCallable{nullptr};
  TaskInvoker mTaskInvoker{nullptr};
};

namespace {

//==============================================================================
constexpr std::size_t kCompactManifoldContactTarget = 3u;
constexpr std::size_t kFourPointPlanarContactTarget = 4u;
constexpr std::size_t kPlaneBoxManifoldContactTarget = 4u;
constexpr double kConvexFaceOppositionCosine = 0.995;
constexpr double kConvexFaceContactNormalCosine = 0.95;
constexpr double kConvexFacePlaneToleranceScale = 1e-8;
constexpr double kConvexFacePolygonToleranceScale = 1e-10;
constexpr double kConvexFacePatchAreaToleranceScale = 1e-12;
constexpr double kConvexFacePatchAreaFractionTolerance = 1e-4;

//==============================================================================
std::size_t getSolverFacingManifoldContactTarget(
    DARTCollisionDetector::ContactManifoldMode mode)
{
  return mode == DARTCollisionDetector::ContactManifoldMode::FourPointPlanar
             ? kFourPointPlanarContactTarget
             : kCompactManifoldContactTarget;
}

// Bound retained parallel scratch independently of scene pair count. Batches
// are merged before the broadphase or Cartesian traversal continues, keeping
// contact-cap behavior deterministic and memory usage O(1) in pair count.
constexpr std::size_t kParallelPairBatchSize = 256u;

//==============================================================================
bool checkGroupValidity(
    const DARTCollisionDetector* detector, CollisionGroup* group)
{
  if (!group) {
    dterr << "[DARTCollisionDetector::collide] Attempting to check collision "
          << "with a nullptr collision group.\n";
    return false;
  }

  if (detector != group->getCollisionDetector().get()) {
    dterr << "[DARTCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
native::CollisionOption makeNativeOption(
    const CollisionOption& option,
    const CollisionResult* result,
    DARTCollisionDetector::ContactManifoldMode mode)
{
  native::CollisionOption nativeOption;
  nativeOption.enableContact = option.enableContact && result != nullptr;
  if (nativeOption.enableContact) {
    const std::size_t maxPairContacts
        = option.getEffectiveMaxNumContactsPerPair();
    // DARTCollide uses an explicit unlimited per-pair request to preserve its
    // released full-manifold behavior. Ordinary detector queries retain the
    // solver-facing three-contact target, including wider finite requests.
    const bool preserveCompleteManifold
        = option.maxNumContactsPerPair
          == std::numeric_limits<std::size_t>::max();
    nativeOption.maxNumContacts
        = preserveCompleteManifold
              ? maxPairContacts
              : std::min(
                  maxPairContacts, getSolverFacingManifoldContactTarget(mode));
  } else {
    nativeOption.maxNumContacts = 1u;
  }
  nativeOption.collisionFilter = nullptr;

  return nativeOption;
}

//==============================================================================
bool shouldSkipPair(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
    const CollisionOption& option)
{
  const bool softPair = detail::isSoftCollisionPair(object1, object2);
  if (!softPair
      && (!detail::DARTCollisionObjectAccessor::getShape(object1)
          || !detail::DARTCollisionObjectAccessor::getShape(object2)))
    return true;

  if (option.collisionFilter
      && option.collisionFilter->ignoresCollision(object1, object2)) {
    return true;
  }

  return false;
}

//==============================================================================
bool shouldSkipDistancePair(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
    const DistanceOption& option)
{
  if (!detail::DARTCollisionObjectAccessor::getShape(object1)
      || !detail::DARTCollisionObjectAccessor::getShape(object2))
    return true;

  if (option.distanceFilter
      && !option.distanceFilter->needDistance(object1, object2)) {
    return true;
  }

  return false;
}

//==============================================================================
native::DistanceOption makeNativeDistanceOption(
    const DistanceOption& option, double upperBound)
{
  native::DistanceOption nativeOption;
  nativeOption.upperBound = upperBound;
  nativeOption.enableNearestPoints = option.enableNearestPoints;

  return nativeOption;
}

//==============================================================================
struct NativeDistanceCandidate
{
  double distance = std::numeric_limits<double>::max();
  native::DistanceResult result;
  const DARTCollisionObject* object1 = nullptr;
  const DARTCollisionObject* object2 = nullptr;
  bool found = false;
};

//==============================================================================
struct NativeRayHitCandidate
{
  RayHit hit;
  double distance = std::numeric_limits<double>::max();
};

//==============================================================================
std::size_t getManifoldCacheId(const CollisionObject* object)
{
  // Every caller passes objects created by this detector (group members,
  // contacts of an in-flight collide(), or objects pre-checked against
  // getCollisionDetector()), so they are always DARTCollisionObjects; this
  // runs per contact per step, so avoid the dynamic_cast.
  const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
  if (dartObject->getCachedShapeKind()
      == DARTCollisionObject::CachedShapeKind::SoftMesh) {
    return 0u;
  }

  return static_cast<std::size_t>(reinterpret_cast<std::uintptr_t>(object));
}

//==============================================================================
class ScratchCollisionResult final : public CollisionResult
{
public:
  explicit ScratchCollisionResult(std::size_t contactCapacity = 0u)
  {
    setCollidingObjectCacheEnabled(false);
    mContacts.reserve(contactCapacity);
  }
};

struct ParallelPairResult
{
  ParallelPairResult() : result(kFourPointPlanarContactTarget)
  {
    // Rigid narrowphase normally returns one manifold. Allocate its reusable
    // storage with the fixed batch scratch instead of on a later contact-
    // topology change.
    narrowphaseScratch.addManifold(native::ContactManifold{});
    narrowphaseScratch.clear();
    for (std::size_t i = 0u; i < kFourPointPlanarContactTarget; ++i)
      narrowphaseScratch.addContact(native::ContactPoint{});
    DART_UNUSED(
        narrowphaseScratch.getContact(kFourPointPlanarContactTarget - 1u));
    narrowphaseScratch.clear();
  }

  native::CollisionResult narrowphaseScratch;
  ScratchCollisionResult result;
  bool eligible{false};
  bool dispatchToWorker{false};
  bool collisionFound{false};
};

struct ParallelObjectPair
{
  ParallelObjectPair(
      DARTCollisionObject* object1,
      DARTCollisionObject* object2,
      bool eligibilityChecked = false)
    : first(object1), second(object2), eligibilityChecked(eligibilityChecked)
  {
  }

  DARTCollisionObject* first;
  DARTCollisionObject* second;
  bool eligibilityChecked;
};

using ContactGapMap = std::unordered_map<const dynamics::ShapeFrame*, double>;

struct ContactGapPolicySnapshot
{
  std::shared_ptr<const ContactGapMap> contactGaps;
  std::size_t revision{0u};
};

//==============================================================================
class ContactGapOverrides final : public common::Observer
{
public:
  void set(const dynamics::ShapeFrame* shapeFrame, double contactGap)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    const auto current = std::atomic_load(&mSnapshot);
    auto updated = current ? std::make_shared<ContactGapMap>(*current)
                           : std::make_shared<ContactGapMap>();
    const auto existing = updated->find(shapeFrame);
    if (contactGap == 0.0) {
      if (existing == updated->end())
        return;

      updated->erase(existing);
      const auto* subject = static_cast<const common::Subject*>(shapeFrame);
      mFramesBySubject.erase(subject);
      removeSubject(subject);
    } else {
      if (existing != updated->end() && existing->second == contactGap)
        return;

      if (existing == updated->end()) {
        const auto* subject = static_cast<const common::Subject*>(shapeFrame);
        addSubject(subject);
        mFramesBySubject.emplace(subject, shapeFrame);
      }
      (*updated)[shapeFrame] = contactGap;
    }
    publish(std::move(updated));
  }

  [[nodiscard]] double get(const dynamics::ShapeFrame* shapeFrame) const
  {
    const auto snapshot = std::atomic_load(&mSnapshot);
    if (!snapshot)
      return 0.0;

    const auto it = snapshot->find(shapeFrame);
    return it == snapshot->end() ? 0.0 : it->second;
  }

  [[nodiscard]] std::shared_ptr<const ContactGapMap> snapshot() const
  {
    return std::atomic_load(&mSnapshot);
  }

  [[nodiscard]] ContactGapPolicySnapshot snapshotWithRevision() const
  {
    std::lock_guard<std::mutex> lock(mMutex);
    return {std::atomic_load(&mSnapshot), mRevision};
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (!std::atomic_load(&mSnapshot))
      return;

    removeAllSubjects();
    mFramesBySubject.clear();
    std::atomic_store(&mSnapshot, std::shared_ptr<const ContactGapMap>{});
    ++mRevision;
  }

protected:
  void handleDestructionNotification(const common::Subject* subject) override
  {
    std::lock_guard<std::mutex> lock(mMutex);
    const auto frameIt = mFramesBySubject.find(subject);
    if (frameIt == mFramesBySubject.end())
      return;

    const auto current = std::atomic_load(&mSnapshot);
    if (current) {
      auto updated = std::make_shared<ContactGapMap>(*current);
      updated->erase(frameIt->second);
      publish(std::move(updated));
    }
    mFramesBySubject.erase(frameIt);
  }

private:
  void publish(std::shared_ptr<ContactGapMap> updated)
  {
    if (updated->empty()) {
      std::atomic_store(&mSnapshot, std::shared_ptr<const ContactGapMap>{});
      ++mRevision;
      return;
    }

    std::shared_ptr<const ContactGapMap> immutable = std::move(updated);
    std::atomic_store(&mSnapshot, std::move(immutable));
    ++mRevision;
  }

  mutable std::mutex mMutex;
  std::unordered_map<const common::Subject*, const dynamics::ShapeFrame*>
      mFramesBySubject;
  std::shared_ptr<const ContactGapMap> mSnapshot;
  std::size_t mRevision{0u};
};

//==============================================================================
double lookupContactGap(
    const ContactGapMap* contactGaps, const dynamics::ShapeFrame* shapeFrame)
{
  if (!contactGaps || !shapeFrame)
    return 0.0;

  const auto it = contactGaps->find(shapeFrame);
  return it == contactGaps->end() ? 0.0 : it->second;
}

//==============================================================================
double summedContactGap(
    const ContactGapMap* contactGaps,
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2)
{
  if (!contactGaps)
    return 0.0;

  const double sum = lookupContactGap(contactGaps, object1->getShapeFrame())
                     + lookupContactGap(contactGaps, object2->getShapeFrame());
  return std::isfinite(sum) ? sum : 0.0;
}

// Per-detector engine state: runtime options, the persistent manifold cache,
// and scratch buffers reused across collide() calls so steady-state stepping
// performs no heap allocations (StepAllocation gate discipline). Concurrent
// collide() on one detector is not supported, matching the cache contract.
struct DetectorEngineState
{
  std::mutex threadConfigMutex;
  mutable std::mutex contactPolicyMutex;
  CollisionThreadPool collisionThreadPool;
  std::atomic<std::size_t> numCollisionThreads{1u};
  std::atomic<bool> softFaceInteriorContactsEnabled{false};
  std::atomic<DARTCollisionDetector::ContactManifoldMode> contactManifoldMode{
      DARTCollisionDetector::ContactManifoldMode::Compact};
  std::shared_ptr<ContactGapOverrides> contactGapOverrides;
  native::PersistentManifoldCache manifoldCache;
  native::CollisionResult narrowphaseScratch;
  ScratchCollisionResult softPairScratch;
  std::vector<std::pair<std::size_t, CollisionObject*>> objectsByIdScratch;
  std::vector<std::size_t> pendingContactIndicesScratch;
  std::vector<Eigen::Vector3d> pendingLocalPointsScratch;
  std::vector<ParallelObjectPair> parallelObjectPairsScratch;
  std::vector<ParallelPairResult> parallelPairResults;
};

//==============================================================================
RayHit convertRayHit(
    const native::RaycastResult& nativeResult,
    const DARTCollisionObject* object,
    double rayLength)
{
  RayHit hit;
  hit.mCollisionObject = object;
  hit.mPoint = nativeResult.point;
  hit.mNormal = nativeResult.normal;
  hit.mFraction = nativeResult.distance / rayLength;
  return hit;
}

//==============================================================================
bool processNativeRaycastObject(
    const DARTCollisionObject* object,
    const native::Ray& ray,
    double rayLength,
    const RaycastOption& option,
    std::vector<NativeRayHitCandidate>& hits)
{
  const native::Shape* shape
      = detail::DARTCollisionObjectAccessor::getShape(object);
  if (!shape)
    return false;

  if (!native::NarrowPhase::isRaycastSupported(shape->getType()))
    return false;

  native::RaycastResult nativeResult;
  native::RaycastOption nativeOption
      = native::RaycastOption::withMaxDistance(rayLength);
  nativeOption.backfaceCulling = false;
  const bool hit = native::NarrowPhase::raycast(
      ray,
      shape,
      detail::DARTCollisionObjectAccessor::getTransform(object),
      nativeOption,
      nativeResult);
  if (!hit || !option.passesFilter(object))
    return false;

  hits.push_back(NativeRayHitCandidate{
      convertRayHit(nativeResult, object, rayLength), nativeResult.distance});
  return true;
}

//==============================================================================
void convertRaycastResults(
    const std::vector<NativeRayHitCandidate>& hits,
    const RaycastOption& option,
    RaycastResult& result)
{
  result.clear();

  if (hits.empty())
    return;

  if (!option.mEnableAllHits) {
    const auto closest = std::min_element(
        hits.begin(),
        hits.end(),
        [](const NativeRayHitCandidate& a, const NativeRayHitCandidate& b) {
          return a.distance < b.distance;
        });
    result.mRayHits.emplace_back(closest->hit);
    return;
  }

  result.mRayHits.reserve(hits.size());
  for (const auto& hit : hits)
    result.mRayHits.emplace_back(hit.hit);

  if (option.mSortByClosest) {
    std::sort(
        result.mRayHits.begin(),
        result.mRayHits.end(),
        [](const RayHit& a, const RayHit& b) {
          return a.mFraction < b.mFraction;
        });
  }
}

//==============================================================================
bool processNativeDistancePair(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
    const DistanceOption& option,
    NativeDistanceCandidate& best)
{
  if (shouldSkipDistancePair(object1, object2, option))
    return false;

  native::DistanceResult nativeResult;
  const native::DistanceOption nativeOption = makeNativeDistanceOption(
      option, best.found ? best.distance : std::numeric_limits<double>::max());

  const double distance = native::NarrowPhase::distance(
      detail::DARTCollisionObjectAccessor::getShape(object1),
      detail::DARTCollisionObjectAccessor::getTransform(object1),
      detail::DARTCollisionObjectAccessor::getShape(object2),
      detail::DARTCollisionObjectAccessor::getTransform(object2),
      nativeOption,
      nativeResult);

  if (!nativeResult.isValid())
    return false;

  if (!best.found || distance < best.distance) {
    best.distance = distance;
    best.result = nativeResult;
    best.object1 = object1;
    best.object2 = object2;
    best.found = true;
  }

  return best.distance <= option.distanceLowerBound;
}

//==============================================================================
double convertDistanceResult(
    const NativeDistanceCandidate& best,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (!best.found)
    return std::max(0.0, option.distanceLowerBound);

  const double minDistance = std::max(best.distance, option.distanceLowerBound);

  if (result) {
    result->minDistance = minDistance;
    result->unclampedMinDistance = best.distance;
    result->shapeFrame1 = best.object1->getShapeFrame();
    result->shapeFrame2 = best.object2->getShapeFrame();

    if (option.enableNearestPoints) {
      result->nearestPoint1 = best.result.pointOnObject1;
      result->nearestPoint2 = best.result.pointOnObject2;
    }
  }

  return minDistance;
}

//==============================================================================
void addPairOnlyContact(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    CollisionResult& result)
{
  Contact contact;
  contact.collisionObject1 = object1;
  contact.collisionObject2 = object2;
  result.addContact(contact);
}

//==============================================================================
void attachCachedContactImpulses(
    CollisionResult* result, DetectorEngineState& engineState)
{
  auto* manifoldCache = &engineState.manifoldCache;
  if (!result)
    return;

  if (result->getNumContacts() == 1u) {
    auto& contact = result->getContact(0);
    auto* object1 = contact.collisionObject1;
    auto* object2 = contact.collisionObject2;
    if (!object1 || !object2)
      return;

    const auto id1 = getManifoldCacheId(object1);
    const auto id2 = getManifoldCacheId(object2);
    if (id1 == 0u || id2 == 0u)
      return;

    const auto tf1 = object1->getTransform();
    const auto tf2 = object2->getTransform();
    const auto tf1Inv = tf1.inverse();
    const auto tf2Inv = tf2.inverse();
    const bool swapped = id2 < id1;

    native::CachedContact cached;
    cached.localPointA
        = swapped ? tf2Inv * contact.point : tf1Inv * contact.point;
    cached.localPointB
        = swapped ? tf1Inv * contact.point : tf2Inv * contact.point;
    cached.normal = contact.normal;
    cached.penetrationDepth = contact.penetrationDepth;

    auto& manifold = manifoldCache->getOrCreate(id1, id2);
    manifold.addOrReplace(cached);
    if (manifold.numContacts == 1) {
      auto& manifoldContact = manifold.contacts[0];
      contact.userData = &manifoldContact;
      return;
    }

    manifold.refresh(swapped ? tf2 : tf1, swapped ? tf1 : tf2);
    if (manifold.numContacts == 0) {
      manifoldCache->remove(id1, id2);
      return;
    }

    const auto match = manifold.findMatch(cached.localPointA);
    if (match < 0)
      return;

    const auto matchIndex = static_cast<std::size_t>(match);
    auto& manifoldContact = manifold.contacts[matchIndex];
    contact.userData = &manifoldContact;
    return;
  }

  CollisionObject* cachedObject1 = nullptr;
  CollisionObject* cachedObject2 = nullptr;
  Eigen::Isometry3d tf1;
  Eigen::Isometry3d tf2;
  Eigen::Isometry3d tf1Inv;
  Eigen::Isometry3d tf2Inv;
  std::size_t id1 = 0u;
  std::size_t id2 = 0u;
  bool swapped = false;
  native::PersistentManifold* manifold = nullptr;
  auto& pendingContactIndices = engineState.pendingContactIndicesScratch;
  auto& pendingLocalPoints = engineState.pendingLocalPointsScratch;
  pendingContactIndices.clear();
  pendingLocalPoints.clear();
  pendingContactIndices.reserve(result->getNumContacts());
  pendingLocalPoints.reserve(result->getNumContacts());

  auto flushPair = [&]() {
    if (!manifold || pendingContactIndices.empty())
      return;

    manifold->refresh(swapped ? tf2 : tf1, swapped ? tf1 : tf2);
    if (manifold->numContacts == 0) {
      manifoldCache->remove(id1, id2);
      manifold = nullptr;
      pendingContactIndices.clear();
      pendingLocalPoints.clear();
      return;
    }

    std::array<bool, native::PersistentManifold::kMaxContacts> assignedSlots{};
    for (std::size_t pendingIndex = 0u;
         pendingIndex < pendingContactIndices.size();
         ++pendingIndex) {
      const auto match = manifold->findMatch(pendingLocalPoints[pendingIndex]);
      if (match < 0)
        continue;

      const auto matchIndex = static_cast<std::size_t>(match);
      if (assignedSlots[matchIndex])
        continue;
      assignedSlots[matchIndex] = true;

      auto& contact = result->getContact(pendingContactIndices[pendingIndex]);
      auto& manifoldContact = manifold->contacts[matchIndex];
      contact.userData = &manifoldContact;
    }

    manifold = nullptr;
    pendingContactIndices.clear();
    pendingLocalPoints.clear();
  };

  for (std::size_t i = 0u; i < result->getNumContacts(); ++i) {
    auto& contact = result->getContact(i);
    auto* object1 = contact.collisionObject1;
    auto* object2 = contact.collisionObject2;
    if (!object1 || !object2)
      continue;

    if (object1 != cachedObject1 || object2 != cachedObject2) {
      flushPair();
      cachedObject1 = object1;
      cachedObject2 = object2;
      tf1 = object1->getTransform();
      tf2 = object2->getTransform();
      tf1Inv = tf1.inverse();
      tf2Inv = tf2.inverse();
      id1 = getManifoldCacheId(object1);
      id2 = getManifoldCacheId(object2);
      swapped = id2 < id1;
    }

    if (id1 == 0u || id2 == 0u) {
      flushPair();
      cachedObject1 = nullptr;
      cachedObject2 = nullptr;
      continue;
    }

    if (!manifold)
      manifold = &manifoldCache->getOrCreate(id1, id2);

    native::CachedContact cached;
    cached.localPointA
        = swapped ? tf2Inv * contact.point : tf1Inv * contact.point;
    cached.localPointB
        = swapped ? tf1Inv * contact.point : tf2Inv * contact.point;
    cached.normal = contact.normal;
    cached.penetrationDepth = contact.penetrationDepth;

    manifold->addOrReplace(cached);
    pendingContactIndices.push_back(i);
    pendingLocalPoints.push_back(cached.localPointA);
  }

  flushPair();
}

namespace {

// Sorted id->object scratch lookup: a rebuilt std::unordered_map would
// allocate one hash node per object on every collide() and trip the
// StepAllocation gates; a capacity-retaining sorted vector allocates nothing
// in steady state.
using ObjectsByIdScratch
    = std::vector<std::pair<std::size_t, CollisionObject*>>;

void appendManifoldCacheIds(
    const std::vector<CollisionObject*>& objects, ObjectsByIdScratch& scratch)
{
  for (auto* object : objects) {
    const auto id = getManifoldCacheId(object);
    if (id != 0u)
      scratch.emplace_back(id, object);
  }
}

void refreshManifoldCacheImpl(
    native::PersistentManifoldCache& manifoldCache, ObjectsByIdScratch& scratch)
{
  std::sort(
      scratch.begin(), scratch.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
      });

  const auto findObject = [&scratch](std::size_t id) -> CollisionObject* {
    const auto it = std::lower_bound(
        scratch.begin(),
        scratch.end(),
        id,
        [](const auto& entry, std::size_t value) {
          return entry.first < value;
        });
    if (it == scratch.end() || it->first != id)
      return nullptr;

    return it->second;
  };

  manifoldCache.refreshAllWith(
      [&findObject](std::size_t idA, std::size_t idB)
          -> std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> {
        auto* objectA = findObject(idA);
        auto* objectB = findObject(idB);
        if (!objectA || !objectB)
          return std::nullopt;

        return std::make_pair(objectA->getTransform(), objectB->getTransform());
      },
      /*breakingThreshold=*/0.04);
}

} // namespace

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects,
    DetectorEngineState& engineState)
{
  auto& scratch = engineState.objectsByIdScratch;
  scratch.clear();
  appendManifoldCacheIds(objects, scratch);
  refreshManifoldCacheImpl(engineState.manifoldCache, scratch);
}

//==============================================================================
void refreshManifoldCache(
    const std::vector<CollisionObject*>& objects1,
    const std::vector<CollisionObject*>& objects2,
    DetectorEngineState& engineState)
{
  auto& scratch = engineState.objectsByIdScratch;
  scratch.clear();
  appendManifoldCacheIds(objects1, scratch);
  appendManifoldCacheIds(objects2, scratch);
  refreshManifoldCacheImpl(engineState.manifoldCache, scratch);
}

//==============================================================================
bool emitContacts(
    const native::CollisionResult& nativeResult,
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const CollisionOption& option,
    CollisionResult& result)
{
  const std::size_t maxPairContacts
      = option.getEffectiveMaxNumContactsPerPair();
  std::size_t emittedForPair = 0u;

  const std::size_t numContacts = nativeResult.numContacts();
  for (std::size_t i = 0u; i < numContacts; ++i) {
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;

    if (emittedForPair >= maxPairContacts)
      return false;

    const auto& nativeContact = nativeResult.getContact(i);
    if (nativeContact.depth < 0.0
        && !option.allowNegativePenetrationDepthContacts) {
      continue;
    }

    if (Contact::isZeroNormal(nativeContact.normal))
      continue;

    Contact contact;
    contact.point = nativeContact.position;
    contact.normal = nativeContact.normal;
    contact.penetrationDepth = nativeContact.depth;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.triID1 = nativeContact.featureIndex1;
    contact.triID2 = nativeContact.featureIndex2;
    result.addContact(contact);
    ++emittedForPair;
  }

  return result.getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
struct ConvexWorldFace
{
  std::size_t index = 0u;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> vertices;
};

//==============================================================================
double cross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
{
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

//==============================================================================
double signedPolygonArea(const std::vector<Eigen::Vector2d>& polygon)
{
  double twiceArea = 0.0;
  for (std::size_t i = 0u; i < polygon.size(); ++i) {
    twiceArea += cross2d(polygon[i], polygon[(i + 1u) % polygon.size()]);
  }
  return 0.5 * twiceArea;
}

//==============================================================================
void removeAdjacentDuplicatePoints(
    std::vector<Eigen::Vector2d>& polygon, double tolerance)
{
  if (polygon.empty())
    return;

  const double toleranceSquared = tolerance * tolerance;
  std::vector<Eigen::Vector2d> unique;
  unique.reserve(polygon.size());
  for (const auto& point : polygon) {
    if (unique.empty()
        || (point - unique.back()).squaredNorm() > toleranceSquared) {
      unique.push_back(point);
    }
  }
  if (unique.size() > 1u
      && (unique.front() - unique.back()).squaredNorm() <= toleranceSquared) {
    unique.pop_back();
  }
  polygon = std::move(unique);
}

//==============================================================================
void orderConvexPolygon(std::vector<Eigen::Vector2d>& polygon, double tolerance)
{
  removeAdjacentDuplicatePoints(polygon, tolerance);
  if (polygon.size() < 3u)
    return;

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& point : polygon)
    centroid += point;
  centroid /= static_cast<double>(polygon.size());

  std::sort(
      polygon.begin(),
      polygon.end(),
      [&](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
        const Eigen::Vector2d lhsDelta = lhs - centroid;
        const Eigen::Vector2d rhsDelta = rhs - centroid;
        const double lhsAngle = std::atan2(lhsDelta.y(), lhsDelta.x());
        const double rhsAngle = std::atan2(rhsDelta.y(), rhsDelta.x());
        if (lhsAngle != rhsAngle)
          return lhsAngle < rhsAngle;
        if (lhsDelta.squaredNorm() != rhsDelta.squaredNorm())
          return lhsDelta.squaredNorm() < rhsDelta.squaredNorm();
        if (lhs.x() != rhs.x())
          return lhs.x() < rhs.x();
        return lhs.y() < rhs.y();
      });
  removeAdjacentDuplicatePoints(polygon, tolerance);
  if (polygon.size() >= 3u && signedPolygonArea(polygon) < 0.0)
    std::reverse(polygon.begin(), polygon.end());
}

//==============================================================================
std::vector<Eigen::Vector2d> clipConvexPolygon(
    std::vector<Eigen::Vector2d> subject,
    const std::vector<Eigen::Vector2d>& clip,
    double tolerance)
{
  for (std::size_t clipIndex = 0u; clipIndex < clip.size(); ++clipIndex) {
    if (subject.empty())
      break;

    const Eigen::Vector2d& edgeStart = clip[clipIndex];
    const Eigen::Vector2d& edgeEnd = clip[(clipIndex + 1u) % clip.size()];
    const Eigen::Vector2d edge = edgeEnd - edgeStart;
    std::vector<Eigen::Vector2d> output;
    output.reserve(subject.size() + 1u);

    Eigen::Vector2d previous = subject.back();
    double previousDistance = cross2d(edge, previous - edgeStart);
    bool previousInside = previousDistance >= -tolerance;
    for (const auto& current : subject) {
      const double currentDistance = cross2d(edge, current - edgeStart);
      const bool currentInside = currentDistance >= -tolerance;
      if (currentInside != previousInside) {
        const double denominator = previousDistance - currentDistance;
        if (std::abs(denominator) > std::numeric_limits<double>::epsilon()) {
          const double ratio = previousDistance / denominator;
          output.push_back(previous + ratio * (current - previous));
        }
      }
      if (currentInside)
        output.push_back(current);

      previous = current;
      previousDistance = currentDistance;
      previousInside = currentInside;
    }
    removeAdjacentDuplicatePoints(output, tolerance);
    subject = std::move(output);
  }
  return subject;
}

//==============================================================================
template <typename DepthFunction>
std::vector<Eigen::Vector2d> clipPolygonByContactDepth(
    const std::vector<Eigen::Vector2d>& polygon,
    const DepthFunction& getDepth,
    double contactTolerance,
    double polygonTolerance)
{
  if (polygon.empty())
    return {};

  std::vector<Eigen::Vector2d> output;
  output.reserve(polygon.size() + 1u);
  Eigen::Vector2d previous = polygon.back();
  double previousValue = getDepth(previous) + contactTolerance;
  bool previousInside = previousValue >= 0.0;
  for (const auto& current : polygon) {
    const double currentValue = getDepth(current) + contactTolerance;
    const bool currentInside = currentValue >= 0.0;
    if (currentInside != previousInside) {
      const double denominator = previousValue - currentValue;
      if (std::abs(denominator) > std::numeric_limits<double>::epsilon()) {
        const double ratio = previousValue / denominator;
        output.push_back(previous + ratio * (current - previous));
      }
    }
    if (currentInside)
      output.push_back(current);

    previous = current;
    previousValue = currentValue;
    previousInside = currentInside;
  }
  removeAdjacentDuplicatePoints(output, polygonTolerance);
  return output;
}

//==============================================================================
double computeConvexWorldScale(
    const native::ConvexShape& shape, const Eigen::Isometry3d& transform)
{
  if (shape.getVertices().empty())
    return 1.0;

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto& vertex : shape.getVertices())
    centroid += transform * vertex;
  centroid /= static_cast<double>(shape.getVertices().size());

  double scale = 0.0;
  for (const auto& vertex : shape.getVertices())
    scale = std::max(scale, (transform * vertex - centroid).norm());
  return std::max(1.0, scale);
}

//==============================================================================
std::vector<ConvexWorldFace> makeConvexWorldFaces(
    const native::ConvexShape& shape,
    const Eigen::Isometry3d& transform,
    double scale)
{
  std::vector<Eigen::Vector3d> worldVertices;
  worldVertices.reserve(shape.getVertices().size());
  for (const auto& vertex : shape.getVertices())
    worldVertices.push_back(transform * vertex);

  const double planeTolerance = kConvexFacePlaneToleranceScale * scale;
  const double duplicateToleranceSquared = planeTolerance * planeTolerance;
  std::vector<ConvexWorldFace> result;
  result.reserve(shape.getFaces().size());
  for (std::size_t faceIndex = 0u; faceIndex < shape.getFaces().size();
       ++faceIndex) {
    const auto& localFace = shape.getFaces()[faceIndex];
    ConvexWorldFace face;
    face.index = faceIndex;
    face.point = transform * localFace.point;
    face.normal = transform.linear() * localFace.normal;
    const double normalNorm = face.normal.norm();
    if (!(normalNorm > 0.0) || !std::isfinite(normalNorm))
      continue;
    face.normal /= normalNorm;

    for (const auto& vertex : worldVertices) {
      if (std::abs(face.normal.dot(vertex - face.point)) > planeTolerance)
        continue;

      bool duplicate = false;
      for (const auto& existing : face.vertices) {
        if ((vertex - existing).squaredNorm() <= duplicateToleranceSquared) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate)
        face.vertices.push_back(vertex);
    }
    if (face.vertices.size() >= 3u)
      result.push_back(std::move(face));
  }
  return result;
}

//==============================================================================
void makePlanarBasis(
    const Eigen::Vector3d& normal,
    Eigen::Vector3d& tangent1,
    Eigen::Vector3d& tangent2)
{
  int axis = 0;
  normal.cwiseAbs().minCoeff(&axis);
  tangent1 = normal.cross(Eigen::Vector3d::Unit(axis)).normalized();
  tangent2 = normal.cross(tangent1);
}

//==============================================================================
bool contactPointLess(
    const native::ContactPoint& lhs, const native::ContactPoint& rhs)
{
  for (Eigen::Index i = 0; i < 3; ++i) {
    if (lhs.position[i] != rhs.position[i])
      return lhs.position[i] < rhs.position[i];
  }
  return lhs.depth > rhs.depth;
}

//==============================================================================
struct ConvexFacePatchCandidate
{
  std::size_t faceIndex1 = 0u;
  std::size_t faceIndex2 = 0u;
  double score = std::numeric_limits<double>::max();
  std::vector<native::ContactPoint> contacts;
};

//==============================================================================
std::optional<ConvexFacePatchCandidate> makeConvexFacePatchCandidate(
    const ConvexWorldFace& face1,
    const ConvexWorldFace& face2,
    const native::ContactPoint& compactContact,
    double scale,
    double proximityGap)
{
  if (face1.normal.dot(face2.normal) > -kConvexFaceOppositionCosine)
    return std::nullopt;

  Eigen::Vector3d outwardNormal = face1.normal - face2.normal;
  const double outwardNormalNorm = outwardNormal.norm();
  if (!(outwardNormalNorm > 0.0))
    return std::nullopt;
  outwardNormal /= outwardNormalNorm;
  const Eigen::Vector3d contactNormal = -outwardNormal;
  const double normalAlignment = contactNormal.dot(compactContact.normal);
  if (normalAlignment < kConvexFaceContactNormalCosine)
    return std::nullopt;

  const double denominator1 = face1.normal.dot(outwardNormal);
  const double denominator2 = face2.normal.dot(outwardNormal);
  if (denominator1 < kConvexFaceOppositionCosine
      || denominator2 > -kConvexFaceOppositionCosine) {
    return std::nullopt;
  }

  const Eigen::Vector3d origin = 0.5 * (face1.point + face2.point);
  Eigen::Vector3d tangent1;
  Eigen::Vector3d tangent2;
  makePlanarBasis(outwardNormal, tangent1, tangent2);
  const auto project = [&](const Eigen::Vector3d& point) {
    const Eigen::Vector3d delta = point - origin;
    return Eigen::Vector2d(delta.dot(tangent1), delta.dot(tangent2));
  };
  const auto unproject = [&](const Eigen::Vector2d& point) {
    return origin + point.x() * tangent1 + point.y() * tangent2;
  };

  const double polygonTolerance = kConvexFacePolygonToleranceScale * scale;
  std::vector<Eigen::Vector2d> polygon1;
  polygon1.reserve(face1.vertices.size());
  for (const auto& vertex : face1.vertices)
    polygon1.push_back(project(vertex));
  orderConvexPolygon(polygon1, polygonTolerance);

  std::vector<Eigen::Vector2d> polygon2;
  polygon2.reserve(face2.vertices.size());
  for (const auto& vertex : face2.vertices)
    polygon2.push_back(project(vertex));
  orderConvexPolygon(polygon2, polygonTolerance);
  if (polygon1.size() < 3u || polygon2.size() < 3u)
    return std::nullopt;

  auto overlap = clipConvexPolygon(polygon1, polygon2, polygonTolerance);
  if (overlap.size() < 3u)
    return std::nullopt;
  orderConvexPolygon(overlap, polygonTolerance);

  const auto getSurfaceData = [&](const Eigen::Vector2d& point,
                                  Eigen::Vector3d* position1,
                                  Eigen::Vector3d* position2) {
    const Eigen::Vector3d planePoint = unproject(point);
    const double offset1
        = face1.normal.dot(face1.point - planePoint) / denominator1;
    const double offset2
        = face2.normal.dot(face2.point - planePoint) / denominator2;
    if (position1)
      *position1 = planePoint + offset1 * outwardNormal;
    if (position2)
      *position2 = planePoint + offset2 * outwardNormal;
    return offset1 - offset2;
  };

  const double numericContactTolerance
      = std::max(1e-9, 64.0 * std::numeric_limits<double>::epsilon() * scale);
  const double contactTolerance
      = proximityGap > 0.0 ? proximityGap : numericContactTolerance;
  overlap = clipPolygonByContactDepth(
      overlap,
      [&](const Eigen::Vector2d& point) {
        return getSurfaceData(point, nullptr, nullptr);
      },
      contactTolerance,
      polygonTolerance);
  if (overlap.size() < 3u)
    return std::nullopt;
  orderConvexPolygon(overlap, polygonTolerance);

  const double patchArea = std::abs(signedPolygonArea(overlap));
  const double faceArea1 = std::abs(signedPolygonArea(polygon1));
  const double faceArea2 = std::abs(signedPolygonArea(polygon2));
  const double areaTolerance = std::max(
      kConvexFacePatchAreaToleranceScale * scale * scale,
      kConvexFacePatchAreaFractionTolerance * std::min(faceArea1, faceArea2));
  if (patchArea <= areaTolerance)
    return std::nullopt;

  ConvexFacePatchCandidate candidate;
  candidate.faceIndex1 = face1.index;
  candidate.faceIndex2 = face2.index;
  candidate.contacts.reserve(overlap.size());
  double depthSum = 0.0;
  for (const auto& point : overlap) {
    Eigen::Vector3d position1;
    Eigen::Vector3d position2;
    const double depth = getSurfaceData(point, &position1, &position2);
    if (proximityGap > 0.0 && !(depth > -proximityGap))
      continue;
    native::ContactPoint contact;
    contact.position = 0.5 * (position1 + position2);
    contact.normal = contactNormal;
    contact.depth = proximityGap > 0.0 ? depth : std::max(0.0, depth);
    candidate.contacts.push_back(contact);
    depthSum += contact.depth;
  }
  if (proximityGap > 0.0 && candidate.contacts.size() < 3u)
    return std::nullopt;
  std::sort(
      candidate.contacts.begin(), candidate.contacts.end(), contactPointLess);

  const double meanDepth
      = depthSum / static_cast<double>(candidate.contacts.size());
  const double compactDepth = proximityGap > 0.0
                                  ? compactContact.depth
                                  : std::max(0.0, compactContact.depth);
  candidate.score
      = std::abs(meanDepth - compactDepth) + (1.0 - normalAlignment) * scale;
  return candidate;
}

//==============================================================================
bool tryBuildFourPointPlanarConvexFacePatch(
    const native::ConvexShape& shape1,
    const Eigen::Isometry3d& transform1,
    const native::ConvexShape& shape2,
    const Eigen::Isometry3d& transform2,
    const native::CollisionOption& option,
    const native::ContactPoint& compactContact,
    double proximityGap,
    native::CollisionResult& result)
{
  const double scale = std::max(
      computeConvexWorldScale(shape1, transform1),
      computeConvexWorldScale(shape2, transform2));
  const auto faces1 = makeConvexWorldFaces(shape1, transform1, scale);
  const auto faces2 = makeConvexWorldFaces(shape2, transform2, scale);

  std::optional<ConvexFacePatchCandidate> best;
  for (const auto& face1 : faces1) {
    for (const auto& face2 : faces2) {
      auto candidate = makeConvexFacePatchCandidate(
          face1, face2, compactContact, scale, proximityGap);
      if (!candidate)
        continue;

      const bool betterScore = !best || candidate->score < best->score;
      const bool deterministicTie
          = best && candidate->score == best->score
            && (candidate->faceIndex1 < best->faceIndex1
                || (candidate->faceIndex1 == best->faceIndex1
                    && candidate->faceIndex2 < best->faceIndex2));
      if (betterScore || deterministicTie)
        best = std::move(candidate);
    }
  }
  if (!best || best->contacts.size() < 3u)
    return false;

  native::ContactManifold reducer;
  for (const auto& contact : best->contacts)
    reducer.addContact(contact);
  std::vector<native::ContactPoint> reduced(
      reducer.getContacts().begin(), reducer.getContacts().end());
  std::sort(reduced.begin(), reduced.end(), contactPointLess);

  const std::size_t numContacts = std::min(
      {reduced.size(), option.maxNumContacts, kFourPointPlanarContactTarget});
  if (numContacts == 0u)
    return false;

  native::ContactManifold manifold;
  manifold.setType(
      numContacts >= 3u ? native::ContactType::Face
                        : (numContacts == 2u ? native::ContactType::Edge
                                             : native::ContactType::Point));
  for (std::size_t i = 0u; i < numContacts; ++i)
    manifold.addContact(reduced[i]);
  result.clear();
  result.addManifold(std::move(manifold));
  return true;
}

//==============================================================================
bool isConvexConvexPair(
    const native::Shape* shape1, const native::Shape* shape2)
{
  return shape1->getType() == native::ShapeType::Convex
         && shape2->getType() == native::ShapeType::Convex;
}

//==============================================================================
bool collideFourPointPlanarConvexConvex(
    const native::Shape* shape1,
    const Eigen::Isometry3d& transform1,
    const native::Shape* shape2,
    const Eigen::Isometry3d& transform2,
    const native::CollisionOption& option,
    native::CollisionResult& result)
{
  const bool hit = native::NarrowPhase::collide(
      shape1, transform1, shape2, transform2, option, result);
  if (!hit || !option.enableContact || result.numContacts() == 0u)
    return hit;

  const native::ContactPoint compactContact = result.getContact(0u);
  (void)tryBuildFourPointPlanarConvexFacePatch(
      static_cast<const native::ConvexShape&>(*shape1),
      transform1,
      static_cast<const native::ConvexShape&>(*shape2),
      transform2,
      option,
      compactContact,
      0.0,
      result);
  return true;
}

//==============================================================================
bool isPlaneBoxPair(const native::Shape* shape1, const native::Shape* shape2)
{
  return (shape1->getType() == native::ShapeType::Plane
          && shape2->getType() == native::ShapeType::Box)
         || (shape1->getType() == native::ShapeType::Box
             && shape2->getType() == native::ShapeType::Plane);
}

//==============================================================================
bool collideFourPointPlanarPlaneBox(
    const native::Shape* shape1,
    const Eigen::Isometry3d& transform1,
    const native::Shape* shape2,
    const Eigen::Isometry3d& transform2,
    const native::CollisionOption& option,
    native::CollisionResult& result)
{
  if (option.maxNumContacts == 0u)
    return false;

  const bool planeFirst = shape1->getType() == native::ShapeType::Plane;
  const auto* plane
      = static_cast<const native::PlaneShape*>(planeFirst ? shape1 : shape2);
  const auto* box
      = static_cast<const native::BoxShape*>(planeFirst ? shape2 : shape1);
  const auto& planeTransform = planeFirst ? transform1 : transform2;
  const auto& boxTransform = planeFirst ? transform2 : transform1;

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane->getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane->getOffset();
  const Eigen::Vector3d& halfExtents = box->getHalfExtents();

  struct Corner
  {
    Eigen::Vector3d position;
    double signedDistance;
  };

  std::array<Corner, 8u> corners;
  double minDistance = std::numeric_limits<double>::max();
  for (std::size_t i = 0u; i < corners.size(); ++i) {
    const Eigen::Vector3d localCorner(
        (i & 1u) ? halfExtents.x() : -halfExtents.x(),
        (i & 2u) ? halfExtents.y() : -halfExtents.y(),
        (i & 4u) ? halfExtents.z() : -halfExtents.z());
    const Eigen::Vector3d worldCorner = boxTransform * localCorner;
    const double signedDistance = worldNormal.dot(worldCorner - planePoint);
    corners[i] = Corner{worldCorner, signedDistance};
    minDistance = std::min(minDistance, signedDistance);
  }

  if (minDistance > 0.0)
    return false;

  if (!option.enableContact)
    return true;

  const double scale = std::max(
      {1.0,
       halfExtents.cwiseAbs().maxCoeff(),
       boxTransform.translation().cwiseAbs().maxCoeff(),
       planePoint.cwiseAbs().maxCoeff(),
       std::abs(minDistance)});
  const double contactTolerance
      = std::max(1e-9, 64.0 * std::numeric_limits<double>::epsilon() * scale);
  const Eigen::Vector3d contactNormal = planeFirst ? -worldNormal : worldNormal;

  std::array<std::size_t, 8u> candidateIndices{};
  std::size_t numCandidates = 0u;
  for (std::size_t i = 0u; i < corners.size(); ++i) {
    if (corners[i].signedDistance <= contactTolerance)
      candidateIndices[numCandidates++] = i;
  }
  std::sort(
      candidateIndices.begin(),
      candidateIndices.begin() + numCandidates,
      [&](std::size_t lhs, std::size_t rhs) {
        if (corners[lhs].signedDistance != corners[rhs].signedDistance)
          return corners[lhs].signedDistance < corners[rhs].signedDistance;
        return lhs < rhs;
      });

  const std::size_t maxContacts = std::min(
      {numCandidates, option.maxNumContacts, kPlaneBoxManifoldContactTarget});
  native::ContactManifold manifold;
  for (std::size_t i = 0u; i < maxContacts; ++i) {
    const Corner& corner = corners[candidateIndices[i]];
    native::ContactPoint contact;
    contact.position = corner.position - worldNormal * corner.signedDistance;
    contact.normal = contactNormal;
    contact.depth = std::max(0.0, -corner.signedDistance);
    manifold.addContact(contact);
  }

  if (manifold.numContacts() >= 3u)
    manifold.setType(native::ContactType::Face);
  else if (manifold.numContacts() == 2u)
    manifold.setType(native::ContactType::Edge);
  else
    manifold.setType(native::ContactType::Point);
  result.addManifold(std::move(manifold));
  return true;
}

//==============================================================================
bool isPlaneConvexPair(const native::Shape* shape1, const native::Shape* shape2)
{
  return (shape1->getType() == native::ShapeType::Plane
          && shape2->getType() == native::ShapeType::Convex)
         || (shape1->getType() == native::ShapeType::Convex
             && shape2->getType() == native::ShapeType::Plane);
}

//==============================================================================
bool collideFourPointPlanarPlaneConvexProximity(
    const native::Shape* shape1,
    const Eigen::Isometry3d& transform1,
    const native::Shape* shape2,
    const Eigen::Isometry3d& transform2,
    double contactGap,
    const native::CollisionOption& option,
    native::CollisionResult& result)
{
  if (!(contactGap > 0.0) || option.maxNumContacts == 0u)
    return false;

  const bool planeFirst = shape1->getType() == native::ShapeType::Plane;
  const auto* plane
      = static_cast<const native::PlaneShape*>(planeFirst ? shape1 : shape2);
  const auto* convex
      = static_cast<const native::ConvexShape*>(planeFirst ? shape2 : shape1);
  const auto& planeTransform = planeFirst ? transform1 : transform2;
  const auto& convexTransform = planeFirst ? transform2 : transform1;

  const Eigen::Vector3d worldNormal
      = planeTransform.rotation() * plane->getNormal();
  const Eigen::Vector3d planePoint
      = planeTransform.translation() + worldNormal * plane->getOffset();

  struct VertexContact
  {
    std::size_t vertexIndex = 0u;
    Eigen::Vector3d vertex = Eigen::Vector3d::Zero();
    double signedDistance = std::numeric_limits<double>::max();
  };

  std::vector<VertexContact> candidates;
  candidates.reserve(convex->getVertices().size());
  for (std::size_t i = 0u; i < convex->getVertices().size(); ++i) {
    const Eigen::Vector3d vertex = convexTransform * convex->getVertices()[i];
    const double signedDistance = worldNormal.dot(vertex - planePoint);
    if (std::isfinite(signedDistance) && signedDistance < contactGap)
      candidates.push_back(VertexContact{i, vertex, signedDistance});
  }
  if (candidates.empty())
    return false;

  if (!option.enableContact)
    return true;

  const Eigen::Vector3d contactNormal = planeFirst ? -worldNormal : worldNormal;
  native::ContactManifold reducer;
  for (const auto& candidate : candidates) {
    native::ContactPoint contact;
    contact.position
        = candidate.vertex - 0.5 * worldNormal * candidate.signedDistance;
    contact.normal = contactNormal;
    contact.depth = -candidate.signedDistance;
    const int featureIndex = static_cast<int>(candidate.vertexIndex);
    if (planeFirst)
      contact.featureIndex2 = featureIndex;
    else
      contact.featureIndex1 = featureIndex;
    reducer.addContact(contact);
  }

  std::vector<native::ContactPoint> reduced(
      reducer.getContacts().begin(), reducer.getContacts().end());
  std::sort(reduced.begin(), reduced.end(), contactPointLess);
  const std::size_t numContacts = std::min(
      {reduced.size(), option.maxNumContacts, kFourPointPlanarContactTarget});
  if (numContacts == 0u)
    return false;

  native::ContactManifold manifold;
  manifold.setType(
      numContacts >= 3u ? native::ContactType::Face
                        : (numContacts == 2u ? native::ContactType::Edge
                                             : native::ContactType::Point));
  for (std::size_t i = 0u; i < numContacts; ++i)
    manifold.addContact(reduced[i]);
  result.addManifold(std::move(manifold));
  return true;
}

//==============================================================================
bool collideWithContactGap(
    const native::Shape* shape1,
    const Eigen::Isometry3d& transform1,
    const native::Shape* shape2,
    const Eigen::Isometry3d& transform2,
    double contactGap,
    DARTCollisionDetector::ContactManifoldMode manifoldMode,
    const native::CollisionOption& option,
    native::CollisionResult& result)
{
  if (!(contactGap > 0.0))
    return false;

  if (manifoldMode
          == DARTCollisionDetector::ContactManifoldMode::FourPointPlanar
      && isPlaneConvexPair(shape1, shape2)) {
    return collideFourPointPlanarPlaneConvexProximity(
        shape1, transform1, shape2, transform2, contactGap, option, result);
  }

  native::DistanceOption distanceOption;
  distanceOption.upperBound = contactGap;
  distanceOption.enableNearestPoints = option.enableContact;
  native::DistanceResult distanceResult;
  const double distance = native::NarrowPhase::distance(
      shape1, transform1, shape2, transform2, distanceOption, distanceResult);
  if (!std::isfinite(distance) || !(distance < contactGap))
    return false;

  if (!option.enableContact)
    return true;

  if (!distanceResult.pointOnObject1.allFinite()
      || !distanceResult.pointOnObject2.allFinite()
      || !distanceResult.normal.allFinite()
      || native::ContactPoint::isZeroNormal(distanceResult.normal)) {
    return false;
  }

  native::ContactPoint compactContact;
  compactContact.position
      = 0.5 * (distanceResult.pointOnObject1 + distanceResult.pointOnObject2);
  compactContact.normal = -distanceResult.normal.normalized();
  compactContact.depth = -distance;
  result.addContact(compactContact);

  if (manifoldMode
          == DARTCollisionDetector::ContactManifoldMode::FourPointPlanar
      && isConvexConvexPair(shape1, shape2)) {
    (void)tryBuildFourPointPlanarConvexFacePatch(
        static_cast<const native::ConvexShape&>(*shape1),
        transform1,
        static_cast<const native::ConvexShape&>(*shape2),
        transform2,
        option,
        compactContact,
        contactGap,
        result);
  }
  return true;
}

//==============================================================================
bool emitSoftContacts(
    const CollisionResult& softResult,
    const CollisionOption& option,
    CollisionResult& result)
{
  const std::size_t maxPairContacts
      = option.getEffectiveMaxNumContactsPerPair();
  std::size_t emittedForPair = 0u;

  const std::size_t numContacts = softResult.getNumContacts();
  for (std::size_t i = 0u; i < numContacts; ++i) {
    if (result.getNumContacts() >= option.maxNumContacts)
      return true;

    if (emittedForPair >= maxPairContacts)
      return false;

    const auto& contact = softResult.getContact(i);
    if (contact.penetrationDepth < 0.0
        && !option.allowNegativePenetrationDepthContacts) {
      continue;
    }

    if (Contact::isZeroNormal(contact.normal))
      continue;

    result.addContact(contact);
    ++emittedForPair;
  }

  return result.getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
bool processRigidNativePairUnchecked(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const CollisionOption& option,
    DARTCollisionDetector::ContactManifoldMode manifoldMode,
    const ContactGapMap* contactGaps,
    CollisionResult* result,
    bool& collisionFound,
    native::CollisionResult& nativeResult)
{
  if (result && result->getNumContacts() >= option.maxNumContacts)
    return true;

  // Reused across pairs by the caller so steady-state stepping stays
  // allocation-free (StepAllocation gate discipline).
  nativeResult.clear();
  const native::CollisionOption nativeOption
      = makeNativeOption(option, result, manifoldMode);
  const auto* shape1 = detail::DARTCollisionObjectAccessor::getShape(object1);
  const auto* shape2 = detail::DARTCollisionObjectAccessor::getShape(object2);
  const auto& transform1
      = detail::DARTCollisionObjectAccessor::getTransform(object1);
  const auto& transform2
      = detail::DARTCollisionObjectAccessor::getTransform(object2);
  const bool useFourPointPlaneBox
      = manifoldMode
            == DARTCollisionDetector::ContactManifoldMode::FourPointPlanar
        && isPlaneBoxPair(shape1, shape2);
  const bool useFourPointConvexFaces
      = manifoldMode
            == DARTCollisionDetector::ContactManifoldMode::FourPointPlanar
        && isConvexConvexPair(shape1, shape2);
  const bool useFourPointNativePatches
      = manifoldMode
        == DARTCollisionDetector::ContactManifoldMode::FourPointPlanar;
  bool hit
      = useFourPointPlaneBox ? collideFourPointPlanarPlaneBox(
            shape1, transform1, shape2, transform2, nativeOption, nativeResult)
        : useFourPointConvexFaces ? collideFourPointPlanarConvexConvex(
              shape1,
              transform1,
              shape2,
              transform2,
              nativeOption,
              nativeResult)
        : useFourPointNativePatches
            ? native::detail::collideWithFourPointPlanarPatches(
                shape1,
                transform1,
                shape2,
                transform2,
                nativeOption,
                nativeResult)
            : native::NarrowPhase::collide(
                shape1,
                transform1,
                shape2,
                transform2,
                nativeOption,
                nativeResult);

  if (!hit && option.allowNegativePenetrationDepthContacts) {
    hit = collideWithContactGap(
        shape1,
        transform1,
        shape2,
        transform2,
        summedContactGap(contactGaps, object1, object2),
        manifoldMode,
        nativeOption,
        nativeResult);
  }

  if (!hit)
    return false;

  collisionFound = true;

  if (!result)
    return true;

  if (!option.enableContact) {
    addPairOnlyContact(object1, object2, *result);
    return result->getNumContacts() >= option.maxNumContacts;
  }

  return emitContacts(nativeResult, object1, object2, option, *result);
}

//==============================================================================
bool processNativePairUnchecked(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const CollisionOption& option,
    DARTCollisionDetector::ContactManifoldMode manifoldMode,
    const ContactGapMap* contactGaps,
    bool enableSoftFaceInteriorContacts,
    CollisionResult* result,
    bool& collisionFound,
    native::CollisionResult& nativeResult,
    CollisionResult& softPairScratch)
{
  if (result && result->getNumContacts() >= option.maxNumContacts)
    return true;

  if (!detail::isSoftCollisionPair(object1, object2)) {
    return processRigidNativePairUnchecked(
        object1,
        object2,
        option,
        manifoldMode,
        contactGaps,
        result,
        collisionFound,
        nativeResult);
  }

  softPairScratch.clear();
  detail::collideSoftPair(
      object1, object2, enableSoftFaceInteriorContacts, softPairScratch);
  if (softPairScratch.getNumContacts() == 0u)
    return false;

  collisionFound = true;

  if (!result) {
    softPairScratch.clear();
    return true;
  }

  if (!option.enableContact) {
    addPairOnlyContact(object1, object2, *result);
    softPairScratch.clear();
    return result->getNumContacts() >= option.maxNumContacts;
  }

  const bool shouldStop = emitSoftContacts(softPairScratch, option, *result);
  softPairScratch.clear();
  return shouldStop;
}

//==============================================================================
bool processNativePair(
    DARTCollisionObject* object1,
    DARTCollisionObject* object2,
    const CollisionOption& option,
    DARTCollisionDetector::ContactManifoldMode manifoldMode,
    const ContactGapMap* contactGaps,
    bool enableSoftFaceInteriorContacts,
    CollisionResult* result,
    bool& collisionFound,
    native::CollisionResult& nativeResult,
    CollisionResult& softPairScratch)
{
  if (shouldSkipPair(object1, object2, option))
    return false;

  return processNativePairUnchecked(
      object1,
      object2,
      option,
      manifoldMode,
      contactGaps,
      enableSoftFaceInteriorContacts,
      result,
      collisionFound,
      nativeResult,
      softPairScratch);
}

//==============================================================================
bool objectAabbsOverlap(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
    const ContactGapMap* contactGaps)
{
  const auto& aabb1 = detail::DARTCollisionObjectAccessor::getAabb(object1);
  const auto& aabb2 = detail::DARTCollisionObjectAccessor::getAabb(object2);
  if (!contactGaps)
    return aabb1.overlaps(aabb2);

  native::Aabb expanded1 = aabb1;
  native::Aabb expanded2 = aabb2;
  expanded1.expand(lookupContactGap(contactGaps, object1->getShapeFrame()));
  expanded2.expand(lookupContactGap(contactGaps, object2->getShapeFrame()));
  return expanded1.overlaps(expanded2);
}

//==============================================================================
bool processNativePairsInParallel(
    const std::vector<ParallelObjectPair>& pairs,
    const CollisionOption& option,
    DARTCollisionDetector::ContactManifoldMode manifoldMode,
    const ContactGapMap* contactGaps,
    bool enableSoftFaceInteriorContacts,
    CollisionResult& result,
    bool& collisionFound,
    DetectorEngineState& engineState,
    CollisionThreadPool& threadPool,
    std::size_t numCollisionThreads)
{
  DART_ASSERT(pairs.size() <= kParallelPairBatchSize);
  auto& pairResults = engineState.parallelPairResults;
  if (pairResults.size() < kParallelPairBatchSize)
    pairResults.resize(kParallelPairBatchSize);

  // CollisionFilter has no thread-safety contract, and
  // BodyNodeCollisionFilter intentionally carries solver state in
  // thread-local storage. Soft collisions also lazily refresh per-object face
  // caches. Preserve both operations on the submitting thread, then dispatch
  // only immutable rigid narrowphase work.
  for (std::size_t i = 0u; i < pairs.size(); ++i) {
    auto& pairResult = pairResults[i];
    pairResult.narrowphaseScratch.clear();
    pairResult.result.clear();
    pairResult.collisionFound = false;
    pairResult.eligible
        = pairs[i].eligibilityChecked
          || !shouldSkipPair(pairs[i].first, pairs[i].second, option);
    pairResult.dispatchToWorker
        = pairResult.eligible
          && !detail::isSoftCollisionPair(pairs[i].first, pairs[i].second);
  }

  const bool profileRecording
      = dart::common::profile::isProfileRecordingEnabled();
  auto processPairAt = [&](std::size_t index) {
    auto& pairResult = pairResults[index];
    if (!pairResult.dispatchToWorker)
      return;

    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "DARTCollisionDetector::rigidPair");
    processRigidNativePairUnchecked(
        pairs[index].first,
        pairs[index].second,
        option,
        manifoldMode,
        contactGaps,
        &pairResult.result,
        pairResult.collisionFound,
        pairResult.narrowphaseScratch);
  };
  threadPool.parallelFor(pairs.size(), numCollisionThreads, processPairAt);

  for (std::size_t i = 0u; i < pairs.size(); ++i) {
    auto& pairResult = pairResults[i];
    if (pairResult.eligible && !pairResult.dispatchToWorker) {
      if (processNativePairUnchecked(
              pairs[i].first,
              pairs[i].second,
              option,
              manifoldMode,
              contactGaps,
              enableSoftFaceInteriorContacts,
              &result,
              collisionFound,
              engineState.narrowphaseScratch,
              engineState.softPairScratch)) {
        return true;
      }
      continue;
    }

    collisionFound = collisionFound || pairResult.collisionFound;
    for (const auto& contact : pairResult.result.getContacts()) {
      if (result.getNumContacts() >= option.maxNumContacts)
        return true;

      result.addContact(contact);
    }
  }

  return result.getNumContacts() >= option.maxNumContacts;
}

} // namespace

//==============================================================================
// Keep all detector-specific runtime state behind CollisionDetector's existing
// manager pointer so the released DARTCollisionDetector object layout remains
// unchanged.
class DARTCollisionDetector::DARTCollisionObjectManager final
  : public CollisionDetector::CollisionObjectManager
{
public:
  explicit DARTCollisionObjectManager(DARTCollisionDetector* detector)
    : CollisionObjectManager(detector), mObjectManager(detector)
  {
  }

  std::shared_ptr<CollisionObject> claimCollisionObject(
      const dynamics::ShapeFrame* shapeFrame) override
  {
    return mObjectManager.claimCollisionObject(shapeFrame);
  }

  void setNumCollisionThreads(std::size_t numThreads)
  {
    std::lock_guard<std::mutex> lock(mEngineState.threadConfigMutex);
    mEngineState.collisionThreadPool.setWorkerCount(
        numThreads > 1u ? numThreads - 1u : 0u);
    mEngineState.numCollisionThreads.store(numThreads);
  }

  std::size_t getNumCollisionThreads() const
  {
    return mEngineState.numCollisionThreads.load();
  }

  void setSoftFaceInteriorContactsEnabled(bool enabled)
  {
    mEngineState.softFaceInteriorContactsEnabled.store(enabled);
  }

  bool getSoftFaceInteriorContactsEnabled() const
  {
    return mEngineState.softFaceInteriorContactsEnabled.load();
  }

  void setContactManifoldMode(DARTCollisionDetector::ContactManifoldMode mode)
  {
    mEngineState.contactManifoldMode.store(mode);
  }

  DARTCollisionDetector::ContactManifoldMode getContactManifoldMode() const
  {
    return mEngineState.contactManifoldMode.load();
  }

  void setContactGap(const dynamics::ShapeFrame* shapeFrame, double contactGap)
  {
    std::shared_ptr<ContactGapOverrides> overrides;
    {
      std::lock_guard<std::mutex> lock(mEngineState.contactPolicyMutex);
      if (contactGap == 0.0 && !mEngineState.contactGapOverrides)
        return;
      if (!mEngineState.contactGapOverrides)
        mEngineState.contactGapOverrides
            = std::make_shared<ContactGapOverrides>();
      overrides = mEngineState.contactGapOverrides;
    }
    overrides->set(shapeFrame, contactGap);
  }

  double getContactGap(const dynamics::ShapeFrame* shapeFrame) const
  {
    std::shared_ptr<ContactGapOverrides> overrides;
    {
      std::lock_guard<std::mutex> lock(mEngineState.contactPolicyMutex);
      overrides = mEngineState.contactGapOverrides;
    }
    return overrides ? overrides->get(shapeFrame) : 0.0;
  }

  std::shared_ptr<const ContactGapMap> getContactGaps() const
  {
    std::shared_ptr<ContactGapOverrides> overrides;
    {
      std::lock_guard<std::mutex> lock(mEngineState.contactPolicyMutex);
      overrides = mEngineState.contactGapOverrides;
    }
    return overrides ? overrides->snapshot() : nullptr;
  }

  ContactGapPolicySnapshot getContactGapPolicy() const
  {
    std::shared_ptr<ContactGapOverrides> overrides;
    {
      std::lock_guard<std::mutex> lock(mEngineState.contactPolicyMutex);
      overrides = mEngineState.contactGapOverrides;
    }
    return overrides ? overrides->snapshotWithRevision()
                     : ContactGapPolicySnapshot{};
  }

  void clearContactGaps()
  {
    std::shared_ptr<ContactGapOverrides> overrides;
    {
      std::lock_guard<std::mutex> lock(mEngineState.contactPolicyMutex);
      overrides = mEngineState.contactGapOverrides;
    }
    if (overrides)
      overrides->clear();
  }

  DetectorEngineState& getEngineState()
  {
    return mEngineState;
  }

  const DetectorEngineState& getEngineState() const
  {
    return mEngineState;
  }

  void createCollisionGroupEngineData(const DARTCollisionGroup* group)
  {
    mGroupEngineData[group]
        = std::make_unique<detail::DARTCollisionGroupEngineData>();
  }

  void removeCollisionGroupEngineData(const DARTCollisionGroup* group)
  {
    mGroupEngineData.erase(group);
  }

  detail::DARTCollisionGroupEngineData& getCollisionGroupEngineData(
      const DARTCollisionGroup* group)
  {
    return *mGroupEngineData.at(group);
  }

  const detail::DARTCollisionGroupEngineData& getCollisionGroupEngineData(
      const DARTCollisionGroup* group) const
  {
    return *mGroupEngineData.at(group);
  }

  std::size_t getNumCollisionGroupEngineData() const
  {
    return mGroupEngineData.size();
  }

  void createCollisionObjectEngineData(const DARTCollisionObject* object)
  {
    mObjectEngineData[object]
        = std::make_unique<detail::DARTCollisionObjectEngineData>();
  }

  void removeCollisionObjectEngineData(const DARTCollisionObject* object)
  {
    mObjectEngineData.erase(object);
  }

  detail::DARTCollisionObjectEngineData& getCollisionObjectEngineData(
      const DARTCollisionObject* object)
  {
    return *mObjectEngineData.at(object);
  }

  const detail::DARTCollisionObjectEngineData& getCollisionObjectEngineData(
      const DARTCollisionObject* object) const
  {
    return *mObjectEngineData.at(object);
  }

  std::size_t getNumCollisionObjectEngineData() const
  {
    return mObjectEngineData.size();
  }

private:
  ManagerForSharableCollisionObjects mObjectManager;
  DetectorEngineState mEngineState;
  std::unordered_map<
      const DARTCollisionGroup*,
      std::unique_ptr<detail::DARTCollisionGroupEngineData>>
      mGroupEngineData;
  std::unordered_map<
      const DARTCollisionObject*,
      std::unique_ptr<detail::DARTCollisionObjectEngineData>>
      mObjectEngineData;
};

//==============================================================================
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mRegistrar{
        DARTCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<DARTCollisionDetector> {
          return DARTCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
void DARTCollisionDetector::setContactManifoldMode(ContactManifoldMode mode)
{
  auto* manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (manager)
    manager->setContactManifoldMode(mode);
}

//==============================================================================
DARTCollisionDetector::ContactManifoldMode
DARTCollisionDetector::getContactManifoldMode() const
{
  const auto* manager = dynamic_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager ? manager->getContactManifoldMode()
                 : ContactManifoldMode::Compact;
}

//==============================================================================
void DARTCollisionDetector::setContactGap(
    const dynamics::ShapeFrame* shapeFrame, double contactGap)
{
  if (!shapeFrame) {
    throw std::invalid_argument(
        "DARTCollisionDetector contact-gap ShapeFrame must not be null");
  }
  if (!std::isfinite(contactGap) || contactGap < 0.0
      || contactGap > 0.5 * std::numeric_limits<double>::max()) {
    throw std::invalid_argument(
        "DARTCollisionDetector contact gap must be finite, non-negative, and "
        "pairwise summable");
  }

  auto* manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (manager)
    manager->setContactGap(shapeFrame, contactGap);
}

//==============================================================================
double DARTCollisionDetector::getContactGap(
    const dynamics::ShapeFrame* shapeFrame) const
{
  if (!shapeFrame)
    return 0.0;

  const auto* manager = dynamic_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager ? manager->getContactGap(shapeFrame) : 0.0;
}

//==============================================================================
void DARTCollisionDetector::clearContactGaps()
{
  auto* manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (manager)
    manager->clearContactGaps();
}

//==============================================================================
CollisionDetectorPtr DARTCollisionDetector::attachCollisionGroupEngineData(
    const DARTCollisionGroup* group,
    const CollisionDetectorPtr& collisionDetector)
{
  // The alias points at the detector while its independent control block owns
  // both the original detector reference and the group sidecar lifetime.
  auto lifetime = std::shared_ptr<const DARTCollisionGroup>(
      group, [collisionDetector](const DARTCollisionGroup* groupToRemove) {
        auto* detector
            = static_cast<DARTCollisionDetector*>(collisionDetector.get());
        detector->removeCollisionGroupEngineData(groupToRemove);
      });

  auto* manager
      = static_cast<DARTCollisionObjectManager*>(mCollisionObjectManager.get());
  manager->createCollisionGroupEngineData(group);
  return CollisionDetectorPtr(std::move(lifetime), collisionDetector.get());
}

//==============================================================================
void DARTCollisionDetector::removeCollisionGroupEngineData(
    const DARTCollisionGroup* group)
{
  auto* manager
      = static_cast<DARTCollisionObjectManager*>(mCollisionObjectManager.get());
  manager->removeCollisionGroupEngineData(group);
}

//==============================================================================
detail::DARTCollisionGroupEngineData&
DARTCollisionDetector::getCollisionGroupEngineData(
    const DARTCollisionGroup* group)
{
  auto* manager
      = static_cast<DARTCollisionObjectManager*>(mCollisionObjectManager.get());
  return manager->getCollisionGroupEngineData(group);
}

//==============================================================================
dynamics::ConstShapePtr DARTCollisionDetector::attachCollisionObjectEngineData(
    const DARTCollisionObject* object, const dynamics::ConstShapePtr& shape)
{
  // The alias preserves normal cached-shape pointer semantics while its
  // independent control block owns the object sidecar lifetime.
  auto lifetime = std::shared_ptr<const DARTCollisionObject>(
      object, [this, shape](const DARTCollisionObject* objectToRemove) {
        static_cast<void>(shape);
        removeCollisionObjectEngineData(objectToRemove);
      });

  auto* manager
      = static_cast<DARTCollisionObjectManager*>(mCollisionObjectManager.get());
  manager->createCollisionObjectEngineData(object);
  return dynamics::ConstShapePtr(std::move(lifetime), shape.get());
}

//==============================================================================
void DARTCollisionDetector::removeCollisionObjectEngineData(
    const DARTCollisionObject* object)
{
  auto* manager
      = static_cast<DARTCollisionObjectManager*>(mCollisionObjectManager.get());
  manager->removeCollisionObjectEngineData(object);
}

//==============================================================================
detail::DARTCollisionObjectEngineData&
DARTCollisionDetector::getCollisionObjectEngineData(
    const DARTCollisionObject* object)
{
  auto* manager
      = static_cast<DARTCollisionObjectManager*>(mCollisionObjectManager.get());
  return manager->getCollisionObjectEngineData(object);
}

//==============================================================================
const detail::DARTCollisionObjectEngineData&
DARTCollisionDetector::getCollisionObjectEngineData(
    const DARTCollisionObject* object) const
{
  const auto* manager = static_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager->getCollisionObjectEngineData(object);
}

//==============================================================================
std::size_t DARTCollisionDetector::getNumCollisionGroupEngineData() const
{
  const auto* manager = static_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager->getNumCollisionGroupEngineData();
}

//==============================================================================
std::size_t DARTCollisionDetector::getNumCollisionObjectEngineData() const
{
  const auto* manager = static_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager->getNumCollisionObjectEngineData();
}

//==============================================================================
std::size_t
detail::DARTCollisionDetectorAccessor::getNumCollisionGroupEngineData(
    const DARTCollisionDetector& detector)
{
  return detector.getNumCollisionGroupEngineData();
}

//==============================================================================
std::size_t
detail::DARTCollisionDetectorAccessor::getNumCollisionObjectEngineData(
    const DARTCollisionDetector& detector)
{
  return detector.getNumCollisionObjectEngineData();
}

//==============================================================================
native::BroadPhaseDebugSnapshot
detail::DARTCollisionDetectorAccessor::getCollisionGroupBroadPhaseSnapshot(
    const DARTCollisionDetector& detector, const DARTCollisionGroup& group)
{
  const auto* manager
      = static_cast<const DARTCollisionDetector::DARTCollisionObjectManager*>(
          detector.mCollisionObjectManager.get());
  native::BroadPhaseDebugSnapshot snapshot;
  manager->getCollisionGroupEngineData(&group).broadPhase.buildDebugSnapshot(
      snapshot);
  return snapshot;
}

//==============================================================================
native::CachedContact* DARTCollisionDetector::getCachedContact(
    const DARTCollisionObject* object1,
    const DARTCollisionObject* object2,
    void* userData) const
{
  if (!object1 || !object2 || !userData)
    return nullptr;

  if (object1->getCollisionDetector() != this
      || object2->getCollisionDetector() != this) {
    return nullptr;
  }

  const auto* manager = dynamic_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (!manager)
    return nullptr;
  const auto& manifoldCache = manager->getEngineState().manifoldCache;

  const auto id1 = getManifoldCacheId(object1);
  const auto id2 = getManifoldCacheId(object2);
  if (!manifoldCache.ownsContact(id1, id2, userData))
    return nullptr;

  return static_cast<native::CachedContact*>(userData);
}

//==============================================================================
void DARTCollisionDetector::notifyCollisionObjectDestroying(
    CollisionObject* object)
{
  CollisionDetector::notifyCollisionObjectDestroying(object);

  if (!object)
    return;

  auto* manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (!manager)
    return;

  manager->getEngineState().manifoldCache.removeObject(
      getManifoldCacheId(object));
}

//==============================================================================
std::shared_ptr<CollisionDetector>
DARTCollisionDetector::cloneWithoutCollisionObjects() const
{
  auto clone = DARTCollisionDetector::create();
  clone->setNumCollisionThreads(getNumCollisionThreads());
  clone->setSoftFaceInteriorContactsEnabled(
      getSoftFaceInteriorContactsEnabled());
  clone->setContactManifoldMode(getContactManifoldMode());
  const auto* manager = dynamic_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  const auto contactGaps = manager ? manager->getContactGaps() : nullptr;
  if (contactGaps) {
    for (const auto& entry : *contactGaps)
      clone->setContactGap(entry.first, entry.second);
  }
  return clone;
}

//==============================================================================
const std::string& DARTCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DARTCollisionDetector::getStaticType()
{
  static const std::string type = "dart";
  return type;
}

//==============================================================================
void DARTCollisionDetector::setNumCollisionThreads(std::size_t numThreads)
{
  if (numThreads == 0u) {
    numThreads = std::thread::hardware_concurrency();
    if (numThreads == 0u)
      numThreads = 1u;
  }

  auto* manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (manager)
    manager->setNumCollisionThreads(std::max<std::size_t>(1u, numThreads));
}

//==============================================================================
std::size_t DARTCollisionDetector::getNumCollisionThreads() const
{
  const auto* manager = dynamic_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager ? manager->getNumCollisionThreads() : 1u;
}

//==============================================================================
void DARTCollisionDetector::setSoftFaceInteriorContactsEnabled(bool enabled)
{
  auto* manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (manager)
    manager->setSoftFaceInteriorContactsEnabled(enabled);
}

//==============================================================================
bool DARTCollisionDetector::getSoftFaceInteriorContactsEnabled() const
{
  const auto* manager = dynamic_cast<const DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  return manager && manager->getSoftFaceInteriorContactsEnabled();
}

//==============================================================================
std::unique_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto* const manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (!manager)
    return false;
  auto& engineState = manager->getEngineState();
  auto& collisionThreadPool = engineState.collisionThreadPool;
  const std::size_t numCollisionThreads
      = engineState.numCollisionThreads.load();
  const bool enableSoftFaceInteriorContacts
      = engineState.softFaceInteriorContactsEnabled.load();
  const auto manifoldMode = engineState.contactManifoldMode.load();
  const auto contactGapPolicy = manager->getContactGapPolicy();
  const auto& contactGaps = contactGapPolicy.contactGaps;

  const bool profileRecording
      = dart::common::profile::isProfileRecordingEnabled();
  auto* nativeGroup = static_cast<DARTCollisionGroup*>(group);
  auto& groupEngineData = nativeGroup->getEngineData();
  {
    DART_PROFILE_SCOPED_IF_N(profileRecording, "Native::updateEngineData");
    nativeGroup->updateEngineData();
    if (groupEngineData.contactGapPolicyRevision != contactGapPolicy.revision) {
      // A decreased or cleared gap must also shrink the tree's retained fat
      // and internal bounds. Rebuild once per policy revision; unchanged
      // policies retain the steady-state incremental update path below.
      groupEngineData.broadPhase.clear();
      for (const auto& entry : groupEngineData.idToObject) {
        native::Aabb expanded
            = detail::DARTCollisionObjectAccessor::getAabb(entry.second);
        expanded.expand(
            lookupContactGap(contactGaps.get(), entry.second->getShapeFrame()));
        groupEngineData.broadPhase.add(entry.first, expanded);
      }
      groupEngineData.contactGapPolicyRevision = contactGapPolicy.revision;
    } else if (contactGaps) {
      for (const auto& entry : groupEngineData.idToObject) {
        native::Aabb expanded
            = detail::DARTCollisionObjectAccessor::getAabb(entry.second);
        expanded.expand(
            lookupContactGap(contactGaps.get(), entry.second->getShapeFrame()));
        groupEngineData.broadPhase.update(entry.first, expanded);
      }
    }
  }
  {
    DART_PROFILE_SCOPED_IF_N(profileRecording, "Native::refreshManifoldCache");
    refreshManifoldCache(nativeGroup->mCollisionObjects, engineState);
  }

  native::CollisionResult& scratchResult = engineState.narrowphaseScratch;
  CollisionResult& softPairScratch = engineState.softPairScratch;
  bool collisionFound = false;
  {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "Native::visitPairs+narrowphase");
    if (result && numCollisionThreads > 1u) {
      auto& objectPairs = engineState.parallelObjectPairsScratch;
      objectPairs.clear();
      if (objectPairs.capacity() < kParallelPairBatchSize)
        objectPairs.reserve(kParallelPairBatchSize);

      struct ParallelPairVisitContext
      {
        detail::DARTCollisionGroupEngineData* groupEngineData;
        std::vector<ParallelObjectPair>* pairs;
        const CollisionOption* option;
        DARTCollisionDetector::ContactManifoldMode manifoldMode;
        const ContactGapMap* contactGaps;
        bool enableSoftFaceInteriorContacts;
        CollisionResult* result;
        bool* collisionFound;
        DetectorEngineState* engineState;
        CollisionThreadPool* threadPool;
        std::size_t numCollisionThreads;
        bool shouldStop{false};
      } context{
          &groupEngineData,
          &objectPairs,
          &option,
          manifoldMode,
          contactGaps.get(),
          enableSoftFaceInteriorContacts,
          result,
          &collisionFound,
          &engineState,
          &collisionThreadPool,
          numCollisionThreads};

      const auto pairVisitor = [&context](std::size_t id1, std::size_t id2) {
        context.pairs->emplace_back(
            context.groupEngineData->idToObject.at(id1),
            context.groupEngineData->idToObject.at(id2));
        if (context.pairs->size() < kParallelPairBatchSize)
          return true;

        context.shouldStop = processNativePairsInParallel(
            *context.pairs,
            *context.option,
            context.manifoldMode,
            context.contactGaps,
            context.enableSoftFaceInteriorContacts,
            *context.result,
            *context.collisionFound,
            *context.engineState,
            *context.threadPool,
            context.numCollisionThreads);
        context.pairs->clear();
        return !context.shouldStop;
      };
      const bool visitedAll
          = groupEngineData.broadPhase.visitPairs(pairVisitor);
      if (visitedAll && !objectPairs.empty()) {
        processNativePairsInParallel(
            objectPairs,
            option,
            manifoldMode,
            contactGaps.get(),
            enableSoftFaceInteriorContacts,
            *result,
            collisionFound,
            engineState,
            collisionThreadPool,
            numCollisionThreads);
        objectPairs.clear();
      }
    } else {
      // Bundle the visitor state behind one pointer so the std::function
      // conversion stays within its small-buffer optimization: a multi-capture
      // lambda heap-allocates on every collide and trips the StepAllocation
      // gates.
      struct PairVisitContext
      {
        detail::DARTCollisionGroupEngineData* groupEngineData;
        const CollisionOption* option;
        DARTCollisionDetector::ContactManifoldMode manifoldMode;
        const ContactGapMap* contactGaps;
        bool enableSoftFaceInteriorContacts;
        CollisionResult* result;
        bool* collisionFound;
        native::CollisionResult* scratchResult;
        CollisionResult* softPairScratch;
      } context{
          &groupEngineData,
          &option,
          manifoldMode,
          contactGaps.get(),
          enableSoftFaceInteriorContacts,
          result,
          &collisionFound,
          &scratchResult,
          &softPairScratch};
      const auto pairVisitor = [&context](std::size_t id1, std::size_t id2) {
        auto* object1 = context.groupEngineData->idToObject.at(id1);
        auto* object2 = context.groupEngineData->idToObject.at(id2);
        return !processNativePair(
            object1,
            object2,
            *context.option,
            context.manifoldMode,
            context.contactGaps,
            context.enableSoftFaceInteriorContacts,
            context.result,
            *context.collisionFound,
            *context.scratchResult,
            *context.softPairScratch);
      };
      if (result) {
        // Result-carrying queries need the sorted, deduplicated visitation
        // order: which contacts survive a maxNumContacts cap is part of the
        // observable behavior, and the ordered walk reproduces BruteForce
        // bit-for-bit.
        groupEngineData.broadPhase.visitPairs(pairVisitor);
      } else {
        // Boolean existence queries return no per-pair data, so traversal
        // order cannot leak; stream the tree walk and stop at the first hit.
        groupEngineData.broadPhase.visitPairsAnyOrder(pairVisitor);
      }
    }
  }

  if (option.enableContact) {
    DART_PROFILE_SCOPED_IF_N(profileRecording, "Native::attachCachedImpulses");
    attachCachedContactImpulses(result, engineState);
  }

  return collisionFound;
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (option.maxNumContacts == 0u)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  if (group1 == group2)
    return collide(group1, option, result);

  auto* const manager = dynamic_cast<DARTCollisionObjectManager*>(
      mCollisionObjectManager.get());
  if (!manager)
    return false;
  auto& engineState = manager->getEngineState();
  auto& collisionThreadPool = engineState.collisionThreadPool;
  const std::size_t numCollisionThreads
      = engineState.numCollisionThreads.load();
  const bool enableSoftFaceInteriorContacts
      = engineState.softFaceInteriorContactsEnabled.load();
  const auto manifoldMode = engineState.contactManifoldMode.load();
  const auto contactGaps = manager->getContactGaps();

  auto* nativeGroup1 = static_cast<DARTCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<DARTCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();
  refreshManifoldCache(
      nativeGroup1->mCollisionObjects,
      nativeGroup2->mCollisionObjects,
      engineState);

  native::CollisionResult& scratchResult = engineState.narrowphaseScratch;
  CollisionResult& softPairScratch = engineState.softPairScratch;
  bool collisionFound = false;
  if (result && numCollisionThreads > 1u) {
    auto& objectPairs = engineState.parallelObjectPairsScratch;
    objectPairs.clear();
    if (objectPairs.capacity() < kParallelPairBatchSize)
      objectPairs.reserve(kParallelPairBatchSize);

    bool shouldStop = false;
    for (auto* object1 : nativeGroup1->mCollisionObjects) {
      for (auto* object2 : nativeGroup2->mCollisionObjects) {
        auto* dartObject1 = static_cast<DARTCollisionObject*>(object1);
        auto* dartObject2 = static_cast<DARTCollisionObject*>(object2);
        if (!objectAabbsOverlap(dartObject1, dartObject2, contactGaps.get()))
          continue;

        bool eligibilityChecked = false;
        if (option.collisionFilter) {
          if (shouldSkipPair(dartObject1, dartObject2, option))
            continue;
          eligibilityChecked = true;
        }

        objectPairs.emplace_back(dartObject1, dartObject2, eligibilityChecked);
        if (objectPairs.size() < kParallelPairBatchSize)
          continue;

        shouldStop = processNativePairsInParallel(
            objectPairs,
            option,
            manifoldMode,
            contactGaps.get(),
            enableSoftFaceInteriorContacts,
            *result,
            collisionFound,
            engineState,
            collisionThreadPool,
            numCollisionThreads);
        objectPairs.clear();
        if (shouldStop)
          break;
      }
      if (shouldStop)
        break;
    }

    if (!shouldStop && !objectPairs.empty()) {
      processNativePairsInParallel(
          objectPairs,
          option,
          manifoldMode,
          contactGaps.get(),
          enableSoftFaceInteriorContacts,
          *result,
          collisionFound,
          engineState,
          collisionThreadPool,
          numCollisionThreads);
      objectPairs.clear();
    }
    if (option.enableContact)
      attachCachedContactImpulses(result, engineState);
    return collisionFound;
  }

  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      auto* dartObject1 = static_cast<DARTCollisionObject*>(object1);
      auto* dartObject2 = static_cast<DARTCollisionObject*>(object2);
      if (!objectAabbsOverlap(dartObject1, dartObject2, contactGaps.get()))
        continue;

      if (processNativePair(
              dartObject1,
              dartObject2,
              option,
              manifoldMode,
              contactGaps.get(),
              enableSoftFaceInteriorContacts,
              result,
              collisionFound,
              scratchResult,
              softPairScratch)) {
        if (option.enableContact)
          attachCachedContactImpulses(result, engineState);
        return collisionFound;
      }
    }
  }

  if (option.enableContact)
    attachCachedContactImpulses(result, engineState);

  return collisionFound;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* group, const DistanceOption& option, DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return 0.0;

  auto* nativeGroup = static_cast<DARTCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  NativeDistanceCandidate best;
  for (std::size_t i = 0u; i < nativeGroup->mCollisionObjects.size(); ++i) {
    for (std::size_t j = i + 1u; j < nativeGroup->mCollisionObjects.size();
         ++j) {
      const auto* object1 = static_cast<const DARTCollisionObject*>(
          nativeGroup->mCollisionObjects[i]);
      const auto* object2 = static_cast<const DARTCollisionObject*>(
          nativeGroup->mCollisionObjects[j]);

      if (processNativeDistancePair(object1, object2, option, best))
        return convertDistanceResult(best, option, result);
    }
  }

  return convertDistanceResult(best, option, result);
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const DistanceOption& option,
    DistanceResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group1))
    return 0.0;

  if (!checkGroupValidity(this, group2))
    return 0.0;

  if (group1 == group2)
    return distance(group1, option, result);

  auto* nativeGroup1 = static_cast<DARTCollisionGroup*>(group1);
  auto* nativeGroup2 = static_cast<DARTCollisionGroup*>(group2);
  nativeGroup1->updateEngineData();
  nativeGroup2->updateEngineData();

  NativeDistanceCandidate best;
  for (auto* object1 : nativeGroup1->mCollisionObjects) {
    for (auto* object2 : nativeGroup2->mCollisionObjects) {
      if (processNativeDistancePair(
              static_cast<const DARTCollisionObject*>(object1),
              static_cast<const DARTCollisionObject*>(object2),
              option,
              best)) {
        return convertDistanceResult(best, option, result);
      }
    }
  }

  return convertDistanceResult(best, option, result);
}

//==============================================================================
bool DARTCollisionDetector::raycast(
    CollisionGroup* group,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return false;

  const Eigen::Vector3d displacement = to - from;
  const double rayLength = displacement.norm();
  if (rayLength <= std::numeric_limits<double>::epsilon())
    return false;

  native::Ray ray(from, displacement, rayLength);

  auto* nativeGroup = static_cast<DARTCollisionGroup*>(group);
  nativeGroup->updateEngineData();

  std::vector<NativeRayHitCandidate> hits;
  for (auto* object : nativeGroup->mCollisionObjects) {
    const auto* nativeObject = static_cast<const DARTCollisionObject*>(object);
    if (processNativeRaycastObject(nativeObject, ray, rayLength, option, hits)
        && result == nullptr) {
      return true;
    }
  }

  if (hits.empty())
    return false;

  if (result)
    convertRaycastResults(hits, option, *result);

  return true;
}

//==============================================================================
DARTCollisionDetector::DARTCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new DARTCollisionObjectManager(this));
  if (const char* value = std::getenv("DART_SOFT_FACE_INTERIOR_CONTACTS")) {
    setSoftFaceInteriorContactsEnabled(
        value[0] != '\0' && std::strcmp(value, "0") != 0);
  }
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  return std::unique_ptr<DARTCollisionObject>(
      new DARTCollisionObject(this, shapeFrame));
}

//==============================================================================
void DARTCollisionDetector::refreshCollisionObject(CollisionObject* /*object*/)
{
  // Do nothing. DARTCollisionObject refreshes lazily in updateEngineData().
}

} // namespace collision
} // namespace dart
