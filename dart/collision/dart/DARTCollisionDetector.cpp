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
#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DARTCollide.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <algorithm>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart {
namespace collision {

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
    for (std::size_t i = 0; i < workerCount; ++i)
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
      for (std::size_t i = 0; i < count; ++i)
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
    for (std::size_t i = 0; i < mainEnd; ++i)
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
  bool mStop = false;
  bool mTaskActive = false;
  std::size_t mTaskGeneration = 0u;
  std::size_t mTaskCount = 0u;
  std::size_t mTaskChunkSize = 0u;
  std::size_t mWorkerLimit = 1u;
  std::size_t mNextWorkerIndex = 1u;
  std::size_t mActiveWorkerCount = 0u;
  void* mTaskCallable = nullptr;
  TaskInvoker mTaskInvoker = nullptr;
};

namespace {

struct BroadphaseEntry
{
  CollisionObject* object{nullptr};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max{Eigen::Vector3d::Zero()};
  bool finite{false};
  bool plane{false};
};

constexpr double kContactDuplicateTolerance = 3.0e-12;
constexpr double kContactPointCellSize = 4.0 * kContactDuplicateTolerance;
constexpr double kContactPointKeyLowerBound = -9223372036854775808.0;
constexpr double kContactPointKeyUpperBound = 9223372036854775808.0;
constexpr std::size_t kInvalidContactPointIndex
    = std::numeric_limits<std::size_t>::max();

struct ContactPointKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};
};

bool operator==(const ContactPointKey& lhs, const ContactPointKey& rhs);

struct ContactPointBucket
{
  ContactPointKey key;
  std::size_t head{kInvalidContactPointIndex};
  bool occupied{false};
};

class ContactPointIndex
{
public:
  void clear();
  void prepare(std::size_t maxContacts);
  bool containsClose(const Eigen::Vector3d& point) const;
  void add(const Eigen::Vector3d& point);

private:
  bool containsCloseLinear(const Eigen::Vector3d& point) const;
  void ensureBucketCapacity(std::size_t requiredPoints);
  void rehash(std::size_t bucketCount);
  std::size_t findBucketIndex(const ContactPointKey& key) const;
  std::size_t findOrCreateBucketIndex(const ContactPointKey& key);

  std::vector<ContactPointBucket> buckets;
  std::vector<std::size_t> usedBuckets;
  std::vector<Eigen::Vector3d> points;
  std::vector<std::size_t> nextPoint;
  std::size_t bucketMask{0u};
};

class ScratchCollisionResult final : public CollisionResult
{
public:
  ScratchCollisionResult()
  {
    setCollidingObjectCacheEnabled(false);
  }
};

struct BroadphaseScratch
{
  std::vector<BroadphaseEntry> finiteEntries1;
  std::vector<BroadphaseEntry> planeEntries1;
  std::vector<BroadphaseEntry> otherEntries1;
  std::vector<BroadphaseEntry> finiteEntries2;
  std::vector<BroadphaseEntry> planeEntries2;
  std::vector<BroadphaseEntry> otherEntries2;
  std::vector<const BroadphaseEntry*> sortedEntries1;
  std::vector<const BroadphaseEntry*> sortedEntries2;
  std::vector<std::size_t> parallelPairIndices;
  std::vector<ScratchCollisionResult> parallelPairResults;
  std::vector<char> parallelPairCollisions;
  // Remaining contacts ordered by the same representative-contact priority.
  std::vector<std::size_t> contactSelectionReserve;
  std::vector<std::size_t> selectedContactIndices;
  ScratchCollisionResult pairResult;
  ContactPointIndex contactPointIndex;

  void clear();
  void prepareParallelPairResults(std::size_t pairCount);
};

BroadphaseScratch& getBroadphaseScratch();

bool checkPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& pairResult,
    CollisionResult* result = nullptr);

bool processPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool shouldStopAfterPair(
    bool pairCollision,
    const CollisionOption& option,
    const CollisionResult* result);

bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol);

void selectContactIndices(
    const CollisionResult& pairResult,
    std::size_t maxContacts,
    std::vector<std::size_t>& candidates,
    std::vector<std::size_t>& selected);

void postProcess(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    const CollisionResult& pairResult);

bool isPlaneShape(const CollisionObject* object);

BroadphaseEntry makeBroadphaseEntry(CollisionObject* object);

void buildBroadphaseEntries(
    const std::vector<CollisionObject*>& objects,
    std::vector<BroadphaseEntry>& finiteEntries,
    std::vector<BroadphaseEntry>& planeEntries,
    std::vector<BroadphaseEntry>& otherEntries);

bool overlaps(const BroadphaseEntry& entry1, const BroadphaseEntry& entry2);

bool processFinitePlanePairs(
    const std::vector<BroadphaseEntry>& finiteEntries,
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    BroadphaseScratch& scratch,
    CollisionThreadPool* threadPool,
    std::size_t numCollisionThreads,
    bool planeFirst = true);

bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries1,
    const std::vector<BroadphaseEntry>& planeEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries,
    std::vector<const BroadphaseEntry*>& sortedEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries1,
    const std::vector<BroadphaseEntry>& entries2,
    std::vector<const BroadphaseEntry*>& sortedEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult);

} // anonymous namespace

//==============================================================================
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mRegistrar{
        DARTCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
          return dart::collision::DARTCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
DARTCollisionDetector::~DARTCollisionDetector() = default;

//==============================================================================
std::shared_ptr<CollisionDetector>
DARTCollisionDetector::cloneWithoutCollisionObjects() const
{
  auto clone = DARTCollisionDetector::create();
  clone->setNumCollisionThreads(mNumCollisionThreads);
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

  mNumCollisionThreads = std::max<std::size_t>(1u, numThreads);
  if (mNumCollisionThreads <= 1u) {
    mCollisionThreadPool.reset();
    return;
  }

  if (!mCollisionThreadPool)
    mCollisionThreadPool = std::make_unique<CollisionThreadPool>();
  mCollisionThreadPool->setWorkerCount(mNumCollisionThreads - 1u);
}

//==============================================================================
std::size_t DARTCollisionDetector::getNumCollisionThreads() const
{
  return mNumCollisionThreads;
}

//==============================================================================
std::unique_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(DARTCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get()) {
    dterr << "[DARTCollisionDetector::collide] Attempting to check collision "
          << "for a collision group that is created from a different collision "
          << "detector instance.\n";

    return false;
  }

  return true;
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group))
    return false;

  auto casted = static_cast<DARTCollisionGroup*>(group);
  casted->updateEngineData();
  const auto& objects = casted->mCollisionObjects;

  if (objects.empty())
    return false;

  auto& scratch = getBroadphaseScratch();
  scratch.clear();
  if (result)
    scratch.contactPointIndex.prepare(option.maxNumContacts);
  buildBroadphaseEntries(
      objects,
      scratch.finiteEntries1,
      scratch.planeEntries1,
      scratch.otherEntries1);

  auto collisionFound = false;
  if (processFinitePlanePairs(
          scratch.finiteEntries1,
          scratch.planeEntries1,
          option,
          result,
          collisionFound,
          scratch,
          mCollisionThreadPool.get(),
          mNumCollisionThreads)) {
    return true;
  }

  if (processFiniteFinitePairs(
          scratch.finiteEntries1,
          scratch.sortedEntries1,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  if (processPlanePlanePairs(
          scratch.planeEntries1,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  const auto& filter = option.collisionFilter;
  for (auto i = 0u; i < scratch.otherEntries1.size(); ++i) {
    for (auto j = i + 1u; j < scratch.otherEntries1.size(); ++j) {
      auto* collObj1 = scratch.otherEntries1[i].object;
      auto* collObj2 = scratch.otherEntries1[j].object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  for (const auto& otherEntry : scratch.otherEntries1) {
    for (const auto& finiteEntry : scratch.finiteEntries1) {
      auto* collObj1 = otherEntry.object;
      auto* collObj2 = finiteEntry.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }

    for (const auto& planeEntry : scratch.planeEntries1) {
      auto* collObj1 = otherEntry.object;
      auto* collObj2 = planeEntry.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  // Either no collision found or not reached the maximum number of contacts
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

  if (0u == option.maxNumContacts)
    return false;

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  auto casted1 = static_cast<DARTCollisionGroup*>(group1);
  auto casted2 = static_cast<DARTCollisionGroup*>(group2);

  casted1->updateEngineData();
  casted2->updateEngineData();

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  if (objects1.empty() || objects2.empty())
    return false;

  auto& scratch = getBroadphaseScratch();
  scratch.clear();
  if (result)
    scratch.contactPointIndex.prepare(option.maxNumContacts);
  buildBroadphaseEntries(
      objects1,
      scratch.finiteEntries1,
      scratch.planeEntries1,
      scratch.otherEntries1);
  buildBroadphaseEntries(
      objects2,
      scratch.finiteEntries2,
      scratch.planeEntries2,
      scratch.otherEntries2);

  auto collisionFound = false;
  if (processFinitePlanePairs(
          scratch.finiteEntries1,
          scratch.planeEntries2,
          option,
          result,
          collisionFound,
          scratch,
          mCollisionThreadPool.get(),
          mNumCollisionThreads,
          false)) {
    return true;
  }

  if (processFinitePlanePairs(
          scratch.finiteEntries2,
          scratch.planeEntries1,
          option,
          result,
          collisionFound,
          scratch,
          mCollisionThreadPool.get(),
          mNumCollisionThreads)) {
    return true;
  }

  if (processFiniteFinitePairs(
          scratch.finiteEntries1,
          scratch.finiteEntries2,
          scratch.sortedEntries2,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  if (processPlanePlanePairs(
          scratch.planeEntries1,
          scratch.planeEntries2,
          option,
          result,
          collisionFound,
          scratch.pairResult)) {
    return true;
  }

  const auto& filter = option.collisionFilter;
  for (const auto& entry1 : scratch.otherEntries1) {
    for (const auto& entry2 : scratch.otherEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }

    for (const auto& entry2 : scratch.finiteEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }

    for (const auto& entry2 : scratch.planeEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  for (const auto& entry1 : scratch.finiteEntries1) {
    for (const auto& entry2 : scratch.otherEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  for (const auto& entry1 : scratch.planeEntries1) {
    for (const auto& entry2 : scratch.otherEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1,
              collObj2,
              option,
              result,
              collisionFound,
              scratch.pairResult)) {
        return true;
      }
    }
  }

  // Either no collision found or not reached the maximum number of contacts
  return collisionFound;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[DARTCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  dtwarn << "[DARTCollisionDetector::distance] This collision detector does "
         << "not support (signed) distance queries. Returning 0.0.\n";

  return 0.0;
}

//==============================================================================
DARTCollisionDetector::DARTCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
}

//==============================================================================
void warnUnsupportedShapeType(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto& shape = shapeFrame->getShape();
  const auto& shapeType = shape->getType();

  if (shapeType == dynamics::SphereShape::getStaticType())
    return;

  if (shapeType == dynamics::BoxShape::getStaticType())
    return;

  if (shapeType == dynamics::PlaneShape::getStaticType())
    return;

  if (shapeType == dynamics::CylinderShape::getStaticType())
    return;

  if (shapeType == dynamics::CapsuleShape::getStaticType())
    return;

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);

    if (ellipsoid->isSphere())
      return;
  }

  dterr << "[DARTCollisionDetector] Attempting to create shape type ["
        << shapeType << "] that is not supported "
        << "by DARTCollisionDetector. Currently, only SphereShape, BoxShape, "
        << "CylinderShape, CapsuleShape, PlaneShape, and EllipsoidShape (only "
        << "when all the radii are equal) are supported. This shape will "
           "always get "
        << "penetrated by other "
        << "objects.\n";
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  warnUnsupportedShapeType(shapeFrame);

  return std::unique_ptr<DARTCollisionObject>(
      new DARTCollisionObject(this, shapeFrame));
}

//==============================================================================
void DARTCollisionDetector::refreshCollisionObject(CollisionObject* /*object*/)
{
  // Do nothing
}

namespace {

//==============================================================================
bool operator==(const ContactPointKey& lhs, const ContactPointKey& rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

//==============================================================================
std::size_t hashContactPointKey(const ContactPointKey& key)
{
  auto mix = [](std::uint64_t value) {
    value ^= value >> 30;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31;
    return value;
  };

  auto hash = mix(static_cast<std::uint64_t>(key.x));
  hash ^= mix(
      static_cast<std::uint64_t>(key.y) + 0x9e3779b97f4a7c15ULL + (hash << 6)
      + (hash >> 2));
  hash ^= mix(
      static_cast<std::uint64_t>(key.z) + 0x9e3779b97f4a7c15ULL + (hash << 6)
      + (hash >> 2));

  return static_cast<std::size_t>(hash);
}

//==============================================================================
std::size_t nextPowerOfTwo(std::size_t value)
{
  std::size_t power = 1u;
  while (power < value
         && power <= std::numeric_limits<std::size_t>::max() / 2u) {
    power <<= 1u;
  }

  return power;
}

//==============================================================================
bool makeContactPointKey(const Eigen::Vector3d& point, ContactPointKey& key)
{
  for (auto i = 0; i < 3; ++i) {
    if (!std::isfinite(point[i]))
      return false;

    const auto cell = std::floor(point[i] / kContactPointCellSize);
    if (cell < kContactPointKeyLowerBound
        || cell >= kContactPointKeyUpperBound) {
      return false;
    }

    if (i == 0)
      key.x = static_cast<std::int64_t>(cell);
    else if (i == 1)
      key.y = static_cast<std::int64_t>(cell);
    else
      key.z = static_cast<std::int64_t>(cell);
  }

  return true;
}

//==============================================================================
void getContactPointNeighborOffsetRange(
    double value, std::int64_t cell, int& first, int& last)
{
  first = 0;
  last = 0;

  const auto lower = static_cast<double>(cell) * kContactPointCellSize;
  const auto upper = lower + kContactPointCellSize;
  const auto roundoff
      = 8.0 * std::numeric_limits<double>::epsilon()
        * std::max({1.0, std::abs(value), std::abs(lower), std::abs(upper)});
  const auto boundaryTolerance = kContactDuplicateTolerance + roundoff;

  if (value - lower <= boundaryTolerance)
    first = -1;

  if (upper - value <= boundaryTolerance)
    last = 1;
}

//==============================================================================
bool offsetCell(std::int64_t cell, int offset, std::int64_t& result)
{
  if (offset > 0) {
    if (cell > std::numeric_limits<std::int64_t>::max() - offset)
      return false;
  } else if (offset < 0) {
    if (cell < std::numeric_limits<std::int64_t>::min() - offset)
      return false;
  }

  result = cell + offset;
  return true;
}

//==============================================================================
void ContactPointIndex::clear()
{
  for (const auto bucketIndex : usedBuckets)
    buckets[bucketIndex] = ContactPointBucket();

  usedBuckets.clear();
  points.clear();
  nextPoint.clear();
}

//==============================================================================
void ContactPointIndex::prepare(std::size_t maxContacts)
{
  clear();

  const auto reserveContacts
      = std::max<std::size_t>(16u, std::min<std::size_t>(maxContacts, 65536u));
  points.reserve(reserveContacts);
  nextPoint.reserve(reserveContacts);
  usedBuckets.reserve(reserveContacts);
  ensureBucketCapacity(reserveContacts);
}

//==============================================================================
bool ContactPointIndex::containsClose(const Eigen::Vector3d& point) const
{
  ContactPointKey key;
  if (buckets.empty() || !makeContactPointKey(point, key))
    return containsCloseLinear(point);

  int firstDx = 0;
  int lastDx = 0;
  int firstDy = 0;
  int lastDy = 0;
  int firstDz = 0;
  int lastDz = 0;
  getContactPointNeighborOffsetRange(point.x(), key.x, firstDx, lastDx);
  getContactPointNeighborOffsetRange(point.y(), key.y, firstDy, lastDy);
  getContactPointNeighborOffsetRange(point.z(), key.z, firstDz, lastDz);

  for (auto dx = firstDx; dx <= lastDx; ++dx) {
    ContactPointKey neighbor;
    if (!offsetCell(key.x, dx, neighbor.x))
      continue;

    for (auto dy = firstDy; dy <= lastDy; ++dy) {
      if (!offsetCell(key.y, dy, neighbor.y))
        continue;

      for (auto dz = firstDz; dz <= lastDz; ++dz) {
        if (!offsetCell(key.z, dz, neighbor.z))
          continue;

        const auto bucketIndex = findBucketIndex(neighbor);
        if (bucketIndex == kInvalidContactPointIndex)
          continue;

        for (auto pointIndex = buckets[bucketIndex].head;
             pointIndex != kInvalidContactPointIndex;
             pointIndex = nextPoint[pointIndex]) {
          if (isClose(point, points[pointIndex], kContactDuplicateTolerance))
            return true;
        }
      }
    }
  }

  return false;
}

//==============================================================================
void ContactPointIndex::add(const Eigen::Vector3d& point)
{
  ensureBucketCapacity(points.size() + 1u);

  const auto pointIndex = points.size();
  points.push_back(point);
  nextPoint.push_back(kInvalidContactPointIndex);

  ContactPointKey key;
  if (!makeContactPointKey(point, key))
    return;

  const auto bucketIndex = findOrCreateBucketIndex(key);
  if (bucketIndex == kInvalidContactPointIndex)
    return;

  nextPoint[pointIndex] = buckets[bucketIndex].head;
  buckets[bucketIndex].head = pointIndex;
}

//==============================================================================
bool ContactPointIndex::containsCloseLinear(const Eigen::Vector3d& point) const
{
  for (const auto& existingPoint : points) {
    if (isClose(point, existingPoint, kContactDuplicateTolerance))
      return true;
  }

  return false;
}

//==============================================================================
void ContactPointIndex::ensureBucketCapacity(std::size_t requiredPoints)
{
  const auto minBucketCount = 64u;
  auto desiredBucketCount = minBucketCount;
  if (requiredPoints <= std::numeric_limits<std::size_t>::max() / 4u) {
    desiredBucketCount
        = std::max<std::size_t>(minBucketCount, requiredPoints * 4u);
  }

  desiredBucketCount = nextPowerOfTwo(desiredBucketCount);
  if (buckets.size() >= desiredBucketCount)
    return;

  rehash(desiredBucketCount);
}

//==============================================================================
void ContactPointIndex::rehash(std::size_t bucketCount)
{
  buckets.assign(bucketCount, ContactPointBucket());
  usedBuckets.clear();
  bucketMask = buckets.empty() ? 0u : buckets.size() - 1u;

  for (auto i = 0u; i < points.size(); ++i) {
    nextPoint[i] = kInvalidContactPointIndex;

    ContactPointKey key;
    if (!makeContactPointKey(points[i], key))
      continue;

    const auto bucketIndex = findOrCreateBucketIndex(key);
    if (bucketIndex == kInvalidContactPointIndex)
      continue;

    nextPoint[i] = buckets[bucketIndex].head;
    buckets[bucketIndex].head = i;
  }
}

//==============================================================================
std::size_t ContactPointIndex::findBucketIndex(const ContactPointKey& key) const
{
  if (buckets.empty())
    return kInvalidContactPointIndex;

  auto bucketIndex = hashContactPointKey(key) & bucketMask;
  for (auto probe = 0u; probe < buckets.size(); ++probe) {
    const auto& bucket = buckets[bucketIndex];
    if (!bucket.occupied)
      return kInvalidContactPointIndex;

    if (bucket.key == key)
      return bucketIndex;

    bucketIndex = (bucketIndex + 1u) & bucketMask;
  }

  return kInvalidContactPointIndex;
}

//==============================================================================
std::size_t ContactPointIndex::findOrCreateBucketIndex(
    const ContactPointKey& key)
{
  if (buckets.empty())
    return kInvalidContactPointIndex;

  auto bucketIndex = hashContactPointKey(key) & bucketMask;
  for (auto probe = 0u; probe < buckets.size(); ++probe) {
    auto& bucket = buckets[bucketIndex];
    if (!bucket.occupied) {
      bucket.occupied = true;
      bucket.key = key;
      bucket.head = kInvalidContactPointIndex;
      usedBuckets.push_back(bucketIndex);
      return bucketIndex;
    }

    if (bucket.key == key)
      return bucketIndex;

    bucketIndex = (bucketIndex + 1u) & bucketMask;
  }

  return kInvalidContactPointIndex;
}

//==============================================================================
void BroadphaseScratch::clear()
{
  finiteEntries1.clear();
  planeEntries1.clear();
  otherEntries1.clear();
  finiteEntries2.clear();
  planeEntries2.clear();
  otherEntries2.clear();
  sortedEntries1.clear();
  sortedEntries2.clear();
  parallelPairIndices.clear();
  pairResult.clear();
  contactPointIndex.clear();
}

//==============================================================================
void BroadphaseScratch::prepareParallelPairResults(std::size_t pairCount)
{
  parallelPairResults.resize(pairCount);
  parallelPairCollisions.assign(pairCount, 0);
}

//==============================================================================
BroadphaseScratch& getBroadphaseScratch()
{
  thread_local BroadphaseScratch scratch;
  return scratch;
}

//==============================================================================
bool checkPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& pairResult,
    CollisionResult* result)
{
  pairResult.clear();

  // Perform narrow-phase detection
  collide(
      static_cast<DARTCollisionObject*>(o1),
      static_cast<DARTCollisionObject*>(o2),
      pairResult);

  // Early return for binary check
  if (!result)
    return pairResult.isCollision();

  postProcess(o1, o2, option, *result, pairResult);

  return pairResult.isCollision();
}

//==============================================================================
bool processPair(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  const auto pairCollision = checkPair(o1, o2, option, pairResult, result);
  collisionFound = collisionFound || pairCollision;
  return shouldStopAfterPair(pairCollision, option, result);
}

//==============================================================================
bool shouldStopAfterPair(
    bool pairCollision,
    const CollisionOption& option,
    const CollisionResult* result)
{
  if (!result)
    return pairCollision;

  return result->getNumContacts() >= option.maxNumContacts;
}

//==============================================================================
bool isClose(
    const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, double tol)
{
  return (pos1 - pos2).squaredNorm() < tol * tol;
}

//==============================================================================
void selectContactIndices(
    const CollisionResult& pairResult,
    std::size_t maxContacts,
    std::vector<std::size_t>& reserve,
    std::vector<std::size_t>& selected)
{
  selected.clear();
  reserve.clear();

  auto& candidates = reserve;

  if (maxContacts == 0u)
    return;

  const auto& contacts = pairResult.getContacts();
  candidates.reserve(contacts.size());
  for (std::size_t i = 0u; i < contacts.size(); ++i) {
    const auto& contact = contacts[i];
    bool duplicate = false;
    for (auto& candidateIndex : candidates) {
      const auto& accepted = contacts[candidateIndex];
      if (isClose(contact.point, accepted.point, kContactDuplicateTolerance)) {
        if (contact.penetrationDepth > accepted.penetrationDepth)
          candidateIndex = i;
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      candidates.push_back(i);
  }

  if (candidates.size() <= maxContacts) {
    selected = candidates;
    candidates.clear();
    return;
  }

  selected.reserve(contacts.size());

  std::size_t deepestCandidate = 0u;
  auto deepestDepth = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0u; i < candidates.size(); ++i) {
    const auto depth = contacts[candidates[i]].penetrationDepth;
    if (depth > deepestDepth) {
      deepestDepth = depth;
      deepestCandidate = i;
    }
  }

  selected.push_back(candidates[deepestCandidate]);
  candidates.erase(
      candidates.begin() + static_cast<std::ptrdiff_t>(deepestCandidate));

  auto selectBestRemainingCandidate
      = [&](const std::vector<std::size_t>& selectedContacts) {
          std::size_t bestCandidate = 0u;
          auto bestDistance = -1.0;
          auto bestDepth = -std::numeric_limits<double>::infinity();

          for (std::size_t i = 0u; i < candidates.size(); ++i) {
            const auto& candidate = contacts[candidates[i]];
            auto minDistance = std::numeric_limits<double>::infinity();
            for (const auto selectedIndex : selectedContacts) {
              minDistance = std::min(
                  minDistance,
                  (candidate.point - contacts[selectedIndex].point)
                      .squaredNorm());
            }

            if (minDistance > bestDistance
                || (minDistance == bestDistance
                    && candidate.penetrationDepth > bestDepth)) {
              bestDistance = minDistance;
              bestDepth = candidate.penetrationDepth;
              bestCandidate = i;
            }
          }

          const auto selectedCandidate = candidates[bestCandidate];
          candidates.erase(
              candidates.begin() + static_cast<std::ptrdiff_t>(bestCandidate));
          return selectedCandidate;
        };

  while (selected.size() < maxContacts && !candidates.empty()) {
    selected.push_back(selectBestRemainingCandidate(selected));
  }

  const auto cappedSelectionSize = selected.size();
  while (!candidates.empty()) {
    selected.push_back(selectBestRemainingCandidate(selected));
  }

  // Keep the remaining unique contacts in the same depth/spread priority order
  // for duplicate backfill in postProcess().
  reserve.assign(
      selected.begin() + static_cast<std::ptrdiff_t>(cappedSelectionSize),
      selected.end());
  selected.resize(cappedSelectionSize);
  std::sort(selected.begin(), selected.end());
}

//==============================================================================
void postProcess(
    CollisionObject* o1,
    CollisionObject* o2,
    const CollisionOption& option,
    CollisionResult& totalResult,
    const CollisionResult& pairResult)
{
  if (!pairResult.isCollision())
    return;

  auto& scratch = getBroadphaseScratch();
  auto& contactPointIndex = scratch.contactPointIndex;
  const auto maxContactsPerPair = option.getEffectiveMaxNumContactsPerPair();
  const auto& pairContacts = pairResult.getContacts();
  if (totalResult.getNumContacts() >= option.maxNumContacts)
    return;

  auto appendContact = [&](const Contact& pairContact) {
    if (totalResult.getNumContacts() >= option.maxNumContacts)
      return std::make_pair(false, false);

    // Don't add repeated points.
    if (contactPointIndex.containsClose(pairContact.point))
      return std::make_pair(true, false);

    auto contact = pairContact;
    contact.collisionObject1 = o1;
    contact.collisionObject2 = o2;
    totalResult.addContact(contact);
    contactPointIndex.add(contact.point);
    return std::make_pair(
        totalResult.getNumContacts() < option.maxNumContacts, true);
  };

  const auto remainingContacts
      = option.maxNumContacts - totalResult.getNumContacts();
  const auto selectionLimit = std::min(maxContactsPerPair, remainingContacts);
  if (option.maxNumContactsPerPair > 0u
      && pairContacts.size() > selectionLimit) {
    selectContactIndices(
        pairResult,
        selectionLimit,
        scratch.contactSelectionReserve,
        scratch.selectedContactIndices);

    std::size_t numPairContacts = 0u;
    auto tryAppendContact = [&](std::size_t contactIndex) {
      const auto [shouldContinue, added]
          = appendContact(pairContacts[contactIndex]);
      if (added)
        ++numPairContacts;

      return shouldContinue && numPairContacts < maxContactsPerPair;
    };

    for (const auto contactIndex : scratch.selectedContactIndices) {
      if (!tryAppendContact(contactIndex))
        return;
    }

    for (const auto contactIndex : scratch.contactSelectionReserve) {
      if (!tryAppendContact(contactIndex))
        return;
    }
    return;
  }

  std::size_t numPairContacts = 0u;
  for (const auto& pairContact : pairContacts) {
    if (numPairContacts >= maxContactsPerPair)
      break;

    const auto [shouldContinue, added] = appendContact(pairContact);
    if (added)
      ++numPairContacts;

    if (!shouldContinue)
      break;
  }
}

//==============================================================================
bool isPlaneShape(const CollisionObject* object)
{
  if (!object)
    return false;

  return static_cast<const DARTCollisionObject*>(object)->isCachedPlaneShape();
}

//==============================================================================
bool hasCachedPlaneCollisionPath(const BroadphaseEntry& finiteEntry)
{
  if (finiteEntry.object == nullptr)
    return false;

  const auto* dartObject
      = static_cast<const DARTCollisionObject*>(finiteEntry.object);
  const auto* shape = dartObject->getCachedShape();
  if (shape == nullptr)
    return false;

  using CachedShapeKind = DARTCollisionObject::CachedShapeKind;
  switch (dartObject->getCachedShapeKind()) {
    case CachedShapeKind::Sphere:
    case CachedShapeKind::SphereEllipsoid:
    case CachedShapeKind::Box:
    case CachedShapeKind::Cylinder:
    case CachedShapeKind::Capsule:
      return true;
    case CachedShapeKind::Unknown:
    case CachedShapeKind::Plane:
      break;
  }

  return false;
}

//==============================================================================
bool allFiniteEntriesHaveCachedPlaneCollisionPath(
    const std::vector<BroadphaseEntry>& finiteEntries)
{
  for (const auto& finiteEntry : finiteEntries) {
    if (!hasCachedPlaneCollisionPath(finiteEntry))
      return false;
  }

  return true;
}

//==============================================================================
Eigen::Vector3d computeBroadphaseRoundoffMargin(
    const Eigen::Isometry3d& transform,
    const Eigen::Matrix3d& absLinear,
    const Eigen::Vector3d& localCenter,
    const Eigen::Vector3d& localHalfExtents)
{
  const Eigen::Vector3d accumulatedMagnitude
      = transform.translation().cwiseAbs()
        + absLinear * (localCenter.cwiseAbs() + localHalfExtents.cwiseAbs());

  constexpr double kRoundoffTerms = 16.0;
  return kRoundoffTerms * std::numeric_limits<double>::epsilon()
         * accumulatedMagnitude;
}

//==============================================================================
BroadphaseEntry makeBroadphaseEntry(CollisionObject* object)
{
  BroadphaseEntry entry;
  entry.object = object;
  entry.plane = isPlaneShape(object);

  if (!object)
    return entry;

  entry.transform = object->getTransform();

  if (entry.plane)
    return entry;

  const auto* dartObject = static_cast<const DARTCollisionObject*>(object);
  if (!dartObject->getCachedShape())
    return entry;

  const auto& localMin = dartObject->getCachedLocalBoundsMin();
  const auto& localMax = dartObject->getCachedLocalBoundsMax();
  if (!dartObject->hasFiniteCachedLocalBounds())
    return entry;

  const Eigen::Vector3d localCenter = 0.5 * (localMin + localMax);
  const Eigen::Vector3d localHalfExtents = 0.5 * (localMax - localMin);
  const Eigen::Vector3d worldCenter = entry.transform * localCenter;
  const Eigen::Matrix3d absLinear = entry.transform.linear().cwiseAbs();
  const Eigen::Vector3d worldHalfExtents = absLinear * localHalfExtents;
  // The center/extent form matches transformed corners mathematically, but the
  // separately rounded dot products can otherwise move a bound inward.
  const Eigen::Vector3d roundoffMargin = computeBroadphaseRoundoffMargin(
      entry.transform, absLinear, localCenter, localHalfExtents);
  entry.min = (worldCenter - worldHalfExtents - roundoffMargin)
                  .unaryExpr([](double value) {
                    return std::nextafter(
                        value, -std::numeric_limits<double>::infinity());
                  });
  entry.max = (worldCenter + worldHalfExtents + roundoffMargin)
                  .unaryExpr([](double value) {
                    return std::nextafter(
                        value, std::numeric_limits<double>::infinity());
                  });

  entry.finite = entry.min.allFinite() && entry.max.allFinite();
  return entry;
}

//==============================================================================
void buildBroadphaseEntries(
    const std::vector<CollisionObject*>& objects,
    std::vector<BroadphaseEntry>& finiteEntries,
    std::vector<BroadphaseEntry>& planeEntries,
    std::vector<BroadphaseEntry>& otherEntries)
{
  finiteEntries.reserve(objects.size());
  planeEntries.reserve(objects.size());
  otherEntries.reserve(objects.size());

  for (auto* object : objects) {
    auto entry = makeBroadphaseEntry(object);
    if (entry.finite)
      finiteEntries.push_back(entry);
    else if (entry.plane)
      planeEntries.push_back(entry);
    else
      otherEntries.push_back(entry);
  }
}

//==============================================================================
bool overlaps(const BroadphaseEntry& entry1, const BroadphaseEntry& entry2)
{
  if (!entry1.finite || !entry2.finite)
    return true;

  for (int axis = 0; axis < 3; ++axis) {
    if (entry1.max[axis] < entry2.min[axis]
        || entry2.max[axis] < entry1.min[axis]) {
      return false;
    }
  }

  return true;
}

//==============================================================================
bool processFinitePlanePairs(
    const std::vector<BroadphaseEntry>& finiteEntries,
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    BroadphaseScratch& scratch,
    CollisionThreadPool* threadPool,
    std::size_t numCollisionThreads,
    bool planeFirst)
{
  constexpr std::size_t kMinParallelFinitePlanePairs = 128u;
  const std::size_t pairCount = finiteEntries.size() * planeEntries.size();
  const bool contactCapCanShortCircuit = option.maxNumContacts < pairCount;
  const bool allPairsHaveCachedPlaneCollisionPath
      = allFiniteEntriesHaveCachedPlaneCollisionPath(finiteEntries);
  const bool canConsiderParallel
      = result != nullptr && threadPool != nullptr && numCollisionThreads > 1u
        && !contactCapCanShortCircuit && allPairsHaveCachedPlaneCollisionPath
        && pairCount >= kMinParallelFinitePlanePairs;
  const auto& filter = option.collisionFilter;
  const bool hasSinglePlane = planeEntries.size() == 1u;

  auto getPairEntries = [&](std::size_t pairIndex) {
    const auto planeIndex
        = hasSinglePlane ? 0u : pairIndex / finiteEntries.size();
    const auto finiteIndex
        = hasSinglePlane ? pairIndex
                         : pairIndex - planeIndex * finiteEntries.size();
    return std::make_pair(
        &planeEntries[planeIndex], &finiteEntries[finiteIndex]);
  };

  auto processSerialPairAt = [&](std::size_t pairIndex) {
    const auto [planeEntry, finiteEntry] = getPairEntries(pairIndex);
    auto* collObj1 = planeFirst ? planeEntry->object : finiteEntry->object;
    auto* collObj2 = planeFirst ? finiteEntry->object : planeEntry->object;

    scratch.pairResult.clear();
    const auto pairCollision = collidePlaneShape(
        static_cast<DARTCollisionObject*>(planeEntry->object),
        static_cast<DARTCollisionObject*>(finiteEntry->object),
        planeEntry->transform,
        finiteEntry->transform,
        planeFirst,
        scratch.pairResult);
    if (result != nullptr)
      postProcess(collObj1, collObj2, option, *result, scratch.pairResult);

    collisionFound = collisionFound || (pairCollision > 0);
    return shouldStopAfterPair(pairCollision > 0, option, result);
  };

  std::size_t parallelPairCount = pairCount;
  if (canConsiderParallel && filter) {
    scratch.parallelPairIndices.clear();
    scratch.parallelPairIndices.reserve(pairCount);
    for (std::size_t pairIndex = 0u; pairIndex < pairCount; ++pairIndex) {
      const auto [planeEntry, finiteEntry] = getPairEntries(pairIndex);
      auto* collObj1 = planeFirst ? planeEntry->object : finiteEntry->object;
      auto* collObj2 = planeFirst ? finiteEntry->object : planeEntry->object;
      if (!filter->ignoresCollision(collObj1, collObj2))
        scratch.parallelPairIndices.push_back(pairIndex);
    }

    parallelPairCount = scratch.parallelPairIndices.size();
    if (parallelPairCount == 0u)
      return false;
  }

  const bool canParallelize
      = canConsiderParallel
        && parallelPairCount >= kMinParallelFinitePlanePairs;

  if (canParallelize) {
    scratch.prepareParallelPairResults(parallelPairCount);

    auto collidePairAt = [&](std::size_t workIndex) {
      const auto pairIndex
          = filter ? scratch.parallelPairIndices[workIndex] : workIndex;
      auto& pairResult = scratch.parallelPairResults[workIndex];
      pairResult.clear();

      const auto [planeEntry, finiteEntry] = getPairEntries(pairIndex);

      collidePlaneShape(
          static_cast<DARTCollisionObject*>(planeEntry->object),
          static_cast<DARTCollisionObject*>(finiteEntry->object),
          planeEntry->transform,
          finiteEntry->transform,
          planeFirst,
          pairResult);
      scratch.parallelPairCollisions[workIndex]
          = pairResult.isCollision() ? 1 : 0;
    };

    threadPool->parallelFor(
        parallelPairCount, numCollisionThreads, collidePairAt);

    for (std::size_t workIndex = 0u; workIndex < parallelPairCount;
         ++workIndex) {
      if (scratch.parallelPairCollisions[workIndex] == 0)
        continue;

      const auto pairIndex
          = filter ? scratch.parallelPairIndices[workIndex] : workIndex;
      const auto [planeEntry, finiteEntry] = getPairEntries(pairIndex);
      auto* collObj1 = planeFirst ? planeEntry->object : finiteEntry->object;
      auto* collObj2 = planeFirst ? finiteEntry->object : planeEntry->object;

      collisionFound = true;
      postProcess(
          collObj1,
          collObj2,
          option,
          *result,
          scratch.parallelPairResults[workIndex]);

      if (shouldStopAfterPair(true, option, result))
        return true;
    }

    return false;
  }

  if (canConsiderParallel && filter) {
    for (const auto pairIndex : scratch.parallelPairIndices) {
      if (processSerialPairAt(pairIndex))
        return true;
    }
    return false;
  }

  for (std::size_t planeIndex = 0u; planeIndex < planeEntries.size();
       ++planeIndex) {
    for (std::size_t finiteIndex = 0u; finiteIndex < finiteEntries.size();
         ++finiteIndex) {
      const auto pairIndex = planeIndex * finiteEntries.size() + finiteIndex;
      const auto [planeEntry, finiteEntry] = getPairEntries(pairIndex);
      auto* collObj1 = planeFirst ? planeEntry->object : finiteEntry->object;
      auto* collObj2 = planeFirst ? finiteEntry->object : planeEntry->object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;

      if (processSerialPairAt(pairIndex))
        return true;
    }
  }

  return false;
}

//==============================================================================
bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  const auto& filter = option.collisionFilter;
  for (std::size_t i = 0u; i + 1u < planeEntries.size(); ++i) {
    for (std::size_t j = i + 1u; j < planeEntries.size(); ++j) {
      auto* collObj1 = planeEntries[i].object;
      auto* collObj2 = planeEntries[j].object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processPlanePlanePairs(
    const std::vector<BroadphaseEntry>& planeEntries1,
    const std::vector<BroadphaseEntry>& planeEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  const auto& filter = option.collisionFilter;
  for (const auto& entry1 : planeEntries1) {
    for (const auto& entry2 : planeEntries2) {
      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries,
    std::vector<const BroadphaseEntry*>& sortedEntries,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  sortedEntries.reserve(entries.size());
  for (const auto& entry : entries)
    sortedEntries.push_back(&entry);

  std::sort(
      sortedEntries.begin(),
      sortedEntries.end(),
      [](const BroadphaseEntry* lhs, const BroadphaseEntry* rhs) {
        return lhs->min.x() < rhs->min.x();
      });

  const auto& filter = option.collisionFilter;
  for (std::size_t i = 0u; i + 1u < sortedEntries.size(); ++i) {
    const auto& entry1 = *sortedEntries[i];
    for (std::size_t j = i + 1u; j < sortedEntries.size(); ++j) {
      const auto& entry2 = *sortedEntries[j];
      if (entry2.min.x() > entry1.max.x())
        break;

      if (!overlaps(entry1, entry2))
        continue;

      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

//==============================================================================
bool processFiniteFinitePairs(
    const std::vector<BroadphaseEntry>& entries1,
    const std::vector<BroadphaseEntry>& entries2,
    std::vector<const BroadphaseEntry*>& sortedEntries2,
    const CollisionOption& option,
    CollisionResult* result,
    bool& collisionFound,
    CollisionResult& pairResult)
{
  sortedEntries2.reserve(entries2.size());
  for (const auto& entry : entries2)
    sortedEntries2.push_back(&entry);

  std::sort(
      sortedEntries2.begin(),
      sortedEntries2.end(),
      [](const BroadphaseEntry* lhs, const BroadphaseEntry* rhs) {
        return lhs->min.x() < rhs->min.x();
      });

  const auto& filter = option.collisionFilter;
  for (const auto& entry1 : entries1) {
    for (const auto* entry2Ptr : sortedEntries2) {
      const auto& entry2 = *entry2Ptr;
      if (entry2.max.x() < entry1.min.x())
        continue;
      if (entry2.min.x() > entry1.max.x())
        break;
      if (!overlaps(entry1, entry2))
        continue;

      auto* collObj1 = entry1.object;
      auto* collObj2 = entry2.object;
      if (filter && filter->ignoresCollision(collObj1, collObj2))
        continue;
      if (processPair(
              collObj1, collObj2, option, result, collisionFound, pairResult)) {
        return true;
      }
    }
  }

  return false;
}

} // anonymous namespace

} // namespace collision
} // namespace dart
