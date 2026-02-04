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

#include "dart/collision/bullet/bullet_collision_group.hpp"

#include "dart/collision/bullet/bullet_collision_object.hpp"
#include "dart/collision/bullet/detail/bullet_collision_dispatcher.hpp"
#include "dart/collision/collision_object.hpp"

#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#if !defined(_WIN32)
  #include <LinearMath/btAlignedAllocator.h>

  #include <atomic>
  #include <mutex>
  #include <new>
  #include <unordered_set>
  #include <vector>

  #include <cstdlib>
#endif

namespace dart {
namespace collision {

#if !defined(_WIN32)
namespace {

using BulletAllocSet = std::unordered_set<void*>;

BulletAllocSet& getOutstandingAllocs()
{
  static BulletAllocSet* allocs = new BulletAllocSet();
  return *allocs;
}

std::mutex& getAllocMutex()
{
  static std::mutex* mtx = new std::mutex();
  return *mtx;
}

std::atomic<bool>& cleanupInProgress()
{
  static auto* flag = new std::atomic<bool>(false);
  return *flag;
}

void dartBulletFreeImpl(void* ptr)
{
  if (!ptr) {
    return;
  }

  bool shouldFree = true;
  {
    std::lock_guard<std::mutex> lock(getAllocMutex());
    auto& allocs = getOutstandingAllocs();
    const auto erased = allocs.erase(ptr);
    if (erased == 0 && cleanupInProgress().load(std::memory_order_relaxed)) {
      shouldFree = false;
    }
  }

  #if defined(_MSC_VER)
  if (!shouldFree) {
    return;
  }
  _aligned_free(ptr);
  #else
  if (!shouldFree) {
    return;
  }
  std::free(ptr);
  #endif
}

void dartBulletFree(void* ptr)
{
  dartBulletFreeImpl(ptr);
}

void* dartBulletAlloc(size_t size, std::size_t alignment)
{
  void* mem = nullptr;
  #if defined(_MSC_VER)
  mem = _aligned_malloc(size, alignment);
  if (!mem) {
    throw std::bad_alloc();
  }
  #else
  if (posix_memalign(&mem, std::max<int>(alignment, 16), size) != 0) {
    throw std::bad_alloc();
  }
  #endif
  {
    std::lock_guard<std::mutex> lock(getAllocMutex());
    getOutstandingAllocs().insert(mem);
  }
  return mem;
}

void* dartBulletAllocStd(size_t size)
{
  return dartBulletAlloc(size, 16);
}

void releaseOutstandingBulletAllocations()
{
  std::vector<void*> toFree;
  {
    std::lock_guard<std::mutex> lock(getAllocMutex());
    auto& allocs = getOutstandingAllocs();
    if (allocs.empty()) {
      return;
    }
    toFree.assign(allocs.begin(), allocs.end());
    cleanupInProgress().store(true, std::memory_order_relaxed);
  }

  for (void* ptr : toFree) {
    dartBulletFreeImpl(ptr);
  }
}

void ensureBulletAllocator()
{
  static std::once_flag flag;
  std::call_once(flag, []() {
    btAlignedAllocSetCustom(&dartBulletAllocStd, &dartBulletFree);
    std::atexit(releaseOutstandingBulletAllocations);
  });
}

} // namespace
#else
namespace {

void ensureBulletAllocator()
{
  // Windows builds use the default Bullet allocator. The custom allocator
  // backed by _aligned_malloc/_aligned_free introduces instability on MSVC.
}

} // namespace
#endif

//==============================================================================
BulletCollisionGroup::BulletCollisionGroup(
    const CollisionDetectorPtr& collisionDetector)
  : CollisionGroup(collisionDetector),
    mBulletProadphaseAlg(new btDbvtBroadphase()),
    mBulletCollisionConfiguration(new btDefaultCollisionConfiguration()),
    mBulletDispatcher(new detail::BulletCollisionDispatcher(
        mBulletCollisionConfiguration.get())),
    mBulletCollisionWorld(new btCollisionWorld(
        mBulletDispatcher.get(),
        mBulletProadphaseAlg.get(),
        mBulletCollisionConfiguration.get()))
{
  ensureBulletAllocator();
  btGImpactCollisionAlgorithm::registerAlgorithm(
      static_cast<btCollisionDispatcher*>(mBulletDispatcher.get()));
}

//==============================================================================
void BulletCollisionGroup::initializeEngineData()
{
  // Do nothing
}

//==============================================================================
void BulletCollisionGroup::addCollisionObjectToEngine(CollisionObject* object)
{
  auto casted = static_cast<BulletCollisionObject*>(object);

  mBulletCollisionWorld->addCollisionObject(casted->getBulletCollisionObject());

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::addCollisionObjectsToEngine(
    std::span<CollisionObject* const> collObjects)
{
  for (auto collObj : collObjects) {
    auto casted = static_cast<BulletCollisionObject*>(collObj);

    mBulletCollisionWorld->addCollisionObject(
        casted->getBulletCollisionObject());
  }

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeCollisionObjectFromEngine(
    CollisionObject* object)
{
  auto casted = static_cast<BulletCollisionObject*>(object);
  auto* btObject = casted->getBulletCollisionObject();
  auto* proxy = btObject->getBroadphaseHandle();
  auto* pairCache
      = mBulletCollisionWorld->getBroadphase()->getOverlappingPairCache();

  if (proxy && pairCache) {
    pairCache->cleanProxyFromPairs(proxy, mBulletDispatcher.get());
    pairCache->removeOverlappingPairsContainingProxy(
        proxy, mBulletDispatcher.get());
  }

  mBulletCollisionWorld->removeCollisionObject(btObject);

  if (mBulletProadphaseAlg) {
    auto* dbvt = static_cast<btDbvtBroadphase*>(mBulletProadphaseAlg.get());
    dbvt->resetPool(mBulletDispatcher.get());
  }

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::removeAllCollisionObjectsFromEngine()
{
  for (const auto& info : mObjectInfoList) {
    removeCollisionObjectFromEngine(info->mObject.get());
  }

  if (mBulletProadphaseAlg) {
    auto* dbvt = static_cast<btDbvtBroadphase*>(mBulletProadphaseAlg.get());
    dbvt->resetPool(mBulletDispatcher.get());
  }

  initializeEngineData();
}

//==============================================================================
void BulletCollisionGroup::updateCollisionGroupEngineData()
{
  mBulletCollisionWorld->updateAabbs();
}

//==============================================================================
btCollisionWorld* BulletCollisionGroup::getBulletCollisionWorld()
{
  return mBulletCollisionWorld.get();
}

//==============================================================================
const btCollisionWorld* BulletCollisionGroup::getBulletCollisionWorld() const
{
  return mBulletCollisionWorld.get();
}

} // namespace collision
} // namespace dart
