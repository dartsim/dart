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

#include "../../helpers/gtest_utils.hpp"

#include <dart/common/memory_manager.hpp>

#include <dart/all.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

//==============================================================================
TEST(MemoryManagerTest, BaseAllocator)
{
  auto mm = MemoryManager();
  auto& baseAllocator = mm.getBaseAllocator();
  auto& freeListAllocator = mm.getFreeListAllocator();
  auto& poolAllocator = mm.getPoolAllocator();

  EXPECT_EQ(&freeListAllocator.getBaseAllocator(), &baseAllocator);
  EXPECT_EQ(&poolAllocator.getBaseAllocator(), &freeListAllocator);
}

//==============================================================================
TEST(MemoryManagerTest, Allocate)
{
  auto mm = MemoryManager();

  // Cannot allocate 0 bytes
  EXPECT_EQ(mm.allocateUsingFree(0), nullptr);
  EXPECT_EQ(mm.allocateUsingPool(0), nullptr);

  // Allocate 1 byte using FreeListAllocator
  auto ptr1 = mm.allocateUsingFree(1);
  EXPECT_NE(ptr1, nullptr);
#if !defined(NDEBUG)
  EXPECT_TRUE(mm.hasAllocated(ptr1, 1));
  EXPECT_FALSE(mm.hasAllocated(nullptr, 1));
  EXPECT_FALSE(mm.hasAllocated(ptr1, 1 * 2));
#endif

  // Allocate 1 byte using PoolAllocator
  auto ptr2 = mm.allocateUsingPool(1);
  EXPECT_NE(ptr2, nullptr);
#if !defined(NDEBUG)
  EXPECT_TRUE(mm.hasAllocated(ptr2, 1));
  EXPECT_FALSE(mm.hasAllocated(nullptr, 1));
  EXPECT_FALSE(mm.hasAllocated(ptr2, 1 * 2));
#endif

  // Deallocate all
  mm.deallocateUsingFree(ptr1, 1);
  mm.deallocateUsingPool(ptr2, 1);
}

//==============================================================================
TEST(MemoryManagerTest, MemoryLeak)
{
  auto a = MemoryManager();

  // Allocate small memory
  auto ptr1 = a.allocateUsingPool(1);
  EXPECT_NE(ptr1, nullptr);

  // Allocate small memory
  auto ptr2 = a.allocateUsingFree(1);
  EXPECT_NE(ptr2, nullptr);

  // Expect that MemoryManager complains that not all the memory is deallocated
}

//==============================================================================
// Helper class that tracks construction/destruction
struct LifecycleTracker
{
  static int constructCount;
  static int destructCount;
  int value;

  explicit LifecycleTracker(int v) : value(v)
  {
    ++constructCount;
  }
  ~LifecycleTracker()
  {
    ++destructCount;
  }
  static void reset()
  {
    constructCount = 0;
    destructCount = 0;
  }
};
int LifecycleTracker::constructCount = 0;
int LifecycleTracker::destructCount = 0;

//==============================================================================
TEST(MemoryManagerTest, ConstructAndDestroy)
{
  auto mm = MemoryManager();
  LifecycleTracker::reset();

  // Construct using Free type dispatch
  auto* obj = mm.construct<LifecycleTracker>(MemoryManager::Type::Free, 42);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(LifecycleTracker::constructCount, 1);
  EXPECT_EQ(obj->value, 42);

  // Destroy using Free type dispatch
  mm.destroy<LifecycleTracker>(MemoryManager::Type::Free, obj);
  EXPECT_EQ(LifecycleTracker::destructCount, 1);
}

//==============================================================================
TEST(MemoryManagerTest, ConstructUsingFreeList)
{
  auto mm = MemoryManager();
  LifecycleTracker::reset();

  // Construct using FreeListAllocator
  auto* obj = mm.constructUsingFree<LifecycleTracker>(42);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(LifecycleTracker::constructCount, 1);
  EXPECT_EQ(obj->value, 42);

  // Destroy using FreeListAllocator
  mm.destroyUsingFree<LifecycleTracker>(obj);
  EXPECT_EQ(LifecycleTracker::destructCount, 1);
}

//==============================================================================
TEST(MemoryManagerTest, ConstructUsingPool)
{
  auto mm = MemoryManager();
  LifecycleTracker::reset();

  // Construct using PoolAllocator
  auto* obj = mm.constructUsingPool<LifecycleTracker>(42);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(LifecycleTracker::constructCount, 1);
  EXPECT_EQ(obj->value, 42);

  // Destroy using PoolAllocator
  mm.destroyUsingPool<LifecycleTracker>(obj);
  EXPECT_EQ(LifecycleTracker::destructCount, 1);
}

//==============================================================================
TEST(MemoryManagerTest, AllocateByTypeFree)
{
  auto mm = MemoryManager();

  // Allocate using Type::Free dispatch
  auto* ptr = mm.allocate(MemoryManager::Type::Free, sizeof(int));
  ASSERT_NE(ptr, nullptr);

  // Deallocate using Type::Free dispatch
  mm.deallocate(MemoryManager::Type::Free, ptr, sizeof(int));
}

//==============================================================================
TEST(MemoryManagerTest, AllocateByTypePool)
{
  auto mm = MemoryManager();

  // Allocate using Type::Pool dispatch
  auto* ptr = mm.allocate(MemoryManager::Type::Pool, sizeof(int));
  ASSERT_NE(ptr, nullptr);

  // Deallocate using Type::Pool dispatch
  mm.deallocate(MemoryManager::Type::Pool, ptr, sizeof(int));
}

//==============================================================================
TEST(MemoryManagerTest, GetDefault)
{
  // GetDefault should return the same instance
  auto& mm1 = MemoryManager::GetDefault();
  auto& mm2 = MemoryManager::GetDefault();
  EXPECT_EQ(&mm1, &mm2);
}

//==============================================================================
TEST(MemoryManagerTest, AllocateByTypeBase)
{
  auto mm = MemoryManager();

  // Allocate using Type::Base dispatch
  auto* ptr = mm.allocate(MemoryManager::Type::Base, sizeof(int));
  ASSERT_NE(ptr, nullptr);

  // Deallocate using Type::Base dispatch
  mm.deallocate(MemoryManager::Type::Base, ptr, sizeof(int));
}

//==============================================================================
TEST(MemoryManagerTest, MultipleAllocationSizes)
{
  auto mm = MemoryManager();

  // Test various allocation sizes using FreeListAllocator
  std::vector<std::pair<void*, size_t>> freeAllocations;
  const std::vector<size_t> sizes = {1, 8, 16, 64, 128, 256, 1024, 4096};

  for (auto size : sizes) {
    auto* ptr = mm.allocateUsingFree(size);
    ASSERT_NE(ptr, nullptr) << "Failed to allocate " << size << " bytes";
    freeAllocations.emplace_back(ptr, size);
  }

  // Deallocate all
  for (const auto& [ptr, size] : freeAllocations) {
    mm.deallocateUsingFree(ptr, size);
  }

  // Test various allocation sizes using PoolAllocator
  std::vector<std::pair<void*, size_t>> poolAllocations;

  for (auto size : sizes) {
    auto* ptr = mm.allocateUsingPool(size);
    ASSERT_NE(ptr, nullptr) << "Failed to allocate " << size << " bytes";
    poolAllocations.emplace_back(ptr, size);
  }

  // Deallocate all
  for (const auto& [ptr, size] : poolAllocations) {
    mm.deallocateUsingPool(ptr, size);
  }
}

//==============================================================================
TEST(MemoryManagerTest, PrintAndStreamOperator)
{
  auto mm = MemoryManager();

  // Test print() method
  std::ostringstream oss1;
  mm.print(oss1);
  EXPECT_FALSE(oss1.str().empty());
  EXPECT_NE(oss1.str().find("free_allocator"), std::string::npos);
  EXPECT_NE(oss1.str().find("pool_allocator"), std::string::npos);
  EXPECT_NE(oss1.str().find("base_allocator"), std::string::npos);

  // Test print() with indent
  std::ostringstream oss2;
  mm.print(oss2, 4);
  EXPECT_FALSE(oss2.str().empty());

  // Note: operator<< is a friend function not exported with DART_API on
  // Windows, so we verify print() output covers the same content instead.
  EXPECT_NE(oss1.str().find("[MemoryManager]"), std::string::npos);
}

//==============================================================================
TEST(MemoryManagerTest, DefaultAllocatorsAndStream)
{
  auto& mm = MemoryManager::GetDefault();

  auto& freeListAllocator = mm.getFreeListAllocator();
  auto& poolAllocator = mm.getPoolAllocator();

  auto* freePtr = mm.allocate(MemoryManager::Type::Free, sizeof(double));
  ASSERT_NE(freePtr, nullptr);
  mm.deallocate(MemoryManager::Type::Free, freePtr, sizeof(double));

  auto* poolPtr = mm.allocate(MemoryManager::Type::Pool, sizeof(double));
  ASSERT_NE(poolPtr, nullptr);
  mm.deallocate(MemoryManager::Type::Pool, poolPtr, sizeof(double));

  std::ostringstream oss;
  mm.print(oss, 2);
  EXPECT_NE(oss.str().find("free_allocator"), std::string::npos);
  EXPECT_NE(oss.str().find("pool_allocator"), std::string::npos);

  EXPECT_NE(&freeListAllocator.getBaseAllocator(), nullptr);
  EXPECT_NE(&poolAllocator.getBaseAllocator(), nullptr);

#ifndef _WIN32
  std::ostringstream stream;
  stream << mm;
  EXPECT_NE(stream.str().find("[MemoryManager]"), std::string::npos);
#endif
}

//==============================================================================
TEST(MemoryManagerTest, ConstructAndDestroyWithTypePool)
{
  auto mm = MemoryManager();
  LifecycleTracker::reset();

  // Construct using Pool type dispatch
  auto* obj = mm.construct<LifecycleTracker>(MemoryManager::Type::Pool, 99);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(LifecycleTracker::constructCount, 1);
  EXPECT_EQ(obj->value, 99);

  // Destroy using Pool type dispatch
  mm.destroy<LifecycleTracker>(MemoryManager::Type::Pool, obj);
  EXPECT_EQ(LifecycleTracker::destructCount, 1);
}

//==============================================================================
TEST(MemoryManagerTest, ConstructAndDestroyWithTypeBase)
{
  auto mm = MemoryManager();
  LifecycleTracker::reset();

  // Construct using Base type dispatch
  auto* obj = mm.construct<LifecycleTracker>(MemoryManager::Type::Base, 77);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(LifecycleTracker::constructCount, 1);
  EXPECT_EQ(obj->value, 77);

  // Destroy using Base type dispatch
  mm.destroy<LifecycleTracker>(MemoryManager::Type::Base, obj);
  EXPECT_EQ(LifecycleTracker::destructCount, 1);
}
