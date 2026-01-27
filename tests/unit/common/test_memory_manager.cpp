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
