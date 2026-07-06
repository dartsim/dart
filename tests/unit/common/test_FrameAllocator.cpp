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

#include "TestHelpers.hpp"

#include <dart/common/FrameAllocator.hpp>
#include <dart/common/MemoryAllocator.hpp>

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <memory>
#include <type_traits>
#include <vector>

#include <cstddef>
#include <cstdint>
#include <cstring>

using namespace dart;
using namespace common;

namespace {

struct alignas(64) OverAlignedStlValue
{
  double values[8] = {};
};

struct alignas(128) WideOverAlignedStlValue
{
  double values[16] = {};
};

struct CacheLineSizedStlValue
{
  double values[8] = {};
};

struct NonTrivialArenaValue
{
  explicit NonTrivialArenaValue(int* destroyCount) : mDestroyCount(destroyCount)
  {
    // Do nothing
  }

  ~NonTrivialArenaValue()
  {
    ++*mDestroyCount;
  }

  int* mDestroyCount;
};

} // namespace

//==============================================================================
TEST(FrameAllocatorTest, Construction)
{
  FrameAllocator allocator;
  EXPECT_EQ(allocator.capacity(), 65536u);
  EXPECT_LE(allocator.usableCapacity(), allocator.capacity());
  EXPECT_GE(allocator.usableCapacity() + 64u, allocator.capacity());
  EXPECT_EQ(allocator.usableCapacity() % 32u, 0u);
  EXPECT_EQ(allocator.used(), 0u);
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, BasicAllocate)
{
  FrameAllocator allocator;
  size_t previousUsed = 0;
  const std::array<size_t, 4> sizes{{8, 64, 256, 1024}};

  for (size_t size : sizes) {
    void* ptr = allocator.allocate(size);
    EXPECT_NE(ptr, nullptr);
    EXPECT_GT(allocator.used(), previousUsed);
    previousUsed = allocator.used();
  }
}

//==============================================================================
TEST(FrameAllocatorTest, UsableCapacityMatchesFastPathBudget)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 128);
  const size_t usableCapacity = allocator.usableCapacity();
  EXPECT_LE(usableCapacity, allocator.capacity());
  EXPECT_GE(usableCapacity + 64u, allocator.capacity());
  ASSERT_GT(usableCapacity, 0u);
  EXPECT_EQ(usableCapacity % 32u, 0u);

  void* ptr = allocator.allocate(usableCapacity);
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.used(), usableCapacity);
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, Alignment)
{
  FrameAllocator allocator;
  const std::array<size_t, 3> alignments{{16, 32, 64}};
  for (size_t alignment : alignments) {
    void* ptr = allocator.allocateAligned(32, alignment);
    EXPECT_NE(ptr, nullptr);
    const uintptr_t address = reinterpret_cast<uintptr_t>(ptr);
    EXPECT_EQ(address % alignment, 0u);
  }

  void* cacheAligned = allocator.allocateCacheAligned(32);
  EXPECT_NE(cacheAligned, nullptr);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(cacheAligned) % 64u, 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, RejectsInvalidAlignment)
{
  FrameAllocator allocator;
  EXPECT_EQ(allocator.allocateAligned(32, 24), nullptr);
  EXPECT_EQ(allocator.allocate(32, 24), nullptr);
  EXPECT_EQ(allocator.used(), 0u);
  EXPECT_EQ(allocator.overflowCount(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, RejectsOverflowingRequestsWithoutChangingArena)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 128);

  EXPECT_EQ(allocator.allocate(std::numeric_limits<size_t>::max()), nullptr);
  EXPECT_EQ(
      allocator.allocateAligned(std::numeric_limits<size_t>::max(), 64),
      nullptr);
  EXPECT_EQ(allocator.used(), 0u);
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);

  FrameAllocator overflowOnly(MemoryAllocator::GetDefault(), 0);
  EXPECT_EQ(overflowOnly.allocate(std::numeric_limits<size_t>::max()), nullptr);
  EXPECT_EQ(
      overflowOnly.allocateAligned(std::numeric_limits<size_t>::max(), 64),
      nullptr);
  EXPECT_EQ(overflowOnly.used(), 0u);
  EXPECT_EQ(overflowOnly.overflowCount(), 0u);
  EXPECT_EQ(overflowOnly.overflowBytes(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, TinyCapacityStaysInBounds)
{
  // A capacity smaller than the 64-byte arena alignment must not let the fast
  // path hand back storage past the buffer end. Exercised under ASan by
  // writing across the whole returned block for a range of tiny capacities.
  for (size_t capacity :
       {size_t{1},
        size_t{8},
        size_t{16},
        size_t{32},
        size_t{48},
        size_t{63},
        size_t{64},
        size_t{65}}) {
    FrameAllocator allocator(MemoryAllocator::GetDefault(), capacity);
    EXPECT_LE(allocator.usableCapacity(), allocator.capacity());

    for (int i = 0; i < 8; ++i) {
      void* p = allocator.allocate(24);
      ASSERT_NE(p, nullptr) << "capacity=" << capacity;
      std::memset(p, 0x5A, 24); // ASan catches an out-of-bounds arena pointer.

      void* q = allocator.allocateAligned(24, 128);
      ASSERT_NE(q, nullptr) << "capacity=" << capacity;
      EXPECT_EQ(reinterpret_cast<uintptr_t>(q) % 128u, 0u);
      std::memset(q, 0x3C, 24);
    }
  }
}

//==============================================================================
TEST(FrameAllocatorTest, MixedAllocatePreserves32ByteAlignment)
{
  FrameAllocator allocator;

  void* a = allocator.allocateAligned(7, 4);
  EXPECT_NE(a, nullptr);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(a) % 4u, 0u);

  void* b = allocator.allocate(64);
  EXPECT_NE(b, nullptr);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(b) % 32u, 0u);

  for (int i = 0; i < 10; ++i) {
    void* small = allocator.allocateAligned(3, 8);
    EXPECT_NE(small, nullptr);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(small) % 8u, 0u);

    void* regular = allocator.allocate(16);
    EXPECT_NE(regular, nullptr);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(regular) % 32u, 0u);
  }
}

//==============================================================================
TEST(FrameAllocatorTest, Reset)
{
  FrameAllocator allocator;
  void* first = allocator.allocate(64);
  EXPECT_NE(first, nullptr);
  EXPECT_GT(allocator.used(), 0u);

  allocator.reset();
  EXPECT_EQ(allocator.used(), 0u);
  EXPECT_EQ(allocator.overflowCount(), 0u);

  void* second = allocator.allocate(64);
  EXPECT_EQ(first, second);
}

//==============================================================================
TEST(FrameAllocatorTest, Overflow)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 256);
  void* ptr = allocator.allocate(300);
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 1u);
  EXPECT_GE(allocator.overflowBytes(), 300u);

  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
  EXPECT_GE(allocator.capacity(), 556u);
}

//==============================================================================
TEST(FrameAllocatorTest, OverflowMultiple)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 128);
  for (int i = 0; i < 5; ++i) {
    void* ptr = allocator.allocate(64);
    EXPECT_NE(ptr, nullptr);
  }
  EXPECT_GE(allocator.overflowCount(), 1u);
  EXPECT_GE(allocator.overflowBytes(), 64u);

  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
  EXPECT_GE(allocator.capacity(), 576u);
}

//==============================================================================
TEST(FrameAllocatorTest, Construct)
{
  struct TestObj
  {
    TestObj(int x, double y) : a(x), b(y) {}

    int a;
    double b;
  };

  FrameAllocator allocator;
  TestObj* obj = allocator.construct<TestObj>(42, 3.5);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(obj->a, 42);
  EXPECT_DOUBLE_EQ(obj->b, 3.5);

  auto* vec = allocator.construct<Eigen::Vector3d>(1.0, 2.0, 3.0);
  ASSERT_NE(vec, nullptr);
  EXPECT_DOUBLE_EQ((*vec)(0), 1.0);
  EXPECT_DOUBLE_EQ((*vec)(1), 2.0);
  EXPECT_DOUBLE_EQ((*vec)(2), 3.0);
}

//==============================================================================
TEST(FrameAllocatorTest, StlAllocatorUsesArena)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 128);
  FrameStlAllocator<int> intAllocator(arena);
  FrameStlAllocator<double> doubleAllocator(intAllocator);

  EXPECT_TRUE(intAllocator == doubleAllocator);

  auto* raw = intAllocator.allocate(2);
  ASSERT_NE(raw, nullptr);
  intAllocator.deallocate(raw, 2);

  std::vector<int, FrameStlAllocator<int>> values(intAllocator);
  values.reserve(4);
  values.push_back(1);
  values.push_back(2);

  EXPECT_EQ(values.size(), 2u);
  EXPECT_GT(arena.used(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, StlAllocatorExposesLightweightStatefulTraits)
{
  using Allocator = FrameStlAllocator<int>;
  using Traits = std::allocator_traits<Allocator>;

  EXPECT_EQ(sizeof(Allocator), sizeof(void*));
  EXPECT_TRUE(std::is_trivially_copy_constructible<Allocator>::value);
  EXPECT_TRUE(std::is_trivially_copy_assignable<Allocator>::value);
  EXPECT_TRUE(std::is_trivially_destructible<Allocator>::value);
  EXPECT_FALSE(Traits::is_always_equal::value);
  EXPECT_TRUE(Traits::propagate_on_container_copy_assignment::value);
  EXPECT_TRUE(Traits::propagate_on_container_move_assignment::value);
  EXPECT_TRUE(Traits::propagate_on_container_swap::value);
}

//==============================================================================
TEST(FrameAllocatorTest, StlAllocatorConstructsAndDestroysNonTrivialObjects)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 512);
  FrameStlAllocator<NonTrivialArenaValue> allocator(arena);
  using Traits = std::allocator_traits<decltype(allocator)>;

  int destroyCount = 0;
  auto* value = allocator.allocate(1);
  ASSERT_NE(value, nullptr);

  Traits::construct(allocator, value, &destroyCount);
  EXPECT_EQ(destroyCount, 0);

  Traits::destroy(allocator, value);
  EXPECT_EQ(destroyCount, 1);
  allocator.deallocate(value, 1);
}

//==============================================================================
TEST(FrameAllocatorTest, StlAllocatorCacheAlignsFrameBackedBlocks)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 256);
  FrameStlAllocator<double> allocator(arena);

  auto* small = allocator.allocate(1);
  ASSERT_NE(small, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(small) % 32u, 0u);

  FrameStlAllocator<CacheLineSizedStlValue> cacheLineAllocator(arena);
  auto* cacheLineSized = cacheLineAllocator.allocate(1);
  ASSERT_NE(cacheLineSized, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(cacheLineSized) % 64u, 0u);
  EXPECT_EQ(arena.overflowCount(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, LargeCacheAlignedAllocationsRotateCacheLineColor)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 16 * 1024);

  auto* first = arena.allocate(4096, 64);
  auto* second = arena.allocate(4096, 64);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  const auto firstAddress = reinterpret_cast<std::uintptr_t>(first);
  const auto secondAddress = reinterpret_cast<std::uintptr_t>(second);
  EXPECT_EQ(firstAddress % 64u, 0u);
  EXPECT_EQ(secondAddress % 64u, 0u);
  EXPECT_NE(firstAddress % 512u, secondAddress % 512u);

  arena.reset();
  auto* afterReset = arena.allocate(4096, 64);
  ASSERT_NE(afterReset, nullptr);
  EXPECT_EQ(
      reinterpret_cast<std::uintptr_t>(afterReset) % 512u, firstAddress % 512u);
}

//==============================================================================
TEST(FrameAllocatorTest, StlAllocatorPreservesOverAlignedValues)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 2048);
  FrameStlAllocator<OverAlignedStlValue> allocator(arena);

  auto* raw = allocator.allocate(2);
  ASSERT_NE(raw, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(raw) % 64u, 0u);
  allocator.deallocate(raw, 2);

  std::vector<OverAlignedStlValue, FrameStlAllocator<OverAlignedStlValue>>
      values(allocator);
  values.resize(2);
  ASSERT_EQ(values.size(), 2u);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(values.data()) % 64u, 0u);

  FrameStlAllocator<WideOverAlignedStlValue> wideAllocator(arena);
  auto* wideRaw = wideAllocator.allocate(2);
  ASSERT_NE(wideRaw, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(wideRaw) % 128u, 0u);
  wideAllocator.deallocate(wideRaw, 2);

  std::vector<
      WideOverAlignedStlValue,
      FrameStlAllocator<WideOverAlignedStlValue>>
      wideValues(wideAllocator);
  wideValues.resize(2);
  ASSERT_EQ(wideValues.size(), 2u);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(wideValues.data()) % 128u, 0u);
  EXPECT_EQ(arena.overflowCount(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, StlAllocatorRejectsCountOverflow)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 512);
  FrameStlAllocator<OverAlignedStlValue> allocator(arena);

  EXPECT_THROW(
      {
        auto* ptr = allocator.allocate(
            std::numeric_limits<std::size_t>::max()
                / sizeof(OverAlignedStlValue)
            + 1);
        (void)ptr;
      },
      std::bad_alloc);
  EXPECT_EQ(arena.used(), 0u);
  EXPECT_EQ(arena.overflowCount(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, BaseAllocator)
{
  auto& defaultAlloc = MemoryAllocator::GetDefault();
  FrameAllocator allocator(defaultAlloc, 512);

  EXPECT_EQ(&allocator.getBaseAllocator(), &defaultAlloc);
  EXPECT_EQ(allocator.capacity(), 512u);
  EXPECT_LE(allocator.usableCapacity(), allocator.capacity());

  void* ptr = allocator.allocate(64);
  EXPECT_NE(ptr, nullptr);
  EXPECT_GT(allocator.used(), 0u);
}

//==============================================================================
TEST(FrameAllocatorTest, MultipleResets)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 128);
  void* ptr = allocator.allocate(512);
  EXPECT_NE(ptr, nullptr);
  allocator.reset();
  const size_t stableCapacity = allocator.capacity();

  for (int i = 0; i < 100; ++i) {
    EXPECT_NE(allocator.allocate(32), nullptr);
    EXPECT_NE(allocator.allocate(96), nullptr);
    EXPECT_NE(allocator.allocate(128), nullptr);
    allocator.reset();
    EXPECT_EQ(allocator.capacity(), stableCapacity);
  }
}

//==============================================================================
TEST(FrameAllocatorTest, ZeroCapacity)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 0);
  EXPECT_EQ(allocator.capacity(), 0u);
  EXPECT_EQ(allocator.usableCapacity(), 0u);
  EXPECT_EQ(allocator.used(), 0u);

  void* ptr = allocator.allocate(64);
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 1u);
  EXPECT_GE(allocator.overflowBytes(), 64u);

  void* aligned = allocator.allocateAligned(32, 64);
  EXPECT_NE(aligned, nullptr);
  const uintptr_t addr = reinterpret_cast<uintptr_t>(aligned);
  EXPECT_EQ(addr % 64u, 0u);
  EXPECT_EQ(allocator.overflowCount(), 2u);
  EXPECT_GE(allocator.overflowBytes(), 96u);

  EXPECT_EQ(allocator.allocate(0), nullptr);
  EXPECT_EQ(allocator.allocateAligned(0, 32), nullptr);
  EXPECT_EQ(allocator.allocateAligned(32, 0), nullptr);
  EXPECT_EQ(allocator.allocateAligned(32, 24), nullptr);
  EXPECT_EQ(allocator.overflowCount(), 2u);

  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
  EXPECT_GT(allocator.capacity(), 0u);
  EXPECT_GT(allocator.usableCapacity(), 0u);

  void* after = allocator.allocate(64);
  EXPECT_NE(after, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
}
