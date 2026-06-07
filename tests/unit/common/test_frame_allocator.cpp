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

#include <dart/common/frame_allocator.hpp>
#include <dart/common/memory_allocator.hpp>

#include <Eigen/Core>
#include <entt/entity/registry.hpp>
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include <cstddef>
#include <cstdint>

using namespace dart::common;

struct alignas(64) OverAlignedStlValue
{
  double values[8] = {};
};

struct CacheLineSizedStlValue
{
  double values[8] = {};
};

struct ArenaRegistryTag
{
};

struct ArenaRegistryVelocity
{
  double values[3] = {};
};

template <typename Registry>
void reserveArenaRegistryStorage(Registry& registry, const size_t entityCount)
{
  registry.template storage<ArenaRegistryTag>().reserve(entityCount);
  registry.template storage<ArenaRegistryVelocity>().reserve(entityCount);
  registry.template storage<OverAlignedStlValue>().reserve(entityCount);
}

template <typename Registry>
void runArenaRegistryChurn(
    Registry& registry,
    std::vector<entt::entity>& entities,
    const size_t entityCount)
{
  for (size_t i = 0; i < entityCount; ++i) {
    const auto entity = registry.create();
    entities[i] = entity;
    registry.template emplace<ArenaRegistryTag>(entity);
    auto& velocity = registry.template emplace<ArenaRegistryVelocity>(entity);
    velocity.values[0] = static_cast<double>(i);
    auto& value = registry.template emplace<OverAlignedStlValue>(entity);
    value.values[0] = static_cast<double>(i + 1);
  }

  double total = 0.0;
  for (size_t i = 0; i < entityCount; ++i) {
    const auto entity = entities[i];
    total += registry.template get<ArenaRegistryVelocity>(entity).values[0];
    total += registry.template get<OverAlignedStlValue>(entity).values[0];
  }
  EXPECT_GT(total, 0.0);

  for (size_t i = entityCount; i > 0; --i) {
    registry.destroy(entities[i - 1]);
  }
}

class FrameAllocatorTest : public ::testing::Test
{
};

//=============================================================================
TEST_F(FrameAllocatorTest, Construction)
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

//=============================================================================
TEST_F(FrameAllocatorTest, BasicAllocate)
{
  FrameAllocator allocator;
  size_t previousUsed = 0;
  const auto sizes = std::to_array<size_t>({8, 64, 256, 1024});
  for (size_t size : sizes) {
    void* ptr = allocator.allocate(size);
    EXPECT_NE(ptr, nullptr);
    EXPECT_GT(allocator.used(), previousUsed);
    previousUsed = allocator.used();
  }
}

//=============================================================================
TEST_F(FrameAllocatorTest, UsableCapacityMatchesFastPathBudget)
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

//=============================================================================
TEST_F(FrameAllocatorTest, Alignment)
{
  FrameAllocator allocator;
  const auto alignments = std::to_array<size_t>({16, 32, 64});
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

//=============================================================================
TEST_F(FrameAllocatorTest, RejectsInvalidAlignment)
{
  FrameAllocator allocator;
  EXPECT_EQ(allocator.allocateAligned(32, 24), nullptr);
  EXPECT_EQ(allocator.allocate(32, 24), nullptr);
  EXPECT_EQ(allocator.used(), 0u);
  EXPECT_EQ(allocator.overflowCount(), 0u);
}

//=============================================================================
TEST_F(FrameAllocatorTest, RejectsOverflowingRequestsWithoutChangingArena)
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

//=============================================================================
TEST_F(FrameAllocatorTest, MixedAllocatePreserves32ByteAlignment)
{
  FrameAllocator allocator;

  // allocateAligned with small alignment must not break allocate()'s
  // 32-byte alignment invariant.
  void* a = allocator.allocateAligned(7, 4);
  EXPECT_NE(a, nullptr);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(a) % 4, 0u);

  void* b = allocator.allocate(64);
  EXPECT_NE(b, nullptr);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(b) % 32, 0u);

  // Interleave several small-aligned and default allocations.
  for (int i = 0; i < 10; ++i) {
    void* small = allocator.allocateAligned(3, 8);
    EXPECT_NE(small, nullptr);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(small) % 8, 0u);

    void* regular = allocator.allocate(16);
    EXPECT_NE(regular, nullptr);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(regular) % 32, 0u);
  }
}

//=============================================================================
TEST_F(FrameAllocatorTest, Reset)
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

//=============================================================================
TEST_F(FrameAllocatorTest, Overflow)
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

//=============================================================================
TEST_F(FrameAllocatorTest, OverflowMultiple)
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

//=============================================================================
TEST_F(FrameAllocatorTest, Construct)
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

//=============================================================================
TEST_F(FrameAllocatorTest, StlAllocatorUsesArena)
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

//=============================================================================
TEST_F(FrameAllocatorTest, StlAllocatorCacheAlignsFrameBackedBlocks)
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

//=============================================================================
TEST_F(FrameAllocatorTest, StlAllocatorPreservesOverAlignedValues)
{
  FrameAllocator arena(MemoryAllocator::GetDefault(), 512);
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
}

//=============================================================================
TEST_F(FrameAllocatorTest, StlAllocatorRejectsCountOverflow)
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

//=============================================================================
TEST_F(
    FrameAllocatorTest, FrameStlAllocatorSupportsReservedEnttRegistryNoGrowth)
{
  constexpr size_t entityCount = 128;
  FrameAllocator arena(
      MemoryAllocator::GetDefault(), entityCount * 4096 + 1024 * 1024);
  FrameStlAllocator<entt::entity> allocator(arena);
  entt::basic_registry<entt::entity, FrameStlAllocator<entt::entity>> registry(
      allocator);
  std::vector<entt::entity> entities(entityCount);

  reserveArenaRegistryStorage(registry, entityCount);
  runArenaRegistryChurn(registry, entities, entityCount);

  const size_t usedAfterPrewarm = arena.used();
  const size_t overflowCountAfterPrewarm = arena.overflowCount();
  const size_t overflowBytesAfterPrewarm = arena.overflowBytes();
  runArenaRegistryChurn(registry, entities, entityCount);

  EXPECT_EQ(arena.used(), usedAfterPrewarm);
  EXPECT_EQ(arena.overflowCount(), overflowCountAfterPrewarm);
  EXPECT_EQ(arena.overflowBytes(), overflowBytesAfterPrewarm);
}

//=============================================================================
TEST_F(FrameAllocatorTest, BaseAllocator)
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

//=============================================================================
TEST_F(FrameAllocatorTest, MultipleResets)
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

//=============================================================================
TEST_F(FrameAllocatorTest, ZeroCapacity)
{
  FrameAllocator allocator(MemoryAllocator::GetDefault(), 0);
  EXPECT_EQ(allocator.capacity(), 0u);
  EXPECT_EQ(allocator.usableCapacity(), 0u);
  EXPECT_EQ(allocator.used(), 0u);

  // allocate() must not crash — it falls back to overflow allocation.
  void* ptr = allocator.allocate(64);
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 1u);
  EXPECT_GE(allocator.overflowBytes(), 64u);

  // allocateAligned() must not crash either.
  void* aligned = allocator.allocateAligned(32, 64);
  EXPECT_NE(aligned, nullptr);
  const uintptr_t addr = reinterpret_cast<uintptr_t>(aligned);
  EXPECT_EQ(addr % 64, 0u);
  EXPECT_EQ(allocator.overflowCount(), 2u);
  EXPECT_GE(allocator.overflowBytes(), 96u);

  // Zero-byte requests return nullptr, consistent with other DART allocators.
  EXPECT_EQ(allocator.allocate(0), nullptr);
  EXPECT_EQ(allocator.allocateAligned(0, 32), nullptr);
  EXPECT_EQ(allocator.allocateAligned(32, 0), nullptr);
  EXPECT_EQ(allocator.allocateAligned(32, 24), nullptr);
  EXPECT_EQ(allocator.overflowCount(), 2u);

  // reset() must not crash; it should grow the buffer via resetSlow().
  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
  EXPECT_GT(allocator.capacity(), 0u);
  EXPECT_GT(allocator.usableCapacity(), 0u);

  // After reset, the buffer is now valid — fast path should work.
  void* after = allocator.allocate(64);
  EXPECT_NE(after, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_EQ(allocator.overflowBytes(), 0u);
}
