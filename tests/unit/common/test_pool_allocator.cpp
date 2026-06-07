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

#include <dart/common/fixed_pool_allocator.hpp>
#include <dart/common/pool_allocator.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <utility>
#include <vector>

#include <cstdint>

using namespace dart;
using namespace common;

namespace {

struct alignas(64) OverAlignedObject
{
  double values[8] = {};
};

class CountingMemoryAllocator final : public MemoryAllocator
{
public:
  [[nodiscard]] std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  [[nodiscard]] void* allocate(size_t bytes) noexcept override
  {
    ++allocationAttempts;
    if (allocationCount >= allocationLimit) {
      return nullptr;
    }

    void* ptr = MemoryAllocator::GetDefault().allocate(bytes);
    if (ptr != nullptr) {
      ++allocationCount;
    }
    return ptr;
  }

  [[nodiscard]] void* allocate(size_t bytes, size_t alignment) noexcept override
  {
    ++allocationAttempts;
    ++alignedAllocationAttempts;
    if (allocationCount >= allocationLimit) {
      return nullptr;
    }

    void* ptr = MemoryAllocator::GetDefault().allocate(bytes, alignment);
    if (ptr != nullptr) {
      ++allocationCount;
      ++alignedAllocationCount;
    }
    return ptr;
  }

  void deallocate(void* pointer, size_t bytes) override
  {
    if (pointer == nullptr) {
      return;
    }

    ++deallocationCount;
    MemoryAllocator::GetDefault().deallocate(pointer, bytes);
  }

  void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    if (pointer == nullptr) {
      return;
    }

    ++deallocationCount;
    ++alignedDeallocationCount;
    MemoryAllocator::GetDefault().deallocate(pointer, bytes, alignment);
  }

  size_t allocationLimit = std::numeric_limits<size_t>::max();
  size_t allocationAttempts = 0;
  size_t alignedAllocationAttempts = 0;
  size_t allocationCount = 0;
  size_t alignedAllocationCount = 0;
  size_t deallocationCount = 0;
  size_t alignedDeallocationCount = 0;
};

} // namespace

//==============================================================================
TEST(PoolAllocatorTest, Constructors)
{
  auto a = PoolAllocator::Debug();
  EXPECT_EQ(
      &a.getInternalAllocator().getBaseAllocator(),
      &MemoryAllocator::GetDefault());

  auto b = PoolAllocator::Debug(MemoryAllocator::GetDefault());
  EXPECT_EQ(
      &b.getInternalAllocator().getBaseAllocator(),
      &MemoryAllocator::GetDefault());

  EXPECT_EQ(b.getInternalAllocator().getNumAllocatedMemoryBlocks(), 0);

  EXPECT_TRUE(a.isEmpty());
  EXPECT_TRUE(b.isEmpty());
}

//==============================================================================
TEST(PoolAllocatorTest, AllocateAlignedUsesAlignedSlots)
{
  PoolAllocator allocator;
  EXPECT_TRUE(allocator.isDiagnosticsEnabled());

  auto* ptr = allocator.allocate(sizeof(OverAlignedObject), 64);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(ptr) % 64u, 0u);

  allocator.deallocate(ptr, sizeof(OverAlignedObject), 64);
}

//==============================================================================
TEST(PoolAllocatorTest, RejectsInvalidSizesWithoutBaseAllocation)
{
  CountingMemoryAllocator base;
  PoolAllocator allocator(base);
  const size_t allocationAttemptsAfterConstruction = base.allocationAttempts;

  EXPECT_EQ(allocator.allocate(std::numeric_limits<size_t>::max()), nullptr);
  EXPECT_EQ(allocator.allocate(16, 3), nullptr);
  EXPECT_EQ(
      allocator.allocate(std::numeric_limits<size_t>::max(), 64), nullptr);
  EXPECT_EQ(base.allocationAttempts, allocationAttemptsAfterConstruction);
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

//==============================================================================
TEST(PoolAllocatorTest, FailedBaseAllocationDoesNotRecordDiagnostics)
{
  CountingMemoryAllocator base;
  base.allocationLimit = 0;
  PoolAllocator allocator(base);

  EXPECT_EQ(allocator.allocate(2048, 64), nullptr);
  EXPECT_EQ(base.alignedAllocationAttempts, 1u);
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

//==============================================================================
TEST(PoolAllocatorTest, DiagnosticsTrackLivePeakAndSlotReuse)
{
  PoolAllocator allocator;

  void* first = allocator.allocate(16);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(allocator.getAllocatedSize(), 16u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 16u);
  EXPECT_EQ(allocator.getAllocationCount(), 1u);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 1);

  void* second = allocator.allocate(24);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(allocator.getAllocatedSize(), 40u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 40u);
  EXPECT_EQ(allocator.getAllocationCount(), 2u);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 2);

  allocator.deallocate(first, 16);
  EXPECT_EQ(allocator.getAllocatedSize(), 24u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 40u);
  EXPECT_EQ(allocator.getAllocationCount(), 1u);

  void* reused = allocator.allocate(16);
  EXPECT_EQ(reused, first);
  EXPECT_EQ(allocator.getAllocatedSize(), 40u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 40u);
  EXPECT_EQ(allocator.getAllocationCount(), 2u);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 2);

  allocator.deallocate(second, 24);
  allocator.deallocate(reused, 16);
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

//==============================================================================
TEST(PoolAllocatorTest, DiagnosticsCanBeDisabledForHotPathUse)
{
  PoolAllocator allocator(PoolAllocator::DiagnosticsPolicy::Disabled);
  EXPECT_FALSE(allocator.isDiagnosticsEnabled());

  void* ptr = allocator.allocate(16);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);

  allocator.deallocate(ptr, 16);
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);

  void* hotPathPtr = allocator.allocateUntracked(24);
  ASSERT_NE(hotPathPtr, nullptr);
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
  allocator.deallocateUntracked(hotPathPtr, 24);

  void* alignedHotPathPtr
      = allocator.allocateUntracked(sizeof(OverAlignedObject), 64);
  ASSERT_NE(alignedHotPathPtr, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(alignedHotPathPtr) % 64u, 0u);
  allocator.deallocateUntracked(
      alignedHotPathPtr, sizeof(OverAlignedObject), 64);

  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, Constructors)
{
  FixedPoolAllocator allocator(32);
  EXPECT_EQ(&allocator.getBaseAllocator(), &MemoryAllocator::GetDefault());
  EXPECT_EQ(allocator.getUnitSize(), 32u);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);

  FixedPoolAllocator zeroSizedAllocator(0);
  EXPECT_EQ(zeroSizedAllocator.getUnitSize(), 0u);
  EXPECT_EQ(zeroSizedAllocator.allocate(1), nullptr);
  EXPECT_EQ(zeroSizedAllocator.allocate(), nullptr);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, PowerOfTwoSlotsUseCacheFriendlyStride)
{
  FixedPoolAllocator allocator(256);

  EXPECT_EQ(allocator.getUnitSize(), 288u);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, ZeroByteRequestsDoNotAllocateBlocks)
{
  CountingMemoryAllocator base;
  FixedPoolAllocator allocator(32, base, 32 * 4);
  const size_t allocationsAfterConstruction = base.allocationCount;

  EXPECT_EQ(allocator.allocate(0), nullptr);
  EXPECT_EQ(allocator.allocate(0, 64), nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
  EXPECT_EQ(base.allocationCount, allocationsAfterConstruction);

  allocator.deallocate(nullptr);
  allocator.deallocate(nullptr, 32);
  allocator.deallocate(nullptr, 32, 64);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, RejectsOverflowingAlignedRequests)
{
  CountingMemoryAllocator base;
  FixedPoolAllocator allocator(32, base, 32 * 4);
  const size_t allocationAttemptsAfterConstruction = base.allocationAttempts;

  EXPECT_EQ(
      allocator.allocate(std::numeric_limits<size_t>::max(), 64), nullptr);
  EXPECT_EQ(base.allocationAttempts, allocationAttemptsAfterConstruction);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, ExtremeUnitSizeConstructor)
{
  constexpr size_t unitSize = size_t{1}
                              << (std::numeric_limits<size_t>::digits - 1);
  FixedPoolAllocator allocator(unitSize, MemoryAllocator::GetDefault(), 0);
  EXPECT_EQ(allocator.getUnitSize(), unitSize);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, AllocateUsesFixedSizeBlocks)
{
  FixedPoolAllocator allocator(32, MemoryAllocator::GetDefault(), 32 * 4);

  void* ptr1 = allocator.allocate(1);
  void* ptr2 = allocator.allocate(32);
  ASSERT_NE(ptr1, nullptr);
  ASSERT_NE(ptr2, nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 1);

  allocator.deallocate(ptr1, 1);
  allocator.deallocate(ptr2, 32);

  void* ptr3 = allocator.allocate();
  ASSERT_NE(ptr3, nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 1);
  allocator.deallocate(ptr3);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, ReusesFreedSlotWithoutGrowing)
{
  FixedPoolAllocator allocator(32, MemoryAllocator::GetDefault(), 32 * 4);

  void* ptr1 = allocator.allocate();
  ASSERT_NE(ptr1, nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 1);

  allocator.deallocate(ptr1);

  void* ptr2 = allocator.allocate();
  EXPECT_EQ(ptr2, ptr1);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 1);
  allocator.deallocate(ptr2);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, AllocateAlignedUsesAlignedSlots)
{
  FixedPoolAllocator allocator(
      sizeof(OverAlignedObject), MemoryAllocator::GetDefault(), 4096);

  auto* ptr = allocator.allocate(sizeof(OverAlignedObject), 64);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(ptr) % 64u, 0u);

  allocator.deallocate(ptr, sizeof(OverAlignedObject), 64);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, OverSizedRequestsUseBaseAllocator)
{
  CountingMemoryAllocator base;
  FixedPoolAllocator allocator(32, base, 32 * 4);
  const size_t allocationsAfterConstruction = base.allocationCount;

  void* ptr = allocator.allocate(64);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
  EXPECT_EQ(base.allocationCount, allocationsAfterConstruction + 1);

  allocator.deallocate(ptr, 64);
  EXPECT_EQ(base.deallocationCount, 1u);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, OverAlignedRequestsUseBaseAllocator)
{
  CountingMemoryAllocator base;
  FixedPoolAllocator allocator(32, base, 32 * 4);
  const size_t allocationsAfterConstruction = base.allocationCount;

  void* ptr = allocator.allocate(16, 64);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(ptr) % 64u, 0u);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
  EXPECT_EQ(base.allocationCount, allocationsAfterConstruction + 1);
  EXPECT_EQ(base.alignedAllocationCount, 1u);

  allocator.deallocate(ptr, 16, 64);
  EXPECT_EQ(base.deallocationCount, 1u);
  EXPECT_EQ(base.alignedDeallocationCount, 1u);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, RejectsInvalidAlignedRequests)
{
  CountingMemoryAllocator base;
  FixedPoolAllocator allocator(32, base, 32 * 4);
  const size_t allocationAttemptsAfterConstruction = base.allocationAttempts;

  EXPECT_EQ(allocator.allocate(16, 3), nullptr);
  EXPECT_EQ(base.allocationAttempts, allocationAttemptsAfterConstruction);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, HandlesBaseAllocatorFailureForBlockTable)
{
  CountingMemoryAllocator base;
  base.allocationLimit = 0;

  FixedPoolAllocator allocator(32, base, 32 * 4);
  EXPECT_EQ(allocator.allocate(), nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
  EXPECT_EQ(base.allocationAttempts, 1u);
  EXPECT_EQ(base.allocationCount, 0u);
  EXPECT_EQ(base.deallocationCount, 0u);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, HandlesBaseAllocatorFailureForMemoryBlock)
{
  CountingMemoryAllocator base;
  base.allocationLimit = 1;

  FixedPoolAllocator allocator(32, base, 32 * 4);
  EXPECT_EQ(allocator.allocate(), nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 0);
  EXPECT_EQ(base.allocationAttempts, 2u);
  EXPECT_EQ(base.allocationCount, 1u);
  EXPECT_EQ(base.deallocationCount, 0u);
}

//==============================================================================
TEST(FixedPoolAllocatorTest, KeepsExistingBlocksWhenBlockTableGrowthFails)
{
  CountingMemoryAllocator base;
  base.allocationLimit = 65;
  FixedPoolAllocator allocator(32, base, 32);

  std::vector<void*> allocations;
  allocations.reserve(64);
  for (int i = 0; i < 64; ++i) {
    void* ptr = allocator.allocate();
    ASSERT_NE(ptr, nullptr) << "allocation " << i;
    allocations.push_back(ptr);
  }

  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 64);
  EXPECT_EQ(allocator.allocate(), nullptr);
  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 64);

  for (void* ptr : allocations) {
    allocator.deallocate(ptr);
  }
}

//==============================================================================
TEST(FixedPoolAllocatorTest, RawAllocatorGrowsMemoryBlockTable)
{
  FixedPoolAllocator allocator(32, MemoryAllocator::GetDefault(), 32);

  std::vector<void*> allocations;
  allocations.reserve(65);
  for (int i = 0; i < 65; ++i) {
    void* ptr = allocator.allocate();
    ASSERT_NE(ptr, nullptr) << "allocation " << i;
    allocations.push_back(ptr);
  }

  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 65);

  for (void* ptr : allocations) {
    allocator.deallocate(ptr);
  }
}

//==============================================================================
TEST(FixedPoolAllocatorTest, DebugRejectsMismatchedAndDoubleFree)
{
  auto allocator = FixedPoolAllocator::Debug(32);

  void* ptr = allocator.allocate(16);
  ASSERT_NE(ptr, nullptr);
  EXPECT_TRUE(allocator.hasAllocated(ptr, 16));

  allocator.deallocate(ptr, 8);
  EXPECT_TRUE(allocator.hasAllocated(ptr, 16));
  EXPECT_FALSE(allocator.isEmpty());

  allocator.deallocate(ptr, 16);
  EXPECT_TRUE(allocator.isEmpty());

  allocator.deallocate(ptr, 16);
  EXPECT_TRUE(allocator.isEmpty());
}

//==============================================================================
TEST(PoolAllocatorTest, Allocate)
{
  auto a = PoolAllocator::Debug();
  EXPECT_TRUE(a.isEmpty());

  // Cannot allocate 0 bytes
  EXPECT_EQ(a.allocate(0), nullptr);

  // Allocate small memory
  auto ptr1 = a.allocate(1);
  EXPECT_NE(ptr1, nullptr);
  EXPECT_TRUE(a.hasAllocated(ptr1, 1));
  EXPECT_FALSE(a.hasAllocated(0, 1));        // incorrect address
  EXPECT_FALSE(a.hasAllocated(ptr1, 1 * 2)); // incorrect size
  EXPECT_EQ(a.getInternalAllocator().getNumAllocatedMemoryBlocks(), 1);

  // Allocate the same size, which doesn't increase the number of memory block
  auto ptr2 = a.allocate(1);
  EXPECT_NE(ptr2, nullptr);
  EXPECT_EQ(a.getInternalAllocator().getNumAllocatedMemoryBlocks(), 1);

  // Allocate different size
  auto ptr3 = a.allocate(64);
  EXPECT_NE(ptr3, nullptr);
  EXPECT_EQ(a.getInternalAllocator().getNumAllocatedMemoryBlocks(), 2);

  // Allocate memory of the max size (= 1024)
  auto ptr4 = a.allocate(1024);
  EXPECT_NE(ptr4, nullptr);
  EXPECT_TRUE(a.hasAllocated(ptr4, 1024));
  EXPECT_FALSE(a.hasAllocated(0, 1024));
  EXPECT_FALSE(a.hasAllocated(ptr4, 1024 * 2));
  EXPECT_EQ(a.getInternalAllocator().getNumAllocatedMemoryBlocks(), 3);

  // Allocate oversized memory (> 1024)
  auto ptr5 = a.allocate(2048);
  EXPECT_NE(ptr5, nullptr);
  EXPECT_TRUE(a.hasAllocated(ptr5, 2048));
  EXPECT_FALSE(a.hasAllocated(0, 2048));
  EXPECT_FALSE(a.hasAllocated(ptr5, 2048 * 2));
  EXPECT_EQ(a.getInternalAllocator().getNumAllocatedMemoryBlocks(), 3);

  EXPECT_FALSE(a.isEmpty());

  a.deallocate(ptr1, 1);
  a.deallocate(ptr2, 1);
  a.deallocate(ptr3, 64);
  a.deallocate(ptr4, 1024);
  a.deallocate(ptr5, 2048);

  EXPECT_TRUE(a.isEmpty());
}

//==============================================================================
TEST(PoolAllocatorTest, DeallocateNullptr)
{
  auto a = PoolAllocator::Debug();

  // Deallocate nullptr should be safe (no-op)
  a.deallocate(nullptr, 16);
  EXPECT_TRUE(a.isEmpty());
}

//==============================================================================
TEST(PoolAllocatorTest, DebugRejectsMismatchedAndDoubleFree)
{
  auto allocator = PoolAllocator::Debug();

  void* ptr = allocator.allocate(16);
  ASSERT_NE(ptr, nullptr);
  EXPECT_TRUE(allocator.hasAllocated(ptr, 16));

  allocator.deallocate(ptr, 8);
  EXPECT_TRUE(allocator.hasAllocated(ptr, 16));
  EXPECT_FALSE(allocator.isEmpty());

  allocator.deallocate(ptr, 16);
  EXPECT_TRUE(allocator.isEmpty());

  allocator.deallocate(ptr, 16);
  EXPECT_TRUE(allocator.isEmpty());
}

//==============================================================================
TEST(PoolAllocatorTest, RawAllocatorGrowsMemoryBlockTable)
{
  PoolAllocator allocator;

  const PoolAllocator& constAllocator = allocator;
  EXPECT_EQ(&constAllocator.getBaseAllocator(), &MemoryAllocator::GetDefault());

  std::vector<std::pair<void*, std::size_t>> allocations;
  for (std::size_t size = 8; size <= 8 * 65; size += 8) {
    auto* ptr = allocator.allocate(size);
    ASSERT_NE(ptr, nullptr);
    allocations.emplace_back(ptr, size);
  }

  EXPECT_EQ(allocator.getNumAllocatedMemoryBlocks(), 65);

  for (const auto& [ptr, size] : allocations) {
    allocator.deallocate(ptr, size);
  }
}

//==============================================================================
TEST(PoolAllocatorTest, Print)
{
  auto a = PoolAllocator::Debug();

  // Test print with indent = 0
  std::ostringstream oss1;
  a.getInternalAllocator().print(oss1, 0);
  EXPECT_NE(oss1.str().find("PoolAllocator"), std::string::npos);
  EXPECT_NE(oss1.str().find("allocated_memory_block_count"), std::string::npos);

  // Test print with indent != 0
  std::ostringstream oss2;
  a.getInternalAllocator().print(oss2, 2);
  EXPECT_NE(oss2.str().find("type:"), std::string::npos);
  EXPECT_NE(oss2.str().find("base_allocator"), std::string::npos);
}
