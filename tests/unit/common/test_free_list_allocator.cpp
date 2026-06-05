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

#include <dart/common/callocator.hpp>
#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

#include <cstddef>
#include <cstdint>

using namespace dart;
using namespace common;

namespace {

struct alignas(64) OverAlignedObject
{
  double values[8] = {};
};

struct alignas(std::max_align_t) MaxAlignedObject
{
  unsigned char values[sizeof(std::max_align_t)] = {};
};

class CountingMemoryAllocator final : public MemoryAllocator
{
public:
  std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  void* allocate(size_t bytes) noexcept override
  {
    ++allocationCount;
    return backing.allocate(bytes);
  }

  void* allocate(size_t bytes, size_t alignment) noexcept override
  {
    ++allocationCount;
    return backing.allocate(bytes, alignment);
  }

  void deallocate(void* pointer, size_t bytes) override
  {
    ++deallocationCount;
    backing.deallocate(pointer, bytes);
  }

  void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    ++deallocationCount;
    backing.deallocate(pointer, bytes, alignment);
  }

  CAllocator backing;
  std::size_t allocationCount{0};
  std::size_t deallocationCount{0};
};

} // namespace

//==============================================================================
TEST(FreeListAllocatorTest, Constructors)
{
  auto a = FreeListAllocator::Debug();
  EXPECT_EQ(
      &a.getInternalAllocator().getBaseAllocator(),
      &MemoryAllocator::GetDefault());

  auto b = FreeListAllocator::Debug(MemoryAllocator::GetDefault());
  EXPECT_EQ(
      &b.getInternalAllocator().getBaseAllocator(),
      &MemoryAllocator::GetDefault());

  EXPECT_TRUE(a.isEmpty());
  EXPECT_TRUE(b.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, AllocateAlignedFallsBackToBaseAllocator)
{
  FreeListAllocator allocator;

  auto* ptr = allocator.allocate(sizeof(OverAlignedObject), 64);
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(ptr) % 64u, 0u);

  allocator.deallocate(ptr, sizeof(OverAlignedObject), 64);
}

//==============================================================================
TEST(FreeListAllocatorTest, DestructorReleasesLeakedDelegatedAlignedAllocations)
{
  MemoryAllocatorDebugger<CAllocator> baseAllocator;

  {
    FreeListAllocator allocator(baseAllocator);

    auto* ptr = allocator.allocate(
        sizeof(OverAlignedObject), alignof(OverAlignedObject));
    ASSERT_NE(ptr, nullptr);
    EXPECT_TRUE(baseAllocator.hasAllocated(ptr, sizeof(OverAlignedObject)));
  }

  EXPECT_TRUE(baseAllocator.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, AllocateAlignedSatisfiesMaxAlignment)
{
  FreeListAllocator allocator;

  auto* oddSized = allocator.allocate(1);
  ASSERT_NE(oddSized, nullptr);

  auto* ptr
      = allocator.allocate(sizeof(MaxAlignedObject), alignof(MaxAlignedObject));
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(
      reinterpret_cast<std::uintptr_t>(ptr) % alignof(MaxAlignedObject), 0u);

  allocator.deallocate(
      ptr, sizeof(MaxAlignedObject), alignof(MaxAlignedObject));
  allocator.deallocate(oddSized, 1);
}

//==============================================================================
TEST(FreeListAllocatorTest, AllocateRejectsAlignmentRoundingOverflow)
{
  FreeListAllocator allocator;

  EXPECT_EQ(
      allocator.allocate(std::numeric_limits<std::size_t>::max()), nullptr);
}

//==============================================================================
TEST(FreeListAllocatorTest, InitialBlockHeaderOverflowDoesNotAllocate)
{
  CountingMemoryAllocator baseAllocator;

  FreeListAllocator allocator(
      baseAllocator,
      std::numeric_limits<std::size_t>::max(),
      FreeListAllocator::GrowthPolicy::FixedCapacity);

  EXPECT_EQ(baseAllocator.allocationCount, 0u);
}

//==============================================================================
TEST(FreeListAllocatorTest, Basics)
{
  auto a = FreeListAllocator::Debug();
  EXPECT_TRUE(a.isEmpty());

  // Cannot allocate 0 bytes
  EXPECT_EQ(a.allocate(0), nullptr);

  // Allocate small memory
  auto ptr1 = a.allocate(1);
  EXPECT_NE(ptr1, nullptr);
  EXPECT_TRUE(a.hasAllocated(ptr1, 1));
  EXPECT_FALSE(a.hasAllocated(0, 1));        // incorrect address
  EXPECT_FALSE(a.hasAllocated(ptr1, 1 * 2)); // incorrect size

  a.deallocate(ptr1, 1);

  EXPECT_TRUE(a.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, MemoryLeak)
{
  auto a = FreeListAllocator::Debug();
  EXPECT_TRUE(a.isEmpty());

  auto ptr1 = a.allocate(1);
  EXPECT_NE(ptr1, nullptr);

  EXPECT_FALSE(a.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, MultipleAllocations)
{
  auto a = FreeListAllocator::Debug();

  auto ptr1 = a.allocate(16);
  auto ptr2 = a.allocate(32);
  auto ptr3 = a.allocate(64);

  EXPECT_NE(ptr1, nullptr);
  EXPECT_NE(ptr2, nullptr);
  EXPECT_NE(ptr3, nullptr);
  EXPECT_NE(ptr1, ptr2);
  EXPECT_NE(ptr2, ptr3);
  EXPECT_NE(ptr1, ptr3);

  EXPECT_TRUE(a.hasAllocated(ptr1, 16));
  EXPECT_TRUE(a.hasAllocated(ptr2, 32));
  EXPECT_TRUE(a.hasAllocated(ptr3, 64));

  a.deallocate(ptr2, 32);
  EXPECT_FALSE(a.hasAllocated(ptr2, 32));
  EXPECT_TRUE(a.hasAllocated(ptr1, 16));
  EXPECT_TRUE(a.hasAllocated(ptr3, 64));

  a.deallocate(ptr1, 16);
  a.deallocate(ptr3, 64);

  EXPECT_TRUE(a.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, AllocateAfterDeallocate)
{
  auto a = FreeListAllocator::Debug();

  auto ptr1 = a.allocate(128);
  EXPECT_NE(ptr1, nullptr);

  a.deallocate(ptr1, 128);
  EXPECT_TRUE(a.isEmpty());

  auto ptr2 = a.allocate(64);
  EXPECT_NE(ptr2, nullptr);

  a.deallocate(ptr2, 64);
  EXPECT_TRUE(a.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, RawAllocatorExpandsWhenCurrentBlocksAreFull)
{
  FreeListAllocator allocator(MemoryAllocator::GetDefault(), 64);
  EXPECT_EQ(
      allocator.getGrowthPolicy(), FreeListAllocator::GrowthPolicy::Expand);

  auto* first = allocator.allocate(64);
  ASSERT_NE(first, nullptr);

  auto* second = allocator.allocate(16);
  ASSERT_NE(second, nullptr);

  allocator.deallocate(first, 64);
  allocator.deallocate(second, 16);
}

//==============================================================================
TEST(FreeListAllocatorTest, FixedCapacityDoesNotGrowWhenBlocksAreFull)
{
  CountingMemoryAllocator baseAllocator;
  FreeListAllocator allocator(
      baseAllocator, 64, FreeListAllocator::GrowthPolicy::FixedCapacity);
  EXPECT_EQ(
      allocator.getGrowthPolicy(),
      FreeListAllocator::GrowthPolicy::FixedCapacity);

  const auto initialBaseAllocations = baseAllocator.allocationCount;
  auto* first = allocator.allocate(64);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(baseAllocator.allocationCount, initialBaseAllocations);

  EXPECT_EQ(allocator.allocate(16), nullptr);
  EXPECT_EQ(baseAllocator.allocationCount, initialBaseAllocations);

  allocator.deallocate(first, 64);
}

//==============================================================================
TEST(FreeListAllocatorTest, FixedCapacitySatisfiesOverAlignedAllocation)
{
  CountingMemoryAllocator baseAllocator;
  FreeListAllocator allocator(
      baseAllocator, 1024, FreeListAllocator::GrowthPolicy::FixedCapacity);

  const auto initialBaseAllocations = baseAllocator.allocationCount;
  auto* ptr = allocator.allocate(
      sizeof(OverAlignedObject), alignof(OverAlignedObject));
  ASSERT_NE(ptr, nullptr);
  EXPECT_EQ(
      reinterpret_cast<std::uintptr_t>(ptr) % alignof(OverAlignedObject), 0u);
  EXPECT_EQ(baseAllocator.allocationCount, initialBaseAllocations);

  allocator.deallocate(
      ptr, sizeof(OverAlignedObject), alignof(OverAlignedObject));
  EXPECT_EQ(baseAllocator.deallocationCount, 0u);
}

//==============================================================================
TEST(FreeListAllocatorTest, FixedCapacityAlignedAllocationDoesNotGrow)
{
  CountingMemoryAllocator baseAllocator;
  FreeListAllocator allocator(
      baseAllocator, 128, FreeListAllocator::GrowthPolicy::FixedCapacity);

  const auto initialBaseAllocations = baseAllocator.allocationCount;
  EXPECT_EQ(allocator.allocate(1024, 64), nullptr);
  EXPECT_EQ(baseAllocator.allocationCount, initialBaseAllocations);
}

//==============================================================================
TEST(FreeListAllocatorTest, RawAllocatorNoOpDeallocateAndPrint)
{
  FreeListAllocator allocator(MemoryAllocator::GetDefault(), 128);

  allocator.deallocate(nullptr, 1);

  auto* ptr = allocator.allocate(16);
  ASSERT_NE(ptr, nullptr);

  allocator.deallocate(ptr, 0);

  std::ostringstream oss;
  allocator.print(oss);
  EXPECT_NE(oss.str().find("[FreeListAllocator]"), std::string::npos);

  allocator.deallocate(ptr, 16);
}

//==============================================================================
TEST(FreeListAllocatorTest, PrintOutput)
{
  auto a = FreeListAllocator::Debug();

  auto ptr = a.allocate(256);
  EXPECT_NE(ptr, nullptr);

  std::ostringstream oss;
  a.print(oss);

  std::string output = oss.str();
  EXPECT_FALSE(output.empty());

  a.deallocate(ptr, 256);
}

//==============================================================================
TEST(FreeListAllocatorTest, GetBaseAllocator)
{
  auto a = FreeListAllocator::Debug();

  const auto& constAllocator = a;
  const MemoryAllocator& baseConst
      = constAllocator.getInternalAllocator().getBaseAllocator();
  EXPECT_EQ(&baseConst, &MemoryAllocator::GetDefault());

  MemoryAllocator& baseNonConst = a.getInternalAllocator().getBaseAllocator();
  EXPECT_EQ(&baseNonConst, &MemoryAllocator::GetDefault());
}

//==============================================================================
TEST(FreeListAllocatorTest, DeallocateNullptrIsNoOp)
{
  auto a = FreeListAllocator::Debug();

  a.deallocate(nullptr, 0);
  EXPECT_TRUE(a.isEmpty());

  auto ptr = a.allocate(32);
  a.deallocate(nullptr, 0);
  EXPECT_TRUE(a.hasAllocated(ptr, 32));

  a.deallocate(ptr, 32);
  EXPECT_TRUE(a.isEmpty());
}

//==============================================================================
TEST(FreeListAllocatorTest, VariousSizes)
{
  auto a = FreeListAllocator::Debug();

  std::vector<std::pair<void*, std::size_t>> allocations;

  for (std::size_t size :
       {1, 7, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128}) {
    auto ptr = a.allocate(size);
    EXPECT_NE(ptr, nullptr);
    EXPECT_TRUE(a.hasAllocated(ptr, size));
    allocations.emplace_back(ptr, size);
  }

  for (const auto& [ptr, size] : allocations) {
    a.deallocate(ptr, size);
  }

  EXPECT_TRUE(a.isEmpty());
}
