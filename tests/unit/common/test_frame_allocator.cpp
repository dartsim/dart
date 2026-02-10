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
#include <gtest/gtest.h>

#include <cstdint>

using namespace dart::common;

class FrameAllocatorTest : public ::testing::Test
{
};

//=============================================================================
TEST_F(FrameAllocatorTest, Construction)
{
  FrameAllocator allocator;
  EXPECT_EQ(allocator.capacity(), 65536u);
  EXPECT_EQ(allocator.used(), 0u);
  EXPECT_EQ(allocator.overflowCount(), 0u);
}

//=============================================================================
TEST_F(FrameAllocatorTest, BasicAllocate)
{
  FrameAllocator allocator;
  size_t previousUsed = 0;
  const size_t sizes[] = {8, 64, 256, 1024};
  for (size_t size : sizes) {
    void* ptr = allocator.allocate(size);
    EXPECT_NE(ptr, nullptr);
    EXPECT_GT(allocator.used(), previousUsed);
    previousUsed = allocator.used();
  }
}

//=============================================================================
TEST_F(FrameAllocatorTest, Alignment)
{
  FrameAllocator allocator;
  const size_t alignments[] = {16, 32, 64};
  for (size_t alignment : alignments) {
    void* ptr = allocator.allocateAligned(32, alignment);
    EXPECT_NE(ptr, nullptr);
    const uintptr_t address = reinterpret_cast<uintptr_t>(ptr);
    EXPECT_EQ(address % alignment, 0u);
  }
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

  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
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

  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
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
TEST_F(FrameAllocatorTest, BaseAllocator)
{
  auto& defaultAlloc = MemoryAllocator::GetDefault();
  FrameAllocator allocator(defaultAlloc, 512);

  EXPECT_EQ(&allocator.getBaseAllocator(), &defaultAlloc);
  EXPECT_EQ(allocator.capacity(), 512u);

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
  EXPECT_EQ(allocator.used(), 0u);

  // allocate() must not crash — it falls back to overflow allocation.
  void* ptr = allocator.allocate(64);
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 1u);

  // allocateAligned() must not crash either.
  void* aligned = allocator.allocateAligned(32, 64);
  EXPECT_NE(aligned, nullptr);
  const uintptr_t addr = reinterpret_cast<uintptr_t>(aligned);
  EXPECT_EQ(addr % 64, 0u);
  EXPECT_EQ(allocator.overflowCount(), 2u);

  // Zero-byte requests return nullptr, consistent with other DART allocators.
  EXPECT_EQ(allocator.allocate(0), nullptr);
  EXPECT_EQ(allocator.allocateAligned(0, 32), nullptr);
  EXPECT_EQ(allocator.allocateAligned(32, 0), nullptr);

  // reset() must not crash; it should grow the buffer via resetSlow().
  allocator.reset();
  EXPECT_EQ(allocator.overflowCount(), 0u);
  EXPECT_GT(allocator.capacity(), 0u);

  // After reset, the buffer is now valid — fast path should work.
  void* after = allocator.allocate(64);
  EXPECT_NE(after, nullptr);
  EXPECT_EQ(allocator.overflowCount(), 0u);
}
