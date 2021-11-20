/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>

#include <gtest/gtest.h>

#include "dart/common/all.hpp"

using namespace dart;
using namespace common;

class NullAllocator : public dart::common::MemoryAllocator
{
public:
  DART_STRING_TYPE(NullAllocator);

  [[nodiscard]] void* allocate(size_t size) noexcept override
  {
    DART_UNUSED(size);
    return nullptr;
  }

  [[nodiscard]] void* allocate_aligned(
      size_t size, size_t alignment) noexcept override
  {
    DART_UNUSED(size, alignment);
    return nullptr;
  }

  void deallocate(void* pointer, size_t size) override
  {
    DART_UNUSED(pointer, size);
  }

  void deallocate_aligned(void* pointer, size_t size) override
  {
    DART_UNUSED(pointer, size);
  }
};

//==============================================================================
TEST(StackAllocatorTest, ConstructorsAndInitialStates)
{
  auto alloc1 = StackAllocator(0);
  EXPECT_EQ(alloc1.get_max_capacity(), 0);
  EXPECT_EQ(alloc1.get_size(), 0);
  EXPECT_EQ(alloc1.get_begin_address(), nullptr);

  auto alloc2 = StackAllocator(64);
  EXPECT_EQ(alloc2.get_max_capacity(), 64);
  EXPECT_EQ(alloc2.get_size(), 0);
  EXPECT_NE(alloc2.get_begin_address(), nullptr);
}

//==============================================================================
TEST(LinearAllocatorTest, Deallocate)
{
  auto null_alloc = NullAllocator();

  auto alloc1 = StackAllocator(0);
  alloc1.deallocate(nullptr, 0);

  auto alloc2 = StackAllocator(8);
  alloc2.deallocate(nullptr, 0);

  auto alloc3 = StackAllocator(8, null_alloc);
  alloc3.deallocate(nullptr, 0);
}

//==============================================================================
TEST(AllocatorTest, TotalSize)
{
  common::set_log_level(common::LogLevel::LOGLEVEL_DEBUG);

  std::uintptr_t size;

  auto allocator1 = StackAllocator(0);
  EXPECT_TRUE(allocator1.allocate(0) == nullptr);
  EXPECT_TRUE(allocator1.allocate(1) == nullptr);
  EXPECT_EQ(allocator1.get_size(), 0);

  auto allocator2 = StackAllocator(16);

  EXPECT_TRUE(allocator2.allocate(0) == nullptr);
  EXPECT_EQ(allocator2.get_size(), 0);
  size = allocator2.get_size();

  void* ptr1 = allocator2.allocate(1);
  EXPECT_TRUE(ptr1 != nullptr);
  EXPECT_TRUE(is_aligned(ptr1, 0));
  EXPECT_TRUE(allocator2.get_size() > size);
  size = allocator2.get_size();

  void* ptr2 = allocator2.allocate_aligned(8, 8);
  EXPECT_TRUE(ptr2 != nullptr);
  EXPECT_TRUE(is_aligned(ptr2, 8));
  EXPECT_TRUE(allocator2.get_size() > size);
  size = allocator2.get_size();

  allocator2.deallocate_aligned(ptr2, 8);
  EXPECT_TRUE(allocator2.get_size() < size);
  size = allocator2.get_size();

  allocator2.deallocate(ptr1, 0);
  EXPECT_TRUE(allocator2.get_size() < size);
  size = allocator2.get_size();
  EXPECT_TRUE(size == 0);
}
