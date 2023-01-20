/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/common/allocator/AlignedAllocatorFreeList.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

//==============================================================================
class AlignedAllocatorFreeListTest : public ::testing::Test
{
protected:
  AlignedAllocatorFreeList::Debug mAllocator;
};

//==============================================================================
class MyObject
{
public:
  MyObject(const std::string& str, int num) : str_(str), num_(num) {}

  std::string str_;
  int num_;
};

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, Type)
{
  // Check if the type is correct
  EXPECT_EQ(
      AlignedAllocatorFreeList::GetType(),
      AlignedAllocatorFreeList().getType());
}

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, Allocate)
{
  // Test allocating a block of memory with a given size and alignment
  void* ptr = mAllocator.allocate(1024, 16);
  EXPECT_NE(ptr, nullptr);
  EXPECT_EQ(0u, reinterpret_cast<uintptr_t>(ptr) % 16);
  mAllocator.deallocate(ptr, 1024);
}

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, AllocateAs)
{
  // Test allocating a block of memory for a specific type
  int* ptr = mAllocator.allocateAs<int>(10);
  EXPECT_NE(ptr, nullptr);
  mAllocator.deallocate(ptr, sizeof(int) * 10);
}

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, Construct)
{
  // Test constructing an object in a block of memory
  MyObject* ptr = mAllocator.construct<MyObject>("hello", 42);
  EXPECT_NE(ptr, nullptr);
  mAllocator.destroy(ptr);
}

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, ConstructAt)
{
  // Test constructing an object at a specific memory location
  void* memory = mAllocator.allocate(sizeof(MyObject), alignof(MyObject));
  MyObject* ptr = mAllocator.constructAt<MyObject>(memory, "hello", 42);
  EXPECT_EQ(ptr, reinterpret_cast<MyObject*>(memory));
  mAllocator.destroy(ptr);
}

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, Destroy)
{
  // Test destroying an object and deallocating its memory
  MyObject* ptr = mAllocator.construct<MyObject>("hello", 42);
  mAllocator.destroy(ptr);
}

//==============================================================================
TEST_F(AlignedAllocatorFreeListTest, Print)
{
  // Test printing the state of the allocator
  std::stringstream stream;
  mAllocator.print(stream);
  EXPECT_FALSE(stream.str().empty());
}
