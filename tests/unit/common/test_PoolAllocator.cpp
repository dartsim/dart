/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <dart/common/PoolAllocator.hpp>
#include <gtest/gtest.h>

#include "TestHelpers.hpp"

using namespace dart;
using namespace common;

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
