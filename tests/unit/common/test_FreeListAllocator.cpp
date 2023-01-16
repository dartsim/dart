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

#include <dart/common/allocator/FreeListAllocator.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

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

  // Allocate small memory
  auto ptr1 = a.allocate(1);
  EXPECT_NE(ptr1, nullptr);

  EXPECT_FALSE(a.isEmpty());
  // Expect that FreeListAllocator complains that not all the memory is
  // deallocated
}
