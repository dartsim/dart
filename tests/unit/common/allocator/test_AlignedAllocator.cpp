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

#include "dart/common/allocator/AlignedAllocatorRaw.hpp"

#include <gtest/gtest.h>

#include <sstream>

using namespace dart;
using namespace common;

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, Allocate)
{
  AlignedAllocator& allocator = AlignedAllocator::GetDefault();
  void* ptr = allocator.allocate(1024, 16);
  EXPECT_NE(ptr, nullptr);
  allocator.deallocate(ptr, 1024);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, AllocateAs)
{
  AlignedAllocator& allocator = AlignedAllocator::GetDefault();
  int* ptr = allocator.allocateAs<int>(10);
  EXPECT_NE(ptr, nullptr);
  allocator.deallocate(ptr, sizeof(int) * 10);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, Construct)
{
  AlignedAllocator& allocator = AlignedAllocator::GetDefault();
  std::string* str = allocator.construct<std::string>("hello");
  EXPECT_NE(str, nullptr);
  EXPECT_EQ(*str, "hello");
  allocator.destroy(str);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, ConstructAt)
{
  AlignedAllocator& allocator = AlignedAllocator::GetDefault();
  void* ptr = allocator.allocate(sizeof(std::string), alignof(std::string));
  std::string* str = allocator.constructAt<std::string>(ptr, "hello");
  EXPECT_EQ(ptr, str);
  EXPECT_EQ(*str, "hello");
  allocator.destroy(str);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, AlignmentZero)
{
  EXPECT_TRUE(AlignedAllocator::ValidateAlignment(100, 0));
  EXPECT_TRUE(AlignedAllocator::ValidateAlignment(0, 0));
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, AlignmentLessThanVoidPointer)
{
  EXPECT_FALSE(AlignedAllocator::ValidateAlignment(100, sizeof(void*) - 1));
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, AlignmentNotPowerOfTwo)
{
  EXPECT_FALSE(AlignedAllocator::ValidateAlignment(100, 7));
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, SizeNotMultipleOfAlignment)
{
  EXPECT_FALSE(AlignedAllocator::ValidateAlignment(100, 8));
}

//==============================================================================
GTEST_TEST(AlignedAllocatorTest, ValidInput)
{
  EXPECT_TRUE(AlignedAllocator::ValidateAlignment(0, 16));
}
