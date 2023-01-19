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

#include "dart/common/allocator/AlignedAllocatorLinear.hpp"

#include <gtest/gtest.h>

#include <sstream>

using namespace dart;
using namespace common;

//==============================================================================
GTEST_TEST(AlignedAllocatorLinearTest, Allocate)
{
  AlignedAllocatorLinear allocator(1024);

  // Allocate memory where the size is not multiple of alignment
  ASSERT_EQ(allocator.allocate(100, 8), nullptr);
  ASSERT_EQ(allocator.allocate(100, 16), nullptr);
  ASSERT_EQ(allocator.allocate(300, 8), nullptr);
  EXPECT_EQ(0, allocator.get_size());

  // Allocate memory where the size is multiple of alignment
  void* ptr3 = allocator.allocate(8 * 16, 8);
  EXPECT_NE(ptr3, nullptr);
  EXPECT_EQ(8 * 16, allocator.get_size());

  void* ptr4 = allocator.allocate(8 * 32, 8);
  EXPECT_NE(ptr4, nullptr);
  EXPECT_EQ(8 * 16 + 8 * 32, allocator.get_size());

  allocator.deallocate(ptr3, 8 * 16);
  allocator.deallocate(ptr4, 8 * 32);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorLinearTest, Deallocate)
{
  AlignedAllocatorLinear allocator(1024);

  void* ptr1 = allocator.allocate(8 * 16, 8);
  void* ptr2 = allocator.allocate(8 * 32, 8);
  void* ptr3 = allocator.allocate(8 * 64, 8);

  EXPECT_EQ(8 * 16 + 8 * 32 + 8 * 64, allocator.get_size());

  allocator.deallocate(ptr2, 8 * 32);

  // deallocate doesn't actually free memory, only check the size remains the
  // same
  EXPECT_EQ(8 * 16 + 8 * 32 + 8 * 64, allocator.get_size());

  allocator.deallocate(ptr1, 8 * 16);
  allocator.deallocate(ptr3, 8 * 64);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorLinearTest, GetMaxCapacity)
{
  AlignedAllocatorLinear allocator(1024);

  EXPECT_EQ(1024, allocator.get_max_capacity());
}

//==============================================================================
GTEST_TEST(AlignedAllocatorLinearTest, GetSize)
{
  AlignedAllocatorLinear allocator(1024);

  void* ptr1 = allocator.allocate(8 * 2, 8);
  EXPECT_EQ(8 * 2, allocator.get_size());

  void* ptr2 = allocator.allocate(8 * 4, 8);
  EXPECT_EQ(8 * 2 + 8 * 4, allocator.get_size());

  void* ptr3 = allocator.allocate(8 * 8, 8);
  EXPECT_EQ(8 * 2 + 8 * 4 + 8 * 8, allocator.get_size());

  allocator.deallocate(ptr1, 8 * 2);
  allocator.deallocate(ptr2, 8 * 4);
  allocator.deallocate(ptr3, 8 * 8);
}

//==============================================================================
GTEST_TEST(AlignedAllocatorLinearTest, GetBeginAddress)
{
  AlignedAllocatorLinear allocator(1024);

  // Allocate some memory
  void* ptr1 = allocator.allocate(32, 8);

  EXPECT_EQ(ptr1, allocator.get_begin_address());
}

//==============================================================================
GTEST_TEST(AlignedAllocatorLinearTest, Print)
{
  AlignedAllocatorLinear allocator(1024);

  // Allocate some memory
  auto ptr1 = allocator.allocate(32, 8);
  auto ptr2 = allocator.allocate(64, 16);

  // Redirect cout to stringstream
  std::stringstream ss;
  std::streambuf* oldCout = std::cout.rdbuf(ss.rdbuf());

  allocator.print();

  // Get the output from stringstream
  std::string output = ss.str();

  // Check that the name is correct
  EXPECT_TRUE(
      output.find("[dart::common::AlignedAllocatorLinear]\n")
      != std::string::npos);

  // Check that the first address is correct
  std::stringstream ss_expected;
  ss_expected << "first_address: " << ptr1;
  EXPECT_TRUE(output.find(ss_expected.str()) != std::string::npos);

  // Check that the size is correct
  EXPECT_TRUE(
      output.find("size_in_bytes: " + std::to_string(32 + 64))
      != std::string::npos);

  // Reset cout to its original buffer
  std::cout.rdbuf(oldCout);

  // Deallocate memory
  allocator.deallocate(ptr1, 32);
  allocator.deallocate(ptr2, 64);
}
