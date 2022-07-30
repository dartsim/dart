/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#include <gtest/gtest.h>

#include "dart/common/all.hpp"

using namespace dart;
using namespace common;

//==============================================================================
TEST(StlAllocatorTest, Basics)
{
  common::set_log_level(LogLevel::LOGLEVEL_DEBUG);

  {
    auto a = StlAllocator<int>();
    auto o1 = a.allocate(1);
    auto o2 = a.allocate(1);
    EXPECT_TRUE(o1 != nullptr);
    EXPECT_TRUE(o2 != nullptr);
    a.print();
    a.deallocate(o1, sizeof(int));
    a.deallocate(o2, sizeof(int));
    a.print();
  }

  {
    LinearAllocator base_allocator(sizeof(int) + 1);
    static_assert(
        sizeof(int) > 1,
        "sizeof(int) should be greater than 1 to keep this test valid.");
    auto a = StlAllocator<int>(base_allocator);
    EXPECT_TRUE(a.allocate(1) != nullptr);
    try {
      EXPECT_TRUE(a.allocate(1) == nullptr);
    } catch (std::bad_alloc& /*e*/) {
      EXPECT_TRUE(true);
    } catch (...) {
      EXPECT_TRUE(false);
    }
    a.print();
  }
}

//==============================================================================
TEST(StlAllocatorTest, StdVector)
{
  common::set_log_level(LogLevel::LOGLEVEL_DEBUG);

  {
    std::vector<int, StlAllocator<int>> vec;
    EXPECT_EQ(vec.capacity(), 0);
    vec.reserve(1);
    EXPECT_EQ(vec.capacity(), 1);
    vec.reserve(2);
    EXPECT_EQ(vec.capacity(), 2);
  }

  {
    LinearAllocator base_allocator(sizeof(int) + 1);
    common::vector<int> vec(
        base_allocator); // equivalent to std::vector<int, StlAllocator<int>>
    EXPECT_EQ(vec.size(), 0);
    vec.resize(1);
    EXPECT_EQ(vec.size(), 1);
    // Cannot allocator more than one because the base_allocator only can
    // allocate up to sizeof(int) + 1.
    try {
      vec.resize(2);
    } catch (std::bad_alloc& /*e*/) {
      EXPECT_TRUE(true);
    } catch (...) {
      EXPECT_TRUE(false);
    }
    EXPECT_EQ(vec.size(), 1);
    vec.get_allocator().print();
  }
}
