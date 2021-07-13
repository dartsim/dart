/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <gtest/gtest.h>

#include "dart/common/allocator.hpp"
#include "dart/common/free_list_allocator.hpp"
#include "dart/common/pool_allocator.hpp"

using namespace dart::common;

template <typename T, std::size_t Alignment = 16>
using AlignedVector = std::vector<T, AlignedAllocator<T, Alignment>>;

struct CustomVector3 {
  double m_data[3];
};

//==============================================================================
TEST(AllocatorTest, AlignedAllocator) {
#ifndef NDEBUG
  set_log_level(LogLevel::LL_DEBUG);
#endif

  auto allocator1 = AlignedAllocator<void, 16>();
  (void)allocator1;
  EXPECT_EQ(allocator1.alignment(), 16);

  auto allocator2 = AlignedAllocator<int, 32>();
  EXPECT_EQ(allocator2.alignment(), 32);

  AlignedVector<CustomVector3> m_vertices;
}

//==============================================================================
TEST(AllocatorTest, PoolAllocator) {
#ifndef NDEBUG
  set_log_level(LogLevel::LL_DEBUG);
#endif

  auto allocator1 = PoolAllocator(8, 8);
  (void)allocator1;

  EXPECT_TRUE(allocator1.allocate(0) == nullptr);
}

//==============================================================================
TEST(AllocatorTest, FreeListAllocator) {
#ifndef NDEBUG
  set_log_level(LogLevel::LL_DEBUG);
#endif

  auto allocator1 = FreeListAllocator<16>();
  (void)allocator1;

  EXPECT_TRUE(allocator1.allocate(0) == nullptr);
}
