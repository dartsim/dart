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

#include <dart/dynamics/detail/body_node_pool.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <unordered_set>
#include <vector>

#include <cstdint>

namespace {

struct alignas(32) FakeBodyNode
{
  char data[1536];
};

using Pool = dart::dynamics::detail::BodyNodePool<FakeBodyNode>;

} // namespace

//==============================================================================
TEST(BodyNodePoolTest, BasicAllocation)
{
  Pool pool;
  std::vector<void*> pointers;
  pointers.reserve(10);

  for (int i = 0; i < 10; ++i) {
    void* ptr = pool.allocate();
    ASSERT_NE(ptr, nullptr);
    EXPECT_EQ(reinterpret_cast<std::uintptr_t>(ptr) % 32, 0u);
    pointers.push_back(ptr);
  }

  std::unordered_set<void*> uniquePtrs(pointers.begin(), pointers.end());
  EXPECT_EQ(uniquePtrs.size(), pointers.size());
}

//==============================================================================
TEST(BodyNodePoolTest, Deallocation)
{
  Pool pool;
  void* p0 = pool.allocate();
  void* p1 = pool.allocate();
  void* p2 = pool.allocate();
  void* p3 = pool.allocate();
  void* p4 = pool.allocate();

  pool.deallocate(p1);
  pool.deallocate(p3);

  void* r0 = pool.allocate();
  void* r1 = pool.allocate();

  EXPECT_EQ(r0, p3);
  EXPECT_EQ(r1, p1);

  pool.deallocate(p0);
  pool.deallocate(p2);
  pool.deallocate(p4);
  pool.deallocate(r0);
  pool.deallocate(r1);
}

//==============================================================================
TEST(BodyNodePoolTest, GrowthPreservesPointers)
{
  Pool pool;
  std::vector<void*> firstChunk;
  firstChunk.reserve(64);

  for (int i = 0; i < 64; ++i) {
    void* ptr = pool.allocate();
    firstChunk.push_back(ptr);
    auto* bytes = static_cast<std::uint8_t*>(ptr);
    bytes[0] = static_cast<std::uint8_t>(i);
  }

  std::vector<void*> laterAllocations;
  laterAllocations.reserve(36);
  for (int i = 0; i < 36; ++i) {
    laterAllocations.push_back(pool.allocate());
  }

  for (int i = 0; i < 64; ++i) {
    auto* bytes = static_cast<std::uint8_t*>(firstChunk[i]);
    EXPECT_EQ(bytes[0], static_cast<std::uint8_t>(i));
  }
}

//==============================================================================
TEST(BodyNodePoolTest, OwnsCheck)
{
  Pool pool;
  void* ptr = pool.allocate();
  EXPECT_TRUE(pool.owns(ptr));

  FakeBodyNode stackNode;
  EXPECT_FALSE(pool.owns(&stackNode));

  auto heapNode = std::make_unique<FakeBodyNode>();
  EXPECT_FALSE(pool.owns(heapNode.get()));
}

//==============================================================================
TEST(BodyNodePoolTest, SizeTracking)
{
  Pool pool;
  EXPECT_EQ(pool.size(), 0u);

  void* p0 = pool.allocate();
  EXPECT_EQ(pool.size(), 1u);

  void* p1 = pool.allocate();
  EXPECT_EQ(pool.size(), 2u);

  pool.deallocate(p0);
  EXPECT_EQ(pool.size(), 1u);

  pool.deallocate(p1);
  EXPECT_EQ(pool.size(), 0u);
}

//==============================================================================
TEST(BodyNodePoolTest, Alignment)
{
  Pool pool;
  for (int i = 0; i < 500; ++i) {
    void* ptr = pool.allocate();
    ASSERT_NE(ptr, nullptr);
    EXPECT_EQ(reinterpret_cast<std::uintptr_t>(ptr) % 32, 0u);
  }
}
