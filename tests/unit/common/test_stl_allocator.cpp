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

#include <dart/common/free_list_allocator.hpp>
#include <dart/common/pool_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <entt/entity/registry.hpp>
#include <gtest/gtest.h>

#include <vector>

#include <cstdint>

using namespace dart;
using namespace common;

namespace {

struct alignas(64) OverAlignedComponent
{
  double values[8] = {};
};

struct RegistryTag
{
};

struct RegistryVelocity
{
  double values[3] = {};
};

template <typename T>
void expectAligned(T* pointer)
{
  ASSERT_NE(pointer, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(pointer) % alignof(T), 0u);
}

} // namespace

//==============================================================================
TEST(StlAllocatorTest, Basics)
{
  auto a = StlAllocator<int>();
  auto o1 = a.allocate(1);
  auto o2 = a.allocate(1);
  EXPECT_TRUE(o1 != nullptr);
  EXPECT_TRUE(o2 != nullptr);
  a.deallocate(o1, 1);
  a.deallocate(o2, 1);
  a.print();
}

//==============================================================================
TEST(StlAllocatorTest, ComparesUnderlyingAllocatorIdentity)
{
  FreeListAllocator first;
  FreeListAllocator second;

  StlAllocator<int> firstInts(first);
  StlAllocator<double> firstDoubles(firstInts);
  StlAllocator<int> secondInts(second);

  EXPECT_TRUE(firstInts == firstDoubles);
  EXPECT_FALSE(firstInts != firstDoubles);
  EXPECT_FALSE(firstInts == secondInts);
  EXPECT_TRUE(firstInts != secondInts);
}

//==============================================================================
TEST(StlAllocatorTest, SupportsOverAlignedVectorStorage)
{
  FreeListAllocator backing;
  StlAllocator<OverAlignedComponent> allocator(backing);

  std::vector<OverAlignedComponent, StlAllocator<OverAlignedComponent>> values(
      allocator);
  values.resize(4);

  expectAligned(values.data());
  for (auto& value : values) {
    expectAligned(&value);
  }
}

//==============================================================================
TEST(StlAllocatorTest, SupportsPoolBackedOverAlignedVectorStorage)
{
  PoolAllocator backing;
  StlAllocator<OverAlignedComponent> allocator(backing);

  std::vector<OverAlignedComponent, StlAllocator<OverAlignedComponent>> values(
      allocator);
  values.resize(8);

  expectAligned(values.data());
  for (auto& value : values) {
    expectAligned(&value);
  }
}

//==============================================================================
TEST(StlAllocatorTest, SupportsAllocatorAwareEnttRegistryStorage)
{
  FreeListAllocator backing;
  StlAllocator<entt::entity> registryAllocator(backing);
  entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
      registryAllocator);

  const entt::entity entity = registry.create();
  auto& component = registry.emplace<OverAlignedComponent>(entity);

  expectAligned(&component);

  auto& storage = registry.storage<OverAlignedComponent>();
  StlAllocator<OverAlignedComponent> componentAllocator(registryAllocator);
  EXPECT_TRUE(storage.get_allocator() == componentAllocator);

  registry.destroy(entity);
}

//==============================================================================
TEST(StlAllocatorTest, SupportsAllocatorAwareEnttRegistryViews)
{
  FreeListAllocator backing;
  StlAllocator<entt::entity> registryAllocator(backing);
  entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
      registryAllocator);

  const entt::entity entity = registry.create();
  registry.emplace<RegistryTag>(entity);
  registry.emplace<OverAlignedComponent>(entity);
  registry.emplace<RegistryVelocity>(entity);

  auto view
      = registry.view<RegistryTag, OverAlignedComponent, RegistryVelocity>();
  EXPECT_EQ(view.size_hint(), 1u);

  registry.destroy(entity);
}
