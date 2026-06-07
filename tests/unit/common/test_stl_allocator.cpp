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

#include <dart/common/fixed_pool_allocator.hpp>
#include <dart/common/free_list_allocator.hpp>
#include <dart/common/pool_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <entt/entity/registry.hpp>
#include <gtest/gtest.h>

#include <limits>
#include <string_view>
#include <vector>

#include <cstddef>
#include <cstdint>

using namespace dart;
using namespace common;

namespace {

struct alignas(64) OverAlignedComponent
{
  double values[8] = {};
};

struct alignas(std::max_align_t) MaxAlignedComponent
{
  double values[2] = {};
};

struct RegistryTag
{
};

struct RegistryVelocity
{
  double values[3] = {};
};

class CountingMemoryAllocator final : public MemoryAllocator
{
public:
  explicit CountingMemoryAllocator(size_t initialAllocation)
    : mBacking(MemoryAllocator::GetDefault(), initialAllocation)
  {
  }

  std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  void* allocate(size_t bytes) noexcept override
  {
    ++allocationCount;
    return mBacking.allocate(bytes);
  }

  void* allocate(size_t bytes, size_t alignment) noexcept override
  {
    ++allocationCount;
    return mBacking.allocate(bytes, alignment);
  }

  void deallocate(void* pointer, size_t bytes) override
  {
    ++deallocationCount;
    mBacking.deallocate(pointer, bytes);
  }

  void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    ++deallocationCount;
    mBacking.deallocate(pointer, bytes, alignment);
  }

  void resetCounts()
  {
    allocationCount = 0;
    deallocationCount = 0;
  }

  size_t allocationCount{0};
  size_t deallocationCount{0};

private:
  FreeListAllocator mBacking;
};

template <typename T>
void expectAligned(T* pointer)
{
  ASSERT_NE(pointer, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(pointer) % alignof(T), 0u);
}

template <typename Registry>
void reserveRegistryStorage(Registry& registry, const size_t entityCount)
{
  registry.template storage<RegistryTag>().reserve(entityCount);
  registry.template storage<RegistryVelocity>().reserve(entityCount);
  registry.template storage<OverAlignedComponent>().reserve(entityCount);
}

template <typename Registry>
void runRegistryChurn(
    Registry& registry,
    std::vector<entt::entity>& entities,
    const size_t entityCount)
{
  for (size_t i = 0; i < entityCount; ++i) {
    const auto entity = registry.create();
    entities[i] = entity;

    registry.template emplace<RegistryTag>(entity);

    auto& velocity = registry.template emplace<RegistryVelocity>(entity);
    velocity.values[0] = static_cast<double>(i);

    auto& component = registry.template emplace<OverAlignedComponent>(entity);
    component.values[0] = static_cast<double>(i + 1);
  }

  double total = 0.0;
  for (const auto entity : entities) {
    total += registry.template get<RegistryVelocity>(entity).values[0];
    total += registry.template get<OverAlignedComponent>(entity).values[0];
    EXPECT_TRUE(registry.template all_of<RegistryTag>(entity));
  }
  EXPECT_GT(total, 0.0);

  for (size_t i = entityCount; i > 0; --i) {
    registry.destroy(entities[i - 1]);
  }
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
TEST(StlAllocatorTest, RejectsCountOverflow)
{
  FreeListAllocator backing;
  StlAllocator<OverAlignedComponent> allocator(backing);

  EXPECT_THROW(
      {
        auto* ptr = allocator.allocate(
            std::numeric_limits<std::size_t>::max()
                / sizeof(OverAlignedComponent)
            + 1);
        (void)ptr;
      },
      std::bad_alloc);
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
TEST(StlAllocatorTest, SupportsFixedPoolBackedMaxAlignedStorage)
{
  const size_t underAlignedUnitSize
      = sizeof(MaxAlignedComponent) + alignof(MaxAlignedComponent) / 2;
  FixedPoolAllocator backing(underAlignedUnitSize);
  StlAllocator<MaxAlignedComponent> allocator(backing);

  auto* first = allocator.allocate(1);
  auto* second = allocator.allocate(1);

  expectAligned(first);
  expectAligned(second);

  allocator.deallocate(second, 1);
  allocator.deallocate(first, 1);
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

//==============================================================================
TEST(StlAllocatorTest, ReservedEnttRegistryChurnDoesNotAllocate)
{
  constexpr size_t entityCount = 128;

  CountingMemoryAllocator backing(entityCount * 4096 + 1024 * 1024);
  StlAllocator<entt::entity> registryAllocator(backing);
  entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
      registryAllocator);
  std::vector<entt::entity> entities(entityCount);

  reserveRegistryStorage(registry, entityCount);
  runRegistryChurn(registry, entities, entityCount);

  backing.resetCounts();
  runRegistryChurn(registry, entities, entityCount);

  EXPECT_EQ(backing.allocationCount, 0u);
  EXPECT_EQ(backing.deallocationCount, 0u);
}
