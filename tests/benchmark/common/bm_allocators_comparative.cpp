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

/// @file bm_allocators_comparative.cpp
/// @brief Comparative allocator benchmarks: DART vs foonathan/memory vs
/// std::pmr
///
/// Provides head-to-head performance comparison across three allocator
/// families on identical workloads. This file is benchmark-only and never
/// linked into the DART library.
///
/// Sections:
///   1. Pool: fixed-size alloc/dealloc (DART Pool vs foonathan memory_pool
///      vs std::pmr::unsynchronized_pool_resource)
///   2. Stack/Frame: bump alloc + bulk reset (DART Frame vs foonathan
///      memory_stack vs std::pmr::monotonic_buffer_resource)
///   3. Multi-pool: mixed-size alloc/dealloc
///   4. Realistic mixed workload (multi-size, random order)
///   5. Steady-state: pre-filled pool with random replace
///   6. Frame bulk varying workload
///   7. STL container workload
///   8. Static fixed-storage stack workload
///   9. Scoped temporary allocator workload
///   10. Two-iteration frame allocator workload
///   11. Raw heap/malloc/new allocator workload
///   12. Aligned/fallback/segregator adapter workloads
///   13. Tracked/deeply-tracked allocator workloads
///   14. EnTT registry workload
///
/// Run:  pixi run bm -- allocators-comparative
/// JSON: pixi run bm -- allocators-comparative
///         --benchmark_out=.benchmark_results/comparative.json
///         --benchmark_out_format=json

#include <dart/common/fixed_pool_allocator.hpp>
#include <dart/common/frame_allocator.hpp>
#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator.hpp>
#include <dart/common/pool_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <benchmark/benchmark.h>
#include <foonathan/memory/aligned_allocator.hpp>
#include <foonathan/memory/fallback_allocator.hpp>
#include <foonathan/memory/heap_allocator.hpp>
#include <foonathan/memory/iteration_allocator.hpp>
#include <foonathan/memory/malloc_allocator.hpp>
#include <foonathan/memory/memory_pool.hpp>
#include <foonathan/memory/memory_pool_collection.hpp>
#include <foonathan/memory/memory_stack.hpp>
#include <foonathan/memory/namespace_alias.hpp>
#include <foonathan/memory/new_allocator.hpp>
#include <foonathan/memory/segregator.hpp>
#include <foonathan/memory/static_allocator.hpp>
#include <foonathan/memory/std_allocator.hpp>
#include <foonathan/memory/temporary_allocator.hpp>
#include <foonathan/memory/tracking.hpp>

#if defined(DART_BENCHMARK_HAS_ENTT) && DART_BENCHMARK_HAS_ENTT
  #include <entt/entity/registry.hpp>
#endif

#include <algorithm>
#include <array>
#include <memory>
#include <memory_resource>
#include <new>
#include <string_view>
#include <vector>

#include <cstddef>
#include <cstdint>
#include <cstdlib>

using namespace dart::common;
namespace fm = foonathan::memory;

// =============================================================================
// Simple LCG random (no heap allocations unlike std::mt19937)
// =============================================================================

static uint32_t lcgState = 12345u;

inline uint32_t lcgNext(uint32_t& state) noexcept
{
  state = state * 1664525u + 1013904223u;
  return state;
}

inline uint32_t lcgNext()
{
  return lcgNext(lcgState);
}

class BenchmarkCountingMemoryAllocator final : public MemoryAllocator
{
public:
  explicit BenchmarkCountingMemoryAllocator(MemoryAllocator& backing) noexcept
    : mBacking(backing)
  {
  }

  std::string_view getType() const override
  {
    return "BenchmarkCountingMemoryAllocator";
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
  MemoryAllocator& mBacking;
};

struct BenchmarkAllocationTracker
{
  void on_node_allocation(void*, std::size_t size, std::size_t) noexcept
  {
    ++allocationCount;
    liveBytes += size;
    peakBytes = std::max(peakBytes, liveBytes);
  }

  void on_node_deallocation(void*, std::size_t size, std::size_t) noexcept
  {
    ++deallocationCount;
    liveBytes -= std::min(liveBytes, size);
  }

  void on_bulk_deallocation(
      std::size_t count, std::size_t size, std::size_t) noexcept
  {
    deallocationCount += count;
    const std::size_t bytes = count * size;
    liveBytes -= std::min(liveBytes, bytes);
  }

  void on_array_allocation(
      void*, std::size_t count, std::size_t size, std::size_t) noexcept
  {
    on_node_allocation(nullptr, count * size, alignof(std::max_align_t));
  }

  void on_array_deallocation(
      void*, std::size_t count, std::size_t size, std::size_t) noexcept
  {
    on_node_deallocation(nullptr, count * size, alignof(std::max_align_t));
  }

  void on_allocator_growth(void*, std::size_t size) noexcept
  {
    growthBytes += size;
  }

  void on_allocator_shrinking(void*, std::size_t size) noexcept
  {
    shrinkBytes += size;
  }

  size_t allocationCount{0};
  size_t deallocationCount{0};
  size_t liveBytes{0};
  size_t peakBytes{0};
  size_t growthBytes{0};
  size_t shrinkBytes{0};
};

// =============================================================================
// Section 1: Pool — Fixed-Size Alloc/Dealloc
//
// Benchmark: allocate N nodes of size S, then deallocate all (LIFO).
// Compares single-size pool performance.
// =============================================================================

static constexpr size_t kPoolCyclesPerIteration = 4096;

[[nodiscard]] static constexpr size_t poolCyclesFor(size_t count) noexcept
{
  return count <= 64 ? kPoolCyclesPerIteration : 256;
}

static void BM_Pool_DART(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t cycleCount = poolCyclesFor(count);
  const size_t blockSize = (size + 16) * count + 4096;
  FixedPoolAllocator alloc(size, MemoryAllocator::GetDefault(), blockSize);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        ptrs[i] = alloc.allocate();
      }
      for (size_t i = count; i > 0; --i) {
        alloc.deallocate(ptrs[i - 1]);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * count * cycleCount));
}
BENCHMARK(BM_Pool_DART)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({32, 1024})
    ->ReportAggregatesOnly(true);

static void BM_Pool_Foonathan(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t cycleCount = poolCyclesFor(count);
  const size_t blockSize = (size + 16) * count + 4096;
  fm::memory_pool<fm::node_pool> pool(size, blockSize);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        ptrs[i] = pool.allocate_node();
      }
      for (size_t i = count; i > 0; --i) {
        pool.deallocate_node(ptrs[i - 1]);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * count * cycleCount));
}
BENCHMARK(BM_Pool_Foonathan)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({32, 1024})
    ->ReportAggregatesOnly(true);

static void BM_Pool_StdPmr(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t cycleCount = poolCyclesFor(count);
  std::pmr::pool_options opts{};
  opts.max_blocks_per_chunk = count;
  opts.largest_required_pool_block = size;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        ptrs[i] = pool.allocate(size, alignof(std::max_align_t));
      }
      for (size_t i = count; i > 0; --i) {
        pool.deallocate(ptrs[i - 1], size, alignof(std::max_align_t));
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * count * cycleCount));
}
BENCHMARK(BM_Pool_StdPmr)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({32, 1024})
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 2: Stack/Frame — Bump Alloc + Bulk Reset
//
// Benchmark: allocate N items sequentially (bump), then reset (bulk free).
// This is the primary use case for per-physics-step temporaries.
// =============================================================================

static constexpr size_t kStackCyclesPerIteration = 8192;

static void BM_Stack_DART(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t arenaSize = count * ((size + 31) & ~size_t{31}) + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kStackCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate(size);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kStackCyclesPerIteration));
}
BENCHMARK(BM_Stack_DART)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({256, 1024})
    ->Args({32, 4096})
    ->ReportAggregatesOnly(true);

static void BM_Stack_Foonathan(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t arenaSize = count * (size + 32) + 4096;
  fm::memory_stack<> stack(arenaSize);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kStackCyclesPerIteration; ++cycle) {
      auto marker = stack.top();
      for (size_t i = 0; i < count; ++i) {
        void* p = stack.allocate(size, 32);
        benchmark::DoNotOptimize(p);
      }
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kStackCyclesPerIteration));
}
BENCHMARK(BM_Stack_Foonathan)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({256, 1024})
    ->Args({32, 4096})
    ->ReportAggregatesOnly(true);

static void BM_Stack_StdPmr(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t arenaSize = count * (size + 32) + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kStackCyclesPerIteration; ++cycle) {
      // Reconstruct each cycle (monotonic_buffer_resource has no reset())
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::null_memory_resource());
      for (size_t i = 0; i < count; ++i) {
        void* p = mono.allocate(size, 32);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kStackCyclesPerIteration));
}
BENCHMARK(BM_Stack_StdPmr)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({256, 1024})
    ->Args({32, 4096})
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 3: Multi-Pool — Mixed-Size Alloc/Dealloc (4096 ops)
//
// Benchmark: allocate objects of varying sizes from a multi-bucket pool.
// Tests how well each allocator handles heterogeneous allocation patterns.
// =============================================================================

static constexpr auto kMultiSizes
    = std::to_array<size_t>({16, 32, 64, 128, 256, 512});
static constexpr size_t kMultiOps = 4096;

static void BM_MultiPool_DART(benchmark::State& state)
{
  PoolAllocator alloc(
      MemoryAllocator::GetDefault(),
      PoolAllocator::DiagnosticsPolicy::Disabled);
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizes.size()];
      ptrs[i] = {alloc.allocateUntracked(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kMultiOps; i > 0; --i) {
      alloc.deallocateUntracked(ptrs[i - 1].first, ptrs[i - 1].second);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * kMultiOps));
}
BENCHMARK(BM_MultiPool_DART)->ReportAggregatesOnly(true);

static void BM_MultiPool_Foonathan(benchmark::State& state)
{
  fm::memory_pool_collection<fm::node_pool, fm::identity_buckets> pool(
      512, 1048576);
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizes.size()];
      ptrs[i] = {pool.allocate_node(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kMultiOps; i > 0; --i) {
      pool.deallocate_node(ptrs[i - 1].first, ptrs[i - 1].second);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * kMultiOps));
}
BENCHMARK(BM_MultiPool_Foonathan)->ReportAggregatesOnly(true);

static void BM_MultiPool_StdPmr(benchmark::State& state)
{
  std::pmr::pool_options opts{};
  opts.largest_required_pool_block = 512;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizes.size()];
      ptrs[i] = {pool.allocate(sz, alignof(std::max_align_t)), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kMultiOps; i > 0; --i) {
      pool.deallocate(
          ptrs[i - 1].first, ptrs[i - 1].second, alignof(std::max_align_t));
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * kMultiOps));
}
BENCHMARK(BM_MultiPool_StdPmr)->ReportAggregatesOnly(true);

// =============================================================================
// Section 4: Realistic Mixed Workload
//
// Benchmark: alloc 1000 objects of random sizes, then dealloc in random order.
// Simulates a baked full physics step allocation pattern after one-time pool
// capacity growth has been moved out of the measured loop.
// =============================================================================

static constexpr auto kRealisticSizes
    = std::to_array<size_t>({16, 32, 64, 128, 256, 512, 1024});
static constexpr size_t kRealisticOps = 1000;
static constexpr size_t kRealisticCyclesPerIteration = 32;

static void BM_Realistic_DART(benchmark::State& state)
{
  PoolAllocator alloc(
      MemoryAllocator::GetDefault(),
      PoolAllocator::DiagnosticsPolicy::Disabled);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);
  for (const size_t sz : kRealisticSizes) {
    void* p = alloc.allocateUntracked(sz);
    alloc.deallocateUntracked(p, sz);
  }
  uint32_t warmRng = 99u;
  for (size_t i = 0; i < kRealisticOps; ++i) {
    const size_t sz
        = kRealisticSizes[lcgNext(warmRng) % kRealisticSizes.size()];
    ptrs[i] = {alloc.allocateUntracked(sz), sz};
  }
  for (size_t i = kRealisticOps; i > 0; --i) {
    alloc.deallocateUntracked(ptrs[i - 1].first, ptrs[i - 1].second);
  }

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kRealisticCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < kRealisticOps; ++i) {
        const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizes.size()];
        ptrs[i] = {alloc.allocateUntracked(sz), sz};
        benchmark::DoNotOptimize(ptrs[i].first);
      }
      for (size_t i = kRealisticOps; i > 0; --i) {
        const size_t idx = lcgNext() % i;
        alloc.deallocateUntracked(ptrs[idx].first, ptrs[idx].second);
        ptrs[idx] = ptrs[i - 1];
      }
    }
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * kRealisticOps * kRealisticCyclesPerIteration));
}
BENCHMARK(BM_Realistic_DART)->ReportAggregatesOnly(true);

static void BM_Realistic_Foonathan(benchmark::State& state)
{
  fm::memory_pool_collection<fm::node_pool, fm::identity_buckets> pool(
      1024, 2097152);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);
  for (const size_t sz : kRealisticSizes) {
    void* p = pool.allocate_node(sz);
    pool.deallocate_node(p, sz);
  }
  uint32_t warmRng = 99u;
  for (size_t i = 0; i < kRealisticOps; ++i) {
    const size_t sz
        = kRealisticSizes[lcgNext(warmRng) % kRealisticSizes.size()];
    ptrs[i] = {pool.allocate_node(sz), sz};
  }
  for (size_t i = kRealisticOps; i > 0; --i) {
    pool.deallocate_node(ptrs[i - 1].first, ptrs[i - 1].second);
  }

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kRealisticCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < kRealisticOps; ++i) {
        const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizes.size()];
        ptrs[i] = {pool.allocate_node(sz), sz};
        benchmark::DoNotOptimize(ptrs[i].first);
      }
      for (size_t i = kRealisticOps; i > 0; --i) {
        const size_t idx = lcgNext() % i;
        pool.deallocate_node(ptrs[idx].first, ptrs[idx].second);
        ptrs[idx] = ptrs[i - 1];
      }
    }
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * kRealisticOps * kRealisticCyclesPerIteration));
}
BENCHMARK(BM_Realistic_Foonathan)->ReportAggregatesOnly(true);

static void BM_Realistic_StdPmr(benchmark::State& state)
{
  std::pmr::pool_options opts{};
  opts.largest_required_pool_block = 1024;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);
  for (const size_t sz : kRealisticSizes) {
    void* p = pool.allocate(sz, alignof(std::max_align_t));
    pool.deallocate(p, sz, alignof(std::max_align_t));
  }
  uint32_t warmRng = 99u;
  for (size_t i = 0; i < kRealisticOps; ++i) {
    const size_t sz
        = kRealisticSizes[lcgNext(warmRng) % kRealisticSizes.size()];
    ptrs[i] = {pool.allocate(sz, alignof(std::max_align_t)), sz};
  }
  for (size_t i = kRealisticOps; i > 0; --i) {
    pool.deallocate(
        ptrs[i - 1].first, ptrs[i - 1].second, alignof(std::max_align_t));
  }

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kRealisticCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < kRealisticOps; ++i) {
        const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizes.size()];
        ptrs[i] = {pool.allocate(sz, alignof(std::max_align_t)), sz};
        benchmark::DoNotOptimize(ptrs[i].first);
      }
      for (size_t i = kRealisticOps; i > 0; --i) {
        const size_t idx = lcgNext() % i;
        pool.deallocate(
            ptrs[idx].first, ptrs[idx].second, alignof(std::max_align_t));
        ptrs[idx] = ptrs[i - 1];
      }
    }
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * kRealisticOps * kRealisticCyclesPerIteration));
}
BENCHMARK(BM_Realistic_StdPmr)->ReportAggregatesOnly(true);

// =============================================================================
// Section 5: Steady-State — Pre-Filled Pool + Random Replace
//
// Benchmark: fill pool with N objects, then repeatedly dealloc+realloc random
// slots. Simulates a running simulation with object churn.
// =============================================================================

static constexpr size_t kSteadyStateOpsMultiplier = 64;

static void BM_SteadyState_DART(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));
  const size_t blockSize = (size + 16) * poolSize + 4096;
  FixedPoolAllocator alloc(size, MemoryAllocator::GetDefault(), blockSize);
  const size_t ops = poolSize * kSteadyStateOpsMultiplier;

  std::vector<void*> ptrs(poolSize);
  for (size_t i = 0; i < poolSize; ++i) {
    ptrs[i] = alloc.allocate();
  }

  lcgState = 42u;
  for (auto _ : state) {
    for (size_t i = 0; i < ops; ++i) {
      const size_t idx = lcgNext() % poolSize;
      alloc.deallocate(ptrs[idx]);
      ptrs[idx] = alloc.allocate();
      benchmark::DoNotOptimize(ptrs[idx]);
    }
  }

  for (size_t i = 0; i < poolSize; ++i) {
    alloc.deallocate(ptrs[i]);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * ops));
}
BENCHMARK(BM_SteadyState_DART)
    ->Args({64, 1024})
    ->Args({256, 512})
    ->ReportAggregatesOnly(true);

static void BM_SteadyState_Foonathan(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));
  const size_t blockSize = (size + 16) * poolSize + 4096;
  fm::memory_pool<fm::node_pool> pool(size, blockSize);
  const size_t ops = poolSize * kSteadyStateOpsMultiplier;

  std::vector<void*> ptrs(poolSize);
  for (size_t i = 0; i < poolSize; ++i) {
    ptrs[i] = pool.allocate_node();
  }

  lcgState = 42u;
  for (auto _ : state) {
    for (size_t i = 0; i < ops; ++i) {
      const size_t idx = lcgNext() % poolSize;
      pool.deallocate_node(ptrs[idx]);
      ptrs[idx] = pool.allocate_node();
      benchmark::DoNotOptimize(ptrs[idx]);
    }
  }

  for (size_t i = 0; i < poolSize; ++i) {
    pool.deallocate_node(ptrs[i]);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * ops));
}
BENCHMARK(BM_SteadyState_Foonathan)
    ->Args({64, 1024})
    ->Args({256, 512})
    ->ReportAggregatesOnly(true);

static void BM_SteadyState_StdPmr(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));
  std::pmr::pool_options opts{};
  opts.max_blocks_per_chunk = poolSize;
  opts.largest_required_pool_block = size;
  std::pmr::unsynchronized_pool_resource pool(opts);
  const size_t ops = poolSize * kSteadyStateOpsMultiplier;

  std::vector<void*> ptrs(poolSize);
  for (size_t i = 0; i < poolSize; ++i) {
    ptrs[i] = pool.allocate(size, alignof(std::max_align_t));
  }

  lcgState = 42u;
  for (auto _ : state) {
    for (size_t i = 0; i < ops; ++i) {
      const size_t idx = lcgNext() % poolSize;
      pool.deallocate(ptrs[idx], size, alignof(std::max_align_t));
      ptrs[idx] = pool.allocate(size, alignof(std::max_align_t));
      benchmark::DoNotOptimize(ptrs[idx]);
    }
  }

  for (size_t i = 0; i < poolSize; ++i) {
    pool.deallocate(ptrs[i], size, alignof(std::max_align_t));
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * ops));
}
BENCHMARK(BM_SteadyState_StdPmr)
    ->Args({64, 1024})
    ->Args({256, 512})
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 6: Frame Bulk Varying — Simulated per-step pattern
//
// Benchmark: allocate objects of varying sizes from bump allocator, then
// reset. Tests the exact usage pattern in DART's ConstraintSolver.
// =============================================================================

static constexpr auto kFrameSizes
    = std::to_array<size_t>({24, 48, 96, 192, 384});
constexpr size_t kStaticStackStorageBytes = 8 * 1024 * 1024;
static constexpr size_t kFrameBulkCyclesPerIteration = 256;
static constexpr size_t kScratchCyclesPerIteration = 128;

static void BM_FrameBulk_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    uint32_t rng = 55u;
    for (size_t cycle = 0; cycle < kFrameBulkCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = alloc.allocate(sz);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kFrameBulkCyclesPerIteration));
}
BENCHMARK(BM_FrameBulk_DART)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_FrameBulk_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  fm::memory_stack<> stack(arenaSize);

  for (auto _ : state) {
    uint32_t rng = 55u;
    for (size_t cycle = 0; cycle < kFrameBulkCyclesPerIteration; ++cycle) {
      auto marker = stack.top();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = stack.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kFrameBulkCyclesPerIteration));
}
BENCHMARK(BM_FrameBulk_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_FrameBulk_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    uint32_t rng = 55u;
    for (size_t cycle = 0; cycle < kFrameBulkCyclesPerIteration; ++cycle) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::null_memory_resource());
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = mono.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kFrameBulkCyclesPerIteration));
}
BENCHMARK(BM_FrameBulk_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 7: STL Vector — Container allocation through allocator adapters
//
// Benchmark: reserve vector storage through each allocator adapter, publish the
// backing storage, then discard. This isolates STL adapter allocation overhead
// instead of measuring identical vector element writes in each row.
// =============================================================================

static constexpr size_t kStlVectorReservationsPerIteration = 65536;

static void BM_StlVector_DART(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) + 16384;
  FrameAllocator arena(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    for (size_t repeat = 0; repeat < kStlVectorReservationsPerIteration;
         ++repeat) {
      arena.reset();
      FrameStlAllocator<int> frameAlloc(arena);
      std::vector<int, FrameStlAllocator<int>> vec(frameAlloc);
      vec.reserve(n);
      benchmark::DoNotOptimize(vec.data());
      benchmark::ClobberMemory();
    }
    arena.reset();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * n * kStlVectorReservationsPerIteration));
}
BENCHMARK(BM_StlVector_DART)->Arg(1000)->Arg(10000)->ReportAggregatesOnly(true);

static void BM_StlVector_Foonathan(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) + 16384;
  fm::memory_stack<> stack(arenaSize);

  for (auto _ : state) {
    for (size_t repeat = 0; repeat < kStlVectorReservationsPerIteration;
         ++repeat) {
      auto marker = stack.top();
      fm::std_allocator<int, fm::memory_stack<>> alloc(stack);
      std::vector<int, fm::std_allocator<int, fm::memory_stack<>>> vec(alloc);
      vec.reserve(n);
      benchmark::DoNotOptimize(vec.data());
      benchmark::ClobberMemory();
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * n * kStlVectorReservationsPerIteration));
}
BENCHMARK(BM_StlVector_Foonathan)
    ->Arg(1000)
    ->Arg(10000)
    ->ReportAggregatesOnly(true);

static void BM_StlVector_StdPmr(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) + 16384;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    for (size_t repeat = 0; repeat < kStlVectorReservationsPerIteration;
         ++repeat) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::null_memory_resource());
      std::pmr::vector<int> vec(&mono);
      vec.reserve(n);
      benchmark::DoNotOptimize(vec.data());
      benchmark::ClobberMemory();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * n * kStlVectorReservationsPerIteration));
}
BENCHMARK(BM_StlVector_StdPmr)
    ->Arg(1000)
    ->Arg(10000)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 8: Static Stack — Fixed storage bump allocation
//
// Benchmark: allocate from a fixed backing store, then reset/unwind. This adds
// foonathan::static_block_allocator coverage for the same HMM role as DART's
// frame allocator.
// =============================================================================

static void BM_StaticStack_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  FrameAllocator alloc(MemoryAllocator::GetDefault(), kStaticStackStorageBytes);

  for (auto _ : state) {
    uint32_t rng = 61u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = alloc.allocate(sz);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_StaticStack_DART)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_StaticStack_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  auto storage = std::make_unique<
      fm::static_allocator_storage<kStaticStackStorageBytes>>();
  fm::memory_stack<fm::static_block_allocator> stack(
      kStaticStackStorageBytes, *storage);

  for (auto _ : state) {
    uint32_t rng = 61u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      auto marker = stack.top();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = stack.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_StaticStack_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_StaticStack_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  auto backing
      = std::make_unique_for_overwrite<char[]>(kStaticStackStorageBytes);

  for (auto _ : state) {
    uint32_t rng = 61u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(),
          kStaticStackStorageBytes,
          std::pmr::null_memory_resource());
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = mono.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_StaticStack_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 9: Temporary — Scoped scratch allocation
//
// Benchmark: allocate step-local scratch from a scoped allocator. DART resets
// the frame arena explicitly; foonathan::temporary_allocator unwinds on scope
// exit.
// =============================================================================

static void BM_Temporary_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    uint32_t rng = 67u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = alloc.allocate(sz);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_Temporary_DART)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_Temporary_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  fm::temporary_stack stack(arenaSize);

  for (auto _ : state) {
    uint32_t rng = 67u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      fm::temporary_allocator alloc(stack);
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = alloc.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_Temporary_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_Temporary_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    uint32_t rng = 67u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::null_memory_resource());
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = mono.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_Temporary_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 10: Iteration — Double-frame scratch allocation
//
// Benchmark: allocate scratch that remains valid for two loop iterations, then
// reuse the older frame. This maps foonathan::iteration_allocator<2> to DART's
// equivalent pair of frame arenas.
// =============================================================================

static void BM_Iteration_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  FrameAllocator first(MemoryAllocator::GetDefault(), arenaSize);
  FrameAllocator second(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    uint32_t rng = 73u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; cycle += 2) {
      first.reset();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = first.allocate(sz);
        benchmark::DoNotOptimize(p);
      }

      second.reset();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = second.allocate(sz);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_Iteration_DART)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_Iteration_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  fm::iteration_allocator<2> alloc(arenaSize * 2 + 4096);

  for (auto _ : state) {
    uint32_t rng = 73u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = alloc.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
      alloc.next_iteration();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_Iteration_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

static void BM_Iteration_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  std::array<std::unique_ptr<char[]>, 2> backing
      = {std::make_unique_for_overwrite<char[]>(arenaSize),
         std::make_unique_for_overwrite<char[]>(arenaSize)};
  std::pmr::monotonic_buffer_resource first(
      backing[0].get(), arenaSize, std::pmr::null_memory_resource());
  std::pmr::monotonic_buffer_resource second(
      backing[1].get(), arenaSize, std::pmr::null_memory_resource());
  std::array<std::pmr::monotonic_buffer_resource*, 2> frames
      = {&first, &second};
  size_t current = 0;

  for (auto _ : state) {
    uint32_t rng = 73u;
    for (size_t cycle = 0; cycle < kScratchCyclesPerIteration; ++cycle) {
      auto& mono = *frames[current];
      mono.release();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kFrameSizes[lcgNext(rng) % kFrameSizes.size()];
        void* p = mono.allocate(sz, 32);
        benchmark::DoNotOptimize(p);
      }
      current = (current + 1) % frames.size();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kScratchCyclesPerIteration));
}
BENCHMARK(BM_Iteration_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 11: Raw heap allocators — Arena replacement for per-step scratch
//
// Benchmark: allocate variable scratch sizes through DART's frame arena and
// compare against foonathan raw heap/malloc/new allocators. DART deliberately
// uses bulk reset here; this is the HMM replacement for per-object heap frees
// inside simulation hot loops.
// =============================================================================

static constexpr auto kRawSizes
    = std::to_array<size_t>({32, 64, 128, 256, 512});
static constexpr size_t kRawCyclesPerIteration = 128;

static void runRawDartFrame(benchmark::State& state, const uint32_t seed)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 768 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    uint32_t rng = seed;
    for (size_t cycle = 0; cycle < kRawCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = alloc.allocate(sz);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kRawCyclesPerIteration));
}

template <typename RawAllocator>
static void runRawFoonathan(benchmark::State& state, const uint32_t seed)
{
  const auto count = static_cast<size_t>(state.range(0));
  RawAllocator alloc;
  std::vector<std::pair<void*, size_t>> ptrs(count);

  for (auto _ : state) {
    uint32_t rng = seed;
    for (size_t cycle = 0; cycle < kRawCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = fm::allocator_traits<RawAllocator>::allocate_node(
            alloc, sz, alignof(std::max_align_t));
        ptrs[i] = {p, sz};
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        auto& entry = ptrs[i - 1];
        fm::allocator_traits<RawAllocator>::deallocate_node(
            alloc, entry.first, entry.second, alignof(std::max_align_t));
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kRawCyclesPerIteration));
}

static void runRawStdPmrNewDelete(benchmark::State& state, const uint32_t seed)
{
  const auto count = static_cast<size_t>(state.range(0));
  std::pmr::memory_resource* resource = std::pmr::new_delete_resource();
  std::vector<std::pair<void*, size_t>> ptrs(count);

  for (auto _ : state) {
    uint32_t rng = seed;
    for (size_t cycle = 0; cycle < kRawCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = resource->allocate(sz, alignof(std::max_align_t));
        ptrs[i] = {p, sz};
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        auto& entry = ptrs[i - 1];
        resource->deallocate(
            entry.first, entry.second, alignof(std::max_align_t));
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kRawCyclesPerIteration));
}

static void runRawStdMalloc(benchmark::State& state, const uint32_t seed)
{
  const auto count = static_cast<size_t>(state.range(0));
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    uint32_t rng = seed;
    for (size_t cycle = 0; cycle < kRawCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = std::malloc(sz);
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        std::free(ptrs[i - 1]);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kRawCyclesPerIteration));
}

static void runRawStdNew(benchmark::State& state, const uint32_t seed)
{
  const auto count = static_cast<size_t>(state.range(0));
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    uint32_t rng = seed;
    for (size_t cycle = 0; cycle < kRawCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = ::operator new(sz);
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        ::operator delete(ptrs[i - 1]);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kRawCyclesPerIteration));
}

static void BM_RawHeap_DART(benchmark::State& state)
{
  runRawDartFrame(state, 79u);
}
BENCHMARK(BM_RawHeap_DART)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_RawHeap_Foonathan(benchmark::State& state)
{
  runRawFoonathan<fm::heap_allocator>(state, 79u);
}
BENCHMARK(BM_RawHeap_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_RawHeap_StdPmr(benchmark::State& state)
{
  runRawStdPmrNewDelete(state, 79u);
}
BENCHMARK(BM_RawHeap_StdPmr)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_RawMalloc_DART(benchmark::State& state)
{
  runRawDartFrame(state, 83u);
}
BENCHMARK(BM_RawMalloc_DART)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_RawMalloc_Foonathan(benchmark::State& state)
{
  runRawFoonathan<fm::malloc_allocator>(state, 83u);
}
BENCHMARK(BM_RawMalloc_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_RawMalloc_StdPmr(benchmark::State& state)
{
  runRawStdMalloc(state, 83u);
}
BENCHMARK(BM_RawMalloc_StdPmr)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_RawNew_DART(benchmark::State& state)
{
  runRawDartFrame(state, 89u);
}
BENCHMARK(BM_RawNew_DART)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_RawNew_Foonathan(benchmark::State& state)
{
  runRawFoonathan<fm::new_allocator>(state, 89u);
}
BENCHMARK(BM_RawNew_Foonathan)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_RawNew_StdPmr(benchmark::State& state)
{
  runRawStdNew(state, 89u);
}
BENCHMARK(BM_RawNew_StdPmr)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

// =============================================================================
// Section 12: Allocator adapters — Alignment, fallback, and segregation
//
// Benchmark: adapter patterns foonathan exposes as distinct allocators. DART
// maps these to direct frame or pool allocator policies instead of stacking
// generic raw-allocator adapters on hot paths.
// =============================================================================

static constexpr auto kSegregatorSizes
    = std::to_array<size_t>({32, 64, 128, 256, 512, 1024});
static constexpr size_t kFallbackStackSize = 32;
static constexpr size_t kTrackedStackSize = 32;
static constexpr size_t kAlignedStackCyclesPerIteration = 512;
static constexpr size_t kFallbackStackCyclesPerIteration = 2048;
static constexpr size_t kSegregatorCyclesPerIteration = 256;

static void BM_AlignedStack_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 768 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    uint32_t rng = 97u;
    for (size_t cycle = 0; cycle < kAlignedStackCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = alloc.allocateCacheAligned(sz);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kAlignedStackCyclesPerIteration));
}
BENCHMARK(BM_AlignedStack_DART)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_AlignedStack_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 768 + 4096;
  fm::memory_stack<> stack(arenaSize);
  auto ref = fm::make_allocator_reference(stack);
  fm::aligned_allocator<decltype(ref)> alloc(64, std::move(ref));

  for (auto _ : state) {
    uint32_t rng = 97u;
    for (size_t cycle = 0; cycle < kAlignedStackCyclesPerIteration; ++cycle) {
      auto marker = stack.top();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = alloc.allocate_node(sz, 32);
        benchmark::DoNotOptimize(p);
      }
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kAlignedStackCyclesPerIteration));
}
BENCHMARK(BM_AlignedStack_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_AlignedStack_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 768 + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    uint32_t rng = 97u;
    for (size_t cycle = 0; cycle < kAlignedStackCyclesPerIteration; ++cycle) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::null_memory_resource());
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kRawSizes[lcgNext(rng) % kRawSizes.size()];
        void* p = mono.allocate(sz, 64);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kAlignedStackCyclesPerIteration));
}
BENCHMARK(BM_AlignedStack_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_FallbackStack_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 64 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kFallbackStackCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate(kFallbackStackSize);
        benchmark::DoNotOptimize(p);
      }
      alloc.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kFallbackStackCyclesPerIteration));
}
BENCHMARK(BM_FallbackStack_DART)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_FallbackStack_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 64 + 4096;
  fm::memory_stack<> stack(arenaSize);
  auto ref = fm::make_allocator_reference(stack);
  fm::fallback_allocator<decltype(ref), fm::heap_allocator> alloc(
      std::move(ref), fm::heap_allocator{});

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kFallbackStackCyclesPerIteration; ++cycle) {
      auto marker = stack.top();
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate_node(kFallbackStackSize, 32);
        benchmark::DoNotOptimize(p);
      }
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kFallbackStackCyclesPerIteration));
}
BENCHMARK(BM_FallbackStack_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_FallbackStack_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 64 + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kFallbackStackCyclesPerIteration; ++cycle) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::new_delete_resource());
      for (size_t i = 0; i < count; ++i) {
        void* p = mono.allocate(kFallbackStackSize, 32);
        benchmark::DoNotOptimize(p);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kFallbackStackCyclesPerIteration));
}
BENCHMARK(BM_FallbackStack_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_Segregator_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  PoolAllocator alloc(
      MemoryAllocator::GetDefault(),
      PoolAllocator::DiagnosticsPolicy::Disabled);
  std::vector<std::pair<void*, size_t>> ptrs(count);

  lcgState = 103u;
  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kSegregatorCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kSegregatorSizes[lcgNext() % kSegregatorSizes.size()];
        void* p = alloc.allocateUntracked(sz);
        ptrs[i] = {p, sz};
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        auto& entry = ptrs[i - 1];
        alloc.deallocateUntracked(entry.first, entry.second);
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kSegregatorCyclesPerIteration));
}
BENCHMARK(BM_Segregator_DART)->Arg(256)->Arg(1024)->ReportAggregatesOnly(true);

static void BM_Segregator_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  fm::memory_pool_collection<fm::node_pool, fm::identity_buckets> smallPool(
      256, count * 512 + 4096);
  fm::memory_stack<> largeStack(count * 2048 + 4096);
  auto smallRef = fm::make_allocator_reference(smallPool);
  auto largeRef = fm::make_allocator_reference(largeStack);
  auto alloc = fm::make_segregator(
      fm::threshold(256, std::move(smallRef)), std::move(largeRef));
  std::vector<std::pair<void*, size_t>> ptrs(count);

  lcgState = 103u;
  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kSegregatorCyclesPerIteration; ++cycle) {
      auto marker = largeStack.top();
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kSegregatorSizes[lcgNext() % kSegregatorSizes.size()];
        void* p = alloc.allocate_node(sz, alignof(std::max_align_t));
        ptrs[i] = {p, sz};
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        auto& entry = ptrs[i - 1];
        alloc.deallocate_node(
            entry.first, entry.second, alignof(std::max_align_t));
      }
      largeStack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kSegregatorCyclesPerIteration));
}
BENCHMARK(BM_Segregator_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_Segregator_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  std::pmr::pool_options opts{};
  opts.largest_required_pool_block = 1024;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<std::pair<void*, size_t>> ptrs(count);

  lcgState = 103u;
  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kSegregatorCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        const size_t sz = kSegregatorSizes[lcgNext() % kSegregatorSizes.size()];
        void* p = pool.allocate(sz, alignof(std::max_align_t));
        ptrs[i] = {p, sz};
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        auto& entry = ptrs[i - 1];
        pool.deallocate(entry.first, entry.second, alignof(std::max_align_t));
      }
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kSegregatorCyclesPerIteration));
}
BENCHMARK(BM_Segregator_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 13: Tracking adapters — Allocation diagnostics in hot paths
// =============================================================================

static constexpr size_t kTrackedCyclesPerIteration = 4096;
static constexpr size_t kDeepTrackedPoolCyclesPerIteration = 512;

static void BM_TrackedStack_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 64 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);
  BenchmarkAllocationTracker tracker;

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kTrackedCyclesPerIteration; ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate(kTrackedStackSize);
        tracker.on_node_allocation(p, kTrackedStackSize, 32);
        benchmark::DoNotOptimize(p);
      }
      tracker.on_bulk_deallocation(count, kTrackedStackSize, 32);
      alloc.reset();
    }
  }
  benchmark::DoNotOptimize(tracker);
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kTrackedCyclesPerIteration));
}
BENCHMARK(BM_TrackedStack_DART)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_TrackedStack_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 64 + 4096;
  fm::memory_stack<> stack(arenaSize);
  auto ref = fm::make_allocator_reference(stack);
  fm::tracked_allocator<BenchmarkAllocationTracker, decltype(ref)> alloc(
      BenchmarkAllocationTracker{}, std::move(ref));
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kTrackedCyclesPerIteration; ++cycle) {
      auto marker = stack.top();
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate_node(kTrackedStackSize, 32);
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        alloc.deallocate_node(ptrs[i - 1], kTrackedStackSize, 32);
      }
      stack.unwind(marker);
    }
  }
  benchmark::DoNotOptimize(alloc.get_tracker());
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kTrackedCyclesPerIteration));
}
BENCHMARK(BM_TrackedStack_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_TrackedStack_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 64 + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);
  BenchmarkAllocationTracker tracker;
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kTrackedCyclesPerIteration; ++cycle) {
      std::pmr::monotonic_buffer_resource mono(
          backing.get(), arenaSize, std::pmr::null_memory_resource());
      for (size_t i = 0; i < count; ++i) {
        void* p = mono.allocate(kTrackedStackSize, 32);
        tracker.on_node_allocation(p, kTrackedStackSize, 32);
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        tracker.on_node_deallocation(ptrs[i - 1], kTrackedStackSize, 32);
        mono.deallocate(ptrs[i - 1], kTrackedStackSize, 32);
      }
    }
  }
  benchmark::DoNotOptimize(tracker);
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kTrackedCyclesPerIteration));
}
BENCHMARK(BM_TrackedStack_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_DeepTrackedPool_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t size = 64;
  const size_t blockSize = (size + 16) * count + 4096;
  FixedPoolAllocator alloc(size, MemoryAllocator::GetDefault(), blockSize);
  BenchmarkAllocationTracker tracker;
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kDeepTrackedPoolCyclesPerIteration;
         ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate();
        tracker.on_node_allocation(p, size, alignof(std::max_align_t));
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        tracker.on_node_deallocation(
            ptrs[i - 1], size, alignof(std::max_align_t));
        alloc.deallocate(ptrs[i - 1]);
      }
    }
  }
  benchmark::DoNotOptimize(tracker);
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kDeepTrackedPoolCyclesPerIteration));
}
BENCHMARK(BM_DeepTrackedPool_DART)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_DeepTrackedPool_Foonathan(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t size = 64;
  const size_t blockSize = (size + 16) * count + 4096;
  auto alloc
      = fm::make_deeply_tracked_allocator<fm::memory_pool<fm::node_pool>>(
          BenchmarkAllocationTracker{}, size, blockSize);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kDeepTrackedPoolCyclesPerIteration;
         ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        void* p = alloc.allocate_node(size, alignof(std::max_align_t));
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        alloc.deallocate_node(ptrs[i - 1], size, alignof(std::max_align_t));
      }
    }
  }
  benchmark::DoNotOptimize(alloc.get_tracker());
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kDeepTrackedPoolCyclesPerIteration));
}
BENCHMARK(BM_DeepTrackedPool_Foonathan)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

static void BM_DeepTrackedPool_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t size = 64;
  std::pmr::pool_options opts{};
  opts.max_blocks_per_chunk = count;
  opts.largest_required_pool_block = size;
  std::pmr::unsynchronized_pool_resource pool(opts);
  BenchmarkAllocationTracker tracker;
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < kDeepTrackedPoolCyclesPerIteration;
         ++cycle) {
      for (size_t i = 0; i < count; ++i) {
        void* p = pool.allocate(size, alignof(std::max_align_t));
        tracker.on_node_allocation(p, size, alignof(std::max_align_t));
        ptrs[i] = p;
        benchmark::DoNotOptimize(p);
      }
      for (size_t i = count; i > 0; --i) {
        tracker.on_node_deallocation(
            ptrs[i - 1], size, alignof(std::max_align_t));
        pool.deallocate(ptrs[i - 1], size, alignof(std::max_align_t));
      }
    }
  }
  benchmark::DoNotOptimize(tracker);
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * count * kDeepTrackedPoolCyclesPerIteration));
}
BENCHMARK(BM_DeepTrackedPool_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 14: EnTT Registry — Component storage through allocator adapters
//
// Benchmark: create entities, attach multiple World-like components, read them
// back through cached storage handles, then destroy the entities. The registry
// is pre-warmed so the measured loop represents steady-state simulation churn
// instead of one-time storage discovery.
// =============================================================================
#if defined(DART_BENCHMARK_HAS_ENTT) && DART_BENCHMARK_HAS_ENTT

struct EnttRegistryTransform
{
  double position[3] = {};
  double rotation[4] = {};
};

struct EnttRegistryVelocity
{
  double linear[3] = {};
  double angular[3] = {};
};

struct EnttRegistryMass
{
  double value = 1.0;
  double inertia[9] = {};
};

struct EnttRegistryTag
{
};

constexpr size_t EnttRegistryStorageCount = 4;
constexpr size_t kEnttRegistryCyclesPerIteration = 16;
constexpr size_t kEnttRegistryBuildCyclesPerIteration = 16;

[[nodiscard]] constexpr size_t enttRegistryCyclesFor(
    size_t entityCount) noexcept
{
  (void)entityCount;
  return kEnttRegistryCyclesPerIteration;
}

[[nodiscard]] constexpr size_t enttRegistryBuildCyclesFor(
    size_t entityCount) noexcept
{
  (void)entityCount;
  return kEnttRegistryBuildCyclesPerIteration;
}

template <typename Registry>
void reserveEnttRegistryStorage(Registry& registry, const size_t entityCount)
{
  registry.template storage<entt::entity>().reserve(entityCount);
  registry.template storage<EnttRegistryTransform>().reserve(entityCount);
  registry.template storage<EnttRegistryVelocity>().reserve(entityCount);
  registry.template storage<EnttRegistryMass>().reserve(entityCount);
  registry.template storage<EnttRegistryTag>().reserve(entityCount);
}

template <typename Registry>
void runEnttRegistryChurnWithStorage(
    Registry& registry,
    auto& entityStorage,
    auto& transforms,
    auto& velocities,
    auto& masses,
    auto& tags,
    auto& entities,
    const size_t entityCount)
{
  for (size_t i = 0; i < entityCount; ++i) {
    const auto entity = registry.create();
    entities[i] = entity;
    auto& transform = transforms.emplace(entity);
    transform.position[0] = static_cast<double>(i);
    transform.position[1] = static_cast<double>(i + 1);
    transform.position[2] = static_cast<double>(i + 2);
    transform.rotation[3] = 1.0;

    auto& velocity = velocities.emplace(entity);
    velocity.linear[0] = static_cast<double>(i & 7u);
    velocity.angular[2] = static_cast<double>((i + 3u) & 15u);

    auto& mass = masses.emplace(entity);
    mass.value = 1.0 + static_cast<double>(i % 17u);
    mass.inertia[0] = mass.value;

    tags.emplace(entity);
  }

  double total = 0.0;
  for (size_t i = 0; i < entityCount; ++i) {
    const auto entity = entities[i];
    const auto& transform = transforms.get(entity);
    const auto& velocity = velocities.get(entity);
    const auto& mass = masses.get(entity);
    total += transform.position[0] + velocity.linear[0] + mass.value;
    benchmark::DoNotOptimize(tags.contains(entity));
  }
  benchmark::DoNotOptimize(total);

  for (size_t i = entityCount; i > 0; --i) {
    const auto entity = entities[i - 1];
    // The benchmark has no EnTT groups/signals; remove the known storages
    // directly to keep the hot path symmetric with the cached emplacement/read
    // path above.
    tags.erase(entity);
    masses.erase(entity);
    velocities.erase(entity);
    transforms.erase(entity);
    entityStorage.erase(entity);
  }
  benchmark::ClobberMemory();
}

template <typename Registry>
void runEnttRegistryChurn(
    Registry& registry, auto& entities, const size_t entityCount)
{
  // World systems know their component set. Cache storage handles so the hot
  // path measures storage allocation/layout rather than repeated type lookups.
  auto& entityStorage = registry.template storage<entt::entity>();
  auto& transforms = registry.template storage<EnttRegistryTransform>();
  auto& velocities = registry.template storage<EnttRegistryVelocity>();
  auto& masses = registry.template storage<EnttRegistryMass>();
  auto& tags = registry.template storage<EnttRegistryTag>();
  runEnttRegistryChurnWithStorage(
      registry,
      entityStorage,
      transforms,
      velocities,
      masses,
      tags,
      entities,
      entityCount);
}

template <typename Registry>
void prewarmEnttRegistry(
    Registry& registry, auto& entities, const size_t entityCount)
{
  reserveEnttRegistryStorage(registry, entityCount);
  runEnttRegistryChurn(registry, entities, entityCount);
}

static void BM_EnttRegistry_DART(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  // Persistent registry storage reserves variable-size arrays. Model it with
  // the world free-list arena directly; the fixed-size pool remains for small
  // same-size/node-like allocations elsewhere in the HMM hierarchy.
  FreeListAllocator worldArena(
      MemoryAllocator::GetDefault(), entityCount * 4096 + 1024 * 1024);
  BenchmarkCountingMemoryAllocator backing(worldArena);
  StlAllocator<entt::entity> allocator(backing);
  entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
      EnttRegistryStorageCount, allocator);

  auto runBenchmark = [&](auto& entities) {
    const size_t cycleCount = enttRegistryCyclesFor(entityCount);
    prewarmEnttRegistry(registry, entities, entityCount);
    backing.resetCounts();
    auto& entityStorage = registry.storage<entt::entity>();
    auto& transforms = registry.storage<EnttRegistryTransform>();
    auto& velocities = registry.storage<EnttRegistryVelocity>();
    auto& masses = registry.storage<EnttRegistryMass>();
    auto& tags = registry.storage<EnttRegistryTag>();

    for (auto _ : state) {
      for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
        runEnttRegistryChurnWithStorage(
            registry,
            entityStorage,
            transforms,
            velocities,
            masses,
            tags,
            entities,
            entityCount);
      }
    }
    if (backing.allocationCount != 0 || backing.deallocationCount != 0) {
      state.SkipWithError(
          "Reserved DART EnTT registry churn used allocator-backed storage.");
    }
  };

  // Keep the driver scratch outside the allocator under test, matching the
  // foonathan and std rows. The DART row below measures only allocator-aware
  // EnTT registry storage and verifies that the reserved pool allocator does
  // not grow during steady-state churn.
  std::vector<entt::entity> entities(entityCount);
  runBenchmark(entities);
  state.counters["dart_allocator_allocations_per_iter"]
      = static_cast<double>(backing.allocationCount);
  state.counters["dart_allocator_deallocations_per_iter"]
      = static_cast<double>(backing.deallocationCount);
  state.SetItemsProcessed(
      static_cast<int64_t>(
          state.iterations() * entityCount
          * enttRegistryCyclesFor(entityCount)));
}
BENCHMARK(BM_EnttRegistry_DART)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistryBuild_DART(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  const size_t cycleCount = enttRegistryBuildCyclesFor(entityCount);
  // Registry build/growth models one-shot storage assembly. Use a resettable
  // frame-backed bake arena so the DART row matches the foonathan memory_stack
  // marker/unwind lifetime below.
  const size_t arenaBytes = entityCount * 4096 + 1024 * 1024;
  FrameAllocator bakeArena(MemoryAllocator::GetDefault(), arenaBytes);
  StlAllocator<entt::entity> allocator(bakeArena);
  std::vector<entt::entity> entities(entityCount);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      {
        entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
            EnttRegistryStorageCount, allocator);
        reserveEnttRegistryStorage(registry, entityCount);
        runEnttRegistryChurn(registry, entities, entityCount);
      }
      bakeArena.reset();
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount * cycleCount));
}
BENCHMARK(BM_EnttRegistryBuild_DART)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistry_Foonathan(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  const size_t maxNodeSize = std::max<size_t>(sizeof(EnttRegistryMass), 8192);
  const size_t blockSize = entityCount * 4096 + 16 * 1024 * 1024;
  fm::memory_pool_collection<fm::array_pool, fm::log2_buckets> pool(
      maxNodeSize, blockSize);
  fm::std_allocator<
      entt::entity,
      fm::memory_pool_collection<fm::array_pool, fm::log2_buckets>>
      allocator(pool);
  entt::basic_registry<entt::entity, decltype(allocator)> registry(
      EnttRegistryStorageCount, allocator);
  std::vector<entt::entity> entities(entityCount);

  prewarmEnttRegistry(registry, entities, entityCount);
  const size_t cycleCount = enttRegistryCyclesFor(entityCount);
  auto& entityStorage = registry.storage<entt::entity>();
  auto& transforms = registry.storage<EnttRegistryTransform>();
  auto& velocities = registry.storage<EnttRegistryVelocity>();
  auto& masses = registry.storage<EnttRegistryMass>();
  auto& tags = registry.storage<EnttRegistryTag>();

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      runEnttRegistryChurnWithStorage(
          registry,
          entityStorage,
          transforms,
          velocities,
          masses,
          tags,
          entities,
          entityCount);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount * cycleCount));
}
BENCHMARK(BM_EnttRegistry_Foonathan)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistryBuild_Foonathan(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  const size_t cycleCount = enttRegistryBuildCyclesFor(entityCount);
  const size_t stackBytes = entityCount * 4096 + 1024 * 1024;
  fm::memory_stack<> stack(fm::memory_stack<>::min_block_size(stackBytes));
  fm::std_allocator<entt::entity, fm::memory_stack<>> allocator(stack);
  std::vector<entt::entity> entities(entityCount);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      const auto marker = stack.top();
      {
        entt::basic_registry<entt::entity, decltype(allocator)> registry(
            EnttRegistryStorageCount, allocator);
        reserveEnttRegistryStorage(registry, entityCount);
        runEnttRegistryChurn(registry, entities, entityCount);
      }
      stack.unwind(marker);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount * cycleCount));
}
BENCHMARK(BM_EnttRegistryBuild_Foonathan)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistry_Std(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  entt::registry registry(EnttRegistryStorageCount);
  std::vector<entt::entity> entities(entityCount);

  prewarmEnttRegistry(registry, entities, entityCount);
  const size_t cycleCount = enttRegistryCyclesFor(entityCount);
  auto& entityStorage = registry.storage<entt::entity>();
  auto& transforms = registry.storage<EnttRegistryTransform>();
  auto& velocities = registry.storage<EnttRegistryVelocity>();
  auto& masses = registry.storage<EnttRegistryMass>();
  auto& tags = registry.storage<EnttRegistryTag>();

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      runEnttRegistryChurnWithStorage(
          registry,
          entityStorage,
          transforms,
          velocities,
          masses,
          tags,
          entities,
          entityCount);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount * cycleCount));
}
BENCHMARK(BM_EnttRegistry_Std)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistryBuild_Std(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  const size_t cycleCount = enttRegistryBuildCyclesFor(entityCount);
  std::vector<entt::entity> entities(entityCount);

  for (auto _ : state) {
    for (size_t cycle = 0; cycle < cycleCount; ++cycle) {
      entt::registry registry(EnttRegistryStorageCount);
      reserveEnttRegistryStorage(registry, entityCount);
      runEnttRegistryChurn(registry, entities, entityCount);
    }
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount * cycleCount));
}
BENCHMARK(BM_EnttRegistryBuild_Std)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

#endif

BENCHMARK_MAIN();
