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
///   8. EnTT registry workload
///
/// Run:  pixi run bm -- allocators-comparative
/// JSON: pixi run bm -- allocators-comparative
///         --benchmark_out=.benchmark_results/comparative.json
///         --benchmark_out_format=json

#include <dart/common/fixed_pool_allocator.hpp>
#include <dart/common/frame_allocator.hpp>
#include <dart/common/memory_allocator.hpp>
#include <dart/common/pool_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <benchmark/benchmark.h>
#include <foonathan/memory/memory_pool.hpp>
#include <foonathan/memory/memory_pool_collection.hpp>
#include <foonathan/memory/memory_stack.hpp>
#include <foonathan/memory/namespace_alias.hpp>
#include <foonathan/memory/std_allocator.hpp>

#if defined(DART_BENCHMARK_HAS_ENTT) && DART_BENCHMARK_HAS_ENTT
  #include <entt/entity/registry.hpp>
#endif

#include <algorithm>
#include <array>
#include <memory_resource>
#include <string_view>
#include <vector>

#include <cstddef>
#include <cstdint>

using namespace dart::common;
namespace fm = foonathan::memory;

// =============================================================================
// Simple LCG random (no heap allocations unlike std::mt19937)
// =============================================================================

static uint32_t lcgState = 12345u;

inline uint32_t lcgNext()
{
  lcgState = lcgState * 1664525u + 1013904223u;
  return lcgState;
}

class BenchmarkCountingPoolMemoryAllocator final : public MemoryAllocator
{
public:
  std::string_view getType() const override
  {
    return "BenchmarkCountingPoolMemoryAllocator";
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
  PoolAllocator mBacking;
};

// =============================================================================
// Section 1: Pool — Fixed-Size Alloc/Dealloc
//
// Benchmark: allocate N nodes of size S, then deallocate all (LIFO).
// Compares single-size pool performance.
// =============================================================================

static void BM_Pool_DART(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t blockSize = (size + 16) * count + 4096;
  FixedPoolAllocator alloc(size, MemoryAllocator::GetDefault(), blockSize);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = alloc.allocate();
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      alloc.deallocate(ptrs[i - 1]);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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
  const size_t blockSize = (size + 16) * count + 4096;
  fm::memory_pool<fm::node_pool> pool(size, blockSize);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = pool.allocate_node();
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      pool.deallocate_node(ptrs[i - 1]);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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
  std::pmr::pool_options opts{};
  opts.max_blocks_per_chunk = count;
  opts.largest_required_pool_block = size;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = pool.allocate(size, alignof(std::max_align_t));
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      pool.deallocate(ptrs[i - 1], size, alignof(std::max_align_t));
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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

static void BM_Stack_DART(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t arenaSize = count * ((size + 31) & ~size_t{31}) + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      void* p = alloc.allocate(size);
      benchmark::DoNotOptimize(p);
    }
    alloc.reset();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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
    auto marker = stack.top();
    for (size_t i = 0; i < count; ++i) {
      void* p = stack.allocate(size, 32);
      benchmark::DoNotOptimize(p);
    }
    stack.unwind(marker);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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
    // Reconstruct each iteration (monotonic_buffer_resource has no reset())
    std::pmr::monotonic_buffer_resource mono(
        backing.get(), arenaSize, std::pmr::null_memory_resource());
    for (size_t i = 0; i < count; ++i) {
      void* p = mono.allocate(size, 32);
      benchmark::DoNotOptimize(p);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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
  PoolAllocator alloc(MemoryAllocator::GetDefault());
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizes.size()];
      ptrs[i] = {alloc.allocate(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kMultiOps; i > 0; --i) {
      alloc.deallocate(ptrs[i - 1].first, ptrs[i - 1].second);
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
// Simulates a full physics step allocation pattern.
// =============================================================================

static constexpr auto kRealisticSizes
    = std::to_array<size_t>({16, 32, 64, 128, 256, 512, 1024});
static constexpr size_t kRealisticOps = 1000;

static void BM_Realistic_DART(benchmark::State& state)
{
  PoolAllocator alloc(MemoryAllocator::GetDefault());
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizes.size()];
      ptrs[i] = {alloc.allocate(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kRealisticOps; i > 0; --i) {
      const size_t idx = lcgNext() % i;
      alloc.deallocate(ptrs[idx].first, ptrs[idx].second);
      ptrs[idx] = ptrs[i - 1];
    }
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * kRealisticOps));
}
BENCHMARK(BM_Realistic_DART)->ReportAggregatesOnly(true)->MinTime(0.1);

static void BM_Realistic_Foonathan(benchmark::State& state)
{
  fm::memory_pool_collection<fm::node_pool, fm::identity_buckets> pool(
      1024, 2097152);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
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
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * kRealisticOps));
}
BENCHMARK(BM_Realistic_Foonathan)->ReportAggregatesOnly(true)->MinTime(0.1);

static void BM_Realistic_StdPmr(benchmark::State& state)
{
  std::pmr::pool_options opts{};
  opts.largest_required_pool_block = 1024;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
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
    benchmark::ClobberMemory();
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * kRealisticOps));
}
BENCHMARK(BM_Realistic_StdPmr)->ReportAggregatesOnly(true)->MinTime(0.1);

// =============================================================================
// Section 5: Steady-State — Pre-Filled Pool + Random Replace
//
// Benchmark: fill pool with N objects, then repeatedly dealloc+realloc random
// slots. Simulates a running simulation with object churn.
// =============================================================================

static void BM_SteadyState_DART(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));
  PoolAllocator alloc(MemoryAllocator::GetDefault());
  const size_t ops = poolSize * 4;

  std::vector<void*> ptrs(poolSize);
  for (size_t i = 0; i < poolSize; ++i) {
    ptrs[i] = alloc.allocate(size);
  }

  lcgState = 42u;
  for (auto _ : state) {
    for (size_t i = 0; i < ops; ++i) {
      const size_t idx = lcgNext() % poolSize;
      alloc.deallocate(ptrs[idx], size);
      ptrs[idx] = alloc.allocate(size);
      benchmark::DoNotOptimize(ptrs[idx]);
    }
  }

  for (size_t i = 0; i < poolSize; ++i) {
    alloc.deallocate(ptrs[i], size);
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
  const size_t ops = poolSize * 4;

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
  const size_t ops = poolSize * 4;

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

static void BM_FrameBulk_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  lcgState = 55u;
  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      const size_t sz = kFrameSizes[lcgNext() % kFrameSizes.size()];
      void* p = alloc.allocateAligned(sz, 32);
      benchmark::DoNotOptimize(p);
    }
    alloc.reset();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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

  lcgState = 55u;
  for (auto _ : state) {
    auto marker = stack.top();
    for (size_t i = 0; i < count; ++i) {
      const size_t sz = kFrameSizes[lcgNext() % kFrameSizes.size()];
      void* p = stack.allocate(sz, 32);
      benchmark::DoNotOptimize(p);
    }
    stack.unwind(marker);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
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

  lcgState = 55u;
  for (auto _ : state) {
    std::pmr::monotonic_buffer_resource mono(
        backing.get(), arenaSize, std::pmr::null_memory_resource());
    for (size_t i = 0; i < count; ++i) {
      const size_t sz = kFrameSizes[lcgNext() % kFrameSizes.size()];
      void* p = mono.allocate(sz, 32);
      benchmark::DoNotOptimize(p);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
}
BENCHMARK(BM_FrameBulk_StdPmr)
    ->Arg(256)
    ->Arg(1024)
    ->Arg(4096)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 7: STL Vector — Container allocation through allocator adapters
//
// Benchmark: push_back 1000 ints into a vector, accumulate, then discard.
// Tests real-world STL integration overhead.
// =============================================================================

static void BM_StlVector_DART(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) * 4 + 4096;
  FrameAllocator arena(MemoryAllocator::GetDefault(), arenaSize);

  for (auto _ : state) {
    {
      FrameStlAllocator<int> frameAlloc(arena);
      std::vector<int, FrameStlAllocator<int>> vec(frameAlloc);
      vec.reserve(n);
      for (size_t i = 0; i < n; ++i) {
        vec.push_back(static_cast<int>(i));
      }
      int sum = 0;
      for (const auto& v : vec) {
        sum += v;
      }
      benchmark::DoNotOptimize(sum);
    }
    arena.reset();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n));
}
BENCHMARK(BM_StlVector_DART)->Arg(1000)->Arg(10000)->ReportAggregatesOnly(true);

static void BM_StlVector_Foonathan(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) * 4 + 4096;
  fm::memory_stack<> stack(arenaSize);

  for (auto _ : state) {
    auto marker = stack.top();
    {
      fm::std_allocator<int, fm::memory_stack<>> alloc(stack);
      std::vector<int, fm::std_allocator<int, fm::memory_stack<>>> vec(alloc);
      vec.reserve(n);
      for (size_t i = 0; i < n; ++i) {
        vec.push_back(static_cast<int>(i));
      }
      int sum = 0;
      for (const auto& v : vec) {
        sum += v;
      }
      benchmark::DoNotOptimize(sum);
    }
    stack.unwind(marker);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n));
}
BENCHMARK(BM_StlVector_Foonathan)
    ->Arg(1000)
    ->Arg(10000)
    ->ReportAggregatesOnly(true);

static void BM_StlVector_StdPmr(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) * 4 + 4096;
  auto backing = std::make_unique_for_overwrite<char[]>(arenaSize);

  for (auto _ : state) {
    std::pmr::monotonic_buffer_resource mono(
        backing.get(), arenaSize, std::pmr::null_memory_resource());
    {
      std::pmr::vector<int> vec(&mono);
      vec.reserve(n);
      for (size_t i = 0; i < n; ++i) {
        vec.push_back(static_cast<int>(i));
      }
      int sum = 0;
      for (const auto& v : vec) {
        sum += v;
      }
      benchmark::DoNotOptimize(sum);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n));
}
BENCHMARK(BM_StlVector_StdPmr)
    ->Arg(1000)
    ->Arg(10000)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 8: EnTT Registry — Component storage through allocator adapters
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

template <typename Registry>
void reserveEnttRegistryStorage(Registry& registry, const size_t entityCount)
{
  registry.template storage<EnttRegistryTransform>().reserve(entityCount);
  registry.template storage<EnttRegistryVelocity>().reserve(entityCount);
  registry.template storage<EnttRegistryMass>().reserve(entityCount);
  registry.template storage<EnttRegistryTag>().reserve(entityCount);
}

template <typename Registry>
void runEnttRegistryChurn(
    Registry& registry,
    std::vector<entt::entity>& entities,
    const size_t entityCount)
{
  // World systems know their component set. Cache storage handles so the hot
  // path measures storage allocation/layout rather than repeated type lookups.
  auto& entityStorage = registry.template storage<entt::entity>();
  auto& transforms = registry.template storage<EnttRegistryTransform>();
  auto& velocities = registry.template storage<EnttRegistryVelocity>();
  auto& masses = registry.template storage<EnttRegistryMass>();
  auto& tags = registry.template storage<EnttRegistryTag>();

  for (size_t i = 0; i < entityCount; ++i) {
    const auto entity = entityStorage.generate();
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
    tags.remove(entity);
    masses.remove(entity);
    velocities.remove(entity);
    transforms.remove(entity);
    entityStorage.erase(entity);
  }
  benchmark::ClobberMemory();
}

template <typename Registry>
void prewarmEnttRegistry(
    Registry& registry,
    std::vector<entt::entity>& entities,
    const size_t entityCount)
{
  reserveEnttRegistryStorage(registry, entityCount);
  runEnttRegistryChurn(registry, entities, entityCount);
}

static void BM_EnttRegistry_DART(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  FrameAllocator backing(
      MemoryAllocator::GetDefault(), entityCount * 4096 + 1024 * 1024);
  FrameStlAllocator<entt::entity> allocator(backing);
  entt::basic_registry<entt::entity, FrameStlAllocator<entt::entity>> registry(
      EnttRegistryStorageCount, allocator);
  std::vector<entt::entity> entities(entityCount);

  prewarmEnttRegistry(registry, entities, entityCount);
  const size_t usedAfterPrewarm = backing.used();
  const size_t overflowCountAfterPrewarm = backing.overflowCount();
  const size_t overflowBytesAfterPrewarm = backing.overflowBytes();

  for (auto _ : state) {
    runEnttRegistryChurn(registry, entities, entityCount);
  }
  state.counters["dart_frame_bytes"] = static_cast<double>(backing.used());
  state.counters["dart_frame_overflow_count"]
      = static_cast<double>(backing.overflowCount());
  state.counters["dart_frame_overflow_bytes"]
      = static_cast<double>(backing.overflowBytes());
  if (backing.used() != usedAfterPrewarm
      || backing.overflowCount() != overflowCountAfterPrewarm
      || backing.overflowBytes() != overflowBytesAfterPrewarm) {
    state.SkipWithError(
        "Reserved DART EnTT registry churn grew frame-backed storage.");
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount));
}
BENCHMARK(BM_EnttRegistry_DART)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistryBuild_DART(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  PoolAllocator backing;
  StlAllocator<entt::entity> allocator(backing);
  std::vector<entt::entity> entities(entityCount);

  for (auto _ : state) {
    entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
        EnttRegistryStorageCount, allocator);
    reserveEnttRegistryStorage(registry, entityCount);
    runEnttRegistryChurn(registry, entities, entityCount);
  }

  BenchmarkCountingPoolMemoryAllocator countingBacking;
  StlAllocator<entt::entity> countingAllocator(countingBacking);
  std::vector<entt::entity> countingEntities(entityCount);
  {
    entt::basic_registry<entt::entity, StlAllocator<entt::entity>> registry(
        EnttRegistryStorageCount, countingAllocator);
    reserveEnttRegistryStorage(registry, entityCount);
    runEnttRegistryChurn(registry, countingEntities, entityCount);
  }

  state.counters["dart_allocator_allocations_per_iter"]
      = static_cast<double>(countingBacking.allocationCount);
  state.counters["dart_allocator_deallocations_per_iter"]
      = static_cast<double>(countingBacking.deallocationCount);
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount));
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
  entt::basic_registry<entt::entity, decltype(allocator)> registry(allocator);
  std::vector<entt::entity> entities(entityCount);

  prewarmEnttRegistry(registry, entities, entityCount);

  for (auto _ : state) {
    runEnttRegistryChurn(registry, entities, entityCount);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount));
}
BENCHMARK(BM_EnttRegistry_Foonathan)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistryBuild_Foonathan(benchmark::State& state)
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
  std::vector<entt::entity> entities(entityCount);

  for (auto _ : state) {
    entt::basic_registry<entt::entity, decltype(allocator)> registry(allocator);
    reserveEnttRegistryStorage(registry, entityCount);
    runEnttRegistryChurn(registry, entities, entityCount);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount));
}
BENCHMARK(BM_EnttRegistryBuild_Foonathan)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistry_Std(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  entt::registry registry;
  std::vector<entt::entity> entities(entityCount);

  prewarmEnttRegistry(registry, entities, entityCount);

  for (auto _ : state) {
    runEnttRegistryChurn(registry, entities, entityCount);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount));
}
BENCHMARK(BM_EnttRegistry_Std)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

static void BM_EnttRegistryBuild_Std(benchmark::State& state)
{
  const auto entityCount = static_cast<size_t>(state.range(0));
  std::vector<entt::entity> entities(entityCount);

  for (auto _ : state) {
    entt::registry registry;
    reserveEnttRegistryStorage(registry, entityCount);
    runEnttRegistryChurn(registry, entities, entityCount);
  }
  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations() * entityCount));
}
BENCHMARK(BM_EnttRegistryBuild_Std)
    ->Arg(256)
    ->Arg(512)
    ->Arg(2048)
    ->ReportAggregatesOnly(true);

#endif

BENCHMARK_MAIN();
