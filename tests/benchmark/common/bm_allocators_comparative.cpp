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
///   6. STL container workload
///
/// Run:  pixi run bm -- allocators-comparative
/// JSON: pixi run bm -- allocators-comparative
///         --benchmark_out=.benchmark_results/comparative.json
///         --benchmark_out_format=json

#include <dart/common/frame_allocator.hpp>
#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator.hpp>
#include <dart/common/pool_allocator.hpp>

#include <benchmark/benchmark.h>
#include <foonathan/memory/memory_pool.hpp>
#include <foonathan/memory/memory_pool_collection.hpp>
#include <foonathan/memory/memory_stack.hpp>
#include <foonathan/memory/namespace_alias.hpp>
#include <foonathan/memory/std_allocator.hpp>

#include <memory_resource>
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
  PoolAllocator alloc(MemoryAllocator::GetDefault());
  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = alloc.allocate(size);
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      alloc.deallocate(ptrs[i - 1], size);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
}
BENCHMARK(BM_Pool_DART)
    ->Args({32, 64})
    ->Args({256, 256})
    ->Args({32, 1024})
    ->Repetitions(5)
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
    ->Repetitions(5)
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
    ->Repetitions(5)
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
    ->Repetitions(5)
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
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

static void BM_Stack_StdPmr(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  const size_t arenaSize = count * (size + 32) + 4096;
  auto backing = std::make_unique<char[]>(arenaSize);

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
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 3: Multi-Pool — Mixed-Size Alloc/Dealloc (4096 ops)
//
// Benchmark: allocate objects of varying sizes from a multi-bucket pool.
// Tests how well each allocator handles heterogeneous allocation patterns.
// =============================================================================

static constexpr size_t kMultiSizes[] = {16, 32, 64, 128, 256, 512};
static constexpr size_t kMultiSizeCount = 6;
static constexpr size_t kMultiOps = 4096;

static void BM_MultiPool_DART(benchmark::State& state)
{
  PoolAllocator alloc(MemoryAllocator::GetDefault());
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizeCount];
      ptrs[i] = {alloc.allocate(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kMultiOps; i > 0; --i) {
      alloc.deallocate(ptrs[i - 1].first, ptrs[i - 1].second);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * kMultiOps));
}
BENCHMARK(BM_MultiPool_DART)->Repetitions(5)->ReportAggregatesOnly(true);

static void BM_MultiPool_Foonathan(benchmark::State& state)
{
  fm::memory_pool_collection<fm::node_pool, fm::identity_buckets> pool(
      512, 1048576);
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizeCount];
      ptrs[i] = {pool.allocate_node(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
    }
    for (size_t i = kMultiOps; i > 0; --i) {
      pool.deallocate_node(ptrs[i - 1].first, ptrs[i - 1].second);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * kMultiOps));
}
BENCHMARK(BM_MultiPool_Foonathan)->Repetitions(5)->ReportAggregatesOnly(true);

static void BM_MultiPool_StdPmr(benchmark::State& state)
{
  std::pmr::pool_options opts{};
  opts.largest_required_pool_block = 512;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<std::pair<void*, size_t>> ptrs(kMultiOps);

  lcgState = 77u;
  for (auto _ : state) {
    for (size_t i = 0; i < kMultiOps; ++i) {
      const size_t sz = kMultiSizes[lcgNext() % kMultiSizeCount];
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
BENCHMARK(BM_MultiPool_StdPmr)->Repetitions(5)->ReportAggregatesOnly(true);

// =============================================================================
// Section 4: Realistic Mixed Workload
//
// Benchmark: alloc 1000 objects of random sizes, then dealloc in random order.
// Simulates a full physics step allocation pattern.
// =============================================================================

static constexpr size_t kRealisticSizes[] = {16, 32, 64, 128, 256, 512, 1024};
static constexpr size_t kRealisticSizeCount = 7;
static constexpr size_t kRealisticOps = 1000;

static void BM_Realistic_DART(benchmark::State& state)
{
  PoolAllocator alloc(MemoryAllocator::GetDefault());
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizeCount];
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
BENCHMARK(BM_Realistic_DART)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->MinTime(0.1);

static void BM_Realistic_Foonathan(benchmark::State& state)
{
  fm::memory_pool_collection<fm::node_pool, fm::identity_buckets> pool(
      1024, 2097152);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizeCount];
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
BENCHMARK(BM_Realistic_Foonathan)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->MinTime(0.1);

static void BM_Realistic_StdPmr(benchmark::State& state)
{
  std::pmr::pool_options opts{};
  opts.largest_required_pool_block = 1024;
  std::pmr::unsynchronized_pool_resource pool(opts);
  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizeCount];
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
BENCHMARK(BM_Realistic_StdPmr)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true)
    ->MinTime(0.1);

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
    ->Repetitions(5)
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
    ->Repetitions(5)
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
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 6: Frame Bulk Varying — Simulated per-step pattern
//
// Benchmark: allocate objects of varying sizes from bump allocator, then
// reset. Tests the exact usage pattern in DART's ConstraintSolver.
// =============================================================================

static constexpr size_t kFrameSizes[] = {24, 48, 96, 192, 384};
static constexpr size_t kFrameSizeCount = 5;

static void BM_FrameBulk_DART(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  FrameAllocator alloc(MemoryAllocator::GetDefault(), arenaSize);

  lcgState = 55u;
  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      const size_t sz = kFrameSizes[lcgNext() % kFrameSizeCount];
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
    ->Repetitions(5)
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
      const size_t sz = kFrameSizes[lcgNext() % kFrameSizeCount];
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
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

static void BM_FrameBulk_StdPmr(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  const size_t arenaSize = count * 512 + 4096;
  auto backing = std::make_unique<char[]>(arenaSize);

  lcgState = 55u;
  for (auto _ : state) {
    std::pmr::monotonic_buffer_resource mono(
        backing.get(), arenaSize, std::pmr::null_memory_resource());
    for (size_t i = 0; i < count; ++i) {
      const size_t sz = kFrameSizes[lcgNext() % kFrameSizeCount];
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
    ->Repetitions(5)
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
BENCHMARK(BM_StlVector_DART)
    ->Arg(1000)
    ->Arg(10000)
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

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
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

static void BM_StlVector_StdPmr(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) * 4 + 4096;
  auto backing = std::make_unique<char[]>(arenaSize);

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
    ->Repetitions(5)
    ->ReportAggregatesOnly(true);

BENCHMARK_MAIN();
