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

/// @file bm_allocators.cpp
/// @brief Comprehensive allocator benchmark suite for DART.
///
/// Exercises all DART memory allocators across 8 categories:
///   1. Single alloc/dealloc throughput
///   2. Bulk alloc + reset (Frame-specific)
///   3. Mixed alloc/dealloc steady state
///   4. Pool boundary behavior at MAX_UNIT_SIZE=1024
///   5. MemoryManager dispatch overhead
///   6. Construct/destroy via MemoryManager
///   7. STL container workload (FrameStlAllocator)
///   8. Multi-size realistic workload
///
/// Run all:     pixi run bm -- allocators
/// Run subset:  pixi run bm -- allocators --benchmark_filter="BM_Single.*"

#include <dart/common/frame_allocator.hpp>
#include <dart/common/free_list_allocator.hpp>
#include <dart/common/memory_allocator.hpp>
#include <dart/common/memory_manager.hpp>
#include <dart/common/pool_allocator.hpp>

#include <benchmark/benchmark.h>

#include <vector>

#include <cstddef>
#include <cstdint>
#include <cstring>

using namespace dart::common;

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
// Section 1: Single Alloc/Dealloc Throughput
// =============================================================================

static void BM_SingleAlloc_Std(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  auto& alloc = MemoryAllocator::GetDefault();

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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations() * count * size));
}
BENCHMARK(BM_SingleAlloc_Std)
    ->ArgsProduct(
        {{8, 32, 64, 128, 256, 512, 1024, 4096}, {1, 16, 64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_SingleAlloc_FreeList(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  FreeListAllocator alloc(MemoryAllocator::GetDefault(), 1048576);

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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations() * count * size));
}
BENCHMARK(BM_SingleAlloc_FreeList)
    ->ArgsProduct(
        {{8, 32, 64, 128, 256, 512, 1024, 4096}, {1, 16, 64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_SingleAlloc_Pool(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));

  if (size > 1024) {
    state.SkipWithMessage("Pool only supports sizes <= 1024");
    return;
  }

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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations() * count * size));
}
BENCHMARK(BM_SingleAlloc_Pool)
    ->ArgsProduct(
        {{8, 32, 64, 128, 256, 512, 1024, 4096}, {1, 16, 64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_SingleAlloc_Frame(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  // Size the arena to fit all allocations (32-byte aligned each)
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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations() * count * size));
}
BENCHMARK(BM_SingleAlloc_Frame)
    ->ArgsProduct(
        {{8, 32, 64, 128, 256, 512, 1024, 4096}, {1, 16, 64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 2: Bulk Alloc + Reset (Frame-specific)
// =============================================================================

static void BM_FrameBulkReset(benchmark::State& state)
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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations() * count * size));
}
BENCHMARK(BM_FrameBulkReset)
    ->ArgsProduct({{32, 128, 512}, {16, 64, 256, 1024, 4096}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 3: Mixed Alloc/Dealloc Steady State
// =============================================================================

static void BM_MixedSteadyState_Std(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));
  auto& alloc = MemoryAllocator::GetDefault();
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
BENCHMARK(BM_MixedSteadyState_Std)
    ->ArgsProduct({{32, 128, 512}, {64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_MixedSteadyState_FreeList(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));
  FreeListAllocator alloc(MemoryAllocator::GetDefault(), poolSize * size * 2);
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
BENCHMARK(BM_MixedSteadyState_FreeList)
    ->ArgsProduct({{32, 128, 512}, {64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_MixedSteadyState_Pool(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto poolSize = static_cast<size_t>(state.range(1));

  if (size > 1024) {
    state.SkipWithMessage("Pool only supports sizes <= 1024");
    return;
  }

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
BENCHMARK(BM_MixedSteadyState_Pool)
    ->ArgsProduct({{32, 128, 512}, {64, 256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 4: Pool Boundary Behavior (cliff at MAX_UNIT_SIZE=1024)
// =============================================================================

static void BM_PoolBoundary(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  constexpr size_t count = 256;
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
  state.SetBytesProcessed(
      static_cast<int64_t>(state.iterations() * count * size));
}
BENCHMARK(BM_PoolBoundary)
    ->Arg(960)
    ->Arg(1024)
    ->Arg(1025)
    ->Arg(2048)
    ->Arg(4096)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 5: MemoryManager Dispatch Overhead
// =============================================================================

static void BM_MMDispatch_Free(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  MemoryManager mm;

  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = mm.allocateUsingFree(size);
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      mm.deallocateUsingFree(ptrs[i - 1], size);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
}
BENCHMARK(BM_MMDispatch_Free)
    ->ArgsProduct({{64, 512}, {256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_MMDispatch_Pool(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  MemoryManager mm;

  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = mm.allocateUsingPool(size);
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      mm.deallocateUsingPool(ptrs[i - 1], size);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
}
BENCHMARK(BM_MMDispatch_Pool)
    ->ArgsProduct({{64, 512}, {256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_MMDispatch_Frame(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  MemoryManager mm;

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      void* p = mm.allocateUsingFrame(size);
      benchmark::DoNotOptimize(p);
    }
    mm.getFrameAllocator().reset();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
}
BENCHMARK(BM_MMDispatch_Frame)
    ->ArgsProduct({{64, 512}, {256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_MMDispatch_Typed(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  MemoryManager mm;

  std::vector<void*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = mm.allocate(MemoryManager::Type::Free, size);
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = count; i > 0; --i) {
      mm.deallocate(MemoryManager::Type::Free, ptrs[i - 1], size);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
}
BENCHMARK(BM_MMDispatch_Typed)
    ->ArgsProduct({{64, 512}, {256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_DirectFreeList(benchmark::State& state)
{
  const auto size = static_cast<size_t>(state.range(0));
  const auto count = static_cast<size_t>(state.range(1));
  FreeListAllocator alloc(MemoryAllocator::GetDefault(), 1048576);

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
BENCHMARK(BM_DirectFreeList)
    ->ArgsProduct({{64, 512}, {256, 1024}})
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 6: Construct/Destroy via MemoryManager
// =============================================================================

namespace {

struct SmallObj
{
  double a;
  double b;
  double c;
  double d;
}; // 32 bytes

struct MediumObj
{
  double data[16];
}; // 128 bytes

struct LargeObj
{
  double data[64];
}; // 512 bytes

} // namespace

template <typename T>
static void BM_MMConstruct_Free(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  MemoryManager mm;

  std::vector<T*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = mm.constructUsingFree<T>();
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = 0; i < count; ++i) {
      mm.destroyUsingFree(ptrs[i]);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
  state.counters["obj_size"] = static_cast<double>(sizeof(T));
}
BENCHMARK(BM_MMConstruct_Free<SmallObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);
BENCHMARK(BM_MMConstruct_Free<MediumObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);
BENCHMARK(BM_MMConstruct_Free<LargeObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

template <typename T>
static void BM_MMConstruct_Pool(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  MemoryManager mm;

  std::vector<T*> ptrs(count);

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      ptrs[i] = mm.constructUsingPool<T>();
      benchmark::DoNotOptimize(ptrs[i]);
    }
    for (size_t i = 0; i < count; ++i) {
      mm.destroyUsingPool(ptrs[i]);
    }
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
  state.counters["obj_size"] = static_cast<double>(sizeof(T));
}
BENCHMARK(BM_MMConstruct_Pool<SmallObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);
BENCHMARK(BM_MMConstruct_Pool<MediumObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);
BENCHMARK(BM_MMConstruct_Pool<LargeObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

template <typename T>
static void BM_MMConstruct_Frame(benchmark::State& state)
{
  const auto count = static_cast<size_t>(state.range(0));
  MemoryManager mm;

  for (auto _ : state) {
    for (size_t i = 0; i < count; ++i) {
      T* p = mm.constructUsingFrame<T>();
      benchmark::DoNotOptimize(p);
    }
    mm.getFrameAllocator().reset();
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * count));
  state.counters["obj_size"] = static_cast<double>(sizeof(T));
}
BENCHMARK(BM_MMConstruct_Frame<SmallObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);
BENCHMARK(BM_MMConstruct_Frame<MediumObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);
BENCHMARK(BM_MMConstruct_Frame<LargeObj>)
    ->Arg(256)
    ->Arg(1024)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 7: STL Container Workload (FrameStlAllocator)
// =============================================================================

static void BM_StlContainer_Std(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));

  for (auto _ : state) {
    std::vector<int> vec;
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
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n));
}
BENCHMARK(BM_StlContainer_Std)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

static void BM_StlContainer_Frame(benchmark::State& state)
{
  const auto n = static_cast<size_t>(state.range(0));
  const size_t arenaSize = n * sizeof(int) * 2 + 4096;
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
BENCHMARK(BM_StlContainer_Frame)
    ->Arg(100)
    ->Arg(1000)
    ->Arg(10000)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true);

// =============================================================================
// Section 8: Multi-Size Realistic Workload
// =============================================================================

static constexpr size_t kRealisticSizes[] = {16, 32, 64, 128, 256, 512, 1024};
static constexpr size_t kRealisticSizeCount = 7;
static constexpr size_t kRealisticOps = 1000;

static void BM_RealisticWorkload_Std(benchmark::State& state)
{
  auto& alloc = MemoryAllocator::GetDefault();

  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    size_t totalBytes = 0;
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizeCount];
      ptrs[i] = {alloc.allocate(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
      totalBytes += sz;
    }
    // Deallocate in pseudo-random order (Fisher-Yates-ish)
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
BENCHMARK(BM_RealisticWorkload_Std)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true)
    ->MinTime(0.1);

static void BM_RealisticWorkload_FreeList(benchmark::State& state)
{
  FreeListAllocator alloc(MemoryAllocator::GetDefault(), 2097152);

  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    size_t totalBytes = 0;
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizeCount];
      ptrs[i] = {alloc.allocate(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
      totalBytes += sz;
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
BENCHMARK(BM_RealisticWorkload_FreeList)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true)
    ->MinTime(0.1);

static void BM_RealisticWorkload_Pool(benchmark::State& state)
{
  PoolAllocator alloc(MemoryAllocator::GetDefault());

  std::vector<std::pair<void*, size_t>> ptrs(kRealisticOps);

  lcgState = 99u;
  for (auto _ : state) {
    size_t totalBytes = 0;
    for (size_t i = 0; i < kRealisticOps; ++i) {
      const size_t sz = kRealisticSizes[lcgNext() % kRealisticSizeCount];
      ptrs[i] = {alloc.allocate(sz), sz};
      benchmark::DoNotOptimize(ptrs[i].first);
      totalBytes += sz;
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
BENCHMARK(BM_RealisticWorkload_Pool)
    ->Repetitions(3)
    ->ReportAggregatesOnly(true)
    ->MinTime(0.1);
