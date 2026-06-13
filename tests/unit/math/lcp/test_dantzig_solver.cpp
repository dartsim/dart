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

#include <dart/math/lcp/lcp_validation.hpp>
#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <dart/common/memory_allocator.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <limits>
#include <new>

#include <cstdlib>

#if defined(_WIN32)
  #include <malloc.h>
#endif

using namespace dart::math;

namespace {

std::atomic<bool> g_heapAllocationTrackingEnabled{false};
std::atomic<std::size_t> g_heapAllocationCount{0};
std::atomic<std::size_t> g_heapAllocationBytes{0};

void recordHeapAllocation(std::size_t bytes) noexcept
{
  if (g_heapAllocationTrackingEnabled.load(std::memory_order_relaxed)) {
    g_heapAllocationCount.fetch_add(1, std::memory_order_relaxed);
    g_heapAllocationBytes.fetch_add(bytes, std::memory_order_relaxed);
  }
}

[[nodiscard]] void* allocateRaw(std::size_t bytes) noexcept
{
  return std::malloc(bytes == 0 ? 1 : bytes);
}

[[nodiscard]] void* allocateAlignedRaw(
    std::size_t bytes, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    return allocateRaw(bytes);
  }

#if defined(_WIN32)
  return _aligned_malloc(bytes == 0 ? 1 : bytes, alignment);
#else
  const auto requested = bytes == 0 ? 1 : bytes;
  if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
    return nullptr;
  }
  if (alignment < alignof(void*)) {
    alignment = alignof(void*);
  }
  void* pointer = nullptr;
  return posix_memalign(&pointer, alignment, requested) == 0 ? pointer
                                                             : nullptr;
#endif
}

void deallocateAlignedRaw(void* pointer, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    std::free(pointer);
    return;
  }

#if defined(_WIN32)
  _aligned_free(pointer);
#else
  std::free(pointer);
#endif
}

void ExpectValidSolution(
    const LcpProblem& problem, const Eigen::VectorXd& x, double tol)
{
  const Eigen::VectorXd w = problem.A * x - problem.b;
  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string message;
  ASSERT_TRUE(
      detail::computeEffectiveBounds(
          problem.lo, problem.hi, problem.findex, x, loEff, hiEff, &message))
      << message;
  EXPECT_TRUE(detail::validateSolution(x, w, loEff, hiEff, tol, &message))
      << message;
}

class ScopedHeapAllocationCounter final
{
public:
  ScopedHeapAllocationCounter()
  {
    g_heapAllocationCount.store(0, std::memory_order_relaxed);
    g_heapAllocationBytes.store(0, std::memory_order_relaxed);
    g_heapAllocationTrackingEnabled.store(true, std::memory_order_relaxed);
  }

  ~ScopedHeapAllocationCounter()
  {
    g_heapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
  }

  ScopedHeapAllocationCounter(const ScopedHeapAllocationCounter&) = delete;
  ScopedHeapAllocationCounter& operator=(const ScopedHeapAllocationCounter&)
      = delete;

  [[nodiscard]] std::size_t allocations() const noexcept
  {
    return g_heapAllocationCount.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t bytes() const noexcept
  {
    return g_heapAllocationBytes.load(std::memory_order_relaxed);
  }
};

class CountingMemoryAllocator final : public dart::common::MemoryAllocator
{
public:
  [[nodiscard]] std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  [[nodiscard]] void* allocate(std::size_t bytes) noexcept override
  {
    if (bytes == 0) {
      return nullptr;
    }

    ++allocationCount;
    return allocateRaw(bytes);
  }

  [[nodiscard]] void* allocate(
      std::size_t bytes, std::size_t alignment) noexcept override
  {
    if (bytes == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
      return nullptr;
    }

    ++allocationCount;
    ++alignedAllocationCount;
    return allocateAlignedRaw(bytes, alignment);
  }

  void deallocate(void* pointer, std::size_t /*bytes*/) override
  {
    ++deallocationCount;
    std::free(pointer);
  }

  void deallocate(
      void* pointer, std::size_t /*bytes*/, std::size_t alignment) override
  {
    ++deallocationCount;
    ++alignedDeallocationCount;
    deallocateAlignedRaw(pointer, alignment);
  }

  std::size_t allocationCount = 0;
  std::size_t alignedAllocationCount = 0;
  std::size_t deallocationCount = 0;
  std::size_t alignedDeallocationCount = 0;
};

} // namespace

void* operator new(std::size_t bytes)
{
  recordHeapAllocation(bytes);
  if (auto* ptr = allocateRaw(bytes)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes)
{
  recordHeapAllocation(bytes);
  if (auto* ptr = allocateRaw(bytes)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateRaw(bytes);
}

void* operator new[](std::size_t bytes, const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateRaw(bytes);
}

void* operator new(std::size_t bytes, std::align_val_t alignment)
{
  recordHeapAllocation(bytes);
  if (auto* ptr
      = allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment))) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new[](std::size_t bytes, std::align_val_t alignment)
{
  recordHeapAllocation(bytes);
  if (auto* ptr
      = allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment))) {
    return ptr;
  }
  throw std::bad_alloc();
}

void* operator new(
    std::size_t bytes,
    std::align_val_t alignment,
    const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment));
}

void* operator new[](
    std::size_t bytes,
    std::align_val_t alignment,
    const std::nothrow_t&) noexcept
{
  recordHeapAllocation(bytes);
  return allocateAlignedRaw(bytes, static_cast<std::size_t>(alignment));
}

void operator delete(void* pointer) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, std::size_t) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer, std::size_t) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, const std::nothrow_t&) noexcept
{
  std::free(pointer);
}

void operator delete[](void* pointer, const std::nothrow_t&) noexcept
{
  std::free(pointer);
}

void operator delete(void* pointer, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](void* pointer, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete(
    void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](
    void* pointer, std::size_t, std::align_val_t alignment) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete(
    void* pointer, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

void operator delete[](
    void* pointer, std::align_val_t alignment, const std::nothrow_t&) noexcept
{
  deallocateAlignedRaw(pointer, static_cast<std::size_t>(alignment));
}

//==============================================================================
TEST(DantzigSolver, SolvesStandardPositiveDefiniteLcp)
{
  Eigen::Matrix2d A;
  A << 4.0, 1.0, 1.0, 3.0;

  const Eigen::Vector2d target(0.5, 0.25);
  const Eigen::Vector2d b = A * target;

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(
      A,
      b,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector2i::Constant(-1));
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_NEAR(x[0], target[0], 1e-8);
  EXPECT_NEAR(x[1], target[1], 1e-8);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, SolvesBoxedLcpWithActiveUpperBound)
{
  Eigen::Matrix2d A;
  A << 2.0, 0.5, 0.5, 1.5;

  Eigen::Vector2d lo;
  lo << -1.0, 0.0;
  Eigen::Vector2d hi;
  hi << 1.0, 0.2;

  Eigen::Vector2d target;
  target << 0.1, 0.2;
  Eigen::Vector2d w;
  w << 0.0, -0.3;
  const Eigen::Vector2d b = A * target - w;

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(A, b, lo, hi, Eigen::Vector2i::Constant(-1));
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, SolvesBoxedLcpWithFrictionIndex)
{
  Eigen::Matrix3d A;
  A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;

  const Eigen::Vector3d target(1.0, 0.2, -0.1);
  const Eigen::Vector3d b = A * target;

  Eigen::Vector3d lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d hi;
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::Vector3i findex;
  findex << -1, 0, 0;

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpProblem problem(A, b, lo, hi, findex);
  const auto result = solver.solve(problem, x, options);

  EXPECT_TRUE(result.succeeded());
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
  EXPECT_NEAR(x[2], target[2], 1e-6);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, ReusedScratchAvoidsHeapAllocationForSameShapeSolve)
{
  Eigen::Matrix3d A;
  A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;

  const Eigen::Vector3d target(1.0, 0.2, -0.1);
  const Eigen::Vector3d b = A * target;

  Eigen::Vector3d lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d hi;
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::Vector3i findex;
  findex << -1, 0, 0;

  DantzigSolver solver;
  DantzigSolver::Scratch scratch;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  LcpProblem problem(A, b, lo, hi, findex);

  ASSERT_TRUE(solver.solve(problem, x, scratch, options).succeeded());
  x.setZero();
  ASSERT_TRUE(solver.solve(problem, x, scratch, options).succeeded());
  x.setZero();

  std::size_t allocations = 0;
  std::size_t bytes = 0;
  LcpResult result;
  {
    ScopedHeapAllocationCounter heapCounter;
    result = solver.solve(problem, x, scratch, options);
    allocations = heapCounter.allocations();
    bytes = heapCounter.bytes();
  }

  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(allocations, 0u) << "allocated " << bytes << " bytes";
  EXPECT_NEAR(x[0], target[0], 1e-6);
  EXPECT_NEAR(x[1], target[1], 1e-6);
  EXPECT_NEAR(x[2], target[2], 1e-6);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, ScratchUsesProvidedAllocatorForDantzigWorkBuffers)
{
  Eigen::Matrix3d A;
  A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;

  const Eigen::Vector3d target(1.0, 0.2, -0.1);
  const Eigen::Vector3d b = A * target;

  Eigen::Vector3d lo = Eigen::Vector3d::Zero();
  Eigen::Vector3d hi;
  hi << std::numeric_limits<double>::infinity(), 0.5, 0.5;
  Eigen::Vector3i findex;
  findex << -1, 0, 0;

  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;
  LcpProblem problem(A, b, lo, hi, findex);

  CountingMemoryAllocator allocator;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(3);
  {
    DantzigSolver::Scratch scratch(allocator);

    ASSERT_TRUE(solver.solve(problem, x, scratch, options).succeeded());
    EXPECT_GT(allocator.allocationCount, 0u)
        << "allocator-aware Dantzig scratch should allocate work buffers "
           "through the provided allocator";
    EXPECT_GT(allocator.alignedAllocationCount, 0u);

    const auto allocationsAfterFirstSolve = allocator.allocationCount;
    const auto alignedAllocationsAfterFirstSolve
        = allocator.alignedAllocationCount;
    x.setZero();
    ASSERT_TRUE(solver.solve(problem, x, scratch, options).succeeded());

    EXPECT_EQ(allocator.allocationCount, allocationsAfterFirstSolve)
        << "same-shape Dantzig solves should reuse provided-allocator scratch";
    EXPECT_EQ(
        allocator.alignedAllocationCount, alignedAllocationsAfterFirstSolve);
  }

  EXPECT_EQ(allocator.deallocationCount, allocator.allocationCount);
  EXPECT_EQ(
      allocator.alignedDeallocationCount, allocator.alignedAllocationCount);
  ExpectValidSolution(problem, x, 1e-6);
}

//==============================================================================
TEST(DantzigSolver, ReportsInvalidProblemForOutOfRangeFindex)
{
  DantzigSolver solver;
  LcpOptions options = solver.getDefaultOptions();
  options.warmStart = false;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  LcpProblem problem(
      Eigen::MatrixXd::Identity(2, 2),
      Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Zero(2),
      Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity()),
      (Eigen::VectorXi(2) << 2, -1).finished());

  const auto result = solver.solve(problem, x, options);
  EXPECT_EQ(result.status, LcpSolverStatus::InvalidProblem);
}
