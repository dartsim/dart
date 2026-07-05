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

#ifndef DART_TESTS_INTEGRATION_ALLOCATIONCOUNTING_HPP_
#define DART_TESTS_INTEGRATION_ALLOCATIONCOUNTING_HPP_

#include <dart/common/MemoryAllocator.hpp>

#include <atomic>
#include <string>

#include <cstddef>

namespace dart::test {

struct HeapAllocationSnapshot
{
  std::size_t allocationCount{0u};
  std::size_t allocationBytes{0u};
};

struct RawHeapAllocationSnapshot
{
  std::size_t allocationCount{0u};
  std::size_t allocationBytes{0u};
  bool skipped{false};
  std::string skipReason;
};

struct CountingMemoryAllocatorSnapshot
{
  std::size_t allocationCount{0u};
  std::size_t allocationBytes{0u};
  std::size_t deallocationCount{0u};
  std::size_t deallocationBytes{0u};
};

class ScopedHeapAllocationCounter final
{
public:
  ScopedHeapAllocationCounter();
  ~ScopedHeapAllocationCounter();

  ScopedHeapAllocationCounter(const ScopedHeapAllocationCounter&) = delete;
  ScopedHeapAllocationCounter& operator=(const ScopedHeapAllocationCounter&)
      = delete;

  void stop() noexcept;

  [[nodiscard]] std::size_t allocationCount() const noexcept;
  [[nodiscard]] std::size_t allocationBytes() const noexcept;
  [[nodiscard]] HeapAllocationSnapshot snapshot() const noexcept;
};

class ScopedRawHeapAllocationCounter final
{
public:
  ScopedRawHeapAllocationCounter();
  ~ScopedRawHeapAllocationCounter();

  ScopedRawHeapAllocationCounter(const ScopedRawHeapAllocationCounter&)
      = delete;
  ScopedRawHeapAllocationCounter& operator=(
      const ScopedRawHeapAllocationCounter&)
      = delete;

  void stop() noexcept;

  [[nodiscard]] std::size_t allocationCount() const noexcept;
  [[nodiscard]] std::size_t allocationBytes() const noexcept;
  [[nodiscard]] bool skipped() const noexcept;
  [[nodiscard]] RawHeapAllocationSnapshot snapshot() const;

  [[nodiscard]] static bool isAvailable() noexcept;
  [[nodiscard]] static const char* skipReason() noexcept;
};

class CountingMemoryAllocator final : public common::MemoryAllocator
{
public:
  CountingMemoryAllocator() noexcept = default;
  ~CountingMemoryAllocator() override = default;

  CountingMemoryAllocator(const CountingMemoryAllocator&) = delete;
  CountingMemoryAllocator& operator=(const CountingMemoryAllocator&) = delete;

  [[nodiscard]] const std::string& getType() const override;
  [[nodiscard]] static const std::string& getStaticType();

  [[nodiscard]] void* allocate(size_t bytes) noexcept override;
  void deallocate(void* pointer, size_t bytes) override;

  void reset() noexcept;
  void setCountingEnabled(bool enabled) noexcept;

  [[nodiscard]] CountingMemoryAllocatorSnapshot snapshot() const noexcept;

private:
  std::atomic<bool> mCountingEnabled{false};
  std::atomic<std::size_t> mAllocationCount{0u};
  std::atomic<std::size_t> mAllocationBytes{0u};
  std::atomic<std::size_t> mDeallocationCount{0u};
  std::atomic<std::size_t> mDeallocationBytes{0u};
};

class ScopedCountingMemoryAllocatorCounter final
{
public:
  explicit ScopedCountingMemoryAllocatorCounter(
      CountingMemoryAllocator& allocator);
  ~ScopedCountingMemoryAllocatorCounter();

  ScopedCountingMemoryAllocatorCounter(
      const ScopedCountingMemoryAllocatorCounter&)
      = delete;
  ScopedCountingMemoryAllocatorCounter& operator=(
      const ScopedCountingMemoryAllocatorCounter&)
      = delete;

  void stop() noexcept;

  [[nodiscard]] CountingMemoryAllocatorSnapshot snapshot() const noexcept;

private:
  CountingMemoryAllocator& mAllocator;
};

} // namespace dart::test

#endif // DART_TESTS_INTEGRATION_ALLOCATIONCOUNTING_HPP_
