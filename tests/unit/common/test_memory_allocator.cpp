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

#include <dart/common/callocator.hpp>
#include <dart/common/memory_allocator.hpp>
#include <dart/common/memory_allocator_debugger.hpp>
#include <dart/common/platform.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>
#include <string_view>

#include <cstdint>

using namespace dart::common;

namespace {

class BareMemoryAllocator final : public MemoryAllocator
{
public:
  std::string_view getType() const override
  {
    return "BareMemoryAllocator";
  }

  void* allocate(size_t) noexcept override
  {
    return nullptr;
  }

  void deallocate(void*, size_t) override
  {
    // Do nothing.
  }
};

class AlignmentTrackingAllocator final : public MemoryAllocator
{
public:
  static std::string_view getStaticType()
  {
    return "AlignmentTrackingAllocator";
  }

  std::string_view getType() const override
  {
    return getStaticType();
  }

  void* allocate(size_t bytes) noexcept override
  {
    ++allocationCount;
    return mBacking.allocate(bytes);
  }

  void* allocate(size_t bytes, size_t alignment) noexcept override
  {
    ++allocationCount;
    ++alignedAllocationCount;
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
    ++alignedDeallocationCount;
    lastDeallocationAlignment = alignment;
    mBacking.deallocate(pointer, bytes, alignment);
  }

  size_t allocationCount = 0;
  size_t alignedAllocationCount = 0;
  size_t deallocationCount = 0;
  size_t alignedDeallocationCount = 0;
  size_t lastDeallocationAlignment = 0;

private:
  CAllocator mBacking;
};

struct LeakReleaseCounters
{
  size_t deallocationCount = 0;
  size_t alignedDeallocationCount = 0;
  size_t deallocatedBytes = 0;
  size_t lastAlignedDeallocationBytes = 0;
  size_t lastAlignedDeallocationAlignment = 0;
};

class LeakReleaseAllocator final : public MemoryAllocator
{
public:
  explicit LeakReleaseAllocator(LeakReleaseCounters& counters)
    : mCounters(&counters)
  {
  }

  static std::string_view getStaticType()
  {
    return "LeakReleaseAllocator";
  }

  std::string_view getType() const override
  {
    return getStaticType();
  }

  void* allocate(size_t bytes) noexcept override
  {
    return mBacking.allocate(bytes);
  }

  void* allocate(size_t bytes, size_t alignment) noexcept override
  {
    return mBacking.allocate(bytes, alignment);
  }

  void deallocate(void* pointer, size_t bytes) override
  {
    ++mCounters->deallocationCount;
    mCounters->deallocatedBytes += bytes;
    mBacking.deallocate(pointer, bytes);
  }

  void deallocate(void* pointer, size_t bytes, size_t alignment) override
  {
    ++mCounters->deallocationCount;
    ++mCounters->alignedDeallocationCount;
    mCounters->deallocatedBytes += bytes;
    mCounters->lastAlignedDeallocationBytes = bytes;
    mCounters->lastAlignedDeallocationAlignment = alignment;
    mBacking.deallocate(pointer, bytes, alignment);
  }

private:
  LeakReleaseCounters* mCounters;
  CAllocator mBacking;
};

struct alignas(64) OverAlignedObject
{
  double values[8] = {};
};

template <typename T>
void expectAligned(T* pointer)
{
  ASSERT_NE(pointer, nullptr);
  EXPECT_EQ(reinterpret_cast<std::uintptr_t>(pointer) % alignof(T), 0u);
}

} // namespace

TEST(MemoryAllocatorTest, DefaultAllocatorIsStable)
{
  MemoryAllocator& allocator1 = MemoryAllocator::GetDefault();
  MemoryAllocator& allocator2 = MemoryAllocator::GetDefault();

  EXPECT_EQ(&allocator1, &allocator2);
  EXPECT_NE(dynamic_cast<CAllocator*>(&allocator1), nullptr);
}

#if !DART_OS_WINDOWS
TEST(MemoryAllocatorTest, PrintWritesMessages)
{
  MemoryAllocator& allocator = MemoryAllocator::GetDefault();

  std::ostringstream noIndent;
  allocator.print(noIndent, 0);
  EXPECT_NE(noIndent.str().find("[CAllocator]"), std::string::npos);

  std::ostringstream withIndent;
  allocator.print(withIndent, 2);
  EXPECT_NE(withIndent.str().find("  type: CAllocator"), std::string::npos);

  std::ostringstream viaStream;
  viaStream << allocator;
  EXPECT_NE(viaStream.str().find("[CAllocator]"), std::string::npos);
}
#endif // !DART_OS_WINDOWS

TEST(MemoryAllocatorTest, BasePrintFallbackWritesMessages)
{
  BareMemoryAllocator allocator;

  std::ostringstream noIndent;
  allocator.print(noIndent, 0);
  EXPECT_NE(
      noIndent.str().find("[*::print is not implemented]"), std::string::npos);

  std::ostringstream withIndent;
  allocator.print(withIndent, 2);
  EXPECT_NE(
      withIndent.str().find("  *::print is not implemented:"),
      std::string::npos);

#if !DART_OS_WINDOWS
  std::ostringstream viaStream;
  viaStream << allocator;
  EXPECT_NE(
      viaStream.str().find("[*::print is not implemented]"), std::string::npos);
#endif // !DART_OS_WINDOWS
}

TEST(MemoryAllocatorTest, DefaultAllocatorSupportsOverAlignedObjects)
{
  auto& allocator = MemoryAllocator::GetDefault();

  auto* raw = static_cast<OverAlignedObject*>(allocator.allocate(
      sizeof(OverAlignedObject), alignof(OverAlignedObject)));
  expectAligned(raw);
  allocator.deallocate(
      raw, sizeof(OverAlignedObject), alignof(OverAlignedObject));

  auto* object = allocator.construct<OverAlignedObject>();
  expectAligned(object);
  allocator.destroy(object);
}

TEST(MemoryAllocatorTest, AllocateAsPreservesByteDeallocateCompatibility)
{
  auto& allocator = MemoryAllocator::GetDefault();

  auto* raw = allocator.allocateAs<OverAlignedObject>();
  ASSERT_NE(raw, nullptr);
  allocator.deallocate(raw, sizeof(OverAlignedObject));
}

TEST(MemoryAllocatorTest, AllocateAsRejectsCountOverflow)
{
  BareMemoryAllocator allocator;

  EXPECT_EQ(
      allocator.allocateAs<OverAlignedObject>(
          std::numeric_limits<size_t>::max() / sizeof(OverAlignedObject) + 1),
      nullptr);
}

TEST(MemoryAllocatorTest, BaseAlignedFallbackRejectsUnsupportedOverAlignment)
{
  BareMemoryAllocator allocator;
  MemoryAllocator& base = allocator;

  EXPECT_EQ(base.allocate(sizeof(OverAlignedObject), 64), nullptr);
}

TEST(MemoryAllocatorTest, DebuggerTracksLiveAllocationsAndPeakBytes)
{
  MemoryAllocatorDebugger<CAllocator> allocator;

  EXPECT_TRUE(allocator.isEmpty());
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
  EXPECT_EQ(
      allocator.getType(),
      std::string_view("MemoryAllocatorDebugger<CAllocator>"));

  auto* first = allocator.allocate(8);
  auto* second = allocator.allocate(16);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  EXPECT_FALSE(allocator.isEmpty());
  EXPECT_EQ(allocator.getAllocatedSize(), 24u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 24u);
  EXPECT_EQ(allocator.getAllocationCount(), 2u);
  EXPECT_TRUE(allocator.hasAllocated(first, 8));
  EXPECT_TRUE(allocator.hasAllocated(second, 16));
  EXPECT_FALSE(allocator.hasAllocated(first, 16));

  std::ostringstream withBoth;
  allocator.print(withBoth);
  EXPECT_NE(withBoth.str().find("size_in_bytes: 24"), std::string::npos);
  EXPECT_NE(withBoth.str().find("peak: 24"), std::string::npos);

  allocator.deallocate(first, 8);
  EXPECT_EQ(allocator.getAllocatedSize(), 16u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 24u);
  EXPECT_EQ(allocator.getAllocationCount(), 1u);
  EXPECT_FALSE(allocator.hasAllocated(first, 8));
  EXPECT_TRUE(allocator.hasAllocated(second, 16));

  std::ostringstream afterFirstFree;
  allocator.print(afterFirstFree);
  EXPECT_NE(afterFirstFree.str().find("size_in_bytes: 16"), std::string::npos);
  EXPECT_NE(afterFirstFree.str().find("peak: 24"), std::string::npos);

  allocator.deallocate(second, 16);
  EXPECT_TRUE(allocator.isEmpty());
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 24u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

TEST(MemoryAllocatorTest, DebuggerRejectsForeignAndMismatchedDeallocations)
{
  MemoryAllocatorDebugger<CAllocator> allocator;
  CAllocator rawAllocator;

  auto* tracked = allocator.allocate(32);
  auto* foreign = rawAllocator.allocate(32);
  ASSERT_NE(tracked, nullptr);
  ASSERT_NE(foreign, nullptr);

  allocator.deallocate(foreign, 32);
  EXPECT_FALSE(allocator.hasAllocated(foreign, 32));
  EXPECT_TRUE(allocator.hasAllocated(tracked, 32));
  EXPECT_EQ(allocator.getAllocatedSize(), 32u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 32u);
  EXPECT_EQ(allocator.getAllocationCount(), 1u);

  allocator.deallocate(tracked, 16);
  EXPECT_TRUE(allocator.hasAllocated(tracked, 32));
  EXPECT_EQ(allocator.getAllocatedSize(), 32u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 32u);
  EXPECT_EQ(allocator.getAllocationCount(), 1u);

  allocator.deallocate(tracked, 32);
  rawAllocator.deallocate(foreign, 32);
  EXPECT_TRUE(allocator.isEmpty());
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), 32u);
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

TEST(MemoryAllocatorTest, DebuggerTracksAlignedAllocationOverload)
{
  MemoryAllocatorDebugger<CAllocator> allocator;

  auto* raw = static_cast<OverAlignedObject*>(allocator.allocate(
      sizeof(OverAlignedObject), alignof(OverAlignedObject)));
  expectAligned(raw);

  EXPECT_TRUE(allocator.hasAllocated(raw, sizeof(OverAlignedObject)));
  EXPECT_EQ(allocator.getAllocatedSize(), sizeof(OverAlignedObject));
  EXPECT_EQ(allocator.getPeakAllocatedSize(), sizeof(OverAlignedObject));
  EXPECT_EQ(allocator.getAllocationCount(), 1u);
  allocator.deallocate(
      raw, sizeof(OverAlignedObject), alignof(OverAlignedObject));
  EXPECT_TRUE(allocator.isEmpty());
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), sizeof(OverAlignedObject));
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
}

TEST(MemoryAllocatorTest, DebuggerRejectsMismatchedAlignedDeallocations)
{
  MemoryAllocatorDebugger<AlignmentTrackingAllocator> allocator;

  auto* raw = static_cast<OverAlignedObject*>(allocator.allocate(
      sizeof(OverAlignedObject), alignof(OverAlignedObject)));
  expectAligned(raw);

  allocator.deallocate(
      raw, sizeof(OverAlignedObject), alignof(OverAlignedObject) / 2);
  EXPECT_TRUE(allocator.hasAllocated(raw, sizeof(OverAlignedObject)));
  EXPECT_EQ(allocator.getAllocatedSize(), sizeof(OverAlignedObject));
  EXPECT_EQ(allocator.getAllocationCount(), 1u);
  EXPECT_EQ(allocator.getInternalAllocator().alignedDeallocationCount, 0u);

  allocator.deallocate(raw, sizeof(OverAlignedObject));
  EXPECT_TRUE(allocator.hasAllocated(raw, sizeof(OverAlignedObject)));
  EXPECT_EQ(allocator.getAllocatedSize(), sizeof(OverAlignedObject));
  EXPECT_EQ(allocator.getAllocationCount(), 1u);
  EXPECT_EQ(allocator.getInternalAllocator().deallocationCount, 0u);

  allocator.deallocate(
      raw, sizeof(OverAlignedObject), alignof(OverAlignedObject));
  EXPECT_TRUE(allocator.isEmpty());
  EXPECT_EQ(allocator.getAllocatedSize(), 0u);
  EXPECT_EQ(allocator.getPeakAllocatedSize(), sizeof(OverAlignedObject));
  EXPECT_EQ(allocator.getAllocationCount(), 0u);
  EXPECT_EQ(allocator.getInternalAllocator().alignedDeallocationCount, 1u);
  EXPECT_EQ(
      allocator.getInternalAllocator().lastDeallocationAlignment,
      alignof(OverAlignedObject));
}

TEST(MemoryAllocatorTest, DebuggerDestructorReleasesLeakedAllocations)
{
  LeakReleaseCounters counters;

  {
    MemoryAllocatorDebugger<LeakReleaseAllocator> allocator(counters);

    void* plain = allocator.allocate(24);
    ASSERT_NE(plain, nullptr);

    auto* aligned = static_cast<OverAlignedObject*>(allocator.allocate(
        sizeof(OverAlignedObject), alignof(OverAlignedObject)));
    expectAligned(aligned);

    EXPECT_EQ(counters.deallocationCount, 0u);
    EXPECT_EQ(counters.alignedDeallocationCount, 0u);
    EXPECT_EQ(counters.deallocatedBytes, 0u);
  }

  EXPECT_EQ(counters.deallocationCount, 2u);
  EXPECT_EQ(counters.alignedDeallocationCount, 1u);
  EXPECT_EQ(counters.deallocatedBytes, 24u + sizeof(OverAlignedObject));
  EXPECT_EQ(counters.lastAlignedDeallocationBytes, sizeof(OverAlignedObject));
  EXPECT_EQ(
      counters.lastAlignedDeallocationAlignment, alignof(OverAlignedObject));
}
