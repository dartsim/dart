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

#include "AllocationCounting.hpp"

#if defined(__linux__)
  #if defined(__has_include)
    #if __has_include(<features.h>)
      #include <features.h>
    #endif
  #endif
#endif

#if defined(_WIN32)
  #include <malloc.h>
#endif

#include <atomic>
#include <limits>
#include <new>
#include <ostream>
#include <string>

#include <cerrno>
#include <cstdlib>
#include <cstring>

#if defined(__linux__) && defined(__GLIBC__)
  #define DART_TEST_HAS_ALLOCATION_BACKTRACE 1
  #include <cxxabi.h>
  #include <dlfcn.h>
  #include <execinfo.h>

  #include <algorithm>
  #include <vector>
#endif

namespace {

std::atomic<bool> g_heapAllocationTrackingEnabled{false};
std::atomic<std::size_t> g_heapAllocationCount{0u};
std::atomic<std::size_t> g_heapAllocationBytes{0u};

[[maybe_unused]] std::atomic<bool> g_rawHeapAllocationTrackingEnabled{false};
[[maybe_unused]] std::atomic<std::size_t> g_rawHeapAllocationCount{0u};
[[maybe_unused]] std::atomic<std::size_t> g_rawHeapAllocationBytes{0u};

#if defined(DART_TEST_HAS_ALLOCATION_BACKTRACE)

// Allocation-site aggregation for the operator-new surface. The record path
// must not allocate: fixed static storage, open addressing, spinlock insert.
constexpr int kBacktraceCaptureDepth = 14;
// Skip captureAllocationBacktrace, recordHeapAllocation, and operator new.
constexpr int kBacktraceSkipFrames = 3;
constexpr int kBacktraceStoredDepth
    = kBacktraceCaptureDepth - kBacktraceSkipFrames;
constexpr std::size_t kBacktraceSlots = 1u << 16;

struct BacktraceSlot
{
  std::size_t hash{0u};
  std::size_t count{0u};
  std::size_t bytes{0u};
  int depth{0};
  void* frames[kBacktraceStoredDepth]{};
};

BacktraceSlot g_backtraceSlots[kBacktraceSlots];
std::atomic<bool> g_backtraceSamplingEnabled{false};
std::atomic<std::size_t> g_backtraceDroppedSamples{0u};
std::atomic_flag g_backtraceLock = ATOMIC_FLAG_INIT;
thread_local bool t_inBacktraceCapture = false;

void captureAllocationBacktrace(std::size_t bytes) noexcept
{
  if (t_inBacktraceCapture) {
    return;
  }
  t_inBacktraceCapture = true;

  void* rawFrames[kBacktraceCaptureDepth];
  const int captured = ::backtrace(rawFrames, kBacktraceCaptureDepth);
  const int depth
      = captured > kBacktraceSkipFrames ? captured - kBacktraceSkipFrames : 0;
  if (depth == 0) {
    t_inBacktraceCapture = false;
    return;
  }
  void** frames = rawFrames + kBacktraceSkipFrames;

  // FNV-1a over the stored frames.
  std::size_t hash = 1469598103934665603ull;
  for (int i = 0; i < depth; ++i) {
    hash ^= reinterpret_cast<std::size_t>(frames[i]);
    hash *= 1099511628211ull;
  }
  if (hash == 0u) {
    hash = 1u;
  }

  while (g_backtraceLock.test_and_set(std::memory_order_acquire)) {
  }
  std::size_t index = hash & (kBacktraceSlots - 1u);
  bool stored = false;
  for (std::size_t probe = 0u; probe < kBacktraceSlots; ++probe) {
    BacktraceSlot& slot = g_backtraceSlots[index];
    if (slot.hash == hash && slot.depth == depth) {
      slot.count += 1u;
      slot.bytes += bytes;
      stored = true;
      break;
    }
    if (slot.hash == 0u) {
      slot.hash = hash;
      slot.count = 1u;
      slot.bytes = bytes;
      slot.depth = depth;
      for (int i = 0; i < depth; ++i) {
        slot.frames[i] = frames[i];
      }
      stored = true;
      break;
    }
    index = (index + 1u) & (kBacktraceSlots - 1u);
  }
  g_backtraceLock.clear(std::memory_order_release);

  if (!stored) {
    g_backtraceDroppedSamples.fetch_add(1u, std::memory_order_relaxed);
  }
  t_inBacktraceCapture = false;
}

#endif // DART_TEST_HAS_ALLOCATION_BACKTRACE

void recordHeapAllocation(std::size_t bytes) noexcept
{
  if (g_heapAllocationTrackingEnabled.load(std::memory_order_relaxed)) {
    g_heapAllocationCount.fetch_add(1u, std::memory_order_relaxed);
    g_heapAllocationBytes.fetch_add(bytes, std::memory_order_relaxed);
#if defined(DART_TEST_HAS_ALLOCATION_BACKTRACE)
    if (g_backtraceSamplingEnabled.load(std::memory_order_relaxed)) {
      captureAllocationBacktrace(bytes);
    }
#endif
  }
}

[[nodiscard]] void* allocateRaw(std::size_t bytes) noexcept
{
  return std::malloc(bytes == 0u ? 1u : bytes);
}

[[nodiscard]] void* allocateAlignedRaw(
    std::size_t bytes, std::size_t alignment) noexcept
{
  if (alignment <= __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    return allocateRaw(bytes);
  }

#if defined(_WIN32)
  return _aligned_malloc(bytes == 0u ? 1u : bytes, alignment);
#else
  const auto requested = bytes == 0u ? 1u : bytes;
  if (alignment == 0u || (alignment & (alignment - 1u)) != 0u) {
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

#if defined(__SANITIZE_ADDRESS__)
  #define DART_TEST_ASAN_ACTIVE 1
#elif defined(__has_feature)
  #if __has_feature(address_sanitizer)
    #define DART_TEST_ASAN_ACTIVE 1
  #endif
#endif

#if defined(__linux__) && defined(__GLIBC__)                                   \
    && !defined(DART_TEST_ASAN_ACTIVE) && !defined(DART_CODECOV)
  #define DART_TEST_HAS_RAW_MALLOC_INTERPOSE 1

extern "C" {
void* __libc_malloc(std::size_t);
void* __libc_calloc(std::size_t, std::size_t);
void* __libc_realloc(void*, std::size_t);
void* __libc_memalign(std::size_t, std::size_t);
}

namespace {

void recordRawHeapAllocation(std::size_t bytes) noexcept
{
  if (g_rawHeapAllocationTrackingEnabled.load(std::memory_order_relaxed)) {
    g_rawHeapAllocationCount.fetch_add(1u, std::memory_order_relaxed);
    g_rawHeapAllocationBytes.fetch_add(bytes, std::memory_order_relaxed);
  #if defined(DART_TEST_HAS_ALLOCATION_BACKTRACE)
    if (g_backtraceSamplingEnabled.load(std::memory_order_relaxed)) {
      captureAllocationBacktrace(bytes);
    }
  #endif
  }
}

} // namespace

extern "C" void* malloc(std::size_t bytes)
{
  recordRawHeapAllocation(bytes);
  return __libc_malloc(bytes);
}

extern "C" void* calloc(std::size_t count, std::size_t size)
{
  const std::size_t bytes
      = (size != 0u && count > std::numeric_limits<std::size_t>::max() / size)
            ? std::numeric_limits<std::size_t>::max()
            : count * size;
  recordRawHeapAllocation(bytes);
  return __libc_calloc(count, size);
}

extern "C" void* realloc(void* pointer, std::size_t bytes)
{
  recordRawHeapAllocation(bytes);
  return __libc_realloc(pointer, bytes);
}

extern "C" void* aligned_alloc(std::size_t alignment, std::size_t bytes)
{
  recordRawHeapAllocation(bytes);
  return __libc_memalign(alignment, bytes);
}

extern "C" int posix_memalign(
    void** out, std::size_t alignment, std::size_t bytes)
{
  recordRawHeapAllocation(bytes);
  void* pointer = __libc_memalign(alignment, bytes);
  if (pointer == nullptr) {
    return ENOMEM;
  }
  *out = pointer;
  return 0;
}
#endif

namespace {

[[nodiscard]] const char* rawHeapSkipReason() noexcept
{
#if defined(DART_CODECOV)
  return "skipped: coverage build";
#elif defined(DART_TEST_ASAN_ACTIVE)
  return "skipped: AddressSanitizer build";
#elif !defined(__linux__) || !defined(__GLIBC__)
  return "skipped: raw malloc interposer requires Linux glibc";
#else
  return "";
#endif
}

} // namespace

namespace dart::test {

ScopedHeapAllocationCounter::ScopedHeapAllocationCounter()
{
  g_heapAllocationCount.store(0u, std::memory_order_relaxed);
  g_heapAllocationBytes.store(0u, std::memory_order_relaxed);
  g_heapAllocationTrackingEnabled.store(true, std::memory_order_relaxed);
}

ScopedHeapAllocationCounter::~ScopedHeapAllocationCounter()
{
  stop();
}

void ScopedHeapAllocationCounter::stop() noexcept
{
  g_heapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
}

std::size_t ScopedHeapAllocationCounter::allocationCount() const noexcept
{
  return g_heapAllocationCount.load(std::memory_order_relaxed);
}

std::size_t ScopedHeapAllocationCounter::allocationBytes() const noexcept
{
  return g_heapAllocationBytes.load(std::memory_order_relaxed);
}

HeapAllocationSnapshot ScopedHeapAllocationCounter::snapshot() const noexcept
{
  return {allocationCount(), allocationBytes()};
}

ScopedRawHeapAllocationCounter::ScopedRawHeapAllocationCounter()
{
#if defined(DART_TEST_HAS_RAW_MALLOC_INTERPOSE)
  g_rawHeapAllocationCount.store(0u, std::memory_order_relaxed);
  g_rawHeapAllocationBytes.store(0u, std::memory_order_relaxed);
  g_rawHeapAllocationTrackingEnabled.store(true, std::memory_order_relaxed);
#else
  g_rawHeapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
#endif
}

ScopedRawHeapAllocationCounter::~ScopedRawHeapAllocationCounter()
{
  stop();
}

void ScopedRawHeapAllocationCounter::stop() noexcept
{
  g_rawHeapAllocationTrackingEnabled.store(false, std::memory_order_relaxed);
}

std::size_t ScopedRawHeapAllocationCounter::allocationCount() const noexcept
{
#if defined(DART_TEST_HAS_RAW_MALLOC_INTERPOSE)
  return g_rawHeapAllocationCount.load(std::memory_order_relaxed);
#else
  return 0u;
#endif
}

std::size_t ScopedRawHeapAllocationCounter::allocationBytes() const noexcept
{
#if defined(DART_TEST_HAS_RAW_MALLOC_INTERPOSE)
  return g_rawHeapAllocationBytes.load(std::memory_order_relaxed);
#else
  return 0u;
#endif
}

bool ScopedRawHeapAllocationCounter::skipped() const noexcept
{
  return !isAvailable();
}

RawHeapAllocationSnapshot ScopedRawHeapAllocationCounter::snapshot() const
{
  return {allocationCount(), allocationBytes(), skipped(), skipReason()};
}

bool ScopedRawHeapAllocationCounter::isAvailable() noexcept
{
#if defined(DART_TEST_HAS_RAW_MALLOC_INTERPOSE)
  return true;
#else
  return false;
#endif
}

const char* ScopedRawHeapAllocationCounter::skipReason() noexcept
{
  return rawHeapSkipReason();
}

const std::string& CountingMemoryAllocator::getType() const
{
  return getStaticType();
}

const std::string& CountingMemoryAllocator::getStaticType()
{
  static const std::string type = "CountingMemoryAllocator";
  return type;
}

void* CountingMemoryAllocator::allocate(size_t bytes) noexcept
{
  if (bytes == 0u) {
    return nullptr;
  }

  if (mCountingEnabled.load(std::memory_order_relaxed)) {
    mAllocationCount.fetch_add(1u, std::memory_order_relaxed);
    mAllocationBytes.fetch_add(bytes, std::memory_order_relaxed);
  }

  return std::malloc(bytes);
}

void CountingMemoryAllocator::deallocate(void* pointer, size_t bytes)
{
  if (mCountingEnabled.load(std::memory_order_relaxed) && pointer != nullptr) {
    mDeallocationCount.fetch_add(1u, std::memory_order_relaxed);
    mDeallocationBytes.fetch_add(bytes, std::memory_order_relaxed);
  }

  std::free(pointer);
}

void CountingMemoryAllocator::reset() noexcept
{
  mAllocationCount.store(0u, std::memory_order_relaxed);
  mAllocationBytes.store(0u, std::memory_order_relaxed);
  mDeallocationCount.store(0u, std::memory_order_relaxed);
  mDeallocationBytes.store(0u, std::memory_order_relaxed);
}

void CountingMemoryAllocator::setCountingEnabled(bool enabled) noexcept
{
  mCountingEnabled.store(enabled, std::memory_order_relaxed);
}

CountingMemoryAllocatorSnapshot CountingMemoryAllocator::snapshot()
    const noexcept
{
  return {
      mAllocationCount.load(std::memory_order_relaxed),
      mAllocationBytes.load(std::memory_order_relaxed),
      mDeallocationCount.load(std::memory_order_relaxed),
      mDeallocationBytes.load(std::memory_order_relaxed)};
}

ScopedCountingMemoryAllocatorCounter::ScopedCountingMemoryAllocatorCounter(
    CountingMemoryAllocator& allocator)
  : mAllocator(allocator)
{
  mAllocator.reset();
  mAllocator.setCountingEnabled(true);
}

ScopedCountingMemoryAllocatorCounter::~ScopedCountingMemoryAllocatorCounter()
{
  stop();
}

void ScopedCountingMemoryAllocatorCounter::stop() noexcept
{
  mAllocator.setCountingEnabled(false);
}

CountingMemoryAllocatorSnapshot ScopedCountingMemoryAllocatorCounter::snapshot()
    const noexcept
{
  return mAllocator.snapshot();
}

#if defined(DART_TEST_HAS_ALLOCATION_BACKTRACE)

void setAllocationBacktraceSamplingEnabled(bool enabled)
{
  if (enabled) {
    // Prime the libgcc unwinder outside any counting window so its lazy
    // initialization allocations are not attributed to measured code.
    void* primeFrames[kBacktraceCaptureDepth];
    t_inBacktraceCapture = true;
    ::backtrace(primeFrames, kBacktraceCaptureDepth);
    t_inBacktraceCapture = false;
  }
  g_backtraceSamplingEnabled.store(enabled, std::memory_order_relaxed);
}

void clearAllocationBacktraces()
{
  while (g_backtraceLock.test_and_set(std::memory_order_acquire)) {
  }
  for (std::size_t i = 0u; i < kBacktraceSlots; ++i) {
    g_backtraceSlots[i] = BacktraceSlot{};
  }
  g_backtraceLock.clear(std::memory_order_release);
  g_backtraceDroppedSamples.store(0u, std::memory_order_relaxed);
}

std::size_t dumpAllocationBacktraces(std::ostream& os, std::size_t topN)
{
  struct SiteView
  {
    const BacktraceSlot* slot;
  };
  std::vector<SiteView> sites;
  sites.reserve(1024u);
  for (std::size_t i = 0u; i < kBacktraceSlots; ++i) {
    if (g_backtraceSlots[i].hash != 0u) {
      sites.push_back({&g_backtraceSlots[i]});
    }
  }
  std::sort(sites.begin(), sites.end(), [](const auto& a, const auto& b) {
    return a.slot->count > b.slot->count;
  });

  const std::size_t shown = std::min(topN, sites.size());
  os << "[AllocationBacktraces] distinct_sites=" << sites.size()
     << " dropped_samples="
     << g_backtraceDroppedSamples.load(std::memory_order_relaxed) << "\n";
  for (std::size_t s = 0u; s < shown; ++s) {
    const BacktraceSlot& slot = *sites[s].slot;
    os << "site#" << s << " count=" << slot.count << " bytes=" << slot.bytes
       << "\n";
    const int printDepth = std::min(slot.depth, 8);
    for (int f = 0; f < printDepth; ++f) {
      Dl_info info{};
      os << "  [" << f << "] ";
      if (::dladdr(slot.frames[f], &info) != 0 && info.dli_fname != nullptr) {
        const char* base = std::strrchr(info.dli_fname, '/');
        os << (base != nullptr ? base + 1 : info.dli_fname) << "+0x" << std::hex
           << (reinterpret_cast<std::size_t>(slot.frames[f])
               - reinterpret_cast<std::size_t>(info.dli_fbase))
           << std::dec;
        if (info.dli_sname != nullptr) {
          int status = 0;
          char* demangled
              = abi::__cxa_demangle(info.dli_sname, nullptr, nullptr, &status);
          os << " " << (status == 0 && demangled ? demangled : info.dli_sname);
          std::free(demangled);
        }
      } else {
        os << slot.frames[f];
      }
      os << "\n";
    }
  }
  return sites.size();
}

#else // !DART_TEST_HAS_ALLOCATION_BACKTRACE

void setAllocationBacktraceSamplingEnabled(bool)
{
  // Backtrace sampling is unsupported on this platform.
}

void clearAllocationBacktraces() {}

std::size_t dumpAllocationBacktraces(std::ostream& os, std::size_t)
{
  os << "[AllocationBacktraces] unsupported on this platform\n";
  return 0u;
}

#endif // DART_TEST_HAS_ALLOCATION_BACKTRACE

} // namespace dart::test
