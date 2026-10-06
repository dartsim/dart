// SPDX-License-Identifier: BSD-2-Clause
#ifndef DART_TEST_BENCHMARK_PERFGUARD_HPP_
#define DART_TEST_BENCHMARK_PERFGUARD_HPP_

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(__linux__)
extern "C" void perf_alloc_begin() __attribute__((weak));
extern "C" void perf_alloc_end() __attribute__((weak));
#endif

// Keeps a Callgrind --toggle-collect target as a real, separately named
// function in optimized builds.
#if defined(_MSC_VER)
  #define DART_PERF_NOINLINE __declspec(noinline)
#else
  #define DART_PERF_NOINLINE __attribute__((noinline))
#endif

namespace dart::test {

// Enclose the exact benchmark function, including fixed setup/teardown. The
// harness subtracts short from long runs, just as it does for Callgrind Ir.
struct PerfWindow
{
  // Query outside the timed loop instead of keeping a flag live across it.
  bool enabled() const
  {
    return std::getenv("PERF_MICRO") != nullptr;
  }

  PerfWindow()
  {
#if defined(__linux__)
    if (enabled() && perf_alloc_begin)
      perf_alloc_begin();
#endif
  }

  ~PerfWindow()
  {
#if defined(__linux__)
    if (enabled() && perf_alloc_end)
      perf_alloc_end();
#endif
  }
};

struct PerfChecksum
{
  std::uint64_t hash = 1469598103934665603ULL;
  bool finite = true;

  void add(double value)
  {
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    hash ^= bits + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    finite = finite && std::isfinite(value);
  }

  void report(const char* name) const
  {
    std::printf(
        "PERFGUARD case=%s hash=0x%016llx finite=%s\n",
        name,
        static_cast<unsigned long long>(hash),
        finite ? "true" : "false");
  }
};

} // namespace dart::test

#endif // DART_TEST_BENCHMARK_PERFGUARD_HPP_
