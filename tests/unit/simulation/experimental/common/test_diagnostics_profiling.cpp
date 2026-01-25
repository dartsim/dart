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

#include <dart/simulation/experimental/common/diagnostics.hpp>
#include <dart/simulation/experimental/common/profiling.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace dart::simulation::experimental::common;

TEST(Diagnostics, GetCompilerInfo_ReturnsNonEmpty)
{
  std::string info = getCompilerInfo();
  EXPECT_FALSE(info.empty());
  EXPECT_TRUE(
      info.find("GCC") != std::string::npos
      || info.find("Clang") != std::string::npos
      || info.find("MSVC") != std::string::npos
      || info.find("Unknown") != std::string::npos);
}

TEST(Diagnostics, GetCxxABI_ReturnsNonEmpty)
{
  std::string abi = getCxxABI();
  EXPECT_FALSE(abi.empty());
  EXPECT_TRUE(
      abi.find("libstdc++") != std::string::npos
      || abi.find("libc++") != std::string::npos
      || abi.find("MSVC") != std::string::npos
      || abi.find("unknown") != std::string::npos);
}

TEST(Diagnostics, GetCxxStandard_ReturnsValidValue)
{
  long standard = getCxxStandard();
  EXPECT_GE(standard, 201703L);
}

TEST(Diagnostics, GetLibraryPath_ReturnsValidPath)
{
  void* funcPtr = reinterpret_cast<void*>(&getCompilerInfo);
  std::string path = getLibraryPath(funcPtr);
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path, "unsupported platform");
}

TEST(Diagnostics, GetLibraryPath_WithNullptr)
{
  std::string path = getLibraryPath(nullptr);
  SUCCEED();
}

TEST(Diagnostics, PrintRuntimeInfo_DoesNotCrash)
{
  EXPECT_NO_THROW(printRuntimeInfo());
}

TEST(Diagnostics, PrintErrorHelp_DoesNotCrash)
{
  EXPECT_NO_THROW(printErrorHelp("TestSection", "TestError"));
}

TEST(Diagnostics, PrintErrorHelp_WithEmptyStrings)
{
  EXPECT_NO_THROW(printErrorHelp("", ""));
}

TEST(Diagnostics, PrintErrorHelp_WithLongStrings)
{
  std::string longSection(100, 'A');
  std::string longError(100, 'B');
  EXPECT_NO_THROW(printErrorHelp(longSection, longError));
}

TEST(Profiling, Stopwatch_TicTock_MeasuresTime)
{
  Stopwatch sw;
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  double elapsed = sw.tock();

  EXPECT_GE(elapsed, 5.0);
  EXPECT_LT(elapsed, 100.0);
}

TEST(Profiling, Stopwatch_ElapsedMs)
{
  Stopwatch sw;
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  double elapsed = sw.elapsed_ms();

  EXPECT_GE(elapsed, 5.0);
  EXPECT_LT(elapsed, 100.0);
}

TEST(Profiling, Stopwatch_ElapsedSec)
{
  Stopwatch sw;
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  double elapsed = sw.elapsed_sec();

  EXPECT_GE(elapsed, 0.025);
  EXPECT_LT(elapsed, 0.5);
}

TEST(Profiling, Stopwatch_ElapsedUs)
{
  Stopwatch sw;
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  long long elapsed = sw.elapsed_us();

  EXPECT_GE(elapsed, 5000);
  EXPECT_LT(elapsed, 100000);
}

TEST(Profiling, Stopwatch_MultipleTic_ResetsTimer)
{
  Stopwatch sw;
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  sw.tic();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  double elapsed = sw.tock();

  EXPECT_LT(elapsed, 25.0);
}

TEST(Profiling, ProfileStats_Record_AddsEntries)
{
  ProfileStats::reset();

  ProfileStats::record("test_entry", 1000);

  const auto& entries = ProfileStats::entries();
  EXPECT_EQ(entries.size(), 1);
  EXPECT_TRUE(entries.find("test_entry") != entries.end());

  ProfileStats::reset();
}

TEST(Profiling, ProfileStats_Entries_RetrievesRecordedEntries)
{
  ProfileStats::reset();

  ProfileStats::record("entry1", 100);
  ProfileStats::record("entry2", 200);

  const auto& entries = ProfileStats::entries();
  EXPECT_EQ(entries.size(), 2);
  EXPECT_TRUE(entries.find("entry1") != entries.end());
  EXPECT_TRUE(entries.find("entry2") != entries.end());

  ProfileStats::reset();
}

TEST(Profiling, ProfileStats_Reset_ClearsAllEntries)
{
  ProfileStats::reset();

  ProfileStats::record("entry1", 100);
  ProfileStats::record("entry2", 200);
  EXPECT_EQ(ProfileStats::entries().size(), 2);

  ProfileStats::reset();
  EXPECT_EQ(ProfileStats::entries().size(), 0);
}

TEST(Profiling, ProfileStats_Entry_AverageMicroseconds)
{
  ProfileStats::reset();

  ProfileStats::record("avg_test", 100);
  ProfileStats::record("avg_test", 200);
  ProfileStats::record("avg_test", 300);

  const auto& entries = ProfileStats::entries();
  auto it = entries.find("avg_test");
  ASSERT_NE(it, entries.end());

  const auto& entry = it->second;
  EXPECT_EQ(entry.count, 3);
  EXPECT_EQ(entry.totalMicroseconds, 600);
  EXPECT_DOUBLE_EQ(entry.averageMicroseconds(), 200.0);

  ProfileStats::reset();
}

TEST(Profiling, ProfileStats_Entry_MinMaxTracking)
{
  ProfileStats::reset();

  ProfileStats::record("minmax_test", 100);
  ProfileStats::record("minmax_test", 50);
  ProfileStats::record("minmax_test", 300);
  ProfileStats::record("minmax_test", 150);

  const auto& entries = ProfileStats::entries();
  auto it = entries.find("minmax_test");
  ASSERT_NE(it, entries.end());

  const auto& entry = it->second;
  EXPECT_EQ(entry.minMicroseconds, 50);
  EXPECT_EQ(entry.maxMicroseconds, 300);

  ProfileStats::reset();
}

TEST(Profiling, ProfileStats_MultipleRecordings_Accumulate)
{
  ProfileStats::reset();

  for (int i = 0; i < 10; ++i) {
    ProfileStats::record("accumulate_test", 100);
  }

  const auto& entries = ProfileStats::entries();
  auto it = entries.find("accumulate_test");
  ASSERT_NE(it, entries.end());

  const auto& entry = it->second;
  EXPECT_EQ(entry.count, 10);
  EXPECT_EQ(entry.totalMicroseconds, 1000);

  ProfileStats::reset();
}

TEST(Profiling, ProfileStats_PrintSummary_DoesNotCrash)
{
  ProfileStats::reset();

  EXPECT_NO_THROW(ProfileStats::printSummary());

  ProfileStats::record("print_test1", 100);
  ProfileStats::record("print_test2", 200);
  EXPECT_NO_THROW(ProfileStats::printSummary());

  ProfileStats::reset();
}

TEST(Profiling, ProfileStats_Entry_AverageWithZeroCount)
{
  ProfileStats::Entry entry;
  entry.count = 0;
  entry.totalMicroseconds = 0;

  EXPECT_DOUBLE_EQ(entry.averageMicroseconds(), 0.0);
}

TEST(Profiling, ScopedTimer_MeasuresElapsedTime)
{
  ProfileStats::reset();

  {
    ScopedTimer timer("scoped_timer_test");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    long long elapsed = timer.elapsed();
    EXPECT_GE(elapsed, 5000);
  }

  const auto& entries = ProfileStats::entries();
  auto it = entries.find("scoped_timer_test");
  ASSERT_NE(it, entries.end());

  const auto& entry = it->second;
  EXPECT_EQ(entry.count, 1);
  EXPECT_GE(entry.totalMicroseconds, 5000);

  ProfileStats::reset();
}

TEST(Profiling, ScopedTimer_ReportsToProfileStats)
{
  ProfileStats::reset();

  {
    ScopedTimer timer("report_test");
    volatile int x = 0;
    for (int i = 0; i < 1000; ++i) {
      x += i;
    }
    (void)x;
  }

  const auto& entries = ProfileStats::entries();
  EXPECT_TRUE(entries.find("report_test") != entries.end());

  ProfileStats::reset();
}

TEST(Profiling, ScopedTimer_MultipleSameName_Accumulate)
{
  ProfileStats::reset();

  for (int i = 0; i < 5; ++i) {
    ScopedTimer timer("multi_scoped_test");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const auto& entries = ProfileStats::entries();
  auto it = entries.find("multi_scoped_test");
  ASSERT_NE(it, entries.end());

  const auto& entry = it->second;
  EXPECT_EQ(entry.count, 5);

  ProfileStats::reset();
}

TEST(Profiling, ScopedTimer_StringViewName)
{
  ProfileStats::reset();

  std::string_view name = "string_view_test";
  {
    ScopedTimer timer(name);
  }

  const auto& entries = ProfileStats::entries();
  EXPECT_TRUE(entries.find("string_view_test") != entries.end());

  ProfileStats::reset();
}
