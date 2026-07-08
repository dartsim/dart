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

#include <dart/common/Profile.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <string_view>

#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  #include <dart/common/detail/profiler.hpp>

  #include <chrono>
  #include <thread>

using dart::common::profile::Profiler;
using dart::common::profile::ProfileScope;

namespace dart::common::profile {

class ProfilerTestAccess
{
public:
  static void recordZeroDurationScope(std::string_view label)
  {
    auto& profiler = Profiler::instance();
    profiler.recordScopeForTesting(label, __FILE__, __LINE__, 0);
  }
};

} // namespace dart::common::profile

namespace {

std::size_t countOccurrences(const std::string& text, const std::string& token)
{
  std::size_t count = 0;
  std::size_t pos = 0;
  while ((pos = text.find(token, pos)) != std::string::npos) {
    ++count;
    pos += token.size();
  }
  return count;
}

} // namespace

  #define DART_TEST_PROFILE_IF_ELSE(profile_statement, marker_value)           \
    if (true)                                                                  \
      profile_statement;                                                       \
    else                                                                       \
      marker = marker_value

TEST(ProfileMacro, ScopedMacrosAreSingleStatements)
{
  int marker = 0;

  DART_TEST_PROFILE_IF_ELSE(DART_PROFILE_SCOPED, 1);
  DART_TEST_PROFILE_IF_ELSE(
      DART_PROFILE_SCOPED_N("single-statement named scope"), 2);
  DART_TEST_PROFILE_IF_ELSE(
      DART_PROFILE_COUNTER_N("single-statement counter", 1), 3);
  DART_TEST_PROFILE_IF_ELSE(
      DART_PROFILE_SCOPED_IF_N(true, "single-statement conditional scope"), 4);
  DART_TEST_PROFILE_IF_ELSE(
      DART_PROFILE_COUNTER_IF_N(
          true, "single-statement conditional counter", 1),
      5);

  EXPECT_EQ(marker, 0);
}

  #undef DART_TEST_PROFILE_IF_ELSE

class ProfileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    Profiler::instance().reset();
    dart::common::profile::setProfileRecordingEnabled(false);
  }

  void TearDown() override
  {
    dart::common::profile::setProfileRecordingEnabled(false);
  }
};

TEST_F(ProfileTest, CapturesHierarchyAndHotspots)
{
  {
    ProfileScope root("root", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    {
      ProfileScope child("child", __FILE__, __LINE__);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
  }

  std::stringstream ss;
  Profiler::instance().printSummary(ss);
  const auto summary = ss.str();

  EXPECT_NE(summary.find("root"), std::string::npos);
  EXPECT_NE(summary.find("root > child"), std::string::npos);
  EXPECT_NE(summary.find("Hotspots"), std::string::npos);
  EXPECT_NE(summary.find("[HOT]"), std::string::npos);

  // Print once for manual inspection when running the test standalone.
  std::cout << summary;
}

TEST_F(ProfileTest, DeduplicatesRepeatedScopeAtSameTreePosition)
{
  for (int i = 0; i < 2; ++i) {
    ProfileScope scope("dedup-scope", __FILE__, __LINE__);
  }

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  const auto threadStart = summary.find("- thread ");
  ASSERT_NE(threadStart, std::string::npos);
  const auto tree = summary.substr(threadStart);

  EXPECT_EQ(countOccurrences(tree, "dedup-scope"), 1u);
  EXPECT_NE(tree.find("calls        2"), std::string::npos);
}

TEST_F(ProfileTest, ResetReclaimsNodeStorageForNewScopes)
{
  constexpr auto numScopes = 4096;
  for (auto i = 0; i < numScopes; ++i) {
    const auto label = "prefill-scope-" + std::to_string(i);
    ProfileScope scope(label, __FILE__, 10000 + i);
  }

  Profiler::instance().reset();
  {
    ProfileScope scope("after-reset-scope", __FILE__, __LINE__);
  }

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_NE(summary.find("after-reset-scope"), std::string::npos);
  EXPECT_EQ(summary.find("prefill-scope-"), std::string::npos);
}

TEST_F(ProfileTest, ZeroDurationScopesRemainVisibleInSummary)
{
  dart::common::profile::ProfilerTestAccess::recordZeroDurationScope(
      "zero-duration-scope");

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_NE(summary.find("zero-duration-scope"), std::string::npos);
  EXPECT_NE(summary.find("calls        1"), std::string::npos);
  EXPECT_EQ(summary.find("no scoped regions were recorded"), std::string::npos);
}

TEST_F(ProfileTest, KeepsDynamicScopeNamesAlive)
{
  std::string label = "dynamic-scope";
  std::string file = "dynamic-file.cpp";
  {
    ProfileScope scope(label, file, 123);
  }

  std::fill(label.begin(), label.end(), 'x');
  std::fill(file.begin(), file.end(), 'y');

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_NE(summary.find("dynamic-scope"), std::string::npos);
  EXPECT_NE(summary.find("dynamic-file.cpp:123"), std::string::npos);
}

TEST_F(ProfileTest, TextSummaryMacroReturnsString)
{
  {
    DART_PROFILE_SCOPED_N("macro-summary");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_NE(summary.find("macro-summary"), std::string::npos);
  EXPECT_NE(summary.find("DART profiler (text backend)"), std::string::npos);
}

TEST_F(ProfileTest, CounterSamplesAreReported)
{
  for (const auto value : {3u, 5u}) {
    DART_PROFILE_COUNTER_N("counter-sample", value);
  }

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_NE(summary.find("Counters:"), std::string::npos);
  EXPECT_NE(summary.find("counter-sample"), std::string::npos);
  EXPECT_NE(summary.find("samples        2"), std::string::npos);
  EXPECT_NE(summary.find("sum        8"), std::string::npos);
  EXPECT_NE(summary.find("mean     4.00"), std::string::npos);
  EXPECT_NE(summary.find("min        3"), std::string::npos);
  EXPECT_NE(summary.find("max        5"), std::string::npos);
  EXPECT_NE(summary.find("last        5"), std::string::npos);
}

TEST_F(ProfileTest, ConditionalMacrosRecordOnlyWhenEnabled)
{
  {
    DART_PROFILE_SCOPED_IF_N(false, "conditional-off-scope");
  }
  DART_PROFILE_COUNTER_IF_N(false, "conditional-off-counter", 7);

  {
    DART_PROFILE_SCOPED_IF_N(true, "conditional-on-scope");
  }
  DART_PROFILE_COUNTER_IF_N(true, "conditional-on-counter", 13);

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_EQ(summary.find("conditional-off-scope"), std::string::npos);
  EXPECT_EQ(summary.find("conditional-off-counter"), std::string::npos);
  EXPECT_NE(summary.find("conditional-on-scope"), std::string::npos);
  EXPECT_NE(summary.find("conditional-on-counter"), std::string::npos);
}

TEST_F(ProfileTest, RecordingFlagReturnsPreviousState)
{
  EXPECT_FALSE(dart::common::profile::isProfileRecordingEnabled());
  EXPECT_FALSE(dart::common::profile::setProfileRecordingEnabled(true));
  EXPECT_TRUE(dart::common::profile::isProfileRecordingEnabled());
  EXPECT_TRUE(dart::common::profile::setProfileRecordingEnabled(false));
  EXPECT_FALSE(dart::common::profile::isProfileRecordingEnabled());
}

TEST_F(ProfileTest, ResetClearsCounterSamples)
{
  DART_PROFILE_COUNTER_N("counter-before-reset", 7);
  Profiler::instance().reset();
  DART_PROFILE_COUNTER_N("counter-after-reset", 11);

  const auto summary = DART_PROFILE_TEXT_SUMMARY();
  EXPECT_EQ(summary.find("counter-before-reset"), std::string::npos);
  EXPECT_NE(summary.find("counter-after-reset"), std::string::npos);
}

TEST_F(ProfileTest, CommonHelperApiReturnsTextSummary)
{
  EXPECT_TRUE(dart::common::profile::isProfilingEnabled());
  EXPECT_TRUE(dart::common::profile::isTextProfilingEnabled());
  EXPECT_FALSE(dart::common::profile::isProfileRecordingEnabled());

  dart::common::profile::resetProfile();
  {
    DART_PROFILE_SCOPED_N("helper-api-summary");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  dart::common::profile::markProfileFrame();

  std::stringstream ss;
  dart::common::profile::printProfileSummary(ss);
  const auto printed = ss.str();
  const auto summary = dart::common::profile::getProfileSummaryText();

  EXPECT_NE(printed.find("helper-api-summary"), std::string::npos);
  EXPECT_NE(summary.find("helper-api-summary"), std::string::npos);
  EXPECT_NE(summary.find("DART profiler (text backend)"), std::string::npos);
}

TEST_F(ProfileTest, CapturesMultipleThreads)
{
  {
    ProfileScope mainScope("main-work", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  {
    std::thread worker([] {
      ProfileScope workerScope("worker-work", __FILE__, __LINE__);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    });
    worker.join();
  }

  std::stringstream ss;
  Profiler::instance().printSummary(ss);
  const auto summary = ss.str();

  EXPECT_NE(summary.find("Threads:"), std::string::npos);
  EXPECT_NE(summary.find("main-work"), std::string::npos);
  EXPECT_NE(summary.find("worker-work"), std::string::npos);
}

#else

TEST(ProfileBackendDisabled, TextProfilerUnavailable)
{
  EXPECT_EQ(
      dart::common::profile::isProfilingEnabled(),
      static_cast<bool>(DART_BUILD_PROFILE));
  EXPECT_FALSE(dart::common::profile::isTextProfilingEnabled());
  dart::common::profile::resetProfile();
  dart::common::profile::markProfileFrame();

  std::stringstream ss;
  dart::common::profile::printProfileSummary(ss);
  EXPECT_TRUE(ss.str().empty());
  EXPECT_TRUE(dart::common::profile::getProfileSummaryText().empty());
  EXPECT_TRUE(DART_PROFILE_TEXT_SUMMARY().empty());
  #if DART_BUILD_PROFILE && DART_PROFILE_HAS_TRACY
  EXPECT_FALSE(dart::common::profile::isProfileRecordingEnabled());
  EXPECT_FALSE(dart::common::profile::setProfileRecordingEnabled(true));
  EXPECT_TRUE(dart::common::profile::isProfileRecordingEnabled());
  EXPECT_TRUE(dart::common::profile::setProfileRecordingEnabled(false));
  EXPECT_FALSE(dart::common::profile::isProfileRecordingEnabled());
  #else
  EXPECT_FALSE(dart::common::profile::isProfileRecordingEnabled());
  EXPECT_FALSE(dart::common::profile::setProfileRecordingEnabled(true));
  #endif
  GTEST_SKIP() << "Text profiling backend disabled at build time.";
}

#endif
