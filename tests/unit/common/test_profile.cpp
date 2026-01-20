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

#include <dart/common/profile.hpp>

#include <gtest/gtest.h>

#if DART_BUILD_PROFILE && DART_PROFILE_ENABLE_TEXT
  #include <dart/common/detail/profiler.hpp>

  #include <chrono>
  #include <sstream>
  #include <string>
  #include <thread>

using dart::common::profile::Profiler;
using dart::common::profile::ProfileScope;

class ProfileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    Profiler::instance().reset();
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

TEST_F(ProfileTest, CapturesMultipleThreads)
{
  {
    ProfileScope mainScope("main-work", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::thread worker([] {
    ProfileScope workerScope("worker-work", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  });
  worker.join();

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
  GTEST_SKIP() << "Text profiling backend disabled at build time.";
}

#endif
