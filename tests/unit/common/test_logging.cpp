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

#include "../../helpers/gtest_utils.hpp"

#include <dart/common/logging.hpp>
// For version macros
#include <dart/config.hpp>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#if DART_HAVE_spdlog
  #include <spdlog/sinks/ostream_sink.h>
  #include <spdlog/spdlog.h>
#endif

#include <gtest/gtest.h>

using namespace dart;

namespace {

std::size_t countOccurrences(
    const std::string& haystack, const std::string& needle)
{
  if (needle.empty()) {
    return 0u;
  }

  std::size_t count = 0u;
  std::string::size_type pos = haystack.find(needle);
  while (pos != std::string::npos) {
    ++count;
    pos = haystack.find(needle, pos + needle.size());
  }
  return count;
}

class WarningCapture
{
public:
  WarningCapture()
  {
#if DART_HAVE_spdlog
    stream = std::make_shared<std::ostringstream>();
    sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream);
    previousLogger = spdlog::default_logger();
    logger = std::make_shared<spdlog::logger>("warn-once-test", sink);
    logger->set_level(spdlog::level::trace);
    logger->flush_on(spdlog::level::trace);
    spdlog::set_default_logger(logger);
#else
    oldCout = std::cout.rdbuf(stream.rdbuf());
    oldCerr = std::cerr.rdbuf(stream.rdbuf());
#endif
  }

  ~WarningCapture()
  {
#if DART_HAVE_spdlog
    if (logger) {
      logger->flush();
    }
    spdlog::set_default_logger(previousLogger);
    if (logger) {
      spdlog::drop(logger->name());
    }
#else
    std::cout.rdbuf(oldCout);
    std::cerr.rdbuf(oldCerr);
#endif
  }

  std::string contents()
  {
#if DART_HAVE_spdlog
    if (logger) {
      logger->flush();
    }
    if (stream) {
      return stream->str();
    }
    return {};
#else
    return stream.str();
#endif
  }

private:
#if DART_HAVE_spdlog
  std::shared_ptr<std::ostringstream> stream;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink;
  std::shared_ptr<spdlog::logger> previousLogger;
  std::shared_ptr<spdlog::logger> logger;
#else
  std::ostringstream stream;
  std::streambuf* oldCout{nullptr};
  std::streambuf* oldCerr{nullptr};
#endif
};

} // namespace

//==============================================================================
TEST(LoggingTest, Basics)
{
  DART_TRACE("trace log");
  DART_DEBUG("debug log");
  DART_INFO("info log");
  DART_WARN("warn log");
  DART_ERROR("error log");
  DART_FATAL("fatal log");
}

//==============================================================================
TEST(LoggingTest, Arguments)
{
  [[maybe_unused]] int val = 10;
  DART_INFO("Log with param '{}' and '{}'", 1, val);
}

//==============================================================================
TEST(LoggingTest, VersionMacros)
{
  DART_INFO("DART_VERSION: {}", DART_VERSION);
  DART_INFO("DART_DESCRIPTION: {}", DART_DESCRIPTION);
  EXPECT_TRUE(DART_VERSION_GE(5, 999, 999));
  EXPECT_TRUE(DART_VERSION_GE(6, 13, 999));
  EXPECT_TRUE(DART_VERSION_GE(6, 14, 1));
}

//==============================================================================
TEST(LoggingTest, WarnOnce)
{
  const std::string sentinel = "warn-once sentinel";
  auto triggerWarningOnce = [&]() {
    DART_WARN_ONCE_IF(true, "{}", sentinel);
  };

  {
    WarningCapture capture;
    for (int i = 0; i < 3; ++i) {
      triggerWarningOnce();
    }

    const std::size_t occurrences
        = countOccurrences(capture.contents(), sentinel);
    if (occurrences == 0u) {
      GTEST_SKIP()
          << "warn-once sentinel already emitted earlier in this process";
    }

    EXPECT_EQ(occurrences, 1u);
  }

  {
    WarningCapture capture;
    triggerWarningOnce();
    EXPECT_EQ(countOccurrences(capture.contents(), sentinel), 0u);
  }
}
