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

#include "TestHelpers.hpp"

#include <dart/dynamics/SphereShape.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>
// For version macros
#include <dart/config.hpp>

#include <gtest/gtest.h>

#if defined(__GLIBCXX__) && !defined(_WIN32)
  #include <pthread.h>
#endif

#if DART_HAVE_spdlog
  #include <spdlog/sinks/base_sink.h>
#endif

#include <atomic>
#include <iomanip>
#include <memory>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cstdlib>

using namespace dart;

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
TEST(LoggingTest, RuntimeFormatString)
{
  const std::string format = "Runtime format string {}";
  dart::common::trace(format, 42);
  dart::common::debug(format, 42);
  dart::common::info(format, 42);
  dart::common::warn(format, 42);
  dart::common::error(format, 42);
  dart::common::fatal(format, 42);
}

//==============================================================================
TEST(LoggingTest, VersionMacros)
{
  DART_INFO("DART_VERSION: {}", DART_VERSION);
  DART_INFO("DART_DESCRIPTION: {}", DART_DESCRIPTION);
  EXPECT_TRUE(DART_VERSION_AT_LEAST(5, 999, 999));
  EXPECT_TRUE(DART_VERSION_AT_LEAST(6, 13, 999));
  EXPECT_TRUE(DART_VERSION_AT_LEAST(6, 14, 1));
}

namespace {

std::ostringstream& sharedOut()
{
  static std::ostringstream stream;
  return stream;
}

std::ostringstream& sharedErr()
{
  static std::ostringstream stream;
  return stream;
}

std::ostream& sharedOutStream()
{
  return sharedOut();
}

std::ostream& sharedErrStream()
{
  return sharedErr();
}

// Each thread gets its own stream, like the dartpy bridge.
std::ostringstream& threadErr()
{
  thread_local std::ostringstream stream;
  return stream;
}

std::ostream& threadErrStream()
{
  return threadErr();
}

} // namespace

//==============================================================================
TEST(LoggingTest, ConsoleStreamsRedirect)
{
  dart::common::setConsoleStreams(&sharedOutStream, &sharedErrStream);
  dtmsg << "message\n";
  dtwarn << "warning\n";
  dterr << "error\n";
  dart::common::setConsoleStreams(nullptr, nullptr);
  dtwarn << "back on std::cerr\n";

  const std::string out = sharedOut().str();
  const std::string err = sharedErr().str();
  EXPECT_NE(out.find("Msg"), std::string::npos);
  EXPECT_NE(out.find("message\n"), std::string::npos);
  EXPECT_NE(err.find("Warning [test_Logging.cpp:"), std::string::npos);
  EXPECT_NE(err.find("warning\n"), std::string::npos);
  EXPECT_NE(err.find("Error [test_Logging.cpp:"), std::string::npos);
  EXPECT_EQ(err.find("back on std::cerr"), std::string::npos);
  EXPECT_EQ(&dart::common::consoleOut(), &std::cout);
  EXPECT_EQ(&dart::common::consoleErr(), &std::cerr);
}

//==============================================================================
// Concurrent dtwarn with per-thread streams: widths, fills and bases are
// stream state, so no thread may see another thread's manipulators.
TEST(LoggingTest, ConsoleStreamsPerThread)
{
  constexpr int kThreads = 8;
  constexpr int kLines = 200;
  dart::common::setConsoleStreams(nullptr, &threadErrStream);

  std::vector<std::string> outputs(kThreads);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([t, &outputs] {
      for (int i = 0; i < kLines; ++i) {
        dtwarn << "t" << std::setw(2 + t) << std::setfill('0') << t
               << std::setfill(' ') << " i" << std::hex << i << std::dec
               << std::endl;
      }
      outputs[t] = threadErr().str();
    });
  }
  for (auto& thread : threads)
    thread.join();
  dart::common::setConsoleStreams(nullptr, nullptr);

  for (int t = 0; t < kThreads; ++t) {
    std::istringstream lines(outputs[t]);
    std::string line;
    int count = 0;
    for (; std::getline(lines, line); ++count) {
      std::ostringstream expected;
      expected << "t" << std::setw(2 + t) << std::setfill('0') << t << " i"
               << std::hex << count;
      ASSERT_NE(line.find("Warning [test_Logging.cpp:"), std::string::npos)
          << line;
      ASSERT_EQ(
          line.substr(line.size() - expected.str().size()), expected.str())
          << "thread " << t << " line " << count << ": " << line;
    }
    EXPECT_EQ(count, kLines) << "thread " << t;
  }
}

namespace {

using LogRecord = std::pair<dart::common::LogLevel, std::string>;

void capture(
    dart::common::LogLevel level,
    const char* message,
    std::size_t length,
    void* userData)
{
  static_cast<std::vector<LogRecord>*>(userData)->emplace_back(
      level, std::string(message, length));
}

} // namespace

//==============================================================================
// A callback replaces spdlog or the console for the DART_WARN family, from
// this test binary and from inside libdart alike, until it is reset.
TEST(LoggingTest, LogCallback)
{
  using dart::common::LogLevel;
  std::vector<LogRecord> records;
  dart::common::setLogCallback(&capture, &records);
  DART_INFO("info {}", 1);
  DART_WARN("warn {} {}", 2, "x");
  dart::common::error(std::string("error"));
  dart::common::fatal("fatal");
  dart::dynamics::SphereShape(1.0).setRadius(-1.0); // warns inside libdart
  dart::common::setLogCallback(nullptr);
  DART_WARN("back on the default output");

  ASSERT_EQ(records.size(), 5u);
  EXPECT_EQ(records[0].first, LogLevel::Info);
  EXPECT_EQ(records[1].first, LogLevel::Warn);
  EXPECT_EQ(records[2].first, LogLevel::Error);
  EXPECT_EQ(records[3].first, LogLevel::Fatal);
  EXPECT_EQ(records[4].first, LogLevel::Warn);
#if DART_HAVE_spdlog
  EXPECT_EQ(records[0].second, "info 1");
  EXPECT_EQ(records[1].second, "warn 2 x");
  EXPECT_EQ(
      records[4].second.find("SphereShape::setRadius: Invalid radius '-1'."),
      0u);
#else
  EXPECT_EQ(records[0].second, "info {} [args]: 1");
  EXPECT_EQ(records[1].second, "warn {} {} [args]: 2, x");
  EXPECT_EQ(
      records[4].second.find("SphereShape::setRadius: Invalid radius '{}'."),
      0u);
#endif
  EXPECT_EQ(records[2].second, "error");
  EXPECT_EQ(records[3].second, "fatal");
}

namespace {

// Counts the messages that reach the default output while a test runs. With
// spdlog that is a sink on the default logger; without it, the console stream.
class DefaultOutputCounter
{
public:
  DefaultOutputCounter()
  {
#if DART_HAVE_spdlog
    mSink = std::make_shared<CountingSink>();
    mPrevious = spdlog::default_logger();
    spdlog::set_default_logger(
        std::make_shared<spdlog::logger>("counting", mSink));
#else
    sCount = &mCount;
    dart::common::setConsoleStreams(&countingStream, &countingStream);
#endif
  }

  ~DefaultOutputCounter()
  {
#if DART_HAVE_spdlog
    spdlog::set_default_logger(mPrevious);
#else
    dart::common::setConsoleStreams(nullptr, nullptr);
    sCount = nullptr;
#endif
  }

  int count() const
  {
#if DART_HAVE_spdlog
    return mSink->count.load();
#else
    return mCount.load();
#endif
  }

private:
#if DART_HAVE_spdlog
  struct CountingSink : spdlog::sinks::base_sink<std::mutex>
  {
    std::atomic<int> count{0};

  protected:
    void sink_it_(const spdlog::details::log_msg&) override
    {
      ++count;
    }
    void flush_() override {}
  };
  std::shared_ptr<CountingSink> mSink;
  std::shared_ptr<spdlog::logger> mPrevious;
#else
  // Every line the console fallback writes ends in one std::endl.
  class CountingBuffer : public std::streambuf
  {
  protected:
    int_type overflow(int_type ch) override
    {
      if (ch == '\n')
        ++*sCount;
      return ch;
    }
  };
  static std::ostream& countingStream()
  {
    thread_local CountingBuffer buffer;
    thread_local std::ostream stream(&buffer);
    return stream;
  }
  static inline std::atomic<int>* sCount = nullptr;
  std::atomic<int> mCount{0};
#endif
};

struct Counter
{
  std::atomic<int> calls{0};
  std::atomic<int> wrongUserData{0};
};

Counter sCounterA;
Counter sCounterB;

void countA(dart::common::LogLevel, const char*, std::size_t, void* userData)
{
  ++sCounterA.calls;
  if (userData != &sCounterA)
    ++sCounterA.wrongUserData;
}

void countB(dart::common::LogLevel, const char*, std::size_t, void* userData)
{
  ++sCounterB.calls;
  if (userData != &sCounterB)
    ++sCounterB.wrongUserData;
}

} // namespace

//==============================================================================
// Installing, replacing and removing the callback while other threads log:
// every message reaches one of the callbacks or the default output, and each
// callback only ever sees the user data it was installed with.
TEST(LoggingTest, LogCallbackConcurrentSetAndUnset)
{
  constexpr int kThreads = 4;
  constexpr int kMessages = 5000;
  constexpr int kToggles = 2000;
  for (Counter* counter : {&sCounterA, &sCounterB}) {
    counter->calls = 0;
    counter->wrongUserData = 0;
  }
  DefaultOutputCounter output;

  std::vector<std::thread> loggers;
  for (int t = 0; t < kThreads; ++t) {
    loggers.emplace_back([t] {
      for (int i = 0; i < kMessages; ++i)
        DART_WARN("thread {} message {}", t, i);
    });
  }
  for (int i = 0; i < kToggles; ++i) {
    dart::common::setLogCallback(&countA, &sCounterA);
    std::this_thread::yield();
    dart::common::setLogCallback(nullptr);
    std::this_thread::yield();
    dart::common::setLogCallback(&countB, &sCounterB);
    std::this_thread::yield();
    dart::common::setLogCallback(nullptr);
    std::this_thread::yield();
  }
  for (auto& logger : loggers)
    logger.join();
  dart::common::setLogCallback(nullptr);

  EXPECT_EQ(sCounterA.wrongUserData.load(), 0);
  EXPECT_EQ(sCounterB.wrongUserData.load(), 0);
  EXPECT_EQ(
      sCounterA.calls.load() + sCounterB.calls.load() + output.count(),
      kThreads * kMessages);
  // Scheduling may let the loggers finish before either callback is installed.
}

namespace {

std::vector<LogRecord> sReentrantRecords;

void logFromInside(
    dart::common::LogLevel level,
    const char* message,
    std::size_t length,
    void* /*userData*/)
{
  sReentrantRecords.emplace_back(level, std::string(message, length));
  // Must not re-enter this callback.
  DART_WARN("from inside the callback");
}

void throwFromInside(
    dart::common::LogLevel, const char*, std::size_t, void* userData)
{
  ++static_cast<Counter*>(userData)->calls;
  throw std::runtime_error("callback failed");
}

// An argument whose formatting logs, as a chatty operator<< might.
struct LogsWhileFormatted
{
};

std::ostream& operator<<(std::ostream& os, LogsWhileFormatted)
{
  DART_WARN("inner");
  return os << "formatted";
}

} // namespace

#if DART_HAVE_spdlog
template <>
struct fmt::formatter<LogsWhileFormatted>
{
  constexpr auto parse(fmt::format_parse_context& ctx)
  {
    return ctx.begin();
  }

  template <typename Context>
  auto format(LogsWhileFormatted value, Context& ctx) const
  {
    std::ostringstream os;
    os << value;
    return fmt::format_to(ctx.out(), "{}", os.str());
  }
};
#endif

//==============================================================================
// A message logged from inside the callback goes to the default output rather
// than recursing into the callback.
TEST(LoggingTest, LogCallbackReentry)
{
  sReentrantRecords.clear();
  DefaultOutputCounter output;
  dart::common::setLogCallback(&logFromInside);
  DART_WARN("outer");
  dart::common::setLogCallback(nullptr);

  ASSERT_EQ(sReentrantRecords.size(), 1u);
  EXPECT_EQ(sReentrantRecords[0].second, "outer");
  EXPECT_EQ(output.count(), 1);
}

//==============================================================================
// A throwing callback neither propagates through the logging call nor loses
// the message: it goes to the default output, with a one-time report.
TEST(LoggingTest, LogCallbackThrows)
{
  Counter calls;
  DefaultOutputCounter output;
  dart::common::setLogCallback(&throwFromInside, &calls);
  EXPECT_NO_THROW(DART_WARN("first"));
  EXPECT_NO_THROW(DART_WARN("second"));
  dart::common::setLogCallback(nullptr);

  EXPECT_EQ(calls.calls.load(), 2);
#if DART_HAVE_spdlog
  // The report uses the error console, separately from spdlog's default output.
  EXPECT_EQ(output.count(), 2);
#else
  // The one-time report adds a third output unless an earlier test or
  // repetition in this process already made it.
  EXPECT_GE(output.count(), 2);
  EXPECT_LE(output.count(), 3);
#endif
}

#if DART_HAVE_spdlog
//==============================================================================
TEST(LoggingDeathTest, LogCallbackThrowsWithQuietDefaultOutput)
{
  // A fresh process keeps the one-time report independent of other tests and
  // of --gtest_repeat: the threadsafe style re-executes the binary instead of
  // forking this process, whose report may already have been made.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        Counter calls;
        DefaultOutputCounter output;
        spdlog::default_logger_raw()->set_level(spdlog::level::off);
        sharedErr().str("");
        dart::common::setConsoleStreams(nullptr, &sharedErrStream);
        dart::common::setLogCallback(&throwFromInside, &calls);
        DART_WARN("first");
        DART_WARN("second");
        dart::common::setLogCallback(nullptr);
        dart::common::setConsoleStreams(nullptr, nullptr);
        const std::string report = sharedErr().str();
        std::_Exit(
            calls.calls == 2 && output.count() == 0
                    && report.find("Error [Logging.cpp:") != std::string::npos
                    && report.find("A log callback threw") != std::string::npos
                    && report.find('\n') == report.size() - 1
                ? 0
                : 1);
      },
      ::testing::ExitedWithCode(0),
      "");
}
#endif

#if defined(__GLIBCXX__) && !defined(_WIN32)
//==============================================================================
TEST(LoggingDeathTest, LogCallbackCanExitItsThread)
{
  EXPECT_EXIT(
      {
        dart::common::setLogCallback(
            [](dart::common::LogLevel, const char*, std::size_t, void*) {
              pthread_exit(nullptr);
            });
        std::thread([] { DART_WARN("thread exits from callback"); }).join();
        dart::common::setLogCallback(nullptr);
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0),
      "");
}
#endif

//==============================================================================
// A message logged while another one is being formatted reaches the callback
// first and leaves the outer message intact.
TEST(LoggingTest, LogCallbackReentrantFormatting)
{
  std::vector<LogRecord> records;
  dart::common::setLogCallback(&capture, &records);
  DART_WARN("outer {}", LogsWhileFormatted{});
  dart::common::setLogCallback(nullptr);

  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].second, "inner");
#if DART_HAVE_spdlog
  EXPECT_EQ(records[1].second, "outer formatted");
#else
  EXPECT_EQ(records[1].second, "outer {} [args]: formatted");
#endif
}

#if DART_HAVE_spdlog
//==============================================================================
// spdlog::drop_all() leaves no default logger. The callback still receives
// every message, from this binary and from inside libdart, and without a
// callback the message goes to the console instead of a null logger.
TEST(LoggingTest, LogCallbackWithoutSpdlogDefaultLogger)
{
  std::vector<LogRecord> records;
  const std::shared_ptr<spdlog::logger> previous = spdlog::default_logger();
  spdlog::drop_all();
  ASSERT_EQ(spdlog::default_logger_raw(), nullptr);
  dart::common::setLogCallback(&capture, &records);
  DART_WARN("after drop_all");
  dart::dynamics::SphereShape(1.0).setRadius(-1.0); // warns inside libdart
  dart::common::setLogCallback(nullptr);
  sharedErr().str("");
  dart::common::setConsoleStreams(&sharedOutStream, &sharedErrStream);
  DART_WARN("no logger, no callback");
  dart::common::setConsoleStreams(nullptr, nullptr);
  spdlog::set_default_logger(previous);

  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].second, "after drop_all");
  EXPECT_EQ(
      records[1].second.find("SphereShape::setRadius: Invalid radius '-1'."),
      0u);
  EXPECT_EQ(sharedErr().str(), "no logger, no callback\n");
}
#endif

#if !DART_HAVE_spdlog
//==============================================================================
// A manipulator among the arguments formats its own message only.
TEST(LoggingTest, FallbackFormattingStateDoesNotPersist)
{
  std::vector<LogRecord> records;
  dart::common::setLogCallback(&capture, &records);
  dart::common::warn("hex", std::hex, 255);
  dart::common::warn("dec", 255);
  dart::common::setLogCallback(nullptr);

  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].second, "hex [args]: , ff");
  EXPECT_EQ(records[1].second, "dec [args]: 255");
}

//==============================================================================
// Without spdlog, the DART_WARN family prints through the same hook.
TEST(LoggingTest, FallbackUsesConsoleStreams)
{
  sharedOut().str("");
  sharedErr().str("");
  dart::common::setConsoleStreams(&sharedOutStream, &sharedErrStream);
  dart::common::info("info {}", 1);
  dart::common::warn("warn {} {}", 2, "x");
  dart::common::error("error");
  dart::common::setConsoleStreams(nullptr, nullptr);

  EXPECT_NE(sharedOut().str().find("[info]"), std::string::npos);
  EXPECT_NE(sharedOut().str().find("info {} [args]: 1\n"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("[warn]"), std::string::npos);
  EXPECT_NE(
      sharedErr().str().find("warn {} {} [args]: 2, x\n"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("[error]"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("error\n"), std::string::npos);
}
#endif
