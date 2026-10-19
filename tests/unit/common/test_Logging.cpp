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

#include <dart/config.hpp>

#include <dart/common/Logging.hpp>
#include <dart/common/detail/DeterministicExecutor.h>

#include <gtest/gtest.h>

#include <atomic>
#include <iostream>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#if __cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
  #include <string_view>
#endif

#include <cmath>
#include <cstdlib>

using namespace dart::common;

namespace {

struct UnsupportedLogArgument
{
  int member;
  void function();
};

static_assert(
    !std::is_constructible<detail::LogArgument, void (*)()>::value,
    "Function pointers must not log as bool");
static_assert(
    !std::is_constructible<
        detail::LogArgument,
        decltype(&UnsupportedLogArgument::member)>::value,
    "Member pointers must not log as bool");
static_assert(
    !std::is_constructible<
        detail::LogArgument,
        decltype(&UnsupportedLogArgument::function)>::value,
    "Member function pointers must not log as bool");
static_assert(
    !std::is_constructible<detail::LogArgument, UnsupportedLogArgument>::value,
    "Only scalar, string-like and enum arguments are supported");

// Constructed before main, so its destructor runs after function-local
// objects first initialized by logging tests. Exercise that order in an exit
// subprocess.
struct LogAtShutdown
{
  bool enabled = false;

  ~LogAtShutdown()
  {
    if (enabled)
      warn("logging at static destruction {}", 42);
  }
} logAtShutdown;

class LoggingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    oldLevel = getLogLevel();
    setLogLevel(LogLevel::Trace);
    setLogSink([this](LogLevel level, const std::string& message) {
      levels.push_back(level);
      messages.push_back(message);
    });
  }

  void TearDown() override
  {
    resetLogSink();
    setLogLevel(oldLevel);
  }

  LogLevel oldLevel;
  std::vector<LogLevel> levels;
  std::vector<std::string> messages;
};

} // namespace

//==============================================================================
TEST_F(LoggingTest, DefaultLevelIsAtLeastInfo)
{
  const auto expected = DART_ACTIVE_LOG_LEVEL < DART_LOG_LEVEL_INFO
                            ? LogLevel::Info
                            : static_cast<LogLevel>(DART_ACTIVE_LOG_LEVEL);
  EXPECT_EQ(expected, oldLevel);
  setLogLevel(oldLevel);
  debug("disabled by default");
  EXPECT_TRUE(messages.empty());
  setLogLevel(LogLevel::Debug);
  debug("enabled at runtime");
  EXPECT_EQ((std::vector<std::string>{"enabled at runtime"}), messages);
}

//==============================================================================
TEST_F(LoggingTest, ZeroArgumentCalls)
{
  info("no arguments");
  info("{{escaped}}");
  EXPECT_EQ((std::vector<std::string>{"no arguments", "{escaped}"}), messages);
}

//==============================================================================
TEST_F(LoggingTest, CapturesAllLevelsAndRuntimeFormatStrings)
{
  const std::string format = "runtime {}";
  trace(format, 42);
  debug(format, 42);
  info(format, 42);
  warn(format, 42);
  error(format, 42);
  fatal(format, 42);
  EXPECT_EQ(
      (std::vector<LogLevel>{
          LogLevel::Trace,
          LogLevel::Debug,
          LogLevel::Info,
          LogLevel::Warn,
          LogLevel::Error,
          LogLevel::Fatal}),
      levels);
  EXPECT_EQ(std::vector<std::string>(6, "runtime 42"), messages);
}

//==============================================================================
TEST_F(LoggingTest, MacrosAndVersionMacros)
{
  DART_TRACE("trace log");
  DART_DEBUG("debug log");
  DART_INFO("DART_VERSION: {}", DART_VERSION);
  DART_WARN("DART_DESCRIPTION: {}", DART_DESCRIPTION);
  DART_ERROR("error log");
  DART_FATAL("fatal log");
  EXPECT_TRUE(DART_VERSION_AT_LEAST(5, 999, 999));
  EXPECT_TRUE(DART_VERSION_AT_LEAST(6, 13, 999));
  EXPECT_TRUE(DART_VERSION_AT_LEAST(6, 14, 1));
  // This expectation also checks the configured header value without relying on
  // a target_compile_definitions entry inherited by CMake consumers.
  EXPECT_EQ(
      static_cast<std::size_t>(6 - DART_ACTIVE_LOG_LEVEL), messages.size());
}

//==============================================================================
TEST_F(LoggingTest, RuntimeThresholdFiltersEveryLevel)
{
  for (int threshold = 0; threshold <= 6; ++threshold) {
    const auto level = static_cast<LogLevel>(threshold);
    setLogLevel(level);
    EXPECT_EQ(level, getLogLevel());
    levels.clear();
    trace("trace");
    debug("debug");
    info("info");
    warn("warn");
    error("error");
    fatal("fatal");
    ASSERT_EQ(static_cast<std::size_t>(6 - threshold), levels.size());
    for (const auto captured : levels)
      EXPECT_GE(captured, level);
  }
}

//==============================================================================
TEST_F(LoggingTest, RejectsInvalidMessageLevels)
{
  setLogLevel(static_cast<LogLevel>(-2));
  for (const bool useDefault : {false, true}) {
    if (useDefault)
      resetLogSink();
    testing::internal::CaptureStdout();
    testing::internal::CaptureStderr();
    for (const int level : {-1, 6, 7})
      detail::logMessage(static_cast<LogLevel>(level), "invalid", nullptr, 0);
    EXPECT_EQ("", testing::internal::GetCapturedStdout());
    EXPECT_EQ("", testing::internal::GetCapturedStderr());
    EXPECT_TRUE(messages.empty());
  }
}

//==============================================================================
TEST_F(LoggingTest, ReturnsPreviousSinkForRestoration)
{
  auto previous = setLogSink([](LogLevel, const std::string&) {});
  ASSERT_TRUE(previous);
  info("temporary sink");
  EXPECT_TRUE(messages.empty());
  EXPECT_TRUE(setLogSink(std::move(previous)));
  info("restored sink");
  EXPECT_EQ((std::vector<std::string>{"restored sink"}), messages);
  EXPECT_TRUE(setLogSink({}));
  EXPECT_FALSE(setLogSink({}));
  EXPECT_FALSE(setLogSink([](LogLevel, const std::string&) {}));
}

//==============================================================================
TEST_F(LoggingTest, DisabledLoggingDoesNotConvertArgumentsOrFormat)
{
  struct CountedString
  {
    int& conversions;
    operator std::string() const
    {
      ++conversions;
      throw std::runtime_error("must not format a disabled message");
    }
  };
  int conversions = 0;
  setLogLevel(LogLevel::Off);
  info(
      CountedString{conversions},
      "an argument longer than the small string buffer");
  EXPECT_EQ(0, conversions);
  EXPECT_TRUE(messages.empty());
}

//==============================================================================
TEST_F(LoggingTest, FormatsScalarAndStringArguments)
{
  info(
      "{} {} {} {} {} {} {} {} {} {} {} {} {} {}",
      std::string("text"),
      "c string",
      true,
      false,
      'x',
      static_cast<signed char>(-2),
      static_cast<unsigned char>(3),
      static_cast<short>(-4),
      5u,
      -6L,
      7ul,
      std::numeric_limits<long long>::min(),
      std::numeric_limits<unsigned long long>::max(),
      std::size_t{8});
  ASSERT_EQ(1u, messages.size());
  EXPECT_EQ(
      "text c string true false x -2 3 -4 5 -6 7 -9223372036854775808 "
      "18446744073709551615 8",
      messages[0]);
  info("{} {} {}", 0.1f, 0.1, 0.125L);
  EXPECT_EQ("0.1 0.1 0.125", messages.back());
  info("{:.4e} {:.6e} {:.2f} {:.3g}", 0.125, 0.125, 0.125, 0.125);
  EXPECT_EQ("1.2500e-01 1.250000e-01 0.12 0.125", messages.back());
  info("{{{}}} {{}}", "braces");
  EXPECT_EQ("{braces} {}", messages.back());
  info("{} {}", static_cast<const void*>(nullptr), nullptr);
  EXPECT_EQ("0x0 0x0", messages.back());
  int value = 0;
  info("{}", static_cast<void*>(&value));
  EXPECT_EQ(0u, messages.back().find("0x"));
  info(
      "{} {}",
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ("inf nan", messages.back());
}

//==============================================================================
TEST_F(LoggingTest, FormatTable)
{
  struct Case
  {
    std::string format;
    std::vector<detail::LogArgument> arguments;
    std::string expected;
  };
  const Case cases[]{
      {"plain", {}, "plain"},
      {"{{}}", {}, "{}"},
      {"{{{}}}", {"text"}, "{text}"},
      {"{}", {std::string("string")}, "string"},
      {"{}", {"c string"}, "c string"},
      {"{}", {static_cast<const char*>(nullptr)}, "(null)"},
      {"{}", {true}, "true"},
      {"{}", {false}, "false"},
      {"{}", {'x'}, "x"},
      {"{}", {'\0'}, std::string(1, '\0')},
      {"{}", {static_cast<signed char>(-2)}, "-2"},
      {"{}", {static_cast<unsigned char>(3)}, "3"},
      {"{}", {static_cast<short>(-4)}, "-4"},
      {"{}", {static_cast<unsigned short>(5)}, "5"},
      {"{}", {-6}, "-6"},
      {"{}", {7u}, "7"},
      {"{}", {-8L}, "-8"},
      {"{}", {9ul}, "9"},
      {"{}", {std::numeric_limits<long long>::min()}, "-9223372036854775808"},
      {"{}",
       {std::numeric_limits<unsigned long long>::max()},
       "18446744073709551615"},
      {"{}", {std::size_t{10}}, "10"},
      {"{}", {0.1f}, "0.1"},
      {"{}", {0.1}, "0.1"},
      {"{}", {0.125L}, "0.125"},
      {"{}", {0.0}, "0"},
      {"{}", {-0.0}, "-0"},
      {"{}", {1e-4}, "0.0001"},
      {"{}", {1e-5}, "1e-05"},
      {"{}", {1e5f}, "100000"},
      {"{}", {1234567.0f}, "1234567"},
      {"{}", {9999999.0f}, "9999999"},
      {"{}", {1e7f}, "1e+07"},
      {"{}", {12345678.0f}, "1.2345678e+07"},
      {"{}", {1e11f}, "1e+11"},
      {"{}", {1e15f}, "1e+15"},
      {"{}", {-1e11f}, "-1e+11"},
      {"{}", {0.0001f}, "0.0001"},
      {"{}", {1e-5f}, "1e-05"},
      {"{}", {0.0125f}, "0.0125"},
      {"{}", {12.5}, "12.5"},
      {"{}", {1e15}, "1000000000000000"},
      {"{}", {1e16}, "1e+16"},
      {"{}", {1e17}, "1e+17"},
      {"{}", {1.2345678f}, "1.2345678"},
      {"{}", {1.2345678901234567}, "1.2345678901234567"},
      {"{}", {std::numeric_limits<float>::denorm_min()}, "1e-45"},
      {"{}", {std::numeric_limits<float>::max()}, "3.4028235e+38"},
      {"{}", {std::numeric_limits<double>::denorm_min()}, "5e-324"},
      {"{}", {std::numeric_limits<double>::max()}, "1.7976931348623157e+308"},
      {"{}", {std::numeric_limits<double>::infinity()}, "inf"},
      {"{}", {-std::numeric_limits<double>::infinity()}, "-inf"},
      {"{}", {std::numeric_limits<double>::quiet_NaN()}, "nan"},
      {"{}", {-std::numeric_limits<double>::quiet_NaN()}, "-nan"},
      {"{}", {nullptr}, "0x0"},
      {"{}", {static_cast<const void*>(nullptr)}, "0x0"},
      {"{:.4e}", {0.125f}, "1.2500e-01"},
      {"{:.4e}", {0.125}, "1.2500e-01"},
      {"{:.4e}", {0.125L}, "1.2500e-01"},
      {"{:.2f}", {0.125f}, "0.12"},
      {"{:.2f}", {0.125}, "0.12"},
      {"{:.2f}", {0.125L}, "0.12"},
      {"{:.3g}", {0.125f}, "0.125"},
      {"{:.3g}", {0.125}, "0.125"},
      {"{:.3g}", {0.125L}, "0.125"},
      {"{:.0e}", {1.5}, "2e+00"},
      {"{:.0f}", {1.5}, "2"},
      {"{:.0g}", {1.5f}, "2"},
      {"{:.0g}", {1.5}, "2"},
      {"{:.0g}", {1.5L}, "2"},
      {"{:.140f}", {1.5}, "1.5" + std::string(139, '0')},
      {"{:.1000f}", {1.5f}, "1.5" + std::string(999, '0')},
      {"{:.1000f}", {1.5}, "1.5" + std::string(999, '0')},
      {"{:.1000f}", {1.5L}, "1.5" + std::string(999, '0')},
      {"{:.1001f}", {1.5f}, "[log format error: invalid precision] {:.1001f}"},
      {"{:.1001e}", {1.5}, "[log format error: invalid precision] {:.1001e}"},
      {"{:.1001g}", {1.5L}, "[log format error: invalid precision] {:.1001g}"},
      {"{:.1001f}",
       {std::numeric_limits<double>::infinity()},
       "[log format error: invalid precision] {:.1001f}"},
      {"{}", {}, "[log format error: too few arguments] {}"},
      {"{} {}", {1}, "[log format error: too few arguments] {} {}"},
      {"literal", {1}, "literal"},
      {"{}", {1, 2}, "1"},
      {"{", {}, "[log format error: unmatched '{'] {"},
      {"}", {}, "[log format error: unmatched '}'] }"},
      {"{}{", {1}, "[log format error: unmatched '{'] {}{"},
      {"{0}", {1}, "[log format error: unsupported format field] {0}"},
      {"{:bogus}",
       {1},
       "[log format error: unsupported format field] {:bogus}"},
      {"{:x}", {1}, "[log format error: unsupported format field] {:x}"},
      {"{:.f}", {1.5}, "[log format error: unsupported format field] {:.f}"},
      {"{:.2E}", {1.5}, "[log format error: unsupported format field] {:.2E}"},
      {"{:.+2f}",
       {1.5},
       "[log format error: unsupported format field] {:.+2f}"},
      {"{:.2fX}",
       {1.5},
       "[log format error: unsupported format field] {:.2fX}"},
      {"{:.{}f}",
       {1.5},
       "[log format error: unsupported format field] {:.{}f}"},
      {"{:.2f}",
       {1},
       "[log format error: precision requires floating-point argument] {:.2f}"},
      {"{:.2f}",
       {"text"},
       "[log format error: precision requires floating-point argument] {:.2f}"},
      {"{:.999999999999f}",
       {1.5},
       "[log format error: invalid precision] {:.999999999999f}"},
  };
  for (const auto& test : cases) {
    SCOPED_TRACE(test.format);
    detail::logMessage(
        LogLevel::Info,
        test.format,
        test.arguments.data(),
        test.arguments.size());
    ASSERT_FALSE(messages.empty());
    EXPECT_EQ(test.expected, messages.back());
    messages.clear();
  }
}

//==============================================================================
TEST_F(LoggingTest, FormatsNonFiniteValues)
{
  const auto check = [&](auto value) {
    using T = decltype(value);
    for (const std::string format : {"{}", "{:.2e}", "{:.2f}", "{:.2g}"}) {
      SCOPED_TRACE(format);
      for (const bool negative : {false, true}) {
        const T sign = negative ? T(-1) : T(1);
        info(format, std::copysign(std::numeric_limits<T>::quiet_NaN(), sign));
        EXPECT_EQ(negative ? "-nan" : "nan", messages.back());
        info(format, std::copysign(std::numeric_limits<T>::infinity(), sign));
        EXPECT_EQ(negative ? "-inf" : "inf", messages.back());
      }
    }
  };
  check(0.0f);
  check(0.0);
  check(0.0L);
}

//==============================================================================
TEST_F(LoggingTest, DefaultLongDoublePreservesRoundTrips)
{
  for (const long double value :
       {0.1L,
        std::nextafter(1.0L, 2.0L),
        std::numeric_limits<long double>::min(),
        std::numeric_limits<long double>::max()}) {
    info("{}", value);
    std::istringstream stream(messages.back());
    stream.imbue(std::locale::classic());
    long double parsed = 0;
    ASSERT_TRUE(stream >> parsed);
    EXPECT_EQ(value, parsed);
  }
}

//==============================================================================
TEST_F(LoggingTest, FormattingIgnoresGlobalLocale)
{
  struct CommaDecimal : std::numpunct<char>
  {
    char do_decimal_point() const override
    {
      return ',';
    }
  };
  struct RestoreLocale
  {
    std::locale previous;
    ~RestoreLocale()
    {
      std::locale::global(previous);
    }
  } restore{std::locale()};
  std::locale::global(std::locale(restore.previous, new CommaDecimal));
  info("{} {:.2f} {:.2f}", 1.5, 1.5f, 1.5L);
  EXPECT_EQ("1.5 1.50 1.50", messages.back());
}

//==============================================================================
TEST_F(LoggingTest, MovesStringArgument)
{
  std::string value(256, 'x');
  const auto* data = value.data();
  const detail::LogArgument argument(std::move(value));
  EXPECT_EQ(data, argument.stringValue.data());
  EXPECT_EQ(std::string(256, 'x'), argument.stringValue);
}

//==============================================================================
TEST_F(LoggingTest, FormatsStringLikeArgumentsAndCopies)
{
  char text[] = "mutable";
  info("{}", text);
  EXPECT_EQ("mutable", messages.back());
  char* pointer = text;
  info("{} {}", pointer, static_cast<char*>(nullptr));
  EXPECT_EQ("mutable (null)", messages.back());

  struct StringLike
  {
    explicit operator std::string() const
    {
      return "string-like";
    }
  };
  info("{}", StringLike{});
  EXPECT_EQ("string-like", messages.back());

  const detail::LogArgument original(std::string("copy"));
  const auto copy = original;
  EXPECT_EQ(nullptr, copy.pointerValue);
  EXPECT_EQ("copy", copy.stringValue);
  const detail::LogArgument cString("copy");
  const auto cStringCopy = cString;
  EXPECT_EQ(nullptr, cStringCopy.pointerValue);
  EXPECT_EQ("copy", cStringCopy.stringValue);
}

#if __cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
//==============================================================================
TEST_F(LoggingTest, FormatsStringViews)
{
  info(std::string_view("view {}"), std::string_view("substring", 3));
  EXPECT_EQ("view sub", messages.back());
}
#endif

//==============================================================================
TEST_F(LoggingTest, FormatsEnumsAsUnderlyingIntegers)
{
  enum Unscoped
  {
    Negative = -7
  };
  enum class Scoped : unsigned long long
  {
    Maximum = std::numeric_limits<unsigned long long>::max()
  };
  enum class Character : char
  {
    Letter = 'x'
  };
  enum class Flag : bool
  {
    Set = true
  };
  info("{} {} {} {}", Negative, Scoped::Maximum, Character::Letter, Flag::Set);
  EXPECT_EQ("-7 18446744073709551615 120 1", messages.back());
}

//==============================================================================
TEST_F(LoggingTest, MalformedFormatsDoNotThrow)
{
  for (const std::string format : {"{", "}", "{} {}", "{:bogus}", "{9}"}) {
    EXPECT_NO_THROW(info(format, 7));
    EXPECT_EQ(0u, messages.back().find("[log format error: "));
    EXPECT_NE(std::string::npos, messages.back().find(format));
  }
  info("{}", "literal {} {broken}");
  EXPECT_EQ("literal {} {broken}", messages.back());
}

//==============================================================================
TEST_F(LoggingTest, DefaultSinkUsesExpectedStreamsAndReset)
{
  resetLogSink();
  testing::internal::CaptureStdout();
  testing::internal::CaptureStderr();
  trace("trace message");
  debug("debug message");
  info("info message");
  warn("warn message");
  error("error message");
  fatal("fatal message");
  const auto out = testing::internal::GetCapturedStdout();
  const auto err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(
      "[trace] trace message\n[debug] debug message\n[info] info message\n",
      out);
  EXPECT_EQ(
      "[warn] warn message\n[error] error message\n[fatal] fatal message\n",
      err);
  EXPECT_TRUE(messages.empty());
}

//==============================================================================
TEST_F(LoggingTest, SerializesThreadsAndDeterministicExecutorWorkers)
{
  std::vector<std::thread> threads;
  for (int worker = 0; worker < 4; ++worker) {
    threads.emplace_back([worker] {
      for (int i = 0; i < 100; ++i)
        info("thread {}:{}", worker, i);
    });
  }
  detail::DeterministicExecutor executor;
  executor.setWorkerCount(3);
  executor.parallelFor(128, 4, [](std::size_t i) { info("executor {}", i); });
  for (auto& thread : threads)
    thread.join();
  EXPECT_EQ(528u, messages.size());
  EXPECT_EQ(
      messages.size(),
      std::set<std::string>(messages.begin(), messages.end()).size());
}

//==============================================================================
TEST_F(LoggingTest, SinkReplacementAndLevelChangesCanRunConcurrently)
{
  std::atomic<unsigned> calls{0};
  const auto sink = [&](LogLevel, const std::string&) {
    ++calls;
  };
  setLogSink(sink);
  std::thread replace([&] {
    for (int i = 0; i < 200; ++i) {
      setLogSink(sink);
      setLogLevel(i % 2 ? LogLevel::Trace : LogLevel::Off);
    }
  });
  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i)
    threads.emplace_back([] {
      for (int n = 0; n < 100; ++n)
        info("concurrent {}", n);
    });
  replace.join();
  for (auto& thread : threads)
    thread.join();
  setLogLevel(LogLevel::Trace);
  info("last message");
  EXPECT_GE(calls.load(), 1u);
  // Release the callback before its captured counter leaves scope.
  resetLogSink();
}

//==============================================================================
TEST_F(LoggingTest, CallbacksCanReplaceThemselvesAndExceptionsAreContained)
{
  setLogSink([&](LogLevel, const std::string&) {
    setLogSink([&](LogLevel, const std::string& message) {
      messages.push_back(message);
    });
    info("nested");
  });
  info("outer");
  info("replacement");
  EXPECT_EQ((std::vector<std::string>{"nested", "replacement"}), messages);
  setLogSink(
      [](LogLevel, const std::string&) { throw std::runtime_error("sink"); });
  EXPECT_NO_THROW(info("ignored exception"));
  EXPECT_NO_THROW(
      detail::logMessage(LogLevel::Info, "direct call", nullptr, 0));
}

//==============================================================================
TEST(LoggingLifetime, StaticDestruction)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        logAtShutdown.enabled = true;
        setLogLevel(LogLevel::Trace);
        setLogSink([](LogLevel, const std::string& message) {
          std::cerr << "late sink: " << message << std::endl;
        });
        std::exit(0);
      },
      ::testing::ExitedWithCode(0),
      "late sink: logging at static destruction 42");
}

//==============================================================================
TEST(LoggingLifetime, StreamsFlushAtExit)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        resetLogSink();
        std::ios_base::sync_with_stdio(false);
        std::cerr.tie(nullptr);
        std::cerr << std::nounitbuf << "buffered output at exit";
        std::exit(0);
      },
      ::testing::ExitedWithCode(0),
      "buffered output at exit");
}
