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

#include <dart/dynamics/SphereShape.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>
#include <dart/common/detail/DeterministicExecutor.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <new>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#if __cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
  #include <string_view>
#endif

#include <cerrno>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#if defined(__GLIBCXX__) && !defined(_WIN32)
  #include <pthread.h>
#endif

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
  bool consoleEnabled = false;

  ~LogAtShutdown()
  {
    if (enabled)
      warn("logging at static destruction {}", 42);
    if (consoleEnabled)
      dtwarn << "console logging at static destruction 100% {}";
  }
} logAtShutdown;

class LoggingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setLogCallback(nullptr);
    oldLevel = getLogLevel();
    setLogLevel(LogLevel::Trace);
    setLogSink([this](LogLevel level, const std::string& message) {
      levels.push_back(level);
      messages.push_back(message);
    });
  }

  void TearDown() override
  {
    setLogCallback(nullptr);
    setConsoleStreams(nullptr, nullptr);
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
  volatile int volatileValue = 0;
  const volatile int constVolatileValue = 0;
  info(
      "{} {} {} {}",
      &volatileValue,
      &constVolatileValue,
      static_cast<volatile void*>(&volatileValue),
      static_cast<const volatile void*>(&constVolatileValue));
  const auto volatileMessage = messages.back();
  info(
      "{} {} {} {}",
      const_cast<int*>(&volatileValue),
      const_cast<int*>(&constVolatileValue),
      static_cast<void*>(const_cast<int*>(&volatileValue)),
      static_cast<const void*>(const_cast<int*>(&constVolatileValue)));
  EXPECT_EQ(messages.back(), volatileMessage);
  EXPECT_EQ(std::string::npos, volatileMessage.find("true"));
  volatile int* nullVolatile = nullptr;
  const volatile void* nullConstVolatile = nullptr;
  info("{} {}", nullVolatile, nullConstVolatile);
  EXPECT_EQ("0x0 0x0", messages.back());
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
TEST_F(LoggingTest, FloatingShortestDigitsAtBinaryPowersAndLimits)
{
  const auto floatFromBits = [](std::uint32_t bits) {
    float value;
    std::memcpy(&value, &bits, sizeof value);
    return value;
  };
  const auto doubleFromBits = [](std::uint64_t bits) {
    double value;
    std::memcpy(&value, &bits, sizeof value);
    return value;
  };
  // 2^-96 and 2^-1017: the nearest N-digit decimal does not round-trip, so a
  // plain precision search prints 1.26217745e-29 / 7.1202363472230444e-307.
  info("{} {}", floatFromBits(0x0f800000u), floatFromBits(0x8f800000u));
  EXPECT_EQ("1.2621775e-29 -1.2621775e-29", messages.back());
  info(
      "{} {}",
      doubleFromBits(0x0060000000000000ull),
      doubleFromBits(0x8060000000000000ull));
  EXPECT_EQ("7.120236347223045e-307 -7.120236347223045e-307", messages.back());
  info("{} {}", std::ldexp(1.0f, 87), std::ldexp(1.0f, 90));
  EXPECT_EQ("1.5474251e+26 1.2379401e+27", messages.back());
  info("{} {}", std::ldexp(-1.0f, 87), std::ldexp(-1.0f, 90));
  EXPECT_EQ("-1.5474251e+26 -1.2379401e+27", messages.back());
  info("{} {}", 0.0f, -0.0);
  EXPECT_EQ("0 -0", messages.back());
  info(
      "{} {}",
      std::numeric_limits<float>::min(),
      std::numeric_limits<double>::min());
  EXPECT_EQ("1.1754944e-38 2.2250738585072014e-308", messages.back());
  info("{} {}", 9999999.0f, 0.00001f);
  EXPECT_EQ("9999999 1e-05", messages.back());
  info("{} {}", 1e16, 999999999999999.9);
  EXPECT_EQ("1e+16 999999999999999.9", messages.back());
  info("{:.1000f}", 0.0f);
  EXPECT_EQ(1002u, messages.back().size());
}

//==============================================================================
TEST_F(LoggingTest, FloatingFormattingIgnoresRoundingModeAndRestoresIt)
{
  struct Restore
  {
    int mode = std::fegetround();
    ~Restore()
    {
      std::fesetround(mode);
    }
  } restore;
  std::vector<int> sinkModes;
  setLogSink([&](LogLevel, const std::string& message) {
    sinkModes.push_back(std::fegetround());
    messages.push_back(message);
  });
  for (const int mode : {FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO}) {
    if (std::fesetround(mode) != 0)
      continue;
    sinkModes.clear();
    messages.clear();
    errno = EDOM;
    info("{} {} {:.2f} {:.2e} {:.3g}", 0.1, 0.1f, 0.125, 0.125f, 0.125);
    EXPECT_EQ("0.1 0.1 0.12 1.25e-01 0.125", messages.back());
    info("{} {}", std::ldexp(1.0f, -96), std::ldexp(1.0, -1017));
    EXPECT_EQ("1.2621775e-29 7.120236347223045e-307", messages.back());
    EXPECT_EQ(EDOM, errno);
    ASSERT_EQ(2u, sinkModes.size());
    EXPECT_EQ(mode, sinkModes[0]);
    EXPECT_EQ(mode, sinkModes[1]);
    EXPECT_EQ(mode, std::fegetround());
  }
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
TEST(LoggingConsole, RequiresOrEnablesVirtualTerminalProcessing)
{
  for (const bool readable : {false, true}) {
    for (const bool enabled : {false, true}) {
      for (const bool writable : {false, true}) {
        bool setCalled = false;
        const auto result = detail::enableVirtualTerminal(
            [&](unsigned long& mode) {
              mode = 0x10 | (enabled ? 0x04 : 0);
              return readable;
            },
            [&](unsigned long mode) {
              setCalled = true;
              EXPECT_EQ(0x14u, mode);
              return writable;
            });
        EXPECT_EQ(readable && (enabled || writable), result);
        EXPECT_EQ(readable && !enabled, setCalled);
      }
    }
  }
}

//==============================================================================
TEST_F(LoggingTest, ConsoleTextIsLiteralAndRetainsItsPrefix)
{
  setLogLevel(LogLevel::Off);
  const std::string text = "100% %s {} {broken} {{braces}}";
  dtmsg << text << std::endl;
  dtdbg << text << '\n';
  const auto warningLine = __LINE__ + 1;
  dtwarn << text << std::endl;
  const auto errorLine = __LINE__ + 1;
  dterr << text << std::endl;
  EXPECT_EQ(
      (std::vector<LogLevel>{
          LogLevel::Info, LogLevel::Debug, LogLevel::Warn, LogLevel::Error}),
      levels);
  EXPECT_EQ(
      (std::vector<std::string>{
          "Msg " + text,
          "Dbg " + text,
          "Warning [test_Logging.cpp:" + std::to_string(warningLine) + "] "
              + text,
          "Error [test_Logging.cpp:" + std::to_string(errorLine) + "] "
              + text}),
      messages);
}

//==============================================================================
TEST_F(LoggingTest, PrefixOnlyStatementsDoNotEmitRecords)
{
  testing::internal::CaptureStdout();
  dtmsg;
  dtmsg << std::flush;
  dtmsg << "text\n";
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ((std::vector<std::string>{"Msg text"}), messages);
  EXPECT_EQ("\033[1;32mMsg\033[0m \033[1;32mMsg\033[0m ", output);
}

//==============================================================================
TEST_F(LoggingTest, WrappedSinksRestoreTheirIdentityAndDefaultState)
{
  resetLogSink();
  std::shared_ptr<LogSink> previousOuter;
  auto outer = detail::wrapLogSink([&](std::shared_ptr<LogSink> previous) {
    previousOuter = std::move(previous);
    return [](LogLevel, const std::string&) {
    };
  });
  std::shared_ptr<LogSink> previousInner;
  auto inner = detail::wrapLogSink([&](std::shared_ptr<LogSink> previous) {
    previousInner = std::move(previous);
    return [](LogLevel, const std::string&) {
    };
  });
  detail::restoreLogSinkIf(outer, previousOuter);
  EXPECT_TRUE(detail::hasCustomLogSink());
  detail::restoreLogSinkIf(inner, previousInner);
  EXPECT_FALSE(detail::hasCustomLogSink());
  detail::restoreLogSinkIf(nullptr, outer);
  EXPECT_FALSE(detail::hasCustomLogSink());
  std::weak_ptr<LogSink> retired = outer;
  previousInner.reset();
  outer.reset();
  EXPECT_TRUE(retired.expired());
  detail::wrapLogSink([](std::shared_ptr<LogSink>) { return LogSink{}; });
  EXPECT_FALSE(detail::hasCustomLogSink());
}

//==============================================================================
TEST_F(LoggingTest, RetiredSinksUnwindAndReleaseTheirPredecessors)
{
  for (const bool abandon : {false, true}) {
    auto token = std::make_shared<int>(0);
    std::weak_ptr<int> released = token;
    setLogSink([token](LogLevel, const std::string&) {});
    token.reset();
    std::vector<std::shared_ptr<LogSink>> wrappers;
    std::vector<std::shared_ptr<LogSink>> predecessors;
    for (int i = 0; i < 3; ++i) {
      wrappers.push_back(
          detail::wrapLogSink([&](std::shared_ptr<LogSink> previous) {
            predecessors.push_back(previous);
            return [previous](LogLevel level, const std::string& text) {
              (*previous)(level, text);
            };
          }));
    }
    detail::restoreLogSinkIf(wrappers[0], predecessors[0]);
    detail::restoreLogSinkIf(wrappers[1], predecessors[1]);
    if (abandon)
      resetLogSink();
    else {
      detail::restoreLogSinkIf(wrappers[2], predecessors[2]);
      EXPECT_TRUE(detail::hasCustomLogSink());
      detail::restoreLogSinkIf(predecessors[0], nullptr);
      EXPECT_FALSE(detail::hasCustomLogSink());
      resetLogSink();
    }
    wrappers.clear();
    predecessors.clear();
    EXPECT_TRUE(released.expired());
    EXPECT_FALSE(detail::hasCustomLogSink());
  }
}

//==============================================================================
TEST_F(LoggingTest, DefaultConsoleDoesNotWaitForTheSinkMutex)
{
  std::thread worker;
  std::promise<void> done;
  auto finished = done.get_future();
  testing::internal::CaptureStdout();
  setLogSink([&](LogLevel, const std::string&) {
    resetLogSink();
    worker = std::thread([&] {
      dtmsg << "default from another thread";
      done.set_value();
    });
    EXPECT_EQ(
        std::future_status::ready, finished.wait_for(std::chrono::seconds(2)));
  });
  info("hold the logging mutex");
  worker.join();
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ("\033[1;32mMsg\033[0m default from another thread", output);
}

//==============================================================================
TEST_F(LoggingTest, ConsoleFlushesPartialLinesAndSeparatesCompleteLines)
{
  dtmsg << "partial" << std::flush;
  dtmsg << "first\n\nsecond\n";
  dtmsg << "continued ";
  dtmsg << "text\n";
  EXPECT_EQ(
      (std::vector<std::string>{
          "Msg partial",
          "Msg first",
          "",
          "second",
          "Msg continued ",
          "Msg text"}),
      messages);
}

//==============================================================================
TEST_F(LoggingTest, ConsoleSeparatesPartialLinesAcrossLevelsAndRecoversBadbit)
{
  auto& stream = dtwarn;
  stream << "partial";
  dtdbg << "next\n";
  stream.setstate(std::ios::badbit);
  dtwarn << "recovered\n";
  ASSERT_EQ(3u, messages.size());
  EXPECT_NE(std::string::npos, messages[0].find("] partial"));
  EXPECT_EQ("Dbg next", messages[1]);
  EXPECT_NE(std::string::npos, messages[2].find("] recovered"));
  EXPECT_EQ(
      (std::vector<LogLevel>{LogLevel::Warn, LogLevel::Debug, LogLevel::Warn}),
      levels);
}

//==============================================================================
TEST_F(LoggingTest, ConsoleSeparatesPartialLinesProducedBySinkCallbacks)
{
  testing::internal::CaptureStdout();
  setLogSink([&](LogLevel, const std::string& text) {
    messages.push_back(text);
    if (text == "Msg partial")
      dtmsg << "nested";
  });
  dtmsg << "partial";
  dtmsg << "next\n";
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ((std::vector<std::string>{"Msg partial", "Msg next"}), messages);
  EXPECT_EQ("\033[1;32mMsg\033[0m nested", output);
}

//==============================================================================
TEST_F(LoggingTest, RecursiveConsoleLinesBypassTheSink)
{
  testing::internal::CaptureStdout();
  setLogSink([&](LogLevel, const std::string& text) {
    messages.push_back(text);
    // Bound a regression so it fails without overflowing the stack.
    if (messages.size() < 5)
      dtmsg << "nested\n";
  });
  dtmsg << "outer\n";
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ((std::vector<std::string>{"Msg outer"}), messages);
  EXPECT_EQ("\033[1;32mMsg\033[0m nested\n", output);
}

//==============================================================================
TEST_F(LoggingTest, SinkChangesLeavePartialLinesForTheNextFlush)
{
  dtmsg << "pending";
  std::vector<std::string> replacement;
  setLogSink(
      [&](LogLevel, const std::string& text) { replacement.push_back(text); });
  EXPECT_TRUE(messages.empty());
  dtmsg << "next\n";
  EXPECT_EQ((std::vector<std::string>{"Msg pending", "Msg next"}), replacement);
  dtmsg << "default pending";
  resetLogSink();
  testing::internal::CaptureStdout();
  dtmsg << "default next\n";
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
      "\033[1;32mMsg\033[0m default pending"
      "\033[1;32mMsg\033[0m default next\n",
      output);
}

//==============================================================================
TEST_F(LoggingTest, ConsoleCanLogDuringThreadExitFlush)
{
  testing::internal::CaptureStdout();
  setLogSink([&](LogLevel level, const std::string& text) {
    levels.push_back(level);
    messages.push_back(text);
    if (text.find("partial") != std::string::npos)
      dtmsg << "nested during cleanup";
  });
  std::thread worker([] { dterr << "partial"; });
  worker.join();
  const auto output = testing::internal::GetCapturedStdout();
  ASSERT_EQ(1u, messages.size());
  EXPECT_NE(std::string::npos, messages.front().find("] partial"));
  EXPECT_EQ("\033[1;32mMsg\033[0m nested during cleanup", output);
  EXPECT_EQ((std::vector<LogLevel>{LogLevel::Error}), levels);
}

//==============================================================================
TEST_F(LoggingTest, ConsoleThreadExitContainsSinkExceptions)
{
  setLogSink([](LogLevel, const std::string&) { throw std::bad_alloc(); });
  std::thread worker([] { dtmsg << "partial"; });
  worker.join();
}

//==============================================================================
TEST_F(LoggingTest, DefaultConsoleIsImmediateUnfilteredAndKeepsLegacyColors)
{
  resetLogSink();
  setLogLevel(LogLevel::Off);
  std::ostringstream out;
  std::ostringstream err;
  auto* oldOut = std::cout.rdbuf(out.rdbuf());
  auto* oldErr = std::cerr.rdbuf(err.rdbuf());
  dtmsg << "partial";
  EXPECT_EQ("\033[1;32mMsg\033[0m partial", out.str());
  dtdbg << "debug\n";
  const auto warningLine = __LINE__ + 1;
  dtwarn << "warning";
  const auto errorLine = __LINE__ + 1;
  dterr << "error\n";
  std::cout.rdbuf(oldOut);
  std::cerr.rdbuf(oldErr);
  EXPECT_EQ(
      "\033[1;32mMsg\033[0m partial\033[1;36mDbg\033[0m debug\n", out.str());
  EXPECT_EQ(
      "\033[1;33mWarning [test_Logging.cpp:" + std::to_string(warningLine)
          + "]\033[0m warning\033[1;31mError [test_Logging.cpp:"
          + std::to_string(errorLine) + "]\033[0m error\n",
      err.str());
}

//==============================================================================
TEST_F(LoggingTest, ConsoleLinesFromExecutorWorkersAreNotInterleaved)
{
  detail::DeterministicExecutor executor;
  executor.setWorkerCount(3);
  executor.parallelFor(
      128, 4, [](std::size_t i) { dtmsg << "worker " << i << " {} 100%\n"; });
  ASSERT_EQ(128u, messages.size());
  std::set<std::string> expected;
  for (int i = 0; i < 128; ++i)
    expected.insert("Msg worker " + std::to_string(i) + " {} 100%");
  EXPECT_EQ(expected, std::set<std::string>(messages.begin(), messages.end()));
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
TEST_F(LoggingTest, ReentrantSinkMessagesUseDefaultOutput)
{
  int calls = 0;
  setLogSink([&](LogLevel, const std::string&) {
    // Bounded so an unpatched build fails the assertion instead of recursing.
    if (++calls > 5)
      return;
    info("nested");
  });
  testing::internal::CaptureStdout();
  info("outer");
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(1, calls);
  EXPECT_NE(std::string::npos, output.find("nested"));
  EXPECT_EQ(std::string::npos, output.find("outer"));
}

//==============================================================================
TEST_F(LoggingTest, ReplacementSinkStillReceivesNestedMessagesAfterReentry)
{
  int calls = 0;
  int bCalls = 0;
  setLogSink([&](LogLevel, const std::string&) {
    if (++calls > 5)
      return;
    setLogSink([&](LogLevel, const std::string& message) {
      if (++bCalls > 5)
        return;
      messages.push_back(message);
      info("from B");
    });
    info("to B");
  });
  testing::internal::CaptureStdout();
  info("outer");
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(1, calls);
  EXPECT_EQ(1, bCalls);
  EXPECT_EQ((std::vector<std::string>{"to B"}), messages);
  EXPECT_NE(std::string::npos, output.find("from B"));
  // A throwing callback leaves no stale active state.
  setLogSink(
      [](LogLevel, const std::string&) { throw std::runtime_error("x"); });
  info("throws");
  int again = 0;
  setLogSink([&](LogLevel, const std::string&) { ++again; });
  info("after");
  EXPECT_EQ(1, again);
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
TEST(LoggingLifetime, ConsoleAfterThreadLocalDestruction)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        logAtShutdown.consoleEnabled = true;
        setLogLevel(LogLevel::Trace);
        setLogSink([](LogLevel, const std::string& text) {
          std::cerr << "unexpected sink: " << text << '\n';
        });
        dtmsg << "initialize thread-local streams\n";
        std::exit(0);
      },
      ::testing::ExitedWithCode(0),
      "\\[0m console logging at static destruction 100%");
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

namespace {

class CallbackLoggingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    oldLevel = getLogLevel();
    setLogCallback(nullptr);
    resetLogSink();
    setLogLevel(LogLevel::Trace);
  }

  void TearDown() override
  {
    setLogCallback(nullptr);
    setConsoleStreams(nullptr, nullptr);
    resetLogSink();
    setLogLevel(oldLevel);
  }

  LogLevel oldLevel;
};

class LoggingDeathTest : public CallbackLoggingTest
{
};

} // namespace

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
TEST_F(CallbackLoggingTest, ConsoleStreamsRedirect)
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
TEST_F(CallbackLoggingTest, ConsoleStreamsPerThread)
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
// A callback replaces the selected sink for the DART_WARN family, from
// this test binary and from inside libdart alike, until it is reset.
TEST_F(CallbackLoggingTest, LogCallback)
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
  EXPECT_EQ(records[0].second, "info 1");
  EXPECT_EQ(records[1].second, "warn 2 x");
  EXPECT_EQ(
      records[4].second.find("SphereShape::setRadius: Invalid radius '-1'."),
      0u);
  EXPECT_EQ(records[2].second, "error");
  EXPECT_EQ(records[3].second, "fatal");
}

namespace {

// Counts lines written through the default Console stream providers.
class DefaultOutputCounter
{
public:
  DefaultOutputCounter()
  {
    sCount = &mCount;
    dart::common::setConsoleStreams(&countingStream, &countingStream);
  }

  ~DefaultOutputCounter()
  {
    dart::common::setConsoleStreams(nullptr, nullptr);
    sCount = nullptr;
  }

  int count() const
  {
    return mCount.load();
  }

private:
  // Every default log line ends in one std::endl.
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
TEST_F(CallbackLoggingTest, LogCallbackConcurrentSetAndUnset)
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

// String-like conversion may itself log before the private formatter runs.
struct LogsWhileFormatted
{
  explicit operator std::string() const
  {
    DART_WARN("inner");
    return "formatted";
  }
};

} // namespace

//==============================================================================
// A message logged from inside the callback goes to the default output rather
// than recursing into the callback.
TEST_F(CallbackLoggingTest, LogCallbackReentry)
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
TEST_F(CallbackLoggingTest, LogCallbackThrows)
{
  Counter calls;
  DefaultOutputCounter output;
  dart::common::setLogCallback(&throwFromInside, &calls);
  EXPECT_NO_THROW(DART_WARN("first"));
  EXPECT_NO_THROW(DART_WARN("second"));
  dart::common::setLogCallback(nullptr);

  EXPECT_EQ(calls.calls.load(), 2);
  // The first throw adds one report unless an earlier test already made it.
  EXPECT_GE(output.count(), 2);
  EXPECT_LE(output.count(), 3);
}

//==============================================================================
TEST_F(LoggingDeathTest, LogCallbackThrowsWithQuietDefaultOutput)
{
  // A fresh process keeps the one-time report independent of other tests and
  // of --gtest_repeat: the threadsafe style re-executes the binary instead of
  // forking this process, whose report may already have been made.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        Counter calls;
        DefaultOutputCounter output;
        dart::common::setLogLevel(dart::common::LogLevel::Off);
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

#if defined(__GLIBCXX__) && !defined(_WIN32)
//==============================================================================
TEST_F(LoggingDeathTest, LogCallbackCanExitItsThread)
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
TEST_F(CallbackLoggingTest, LogCallbackReentrantFormatting)
{
  std::vector<LogRecord> records;
  dart::common::setLogCallback(&capture, &records);
  DART_WARN("outer {}", LogsWhileFormatted{});
  dart::common::setLogCallback(nullptr);

  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].second, "inner");
  EXPECT_EQ(records[1].second, "outer formatted");
}

//==============================================================================
// The bridge receives messages even with the independent sink disabled.
TEST_F(CallbackLoggingTest, LogCallbackWithSinkDisabled)
{
  std::vector<LogRecord> records;
  dart::common::setLogLevel(LogLevel::Off);
  dart::common::setLogCallback(&capture, &records);
  dart::common::trace("trace despite Off");
  DART_WARN("sink off");
  dart::dynamics::SphereShape(1.0).setRadius(-1.0);
  dart::common::setLogCallback(nullptr);
  sharedErr().str("");
  dart::common::setConsoleStreams(&sharedOutStream, &sharedErrStream);
  DART_WARN("filtered without callback");
  EXPECT_TRUE(sharedErr().str().empty());
  dart::common::setLogLevel(LogLevel::Info);
  DART_WARN("no callback");

  ASSERT_EQ(records.size(), 3u);
  EXPECT_EQ(records[0].second, "trace despite Off");
  EXPECT_EQ(records[1].second, "sink off");
  EXPECT_EQ(
      records[2].second.find("SphereShape::setRadius: Invalid radius '-1'."),
      0u);
  EXPECT_EQ(sharedErr().str(), "[warn] no callback\n");
}

//==============================================================================
// Format specifiers affect their own message only; manipulators are not
// arguments.
TEST_F(CallbackLoggingTest, FallbackFormattingStateDoesNotPersist)
{
  std::vector<LogRecord> records;
  dart::common::setLogCallback(&capture, &records);
  dart::common::warn("fixed {:.2f}", 1.25);
  dart::common::warn("general {}", 1.23456);
  dart::common::warn("unsupported {:x}", 255);
  dart::common::warn("dec {}", 255);
  dart::common::setLogCallback(nullptr);

  ASSERT_EQ(records.size(), 4u);
  EXPECT_EQ(records[0].second, "fixed 1.25");
  EXPECT_EQ(records[1].second, "general 1.23456");
  EXPECT_EQ(0u, records[2].second.find("[log format error:"));
  EXPECT_EQ(records[3].second, "dec 255");
}

//==============================================================================
// The DART_WARN family prints through the Console stream providers.
TEST_F(CallbackLoggingTest, FallbackUsesConsoleStreams)
{
  sharedOut().str("");
  sharedErr().str("");
  dart::common::setConsoleStreams(&sharedOutStream, &sharedErrStream);
  dart::common::info("info {}", 1);
  dart::common::warn("warn {} {}", 2, "x");
  dart::common::error("error");
  dart::common::setConsoleStreams(nullptr, nullptr);

  EXPECT_NE(sharedOut().str().find("[info]"), std::string::npos);
  EXPECT_NE(sharedOut().str().find("info 1\n"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("[warn]"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("warn 2 x\n"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("[error]"), std::string::npos);
  EXPECT_NE(sharedErr().str().find("error\n"), std::string::npos);
}

//==============================================================================
TEST_F(CallbackLoggingTest, LogCallbackBypassesSinkMutex)
{
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
  std::atomic<bool> delivered{false};
  setLogSink([&](LogLevel, const std::string&) {
    entered = true;
    while (!release.load())
      std::this_thread::yield();
  });
  std::thread blocked([] { warn("holds the sink lock"); });
  while (!entered.load())
    std::this_thread::yield();
  setLogCallback(
      [](LogLevel, const char*, std::size_t, void* data) {
        *static_cast<std::atomic<bool>*>(data) = true;
      },
      &delivered);
  std::thread callback([] { warn("callback bypasses the sink lock"); });
  const auto deadline
      = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!delivered.load() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();
  const bool bypassed = delivered.load();
  release = true;
  callback.join();
  blocked.join();
  EXPECT_TRUE(bypassed);
}

#if defined(__GLIBCXX__) && !defined(_WIN32)
//==============================================================================
TEST_F(LoggingDeathTest, NestedSinkCallbackCanExitItsThread)
{
  EXPECT_EXIT(
      {
        setLogSink([](LogLevel, const std::string&) {
          setLogCallback([](LogLevel, const char*, std::size_t, void*) {
            pthread_exit(nullptr);
          });
          warn("exit through the sink catch barrier {}", 42);
        });
        std::thread([] { warn("outer sink {}", 42); }).join();
        setLogCallback(nullptr);
        resetLogSink();
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0),
      "");
}
#endif

//==============================================================================
TEST_F(CallbackLoggingTest, ConsoleProvidersRemainValidWithCustomSink)
{
  sharedOut().str("");
  sharedErr().str("");
  setConsoleStreams(&sharedOutStream, &sharedErrStream);
  std::vector<std::string> records;
  setLogSink([&](LogLevel level, const std::string& message) {
    records.push_back(message);
    detail::defaultLogSink(level, message);
  });
  dtmsg << "literal {} %s" << std::flush;
  dtwarn << "warning" << std::endl;
  ASSERT_EQ(2u, records.size());
  EXPECT_EQ("Msg literal {} %s", records[0]);
  EXPECT_EQ("\033[1;32mMsg\033[0m literal {} %s", sharedOut().str());
  EXPECT_NE(std::string::npos, sharedErr().str().find("warning\n"));
  resetLogSink();
  dtmsg << "arbitrary provider" << std::endl;
  EXPECT_NE(std::string::npos, sharedOut().str().find("arbitrary provider\n"));
}

//==============================================================================
TEST_F(CallbackLoggingTest, ObserverRestorationDoesNotReviveRetiredOwners)
{
  std::vector<LogRecord> records;
  setLogCallback(&capture, &records);
  int first = 0;
  int second = 0;
  auto a = std::make_shared<LogSink>(
      [&](LogLevel, const std::string&) { ++first; });
  auto b = std::make_shared<LogSink>(
      [&](LogLevel, const std::string&) { ++second; });
  auto* outer = detail::registerLogObserver(a);
  warn("outer");
  auto* inner = detail::registerLogObserver(b);
  warn("inner");
  detail::unregisterLogObserver(outer);
  std::weak_ptr<LogSink> retired = a;
  a.reset();
  EXPECT_TRUE(retired.expired());
  warn("inner after outer retired");
  detail::unregisterLogObserver(inner);
  warn("after both retired");
  EXPECT_EQ(1, first);
  EXPECT_EQ(2, second);
  EXPECT_EQ(4u, records.size());
}

//==============================================================================
TEST_F(CallbackLoggingTest, NestedObserversRestoreWithoutRetainingSinks)
{
  setLogCallback([](LogLevel, const char*, std::size_t, void*) {});
  int first = 0;
  int second = 0;
  auto a = std::make_shared<LogSink>(
      [&](LogLevel, const std::string&) { ++first; });
  auto b = std::make_shared<LogSink>(
      [&](LogLevel, const std::string&) { ++second; });
  auto* outer = detail::registerLogObserver(a);
  auto* inner = detail::registerLogObserver(b);
  warn("inner");
  detail::unregisterLogObserver(inner);
  warn("outer restored");
  detail::unregisterLogObserver(outer);
  EXPECT_EQ(1, first);
  EXPECT_EQ(1, second);
}

//==============================================================================
TEST_F(CallbackLoggingTest, ObserverFailureDoesNotUndoCallbackConsumption)
{
  std::vector<LogRecord> records;
  setLogCallback(&capture, &records);
  int fallback = 0;
  setLogSink([&](LogLevel, const std::string&) { ++fallback; });
  auto observer = std::make_shared<LogSink>([](LogLevel, const std::string&) {
    throw std::runtime_error("observer");
  });
  auto* token = detail::registerLogObserver(observer);
  warn("consumed");
  detail::unregisterLogObserver(token);
  EXPECT_EQ(1u, records.size());
  EXPECT_EQ(0, fallback);
}

#if defined(__GLIBCXX__) && !defined(_WIN32)
//==============================================================================
TEST_F(LoggingDeathTest, ObserverCanExitItsThread)
{
  EXPECT_EXIT(
      {
        setLogCallback([](LogLevel, const char*, std::size_t, void*) {});
        auto observer = std::make_shared<LogSink>(
            [](LogLevel, const std::string&) { pthread_exit(nullptr); });
        auto* token = detail::registerLogObserver(observer);
        std::thread([] { warn("exit through observation {}", 42); }).join();
        detail::unregisterLogObserver(token);
        setLogCallback(nullptr);
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0),
      "");
}
#endif

//==============================================================================
TEST_F(LoggingTest, FormattedRecursionDoesNotInheritConsolePrefix)
{
  setLogSink([](LogLevel, const std::string&) { info("x"); });
  testing::internal::CaptureStdout();
  dtwarn << "outer console record" << std::endl;
  const auto output = testing::internal::GetCapturedStdout();
  EXPECT_EQ("[info] x\n", output);
}
