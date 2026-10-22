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

#include <dart/common/Console.hpp>
#include <dart/common/Logging.hpp>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <cerrno>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
  #include <io.h>
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#else
  #include <unistd.h>
#endif

// Apple's system libc++ marks floating-point to_chars as macOS 13.3+, so older
// deployment targets (or SDK headers without the capability macro) format
// float/double through classic streams. DART_LOGGING_FORCE_LEGACY_FLOAT is a
// source-private switch that lets the standalone fixture exercise this path.
#if defined(DART_LOGGING_FORCE_LEGACY_FLOAT)
  #define DART_LOGGING_LEGACY_FLOAT 1
#elif defined(__APPLE__) && defined(_LIBCPP_VERSION)
  #if !defined(__ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__)                  \
      || __ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__ < 130300                \
      || !defined(_LIBCPP_AVAILABILITY_HAS_TO_CHARS_FLOATING_POINT)            \
      || !_LIBCPP_AVAILABILITY_HAS_TO_CHARS_FLOATING_POINT
    #define DART_LOGGING_LEGACY_FLOAT 1
  #endif
#endif
#ifndef DART_LOGGING_LEGACY_FLOAT
  #define DART_LOGGING_LEGACY_FLOAT 0
#endif

namespace dart::common {
namespace detail {
struct LogObserver
{
  std::weak_ptr<LogSink> sink;
  LogObserver* previous;
  LogObserver* allocated;
  std::atomic<bool> active{true};
};
} // namespace detail
namespace {

std::atomic<detail::LogObserver*> logObserver{nullptr};
// Keep retired tokens reachable for readers; their payload owns no sink.
detail::LogObserver* allocatedObservers = nullptr;

// One immutable {callback, userData} pair per setLogCallback() call, published
// through a single lock-free pointer, so a reader never pairs one call's
// callback with another call's user data. Every slot stays reachable through
// \c previous: an in-flight forwardToLogCallback() may still be reading an old
// one, so none is freed, and the chain keeps leak checkers quiet. The cost is
// one small allocation per setLogCallback() call.
struct Slot
{
  LogCallback callback;
  void* userData;
  const Slot* previous;
};

std::atomic<const Slot*> sSlot{nullptr};
static_assert(
    std::atomic<const Slot*>::is_always_lock_free,
    "the logging path must not take a lock, so that fork() cannot inherit it "
    "held");

// Set while this thread runs the callback: a log call from inside it goes to
// the default output rather than back into the callback. The child of a
// fork() inherits the forking thread's copy, so do not fork from inside the
// callback.
thread_local bool tInsideCallback = false;

// Set by the first callback that throws, so that the throw is reported once.
// Namespace scope, so no guard lock sits on the throw-report path.
std::atomic<bool> sThrowReported{false};

struct InsideCallback
{
  InsideCallback()
  {
    tInsideCallback = true;
  }
  ~InsideCallback()
  {
    tInsideCallback = false;
  }
};

std::atomic<LogLevel> loggingLevel{
    DART_ACTIVE_LOG_LEVEL < DART_LOG_LEVEL_INFO
        ? LogLevel::Info
        : static_cast<LogLevel>(DART_ACTIVE_LOG_LEVEL)};

// Flush at exit; stream objects survive Init destruction for late logging.
std::ios_base::Init streams;

struct LoggingState
{
  std::recursive_mutex mutex;
  std::shared_ptr<LogSink> sink;
  std::map<
      std::weak_ptr<LogSink>,
      std::weak_ptr<LogSink>,
      std::owner_less<std::weak_ptr<LogSink>>>
      retired;
};

std::atomic<bool> hasSink{false};

struct ConsoleOrigin
{
  std::size_t prefixLength;
  int color;
  bool newline;
};

thread_local const ConsoleOrigin* consoleOrigin = nullptr;

//==============================================================================
LoggingState& loggingState()
{
  // Process lifetime: logging is also used by static destructors.
  static auto* state = new LoggingState;
  return *state;
}

// Sinks currently executing on this thread, innermost first.
struct ActiveSink
{
  const LogSink* sink;
  const ActiveSink* outer;
};

thread_local const ActiveSink* activeSinks = nullptr;

struct ActiveSinkScope : ActiveSink
{
  explicit ActiveSinkScope(const LogSink* s) : ActiveSink{s, activeSinks}
  {
    activeSinks = this;
  }

  ~ActiveSinkScope()
  {
    activeSinks = outer;
  }
};

bool isSinkActive(const LogSink* sink)
{
  for (auto* active = activeSinks; active; active = active->outer) {
    if (active->sink == sink)
      return true;
  }
  return false;
}

//==============================================================================
bool useLogColor(std::FILE* stream)
{
  const auto* term = std::getenv("TERM");
  if (std::getenv("NO_COLOR") || (term && std::strcmp(term, "dumb") == 0))
    return false;
#ifdef _WIN32
  if (!_isatty(_fileno(stream)))
    return false;
  const auto handle = reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(stream)));
  return detail::enableVirtualTerminal(
      [handle](unsigned long& mode) {
        return GetConsoleMode(handle, &mode) != 0;
      },
      [handle](unsigned long mode) {
        return SetConsoleMode(handle, mode) != 0;
      });
#else
  return isatty(fileno(stream)) != 0;
#endif
}

//==============================================================================
void writeDefaultLog(LogLevel level, const std::string& message)
{
  static std::ios_base::Init init;
  if (level < LogLevel::Trace || level >= LogLevel::Off)
    return;

  static const char* const names[]{
      "trace", "debug", "info", "warn", "error", "fatal"};
  static const int colors[]{37, 36, 32, 33, 31, 35};
  static const bool colorStdout = useLogColor(stdout);
  static const bool colorStderr = useLogColor(stderr);
  const auto index = static_cast<unsigned>(level);
  auto& stream = level >= LogLevel::Warn ? consoleErr() : consoleOut();
  if (consoleOrigin) {
    const auto prefix = consoleOrigin->prefixLength;
    if (prefix)
      stream << "\033[1;" << consoleOrigin->color << 'm';
    stream.write(message.data(), prefix);
    if (prefix)
      stream << "\033[0m";
    stream.write(message.data() + prefix, message.size() - prefix);
    if (consoleOrigin->newline)
      stream << '\n';
    stream.flush();
    return;
  }
  const bool color = level >= LogLevel::Warn ? colorStderr : colorStdout;
  if (color)
    stream << "\033[1;" << colors[index] << 'm';
  stream << '[' << names[index] << ']';
  if (color)
    stream << "\033[0m";
  stream << ' ' << message << std::endl;
}

//==============================================================================
void dispatchLog(
    LogLevel level, const std::string& message, const ConsoleOrigin* origin)
{
  struct OriginScope
  {
    const ConsoleOrigin* previous = consoleOrigin;
    ~OriginScope()
    {
      consoleOrigin = previous;
    }
  } scope;
  consoleOrigin = origin;
  try {
    auto& state = loggingState();
    std::lock_guard<std::recursive_mutex> lock(state.mutex);
    // A callback may replace itself. Keep it alive until it returns.
    auto sink = state.sink;
    // Prefix-only statements retain terminal bytes without an empty record.
    const bool prefixOnly = origin && !origin->newline && origin->prefixLength
                            && message.size() == origin->prefixLength + 1;
    if (sink && !prefixOnly && !isSinkActive(sink.get())) {
      ActiveSinkScope active(sink.get());
      (*sink)(level, message);
    } else
      writeDefaultLog(level, message);
#if defined(__GLIBCXX__)
  } catch (const abi::__forced_unwind&) {
    throw;
#endif
  } catch (...) {
    // Logging must not interrupt application error handling or thread cleanup.
  }
}

//==============================================================================
template <typename T>
void appendNumber(std::string& message, T value, int base = 10)
{
  char buffer[32];
  const auto result
      = std::to_chars(buffer, buffer + sizeof(buffer), value, base);
  if (result.ec != std::errc{})
    throw std::invalid_argument("number conversion failed");
  message.append(buffer, result.ptr);
}

//==============================================================================
template <typename T>
bool appendNonFinite(std::string& message, T value)
{
  if (std::isfinite(value))
    return false;
  if (std::signbit(value))
    message += '-';
  message += std::isnan(value) ? "nan" : "inf";
  return true;
}

//==============================================================================
#if DART_LOGGING_LEGACY_FLOAT
//==============================================================================
// Saves the full floating-point environment, forces round-to-nearest and
// restores both (and errno) on every exit, before any sink callback runs.
class NearestRoundingScope
{
public:
  NearestRoundingScope() : mErrno(errno)
  {
    if (std::feholdexcept(&mEnv) != 0) {
      errno = mErrno;
      throw std::invalid_argument("number conversion failed");
    }
    if (std::fesetround(FE_TONEAREST) != 0) {
      std::fesetenv(&mEnv);
      errno = mErrno;
      throw std::invalid_argument("number conversion failed");
    }
  }

  ~NearestRoundingScope()
  {
    std::fesetenv(&mEnv);
    errno = mErrno;
  }

  NearestRoundingScope(const NearestRoundingScope&) = delete;
  NearestRoundingScope& operator=(const NearestRoundingScope&) = delete;

private:
  std::fenv_t mEnv;
  int mErrno;
};

//==============================================================================
template <typename T>
std::string streamFloat(T value, char presentation, int precision)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  if (presentation == 'e')
    stream << std::scientific;
  else if (presentation == 'f')
    stream << std::fixed;
  stream << std::setprecision(precision) << value;
  if (!stream)
    throw std::invalid_argument("number conversion failed");
  return stream.str();
}

//==============================================================================
// Parses directly into T (never via a wider type). A failbit is accepted only
// for a matching nonzero subnormal, where strtof/strtod report ERANGE.
template <typename T>
bool parsesTo(const std::string& text, T value)
{
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  T parsed{};
  stream >> parsed;
  if (stream.bad() || stream.rdbuf()->sgetc() != std::char_traits<char>::eof())
    return false;
  if (stream.fail() && std::fpclassify(parsed) != FP_SUBNORMAL)
    return false;
  return parsed == value && std::signbit(parsed) == std::signbit(value);
}

//==============================================================================
// Decimal-lattice neighbor of "d.ddde+XX" at the same digit count, with
// decade carry/borrow (9.99e-01 + 1 = 1.00e+00, 1.00e+00 - 1 = 9.99e-01).
std::string decimalNeighbor(const std::string& text, int delta)
{
  const bool negative = text.front() == '-';
  const auto exponentPosition = text.find('e');
  std::uint64_t digits = 0;
  int count = 0;
  for (std::size_t i = negative ? 1 : 0; i < exponentPosition; ++i) {
    if (text[i] == '.')
      continue;
    digits = digits * 10 + static_cast<std::uint64_t>(text[i] - '0');
    ++count;
  }
  int exponent = std::atoi(text.c_str() + exponentPosition + 1);
  std::uint64_t lower = 1;
  for (int i = 1; i < count; ++i)
    lower *= 10;
  if (delta > 0) {
    if (digits == lower * 10 - 1) {
      digits = lower;
      ++exponent;
    } else {
      ++digits;
    }
  } else {
    if (digits == lower) {
      digits = lower * 10 - 1;
      --exponent;
    } else {
      --digits;
    }
  }
  const std::string digitText = std::to_string(digits);
  std::string result = negative ? "-" : "";
  result += digitText[0];
  if (digitText.size() > 1)
    result += '.' + digitText.substr(1);
  result += exponent < 0 ? "e-" : "e+";
  const std::string exponentText = std::to_string(std::abs(exponent));
  if (exponentText.size() < 2)
    result += '0';
  return result + exponentText;
}

//==============================================================================
// Shortest round-tripping scientific digits, matching to_chars. The nearest
// stream candidate can be longer than the shortest at binary powers, so when
// it does not round-trip the immediate neighbors at that digit count are tried.
template <typename T>
std::string shortestScientific(T value)
{
  NearestRoundingScope scope;
  for (int n = 1; n <= std::numeric_limits<T>::max_digits10; ++n) {
    const std::string nearest = streamFloat(value, 'e', n - 1);
    if (parsesTo(nearest, value))
      return nearest;
    const std::string above = decimalNeighbor(nearest, 1);
    const std::string below = decimalNeighbor(nearest, -1);
    const bool aboveOk = parsesTo(above, value);
    const bool belowOk = parsesTo(below, value);
    if (aboveOk && belowOk)
      throw std::invalid_argument("number conversion failed");
    if (aboveOk || belowOk)
      return aboveOk ? above : below;
  }
  throw std::invalid_argument("number conversion failed");
}

//==============================================================================
template <typename T>
void appendExplicitFloat(
    std::string& message, T value, char presentation, int precision)
{
  NearestRoundingScope scope;
  message += streamFloat(value, presentation, precision);
}

#endif // DART_LOGGING_LEGACY_FLOAT

//==============================================================================
template <typename T>
void appendFloat(
    std::string& message, T value, char presentation, int precision)
{
  if (appendNonFinite(message, value))
    return;

  if (!presentation) {
#if DART_LOGGING_LEGACY_FLOAT
    const std::string scientific = shortestScientific(value);
    const char* const numberBegin = scientific.data();
    const char* const numberEnd = numberBegin + scientific.size();
#else
    char buffer[64];
    const auto result = std::to_chars(
        buffer, buffer + sizeof(buffer), value, std::chars_format::scientific);
    if (result.ec != std::errc{})
      throw std::invalid_argument("number conversion failed");
    const char* const numberBegin = buffer;
    const char* const numberEnd = result.ptr;
#endif
    std::string_view number(numberBegin, numberEnd - numberBegin);
    const auto exponentPosition = number.find('e');
    if (exponentPosition == std::string_view::npos) {
      message += number;
      return;
    }
    int exponent = 0;
    const char* digits = numberBegin + exponentPosition + 1;
    // from_chars does not accept a leading plus sign.
    if (digits != numberEnd && *digits == '+')
      ++digits;
    if (digits == numberEnd)
      throw std::invalid_argument("number conversion failed");
    const auto exponentResult = std::from_chars(digits, numberEnd, exponent);
    if (exponentResult.ec != std::errc{} || exponentResult.ptr != numberEnd)
      throw std::invalid_argument("number conversion failed");
    // Keep the shortest digits from the conversion, with fmt's default layout.
    if (exponent < -4
        || exponent >= std::min(16, std::numeric_limits<T>::digits10 + 1)) {
      message += number;
      return;
    }
    number = number.substr(0, exponentPosition);
    if (number.front() == '-') {
      message += '-';
      number.remove_prefix(1);
    }
    if (exponent < 0) {
      message += "0.";
      message.append(-exponent - 1, '0');
    }
    int written = 0;
    const int digitCount = static_cast<int>(number.size())
                           - (number.find('.') != std::string_view::npos);
    for (const char digit : number) {
      if (digit == '.')
        continue;
      message += digit;
      ++written;
      if (written == exponent + 1 && written < digitCount)
        message += '.';
    }
    if (written < exponent + 1)
      message.append(exponent + 1 - written, '0');
    return;
  }

#if DART_LOGGING_LEGACY_FLOAT
  appendExplicitFloat(message, value, presentation, precision);
#else
  const auto style = presentation == 'e'   ? std::chars_format::scientific
                     : presentation == 'f' ? std::chars_format::fixed
                                           : std::chars_format::general;
  const auto convert = [&](char* begin, char* end) {
    return std::to_chars(begin, end, value, style, precision);
  };
  char buffer[128];
  auto result = convert(buffer, buffer + sizeof(buffer));
  if (result.ec == std::errc{}) {
    message.append(buffer, result.ptr);
    return;
  }
  if (result.ec != std::errc::value_too_large)
    throw std::invalid_argument("number conversion failed");

  std::string largeBuffer(sizeof(buffer) * 2, '\0');
  for (;;) {
    result
        = convert(largeBuffer.data(), largeBuffer.data() + largeBuffer.size());
    if (result.ec == std::errc{}) {
      message.append(largeBuffer.data(), result.ptr);
      return;
    }
    if (result.ec != std::errc::value_too_large || largeBuffer.size() >= 8192)
      throw std::invalid_argument("number conversion failed");
    largeBuffer.resize(largeBuffer.size() * 2);
  }
#endif
}

//==============================================================================
void appendArgument(
    std::string& message,
    const detail::LogArgument& argument,
    std::string_view field)
{
  using Type = detail::LogArgument::Type;
  char presentation = 0;
  int precision = 0;
  if (!field.empty()) {
    if (field.size() < 4 || field.substr(0, 2) != ":."
        || field.find_first_not_of("0123456789", 2) != field.size() - 1
        || (field.back() != 'e' && field.back() != 'f' && field.back() != 'g'))
      throw std::invalid_argument("unsupported format field");
    const auto result = std::from_chars(
        field.data() + 2, field.data() + field.size() - 1, precision);
    if (result.ec != std::errc{}
        || result.ptr != field.data() + field.size() - 1 || precision > 1000)
      throw std::invalid_argument("invalid precision");
    if (argument.type != Type::Float && argument.type != Type::Double
        && argument.type != Type::LongDouble)
      throw std::invalid_argument("precision requires floating-point argument");
    presentation = field.back();
    if (presentation == 'g' && precision == 0)
      precision = 1;
  }

  switch (argument.type) {
    case Type::String:
      message += argument.stringValue;
      break;
    case Type::Signed:
      appendNumber(message, argument.signedValue);
      break;
    case Type::Unsigned:
      appendNumber(message, argument.unsignedValue);
      break;
    case Type::Float:
      appendFloat(message, argument.floatValue, presentation, precision);
      break;
    case Type::Double:
      appendFloat(message, argument.doubleValue, presentation, precision);
      break;
    case Type::LongDouble: {
      if (appendNonFinite(message, argument.longDoubleValue))
        break;
      // Classic streams also support platforms without long double to_chars.
      // Default precision preserves round trips; layout may differ from fmt.
      std::ostringstream stream;
      stream.imbue(std::locale::classic());
      if (presentation == 'e')
        stream << std::scientific;
      else if (presentation == 'f')
        stream << std::fixed;
      stream << std::setprecision(
          presentation ? precision
                       : std::numeric_limits<long double>::max_digits10)
             << argument.longDoubleValue;
      message += stream.str();
      break;
    }
    case Type::Bool:
      message += argument.boolValue ? "true" : "false";
      break;
    case Type::Char:
      message += argument.charValue;
      break;
    case Type::Pointer:
      message += "0x";
      appendNumber(
          message, reinterpret_cast<std::uintptr_t>(argument.pointerValue), 16);
      break;
  }
}

//==============================================================================
std::string formatMessage(
    const std::string& format,
    const detail::LogArgument* arguments,
    std::size_t count)
{
  std::string message;
  message.reserve(format.size());
  std::size_t argument = 0;
  for (std::size_t i = 0; i < format.size(); ++i) {
    const char character = format[i];
    if (character != '{' && character != '}') {
      message += character;
      continue;
    }
    if (i + 1 < format.size() && format[i + 1] == character) {
      message += character;
      ++i;
      continue;
    }
    if (character == '}')
      throw std::invalid_argument("unmatched '}'");
    const auto end = format.find('}', i + 1);
    if (end == std::string::npos)
      throw std::invalid_argument("unmatched '{'");
    if (argument == count)
      throw std::invalid_argument("too few arguments");
    appendArgument(
        message,
        arguments[argument++],
        std::string_view(format.data() + i + 1, end - i - 1));
    i = end;
  }
  return message;
}

} // namespace

void setLogCallback(LogCallback callback, void* userData)
{
  const Slot* previous = sSlot.load(std::memory_order_acquire);
  sSlot.store(
      new Slot{callback, callback ? userData : nullptr, previous},
      std::memory_order_release);
}

//==============================================================================
LogSink setLogSink(LogSink sink)
{
  auto replacement
      = sink ? std::make_shared<LogSink>(std::move(sink)) : nullptr;
  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  auto previous = state.sink ? *state.sink : LogSink{};
  state.sink.swap(replacement);
  state.retired.clear();
  hasSink.store(static_cast<bool>(state.sink), std::memory_order_relaxed);
  return previous;
}

//==============================================================================
void resetLogSink()
{
  std::shared_ptr<LogSink> previous;
  auto& state = loggingState();
  {
    std::lock_guard<std::recursive_mutex> lock(state.mutex);
    previous.swap(state.sink);
    state.retired.clear();
    hasSink.store(false, std::memory_order_relaxed);
  }
}

//==============================================================================
void setLogLevel(LogLevel level)
{
  loggingLevel.store(level, std::memory_order_relaxed);
}

//==============================================================================
LogLevel getLogLevel()
{
  return loggingLevel.load(std::memory_order_relaxed);
}

namespace detail {

std::recursive_mutex& logSinkMutex()
{
  return loggingState().mutex;
}

LogObserver* registerLogObserver(const std::shared_ptr<LogSink>& observer)
{
  auto* slot
      = new LogObserver{observer, logObserver.load(), allocatedObservers};
  allocatedObservers = slot;
  logObserver.store(slot, std::memory_order_release);
  return slot;
}

void unregisterLogObserver(LogObserver* observer)
{
  if (!observer)
    return;
  observer->active.store(false, std::memory_order_release);
  auto* current = logObserver.load(std::memory_order_acquire);
  while (current && !current->active.load(std::memory_order_acquire))
    current = current->previous;
  logObserver.store(current, std::memory_order_release);
}

bool hasLogCallback()
{
  const Slot* slot = sSlot.load(std::memory_order_acquire);
  return slot && slot->callback;
}

bool forwardToLogCallback(
    LogLevel level, const char* message, std::size_t length)
{
  const Slot* slot = sSlot.load(std::memory_order_acquire);
  if (!slot || !slot->callback || tInsideCallback)
    return false;
  InsideCallback guard;
  try {
    slot->callback(level, message, length, slot->userData);
    return true;
#if defined(__GLIBCXX__)
  } catch (const abi::__forced_unwind&) {
    throw;
#endif
  } catch (...) {
    if (!sThrowReported.exchange(true)) {
      // DART's console header marks it as an error for consoles and for
      // bridges that read the header, such as dartpy's.
      colorErr("Error", __FILE__, __LINE__, 31)
          << "A log callback threw. DART ignores the exception and sends the "
             "message to the default output; later throws are not reported."
          << std::endl;
    }
    return false;
  }
}

//==============================================================================
std::shared_ptr<LogSink> wrapLogSink(
    const std::function<LogSink(std::shared_ptr<LogSink>)>& wrap)
{
  std::shared_ptr<LogSink> replacement;
  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  auto sink = wrap(state.sink);
  if (sink)
    replacement = std::make_shared<LogSink>(std::move(sink));
  state.sink.swap(replacement);
  hasSink.store(static_cast<bool>(state.sink), std::memory_order_relaxed);
  return state.sink;
}

//==============================================================================
void restoreLogSinkIf(
    std::shared_ptr<LogSink> installed, std::shared_ptr<LogSink> previous)
{
  if (!installed)
    return;
  std::vector<std::shared_ptr<LogSink>> removed;
  auto& state = loggingState();
  std::lock_guard<std::recursive_mutex> lock(state.mutex);
  for (auto it = state.retired.begin(); it != state.retired.end();) {
    if (it->first.expired())
      it = state.retired.erase(it);
    else
      ++it;
  }
  if (state.sink == installed) {
    state.sink.swap(previous);
    while (state.sink) {
      const auto retired = state.retired.find(state.sink);
      if (retired == state.retired.end())
        break;
      // Keep the wrapper (and its predecessor) alive until after unlocking.
      removed.push_back(std::move(state.sink));
      state.sink = retired->second.lock();
      state.retired.erase(retired);
    }
    hasSink.store(static_cast<bool>(state.sink), std::memory_order_relaxed);
  } else {
    state.retired[installed] = previous;
  }
}

//==============================================================================
bool hasCustomLogSink()
{
  return hasSink.load(std::memory_order_relaxed);
}

//==============================================================================
void logRecord(LogLevel level, const std::string& message)
{
  dispatchLog(level, message, nullptr);
}

//==============================================================================
void logConsoleRecord(
    LogLevel level,
    const std::string& message,
    std::size_t prefixLength,
    int color,
    bool newline)
{
  const ConsoleOrigin origin{prefixLength, color, newline};
  dispatchLog(level, message, &origin);
}

//==============================================================================
void defaultLogSink(LogLevel level, const std::string& message)
{
  writeDefaultLog(level, message);
}

//==============================================================================
LogArgument::LogArgument(const std::string& value)
  : type(Type::String), pointerValue(nullptr), stringValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(std::string&& value)
  : type(Type::String), pointerValue(nullptr), stringValue(std::move(value))
{
}

//==============================================================================
LogArgument::LogArgument(const char* value)
  : type(Type::String),
    pointerValue(nullptr),
    stringValue(value ? value : "(null)")
{
}

//==============================================================================
LogArgument::LogArgument(bool value) : type(Type::Bool), boolValue(value) {}

//==============================================================================
LogArgument::LogArgument(char value) : type(Type::Char), charValue(value) {}

//==============================================================================
LogArgument::LogArgument(float value) : type(Type::Float), floatValue(value) {}

//==============================================================================
LogArgument::LogArgument(double value) : type(Type::Double), doubleValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(long double value)
  : type(Type::LongDouble), longDoubleValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(const void* value)
  : type(Type::Pointer), pointerValue(value)
{
}

//==============================================================================
LogArgument::LogArgument(std::nullptr_t)
  : LogArgument(static_cast<const void*>(nullptr))
{
}

//==============================================================================
void logMessage(
    LogLevel level,
    const std::string& format,
    const LogArgument* arguments,
    std::size_t count)
{
  if (level < LogLevel::Trace || level >= LogLevel::Off
      || (level < getLogLevel() && !hasLogCallback()))
    return;

  std::string message;
  try {
    message = formatMessage(format, arguments, count);
  } catch (const std::invalid_argument& error) {
    message = "[log format error: " + std::string(error.what()) + "] " + format;
  }

  if (forwardToLogCallback(level, message.data(), message.size())) {
    // With no GUI capture this path never initializes or locks sink state.
    auto* observer = logObserver.load(std::memory_order_acquire);
    if (observer && level >= getLogLevel()
        && observer->active.load(std::memory_order_acquire)) {
      try {
        if (auto sink = observer->sink.lock())
          (*sink)(level, message);
#if defined(__GLIBCXX__)
      } catch (const abi::__forced_unwind&) {
        throw;
#endif
      } catch (...) {
        // Observation failure cannot undo successful callback consumption.
      }
    }
    return;
  }
  if (level < getLogLevel())
    return;

  dispatchLog(level, message, nullptr);
}

} // namespace detail
} // namespace dart::common

#undef DART_LOGGING_LEGACY_FLOAT
