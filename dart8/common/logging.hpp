/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#pragma once

#include <fmt/format.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <memory>
#include <string>

#define DART8_DETAIL_PP_CONCAT_IMPL(a, b) a##b
#define DART8_DETAIL_PP_CONCAT(a, b) DART8_DETAIL_PP_CONCAT_IMPL(a, b)
#if defined(__COUNTER__)
  #define DART8_DETAIL_UNIQUE_NAME(prefix)                                     \
    DART8_DETAIL_PP_CONCAT(prefix, __COUNTER__)
#else
  #define DART8_DETAIL_UNIQUE_NAME(prefix)                                     \
    DART8_DETAIL_PP_CONCAT(prefix, __LINE__)
#endif

#define DART8_DETAIL_LOG_ONCE_IMPL(flag, log_call)                             \
  do {                                                                         \
    static bool flag = false;                                                  \
    if (!flag) {                                                               \
      flag = true;                                                             \
      log_call;                                                                \
    }                                                                          \
  } while (false)

#define DART8_DETAIL_LOG_ONCE(log_call)                                        \
  DART8_DETAIL_LOG_ONCE_IMPL(                                                  \
      DART8_DETAIL_UNIQUE_NAME(_dart8_log_once_flag_), log_call)

#define DART8_DETAIL_LOG_ONCE_IF_IMPL(flag, condition_expr, log_call)          \
  do {                                                                         \
    static bool flag = false;                                                  \
    if (!flag) {                                                               \
      if (condition_expr) {                                                    \
        flag = true;                                                           \
        log_call;                                                              \
      }                                                                        \
    }                                                                          \
  } while (false)

#define DART8_DETAIL_LOG_ONCE_IF(condition_expr, log_call)                     \
  DART8_DETAIL_LOG_ONCE_IF_IMPL(                                               \
      DART8_DETAIL_UNIQUE_NAME(_dart8_log_once_flag_),                         \
      condition_expr,                                                          \
      log_call)

namespace dart8::common {

/// Log level enum for easier Python/user API
enum class LogLevel
{
  Trace = 0,
  Debug = 1,
  Info = 2,
  Warn = 3,
  Error = 4,
  Critical = 5,
  Off = 6
};

/// Convert LogLevel to spdlog level
inline spdlog::level::level_enum toSpdlogLevel(LogLevel level)
{
  switch (level) {
    case LogLevel::Trace:
      return spdlog::level::trace;
    case LogLevel::Debug:
      return spdlog::level::debug;
    case LogLevel::Info:
      return spdlog::level::info;
    case LogLevel::Warn:
      return spdlog::level::warn;
    case LogLevel::Error:
      return spdlog::level::err;
    case LogLevel::Critical:
      return spdlog::level::critical;
    case LogLevel::Off:
      return spdlog::level::off;
  }
  return spdlog::level::info;
}

/// Convert spdlog level to LogLevel
inline LogLevel fromSpdlogLevel(spdlog::level::level_enum level)
{
  switch (level) {
    case spdlog::level::trace:
      return LogLevel::Trace;
    case spdlog::level::debug:
      return LogLevel::Debug;
    case spdlog::level::info:
      return LogLevel::Info;
    case spdlog::level::warn:
      return LogLevel::Warn;
    case spdlog::level::err:
      return LogLevel::Error;
    case spdlog::level::critical:
      return LogLevel::Critical;
    case spdlog::level::off:
      return LogLevel::Off;
    default:
      return LogLevel::Info;
  }
}

/// Strip source directory prefix from file path at compile time
constexpr const char* stripSourceDir(const char* path)
{
#ifdef DART8_SOURCE_DIR
  const char* base = DART8_SOURCE_DIR;
  const char* p = path;
  const char* b = base;

  // Find where path and base diverge
  while (*p && *b && *p == *b) {
    ++p;
    ++b;
  }

  // If we matched the entire base, skip the trailing slash
  if (*b == '\0' && *p == '/') {
    return p + 1;
  }
#else
  (void)path;
  return "";
#endif
  return path;
}

/// Logging system using spdlog
class Logger
{
public:
  /// Get the global logger instance
  static std::shared_ptr<spdlog::logger>& instance()
  {
    static auto logger = spdlog::default_logger();
    return logger;
  }

  /// Set the global logger
  static void setLogger(std::shared_ptr<spdlog::logger> logger)
  {
    instance() = logger;
  }

  /// Set log level
  static void setLevel(spdlog::level::level_enum level)
  {
    instance()->set_level(level);
  }

  /// Set log level using DART8 LogLevel enum
  static void setLevel(LogLevel level)
  {
    instance()->set_level(toSpdlogLevel(level));
  }

  /// Get current log level as spdlog level
  static spdlog::level::level_enum getLevel()
  {
    return instance()->level();
  }

  /// Get current log level as DART8 LogLevel enum
  static LogLevel getLevelEnum()
  {
    return fromSpdlogLevel(instance()->level());
  }

  /// Set log pattern
  static void setPattern(const std::string& pattern)
  {
    instance()->set_pattern(pattern);
  }
};

namespace detail {

inline std::string makeContext(const char* file, int line, const char* function)
{
#ifndef NDEBUG
  std::string prefix;
  bool has_component = false;
  if (file != nullptr && *file != '\0') {
    prefix += stripSourceDir(file);
    has_component = true;
  }
  if (line > 0) {
    if (has_component)
      prefix += ':';
    prefix += std::to_string(line);
    has_component = true;
  }
  if (function != nullptr && *function != '\0') {
    if (has_component)
      prefix += "::";
    prefix += function;
    has_component = true;
  }
  if (has_component) {
    prefix.insert(prefix.begin(), '[');
    prefix.append("] ");
    return prefix;
  }
#else
  (void)file;
  (void)line;
  (void)function;
#endif
  return {};
}

template <typename Format, typename... Args>
void log(
    LogLevel level,
    const char* file,
    int line,
    const char* function,
    Format&& format,
    Args&&... args)
{
  auto& logger = Logger::instance();
#ifndef NDEBUG
  std::string prefix = makeContext(file, line, function);
  if (!prefix.empty()) {
    auto message = fmt::format(
        std::forward<Format>(format), std::forward<Args>(args)...);
    logger->log(toSpdlogLevel(level), "{}{}", prefix, message);
    return;
  }
#else
  (void)file;
  (void)line;
  (void)function;
#endif
  logger->log(
      toSpdlogLevel(level),
      std::forward<Format>(format),
      std::forward<Args>(args)...);
}

} // namespace detail

/// Initialize DART8 logging with default settings
inline void initializeLogging()
{
  // Check if logger already exists to make this function idempotent
  auto existing_logger = spdlog::get("dart8");
  if (existing_logger) {
    Logger::setLogger(existing_logger);
    return;
  }

  auto logger = spdlog::stdout_color_mt("dart8");
  logger->set_level(spdlog::level::info);
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  Logger::setLogger(logger);
#ifndef NDEBUG
  detail::log(
      LogLevel::Info,
      __FILE__,
      __LINE__,
      __func__,
      "DART8 logging initialized");
#endif
}

/// Set log level (convenience function)
inline void setLogLevel(LogLevel level)
{
  Logger::setLevel(level);
}

/// Get log level (convenience function)
inline LogLevel getLogLevel()
{
  return Logger::getLevelEnum();
}

} // namespace dart8::common

// Logging macros with optional source context
#define DART8_TRACE(...)                                                       \
  ::dart8::common::detail::log(                                                \
      ::dart8::common::LogLevel::Trace,                                        \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART8_DEBUG(...)                                                       \
  ::dart8::common::detail::log(                                                \
      ::dart8::common::LogLevel::Debug,                                        \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART8_INFO(...)                                                        \
  ::dart8::common::detail::log(                                                \
      ::dart8::common::LogLevel::Info,                                         \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART8_WARN(...)                                                        \
  ::dart8::common::detail::log(                                                \
      ::dart8::common::LogLevel::Warn,                                         \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART8_ERROR(...)                                                       \
  ::dart8::common::detail::log(                                                \
      ::dart8::common::LogLevel::Error,                                        \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART8_CRITICAL(...)                                                    \
  ::dart8::common::detail::log(                                                \
      ::dart8::common::LogLevel::Critical,                                     \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

// Conditional logging
#define DART8_INFO_IF(condition, ...)                                          \
  do {                                                                         \
    if (condition) {                                                           \
      DART8_INFO(__VA_ARGS__);                                                 \
    }                                                                          \
  } while (false)

#define DART8_WARN_IF(condition, ...)                                          \
  do {                                                                         \
    if (condition) {                                                           \
      DART8_WARN(__VA_ARGS__);                                                 \
    }                                                                          \
  } while (false)

#define DART8_ERROR_IF(condition, ...)                                         \
  do {                                                                         \
    if (condition) {                                                           \
      DART8_ERROR(__VA_ARGS__);                                                \
    }                                                                          \
  } while (false)

// Log once helpers
#define DART8_TRACE_ONCE(...) DART8_DETAIL_LOG_ONCE(DART8_TRACE(__VA_ARGS__))

#define DART8_TRACE_ONCE_IF(condition, ...)                                    \
  DART8_DETAIL_LOG_ONCE_IF(condition, DART8_TRACE(__VA_ARGS__))

#define DART8_DEBUG_ONCE(...) DART8_DETAIL_LOG_ONCE(DART8_DEBUG(__VA_ARGS__))

#define DART8_DEBUG_ONCE_IF(condition, ...)                                    \
  DART8_DETAIL_LOG_ONCE_IF(condition, DART8_DEBUG(__VA_ARGS__))

#define DART8_INFO_ONCE(...) DART8_DETAIL_LOG_ONCE(DART8_INFO(__VA_ARGS__))

#define DART8_INFO_ONCE_IF(condition, ...)                                     \
  DART8_DETAIL_LOG_ONCE_IF(condition, DART8_INFO(__VA_ARGS__))

#define DART8_WARN_ONCE(...) DART8_DETAIL_LOG_ONCE(DART8_WARN(__VA_ARGS__))

#define DART8_WARN_ONCE_IF(condition, ...)                                     \
  DART8_DETAIL_LOG_ONCE_IF(condition, DART8_WARN(__VA_ARGS__))

#define DART8_ERROR_ONCE(...) DART8_DETAIL_LOG_ONCE(DART8_ERROR(__VA_ARGS__))

#define DART8_ERROR_ONCE_IF(condition, ...)                                    \
  DART8_DETAIL_LOG_ONCE_IF(condition, DART8_ERROR(__VA_ARGS__))

#define DART8_CRITICAL_ONCE(...)                                               \
  DART8_DETAIL_LOG_ONCE(DART8_CRITICAL(__VA_ARGS__))

#define DART8_CRITICAL_ONCE_IF(condition, ...)                                 \
  DART8_DETAIL_LOG_ONCE_IF(condition, DART8_CRITICAL(__VA_ARGS__))
