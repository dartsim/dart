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

namespace dart7::common {

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
#ifdef DART7_SOURCE_DIR
  const char* base = DART7_SOURCE_DIR;
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

  /// Set log level using DART7 LogLevel enum
  static void setLevel(LogLevel level)
  {
    instance()->set_level(toSpdlogLevel(level));
  }

  /// Get current log level as spdlog level
  static spdlog::level::level_enum getLevel()
  {
    return instance()->level();
  }

  /// Get current log level as DART7 LogLevel enum
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

/// Initialize DART7 logging with default settings
inline void initializeLogging()
{
  // Check if logger already exists to make this function idempotent
  auto existing_logger = spdlog::get("dart7");
  if (existing_logger) {
    Logger::setLogger(existing_logger);
    return;
  }

  auto logger = spdlog::stdout_color_mt("dart7");
  logger->set_level(spdlog::level::info);
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  Logger::setLogger(logger);
#ifndef NDEBUG
  detail::log(
      LogLevel::Info,
      __FILE__,
      __LINE__,
      __func__,
      "DART7 logging initialized");
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

} // namespace dart7::common

// Logging macros with optional source context
#define DART7_TRACE(...)                                                       \
  ::dart7::common::detail::log(                                                \
      ::dart7::common::LogLevel::Trace,                                        \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART7_DEBUG(...)                                                       \
  ::dart7::common::detail::log(                                                \
      ::dart7::common::LogLevel::Debug,                                        \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART7_INFO(...)                                                        \
  ::dart7::common::detail::log(                                                \
      ::dart7::common::LogLevel::Info,                                         \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART7_WARN(...)                                                        \
  ::dart7::common::detail::log(                                                \
      ::dart7::common::LogLevel::Warn,                                         \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART7_ERROR(...)                                                       \
  ::dart7::common::detail::log(                                                \
      ::dart7::common::LogLevel::Error,                                        \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

#define DART7_CRITICAL(...)                                                    \
  ::dart7::common::detail::log(                                                \
      ::dart7::common::LogLevel::Critical,                                     \
      __FILE__,                                                                \
      __LINE__,                                                                \
      __func__,                                                                \
      __VA_ARGS__)

// Conditional logging
#define DART7_INFO_IF(condition, ...)                                          \
  do {                                                                         \
    if (condition) {                                                           \
      DART7_INFO(__VA_ARGS__);                                                 \
    }                                                                          \
  } while (false)

#define DART7_WARN_IF(condition, ...)                                          \
  do {                                                                         \
    if (condition) {                                                           \
      DART7_WARN(__VA_ARGS__);                                                 \
    }                                                                          \
  } while (false)

#define DART7_ERROR_IF(condition, ...)                                         \
  do {                                                                         \
    if (condition) {                                                           \
      DART7_ERROR(__VA_ARGS__);                                                \
    }                                                                          \
  } while (false)

// Log once (useful for warnings in loops)
#define DART7_WARN_ONCE(...)                                                   \
  do {                                                                         \
    static bool _dart_logged_once = false;                                     \
    if (!_dart_logged_once) {                                                  \
      DART7_WARN(__VA_ARGS__);                                                 \
      _dart_logged_once = true;                                                \
    }                                                                          \
  } while (false)

#define DART7_ERROR_ONCE(...)                                                  \
  do {                                                                         \
    static bool _dart_logged_once = false;                                     \
    if (!_dart_logged_once) {                                                  \
      DART7_ERROR(__VA_ARGS__);                                                \
      _dart_logged_once = true;                                                \
    }                                                                          \
  } while (false)
