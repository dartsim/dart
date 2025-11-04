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

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

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
  return static_cast<spdlog::level::level_enum>(static_cast<int>(level));
}

/// Convert spdlog level to LogLevel
inline LogLevel fromSpdlogLevel(spdlog::level::level_enum level)
{
  return static_cast<LogLevel>(static_cast<int>(level));
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

} // namespace dart7::common

// Logging macros with source location (relative paths)
#define DART7_TRACE(...)                                                       \
  SPDLOG_LOGGER_TRACE(                                                         \
      ::dart7::common::Logger::instance(),                                     \
      "[{}:{}] " __VA_ARGS__,                                                  \
      ::dart7::common::stripSourceDir(__FILE__),                               \
      __LINE__)

#define DART7_DEBUG(...)                                                       \
  SPDLOG_LOGGER_DEBUG(                                                         \
      ::dart7::common::Logger::instance(),                                     \
      "[{}:{}] " __VA_ARGS__,                                                  \
      ::dart7::common::stripSourceDir(__FILE__),                               \
      __LINE__)

#define DART7_INFO(...)                                                        \
  SPDLOG_LOGGER_INFO(::dart7::common::Logger::instance(), __VA_ARGS__)

#define DART7_WARN(...)                                                        \
  SPDLOG_LOGGER_WARN(                                                          \
      ::dart7::common::Logger::instance(),                                     \
      "[{}:{}] " __VA_ARGS__,                                                  \
      ::dart7::common::stripSourceDir(__FILE__),                               \
      __LINE__)

#define DART7_ERROR(...)                                                       \
  SPDLOG_LOGGER_ERROR(                                                         \
      ::dart7::common::Logger::instance(),                                     \
      "[{}:{}] " __VA_ARGS__,                                                  \
      ::dart7::common::stripSourceDir(__FILE__),                               \
      __LINE__)

#define DART7_CRITICAL(...)                                                    \
  SPDLOG_LOGGER_CRITICAL(                                                      \
      ::dart7::common::Logger::instance(),                                     \
      "[{}:{}] " __VA_ARGS__,                                                  \
      ::dart7::common::stripSourceDir(__FILE__),                               \
      __LINE__)

// Conditional logging
#define DART7_INFO_IF(condition, ...)                                          \
  if (condition) {                                                             \
    DART7_INFO(__VA_ARGS__);                                                   \
  }

#define DART7_WARN_IF(condition, ...)                                          \
  if (condition) {                                                             \
    DART7_WARN(__VA_ARGS__);                                                   \
  }

#define DART7_ERROR_IF(condition, ...)                                         \
  if (condition) {                                                             \
    DART7_ERROR(__VA_ARGS__);                                                  \
  }

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
