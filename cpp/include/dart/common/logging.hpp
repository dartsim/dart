/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/common/export.hpp"

// clang-format off
#define DART_LOG_LEVEL_TRACE 0
#define DART_LOG_LEVEL_DEBUG 1
#define DART_LOG_LEVEL_INFO  2
#define DART_LOG_LEVEL_WARN  3
#define DART_LOG_LEVEL_ERROR 4
#define DART_LOG_LEVEL_FATAL 5
#define DART_LOG_LEVEL_OFF   6
// clang-format on

#include <iostream>

#if DART_HAVE_spdlog
  #include <spdlog/spdlog.h>
#endif

// Default active log level
#if !defined(DART_ACTIVE_LOG_LEVEL)
  #define DART_ACTIVE_LOG_LEVEL DART_LOG_LEVEL_INFO
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_TRACE
  #define DART_TRACE(...) ::dart::common::trace(__VA_ARGS__)
#else
  #define DART_TRACE(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_DEBUG
  #define DART_DEBUG(...) ::dart::common::debug(__VA_ARGS__)
#else
  #define DART_DEBUG(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_INFO
  #define DART_INFO(...) ::dart::common::info(__VA_ARGS__)
#else
  #define DART_INFO(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_WARN
  #define DART_WARN(...) ::dart::common::warn(__VA_ARGS__)
#else
  #define DART_WARN(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_ERROR
  #define DART_ERROR(...) ::dart::common::error(__VA_ARGS__)
#else
  #define DART_ERROR(...) (void)0
#endif

#if DART_ACTIVE_LOG_LEVEL <= DART_LOG_LEVEL_FATAL
  #define DART_FATAL(...) ::dart::common::fatal(__VA_ARGS__)
#else
  #define DART_FATAL(...) (void)0
#endif

namespace dart::common {

enum class LogLevel
{
  LOGLEVEL_UNKNOWN,
  LOGLEVEL_TRACE,
  LOGLEVEL_DEBUG,
  LOGLEVEL_INFO,
  LOGLEVEL_WARN,
  LOGLEVEL_ERROR,
  LOGLEVEL_FATAL,
  LOGLEVEL_OFF,
};

#if DART_HAVE_spdlog
DART_COMMON_API LogLevel convert_log_level(spdlog::level::level_enum level);
#endif

DART_COMMON_API void set_log_level(LogLevel level);

DART_COMMON_API void set_log_level_to_trace();

DART_COMMON_API void set_log_level_to_debug();

DART_COMMON_API void set_log_level_to_info();

DART_COMMON_API void set_log_level_to_warn();

DART_COMMON_API void set_log_level_to_error();

DART_COMMON_API void set_log_level_to_fatal();

#if SPDLOG_VERSION >= 10801
inline LogLevel get_log_level()
{
  return convert_log_level(spdlog::get_level());
}
#endif

template <typename S, typename... Args>
void trace(const S& format_str, Args&&... args);

template <typename S, typename... Args>
void debug(const S& format_str, Args&&... args);

template <typename S, typename... Args>
void info(const S& format_str, Args&&... args);

template <typename S, typename... Args>
void warn(const S& format_str, Args&&... args);

template <typename S, typename... Args>
void error(const S& format_str, Args&&... args);

template <typename S, typename... Args>
void fatal(const S& format_str, Args&&... args);

} // namespace dart::common

#include "dart/common/detail/logging_impl.hpp"
