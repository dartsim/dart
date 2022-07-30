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

#include "dart/common/logging.hpp"

namespace dart::common {

//========================================================================================
#if DART_HAVE_spdlog
LogLevel convert_log_level(spdlog::level::level_enum level)
{
  switch (level) {
    case spdlog::level::trace:
      return LogLevel::LOGLEVEL_TRACE;
    case spdlog::level::debug:
      return LogLevel::LOGLEVEL_DEBUG;
    case spdlog::level::info:
      return LogLevel::LOGLEVEL_INFO;
    case spdlog::level::warn:
      return LogLevel::LOGLEVEL_WARN;
    case spdlog::level::err:
      return LogLevel::LOGLEVEL_ERROR;
    case spdlog::level::critical:
      return LogLevel::LOGLEVEL_FATAL;
    case spdlog::level::off:
      return LogLevel::LOGLEVEL_OFF;
    default:
      return LogLevel::LOGLEVEL_UNKNOWN;
  }
}
#endif

void set_log_level(LogLevel level)
{
#if DART_HAVE_spdlog
  switch (level) {
    case LogLevel::LOGLEVEL_TRACE:
      spdlog::set_level(spdlog::level::trace);
      break;
    case LogLevel::LOGLEVEL_DEBUG:
      spdlog::set_level(spdlog::level::debug);
      break;
    case LogLevel::LOGLEVEL_INFO:
      spdlog::set_level(spdlog::level::info);
      break;
    case LogLevel::LOGLEVEL_WARN:
      spdlog::set_level(spdlog::level::warn);
      break;
    case LogLevel::LOGLEVEL_ERROR:
      spdlog::set_level(spdlog::level::err);
      break;
    case LogLevel::LOGLEVEL_FATAL:
      spdlog::set_level(spdlog::level::critical);
      break;
    case LogLevel::LOGLEVEL_OFF:
      spdlog::set_level(spdlog::level::off);
      break;
    default:
      std::cerr << "[ERROR] Unsupported logging level.\n";
      break;
  }
#else
  (void)level;
#endif
}

//==============================================================================
void set_log_level_to_trace()
{
  set_log_level(LogLevel::LOGLEVEL_TRACE);
}

//==============================================================================
void set_log_level_to_debug()
{
  set_log_level(LogLevel::LOGLEVEL_DEBUG);
}

//==============================================================================
void set_log_level_to_info()
{
  set_log_level(LogLevel::LOGLEVEL_INFO);
}

//==============================================================================
void set_log_level_to_warn()
{
  set_log_level(LogLevel::LOGLEVEL_WARN);
}

//==============================================================================
void set_log_level_to_error()
{
  set_log_level(LogLevel::LOGLEVEL_ERROR);
}

//==============================================================================
void set_log_level_to_fatal()
{
  set_log_level(LogLevel::LOGLEVEL_FATAL);
}

} // namespace dart::common
