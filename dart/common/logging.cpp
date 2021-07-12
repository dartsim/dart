/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/common/logging.hpp"

#include <iostream>

#if DART_HAVE_spdlog
  #include <spdlog/spdlog.h>
#else
  #include "dart/common/Console.hpp"
#endif

namespace dart {
namespace common {

//==============================================================================
void set_log_level(LogLevel level) {
#if DART_HAVE_spdlog
  switch (level) {
    case LogLevel::LL_TRACE:
  #if DART_ACTIVE_LOG_LEVEL > DART_LOG_LEVEL_TRACE
      std::cout
          << "[warning] Log level is set to TRACE, but trace "
          << "logs will still not be logged because the active log level is "
          << "higher than TRACE.\n";
  #endif
      spdlog::set_level(spdlog::level::trace);
      break;
    case LogLevel::LL_DEBUG:
  #if DART_ACTIVE_LOG_LEVEL > DART_LOG_LEVEL_DEBUG
      std::cout
          << "[warning] Log level is set to DEBUG, but debug or lower level "
          << "logs will still not be logged because the active log level is "
          << "higher than DEBUG.\n";
  #endif
      spdlog::set_level(spdlog::level::debug);
      break;
    case LogLevel::LL_INFO:
  #if DART_ACTIVE_LOG_LEVEL > DART_LOG_LEVEL_INFO
      std::cout
          << "[warning] Log level is set to INFO, but info or lower level "
          << "logs will still not be logged because the active log level is "
          << "higher than INFO.\n";
  #endif
      spdlog::set_level(spdlog::level::info);
      break;
    case LogLevel::LL_WARN:
  #if DART_ACTIVE_LOG_LEVEL > DART_LOG_LEVEL_WARN
      std::cout
          << "[warning] Log level is set to WARN, but warn or lower level "
          << "logs will still not be logged because the active log level is "
          << "higher than WARN.\n";
  #endif
      spdlog::set_level(spdlog::level::warn);
      break;
    case LogLevel::LL_ERROR:
  #if DART_ACTIVE_LOG_LEVEL > DART_LOG_LEVEL_ERROR
      std::cout
          << "[warning] Log level is set to ERROR, but error or lower level "
          << "logs will still not be logged because the active log level is "
          << "higher than ERROR.\n";
  #endif
      spdlog::set_level(spdlog::level::err);
      break;
    case LogLevel::LL_FATAL:
  #if DART_ACTIVE_LOG_LEVEL > DART_LOG_LEVEL_FATAL
      std::cout
          << "[warning] Log level is set to FATAL, but fatal "
          << "logs will still not be logged because the active log level is "
          << "set to OFF.\n";
  #endif
      spdlog::set_level(spdlog::level::critical);
      break;
    case LogLevel::LL_OFF:
      spdlog::set_level(spdlog::level::off);
      break;
    default:
      break;
  }
#else
  DART_UNUSED(level);
  DART_WARN(
      "Setting log level is not supported when DART isn't built with spdlog.");
#endif
}

} // namespace common
} // namespace dart
