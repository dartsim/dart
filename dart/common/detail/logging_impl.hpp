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

#pragma once

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"

#if DART_HAVE_spdlog
  #include <spdlog/spdlog.h>
#else
  #include "dart/common/Console.hpp"
#endif

namespace dart {
namespace common {

//========================================================================================
template <typename S, typename... Args>
void trace(const S& format_str, [[maybe_unused]] Args&&... args) {
#if DART_HAVE_spdlog
  spdlog::trace(format_str, std::forward<Args>(args)...);
#else
  colorMsg("[trace]", 37) << format_str << "\n";
#endif
}

//========================================================================================
template <typename S, typename... Args>
void debug(const S& format_str, [[maybe_unused]] Args&&... args) {
#if DART_HAVE_spdlog
  spdlog::debug(format_str, std::forward<Args>(args)...);
#else
  colorMsg("[debug]", 36) << format_str << "\n";
#endif
}

//========================================================================================
template <typename S, typename... Args>
void info(const S& format_str, [[maybe_unused]] Args&&... args) {
#if DART_HAVE_spdlog
  spdlog::info(format_str, std::forward<Args>(args)...);
#else
  colorMsg("[info]", 32) << format_str << "\n";
#endif
}

//========================================================================================
template <typename S, typename... Args>
void warn(const S& format_str, [[maybe_unused]] Args&&... args) {
#if DART_HAVE_spdlog
  spdlog::warn(format_str, std::forward<Args>(args)...);
#else
  colorMsg("[warn]", 33) << format_str << "\n";
#endif
}

//========================================================================================
template <typename S, typename... Args>
void error(const S& format_str, [[maybe_unused]] Args&&... args) {
#if DART_HAVE_spdlog
  spdlog::error(format_str, std::forward<Args>(args)...);
#else
  colorMsg("[error]", 31) << format_str << "\n";
#endif
}

//========================================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, [[maybe_unused]] Args&&... args) {
#if DART_HAVE_spdlog
  spdlog::critical(format_str, std::forward<Args>(args)...);
#else
  colorMsg("[fatal]", 30) << format_str << "\n";
#endif
}

} // namespace common
} // namespace dart
