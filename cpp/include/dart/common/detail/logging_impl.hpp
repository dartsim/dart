/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#if !(DART_HAVE_spdlog)
  #include <fmt/format.h>
#endif

#include "dart/common/logging.hpp"

namespace dart::common {

//========================================================================================
template <typename S, typename... Args>
void trace(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  spdlog::trace(format_str, std::forward<Args>(args)...);
#else
  fmt::print("[trace] " + std::string(format_str), std::forward<Args>(args)...);
#endif
}

//========================================================================================
template <typename S, typename... Args>
void debug(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  spdlog::debug(format_str, std::forward<Args>(args)...);
#else
  fmt::print("[debug] " + std::string(format_str), std::forward<Args>(args)...);
#endif
}

//========================================================================================
template <typename S, typename... Args>
void info(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  spdlog::info(format_str, std::forward<Args>(args)...);
#else
  fmt::print("[info] " + std::string(format_str), std::forward<Args>(args)...);
#endif
}

//========================================================================================
template <typename S, typename... Args>
void warn(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  spdlog::warn(format_str, std::forward<Args>(args)...);
#else
  fmt::print("[warn] " + std::string(format_str), std::forward<Args>(args)...);
#endif
}

//========================================================================================
template <typename S, typename... Args>
void error(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  spdlog::error(format_str, std::forward<Args>(args)...);
#else
  fmt::print("[error] " + std::string(format_str), std::forward<Args>(args)...);
#endif
}

//========================================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_spdlog
  spdlog::critical(format_str, std::forward<Args>(args)...);
#else
  fmt::print("[fatal] " + std::string(format_str), std::forward<Args>(args)...);
#endif
}

} // namespace dart::common
