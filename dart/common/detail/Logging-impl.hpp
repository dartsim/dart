/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
#define DART_COMMON_DETAIL_LOGGING_IMPL_HPP_

#include <dart/common/Logging.hpp>

#if DART_HAVE_SPDLOG
  #include <spdlog/spdlog.h>
#else
  #include <iostream>
  #include <string>
#endif

namespace dart::common {

#if !DART_HAVE_SPDLOG
namespace detail {

//==============================================================================
template <typename S1, typename S2>
void print(std::ostream& os, const S1& header, const S2& format_str, int color)
{
  os << "\033[1;" << color << "m" << header << "\033[0m " << format_str
     << std::endl;
}

//==============================================================================
template <typename S, typename Arg, typename... Args>
void print(
    std::ostream& os,
    const S& header,
    std::string format_str,
    int color,
    Arg&& arg,
    Args&&... args)
{
  os << "\033[1;" << color << "m" << header << "\033[0m " << format_str
     << " [args]: ";
  os << std::forward<Arg>(arg);
  ((os << ", " << std::forward<Args>(args)), ...);
  os << std::endl;
}

} // namespace detail
#endif

//==============================================================================
template <typename S, typename... Args>
void trace(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::trace(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[trace]", format_str, 38, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void debug(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::debug(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[debug]", format_str, 36, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void info(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::info(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cout, "[info]", format_str, 32, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void warn(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::warn(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[warn]", format_str, 33, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void error(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::error(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[error]", format_str, 31, std::forward<Args>(args)...);
#endif
}

//==============================================================================
template <typename S, typename... Args>
void fatal(const S& format_str, [[maybe_unused]] Args&&... args)
{
#if DART_HAVE_SPDLOG
  spdlog::critical(format_str, std::forward<Args>(args)...);
#else
  detail::print(
      std::cerr, "[fatal]", format_str, 35, std::forward<Args>(args)...);
#endif
}

} // namespace dart::common

#endif // DART_COMMON_DETAIL_LOGGING_IMPL_HPP_
