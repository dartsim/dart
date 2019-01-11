/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_CONSOLE_HPP_
#define DART_COMMON_CONSOLE_HPP_

#include <string>
#include <ostream>

/// \brief Output a message
#define dtmsg (::dart::common::colorMsg("Msg", 32))

/// \brief Output a debug message
#define dtdbg (::dart::common::colorMsg("Dbg", 36))

/// \brief Output a warning message
#define dtwarn (::dart::common::colorErr("Warning", __FILE__, __LINE__, 33))

/// \brief Output an error message
#define dterr (::dart::common::colorErr("Error", __FILE__, __LINE__, 31))

namespace dart {
namespace common {

/// \brief
std::ostream& colorMsg(const std::string& _msg, int _color);

/// \brief
std::ostream& colorErr(const std::string& _msg,
                       const std::string& _file,
                       unsigned int _line,
                       int _color);


}  // namespace common

template <class T>
auto operator<<(std::ostream& os, const T& t) -> decltype(t.print(os), os)
{
  t.print(os);
  return os;
}

}  // namespace dart

#endif  // DART_COMMON_CONSOLE_HPP_
