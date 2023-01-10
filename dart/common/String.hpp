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

#ifndef DART_COMMON_STRING_HPP_
#define DART_COMMON_STRING_HPP_

#include <dart/common/Export.hpp>

#include <string>
#include <vector>

namespace dart::common {

/// Converts string to upper cases
DART_COMMON_API std::string toUpper(std::string str);

/// Converts string to upper cases in place
DART_COMMON_API void toUpperInPlace(std::string& str);

/// Converts string to lower cases
DART_COMMON_API std::string toLower(std::string str);

/// Converts string to lower cases in place
DART_COMMON_API void toLowerInPlace(std::string& str);

/// Trims both sides of string
DART_COMMON_API std::string trim(
    const std::string& str, const std::string& whitespaces = " \n\r\t");

/// Trims left side of string
DART_COMMON_API std::string trimLeft(
    const std::string& str, const std::string& whitespaces = " \n\r\t");

/// Trims right side of string
DART_COMMON_API std::string trimRight(
    const std::string& str, const std::string& whitespaces = " \n\r\t");

/// Splits string given delimiters
DART_COMMON_API std::vector<std::string> split(
    const std::string& str, const std::string& delimiters = " \n\r\t");

} // namespace dart::common

#endif // DART_COMMON_STRING_HPP_
