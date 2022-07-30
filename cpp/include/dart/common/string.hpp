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

#include <string>
#include <vector>

#include "dart/common/export.hpp"

namespace dart::common {

DART_COMMON_API std::string to_upper(std::string str);
DART_COMMON_API void to_upper_in_place(std::string& str);
DART_COMMON_API std::string to_lower(std::string str);
DART_COMMON_API void to_lower_in_place(std::string& str);

DART_COMMON_API std::string trim(
    const std::string& s, const std::string& whitespaces = " \n\r\t");

DART_COMMON_API std::string trim_left(
    const std::string& s, const std::string& whitespaces = " \n\r\t");

DART_COMMON_API std::string trim_right(
    const std::string& s, const std::string& whitespaces = " \n\r\t");

DART_COMMON_API std::vector<std::string> split(
    const std::string& s, const std::string& delims = " ");

/// Returns the extension given filepath
DART_COMMON_API std::string get_extension(const std::string& filepath);

DART_COMMON_API std::string to_string(bool v);
DART_COMMON_API std::string to_string(char v);
DART_COMMON_API std::string to_string(int v);
DART_COMMON_API std::string to_string(long v);
DART_COMMON_API std::string to_string(long long v);
DART_COMMON_API std::string to_string(unsigned v);
DART_COMMON_API std::string to_string(unsigned long v);
DART_COMMON_API std::string to_string(unsigned long long v);
DART_COMMON_API std::string to_string(float v);
DART_COMMON_API std::string to_string(double v);
DART_COMMON_API std::string to_string(long double v);

DART_COMMON_API bool to_bool(const std::string& str);
DART_COMMON_API char to_char(const std::string& str);
DART_COMMON_API int to_int(const std::string& str);
DART_COMMON_API unsigned int to_uint(const std::string& str);
DART_COMMON_API long to_long(const std::string& str);
DART_COMMON_API long long to_long_long(const std::string& str);
DART_COMMON_API float to_float(const std::string& str);
DART_COMMON_API double to_double(const std::string& str);

template <typename S>
S to_scalar(const std::string& str);

DART_COMMON_API std::string strerror(int error_number);

} // namespace dart::common

#include "dart/common/detail/string_impl.hpp"
