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

#include "dart/common/string.hpp"

#include <algorithm>

#include <fmt/format.h>

#include "dart/common/logging.hpp"

namespace dart::common {

//==============================================================================
std::string to_upper(std::string str)
{
  to_upper_in_place(str);
  return str;
}

//==============================================================================
void to_upper_in_place(std::string& str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
}

//==============================================================================
std::string to_lower(std::string str)
{
  to_lower_in_place(str);
  return str;
}

//==============================================================================
void to_lower_in_place(std::string& str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
}

//==============================================================================
std::string trim(const std::string& str, const std::string& whitespaces)
{
  return trim_right(trim_left(str, whitespaces), whitespaces);
}

//==============================================================================
std::string trim_left(const std::string& str, const std::string& whitespaces)
{
  size_t startpos = str.find_first_not_of(whitespaces);
  return (startpos == std::string::npos) ? "" : str.substr(startpos);
}

//==============================================================================
std::string trim_right(const std::string& str, const std::string& whitespaces)
{
  size_t endpos = str.find_last_not_of(whitespaces);
  return (endpos == std::string::npos) ? "" : str.substr(0, endpos + 1);
}

//==============================================================================
std::vector<std::string> split(
    const std::string& str, const std::string& delims)
{
  std::vector<std::string> tokens;
  std::size_t start = str.find_first_not_of(delims), end = 0;

  while ((end = str.find_first_of(delims, start)) != std::string::npos) {
    tokens.push_back(str.substr(start, end - start));
    start = str.find_first_not_of(delims, end);
  }
  if (start != std::string::npos) {
    tokens.push_back(str.substr(start));
  }

  return tokens;
}

//==============================================================================
std::string get_extension(const std::string& filepath)
{
  return filepath.substr(filepath.rfind(".") + 1, filepath.size());
}

//==============================================================================
std::string to_string(bool v)
{
  return fmt::format("{}", v);
}

//==============================================================================
std::string to_string(char v)
{
  return fmt::format("{}", v);
}

//==============================================================================
std::string to_string(int v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(long v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(long long v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(unsigned v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(unsigned long v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(unsigned long long v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(float v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(double v)
{
  return std::to_string(v);
}

//==============================================================================
std::string to_string(long double v)
{
  return std::to_string(v);
}

//==============================================================================
bool to_bool(const std::string& str)
{
  if (to_upper(str) == "TRUE" || str == "1") {
    return true;
  } else if (to_upper(str) == "FALSE" || str == "0") {
    return false;
  } else {
    DART_ERROR("");
    return false;
  }
}

//==============================================================================
char to_char(const std::string& str)
{
  if (str.empty()) {
    DART_ERROR("");
    return 0;
  }

  if (str.size() != 1) {
    DART_ERROR("");
  }

  return str[0];
}

//==============================================================================
int to_int(const std::string& str)
{
  return std::stoi(str);
}

//==============================================================================
unsigned int to_uint(const std::string& str)
{
  return static_cast<unsigned int>(std::stoul(str));
}

//==============================================================================
long to_long(const std::string& str)
{
  return std::stol(str);
}

//==============================================================================
long long to_long_long(const std::string& str)
{
  return std::stoll(str);
}

//==============================================================================
float to_float(const std::string& str)
{
  return std::stof(str);
}

//==============================================================================
double to_double(const std::string& str)
{
  return std::stod(str);
}

} // namespace dart::common
