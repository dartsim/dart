/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/common/string.hpp"

#include <algorithm>

namespace dart::common {

//==============================================================================
std::string toUpper(std::string str)
{
  toUpperInPlace(str);
  return str;
}

//==============================================================================
void toUpperInPlace(std::string& str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
}

//==============================================================================
std::string toLower(std::string str)
{
  toLowerInPlace(str);
  return str;
}

//==============================================================================
void toLowerInPlace(std::string& str)
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
}

//==============================================================================
std::string trim(std::string_view str, std::string_view whitespaces)
{
  const auto leftTrimmed = trimLeft(str, whitespaces);
  return trimRight(leftTrimmed, whitespaces);
}

//==============================================================================
std::string trimLeft(std::string_view str, std::string_view whitespaces)
{
  const auto startpos = str.find_first_not_of(whitespaces);
  if (startpos == std::string_view::npos)
    return std::string();
  return std::string(str.substr(startpos));
}

//==============================================================================
std::string trimRight(std::string_view str, std::string_view whitespaces)
{
  const auto endpos = str.find_last_not_of(whitespaces);
  if (endpos == std::string_view::npos)
    return std::string();
  return std::string(str.substr(0, endpos + 1));
}

//==============================================================================
std::vector<std::string> split(
    std::string_view str, std::string_view delimiters)
{
  std::vector<std::string> tokens;
  std::size_t start = str.find_first_not_of(delimiters), end = 0;

  while ((end = str.find_first_of(delimiters, start))
         != std::string_view::npos) {
    tokens.emplace_back(str.substr(start, end - start));
    start = str.find_first_not_of(delimiters, end);
  }

  if (start != std::string_view::npos) {
    tokens.emplace_back(str.substr(start));
  }

  return tokens;
}

} // namespace dart::common
