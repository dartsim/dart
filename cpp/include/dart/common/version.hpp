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

#include <string>
#include <vector>

#define DART_MAJOR_VERSION 8
#define DART_MINOR_VERSION 0
#define DART_PATCH_VERSION 0

#define DART_VERSION_STRING "8.0.0"

namespace dart::common {

class Version
{
public:
  static constexpr const char* AsConstChar();
  static const std::string& AsString();

  static constexpr int GetMajor();
  static constexpr int GetMinor();
  static constexpr int GetPatch();
};

//==============================================================================
inline constexpr const char* Version::AsConstChar()
{
  return DART_VERSION_STRING;
}

//==============================================================================
inline const std::string& Version::AsString()
{
  static const std::string version(AsConstChar());
  return version;
}

//==============================================================================
inline constexpr int Version::GetMajor()
{
  return DART_MAJOR_VERSION;
}

//==============================================================================
inline constexpr int Version::GetMinor()
{
  return DART_MINOR_VERSION;
}

//==============================================================================
inline constexpr int Version::GetPatch()
{
  return DART_PATCH_VERSION;
}

} // namespace dart::common
