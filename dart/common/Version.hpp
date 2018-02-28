/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#ifndef DART_COMMON_VERSION_HPP_
#define DART_COMMON_VERSION_HPP_

#include <string>

namespace dart {
namespace common {

class Version
{
public:
  /// Constructs from version numbers
  Version(
      unsigned short major = 0u,
      unsigned short minor = 0u,
      unsigned short patch = 0u);

  /// Constructs from version string (e.g., 5.1.4)
  explicit Version(const std::string& version);

  /// Sets the major version number
  void setMajor(unsigned short major);

  /// Sets the minor version number
  void setMinor(unsigned short minor);

  /// Sets the patch version number
  void setPatch(unsigned short patch);

  /// Sets the major, minor, and patch version numbers
  void fromString(
      unsigned short major, unsigned short minor, unsigned short patch);

  /// Returns the major version number
  unsigned short getMajor() const;

  /// Returns the minor version number
  unsigned short getMinor() const;

  /// Returns the patch version number
  unsigned short getPatch() const;

  /// Sets the version numbers from string
  void fromString(const std::string& version);

  /// Returns the version as string (e.g., 5.1.4)
  std::string toString() const;

  /// Equality operator
  bool operator==(const Version& other);

  /// Inequality operator
  bool operator!=(const Version& other);

  /// Comparison operator
  bool operator<(const Version& other);

  /// Comparison operator
  bool operator>(const Version& other);

  /// Comparison operator
  bool operator<=(const Version& other);

  /// Comparison operator
  bool operator>=(const Version& other);

private:
  unsigned short mMajor;
  unsigned short mMinor;
  unsigned short mPatch;
};

std::ostream& operator<<(std::ostream& os, const Version& version);

std::istream& operator>>(std::istream& is, Version& version);

} // namespace common
} // namespace dart

#endif // DART_COMMON_VERSION_HPP_
