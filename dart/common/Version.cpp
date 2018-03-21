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

#include "dart/common/Version.hpp"

#include <iostream>
#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include "dart/common/Console.hpp"

namespace dart {
namespace common {

//==============================================================================
Version::Version(
    unsigned short major, unsigned short minor, unsigned short patch)
  : mMajor(major), mMinor(minor), mPatch(patch)
{
  // Do nothing
}

//==============================================================================
Version::Version(const std::string& version)
{
  fromString(version);
}

//==============================================================================
unsigned short Version::getMajor() const
{
  return mMajor;
}

//==============================================================================
unsigned short Version::getMinor() const
{
  return mMinor;
}

//==============================================================================
unsigned short Version::getPatch() const
{
  return mPatch;
}

//==============================================================================
std::string Version::toString() const
{
  return std::to_string(mMajor) + "." + std::to_string(mMinor) + "."
         + std::to_string(mPatch);
}

//==============================================================================
void Version::setMajor(unsigned short major)
{
  mMajor = major;
}

//==============================================================================
void Version::setMinor(unsigned short minor)
{
  mMinor = minor;
}

//==============================================================================
void Version::setPatch(unsigned short patch)
{
  mPatch = patch;
}

//==============================================================================
void Version::set(
    unsigned short major, unsigned short minor, unsigned short patch)
{
  setMajor(major);
  setMinor(minor);
  setPatch(patch);
}

//==============================================================================
void Version::fromString(const std::string& version)
{
  unsigned short major = 0;
  unsigned short minor = 0;
  unsigned short patch = 0;

  std::vector<std::string> splitResult;
  boost::algorithm::split(
      splitResult, version, boost::algorithm::is_any_of(".,"));
  // TODO(JS): replace with non-boost version

  if (splitResult.size() == 0 || splitResult.size() > 3)
  {
    dterr << "[Version::fromString] Failed to parse a version string '"
          << version << "'. Ignoring this action.\n";
    return;
  }

  boost::algorithm::trim(splitResult[0]);
  // TODO(JS): replace with non-boost version
  major = std::stoul(splitResult[0]);

  if (splitResult.size() >= 2)
  {
    boost::algorithm::trim(splitResult[1]);
    // TODO(JS): replace with non-boost version
    minor = std::stoul(splitResult[1]);
  }

  if (splitResult.size() == 3)
  {
    boost::algorithm::trim(splitResult[2]);
    // TODO(JS): replace with non-boost version
    patch = std::stoul(splitResult[2]);
  }

  mMajor = major;
  mMinor = minor;
  mPatch = patch;
}

//==============================================================================
bool Version::operator==(const Version& other) const
{
  return getMajor() == other.getMajor() && getMinor() == other.getMinor()
         && getPatch() == other.getPatch();
}

//==============================================================================
bool Version::operator!=(const Version& other) const
{
  return !(*this == other);
}

//==============================================================================
bool Version::operator<(const Version& other) const
{
  if (getMajor() < other.getMajor())
  {
    return true;
  }
  else if (getMajor() > other.getMajor())
  {
    return false;
  }
  else
  {
    if (getMinor() < other.getMinor())
    {
      return true;
    }
    else if (getMinor() > other.getMinor())
    {
      return false;
    }
    else
    {
      if (getPatch() < other.getPatch())
        return true;
      else
        return false;
    }
  }
}

//==============================================================================
bool Version::operator>(const Version& other) const
{
  if (getMajor() > other.getMajor())
  {
    return true;
  }
  else if (getMajor() < other.getMajor())
  {
    return false;
  }
  else
  {
    if (getMinor() > other.getMinor())
    {
      return true;
    }
    else if (getMinor() < other.getMinor())
    {
      return false;
    }
    else
    {
      if (getPatch() > other.getPatch())
        return true;
      else
        return false;
    }
  }
}

//==============================================================================
bool Version::operator<=(const Version& other) const
{
  return !(*this > other);
}

//==============================================================================
bool Version::operator>=(const Version& other) const
{
  return !(*this < other);
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const Version& version)
{
  os << version.getMajor() << "." << version.getMinor() << "."
     << version.getPatch();

  return os;
}

//==============================================================================
std::istream& operator>>(std::istream& is, Version& version)
{
  char separator;
  unsigned short major, minor, patch;

  is >> major >> separator >> minor >> separator >> patch;
  version.set(major, minor, patch);

  return is;
}

} // namespace common
} // namespace dart
