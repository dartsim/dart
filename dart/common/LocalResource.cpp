/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <cstring>
#include <limits>
#include <iostream>
#include "dart/common/Console.hpp"
#include "LocalResource.hpp"

namespace dart {
namespace common {

//==============================================================================
LocalResource::LocalResource(const std::string& _path)
  : mFile(std::fopen(_path.c_str(), "rb"))
{
  if(!mFile)
  {
    dtwarn << "[LocalResource::constructor] Failed opening file '"
           << _path << "' for reading: "
           << std::strerror(errno) << "\n";
  }
}

//==============================================================================
LocalResource::~LocalResource()
{
  if (!mFile)
    return;

  if (std::fclose(mFile) == EOF)
  {
    dtwarn << "[LocalResource::destructor] Failed closing file: "
           << std::strerror(errno) << "\n";
  }
}

//==============================================================================
bool LocalResource::isGood() const
{
  return !!mFile;
}

//==============================================================================
std::size_t LocalResource::getSize()
{
  if(!mFile)
    return 0;

  const long offset = std::ftell(mFile);
  if(offset == -1L)
  {
    dtwarn << "[LocalResource::getSize] Unable to compute file size: Failed"
              " getting current offset: "
           << std::strerror(errno) << "\n";
    return 0;
  }

  // The SEEK_END option is not required by the C standard. However, it is
  // required by POSIX.
  if(std::fseek(mFile, 0, SEEK_END) || std::ferror(mFile))
  {
    dtwarn << "[LocalResource::getSize] Unable to compute file size: Failed"
              " seeking to the end of the file: "
           << std::strerror(errno) << "\n";
    return 0;
  }

  const long size = std::ftell(mFile);
  if(size == -1L)
  {
    dtwarn << "[LocalResource::getSize] Unable to compute file size: Failed"
              " getting end of file offset: "
           << std::strerror(errno) << "\n";
    return 0;
  }
  // fopen, ftell, and fseek produce undefined behavior when called on
  // directories. ftell() on Linux libc returns LONG_MAX, unless you are in an
  // NFS mount.
  //
  // See here: http://stackoverflow.com/a/18193383/111426
  else if(size == std::numeric_limits<long>::max())
  {
    dtwarn << "[LocalResource::getSize] Unable to compute file size: Computed"
              " file size of LONG_MAX. Is this a directory?\n";
    return 0;
  }

  if(std::fseek(mFile, offset, SEEK_SET) || std::ferror(mFile))
  {
    dtwarn << "[LocalResource::getSize] Unable to compute file size: Failed"
              " restoring offset: "
           << std::strerror(errno) << "\n";
    return 0;
  }

  return size;
}

//==============================================================================
std::size_t LocalResource::tell()
{
  if(!mFile)
    return 0;
  
  const long offset = std::ftell(mFile);
  if(offset == -1L)
  {
    dtwarn << "[LocalResource::tell] Failed getting current offset: "
           << std::strerror(errno) << "\n";
  }
  // fopen, ftell, and fseek produce undefined behavior when called on
  // directories. ftell() on Linux libc returns LONG_MAX, unless you are in an
  // NFS mount.
  //
  // See here: http://stackoverflow.com/a/18193383/111426
  else if(offset == std::numeric_limits<long>::max())
  {
    dtwarn << "[LocalResource::tell] Failed getting current offset: ftell"
              " returned LONG_MAX. Is this a directory?\n";
    return -1L;
  }

  // We return -1 to match the beahvior of DefaultIoStream in Assimp.
  return offset;
}

//==============================================================================
bool LocalResource::seek(ptrdiff_t _offset, SeekType _mode)
{
  int origin;
  switch(_mode)
  {
  case Resource::SEEKTYPE_CUR:
    origin = SEEK_CUR;
    break;

  case Resource::SEEKTYPE_END:
    origin = SEEK_END;
    break;

  case Resource::SEEKTYPE_SET:
    origin = SEEK_SET;
    break;

  default:
    dtwarn << "[LocalResource::seek] Invalid origin. Expected"
              " SEEKTYPE_CUR, SEEKTYPE_END, or SEEKTYPE_SET.\n";
    return false;
  }

  if (!std::fseek(mFile, _offset, origin) && !std::ferror(mFile))
    return true;
  else
  {
    dtwarn << "[LocalResource::seek] Failed seeking: "
           << std::strerror(errno) << "\n";
    return false;
  }
}

//==============================================================================
std::size_t LocalResource::read(void *_buffer, std::size_t _size, std::size_t _count)
{
  if (!mFile)
    return 0;

  const std::size_t result = std::fread(_buffer, _size, _count, mFile);
  if (std::ferror(mFile)) 
  {
    dtwarn << "[LocalResource::read] Failed reading file: "
           << std::strerror(errno) << "\n";
  }
  return result;
}

} // namespace common
} // namespace dart
