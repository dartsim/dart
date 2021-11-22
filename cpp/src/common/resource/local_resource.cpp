/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/common/resource/local_resource.hpp"

#include <cstring>
#include <iostream>
#include <limits>

#include "dart/common/logging.hpp"
#include "dart/common/stdio.hpp"
#include "dart/common/string.hpp"

namespace dart {
namespace common {

//==============================================================================
LocalResource::LocalResource(const std::string& path)
  : m_file(common::fopen(path.c_str(), "rb"))
{
  if (!m_file) {
    DART_WARN(
        "Failed opening file '{}' for reading: {}.",
        path,
        common::strerror(errno));
  }
}

//==============================================================================
LocalResource::~LocalResource()
{
  if (!m_file) {
    return;
  }

  if (std::fclose(m_file) == EOF) {
    DART_WARN("Failed closing file: {}", common::strerror(errno));
  }
}

//==============================================================================
bool LocalResource::is_good() const
{
  return !!m_file;
}

//==============================================================================
std::size_t LocalResource::get_size()
{
  if (!m_file)
    return 0;

  const long offset = std::ftell(m_file);
  if (offset == -1L) {
    DART_WARN(
        "Unable to compute file size: Failed getting current offset: {}",
        common::strerror(errno));
    return 0;
  }

  // The SEEK_END option is not required by the C standard. However, it is
  // required by POSIX.
  if (std::fseek(m_file, 0, SEEK_END) || std::ferror(m_file)) {
    DART_WARN(
        "Unable to compute file size: Failed seeking to the end of the file: "
        "{}",
        common::strerror(errno));
    return 0;
  }

  const long size = std::ftell(m_file);
  if (size == -1L) {
    DART_WARN(
        "[LocalResource::getSize] Unable to compute file size: Failed"
        " getting end of file offset: {}",
        common::strerror(errno));
    return 0;
  }
  // fopen, ftell, and fseek produce undefined behavior when called on
  // directories. ftell() on Linux libc returns LONG_MAX, unless you are in an
  // NFS mount.
  //
  // See here: http://stackoverflow.com/a/18193383/111426
  else if (size == std::numeric_limits<long>::max()) {
    DART_WARN(
        "Unable to compute file size: Computed file size of LONG_MAX. Is this "
        "a directory?\n");
    return 0;
  }

  if (std::fseek(m_file, offset, SEEK_SET) || std::ferror(m_file)) {
    DART_WARN(
        "Unable to compute file size: Failed restoring offset: {}",
        common::strerror(errno));
    return 0;
  }

  return size;
}

//==============================================================================
std::size_t LocalResource::tell()
{
  if (!m_file)
    return 0;

  const long offset = std::ftell(m_file);
  if (offset == -1L) {
    DART_WARN("Failed getting current offset: {}", common::strerror(errno));
  }
  // fopen, ftell, and fseek produce undefined behavior when called on
  // directories. ftell() on Linux libc returns LONG_MAX, unless you are in an
  // NFS mount.
  //
  // See here: http://stackoverflow.com/a/18193383/111426
  else if (offset == std::numeric_limits<long>::max()) {
    DART_WARN(
        "Failed getting current offset: ftell returned LONG_MAX. Is this a "
        "directory?");
    return -1L;
  }

  // We return -1 to match the beahvior of DefaultIoStream in Assimp.
  return offset;
}

//==============================================================================
bool LocalResource::seek(ptrdiff_t offset, SeekType mode)
{
  int origin;
  switch (mode) {
    case Resource::SeekType::CUR:
      origin = SEEK_CUR;
      break;

    case Resource::SeekType::END:
      origin = SEEK_END;
      break;

    case Resource::SeekType::SET:
      origin = SEEK_SET;
      break;

    default:
      DART_WARN(
          "[LocalResource::seek] Invalid origin. Expected SEEKTYPE_CUR, "
          "SEEKTYPE_END, or SEEKTYPE_SET.");
      return false;
  }

  if (!std::fseek(m_file, offset, origin) && !std::ferror(m_file))
    return true;
  else {
    DART_WARN("Failed seeking: {}", common::strerror(errno));
    return false;
  }
}

//==============================================================================
std::size_t LocalResource::read(
    void* buffer, std::size_t size, std::size_t count)
{
  if (!m_file)
    return 0;

  const std::size_t result = std::fread(buffer, size, count, m_file);
  if (std::ferror(m_file)) {
    DART_WARN("Failed reading file: {}", common::strerror(errno));
  }
  return result;
}

} // namespace common
} // namespace dart
