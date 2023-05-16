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

#include "dart/common/MemoryResource.hpp"

#include <cstring>

namespace dart::common {

//==============================================================================
MemoryResource::MemoryResource(const std::string& content)
  : mContent(content), mCursor(0)
{
  // Empty
}

//==============================================================================
std::size_t MemoryResource::getSize()
{
  return mContent.size();
}

//==============================================================================
std::size_t MemoryResource::tell()
{
  return mCursor;
}

//==============================================================================
bool MemoryResource::seek(ptrdiff_t offset, SeekType mode)
{
  switch (mode) {
    case Resource::SeekType::CUR:
      mCursor += offset;
      break;
    case Resource::SeekType::END:
      mCursor = mContent.size() + offset;
      break;
    case Resource::SeekType::SET:
      mCursor = offset;
      break;
    default:
      return false;
  }

  if (mCursor > mContent.size()) {
    mCursor = mContent.size();
  }

  return true;
}

//==============================================================================
std::size_t MemoryResource::read(
    void* buffer, std::size_t size, std::size_t count)
{
  std::size_t bytesToRead = size * count;
  if (mCursor + bytesToRead > mContent.size()) {
    bytesToRead = mContent.size() - mCursor;
  }

  std::memcpy(buffer, mContent.data() + mCursor, bytesToRead);
  mCursor += bytesToRead;

  return bytesToRead / size;
}

} // namespace dart::common
