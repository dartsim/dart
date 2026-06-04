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

#include "dart/common/memory_allocator.hpp"

#include "dart/common/callocator.hpp"
#include "dart/common/logging.hpp"

#include <cstddef>

namespace dart::common {

namespace {

bool isPowerOfTwo(size_t value)
{
  return value != 0 && (value & (value - 1)) == 0;
}

} // namespace

//==============================================================================
MemoryAllocator& MemoryAllocator::GetDefault()
{
  static CAllocator defaultAllocator;
  return defaultAllocator;
}

//==============================================================================
void* MemoryAllocator::allocate(size_t bytes, size_t alignment) noexcept
{
  if (bytes == 0 || !isPowerOfTwo(alignment)) {
    return nullptr;
  }

  if (alignment <= alignof(std::max_align_t)) {
    return allocate(bytes);
  }

  return nullptr;
}

//==============================================================================
void MemoryAllocator::deallocate(
    void* pointer, size_t bytes, size_t /*alignment*/)
{
  deallocate(pointer, bytes);
}

//==============================================================================
void MemoryAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[*::print is not implemented]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "*::print is not implemented:\n";
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const MemoryAllocator& allocator)
{
  allocator.print(os);
  return os;
}

} // namespace dart::common
