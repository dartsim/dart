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

#include "dart/common/memory_allocator/memory_allocator.hpp"

#include "dart/common/bit.hpp"
#include "dart/common/memory_allocator/raw_allocator.hpp"

namespace dart::common {

//==============================================================================
MemoryAllocator& MemoryAllocator::GetDefault()
{
  static RawAllocator default_allocator;
  return default_allocator;
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
bool MemoryAllocator::is_valid_alignment(size_t size, size_t alignment) const
{
  if (alignment == 0) {
    return true;
  }

  if (alignment < sizeof(void*)) {
    DART_DEBUG("Alignment '{}' must be greater than sizeof(void*).", alignment);
    return false;
  }

  if (!ispow2(alignment)) {
    DART_DEBUG("Alignment '{}' must be a power of 2.", alignment);
    return false;
  }

  if (size % alignment != 0) {
    DART_DEBUG(
        "Size '{}' must be a multiple of alignment '{}'.", size, alignment);
    return false;
  }

  return true;
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const MemoryAllocator& allocator)
{
  allocator.print(os);
  return os;
}

} // namespace dart::common
