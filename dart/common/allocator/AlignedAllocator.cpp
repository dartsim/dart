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

#include "dart/common/allocator/AlignedAllocator.hpp"

#include "dart/common/allocator/AlignedAllocatorRaw.hpp"

namespace dart::common {

namespace {

template <typename T>
constexpr bool ispow2(T x) noexcept
{
  return (x & (x - 1)) == 0;
}

} // namespace

//==============================================================================
AlignedAllocator& AlignedAllocator::GetDefault()
{
  static AlignedAllocatorRaw defaultAllocator;
  return defaultAllocator;
}

//==============================================================================
void AlignedAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[*::print is not implemented]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "*::print is not implemented:\n";
}

//==============================================================================
bool AlignedAllocator::validateAlignment(size_t size, size_t alignment) const
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
std::ostream& operator<<(std::ostream& os, const AlignedAllocator& allocator)
{
  allocator.print(os);
  return os;
}

} // namespace dart::common
