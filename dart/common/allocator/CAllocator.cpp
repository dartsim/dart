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

#include "dart/common/allocator/CAllocator.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"

namespace dart::common {

//==============================================================================
CAllocator::CAllocator() noexcept
{
  // Do nothing
}

//==============================================================================
CAllocator::~CAllocator()
{
  // Do nothing
}

//==============================================================================
void* CAllocator::allocate(size_t bytes) noexcept
{
  if (bytes == 0) {
    return nullptr;
  }

  DART_TRACE("Allocated {} bytes.", bytes);
  return std::malloc(bytes);
}

//==============================================================================
void CAllocator::deallocate(void* pointer, size_t bytes)
{
  DART_UNUSED(bytes);
  std::free(pointer);
  DART_TRACE("Deallocated.");
}

//==============================================================================
void CAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[CAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << getType() << "\n";
  }
}

} // namespace dart::common
