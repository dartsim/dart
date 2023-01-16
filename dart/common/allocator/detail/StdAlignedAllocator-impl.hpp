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

#pragma once

#include <dart/common/Logging.hpp>
#include <dart/common/allocator/StdAlignedAllocator.hpp>

namespace dart::common {

//==============================================================================
template <typename T>
StdAlignedAllocator<T>::StdAlignedAllocator(
    AlignedAllocator& baseAllocator) noexcept
  : mBaseAllocator(baseAllocator)
{
  // Do nothing
}

//==============================================================================
template <typename T>
StdAlignedAllocator<T>::StdAlignedAllocator(
    const StdAlignedAllocator& other) throw()
  : std::allocator<T>(other), mBaseAllocator(other.mBaseAllocator)
{
  // Do nothing
}

//==============================================================================
template <typename T>
template <class U>
StdAlignedAllocator<T>::StdAlignedAllocator(
    const StdAlignedAllocator<U>& other) throw()
  : std::allocator<T>(other), mBaseAllocator(other.mBaseAllocator)
{
  // Do nothing
}

//==============================================================================
template <typename T>
typename StdAlignedAllocator<T>::pointer StdAlignedAllocator<T>::allocate(
    size_type n, const void* hint)
{
  (void)hint;
  pointer ptr = reinterpret_cast<pointer>(
      mBaseAllocator.allocate(n * sizeof(T), alignof(T)));

  // Throw std::bad_alloc to comply 23.10.9.1
  // Reference: https://stackoverflow.com/a/50326956/3122234
  if (!ptr) {
    throw std::bad_alloc();
  }

  return ptr;
}

//==============================================================================
template <typename T>
void StdAlignedAllocator<T>::deallocate(pointer pointer, size_type n)
{
  mBaseAllocator.deallocate(pointer, n * sizeof(T));
}

//==============================================================================
template <typename T>
void StdAlignedAllocator<T>::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[dart::common::StdAlignedAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

//==============================================================================
template <typename T>
std::ostream& operator<<(
    std::ostream& os, const StdAlignedAllocator<T>& allocator)
{
  allocator.print(os);
  return os;
}

} // namespace dart::common
