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

#pragma once

#include "dart/common/memory_allocator/stl_allocator.hpp"

namespace dart::common {

//==============================================================================
template <typename T>
StlAllocator<T>::StlAllocator(MemoryAllocator& base_allocator) noexcept
  : m_base_allocator(base_allocator)
{
  // Do nothing
}

//==============================================================================
template <typename T>
StlAllocator<T>::StlAllocator(const StlAllocator& other) throw()
  : std::allocator<T>(other), m_base_allocator(other.m_base_allocator)
{
  // Do nothing
}

//==============================================================================
template <typename T>
template <class U>
StlAllocator<T>::StlAllocator(const StlAllocator<U>& other) throw()
  : std::allocator<T>(other), m_base_allocator(other.m_base_allocator)
{
  // Do nothing
}

//==============================================================================
template <typename T>
typename StlAllocator<T>::pointer StlAllocator<T>::allocate(
    size_type n, const void* hint)
{
  DART_UNUSED(hint);
  pointer ptr
      = reinterpret_cast<pointer>(m_base_allocator.allocate(n * sizeof(T)));

  // Throw std::bad_alloc to comply 23.10.9.1
  // Reference: https://stackoverflow.com/a/50326956/3122234
  if (!ptr) {
    throw std::bad_alloc();
  }

  return ptr;
}

//==============================================================================
template <typename T>
void StlAllocator<T>::deallocate(pointer pointer, size_type n)
{
  DART_UNUSED(n);
  m_base_allocator.deallocate(pointer);
}

// void deallocate(pointer pointer, size_t n);

} // namespace dart::common
