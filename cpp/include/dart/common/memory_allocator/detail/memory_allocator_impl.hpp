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

#include "dart/common/bit.hpp"
#include "dart/common/memory_allocator/c_allocator.hpp"
#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common {

//==============================================================================
template <typename T>
template <typename... Args>
T* MemoryAllocator<T>::construct(Args&&... args) noexcept
{
  return aligned_construct<T>(0, std::forward<Args>(args)...);
}

//==============================================================================
template <typename T>
template <typename... Args>
T* MemoryAllocator<T>::aligned_construct(
    size_t alignment, Args&&... args) noexcept
{
  // Allocate new memory for a new object (without calling the constructor)
  void* object = allocate(sizeof(T), alignment);
  if (!object) {
    return nullptr;
  }

  // Call constructor. Return nullptr if failed.
  try {
    new (object) T(std::forward<Args>(args)...);
  } catch (...) {
    deallocate(object, sizeof(T));
    return nullptr;
  }

  return reinterpret_cast<T*>(object);
}

//==============================================================================
template <typename T>
void MemoryAllocator<T>::destroy(T* object) noexcept
{
  if (!object) {
    return;
  }
  object->~T();
  deallocate(object, sizeof(T));
}

//==============================================================================
template <typename T>
bool MemoryAllocator<T>::is_valid_alignment(size_t size, size_t alignment) const
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

} // namespace dart::common
