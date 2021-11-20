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

#pragma once

#include "dart/common/memory_allocator/memory_manager.hpp"

namespace dart::common {

//==============================================================================
template <typename T, typename... Args>
T* MemoryManager::construct(Type type, Args&&... args) noexcept
{
  // Allocate new memory for a new object (without calling the constructor)
  void* object = allocate(type, sizeof(T));
  if (!object) {
    return nullptr;
  }

  // Call constructor. Return nullptr if failed.
  try {
    new (object) T(std::forward<Args>(args)...);
  } catch (...) {
    deallocate(type, object, sizeof(T));
    return nullptr;
  }

  return reinterpret_cast<T*>(object);
}

//==============================================================================
template <typename T, typename... Args>
T* MemoryManager::construct_using_free(Args&&... args) noexcept
{
  return construct<T, Args...>(Type::Free, std::forward<Args>(args)...);
}

//==============================================================================
template <typename T, typename... Args>
T* MemoryManager::construct_using_pool(Args&&... args) noexcept
{
  return construct<T, Args...>(Type::Pool, std::forward<Args>(args)...);
}

//==============================================================================
template <typename T, typename... Args>
T* MemoryManager::construct_using_frame(Args&&... args) noexcept
{
  return construct<T, Args...>(Type::Frame, std::forward<Args>(args)...);
}

//==============================================================================
template <typename T>
void MemoryManager::destroy(Type type, T* object) noexcept
{
  if (!object) {
    return;
  }
  object->~T();
  deallocate(type, object, sizeof(T));
}

//==============================================================================
template <typename T>
void MemoryManager::destroy_using_free(T* pointer) noexcept
{
  destroy(Type::Free, pointer);
}

//==============================================================================
template <typename T>
void MemoryManager::destroy_using_pool(T* pointer) noexcept
{
  destroy(Type::Pool, pointer);
}

//==============================================================================
template <typename T>
void MemoryManager::destroy_using_frame(T* pointer) noexcept
{
  destroy(Type::Frame, pointer);
}

} // namespace dart::common
