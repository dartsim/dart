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

#include <dart/simulation/MemoryManager.hpp>

namespace dart::simulation {

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
T* MemoryManager::constructUsingFree(Args&&... args) noexcept
{
  return construct<T, Args...>(Type::Free, std::forward<Args>(args)...);
}

//==============================================================================
template <typename T, typename... Args>
T* MemoryManager::constructUsingPool(Args&&... args) noexcept
{
  return construct<T, Args...>(Type::Pool, std::forward<Args>(args)...);
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
void MemoryManager::destroyUsingFree(T* pointer) noexcept
{
  destroy(Type::Free, pointer);
}

//==============================================================================
template <typename T>
void MemoryManager::destroyUsingPool(T* pointer) noexcept
{
  destroy(Type::Pool, pointer);
}

} // namespace dart::simulation
