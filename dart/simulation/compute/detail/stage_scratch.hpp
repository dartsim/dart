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

#pragma once

#include <dart/common/memory_manager.hpp>

#include <new>
#include <utility>

namespace dart::simulation::compute::stage_detail {

template <typename T, typename... Args>
[[nodiscard]] T* constructStageOwnedScratch(
    common::MemoryManager* memoryManager, Args&&... args)
{
  if (memoryManager != nullptr) {
    auto* scratch
        = memoryManager->constructUsingFree<T>(std::forward<Args>(args)...);
    if (scratch == nullptr) {
      throw std::bad_alloc();
    }
    return scratch;
  }

  return new T(std::forward<Args>(args)...);
}

template <typename T>
void destroyStageOwnedScratch(
    common::MemoryManager* memoryManager, T* scratch) noexcept
{
  if (scratch == nullptr) {
    return;
  }
  if (memoryManager != nullptr) {
    memoryManager->destroyUsingFree(scratch);
  } else {
    delete scratch;
  }
}

} // namespace dart::simulation::compute::stage_detail
