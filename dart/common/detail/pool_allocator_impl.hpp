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

#include "dart/common/logging.hpp"
#include "dart/common/pool_allocator.hpp"

namespace dart::common {

//==============================================================================
// template <std::size_t ChunkSize, std::size_t Alignment>
// PoolAllocator<ChunkSize, Alignment>::PoolAllocator(
//    std::shared_ptr<Allocator> base_allocator) {
//  // Template parameter checks
//  static_assert(ChunkSize >= 8, "Chunk size must be greater or equal to 8");

//  // Set base allocator
//  m_base_allocator = base_allocator ? std::move(base_allocator)
//                                    : std::make_shared<DefaultAllocator>();
//}

////==============================================================================
// template <std::size_t ChunkSize, std::size_t Alignment>
// void* PoolAllocator<ChunkSize, Alignment>::allocate(size_t size) {
//#if DART_ENABLE_THREAD_SAFE
//  std::lock_guard<std::mutex> lock(m_mutex);
//#endif

//  if (size == 0) {
//    DART_DEBUG("Not allowed to allocate zero bytes");
//    return nullptr;
//  }

//  return nullptr;
//}

////==============================================================================
// template <std::size_t ChunkSize, std::size_t Alignment>
// void PoolAllocator<ChunkSize, Alignment>::release(
//    void* pointer, std::size_t size) {
//#if DART_ENABLE_THREAD_SAFE
//  std::lock_guard<std::mutex> lock(m_mutex);
//#endif

//  if (size == 0) {
//    DART_DEBUG("Not allowed to release zero bytes");
//    return;
//  }
//}

} // namespace dart::common
