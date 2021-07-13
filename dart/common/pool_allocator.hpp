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

#if DART_ENABLE_THREAD_SAFE
  #include <mutex>
#endif

#include "dart/common/allocator.hpp"
#include "dart/common/default_allocator.hpp"

namespace dart::common {

class MemoryAddressStack {
public:
  MemoryAddressStack() = default;
  MemoryAddressStack(const MemoryAddressStack&) = delete;
  MemoryAddressStack(MemoryAddressStack&&) = delete;

  void push(void* pointer) {
    Node* new_node = reinterpret_cast<Node*>(pointer);
    new_node->next = m_head;
  }

  void ordered_push(void* pointer) {
    if (empty()) {
      push(pointer);
      return;
    }

    // Find location to add the new pointer
    Node* iter = m_head;
    while (true) {
      if (iter->next == nullptr || std::greater<void*>()(iter->next, pointer)) {
        break;
      }
      iter = iter->next;
    }

    // Insert the new object to the location
    Node* new_node = reinterpret_cast<Node*>(pointer);
    new_node->next = iter->next;
    iter->next = new_node;

    // Update the top if the new object is inserted to the top location
    if (iter == m_head) {
      m_head = new_node;
    }
  }

  void* pop() {
    Node* top = m_head;
    m_head = m_head->next;
    return reinterpret_cast<void*>(top);
  }

  bool empty() const {
    return (m_head == nullptr);
  }

  void clear() {
    m_head = nullptr;
  }

private:
  struct Node {
    union {
      Node* next;
      void* object;
    };
  };

  Node* m_head = nullptr;
};

class PoolAllocator : public Allocator {
public:
  PoolAllocator(
      std::size_t total_size,
      std::size_t unit_size,
      std::shared_ptr<Allocator> base_allocator = nullptr);

  void* allocate(std::size_t size) override;
  void release(void* pointer, std::size_t size) override;

private:
#if DART_ENABLE_THREAD_SAFE
  mutable std::mutex m_mutex;
#endif
  std::size_t m_total_size;
  std::size_t m_unit_size;
  std::shared_ptr<Allocator> m_base_allocator;
};

} // namespace dart::common

#include "dart/common/detail/pool_allocator_impl.hpp"
