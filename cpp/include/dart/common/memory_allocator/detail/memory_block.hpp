/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#ifndef NDEBUG
  #include <iostream>
#endif
#include <functional>

#include "dart/common/memory_allocator/memory_allocator.hpp"

namespace dart::common::detail {

//==============================================================================
template <typename T>
class ObjectMemoryStack
{
private:
  struct Node
  {
    union
    {
      Node* next;
      T* object;
    };
  };

public:
  ObjectMemoryStack()
  {
    static_assert(
        sizeof(T) >= sizeof(Node),
        "Object size should be greater than the size of node.");
  }

  ObjectMemoryStack(const ObjectMemoryStack& other) = delete;

  ObjectMemoryStack& operator=(const ObjectMemoryStack& other) = delete;

  void push(T* ptr)
  {
    Node* new_node = reinterpret_cast<Node*>(ptr);
    new_node->next = m_head;
    m_head = new_node;
    m_size++;
  }

  void push_sorted(T* object)
  {
    if (is_empty()) {
      push(object);
      return;
    }

    // Find location to add the new object
    DART_ASSERT(m_head != nullptr);
    Node* new_node = reinterpret_cast<Node*>(object);

    if (object < reinterpret_cast<T*>(m_head)) {
      // Insert object above the current top
      new_node->next = m_head;
      m_head = new_node;
    } else {
      // Insert object below the iter
      Node* iter = m_head;
      while (true) {
        if (iter->next == nullptr
            || object < reinterpret_cast<T*>(iter->next)) {
          break;
        }
        iter = iter->next;
      }
      new_node->next = iter->next;
      iter->next = new_node;
    }

    m_size++;
  }

  T* pop()
  {
    if (is_empty()) {
      return nullptr;
    }
    Node* top = m_head;
    m_head = m_head->next;
    m_size--;
    return reinterpret_cast<T*>(top);
  }

  size_t get_size() const
  {
    return m_size;
  }

  bool is_empty() const
  {
    return (m_head == nullptr);
  }

  void clear()
  {
    m_head = nullptr;
    m_size = 0;
  }

#ifndef NDEBUG
  void print_list() const
  {
    Node* head = m_head;
    while (head) {
      std::cout << reinterpret_cast<T*>(head) << " ";
      head = head->next;
    }
  }
#endif

private:
  Node* m_head{nullptr};
  size_t m_size{0};
};

} // namespace dart::common::detail
