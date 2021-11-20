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

#include "dart/common/memory_allocator/frame_allocator.hpp"
#include "dart/common/memory_allocator/free_list_allocator.hpp"
#include "dart/common/memory_allocator/pool_allocator.hpp"
#include "dart/common/memory_allocator/raw_allocator.hpp"

namespace dart::common {

class DART_COMMON_API MemoryManager final
{
public:
  /// Type of the memory allocators
  enum class Type
  {
    Base,
    Free,
    Pool,
    Frame,
  };

  /// Returns the default memory manager
  [[nodiscard]] static MemoryManager& GetDefault();

  /// Constructor
  ///
  /// @param[in] base_allocator: Low level allocator to be used for allocating
  /// memory required by this memory allocator
  explicit MemoryManager(
      MemoryAllocator& base_allocator = MemoryAllocator::GetDefault());

  /// Destructor
  ~MemoryManager();

  /// Returns the base allocator
  [[nodiscard]] MemoryAllocator& get_mutable_base_allocator();

  /// Returns the free list allocator
  [[nodiscard]] FreeListAllocator& get_mutable_free_list_allocator();

  /// Returns the pool allocator
  [[nodiscard]] PoolAllocator& get_mutable_pool_allocator();

  /// Returns the frame allocator
  [[nodiscard]] FrameAllocator& get_mutable_frame_allocator();

  /// Allocates @c size bytes of uninitialized storage.
  [[nodiscard]] void* allocate(Type type, size_t size);

  /// Deallocates the storage referenced by the pointer @c p, which must be a
  /// pointer obtained by an earlier cal to allocate() or allocate_aligned().
  void deallocate(Type type, void* pointer, size_t size);

  // TODO(JS): Add allocate_aligned() and deallocate_aligned()

  /// Allocates uninitialized storage and constructs an object of type T to the
  /// allocated storage.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Type type, Args&&... args) noexcept;

  /// Allocates uninitialized storage using FreeListAllocator and constructs an
  /// object of type T to the allocated storage.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct_using_free(Args&&... args) noexcept;

  /// Allocates uninitialized storage using PoolAllocator and constructs an
  /// object of type T to the allocated storage.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct_using_pool(Args&&... args) noexcept;

  /// Allocates uninitialized storage using FrameAllocator and constructs an
  /// object of type T to the allocated storage.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct_using_frame(Args&&... args) noexcept;

  /// Calls the destructor of the object and deallocate the storage.
  template <typename T>
  void destroy(Type type, T* object) noexcept;

  /// Calls the destructor of the object and deallocate the storage using
  /// FreeListAllocator.
  template <typename T>
  void destroy_using_free(T* pointer) noexcept;

  /// Calls the destructor of the object and deallocate the storage using
  /// PoolAllocator.
  template <typename T>
  void destroy_using_pool(T* pointer) noexcept;

  /// Calls the destructor of the object and deallocate the storage using
  /// FrameAllocator.
  template <typename T>
  void destroy_using_frame(T* pointer) noexcept;

  /// Resets the FrameAllocator
  void reset_frame_allocator();

  /// Prints state of the memory manager.
  void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory manager.
  friend std::ostream& operator<<(
      std::ostream& os, const MemoryManager& memory_manager);

private:
  /// The base allocator to allocate memory chunck.
  MemoryAllocator& m_base_allocator;

  /// The free list allocator.
  FreeListAllocator m_free_list_allocator;

  /// The pool allocator.
  PoolAllocator m_pool_allocator;

  /// The frame allocator.
  FrameAllocator m_frame_allocator;
};

} // namespace dart::common

#include "dart/common/memory_allocator/detail/memory_manager_impl.hpp"
