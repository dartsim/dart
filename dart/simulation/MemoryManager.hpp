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

#include <dart/simulation/Fwd.hpp>

#include <dart/common/allocator/FreeListAllocator.hpp>
#include <dart/common/allocator/PoolAllocator.hpp>

#include <iostream>
#ifndef NDEBUG
  #include <mutex>
#endif

namespace dart::simulation {

/// A composite memory allocator that contains various memory allocators that
/// are optimized for different use cases.
class DART_SIMULATION_API MemoryManager final
{
public:
  /// Type of the memory allocators
  enum class Type
  {
    Base,
    Free,
    Pool,
  };

  /// Returns the default memory manager
  [[nodiscard]] static MemoryManager& GetDefault();

  /// Constructor
  ///
  /// \param[in] baseAllocator: (optional) The most low level allocator to be
  /// used by all the underlying memory allocators.
  explicit MemoryManager(
      common::MemoryAllocator& baseAllocator
      = common::MemoryAllocator::GetDefault());

  /// Destructor
  ~MemoryManager();

  /// Returns the base allocator
  [[nodiscard]] common::MemoryAllocator& getBaseAllocator();

  /// Returns the free list allocator
  [[nodiscard]] common::FreeListAllocator& getFreeListAllocator();

  /// Returns the pool allocator
  [[nodiscard]] common::PoolAllocator& getPoolAllocator();

  /// Allocates \c size bytes of uninitialized storage.
  ///
  /// \param[in] type: The memory allocator type.
  /// \param[in] bytes: The byte size to allocate sotrage for.
  /// \return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// \return On failure, a null pointer
  [[nodiscard]] void* allocate(Type type, size_t bytes);

  /// Allocates \c size bytes of uninitialized storage using FreeListAllocator.
  ///
  /// \param[in] bytes: The byte size to allocate sotrage for.
  /// \return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// \return On failure, a null pointer
  [[nodiscard]] void* allocateUsingFree(size_t bytes);

  /// Allocates \c size bytes of uninitialized storage using PoolAllocator.
  ///
  /// \param[in] bytes: The byte size to allocate sotrage for.
  /// \return On success, the pointer to the beginning of newly allocated
  /// memory.
  /// \return On failure, a null pointer
  [[nodiscard]] void* allocateUsingPool(size_t bytes);

  /// Deallocates the storage referenced by the pointer \c p, which must be a
  /// pointer obtained by an earlier cal to allocate().
  ///
  /// \param[in] type: The memory allocator type.
  /// \param[in] pointer: Pointer obtained from allocate().
  /// \param[in] bytes: The bytes of the allocated memory.
  void deallocate(Type type, void* pointer, size_t bytes);
  // TODO(JS): Make this constexpr once migrated to C++20

  void deallocateUsingFree(void* pointer, size_t bytes);

  void deallocateUsingPool(void* pointer, size_t bytes);

  /// Allocates uninitialized storage and constructs an object of type T to the
  /// allocated storage.
  ///
  /// \tparam T: The object type to construct.
  /// \tparam Args...: The argument types to pass to the object constructor.
  ///
  /// \param[in] type: The memory allocator type.
  /// \param[in] args: The constructor arguments to use.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Type type, Args&&... args) noexcept;

  /// Allocates uninitialized storage using FreeListAllocator and constructs an
  /// object of type T to the allocated storage.
  template <typename T, typename... Args>
  [[nodiscard]] T* constructUsingFree(Args&&... args) noexcept;

  /// Allocates uninitialized storage using PoolAllocator and constructs an
  /// object of type T to the allocated storage.
  template <typename T, typename... Args>
  [[nodiscard]] T* constructUsingPool(Args&&... args) noexcept;

  /// Calls the destructor of the object and deallocate the storage.
  template <typename T>
  void destroy(Type type, T* object) noexcept;

  /// Calls the destructor of the object and deallocate the storage using
  /// FreeListAllocator.
  template <typename T>
  void destroyUsingFree(T* pointer) noexcept;

  /// Calls the destructor of the object and deallocate the storage using
  /// PoolAllocator.
  template <typename T>
  void destroyUsingPool(T* pointer) noexcept;

#ifndef NDEBUG
  /// Returns true if a pointer is allocated by the internal allocator.
  [[nodiscard]] bool hasAllocated(void* pointer, size_t size) const noexcept;
#endif

  /// Prints state of the memory manager.
  void print(std::ostream& os = std::cout, int indent = 0) const;

  /// Prints state of the memory manager.
  friend std::ostream& operator<<(
      std::ostream& os, const MemoryManager& memoryManager);

private:
  /// The base allocator to allocate memory chunk.
  common::MemoryAllocator& mBaseAllocator;

#ifdef NDEBUG
  /// The free list allocator.
  common::FreeListAllocator mFreeListAllocator;

  /// The pool allocator.
  common::PoolAllocator mPoolAllocator;
#else
  /// The free list allocator.
  common::FreeListAllocator::Debug mFreeListAllocator;

  /// The pool allocator.
  common::PoolAllocator::Debug mPoolAllocator;
#endif
};

} // namespace dart::simulation

#include <dart/simulation/detail/MemoryManager-impl.hpp>
