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

#include <mutex>

namespace dart::common {

/// A simple object pool that uses a linked list to keep track of free
/// objects.
/// @tparam T The type of object to pool.
/// @tparam Alignment The alignment of the objects in the pool.
template <typename T, std::size_t Alignment = alignof(T)>
class AlignedObjectPool
{
public:
  /// Constructor
  explicit AlignedObjectPool();

  /// Creates a new object in the pool.
  /// @tparam ...Args The types of the arguments to pass to the constructor.
  /// @param ...args The arguments to pass to the constructor.
  /// @return A pointer to the newly created object.
  template <typename... Args>
  [[nodiscard]] T* create(Args&&... args) noexcept;

  /// Destroys an object in the pool.
  /// @param object The object to destroy.
  void destroy(T* object);

private:
  /// A memory unit used to construct the linked list of free objects.
  struct MemoryUnit
  {
    /// Pointer to next memory block.
    MemoryUnit* next;
  };

  /// A memory block used to construct the linked list of free objects.
  struct MemoryBlock
  {
    /// The next memory block.
    MemoryBlock* next;
  };

  /// The head of the linked list of free objects
  T* mFreeList{nullptr};

  /// The head of the linked list of free memory blocks.
  MemoryBlock* mFreeBlock{nullptr};

  /// The mutex used to protect the pool
  std::mutex mMutex;
};

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

#include <dart/common/Macros.hpp>

namespace dart::common {

//==============================================================================
template <typename T, std::size_t Alignment>
AlignedObjectPool<T, Alignment>::AlignedObjectPool()
{
  static_assert(
      (Alignment & (Alignment - 1)) == 0, "Alignment must be a power of 2");
  static_assert(
      sizeof(MemoryUnit) <= sizeof(T),
      "MemoryUnit should be equal to or less than T");
  static_assert(
      sizeof(MemoryBlock) <= sizeof(T),
      "MemoryBlock should be equal to or less than T");
}

//==============================================================================
template <typename T, std::size_t Alignment>
template <typename... Args>
T* AlignedObjectPool<T, Alignment>::create(Args&&... args) noexcept
{
  static constexpr std::size_t N = 1024;
  static_assert((N & (N - 1)) == 0, "N must be a power of 2");

  std::lock_guard<std::mutex> lock(mMutex);

  // If the free list is empty, then allocate a new block of memory.
  if (mFreeList == nullptr) {
    if (mFreeBlock == nullptr) {
      // Allocate a new block of memory for the pool.
      char* block
          = static_cast<char*>(std::aligned_alloc(Alignment, N * sizeof(T)));
      if (block == nullptr)
        return nullptr;

      // Create a linked list of free objects in the block.
      for (std::size_t i = 0; i < N - 1; ++i) {
        reinterpret_cast<MemoryUnit*>(block + i * sizeof(T))->next
            = reinterpret_cast<MemoryUnit*>(block + (i + 1) * sizeof(T));
      }
      reinterpret_cast<MemoryUnit*>(block + (N - 1) * sizeof(T))->next
          = nullptr;
      mFreeList = reinterpret_cast<T*>(block);
    } else {
      // Pop the first block off the free block list.
      DART_ASSERT(mFreeBlock);
      MemoryBlock* block = mFreeBlock;
      mFreeBlock = block->next;

      // Create a linked list of free objects in the block.
      for (std::size_t i = 0; i < N - 1; ++i) {
        reinterpret_cast<MemoryUnit*>(block + i * sizeof(T))->next
            = reinterpret_cast<MemoryUnit*>(block + (i + 1) * sizeof(T));
      }
      reinterpret_cast<MemoryUnit*>(block + (N - 1) * sizeof(T))->next
          = nullptr;
      mFreeList = reinterpret_cast<T*>(block);
    }

    // Allocate a new block of memory for the pool.
    char* block
        = static_cast<char*>(std::aligned_alloc(Alignment, N * sizeof(T)));
    if (block == nullptr)
      return nullptr;

    // Create a linked list of free objects in the block.
    for (std::size_t i = 0; i < N - 1; ++i) {
      reinterpret_cast<MemoryUnit*>(block + i * sizeof(T))->next
          = reinterpret_cast<MemoryUnit*>(block + (i + 1) * sizeof(T));
    }
    reinterpret_cast<MemoryUnit*>(block + (N - 1) * sizeof(T))->next = nullptr;
    mFreeList = reinterpret_cast<T*>(block);
  }

  // Pop the first object off the free list.
  DART_ASSERT(mFreeList);
  T* object = mFreeList;
  mFreeList = reinterpret_cast<T*>(reinterpret_cast<MemoryUnit*>(object)->next);

  try {
    // Construct the object in place.
    new (object) T(std::forward<Args>(args)...);
  } catch (...) {
    reinterpret_cast<MemoryUnit*>(object)->next
        = reinterpret_cast<MemoryUnit*>(mFreeList);
    mFreeList = object;
    return nullptr;
  }

  return object;
}

//==============================================================================
template <typename T, std::size_t Alignment>
void AlignedObjectPool<T, Alignment>::destroy(T* object)
{
  // If the object is null, then there's nothing to do.
  if (!object)
    return;

  // Call the destructor.
  object->~T();

  {
    std::lock_guard<std::mutex> lock(mMutex);

    // Push the object back onto the free list.
    reinterpret_cast<MemoryUnit*>(object)->next
        = reinterpret_cast<MemoryUnit*>(mFreeList);
    mFreeList = object;
  }
}

} // namespace dart::common
