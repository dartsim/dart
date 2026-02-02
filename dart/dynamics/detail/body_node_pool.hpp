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

#ifndef DART_DYNAMICS_DETAIL_BODY_NODE_POOL_HPP_
#define DART_DYNAMICS_DETAIL_BODY_NODE_POOL_HPP_

#include <dart/common/memory_allocator.hpp>

#include <memory>
#include <new>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::dynamics::detail {

//==============================================================================
template <typename T>
class BodyNodePool
{
public:
  static_assert(alignof(T) <= 32, "BodyNodePool requires alignof(T) <= 32.");
  static_assert(
      sizeof(T) >= sizeof(void*),
      "BodyNodePool requires sizeof(T) >= sizeof(void*).");

  static constexpr std::size_t kSlotSize = ((sizeof(T) + 31) / 32) * 32;

  explicit BodyNodePool(
      common::MemoryAllocator& baseAllocator
      = common::MemoryAllocator::GetDefault(),
      std::size_t initialChunkCapacity = 64);
  ~BodyNodePool(); // Calls clear()

  // Non-copyable, non-movable (pool owns memory that live objects point into)
  BodyNodePool(const BodyNodePool&) = delete;
  BodyNodePool& operator=(const BodyNodePool&) = delete;
  BodyNodePool(BodyNodePool&&) = delete;
  BodyNodePool& operator=(BodyNodePool&&) = delete;

  /// Allocate a raw slot (32-byte aligned, sizeof(T) usable bytes).
  /// Does NOT construct an object. Caller must use placement new.
  [[nodiscard]] void* allocate();

  /// Return a slot to the free list. Does NOT call destructor.
  /// Caller must explicitly destruct before calling this.
  void deallocate(void* ptr) noexcept;

  /// Returns true if ptr was allocated from this pool
  [[nodiscard]] bool owns(const void* ptr) const noexcept;

  /// Number of currently allocated (in-use) slots
  [[nodiscard]] std::size_t size() const noexcept;

  /// Total capacity across all chunks
  [[nodiscard]] std::size_t capacity() const noexcept;

private:
  struct FreeNode
  {
    FreeNode* mNext;
  };

  struct Chunk
  {
    Chunk(common::MemoryAllocator& allocator, std::size_t slotCount);
    ~Chunk();

    std::byte* getSlot(std::size_t index) const;

    common::MemoryAllocator& mAllocator;
    void* mRawPtr;
    std::byte* mData;
    std::size_t mAllocatedSize;
  };

  void clear() noexcept;
  void addChunk();

  static constexpr std::size_t kDefaultSlotsPerChunk = 64;

  common::MemoryAllocator& mBaseAllocator;
  const std::size_t mSlotsPerChunk;
  std::vector<std::shared_ptr<Chunk>> mChunks;
  FreeNode* mFreeList;
  std::size_t mSize;
  std::size_t mNextSlotIndex;

public:
  /// Get the shared_ptr to the chunk owning ptr (for cross-skeleton moves).
  /// Returns nullptr if ptr is not owned by this pool.
  [[nodiscard]] std::shared_ptr<Chunk> getChunkFor(
      const void* ptr) const noexcept;
};

//==============================================================================
template <typename T>
BodyNodePool<T>::Chunk::Chunk(
    common::MemoryAllocator& allocator, std::size_t slotCount)
  : mAllocator(allocator), mAllocatedSize(slotCount * kSlotSize + 32)
{
  mRawPtr = allocator.allocate(mAllocatedSize);
  if (mRawPtr) {
    auto raw = reinterpret_cast<std::uintptr_t>(mRawPtr);
    auto aligned = (raw + 31) & ~std::uintptr_t{31};
    mData = reinterpret_cast<std::byte*>(aligned);
  } else {
    mData = nullptr;
  }
}

//==============================================================================
template <typename T>
BodyNodePool<T>::Chunk::~Chunk()
{
  if (mRawPtr) {
    mAllocator.deallocate(mRawPtr, mAllocatedSize);
  }
}

//==============================================================================
template <typename T>
std::byte* BodyNodePool<T>::Chunk::getSlot(std::size_t index) const
{
  return mData + (index * kSlotSize);
}

//==============================================================================
template <typename T>
BodyNodePool<T>::BodyNodePool(
    common::MemoryAllocator& baseAllocator, std::size_t initialChunkCapacity)
  : mBaseAllocator(baseAllocator),
    mSlotsPerChunk(
        initialChunkCapacity == 0 ? kDefaultSlotsPerChunk
                                  : initialChunkCapacity),
    mChunks(),
    mFreeList(nullptr),
    mSize(0),
    mNextSlotIndex(0)
{
}

//==============================================================================
template <typename T>
BodyNodePool<T>::~BodyNodePool()
{
  clear();
}

//==============================================================================
template <typename T>
void* BodyNodePool<T>::allocate()
{
  if (mFreeList != nullptr) {
    FreeNode* node = mFreeList;
    mFreeList = node->mNext;
    ++mSize;
    return node;
  }

  if (mChunks.empty() || mNextSlotIndex >= mSlotsPerChunk) {
    addChunk();
  }

  void* slot = mChunks.back()->getSlot(mNextSlotIndex);
  ++mNextSlotIndex;
  ++mSize;
  return slot;
}

//==============================================================================
template <typename T>
void BodyNodePool<T>::deallocate(void* ptr) noexcept
{
  if (ptr == nullptr) {
    return;
  }

  FreeNode* node = static_cast<FreeNode*>(ptr);
  node->mNext = mFreeList;
  mFreeList = node;
  --mSize;
}

//==============================================================================
template <typename T>
bool BodyNodePool<T>::owns(const void* ptr) const noexcept
{
  if (ptr == nullptr) {
    return false;
  }

  const auto* bytePtr = static_cast<const std::byte*>(ptr);
  const std::size_t chunkBytes = mSlotsPerChunk * kSlotSize;
  for (const auto& chunk : mChunks) {
    const std::byte* begin = chunk->mData;
    const std::byte* end = begin + chunkBytes;
    if (bytePtr >= begin && bytePtr < end) {
      return true;
    }
  }

  return false;
}

//==============================================================================
template <typename T>
std::size_t BodyNodePool<T>::size() const noexcept
{
  return mSize;
}

//==============================================================================
template <typename T>
std::size_t BodyNodePool<T>::capacity() const noexcept
{
  return mChunks.size() * mSlotsPerChunk;
}

//==============================================================================
template <typename T>
void BodyNodePool<T>::clear() noexcept
{
  mChunks.clear();
  mFreeList = nullptr;
  mSize = 0;
  mNextSlotIndex = 0;
}

//==============================================================================
template <typename T>
void BodyNodePool<T>::addChunk()
{
  mChunks.emplace_back(std::make_shared<Chunk>(mBaseAllocator, mSlotsPerChunk));
  mNextSlotIndex = 0;
}

//==============================================================================
template <typename T>
std::shared_ptr<typename BodyNodePool<T>::Chunk> BodyNodePool<T>::getChunkFor(
    const void* ptr) const noexcept
{
  if (ptr == nullptr) {
    return nullptr;
  }

  const auto* bytePtr = static_cast<const std::byte*>(ptr);
  const std::size_t chunkBytes = mSlotsPerChunk * kSlotSize;
  for (const auto& chunk : mChunks) {
    const std::byte* begin = chunk->mData;
    const std::byte* end = begin + chunkBytes;
    if (bytePtr >= begin && bytePtr < end) {
      return chunk;
    }
  }

  return nullptr;
}

} // namespace dart::dynamics::detail

#endif // DART_DYNAMICS_DETAIL_BODY_NODE_POOL_HPP_
