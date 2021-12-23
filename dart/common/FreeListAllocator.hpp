/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COMMON_FREELISTALLOCATOR_HPP_
#define DART_COMMON_FREELISTALLOCATOR_HPP_

#include <mutex>
#ifndef NDEBUG
  #include <unordered_map>
#endif

#include "dart/common/MemoryAllocator.hpp"

namespace dart::common {

/// Most general heap memory allocator for allocating memory of various sizes.
///
/// This allocator preallocates a large chunk of contiguous memory on
/// construction, using the base memory allocator and returns the fraction of
/// the allocated memories by request. This way the class controls the timing
/// and frequency of dynamic memory allocations to minimize the performance hit
/// by dynamic memory allocation by the system.
///
/// As the name saying, this class manages the information of which part of the
/// preallocated memory is actually used using the free list data structure.
///
/// If the preallocated memory is all used up, then this class allocates
/// additional memory chunck using the base allocator.
class FreeListAllocator : public MemoryAllocator
{
public:
  /// Constructor
  ///
  /// \param[in] baseAllocator: (optional) Base memory allocator.
  /// \param[in] initialAllocation: (optional) Bytes to initially allocate.
  explicit FreeListAllocator(
      MemoryAllocator& baseAllocator = MemoryAllocator::GetDefault(),
      size_t initialAllocation = 1048576 /* 1 MB */);

  /// Destructor
  ~FreeListAllocator() override;

  DART_STRING_TYPE(FreeListAllocator);

  // Documentation inherited
  [[nodiscard]] void* allocate(size_t size) noexcept override;

  // Documentation inherited
  void deallocate(void* pointer, size_t size) override;

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

private:
  struct MemoryBlockHeader
  {
    /// Memory block size in bytes
    size_t mSize;

    /// Pointer to previous memory block
    MemoryBlockHeader* mPrev;

    /// Pointer to next memory block
    MemoryBlockHeader* mNext;

    /// Whether this block is used
    bool mIsAllocated;

    /// Whether the next memory block is contiguous
    bool mIsNextContiguous;

    /// Constructor
    explicit MemoryBlockHeader(
        size_t size,
        MemoryBlockHeader* prev,
        MemoryBlockHeader* next,
        bool isNextContiguous);

    /// Casts to size_t
    size_t asSizeT() const;

    /// Casts to unsigned char*
    unsigned char* asCharPtr();

    /// Casts to const unsigned char*
    const unsigned char* asCharPtr() const;

    /// Splits the memory block
    void split(size_t sizeToSplit);

    /// Merges this memory block with the given memory block
    void merge(MemoryBlockHeader* other);

#ifndef NDEBUG
    /// Returns whether this memory block is valid
    bool isValid() const;
#endif
  };

  /// Allocates
  bool allocateMemoryBlock(size_t sizeToAllocate);

  /// The base allocator
  MemoryAllocator& mBaseAllocator;

  /// Mutex for private variables except the base allocator
  mutable std::mutex mMutex;

  /// Pointer to the first memory block
  MemoryBlockHeader* mBlockHead{nullptr};

  /// Pointer to the current free memory block
  MemoryBlockHeader* mFreeBlock{nullptr};

  /// The allocated size in bytes
  size_t mAllocatedSize{0};

#ifndef NDEBUG
private:
  size_t mSize = 0;
  size_t mPeak = 0;
  std::unordered_map<void*, size_t> mMapPointerToSize;
#endif
};

} // namespace dart::common

#endif // DART_COMMON_FREELISTALLOCATOR_HPP_
