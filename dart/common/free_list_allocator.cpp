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

#include "dart/common/free_list_allocator.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"

#include <limits>
#include <memory>

#include <cstddef>
#include <cstdint>

namespace dart::common {

namespace {

bool isPowerOfTwo(size_t value)
{
  return value != 0 && (value & (value - 1)) == 0;
}

bool roundUpToAlignment(size_t bytes, size_t alignment, size_t& rounded)
{
  const size_t mask = alignment - 1;
  if (bytes > std::numeric_limits<size_t>::max() - mask) {
    return false;
  }

  rounded = (bytes + mask) & ~mask;
  return true;
}

bool checkedAdd(size_t lhs, size_t rhs, size_t& result)
{
  if (lhs > std::numeric_limits<size_t>::max() - rhs) {
    return false;
  }

  result = lhs + rhs;
  return true;
}

constexpr size_t kAlignedAllocationMagic = 0xDA771A11u;

} // namespace

//==============================================================================
FreeListAllocator::FreeListAllocator(
    MemoryAllocator& baseAllocator,
    size_t initialAllocation,
    GrowthPolicy growthPolicy)
  : mBaseAllocator(baseAllocator), mGrowthPolicy(growthPolicy)
{
  // Avoid bookkeeping-vector growth on the first few hot-path block expansions.
  mAllocatedBlocks.reserve(8);
  allocateMemoryBlock(initialAllocation);
}

//==============================================================================
FreeListAllocator::~FreeListAllocator()
{
  // If there are outstanding allocations, just log the leak and release the
  // backing chunks wholesale. The internal bookkeeping structures may be
  // inconsistent at this point, so we avoid walking them while deallocating.
  if (mTotalAllocatedSize != 0) {
    DART_ERROR(
        "Forcefully deallocated memory {} of byte(s) that is not deallocated "
        "before destructing this memory allocator.",
        mTotalAllocatedSize);
  } else {
    // Deallocate memory blocks when everything was properly released.
    MemoryBlockHeader* curr = mFirstMemoryBlock;
    while (curr) {
      DART_ASSERT(!curr->mIsAllocated); // pointers should already be freed
      MemoryBlockHeader* next = curr->mNext;

      std::destroy_at(curr);
      curr = next;
    }
  }

  for (const auto& block : mAllocatedBlocks) {
    if (block.alignment <= alignof(std::max_align_t)) {
      mBaseAllocator.deallocate(block.pointer, block.size);
    } else {
      mBaseAllocator.deallocate(block.pointer, block.size, block.alignment);
    }
  }
  mAllocatedBlocks.clear();
}

//==============================================================================
const MemoryAllocator& FreeListAllocator::getBaseAllocator() const
{
  return mBaseAllocator;
}

//==============================================================================
MemoryAllocator& FreeListAllocator::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
FreeListAllocator::GrowthPolicy FreeListAllocator::getGrowthPolicy() const
{
  return mGrowthPolicy;
}

//==============================================================================
void* FreeListAllocator::allocate(size_t bytes) noexcept
{
  // Not allowed to allocate zero bytes
  if (bytes == 0) {
    return nullptr;
  }
  const size_t requestedBytes = bytes;

  size_t roundedBytes = 0;
  if (!roundUpToAlignment(bytes, alignof(std::max_align_t), roundedBytes)) {
    return nullptr;
  }
  bytes = roundedBytes;

  if (mFirstMemoryBlock == nullptr) {
    if (mGrowthPolicy == GrowthPolicy::FixedCapacity) {
      return nullptr;
    }

    if (!allocateMemoryBlock(bytes)) {
      return nullptr;
    }
  }

  DART_ASSERT(mFirstMemoryBlock != nullptr);
  // Ensure that the first memory block doesn't have the previous block
  DART_ASSERT(mFirstMemoryBlock->mPrev == nullptr);

  // Iterate from the first memory block
  MemoryBlockHeader* curr = mFirstMemoryBlock;

  // Use free block if available
  if (mFreeBlock) {
    // Ensure the free block is not in use
    DART_ASSERT(!mFreeBlock->mIsAllocated);

    // Use the free block if the requested size is equal to or smaller than
    // the free block
    if (bytes <= mFreeBlock->mSize) {
      curr = mFreeBlock;
      mFreeBlock = nullptr;
    }
  }

  // Search for a memory block that is not used and has sufficient free space
  while (curr) {
    if (!curr->mIsAllocated && bytes <= curr->mSize) {
      curr->split(bytes);
      break;
    }

    curr = curr->mNext;
  }

  // If failed to find an available memory block, allocate a new memory block
  if (curr == nullptr) {
    if (mGrowthPolicy == GrowthPolicy::FixedCapacity) {
      return nullptr;
    }

    // Allocate a sufficient size
    size_t requiredSize = 0;
    size_t nextBlockSize = 0;
    if (!checkedAdd(mTotalAllocatedBlockSize, bytes, requiredSize)
        || !checkedAdd(requiredSize, requiredSize, nextBlockSize)
        || !allocateMemoryBlock(nextBlockSize)) {
      return nullptr;
    }

    DART_ASSERT(mFreeBlock != nullptr);
    DART_ASSERT(!mFreeBlock->mIsAllocated);

    curr = mFreeBlock;
    DART_ASSERT(curr->mSize >= bytes);

    // Split the new memory block for the requested size
    curr->split(bytes);
  }

  // Mark the current block is allocated
  curr->mIsAllocated = true;

  // Set free block if the next block is free
  if (curr->mNext != nullptr && !curr->mNext->mIsAllocated) {
    mFreeBlock = curr->mNext;
  }

  mTotalAllocatedSize += bytes;
  recordAllocation(requestedBytes);

  return static_cast<void*>(curr->asCharPtr() + sizeof(MemoryBlockHeader));
}

//==============================================================================
void* FreeListAllocator::allocate(size_t bytes, size_t alignment) noexcept
{
  if (bytes == 0 || !isPowerOfTwo(alignment)) {
    return nullptr;
  }

  if (alignment <= alignof(std::max_align_t)) {
    return allocate(bytes);
  }

  return allocateFromReservedBlockAligned(bytes, alignment);
}

//==============================================================================
void* FreeListAllocator::allocateFromReservedBlockAligned(
    size_t bytes, size_t alignment) noexcept
{
  // Large component/storage pages are often power-of-two sized. Rotating
  // cache-line starts avoids mapping consecutive arrays to identical L1 sets.
  const bool colorLargeCacheAlignedAllocation
      = alignment == 64 && bytes >= 2048;
  size_t allocationSize = 0;
  if (!checkedAdd(bytes, sizeof(AlignedAllocationHeader), allocationSize)
      || !checkedAdd(allocationSize, alignment - 1, allocationSize)
      || (colorLargeCacheAlignedAllocation
          && !checkedAdd(allocationSize, 7 * alignment, allocationSize))) {
    return nullptr;
  }

  const size_t diagnosticAllocatedSize = mDiagnosticAllocatedSize;
  const size_t diagnosticPeakAllocatedSize = mDiagnosticPeakAllocatedSize;
  const size_t diagnosticAllocationCount = mDiagnosticAllocationCount;
  auto* allocation = static_cast<unsigned char*>(allocate(allocationSize));
  if (allocation == nullptr) {
    return nullptr;
  }
  mDiagnosticAllocatedSize = diagnosticAllocatedSize;
  mDiagnosticPeakAllocatedSize = diagnosticPeakAllocatedSize;
  mDiagnosticAllocationCount = diagnosticAllocationCount;

  const auto payloadBegin = reinterpret_cast<uintptr_t>(allocation)
                            + sizeof(AlignedAllocationHeader);
  auto alignedPayload
      = (payloadBegin + alignment - 1) & ~(uintptr_t{alignment} - 1);
  if (colorLargeCacheAlignedAllocation) {
    const auto colorPadding
        = static_cast<uintptr_t>(mLargeAlignedAllocationColor) * alignment;
    if (alignedPayload > std::numeric_limits<uintptr_t>::max() - colorPadding) {
      deallocate(allocation, allocationSize);
      mDiagnosticAllocatedSize = diagnosticAllocatedSize;
      mDiagnosticPeakAllocatedSize = diagnosticPeakAllocatedSize;
      mDiagnosticAllocationCount = diagnosticAllocationCount;
      return nullptr;
    }
    alignedPayload += colorPadding;
    mLargeAlignedAllocationColor
        = static_cast<std::uint8_t>((mLargeAlignedAllocationColor + 1u) & 7u);
  }
  auto* header = reinterpret_cast<AlignedAllocationHeader*>(
      alignedPayload - sizeof(AlignedAllocationHeader));
  std::construct_at(
      header,
      AlignedAllocationHeader{
          kAlignedAllocationMagic,
          allocation,
          allocationSize,
          bytes,
          alignment});
  recordAllocation(bytes);

  return reinterpret_cast<void*>(alignedPayload);
}

//==============================================================================
void FreeListAllocator::deallocate(void* pointer, size_t bytes)
{
  DART_UNUSED(bytes, pointer);

  // Cannot deallocate nullptr or zero bytes
  if (pointer == nullptr || bytes == 0) {
    return;
  }
  const size_t requestedBytes = bytes;

  size_t roundedBytes = 0;
  if (!roundUpToAlignment(bytes, alignof(std::max_align_t), roundedBytes)) {
    DART_ASSERT(false);
    return;
  }
  bytes = roundedBytes;

  unsigned char* block_addr
      = static_cast<unsigned char*>(pointer) - sizeof(MemoryBlockHeader);
  MemoryBlockHeader* block = reinterpret_cast<MemoryBlockHeader*>(block_addr);
  DART_ASSERT(block->mIsAllocated);
  block->mIsAllocated = false;

  MemoryBlockHeader* curr = block;

  if (block->mPrev != nullptr && !block->mPrev->mIsAllocated
      && block->mPrev->mIsNextContiguous) {
    curr = block->mPrev;
    block->mPrev->merge(block);
  }

  if (curr->mNext != nullptr && !curr->mNext->mIsAllocated
      && curr->mIsNextContiguous) {
    curr->merge(curr->mNext);
  }

  mFreeBlock = curr;

  mTotalAllocatedSize -= bytes;
  recordDeallocation(requestedBytes);

  DART_TRACE("Deallocated {} bytes.", bytes);
}

//==============================================================================
void FreeListAllocator::deallocate(
    void* pointer, size_t bytes, size_t alignment)
{
  if (pointer == nullptr || bytes == 0) {
    return;
  }

  if (alignment <= alignof(std::max_align_t)) {
    deallocate(pointer, bytes);
    return;
  }

  if (releaseReservedAlignedAllocation(pointer, bytes, alignment)) {
    return;
  }

  DART_ASSERT(false);
}

//==============================================================================
bool FreeListAllocator::allocateMemoryBlock(size_t sizeToAllocate)
{
  size_t totalAllocatedBlockSize = 0;
  if (!checkedAdd(
          mTotalAllocatedBlockSize, sizeToAllocate, totalAllocatedBlockSize)) {
    return false;
  }

  size_t allocationSize = 0;
  if (!checkedAdd(sizeToAllocate, sizeof(MemoryBlockHeader), allocationSize)) {
    return false;
  }

  // Allocate memory chunk for header and the actual requested size
  void* memory = mBaseAllocator.allocate(allocationSize);

  // Return false if failed to allocate
  if (memory == nullptr) {
    return false;
  }

  // Construct the memory block header, linking the current block as the next
  // block
  mFirstMemoryBlock = mBaseAllocator.constructAt<MemoryBlockHeader>(
      memory,            // address to construct
      sizeToAllocate,    // size of the memory block
      nullptr,           // previous memory block
      mFirstMemoryBlock, // next memory block
      false              // whether the next memory block is contiguous
  );

  mAllocatedBlocks.push_back(
      AllocatedBlock{memory, allocationSize, alignof(std::max_align_t)});

  // Set the new memory block as free block
  mFreeBlock = mFirstMemoryBlock;

  // Update the allocated size (without memory size for the headers of blocks)
  mTotalAllocatedBlockSize = totalAllocatedBlockSize;

  return true;
}

//==============================================================================
bool FreeListAllocator::releaseReservedAlignedAllocation(
    void* pointer, size_t bytes, size_t alignment)
{
  DART_UNUSED(bytes, alignment);

  auto* header = reinterpret_cast<AlignedAllocationHeader*>(
      static_cast<unsigned char*>(pointer) - sizeof(AlignedAllocationHeader));
  if (header->magic != kAlignedAllocationMagic) {
    return false;
  }

  DART_ASSERT(header->requestedSize == bytes);
  DART_ASSERT(header->alignment == alignment);

  void* allocationPointer = header->allocationPointer;
  const size_t allocationSize = header->allocationSize;
  const size_t diagnosticAllocatedSize = mDiagnosticAllocatedSize;
  const size_t diagnosticPeakAllocatedSize = mDiagnosticPeakAllocatedSize;
  const size_t diagnosticAllocationCount = mDiagnosticAllocationCount;
  std::destroy_at(header);
  deallocate(allocationPointer, allocationSize);
  mDiagnosticAllocatedSize
      = bytes <= diagnosticAllocatedSize ? diagnosticAllocatedSize - bytes : 0;
  mDiagnosticPeakAllocatedSize = diagnosticPeakAllocatedSize;
  mDiagnosticAllocationCount
      = diagnosticAllocationCount > 0 ? diagnosticAllocationCount - 1 : 0;

  return true;
}

//==============================================================================
size_t FreeListAllocator::getAllocatedSize() const
{
  return mDiagnosticAllocatedSize;
}

//==============================================================================
size_t FreeListAllocator::getPeakAllocatedSize() const
{
  return mDiagnosticPeakAllocatedSize;
}

//==============================================================================
size_t FreeListAllocator::getAllocationCount() const
{
  return mDiagnosticAllocationCount;
}

//==============================================================================
void FreeListAllocator::recordAllocation(size_t bytes) noexcept
{
  mDiagnosticAllocatedSize += bytes;
  if (mDiagnosticPeakAllocatedSize < mDiagnosticAllocatedSize) {
    mDiagnosticPeakAllocatedSize = mDiagnosticAllocatedSize;
  }
  ++mDiagnosticAllocationCount;
}

//==============================================================================
void FreeListAllocator::recordDeallocation(size_t bytes) noexcept
{
  if (bytes <= mDiagnosticAllocatedSize) {
    mDiagnosticAllocatedSize -= bytes;
  } else {
    mDiagnosticAllocatedSize = 0;
  }

  if (mDiagnosticAllocationCount > 0) {
    --mDiagnosticAllocationCount;
  }
}

//==============================================================================
void FreeListAllocator::print(std::ostream& os, int indent) const
{
  if (indent == 0) {
    os << "[FreeListAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0) {
    os << spaces << "type: " << getType() << "\n";
  }
  os << spaces << "growth_policy: "
     << (mGrowthPolicy == GrowthPolicy::Expand ? "expand" : "fixed_capacity")
     << "\n";
  os << spaces << "reserved_size: " << mTotalAllocatedBlockSize << "\n";
  os << spaces << "memory_blocks:\n";
  auto curr = mFirstMemoryBlock;
  while (curr) {
    os << spaces << "- block_addr: " << curr << "\n";
    os << spaces << "  size: " << curr->mSize << "\n";
    os << spaces << "  prev: " << curr->mPrev << "\n";
    os << spaces << "  next: " << curr->mNext << "\n";
    os << spaces << "  is_allocated: " << curr->mIsAllocated << "\n";
    os << spaces << "  is_next_contiguous: " << curr->mIsNextContiguous << "\n";
    curr = curr->mNext;
  }
  os << spaces << "free_block_addr: " << mFreeBlock << "\n";
  os << spaces << "header_size: " << sizeof(MemoryBlockHeader) << "\n";
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

//==============================================================================
FreeListAllocator::MemoryBlockHeader::MemoryBlockHeader(
    size_t size,
    MemoryBlockHeader* prev,
    MemoryBlockHeader* next,
    bool isNextContiguous)
  : mSize(size),
    mPrev(prev),
    mNext(next),
    mIsAllocated(false),
    mIsNextContiguous(isNextContiguous)
{
  if (prev) {
    prev->mNext = this;
  }
}

//==============================================================================
size_t FreeListAllocator::MemoryBlockHeader::asSizeT() const
{
  return reinterpret_cast<size_t>(this);
}

//==============================================================================
unsigned char* FreeListAllocator::MemoryBlockHeader::asCharPtr()
{
  return reinterpret_cast<unsigned char*>(this);
}

//==============================================================================
const unsigned char* FreeListAllocator::MemoryBlockHeader::asCharPtr() const
{
  return reinterpret_cast<const unsigned char*>(this);
}

//==============================================================================
void FreeListAllocator::MemoryBlockHeader::split(size_t sizeToSplit)
{
  DART_ASSERT(sizeToSplit <= mSize);
  DART_ASSERT(!mIsAllocated);

  if (sizeToSplit + sizeof(MemoryBlockHeader) >= mSize) {
    // TODO(JS): Treat this as en error?
    return;
  }

  DART_ASSERT(mSize > sizeToSplit);

  unsigned char* new_block_addr
      = asCharPtr() + sizeof(MemoryBlockHeader) + sizeToSplit;
  MemoryBlockHeader* new_block = std::construct_at(
      reinterpret_cast<MemoryBlockHeader*>(new_block_addr),
      mSize - sizeof(MemoryBlockHeader) - sizeToSplit,
      this,
      mNext,
      mIsNextContiguous);
  mNext = new_block;
  if (new_block->mNext) {
    new_block->mNext->mPrev = new_block;
  }
  DART_ASSERT(mNext != this);
  mIsNextContiguous = true;
  mSize = sizeToSplit;

  DART_ASSERT(isValid());
  DART_ASSERT(new_block->isValid());
}

//==============================================================================
void FreeListAllocator::MemoryBlockHeader::merge(MemoryBlockHeader* other)
{
  DART_ASSERT(other->mPrev == this);
  DART_ASSERT(other->mNext != this);
  DART_ASSERT(!other->mIsAllocated);
  DART_ASSERT(this->mNext == other);
  DART_ASSERT(!this->mIsAllocated);
  DART_ASSERT(this->mIsNextContiguous);

  mSize += other->mSize + sizeof(MemoryBlockHeader);
  mNext = other->mNext;
  if (other->mNext) {
    other->mNext->mPrev = this;
  }
  mIsNextContiguous = other->mIsNextContiguous;

  std::destroy_at(other); // TODO(JS): Need this?

  DART_ASSERT(isValid());
}

bool FreeListAllocator::MemoryBlockHeader::isValid() const
{
  if (mPrev != nullptr && mPrev->mNext != this) {
    return false;
  }

  if (mNext != nullptr && mNext->mPrev != this) {
    return false;
  }

  return true;
}

} // namespace dart::common
