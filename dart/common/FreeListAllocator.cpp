/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/common/FreeListAllocator.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"

namespace dart::common {

//==============================================================================
FreeListAllocator::FreeListAllocator(
    MemoryAllocator& baseAllocator, size_t initialAllocation)
  : mBaseAllocator(baseAllocator)
{
  allocateMemoryBlock(initialAllocation);
}

//==============================================================================
FreeListAllocator::~FreeListAllocator()
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  // Forcefully deallocate all the memory blocks if destructing this allocator
  // without deallocating individual memories allocated by this allocator.
  if (mTotalAllocatedSize != 0)
  {
    MemoryBlockHeader* currBlock = mFirstMemoryBlock;
    while (currBlock)
    {
      MemoryBlockHeader* currSubBlock = currBlock;
      MemoryBlockHeader* next = currBlock->mNext;
      size_t sizeToDeallocate = 0;

      while (currSubBlock)
      {
        sizeToDeallocate += currSubBlock->mSize + sizeof(MemoryBlockHeader);
        if (!currSubBlock->mIsNextContiguous)
        {
          next = currSubBlock->mNext;
          break;
        }
        currSubBlock = currSubBlock->mNext;
      }

      mBaseAllocator.deallocate(currBlock, sizeToDeallocate);
      currBlock = next;
    }

    dterr
        << "Forcefully deallocated memory " << mTotalAllocatedSize
        << " of byte(s) that is not deallocated before destructing this memory "
        << "allocator.\n";
    // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
    // destructor is resolved.

    return;
  }

  // Deallocate memory blocks
  MemoryBlockHeader* curr = mFirstMemoryBlock;
  while (curr)
  {
    DART_ASSERT(!curr->mIsAllocated); // TODO(JS): This means some of pointers
    // are not deallocated
    MemoryBlockHeader* next = curr->mNext;
    const auto size = curr->mSize;

    curr->~MemoryBlockHeader();
    mBaseAllocator.deallocate(curr, size + sizeof(MemoryBlockHeader));

    curr = next;
  }
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
void* FreeListAllocator::allocate(size_t bytes) noexcept
{
  // Not allowed to allocate zero bytes
  if (bytes == 0)
  {
    return nullptr;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  // Ensure that the first memory block doesn't have the previous block
  DART_ASSERT(mFirstMemoryBlock->mPrev == nullptr);

  // Iterate from the first memory block
  MemoryBlockHeader* curr = mFirstMemoryBlock;

  // Use free block if available
  if (mFreeBlock)
  {
    // Ensure the free block is not in use
    DART_ASSERT(!mFreeBlock->mIsAllocated);

    // Use the free block if the requested size is equal to or smaller than
    // the free block
    if (bytes <= mFreeBlock->mSize)
    {
      curr = mFreeBlock;
      mFreeBlock = nullptr;
    }
  }

  // Search for a memory block that is not used and has sufficient free space
  while (curr)
  {
    if (!curr->mIsAllocated && bytes <= curr->mSize)
    {
      curr->split(bytes);
      break;
    }

    curr = curr->mNext;
  }

  // If failed to find an avaliable memory block, allocate a new memory block
  if (curr == nullptr)
  {
    // Allocate a sufficient size
    if (!allocateMemoryBlock((mTotalAllocatedBlockSize + bytes) * 2))
    {
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
  if (curr->mNext != nullptr && !curr->mNext->mIsAllocated)
  {
    mFreeBlock = curr->mNext;
  }

  mTotalAllocatedSize += bytes;

  return static_cast<void*>(curr->asCharPtr() + sizeof(MemoryBlockHeader));
}

//==============================================================================
void FreeListAllocator::deallocate(void* pointer, size_t bytes)
{
  DART_UNUSED(bytes, pointer);

  // Cannot deallocate nullptr or zero bytes
  if (pointer == nullptr || bytes == 0)
  {
    return;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  unsigned char* block_addr
      = static_cast<unsigned char*>(pointer) - sizeof(MemoryBlockHeader);
  MemoryBlockHeader* block = reinterpret_cast<MemoryBlockHeader*>(block_addr);
  DART_ASSERT(block->mIsAllocated);
  block->mIsAllocated = false;

  MemoryBlockHeader* curr = block;

  if (block->mPrev != nullptr && !block->mPrev->mIsAllocated
      && block->mPrev->mIsNextContiguous)
  {
    curr = block->mPrev;
    block->mPrev->merge(block);
  }

  if (curr->mNext != nullptr && !curr->mNext->mIsAllocated
      && curr->mIsNextContiguous)
  {
    curr->merge(curr->mNext);
  }

  mFreeBlock = curr;

  mTotalAllocatedSize -= bytes;

  DART_TRACE("Deallocated {} bytes.", bytes);
}

//==============================================================================
bool FreeListAllocator::allocateMemoryBlock(size_t sizeToAllocate)
{
  // Allocate memory chunck for header and the actual requested size
  void* memory
      = mBaseAllocator.allocate(sizeToAllocate + sizeof(MemoryBlockHeader));

  // Return false if failed to allocate
  if (memory == nullptr)
    return false;

  // Construct the memory block header, linking the current block as the next
  // block
  mFirstMemoryBlock = mBaseAllocator.constructAt<MemoryBlockHeader>(
      memory,            // address to construct
      sizeToAllocate,    // size of the memory block
      nullptr,           // previous memory block
      mFirstMemoryBlock, // next memory block
      false              // whether the next memory block is contiguous
  );

  // Set the new memory block as free block
  mFreeBlock = mFirstMemoryBlock;

  // Update the allocated size (without memory size for the headers of blocks)
  mTotalAllocatedBlockSize += sizeToAllocate;

  return true;
}

//==============================================================================
void FreeListAllocator::print(std::ostream& os, int indent) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  if (indent == 0)
  {
    os << "[FreeListAllocator]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0)
  {
    os << spaces << "type: " << getType() << "\n";
  }
  os << spaces << "reserved_size: " << mTotalAllocatedBlockSize << "\n";
  os << spaces << "memory_blocks:\n";
  auto curr = mFirstMemoryBlock;
  while (curr)
  {
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
  if (prev)
  {
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

  if (sizeToSplit + sizeof(MemoryBlockHeader) >= mSize)
  {
    // TODO(JS): Treat this as en error?
    return;
  }

  DART_ASSERT(mSize > sizeToSplit);

  unsigned char* new_block_addr
      = asCharPtr() + sizeof(MemoryBlockHeader) + sizeToSplit;
  MemoryBlockHeader* new_block
      = new (static_cast<void*>(new_block_addr)) MemoryBlockHeader(
          mSize - sizeof(MemoryBlockHeader) - sizeToSplit,
          this,
          mNext,
          mIsNextContiguous);
  mNext = new_block;
  if (new_block->mNext)
  {
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
  if (other->mNext)
  {
    other->mNext->mPrev = this;
  }
  mIsNextContiguous = other->mIsNextContiguous;

  other->~MemoryBlockHeader(); // TODO(JS): Need this?

  DART_ASSERT(isValid());
}

//==============================================================================
#ifndef NDEBUG
bool FreeListAllocator::MemoryBlockHeader::isValid() const
{
  if (mPrev != nullptr && mPrev->mNext != this)
  {
    return false;
  }

  if (mNext != nullptr && mNext->mPrev != this)
  {
    return false;
  }

  return true;
}
#endif

} // namespace dart::common
