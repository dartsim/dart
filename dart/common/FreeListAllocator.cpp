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

#ifndef NDEBUG
  if (!mMapPointerToSize.empty())
  {
    size_t totalSize = 0;
    for (auto it : mMapPointerToSize)
    {
      void* pointer = it.first;
      size_t size = it.second;
      totalSize += size;
      dterr << "Found memory leak of " << size << " bytes at " << pointer
            << "\n";
      // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
      // destructor is resolved.
    }
    dterr << "Found potential memory leak of total " << totalSize
          << " bytes!\n";
    // TODO(JS): Change to DART_FATAL once the issue of calling spdlog in
    // destructor is resolved.
  }
#endif

  MemoryBlockHeader* curr = mBlockHead;
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
void* FreeListAllocator::allocate(size_t size) noexcept
{
  DART_UNUSED(size);

  if (size == 0)
  {
    return nullptr;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  MemoryBlockHeader* curr = mBlockHead;
  DART_ASSERT(mBlockHead->mPrev == nullptr);

  if (mFreeBlock)
  {
    DART_ASSERT(!mFreeBlock->mIsAllocated);
    if (size <= mFreeBlock->mSize)
    {
      curr = mFreeBlock;
      mFreeBlock = nullptr;
    }
  }

  while (curr)
  {
    if (!curr->mIsAllocated && size <= curr->mSize)
    {
      curr->split(size);
      break;
    }

    curr = curr->mNext;
  }

  if (curr == nullptr)
  {
    if (!allocateMemoryBlock((mAllocatedSize + size) * 2))
    {
      return nullptr;
    }

    DART_ASSERT(mFreeBlock != nullptr);
    DART_ASSERT(!mFreeBlock->mIsAllocated);

    curr = mFreeBlock;
    DART_ASSERT(curr->mSize >= size);

    curr->split(size);
  }

  curr->mIsAllocated = true;

  if (curr->mNext != nullptr && !curr->mNext->mIsAllocated)
  {
    mFreeBlock = curr->mNext;
  }

#ifndef NDEBUG
  auto out = static_cast<void*>(curr->asCharPtr() + sizeof(MemoryBlockHeader));
  if (out)
  {
    mSize += size;
    mPeak = std::max(mPeak, mSize);
    mMapPointerToSize[out] = size;
  }
  return out;
#else
  return static_cast<void*>(curr->asCharPtr() + sizeof(MemoryBlockHeader));
#endif
}

//==============================================================================
void FreeListAllocator::deallocate(void* pointer, size_t size)
{
  DART_UNUSED(size, pointer);

  if (pointer == nullptr || size == 0)
  {
    return;
  }

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

#ifndef NDEBUG
  auto it = mMapPointerToSize.find(pointer);
  if (it != mMapPointerToSize.end())
  {
    auto allocatedSize = it->second;
    if (size != allocatedSize)
    {
      DART_FATAL(
          "Cannot deallocated memory {} because the deallocating size {} is "
          "different from the allocated size {}.",
          pointer,
          size,
          allocatedSize);
      return;
    }
    mSize -= size;
    mMapPointerToSize.erase(it);
    DART_TRACE("Deallocated {} bytes.", size);
  }
  else
  {
    DART_FATAL(
        "Cannot deallocate memory {} that is not allocated by this allocator!",
        pointer);
    return;
  }
#endif

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
}

//==============================================================================
bool FreeListAllocator::allocateMemoryBlock(size_t sizeToAllocate)
{
  void* memory
      = mBaseAllocator.allocate(sizeToAllocate + sizeof(MemoryBlockHeader));
  if (memory == nullptr)
  {
    return false;
  }

  mBlockHead = mBaseAllocator.constructAt<MemoryBlockHeader>(
      memory, sizeToAllocate, nullptr, mBlockHead, false);

  mFreeBlock = mBlockHead;

  mAllocatedSize += sizeToAllocate;

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
  os << spaces << "reserved_size: " << mAllocatedSize << "\n";
  os << spaces << "memory_blocks:\n";
  auto curr = mBlockHead;
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
  os << spaces << "baseAllocator:\n";
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
