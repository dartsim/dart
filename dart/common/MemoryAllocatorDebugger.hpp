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

#ifndef DART_COMMON_MEMORYALLOCATORDEBUGGER_HPP_
#define DART_COMMON_MEMORYALLOCATORDEBUGGER_HPP_

#include "dart/common/MemoryAllocator.hpp"

#include <iostream>
#include <mutex>
#include <unordered_map>

namespace dart::common {

template <typename T>
class MemoryAllocatorDebugger : public MemoryAllocator
{
public:
  /// Constructor
  template <typename... Args>
  MemoryAllocatorDebugger(Args&&... args);

  /// Destructor
  ~MemoryAllocatorDebugger();

  /// Returns type string.
  [[nodiscard]] static const std::string& getStaticType();

  // Documentation inherited
  [[nodiscard]] const std::string& getType() const override;

  // Documentation inherited
  [[nodiscard]] void* allocate(size_t bytes) noexcept override;

  // Documentation inherited
  void deallocate(void* pointer, size_t bytes) override;

  /// Returns true if there is no memory allocated by the internal allocator.
  [[nodiscard]] bool isEmpty() const;

  /// Returns true if a pointer is allocated by the internal allocator.
  [[nodiscard]] bool hasAllocated(void* pointer, size_t size) const;

  /// Returns the internal allocator
  [[nodiscard]] const T& getInternalAllocator() const;

  /// Returns the internal allocator
  [[nodiscard]] T& getInternalAllocator();

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;

private:
  T mInternalAllocator;

  size_t mSize = 0;

  size_t mPeak = 0;

  std::unordered_map<void*, size_t> mMapPointerToSize;

  mutable std::mutex mMutex;
};

} // namespace dart::common

#include "dart/common/detail/MemoryAllocatorDebugger-impl.hpp"

#endif // DART_COMMON_MEMORYALLOCATORDEBUGGER_HPP_
