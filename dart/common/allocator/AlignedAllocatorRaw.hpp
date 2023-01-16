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

#include <dart/common/Fwd.hpp>
#include <dart/common/allocator/AlignedAllocator.hpp>

namespace dart::common {

/// AlignedAllocatorRaw is a stateless memory allocator that uses
/// std::aligned_alloc and std::free for memory allocation and deallocation.
///
/// It is designed to be used in release mode and is derived from the
/// AlignedAllocator base class.
class DART_COMMON_API AlignedAllocatorRaw : public AlignedAllocator
{
public:
  /// Constructor
  AlignedAllocatorRaw() noexcept;

  /// Destructor
  ~AlignedAllocatorRaw() override;

  DART_STRING_TYPE(AlignedAllocatorRaw);

  // Documentation inherited
  [[nodiscard]] void* allocate(
      size_t bytes, size_t alignment) noexcept override;

  // Documentation inherited
  void deallocate(void* pointer, size_t bytes) override;

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const override;
};

} // namespace dart::common
