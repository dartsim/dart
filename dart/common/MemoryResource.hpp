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

#include <dart/common/ClassWithVirtualBase.hpp>
#include <dart/common/Resource.hpp>

namespace dart::common {

/// MemoryResource provides file-like access to a resource loaded from a string.
///
/// MemoryResource is a concrete implementation of the common::Resource class
/// that is used to access resources loaded from strings. It provides an API
/// similar to that of the standard C file manipulation functions, and can be
/// used to read, seek and retrieve the size of a resource.
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
class DART_COMMON_API MemoryResource : public virtual common::Resource
{
public:
  /// Constructs a MemoryResource object with a given content string.
  /// @param[in] content The content of the resource as a string.
  explicit MemoryResource(const std::string& content);

  /// Returns the size of the resource in bytes.
  [[nodiscard]] std::size_t getSize() override;

  /// Returns the current position of the resource.
  [[nodiscard]] std::size_t tell() override;

  /// Sets the position of the resource to a new position.
  /// @param[in] offset The offset from the origin specified by the mode.
  /// @param[in] mode The SeekType mode determining the reference position.
  /// @return true if the operation was successful, false otherwise.
  bool seek(ptrdiff_t offset, SeekType mode) override;

  /// Reads data from the resource into a buffer.
  /// @param[out] buffer The buffer to store the read data.
  /// @param[in] size The size of each element to be read.
  /// @param[in] count The number of elements to be read.
  /// @return The number of elements successfully read.
  std::size_t read(void* buffer, std::size_t size, std::size_t count) override;

private:
  std::string mContent;
  std::size_t mCursor;
};
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END

} // namespace dart::common
