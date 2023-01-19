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

#include <memory>
#include <string>

#include <cstddef>

namespace dart::common {

/// Resource provides file-like access to a resource loaded from URI.
///
/// It is expected that each @a ResourceRetriever will provide a concrete
/// instantiation of the Resource class. This interface exposes a similar API
/// to that of the standard C file manipulation functions.
class DART_COMMON_API Resource
{
public:
  /// Position to seek relative to.
  enum class SeekType
  {
    CUR, ///< Current position.
    END, ///< End of file.
    SET  ///< Begining of file.
  };

  virtual ~Resource() = default;

  /// Returns the size of the resource, in bytes.
  virtual std::size_t getSize() = 0;

  /// Returns the current value of the position indicator.
  /// @note This method has the same API as the standard ftell function.
  virtual std::size_t tell() = 0;

  /// Sets the position indicator to a new position.
  /// @param[in] offset Offset, in bytes, relative to origin.
  /// @param[in] origin Position used as the reference of offset.
  /// @note This method has the same API as the standard fseek function.
  virtual bool seek(ptrdiff_t offset, SeekType origin) = 0;

  /// Reads count element, each of size size, into buffer.
  /// @param[out] buffer Pointer to a block of memory with a size of at least
  /// (size * count) bytes.
  /// @param[in] size Size, in bytes, of each element.
  /// @param[in] count Number of elements, each of size bytes.
  /// @note This method has the same API as the standard fread function.
  virtual std::size_t read(void* buffer, std::size_t size, std::size_t count)
      = 0;

  /// Reads all data from this resource, and returns it as a string.
  ///
  /// @return The string retrieved from the resource.
  /// @throw std::runtime_error when failed to read sucessfully.
  virtual std::string readAll();
};

using ResourcePtr = std::shared_ptr<Resource>;

} // namespace dart::common
