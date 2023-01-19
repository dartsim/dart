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

/// LocalResource provides file-like access to a resource loaded from a local
/// file path.
///
/// LocalResource is a concrete implementation of the Resource class that is
/// used to access resources located on the local file system. It provides an
/// API similar to that of the standard C file manipulation functions, and can
/// be used to read, seek and retrieve the size of a file.
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
class DART_COMMON_API LocalResource : public virtual Resource
{
public:
  /// Constructs a LocalResource object with a given file path.
  /// @param[in] path The file path of the resource.
  explicit LocalResource(const std::string& path);

  /// Destructs the LocalResource object, closing the resource if it is open.
  virtual ~LocalResource();

  /// Deleted copy constructor to prevent copying of LocalResource objects.
  LocalResource(const LocalResource& other) = delete;

  /// Deleted assignment operator to prevent assignment of LocalResource
  /// objects.
  LocalResource& operator=(const LocalResource& other) = delete;

  /// Checks if the resource is open and in a valid state.
  /// @return true if the resource is open and in a valid state, false
  /// otherwise.
  bool isGood() const;

  /// Returns the size of the resource in bytes.
  std::size_t getSize() override;

  /// Returns the current position of the resource.
  std::size_t tell() override;

  /// Sets the position of the resource to a new position.
  bool seek(ptrdiff_t origin, SeekType mode) override;

  /// Reads data from the resource into a buffer.
  std::size_t read(void* buffer, std::size_t size, std::size_t count) override;

private:
  std::FILE* mFile;
};
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END

} // namespace dart::common
