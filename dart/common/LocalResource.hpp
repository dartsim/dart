/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_LOCALRESOURCE_HPP_
#define DART_COMMON_LOCALRESOURCE_HPP_

#include "dart/common/Resource.hpp"

namespace dart {
namespace common {

class LocalResource : public virtual Resource
{
public:
  explicit LocalResource(const std::string& _path);
  virtual ~LocalResource();

  LocalResource(const LocalResource& _other) = delete;
  LocalResource& operator=(const LocalResource& _other) = delete;

  /// Returns true if the resource is open and in a valid state.
  bool isGood() const;

  // Documentation inherited.
  std::size_t getSize() override;

  // Documentation inherited.
  std::size_t tell() override;

  // Documentation inherited.
  bool seek(ptrdiff_t _origin, SeekType _mode) override;

  // Documentation inherited.
  std::size_t read(void* _buffer, std::size_t _size, std::size_t _count) override;

private:
  std::FILE* mFile;
};

} // namespace common
} // namespace dart

#endif // ifndef DART_COMMON_LOCALRESOURCE_HPP_
