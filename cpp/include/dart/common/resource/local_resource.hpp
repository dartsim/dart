/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#pragma once

#include "dart/common/export.hpp"
#include "dart/common/macro.hpp"
#include "dart/common/resource/resource.hpp"

namespace dart::common {

// DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
class DART_COMMON_API LocalResource : public Resource
{
public:
  explicit LocalResource(const std::string& path);
  ~LocalResource() override;

  LocalResource(const LocalResource& other) = delete;
  LocalResource& operator=(const LocalResource& other) = delete;

  /// Returns true if the resource is open and in a valid state.
  bool is_good() const;

  // Documentation inherited.
  std::size_t get_size() override;

  // Documentation inherited.
  std::size_t tell() override;

  // Documentation inherited.
  bool seek(ptrdiff_t origin, SeekType mode) override;

  // Documentation inherited.
  std::size_t read(void* buffer, std::size_t size, std::size_t count) override;

private:
  std::FILE* m_file;
};
// DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END

} // namespace dart::common
