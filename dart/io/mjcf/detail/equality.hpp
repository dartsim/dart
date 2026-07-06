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

#ifndef DART_IO_MJCF_DETAIL_EQUALITY_HPP_
#define DART_IO_MJCF_DETAIL_EQUALITY_HPP_

#include <dart/io/export.hpp>
#include <dart/io/mjcf/detail/error.hpp>
#include <dart/io/mjcf/detail/weld.hpp>

#include <tinyxml2.h>

#include <vector>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class DART_IO_API Equality final
{
public:
  Equality() = default;

  std::size_t getNumWelds() const;
  const Weld& getWeld(std::size_t index) const;

  Errors read(tinyxml2::XMLElement* element, const Defaults& defaults);

private:
  friend class MujocoModel;

  std::vector<Weld> mWelds;
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_EQUALITY_HPP_
