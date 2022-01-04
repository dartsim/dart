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

#ifndef DART_UTILS_MJCF_DETAIL_ERROR_HPP_
#define DART_UTILS_MJCF_DETAIL_ERROR_HPP_

#include <string>
#include <vector>

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

enum class ErrorCode
{
  NONE = 0,

  FILE_READ,

  DUPLICATE_NAME,

  ATTRIBUTE_MISSING,

  ATTRIBUTE_INVALID,

  ATTRIBUTE_DEPRECATED,

  ATTRIBUTE_CONFLICT,

  ELEMENT_MISSING,

  ELEMENT_INVALID,

  INCORRECT_ELEMENT_TYPE,

  UNDEFINED_ERROR,
};

class Error final
{
public:
  Error() = default;
  Error(ErrorCode code, const std::string& message);

  explicit operator bool() const;

  ErrorCode getCode() const;
  const std::string& getMessage() const;

private:
  ErrorCode mCode{ErrorCode::NONE};
  std::string mMessage{""};
};

using Errors = std::vector<Error>;

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_ERROR_HPP_
