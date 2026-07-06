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

#ifndef DART_IO_MJCF_DETAIL_ERROR_HPP_
#define DART_IO_MJCF_DETAIL_ERROR_HPP_

#include <dart/io/export.hpp>

#include <algorithm>
#include <concepts>
#include <iterator>
#include <ranges>
#include <string>
#include <type_traits>
#include <vector>

namespace dart {
namespace io {
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

  ELEMENT_UNSUPPORTED,

  UNDEFINED_ERROR,
};

class DART_IO_API Error final
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

template <typename Range>
concept ErrorRange = std::ranges::input_range<Range>
                     && std::same_as<
                         std::remove_cvref_t<std::ranges::range_value_t<Range>>,
                         Error>;

template <ErrorRange Range>
void appendErrorRange(Errors& errors, Range&& newErrors)
{
  if constexpr (std::ranges::sized_range<Range>) {
    errors.reserve(errors.size() + std::ranges::size(newErrors));
  }

  if constexpr (
      std::is_rvalue_reference_v<Range&&>
      && !std::is_const_v<std::remove_reference_t<Range>>) {
    std::ranges::move(newErrors, std::back_inserter(errors));
  } else {
    std::ranges::copy(newErrors, std::back_inserter(errors));
  }
}

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_ERROR_HPP_
