/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <type_traits>

#include <fmt/format.h>

#include "dart/common/logging.hpp"
#include "dart/common/string.hpp"

namespace dart::common {

//==============================================================================
template <typename S>
S to_scalar(const std::string& str)
{
  if constexpr (std::is_same_v<S, bool>) {
    return to_bool(str);
  } else if constexpr (std::is_same_v<S, char>) {
    return to_char(str);
  } else if constexpr (std::is_same_v<S, int>) {
    return to_int(str);
  } else if constexpr (std::is_same_v<S, unsigned int>) {
    return to_uint(str);
  } else if constexpr (std::is_same_v<S, long>) {
    return to_long(str);
  } else if constexpr (std::is_same_v<S, long long>) {
    return to_long_long(str);
  } else if constexpr (std::is_same_v<S, float>) {
    return to_float(str);
  } else if constexpr (std::is_same_v<S, double>) {
    return to_double(str);
  } else {
    DART_ERROR("Unsupported scalar type [{}] to convert to.", typeid(S).name());
    return 0;
  }
}

} // namespace dart::common
