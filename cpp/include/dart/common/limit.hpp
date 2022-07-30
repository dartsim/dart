/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#include <limits>

namespace dart::common {

/// Zero for type T.
template <typename T = double>
constexpr T zero()
{
  return 0;
}

/// One for type T.
template <typename T = double>
constexpr T one()
{
  return 1;
}

/// Returns epsilon value.
template <typename S = double>
constexpr S epsilon()
{
  return std::numeric_limits<S>::epsilon();
}

/// Returns the smallest finite value close to zero
template <typename S = double>
static constexpr S min()
{
  return std::numeric_limits<S>::min();
}

/// Returns the lowest finite value
template <typename S = double>
static constexpr S lowest()
{
  return std::numeric_limits<S>::lowest();
}

/// Returns the largest finite value
template <typename S = double>
static constexpr S max()
{
  return std::numeric_limits<S>::max();
}

/// Returns the positive infinity value
template <typename S = double>
static constexpr S inf()
{
  static_assert(
      std::numeric_limits<S>::has_infinity,
      "Infinity is not defined for this type.");
  return std::numeric_limits<S>::infinity();
}

template <typename S>
constexpr bool has_inf()
{
  return std::numeric_limits<S>::has_infinity;
}

/// Returns infinity value.
template <typename S = double>
constexpr S nan()
{
  return std::numeric_limits<S>::quiet_NaN();
}

template <typename S>
constexpr bool has_nan()
{
  return std::numeric_limits<S>::has_quiet_NaN;
}

} // namespace dart::common
