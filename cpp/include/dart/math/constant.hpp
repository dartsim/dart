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

#include <limits>
#include <type_traits>

namespace dart::math {

/// Returns default epsilon
template <typename S = double>
static constexpr S eps()
{
  if constexpr (std::is_same_v<S, float>) {
    return 1e-4;
  } else if constexpr (std::is_same_v<S, double>) {
    return 1e-8;
  } else if constexpr (std::is_same_v<S, long double>) {
    return 1e-16;
  } else {
    return 0;
  }
}

template <typename S = double>
static constexpr S pi()
{
  return static_cast<S>(
      3.141592653589793238462643383279502884197169399375105820974944592L);
}

template <typename S = double>
static constexpr S phi()
{
  return static_cast<S>(
      1.618033988749894848204586834365638117720309179805762862135448623L);
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
  return std::numeric_limits<S>::infinity();
}

} // namespace dart::math
