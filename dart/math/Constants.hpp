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

namespace dart::math {

/// Returns the maximum value of the given type
template <typename S = double>
constexpr S max();

/// Returns the smallest finite value of the given type
///
/// For floating-point types with denormalization, min() returns the minimum
/// positive normalized value. Note that this behavior may be unexpected,
/// especially when compared to the behavior of min for integral types. To find
/// the value that has no values less than it, use lowest().
template <typename S = double>
constexpr S min();

/// Returns the lowest value of the given type
///
/// This is the same as min() for unsigned types, and the same as -max() for
template <typename S = double>
constexpr S lowest();

/// Returns the epsilon value of the given type
///
/// The epsilon value is the difference between 1 and the next representable.
/// For example, for float, the epsilon value is 1.19209e-07. For double, the
/// epsilon value is 2.22045e-16.
template <typename S = double>
constexpr S eps();

/// Returns the NaN value of the given type
///
/// The NaN value is a special value that is not equal to any other value,
/// including itself. For example, for float, the NaN value is 0x7fc00000. For
/// double, the NaN value is 0x7ff8000000000000.
template <typename S = double>
constexpr S nan();

/// Returns the infinity value of the given type
template <typename S = double>
constexpr S inf();

/// Returns the value of pi
template <typename S = double>
constexpr S pi();

/// Returns the value of pi/2
template <typename S = double>
constexpr S half_pi();

/// Returns the value of pi/4
template <typename S = double>
constexpr S two_pi();

/// Returns the value of phi, golden ratio
template <typename S = double>
constexpr S phi();

} // namespace dart::math

#include <limits>

namespace dart::math {

//==============================================================================
// Implementation
//==============================================================================

//=========================================================================================
template <typename S>
constexpr S max()
{
  return std::numeric_limits<S>::max();
}

//=========================================================================================
template <typename S>
constexpr S min()
{
  return std::numeric_limits<S>::min();
}

//=========================================================================================
template <typename S>
constexpr S lowest()
{
  return std::numeric_limits<S>::lowest();
}

//=========================================================================================
template <typename S>
constexpr S eps()
{
  return std::numeric_limits<S>::epsilon();
}

//=========================================================================================
template <typename S>
constexpr S nan()
{
  return std::numeric_limits<S>::quiet_NaN();
}

//=========================================================================================
template <typename S>
constexpr S inf()
{
  return std::numeric_limits<S>::infinity();
}

//=========================================================================================
template <typename S>
constexpr S pi()
{
  return 3.141592653589793238462643383279502884197169399375105820974944592L;
}

//=========================================================================================
template <typename S>
constexpr S half_pi()
{
  return S(0.5) * pi<S>();
}

//=========================================================================================
template <typename S>
constexpr S two_pi()
{
  return S(2) * pi<S>();
}

//=========================================================================================
template <typename S>
constexpr S phi()
{
  return 1.618033988749894848204586834365638117720309179805762862135448623L;
}

} // namespace dart::math
