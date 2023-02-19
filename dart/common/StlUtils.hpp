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

#include <dart/common/Memory.hpp>
#include <dart/common/detail/StlUtils.hpp>

#include <vector>

#include <cassert>
#include <cstddef>

namespace dart::common {

/// Generates a sequence of integer values from 0 (inclusive) to end
/// (exclusive), with a step size of 1.
///
/// @code
///   for (int i : Range(5)) {
///     std::cout << i << std::endl;
///   } // Prints "0", "1", "2", "3", "4"
/// @endcode
///
/// @param end The end of the sequence.
/// @return An instance of `RangeProxy` for the input end.
[[nodiscard]] inline auto Range(int end);

/// Generates a sequence of integer values from start (inclusive) to end
/// (exclusive), with a step size of step.
///
/// @code
///   for (int i : Range(0, 10, 2)) {
///     std::cout << i << std::endl;
///   } // Prints "0", "2", "4", "6", "8"
/// @endcode
///
/// @param start The start of the sequence.
/// @param end The end of the sequence.
/// @param step The step size of the sequence.
/// @return An instance of `RangeProxy` for the input start, end, and step.
[[nodiscard]] inline auto Range(int start, int end, int step = 1);

/// Enumerate the elements in a container.
///
/// @code
///   std::vector<int> v = {1, 2, 3, 4, 5};
///   for (auto [i, x] : Enumerate(v)) {
///     std::cout << i << ": " << x << std::endl;
///   } // Prints "0: 1", "1: 2", "2: 3", "3: 4", "4: 5"
/// @endcode
///
/// @tparam ContainerT The type of the container to be enumerated.
/// @param a The container to be enumerated.
/// @param start The starting index of the enumeration.
/// @return An instance of `EnumerateProxy` for the input container.
template <typename ContainerT>
[[nodiscard]] auto Enumerate(ContainerT&& a, size_t start = 0);

/// Returns an object from a vector if the index is valid, otherwise return
/// nullptr.
///
/// @param[in] index Index of the object to be returned.
/// @param[in] vec Vector of objects.
/// @return Object at the given index if the index is valid, otherwise return
/// nullptr.
template <typename T>
T getVectorObjectIfAvailable(std::size_t index, const std::vector<T>& vec);

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

namespace dart::common {

//==============================================================================
inline auto Range(int end)
{
  return detail::RangeImpl<int>(0, end, 1);
}

//==============================================================================
inline auto Range(int start, int end, int step)
{
  return detail::RangeImpl<int>(start, end, step);
}

//==============================================================================
template <typename ContainerT>
auto Enumerate(ContainerT&& a, size_t start)
{
  return detail::EnumerateImpl<ContainerT>(std::forward<ContainerT>(a), start);
}

//==============================================================================
template <typename T>
T getVectorObjectIfAvailable(std::size_t index, const std::vector<T>& vec)
{
  assert(index < vec.size());
  if (index < vec.size())
    return vec[index];

  return nullptr;
}

} // namespace dart::common
