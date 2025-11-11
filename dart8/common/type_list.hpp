/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <type_traits>

namespace dart8 {

//==============================================================================
/// TypeList - compile-time list of types
///
/// Used for declaring component access patterns in EntityObject.
///
/// Example:
/// @code
/// using MyComponents = TypeList<ComponentA, ComponentB, ComponentC>;
/// @endcode
template <typename... Types>
struct TypeList
{
  static constexpr std::size_t size = sizeof...(Types);
};

//==============================================================================
/// Check if a type is in a TypeList
///
/// Example:
/// @code
/// using List = TypeList<int, float, double>;
/// static_assert(Contains<int, List>::value);
/// static_assert(!Contains<char, List>::value);
/// @endcode
template <typename T, typename List>
struct Contains;

template <typename T, typename... Types>
struct Contains<T, TypeList<Types...>>
  : std::bool_constant<(std::is_same_v<T, Types> || ...)>
{
};

template <typename T, typename List>
inline constexpr bool Contains_v = Contains<T, List>::value;

//==============================================================================
/// Check if two TypeLists have any overlapping types
///
/// Example:
/// @code
/// using List1 = TypeList<int, float>;
/// using List2 = TypeList<float, double>;
/// static_assert(HasOverlap<List1, List2>::value);  // float overlaps
/// @endcode
template <typename List1, typename List2>
struct HasOverlap;

template <typename... Types1, typename... Types2>
struct HasOverlap<TypeList<Types1...>, TypeList<Types2...>>
  : std::bool_constant<(Contains_v<Types1, TypeList<Types2...>> || ...)>
{
};

template <typename List1, typename List2>
inline constexpr bool HasOverlap_v = HasOverlap<List1, List2>::value;

//==============================================================================
/// Concatenate two TypeLists
///
/// Example:
/// @code
/// using List1 = TypeList<int, float>;
/// using List2 = TypeList<double, char>;
/// using Combined = Concat<List1, List2>::type;  // TypeList<int, float,
/// double, char>
/// @endcode
template <typename List1, typename List2>
struct Concat;

template <typename... Types1, typename... Types2>
struct Concat<TypeList<Types1...>, TypeList<Types2...>>
{
  using type = TypeList<Types1..., Types2...>;
};

template <typename List1, typename List2>
using Concat_t = typename Concat<List1, List2>::type;

} // namespace dart8
