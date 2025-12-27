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

#ifndef DART_SIMULATION_OBJECT_TYPELIST_HPP_
#define DART_SIMULATION_OBJECT_TYPELIST_HPP_

#include <type_traits>

namespace dart::simulation::object {

template <typename... Types>
struct TypeList
{
  static constexpr std::size_t size = sizeof...(Types);
};

template <typename T, typename List>
struct Contains;

template <typename T, typename... Types>
struct Contains<T, TypeList<Types...>>
  : std::bool_constant<(std::is_same_v<T, Types> || ...)>
{
};

template <typename T, typename List>
inline constexpr bool Contains_v = Contains<T, List>::value;

template <typename List1, typename List2>
struct HasOverlap;

template <typename... Types1, typename... Types2>
struct HasOverlap<TypeList<Types1...>, TypeList<Types2...>>
  : std::bool_constant<(Contains_v<Types1, TypeList<Types2...>> || ...)>
{
};

template <typename List1, typename List2>
inline constexpr bool HasOverlap_v = HasOverlap<List1, List2>::value;

template <typename List1, typename List2>
struct Concat;

template <typename... Types1, typename... Types2>
struct Concat<TypeList<Types1...>, TypeList<Types2...>>
{
  using type = TypeList<Types1..., Types2...>;
};

template <typename List1, typename List2>
using Concat_t = typename Concat<List1, List2>::type;

} // namespace dart::simulation::object

#endif // DART_SIMULATION_OBJECT_TYPELIST_HPP_
