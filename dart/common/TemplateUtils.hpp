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

#if defined(_MSC_VER) || defined(__apple_build_version__)
  #include <array>
#endif
#include <iterator>
#include <type_traits>

// DETAIL_DART_CREATE_MEMBER_CHECK is a macro that can be used to check if a
// given type has a specific member (specified by the member parameter). The
// macro generates several structs and a constant expression that can be used to
// check for the presence of the member.
//
// The struct Alias_name<T, std::true_type> is defined for any type T that has a
// member member. The struct has a static member value of type
// decltype(&T::member).
//
// The struct has_member_name<T> is defined for any type T and has a static
// member value of type bool that is true if T has a member member and false
// otherwise. The has_member_name<T>::value is determined by using
// dart::common::detail::has_member with
// Alias_name<dart::common::detail::ambiguate<T, AmbiguitySeed_name>> and
// Alias_name<AmbiguitySeed_member>.
//
// A constant expression has_member_name_v<T> is also defined for any type T
// that has the same value as has_member_name<T>::value.
#define DART_CREATE_MEMBER_CHECK(name) DETAIL_DART_CREATE_MEMBER_CHECK(name)

namespace dart::common {

/// Determines if the iterator category of the given type `T` is a random access
/// iterator, which is a requirement for contiguous containers.
///
/// @tparam T The type to check.
template <typename T>
struct is_contiguous_iterator
{
  static constexpr bool value = std::is_same<
      typename std::iterator_traits<T>::iterator_category,
      std::random_access_iterator_tag>::value;
};

/// A convenience variable template that returns `true` if the given type `T`
/// satisfies `is_contiguous_iterator<T>`.
template <typename T>
constexpr bool is_contiguous_iterator_v = is_contiguous_iterator<T>::value;

/// A type trait that determines whether the given type `T` is a contiguous
/// iterator that is both non-const and has non-void `value_type`.
///
/// @tparam T The type to check.
template <typename T>
struct is_const_contiguous_iterator
{
  static constexpr bool value
      = is_contiguous_iterator_v<
            T> && std::is_const_v<typename std::remove_reference<decltype(*std::declval<T>())>::type>;
};

/// A convenience variable template that returns `true` if the given type `T`
/// satisfies `is_const_contiguous_iterator<T>`.
template <typename T>
constexpr bool is_const_contiguous_iterator_v
    = is_const_contiguous_iterator<T>::value;

/// A type trait that determines whether the given type `T` is a contiguous
/// iterator that is const and has non-void `value_type`.
///
/// @tparam T The type to check.
template <typename T>
struct is_non_const_contiguous_iterator
{
  static constexpr bool value
      = is_contiguous_iterator_v<
            T> && !std::is_const_v<typename std::remove_reference<decltype(*std::declval<T>())>::type>;
};

/// A convenience variable template that returns `true` if the given type `T`
/// satisfies `is_non_const_contiguous_iterator<T>`.
template <typename T>
constexpr bool is_non_const_contiguous_iterator_v
    = is_non_const_contiguous_iterator<T>::value;

/// A type trait that determines whether the given type `Container` is a
/// contiguous container, i.e., a container that has a member function `data`
/// that returns a pointer to its contiguous memory buffer.
///
/// @tparam Container The type to check.
/// @tparam Void A dummy parameter to enable SFINAE.
template <typename Container, typename Enable = void>
struct is_contiguous_container : std::false_type
{
  // Empty
};

#if defined(_MSC_VER) || defined(__apple_build_version__)

/// Specializtion of is_contiguous_container for std::array for MSVC
template <typename T, std::size_t N>
struct is_contiguous_container<std::array<T, N>> : std::true_type
{
  // Empty
};

#endif

/// A type trait that determines whether the given type `Container` is a
/// contiguous container, i.e., a container that has a member function `data`
/// that returns a pointer to its contiguous memory buffer.
///
/// @tparam Container The type to check.
/// @tparam Void A dummy parameter to enable SFINAE.
template <typename Container>
struct is_contiguous_container<
    Container,
    std::void_t<decltype(std::declval<Container>().data())>>
  : std::bool_constant<
        std::is_pointer_v<decltype(std::declval<Container>().data())>
        && std::is_same_v<
            typename std::remove_pointer_t<
                decltype(std::declval<Container>().data())>,
            typename Container::value_type>>
{
  // Empty
};

/// A convenience variable template that returns `true` if the given type `T`
/// satisfies `is_contiguous_container<T>`.
template <typename T>
constexpr bool is_contiguous_container_v = is_contiguous_container<T>::value;

} // namespace dart::common

#include <dart/common/detail/TemplateUtils-impl.hpp>
