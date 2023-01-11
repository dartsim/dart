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

#include <typeindex>
#if defined(__GLIBCXX__)
  #include <map>
  #include <unordered_set>
#else
  #include <map>
  #include <string>
  #include <unordered_set>
#endif

namespace dart::common {

/// The std::type_info::operator== function may not work for types from distinct
/// libraries when using the libc++ standard library implementation. This is
/// because equality and the type_index hash are based on the type name, which
/// may not be unique across different libraries. To maintain consistent
/// behavior across different implementations, the following typedefs are
/// introduced.

// TODO(JS): Remove this workaround when the following issue is resolved.
#if defined(__GLIBCXX__)

/// A map that uses std::type_index as the key type
/// @tparam T The value type of the map
template <typename T>
using TypeMap = std::map<std::type_index, T>;

/// A set that uses std::type_index as the key type
using TypeSet = std::unordered_set<std::type_index>;

#else

/// A map that uses std::type_index as the key type
struct TypeLess
{
  /// Less-than operator for std::type_index
  /// @param lhs The left-hand side of the comparison
  /// @param rhs The right-hand side of the comparison
  /// @return True if the name of the left-hand side is less than the name of
  bool operator()(const std::type_index& lhs, const std::type_index& rhs) const
  {
    return std::string(lhs.name()) < std::string(rhs.name());
  }
};

/// A hash function for std::type_index
struct TypeHash
{
  ///
  /// @param t
  /// @return
  std::size_t operator()(const std::type_index& t) const
  {
    return std::hash<std::string>()(t.name());
  }
};

/// An equality function for std::type_index
struct TypeEqualTo
{
  /// Equality operator for std::type_index
  /// @param lhs The left-hand side of the comparison
  /// @param rhs The right-hand side of the comparison
  /// @return True if the names of both sides are equal
  bool operator()(const std::type_index& lhs, const std::type_index& rhs) const
  {
    return lhs.name() == rhs.name() || std::strcmp(lhs.name(), rhs.name()) == 0;
  }
};

/// A map that uses std::type_index as the key type
/// @tparam T The value type of the map
template <typename T>
using TypeMap = std::map<std::type_index, T, TypeLess>;

/// A set that uses std::type_index as the key type
using TypeSet = std::unordered_set<std::type_index, TypeHash, TypeEqualTo>;

#endif

} // namespace dart::common
