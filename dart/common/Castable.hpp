/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COMMON_CASTABLE_HPP_
#define DART_COMMON_CASTABLE_HPP_

#include <string>

#define DART_STRING_TYPE(type_name)                                            \
  /** Returns type string. */                                                  \
  [[nodiscard]] static const std::string& getStaticType()                      \
  {                                                                            \
    static const std::string type = #type_name;                                \
    return type;                                                               \
  }                                                                            \
                                                                               \
  [[nodiscard]] const std::string& getType() const override                    \
  {                                                                            \
    return getStaticType();                                                    \
  }                                                                            \
  void _ANONYMOUS_FUNCTION_1()

namespace dart::common {

/// A CRTP base class that provides an interface for easily casting to the
/// derived types.
template <typename Base>
class Castable
{
public:
  /// Returns true if the types of this \c Base and the template parameter (a
  /// base class) are identical. This function is a syntactic sugar, which
  /// is identical to: (getType() == ShapeType::getStaticType()).
  ///
  /// Example code:
  /// \code
  /// if (shape->is<Sphere>())
  ///   std::cout << "The shape type is sphere!\n";
  /// \endcode
  template <typename Derived>
  [[nodiscard]] bool is() const;

  /// Casts to pointer of Derived if Base is its base class. Returns nullptr
  /// otherwise.
  template <typename Derived>
  [[nodiscard]] const Derived* as() const;

  /// Casts to pointer of Derived if Base is its base class. Returns nullptr
  /// otherwise.
  template <typename Derived>
  [[nodiscard]] Derived* as();

  /// Casts to reference of Derived if Base is its base class. UB otherwise.
  template <typename Derived>
  [[nodiscard]] const Derived& asRef() const;

  /// Casts to reference of Derived if Base is its base class. UB otherwise.
  template <typename Derived>
  [[nodiscard]] Derived& asRef();

private:
  /// Casts to Base const-reference
  [[nodiscard]] const Base& base() const;

  /// Casts to Base reference
  [[nodiscard]] Base& base();
};

} // namespace dart::common

#include "dart/common/detail/Castable-impl.hpp"

#endif // DART_COMMON_CASTABLE_HPP_
