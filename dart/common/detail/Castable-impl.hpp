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

#include <dart/common/Castable.hpp>
#include <dart/common/Metaprogramming.hpp>

#include <string>

#define DETAIL_DART_STRING_TYPE(type_name)                                     \
  /** Returns static type string. */                                           \
  [[nodiscard]] static const std::string& GetType()                            \
  {                                                                            \
    static const std::string type = #type_name;                                \
    return type;                                                               \
  }                                                                            \
                                                                               \
  /** Returns type string */                                                   \
  [[nodiscard]] const std::string& getType() const override                    \
  {                                                                            \
    return GetType();                                                          \
  }                                                                            \
  void _ANONYMOUS_FUNCTION_1()

#define DETAIL_DART_STRING_TYPE_TEMPLATE_1(type_name, templ_arg1)              \
  /** Returns static type string. */                                           \
  [[nodiscard]] static const std::string& GetType()                            \
  {                                                                            \
    static const std::string type = std::string(#type_name) + std::string("<") \
                                    + std::string(typeid(templ_arg1).name())   \
                                    + std::string(">");                        \
    return type;                                                               \
  }                                                                            \
                                                                               \
  /** Returns type string */                                                   \
  [[nodiscard]] const std::string& getType() const override                    \
  {                                                                            \
    return GetType();                                                          \
  }                                                                            \
  void _ANONYMOUS_FUNCTION_2()

namespace dart::common {

//==============================================================================
DART_CREATE_MEMBER_CHECK(getType);
DART_CREATE_MEMBER_CHECK(getStaticType);
DART_CREATE_MEMBER_CHECK(GetType);

//==============================================================================
template <typename Base>
template <typename Derived>
bool Castable<Base>::is() const
{
  if constexpr (
      has_member_getType<Base>::value
      && has_member_getStaticType<Derived>::value) {
    return (base().getType() == Derived::getStaticType());
  } else if constexpr (
      has_member_getType<Base>::value && has_member_GetType<Derived>::value) {
    return (base().getType() == Derived::GetType());
  } else {
    return (dynamic_cast<const Derived*>(&base()) != nullptr);
  }
}

//==============================================================================
template <typename Base>
template <typename Derived>
const Derived* Castable<Base>::as() const
{
  return is<Derived>() ? static_cast<const Derived*>(&base()) : nullptr;
}

//==============================================================================
template <typename Base>
template <typename Derived>
Derived* Castable<Base>::as()
{
  return is<Derived>() ? static_cast<Derived*>(&base()) : nullptr;
}

//==============================================================================
template <typename Base>
template <typename Derived>
const Derived& Castable<Base>::asRef() const
{
  assert(is<Derived>());
  return *as<Derived>();
}

//==============================================================================
template <typename Base>
template <typename Derived>
Derived& Castable<Base>::asRef()
{
  assert(is<Derived>());
  return *as<Derived>();
}

//==============================================================================
template <typename Base>
const Base& Castable<Base>::base() const
{
  return *static_cast<const Base*>(this);
}

//==============================================================================
template <typename Base>
Base& Castable<Base>::base()
{
  return *static_cast<Base*>(this);
}

} // namespace dart::common
