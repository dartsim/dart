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

#include "dart/common/castable.hpp"

namespace dart::common {

//==============================================================================
DART_CREATE_MEMBER_CHECK(get_type);
DART_CREATE_MEMBER_CHECK(GetType);

//==============================================================================
template <typename Base>
template <typename Derived>
bool Castable<Base>::is() const
{
  if constexpr (
      has_member_get_type<Base>::value && has_member_GetType<Derived>::value) {
    return (base().get_type() == Derived::GetType());
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
const Derived& Castable<Base>::as_ref() const
{
  return *as<Derived>();
}

//==============================================================================
template <typename Base>
template <typename Derived>
Derived& Castable<Base>::as_ref()
{
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
