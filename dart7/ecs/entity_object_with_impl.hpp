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

#include "dart7/world.hpp"

namespace dart7 {

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
const Component&
EntityObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::getReadOnly() const
{
  // Can read from ReadOnly or ReadWrite components
  static_assert(
      Contains_v<
          Component,
          ReadOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in ReadOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists.");

  return m_world->getRegistry().template get<Component>(m_entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
Component& EntityObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::getMutable()
{
  // Can write to WriteOnly or ReadWrite components
  static_assert(
      Contains_v<
          Component,
          WriteOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in WriteOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists or use getReadOnly().");

  return m_world->getRegistry().template get<Component>(m_entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
const Component*
EntityObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::tryGetReadOnly() const
{
  // Can read from ReadOnly or ReadWrite components
  static_assert(
      Contains_v<
          Component,
          ReadOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in ReadOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists.");

  return m_world->getRegistry().template try_get<Component>(m_entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
Component*
EntityObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::tryGetMutable()
{
  // Can write to WriteOnly or ReadWrite components
  static_assert(
      Contains_v<
          Component,
          WriteOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in WriteOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists or use tryGetReadOnly().");

  return m_world->getRegistry().template try_get<Component>(m_entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
Component*
EntityObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::getCacheMutable() const
{
  // Component must be in ReadWriteComps for cache access
  static_assert(
      Contains_v<Component, ReadWriteList>,
      "Component must be in ReadWriteComps<> for cache access. "
      "Cache components need read-write access.");

  // const_cast is safe here: cache mutation doesn't change logical const-ness
  // The component must be in ReadWriteComps<>, ensuring it's designed for this
  return const_cast<EntityObjectWith*>(this)->tryGetMutable<Component>();
}

} // namespace dart7
