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

#include <dart/simulation/World.hpp>
#include <dart/simulation/detail/WorldEcsAccess.hpp>

namespace dart::simulation::object {

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
const Component& ObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::getReadOnly()
    const
{
  static_assert(
      Contains_v<
          Component,
          ReadOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in ReadOnlyComps<> or ReadWriteComps<> for this "
      "class. Add it to one of these lists.");

  const auto entity = detail::WorldEcsAccess::toEntt(mEntity);
  return detail::WorldEcsAccess::getEntityManager(*mWorld)
      .template get<Component>(entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
Component& ObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::getMutable()
{
  static_assert(
      Contains_v<
          Component,
          WriteOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in WriteOnlyComps<> or ReadWriteComps<> for this "
      "class. Add it to one of these lists or use getReadOnly().");

  const auto entity = detail::WorldEcsAccess::toEntt(mEntity);
  return detail::WorldEcsAccess::getEntityManager(*mWorld)
      .template get<Component>(entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
const Component*
ObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::tryGetReadOnly() const
{
  static_assert(
      Contains_v<
          Component,
          ReadOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in ReadOnlyComps<> or ReadWriteComps<> for this "
      "class. Add it to one of these lists.");

  const auto entity = detail::WorldEcsAccess::toEntt(mEntity);
  return detail::WorldEcsAccess::getEntityManager(*mWorld)
      .template try_get<Component>(entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
Component* ObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::tryGetMutable()
{
  static_assert(
      Contains_v<
          Component,
          WriteOnlyList> || Contains_v<Component, ReadWriteList>,
      "Component not declared in WriteOnlyComps<> or ReadWriteComps<> for this "
      "class. Add it to one of these lists or use tryGetReadOnly().");

  const auto entity = detail::WorldEcsAccess::toEntt(mEntity);
  return detail::WorldEcsAccess::getEntityManager(*mWorld)
      .template try_get<Component>(entity);
}

//==============================================================================
template <
    typename Tags,
    typename ReadOnly,
    typename WriteOnly,
    typename ReadWrite>
template <typename Component>
Component* ObjectWith<Tags, ReadOnly, WriteOnly, ReadWrite>::getCacheMutable()
    const
{
  static_assert(
      Contains_v<Component, ReadWriteList>,
      "Component must be in ReadWriteComps<> for cache access. Cache "
      "components need read-write access.");

  return const_cast<ObjectWith*>(this)->tryGetMutable<Component>();
}

} // namespace dart::simulation::object
