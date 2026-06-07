/*
 * Copyright (c) 2011, The DART development contributors
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

// INTERNAL HEADER — do NOT include from any promoted public header.
//
// This header provides compile-time validated component access for handle
// classes (Frame, FreeFrame, FixedFrame, ...). Each handle's allowed component
// access is described by a `ComponentAccessPolicy<Handle>` trait, and the
// free-function accessors below `static_assert` that the requested component is
// permitted by that policy before reaching into the ECS registry. This keeps
// the validation that used to live on the public `EntityObjectWith<>` base
// (removed during promotion) while leaving the promoted handle headers free of
// any ECS storage symbols. Include it ONLY from handle `.cpp` files.

#include <dart/simulation/common/type_list.hpp>
#include <dart/simulation/comps/frame_types.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/fwd.hpp>

namespace dart::simulation::ecs {

//==============================================================================
/// ComponentAccessPolicy - per-handle compile-time component access policy.
///
/// The primary template is intentionally left undefined: a handle without a
/// policy specialization cannot use the validated accessors below, mirroring
/// the historical behavior where only handles that inherited
/// `EntityObjectWith<>` had validated access.
///
/// Each specialization declares four `TypeList<>` aliases:
/// - `Tag`       : tag-only components (ECS filtering, no data access)
/// - `ReadOnly`  : components accessible via `getReadOnly`/`tryGetReadOnly`
/// - `WriteOnly` : components accessible via `getMutable`/`tryGetMutable`
/// - `ReadWrite` : components accessible via all of the above + cache access
template <typename Handle>
struct ComponentAccessPolicy;

//==============================================================================
template <>
struct ComponentAccessPolicy<Frame>
{
  using Tag = TypeList<comps::FrameTag>;
  using ReadOnly = TypeList<>;
  using WriteOnly = TypeList<>;
  using ReadWrite = TypeList<comps::FrameState, comps::FrameCache>;
};

//==============================================================================
template <>
struct ComponentAccessPolicy<FreeFrame>
{
  using Tag = TypeList<comps::FreeFrameTag>;
  using ReadOnly = TypeList<>;
  using WriteOnly = TypeList<>;
  using ReadWrite = TypeList<comps::FreeFrameProperties>;
};

//==============================================================================
template <>
struct ComponentAccessPolicy<FixedFrame>
{
  using Tag = TypeList<comps::FixedFrameTag>;
  using ReadOnly = TypeList<>;
  using WriteOnly = TypeList<>;
  using ReadWrite = TypeList<comps::FixedFrameProperties>;
};

//==============================================================================
/// Get read-only access to a component (must exist).
///
/// Component must be in the handle's ReadOnly or ReadWrite policy list.
template <typename Handle, typename Component>
const Component& getReadOnly(const Handle& h)
{
  using P = ComponentAccessPolicy<Handle>;
  // Can read from ReadOnly or ReadWrite components
  static_assert(
      Contains_v<Component, typename P::ReadOnly>
          || Contains_v<Component, typename P::ReadWrite>,
      "Component not declared in ReadOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists.");

  return dart::simulation::detail::registryOf(*h.getWorld())
      .template get<Component>(detail::toRegistryEntity(h.getEntity()));
}

//==============================================================================
/// Get mutable access to a component (must exist).
///
/// Component must be in the handle's WriteOnly or ReadWrite policy list.
template <typename Handle, typename Component>
Component& getMutable(Handle& h)
{
  using P = ComponentAccessPolicy<Handle>;
  // Can write to WriteOnly or ReadWrite components
  static_assert(
      Contains_v<Component, typename P::WriteOnly>
          || Contains_v<Component, typename P::ReadWrite>,
      "Component not declared in WriteOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists or use getReadOnly().");

  return dart::simulation::detail::registryOf(*h.getWorld())
      .template get<Component>(detail::toRegistryEntity(h.getEntity()));
}

//==============================================================================
/// Try to get read-only access to a component (may not exist).
///
/// Component must be in the handle's ReadOnly or ReadWrite policy list.
/// Returns nullptr if the component is not present on the entity.
template <typename Handle, typename Component>
const Component* tryGetReadOnly(const Handle& h)
{
  using P = ComponentAccessPolicy<Handle>;
  // Can read from ReadOnly or ReadWrite components
  static_assert(
      Contains_v<Component, typename P::ReadOnly>
          || Contains_v<Component, typename P::ReadWrite>,
      "Component not declared in ReadOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists.");

  return dart::simulation::detail::registryOf(*h.getWorld())
      .template try_get<Component>(detail::toRegistryEntity(h.getEntity()));
}

//==============================================================================
/// Try to get mutable access to a component (may not exist).
///
/// Component must be in the handle's WriteOnly or ReadWrite policy list.
/// Returns nullptr if the component is not present on the entity.
template <typename Handle, typename Component>
Component* tryGetMutable(Handle& h)
{
  using P = ComponentAccessPolicy<Handle>;
  // Can write to WriteOnly or ReadWrite components
  static_assert(
      Contains_v<Component, typename P::WriteOnly>
          || Contains_v<Component, typename P::ReadWrite>,
      "Component not declared in WriteOnlyComps<> or ReadWriteComps<> for "
      "this class. "
      "Add it to one of these lists or use tryGetReadOnly().");

  return dart::simulation::detail::registryOf(*h.getWorld())
      .template try_get<Component>(detail::toRegistryEntity(h.getEntity()));
}

//==============================================================================
/// Get mutable access to a cache component from a const handle (may not exist).
///
/// This mirrors the lazy-evaluation cache pattern: the const-ness is logical
/// (observable state is unchanged), but the cache needs physical mutation.
/// Component must be in the handle's ReadWrite policy list.
template <typename Handle, typename Component>
Component* getCacheMutable(const Handle& h)
{
  using P = ComponentAccessPolicy<Handle>;
  // Component must be in ReadWriteComps for cache access
  static_assert(
      Contains_v<Component, typename P::ReadWrite>,
      "Component must be in ReadWriteComps<> for cache access. "
      "Cache components need read-write access.");

  // const_cast is safe here: cache mutation doesn't change logical const-ness.
  // The component must be in ReadWriteComps<>, ensuring it's designed for this.
  return tryGetMutable<Handle, Component>(const_cast<Handle&>(h));
}

} // namespace dart::simulation::ecs
