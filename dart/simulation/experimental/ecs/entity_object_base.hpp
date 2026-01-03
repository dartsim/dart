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

#include <dart/simulation/experimental/fwd.hpp>

#include <entt/entt.hpp>

namespace dart::simulation::experimental {

/// EntityObject - Non-template base class for all entity-wrapping objects
///
/// This is the base class that stores the entity ID and World pointer.
/// It uses virtual inheritance to enable the diamond inheritance pattern
/// used by EntityObjectWith.
///
/// Design Pattern (from DART 6's Composite/SpecializedForAspect):
/// ---------------------------------------------------------------
/// This uses the diamond inheritance pattern via virtual inheritance:
///
/// - EntityObject (non-template) is the shared base
/// - EntityObjectWith<A> virtually inherits from EntityObject
/// - EntityObjectWith<B> virtually inherits from EntityObject
/// - DerivedClass inherits from both EntityObjectWith<A> and
/// EntityObjectWith<B>
/// - Result: Single EntityObject storage shared by both paths
///
/// Both EntityObjectWith instantiations share the same EntityObject storage
/// via virtual inheritance. This enables:
/// - Compile-time component validation (from EntityObjectWith)
/// - Normal inheritance (Frame -> FreeFrame)
/// - Virtual methods (from non-template base)
/// - Return by value (can return EntityObject or Frame)
///
/// @see EntityObjectWith, Frame, FreeFrame, FixedFrame
class EntityObject
{
public:
  /// Constructor
  ///
  /// @param entity Entity ID in the ECS registry
  /// @param world Pointer to the owning World instance
  EntityObject(entt::entity entity = entt::null, World* world = nullptr)
    : m_entity(entity), m_world(world)
  {
    // Empty
  }

  /// Virtual destructor
  virtual ~EntityObject() = default;

  /// Get the underlying entity ID
  ///
  /// @return Entity ID in the ECS registry
  [[nodiscard]] entt::entity getEntity() const
  {
    return m_entity;
  }

  /// Get the owning World instance
  ///
  /// @return Pointer to World instance
  [[nodiscard]] World* getWorld() const
  {
    return m_world;
  }

protected:
  entt::entity m_entity; ///< Entity ID in ECS registry
  World* m_world;        ///< Owning World instance
};

} // namespace dart::simulation::experimental
