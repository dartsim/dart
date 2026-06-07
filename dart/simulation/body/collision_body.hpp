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

#include <dart/simulation/entity.hpp>
#include <dart/simulation/export.hpp>
#include <dart/simulation/fwd.hpp>

#include <optional>
#include <string>

namespace dart::simulation {

class RigidBody;
class Link;

/// Lightweight handle to a body that can carry collision geometry.
///
/// A collision body is either a `RigidBody` or a multibody `Link`. This handle
/// is the body reference reported by `World::collide()` through `Contact`,
/// letting contacts refer uniformly to rigid bodies and articulated links.
///
/// @note Handles are lightweight and safe to copy. They become invalid if the
///       referenced body or World is destroyed.
class DART_SIMULATION_API CollisionBody
{
public:
  /// Construct an invalid handle.
  CollisionBody() = default;

  /// Construct a handle to the given entity in the given World.
  CollisionBody(Entity entity, World* world);

  /// Get the backend-neutral identity token for this body.
  [[nodiscard]] Entity getEntity() const;

  /// Get the owning World pointer.
  [[nodiscard]] World* getWorld() const;

  /// Return whether this handle refers to a live entity.
  [[nodiscard]] bool isValid() const;

  /// Get the body's name.
  [[nodiscard]] std::string getName() const;

  /// Return whether this body is a rigid body.
  [[nodiscard]] bool isRigidBody() const;

  /// Return whether this body is a multibody link.
  [[nodiscard]] bool isLink() const;

  /// Get a RigidBody handle if this body is a rigid body, else std::nullopt.
  [[nodiscard]] std::optional<RigidBody> asRigidBody() const;

  /// Get a Link handle if this body is a multibody link, else std::nullopt.
  [[nodiscard]] std::optional<Link> asLink() const;

private:
  Entity m_entity{}; ///< Opaque entity token (default-constructed = invalid)
  World* m_world{nullptr};
};

} // namespace dart::simulation
