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

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/export.hpp>
#include <dart/collision/experimental/fwd.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <cstddef>

namespace dart::collision::experimental {

class CollisionWorld;

/// Lightweight handle to a collision entity in the ECS registry.
/// Data (shape, transform, AABB) is stored in CollisionWorld's registry.
/// This handle is cheap to copy (entity ID + pointer).
class DART_COLLISION_EXPERIMENTAL_API CollisionObject
{
public:
  CollisionObject() = default;
  CollisionObject(entt::entity entity, CollisionWorld* world);

  CollisionObject(const CollisionObject&) = default;
  CollisionObject& operator=(const CollisionObject&) = default;
  CollisionObject(CollisionObject&&) = default;
  CollisionObject& operator=(CollisionObject&&) = default;
  ~CollisionObject() = default;

  [[nodiscard]] const Shape* getShape() const;
  [[nodiscard]] ShapeType getShapeType() const;

  [[nodiscard]] ObjectId getObjectId() const;

  [[nodiscard]] const Eigen::Isometry3d& getTransform() const;
  void setTransform(const Eigen::Isometry3d& transform);

  [[nodiscard]] Aabb computeAabb() const;

  void setUserData(void* data);
  [[nodiscard]] void* getUserData() const;

  [[nodiscard]] bool isValid() const;

  [[nodiscard]] entt::entity getEntity() const
  {
    return m_entity;
  }

  [[nodiscard]] CollisionWorld* getWorld() const
  {
    return m_world;
  }

  bool operator==(const CollisionObject& other) const
  {
    return m_entity == other.m_entity && m_world == other.m_world;
  }

  bool operator!=(const CollisionObject& other) const
  {
    return !(*this == other);
  }

  bool operator<(const CollisionObject& other) const
  {
    if (m_world != other.m_world) {
      return m_world < other.m_world;
    }
    return m_entity < other.m_entity;
  }

private:
  entt::entity m_entity{entt::null};
  CollisionWorld* m_world{nullptr};
};

} // namespace dart::collision::experimental
