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
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#include <dart/dynamics/Fwd.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <string_view>

namespace dart::simulation::experimental {

/// Options for creating ShapeNodes
struct DART_EXPERIMENTAL_API ShapeNodeOptions
{
  Eigen::Isometry3d relativeTransform = Eigen::Isometry3d::Identity();
  bool collidable = true;
  double frictionCoeff = 1.0;
  double restitutionCoeff = 0.0;
};

/// ShapeNode handle class
///
/// Represents a collision shape attached to a Link or RigidBody.
/// This is a lightweight handle to the underlying entity in the ECS registry.
class DART_EXPERIMENTAL_API ShapeNode
{
public:
  /// Constructor (typically called by Link/RigidBody)
  ShapeNode(entt::entity entity, World* world);

  /// Get the name of this shape node
  [[nodiscard]] std::string_view getName() const;

  /// Get the underlying ECS entity handle
  [[nodiscard]] entt::entity getEntity() const;

  /// Check if this handle is valid (entity still exists)
  [[nodiscard]] bool isValid() const;

  /// Get the parent frame (Link or RigidBody)
  [[nodiscard]] Frame getParentFrame() const;

  /// Get the relative transform of this shape (parent frame -> shape)
  [[nodiscard]] Eigen::Isometry3d getRelativeTransform() const;

  /// Set the relative transform of this shape (parent frame -> shape)
  void setRelativeTransform(const Eigen::Isometry3d& transform);

  /// Get the world transform of this shape
  [[nodiscard]] Eigen::Isometry3d getWorldTransform() const;

  /// Get the underlying shape
  [[nodiscard]] dart::dynamics::ConstShapePtr getShape() const;

  /// Set the underlying shape
  void setShape(const dart::dynamics::ShapePtr& shape);

  /// Enable or disable collision for this shape
  void setCollidable(bool collidable);

  /// Check if this shape is collidable
  [[nodiscard]] bool isCollidable() const;

  /// Set the friction coefficient (primary + secondary)
  void setFrictionCoeff(double friction);

  /// Get the friction coefficient
  [[nodiscard]] double getFrictionCoeff() const;

  /// Set restitution coefficient
  void setRestitutionCoeff(double restitution);

  /// Get restitution coefficient
  [[nodiscard]] double getRestitutionCoeff() const;

private:
  entt::entity m_entity;
  World* m_world;
};

} // namespace dart::simulation::experimental
