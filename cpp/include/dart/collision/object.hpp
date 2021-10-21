/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#include <memory>

#include "dart/collision/export.hpp"
#include "dart/collision/type.hpp"
#include "dart/math/geometry/type.hpp"

namespace dart::collision {

template <typename Scalar_>
class Object
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Constructor
  Object();

  /// Destructor
  virtual ~Object();

  /// Returns collision detection engine associated with this object
  Engine<Scalar>* get_mutable_engine();

  /// Returns collision detection engine associated with this object
  const Engine<Scalar>* get_engine() const;

  /// Returns the user data associated with this object.
  const void* get_user_data() const;

  /// Returns the associated geometry
  math::ConstGeometryPtr get_geometry() const;

  /// Returns the pose of this object in world coordinates
  virtual math::Isometry3<Scalar> get_pose() const = 0;

  /// Sets the pose of this object
  virtual void set_pose(const math::Isometry3<Scalar>& tf) = 0;

  /// Returns the position of this object in world coordinates
  virtual math::Vector3<Scalar> get_position() const = 0;

  /// Sets the position os this object in world coordinates
  virtual void set_position(const math::Vector3<Scalar>& pos) = 0;

protected:
  /// Contructor
  Object(Scene<Scalar>* group, math::GeometryPtr shape);

  /// Update the collision object of the collision detection engine. This
  /// function will be called ahead of every collision checking by
  /// Group.
  virtual void update_engine_data() = 0;

protected:
  /// Collision group
  Scene<Scalar>* m_scene;

  /// Geometry
  const math::GeometryPtr m_geometry;

  void* m_user_data = nullptr;

private:
  friend class Scene<Scalar>;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, Object)

} // namespace dart::collision

#include "dart/collision/detail/object_impl.hpp"
