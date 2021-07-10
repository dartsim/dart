/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <Eigen/Geometry>

#include "dart/collision/types.hpp"
#include "dart/math/SmartPointer.hpp"
#include "dart/math/Types.hpp"

namespace dart {
namespace collision {

/// Collision object
template <typename S_>
class Object
{
public:
  // Type aliases
  using S = S_;

  /// Constructor
  Object();

  /// Destructor
  virtual ~Object();

  /// Returns collision detection engine associated with this object
  Engine<S>* get_mutable_engine();

  /// Returns collision detection engine associated with this object
  const Engine<S>* get_engine() const;

  /// Returns the user data associated with this object.
  const void* get_user_data() const;

  /// Returns the associated shape
  math::ConstGeometryPtr get_geometry() const;

  /// Returns the pose of this object in world coordinates
  virtual math::Isometry3<S> get_pose() const = 0;

  /// Sets the pose of this object
  virtual void set_pose(const math::Isometry3<S>& tf) = 0;

  /// Returns the position of this object in world coordinates
  virtual math::Vector3<S> get_position() const = 0;

  /// Sets the position os this object in world coordinates
  virtual void set_position(const math::Vector3<S>& pos) = 0;

protected:
  /// Contructor
  Object(Group<S>* group, math::GeometryPtr shape);

  /// Update the collision object of the collision detection engine. This
  /// function will be called ahead of every collision checking by
  /// Group.
  virtual void update_engine_data() = 0;

protected:
  /// Collision group
  Group<S>* m_group;

  /// Geometry
  const math::GeometryPtr m_geometry;

  void* m_user_data = nullptr;

private:
  friend class Group<S>;
};

using Objectf = Object<float>;
using Objectd = Object<double>;

extern template class Object<double>;

} // namespace collision
} // namespace dart

#include "dart/collision/detail/object_impl.hpp"
