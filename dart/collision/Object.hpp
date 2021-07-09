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

#include "dart/collision/Types.hpp"
#include "dart/math/SmartPointer.hpp"
#include "dart/math/Types.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class Object
{
public:
  // Type aliases
  using S = S_;

  Object();

  /// Destructor
  virtual ~Object();

  /// Return collision detection engine associated with this Object
  Engine<S>* getEngine();

  /// Return collision detection engine associated with this Object
  const Engine<S>* getEngine() const;

  const void* getUserData() const;

  /// Return the associated Shape
  math::ConstGeometryPtr getShape() const;

  /// Return the transformation of this Object in world coordinates
  virtual math::Isometry3<S> getTransform() const = 0;

  virtual void setTransform(const math::Isometry3<S>& tf) = 0;

  virtual math::Vector3<S> getTranslation() const = 0;

  virtual void setTranslation(const math::Vector3<S>& pos) = 0;

protected:
  /// Contructor
  Object(Group<S>* mGroup, math::GeometryPtr shape);

  /// Update the collision object of the collision detection engine. This
  /// function will be called ahead of every collision checking by
  /// Group.
  virtual void updateEngineData() = 0;

protected:
  /// Collision group
  Group<S>* mGroup;

  const math::GeometryPtr mShape;

  void* mUserData{nullptr};

private:
  friend class Group<S>;
};

extern template class Object<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/detail/Object-impl.hpp"
