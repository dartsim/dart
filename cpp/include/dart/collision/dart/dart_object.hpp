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

#include "dart/collision/dart/dart_type.hpp"
#include "dart/collision/object.hpp"
#include "dart/math/lie_group/se3.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class DartObject : public Object<Scalar_>
{
public:
  using Scalar = Scalar_;

  // Documentation inherited
  math::SE3<Scalar> get_pose() const override;

  // Documentation inherited
  void set_pose(const math::SE3<Scalar>& tf) override;

  // Documentation inherited
  math::R3<Scalar> get_position() const override;

  // Documentation inherited
  void set_position(const math::R3<Scalar>& pos) override;

  /// @{ @name AABB

  const math::Aabb3<Scalar>& get_aabb() const;

  /// @}

protected:
  /// Constructor
  DartObject(DartScene<Scalar>* group, math::Geometry3Ptr<Scalar> shape);

  // Documentation inherited
  void update_engine_data() override;

private:
  void update_aabb() const;

  friend class DartEngine<Scalar>;
  friend class DartScene<Scalar>;

  /// Pose of the collision object
  math::SE3<Scalar> m_pose;

  mutable math::Aabb3<Scalar> m_aabb;

  mutable bool m_aabb_dirty{true};
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, DartObject)

} // namespace collision
} // namespace dart

#include "dart/collision/dart/detail/dart_object_impl.hpp"
