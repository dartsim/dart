/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/collision/fcl/backward_compatibility.hpp"
#include "dart/collision/fcl/fcl_type.hpp"
#include "dart/collision/object.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class FclObject : public Object<Scalar_>
{
public:
  // Type aliases
  using Scalar = Scalar_;

  // Documentation inherited
  math::SE3<Scalar> get_pose() const override;

  // Documentation inherited
  void set_pose(const math::SE3<Scalar>& tf) override;

  // Documentation inherited
  math::R3<Scalar> get_position() const override;

  // Documentation inherited
  void set_position(const math::R3<Scalar>& pos) override;

  /// Return FCL collision object
  FclCollisionObject<Scalar>* get_fcl_collision_object();

  /// Return FCL collision object
  const FclCollisionObject<Scalar>* get_fcl_collision_object() const;

public:
  /// Constructor
  FclObject(
      Scene<Scalar>* collision_scene,
      ObjectId id,
      math::Geometry3Ptr<Scalar> geometry,
      const std::shared_ptr<FclCollisionGeometry<Scalar>>& fcl_coll_geom);

protected:
  // Documentation inherited
  void update_engine_data() override;

  /// FCL collision object
  std::unique_ptr<FclCollisionObject<Scalar>> m_fcl_collision_object;

private:
  friend class FclEngine<Scalar>;
  friend class FclScene<Scalar>;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, FclObject)

} // namespace collision
} // namespace dart

#include "dart/collision/fcl/detail/fcl_object_impl.hpp"
