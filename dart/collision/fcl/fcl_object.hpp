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

#include "dart/collision/object.hpp"
#include "dart/collision/fcl/backward_compatibility.hpp"
#include "dart/collision/fcl/fcl_types.hpp"
#include "dart/math/Types.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class FclObject : public Object<S_>
{
public:
  // Type aliases
  using S = S_;

  // Documentation inherited
  math::Isometry3<S> get_pose() const override;

  // Documentation inherited
  void set_pose(const math::Isometry3<S>& tf) override;

  /// Return FCL collision object
  FclCollisionObject<S>* get_fcl_collision_object();

  /// Return FCL collision object
  const FclCollisionObject<S>* get_fcl_collision_object() const;

  math::Vector3<S> get_position() const;

  void set_position(const math::Vector3<S>& pos);

protected:
  /// Constructor
  FclObject(
      Group<S>* collisionGroup,
      math::GeometryPtr shape,
      const std::shared_ptr<FclCollisionGeometry<S>>& fclCollGeom);

  // Documentation inherited
  void update_engine_data() override;

protected:
  FclObject(Group<S>* collisionGroup, math::GeometryPtr shape);

  /// FCL collision object
  std::unique_ptr<FclCollisionObject<S>> mFclCollisionObject;

private:
  friend class FclEngine<S>;
  friend class FclGroup<S>;
};

extern template class FclObject<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/fcl/detail/fcl_object_impl.hpp"
