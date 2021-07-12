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
#include "dart/collision/ode/detail/ode_geom.hpp"
#include "dart/collision/ode/ode_include.hpp"
#include "dart/collision/ode/ode_type.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {

template <typename S_>
class OdeObject : public Object<S_> {
public:
  // Type aliases
  using S = S_;

  // Documentation inherited
  math::Isometry3<S> get_pose() const override;

  // Documentation inherited
  void set_pose(const math::Isometry3<S>& tf) override;

  // Documentation inherited
  math::Vector3<S> get_position() const override;

  // Documentation inherited
  void set_position(const math::Vector3<S>& pos) override;

protected:
  /// Constructor
  OdeObject(OdeGroup<S>* collision_group, math::GeometryPtr shape);

  // Documentation inherited
  void update_engine_data() override;

  /// Returns the ODE body id associated to this object
  dBodyID get_ode_body_id() const;

  /// Returns the ODE body id associated to this object
  dGeomID get_ode_geom_id() const;

private:
  friend class OdeEngine<S>;
  friend class OdeGroup<S>;

  /// ODE geom
  std::unique_ptr<detail::OdeGeom> m_ode_geom;

  /// ODE body id associated with this object
  ///
  /// If the ODE geom type is immobile, this is nullptr.
  dBodyID m_ode_body_id;
};

using OdeObjectf = OdeObject<float>;
using OdeObjectd = OdeObject<double>;

extern template class OdeObject<double>;

} // namespace collision
} // namespace dart

#include "dart/collision/ode/detail/ode_object_impl.hpp"
