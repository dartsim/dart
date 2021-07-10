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

#include <map>

#include "dart/collision/engine.hpp"
#include "dart/collision/fcl/fcl_types.hpp"
#include "dart/math/SmartPointer.hpp"
#include "dart/collision/fcl/backward_compatibility.hpp"

namespace dart {
namespace collision {

template <typename S_>
class FclEngine : public Engine<S_>
{
public:
  // Type aliases
  using S = S_;

  static std::shared_ptr<FclEngine> Create();

  /// Whether to use analytic collision checking for primitive shapes.
  ///
  /// PRIMITIVE: Use FCL's analytic collision checking for primitive shapes.
  /// MESH: Don't use it. Instead, use approximate mesh shapes for the primitive
  /// shapes. The contact result is probably less accurate than the analytic
  /// result.
  ///
  /// Warning: FCL's primitive shape support is not complete. FCL 0.4.0 improved
  /// the support alot, but it still returns single contact point for a shape
  /// pair except for box-box collision. For this reason, we recommend using
  /// MESH until FCL fully supports primitive shapes.
  enum PrimitiveShape
  {
    PRIMITIVE = 0,
    MESH
  };

  /// Whether to use FCL's contact point computation.
  ///
  /// FCL: Use FCL's contact point computation.
  /// DART: Use DART's own contact point computation.
  ///
  /// Warning: FCL's contact computation is not correct. See:
  /// https://github.com/flexible-collision-library/fcl/issues/106
  /// We recommend using DART until it's fixed in FCL.
  enum ContactPointComputationMethod
  {
    FCL = 0,
    DART
  };

  /// Constructor
  ~FclEngine() override;

  // Documentation inherited
  const std::string& get_type() const override;

  /// Get collision detector type for this class
  static const std::string& GetStaticType();

  // Documentation inherited
  GroupPtr<S> create_group() override;

  // Documentation inherited
  bool collide(ObjectPtr<S> object1, ObjectPtr<S> object2) override;

protected:
  /// Constructor
  FclEngine() = default;

  /// Returns ::fcl::CollisionGeometry associated with give Shape. New
  /// ::fcl::CollisionGeome will be created if it hasn't created yet.
  std::shared_ptr<FclCollisionGeometry<S>> create_fcl_collision_geometry(
      math::ConstGeometryPtr shape);

private:
  std::shared_ptr<FclCollisionGeometry<S>> create_fcl_collision_geometry_impl(
      const math::ConstGeometryPtr& shape, FclEngine::PrimitiveShape type);


  PrimitiveShape m_primitive_shape_type = PrimitiveShape::MESH;

  DART_REGISTER_ENGINE_IN_HEADER(FclEngine<S>);
};

DART_REGISTER_ENGINE_OUT_HEADER(FclEngine<S>);

using FclEnginef = FclEngine<float>;
using FclEngined = FclEngine<double>;

extern template class FclEngine<double>;

} // namespace collision
} // namespace dart

#include "dart/collision/fcl/detail/fcl_engine_impl.hpp"
