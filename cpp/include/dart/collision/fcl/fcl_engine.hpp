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
#include "dart/collision/fcl/backward_compatibility.hpp"
#include "dart/collision/fcl/fcl_type.hpp"
#include "dart/math/geometry/type.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class FclEngine : public Engine<Scalar_>
{
public:
  // Type aliases
  using Scalar = Scalar_;

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

  /// Destructor
  ~FclEngine() override;

  // Documentation inherited
  const std::string& get_type() const override;

  /// Get collision detector type for this class
  static const std::string& GetType();

  // Documentation inherited
  bool collide(
      Object<Scalar>* object1,
      Object<Scalar>* object2,
      const CollisionOption<Scalar>& option = {},
      CollisionResult<Scalar>* result = nullptr) override;

protected:
  /// Constructor
  FclEngine(
      common::MemoryManager& memory_manager
      = common::MemoryManager::GetDefault());

  // Documentation inherited
  Scene<Scalar>* create_scene_impl() override;

  // Documentation inherited
  void destroy_scene_impl(Scene<Scalar>* scene) override;

  // Documentation inherited
  const common::ArrayForBasePtr<Scene<Scalar>>& get_scenes() const override;

  // Documentation inherited
  common::ArrayForBasePtr<Scene<Scalar>>& get_mutable_scenes() override;

  /// Returns fcl::CollisionGeometry for a shape
  std::shared_ptr<FclCollisionGeometry<Scalar>> create_fcl_collision_geometry(
      const math::ConstGeometryPtr& shape);

private:
  friend class FclScene<Scalar>;

  common::ArrayForDerivedPtr<Scene<Scalar>, FclScene<Scalar>> m_scenes;

  PrimitiveShape m_primitive_shape_type = PrimitiveShape::PRIMITIVE;

  //  DART_REGISTER_ENGINE_IN_HEADER(FclEngine<Scalar>);
};

// DART_REGISTER_ENGINE_OUT_HEADER(FclEngine<Scalar>);
DART_TEMPLATE_CLASS_HEADER(COLLISION, FclEngine)

} // namespace collision
} // namespace dart

#include "dart/collision/fcl/detail/fcl_engine_impl.hpp"
