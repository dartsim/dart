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

#ifndef DART_COLLISION_FCL_COMPAT_FCLCOLLISIONDETECTOR_HPP_
#define DART_COLLISION_FCL_COMPAT_FCLCOLLISIONDETECTOR_HPP_

#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/collision/dart/dart_collision_group.hpp>
#include <dart/collision/detail/legacy_deprecation.hpp>

#include <string>

namespace dart {
namespace collision {

/// Compatibility facade for legacy FCL detector code.
///
/// This class does not select the FCL engine. Runtime use is backed by the
/// built-in DART collision detector; the retained FCL-specific settings are
/// compatibility state for source compatibility only. Reference tests and
/// benchmarks that intentionally compare against FCL must use the explicit
/// reference header and `createReference()` API in a reference-enabled target.
class DART_COLLISION_LEGACY_NAME_DEPRECATED FCLCollisionDetector
  : public DartCollisionDetector
{
public:
  enum PrimitiveShape
  {
    PRIMITIVE = 0,
    MESH
  };

  enum ContactPointComputationMethod
  {
    FCL = 0,
    DART
  };

  static std::shared_ptr<FCLCollisionDetector> create()
  {
    return std::shared_ptr<FCLCollisionDetector>(new FCLCollisionDetector());
  }

  const std::string& getType() const override
  {
    return getStaticType();
  }

  static const std::string& getStaticType()
  {
    static const std::string type = "fcl";
    return type;
  }

  bool raycast(
      CollisionGroup* group,
      const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,
      const RaycastOption& option = RaycastOption(),
      RaycastResult* result = nullptr) override
  {
    // gz-physics treats the legacy "fcl" facade as unsupported for raycasts.
    return CollisionDetector::raycast(group, from, to, option, result);
  }

  std::shared_ptr<CollisionDetector> cloneWithoutCollisionObjects()
      const override
  {
    auto clone = FCLCollisionDetector::create();
    clone->setPrimitiveShapeType(mPrimitiveShapeType);
    clone->setContactPointComputationMethod(mContactPointComputationMethod);
    return clone;
  }

  void setPrimitiveShapeType(PrimitiveShape type)
  {
    mPrimitiveShapeType = type;
  }

  PrimitiveShape getPrimitiveShapeType() const
  {
    return mPrimitiveShapeType;
  }

  void setContactPointComputationMethod(ContactPointComputationMethod method)
  {
    mContactPointComputationMethod = method;
  }

  ContactPointComputationMethod getContactPointComputationMethod() const
  {
    return mContactPointComputationMethod;
  }

protected:
  FCLCollisionDetector() = default;

private:
  PrimitiveShape mPrimitiveShapeType{PRIMITIVE};
  ContactPointComputationMethod mContactPointComputationMethod{DART};
};

using FCLCollisionGroup = DartCollisionGroup;

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_FCL_COMPAT_FCLCOLLISIONDETECTOR_HPP_
