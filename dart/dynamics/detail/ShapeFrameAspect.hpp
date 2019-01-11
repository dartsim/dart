/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_DYNAMICS_DETAIL_SHAPEFRAMEASPECT_HPP_
#define DART_DYNAMICS_DETAIL_SHAPEFRAMEASPECT_HPP_

#include <Eigen/Core>

#include "dart/common/EmbeddedAspect.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {

class VisualAspect;
class CollisionAspect;
class DynamicsAspect;
class ShapeFrame;

namespace detail {

struct VisualAspectProperties
{
  /// Color for the primitive shape
  Eigen::Vector4d mRGBA;

  bool mUseDefaultColor;

  /// True if this shape node should be kept from rendering
  bool mHidden;

  /// True if this shape node should be shadowed
  bool mShadowed;

  /// Constructor
  VisualAspectProperties(
      const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 1.0, 1.0),
      const bool hidden = false,
      const bool shadowed = true);

  /// Destructor
  virtual ~VisualAspectProperties() = default;

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CollisionAspectProperties
{
  /// This object is collidable if true
  bool mCollidable;

  /// Constructor
  CollisionAspectProperties(const bool collidable = true);

  /// Destructor
  virtual ~CollisionAspectProperties() = default;
};

struct DynamicsAspectProperties
{
  /// Coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Constructor
  DynamicsAspectProperties(const double frictionCoeff = 1.0,
                           const double restitutionCoeff = 0.0);

  /// Destructor
  virtual ~DynamicsAspectProperties() = default;
};

struct ShapeFrameProperties
{
  /// Pointer to a shape
  ShapePtr mShape;

  /// Constructor
  ShapeFrameProperties(const ShapePtr& shape = nullptr);

  /// Virtual destructor
  virtual ~ShapeFrameProperties() = default;
};

using ShapeFrameCompositeBase = common::EmbedPropertiesOnTopOf<
    ShapeFrame, ShapeFrameProperties,
    common::SpecializedForAspect<
        VisualAspect, CollisionAspect, DynamicsAspect> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SHAPEFRAMEASPECT_HPP_
