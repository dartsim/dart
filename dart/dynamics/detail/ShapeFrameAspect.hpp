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

#ifndef DART_DYNAMICS_DETAIL_SHAPEFRAMEASPECT_HPP_
#define DART_DYNAMICS_DETAIL_SHAPEFRAMEASPECT_HPP_

#include "dart/common/EmbeddedAspect.hpp"
#include "dart/dynamics/SmartPointer.hpp"

#include <Eigen/Core>

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
  /// Primary coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Secondary coefficient of friction
  double mSecondaryFrictionCoeff;

  /// Primary slip compliance coefficient
  double mPrimarySlipCompliance;

  /// Secondary slip compliance coefficient
  double mSecondarySlipCompliance;

  /// First friction direction unit vector
  Eigen::Vector3d mFirstFrictionDirection;

  /// First friction direction frame
  /// The first friction direction unit vector is expressed in this frame
  const Frame* mFirstFrictionDirectionFrame;

  /// Constructors
  /// The frictionCoeff argument will be used for both primary and secondary
  /// friction
  DynamicsAspectProperties(
      const double frictionCoeff = 1.0, const double restitutionCoeff = 0.0);

  /// Set primary and secondary friction and restitution coefficients.
  /// The first friction direction vector and frame may optionally be set.
  /// The vector defaults to a zero-vector, which will cause it to be ignored
  /// and the global friction directions used instead.
  /// If the vector is set to a non-zero vector, the first friction direction
  /// for this shape is computed from this vector expressed in the frame
  /// given by the Frame pointer. THe Frame pointer defaults to nullptr,
  /// which is interpreted as the body-fixed frame of this Shape.
  /// Note that if two objects with custom friction directions come into
  /// contact, only one of the directions can be chosen.
  /// One approach is to use the first friction direction for the ShapeNode
  /// with the smaller primary friction coefficient, since that has the
  /// dominant effect. See the ContactConstraint implementation for
  /// further details.
  DynamicsAspectProperties(
      const double primaryFrictionCoeff,
      const double secondaryFrictionCoeff,
      const double restitutionCoeff,
      const double primarySlipCompliance = -1.0,
      const double secondarySlipCompliance = -1.0,
      const Eigen::Vector3d& firstFrictionDirection = Eigen::Vector3d::Zero(),
      const Frame* firstFrictionDirectionFrame = nullptr);

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
    ShapeFrame,
    ShapeFrameProperties,
    common::
        SpecializedForAspect<VisualAspect, CollisionAspect, DynamicsAspect> >;

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_SHAPEFRAMEASPECT_HPP_
