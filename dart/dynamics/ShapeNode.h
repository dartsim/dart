/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_SHAPENODE_H_
#define DART_DYNAMICS_SHAPENODE_H_

#include <Eigen/Dense>

#include "dart/common/Signal.h"
#include "dart/dynamics/ShapeFrame.h"
#include "dart/dynamics/FixedJacobianNode.h"
#include "dart/dynamics/CompositeNode.h"

namespace dart {
namespace dynamics {

namespace detail {

using ShapeNodeCompositeBase = CompositeNode<
    common::CompositeJoiner<
        FixedJacobianNode,
        ShapeFrame
    >
>;

} // namespace detail

class VisualAspect;
class CollisionAspect;
class DynamicsAspect;
class ShapeFrame;

class ShapeNode : public detail::ShapeNodeCompositeBase
{
public:

  friend class BodyNode;

  using ShapeUpdatedSignal
      = common::Signal<void(const ShapeNode* thisShapeNode,
                            const ShapePtr& oldShape,
                            const ShapePtr& newShape)>;

  using RelativeTransformUpdatedSignal
      = common::Signal<void(const ShapeNode* thisShapeNode,
                            const Eigen::Isometry3d& oldTransform,
                            const Eigen::Isometry3d& newTransform)>;

  using CompositeProperties = common::Composite::Properties;

  struct UniqueProperties
  {
    /// Name of ShapeNode
    std::string mName;

    /// The current relative transform of the Shape in the ShapeNode
    Eigen::Isometry3d mRelativeTransform;
    // TODO(JS): Consider moving this to a FixedFrame::State
    // (or FixedFrame::Properties) struct and the inheriting that struct.
    // Endeffector has similar issue.

    /// Composed constructor
    UniqueProperties(
        const std::string& name = "",
        const Eigen::Isometry3d& relativeTransform
            = Eigen::Isometry3d::Identity());
  };

  struct Properties : ShapeFrame::Properties, UniqueProperties
  {
    /// Composed constructor
    Properties(
        const ShapeFrame::Properties& shapeFrameProperties
            = ShapeFrame::Properties(),
        const ShapeNode::UniqueProperties& shapeNodeProperties
            = ShapeNode::UniqueProperties(),
        const CompositeProperties& compositeProperties = CompositeProperties());

    /// Composed move constructor
    Properties(
        ShapeFrame::Properties&& shapeFrameProperties,
        ShapeNode::UniqueProperties&& shapeNodeProperties,
        CompositeProperties&& compositeProperties);

    /// The properties of the ShapeNode's Aspects
    CompositeProperties mCompositeProperties;
  };

  /// Destructor
  virtual ~ShapeNode() = default;

  /// Set the Properties of this ShapeNode
  void setProperties(const Properties& properties);

  /// Set the Properties of this ShapeNode
  void setProperties(const UniqueProperties& properties);

  /// Get the Properties of this ShapeNode
  const Properties getShapeNodeProperties() const;

  /// Copy the properties of another ShapeNode
  void copy(const ShapeNode& other);

  /// Copy the properties of another ShapeNode
  void copy(const ShapeNode* other);

  /// Same as copy(const ShapeNode&)
  ShapeNode& operator=(const ShapeNode& other);

  /// Set transformation of this shape node relative to the parent frame
  void setRelativeTransform(const Eigen::Isometry3d& transform) override;

  /// Set rotation of this shape node relative to the parent frame
  void setRelativeRotation(const Eigen::Matrix3d& rotation);

  /// Get rotation of this shape node relative to the parent frame
  Eigen::Matrix3d getRelativeRotation() const;

  /// Set translation of this shape node relative to the parent frame
  void setRelativeTranslation(const Eigen::Vector3d& translation);

  /// Same as setRelativeTranslation(offset)
  void setOffset(const Eigen::Vector3d& offset);

  /// Get translation of this shape node relative to the parent frame
  Eigen::Vector3d getRelativeTranslation() const;

  /// Same as getRelativeTranslation()
  Eigen::Vector3d getOffset() const;

protected:

  /// Constructor used by the Skeleton class
  ShapeNode(BodyNode* bodyNode, const Properties& properties);

  /// Constructor used by the Skeleton class
  ShapeNode(BodyNode* bodyNode, const ShapePtr& shape,
            const std::string& name = "ShapeNode");

  /// Create a clone of this ShapeNode. This may only be called by the Skeleton
  /// class.
  Node* cloneNode(BodyNode* parent) const override;

protected:

  /// Properties of this ShapeNode
  DEPRECATED(6.0)
  UniqueProperties mShapeNodeP;

  /// Shape updated signal
  ShapeUpdatedSignal mShapeUpdatedSignal;

  /// Relative transformation updated signal
  RelativeTransformUpdatedSignal mRelativeTransformUpdatedSignal;

public:

  /// Slot register for shape updated signal
  common::SlotRegister<ShapeUpdatedSignal> onShapeUpdated;

  /// Slot register for relative transformation updated signal
  common::SlotRegister<RelativeTransformUpdatedSignal>
      onRelativeTransformUpdated;

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SHAPENODE_H_
