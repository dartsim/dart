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
#include "dart/dynamics/FixedFrame.h"
#include "dart/dynamics/ShapeFrame.h"
#include "dart/dynamics/Node.h"
#include "dart/dynamics/TemplatedJacobianNode.h"

namespace dart {
namespace dynamics {

class VisualAddon;
class CollisionAddon;
class DynamicsAddon;
class ShapeFrame;

class ShapeNode : public virtual FixedFrame,
                  public ShapeFrame,
                  public AccessoryNode<ShapeNode>,
                  public TemplatedJacobianNode<ShapeNode>
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

  using AddonProperties = common::AddonManager::Properties;

  struct Properties : ShapeFrame::Properties
  {
    /// Composed constructor
    Properties(
        const ShapeFrame::Properties& shapeFrameProperties
            = ShapeFrame::Properties(),
        const AddonProperties& addonProperties = AddonProperties());

    /// Composed move constructor
    Properties(
        ShapeFrame::Properties&& shapeFrameProperties,
        AddonProperties&& addonProperties);

    /// The properties of the ShapeNode's Addons
    AddonProperties mAddonProperties;
  };

  /// Destructor
  virtual ~ShapeNode() = default;

  /// Set the Properties of this ShapeNode
  void setProperties(const Properties& properties);

  /// Get the Properties of this ShapeNode
  const Properties getShapeNodeProperties() const;

  /// Copy the properties of another ShapeNode
  void copy(const ShapeNode& other);

  /// Copy the properties of another ShapeNode
  void copy(const ShapeNode* other);

  /// Same as copy(const ShapeNode&)
  ShapeNode& operator=(const ShapeNode& other);

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name) override;

  // Documentation inherited
  size_t incrementVersion() override;

  // Documentation inherited
  size_t getVersion() const override;

  /// Set transformation of this shape node relative to the parent frame
  void setRelativeTransform(const Eigen::Isometry3d& transform);

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

  // Documentation inherited
  std::shared_ptr<Skeleton> getSkeleton() override;

  // Documentation inherited
  std::shared_ptr<const Skeleton> getSkeleton() const override;

  // Documentation inherited
  bool dependsOn(size_t genCoordIndex) const override;

  // Documentation inherited
  size_t getNumDependentGenCoords() const override;

  // Documentation inherited
  size_t getDependentGenCoordIndex(size_t arrayIndex) const override;

  // Documentation inherited
  const std::vector<size_t>& getDependentGenCoordIndices() const override;

  // Documentation inherited
  size_t getNumDependentDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDependentDof(size_t index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDependentDof(size_t index) const override;

  // Documentation inherited
  const std::vector<DegreeOfFreedom*>& getDependentDofs() override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*>& getDependentDofs() const override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*> getChainDofs() const override;

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian Functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian& getJacobian() const override final;

  // Prevent the inherited getJacobian functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getJacobian;

  // Documentation inherited
  const math::Jacobian& getWorldJacobian() const override final;

  // Prevent the inherited getWorldJacobian functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getWorldJacobian;

  // Documentation inherited
  const math::Jacobian& getJacobianSpatialDeriv() const override final;

  // Prevent the inherited getJacobianSpatialDeriv functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getJacobianSpatialDeriv;

  // Documentation inherited
  const math::Jacobian& getJacobianClassicDeriv() const override final;

  // Prevent the inherited getJacobianClassicDeriv functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getJacobianClassicDeriv;

  /// \}

protected:

  /// Constructor used by the Skeleton class
  ShapeNode(BodyNode* bodyNode, const Properties& properties);

  /// Constructor used by the Skeleton class
  ShapeNode(BodyNode* bodyNode, const ShapePtr& shape,
            const std::string& name = "ShapeNode");

  /// Create a clone of this ShapeNode. This may only be called by the Skeleton
  /// class.
  Node* cloneNode(BodyNode* parent) const override;

  /// Update the Jacobian of this ShapeNode. getJacobian() calls this function
  /// if mIsShapeNodeJacobianDirty is true.
  void updateShapeNodeJacobian() const;

  /// Update the World Jacobian cache.
  void updateWorldJacobian() const;

  /// Update the spatial time derivative of the ShapeNode Jacobian.
  /// getJacobianSpatialDeriv() calls this function if
  /// mIsShapeNodeJacobianSpatialDerivDirty is true.
  void updateShapeNodeJacobianSpatialDeriv() const;

  /// Update the classic time derivative of the ShapeNode Jacobian.
  /// getJacobianClassicDeriv() calls this function if
  /// mIsWorldJacobianClassicDerivDirty is true.
  void updateWorldJacobianClassicDeriv() const;

protected:

  /// Cached Jacobian of this ShapeNode
  ///
  /// Do not use directly! Use getJacobian() to access this quantity
  mutable math::Jacobian mShapeNodeJacobian;

  /// Cached World Jacobian of this ShapeNode
  ///
  /// Do not use directly! Use getWorldJacobian() to access this quantity
  mutable math::Jacobian mWorldJacobian;

  /// Spatial time derivative of ShapeNode Jacobian
  ///
  /// Do not use directly! Use getJacobianSpatialDeriv() to access this quantity
  mutable math::Jacobian mShapeNodeJacobianSpatialDeriv;

  /// Classic time derivative of the ShapeNode Jacobian
  ///
  /// Do not use directly! Use getJacobianClassicDeriv() to access this quantity
  mutable math::Jacobian mWorldJacobianClassicDeriv;

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
