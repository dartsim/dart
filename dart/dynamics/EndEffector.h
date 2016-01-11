/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef KIDO_DYNAMICS_ENDEFFECTOR_H_
#define KIDO_DYNAMICS_ENDEFFECTOR_H_

#include "dart/dynamics/FixedFrame.h"
#include "dart/dynamics/TemplatedJacobianNode.h"
//#include "dart/math/Geometry.h"

namespace kido {
namespace dynamics {

class BodyNode;
class Skeleton;
class EndEffector;

class Support // Inherit the Addon class once it is implemented
{
public:

  /// Constructor
  Support(EndEffector* _ee);

  /// Set the support geometry for this EndEffector. The SupportGeometry
  /// represents points in the EndEffector frame that can be used for contact
  /// when solving balancing or manipulation constraints.
  void setGeometry(const math::SupportGeometry& _newSupport);

  /// Get a const-reference to the SupportGeometry for this EndEffector
  const math::SupportGeometry& getGeometry() const;

  /// Pass in true if this EndEffector should be used to support the robot, like
  /// a foot
  void setActive(bool _supporting = true);

  /// Get whether this EndEffector is currently being used for support
  bool isActive() const;

protected:

  /// The support geometry that this EndEffector is designed to use
  math::SupportGeometry mGeometry;

  /// True if this EndEffector is currently usable for support
  bool mActive;

  /// EndEffector that this support is associated with
  EndEffector* mEndEffector;

};

class EndEffector : public FixedFrame,
                    public AccessoryNode,
                    public TemplatedJacobianNode<EndEffector>
{
public:

  friend class Skeleton;
  friend class BodyNode;

  struct UniqueProperties
  {
    /// The relative transform will be set to this whenever
    /// resetRelativeTransform() is called
    Eigen::Isometry3d mDefaultTransform;

    UniqueProperties(
        const Eigen::Isometry3d& _defaultTransform =
            Eigen::Isometry3d::Identity(),
        const math::SupportGeometry& _supportGeometry =
            math::SupportGeometry(),
        bool _supporting = false);
  };

  struct Properties : Entity::Properties, UniqueProperties
  {
    Properties(
        const Entity::Properties& _entityProperties = Entity::Properties(),
        const UniqueProperties& _effectorProperties = UniqueProperties() );
  };

  /// Destructor
  virtual ~EndEffector();

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform.
  void setProperties(const Properties& _properties, bool _useNow=true);

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform.
  void setProperties(const UniqueProperties& _properties, bool _useNow=true);

  Properties getEndEffectorProperties() const;

  /// Copy the Properties of another EndEffector
  void copy(const EndEffector& _otherEndEffector);

  /// Copy the Properties of another EndEffector
  void copy(const EndEffector* _otherEndEffector);

  /// Copy the Properties of another EndEffector
  EndEffector& operator=(const EndEffector& _otherEndEffector);

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name) override;

  /// Set the current relative transform of this EndEffector
  void setRelativeTransform(const Eigen::Isometry3d& _newRelativeTf);

  /// Set the default relative transform of this EndEffector. The relative
  /// transform of this EndEffector will be set to _newDefaultTf the next time
  /// resetRelativeTransform() is called. If _useNow is set to true, then
  /// resetRelativeTransform() will be called at the end of this function.
  void setDefaultRelativeTransform(const Eigen::Isometry3d& _newDefaultTf,
                                   bool _useNow);

  /// Set the current relative transform of this EndEffector to the default
  /// relative transform of this EndEffector. The default relative transform can
  /// be set with setDefaultRelativeTransform()
  void resetRelativeTransform();

  /// Get a pointer to the Support Addon for this EndEffector. If _createIfNull
  /// is true, then the Support will be generated if one does not already exist.
  Support* getSupport(bool _createIfNull = false);

  /// Get a pointer to the Support Addon for this EndEffector.
  const Support* getSupport() const;

  /// Create a new Support Addon for this EndEffector. If a Support Addon
  /// already exists for this EndEffector, it will be deleted and replaced.
  Support* createSupport();

  /// Erase the Support Addon from this EndEffector
  void eraseSupport();

  // Documentation inherited
  std::shared_ptr<Skeleton> getSkeleton() override;

  // Documentation inherited
  std::shared_ptr<const Skeleton> getSkeleton() const override;

  // Documentation inherited
  bool dependsOn(size_t _genCoordIndex) const override;

  // Documentation inherited
  size_t getNumDependentGenCoords() const override;

  // Documentation inherited
  size_t getDependentGenCoordIndex(size_t _arrayIndex) const override;

  // Documentation inherited
  const std::vector<size_t>& getDependentGenCoordIndices() const override;

  // Documentation inherited
  size_t getNumDependentDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDependentDof(size_t _index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDependentDof(size_t _index) const override;

  // Documentation inherited
  const std::vector<DegreeOfFreedom*>& getDependentDofs() override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*>& getDependentDofs() const override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*> getChainDofs() const override;

  /// Get the BodyNode that this EndEffector is rigidly attached to
  BodyNode* getParentBodyNode();

  /// Get the BodyNode that this EndEffector is rigidly attached to
  const BodyNode* getParentBodyNode() const;

  /// Get the index of this EndEffector within the Skeleton
  size_t getIndexInSkeleton() const;

  /// Get the tree index of the BodyNode that this EndEffector is attached to
  size_t getTreeIndex() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian Functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian& getJacobian() const override final;

  // Prevent the inherited getJacobian functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getJacobian;

  // Documentation inherited
  const math::Jacobian& getWorldJacobian() const override final;

  // Prevent the inherited getWorldJacobian functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getWorldJacobian;

  // Documentation inherited
  const math::Jacobian& getJacobianSpatialDeriv() const override final;

  // Prevent the inherited getJacobianSpatialDeriv functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getJacobianSpatialDeriv;

  // Documentation inherited
  const math::Jacobian& getJacobianClassicDeriv() const override final;

  // Prevent the inherited getJacobianClassicDeriv functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getJacobianClassicDeriv;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Notifications
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void notifyTransformUpdate() override;

  // Documentation inherited
  virtual void notifyVelocityUpdate() override;

protected:

  /// Constructor used by the Skeleton class
  explicit EndEffector(BodyNode* _parent, const Properties& _properties);

  /// Create a clone of this BodyNode. This may only be called by the Skeleton
  /// class.
  virtual EndEffector* clone(BodyNode* _parent) const;

  /// Update the Jacobian of this EndEffector. getJacobian() calls this function
  /// if mIsEffectorJacobianDirty is true.
  void updateEffectorJacobian() const;

  /// Update the World Jacobian cache.
  void updateWorldJacobian() const;

  /// Update the spatial time derivative of the end effector Jacobian.
  /// getJacobianSpatialDeriv() calls this function if
  /// mIsEffectorJacobianSpatialDerivDirty is true.
  void updateEffectorJacobianSpatialDeriv() const;

  /// Update the classic time derivative of the end effector Jacobian.
  /// getJacobianClassicDeriv() calls this function if
  /// mIsWorldJacobianClassicDerivDirty is true.
  void updateWorldJacobianClassicDeriv() const;

  /// Properties of this EndEffector
  UniqueProperties mEndEffectorP;

  /// The index of this EndEffector within its Skeleton
  size_t mIndexInSkeleton;

  /// The index of this EndEffector within its BodyNode
  size_t mIndexInBodyNode;

  /// TODO(MXG): When Addons are implemented, this should be changed
  std::unique_ptr<Support> mSupport;

  /// Cached Jacobian of this EndEffector
  ///
  /// Do not use directly! Use getJacobian() to access this quantity
  mutable math::Jacobian mEffectorJacobian;

  /// Cached World Jacobian of this EndEffector
  ///
  /// Do not use directly! Use getWorldJacobian() to access this quantity
  mutable math::Jacobian mWorldJacobian;

  /// Spatial time derivative of end effector Jacobian
  ///
  /// Do not use directly! Use getJacobianSpatialDeriv() to access this quantity
  mutable math::Jacobian mEffectorJacobianSpatialDeriv;

  /// Classic time derivative of the end effector Jacobian
  ///
  /// Do not use directly! Use getJacobianClassicDeriv() to access this quantity
  mutable math::Jacobian mWorldJacobianClassicDeriv;
};

} // namespace dynamics
} // namespace kido


#endif // KIDO_DYNAMICS_ENDEFFECTOR_H_
