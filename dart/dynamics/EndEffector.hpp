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

#ifndef DART_DYNAMICS_ENDEFFECTOR_HPP_
#define DART_DYNAMICS_ENDEFFECTOR_HPP_

#include "dart/common/Addon.hpp"
#include "dart/common/SpecializedForAddon.hpp"
#include "dart/common/AddonWithVersion.hpp"
#include "dart/dynamics/FixedFrame.hpp"
#include "dart/dynamics/TemplatedJacobianNode.hpp"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;
class EndEffector;

class Support;
namespace detail
{

struct SupportStateData
{
  inline SupportStateData(bool active = false) : mActive(active) { }

  bool mActive;
};

struct SupportPropertiesData
{
  math::SupportGeometry mGeometry;
};

void SupportUpdate(Support* support);

} // namespace detail

class Support final :
    public common::AddonWithStateAndVersionedProperties<
        Support,
        detail::SupportStateData,
        detail::SupportPropertiesData,
        EndEffector,
        &detail::SupportUpdate>
{
public:

//  DART_COMMON_ADDON_STATE_PROPERTY_CONSTRUCTORS( Support, &detail::SupportUpdate, &detail::SupportUpdate )
  DART_COMMON_ADDON_STATE_PROPERTY_CONSTRUCTORS(Support)

  /// Set/Get the support geometry for this EndEffector. The SupportGeometry
  /// represents points in the EndEffector frame that can be used for contact
  /// when solving balancing or manipulation constraints.
  DART_COMMON_SET_GET_ADDON_PROPERTY(math::SupportGeometry, Geometry)
  // void setGeometry(const math::SupportGeometry&);
  // const math::SupportGeometry& getGeometry() const;

  /// Pass in true if this EndEffector should be used to support the robot, like
  /// a foot
  void setActive(bool _supporting = true);

  /// Get whether this EndEffector is currently being used for support
  bool isActive() const;

};

class EndEffector final :
    public virtual common::SpecializedForAddon<Support>,
    public FixedFrame,
    public AccessoryNode<EndEffector>,
    public TemplatedJacobianNode<EndEffector>
{
public:

  friend class Skeleton;
  friend class BodyNode;

  struct StateData
  {
    StateData(const Eigen::Isometry3d& relativeTransform =
                  Eigen::Isometry3d::Identity(),
              const common::AddonManager::State& addonStates =
                  common::AddonManager::State());

    /// The current relative transform of the EndEffector
    // TODO(MXG): Consider moving this to a FixedFrame::State struct and then
    // inheriting that struct
    Eigen::Isometry3d mRelativeTransform;

    /// The current states of the EndEffector's Addons
    common::AddonManager::State mAddonStates;
  };

  using State = Node::StateMixer<StateData>;

  struct UniqueProperties
  {
    /// The relative transform will be set to this whenever
    /// resetRelativeTransform() is called
    Eigen::Isometry3d mDefaultTransform;

    /// Default constructor
    UniqueProperties(
        const Eigen::Isometry3d& _defaultTransform =
            Eigen::Isometry3d::Identity());
  };

  struct PropertiesData : Entity::Properties, UniqueProperties
  {
    PropertiesData(
        const Entity::Properties& _entityProperties = Entity::Properties(),
        const UniqueProperties& _effectorProperties = UniqueProperties(),
        const common::AddonManager::Properties& _addonProperties =
            common::AddonManager::Properties());

    /// The properties of the EndEffector's Addons
    common::AddonManager::Properties mAddonProperties;
  };

  using Properties = Node::PropertiesMixer<PropertiesData>;

  /// Destructor
  virtual ~EndEffector() = default;

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Set the State of this EndEffector.
  void setState(const StateData& _state);

  /// Set the State of this EndEffector using move semantics
  void setState(StateData&& _state);

  /// Get the State of this EndEffector
  StateData getEndEffectorState() const;

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform.
  void setProperties(const PropertiesData& _properties, bool _useNow=false);

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform.
  void setProperties(const UniqueProperties& _properties, bool _useNow=false);

  /// Get the Properties of this EndEffector
  PropertiesData getEndEffectorProperties() const;

  /// Copy the State and Properties of another EndEffector
  void copy(const EndEffector& _otherEndEffector);

  /// Copy the State and Properties of another EndEffector
  void copy(const EndEffector* _otherEndEffector);

  /// Copy the State and Properties of another EndEffector
  EndEffector& operator=(const EndEffector& _otherEndEffector);

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name) override;

  // Documentation inherited
  void setNodeState(const Node::State& otherState) override final;

  // Documentation inherited
  std::unique_ptr<Node::State> getNodeState() const override final;

  // Documentation inherited
  void copyNodeStateTo(std::unique_ptr<Node::State>& outputState) const override final;

  // Documentation inherited
  void setNodeProperties(const Node::Properties& otherProperties) override final;

  // Documentation inherited
  std::unique_ptr<Node::Properties> getNodeProperties() const override final;

  // Documentation inherited
  void copyNodePropertiesTo(
      std::unique_ptr<Node::Properties>& outputProperties) const override final;

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

  DART_BAKE_SPECIALIZED_ADDON(Support)

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
  explicit EndEffector(BodyNode* _parent, const PropertiesData& _properties);

  /// Create a clone of this BodyNode. This may only be called by the Skeleton
  /// class.
  virtual Node* cloneNode(BodyNode* _parent) const override;

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
} // namespace dart


#endif // DART_DYNAMICS_ENDEFFECTOR_HPP_
