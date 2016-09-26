/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_DYNAMICS_DEGREEOFFREEDOM_HPP_
#define DART_DYNAMICS_DEGREEOFFREEDOM_HPP_

#include <string>
#include <memory>
#include <Eigen/Core>

#include "dart/common/Subject.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {

class Skeleton;
class Joint;
class BodyNode;

/// DegreeOfFreedom class is a proxy class for accessing single degrees of
/// freedom (aka generalized coordinates) of the Skeleton.
class DegreeOfFreedom : public virtual common::Subject
{
public:

  friend class Joint;
  template<class> friend class GenericJoint;
  friend class Skeleton;

  DegreeOfFreedom(const DegreeOfFreedom&) = delete;

  /// Change the name of this DegreeOfFreedom
  ///
  /// The _preserveName argument will be passed to the preserveName(bool)
  /// function. Set _preserveName to true when customizing the name of the
  /// DegreeOfFreedom; that way the name will not be overwritten if the Joint
  /// name changes.
  const std::string& setName(const std::string& _name, bool _preserveName=true);

  /// \brief Get the name of this DegreeOfFreedom
  ///
  /// DegreeOfFreedom's name will be automatically given by the joint it belongs
  /// to. Below is the naming policy:
  ///   - GenericJoint<RealSpace> \n
  ///       Same name as the joint it belongs to.
  ///   - GenericJoint<[multi_dof_joint_space]> \n
  ///       "[Joint_name]+[affix]" is used. The affix is determined according
  ///       to the role they play in the joint. For example, suppose there's a
  ///       TranslationalJoint named "trans_joint". Then the each dof to be
  ///       named "trans_joint_x", "trans_joint_y", and "trans_joint_z".
  ///   - ZeroDofJoint \n
  ///       ZeroDofJoint doesn't have dof.
  ///
  /// The default name can be renamed by setName() as well.
  const std::string& getName() const;

  /// Prevent Joint::updateDegreeOfFreedomNames() from changing the name of this
  /// degree of freedom. This is useful if you (the user) have customized the
  /// name for this DegreeOfFreedom and want to prevent DART from automatically
  /// updating its name if its parent Joint properties ever change.
  void preserveName(bool _preserve);

  /// Check whether DegreeOfFreedom::lockName(bool) is activate
  bool isNamePreserved() const;

  /// Get this DegreeOfFreedom's index within its Skeleton
  std::size_t getIndexInSkeleton() const;

  /// Get this DegreeOfFreedom's index within its tree
  std::size_t getIndexInTree() const;

  /// Get this DegreeOfFreedom's index within its Joint
  std::size_t getIndexInJoint() const;

  /// Get the index of the tree that this DegreeOfFreedom belongs to
  std::size_t getTreeIndex() const;

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set the command of this DegreeOfFreedom
  void setCommand(double _command);

  /// Get the command of this DegreeOfFreedom
  double getCommand() const;

  /// Set the command of this DegreeOfFreedom to zero
  void resetCommand();

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  /// Set the position of this DegreeOfFreedom
  void setPosition(double _position);

  /// Get the position of this DegreeOfFreedom
  double getPosition() const;

  /// Set the position limits of this DegreeOfFreedom
  void setPositionLimits(double _lowerLimit, double _upperLimit);

  /// Set the position limits of this DegreeOfFreedom
  void setPositionLimits(const std::pair<double,double>& _limits);

  /// Get the position limits of this DegreeOfFreedom
  std::pair<double,double> getPositionLimits() const;

  /// Set the lower position limit of this DegreeOfFreedom
  void setPositionLowerLimit(double _limit);

  /// Get the lower position limit of this DegreeOfFreedom
  double getPositionLowerLimit() const;

  /// Set the upper position limit of this DegreeOfFreedom
  void setPositionUpperLimit(double _limit);

  /// Get the upper position limit of this DegreeOfFreedom
  double getPositionUpperLimit() const;

  /// Get whether this DOF is cyclic. Return true if and only if an infinite
  /// number of DOF positions produce the same local transform. If this DOF is
  /// part of a multi-DOF joint, then producing a cycle may require altering
  /// the position of the Joint's other DOFs.
  bool isCyclic() const;

  /// Get whether the position of this DegreeOfFreedom has limits.
  bool hasPositionLimit() const;

  /// Set the position of this DegreeOfFreedom to zero
  void resetPosition();

  /// Change the position that resetPosition() will give to this DegreeOfFreedom
  void setInitialPosition(double _initial);

  /// Get the position that resetPosition() will give to this DegreeOfFreedom
  double getInitialPosition() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  /// Set the velocity of this DegreeOfFreedom
  void setVelocity(double _velocity);

  /// Get the velocity of this DegreeOfFreedom
  double getVelocity() const;

  /// Set the velocity limits of this DegreeOfFreedom
  void setVelocityLimits(double _lowerLimit, double _upperLimit);

  /// Set the velocity limtis of this DegreeOfFreedom
  void setVelocityLimits(const std::pair<double,double>& _limits);

  /// Get the velocity limits of this DegreeOfFreedom
  std::pair<double,double> getVelocityLimits() const;

  /// Set the lower velocity limit of this DegreeOfFreedom
  void setVelocityLowerLimit(double _limit);

  /// Get the lower velocity limit of this DegreeOfFreedom
  double getVelocityLowerLimit() const;

  /// Set the upper velocity limit of this DegreeOfFreedom
  void setVelocityUpperLimit(double _limit);

  /// Get the upper Velocity limit of this DegreeOfFreedom
  double getVelocityUpperLimit() const;

  /// Set the velocity of this DegreeOfFreedom to zero
  void resetVelocity();

  /// Change the velocity that resetVelocity() will give to this DegreeOfFreedom
  void setInitialVelocity(double _initial);

  /// Get the velocity that resetVelocity() will give to this DegreeOfFreedom
  double getInitialVelocity() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  /// Set the acceleration of this DegreeOfFreedom
  void setAcceleration(double _acceleration);

  /// Get the acceleration of this DegreeOfFreedom
  double getAcceleration() const;

  /// Set the acceleration of this DegreeOfFreedom to zero
  void resetAcceleration();

  /// Set the acceleration limits of this DegreeOfFreedom
  void setAccelerationLimits(double _lowerLimit, double _upperLimit);

  /// Set the acceleartion limits of this DegreeOfFreedom
  void setAccelerationLimits(const std::pair<double,double>& _limits);

  /// Get the acceleration limits of this DegreeOfFreedom
  std::pair<double,double> getAccelerationLimits() const;

  /// Set the lower acceleration limit of this DegreeOfFreedom
  void setAccelerationLowerLimit(double _limit);

  /// Get the lower acceleration limit of this DegreeOfFreedom
  double getAccelerationLowerLimit() const;

  /// Set the upper acceleration limit of this DegreeOfFreedom
  void setAccelerationUpperLimit(double _limit);

  /// Get the upper acceleration limit of this DegreeOfFreedom
  double getAccelerationUpperLimit() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  /// Set the generalized force of this DegreeOfFreedom
  void setForce(double _force);

  /// Get the generalized force of this DegreeOfFreedom
  double getForce() const;

  /// Set the generalized force of this DegreeOfFreedom to zero
  void resetForce();

  /// Set the generalized force limits of this DegreeOfFreedom
  void setForceLimits(double _lowerLimit, double _upperLimit);

  /// Set the generalized force limits of this DegreeOfFreedom
  void setForceLimits(const std::pair<double,double>& _limits);

  /// Get the generalized force limits of this DegreeOfFreedom
  std::pair<double,double> getForceLimits() const;

  /// Set the lower generalized force limit of this DegreeOfFreedom
  void setForceLowerLimit(double _limit);

  /// Get the lower generalized force limit of this DegreeOfFreedom
  double getForceLowerLimit() const;

  /// Set the upper generalized force limit of this DegreeOfFreedom
  void setForceUpperLimit(double _limit);

  /// Get the upper generalized force limit of this DegreeOfFreedom
  double getForceUpperLimit() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity change
  //----------------------------------------------------------------------------

  /// Set the velocity change of this DegreeOfFreedom
  void setVelocityChange(double _velocityChange);

  /// Get the velocity change of this DegreeOfFreedom
  double getVelocityChange() const;

  /// Set the velocity change of this DegreeOfFreedom to zero
  void resetVelocityChange();

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Constraint impulse
  //----------------------------------------------------------------------------

  /// Set the constraint impulse of this generalized coordinate
  void setConstraintImpulse(double _impulse);

  /// Get the constraint impulse of this generalized coordinate
  double getConstraintImpulse() const;

  /// Set the constraint impulse of this generalized coordinate to zero
  void resetConstraintImpulse();

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Passive forces - spring, viscous friction, Coulomb friction
  //----------------------------------------------------------------------------

  /// Set stiffness of the spring force for this generalized coordinate
  void setSpringStiffness(double _k);

  /// Get stiffness of the spring force for this generalized coordinate
  double getSpringStiffness() const;

  /// Set rest position for the spring force of this generalized coordinate
  void setRestPosition(double _q0);

  /// Get rest position for the spring force of this generalized coordinate
  double getRestPosition() const;

  /// Set coefficient of damping (viscous friction) force for this generalized
  /// coordinate
  void setDampingCoefficient(double _coeff);

  /// Get coefficient of damping (viscous friction) force for this generalized
  /// coordinate
  double getDampingCoefficient() const;

  /// Set Coulomb friction force for this generalized coordinate
  void setCoulombFriction(double _friction);

  /// Get Coulomb friction force for this generalized coordinate
  double getCoulombFriction() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Relationships
  //----------------------------------------------------------------------------

  /// Get the Joint that this DegreeOfFreedom belongs to
  Joint* getJoint();

  /// Get the Joint that this DegreeOfFreedom belongs to
  const Joint* getJoint() const;

  /// Get the Skeleton that this DegreeOfFreedom is inside of
  SkeletonPtr getSkeleton();

  /// Get the Skeleton that this DegreeOfFreedom is inside of
  ConstSkeletonPtr getSkeleton() const;

  /// Get the BodyNode downstream of this DegreeOfFreedom
  BodyNode* getChildBodyNode();

  /// Get the BodyNode downstream of this DegreeOfFreedom
  const BodyNode* getChildBodyNode() const;

  /// Get the BodyNode upstream of this DegreeOfFreedom
  BodyNode* getParentBodyNode();

  /// Get the BodyNode upstream of this DegreeOfFreedom
  const BodyNode* getParentBodyNode() const;

  /// \}

protected:
  /// The constructor is protected so that only Joints can create
  /// DegreeOfFreedom classes
  DegreeOfFreedom(Joint* _joint, std::size_t _indexInJoint);

  /// \brief Index of this DegreeOfFreedom within its Joint
  ///
  /// The index is determined when this DegreeOfFreedom is created by the Joint
  /// it belongs to. Note that the index should be unique within the Joint.
  std::size_t mIndexInJoint;

  /// Index of this DegreeOfFreedom within its Skeleton
  std::size_t mIndexInSkeleton;

  /// Index of this DegreeOfFreedom within its tree
  std::size_t mIndexInTree;

  /// The joint that this DegreeOfFreedom belongs to
  Joint* mJoint;
  // Note that we do not need to store BodyNode or Skeleton, because we can
  // access them through this joint pointer. Moreover, we never need to check
  // whether mJoint is nullptr, because only Joints are allowed to create a
  // DegreeOfFreedom and every DegreeOfFreedom is deleted when its Joint is
  // destructed.

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DEGREEOFFREEDOM_HPP_
