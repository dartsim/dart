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

#ifndef DART_DYNAMICS_METASKELETON_HPP_
#define DART_DYNAMICS_METASKELETON_HPP_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/common/LockableReference.hpp"
#include "dart/common/Signal.hpp"
#include "dart/common/Subject.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/InvalidIndex.hpp"

namespace dart {
namespace dynamics {

class BodyNode;
class SoftBodyNode;
class PointMass;
class Joint;
class DegreeOfFreedom;
class Marker;

/// MetaSkeleton is a pure abstract base class that provides a common interface
/// for obtaining data (such as Jacobians and Mass Matrices) from groups of
/// BodyNodes.
class MetaSkeleton : public common::Subject
{
public:
  using NameChangedSignal
      = common::Signal<void(std::shared_ptr<const MetaSkeleton> _skeleton,
                            const std::string& _oldName,
                            const std::string& _newName)>;

  MetaSkeleton(const MetaSkeleton&) = delete;

  /// Default destructor
  virtual ~MetaSkeleton() = default;

  /// Creates an identical clone of this MetaSkeleton
  virtual MetaSkeletonPtr cloneMetaSkeleton(
      const std::string& cloneName) const = 0;
  // TODO: In DART7, rename this to clone() and change the current
  // Skeleton::clone() to override it.

  /// Creates an identical clone of this MetaSkeleton
  MetaSkeletonPtr cloneMetaSkeleton() const;

  /// Returns mutex.
  virtual std::unique_ptr<common::LockableReference> getLockableReference()
  const = 0;
  // TODO: In DART7, rename this to getMutex() and change the current
  // Skeleton::getMutex() to override it.

  //----------------------------------------------------------------------------
  /// \{ \name Name
  //----------------------------------------------------------------------------

  /// Set the name of this MetaSkeleton
  virtual const std::string& setName(const std::string& _name) = 0;

  /// Get the name of this MetaSkeleton
  virtual const std::string& getName() const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Get number of body nodes
  virtual std::size_t getNumBodyNodes() const = 0;

  /// Get BodyNode whose index is _idx
  virtual BodyNode* getBodyNode(std::size_t _idx) = 0;

  /// Get const BodyNode whose index is _idx
  virtual const BodyNode* getBodyNode(std::size_t _idx) const = 0;

  /// Returns the BodyNode of given name.
  ///
  /// \param[in] name The BodyNode name that want to search.
  /// \return The body node of given name.
  virtual BodyNode* getBodyNode(const std::string& name) = 0;

  /// Returns the BodyNode of given name.
  ///
  /// \param[in] name The BodyNode name that want to search.
  /// \return The body node of given name.
  virtual const BodyNode* getBodyNode(const std::string& name) const = 0;

  /// Get all the BodyNodes that are held by this MetaSkeleton
  virtual const std::vector<BodyNode*>& getBodyNodes() = 0;

  /// Get all the BodyNodes that are held by this MetaSkeleton
  virtual const std::vector<const BodyNode*>& getBodyNodes() const = 0;

  /// Returns all the BodyNodes of given name.
  /// \param[in] name The BodyNode name that want to search.
  /// \return The list of BodyNodes of given name.
  virtual std::vector<BodyNode*> getBodyNodes(const std::string& name) = 0;

  /// Returns all the BodyNodes of given name.
  /// \param[in] name The BodyNode name that want to search.
  /// \return The list of BodyNodes of given name.
  virtual std::vector<const BodyNode*> getBodyNodes(
      const std::string& name) const = 0;

  /// Returns whether this Skeleton contains \c bodyNode.
  virtual bool hasBodyNode(const BodyNode* bodyNode) const = 0;

  /// Get the index of a specific BodyNode within this ReferentialSkeleton.
  /// Returns INVALID_INDEX if it is not held in this ReferentialSkeleton.
  /// When _warning is true, a warning message will be printed if the BodyNode
  /// is not in the MetaSkeleton.
  virtual std::size_t getIndexOf(const BodyNode* _bn, bool _warning=true) const = 0;

  /// Get number of Joints
  virtual std::size_t getNumJoints() const = 0;

  /// Get Joint whose index is _idx
  virtual Joint* getJoint(std::size_t _idx) = 0;

  /// Get const Joint whose index is _idx
  virtual const Joint* getJoint(std::size_t _idx) const = 0;

  /// Returns the Joint of given name.
  /// \param[in] name The joint name that want to search.
  /// \return The joint of given name.
  virtual Joint* getJoint(const std::string& name) = 0;

  /// Returns the joint of given name.
  /// \param[in] name The joint name that want to search.
  /// \return The joint of given name.
  virtual const Joint* getJoint(const std::string& name) const = 0;

  /// Returns all the joints that are held by this MetaSkeleton.
  virtual std::vector<Joint*> getJoints() = 0;

  /// Returns all the joints that are held by this MetaSkeleton.
  virtual std::vector<const Joint*> getJoints() const = 0;

  /// Returns all the Joint of given name.
  ///
  /// This MetaSkeleton can contain multiple Joints with the same name when
  /// this MetaSkeleton contains Joints from multiple Skeletons.
  ///
  /// \param[in] name The joint name that want to search.
  /// \return The list of joints of given name.
  virtual std::vector<Joint*> getJoints(const std::string& name) = 0;

  /// Returns all the Joint of given name.
  ///
  /// This MetaSkeleton can contain multiple Joints with the same name when
  /// this MetaSkeleton contains Joints from multiple Skeletons.
  ///
  /// \param[in] name The joint name that want to search.
  /// \return The list of joints of given name.
  virtual std::vector<const Joint*> getJoints(
      const std::string& name) const = 0;

  /// Returns whether this Skeleton contains \c join.
  virtual bool hasJoint(const Joint* joint) const = 0;

  /// Get the index of a specific Joint within this ReferentialSkeleton. Returns
  /// INVALID_INDEX if it is not held in this ReferentialSkeleton.
  /// When _warning is true, a warning message will be printed if the Joint is
  /// not in the MetaSkeleton.
  virtual std::size_t getIndexOf(const Joint* _joint, bool _warning=true) const = 0;

  /// Return the number of degrees of freedom in this skeleton
  virtual std::size_t getNumDofs() const = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual DegreeOfFreedom* getDof(std::size_t _idx) = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual const DegreeOfFreedom* getDof(std::size_t _idx) const = 0;

  /// Get the vector of DegreesOfFreedom for this MetaSkeleton
  virtual const std::vector<DegreeOfFreedom*>& getDofs() = 0;

  /// Get a vector of const DegreesOfFreedom for this MetaSkeleton
  virtual std::vector<const DegreeOfFreedom*> getDofs() const = 0;

  /// Get the index of a specific DegreeOfFreedom within this
  /// ReferentialSkeleton. Returns INVALID_INDEX if it is not held in this
  /// ReferentialSkeleton. When _warning is true, a warning message will be
  /// printed if the DegreeOfFreedom is not in the MetaSkeleton.
  virtual std::size_t getIndexOf(const DegreeOfFreedom* _dof,
                            bool _warning=true) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  void setCommand(std::size_t _index, double _command);

  /// Get a single command
  double getCommand(std::size_t _index) const;

  /// Set commands for all generalized coordinates
  void setCommands(const Eigen::VectorXd& _commands);

  /// Set commands for a subset of the generalized coordinates
  void setCommands(const std::vector<std::size_t>& _indices,
                           const Eigen::VectorXd& _commands);

  /// Get commands for all generalized coordinates
  Eigen::VectorXd getCommands() const;

  /// Get commands for a subset of the generalized coordinates
  Eigen::VectorXd getCommands(const std::vector<std::size_t>& _indices) const;

  /// Set all commands to zero
  void resetCommands();

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  /// Set the position of a single generalized coordinate
  void setPosition(std::size_t index, double _position);

  /// Get the position of a single generalized coordinate
  double getPosition(std::size_t _index) const;

  /// Set the positions for all generalized coordinates
  void setPositions(const Eigen::VectorXd& _positions);

  /// Set the positions for a subset of the generalized coordinates
  void setPositions(const std::vector<std::size_t>& _indices,
                    const Eigen::VectorXd& _positions);

  /// Get the positions for all generalized coordinates
  Eigen::VectorXd getPositions() const;

  /// Get the positions for a subset of the generalized coordinates
  Eigen::VectorXd getPositions(const std::vector<std::size_t>& _indices) const;

  /// Set all positions to zero
  void resetPositions();

  /// Set the lower limit of a generalized coordinate's position
  void setPositionLowerLimit(std::size_t _index, double _position);

  /// Set the lower limits for all generalized coordinates
  void setPositionLowerLimits(const Eigen::VectorXd& positions);

  /// Set the lower limits for a subset of the generalized coordinates
  void setPositionLowerLimits(const std::vector<std::size_t>& indices,
                              const Eigen::VectorXd& positions);

  /// Get the lower limit of a generalized coordinate's position
  double getPositionLowerLimit(std::size_t _index) const;

  /// Get the lower limits for all generalized coordinates
  Eigen::VectorXd getPositionLowerLimits() const;

  /// Get the lower limits for a subset of the generalized coordinates
  Eigen::VectorXd getPositionLowerLimits(
      const std::vector<std::size_t>& indices) const;

  /// Set the upper limit of a generalized coordainte's position
  void setPositionUpperLimit(std::size_t _index, double _position);

  /// Set the upper limits for all generalized coordinates
  void setPositionUpperLimits(const Eigen::VectorXd& positions);

  /// Set the upper limits for a subset of the generalized coordinates
  void setPositionUpperLimits(const std::vector<std::size_t>& indices,
                              const Eigen::VectorXd& positions);

  /// Get the upper limit of a generalized coordinate's position
  double getPositionUpperLimit(std::size_t _index) const;

  /// Get the upper limits for all generalized coordinates
  Eigen::VectorXd getPositionUpperLimits() const;

  /// Get the upper limits for a subset of the generalized coordinates
  Eigen::VectorXd getPositionUpperLimits(
      const std::vector<std::size_t>& indices) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  /// Set the velocity of a single generalized coordinate
  void setVelocity(std::size_t _index, double _velocity);

  /// Get the velocity of a single generalized coordinate
  double getVelocity(std::size_t _index) const;

  /// Set the velocities of all generalized coordinates
  void setVelocities(const Eigen::VectorXd& _velocities);

  /// Set the velocities of a subset of the generalized coordinates
  void setVelocities(const std::vector<std::size_t>& _indices,
                     const Eigen::VectorXd& _velocities);

  /// Get the velocities for all generalized coordinates
  Eigen::VectorXd getVelocities() const;

  /// Get the velocities for a subset of the generalized coordinates
  Eigen::VectorXd getVelocities(const std::vector<std::size_t>& _indices) const;

  /// Set all velocities to zero
  void resetVelocities();

  /// Set the lower limit of a generalized coordinate's velocity
  void setVelocityLowerLimit(std::size_t _index, double _velocity);

  /// Set the lower limits for all generalized coordinates's velocity
  void setVelocityLowerLimits(const Eigen::VectorXd& velocities);

  /// Set the lower limits for a subset of the generalized coordinates's
  /// velocity
  void setVelocityLowerLimits(const std::vector<std::size_t>& indices,
                              const Eigen::VectorXd& velocities);

  /// Get the lower limit of a generalized coordinate's velocity
  double getVelocityLowerLimit(std::size_t _index);

  /// Get the lower limits for all generalized coordinates's velocity
  Eigen::VectorXd getVelocityLowerLimits() const;

  /// Get the lower limits for a subset of the generalized coordinates's
  /// velocity
  Eigen::VectorXd getVelocityLowerLimits(
      const std::vector<std::size_t>& indices) const;

  /// Set the upper limit of a generalized coordinate's velocity
  void setVelocityUpperLimit(std::size_t _index, double _velocity);

  /// Set the upper limits for all generalized coordinates's velocity
  void setVelocityUpperLimits(const Eigen::VectorXd& velocities);

  /// Set the upper limits for a subset of the generalized coordinates's
  /// velocity
  void setVelocityUpperLimits(const std::vector<std::size_t>& indices,
                              const Eigen::VectorXd& velocities);

  /// Get the upper limit of a generalized coordinate's velocity
  double getVelocityUpperLimit(std::size_t _index);

  /// Get the upper limits for all generalized coordinates's velocity
  Eigen::VectorXd getVelocityUpperLimits() const;

  /// Get the upper limits for a subset of the generalized coordinates's
  /// velocity
  Eigen::VectorXd getVelocityUpperLimits(
      const std::vector<std::size_t>& indices) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  /// Set the acceleration of a single generalized coordinate
  void setAcceleration(std::size_t _index, double _acceleration);

  /// Get the acceleration of a single generalized coordinate
  double getAcceleration(std::size_t _index) const;

  /// Set the accelerations of all generalized coordinates
  void setAccelerations(const Eigen::VectorXd& _accelerations);

  /// Set the accelerations of a subset of the generalized coordinates
  void setAccelerations(const std::vector<std::size_t>& _indices,
                        const Eigen::VectorXd& _accelerations);

  /// Get the accelerations for all generalized coordinates
  Eigen::VectorXd getAccelerations() const;

  /// Get the accelerations for a subset of the generalized coordinates
  Eigen::VectorXd getAccelerations(const std::vector<std::size_t>& _indices) const;

  /// Set all accelerations to zero
  void resetAccelerations();

  /// Set the lower limit of a generalized coordinate's acceleration
  void setAccelerationLowerLimit(std::size_t _index, double _acceleration);

  /// Set the lower limits for all generalized coordinates's acceleration
  void setAccelerationLowerLimits(const Eigen::VectorXd& accelerations);

  /// Set the lower limits for a subset of the generalized coordinates's
  /// acceleration
  void setAccelerationLowerLimits(const std::vector<std::size_t>& indices,
                              const Eigen::VectorXd& accelerations);

  /// Get the lower limit of a generalized coordinate's acceleration
  double getAccelerationLowerLimit(std::size_t _index) const;

  /// Get the lower limits for all generalized coordinates's acceleration
  Eigen::VectorXd getAccelerationLowerLimits() const;

  /// Get the lower limits for a subset of the generalized coordinates's
  /// acceleration
  Eigen::VectorXd getAccelerationLowerLimits(
      const std::vector<std::size_t>& indices) const;

  /// Set the upper limit of a generalized coordinate's acceleration
  void setAccelerationUpperLimit(std::size_t _index, double _acceleration);

  /// Set the upper limits for all generalized coordinates's acceleration
  void setAccelerationUpperLimits(const Eigen::VectorXd& accelerations);

  /// Set the upper limits for a subset of the generalized coordinates's
  /// acceleration
  void setAccelerationUpperLimits(const std::vector<std::size_t>& indices,
                              const Eigen::VectorXd& accelerations);

  /// Get the upper limit of a generalized coordinate's acceleration
  double getAccelerationUpperLimit(std::size_t _index) const;

  /// Get the upper limits for all generalized coordinates's acceleration
  Eigen::VectorXd getAccelerationUpperLimits() const;

  /// Get the upper limits for a subset of the generalized coordinates's
  /// acceleration
  Eigen::VectorXd getAccelerationUpperLimits(
      const std::vector<std::size_t>& indices) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  /// Set the force of a single generalized coordinate
  void setForce(std::size_t _index, double _force);

  /// Get the force of a single generalized coordinate
  double getForce(std::size_t _index) const;

  /// Set the forces of all generalized coordinates
  void setForces(const Eigen::VectorXd& _forces);

  /// Set the forces of a subset of the generalized coordinates
  void setForces(const std::vector<std::size_t>& _index,
                 const Eigen::VectorXd& _forces);

  /// Get the forces for all generalized coordinates
  Eigen::VectorXd getForces() const;

  /// Get the forces for a subset of the generalized coordinates
  Eigen::VectorXd getForces(const std::vector<std::size_t>& _indices) const;

  /// Set all forces of the generalized coordinates to zero
  void resetGeneralizedForces();

  /// Set the lower limit of a generalized coordinate's force
  void setForceLowerLimit(std::size_t _index, double _force);

  /// Set the lower limits for all generalized coordinates's force
  void setForceLowerLimits(const Eigen::VectorXd& forces);

  /// Set the lower limits for a subset of the generalized coordinates's force
  void setForceLowerLimits(const std::vector<std::size_t>& indices,
                           const Eigen::VectorXd& forces);

  /// Get the lower limit of a generalized coordinate's force
  double getForceLowerLimit(std::size_t _index) const;

  /// Get the lower limits for all generalized coordinates's force
  Eigen::VectorXd getForceLowerLimits() const;

  /// Get the lower limits for a subset of the generalized coordinates's force
  Eigen::VectorXd getForceLowerLimits(
      const std::vector<std::size_t>& indices) const;

  /// Set the upper limit of a generalized coordinate's force
  void setForceUpperLimit(std::size_t _index, double _force);

  /// Set the upperlimits for all generalized coordinates's force
  void setForceUpperLimits(const Eigen::VectorXd& forces);

  /// Set the upper limits for a subset of the generalized coordinates's force
  void setForceUpperLimits(const std::vector<std::size_t>& indices,
                           const Eigen::VectorXd& forces);

  /// Get the upper limit of a generalized coordinate's force
  double getForceUpperLimit(std::size_t _index) const;

  /// Get the upper limits for all generalized coordinates's force
  Eigen::VectorXd getForceUpperLimits() const;

  /// Get the upper limits for a subset of the generalized coordinates's force
  Eigen::VectorXd getForceUpperLimits(
      const std::vector<std::size_t>& indices) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity Change
  //----------------------------------------------------------------------------

  /// Get the velocity changes for all the generalized coordinates
  Eigen::VectorXd getVelocityChanges() const;

  //----------------------------------------------------------------------------
  /// \{ \name Constraint Impulse
  //----------------------------------------------------------------------------

  /// Set the constraint impulses for the generalized coordinates
  void setJointConstraintImpulses(const Eigen::VectorXd& _impulses);

  /// Get the constraint impulses for the generalized coordinates
  Eigen::VectorXd getJointConstraintImpulses() const;


  //----------------------------------------------------------------------------
  /// \{ \name Jacobians
  //----------------------------------------------------------------------------

  /// Get the spatial Jacobian targeting the origin of a BodyNode. The Jacobian
  /// is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobian(const JacobianNode* _node) const = 0;

  /// Get the spatial Jacobian targeting the origin of a BodyNode. You can
  /// specify a coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian targeting the origin of a BodyNode relative to
  /// another BodyNode in the same Skeleton. You can specify a coordinate Frame
  /// to express the Jacobian in.
  math::Jacobian getJacobian(
      const JacobianNode* _node,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the Frame of the BodyNode.
  virtual math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode relative to
  /// another BodyNode in the same Skeleton. The _offset is expected in
  /// coordinates of the BodyNode Frame. You can specify a coordinate Frame to
  /// express the Jacobian in.
  math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian targeting the origin of a BodyNode. The Jacobian
  /// is expressed in the World Frame.
  virtual math::Jacobian getWorldJacobian(const JacobianNode* _node) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the World Frame.
  virtual math::Jacobian getWorldJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the linear Jacobian targeting the origin of a BodyNode. You can
  /// specify a coordinate Frame to express the Jacobian in.
  virtual math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jacobian in.
  virtual math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian targeting the origin of a BodyNode relative to
  /// another BodyNode in the same Skeleton. You can specify a coordinate Frame
  /// to express the Jacobian in.
  math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear Jacobian targeting an offset in a BodyNode relative to
  /// another BodyNode in the same Skeleton. The _offset is expected in
  /// coordinates of the BodyNode Frame. You can specify a coordinate Frame to
  /// express the Jacobian in.
  math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian derivatives
  //----------------------------------------------------------------------------

  /// Get the angular Jacobian of a BodyNode. You can specify a coordinate Frame
  /// to express the Jacobian in.
  virtual math::AngularJacobian getAngularJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the angular Jacobian of a BodyNode relative to another BodyNode in the
  /// same Skeleton. You can specify a coordinate Frame to express the Jacobian
  /// in.
  math::AngularJacobian getAngularJacobian(
      const JacobianNode* _node,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. You can specify a coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// The Jacobian is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode relative to another BodyNode in the same Skeleton. You can
  /// specify a coordinate Frame to express the Jacobian in.
  math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode relative to another Bodynode in the same Skeleton. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jacobian in.
  math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const JacobianNode* _relativeTo,
      const Frame* _inCoordinatesOf) const;

  /// Get the spatial Jacobian (classical) time derivative targeting the origin
  /// of a BodyNode. The Jacobian is expressed in the World Frame.
  virtual math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node) const = 0;

  /// Get the spatial Jacobian (classical) time derivative targeting the origin
  /// a BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian (classical) time derivative targeting an offset
  /// in a BodyNode. The _offset is expected in coordinates of the BodyNode
  /// Frame. You can specify a coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// of a BodyNode. The _offset is expected in coordinates of the BodyNode
  /// Frame. You can specify a coordinate Frame to express the Jacobian in.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian (classical) time derivative targeting an offset in
  /// a BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jacobian in.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the angular Jacobian time derivative of a BodyNode. You can specify a
  /// coordinate Frame to express the Jacobian in.
  virtual math::AngularJacobian getAngularJacobianDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of Motion
  //----------------------------------------------------------------------------

  /// Get the total mass of all BodyNodes in this MetaSkeleton. Note that
  /// for the ReferentialSkeleton extension of MetaSkeleton, this will be an
  /// O(n) operation, while the Skeleton extension will be O(1).
  virtual double getMass() const = 0;

  /// Get the Mass Matrix of the MetaSkeleton
  virtual const Eigen::MatrixXd& getMassMatrix() const = 0;

  /// Get augmented mass matrix of the skeleton. This matrix is used in
  /// ConstraintDynamics to compute constraint forces. The matrix is
  /// M + h*D + h*h*K where D is diagonal joint damping coefficient matrix, K is
  /// diagonal joint stiffness matrix, and h is simulation time step.
  virtual const Eigen::MatrixXd& getAugMassMatrix() const = 0;

  /// Get inverse of Mass Matrix of the MetaSkeleton.
  virtual const Eigen::MatrixXd& getInvMassMatrix() const = 0;

  /// Get inverse of augmented Mass Matrix of the MetaSkeleton.
  virtual const Eigen::MatrixXd& getInvAugMassMatrix() const = 0;

  /// Get Coriolis force vector of the MetaSkeleton's BodyNodes.
  virtual const Eigen::VectorXd& getCoriolisForces() const = 0;

  /// Get gravity force vector of the MetaSkeleton.
  virtual const Eigen::VectorXd& getGravityForces() const = 0;

  /// Get combined vector of Coriolis force and gravity force of the MetaSkeleton.
  virtual const Eigen::VectorXd& getCoriolisAndGravityForces() const = 0;

  /// Get external force vector of the MetaSkeleton.
  virtual const Eigen::VectorXd& getExternalForces() const = 0;

  /// Get constraint force vector.
  virtual const Eigen::VectorXd& getConstraintForces() const = 0;

  /// Clear the external forces of the BodyNodes in this MetaSkeleton
  virtual void clearExternalForces() = 0;

  /// Clear the internal forces of the BodyNodes in this MetaSkeleton
  virtual void clearInternalForces() = 0;

  /// Compute and return Lagrangian of this MetaSkeleton
  double computeLagrangian() const;

  /// Get the kinetic energy of this MetaSkeleton
  DART_DEPRECATED(6.1)
  double getKineticEnergy() const;

  /// Get the kinetic energy of this MetaSkeleton
  virtual double computeKineticEnergy() const = 0;

  /// Get the potential energy of this MetaSkeleton
  DART_DEPRECATED(6.1)
  double getPotentialEnergy() const;

  /// Get the potential energy of this MetaSkeleton
  virtual double computePotentialEnergy() const = 0;

  /// Clear collision flags of the BodyNodes in this MetaSkeleton
  DART_DEPRECATED(6.0)
  virtual void clearCollidingBodies() = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Center of Mass Jacobian
  //----------------------------------------------------------------------------

  /// Get the MetaSkeleton's COM with respect to any Frame (default is World
  /// Frame)
  virtual Eigen::Vector3d getCOM(
      const Frame* _withRespectTo = Frame::World()) const = 0;

  /// Get the Skeleton's COM spatial velocity in terms of any Frame (default is
  /// World Frame)
  virtual Eigen::Vector6d getCOMSpatialVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the Skeleton's COM linear velocity in terms of any Frame (default is
  /// World Frame)
  virtual Eigen::Vector3d getCOMLinearVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the Skeleton's COM spatial acceleration in terms of any Frame (default
  /// is World Frame)
  virtual Eigen::Vector6d getCOMSpatialAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the MetaSkeleton's COM linear acceleration in terms of any Frame
  /// (default is World Frame)
  virtual Eigen::Vector3d getCOMLinearAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the MetaSkeleton's COM Jacobian in terms of any Frame (default is
  /// World Frame)
  virtual math::Jacobian getCOMJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the MetaSkeleton's COM Linear Jacobian in terms of any Frame (default is
  /// World Frame)
  virtual math::LinearJacobian getCOMLinearJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the Skeleton's COM Jacobian spatial time derivative in terms of any
  /// Frame (default is World Frame).
  ///
  /// NOTE: Since this is a spatial time derivative, it is only meant to be used
  /// with spatial acceleration vectors. If you are using classical linear
  /// vectors, then use getCOMLinearJacobianDeriv() instead.
  virtual math::Jacobian getCOMJacobianSpatialDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the Skeleton's COM Linear Jacobian time derivative in terms of any
  /// Frame (default is World Frame).
  ///
  /// NOTE: Since this is a classical time derivative, it is only meant to be
  /// used with classical acceleration vectors. If you are using spatial
  /// vectors, then use getCOMJacobianSpatialDeriv() instead.
  virtual math::LinearJacobian getCOMLinearJacobianDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// \}

protected:

  /// Default constructor
  MetaSkeleton();

  //--------------------------------------------------------------------------
  // Signals
  //--------------------------------------------------------------------------
  NameChangedSignal mNameChangedSignal;

public:

  //--------------------------------------------------------------------------
  // Slot registers
  //--------------------------------------------------------------------------
  common::SlotRegister<NameChangedSignal> onNameChanged;
};

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_METASKELETON_HPP_
