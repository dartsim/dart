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

#ifndef DART_DYNAMICS_METASKELETON_H_
#define DART_DYNAMICS_METASKELETON_H_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/common/Signal.h"
#include "dart/common/Subject.h"
#include "dart/math/Geometry.h"
#include "dart/dynamics/Frame.h"

namespace dart {
namespace dynamics {

class BodyNode;
class SoftBodyNode;
class PointMass;
class Joint;
class Marker;
class DegreeOfFreedom;

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

  virtual ~MetaSkeleton() = default;

  //----------------------------------------------------------------------------
  /// \{ \name Properties
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
  virtual size_t getNumBodyNodes() const = 0;

  /// Get BodyNode whose index is _idx
  virtual BodyNode* getBodyNode(size_t _idx) = 0;

  /// Get const BodyNode whose index is _idx
  virtual const BodyNode* getBodyNode(size_t _idx) const = 0;

  /// Get number of Joints
  virtual size_t getNumJoints() const = 0;

  /// Get Joint whose index is _idx
  virtual Joint* getJoint(size_t _idx) = 0;

  /// Get const Joint whose index is _idx
  virtual const Joint* getJoint(size_t _idx) const = 0;

  /// Return the number of degrees of freedom in this skeleton
  virtual size_t getNumDofs() const = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual DegreeOfFreedom* getDof(size_t _idx) = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual const DegreeOfFreedom* getDof(size_t _idx) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  virtual void setCommand(size_t _index, double _command) = 0;

  /// Get a single command
  virtual double getCommand(size_t _index) const = 0;

  /// Set commands for all generalized coordinates
  virtual void setCommands(const Eigen::VectorXd& _commands) = 0;

  /// Set commands for a subset of the generalized coordinates
  virtual void setCommands(const std::vector<size_t>& _indices,
                           const Eigen::VectorXd& _commands) = 0;

  /// Get commands for all generalized coordinates
  virtual Eigen::VectorXd getCommands() const = 0;

  /// Get commands for a subset of the generalized coordinates
  virtual Eigen::VectorXd getCommands(const std::vector<size_t>& _indices) const = 0;

  /// Set all commands to zero
  virtual void resetCommands() = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  /// Set the position of a single generalized coordinate
  virtual void setPosition(size_t index, double _position) = 0;

  /// Get the position of a single generalized coordinate
  virtual double getPosition(size_t _index) const = 0;

  /// Set the positions for all generalized coordinates
  virtual void setPositions(const Eigen::VectorXd& _positions) = 0;

  /// Set the positions for a subset of the generalized coordinates
  virtual void setPositions(const std::vector<size_t>& _indices,
                            const Eigen::VectorXd& _positions) = 0;

  /// Get the positions for all generalized coordinates
  virtual Eigen::VectorXd getPositions() const = 0;

  /// Get the positions for a subset of the generalized coordinates
  virtual Eigen::VectorXd getPositions(const std::vector<size_t>& _indices) const = 0;

  /// Set all positions to zero
  virtual void resetPositions() = 0;

  /// Set the lower limit of a generalized coordinate's position
  virtual void setPositionLowerLimit(size_t _index, double _position) = 0;

  /// Get the lower limit of a generalized coordinate's position
  virtual double getPositionLowerLimit(size_t _index) const = 0;

  /// Set the upper limit of a generalized coordainte's position
  virtual void setPositionUpperLimit(size_t _index, double _position) = 0;

  /// Get the upper limit of a generalized coordinate's position
  virtual double getPositionUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  /// Set the velocity of a single generalized coordinate
  virtual void setVelocity(size_t _index, double _velocity) = 0;

  /// Get the velocity of a single generalized coordinate
  virtual double getVelocity(size_t _index) const = 0;

  /// Set the velocities of all generalized coordinates
  virtual void setVelocities(const Eigen::VectorXd& _velocities) = 0;

  /// Set the velocities of a subset of the generalized coordinates
  virtual void setVelocities(const std::vector<size_t>& _indices,
                             const Eigen::VectorXd& _velocities) = 0;

  /// Get the velocities for all generalized coordinates
  virtual Eigen::VectorXd getVelocities() const = 0;

  /// Get the velocities for a subset of the generalized coordinates
  virtual Eigen::VectorXd getVelocities(const std::vector<size_t>& _indices) const = 0;

  /// Set all velocities to zero
  virtual void resetVelocities() = 0;

  /// Set the lower limit of a generalized coordinate's velocity
  virtual void setVelocityLowerLimit(size_t _index, double _velocity) = 0;

  /// Get the lower limit of a generalized coordinate's velocity
  virtual double getVelocityLowerLimit(size_t _index) = 0;

  /// Set the upper limit of a generalized coordinate's velocity
  virtual void setVelocityUpperLimit(size_t _index, double _velocity) = 0;

  /// Get the upper limit of a generalized coordinate's velocity
  virtual double getVelocityUpperLimit(size_t _index) = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  /// Set the acceleration of a single generalized coordinate
  virtual void setAcceleration(size_t _index, double _acceleration) = 0;

  /// Get the acceleration of a single generalized coordinate
  virtual double getAcceleration(size_t _index) const = 0;

  /// Set the accelerations of all generalized coordinates
  virtual void setAccelerations(const Eigen::VectorXd& _accelerations) = 0;

  /// Set the accelerations of a subset of the generalized coordinates
  virtual void setAccelerations(const std::vector<size_t>& _indices,
                                const Eigen::VectorXd& _accelerations) = 0;

  /// Get the accelerations for all generalized coordinates
  virtual Eigen::VectorXd getAccelerations() const = 0;

  /// Get the accelerations for a subset of the generalized coordinates
  virtual Eigen::VectorXd getAccelerations(const std::vector<size_t>& _indices) const = 0;

  /// Set all accelerations to zero
  virtual void resetAccelerations() = 0;

  /// Set the lower limit of a generalized coordinate's acceleration
  virtual void setAccelerationLowerLimit(size_t _index, double _acceleration) = 0;

  /// Get the lower limit of a generalized coordinate's acceleration
  virtual double getAccelerationLowerLimit(size_t _index) const = 0;

  /// Set the upper limit of a generalized coordinate's acceleration
  virtual void setAccelerationUpperLimit(size_t _index, double _acceleration) = 0;

  /// Get the upper limit of a generalized coordinate's acceleration
  virtual double getAccelerationUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  /// Set the force of a single generalized coordinate
  virtual void setForce(size_t _index, double _force) = 0;

  /// Get the force of a single generalized coordinate
  virtual double getForce(size_t _index) const = 0;

  /// Set the forces of all generalized coordinates
  virtual void setForces(const Eigen::VectorXd& _forces) = 0;

  /// Set the forces of a subset of the generalized coordinates
  virtual void setForces(const std::vector<size_t>& _index,
                         const Eigen::VectorXd& _forces) = 0;

  /// Get the forces for all generalized coordinates
  virtual Eigen::VectorXd getForces() const;

  /// Get the forces for a subset of the generalized coordinates
  virtual Eigen::VectorXd getForces(const std::vector<size_t>& _indices) const = 0;

  /// Set all forces of the generalized coordinates to zero
  virtual void resetForces();

  /// Set the lower limit of a generalized coordinate's force
  virtual void setForceLowerLimit(size_t _index, double _force) = 0;

  /// Get the lower limit of a generalized coordinate's force
  virtual double getForceLowerLimit(size_t _index) const = 0;

  /// Set the upper limit of a generalized coordinate's force
  virtual void setForceUpperLimit(size_t _index, double _force) = 0;

  /// Get the upper limit of a generalized coordinate's force
  virtual double getForceUpperLimit(size_t _index) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobians
  //----------------------------------------------------------------------------

  /// Get the spatial Jacobian targeting the origin of a BodyNode. The Jacobian
  /// is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobian(const BodyNode* _bodyNode) const = 0;

  /// Get the spatial Jacobian targeting the origin of a BodyNode. You can
  /// specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the Frame of the BodyNode.
  virtual math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian targeting the origin of a BodyNode. The Jacobian
  /// is expressed in the World Frame.
  virtual math::Jacobian getWorldJacobian(const BodyNode* _bodyNode) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the World Frame.
  virtual math::Jacobian getWorldJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the linear Jacobian targeting the origin of a BodyNode. You can
  /// specify a coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobian(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the angular Jacobian of a BodyNode. You can specify a coordinate Frame
  /// to express the Jabocian in.
  virtual math::AngularJacobian getAngularJacobian(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobianSpatialDeriv(const BodyNode* _bodyNode) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// The Jacobian is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the World Frame.
  virtual math::Jacobian getJacobianClassicDeriv(const BodyNode* _bodyNode) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianClassicDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianClassicDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian (classical) time derivative targeting the origin
  /// of a BodyNode. The _offset is expected in coordinates of the BodyNode
  /// Frame. You can specify a coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const BodyNode* _bodyNode,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian (classical) time derivative targeting an offset in
  /// a BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const BodyNode* _bodyNode,
      const Eigen::Vector3d& _localOffset = Eigen::Vector3d::Zero(),
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the angular Jacobian time derivative of a BodyNode. You can specify a
  /// coordinate Frame to express the Jabocian in.
  virtual math::AngularJacobian getAngularJacobianDeriv(
      const BodyNode* _bodyNode,
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

  /// Get the kinetic energy of this MetaSkeleton
  virtual double getKineticEnergy() const = 0;

  /// Get the potential energy of this MetaSkeleton
  virtual double getPotentialEnergy() const = 0;

  // -- Center of Mass Jacobian ---

  /// Get the MetaSkeleton's COM with respect to any Frame (default is World
  /// Frame)
  virtual Eigen::Vector3d getCOM(const Frame* _withRespectTo = Frame::World()) const = 0;

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

  inline MetaSkeleton() : onNameChanged(mNameChangedSignal) { }

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

typedef std::shared_ptr<MetaSkeleton> MetaSkeletonPtr;
typedef std::shared_ptr<const MetaSkeleton> ConstMetaSkeletonPtr;

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_METASKELETON_H_
