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

#ifndef DART_DYNAMICS_METASKELETON_HPP_
#define DART_DYNAMICS_METASKELETON_HPP_

#include <vector>
#include <string>

#include <Eigen/Dense>

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

  MetaSkeleton(const MetaSkeleton&) = delete;

  /// Default destructor
  virtual ~MetaSkeleton() = default;

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
  virtual size_t getNumBodyNodes() const = 0;

  /// Get BodyNode whose index is _idx
  virtual BodyNode* getBodyNode(size_t _idx) = 0;

  /// Get const BodyNode whose index is _idx
  virtual const BodyNode* getBodyNode(size_t _idx) const = 0;

  /// Get all the BodyNodes that are held by this MetaSkeleton
  virtual const std::vector<BodyNode*>& getBodyNodes() = 0;

  /// Get all the BodyNodes that are held by this MetaSkeleton
  virtual const std::vector<const BodyNode*>& getBodyNodes() const = 0;

  /// Get the index of a specific BodyNode within this ReferentialSkeleton.
  /// Returns INVALID_INDEX if it is not held in this ReferentialSkeleton.
  /// When _warning is true, a warning message will be printed if the BodyNode
  /// is not in the MetaSkeleton.
  virtual size_t getIndexOf(const BodyNode* _bn, bool _warning=true) const = 0;

  /// Get number of Joints
  virtual size_t getNumJoints() const = 0;

  /// Get Joint whose index is _idx
  virtual Joint* getJoint(size_t _idx) = 0;

  /// Get const Joint whose index is _idx
  virtual const Joint* getJoint(size_t _idx) const = 0;

  /// Get the index of a specific Joint within this ReferentialSkeleton. Returns
  /// INVALID_INDEX if it is not held in this ReferentialSkeleton.
  /// When _warning is true, a warning message will be printed if the Joint is
  /// not in the MetaSkeleton.
  virtual size_t getIndexOf(const Joint* _joint, bool _warning=true) const = 0;

  /// Return the number of degrees of freedom in this skeleton
  virtual size_t getNumDofs() const = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual DegreeOfFreedom* getDof(size_t _idx) = 0;

  /// Get degree of freedom (aka generalized coordinate) whose index is _idx
  virtual const DegreeOfFreedom* getDof(size_t _idx) const = 0;

  /// Get the vector of DegreesOfFreedom for this MetaSkeleton
  virtual const std::vector<DegreeOfFreedom*>& getDofs() = 0;

  /// Get a vector of const DegreesOfFreedom for this MetaSkeleton
  virtual std::vector<const DegreeOfFreedom*> getDofs() const = 0;

  /// Get the index of a specific DegreeOfFreedom within this
  /// ReferentialSkeleton. Returns INVALID_INDEX if it is not held in this
  /// ReferentialSkeleton. When _warning is true, a warning message will be
  /// printed if the DegreeOfFreedom is not in the MetaSkeleton.
  virtual size_t getIndexOf(const DegreeOfFreedom* _dof,
                            bool _warning=true) const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Command
  //----------------------------------------------------------------------------

  /// Set a single command
  void setCommand(size_t _index, double _command);

  /// Get a single command
  double getCommand(size_t _index) const;

  /// Set commands for all generalized coordinates
  void setCommands(const Eigen::VectorXd& _commands);

  /// Set commands for a subset of the generalized coordinates
  void setCommands(const std::vector<size_t>& _indices,
                           const Eigen::VectorXd& _commands);

  /// Get commands for all generalized coordinates
  Eigen::VectorXd getCommands() const;

  /// Get commands for a subset of the generalized coordinates
  Eigen::VectorXd getCommands(const std::vector<size_t>& _indices) const;

  /// Set all commands to zero
  void resetCommands();

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Position
  //----------------------------------------------------------------------------

  /// Set the position of a single generalized coordinate
  void setPosition(size_t index, double _position);

  /// Get the position of a single generalized coordinate
  double getPosition(size_t _index) const;

  /// Set the positions for all generalized coordinates
  void setPositions(const Eigen::VectorXd& _positions);

  /// Set the positions for a subset of the generalized coordinates
  void setPositions(const std::vector<size_t>& _indices,
                    const Eigen::VectorXd& _positions);

  /// Get the positions for all generalized coordinates
  Eigen::VectorXd getPositions() const;

  /// Get the positions for a subset of the generalized coordinates
  Eigen::VectorXd getPositions(const std::vector<size_t>& _indices) const;

  /// Set all positions to zero
  void resetPositions();

  /// Set the lower limit of a generalized coordinate's position
  void setPositionLowerLimit(size_t _index, double _position);

  /// Get the lower limit of a generalized coordinate's position
  double getPositionLowerLimit(size_t _index) const;

  /// Set the upper limit of a generalized coordainte's position
  void setPositionUpperLimit(size_t _index, double _position);

  /// Get the upper limit of a generalized coordinate's position
  double getPositionUpperLimit(size_t _index) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Velocity
  //----------------------------------------------------------------------------

  /// Set the velocity of a single generalized coordinate
  void setVelocity(size_t _index, double _velocity);

  /// Get the velocity of a single generalized coordinate
  double getVelocity(size_t _index) const;

  /// Set the velocities of all generalized coordinates
  void setVelocities(const Eigen::VectorXd& _velocities);

  /// Set the velocities of a subset of the generalized coordinates
  void setVelocities(const std::vector<size_t>& _indices,
                     const Eigen::VectorXd& _velocities);

  /// Get the velocities for all generalized coordinates
  Eigen::VectorXd getVelocities() const;

  /// Get the velocities for a subset of the generalized coordinates
  Eigen::VectorXd getVelocities(const std::vector<size_t>& _indices) const;

  /// Set all velocities to zero
  void resetVelocities();

  /// Set the lower limit of a generalized coordinate's velocity
  void setVelocityLowerLimit(size_t _index, double _velocity);

  /// Get the lower limit of a generalized coordinate's velocity
  double getVelocityLowerLimit(size_t _index);

  /// Set the upper limit of a generalized coordinate's velocity
  void setVelocityUpperLimit(size_t _index, double _velocity);

  /// Get the upper limit of a generalized coordinate's velocity
  double getVelocityUpperLimit(size_t _index);

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Acceleration
  //----------------------------------------------------------------------------

  /// Set the acceleration of a single generalized coordinate
  void setAcceleration(size_t _index, double _acceleration);

  /// Get the acceleration of a single generalized coordinate
  double getAcceleration(size_t _index) const;

  /// Set the accelerations of all generalized coordinates
  void setAccelerations(const Eigen::VectorXd& _accelerations);

  /// Set the accelerations of a subset of the generalized coordinates
  void setAccelerations(const std::vector<size_t>& _indices,
                        const Eigen::VectorXd& _accelerations);

  /// Get the accelerations for all generalized coordinates
  Eigen::VectorXd getAccelerations() const;

  /// Get the accelerations for a subset of the generalized coordinates
  Eigen::VectorXd getAccelerations(const std::vector<size_t>& _indices) const;

  /// Set all accelerations to zero
  void resetAccelerations();

  /// Set the lower limit of a generalized coordinate's acceleration
  void setAccelerationLowerLimit(size_t _index, double _acceleration);

  /// Get the lower limit of a generalized coordinate's acceleration
  double getAccelerationLowerLimit(size_t _index) const;

  /// Set the upper limit of a generalized coordinate's acceleration
  void setAccelerationUpperLimit(size_t _index, double _acceleration);

  /// Get the upper limit of a generalized coordinate's acceleration
  double getAccelerationUpperLimit(size_t _index) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Force
  //----------------------------------------------------------------------------

  /// Set the force of a single generalized coordinate
  void setForce(size_t _index, double _force);

  /// Get the force of a single generalized coordinate
  double getForce(size_t _index) const;

  /// Set the forces of all generalized coordinates
  void setForces(const Eigen::VectorXd& _forces);

  /// Set the forces of a subset of the generalized coordinates
  void setForces(const std::vector<size_t>& _index,
                 const Eigen::VectorXd& _forces);

  /// Get the forces for all generalized coordinates
  Eigen::VectorXd getForces() const;

  /// Get the forces for a subset of the generalized coordinates
  Eigen::VectorXd getForces(const std::vector<size_t>& _indices) const;

  /// Set all forces of the generalized coordinates to zero
  void resetGeneralizedForces();

  /// Set the lower limit of a generalized coordinate's force
  void setForceLowerLimit(size_t _index, double _force);

  /// Get the lower limit of a generalized coordinate's force
  double getForceLowerLimit(size_t _index) const;

  /// Set the upper limit of a generalized coordinate's force
  void setForceUpperLimit(size_t _index, double _force);

  /// Get the upper limit of a generalized coordinate's force
  double getForceUpperLimit(size_t _index) const;

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
  /// specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. The Jacobian is expressed
  /// in the Frame of the BodyNode.
  virtual math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const = 0;

  /// Get the spatial Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const = 0;

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
  /// specify a coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian targeting an offset in a BodyNode. The _offset is
  /// expected in coordinates of the BodyNode Frame. You can specify a
  /// coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the angular Jacobian of a BodyNode. You can specify a coordinate Frame
  /// to express the Jabocian in.
  virtual math::AngularJacobian getAngularJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the Frame of the BodyNode.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. You can specify a coordinate Frame to express the Jabocian in.
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
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin of a
  /// BodyNode. The Jacobian is expressed in the World Frame.
  virtual math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node) const = 0;

  /// Get the spatial Jacobian time derivative targeting the origin a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const = 0;

  /// Get the spatial Jacobian time derivative targeting an offset in a
  /// BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian (classical) time derivative targeting the origin
  /// of a BodyNode. The _offset is expected in coordinates of the BodyNode
  /// Frame. You can specify a coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the linear Jacobian (classical) time derivative targeting an offset in
  /// a BodyNode. The _offset is expected in coordinates of the BodyNode Frame.
  /// You can specify a coordinate Frame to express the Jabocian in.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Get the angular Jacobian time derivative of a BodyNode. You can specify a
  /// coordinate Frame to express the Jabocian in.
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

  /// Get the kinetic energy of this MetaSkeleton
  virtual double getKineticEnergy() const = 0;

  /// Get the potential energy of this MetaSkeleton
  virtual double getPotentialEnergy() const = 0;

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
