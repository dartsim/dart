/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_BODYNODE_H_
#define DART_DYNAMICS_BODYNODE_H_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "dart/config.h"
#include "dart/common/Deprecated.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

class GenCoord;
class Skeleton;
class Joint;
class Shape;
class Marker;

/// BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
class BodyNode
{
public:
  /// Constructor
  explicit BodyNode(const std::string& _name = "BodyNode");

  /// Destructor
  virtual ~BodyNode();

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name);

  /// Return the name of the bodynode
  const std::string& getName() const;

  /// Set whether gravity affects this body
  /// \param[in] _gravityMode True to enable gravity
  void setGravityMode(bool _gravityMode);

  /// Return true if gravity mode is enabled
  bool getGravityMode() const;

  /// Return true if this body can collide with others bodies
  bool isCollidable() const;

  /// Set whether this body node will collide with others in the world
  /// \param[in] _isCollidable True to enable collisions
  void setCollidable(bool _isCollidable);

  /// Set the mass of the bodynode
  void setMass(double _mass);

  /// Return the mass of the bodynode
  double getMass() const;

  /// Set moment of inertia defined around the center of mass
  ///
  /// Principal moments of inertia (_Ixx, _Iyy, _Izz) must be positive or zero
  /// values.
  void setMomentOfInertia(
      double _Ixx, double _Iyy, double _Izz,
      double _Ixy = 0.0, double _Ixz = 0.0, double _Iyz = 0.0);

  /// Return moment of inertia defined around the center of mass
  void getMomentOfInertia(
      double& _Ixx, double& _Iyy, double& _Izz,
      double& _Ixy, double& _Ixz, double& _Iyz);

  /// Return spatial inertia
  const Eigen::Matrix6d& getSpatialInertia() const;

  /// Set center of mass expressed in body frame
  void setLocalCOM(const Eigen::Vector3d& _com);

  /// Return center of mass expressed in body frame
  const Eigen::Vector3d& getLocalCOM() const;

  /// Return center of mass expressed in world frame
  Eigen::Vector3d getWorldCOM() const;

  /// Return velocity of center of mass expressed in world frame
  Eigen::Vector3d getWorldCOMVelocity() const;

  /// Return acceleration of center of mass expressed in world frame
  Eigen::Vector3d getWorldCOMAcceleration() const;

  /// Set coefficient of friction in range of [0, ~]
  void setFrictionCoeff(double _coeff);

  /// Return frictional coefficient.
  double getFrictionCoeff() const;

  /// Set coefficient of restitution in range of [0, 1]
  void setRestitutionCoeff(double _coeff);

  /// Return coefficient of restitution
  double getRestitutionCoeff() const;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------

  /// Add a visualization shape into the bodynode
  void addVisualizationShape(Shape* _p);

  /// Return the number of visualization shapes
  size_t getNumVisualizationShapes() const;

  /// Return _index-th visualization shape
  Shape* getVisualizationShape(int _index);
  /// Return (const) _index-th visualization shape
  const Shape* getVisualizationShape(int _index) const;

  /// Add a collision shape into the bodynode
  void addCollisionShape(Shape* _p);

  /// Return the number of collision shapes
  size_t getNumCollisionShapes() const;

  /// Return _index-th collision shape
  Shape* getCollisionShape(int _index);

  /// Return (const) _index-th collision shape
  const Shape* getCollisionShape(int _index) const;

  /// Return the Skeleton this BodyNode belongs to
  Skeleton* getSkeleton();

  /// Return the (const) Skeleton this BodyNode belongs to
  const Skeleton* getSkeleton() const;

  /// Set _joint as the parent joint of the bodynode
  void setParentJoint(Joint* _joint);

  /// Return the parent Joint of this BodyNode
  Joint* getParentJoint();

  /// Return the (const) parent Joint of this BodyNode
  const Joint* getParentJoint() const;

  /// Return the parent BodyNdoe of this BodyNode
  BodyNode* getParentBodyNode();

  /// Return the (const) parent BodyNode of this BodyNode
  const BodyNode* getParentBodyNode() const;

  /// Add a child bodynode into the bodynode
  void addChildBodyNode(BodyNode* _body);

  /// Return the _index-th child BodyNode of this BodyNode
  BodyNode* getChildBodyNode(size_t _index);

  /// Return the (const) _index-th child BodyNode of this BodyNode
  const BodyNode* getChildBodyNode(size_t _index) const;

  /// Return the number of child bodynodes
  size_t getNumChildBodyNodes() const;

  /// Add a marker into the bodynode
  void addMarker(Marker* _marker);

  /// Return the number of markers of the bodynode
  size_t getNumMarkers() const;

  /// Return _index-th marker of the bodynode
  Marker* getMarker(size_t _index);

  /// Return (const) _index-th marker of the bodynode
  const Marker* getMarker(size_t _index) const;

  /// Return true if _genCoordIndex-th generalized coordinate
  bool dependsOn(int _genCoordIndex) const;

  /// The number of the generalized coordinates by which this node is affected
  size_t getNumDependentGenCoords() const;

  /// Return a generalized coordinate index from the array index
  /// (< getNumDependentDofs)
  size_t getDependentGenCoordIndex(size_t _arrayIndex) const;

  //--------------------------------------------------------------------------
  // Properties updated by dynamics (kinematics)
  //--------------------------------------------------------------------------

  /// Return the current position and orientation expressed in world frame
  const Eigen::Isometry3d& getTransform() const;

  /// Return the spatial velocity at the origin of the bodynode expressed in
  /// the body-fixed frame
  const Eigen::Vector6d& getBodyVelocity() const;

  /// Return the linear velocity of the origin of the bodynode expressed in
  /// body-fixed frame
  Eigen::Vector3d getBodyLinearVelocity() const;

  /// Return the linear velocity of a point on the bodynode expressed in
  /// body-fixed frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  Eigen::Vector3d getBodyLinearVelocity(const Eigen::Vector3d& _offset) const;

  /// Return the angular velocity expressed in body-fixed frame
  Eigen::Vector3d getBodyAngularVelocity() const;

  /// Return the linear velocity of the origin of the bodynode expressed in
  /// world frame
  Eigen::Vector3d getWorldLinearVelocity() const;

  /// Return the linear velocity of a point on the bodynode expressed in world
  /// frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  Eigen::Vector3d getWorldLinearVelocity(const Eigen::Vector3d& _offset) const;

  /// Return the angular velocity expressed in world frame
  Eigen::Vector3d getWorldAngularVelocity() const;

  /// Return generalized acceleration at the origin of this body node where the
  /// acceleration is expressed in this body node frame
  const Eigen::Vector6d& getBodyAcceleration() const;

  /// Return the linear acceleration of the origin of the bodynode expressed in
  /// body-fixed frame
  Eigen::Vector3d getBodyLinearAcceleration() const;

  /// Return the linear acceleration of a point on the bodynode expressed in
  /// body-fixed frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  Eigen::Vector3d getBodyLinearAcceleration(
      const Eigen::Vector3d& _offset) const;

  /// Return the angular acceleration expressed in body-fixed frame
  Eigen::Vector3d getBodyAngularAcceleration() const;

  /// Return the linear acceleration of the origin of the bodynode expressed in
  /// world frame
  Eigen::Vector3d getWorldLinearAcceleration() const;

  /// Return the linear acceleration of a point on the bodynode expressed in
  /// world frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  Eigen::Vector3d getWorldLinearAcceleration(
      const Eigen::Vector3d& _offset) const;

  /// Return the angular acceleration expressed in world frame
  Eigen::Vector3d getWorldAngularAcceleration() const;

  /// Return generalized Jacobian at the origin of this body node where the
  /// Jacobian is expressed in this body node frame
  const math::Jacobian& getBodyJacobian();

  /// Return the linear Jacobian of the origin of the bodynode expressed in
  /// body-fixed frame
  math::LinearJacobian getBodyLinearJacobian();

  /// Return the linear Jacobian of a point on the bodynode expressed in
  /// body-fixed frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  math::LinearJacobian getBodyLinearJacobian(const Eigen::Vector3d& _offset);

  /// Return the angular Jacobian expressed in body-fixed frame
  math::AngularJacobian getBodyAngularJacobian();

  /// Return the linear Jacobian of the origin of the bodynode expressed in
  /// world frame
  math::LinearJacobian getWorldLinearJacobian();

  /// Return the linear Jacobian of a point on the bodynode expressed in world
  /// frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  math::LinearJacobian getWorldLinearJacobian(const Eigen::Vector3d& _offset);

  /// Return the angular Jacobian expressed in world frame
  math::AngularJacobian getWorldAngularJacobian();

  /// Return time derivative of generalized Jacobian at the origin of this body
  /// node where the Jacobian is expressed in this body node frame
  const math::Jacobian& getBodyJacobianDeriv();

  /// Return the time derivative of the linear Jacobian of the origin of the
  /// bodynode expressed in body-fixed frame
  math::LinearJacobian getBodyLinearJacobianDeriv();

  /// Return the time derivative of the linear Jacobian of a point on the
  /// bodynode expressed in body-fixed frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  math::LinearJacobian getBodyLinearJacobianDeriv(
      const Eigen::Vector3d& _offset);

  /// Return the time derivative of the angular Jacobian expressed in body-fixed
  /// frame
  math::AngularJacobian getBodyAngularJacobianDeriv();

  /// Return the time derivative of the linear Jacobian of the origin of the
  /// bodynode expressed in world frame
  math::LinearJacobian getWorldLinearJacobianDeriv();

  /// Return the time derivative of the linear Jacobian of a point on the
  /// bodynode expressed in world frame
  /// \param[in] _offset Offset of the point from the origin of the bodynode
  /// frame. The offset is expressed in the body-fixed frame
  math::LinearJacobian getWorldLinearJacobianDeriv(
      const Eigen::Vector3d& _offset);

  /// Return the angular Jacobian expressed in world frame
  math::AngularJacobian getWorldAngularJacobianDeriv();

  /// Return the velocity change due to the constraint impulse
  const Eigen::Vector6d& getBodyVelocityChange() const;

  /// Set whether this body node is colliding with others. This is called by
  /// collision detector.
  /// \param[in] True if this body node is colliding.
  void setColliding(bool _isColliding);

  /// Return whether this body node is colliding with others
  /// \return True if this body node is colliding.
  bool isColliding();

  /// Add applying linear Cartesian forces to this node
  ///
  /// A force is defined by a point of application and a force vector. The
  /// last two parameters specify frames of the first two parameters.
  /// Coordinate transformations are applied when needed. The point of
  /// application and the force in local coordinates are stored in mContacts.
  /// When conversion is needed, make sure the transformations are avaialble.
  void addExtForce(const Eigen::Vector3d& _force,
                   const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
                   bool _isForceLocal = false,
                   bool _isOffsetLocal = true);

  /// Set Applying linear Cartesian forces to this node.
  void setExtForce(const Eigen::Vector3d& _force,
                   const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
                   bool _isForceLocal = false,
                   bool _isOffsetLocal = true);

  /// Add applying Cartesian torque to the node.
  ///
  /// The torque in local coordinates is accumulated in mExtTorqueBody.
  void addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal = false);

  /// Set applying Cartesian torque to the node.
  ///
  /// The torque in local coordinates is accumulated in mExtTorqueBody.
  void setExtTorque(const Eigen::Vector3d& _torque, bool _isLocal = false);

  /// Clean up structures that store external forces: mContacts, mFext,
  /// mExtForceBody and mExtTorqueBody.
  ///
  /// Called by Skeleton::clearExternalForces.
  virtual void clearExternalForces();

  ///
  const Eigen::Vector6d& getExternalForceLocal() const;

  ///
  Eigen::Vector6d getExternalForceGlobal() const;

  ///
  const Eigen::Vector6d& getBodyForce() const;

  //----------------------------------------------------------------------------
  // Constraints
  //   - Following functions are managed by constraint solver.
  //----------------------------------------------------------------------------

  /// Deprecated in 4.2. Please use isReactive().
  DEPRECATED(4.2) bool isImpulseReponsible() const;

  /// Return true if the body can react on force or constraint impulse.
  ///
  /// A body node can be called reactive if the parent skeleton is mobile and
  /// the number of dependent generalized coordinates is non zero. The body
  /// should be initialized first by calling BodyNode::init().
  bool isReactive() const;

  /// Set constraint impulse
  /// \param[in] _constImp Spatial constraint impulse w.r.t. body frame
  void setConstraintImpulse(const Eigen::Vector6d& _constImp);

  /// Add constraint impulse
  /// \param[in] _constImp Spatial constraint impulse w.r.t. body frame
  void addConstraintImpulse(const Eigen::Vector6d& _constImp);

  /// Add constraint impulse
  void addConstraintImpulse(const Eigen::Vector3d& _constImp,
                            const Eigen::Vector3d& _offset,
                            bool _isImpulseLocal = false,
                            bool _isOffsetLocal = true);

  /// Clear constraint impulse
  virtual void clearConstraintImpulse();

  /// Return constraint impulse
  const Eigen::Vector6d& getConstraintImpulse() const;

  //----------------------------------------------------------------------------
  // Energies
  //----------------------------------------------------------------------------

  /// Return kinetic energy.
  virtual double getKineticEnergy() const;

  /// Return potential energy.
  virtual double getPotentialEnergy(const Eigen::Vector3d& _gravity) const;

  /// Return linear momentum.
  Eigen::Vector3d getLinearMomentum() const;

  /// Return angular momentum.
  Eigen::Vector3d getAngularMomentum(
      const Eigen::Vector3d& _pivot = Eigen::Vector3d::Zero());

  //--------------------------------------------------------------------------
  // Rendering
  //--------------------------------------------------------------------------

  /// Render the entire subtree rooted at this body node.
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true, int _depth = 0) const;

  /// Render the markers
  void drawMarkers(renderer::RenderInterface* _ri = NULL,
                   const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                   bool _useDefaultColor = true) const;

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class Skeleton;
  friend class Joint;
  friend class SoftBodyNode;
  friend class PointMass;

protected:
  /// Initialize the vector members with proper sizes.
  virtual void init(Skeleton* _skeleton);

  //--------------------------------------------------------------------------
  // Recursive algorithms
  //--------------------------------------------------------------------------

  /// Update transformation
  virtual void updateTransform();

  /// Update spatial velocity
  virtual void updateVelocity();

  /// Update partial spatial acceleration due to parent joint's velocity
  virtual void updatePartialAcceleration();

  /// Update spatial acceleration with the partial spatial acceleration
  virtual void updateAcceleration();

  // TODO(JS): Need revise
  ///
  virtual void updateBodyWrench(const Eigen::Vector3d& _gravity,
                                bool _withExternalForces = false);

  // TODO(JS): Need revise
  ///
  virtual void updateGeneralizedForce(bool _withDampingForces = false);

  /// Update articulated body inertia with implicit joint damping and spring
  /// forces
  virtual void updateArtInertia(double _timeStep);

  /// Update bias force with implicit joint damping and spring forces
  virtual void updateBiasForce(const Eigen::Vector3d& _gravity,
                               double _timeStep);

  /// Update joint acceleration and body acceleration for forward dynamics
  virtual void updateJointAndBodyAcceleration();

  /// Update transmitted force from parent joint
  virtual void updateTransmittedWrench();

  //----------------------------------------------------------------------------
  // Impulse based dynamics
  //----------------------------------------------------------------------------

  /// Update impulsive bias force for impulse-based forward dynamics
  /// algorithm
  virtual void updateBiasImpulse();

  /// Update joint velocity change for impulse-based forward dynamics
  /// algorithm
  virtual void updateJointVelocityChange();

  /// Update body velocity change for impulse-based forward dynamics
  /// algorithm
//  virtual void updateBodyVelocityChange();

  ///
  virtual void updateBodyImpForceFwdDyn();

  ///
  virtual void updateConstrainedJointAndBodyAcceleration(double _timeStep);

  ///
  virtual void updateConstrainedTransmittedForce(double _timeStep);

  ///
  virtual void updateMassMatrix();
  virtual void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);
  virtual void aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                      double _timeStep);

  ///
  virtual void updateInvMassMatrix();
  virtual void updateInvAugMassMatrix();
  virtual void aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col);
  virtual void aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol, int _col,
                                         double _timeStep);

  ///
  virtual void aggregateCoriolisForceVector(Eigen::VectorXd* _C);

  ///
  virtual void aggregateGravityForceVector(Eigen::VectorXd* _g,
                                           const Eigen::Vector3d& _gravity);

  ///
  virtual void updateCombinedVector();
  virtual void aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                       const Eigen::Vector3d& _gravity);

  /// Aggregate the external forces mFext in the generalized coordinates
  /// recursively
  virtual void aggregateExternalForces(Eigen::VectorXd* _Fext);

  ///
  virtual void aggregateSpatialToGeneralized(Eigen::VectorXd* _generalized,
                                             const Eigen::Vector6d& _spatial);

  //--------------------------------------------------------------------------
  // General properties
  //--------------------------------------------------------------------------

  /// A unique ID of this node globally.
  int mID;

  /// Counts the number of nodes globally.
  static int msBodyNodeCount;

  ///
  std::string mName;

  /// If the gravity mode is false, this body node does not
  /// being affected by gravity.
  bool mGravityMode;

  /// Generalized inertia.
  math::Inertia mI;

  /// Generalized inertia at center of mass.
  Eigen::Vector3d mCenterOfMass;
  double mIxx;
  double mIyy;
  double mIzz;
  double mIxy;
  double mIxz;
  double mIyz;
  double mMass;

  /// Coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of friction
  double mRestitutionCoeff;

  ///
  std::vector<Shape*> mVizShapes;

  ///
  std::vector<Shape*> mColShapes;

  /// Indicating whether this node is collidable.
  bool mIsCollidable;

  /// Whether the node is currently in collision with another node.
  bool mIsColliding;

  //--------------------------------------------------------------------------
  // Structual Properties
  //--------------------------------------------------------------------------

  /// Pointer to the model this body node belongs to.
  Skeleton* mSkeleton;

  ///
  Joint* mParentJoint;

  ///
  BodyNode* mParentBodyNode;

  ///
  std::vector<BodyNode*> mChildBodyNodes;

  /// List of markers associated
  std::vector<Marker*> mMarkers;

  /// A increasingly sorted list of dependent dof indices.
  std::vector<int> mDependentGenCoordIndices;

  //--------------------------------------------------------------------------
  // Dynamical Properties
  //--------------------------------------------------------------------------

  /// World transformation
  Eigen::Isometry3d mW;

  /// Body Jacobian
  math::Jacobian mBodyJacobian;

  /// Dirty flag for body Jacobian.
  bool mIsBodyJacobianDirty;

  /// Time derivative of body Jacobian.
  math::Jacobian mBodyJacobianDeriv;

  /// Dirty flag for time derivative of body Jacobian.
  bool mIsBodyJacobianDerivDirty;

  /// Spatial body velocity w.r.t. body frame
  Eigen::Vector6d mV;

  /// Partial spatial body acceleration due to parent joint's velocity
  Eigen::Vector6d mPartialAcceleration;

  /// Spatial body acceleration w.r.t. body frame
  Eigen::Vector6d mA;

  /// Transmitted wrench from parent to the bodynode expressed in body-fixed
  /// frame
  Eigen::Vector6d mF;

  /// External spatial force
  Eigen::Vector6d mFext;

  /// Spatial gravity force
  Eigen::Vector6d mFgravity;

  /// Articulated body inertia
  math::Inertia mArtInertia;

  /// Articulated body inertia for implicit joing damping and spring forces
  math::Inertia mArtInertiaImplicit;

  /// Bias force
  Eigen::Vector6d mBiasForce;

  /// Cache data for combined vector of the system.
  Eigen::Vector6d mCg_dV;
  Eigen::Vector6d mCg_F;

  /// Cache data for gravity force vector of the system.
  Eigen::Vector6d mG_F;

  /// Cache data for external force vector of the system.
  Eigen::Vector6d mFext_F;

  /// Cache data for mass matrix of the system.
  Eigen::Vector6d mM_dV;
  Eigen::Vector6d mM_F;

  /// Cache data for inverse mass matrix of the system.
  Eigen::Vector6d mInvM_c;
  Eigen::Vector6d mInvM_U;

  /// Cache data for arbitrary spatial value
  Eigen::Vector6d mArbitrarySpatial;

  //------------------------- Impulse-based Dyanmics ---------------------------
  /// Velocity change due to to external impulsive force exerted on
  ///        bodies of the parent skeleton.
  Eigen::Vector6d mDelV;

  /// Impulsive bias force due to external impulsive force exerted on
  ///        bodies of the parent skeleton.
  Eigen::Vector6d mBiasImpulse;

  /// Constraint impulse: contact impulse, dynamic joint impulse
  Eigen::Vector6d mConstraintImpulse;

  /// Generalized impulsive body force w.r.t. body frame.
  Eigen::Vector6d mImpF;

  /// Update body Jacobian. getBodyJacobian() calls this function if
  ///        mIsBodyJacobianDirty is true.
  void _updateBodyJacobian();

  /// Update time derivative of body Jacobian. getBodyJacobianTimeDeriv()
  ///        calls this function if mIsBodyJacobianTimeDerivDirty is true.
  void _updateBodyJacobianDeriv();

private:
  ///
  void _updateSpatialInertia();

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BODYNODE_H_
