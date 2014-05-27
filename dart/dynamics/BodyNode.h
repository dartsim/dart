/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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

/*! \mainpage Dynamic Animation and Robotics Toolkits

DART is an open source library for developing kinematic and dynamic
applications in robotics and computer animation. DART features two
main components: a multibody dynamic simulator developed by Georgia
Tech Graphics Lab and a variety of motion planning algorithms
developed by Georgia Tech Humanoid Robotics Lab. This document focuses
only on the dynamic simulator.

The multibody dynamics simulator in DART is designed to aid
development of motion control algorithms. DART uses generalized
coordinates to represent articulated rigid body systems and computes
Lagrange’s equations derived from D’Alembert’s principle to describe
the dynamics of motion. In contrast to many popular physics engines
which view the simulator as a black box, DART provides full access to
internal kinematic and dynamic quantities, such as mass matrix,
Coriolis and centrifugal force, transformation matrices and their
derivatives, etc. DART also provides efficient computation of Jacobian
matrices for arbitrary body points and coordinate frames.

The contact and collision are handled using an implicit time-stepping,
velocity-based LCP (linear-complementarity problem) to guarantee
non-penetration, directional friction, and approximated Coulombs
friction cone conditions. The LCP problem is solved efficiently by
Lemke's algorithm. For the collision detection, DART directly uses FCL
package developed by UNC Gamma Lab.

In addition, DART supports various joint types (ball-and-socket,
universal, hinge, and prismatic joints) and arbitrary meshes. DART
also provides two explicit integration methods: first-order
Runge-Kutta and fourth-order Runge Kutta.

*/

#ifndef DART_DYNAMICS_BODYNODE_H_
#define DART_DYNAMICS_BODYNODE_H_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "dart/common/Deprecated.h"
#include "dart/math/Geometry.h"
#include "dart/optimizer/Function.h"

namespace dart {
namespace optimizer {
class Function;
}  // namespace optimizer
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
  /// Inverse kinematics policy
  /// IKP_PARENT_JOINT   : search optimal configuration for the parent joint of
  ///                      the target body node
  /// IKP_ANCESTOR_JOINTS: search optimal configurations for the ancestor joints
  ///                      of the target body node
  /// IKP_ALL_JOINTS     : search optimal configurations for the all joints in
  ///                      the skeleton.
  enum InverseKinematicsPolicy {
    IKP_PARENT_JOINT,
    IKP_ANCESTOR_JOINTS,
    IKP_ALL_JOINTS
  };

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------

  ///
  explicit BodyNode(const std::string& _name = "BodyNode");

  ///
  virtual ~BodyNode();

  //--------------------------------------------------------------------------
  // Static properties().
  //--------------------------------------------------------------------------

  ///
  void setName(const std::string& _name);

  ///
  const std::string& getName() const;

  /// Set whether gravity affects this body.
  /// \param[in] _mode True to enable gravity.
  void setGravityMode(bool _gravityMode);

  /// Get the gravity mode.
  /// \return True if gravity is enabled.
  bool getGravityMode() const;

  /// Get whether this body node will collide with others in the world.
  /// \return True if collisions is enabled.
  bool isCollidable() const;

  /// Set whether this body node will collide with others in the world.
  /// \param[in] _isCollidable True to enable collisions.
  void setCollidable(bool _isCollidable);

  ///
  void setMass(double _mass);

  ///
  double getMass() const;

  /// Set moment of inertia defined around the center of mass.
  /// Moments of inertia, _Ixx, _Iyy, _Izz, must be positive or zero values.
  void setInertia(double _Ixx, double _Iyy, double _Izz,
                  double _Ixy = 0.0, double _Ixz = 0.0, double _Iyz = 0.0);

  ///
  void setLocalCOM(const Eigen::Vector3d& _com);

  /// Get body's COM w.r.t. body frame.
  const Eigen::Vector3d& getLocalCOM() const;

  /// Get body's COM w.r.t. world frame.
  Eigen::Vector3d getWorldCOM() const;

  /// Get body's COM velocity w.r.t. world frame.
  Eigen::Vector3d getWorldCOMVelocity() const;

  /// Get body's COM acceleration w.r.t. world frame.
  Eigen::Vector3d getWorldCOMAcceleration() const;

  ///
  Eigen::Matrix6d getInertia() const;

  /// Set coefficient of friction in range of [0, ~]
  void setFrictionCoeff(double _coeff);

  /// Get frictional coefficient.
  double getFrictionCoeff() const;

  /// Set coefficient of restitution in range of [0, 1]
  void setRestitutionCoeff(double _coeff);

  /// Get coefficient of restitution
  double getRestitutionCoeff() const;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------

  ///
  int getSkeletonIndex() const;

  ///
  void addVisualizationShape(Shape *_p);

  ///
  int getNumVisualizationShapes() const;

  ///
  Shape* getVisualizationShape(int _idx) const;

  ///
  void addCollisionShape(Shape *_p);

  ///
  int getNumCollisionShapes() const;

  ///
  Shape* getCollisionShape(int _idx) const;

  ///
  Skeleton* getSkeleton() const;

  ///
  void setParentJoint(Joint* _joint);

  ///
  Joint* getParentJoint() const;

  ///
  BodyNode* getParentBodyNode() const;

  ///
  void addChildBodyNode(BodyNode* _body);

  ///
  BodyNode* getChildBodyNode(int _idx) const;

  ///
  int getNumChildBodyNodes() const;

  ///
  void addMarker(Marker* _marker);

  ///
  int getNumMarkers() const;

  ///
  Marker* getMarker(int _idx) const;

  /// Test whether this generalized coordinate is dependent or not.
  /// \return True if this body node is dependent on the generalized
  ///         coordinate.
  /// \param[in] _genCoordIndex Index of generalized coordinate in the
  ///                           skeleton.
  /// \warning You may want to use getNumDependentGenCoords or
  ///          getDependentGenCoordIndex for efficiency.
  bool dependsOn(int _genCoordIndex) const;

  /// The number of the generalized coordinates by which this node is
  ///        affected.
  int getNumDependentGenCoords() const;

  /// Return a generalized coordinate index from the array index
  ///        (< getNumDependentDofs).
  int getDependentGenCoordIndex(int _arrayIndex) const;

  //----------------------------------------------------------------------------
  // Inverse kinematics
  //----------------------------------------------------------------------------

  /// Try to find optimal configurations to fit the world transform
  ///        of this body node to the target transformation.
  /// This problem is solved by optimization. The objective is to minimize the
  /// geometric distance between the target transformation and the world
  /// transformation of this body.
  void fitWorldTransform(const Eigen::Isometry3d& _target,
                         InverseKinematicsPolicy _policy = IKP_PARENT_JOINT,
                         bool _jointLimit = true);

  /// Try to find optimal joint velocities to fit the linear velocity of
  ///        this body node to the target linear velocity.
  void fitWorldLinearVel(const Eigen::Vector3d& _targetLinVel,
                         InverseKinematicsPolicy _policy = IKP_PARENT_JOINT,
                         bool _jointVelLimit = true);

  /// Try to find optimal joint velocities to fit the linear velocity of
  ///        this body node to the target linear velocity.
  void fitWorldAngularVel(const Eigen::Vector3d& _targetAngVel,
                          InverseKinematicsPolicy _policy = IKP_PARENT_JOINT,
                          bool _jointVelLimit = true);

  //--------------------------------------------------------------------------
  // Properties updated by dynamics (kinematics)
  //--------------------------------------------------------------------------

  /// Get the transformation from the world frame to this body node
  ///        frame.
  const Eigen::Isometry3d& getWorldTransform() const;

  /// Get the generalized velocity at the origin of this body node
  ///        where the velocity is expressed in this body node frame.
  const Eigen::Vector6d& getBodyVelocity() const;

  /// Get the generalized velocity at a point on this body node where
  ///        the velocity is expressed in the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  Eigen::Vector6d getWorldVelocity(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isLocal                  = false) const;

  /// Get generalized acceleration at the origin of this body node
  /// where the acceleration is expressed in this body node frame.
  const Eigen::Vector6d& getBodyAcceleration() const;

  /// Get generalized acceleration at a point on this body node where
  ///        the acceleration is expressed in the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  Eigen::Vector6d getWorldAcceleration(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isOffsetLocal            = false) const;

  /// Get generalized Jacobian at the origin of this body node where
  ///        the Jacobian is expressed in this body node frame.
  const math::Jacobian& getBodyJacobian();

  /// Get generalized Jacobian at a point on this body node where the
  ///        Jacobian is expressed in the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  math::Jacobian getWorldJacobian(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isOffsetLocal            = false);

  /// Get time derivative of generalized Jacobian at the origin of this
  ///        body node where the Jacobian is expressed in this body node
  ///        frame.
  const math::Jacobian& getBodyJacobianTimeDeriv();

  /// Get time derivative of generalized Jacobian at a point on this
  ///        body node where the time derivative of Jacobian is expressed in
  ///        the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  math::Jacobian getWorldJacobianTimeDeriv(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isOffsetLocal            = false);

  ///
  const Eigen::Vector6d& getBodyVelocityChange() const;

  /// Set whether this body node is colliding with others. This is
  /// called by collision detector.
  /// \param[in] True if this body node is colliding.
  void setColliding(bool _isColliding);

  /// Get whether this body node is colliding with others.
  /// \return True if this body node is colliding.
  bool isColliding();

  /// Add applying linear Cartesian forces to this node.
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

  /// Set constraint impulse
  /// \param[in] _constImp Spatial constraint impulse w.r.t. body frame
  void setConstraintImpulse(const Eigen::Vector6d& _constImp);

  /// Add constraint impulse
  /// \param[in] _constImp Spatial constraint impulse w.r.t. body frame
  void addConstraintImpulse(const Eigen::Vector6d& _constImp);

  /// Add constraint impulse
  void addConstraintImpulse(
      const Eigen::Vector3d& _constImp,
      const Eigen::Vector3d& _offset,
      bool _isImpulseLocal = false,
      bool _isOffsetLocal = true);

  /// Clear constraint impulse
  virtual void clearConstraintImpulse();

  /// Get constraint impulse
  const Eigen::Vector6d& getConstraintImpulse() const;

  //----------------------------------------------------------------------------
  // Energies
  //----------------------------------------------------------------------------

  /// Get kinetic energy.
  virtual double getKineticEnergy() const;

  /// Get potential energy.
  virtual double getPotentialEnergy(const Eigen::Vector3d& _gravity) const;

  /// Get linear momentum.
  Eigen::Vector3d getLinearMomentum() const;

  /// Get angular momentum.
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

// TODO(JS): Temporary change for soft body dynamics
protected:
//public:
  /// Initialize the vector members with proper sizes.
  virtual void init(Skeleton* _skeleton, int _skeletonIndex);

  // TODO(JS): This function will be deprecated when we stop using GenCoord
  //           and GenCoordSystem classes.
  /// Aggregate generalized coordinates of this body node to
  ///        generalized of the system.
  virtual void aggregateGenCoords(std::vector<GenCoord*>* _genCoords);

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
  virtual void updateBodyForce(const Eigen::Vector3d& _gravity,
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
  virtual void updateTransmittedForce();

  //----------------------------------------------------------------------------
  // Impulse based dynamics
  //----------------------------------------------------------------------------
public:
  ///
  bool isImpulseReponsible() const;

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

protected:
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

  /// Aggregate the external forces mFext in the generalized
  ///        coordinates recursively.
  virtual void aggregateExternalForces(Eigen::VectorXd* _Fext);

  /// class TransformObjFunc
  class TransformObjFunc : public optimizer::Function
  {
  public:
    /// Constructor
    TransformObjFunc(BodyNode* _body, const Eigen::Isometry3d& _T,
                       Skeleton* _skeleton);

    /// Destructor
    virtual ~TransformObjFunc();

    /// \copydoc Function::eval
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x);

  protected:
    /// Target body node
    BodyNode* mBodyNode;

    /// Target transform
    Eigen::Isometry3d mT;

    /// Skeleton
    Skeleton* mSkeleton;
  };

  /// class VelocityObjFunc
  class VelocityObjFunc : public optimizer::Function
  {
  public:
    /// Velocity type
    enum VelocityType
    {
      VT_LINEAR,
      VT_ANGULAR
    };

    /// Constructor
    VelocityObjFunc(BodyNode* _body, const Eigen::Vector3d& _vel,
                    VelocityType _velType, Skeleton* _skeleton);

    /// Destructor
    virtual ~VelocityObjFunc();

    /// \copydoc Function::eval
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x);

  protected:
    /// Target body node
    BodyNode* mBodyNode;

    /// Target transform
    Eigen::Vector6d mVelocity;

    /// Skeleton
    Skeleton* mSkeleton;

    /// Velocity type [ linear | angular ]
    VelocityType mVelocityType;
  };

  /// Implementation for fitWorldTransform with IKP_PARENT_JOINT policy
  void fitWorldTransformParentJointImpl(const Eigen::Isometry3d& _target,
                                        bool _jointLimit = true);

  /// Implementation for fitWorldTransform with IKP_ANCESTOR_JOINTS
  ///        policy
  void fitWorldTransformAncestorJointsImpl(const Eigen::Isometry3d& _target,
                                           bool _jointLimit = true);

  /// Implementation for fitWorldTransform with IKP_ALL_JOINTS policy
  void fitWorldTransformAllJointsImpl(const Eigen::Isometry3d& _target,
                                      bool _jointLimit = true);

  //--------------------------------------------------------------------------
  // General properties
  //--------------------------------------------------------------------------

  /// A unique ID of this node globally.
  int mID;

  /// Counts the number of nodes globally.
  static int msBodyNodeCount;

  ///
  std::string mName;

  /// Index in the model
  int mSkelIndex;

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
  math::Jacobian mBodyJacobianTimeDeriv;

  /// Dirty flag for time derivative of body Jacobian.
  bool mIsBodyJacobianTimeDerivDirty;

  /// Spatial body velocity w.r.t. body frame
  Eigen::Vector6d mV;

  /// Partial spatial body acceleration due to parent joint's velocity
  Eigen::Vector6d mPartialAcceleration;

  /// Spatial body acceleration w.r.t. body frame
  Eigen::Vector6d mA;

  /// Transmitted spatial body force by parent joint w.r.t. body frame
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

// TODO(JS): Temporary code for soft body dynamics
// protected:
public:
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
  void _updateBodyJacobianTimeDeriv();

private:
  ///
  void _updateGeralizedInertia();

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BODYNODE_H_
