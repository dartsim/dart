/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
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

/// \brief BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
class BodyNode {
public:
  friend class Skeleton;

  //--------------------------------------------------------------------------
  // Constructor and Desctructor
  //--------------------------------------------------------------------------
  /// \brief
  explicit BodyNode(const std::string& _name = "Noname BodyNode");

  /// \brief
  virtual ~BodyNode();

  //--------------------------------------------------------------------------
  // Static properties().
  //--------------------------------------------------------------------------
  /// \brief
  void setName(const std::string& _name);

  /// \brief
  const std::string& getName() const;

  /// \brief Set whether gravity affects this body.
  /// \param[in] _mode True to enable gravity.
  void setGravityMode(bool _gravityMode);

  /// \brief Get the gravity mode.
  /// \return True if gravity is enabled.
  bool getGravityMode() const;

  /// \brief Get whether this body node will collide with others in the world.
  /// \return True if collisions is enabled.
  bool isCollidable() const;

  /// \brief Set whether this body node will collide with others in the world.
  /// \param[in] _isCollidable True to enable collisions.
  void setCollidable(bool _isCollidable);

  /// \brief
  void setMass(double _mass);

  /// \brief
  double getMass() const;

  /// \brief Set moment of inertia defined around the center of mass.
  /// Moments of inertia, _Ixx, _Iyy, _Izz, must be positive or zero values.
  void setInertia(double _Ixx, double _Iyy, double _Izz,
                  double _Ixy = 0.0, double _Ixz = 0.0, double _Iyz = 0.0);

  /// \brief
  void setLocalCOM(const Eigen::Vector3d& _com);

  /// \brief Get body's COM w.r.t. body frame.
  const Eigen::Vector3d& getLocalCOM() const;

  /// \brief Get body's COM w.r.t. world frame.
  Eigen::Vector3d getWorldCOM() const;

  /// \brief Get body's COM velocity w.r.t. world frame.
  Eigen::Vector3d getWorldCOMVelocity() const;

  /// \brief Get body's COM acceleration w.r.t. world frame.
  Eigen::Vector3d getWorldCOMAcceleration() const;

  /// \brief
  Eigen::Matrix6d getInertia() const;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------
  /// \brief
  int getSkeletonIndex() const;

  /// \brief
  void addVisualizationShape(Shape *_p);

  /// \brief
  int getNumVisualizationShapes() const;

  /// \brief
  Shape* getVisualizationShape(int _idx) const;

  /// \brief
  void addCollisionShape(Shape *_p);

  /// \brief
  int getNumCollisionShapes() const;

  /// \brief
  Shape* getCollisionShape(int _idx) const;

  /// \brief
  Skeleton* getSkeleton() const;

  /// \brief
  void setParentJoint(Joint* _joint);

  /// \brief
  Joint* getParentJoint() const;

  /// \brief
  BodyNode* getParentBodyNode() const;

  /// \brief
  void addChildBodyNode(BodyNode* _body);

  /// \brief
  BodyNode* getChildBodyNode(int _idx) const;

  /// \brief
  int getNumChildBodyNodes() const;

  /// \brief
  void addMarker(Marker* _marker);

  /// \brief
  int getNumMarkers() const;

  /// \brief
  Marker* getMarker(int _idx) const;

  /// \brief Test whether this generalized coordinate is dependent or not.
  /// \return True if this body node is dependent on the generalized
  ///         coordinate.
  /// \param[in] _genCoordIndex Index of generalized coordinate in the
  ///                           skeleton.
  /// \warning You may want to use getNumDependentGenCoords or
  ///          getDependentGenCoordIndex for efficiency.
  bool dependsOn(int _genCoordIndex) const;

  /// \brief The number of the generalized coordinates by which this node is
  ///        affected.
  int getNumDependentGenCoords() const;

  /// \brief Return a generalized coordinate index from the array index
  ///        (< getNumDependentDofs).
  int getDependentGenCoordIndex(int _arrayIndex) const;

  //--------------------------------------------------------------------------
  // Properties updated by dynamics (kinematics)
  //--------------------------------------------------------------------------
  /// \brief Get the transformation from the world frame to this body node
  ///        frame.
  const Eigen::Isometry3d& getWorldTransform() const;

  /// \brief Get the generalized velocity at the origin of this body node
  ///        where the velocity is expressed in this body node frame.
  const Eigen::Vector6d& getBodyVelocity() const;

  /// \brief Get the generalized velocity at a point on this body node where
  ///        the velocity is expressed in the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  Eigen::Vector6d getWorldVelocity(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isLocal                  = false) const;

  /// \brief Get generalized acceleration at the origin of this body node
  /// where the acceleration is expressed in this body node frame.
  const Eigen::Vector6d& getBodyAcceleration() const;

  /// \brief Get generalized acceleration at a point on this body node where
  ///        the acceleration is expressed in the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  Eigen::Vector6d getWorldAcceleration(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isOffsetLocal            = false) const;

  /// \brief Get generalized Jacobian at the origin of this body node where
  ///        the Jacobian is expressed in this body node frame.
  const math::Jacobian& getBodyJacobian();

  /// \brief Get generalized Jacobian at a point on this body node where the
  ///        Jacobian is expressed in the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  math::Jacobian getWorldJacobian(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isOffsetLocal            = false);

  /// \brief Get time derivative of generalized Jacobian at the origin of this
  ///        body node where the Jacobian is expressed in this body node
  ///        frame.
  const math::Jacobian& getBodyJacobianTimeDeriv();

  /// \brief Get time derivative of generalized Jacobian at a point on this
  ///        body node where the time derivative of Jacobian is expressed in
  ///        the world frame.
  /// \param[in] _offset Position vector relative to the origin the body frame.
  /// \param[in] _isLocal True if _offset is expressed in the body frame.
  ///                     False if _offset is expressed in the world frame.
  math::Jacobian getWorldJacobianTimeDeriv(
      const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
      bool _isOffsetLocal            = false);

  /// \brief Set whether this body node is colliding with others.
  /// \param[in] True if this body node is colliding.
  void setColliding(bool _isColliding);

  /// \brief Get whether this body node is colliding with others.
  /// \return True if this body node is colliding.
  bool isColliding();

  /// \brief Add applying linear Cartesian forces to this node.
  ///
  /// A force is defined by a point of application and a force vector. The
  /// last two parameters specify frames of the first two parameters.
  /// Coordinate transformations are applied when needed. The point of
  /// application and the force in local coordinates are stored in mContacts.
  /// When conversion is needed, make sure the transformations are avaialble.
  void addExtForce(const Eigen::Vector3d& _force,
                   const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
                   bool _isOffsetLocal = true,
                   bool _isForceLocal = false);

  /// \brief Set Applying linear Cartesian forces to this node.
  void setExtForce(const Eigen::Vector3d& _force,
                   const Eigen::Vector3d& _offset = Eigen::Vector3d::Zero(),
                   bool _isOffsetLocal = true,
                   bool _isForceLocal = false);

  /// \brief Add applying Cartesian torque to the node.
  ///
  /// The torque in local coordinates is accumulated in mExtTorqueBody.
  void addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal = false);

  /// \brief Set applying Cartesian torque to the node.
  ///
  /// The torque in local coordinates is accumulated in mExtTorqueBody.
  void setExtTorque(const Eigen::Vector3d& _torque, bool _isLocal = false);

  /// \brief Clean up structures that store external forces: mContacts, mFext,
  ///        mExtForceBody and mExtTorqueBody.
  ///
  /// Called from @Skeleton::clearExternalForces.
  virtual void clearExternalForces();

  /// \brief
  void addContactForce(const Eigen::Vector6d& _contactForce);

  /// \brief
  int getNumContactForces() const;

  /// \brief
  const Eigen::Vector6d& getContactForce(int _idx);

  /// \brief
  void clearContactForces();

  /// \brief
  const Eigen::Vector6d& getExternalForceLocal() const;

  /// \brief
  Eigen::Vector6d getExternalForceGlobal() const;

  /// \brief
  const Eigen::Vector6d& getBodyForce() const;

  /// \brief Get kinetic energy.
  virtual double getKineticEnergy() const;

  /// \brief Get potential energy.
  virtual double getPotentialEnergy(const Eigen::Vector3d& _gravity) const;

  /// \brief Get linear momentum.
  Eigen::Vector3d getLinearMomentum() const;

  /// \brief Get angular momentum.
  Eigen::Vector3d getAngularMomentum(
      const Eigen::Vector3d& _pivot = Eigen::Vector3d::Zero());

  //--------------------------------------------------------------------------
  // Rendering
  //--------------------------------------------------------------------------
  /// \brief Render the entire subtree rooted at this body node.
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true, int _depth = 0) const;

  /// \brief Render the markers
  void drawMarkers(renderer::RenderInterface* _ri = NULL,
                   const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                   bool _useDefaultColor = true) const;

// TODO(JS): Temporary change for soft body dynamics
// protected:
public:
  /// \brief Initialize the vector members with proper sizes.
  virtual void init(Skeleton* _skeleton, int _skeletonIndex);

  // TODO(JS): This function will be deprecated when we stop using GenCoord
  //           and GenCoordSystem classes.
  /// \brief Aggregate generalized coordinates of this body node to
  ///        generalized of the system.
  virtual void aggregateGenCoords(std::vector<GenCoord*>* _genCoords);

  //--------------------------------------------------------------------------
  // Sub-functions for Recursive Kinematics Algorithms
  //--------------------------------------------------------------------------
  /// \brief Update local transformations and world transformations.
  virtual void updateTransform();

  /// @brief // TODO(JS): This is workaround for Issue #122.
  virtual void updateTransform_Issue122(double _timeStep);

  /// \brief
  virtual void updateVelocity();

  /// \brief
  /// parentJoint.dS --> dJ
  virtual void updateEta();

  /// @brief // TODO(JS): This is workaround for Issue #122.
  virtual void updateEta_Issue122();

  /// \brief
  /// parentJoint.V, parentJoint.dV, parentBody.dV, V --> dV
  virtual void updateAcceleration();

  /// \brief
  /// childBodies.F, V, dV --> F, Fgravity
  virtual void updateBodyForce(const Eigen::Vector3d& _gravity,
                               bool _withExternalForces = false);

  /// \brief
  /// parentJoint.S, F --> tau
  virtual void updateGeneralizedForce(bool _withDampingForces = false);

  /// \brief
  virtual void updateArticulatedInertia(double _timeStep);

  /// \brief
  virtual void updateBiasForce(double _timeStep,
                               const Eigen::Vector3d& _gravity);

  /// \brief
  virtual void update_ddq();

  /// \brief
  virtual void update_F_fs();

  /// \brief
  virtual void updateMassMatrix();
  virtual void aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col);
  virtual void aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                      double _timeStep);

  /// \brief
  virtual void updateInvMassMatrix();
  virtual void updateInvAugMassMatrix();
  virtual void aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col);
  virtual void aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol, int _col,
                                         double _timeStep);

  /// \brief
  virtual void aggregateCoriolisForceVector(Eigen::VectorXd* _C);

  /// \brief
  virtual void aggregateGravityForceVector(Eigen::VectorXd* _g,
                                           const Eigen::Vector3d& _gravity);

  /// \brief
  virtual void updateCombinedVector();
  virtual void aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                       const Eigen::Vector3d& _gravity);

  /// \brief Aggregate the external forces mFext in the generalized
  ///        coordinates recursively.
  virtual void aggregateExternalForces(Eigen::VectorXd* _Fext);

  //--------------------------------------------------------------------------
  // General properties
  //--------------------------------------------------------------------------
  /// \brief A unique ID of this node globally.
  int mID;

  /// \brief Counts the number of nodes globally.
  static int msBodyNodeCount;

  /// \brief
  std::string mName;

  /// \brief Index in the model
  int mSkelIndex;

  /// \brief If the gravity mode is false, this body node does not
  /// being affected by gravity.
  bool mGravityMode;

  /// \brief Generalized inertia.
  math::Inertia mI;

  /// \brief Generalized inertia at center of mass.
  Eigen::Vector3d mCenterOfMass;
  double mIxx;
  double mIyy;
  double mIzz;
  double mIxy;
  double mIxz;
  double mIyz;
  double mMass;

  /// \brief
  std::vector<Shape*> mVizShapes;

  /// \brief
  std::vector<Shape*> mColShapes;

  /// \brief Indicating whether this node is collidable.
  bool mIsCollidable;

  /// \brief Whether the node is currently in collision with another node.
  bool mIsColliding;

  //--------------------------------------------------------------------------
  // Structual Properties
  //--------------------------------------------------------------------------
  /// \brief Pointer to the model this body node belongs to.
  Skeleton* mSkeleton;

  /// \brief
  Joint* mParentJoint;

  /// \brief
  BodyNode* mParentBodyNode;

  /// \brief
  std::vector<BodyNode*> mChildBodyNodes;

  /// \brief List of markers associated
  std::vector<Marker*> mMarkers;

  /// \brief A increasingly sorted list of dependent dof indices.
  std::vector<int> mDependentGenCoordIndices;

  //--------------------------------------------------------------------------
  // Dynamical Properties
  //--------------------------------------------------------------------------
  /// \brief World transformation.
  Eigen::Isometry3d mW;

  /// \brief Body Jacobian.
  math::Jacobian mBodyJacobian;

  /// \brief Dirty flag for body Jacobian.
  bool mIsBodyJacobianDirty;

  /// \brief Time derivative of body Jacobian.
  math::Jacobian mBodyJacobianTimeDeriv;

  /// \brief Dirty flag for time derivative of body Jacobian.
  bool mIsBodyJacobianTimeDerivDirty;

  /// \brief Generalized body velocity w.r.t. body frame.
  Eigen::Vector6d mV;

  /// \brief
  Eigen::Vector6d mEta;

  /// \brief Generalized body acceleration w.r.t. body frame.
  Eigen::Vector6d mdV;

  /// \brief Generalized body force w.r.t. body frame.
  Eigen::Vector6d mF;

  /// \brief
  Eigen::Vector6d mFext;

  /// \brief
  Eigen::Vector6d mFgravity;

  /// \brief Articulated inertia
  math::Inertia mAI;

  /// \brief Articulated inertia
  math::Inertia mImplicitAI;

  /// \brief Bias force
  Eigen::Vector6d mB;

  /// \brief
  math::Jacobian mAI_S;

  /// \brief
  math::Jacobian mImplicitAI_S;

  /// \brief
  math::Jacobian mAI_S_Psi;

  /// \brief
  math::Jacobian mImplicitAI_S_ImplicitPsi;

  /// \brief
  Eigen::MatrixXd mPsi;

  /// \brief
  Eigen::MatrixXd mImplicitPsi;

public:  // TODO(JS): This will be removed once Node class is implemented.
  /// \brief
  math::Inertia mPi;

  /// \brief
  math::Inertia mImplicitPi;

protected:  // TODO(JS):
  /// \brief
  Eigen::VectorXd mAlpha;

public:  // TODO(JS): This will be removed once Node class is implemented.
  /// \brief
  Eigen::Vector6d mBeta;

// TODO(JS): Temporary code for soft body dynamics
// protected:
public:
  /// \brief Cache data for combined vector of the system.
  Eigen::Vector6d mCg_dV;
  Eigen::Vector6d mCg_F;

  /// \brief Cache data for gravity force vector of the system.
  Eigen::Vector6d mG_F;

  /// \brief Cache data for external force vector of the system.
  Eigen::Vector6d mFext_F;

  /// \brief
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> >
  mContactForces;

  /// \brief Cache data for mass matrix of the system.
  Eigen::Vector6d mM_dV;
  Eigen::Vector6d mM_F;

  /// \brief Cache data for inverse mass matrix of the system.
  Eigen::VectorXd mInvM_a;
  Eigen::Vector6d mInvM_b;
  Eigen::Vector6d mInvM_c;
  Eigen::VectorXd mInvM_MInvVec;
  Eigen::Vector6d mInvM_U;

  /// \brief Update body Jacobian. getBodyJacobian() calls this function if
  ///        mIsBodyJacobianDirty is true.
  void _updateBodyJacobian();

  /// \brief Update time derivative of body Jacobian. getBodyJacobianTimeDeriv()
  ///        calls this function if mIsBodyJacobianTimeDerivDirty is true.
  void _updateBodyJacobianTimeDeriv();

private:
  /// \brief
  void _updateGeralizedInertia();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BODYNODE_H_
