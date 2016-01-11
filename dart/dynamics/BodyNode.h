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

#ifndef KIDO_DYNAMICS_BODYNODE_H_
#define KIDO_DYNAMICS_BODYNODE_H_

#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "dart/config.h"
#include "dart/common/Signal.h"
#include "dart/math/Geometry.h"
#include "dart/dynamics/Node.h"
#include "dart/dynamics/Frame.h"
#include "dart/dynamics/Inertia.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/SmartPointer.h"
#include "dart/dynamics/TemplatedJacobianNode.h"
#include "dart/dynamics/EndEffector.h"

const double KIDO_DEFAULT_FRICTION_COEFF = 1.0;
const double KIDO_DEFAULT_RESTITUTION_COEFF = 0.0;

namespace kido {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace kido

namespace kido {
namespace dynamics {

class GenCoord;
class Skeleton;
class Joint;
class DegreeOfFreedom;
class Shape;
class Marker;

/// BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
///
/// BodyNode inherits Frame, and a parent Frame of a BodyNode is the parent
/// BodyNode of the BodyNode.
class BodyNode :
    public SkeletonRefCountingBase,
    public TemplatedJacobianNode<BodyNode>
{
public:

  using ColShapeAddedSignal
      = common::Signal<void(const BodyNode*, ConstShapePtr _newColShape)>;

  using ColShapeRemovedSignal = ColShapeAddedSignal;

  using StructuralChangeSignal
      = common::Signal<void(const BodyNode*)>;

  struct UniqueProperties
  {
    /// Inertia information for the BodyNode
    Inertia mInertia;

    /// Array of collision shapes
    std::vector<ShapePtr> mColShapes;

    /// Indicates whether this node is collidable;
    bool mIsCollidable;

    /// Coefficient of friction
    double mFrictionCoeff;

    /// Coefficient of restitution
    double mRestitutionCoeff;

    /// Gravity will be applied if true
    bool mGravityMode;

    /// Properties of the Markers belonging to this BodyNode
    std::vector<Marker::Properties> mMarkerProperties;

    /// Constructor
    UniqueProperties(
        const Inertia& _inertia = Inertia(),
        const std::vector<ShapePtr>& _collisionShapes = std::vector<ShapePtr>(),
        bool _isCollidable = true,
        double _frictionCoeff = KIDO_DEFAULT_FRICTION_COEFF,
        double _restitutionCoeff = KIDO_DEFAULT_RESTITUTION_COEFF,
        bool _gravityMode = true);

    virtual ~UniqueProperties() = default;

    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /// Composition of Entity and BodyNode properties
  struct Properties : Entity::Properties, UniqueProperties
  {
    /// Composed constructor
    Properties(
        const Entity::Properties& _entityProperties = Entity::Properties("BodyNode"),
        const UniqueProperties& _bodyNodeProperties = UniqueProperties());

    virtual ~Properties() = default;
  };

  BodyNode(const BodyNode&) = delete;

  /// Destructor
  virtual ~BodyNode();

  /// Set the Properties of this BodyNode
  void setProperties(const Properties& _properties);

  /// Set the Properties of this BodyNode
  void setProperties(const UniqueProperties& _properties);

  /// Get the Properties of this BodyNode
  Properties getBodyNodeProperties() const;

  /// Copy the Properties of another BodyNode
  void copy(const BodyNode& _otherBodyNode);

  /// Copy the Properties of another BodyNode
  void copy(const BodyNode* _otherBodyNode);

  /// Same as copy(const BodyNode&)
  BodyNode& operator=(const BodyNode& _otherBodyNode);

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name) override;

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
      double& _Ixy, double& _Ixz, double& _Iyz) const;

  /// Return spatial inertia
  const Eigen::Matrix6d& getSpatialInertia() const;

  /// Set the inertia data for this BodyNode
  void setInertia(const Inertia& _inertia);

  /// Get the inertia data for this BodyNode
  const Inertia& getInertia() const;

  /// Return the articulated body inertia
  const math::Inertia& getArticulatedInertia() const;

  /// Return the articulated body inertia for implicit joint damping and spring
  /// forces
  const math::Inertia& getArticulatedInertiaImplicit() const;

  /// Set center of mass expressed in body frame
  void setLocalCOM(const Eigen::Vector3d& _com);

  /// Return center of mass expressed in body frame
  const Eigen::Vector3d& getLocalCOM() const;

  /// Return the center of mass with respect to an arbitrary Frame
  Eigen::Vector3d getCOM(const Frame* _withRespectTo = Frame::World()) const;

  /// Return the linear velocity of the center of mass, expressed in terms of
  /// arbitrary Frames
  Eigen::Vector3d getCOMLinearVelocity(
                          const Frame* _relativeTo = Frame::World(),
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Return the spatial velocity of the center of mass, expressed in
  /// coordinates of this Frame and relative to the World Frame
  Eigen::Vector6d getCOMSpatialVelocity() const;

  /// Return the spatial velocity of the center of mass, expressed in terms of
  /// arbitrary Frames
  Eigen::Vector6d getCOMSpatialVelocity(const Frame* _relativeTo,
                                        const Frame* _inCoordinatesOf) const;

  /// Return the linear acceleration of the center of mass, expressed in terms
  /// of arbitary Frames
  Eigen::Vector3d getCOMLinearAcceleration(
                          const Frame* _relativeTo = Frame::World(),
                          const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Return the acceleration of the center of mass expressed in coordinates of
  /// this BodyNode Frame and relative to the World Frame
  Eigen::Vector6d getCOMSpatialAcceleration() const;

  /// Return the spatial acceleration of the center of mass, expressed in terms
  /// of arbitrary Frames
  Eigen::Vector6d getCOMSpatialAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const;

  /// Set coefficient of friction in range of [0, ~]
  void setFrictionCoeff(double _coeff);

  /// Return frictional coefficient.
  double getFrictionCoeff() const;

  /// Set coefficient of restitution in range of [0, 1]
  void setRestitutionCoeff(double _coeff);

  /// Return coefficient of restitution
  double getRestitutionCoeff() const;

  //--------------------------------------------------------------------------
  // Structural Properties
  //--------------------------------------------------------------------------

  /// Add a collision Shape into the BodyNode
  void addCollisionShape(const ShapePtr& _shape);

  /// Remove a collision Shape from this BodyNode
  void removeCollisionShape(const ShapePtr& _shape);

  /// Remove all collision Shapes from this BodyNode
  void removeAllCollisionShapes();

  /// Return the number of collision shapes
  size_t getNumCollisionShapes() const;

  /// Return _index-th collision shape
  ShapePtr getCollisionShape(size_t _index);

  /// Return (const) _index-th collision shape
  ConstShapePtr getCollisionShape(size_t _index) const;

  /// Return the index of this BodyNode within its Skeleton
  size_t getIndexInSkeleton() const;

  /// Return the index of this BodyNode within its tree
  size_t getIndexInTree() const;

  /// Return the index of the tree that this BodyNode belongs to
  size_t getTreeIndex() const;

  /// Remove this BodyNode and all of its children (recursively) from their
  /// Skeleton. If a BodyNodePtr that references this BodyNode (or any of its
  /// children) still exists, the subtree will be moved into a new Skeleton
  /// with the given name. If the returned SkeletonPtr goes unused and no
  /// relevant BodyNodePtrs are held anywhere, then this BodyNode and all its
  /// children will be deleted.
  ///
  /// Note that this function is actually the same as split(), but given a
  /// different name for semantic reasons.
  SkeletonPtr remove(const std::string& _name = "temporary");

  /// Remove this BodyNode and all of its children (recursively) from their
  /// current parent BodyNode, and move them to another parent BodyNode. The new
  /// parent BodyNode can either be in a new Skeleton or the current one. If you
  /// pass in a nullptr, this BodyNode will become a new root BodyNode for its
  /// current Skeleton.
  ///
  /// Using this function will result in changes to the indexing of
  /// (potentially) all BodyNodes and Joints in the current Skeleton, even if
  /// the BodyNodes are kept within the same Skeleton.
  bool moveTo(BodyNode* _newParent);

  /// This is a version of moveTo(BodyNode*) that allows you to explicitly move
  /// this BodyNode into a different Skeleton. The key difference for this
  /// version of the function is that you can make this BodyNode a root node in
  /// a different Skeleton, which is not something that can be done by the other
  /// version.
  bool moveTo(const SkeletonPtr& _newSkeleton, BodyNode* _newParent);

#ifdef _WIN32
  template <typename JointType>
  static typename JointType::Properties createJointProperties()
  {
    return typename JointType::Properties();
  }

  template <typename NodeType>
  static typename NodeType::Properties createBodyNodeProperties()
  {
    return typename NodeType::Properties();
  }
#endif
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// A version of moveTo(BodyNode*) that also changes the Joint type of the
  /// parent Joint of this BodyNode. This function returns the pointer to the
  /// newly created Joint. The original parent Joint will be deleted.
  ///
  /// This function can be used to change the Joint type of the parent Joint of
  /// this BodyNode, but note that the indexing of the BodyNodes and Joints in
  /// this Skeleton will still be changed, even if only the Joint type is
  /// changed.
  template <class JointType>
  JointType* moveTo(BodyNode* _newParent,
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>());
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties());
#endif
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// A version of moveTo(SkeletonPtr, BodyNode*) that also changes the Joint
  /// type of the parent Joint of this BodyNode. This function returns the
  /// pointer to the newly created Joint. The original Joint will be deleted.
  template <class JointType>
  JointType* moveTo(const SkeletonPtr& _newSkeleton, BodyNode* _newParent,
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>());
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties());
#endif
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// Remove this BodyNode and all of its children (recursively) from their
  /// current Skeleton and move them into a newly created Skeleton. The newly
  /// created Skeleton will have the same Skeleton::Properties as the current
  /// Skeleton, except it will use the specified name. The return value is a
  /// shared_ptr to the newly created Skeleton.
  ///
  /// Note that the parent Joint of this BodyNode will remain the same. If you
  /// want to change the Joint type of this BodyNode's parent Joint (for
  /// example, make it a FreeJoint), then use the templated split<JointType>()
  /// function.
  SkeletonPtr split(const std::string& _skeletonName);

  /// A version of split(const std::string&) that also changes the Joint type of
  /// the parent Joint of this BodyNode.
  template <class JointType>
  SkeletonPtr split(const std::string& _skeletonName,
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>());
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties());
#endif
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// Change the Joint type of this BodyNode's parent Joint.
  ///
  /// Note that this function will change the indexing of (potentially) all
  /// BodyNodes and Joints in the Skeleton.
  template <class JointType>
  JointType* changeParentJointType(
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>());
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties());
#endif
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// Create clones of this BodyNode and all of its children recursively (unless
  /// _recursive is set to false) and attach the clones to the specified
  /// BodyNode. The specified BodyNode can be in this Skeleton or a different
  /// Skeleton. Passing in nullptr will set the copy as a root node of the
  /// current Skeleton.
  ///
  /// The return value is a pair of pointers to the root of the newly created
  /// BodyNode tree.
  std::pair<Joint*, BodyNode*> copyTo(BodyNode* _newParent,
                                      bool _recursive=true);

  /// Create clones of this BodyNode and all of its children recursively (unless
  /// recursive is set to false) and attach the clones to the specified BodyNode
  /// of the specified Skeleton.
  ///
  /// The key differences between this function and the copyTo(BodyNode*)
  /// version is that this one allows the copied BodyNode to be const and allows
  /// you to copy it as a root node of another Skeleton.
  ///
  /// The return value is a pair of pointers to the root of the newly created
  /// BodyNode tree.
  std::pair<Joint*, BodyNode*> copyTo(const SkeletonPtr& _newSkeleton,
                                      BodyNode* _newParent,
                                      bool _recursive=true) const;

  /// A version of copyTo(BodyNode*) that also changes the Joint type of the
  /// parent Joint of this BodyNode.
  template <class JointType>
  std::pair<JointType*, BodyNode*> copyTo(
      BodyNode* _newParent,
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>(),
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties(),
#endif
      bool _recursive = true);
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// A version of copyTo(Skeleton*,BodyNode*) that also changes the Joint type
  /// of the parent Joint of this BodyNode.
  template <class JointType>
  std::pair<JointType*, BodyNode*> copyTo(
      const SkeletonPtr& _newSkeleton, BodyNode* _newParent,
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>(),
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties(),
#endif
      bool _recursive = true) const;
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// Create clones of this BodyNode and all of its children (recursively) and
  /// create a new Skeleton with the specified name to attach them to. The
  /// Skeleton::Properties of the current Skeleton will also be copied into the
  /// new Skeleton that gets created.
  SkeletonPtr copyAs(const std::string& _skeletonName,
                     bool _recursive=true) const;

  /// A version of copyAs(const std::string&) that also changes the Joint type
  /// of the root BodyNode.
  template <class JointType>
  SkeletonPtr copyAs(
      const std::string& _skeletonName,
#ifdef _WIN32
      const typename JointType::Properties& _joint
          = BodyNode::createJointProperties<JointType>(),
#else
      const typename JointType::Properties& _joint
          = typename JointType::Properties(),
#endif
      bool _recursive=true) const;
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  // Documentation inherited
  SkeletonPtr getSkeleton() override;

  // Documentation inherited
  ConstSkeletonPtr getSkeleton() const override;

  /// Return the parent Joint of this BodyNode
  Joint* getParentJoint();

  /// Return the (const) parent Joint of this BodyNode
  const Joint* getParentJoint() const;

  /// Return the parent BodyNdoe of this BodyNode
  BodyNode* getParentBodyNode();

  /// Return the (const) parent BodyNode of this BodyNode
  const BodyNode* getParentBodyNode() const;

  /// Create a Joint and BodyNode pair as a child of this BodyNode
  template <class JointType, class NodeType = BodyNode>
  std::pair<JointType*, NodeType*> createChildJointAndBodyNodePair(
#ifdef _WIN32
      const typename JointType::Properties& _jointProperties
          = BodyNode::createJointProperties<JointType>(),
      const typename NodeType::Properties& _bodyProperties
          = BodyNode::createBodyNodeProperties<NodeType>());
#else
      const typename JointType::Properties& _jointProperties
          = typename JointType::Properties(),
      const typename NodeType::Properties& _bodyProperties
          = typename NodeType::Properties());
#endif
  // TODO: Workaround for MSVC bug on template function specialization with
  // default argument. Please see #487 for detail

  /// Return the number of child BodyNodes
  size_t getNumChildBodyNodes() const;

  /// Return the _index-th child BodyNode of this BodyNode
  BodyNode* getChildBodyNode(size_t _index);

  /// Return the (const) _index-th child BodyNode of this BodyNode
  const BodyNode* getChildBodyNode(size_t _index) const;

  /// Return the number of child Joints
  size_t getNumChildJoints() const;

  /// Return the _index-th child Joint of this BodyNode
  Joint* getChildJoint(size_t _index);

  /// Return the (const) _index-th child Joint of this BodyNode
  const Joint* getChildJoint(size_t _index) const;

  /// Return the number of EndEffectors attached to this BodyNode
  size_t getNumEndEffectors() const;

  /// Return an EndEffector attached to this BodyNode
  EndEffector* getEndEffector(size_t _index);

  /// Return an EndEffector attached to this BodyNode
  const EndEffector* getEndEffector(size_t _index) const;

  /// Create an EndEffector attached to this BodyNode
  EndEffector* createEndEffector(const EndEffector::Properties& _properties);

  /// Create an EndEffector with the specified name
  EndEffector* createEndEffector(const std::string& _name = "EndEffector");

  /// Add a marker into the bodynode
  void addMarker(Marker* _marker);

  /// Return the number of markers of the bodynode
  size_t getNumMarkers() const;

  /// Return _index-th marker of the bodynode
  Marker* getMarker(size_t _index);

  /// Return (const) _index-th marker of the bodynode
  const Marker* getMarker(size_t _index) const;

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

  //--------------------------------------------------------------------------
  // Properties updated by dynamics (kinematics)
  //--------------------------------------------------------------------------

  /// Get the transform of this BodyNode with respect to its parent BodyNode,
  /// which is also its parent Frame.
  const Eigen::Isometry3d& getRelativeTransform() const override;

  // Documentation inherited
  const Eigen::Vector6d& getRelativeSpatialVelocity() const override;

  // Documentation inherited
  const Eigen::Vector6d& getRelativeSpatialAcceleration() const override;

  // Documentation inherited
  const Eigen::Vector6d& getPrimaryRelativeAcceleration() const override;

  /// Return the partial acceleration of this body
  const Eigen::Vector6d& getPartialAcceleration() const override;

  /// Return the generalized Jacobian targeting the origin of this BodyNode. The
  /// Jacobian is expressed in the Frame of this BodyNode.
  const math::Jacobian& getJacobian() const override final;

  // Prevent the inherited getJacobian functions from being shadowed
  using TemplatedJacobianNode<BodyNode>::getJacobian;

  /// Return the generalized Jacobian targeting the origin of this BodyNode. The
  /// Jacobian is expressed in the World Frame.
  const math::Jacobian& getWorldJacobian() const override final;

  // Prevent the inherited getWorldJacobian functions from being shadowed
  using TemplatedJacobianNode<BodyNode>::getWorldJacobian;

  /// Return the spatial time derivative of the generalized Jacobian targeting
  /// the origin of this BodyNode. The Jacobian is expressed in this BodyNode's
  /// coordinate Frame.
  ///
  /// NOTE: Since this is a spatial time derivative, it should be used with
  /// spatial vectors. If you are using classical linear and angular
  /// acceleration vectors, then use getJacobianClassicDeriv(),
  /// getLinearJacobianDeriv(), or getAngularJacobianDeriv() instead.
  const math::Jacobian& getJacobianSpatialDeriv() const override final;

  // Prevent the inherited getJacobianSpatialDeriv functions from being shadowed
  using TemplatedJacobianNode<BodyNode>::getJacobianSpatialDeriv;

  /// Return the classical time derivative of the generalized Jacobian targeting
  /// the origin of this BodyNode. The Jacobian is expressed in the World
  /// coordinate Frame.
  ///
  /// NOTE: Since this is a classical time derivative, it should be used with
  /// classical linear and angular vectors. If you are using spatial vectors,
  /// use getJacobianSpatialDeriv() instead.
  const math::Jacobian& getJacobianClassicDeriv() const override final;

  // Prevent the inherited getJacobianClassicDeriv functions from being shadowed
  using TemplatedJacobianNode<BodyNode>::getJacobianClassicDeriv;

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

  /// Clear out the generalized forces of the parent Joint and any other forces
  /// related to this BodyNode that are internal to the Skeleton. For example,
  /// the point mass forces for SoftBodyNodes.
  virtual void clearInternalForces();

  ///
  const Eigen::Vector6d& getExternalForceLocal() const;

  ///
  Eigen::Vector6d getExternalForceGlobal() const;

  /// Get spatial body force transmitted from the parent joint.
  ///
  /// The spatial body force is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  const Eigen::Vector6d& getBodyForce() const;

  //----------------------------------------------------------------------------
  // Constraints
  //   - Following functions are managed by constraint solver.
  //----------------------------------------------------------------------------

  /// Return true if the body can react to force or constraint impulse.
  ///
  /// A body node is reactive if the skeleton is mobile and the number of
  /// dependent generalized coordinates is non zero.
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

  /// Clear constraint impulses and cache data used for impulse-based forward
  /// dynamics algorithm
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

  //----------------------------------------------------------------------------
  // Rendering
  //----------------------------------------------------------------------------

  /// Render the markers
  void drawMarkers(renderer::RenderInterface* _ri = nullptr,
                   const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                   bool _useDefaultColor = true) const;

  //----------------------------------------------------------------------------
  // Notifications
  //----------------------------------------------------------------------------

  // Documentation inherited
  void notifyTransformUpdate() override;

  // Documentation inherited
  void notifyVelocityUpdate() override;

  // Documentation inherited
  void notifyAccelerationUpdate() override;

  /// Notify the Skeleton that the tree of this BodyNode needs an articulated
  /// inertia update
  void notifyArticulatedInertiaUpdate();

  /// Tell the Skeleton that the external forces need to be updated
  void notifyExternalForcesUpdate();

  /// Tell the Skeleton that the coriolis forces need to be update
  void notifyCoriolisUpdate();

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class Skeleton;
  friend class Joint;
  friend class EndEffector;
  friend class SoftBodyNode;
  friend class PointMass;
  friend class Node;

protected:

  /// Constructor called by Skeleton class
  BodyNode(BodyNode* _parentBodyNode, Joint* _parentJoint,
           const Properties& _properties);

  /// Create a clone of this BodyNode. This may only be called by the Skeleton
  /// class.
  virtual BodyNode* clone(BodyNode* _parentBodyNode, Joint* _parentJoint) const;

  /// Initialize the vector members with proper sizes.
  virtual void init(const SkeletonPtr& _skeleton);

  /// Add a child bodynode into the bodynode
  void addChildBodyNode(BodyNode* _body);

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  /// Separate generic child Entities from child BodyNodes for more efficient
  /// update notices
  void processNewEntity(Entity* _newChildEntity) override;

  /// Remove this Entity from mChildBodyNodes or mNonBodyNodeEntities
  void processRemovedEntity(Entity* _oldChildEntity) override;

  /// Update transformation
  virtual void updateTransform();

  /// Update spatial body velocity.
  virtual void updateVelocity();

  /// Update partial spatial body acceleration due to parent joint's velocity.
  virtual void updatePartialAcceleration() const;

  /// Update articulated body inertia for forward dynamics.
  /// \param[in] _timeStep Rquired for implicit joint stiffness and damping.
  virtual void updateArtInertia(double _timeStep) const;

  /// Update bias force associated with the articulated body inertia for forward
  /// dynamics.
  /// \param[in] _gravity Vector of gravitational acceleration
  /// \param[in] _timeStep Rquired for implicit joint stiffness and damping.
  virtual void updateBiasForce(const Eigen::Vector3d& _gravity,
                               double _timeStep);

  /// Update bias impulse associated with the articulated body inertia for
  /// impulse-based forward dynamics.
  virtual void updateBiasImpulse();

  /// Update spatial body acceleration with the partial spatial body
  /// acceleration for inverse dynamics.
  virtual void updateAccelerationID();

  /// Update spatial body acceleration for forward dynamics.
  virtual void updateAccelerationFD();

  /// Update spatical body velocity change for impluse-based forward dynamics.
  virtual void updateVelocityChangeFD();

  /// Update spatial body force for inverse dynamics.
  ///
  /// The spatial body force is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  virtual void updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                        bool _withExternalForces = false);

  /// Update spatial body force for forward dynamics.
  ///
  /// The spatial body force is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  virtual void updateTransmittedForceFD();

  /// Update spatial body force for impulse-based forward dynamics.
  ///
  /// The spatial body impulse is transmitted to this BodyNode from the parent
  /// body through the connecting joint. It is expressed in this BodyNode's
  /// frame.
  virtual void updateTransmittedImpulse();
  // TODO: Rename to updateTransmittedImpulseFD if impulse-based inverse
  // dynamics is implemented.

  /// Update the joint force for inverse dynamics.
  virtual void updateJointForceID(double _timeStep,
                                  double _withDampingForces,
                                  double _withSpringForces);

  /// Update the joint force for forward dynamics.
  virtual void updateJointForceFD(double _timeStep,
                                  double _withDampingForces,
                                  double _withSpringForces);

  /// Update the joint impulse for forward dynamics.
  virtual void updateJointImpulseFD();

  /// Update constrained terms due to the constraint impulses for foward
  /// dynamics.
  virtual void updateConstrainedTerms(double _timeStep);

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of motion related routines
  //----------------------------------------------------------------------------

  ///
  virtual void updateMassMatrix();
  virtual void aggregateMassMatrix(Eigen::MatrixXd& _MCol, size_t _col);
  virtual void aggregateAugMassMatrix(Eigen::MatrixXd& _MCol, size_t _col,
                                      double _timeStep);

  ///
  virtual void updateInvMassMatrix();
  virtual void updateInvAugMassMatrix();
  virtual void aggregateInvMassMatrix(Eigen::MatrixXd& _InvMCol, size_t _col);
  virtual void aggregateInvAugMassMatrix(Eigen::MatrixXd& _InvMCol, size_t _col,
                                         double _timeStep);

  ///
  virtual void aggregateCoriolisForceVector(Eigen::VectorXd& _C);

  ///
  virtual void aggregateGravityForceVector(Eigen::VectorXd& _g,
                                           const Eigen::Vector3d& _gravity);

  ///
  virtual void updateCombinedVector();
  virtual void aggregateCombinedVector(Eigen::VectorXd& _Cg,
                                       const Eigen::Vector3d& _gravity);

  /// Aggregate the external forces mFext in the generalized coordinates
  /// recursively
  virtual void aggregateExternalForces(Eigen::VectorXd& _Fext);

  ///
  virtual void aggregateSpatialToGeneralized(Eigen::VectorXd& _generalized,
                                             const Eigen::Vector6d& _spatial);

  /// Update body Jacobian. getJacobian() calls this function if
  /// mIsBodyJacobianDirty is true.
  void updateBodyJacobian() const;

  /// Update the World Jacobian. The commonality of using the World Jacobian
  /// makes it worth caching.
  void updateWorldJacobian() const;

  /// Update spatial time derivative of body Jacobian.
  /// getJacobianSpatialTimeDeriv() calls this function if
  /// mIsBodyJacobianSpatialDerivDirty is true.
  void updateBodyJacobianSpatialDeriv() const;

  /// Update classic time derivative of body Jacobian.
  /// getJacobianClassicDeriv() calls this function if
  /// mIsWorldJacobianClassicDerivDirty is true.
  void updateWorldJacobianClassicDeriv() const;

  /// \}

protected:

  //--------------------------------------------------------------------------
  // General properties
  //--------------------------------------------------------------------------

  /// A unique ID of this node globally.
  int mID;

  /// Counts the number of nodes globally.
  static size_t msBodyNodeCount;

  /// BodyNode-specific properties
  UniqueProperties mBodyP;

  /// Whether the node is currently in collision with another node.
  bool mIsColliding;

  //--------------------------------------------------------------------------
  // Structural Properties
  //--------------------------------------------------------------------------

  /// Index of this BodyNode in its Skeleton
  size_t mIndexInSkeleton;

  /// Index of this BodyNode in its Tree
  size_t mIndexInTree;

  /// Index of this BodyNode's tree
  size_t mTreeIndex;

  /// Parent joint
  Joint* mParentJoint;

  /// Parent body node
  BodyNode* mParentBodyNode;

  /// Array of child body nodes
  std::vector<BodyNode*> mChildBodyNodes;

  /// Array of child Entities that are not BodyNodes. Organizing them separately
  /// allows some performance optimizations.
  std::set<Entity*> mNonBodyNodeEntities;

  /// List of EndEffectors that are attached to this BodyNode
  std::vector<EndEffector*> mEndEffectors;

  /// List of markers associated
  std::vector<Marker*> mMarkers;

  /// Map that retrieves the destructors for a given Node
  std::unordered_map<const Node*, std::shared_ptr<NodeDestructor>> mNodeMap;

  /// A increasingly sorted list of dependent dof indices.
  std::vector<size_t> mDependentGenCoordIndices;

  /// A version of mDependentGenCoordIndices that holds DegreeOfFreedom pointers
  /// instead of indices
  std::vector<DegreeOfFreedom*> mDependentDofs;

  /// Same as mDependentDofs, but holds const pointers
  std::vector<const DegreeOfFreedom*> mConstDependentDofs;

  //--------------------------------------------------------------------------
  // Dynamical Properties
  //--------------------------------------------------------------------------

  /// Body Jacobian
  ///
  /// Do not use directly! Use getJacobian() to access this quantity
  mutable math::Jacobian mBodyJacobian;

  /// Cached World Jacobian
  ///
  /// Do not use directly! Use getJacobian() to access this quantity
  mutable math::Jacobian mWorldJacobian;

  /// Spatial time derivative of body Jacobian.
  ///
  /// Do not use directly! Use getJacobianSpatialDeriv() to access this quantity
  mutable math::Jacobian mBodyJacobianSpatialDeriv;

  /// Classic time derivative of Body Jacobian
  ///
  /// Do not use directly! Use getJacobianClassicDeriv() to access this quantity
  mutable math::Jacobian mWorldJacobianClassicDeriv;

  /// Partial spatial body acceleration due to parent joint's velocity
  ///
  /// Do not use directly! Use getPartialAcceleration() to access this quantity
  mutable Eigen::Vector6d mPartialAcceleration;
  // TODO(JS): Rename with more informative name

  /// Is the partial acceleration vector dirty
  mutable bool mIsPartialAccelerationDirty;

  /// Transmitted wrench from parent to the bodynode expressed in body-fixed
  /// frame
  Eigen::Vector6d mF;

  /// External spatial force
  Eigen::Vector6d mFext;

  /// Spatial gravity force
  Eigen::Vector6d mFgravity;

  /// Articulated body inertia
  ///
  /// Do not use directly! Use getArticulatedInertia() to access this quantity
  mutable math::Inertia mArtInertia;

  /// Articulated body inertia for implicit joint damping and spring forces
  ///
  /// DO not use directly! Use getArticulatedInertiaImplicit() to access this
  mutable math::Inertia mArtInertiaImplicit;

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

  // TODO(JS): rename with more informative one
  /// Generalized impulsive body force w.r.t. body frame.
  Eigen::Vector6d mImpF;

  /// Collision shape added signal
  ColShapeAddedSignal mColShapeAddedSignal;

  /// Collision shape removed signal
  ColShapeRemovedSignal mColShapeRemovedSignal;

  /// Structural change signal
  StructuralChangeSignal mStructuralChangeSignal;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //----------------------------------------------------------------------------
  /// \{ \name Slot registers
  //----------------------------------------------------------------------------

  /// Slot register for collision shape added signal
  common::SlotRegister<ColShapeAddedSignal> onColShapeAdded;

  /// Slot register for collision shape removed signal
  common::SlotRegister<ColShapeRemovedSignal> onColShapeRemoved;

  /// Raised when (1) parent BodyNode is changed, (2) moved between Skeletons,
  /// (3) parent Joint is changed
  mutable common::SlotRegister<StructuralChangeSignal> onStructuralChange;

  /// \}

private:

  /// Hold onto a reference to this BodyNode's own Destructor to make sure that
  /// it never gets destroyed.
  std::shared_ptr<NodeDestructor> mSelfDestructor;

};

#include "dart/dynamics/detail/BodyNode.h"

}  // namespace dynamics
}  // namespace kido

#endif  // KIDO_DYNAMICS_BODYNODE_H_
