/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_BODYNODE_HPP_
#define DART_DYNAMICS_BODYNODE_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "dart/config.hpp"
#include "dart/common/Deprecated.hpp"
#include "dart/common/Signal.hpp"
#include "dart/common/EmbeddedAspect.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/dynamics/Node.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/SmartPointer.hpp"
#include "dart/dynamics/TemplatedJacobianNode.hpp"
#include "dart/dynamics/SpecializedNodeManager.hpp"
#include "dart/dynamics/detail/BodyNodeAspect.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

class GenCoord;
class Skeleton;
class Joint;
class DegreeOfFreedom;
class Shape;
class EndEffector;
class Marker;

/// BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
///
/// BodyNode inherits Frame, and a parent Frame of a BodyNode is the parent
/// BodyNode of the BodyNode.
class BodyNode :
    public detail::BodyNodeCompositeBase,
    public virtual BodyNodeSpecializedFor<ShapeNode, EndEffector, Marker>,
    public SkeletonRefCountingBase,
    public TemplatedJacobianNode<BodyNode>
{
public:

  using ColShapeAddedSignal
      = common::Signal<void(const BodyNode*, ConstShapePtr _newColShape)>;

  using ColShapeRemovedSignal = ColShapeAddedSignal;

  using StructuralChangeSignal
      = common::Signal<void(const BodyNode*)>;
  using CompositeProperties = common::Composite::Properties;

  using AllNodeStates = detail::AllNodeStates;
  using NodeStateMap = detail::NodeStateMap;

  using AllNodeProperties = detail::AllNodeProperties;
  using NodePropertiesMap = detail::NodePropertiesMap;

  using AspectProperties = detail::BodyNodeAspectProperties;
  using Properties = common::Composite::MakeProperties<BodyNode>;

  BodyNode(const BodyNode&) = delete;

  /// Destructor
  virtual ~BodyNode();

  /// Convert 'this' into a SoftBodyNode pointer if this BodyNode is a
  /// SoftBodyNode, otherwise return nullptr
  virtual SoftBodyNode* asSoftBodyNode();

  /// Convert 'const this' into a SoftBodyNode pointer if this BodyNode is a
  /// SoftBodyNode, otherwise return nullptr
  virtual const SoftBodyNode* asSoftBodyNode() const;

  /// Set the Node::State of all Nodes attached to this BodyNode
  void setAllNodeStates(const AllNodeStates& states);

  /// Get the Node::State of all Nodes attached to this BodyNode
  AllNodeStates getAllNodeStates() const;

  /// Set the Node::Properties of all Nodes attached to this BodyNode
  void setAllNodeProperties(const AllNodeProperties& properties);

  /// Get the Node::Properties of all Nodes attached to this BodyNode
  AllNodeProperties getAllNodeProperties() const;

  /// Same as setCompositeProperties()
  void setProperties(const CompositeProperties& _properties);

  /// Set the UniqueProperties of this BodyNode
  void setProperties(const AspectProperties& _properties);

  /// Set the AspectState of this BodyNode
  void setAspectState(const AspectState& state);

  /// Set the AspectProperties of this BodyNode
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this BodyNode
  Properties getBodyNodeProperties() const;

  /// Copy the Properties of another BodyNode
  void copy(const BodyNode& otherBodyNode);

  /// Copy the Properties of another BodyNode
  void copy(const BodyNode* otherBodyNode);

  /// Same as copy(const BodyNode&)
  BodyNode& operator=(const BodyNode& _otherBodyNode);

  /// Give this BodyNode a copy of each Node from otherBodyNode
  void duplicateNodes(const BodyNode* otherBodyNode);

  /// Make the Nodes of this BodyNode match the Nodes of otherBodyNode. All
  /// existing Nodes in this BodyNode will be removed.
  void matchNodes(const BodyNode* otherBodyNode);

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name) override;

  // Documentation inherited
  const std::string& getName() const override;

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

  /// Return the index of this BodyNode within its Skeleton
  std::size_t getIndexInSkeleton() const;

  /// Return the index of this BodyNode within its tree
  std::size_t getIndexInTree() const;

  /// Return the index of the tree that this BodyNode belongs to
  std::size_t getTreeIndex() const;

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
  std::size_t getNumChildBodyNodes() const;

  /// Return the _index-th child BodyNode of this BodyNode
  BodyNode* getChildBodyNode(std::size_t _index);

  /// Return the (const) _index-th child BodyNode of this BodyNode
  const BodyNode* getChildBodyNode(std::size_t _index) const;

  /// Return the number of child Joints
  std::size_t getNumChildJoints() const;

  /// Return the _index-th child Joint of this BodyNode
  Joint* getChildJoint(std::size_t _index);

  /// Return the (const) _index-th child Joint of this BodyNode
  const Joint* getChildJoint(std::size_t _index) const;

  /// Create some Node type and attach it to this BodyNode.
  template <class NodeType, typename ...Args>
  NodeType* createNode(Args&&... args);

  DART_BAKE_SPECIALIZED_NODE_DECLARATIONS( ShapeNode )

  /// Create an ShapeNode attached to this BodyNode. Pass a
  /// ShapeNode::Properties argument into its constructor. If automaticName is
  /// true, then the mName field of properties will be ignored, and the
  /// ShapeNode will be automatically assigned a name:
  /// <BodyNodeName>_ShapeNode_<#>
  template <class ShapeNodeProperties>
  ShapeNode* createShapeNode(ShapeNodeProperties properties,
                             bool automaticName = true);

  /// Create a ShapeNode with an automatically assigned name:
  /// <BodyNodeName>_ShapeNode_<#>.
  ShapeNode* createShapeNode(const ShapePtr& shape);

  /// Create an ShapeNode with the specified name
  ShapeNode* createShapeNode(
      const ShapePtr& shape, const std::string& name);

  /// Create an ShapeNode with the specified name
  ShapeNode* createShapeNode(const ShapePtr& shape, const char* name);

  /// Return the list of ShapeNodes
  const std::vector<ShapeNode*> getShapeNodes();

  /// Return the list of (const) ShapeNodes
  const std::vector<const ShapeNode*> getShapeNodes() const;

  /// Remove all ShapeNodes from this BodyNode
  void removeAllShapeNodes();

  /// Create a ShapeNode with the specified Aspects and an automatically assigned
  /// name: <BodyNodeName>_ShapeNode_<#>.
  template <class... Aspects>
  ShapeNode* createShapeNodeWith(const ShapePtr& shape);

  /// Create a ShapeNode with the specified name and Aspects
  template <class... Aspects>
  ShapeNode* createShapeNodeWith(const ShapePtr& shape,
                                 const std::string& name);

  /// Return the number of ShapeNodes containing given Aspect in this BodyNode
  template <class Aspect>
  std::size_t getNumShapeNodesWith() const;

  /// Return the list of ShapeNodes containing given Aspect
  template <class Aspect>
  const std::vector<ShapeNode*> getShapeNodesWith();

  /// Return the list of ShapeNodes containing given Aspect
  template <class Aspect>
  const std::vector<const ShapeNode*> getShapeNodesWith() const;

  /// Remove all ShapeNodes containing given Aspect from this BodyNode
  template <class Aspect>
  void removeAllShapeNodesWith();

  DART_BAKE_SPECIALIZED_NODE_DECLARATIONS( EndEffector )

  /// Create an EndEffector attached to this BodyNode. Pass an
  /// EndEffector::Properties argument into this function.
  EndEffector* createEndEffector(
      const EndEffector::BasicProperties& _properties);

  /// Create an EndEffector with the specified name
  EndEffector* createEndEffector(const std::string& _name = "EndEffector");

  /// Create an EndEffector with the specified name
  EndEffector* createEndEffector(const char* _name);

  DART_BAKE_SPECIALIZED_NODE_DECLARATIONS( Marker )

  /// Create a Marker with the given fields
  Marker* createMarker(
      const std::string& name = "marker",
      const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
      const Eigen::Vector4d& color = Eigen::Vector4d::Constant(1.0));

  /// Create a Marker given its basic properties
  Marker* createMarker(const Marker::BasicProperties& properties);

  // Documentation inherited
  bool dependsOn(std::size_t _genCoordIndex) const override;

  // Documentation inherited
  std::size_t getNumDependentGenCoords() const override;

  // Documentation inherited
  std::size_t getDependentGenCoordIndex(std::size_t _arrayIndex) const override;

  // Documentation inherited
  const std::vector<std::size_t>& getDependentGenCoordIndices() const override;

  // Documentation inherited
  std::size_t getNumDependentDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDependentDof(std::size_t _index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDependentDof(std::size_t _index) const override;

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

  /// Set whether this body node is colliding with other objects. Note that
  /// this status is set by the constraint solver during dynamics simulation but
  /// not by collision detector.
  /// \param[in] True if this body node is colliding.
  DART_DEPRECATED(6.0)
  void setColliding(bool _isColliding);

  /// Return whether this body node is set to be colliding with other objects.
  /// \return True if this body node is colliding.
  DART_DEPRECATED(6.0)
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

  /// Return Lagrangian of this body
  double computeLagrangian(const Eigen::Vector3d& gravity) const;

  /// Return kinetic energy.
  DART_DEPRECATED(6.1)
  virtual double getKineticEnergy() const;

  /// Return kinetic energy
  double computeKineticEnergy() const;

  /// Return potential energy.
  DART_DEPRECATED(6.1)
  virtual double getPotentialEnergy(const Eigen::Vector3d& _gravity) const;

  /// Return potential energy.
  double computePotentialEnergy(const Eigen::Vector3d& gravity) const;

  /// Return linear momentum.
  Eigen::Vector3d getLinearMomentum() const;

  /// Return angular momentum.
  Eigen::Vector3d getAngularMomentum(
      const Eigen::Vector3d& _pivot = Eigen::Vector3d::Zero());

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

  /// Delegating constructor
  BodyNode(const std::tuple<BodyNode*, Joint*, Properties>& args);

  /// Create a clone of this BodyNode. This may only be called by the Skeleton
  /// class.
  virtual BodyNode* clone(BodyNode* _parentBodyNode, Joint* _parentJoint,
                          bool cloneNodes) const;

  /// This is needed in order to inherit the Node class, but it does nothing
  Node* cloneNode(BodyNode* bn) const override final;

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
                                  bool _withDampingForces,
                                  bool _withSpringForces);

  /// Update the joint force for forward dynamics.
  virtual void updateJointForceFD(double _timeStep,
                                  bool _withDampingForces,
                                  bool _withSpringForces);

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
  virtual void aggregateMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col);
  virtual void aggregateAugMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col,
                                      double _timeStep);

  ///
  virtual void updateInvMassMatrix();
  virtual void updateInvAugMassMatrix();
  virtual void aggregateInvMassMatrix(Eigen::MatrixXd& _InvMCol, std::size_t _col);
  virtual void aggregateInvAugMassMatrix(Eigen::MatrixXd& _InvMCol, std::size_t _col,
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
  static std::size_t msBodyNodeCount;

  /// Whether the node is currently in collision with another node.
  /// \deprecated DART_DEPRECATED(6.0) See #670 for more detail.
  bool mIsColliding;

  //--------------------------------------------------------------------------
  // Structural Properties
  //--------------------------------------------------------------------------

  /// Index of this BodyNode in its Skeleton
  std::size_t mIndexInSkeleton;

  /// Index of this BodyNode in its Tree
  std::size_t mIndexInTree;

  /// Index of this BodyNode's tree
  std::size_t mTreeIndex;

  /// Parent joint
  Joint* mParentJoint;

  /// Parent body node
  BodyNode* mParentBodyNode;

  /// Array of child body nodes
  std::vector<BodyNode*> mChildBodyNodes;

  /// Array of child Entities that are not BodyNodes. Organizing them separately
  /// allows some performance optimizations.
  std::set<Entity*> mNonBodyNodeEntities;

  /// A increasingly sorted list of dependent dof indices.
  std::vector<std::size_t> mDependentGenCoordIndices;

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

}  // namespace dynamics
}  // namespace dart

#include "dart/dynamics/detail/BodyNode.hpp"

#endif  // DART_DYNAMICS_BODYNODE_HPP_
