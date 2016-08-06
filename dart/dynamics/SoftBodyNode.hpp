/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_SOFTBODYNODE_HPP_
#define DART_DYNAMICS_SOFTBODYNODE_HPP_

#include "dart/dynamics/detail/SoftBodyNodeAspect.hpp"

namespace dart {
namespace dynamics {

/// SoftBodyNode represent a soft body that has one deformable skin
///
/// This class is implementation of Sumit Jain and C. Karen Liu's paper:
/// http://www.cc.gatech.edu/graphics/projects/Sumit/homepage/projects/softcontacts/index.html
class SoftBodyNode : public detail::SoftBodyNodeBase
{
public:

  using UniqueProperties = detail::SoftBodyNodeUniqueProperties;
  using Properties = detail::SoftBodyNodeProperties;
  using Base = detail::SoftBodyNodeBase;

  friend class Skeleton;
  friend class PointMass;
  friend class PointMassNotifier;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, SoftBodyAspect)

  /// \brief
  virtual ~SoftBodyNode();

  // Documentation inherited
  SoftBodyNode* asSoftBodyNode() override;

  // Documentation inherited
  const SoftBodyNode* asSoftBodyNode() const override;

  /// Set the Properties of this SoftBodyNode
  void setProperties(const Properties& _properties);

  /// Set the Properties of this SoftBodyNode
  void setProperties(const UniqueProperties& _properties);

  /// Set the AspectState of this SoftBodyNode
  void setAspectState(const AspectState& state);

  /// Set the AspectProperties of this SoftBodyNode
  void setAspectProperties(const AspectProperties& properties);

  using SkeletonRefCountingBase::getSkeleton;

  /// Get the Properties of this SoftBodyNode
  Properties getSoftBodyNodeProperties() const;

  /// Copy the Properties of another SoftBodyNode
  void copy(const SoftBodyNode& _otherSoftBodyNode);

  /// Copy the Properties of another SoftBodyNode
  void copy(const SoftBodyNode* _otherSoftBodyNode);

  /// Copy the Properties of another SoftBodyNode
  SoftBodyNode& operator=(const SoftBodyNode& _otherSoftBodyNode);

  /// Get the update notifier for the PointMasses of this SoftBodyNode
  PointMassNotifier* getNotifier();

  /// Get the update notifier for the PointMasses of this SoftBodyNode
  const PointMassNotifier* getNotifier() const;

  /// \brief Get mass.
  double getMass() const;

  /// \brief
  void setVertexSpringStiffness(double _kv);

  /// \brief
  double getVertexSpringStiffness() const;

  /// \brief
  void setEdgeSpringStiffness(double _ke);

  /// \brief
  double getEdgeSpringStiffness() const;

  /// \brief
  void setDampingCoefficient(double _damp);

  /// \brief
  double getDampingCoefficient() const;

  /// \brief
  void removeAllPointMasses();

  /// \brief
  PointMass* addPointMass(const PointMass::Properties& _properties);

  /// \brief
  std::size_t getNumPointMasses() const;

  /// \brief
  PointMass* getPointMass(std::size_t _idx);

  /// \brief
  const PointMass* getPointMass(std::size_t _idx) const;

  /// Return all the point masses in this SoftBodyNode
  const std::vector<PointMass*>& getPointMasses() const;

  /// \brief
  void connectPointMasses(std::size_t _idx1, std::size_t _idx2);

  /// \brief
  void addFace(const Eigen::Vector3i& _face);

  /// \brief
  const Eigen::Vector3i& getFace(std::size_t _idx) const;

  /// \brief
  std::size_t getNumFaces() const;

  // Documentation inherited.
  void clearConstraintImpulse() override;

protected:

  /// Constructor called by Skeleton class
  SoftBodyNode(BodyNode* _parentBodyNode, Joint* _parentJoint,
               const Properties& _properties);

  /// Create a clone of this SoftBodyNode. This may only be called by the
  /// Skeleton class.
  BodyNode* clone(BodyNode* _parentBodyNode, Joint* _parentJoint,
                  bool cloneNodes) const override;

  /// Used by SoftBodyAspect to have this SoftBodyNode reconstruct its
  /// SoftMeshShape
  void configurePointMasses(ShapeNode* softNode);

  //--------------------------------------------------------------------------
  // Sub-functions for Recursive Kinematics Algorithms
  //--------------------------------------------------------------------------
  // Documentation inherited.
  void init(const SkeletonPtr& _skeleton) override;

  // Documentation inherited.
//  virtual void aggregateGenCoords(std::vector<GenCoord*>* _genCoords);

  // Documentation inherited.
//  virtual void aggregatePointMassGenCoords(std::vector<GenCoord*>* _genCoords);

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  /// Update articulated inertia if necessary
  void checkArticulatedInertiaUpdate() const;

  // Documentation inherited.
  void updateTransform() override;

  // Documentation inherited.
  void updateVelocity() override;

  // Documentation inherited.
  void updatePartialAcceleration() const override;

  // Documentation inherited.
  void updateArtInertia(double _timeStep) const override;

  // Documentation inherited.
  void updateBiasForce(const Eigen::Vector3d& _gravity,
                       double _timeStep) override;

  // Documentation inherited.
  void updateBiasImpulse() override;

  // Documentation inherited.
  void updateAccelerationID() override;

  // Documentation inherited.
  void updateAccelerationFD() override;

  // Documentation inherited.
  void updateVelocityChangeFD() override;

  // Documentation inherited.
  void updateTransmittedForceID(
      const Eigen::Vector3d& _gravity,
      bool _withExternalForces = false) override;

  // Documentation inherited.
  void updateTransmittedForceFD() override;

  // Documentation inherited.
  void updateTransmittedImpulse() override;

  // Documentation inherited.
  void updateJointForceID(double _timeStep,
                          bool _withDampingForces,
                          bool _withSpringForces) override;

  // Documentation inherited.
  void updateJointForceFD(double _timeStep,
                          bool _withDampingForces,
                          bool _withSpringForces) override;

  // Documentation inherited.
  void updateJointImpulseFD() override;

  // Documentation inherited.
  void updateConstrainedTerms(double _timeStep) override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of motion related routines
  //----------------------------------------------------------------------------

  // Documentation inherited.
  void updateMassMatrix() override;

  // Documentation inherited.
  void aggregateMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col) override;

  // Documentation inherited.
  void aggregateAugMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col,
                              double _timeStep) override;

  // Documentation inherited.
  void updateInvMassMatrix() override;

  // Documentation inherited.
  void updateInvAugMassMatrix() override;

  // Documentation inherited.
  void aggregateInvMassMatrix(Eigen::MatrixXd& _InvMCol, std::size_t _col) override;

  // Documentation inherited.
  void aggregateInvAugMassMatrix(Eigen::MatrixXd& _InvMCol, std::size_t _col,
                                 double _timeStep) override;

  // Documentation inherited.
  // TODO(JS): Not implemented yet.
  void aggregateCoriolisForceVector(Eigen::VectorXd& _C) override;

  // Documentation inherited.
  void aggregateGravityForceVector(Eigen::VectorXd& _g,
                                   const Eigen::Vector3d& _gravity) override;

  // Documentation inherited.
  void updateCombinedVector() override;

  // Documentation inherited.
  void aggregateCombinedVector(Eigen::VectorXd& _Cg,
                               const Eigen::Vector3d& _gravity) override;

  // Documentation inherited.
  void aggregateExternalForces(Eigen::VectorXd& _Fext) override;

  /// \}

  // Documentation inherited.
  void clearExternalForces() override;

  void clearInternalForces() override;

protected:

  /// \brief List of point masses composing deformable mesh.
  std::vector<PointMass*> mPointMasses;

  /// An Entity which tracks when the point masses need to be updated
  PointMassNotifier* mNotifier;

  /// \brief Soft mesh shape belonging to this node.
  WeakShapeNodePtr mSoftShapeNode;

  /// Generalized inertia with point masses
  math::Inertia mI2;

  ///
  math::Inertia mArtInertia2;

  ///
  math::Inertia mArtInertiaImplicit2;

private:
  /// \brief
  void _addPiToArtInertia(const Eigen::Vector3d& _p, double _Pi) const;

  /// \brief
  void _addPiToArtInertiaImplicit(const Eigen::Vector3d& _p,
                                  double _ImplicitPi) const;

  ///
  void updateInertiaWithPointMass();
};

class SoftBodyNodeHelper
{
public:

  /// Create a Properties struct for a box-shaped SoftBodyNode with 8
  /// PointMasses
  static SoftBodyNode::UniqueProperties makeBoxProperties(
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setBox(SoftBodyNode*            _softBodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for a box-shaped SoftBodyNode. Specify the
  /// number of vertices along each axis with _frags. Each component should be
  /// equal to or greater than 3. For example, [3 3 3] is allowed but [2 2 2] is
  /// not.
  static SoftBodyNode::UniqueProperties makeBoxProperties(
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      const Eigen::Vector3i&   _frags,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  /// \param[in] _frags Number of vertices of box mesh. Each component should be
  ///   equal or greater than 3. For example, [3 3 3] is allowed but [2 2 2] is
  ///   not.
  // TODO: The component of _frags should allow 2.
  static void setBox(SoftBodyNode*            _softBodyNode,
      const Eigen::Vector3d&   _size,
      const Eigen::Isometry3d& _localTransform,
      const Eigen::Vector3i&   _frags,
      double                   _totalMass,
      double                   _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                   _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                   _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for a SoftBodyNode with a single PointMass
  static SoftBodyNode::UniqueProperties makeSinglePointMassProperties(
      double _totalMass,
      double _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setSinglePointMass(
      SoftBodyNode*          _softBodyNode,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for an Sphere-shaped SoftBodyNode
  static SoftBodyNode::UniqueProperties makeSphereProperties(
      double                 _radius,
      std::size_t            _nSlices,
      std::size_t            _nStacks,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for an Ellipsoid-shaped SoftBodyNode
  static SoftBodyNode::UniqueProperties makeEllipsoidProperties(
      const Eigen::Vector3d& _size,
      std::size_t                 _nSlices,
      std::size_t                 _nStacks,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// \brief
  /// This should be called before SoftBodyNode::init() is called
  static void setEllipsoid(SoftBodyNode*          _softBodyNode,
      const Eigen::Vector3d& _size,
      std::size_t _nSlices,
      std::size_t _nStacks,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  /// Create a Properties struct for a cylinder-shaped SoftBodyNode
  static SoftBodyNode::UniqueProperties makeCylinderProperties(
      double _radius,
      double _height,
      std::size_t _nSlices,
      std::size_t _nStacks,
      std::size_t _nRings,
      double _totalMass,
      double _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);

  ///
  /// This should be called before SoftBodyNode::init() is called
  static void setCylinder(SoftBodyNode*          _softBodyNode,
      double _radius,
      double _height,
      std::size_t _nSlices,
      std::size_t _nStacks,
      std::size_t _nRings,
      double                 _totalMass,
      double                 _vertexStiffness = DART_DEFAULT_VERTEX_STIFFNESS,
      double                 _edgeStiffness   = DART_DEFAULT_EDGE_STIFNESS,
      double                 _dampingCoeff    = DART_DEFAULT_DAMPING_COEFF);
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SOFTBODYNODE_HPP_
