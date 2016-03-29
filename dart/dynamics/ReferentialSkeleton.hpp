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

#ifndef DART_DYNAMICS_REFERENTIALSKELETON_HPP_
#define DART_DYNAMICS_REFERENTIALSKELETON_HPP_

#include <unordered_map>

#include "dart/dynamics/MetaSkeleton.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {

/// ReferentialSkeleton is a base class used to implement Linkage, Group, and
/// other classes that are used to reference subsections of Skeletons.
class ReferentialSkeleton : public MetaSkeleton
{
public:

  /// Remove copy operator
  /// TODO(MXG): Consider allowing this
  ReferentialSkeleton& operator=(const ReferentialSkeleton& _other) = delete;

  /// Default destructor
  virtual ~ReferentialSkeleton() = default;

  //----------------------------------------------------------------------------
  /// \{ \name Name
  //----------------------------------------------------------------------------

  // Documentation inherited
  const std::string& setName(const std::string& _name) override;

  // Documentation inherited
  const std::string& getName() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  // Documentation inherited
  size_t getNumBodyNodes() const override;

  // Documentation inherited
  BodyNode* getBodyNode(size_t _idx) override;

  // Documentation inherited
  const BodyNode* getBodyNode(size_t _idx) const override;

  // Documentation inherited
  const std::vector<BodyNode*>& getBodyNodes() override;

  // Documentation inherited
  const std::vector<const BodyNode*>& getBodyNodes() const override;

  // Documentation inherited
  size_t getIndexOf(const BodyNode* _bn, bool _warning=true) const override;

  // Documentation inherited
  size_t getNumJoints() const override;

  // Documentation inherited
  Joint* getJoint(size_t _idx) override;

  // Documentation inherited
  const Joint* getJoint(size_t _idx) const override;

  // Documentation inherited
  size_t getIndexOf(const Joint* _joint, bool _warning=true) const override;

  // Documentation inherited
  size_t getNumDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDof(size_t _idx) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(size_t _idx) const override;

  // Documentation inherited
  const std::vector<DegreeOfFreedom*>& getDofs() override;

  // Documentation inherited
  std::vector<const DegreeOfFreedom*> getDofs() const override;

  // Documentation inherited
  size_t getIndexOf(const DegreeOfFreedom* _dof,
                    bool _warning=true) const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobians
  //----------------------------------------------------------------------------

  // Documentation inherited
  math::Jacobian getJacobian(const JacobianNode* _node) const override;

  // Documentation inherited
  math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const override;

  // Documentation inherited
  math::Jacobian getJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getWorldJacobian(const JacobianNode* _node) const override;

  // Documentation inherited
  math::Jacobian getWorldJacobian(const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobian(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::AngularJacobian getAngularJacobian(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset) const override;

  // Documentation inherited
  math::Jacobian getJacobianSpatialDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node) const override;

  // Documentation inherited
  math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf) const override;

  // Documentation inherited
  math::Jacobian getJacobianClassicDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobianDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getLinearJacobianDeriv(
      const JacobianNode* _node,
      const Eigen::Vector3d& _localOffset,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::AngularJacobian getAngularJacobianDeriv(
      const JacobianNode* _node,
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Equations of Motion
  //----------------------------------------------------------------------------

  /// Get the total mass of all BodyNodes in this ReferentialSkeleton. For
  /// ReferentialSkeleton::getMass(), the total mass is computed upon request,
  /// so this is a linear-time O(N) operation. The Skeleton::getMass() version,
  /// however, is constant-time.
  double getMass() const override;

  // Documentation inherited
  const Eigen::MatrixXd& getMassMatrix() const override;

  // Documentation inherited
  const Eigen::MatrixXd& getAugMassMatrix() const override;

  // Documentation inherited
  const Eigen::MatrixXd& getInvMassMatrix() const override;

  // Documentation inherited
  const Eigen::MatrixXd& getInvAugMassMatrix() const override;

  // Documentation inherited
  const Eigen::VectorXd& getCoriolisForces() const override;

  // Documentation inherited
  const Eigen::VectorXd& getGravityForces() const override;

  // Documentation inherited
  const Eigen::VectorXd& getCoriolisAndGravityForces() const override;

  // Documentation inherited
  const Eigen::VectorXd& getExternalForces() const override;

  // Documentation inherited
  const Eigen::VectorXd& getConstraintForces() const override;

  // Documentation inherited
  void clearExternalForces() override;

  // Documentation inherited
  void clearInternalForces() override;

  // Documentation inherited
  double getKineticEnergy() const override;

  // Documentation inherited
  double getPotentialEnergy() const override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Center of Mass Jacobian
  //----------------------------------------------------------------------------

  // Documentation inherited
  Eigen::Vector3d getCOM(
      const Frame* _withRespectTo = Frame::World()) const override;

  // Documentation inherited
  Eigen::Vector6d getCOMSpatialVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  Eigen::Vector3d getCOMLinearVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  Eigen::Vector6d getCOMSpatialAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  Eigen::Vector3d getCOMLinearAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::Jacobian getCOMJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getCOMLinearJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::Jacobian getCOMJacobianSpatialDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  // Documentation inherited
  math::LinearJacobian getCOMLinearJacobianDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const override;

  /// \}

protected:

  /// Default constructor. Protected to avoid blank and useless instantiations
  /// of ReferentialSkeleton.
  ReferentialSkeleton() = default;

  /// Add a BodyNode, along with its parent Joint and parent DegreesOfFreedom
  /// to this ReferentialSkeleton. This can only be used by derived classes.
  void registerComponent(BodyNode* _bn);

  /// Add a BodyNode to this ReferentialSkeleton, ignoring its Joint and
  /// DegreesOfFreedom. This can only be used by derived classes.
  void registerBodyNode(BodyNode* _bn);

  /// Add a Joint to this Referential Skeleton, ignoring its DegreesOfFreedom.
  /// This can only be used by derived classes.
  void registerJoint(Joint* _joint);

  /// Add a DegreeOfFreedom to this ReferentialSkeleton. This can only be used
  /// by derived classes.
  void registerDegreeOfFreedom(DegreeOfFreedom* _dof);

  /// Completely remove a BodyNode and its parent DegreesOfFreedom from this
  /// ReferentialSkeleton. This can only be used by derived classes.
  void unregisterComponent(BodyNode* _bn);

  /// Remove a BodyNode from this ReferentialSkeleton, ignoring its parent
  /// DegreesOfFreedom. This can only be used by derived classes.
  void unregisterBodyNode(BodyNode* _bn, bool _unregisterDofs);

  /// Remove a Joint from this ReferentialSkeleton. This can only be used by
  /// derived classes.
  void unregisterJoint(BodyNode* _child);

  /// Remove a DegreeOfFreedom from this ReferentialSkeleton. This can only be
  /// used by derived classes.
  void unregisterDegreeOfFreedom(BodyNode* _bn, size_t _localIndex);

  /// Update the caches to match any changes to the structure of this
  /// ReferentialSkeleton
  void updateCaches();

  /// Weak pointer to this Skeleton
  std::weak_ptr<MetaSkeleton> mPtr;

  /// A simple struct that contains the indexing of a BodyNode and its parent
  /// DegreesOfFreedom
  struct IndexMap
  {
    /// Index of the BodyNode
    size_t mBodyNodeIndex;

    /// Index of the parent Joint
    size_t mJointIndex;

    /// Indices of the parent DegreesOfFreedom
    std::vector<size_t> mDofIndices;

    /// Default constructor. Initializes mBodyNodeIndex and mJointIndex to
    /// INVALID_INDEX
    IndexMap();

    /// Returns true if nothing in this entry is mapping to a valid index any
    /// longer.
    bool isExpired() const;
  };

  /// Name of this ReferentialSkeleton
  std::string mName;

  /// BodyNodes that this ReferentialSkeleton references. These hold strong
  /// references to ensure that the BodyNodes do not disappear
  std::vector<BodyNodePtr> mBodyNodes;

  /// Raw BodyNode pointers. This vector is used for the MetaSkeleton API
  mutable std::vector<BodyNode*> mRawBodyNodes;

  /// Raw const BodyNode pointers. This vector is used for the MetaSkeleton API
  mutable std::vector<const BodyNode*> mRawConstBodyNodes;

  /// Joints that this ReferentialSkeleton references
  std::vector<JointPtr> mJoints;

  /// DegreesOfFreedom that this ReferentialSkeleton references
  std::vector<DegreeOfFreedomPtr> mDofs;

  /// Raw DegreeOfFreedom vector. This vector is used for the MetaSkeleton API
  mutable std::vector<DegreeOfFreedom*> mRawDofs;

  /// Raw const DegreeOfFreedom vector. This vector is used for the MetaSkeleton
  /// API
  mutable std::vector<const DegreeOfFreedom*> mRawConstDofs;

  /// Raw const DegreeOfFreedom. This vector is used for the MetaSkeleton API

  /// Map for keeping track of the indices of BodyNodes, Joints, and
  /// DegreesOfFreedom
  std::unordered_map<const BodyNode*, IndexMap> mIndexMap;

  /// Cache for Mass Matrix
  mutable Eigen::MatrixXd mM;

  /// Cache for Augmented Mass Matrix
  mutable Eigen::MatrixXd mAugM;

  /// Cache for inverse Mass Matrix
  mutable Eigen::MatrixXd mInvM;

  /// Cache for inverse Augmented Mass Matrix
  mutable Eigen::MatrixXd mInvAugM;

  /// Cache for Coriolis vector
  mutable Eigen::VectorXd mCvec;

  /// Cache for gravity vector
  mutable Eigen::VectorXd mG;

  /// Cache for combined Coriolis and gravity vector
  mutable Eigen::VectorXd mCg;

  /// Cache for external force vector
  mutable Eigen::VectorXd mFext;

  /// Cache for constraint force vector
  mutable Eigen::VectorXd mFc;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_REFERENTIALSKELETON_HPP_
