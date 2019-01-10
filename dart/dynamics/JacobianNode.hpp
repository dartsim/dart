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

#ifndef DART_DYNAMICS_JACOBIANNODE_HPP_
#define DART_DYNAMICS_JACOBIANNODE_HPP_

#include <memory>
#include <unordered_set>

#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/Node.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {

class Skeleton;
class DegreeOfFreedom;
class InverseKinematics;

/// The JacobianNode class serves as a common interface for BodyNodes and
/// EndEffectors to both be used as references for IK modules. This is a pure
/// abstract class.
class JacobianNode : public virtual Frame, public Node
{
public:

  /// Virtual destructor
  virtual ~JacobianNode();

  using Node::setName;
  using Node::getName;

  /// Get a pointer to an IK module for this JacobianNode. If _createIfNull is
  /// true, then the IK module will be generated if one does not already exist.
  const std::shared_ptr<InverseKinematics>& getIK(bool _createIfNull = false);

  /// Get a pointer to an IK module for this JacobianNode. The IK module will be
  /// generated if one does not already exist. This function is actually the
  /// same as getIK(true).
  const std::shared_ptr<InverseKinematics>& getOrCreateIK();

  /// Get a pointer to an IK module for this JacobianNode. Because this is a
  /// const function, a new IK module cannot be created if one does not already
  /// exist.
  std::shared_ptr<const InverseKinematics> getIK() const;

  /// Create a new IK module for this JacobianNode. If an IK module already
  /// exists in this JacobianNode, it will be destroyed and replaced by a brand
  /// new one.
  const std::shared_ptr<InverseKinematics>& createIK();

  /// Wipe away the IK module for this JacobianNode, leaving it as a nullptr.
  void clearIK();

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Return true if _genCoordIndex-th generalized coordinate
  virtual bool dependsOn(std::size_t _genCoordIndex) const = 0;

  /// The number of the generalized coordinates which affect this JacobianNode
  virtual std::size_t getNumDependentGenCoords() const = 0;

  /// Return a generalized coordinate index from the array index
  /// (< getNumDependentDofs)
  virtual std::size_t getDependentGenCoordIndex(std::size_t _arrayIndex) const = 0;

  /// Indices of the generalized coordinates which affect this JacobianNode
  virtual const std::vector<std::size_t>& getDependentGenCoordIndices() const = 0;

  /// Same as getNumDependentGenCoords()
  virtual std::size_t getNumDependentDofs() const = 0;

  /// Get a pointer to the _indexth dependent DegreeOfFreedom for this BodyNode
  virtual DegreeOfFreedom* getDependentDof(std::size_t _index) = 0;

  /// Get a pointer to the _indexth dependent DegreeOfFreedom for this BodyNode
  virtual const DegreeOfFreedom* getDependentDof(std::size_t _index) const = 0;

  /// Return a std::vector of DegreeOfFreedom pointers that this Node depends on
  virtual const std::vector<DegreeOfFreedom*>& getDependentDofs() = 0;

  /// Return a std::vector of DegreeOfFreedom pointers that this Node depends on
  virtual const std::vector<const DegreeOfFreedom*>& getDependentDofs() const = 0;

  /// Returns a DegreeOfFreedom vector containing the dofs that form a Chain
  /// leading up to this JacobianNode from the root of the Skeleton.
  virtual const std::vector<const DegreeOfFreedom*> getChainDofs() const = 0;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian Functions
  //----------------------------------------------------------------------------

  /// Return the generalized Jacobian targeting the origin of this JacobianNode.
  /// The Jacobian is expressed in the Frame of this JacobianNode.
  virtual const math::Jacobian& getJacobian() const = 0;

  /// A version of getJacobian() that lets you specify a coordinate Frame to
  /// express the Jacobian in.
  virtual math::Jacobian getJacobian(const Frame* _inCoordinatesOf) const = 0;

  /// Return the generalized Jacobian targeting an offset within the Frame of
  /// this JacobianNode.
  virtual math::Jacobian getJacobian(const Eigen::Vector3d& _offset) const = 0;

  /// A version of getJacobian(const Eigen::Vector3d&) that lets you specify a
  /// coordinate Frame to express the Jacobian in.
  virtual math::Jacobian getJacobian(const Eigen::Vector3d& _offset,
                                     const Frame* _inCoordinatesOf) const = 0;

  /// Return the generalized Jacobian targeting the origin of this JacobianNode.
  /// The Jacobian is expressed in the World Frame.
  virtual const math::Jacobian& getWorldJacobian() const = 0;

  /// Return the generalized Jacobian targeting an offset in this JacobianNode.
  /// The _offset is expected in coordinates of this BodyNode Frame. The
  /// Jacobian is expressed in the World Frame.
  virtual math::Jacobian getWorldJacobian(
      const Eigen::Vector3d& _offset) const = 0;

  /// Return the linear Jacobian targeting the origin of this BodyNode. You can
  /// specify a coordinate Frame to express the Jacobian in.
  virtual math::LinearJacobian getLinearJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Return the generalized Jacobian targeting an offset within the Frame of
  /// this BodyNode.
  virtual math::LinearJacobian getLinearJacobian(
      const Eigen::Vector3d& _offset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Return the angular Jacobian targeting the origin of this BodyNode. You can
  /// specify a coordinate Frame to express the Jacobian in.
  virtual math::AngularJacobian getAngularJacobian(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Return the spatial time derivative of the generalized Jacobian targeting
  /// the origin of this BodyNode. The Jacobian is expressed in this BodyNode's
  /// coordinate Frame.
  ///
  /// NOTE: Since this is a spatial time derivative, it should be used with
  /// spatial vectors. If you are using classical linear and angular
  /// acceleration vectors, then use getJacobianClassicDeriv(),
  /// getLinearJacobianDeriv(), or getAngularJacobianDeriv() instead.
  virtual const math::Jacobian& getJacobianSpatialDeriv() const = 0;

  /// A version of getJacobianSpatialDeriv() that can return the Jacobian in
  /// coordinates of any Frame.
  ///
  /// NOTE: This Jacobian Derivative is only for use with spatial vectors. If
  /// you are using classical linear and angular vectors, then use
  /// getJacobianClassicDeriv(), getLinearJacobianDeriv(), or
  /// getAngularJacobianDeriv() instead.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const Frame* _inCoordinatesOf) const = 0;

  /// Return the spatial time derivative of the generalized Jacobian targeting
  /// an offset in the Frame of this BodyNode. The Jacobian is expressed in
  /// this BodyNode's coordinate Frame.
  ///
  /// NOTE: This Jacobian Derivative is only for use with spatial vectors. If
  /// you are using classic linear and angular vectors, then use
  /// getJacobianClassicDeriv(), getLinearJacobianDeriv(), or
  /// getAngularJacobianDeriv() instead.
  ///
  /// \sa getJacobianSpatialDeriv()
  virtual math::Jacobian getJacobianSpatialDeriv(
      const Eigen::Vector3d& _offset) const = 0;

  /// A version of getJacobianSpatialDeriv(const Eigen::Vector3d&) that allows
  /// an arbitrary coordinate Frame to be specified.
  virtual math::Jacobian getJacobianSpatialDeriv(
      const Eigen::Vector3d& _offset,
      const Frame* _inCoordinatesOf) const = 0;

  /// Return the classical time derivative of the generalized Jacobian targeting
  /// the origin of this BodyNode. The Jacobian is expressed in the World
  /// coordinate Frame.
  ///
  /// NOTE: Since this is a classical time derivative, it should be used with
  /// classical linear and angular vectors. If you are using spatial vectors,
  /// use getJacobianSpatialDeriv() instead.
  virtual const math::Jacobian& getJacobianClassicDeriv() const = 0;

  /// A version of getJacobianClassicDeriv() that can return the Jacobian in
  /// coordinates of any Frame.
  ///
  /// NOTE: Since this is a classical time derivative, it should be used with
  /// classical linear and angular vectors. If you are using spatial vectors,
  /// use getJacobianSpatialDeriv() instead.
  virtual math::Jacobian getJacobianClassicDeriv(
      const Frame* _inCoordinatesOf) const = 0;

  /// A version of getJacobianClassicDeriv() that can compute the Jacobian for
  /// an offset within the Frame of this BodyNode. The offset must be expressed
  /// in the coordinates of this BodyNode Frame.
  ///
  /// NOTE: Since this is a classical time derivative, it should be used with
  /// classical linear and angular vectors. If you are using spatial vectors,
  /// use getJacobianSpatialDeriv() instead.
  virtual math::Jacobian getJacobianClassicDeriv(
      const Eigen::Vector3d& _offset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Return the linear Jacobian (classical) time derivative, in terms of any
  /// coordinate Frame.
  ///
  /// NOTE: Since this is a classical time derivative, it should be used with
  /// classical linear vectors. If you are using spatial vectors, use
  /// getJacobianSpatialDeriv() instead.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// A version of getLinearJacobianDeriv() that can compute the Jacobian for
  /// an offset within the Frame of this BodyNode. The offset must be expressed
  /// in coordinates of this BodyNode Frame.
  ///
  /// NOTE: Since this is a classical time derivative, it should be used with
  /// classical linear vectors. If you are using spatial vectors, use
  /// getJacobianSpatialDeriv() instead.
  virtual math::LinearJacobian getLinearJacobianDeriv(
      const Eigen::Vector3d& _offset,
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// Return the angular Jacobian time derivative, in terms of any coordinate
  /// Frame.
  virtual math::AngularJacobian getAngularJacobianDeriv(
      const Frame* _inCoordinatesOf = Frame::World()) const = 0;

  /// \}

  /// Notify this BodyNode and all its descendents that their Jacobians need to
  /// be updated.
  DART_DEPRECATED(6.2)
  void notifyJacobianUpdate();

  /// Notify this BodyNode and all its descendents that their Jacobians need to
  /// be updated.
  void dirtyJacobian();

  /// Notify this BodyNode and all its descendents that their Jacobian
  /// derivatives need to be updated.
  DART_DEPRECATED(6.2)
  void notifyJacobianDerivUpdate();

  /// Notify this BodyNode and all its descendents that their Jacobian
  /// derivatives need to be updated.
  void dirtyJacobianDeriv();

protected:

  /// Constructor
  JacobianNode(BodyNode* bn);

  /// Dirty flag for body Jacobian.
  mutable bool mIsBodyJacobianDirty;

  /// Dirty flag for world Jacobian
  mutable bool mIsWorldJacobianDirty;

  /// Dirty flag for spatial time derivative of body Jacobian.
  mutable bool mIsBodyJacobianSpatialDerivDirty;

  /// Dirty flag for the classic time derivative of the Jacobian
  mutable bool mIsWorldJacobianClassicDerivDirty;

  /// Inverse kinematics module which gets lazily created upon request
  std::shared_ptr<InverseKinematics> mIK;

  /// JacobianNode children that descend from this JacobianNode
  std::unordered_set<JacobianNode*> mChildJacobianNodes;

};

} // namespace dart
} // namespace dynamics

#endif // DART_DYNAMICS_JACOBIANNODE_HPP_
