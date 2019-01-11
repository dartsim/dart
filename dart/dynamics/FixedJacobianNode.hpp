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

#ifndef DART_DYNAMICS_FIXEDJACOBIANNODE_HPP_
#define DART_DYNAMICS_FIXEDJACOBIANNODE_HPP_

#include "dart/dynamics/detail/FixedJacobianNode.hpp"

namespace dart {
namespace dynamics {

class FixedJacobianNode :
    public detail::FixedJacobianNodeCompositeBase,
    public AccessoryNode<FixedJacobianNode>
{
public:

  /// Set the current relative transform of this Fixed Frame
  void setRelativeTransform(const Eigen::Isometry3d& newRelativeTf) override;

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

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian Functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian& getJacobian() const override final;

  // Prevent the inherited getJacobian functions from being shadowed
  using TemplatedJacobianNode<FixedJacobianNode>::getJacobian;

  // Documentation inherited
  const math::Jacobian& getWorldJacobian() const override final;

  // Prevent the inherited getWorldJacobian functions from being shadowed
  using TemplatedJacobianNode<FixedJacobianNode>::getWorldJacobian;

  // Documentation inherited
  const math::Jacobian& getJacobianSpatialDeriv() const override final;

  // Prevent the inherited getJacobianSpatialDeriv functions from being shadowed
  using TemplatedJacobianNode<FixedJacobianNode>::getJacobianSpatialDeriv;

  // Documentation inherited
  const math::Jacobian& getJacobianClassicDeriv() const override final;

  // Prevent the inherited getJacobianClassicDeriv functions from being shadowed
  using TemplatedJacobianNode<FixedJacobianNode>::getJacobianClassicDeriv;

  /// \}

protected:

  /// Constructor
  FixedJacobianNode(BodyNode* parent, const Eigen::Isometry3d& transform);

  /// Tuple constructor
  FixedJacobianNode(const std::tuple<BodyNode*, Eigen::Isometry3d>& args);

  /// Update the Jacobian of this Fixed Frame. getJacobian() calls this function
  /// if mIsBodyJacobianDirty is true.
  void updateBodyJacobian() const;

  /// Update the World Jacobian cache.
  void updateWorldJacobian() const;

  /// Update the spatial time derivative of the Fixed Frame Jacobian.
  /// getJacobianSpatialDeriv() calls this function if
  /// mIsBodyJacobianSpatialDerivDirty is true.
  void updateBodyJacobianSpatialDeriv() const;

  /// Update the classic time derivative of the Fixed Frame Jacobian.
  /// getJacobianClassicDeriv() calls this function if
  /// mIsWorldJacobianClassicDerivDirty is true.
  void updateWorldJacobianClassicDeriv() const;

  struct Cache
  {
    /// Cached Jacobian of this Fixed Frame
    ///
    /// Do not use directly! Use getJacobian() to access this quantity
    math::Jacobian mBodyJacobian;

    /// Cached World Jacobian of this Fixed Frame
    ///
    /// Do not use directly! Use getWorldJacobian() to access this quantity
    math::Jacobian mWorldJacobian;

    /// Spatial time derivative of Fixed Frame Jacobian
    ///
    /// Do not use directly! Use getJacobianSpatialDeriv() to access this quantity
    math::Jacobian mBodyJacobianSpatialDeriv;

    /// Classic time derivative of the Fixed Frame Jacobian
    ///
    /// Do not use directly! Use getJacobianClassicDeriv() to access this quantity
    math::Jacobian mWorldJacobianClassicDeriv;

    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  mutable Cache mCache;

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_FIXEDJACOBIANNODE_HPP_
