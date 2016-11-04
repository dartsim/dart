/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_BODYNODESECONDDERIVATIVES_HPP_
#define DART_DYNAMICS_BODYNODESECONDDERIVATIVES_HPP_

#include <Eigen/Dense>

#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/BodyNodeDerivatives.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

class BodyNodeSecondDerivatives final
    : public common::CompositeTrackingAspect<BodyNode>
{
public:
  friend class SkeletonDerivatives;

  using Base = common::CompositeTrackingAspect<BodyNode>;
  using SpatialVelocityDerivative = Eigen::Matrix<double, 6, Eigen::Dynamic>;
  using SpatialVelocitySecondDerivative
      = std::vector<SpatialVelocityDerivative>;

  BodyNodeSecondDerivatives() = default;

  BodyNodeSecondDerivatives(const BodyNodeSecondDerivatives&) = delete;

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override;

  //----------------------------------------------------------------------------
  /// \{ \name Derivative of spatial velocity
  //----------------------------------------------------------------------------

  const SpatialVelocityDerivative&
  getSpatialVelocityDerivativeWrtPositions() const;

  Eigen::Vector6d
  getSpatialVelocityDerivativeWrtPositions(std::size_t indexInSkeleton) const;

  Eigen::Vector6d getSpatialVelocityDerivativeWrtPositions(
      const DegreeOfFreedom* withRespectTo) const;

  SpatialVelocityDerivative
  getSpatialVelocityDerivativeWrtPositions(const Joint* withRespectTo) const;

  const SpatialVelocityDerivative&
  getSpatialVelocityDerivativeWrtVelocities() const;

  Eigen::Vector6d
  getSpatialVelocityDerivativeWrtVelocities(std::size_t indexInSkeleton) const;

  Eigen::Vector6d getSpatialVelocityDerivativeWrtVelocities(
      const DegreeOfFreedom* withRespectTo) const;

  SpatialVelocityDerivative
  getSpatialVelocityDerivativeWrtVelocities(const Joint* withRespectTo) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Second derivative of spatial velocity
  //----------------------------------------------------------------------------

  const SpatialVelocitySecondDerivative&
  getSpatialVelocitySecondDerivativeWrtPositions() const;

  const SpatialVelocitySecondDerivative&
  getSpatialVelocitySecondDerivativeWrtPositionsVelocities() const;

  const SpatialVelocitySecondDerivative&
  getSpatialVelocitySecondDerivativeWrtVelocities() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Gradients of Lagrangian
  //----------------------------------------------------------------------------

  Eigen::VectorXd computeKineticEnergyGradientWrtPositions() const;
  Eigen::VectorXd computeKineticEnergyGradientWrtVelocities() const;

  Eigen::VectorXd computeLagrangianGradientWrtPositions() const;
  Eigen::VectorXd computeLagrangianGradientWrtVelocities() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Hessians of Lagrangian
  //----------------------------------------------------------------------------

  Eigen::MatrixXd computeKineticEnergyHessianWrtPositions() const;
  Eigen::MatrixXd computeKineticEnergyHessianWrtPositionsVelocities() const;
  Eigen::MatrixXd computeKineticEnergyHessianWrtVelocities() const;

  Eigen::MatrixXd computeLagrangianHessianWrtPositions() const;
  Eigen::MatrixXd computeLagrangianHessianWrtPositionsVelocities() const;
  Eigen::MatrixXd computeLagrangianHessianWrtVelocities() const;

  /// \}

protected:
  void setComposite(common::Composite* newComposite) override;

  void dirtySpatialVelocitySecondDerivativeWrtPositions();
  void dirtySpatialVelocitySecondDerivativeWrtPositionsVelocities();

protected:
  BodyNodeDerivatives* mBodyNodeDerivatives{nullptr};

  mutable std::vector<SpatialVelocityDerivative> mV_q_q;
  mutable std::vector<SpatialVelocityDerivative> mV_q_dq;
  mutable std::vector<SpatialVelocityDerivative> mV_dq_dq;

  mutable bool mNeedSpatialVelocitySecondDerivativeWrtPositionsUpdate{true};
  mutable bool mNeedSpatialVelocitySecondDerivativeWrtPositionsVelocitiesUpdate{
      true};
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_BODYNODESECONDDERIVATIVES_HPP_
