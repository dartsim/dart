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

#ifndef DART_DYNAMICS_SKELETONDERIVATIVES_HPP_
#define DART_DYNAMICS_SKELETONDERIVATIVES_HPP_

#include <Eigen/Dense>

#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
class SkeletonDerivatives final
    : public common::CompositeTrackingAspect<Skeleton>
{
public:
  using Base = common::CompositeTrackingAspect<Skeleton>;
  using SpatialVelocityDerivative = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  SkeletonDerivatives() = default;

  SkeletonDerivatives(const SkeletonDerivatives&) = delete;

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override;

  //----------------------------------------------------------------------------
  /// \{ \name Derivatives of spatial velocities of bodies
  //----------------------------------------------------------------------------

  SpatialVelocityDerivative getSpatialVelocityDerivativeWrtPositions(
      std::size_t bodyNodeIndexInSkeleton) const;

  Eigen::Vector6d getSpatialVelocityDerivativeWrtPositions(
      std::size_t bodyNodeIndexInSkeleton, std::size_t withRespectTo) const;

  Eigen::Vector6d getSpatialVelocityDerivativeWrtPositions(
      std::size_t bodyNodeIndexInSkeleton,
      const DegreeOfFreedom* withRespectTo) const;

  SpatialVelocityDerivative getSpatialVelocityDerivativeWrtVelocities(
      std::size_t bodyNodeIndexInSkeleton) const;

  Eigen::Vector6d getSpatialVelocityDerivativeWrtVelocities(
      std::size_t bodyNodeIndexInSkeleton, std::size_t withRespectTo) const;

  Eigen::Vector6d getSpatialVelocityDerivativeWrtVelocities(
      std::size_t bodyNodeIndexInSkeleton,
      const DegreeOfFreedom* withRespectTo) const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Gradients of Lagrangian
  //----------------------------------------------------------------------------

  /// Compute and return the first derivative of the Lagrangian with respect to
  /// the generalized coordinates.
  const Eigen::VectorXd& computeLagrangianGradientWrtPositions();

  /// Compute and return the first derivative of the Lagrangian with respect to
  /// the generalized velocities.
  const Eigen::VectorXd& computeLagrangianGradientWrtVelocities();

  /// \}

protected:
  void setComposite(common::Composite* newComposite) override;

protected:
  Eigen::VectorXd mDM_GradientKineticEnergy_q;
  Eigen::VectorXd mDM_GradientKineticEnergy_dq;

  Eigen::VectorXd mDM_GradientOfLagrangian_q;
  Eigen::VectorXd mDM_GradientOfLagrangian_dq;

  Eigen::VectorXd mDM_D2LD;
  Eigen::VectorXd mDM_D1LD;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SKELETONDERIVATIVES_HPP_
