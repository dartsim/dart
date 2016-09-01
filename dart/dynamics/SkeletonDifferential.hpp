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

#ifndef DART_DYNAMICS_SKELETONDIFFERENTIAL_HPP_
#define DART_DYNAMICS_SKELETONDIFFERENTIAL_HPP_

#include <Eigen/Dense>

#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

//class Skeleton;

namespace detail {

struct SkeletonLagrangianAspectState
{
  /// Constructor
  SkeletonLagrangianAspectState();

  /// Destructor
  virtual ~SkeletonLagrangianAspectState() = default;

  Eigen::VectorXd mDM_GradientKineticEnergy_q;
  Eigen::VectorXd mDM_GradientKineticEnergy_dq;
  Eigen::MatrixXd mDM_HessianKineticEnergy_q_q;
  Eigen::MatrixXd mDM_HessianKineticEnergy_q_dq;
  Eigen::MatrixXd mDM_HessianKineticEnergy_dq_dq;

  Eigen::VectorXd mDM_GradientOfLagrangian_q;
  Eigen::VectorXd mDM_GradientOfLagrangian_dq;
  Eigen::MatrixXd mDM_HessianOfLagrangian_q_q;
  Eigen::MatrixXd mDM_HessianOfLagrangian_q_dq;
  Eigen::MatrixXd mDM_HessianOfLagrangian_dq_dq;

  Eigen::VectorXd mDM_D2LD;
  Eigen::VectorXd mDM_D1LD;
  Eigen::MatrixXd mDM_D2D1LD;
};

}  // namespace detail

//==============================================================================
class SkeletonDifferential final :
    public common::AspectWithState<
        SkeletonDifferential,
        detail::SkeletonLagrangianAspectState,
        Skeleton>
{
public:

  using Base = common::AspectWithState<
      SkeletonDifferential,
      detail::SkeletonLagrangianAspectState,
      Skeleton>;

  using GradientMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

//  SkeletonLagrangianAspect(const PropertiesData& properties = PropertiesData());

  SkeletonDifferential(const StateData& state = StateData());

  SkeletonDifferential(const SkeletonDifferential&) = delete;

  void updateBodyVelocityGradients();

  void updateLagrangianGradientWrtPositions();

  Eigen::VectorXd computeLagrangianGradientWrtPositions();

  Eigen::VectorXd computeLagrangianGradientWrtVelocities();

  GradientMatrix getBodyVelocityGradientWrtQ(
      std::size_t bodyNodeIndexInSkeleton) const;

  Eigen::Vector6d getBodyVelocityGradientWrtQ(
      std::size_t bodyNodeIndexInSkeleton,
      std::size_t withRespectTo) const;

  Eigen::Vector6d getBodyVelocityGradientWrtQ(
      std::size_t bodyNodeIndexInSkeleton,
      const DegreeOfFreedom* withRespectTo) const;

  GradientMatrix getBodyVelocityGradientWrtDQ(
      std::size_t bodyNodeIndexInSkeleton) const;

  Eigen::Vector6d getBodyVelocityGradientWrtDQ(
      std::size_t bodyNodeIndexInSkeleton,
      std::size_t withRespectTo) const;

  Eigen::Vector6d getBodyVelocityGradientWrtDQ(
      std::size_t bodyNodeIndexInSkeleton,
      const DegreeOfFreedom* withRespectTo) const;

  void print();

protected:

  void setComposite(common::Composite* newComposite) override;

  void loseComposite(common::Composite* oldComposite) override;


};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SKELETONDIFFERENTIAL_HPP_
