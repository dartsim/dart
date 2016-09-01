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

#ifndef DART_DYNAMICS_BODYNODEDIFFERENTIAL_HPP_
#define DART_DYNAMICS_BODYNODEDIFFERENTIAL_HPP_

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

//class Skeleton;

namespace detail {

struct BodyNodeDifferentialState
{
  using GradientMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  GradientMatrix mV_q;
  GradientMatrix mV_dq;

  std::vector<GradientMatrix> mV_q_q;
  std::vector<GradientMatrix> mV_q_dq;
  std::vector<GradientMatrix> mV_dq_dq;

  /// Constructor
  BodyNodeDifferentialState();

  /// Destructor
  virtual ~BodyNodeDifferentialState() = default;
};

}  // namespace detail

//==============================================================================
class BodyNodeDifferential final :
    public common::AspectWithState<
        BodyNodeDifferential,
        detail::BodyNodeDifferentialState,
        BodyNode>
{
public:

  friend class SkeletonDifferential;

  using Base = common::AspectWithState<
      BodyNodeDifferential,
      detail::BodyNodeDifferentialState,
      BodyNode>;

  using GradientMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  BodyNodeDifferential(const StateData& state = StateData());

  BodyNodeDifferential(const BodyNodeDifferential&) = delete;

  /// Update the gradients of spatial body velocities with respect to both of
  /// joint positions and velocities.
  void updateBodyVelocityGradients();
  // TODO(JS): apply automatic updating method

  /// Update the Hessians of spatial body velocities with respect to both of
  /// joint positions and velocities.
  void updateSpatialVelocityHessian();
  // TODO(JS): apply automatic updating method

  Eigen::Vector6d getBodyVelocityGradientWrtQ(
      std::size_t indexInSkeleton) const;

  Eigen::Vector6d getBodyVelocityGradientWrtQ(
      const DegreeOfFreedom* withRespectTo) const;

  GradientMatrix getBodyVelocityGradientWrtQ(
      const Joint* withRespectTo) const;

  GradientMatrix getBodyVelocityGradientWrtQ() const;

  Eigen::Vector6d getBodyVelocityGradientWrtDQ(
      std::size_t indexInSkeleton) const;

  Eigen::Vector6d getBodyVelocityGradientWrtDQ(
      const DegreeOfFreedom* withRespectTo) const;

  GradientMatrix getBodyVelocityGradientWrtDQ(
      const Joint* withRespectTo) const;

  GradientMatrix getBodyVelocityGradientWrtDQ() const;

  void print();

protected:

  void setComposite(common::Composite* newComposite) override;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BODYNODEDIFFERENTIAL_HPP_
