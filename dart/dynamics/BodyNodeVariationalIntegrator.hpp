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

#ifndef DART_DYNAMICS_BODYNODEVARIATIONALINTEGRATOR_HPP_
#define DART_DYNAMICS_BODYNODEVARIATIONALINTEGRATOR_HPP_

#include <Eigen/Dense>

#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

class JointVariationalIntegrator;

namespace detail {

struct BodyNodeVariationalIntegratorState
{
  using GradientMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  /// Constructor
  BodyNodeVariationalIntegratorState();

  /// Destructor
  virtual ~BodyNodeVariationalIntegratorState() = default;

  /// The prediction of the transform for the next discrete time (k+1)
  Eigen::Isometry3d mNextWorldTransform{Eigen::Isometry3d::Identity()};

  /// The relative transform of the next transform relative to the current
  /// transform.
  Eigen::Isometry3d mDeltaWorldTransform{Eigen::Isometry3d::Identity()};

  /// Discrete spatial velocity for the duration of (k-1, k).
  Eigen::Vector6d mPreAverageVelocity{Eigen::Vector6d::Zero()};

  /// Discrete spatial velocity for the duration of (k, k+1).
  Eigen::Vector6d mPostAverageVelocity{Eigen::Vector6d::Zero()};

  /// Discrete spatial momentum for the duration of (k-1, k).
  Eigen::Vector6d mPrevMomentum{Eigen::Vector6d::Zero()};

  /// Discrete spatial momentum for the duration of (k, k+1).
  Eigen::Vector6d mPostMomentum{Eigen::Vector6d::Zero()};

  /// Spatial impulse transmitted from the parent BodyNode.
  Eigen::Vector6d mParentImpulse{Eigen::Vector6d::Zero()};
};

}  // namespace detail

//==============================================================================
class BodyNodeVariationalIntegrator final :
    public common::AspectWithState<
        BodyNodeVariationalIntegrator,
        detail::BodyNodeVariationalIntegratorState,
        BodyNode>
{
public:

  friend class SkeletonVariationalIntegrator;

  using Base = common::AspectWithState<
      BodyNodeVariationalIntegrator,
      detail::BodyNodeVariationalIntegratorState,
      BodyNode>;

  using GradientMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  BodyNodeVariationalIntegrator(const StateData& state = StateData());

  BodyNodeVariationalIntegrator(const BodyNodeVariationalIntegrator&) = delete;

  JointVariationalIntegrator* getJointVi();
  const JointVariationalIntegrator* getJointVi() const;

  void initialize(double timeStep);

protected:

  void setComposite(common::Composite* newComposite) override;

  void updateNextTransform();
  void updateNextVelocity(double timeStep);

  void updateFdel(const Eigen::Vector3d& gravity, double timeStep);

};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BODYNODEVARIATIONALINTEGRATOR_HPP_
