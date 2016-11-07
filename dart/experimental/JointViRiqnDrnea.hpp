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

#ifndef DART_DYNAMICS_JOINTVIRIQNDRNEA_HPP_
#define DART_DYNAMICS_JOINTVIRIQNDRNEA_HPP_

#include <Eigen/Dense>

#include "dart/common/AspectWithVersion.hpp"
#include "dart/dynamics/GenericJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
class JointViRiqnDrnea : public common::CompositeTrackingAspect<Joint>
{
public:
  friend class SkeletonViRiqnDrnea;
  friend class BodyNodeViRiqnDrnea;

  using Base = common::CompositeTrackingAspect<Joint>;

protected:
  virtual void setNextPositions(const Eigen::VectorXd& nextPositions) = 0;
  virtual void updateNextRelativeTransform() = 0;
  virtual void evaluateDel(const Eigen::Vector6d& force, double timeStep) = 0;
  virtual Eigen::VectorXd getError() const = 0;

  /// Transform for the next configuration
  Eigen::Isometry3d mNextRelativeTransform{Eigen::Isometry3d::Identity()};
};

//==============================================================================
template <typename ConfigSpaceT>
class GenericJointViRiqnDrnea : public JointViRiqnDrnea
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class SkeletonViRiqnDrnea;
  friend class BodyNodeViRiqnDrnea;

  using Base = JointViRiqnDrnea;

  using Vector = typename ConfigSpaceT::Vector;
  using GradientMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  GenericJointViRiqnDrnea()
  {
    // Do nothing
  }

  GenericJointViRiqnDrnea(const GenericJointViRiqnDrnea&) = delete;

protected:
  void setComposite(common::Composite* newComposite) override
  {
    Base::setComposite(newComposite);
  }

  void setNextPositions(const Eigen::VectorXd& nextPositions) override
  {
    assert(
        mComposite->getNumDofs()
        == static_cast<std::size_t>(nextPositions.size()));

    mNextPositions = nextPositions;
  }

  void evaluateDel(const Eigen::Vector6d& force, double timeStep) override
  {
    auto* genJoint = getGenericJoint();

    mFdel = genJoint->getRelativeJacobianStatic().transpose() * force;
    mFdel -= timeStep * genJoint->getForcesStatic();
  }

  Eigen::VectorXd getError() const override { return mFdel; }

protected:
  Vector mNextPositions{Vector::Zero()};
  Vector mFdel{Vector::Zero()};

private:
  GenericJoint<ConfigSpaceT>* getGenericJoint()
  {
    assert(dynamic_cast<GenericJoint<ConfigSpaceT>*>(mComposite));

    return static_cast<GenericJoint<ConfigSpaceT>*>(mComposite);
  }
};

//==============================================================================
class RevoluteJointViRiqnDrnea final
    : public GenericJointViRiqnDrnea<math::R1Space>
{
public:
  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override;

protected:
  // Documentation inherited
  void updateNextRelativeTransform() override;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_JOINTVIRIQNDRNEA_HPP_
