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

#ifndef DART_DYNAMICS_BALLJOINT_HPP_
#define DART_DYNAMICS_BALLJOINT_HPP_

#include <Eigen/Dense>

#include "dart/dynamics/GenericJoint.hpp"

namespace dart {
namespace dynamics {

/// class BallJoint
class BallJoint : public GenericJoint<math::SO3Space>
{
public:

  friend class Skeleton;

  using Base = GenericJoint<math::SO3Space>;

  struct Properties : Base::Properties
  {
    Properties(const Base::Properties& properties
               = Base::Properties());
    virtual ~Properties() = default;
  };

  /// Destructor
  virtual ~BallJoint();

  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  /// Get the Properties of this BallJoint
  Properties getBallJointProperties() const;

  /// Convert a rotation into a 3D vector that can be used to set the positions
  /// of a BallJoint. The positions returned by this function will result in a
  /// relative transform of
  /// getTransformFromParentBodyNode() * _rotation * getTransformFromChildBodyNode().inverse()
  /// between the parent BodyNode and the child BodyNode frames when applied to
  /// a BallJoint.
  template <typename RotationType>
  static Eigen::Vector3d convertToPositions(const RotationType& _rotation)
  {
    return math::logMap(_rotation);
  }

  /// Convert a BallJoint-style position vector into a transform
  static Eigen::Isometry3d convertToTransform(const Eigen::Vector3d& _positions);

  /// Convert a BallJoint-style position vector into a rotation matrix
  static Eigen::Matrix3d convertToRotation(const Eigen::Vector3d& _positions);

  // Documentation inherited
  Eigen::Matrix<double, 6, 3> getRelativeJacobianStatic(
      const Eigen::Vector3d& _positions) const override;

  // Documentation inherited
  Eigen::Vector3d getPositionDifferencesStatic(
      const Eigen::Vector3d& _q2, const Eigen::Vector3d& _q1) const override;

protected:

  /// Constructor called by Skeleton class
  BallJoint(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  using Base::getRelativeJacobianStatic;

  // Documentation inherited
  void integratePositions(double _dt) override;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateRelativeTransform() const override;

  // Documentation inherited
  void updateRelativeJacobian(bool _mandatory = true) const override;

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override;

protected:

  /// Access mR, which is an auto-updating variable
  const Eigen::Isometry3d& getR() const;

  /// Rotation matrix dependent on the generalized coordinates
  ///
  /// Do not use directly! Use getR() to access this
  mutable Eigen::Isometry3d mR;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_BALLJOINT_HPP_

