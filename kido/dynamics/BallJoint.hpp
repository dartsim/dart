/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef KIDO_DYNAMICS_BALLJOINT_H_
#define KIDO_DYNAMICS_BALLJOINT_H_

#include <Eigen/Dense>

#include "kido/dynamics/MultiDofJoint.hpp"

namespace kido {
namespace dynamics {

/// class BallJoint
class BallJoint : public MultiDofJoint<3>
{
public:

  friend class Skeleton;

  struct Properties : MultiDofJoint<3>::Properties
  {
    Properties(const MultiDofJoint<3>::Properties& _properties =
                                                MultiDofJoint<3>::Properties());
    virtual ~Properties() = default;
  };

  /// Destructor
  virtual ~BallJoint();

  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(size_t _index) const override;

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
  Eigen::Matrix<double, 6, 3> getLocalJacobianStatic(
      const Eigen::Vector3d& _positions) const override;

  // Documentation inherited
  Eigen::Vector3d getPositionDifferencesStatic(
      const Eigen::Vector3d& _q2, const Eigen::Vector3d& _q1) const override;

protected:

  /// Constructor called by Skeleton class
  BallJoint(const Properties& _properties);

  // Documentation inherited
  Joint* clone() const override;

  using MultiDofJoint::getLocalJacobianStatic;

  // Documentation inherited
  void integratePositions(double _dt) override;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateLocalTransform() const override;

  // Documentation inherited
  void updateLocalJacobian(bool _mandatory = true) const override;

  // Documentation inherited
  void updateLocalJacobianTimeDeriv() const override;

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
}  // namespace kido

#endif  // KIDO_DYNAMICS_BALLJOINT_H_

