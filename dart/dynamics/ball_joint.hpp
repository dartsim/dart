/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/dynamics/detail/ball_joint_aspect.hpp>

#include <dart/math/geometry.hpp>

#include <dart/export.hpp>

#include <Eigen/Dense>

namespace dart {
namespace dynamics {

/// class BallJoint
class DART_API BallJoint : public detail::BallJointBase
{
public:
  friend class Skeleton;

  using CoordinateChart = detail::CoordinateChart;
  using UniqueProperties = detail::BallJointUniqueProperties;
  using Properties = detail::BallJointProperties;
  using Base = detail::BallJointBase;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, BallJointAspect)

  /// Destructor
  virtual ~BallJoint();

  /// Set the Properties of this BallJoint
  void setProperties(const Properties& properties);

  /// Set the Properties of this BallJoint
  void setProperties(const UniqueProperties& properties);

  /// Set the AspectProperties of this BallJoint
  void setAspectProperties(const AspectProperties& properties);

  // Documentation inherited
  std::string_view getType() const override;

  /// Get joint type for this class
  static std::string_view getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  /// Get the Properties of this BallJoint
  Properties getBallJointProperties() const;

  /// Copy the Properties of another BallJoint
  void copy(const BallJoint& otherJoint);

  /// Copy the Properties of another BallJoint
  void copy(const BallJoint* otherJoint);

  /// Same as copy(const BallJoint&)
  BallJoint& operator=(const BallJoint& otherJoint);

  /// Set the coordinate chart used for generalized positions.
  ///
  /// This affects how positions map to rotations. Generalized velocities
  /// remain angular velocities.
  ///
  /// Preserves the current pose by reparameterizing existing positions.
  void setCoordinateChart(CoordinateChart chart);

  /// Get the coordinate chart used for generalized positions.
  CoordinateChart getCoordinateChart() const;

  /// Convert a rotation into a 3D vector that can be used to set the positions
  /// of a BallJoint. The positions returned by this function will result in a
  /// relative transform of
  /// getTransformFromParentBodyNode() * _rotation *
  /// getTransformFromChildBodyNode().inverse() between the parent BodyNode and
  /// the child BodyNode frames when applied to a BallJoint.
  template <typename RotationType>
  static Eigen::Vector3d convertToPositions(const RotationType& _rotation)
  {
    return convertToPositions(_rotation, CoordinateChart::EXP_MAP);
  }

  /// Convert a rotation into a 3D vector that can be used to set the positions
  /// of a BallJoint using the specified coordinate chart.
  template <typename RotationType>
  static Eigen::Vector3d convertToPositions(
      const RotationType& _rotation, CoordinateChart chart)
  {
    switch (chart) {
      case CoordinateChart::EXP_MAP:
        return math::logMap(_rotation);
      case CoordinateChart::EULER_XYZ:
        return math::matrixToEulerXYZ(_rotation);
      case CoordinateChart::EULER_ZYX:
        return math::matrixToEulerZYX(_rotation);
      default:
        DART_WARN(
            "Unsupported coordinate chart ({}); returning zero vector",
            static_cast<int>(chart));
        return Eigen::Vector3d::Zero();
    }
  }

  /// Convert a BallJoint-style position vector into a transform
  static Eigen::Isometry3d convertToTransform(
      const Eigen::Vector3d& _positions);

  /// Convert a BallJoint-style position vector into a transform using the
  /// specified coordinate chart.
  static Eigen::Isometry3d convertToTransform(
      const Eigen::Vector3d& _positions, CoordinateChart chart);

  /// Convert a BallJoint-style position vector into a rotation matrix
  static Eigen::Matrix3d convertToRotation(const Eigen::Vector3d& _positions);

  /// Convert a BallJoint-style position vector into a rotation matrix using
  /// the specified coordinate chart.
  static Eigen::Matrix3d convertToRotation(
      const Eigen::Vector3d& _positions, CoordinateChart chart);

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
  void integratePositions(
      const Eigen::VectorXd& q0,
      const Eigen::VectorXd& v,
      double dt,
      Eigen::VectorXd& result) const override;

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

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_BALLJOINT_HPP_
