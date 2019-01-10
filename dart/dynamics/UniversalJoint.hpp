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

#ifndef DART_DYNAMICS_UNIVERSALJOINT_HPP_
#define DART_DYNAMICS_UNIVERSALJOINT_HPP_

#include "dart/dynamics/detail/UniversalJointAspect.hpp"

namespace dart {
namespace dynamics {

/// class UniversalJoint
class UniversalJoint : public detail::UniversalJointBase
{
public:

  friend class Skeleton;
  using UniqueProperties = detail::UniversalJointUniqueProperties;
  using Properties = detail::UniversalJointProperties;
  using Base = detail::UniversalJointBase;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, UniversalJointAspect)

  UniversalJoint(const UniversalJoint&) = delete;

  /// Destructor
  virtual ~UniversalJoint();

  /// Set the Properties of this UniversalJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this UniversalJoint
  void setProperties(const UniqueProperties& _properties);

  /// Set the AspectProperties of this UniversalJoint
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this UniversalJoint
  Properties getUniversalJointProperties() const;

  /// Copy the Properties of another UniversalJoint
  void copy(const UniversalJoint& _otherJoint);

  /// Copy the Properties of another UniversalJoint
  void copy(const UniversalJoint* _otherJoint);

  /// Copy the Properties of another UniversalJoint
  UniversalJoint& operator=(const UniversalJoint& _otherJoint);

  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  ///
  void setAxis1(const Eigen::Vector3d& _axis);

  ///
  void setAxis2(const Eigen::Vector3d& _axis);

  ///
  const Eigen::Vector3d& getAxis1() const;

  ///
  const Eigen::Vector3d& getAxis2() const;

  // Documentation inherited
  Eigen::Matrix<double, 6, 2> getRelativeJacobianStatic(
      const Eigen::Vector2d& _positions) const override;

protected:

  /// Constructor called by Skeleton class
  UniversalJoint(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  using GenericJoint<math::R2Space>::getRelativeJacobianStatic;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateRelativeTransform() const override;

  // Documentation inherited
  void updateRelativeJacobian(bool =true) const override;

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_UNIVERSALJOINT_HPP_
