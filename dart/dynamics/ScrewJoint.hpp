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

#ifndef DART_DYNAMICS_SCREWJOINT_HPP_
#define DART_DYNAMICS_SCREWJOINT_HPP_

#include "dart/dynamics/detail/ScrewJointAspect.hpp"

namespace dart {
namespace dynamics {

/// class ScrewJoint
class ScrewJoint : public detail::ScrewJointBase
{
public:

  friend class Skeleton;
  using UniqueProperties = detail::ScrewJointUniqueProperties;
  using Properties = detail::ScrewJointProperties;
  using Base = detail::ScrewJointBase;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, ScrewJointAspect)

  ScrewJoint(const ScrewJoint&) = delete;

  /// Destructor
  virtual ~ScrewJoint();

  /// Set the Properties of this ScrewJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this ScrewJoint
  void setProperties(const UniqueProperties& _properties);

  /// Set the AspectProperties of this ScrewJoint
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this ScrewJoint
  Properties getScrewJointProperties() const;

  /// Copy the Properties of another ScrewJoint
  void copy(const ScrewJoint& _otherJoint);

  /// Copy the Properties of another ScrewJoint
  void copy(const ScrewJoint* _otherJoint);

  /// Copy the Properties of another ScrewJoint
  ScrewJoint& operator=(const ScrewJoint& _otherJoint);

  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  ///
  void setAxis(const Eigen::Vector3d& _axis);

  ///
  const Eigen::Vector3d& getAxis() const;

  ///
  void setPitch(double _pitch);

  ///
  double getPitch() const;

  // Documentation inherited
  GenericJoint<math::R1Space>::JacobianMatrix getRelativeJacobianStatic(
      const GenericJoint<math::R1Space>::Vector& positions) const override;

protected:

  /// Constructor called by Skeleton class
  ScrewJoint(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateRelativeTransform() const override;

  // Documentation inherited
  void updateRelativeJacobian(bool _mandatory=true) const override;

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SCREWJOINT_HPP_
