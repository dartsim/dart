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

#ifndef KIDO_DYNAMICS_UNIVERSALJOINT_HPP_
#define KIDO_DYNAMICS_UNIVERSALJOINT_HPP_

#include <string>

#include <Eigen/Dense>

#include "kido/dynamics/MultiDofJoint.hpp"

namespace kido {
namespace dynamics {

/// class UniversalJoint
class UniversalJoint : public MultiDofJoint<2>
{
public:

  friend class Skeleton;

  struct UniqueProperties
  {
    std::array<Eigen::Vector3d,2> mAxis;

    UniqueProperties(const Eigen::Vector3d& _axis1 = Eigen::Vector3d::UnitX(),
                     const Eigen::Vector3d& _axis2 = Eigen::Vector3d::UnitY());

    virtual ~UniqueProperties() = default;
  };

  struct Properties : MultiDofJoint<2>::Properties,
                      UniversalJoint::UniqueProperties
  {
    Properties(const MultiDofJoint<2>::Properties& _multiDofProperties =
                                            MultiDofJoint<2>::Properties(),
               const UniversalJoint::UniqueProperties& _universalProperties =
                                            UniversalJoint::UniqueProperties());
    virtual ~Properties() = default;
  };

  UniversalJoint(const UniversalJoint&) = delete;

  /// Destructor
  virtual ~UniversalJoint();

  /// Set the Properties of this UniversalJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this UniversalJoint
  void setProperties(const UniqueProperties& _properties);

  /// Get the Properties of this UniversalJoint
  Properties getUniversalJointProperties() const;

  /// Copy the Properties of another UniversalJoint
  void copy(const UniversalJoint& _otherJoint);

  /// Copy the Properties of another UniversalJoint
  void copy(const UniversalJoint* _otherJoint);

  /// Copy the Properties of another UniversalJoint
  UniversalJoint& operator=(const UniversalJoint& _otherJoint);

  // Documentation inherited
  virtual const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  virtual bool isCyclic(size_t _index) const override;

  ///
  void setAxis1(const Eigen::Vector3d& _axis);

  ///
  void setAxis2(const Eigen::Vector3d& _axis);

  ///
  const Eigen::Vector3d& getAxis1() const;

  ///
  const Eigen::Vector3d& getAxis2() const;

  // Documentation inherited
  Eigen::Matrix<double, 6, 2> getLocalJacobianStatic(
      const Eigen::Vector2d& _positions) const override;

protected:

  /// Constructor called by Skeleton class
  UniversalJoint(const Properties& _properties);

  // Documentation inherited
  virtual Joint* clone() const override;

  using MultiDofJoint::getLocalJacobianStatic;

  // Documentation inherited
  virtual void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  virtual void updateLocalTransform() const override;

  // Documentation inherited
  virtual void updateLocalJacobian(bool =true) const override;

  // Documentation inherited
  virtual void updateLocalJacobianTimeDeriv() const override;

protected:

  /// UniversalJoint Properties
  UniqueProperties mUniversalP;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace kido

#endif  // KIDO_DYNAMICS_UNIVERSALJOINT_HPP_
