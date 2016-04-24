/*
 * Copyright (c) 2013-2016, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_WELDJOINT_HPP_
#define DART_DYNAMICS_WELDJOINT_HPP_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/ZeroDofJoint.hpp"

namespace dart {
namespace dynamics {

/// class WeldJoint
class WeldJoint : public ZeroDofJoint
{
public:

  friend class Skeleton;

  struct Properties : ZeroDofJoint::Properties
  {
    Properties(const Joint::Properties& _properties = Joint::Properties());
    virtual ~Properties() = default;
  };

  WeldJoint(const WeldJoint&) = delete;

  /// Destructor
  virtual ~WeldJoint();

  /// Get the Properties of this WeldJoint
  Properties getWeldJointProperties() const;

  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  // Documentation inherited
  void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T) override;

  // Documentation inherited
  void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T) override;

protected:

  /// Constructor called by Skeleton class
  WeldJoint(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  //----------------------------------------------------------------------------
  // Recursive algorithms
  //----------------------------------------------------------------------------

  // Documentation inherited
  void updateLocalTransform() const override;

  // Documentation inherited
  void updateLocalSpatialVelocity() const override;

  // Documentation inherited
  void updateLocalSpatialAcceleration() const override;

  // Documentation inherited
  void updateLocalPrimaryAcceleration() const override;

  // Documentation inherited
  void updateLocalJacobian(bool =true) const override;

  // Documentation inherited
  void updateLocalJacobianTimeDeriv() const override;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_WELDJOINT_HPP_

