/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_DEGREEOFFREEDOM_H_
#define DART_DYNAMICS_DEGREEOFFREEDOM_H_

#include <string>
#include <Eigen/Core>

namespace dart {
namespace dynamics {

class Skeleton;
class Joint;
class BodyNode;

/// DegreeOfFreedom class represents a single degree of freedom (or generalized coordinate)
/// of the Skeleton.
///
/// DegreeOfFreedom stores properties like position, velocity, and acceleration values and
/// limits for each DegreeOfFreedom in the Skeleton, and the Skeleton's DegreesOfFreedom
/// will be accessible by name or by index

class DegreeOfFreedomProperties
{
public:

  DegreeOfFreedomProperties();

  double mPosition;
  double mPositionLowerLimit;
  double mPositionUpperLimit;

  double mVelocity;
  double mVelocityLowerLimit;
  double mVelocityUpperLimit;

  double mAcceleration;
  double mAccelerationLowerLimit;
  double mAccelerationUpperLimit;

  double mEffort;
  double mEffortLowerLimit;
  double mEffortUpperLimit;

  double mVelocityChange;

  double mConstraintImpulse;

  double mSpringStiffness;
  double mRestPosition;
  double mDampingCoefficient;

  Eigen::Matrix<double, 6, 1> mJacobian;
  Eigen::Matrix<double, 6, 1> mJacobianDeriv;



};

class DegreeOfFreedom : protected DegreeOfFreedomProperties
{
public:

  DegreeOfFreedom();

  bool setPosition();
  double getPosition() const;

  void setPositionLowerLimit();
  double getPositionLowerLimit() const;
  void setPositionUpperLimit();
  double getPositionUpperLimit() const;

  // TODO: the rest

};





} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DEGREEOFFREEDOM_H_
