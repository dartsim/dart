/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_UNIVERSALJOINT_H_
#define DART_DYNAMICS_UNIVERSALJOINT_H_

#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

class UniversalJoint : public Joint {
public:
  /// \brief Constructor.
  UniversalJoint(const Eigen::Vector3d& _axis0 = Eigen::Vector3d(1.0, 0.0, 0.0),
                 const Eigen::Vector3d& _axis1 = Eigen::Vector3d(0.0, 1.0, 0.0),
                 const std::string& _name = "Universal joint");

  /// \brief Destructor.
  virtual ~UniversalJoint();

  /// \brief
  void setAxis1(const Eigen::Vector3d& _axis);

  /// \brief
  void setAxis2(const Eigen::Vector3d& _axis);

  /// \brief
  const Eigen::Vector3d& getAxis1() const;

  /// \brief
  const Eigen::Vector3d& getAxis2() const;

  // Documentation inherited.
  virtual void updateTransform();

  // Documentation inherited.
  virtual void updateJacobian();

  // Documentation inherited.
  virtual void updateJacobianTimeDeriv();

protected:
  /// \brief Euler angles X, Y, Z
  GenCoord mCoordinate[2];

  /// \brief Rotational axis.
  Eigen::Vector3d mAxis[2];

public:
  //
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_UNIVERSALJOINT_H_
