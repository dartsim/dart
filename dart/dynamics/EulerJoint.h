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

#ifndef DART_DYNAMICS_EULERJOINT_H_
#define DART_DYNAMICS_EULERJOINT_H_

#include <string>

#include "dart/dynamics/MultiDofJoint.h"

namespace dart {
namespace dynamics {

/// class EulerJoint
class EulerJoint : public MultiDofJoint<3>
{
public:
  /// Axis order
  enum AxisOrder
  {
    AO_ZYX = 0,
    AO_XYZ = 1
  };

  /// Constructor
  explicit EulerJoint(const std::string& _name = "EulerJoint");

  /// Destructor
  virtual ~EulerJoint();

  /// Set the axis order
  /// \param[in] _order Axis order
  /// \param[in] _renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the axis order.
  void setAxisOrder(AxisOrder _order, bool _renameDofs = true);

  /// Return the axis order
  AxisOrder getAxisOrder() const;

protected:
  /// Set the names of this joint's DegreesOfFreedom. Used during construction
  /// and when axis order is changed.
  virtual void updateDegreeOfFreedomNames();

  // Documentation inherited
  virtual void updateLocalTransform();

  // Documentation inherited
  virtual void updateLocalJacobian();

  // Documentation inherited
  virtual void updateLocalJacobianTimeDeriv();

protected:
  /// Euler angle order
  AxisOrder mAxisOrder;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_EULERJOINT_H_
