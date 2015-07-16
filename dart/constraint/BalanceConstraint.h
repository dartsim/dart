/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_CONSTRAINT_BALANCECONSTRAINT_H_
#define DART_CONSTRAINT_BALANCECONSTRAINT_H_

#include "dart/dynamics/HierarchicalIK.h"

namespace dart {
namespace constraint {

/// BalanceConstraint is a kinematic constraint function designed to be passed
/// into a HierarchicalIK module. Adding this constraint to the Problem of a
/// HierarchicalIK will allow the IK solver to constrain the Skeleton so that it
/// satisfies a support polygon style balancing constraint.
class BalanceConstraint : public optimizer::Function,
                          public dynamics::HierarchicalIK::Function
{
public:

  /// The BalanceMethod_t determines whether balancing should be achieved by
  /// shifting the locations of the supporting EndEffectors or by shifting the
  /// center of mass without moving the support locations.
  enum BalanceMethod_t
  {
    SHIFT_SUPPORT = 0,
    SHIFT_COM
  }

  BalanceConstraint(const std::shared_ptr<dynamics::HierarchicalIK>& _ik,
                    BalanceMethod_t _method = SHIFT_SUPPORT);

  virtual ~BalanceConstraint() = default;

  optimizer::FunctionPtr clone(
      const std::shared_ptr<dynamics::HierarchicalIK>& _newIK) const override;

  double eval(const Eigen::VectorXd& _x) override;

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) override;

  void setBalanceMethod(BalanceMethod_t _method);

  BalanceMethod_t getBalanceMethod() const;

  void clearCaches();

protected:

  std::weak_ptr<dynamics::HierarchicalIK> mIK;

  BalanceMethod_t mBalanceMethod;

  Eigen::VectorXd mLastConfig;

};

}
}

#endif // DART_CONSTRAINT_BALANCECONSTRAINT_H_
