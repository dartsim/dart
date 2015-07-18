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

  /// The ErrorMethod_t determines whether the error should be computed based on
  /// the center of mass's distance from the centroid of the support polygon
  /// (FROM_CENTROID) or the COM's distance from the edge of the support polygon
  /// (FROM_EDGE). Note that once the center of mass is inside the support
  /// polygon, the error will immediately drop to zero regardless of which is
  /// chosen.
  ///
  /// Alternatively, choosing OPTIMIZE_BALANCE will not drop the error to zero
  /// while inside the support polygon. Instead, it will try to drive the center
  /// of mass to the centroid of the support polygon. The error will only drop
  /// to zero once it is within a certain tolerance of the centroid. The
  /// tolerance can be set by using setOptimizationTolerance(). This method is
  /// ideal for use as an Objective function to improve the quality of a
  /// configuration rather than as a constraint function.
  enum ErrorMethod_t
  {
    FROM_CENTROID = 0,
    FROM_EDGE,
    OPTIMIZE_BALANCE
  };

  /// The BalanceMethod_t determines whether balancing should be achieved by
  /// shifting the locations of the supporting EndEffectors or by shifting the
  /// center of mass without moving the support locations.
  ///
  /// Note that if the BalanceMethod_t is SHIFT_SUPPORT, then its behavior will
  /// change significantly based on the ErrorMethod_t: if the ErrorMethod_t is
  /// FROM_CENTROID or OPTIMIZE BALANCE, then all support points will be shifted
  /// towards the center of mass simultaneously. However, if the ErrorMethod_t
  /// is FROM_EDGE, then only the EndEffector that is closest to the center of
  /// mass will be shifted.
  enum BalanceMethod_t
  {
    SHIFT_SUPPORT = 0,
    SHIFT_COM
  };

  BalanceConstraint(const std::shared_ptr<dynamics::HierarchicalIK>& _ik,
                    BalanceMethod_t _balanceMethod = SHIFT_SUPPORT,
                    ErrorMethod_t _errorMethod = FROM_CENTROID);

  virtual ~BalanceConstraint() = default;

  optimizer::FunctionPtr clone(
      const std::shared_ptr<dynamics::HierarchicalIK>& _newIK) const override;

  double eval(const Eigen::VectorXd& _x) override;

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) override;

  void setErrorMethod(ErrorMethod_t _method);

  ErrorMethod_t getErrorMethod() const;

  void setBalanceMethod(BalanceMethod_t _method);

  BalanceMethod_t getBalanceMethod() const;

  void setOptimizationTolerance(double _tol);

  double getOptimizationTolerance() const;

  void setPseudoInverseDamping(double _damping);

  double getPseudoInverseDamping();

  const Eigen::Vector3d& getLastError() const;

  void clearCaches();

protected:

  std::weak_ptr<dynamics::HierarchicalIK> mIK;

  BalanceMethod_t mBalanceMethod;

  ErrorMethod_t mErrorMethod;

  double mOptimizationTolerance;

  double mDamping;

  size_t mClosestEndEffector[2];

  /// The error vector points away from the direction that the center of mass
  /// should move in order to reduce the balance error
  Eigen::Vector3d mLastError;

  Eigen::Vector3d mLastCOM;

  size_t mLastSupportVersion;

  math::LinearJacobian mComJacCache;

  math::LinearJacobian mEEJacCache;

  Eigen::JacobiSVD<math::LinearJacobian> mSVDCache;

  Eigen::MatrixXd mNullSpaceCache;

  Eigen::MatrixXd mPartialNullSpaceCache;
};

}
}

#endif // DART_CONSTRAINT_BALANCECONSTRAINT_H_
