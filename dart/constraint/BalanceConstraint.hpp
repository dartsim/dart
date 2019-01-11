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

#ifndef DART_CONSTRAINT_BALANCECONSTRAINT_HPP_
#define DART_CONSTRAINT_BALANCECONSTRAINT_HPP_

#include "dart/dynamics/HierarchicalIK.hpp"

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

  /// Constructor
  BalanceConstraint(const std::shared_ptr<dynamics::HierarchicalIK>& _ik,
                    BalanceMethod_t _balanceMethod = SHIFT_SUPPORT,
                    ErrorMethod_t _errorMethod = FROM_CENTROID);

  /// Virtual destructor
  virtual ~BalanceConstraint() = default;

  // Documentation inherited
  optimizer::FunctionPtr clone(
      const std::shared_ptr<dynamics::HierarchicalIK>& _newIK) const override;

  // Documentation inherited
  double eval(const Eigen::VectorXd& _x) const override;

  // Documentation inherited
  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) const override;

  /// Set the method that this constraint function will use to compute the
  /// error. See the ErrorMethod_t docs for more information.
  void setErrorMethod(ErrorMethod_t _method);

  /// Get the method that this constraint function will use to compute the
  /// error.
  ErrorMethod_t getErrorMethod() const;

  /// Set the method that this constraint function will use to achieve balance.
  /// See the BalanceMethod_t docs for more information.
  void setBalanceMethod(BalanceMethod_t _method);

  /// Get the method that this constraint function will use to achieve balance.
  BalanceMethod_t getBalanceMethod() const;

  /// Set the tolerance for how far the center of mass can be from the support
  /// polygon centroid before the balance is considered optimized.
  void setOptimizationTolerance(double _tol);

  /// Get the tolerance for how far the center of mass can be from the support
  /// polygon centroid before the balance is considered optimized.
  double getOptimizationTolerance() const;

  /// Set the damping factor that will be used when computing the pseudoinverse
  void setPseudoInverseDamping(double _damping);

  /// Get the damping factor that will be used when computing the pseudoinverse
  double getPseudoInverseDamping() const;

  /// Get the last error vector that was computed by this BalanceConstraint
  const Eigen::Vector3d& getLastError() const;

  /// Clear the caches to force the error computation to update. It should not
  /// generally be necessary to call this function.
  void clearCaches();

protected:

  /// Convert the gradient that gets generated via Jacobian methods into a
  /// gradient that can be used by a GradientDescentSolver.
  void convertJacobianMethodOutputToGradient(
      Eigen::Map<Eigen::VectorXd>& grad) const;

  /// Pointer to the hierarchical IK that owns this Function. Note that this
  /// Function does not work correctly without a HierarchicalIK.
  std::weak_ptr<dynamics::HierarchicalIK> mIK;

  /// The method that will be used to compute error
  ErrorMethod_t mErrorMethod;

  /// The method that will be used for balance
  BalanceMethod_t mBalanceMethod;

  /// The tolerance allowed before optimization is considered successful
  double mOptimizationTolerance;

  /// The damping factor for the pseudoinverse
  double mDamping;

  /// The indices of the supporting end effectors that are closest to the center
  /// of mass. These are used when using FROM_EDGE
  mutable std::size_t mClosestEndEffector[2];

  /// The error vector points away from the direction that the center of mass
  /// should move in order to reduce the balance error
  mutable Eigen::Vector3d mLastError;

  /// The last computed location of the center of mass
  Eigen::Vector3d mLastCOM;

  /// The last version of the support polygon that was computed
  std::size_t mLastSupportVersion;

  /// Cache for the center of mass Jacobian so that the memory space does not
  /// need to be reallocated each loop
  mutable math::LinearJacobian mComJacCache;

  /// Cache for the end effector Jacobians so the space does not need to be
  /// reallocated each loop
  mutable math::LinearJacobian mEEJacCache;

  /// Cache for the SVD
  mutable Eigen::JacobiSVD<math::LinearJacobian> mSVDCache;

  /// Cache for the full null space
  mutable Eigen::MatrixXd mNullSpaceCache;

  /// Cache for an individual null space
  mutable Eigen::MatrixXd mPartialNullSpaceCache;

  /// Cache used by convertJacobianMethodOutputToGradient to avoid reallocating
  /// this vector on each iteration.
  mutable Eigen::VectorXd mInitialPositionsCache;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_BALANCECONSTRAINT_HPP_
