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

#ifndef DART_CONSTRAINT_SOFTCONTACTCONSTRAINT_HPP_
#define DART_CONSTRAINT_SOFTCONTACTCONSTRAINT_HPP_

#include "dart/constraint/ConstraintBase.hpp"

#include "dart/math/MathTypes.hpp"
#include "dart/collision/CollisionDetector.hpp"

namespace dart {

namespace collision{
class SoftCollisionInfo;
}  // namespace collision

namespace dynamics {
class BodyNode;
class SoftBodyNode;
class PointMass;
class Skeleton;
}  // namespace dynamics

namespace constraint {

/// SoftContactConstraint represents a contact constraint between two bodies
class SoftContactConstraint : public ConstraintBase
{
public:
  /// Constructor
  SoftContactConstraint(collision::Contact& _contact, double _timeStep);

  /// Destructor
  virtual ~SoftContactConstraint();

  //----------------------------------------------------------------------------
  // Property settings
  //----------------------------------------------------------------------------

  /// Set global error reduction parameter
  static void setErrorAllowance(double _allowance);

  /// Get global error reduction parameter
  static double getErrorAllowance();

  /// Set global error reduction parameter
  static void setErrorReductionParameter(double _erp);

  /// Get global error reduction parameter
  static double getErrorReductionParameter();

  /// Set global error reduction parameter
  static void setMaxErrorReductionVelocity(double _erv);

  /// Get global error reduction parameter
  static double getMaxErrorReductionVelocity();

  /// Set global constraint force mixing parameter
  static void setConstraintForceMixing(double _cfm);

  /// Get global constraint force mixing parameter
  static double getConstraintForceMixing();

  /// Set first frictional direction
  void setFrictionDirection(const Eigen::Vector3d& _dir);

  /// Get first frictional direction
  const Eigen::Vector3d& getFrictionDirection1() const;

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;
  friend class ConstrainedGroup;

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* _info) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t _idx) override;

  // Documentation inherited
  void getVelocityChange(double* _vel, bool _withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(double* _lambda) override;

  // Documentation inherited
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  void uniteSkeletons() override;

  // Documentation inherited
  bool isActive() const override;

private:
  /// Get change in relative velocity at contact point due to external impulse
  /// \param[out] _vel Change in relative velocity at contact point of the two
  ///                  colliding bodies
  void getRelVelocity(double* _vel);

  ///
  void updateFirstFrictionalDirection();

  ///
  Eigen::MatrixXd getTangentBasisMatrixODE(const Eigen::Vector3d& _n);

  /// Find the nearest point mass from _point in a face, of which id is _faceId
  /// in _softBodyNode.
  dynamics::PointMass* selectCollidingPointMass(
      dynamics::SoftBodyNode* _softBodyNode,
      const Eigen::Vector3d& _point,
      int _faceId) const;

  /// Find the nearest point mass from _point in a face, of which id is _faceId
  /// in _softBodyNode. Returns a pointer to a const, and is usable with a const
  /// SoftBodyNode.
  const dynamics::PointMass* selectCollidingPointMass(
      const dynamics::SoftBodyNode* _softBodyNode,
      const Eigen::Vector3d& _point,
      int _faceId) const;

private:
  /// Time step
  double mTimeStep;

  /// Fircst body node
  dynamics::BodyNode* mBodyNode1;

  /// Second body node
  dynamics::BodyNode* mBodyNode2;

  /// First soft body node
  dynamics::SoftBodyNode* mSoftBodyNode1;

  /// Second soft body node
  dynamics::SoftBodyNode* mSoftBodyNode2;

  /// First point mass
  dynamics::PointMass* mPointMass1;

  /// Second point mass
  dynamics::PointMass* mPointMass2;

  // TODO(JS): For now, there is only one contact per contact constraint
  /// Contacts between mBodyNode1 and mBodyNode2
  std::vector<collision::Contact*> mContacts;

  /// Soft collision information
  collision::SoftCollisionInfo* mSoftCollInfo;

  /// First frictional direction
  Eigen::Vector3d mFirstFrictionalDirection;

  /// Coefficient of Friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Local body jacobians for mBodyNode1
  common::aligned_vector<Eigen::Vector6d> mJacobians1;

  /// Local body jacobians for mBodyNode2
  common::aligned_vector<Eigen::Vector6d> mJacobians2;

  /// Contact normal expressed in body frame of the first body node
  Eigen::Vector3d mBodyDirection1;

  /// Contact normal expressed in body frame of the second body node
  Eigen::Vector3d mBodyDirection2;

  ///
  bool mIsFrictionOn;

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

  ///
  bool mIsBounceOn;

  ///
  bool mActive;

  /// Global constraint error allowance
  static double mErrorAllowance;

  /// Global constraint error redection parameter in the range of [0, 1]. The
  /// default is 0.01.
  static double mErrorReductionParameter;

  /// Maximum error reduction velocity
  static double mMaxErrorReductionVelocity;

  /// Global constraint force mixing parameter in the range of [1e-9, 1]. The
  /// default is 1e-5
  /// \sa http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
  static double mConstraintForceMixing;
};

} // namespace constraint
} // namespace dart

#endif  // DART_CONSTRAINT_SOFTCONTACTCONSTRAINT_HPP_

