/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_CONSTRAINT_CONTACTCONSTRAINT_HPP_
#define DART_CONSTRAINT_CONTACTCONSTRAINT_HPP_

#include "dart/collision/CollisionDetector.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ContactSurface.hpp"
#include "dart/math/MathTypes.hpp"

namespace dart {

namespace dynamics {
class BodyNode;
class Skeleton;
} // namespace dynamics

namespace constraint {

/// ContactConstraint represents a contact constraint between two bodies
class ContactConstraint : public ConstraintBase
{
public:
  /// Constructor
  ContactConstraint(
      collision::Contact& contact,
      double timeStep,
      const ContactSurfaceParams& contactSurfaceParams);

  /// Constructor
  ContactConstraint(collision::Contact& contact, double timeStep);

  /// Destructor
  ~ContactConstraint() override = default;

  // Documentation inherited
  const std::string& getType() const override;

  /// Returns constraint type for this class.
  static const std::string& getStaticType();

  //----------------------------------------------------------------------------
  // Property settings
  //----------------------------------------------------------------------------

  /// Set global error reduction parameter
  static void setErrorAllowance(double allowance);

  /// Get global error reduction parameter
  static double getErrorAllowance();

  /// Set global error reduction parameter
  static void setErrorReductionParameter(double erp);

  /// Get global error reduction parameter
  static double getErrorReductionParameter();

  /// Set global error reduction parameter
  static void setMaxErrorReductionVelocity(double erv);

  /// Get global error reduction parameter
  static double getMaxErrorReductionVelocity();

  /// Set global constraint force mixing parameter
  static void setConstraintForceMixing(double cfm);

  /// Get global constraint force mixing parameter
  static double getConstraintForceMixing();

  /// Set first frictional direction
  void setFrictionDirection(const Eigen::Vector3d& dir);

  /// Get first frictional direction
  const Eigen::Vector3d& getFrictionDirection1() const;

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;
  friend class ConstrainedGroup;
  friend class DefaultContactSurfaceHandler;

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* info) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t index) override;

  // Documentation inherited
  void getVelocityChange(double* vel, bool withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(double* lambda) override;

  // Documentation inherited
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  void uniteSkeletons() override;

  // Documentation inherited
  bool isActive() const override;

  DART_DEPRECATED(6.13)
  static double computeFrictionCoefficient(
      const dynamics::ShapeNode* shapeNode);
  DART_DEPRECATED(6.13)
  static double computePrimaryFrictionCoefficient(
      const dynamics::ShapeNode* shapeNode);
  DART_DEPRECATED(6.13)
  static double computeSecondaryFrictionCoefficient(
      const dynamics::ShapeNode* shapeNode);
  DART_DEPRECATED(6.13)
  static double computePrimarySlipCompliance(
      const dynamics::ShapeNode* shapeNode);
  DART_DEPRECATED(6.13)
  static double computeSecondarySlipCompliance(
      const dynamics::ShapeNode* shapeNode);
  DART_DEPRECATED(6.13)
  static Eigen::Vector3d computeWorldFirstFrictionDir(
      const dynamics::ShapeNode* shapenode);
  DART_DEPRECATED(6.13)
  static double computeRestitutionCoefficient(
      const dynamics::ShapeNode* shapeNode);

private:
  using TangentBasisMatrix = Eigen::Matrix<double, 3, 2>;

  /// Get change in relative velocity at contact point due to external impulse
  /// \param[out] relVel Change in relative velocity at contact point of the
  /// two colliding bodies.
  void getRelVelocity(double* relVel);

  ///
  void updateFirstFrictionalDirection();

  ///
  TangentBasisMatrix getTangentBasisMatrixODE(const Eigen::Vector3d& n);

  // The following functions for getting and setting slip compliance and
  // accessing the contact object are meant to be used by ConstraintSolver to
  // update the slip compliances based on the number of contacts between the
  // collision objects.
  //
  /// Get primary slip compliance
  double getPrimarySlipCompliance() const;

  /// Set primary slip compliance
  void setPrimarySlipCompliance(double slip);

  /// Get secondary slip compliance
  double getSecondarySlipCompliance() const;

  /// Set secondary slip compliance
  void setSecondarySlipCompliance(double slip);

  /// Get contact object associated witht this constraint
  const collision::Contact& getContact() const;

private:
  /// Time step
  double mTimeStep;

  /// Fircst body node
  dynamics::BodyNode* mBodyNodeA;

  /// Second body node
  dynamics::BodyNode* mBodyNodeB;

  /// Contact between mBodyNode1 and mBodyNode2
  collision::Contact& mContact;

  /// First frictional direction
  Eigen::Vector3d mFirstFrictionalDirection;

  /// Primary Coefficient of Friction
  double mPrimaryFrictionCoeff;

  /// Secondary Coefficient of Friction
  double mSecondaryFrictionCoeff;

  /// Primary Coefficient of Slip Compliance
  double mPrimarySlipCompliance;

  /// Secondary Coefficient of Slip Compliance
  double mSecondarySlipCompliance;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Velocity of the contact independent of friction
  /// x = vel. in direction of contact normal
  /// y = vel. in first friction direction
  /// z = vel. in second friction direction
  Eigen::Vector3d mContactSurfaceMotionVelocity;

  /// Whether this contact is self-collision.
  bool mIsSelfCollision;

  /// Local body jacobians for mBodyNode1
  Eigen::Matrix<double, 6, Eigen::Dynamic> mSpatialNormalA;

  /// Local body jacobians for mBodyNode2
  Eigen::Matrix<double, 6, Eigen::Dynamic> mSpatialNormalB;

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
// TODO(JS): Create SelfContactConstraint.

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_CONTACTCONSTRAINT_HPP_
