/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_
#define DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_

#include <dart/constraint/constraint_base.hpp>

#include <dart/dynamics/fwd.hpp>
#include <dart/dynamics/mimic_dof_properties.hpp>

#include <dart/export.hpp>

#include <span>
#include <vector>

namespace dart {
namespace constraint {

/// CouplerConstraint enforces the mimic relationship between two joints by
/// applying equal-and-opposite impulses based on MimicDofProperties. Unlike
/// MimicMotorConstraint, which behaves like a servo that only drives the
/// dependent joint, CouplerConstraint is bilateral: both the reference and
/// dependent joints participate in the constraint solve, so any impulse on one
/// joint is reflected on the other joint.
class DART_API CouplerConstraint : public ConstraintBase
{
public:
  /// Constructor that creates a CouplerConstraint using the given
  /// MimicDofProperties for each dependent joint's DoF.
  /// @param[in] joint The dependent joint.
  /// @param[in] mimicDofProperties A span of MimicDofProperties for each DoF of
  /// the dependent joint.
  explicit CouplerConstraint(
      dynamics::Joint* joint,
      std::span<const dynamics::MimicDofProperties> mimicDofProperties);

  /// Destructor
  ~CouplerConstraint() override;

  // Documentation inherited
  std::string_view getType() const override;

  /// Returns constraint type for this class.
  static std::string_view getStaticType();

  /// Set global constraint force mixing parameter
  static void setConstraintForceMixing(double cfm);

  /// Get global constraint force mixing parameter
  static double getConstraintForceMixing();

  // Friendship
  friend class ConstraintSolver;
  friend class ConstrainedGroup;

protected:
  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* lcp) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t index) override;

  // Documentation inherited
  void getVelocityChange(double* delVel, bool withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(double* lambda) override;

  // Documentation inherited
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  bool isActive() const override;

  // Documentation inherited
  void uniteSkeletons() override;

private:
  /// Dependent joint whose motion is influenced by the reference joint.
  dynamics::Joint* mJoint;

  /// Vector of MimicDofProperties for the dependent joint.
  std::vector<dynamics::MimicDofProperties> mMimicProps;

  /// BodyNode associated with the dependent joint.
  dynamics::BodyNode* mBodyNode;

  /// Index of the applied impulse for the dependent joint.
  std::size_t mAppliedImpulseIndex;

  /// Array storing the lifetime of each constraint (in iterations).
  std::size_t mLifeTime[6];

  /// Array indicating whether each constraint is active or not.
  bool mActive[6];

  /// Array storing the negative velocity errors for each constraint.
  double mNegativeVelocityError[6];

  /// Array storing the previous values of the constraint forces.
  double mOldX[6];

  /// Array storing the upper bounds for the constraint forces.
  double mUpperBound[6];

  /// Array storing the lower bounds for the constraint forces.
  double mLowerBound[6];

  /// Global constraint force mixing parameter in the range of [1e-9, 1]. The
  /// default is 1e-6
  /// @sa http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
  static double mConstraintForceMixing;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_
