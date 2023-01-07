/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_CONSTRAINT_DYNAMICJOINTCONSTRAINT_HPP_
#define DART_CONSTRAINT_DYNAMICJOINTCONSTRAINT_HPP_

#include "dart/dynamics/ConstraintBase.hpp"
#include "dart/dynamics/Export.hpp"

namespace dart {

namespace dynamics {
class BodyNode;
} // namespace dynamics

namespace dynamics {

/// Base class for joint constraints that are created or destructed dynamically
/// during simulation.
class DART_DYNAMICS_API DynamicJointConstraint : public ConstraintBase
{
public:
  /// Contructor
  explicit DynamicJointConstraint(dynamics::BodyNode* body);

  /// Contructor
  DynamicJointConstraint(dynamics::BodyNode* body1, dynamics::BodyNode* body2);

  /// Destructor
  ~DynamicJointConstraint() override;

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

  /// Get the first BodyNode that this constraint is associated with
  dynamics::BodyNode* getBodyNode1() const;

  /// Get the second BodyNode that this constraint is associated with
  dynamics::BodyNode* getBodyNode2() const;

protected:
  /// First body node
  dynamics::BodyNode* mBodyNode1;

  /// Second body node
  dynamics::BodyNode* mBodyNode2;

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

} // namespace dynamics
} // namespace dart

#endif // DART_CONSTRAINT_DYNAMICJOINTCONSTRAINT_HPP_
