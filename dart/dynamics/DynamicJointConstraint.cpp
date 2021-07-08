/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/dynamics/DynamicJointConstraint.hpp"

#include <cassert>

#include "dart/common/Console.hpp"

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM 1e-9

namespace dart {
namespace dynamics {

double DynamicJointConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
double DynamicJointConstraint::mErrorReductionParameter = DART_ERP;
double DynamicJointConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double DynamicJointConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
DynamicJointConstraint::DynamicJointConstraint(dynamics::BodyNode* body)
  : ConstraintBase(), mBodyNode1(body), mBodyNode2(nullptr)
{
  assert(body);
}

//==============================================================================
DynamicJointConstraint::DynamicJointConstraint(
    dynamics::BodyNode* body1, dynamics::BodyNode* body2)
  : ConstraintBase(), mBodyNode1(body1), mBodyNode2(body2)
{
  assert(body1);
  assert(body2);
}

//==============================================================================
DynamicJointConstraint::~DynamicJointConstraint()
{
  // Do nothing
}

//==============================================================================
void DynamicJointConstraint::setErrorAllowance(double allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (allowance < 0.0)
  {
    dtwarn << "Error reduction parameter[" << allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = allowance;
}

//==============================================================================
double DynamicJointConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void DynamicJointConstraint::setErrorReductionParameter(double erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (erp < 0.0)
  {
    dtwarn << "Error reduction parameter [" << erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (erp > 1.0)
  {
    dtwarn << "Error reduction parameter [" << erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = erp;
}

//==============================================================================
double DynamicJointConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void DynamicJointConstraint::setMaxErrorReductionVelocity(double erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity [" << erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = erv;
}

//==============================================================================
double DynamicJointConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void DynamicJointConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter [" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double DynamicJointConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
dynamics::BodyNode* DynamicJointConstraint::getBodyNode1() const
{
  return mBodyNode1;
}

//==============================================================================
dynamics::BodyNode* DynamicJointConstraint::getBodyNode2() const
{
  return mBodyNode2;
}

} // namespace dynamics
} // namespace dart
