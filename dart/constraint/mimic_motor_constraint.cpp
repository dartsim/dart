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

#include "dart/constraint/mimic_motor_constraint.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <iostream>

#include <cmath>

namespace {

constexpr inline double kConstraintForceMixing = 1e-6;
constexpr inline double kDefaultForceLimit = 800.0;
constexpr inline double kDefaultVelocityLimit = 50.0;
constexpr inline double kDefaultErp = 0.4;

} // namespace

namespace dart {
namespace constraint {

double MimicMotorConstraint::mConstraintForceMixing = kConstraintForceMixing;
double MimicMotorConstraint::mErrorReductionParameter = kDefaultErp;

//==============================================================================
MimicMotorConstraint::MimicMotorConstraint(
    dynamics::Joint* joint,
    std::span<const dynamics::MimicDofProperties> mimicDofProperties)
  : ConstraintBase(),
    mJoint(joint),
    mMimicProps(mimicDofProperties.begin(), mimicDofProperties.end()),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  DART_ASSERT(joint);
  DART_ASSERT(joint->getNumDofs() <= mMimicProps.size());
  DART_ASSERT(mBodyNode);

  mLifeTime[0] = 0;
  mLifeTime[1] = 0;
  mLifeTime[2] = 0;
  mLifeTime[3] = 0;
  mLifeTime[4] = 0;
  mLifeTime[5] = 0;

  mActive[0] = false;
  mActive[1] = false;
  mActive[2] = false;
  mActive[3] = false;
  mActive[4] = false;
  mActive[5] = false;
}

//==============================================================================
MimicMotorConstraint::~MimicMotorConstraint()
{
  // Do nothing
}

//==============================================================================
std::string_view MimicMotorConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view MimicMotorConstraint::getStaticType()
{
  static constexpr std::string_view name = "MimicMotorConstraint";
  return name;
}

//==============================================================================
void MimicMotorConstraint::setConstraintForceMixing(double cfm)
{
  double clamped = cfm;
  if (clamped < 1e-9) {
    DART_WARN(
        "Constraint force mixing parameter[{}] is lower than 1e-9. It is set "
        "to 1e-9.",
        cfm);
    clamped = 1e-9;
  }

  mConstraintForceMixing = clamped;
}

//==============================================================================
double MimicMotorConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void MimicMotorConstraint::setErrorReductionParameter(double erp)
{
  double clamped = erp;
  if (clamped < 0.0) {
    DART_WARN(
        "Error reduction parameter [{}] is lower than 0.0. It is set to 0.0.",
        erp);
    clamped = 0.0;
  } else if (clamped > 1.0) {
    DART_WARN(
        "Error reduction parameter [{}] is greater than 1.0. It is set to 1.0.",
        erp);
    clamped = 1.0;
  }

  mErrorReductionParameter = clamped;
}

//==============================================================================
double MimicMotorConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void MimicMotorConstraint::update()
{
  // Reset dimension
  mDim = 0;

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    const auto& mimicProp = mMimicProps[i];

    if (mJoint->getActuatorType(i) != dynamics::Joint::MIMIC
        || mimicProp.mReferenceJoint == nullptr) {
      mActive[i] = false;
      continue;
    }

    double timeStep = mJoint->getSkeleton()->getTimeStep();
    double velLower = mJoint->getVelocityLowerLimit(i);
    double velUpper = mJoint->getVelocityUpperLimit(i);
    if (!std::isfinite(velLower))
      velLower = -kDefaultVelocityLimit;
    if (!std::isfinite(velUpper))
      velUpper = kDefaultVelocityLimit;
    double qError
        = mimicProp.mReferenceJoint->getPosition(mimicProp.mReferenceDofIndex)
              * mimicProp.mMultiplier
          + mimicProp.mOffset - mJoint->getPosition(i);
    const double erp = mErrorReductionParameter;
    double desiredVelocity
        = math::clip((erp * qError) / timeStep, velLower, velUpper);

    mNegativeVelocityError[i] = desiredVelocity - mJoint->getVelocity(i);

    if (mNegativeVelocityError[i] != 0.0) {
      // Note that we are computing impulse not force
      double upper = mJoint->getForceUpperLimit(i);
      double lower = mJoint->getForceLowerLimit(i);
      if (!std::isfinite(upper))
        upper = kDefaultForceLimit;
      if (!std::isfinite(lower))
        lower = -kDefaultForceLimit;

      mUpperBound[i] = upper * timeStep;
      mLowerBound[i] = lower * timeStep;

      if (mActive[i]) {
        ++(mLifeTime[i]);
      } else {
        mActive[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
    } else {
      mActive[i] = false;
    }
  }
}

//==============================================================================
void MimicMotorConstraint::getInformation(ConstraintInfo* lcp)
{
  std::size_t index = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    DART_ASSERT(lcp->w[index] == 0.0);

    lcp->b[index] = mNegativeVelocityError[i];
    lcp->lo[index] = mLowerBound[i];
    lcp->hi[index] = mUpperBound[i];

    DART_ASSERT(lcp->findex[index] == -1);

    if (mLifeTime[i])
      lcp->x[index] = mOldX[i];
    else
      lcp->x[index] = 0.0;

    index++;
  }
}

//==============================================================================
void MimicMotorConstraint::applyUnitImpulse(std::size_t index)
{
  DART_ASSERT(index < mDim && "Invalid Index.");

  std::size_t localIndex = 0;
  const dynamics::SkeletonPtr& skeleton = mJoint->getSkeleton();

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (localIndex == index) {
      skeleton->clearConstraintImpulses();
      mJoint->setConstraintImpulse(i, 1.0);
      skeleton->updateBiasImpulse(mBodyNode);
      skeleton->updateVelocityChange();
      mJoint->setConstraintImpulse(i, 0.0);
    }

    ++localIndex;
  }

  mAppliedImpulseIndex = index;
}

//==============================================================================
void MimicMotorConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  DART_ASSERT(delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (mJoint->getSkeleton()->isImpulseApplied())
      delVel[localIndex] = mJoint->getVelocityChange(i);
    else
      delVel[localIndex] = 0.0;

    ++localIndex;
  }

  // Add small values to diagnal to keep it away from singular, similar to cfm
  // variable in ODE
  if (withCfm) {
    delVel[mAppliedImpulseIndex]
        += delVel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }

  DART_ASSERT(localIndex == mDim);
}

//==============================================================================
void MimicMotorConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void MimicMotorConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void MimicMotorConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    mJoint->setConstraintImpulse(
        i, mJoint->getConstraintImpulse(i) + lambda[localIndex]);
    // TODO(JS): consider to add Joint::addConstraintImpulse()

    mOldX[i] = lambda[localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::SkeletonPtr MimicMotorConstraint::getRootSkeleton() const
{
  return ConstraintBase::getRootSkeleton(mJoint->getSkeleton()->getSkeleton());
}

//==============================================================================
bool MimicMotorConstraint::isActive() const
{
  const std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mJoint->getActuatorType(i) == dynamics::Joint::MIMIC
        && i < mMimicProps.size()
        && mMimicProps[i].mReferenceJoint != nullptr) {
      return true;
    }
  }
  return false;
}

} // namespace constraint
} // namespace dart
