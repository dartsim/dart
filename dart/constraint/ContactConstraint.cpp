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

#include "dart/constraint/ContactConstraint.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/external/odelcpsolver/lcp.h"
#include "dart/math/Helpers.hpp"

#include <iostream>

#define DART_EPSILON 1e-6
#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e-3
#define DART_CFM 1e-5
// #define DART_MAX_NUMBER_OF_CONTACTS 32

namespace dart {
namespace constraint {

namespace {

//==============================================================================
dynamics::BodyNode* getContactBodyNode(
    const collision::CollisionObject* collisionObject)
{
  if (collisionObject == nullptr)
    return nullptr;

  return collisionObject->getBodyNode();
}

//==============================================================================
bool canSkipContactRelativeVelocity(
    const dynamics::BodyNode* bodyNode,
    const dynamics::Skeleton* skeleton,
    bool isReactive)
{
  if (isReactive || bodyNode == nullptr || skeleton == nullptr)
    return false;

  return bodyNode->getNumDependentGenCoords() == 0u;
}

//==============================================================================
bool isContactBodyNodeReactive(
    const dynamics::BodyNode* bodyNode, const dynamics::Skeleton* skeleton)
{
  if (bodyNode == nullptr || skeleton == nullptr || !skeleton->isMobile()
      || bodyNode->getNumDependentGenCoords() == 0u) {
    return false;
  }

  auto* ancestorBody = bodyNode;
  while (ancestorBody != nullptr) {
    const auto* parentJoint = ancestorBody->getParentJoint();
    if (parentJoint != nullptr && parentJoint->isDynamic())
      return true;

    ancestorBody = ancestorBody->getParentBodyNode();
  }

  return false;
}

} // namespace

double ContactConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
double ContactConstraint::mErrorReductionParameter = DART_ERP;
double ContactConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double ContactConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
ContactConstraint::ContactConstraint(
    collision::Contact& contact, double timeStep)
  : ContactConstraint(
      contact,
      timeStep,
      DefaultContactSurfaceHandler().createParams(contact, 1u))
{
  // Do nothing
}

//==============================================================================
ContactConstraint::ContactConstraint(
    collision::Contact& contact,
    double timeStep,
    const ContactSurfaceParams& contactSurfaceParams)
  : ConstraintBase(),
    mTimeStep(timeStep),
    mBodyNodeA(getContactBodyNode(contact.collisionObject1)),
    mBodyNodeB(getContactBodyNode(contact.collisionObject2)),
    mContact(contact),
    mFirstFrictionalDirection(DART_DEFAULT_FRICTION_DIR),
    mPrimaryFrictionCoeff(DART_DEFAULT_FRICTION_COEFF),
    mSecondaryFrictionCoeff(DART_DEFAULT_FRICTION_COEFF),
    mPrimarySlipCompliance(DART_DEFAULT_SLIP_COMPLIANCE),
    mSecondarySlipCompliance(DART_DEFAULT_SLIP_COMPLIANCE),
    mRestitutionCoeff(DART_DEFAULT_RESTITUTION_COEFF),
    mContactSurfaceMotionVelocity(DART_DEFAULT_CONTACT_SURFACE_MOTION_VELOCITY),
    mIsSelfCollision(false),
    mIsFrictionOn(true),
    mAppliedImpulseIndex(dynamics::INVALID_INDEX),
    mIsBounceOn(false),
    mActive(false)
{
  if (!hasValidBodyNodes()) {
    dtwarn << "[ContactConstraint] Ignoring contact with a null collision "
           << "object or missing ShapeNode.\n";
    return;
  }

  // Resolve skeleton pointers and reactivity once; the LCP-assembly hot path
  // reads these cached values instead of repeatedly calling getSkeleton()
  // (shared_ptr churn) and isReactive() (ancestor-joint walk). See header.
  mSkeletonA = mBodyNodeA->getSkeletonRawPtr();
  mSkeletonB = mBodyNodeB->getSkeletonRawPtr();
  mIsReactiveA = isContactBodyNodeReactive(mBodyNodeA, mSkeletonA);
  mIsReactiveB = isContactBodyNodeReactive(mBodyNodeB, mSkeletonB);
  mSkipRelVelocityA
      = canSkipContactRelativeVelocity(mBodyNodeA, mSkeletonA, mIsReactiveA);
  mSkipRelVelocityB
      = canSkipContactRelativeVelocity(mBodyNodeB, mSkeletonB, mIsReactiveB);

  DART_ASSERT(
      contact.normal.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);

  //----------------------------------------------
  // Bounce
  //----------------------------------------------
  mRestitutionCoeff = contactSurfaceParams.mRestitutionCoeff;
  if (mRestitutionCoeff > DART_RESTITUTION_COEFF_THRESHOLD)
    mIsBounceOn = true;
  else
    mIsBounceOn = false;

  //----------------------------------------------
  // Friction
  //----------------------------------------------
  mPrimaryFrictionCoeff = contactSurfaceParams.mPrimaryFrictionCoeff;
  mSecondaryFrictionCoeff = contactSurfaceParams.mSecondaryFrictionCoeff;
  if (mPrimaryFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD
      || mSecondaryFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD) {
    mIsFrictionOn = true;

    // Combine slip compliances through addition
    mPrimarySlipCompliance = contactSurfaceParams.mPrimarySlipCompliance;
    mSecondarySlipCompliance = contactSurfaceParams.mSecondarySlipCompliance;

    mFirstFrictionalDirection = contactSurfaceParams.mFirstFrictionalDirection;

    // Update frictional direction
    updateFirstFrictionalDirection();
  } else {
    mIsFrictionOn = false;
  }

  mContactSurfaceMotionVelocity
      = contactSurfaceParams.mContactSurfaceMotionVelocity;

  DART_ASSERT(mSkeletonA != nullptr);
  DART_ASSERT(mSkeletonB != nullptr);
  mIsSelfCollision = (mSkeletonA == mSkeletonB);
  if (!mIsSelfCollision && mIsReactiveA != mIsReactiveB) {
    mSingleReactiveBodyNode = mIsReactiveA ? mBodyNodeA : mBodyNodeB;
    mSingleReactiveSkeleton = mIsReactiveA ? mSkeletonA : mSkeletonB;
  }

  // Compute local contact Jacobians expressed in body frame
  if (mIsFrictionOn) {
    // Set the dimension of this constraint. 1 is for Normal direction
    // constraint.
    // TODO(JS): Assumed that the number of contact is not static.
    // TODO(JS): Adjust following code once use of mNumFrictionConeBases is
    //           implemented.
    //  mDim = mContacts.size() * (1 + mNumFrictionConeBases);
    mDim = 3;

#ifndef NDEBUG
    // Release readers skip these sides through mSkipRelVelocity*/mIsReactive*.
    // Keep debug zero-fill so the NaN assertions can inspect both matrices.
    if (mSkipRelVelocityA)
      mSpatialNormalA.setZero();
    if (mSkipRelVelocityB)
      mSpatialNormalB.setZero();
#endif

    Eigen::Vector3d bodyDirectionA;
    Eigen::Vector3d bodyDirectionB;

    Eigen::Vector3d bodyPointA;
    Eigen::Vector3d bodyPointB;

    collision::Contact& ct = mContact;

    // TODO(JS): Assumed that the number of tangent basis is 2.
    mTangentBasis = getTangentBasisMatrixODE(ct.normal);
    const TangentBasisMatrix& D = mTangentBasis;

    DART_ASSERT(std::abs(ct.normal.dot(D.col(0))) < DART_EPSILON);
    DART_ASSERT(std::abs(ct.normal.dot(D.col(1))) < DART_EPSILON);
    DART_ASSERT(std::abs(D.col(0).dot(D.col(1))) < DART_EPSILON);

    if (!mSkipRelVelocityA) {
      const Eigen::Isometry3d& tfA = mBodyNodeA->getWorldTransform();
      const Eigen::Matrix3d rotationA = tfA.linear().transpose();
      const Eigen::Isometry3d tfAInv = tfA.inverse();
      bodyPointA.noalias() = tfAInv * ct.point;

      // Jacobian for normal contact
      bodyDirectionA.noalias() = rotationA * ct.normal;
      mSpatialNormalA.col(0).head<3>().noalias()
          = bodyPointA.cross(bodyDirectionA);
      mSpatialNormalA.col(0).tail<3>() = bodyDirectionA;

      // Jacobian for directional friction 1
      bodyDirectionA.noalias() = rotationA * D.col(0);
      mSpatialNormalA.col(1).head<3>().noalias()
          = bodyPointA.cross(bodyDirectionA);
      mSpatialNormalA.col(1).tail<3>() = bodyDirectionA;

      // Jacobian for directional friction 2
      bodyDirectionA.noalias() = rotationA * D.col(1);
      mSpatialNormalA.col(2).head<3>().noalias()
          = bodyPointA.cross(bodyDirectionA);
      mSpatialNormalA.col(2).tail<3>() = bodyDirectionA;
    }

    if (!mSkipRelVelocityB) {
      const Eigen::Isometry3d& tfB = mBodyNodeB->getWorldTransform();
      const Eigen::Matrix3d rotationB = tfB.linear().transpose();
      const Eigen::Isometry3d tfBInv = tfB.inverse();
      bodyPointB.noalias() = tfBInv * ct.point;

      // Jacobian for normal contact
      bodyDirectionB.noalias() = rotationB * -ct.normal;
      mSpatialNormalB.col(0).head<3>().noalias()
          = bodyPointB.cross(bodyDirectionB);
      mSpatialNormalB.col(0).tail<3>() = bodyDirectionB;

      // Jacobian for directional friction 1
      bodyDirectionB.noalias() = rotationB * -D.col(0);
      mSpatialNormalB.col(1).head<3>().noalias()
          = bodyPointB.cross(bodyDirectionB);
      mSpatialNormalB.col(1).tail<3>() = bodyDirectionB;

      // Jacobian for directional friction 2
      bodyDirectionB.noalias() = rotationB * -D.col(1);
      mSpatialNormalB.col(2).head<3>().noalias()
          = bodyPointB.cross(bodyDirectionB);
      mSpatialNormalB.col(2).tail<3>() = bodyDirectionB;
    }

    DART_ASSERT(!dart::math::isNan(mSpatialNormalA));
    DART_ASSERT(!dart::math::isNan(mSpatialNormalB));
  } else {
    // Set the dimension of this constraint.
    mDim = 1;

#ifndef NDEBUG
    // Release readers skip these sides through mSkipRelVelocity*/mIsReactive*.
    // Keep debug zero-fill so the NaN assertions can inspect both matrices.
    if (mSkipRelVelocityA)
      mSpatialNormalA.setZero();
    if (mSkipRelVelocityB)
      mSpatialNormalB.setZero();
#endif

    collision::Contact& ct = mContact;

    if (!mSkipRelVelocityA) {
      const Eigen::Isometry3d& tfA = mBodyNodeA->getWorldTransform();

      // Contact normal in the local coordinates
      const Eigen::Vector3d bodyDirectionA
          = tfA.linear().transpose() * ct.normal;

      // Contact points in the local coordinates
      const Eigen::Vector3d bodyPointA = tfA.inverse() * ct.point;
      mSpatialNormalA.col(0).head<3>().noalias()
          = bodyPointA.cross(bodyDirectionA);
      mSpatialNormalA.col(0).tail<3>().noalias() = bodyDirectionA;
    }

    if (!mSkipRelVelocityB) {
      const Eigen::Isometry3d& tfB = mBodyNodeB->getWorldTransform();

      // Contact normal in the local coordinates
      const Eigen::Vector3d bodyDirectionB
          = tfB.linear().transpose() * -ct.normal;

      // Contact points in the local coordinates
      const Eigen::Vector3d bodyPointB = tfB.inverse() * ct.point;
      mSpatialNormalB.col(0).head<3>().noalias()
          = bodyPointB.cross(bodyDirectionB);
      mSpatialNormalB.col(0).tail<3>().noalias() = bodyDirectionB;
    }
  }
}

//==============================================================================
const std::string& ContactConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ContactConstraint::getStaticType()
{
  static const std::string name = "ContactConstraint";
  return name;
}

//==============================================================================
void ContactConstraint::setErrorAllowance(double allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (allowance < 0.0) {
    dtwarn << "Error reduction parameter[" << allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = allowance;
}

//==============================================================================
double ContactConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void ContactConstraint::setErrorReductionParameter(double erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (erp < 0.0) {
    dtwarn << "Error reduction parameter[" << erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (erp > 1.0) {
    dtwarn << "Error reduction parameter[" << erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = erp;
}

//==============================================================================
double ContactConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void ContactConstraint::setMaxErrorReductionVelocity(double erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (erv < 0.0) {
    dtwarn << "Maximum error reduction velocity[" << erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = erv;
}

//==============================================================================
double ContactConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void ContactConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9) {
    dtwarn << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double ContactConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void ContactConstraint::setFrictionDirection(const Eigen::Vector3d& dir)
{
  mFirstFrictionalDirection = dir.normalized();
}

//==============================================================================
const Eigen::Vector3d& ContactConstraint::getFrictionDirection1() const
{
  return mFirstFrictionalDirection;
}

//==============================================================================
void ContactConstraint::update()
{
  if (!hasValidBodyNodes()) {
    mActive = false;
    return;
  }

  if (mIsReactiveA || mIsReactiveB)
    mActive = true;
  else
    mActive = false;
}

//==============================================================================
void ContactConstraint::getInformation(ConstraintInfo* info)
{
  if (!hasValidBodyNodes())
    return;

  // Fill w, where the LCP form is Ax = b + w (x >= 0, w >= 0, x^T w = 0)
  getRelVelocity(info->b);

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    // Bias term, w, should be zero
    DART_ASSERT(info->w[0] == 0.0);
    DART_ASSERT(info->w[1] == 0.0);
    DART_ASSERT(info->w[2] == 0.0);

    // Upper and lower bounds of normal impulsive force
    info->lo[0] = 0.0;
    info->hi[0] = static_cast<double>(dInfinity);
    DART_ASSERT(info->findex[0] == -1);

    // Upper and lower bounds of tangential direction-1 impulsive force
    info->lo[1] = -mPrimaryFrictionCoeff;
    info->hi[1] = mPrimaryFrictionCoeff;
    info->findex[1] = 0;

    // Upper and lower bounds of tangential direction-2 impulsive force
    info->lo[2] = -mSecondaryFrictionCoeff;
    info->hi[2] = mSecondaryFrictionCoeff;
    info->findex[2] = 0;

    //------------------------------------------------------------------------
    // Bouncing
    //------------------------------------------------------------------------
    // A. Penetration correction
    double bouncingVelocity = mContact.penetrationDepth - mErrorAllowance;
    if (bouncingVelocity < 0.0) {
      bouncingVelocity = 0.0;
    } else {
      bouncingVelocity *= mErrorReductionParameter * info->invTimeStep;
      if (bouncingVelocity > mMaxErrorReductionVelocity)
        bouncingVelocity = mMaxErrorReductionVelocity;
    }

    // B. Restitution
    if (mIsBounceOn) {
      double& negativeRelativeVel = info->b[0];
      double restitutionVel = negativeRelativeVel * mRestitutionCoeff;

      if (restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD) {
        if (restitutionVel > bouncingVelocity) {
          bouncingVelocity = restitutionVel;

          if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY) {
            bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;
          }
        }
      }
    }

    info->b[0] += bouncingVelocity;
    info->b[0] += mContactSurfaceMotionVelocity.x();
    info->b[1] += mContactSurfaceMotionVelocity.y();
    info->b[2] += mContactSurfaceMotionVelocity.z();

    // TODO(JS): Initial guess
    // x
    info->x[0] = 0.0;
    info->x[1] = 0.0;
    info->x[2] = 0.0;
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    // Bias term, w, should be zero
    info->w[0] = 0.0;

    // Upper and lower bounds of normal impulsive force
    info->lo[0] = 0.0;
    info->hi[0] = static_cast<double>(dInfinity);
    DART_ASSERT(info->findex[0] == -1);

    //------------------------------------------------------------------------
    // Bouncing
    //------------------------------------------------------------------------
    // A. Penetration correction
    double bouncingVelocity = mContact.penetrationDepth - DART_ERROR_ALLOWANCE;
    if (bouncingVelocity < 0.0) {
      bouncingVelocity = 0.0;
    } else {
      bouncingVelocity *= mErrorReductionParameter * info->invTimeStep;
      if (bouncingVelocity > mMaxErrorReductionVelocity)
        bouncingVelocity = mMaxErrorReductionVelocity;
    }

    // B. Restitution
    if (mIsBounceOn) {
      double& negativeRelativeVel = info->b[0];
      double restitutionVel = negativeRelativeVel * mRestitutionCoeff;

      if (restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD) {
        if (restitutionVel > bouncingVelocity) {
          bouncingVelocity = restitutionVel;

          if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY)
            bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;
        }
      }
    }

    info->b[0] += bouncingVelocity;
    info->b[0] += mContactSurfaceMotionVelocity.x();

    // TODO(JS): Initial guess
    // x
    info->x[0] = 0.0;
  }
}

//==============================================================================
void ContactConstraint::applyUnitImpulse(std::size_t index)
{
  if (!hasValidBodyNodes())
    return;

  DART_ASSERT(index < mDim && "Invalid Index.");
  // DART_ASSERT(isActive());
  DART_ASSERT(mIsReactiveA || mIsReactiveB);

  dynamics::Skeleton* skelA = mSkeletonA;
  dynamics::Skeleton* skelB = mSkeletonB;

  // Self collision case
  if (mIsSelfCollision) {
    skelA->clearConstraintImpulses();

    if (mIsReactiveA) {
      // Both bodies are reactive
      if (mIsReactiveB) {
        skelA->updateBiasImpulse(
            mBodyNodeA,
            mSpatialNormalA.col(index),
            mBodyNodeB,
            mSpatialNormalB.col(index));
      }
      // Only body1 is reactive
      else {
        skelA->updateBiasImpulse(mBodyNodeA, mSpatialNormalA.col(index));
      }
    } else {
      // Only body2 is reactive
      if (mIsReactiveB) {
        skelB->updateBiasImpulse(mBodyNodeB, mSpatialNormalB.col(index));
      }
      // Both bodies are not reactive
      else {
        // This case should not be happened
        DART_ASSERT(0);
      }
    }

    skelA->updateVelocityChange();
  }
  // Colliding two distinct skeletons
  else {
    if (mIsReactiveA) {
      skelA->clearConstraintImpulses();
      skelA->updateBiasImpulse(mBodyNodeA, mSpatialNormalA.col(index));
      skelA->updateVelocityChange();
    }

    if (mIsReactiveB) {
      skelB->clearConstraintImpulses();
      skelB->updateBiasImpulse(mBodyNodeB, mSpatialNormalB.col(index));
      skelB->updateVelocityChange();
    }
  }

  mAppliedImpulseIndex = index;
}

//==============================================================================
void ContactConstraint::getVelocityChange(double* vel, bool withCfm)
{
  DART_ASSERT(vel != nullptr && "Null pointer is not allowed.");

  if (!hasValidBodyNodes())
    return;

  if (mDim == 3) {
    Eigen::Map<Eigen::Vector3d> velMap(vel);
    velMap.setZero();

    if (mIsReactiveA && mSkeletonA->isImpulseApplied()) {
      velMap.noalias() += mSpatialNormalA.template leftCols<3>().transpose()
                          * mBodyNodeA->getBodyVelocityChange();
    }

    if (mIsReactiveB && mSkeletonB->isImpulseApplied()) {
      velMap.noalias() += mSpatialNormalB.template leftCols<3>().transpose()
                          * mBodyNodeB->getBodyVelocityChange();
    }
  } else if (mDim == 1) {
    vel[0] = 0.0;

    if (mIsReactiveA && mSkeletonA->isImpulseApplied())
      vel[0] += mSpatialNormalA.col(0).dot(mBodyNodeA->getBodyVelocityChange());

    if (mIsReactiveB && mSkeletonB->isImpulseApplied())
      vel[0] += mSpatialNormalB.col(0).dot(mBodyNodeB->getBodyVelocityChange());
  } else {
    Eigen::Map<Eigen::VectorXd> velMap(vel, static_cast<int>(mDim));
    velMap.setZero();

    if (mIsReactiveA && mSkeletonA->isImpulseApplied())
      velMap.noalias()
          += mSpatialNormalA.leftCols(static_cast<int>(mDim)).transpose()
             * mBodyNodeA->getBodyVelocityChange();

    if (mIsReactiveB && mSkeletonB->isImpulseApplied())
      velMap.noalias()
          += mSpatialNormalB.leftCols(static_cast<int>(mDim)).transpose()
             * mBodyNodeB->getBodyVelocityChange();
  }

  // Add small values to the diagnal to keep it away from singular, similar to
  // cfm variable in ODE
  if (withCfm) {
    vel[mAppliedImpulseIndex]
        += vel[mAppliedImpulseIndex] * mConstraintForceMixing;
    switch (mAppliedImpulseIndex) {
      case 1:
        vel[1] += (mPrimarySlipCompliance / mTimeStep);
        break;
      case 2:
        vel[2] += (mSecondarySlipCompliance / mTimeStep);
        break;
      default:
        break;
    }
  }
}

//==============================================================================
dynamics::BodyNode* ContactConstraint::getSingleReactiveBodyNode() const
{
  return mSingleReactiveBodyNode;
}

//==============================================================================
dynamics::Skeleton* ContactConstraint::getSingleReactiveSkeleton() const
{
  return mSingleReactiveSkeleton;
}

Eigen::Vector6d ContactConstraint::getSpatialNormalForSingleReactiveBody(
    std::size_t index) const
{
  DART_ASSERT(index < mDim && "Invalid Index.");
  DART_ASSERT(getSingleReactiveBodyNode() != nullptr);

  if (mIsReactiveA)
    return mSpatialNormalA.col(index);

  return mSpatialNormalB.col(index);
}

//==============================================================================
void ContactConstraint::getVelocityChangeFromSingleBody(
    const Eigen::Vector6d& bodyVelocityChange,
    double* vel,
    bool withCfm,
    std::size_t appliedImpulseIndex) const
{
  DART_ASSERT(vel != nullptr && "Null pointer is not allowed.");
  DART_ASSERT(appliedImpulseIndex < mDim && "Invalid Index.");
  DART_ASSERT(getSingleReactiveBodyNode() != nullptr);

  const auto& spatialNormal = mIsReactiveA ? mSpatialNormalA : mSpatialNormalB;
  if (mDim == 3) {
    Eigen::Map<Eigen::Vector3d> velMap(vel);
    velMap.noalias()
        = spatialNormal.template leftCols<3>().transpose() * bodyVelocityChange;
  } else if (mDim == 1) {
    vel[0] = spatialNormal.col(0).dot(bodyVelocityChange);
  } else {
    Eigen::Map<Eigen::VectorXd> velMap(vel, static_cast<int>(mDim));
    velMap.noalias() = spatialNormal.transpose() * bodyVelocityChange;
  }

  // Add small values to the diagnal to keep it away from singular, similar to
  // cfm variable in ODE
  if (withCfm) {
    vel[appliedImpulseIndex]
        += vel[appliedImpulseIndex] * mConstraintForceMixing;
    switch (appliedImpulseIndex) {
      case 1:
        vel[1] += (mPrimarySlipCompliance / mTimeStep);
        break;
      case 2:
        vel[2] += (mSecondarySlipCompliance / mTimeStep);
        break;
      default:
        break;
    }
  }
}

//==============================================================================
void ContactConstraint::excite()
{
  if (!hasValidBodyNodes())
    return;

  if (mIsReactiveA)
    mSkeletonA->setImpulseApplied(true);

  if (mIsReactiveB)
    mSkeletonB->setImpulseApplied(true);
}

//==============================================================================
void ContactConstraint::unexcite()
{
  if (!hasValidBodyNodes())
    return;

  if (mIsReactiveA)
    mSkeletonA->setImpulseApplied(false);

  if (mIsReactiveB)
    mSkeletonB->setImpulseApplied(false);
}

//==============================================================================
void ContactConstraint::applyImpulse(double* lambda)
{
  if (!hasValidBodyNodes())
    return;

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    DART_ASSERT(!math::isNan(lambda[0]));
    DART_ASSERT(!math::isNan(lambda[1]));
    DART_ASSERT(!math::isNan(lambda[2]));

    // Store contact impulse (force) toward the normal w.r.t. world frame
    mContact.force = mContact.normal * lambda[0] / mTimeStep;

    // Normal impulsive force
    if (mIsReactiveA)
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(0) * lambda[0]);
    if (mIsReactiveB)
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(0) * lambda[0]);

    // Add contact impulse (force) toward the tangential w.r.t. world frame.
    // Reuse the tangent basis computed at construction (see mTangentBasis).
    const TangentBasisMatrix& D = mTangentBasis;
    mContact.force += D.col(0) * lambda[1] / mTimeStep;

    // Tangential direction-1 impulsive force
    if (mIsReactiveA)
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(1) * lambda[1]);
    if (mIsReactiveB)
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(1) * lambda[1]);

    // Add contact impulse (force) toward the tangential w.r.t. world frame
    mContact.force += D.col(1) * lambda[2] / mTimeStep;

    // Tangential direction-2 impulsive force
    if (mIsReactiveA)
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(2) * lambda[2]);
    if (mIsReactiveB)
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(2) * lambda[2]);
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    // Normal impulsive force
    if (mIsReactiveA)
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(0) * lambda[0]);

    if (mIsReactiveB)
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(0) * lambda[0]);

    // Store contact impulse (force) toward the normal w.r.t. world frame
    mContact.force = mContact.normal * lambda[0] / mTimeStep;
  }
}

//==============================================================================
void ContactConstraint::getRelVelocity(double* relVel)
{
  DART_ASSERT(relVel != nullptr && "Null pointer is not allowed.");

  if (!hasValidBodyNodes())
    return;

  if (mDim == 3) {
    Eigen::Map<Eigen::Vector3d> relVelMap(relVel);
    const bool canUseSingleReactiveRelVelocity
        = mSingleReactiveBodyNode != nullptr
          && ((mIsReactiveA && mSkipRelVelocityB)
              || (mIsReactiveB && mSkipRelVelocityA));
    if (canUseSingleReactiveRelVelocity) {
      const auto& spatialNormal
          = mIsReactiveA ? mSpatialNormalA : mSpatialNormalB;
      relVelMap.noalias() = spatialNormal.template leftCols<3>().transpose()
                            * mSingleReactiveBodyNode->getSpatialVelocity();
      relVelMap *= -1.0;
      return;
    }

    relVelMap.setZero();
    if (!mSkipRelVelocityA) {
      relVelMap.noalias() -= mSpatialNormalA.template leftCols<3>().transpose()
                             * mBodyNodeA->getSpatialVelocity();
    }
    if (!mSkipRelVelocityB) {
      relVelMap.noalias() -= mSpatialNormalB.template leftCols<3>().transpose()
                             * mBodyNodeB->getSpatialVelocity();
    }
    return;
  }

  if (mDim == 1) {
    const bool canUseSingleReactiveRelVelocity
        = mSingleReactiveBodyNode != nullptr
          && ((mIsReactiveA && mSkipRelVelocityB)
              || (mIsReactiveB && mSkipRelVelocityA));
    if (canUseSingleReactiveRelVelocity) {
      const auto& spatialNormal
          = mIsReactiveA ? mSpatialNormalA : mSpatialNormalB;
      relVel[0] = -spatialNormal.col(0).dot(
          mSingleReactiveBodyNode->getSpatialVelocity());
      return;
    }

    relVel[0] = 0.0;
    if (!mSkipRelVelocityA)
      relVel[0] -= mSpatialNormalA.col(0).dot(mBodyNodeA->getSpatialVelocity());
    if (!mSkipRelVelocityB)
      relVel[0] -= mSpatialNormalB.col(0).dot(mBodyNodeB->getSpatialVelocity());
    return;
  }

  Eigen::Map<Eigen::VectorXd> relVelMap(relVel, static_cast<int>(mDim));
  relVelMap.setZero();
  // noalias(): evaluate the dynamic-sized product directly into the caller's
  // buffer instead of a heap-allocated temporary (see getVelocityChange).
  if (!mSkipRelVelocityA)
    relVelMap.noalias()
        -= mSpatialNormalA.leftCols(static_cast<int>(mDim)).transpose()
           * mBodyNodeA->getSpatialVelocity();
  if (!mSkipRelVelocityB)
    relVelMap.noalias()
        -= mSpatialNormalB.leftCols(static_cast<int>(mDim)).transpose()
           * mBodyNodeB->getSpatialVelocity();
}

//==============================================================================
bool ContactConstraint::isActive() const
{
  return mActive;
}

//==============================================================================
double ContactConstraint::computeFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computeFrictionCoefficient(shapeNode);
}

//==============================================================================
double ContactConstraint::computePrimaryFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computePrimaryFrictionCoefficient(
      shapeNode);
}

//==============================================================================
double ContactConstraint::computeSecondaryFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computeSecondaryFrictionCoefficient(
      shapeNode);
}

//==============================================================================
double ContactConstraint::computePrimarySlipCompliance(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computePrimarySlipCompliance(shapeNode);
}

//==============================================================================
double ContactConstraint::computeSecondarySlipCompliance(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computeSecondarySlipCompliance(
      shapeNode);
}

//==============================================================================
Eigen::Vector3d ContactConstraint::computeWorldFirstFrictionDir(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computeWorldFirstFrictionDir(shapeNode);
}

//==============================================================================
double ContactConstraint::computeRestitutionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  return DefaultContactSurfaceHandler::computeRestitutionCoefficient(shapeNode);
}

//==============================================================================
dynamics::SkeletonPtr ContactConstraint::getRootSkeleton() const
{
  DART_ASSERT(isActive());

  if (!hasValidBodyNodes())
    return nullptr;

  if (mIsReactiveA)
    return ConstraintBase::getRootSkeleton(mBodyNodeA->getSkeleton());
  else
    return ConstraintBase::getRootSkeleton(mBodyNodeB->getSkeleton());
}

//==============================================================================
void ContactConstraint::updateFirstFrictionalDirection()
{
  //  std::cout << "ContactConstraintTEST::_updateFirstFrictionalDirection(): "
  //            << "Not finished implementation."
  //            << std::endl;

  // TODO(JS): Not implemented
  // Refer to:
  // https://github.com/erwincoumans/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp#L910
  //  mFirstFrictionalDirection;
}

//==============================================================================
ContactConstraint::TangentBasisMatrix
ContactConstraint::getTangentBasisMatrixODE(const Eigen::Vector3d& n)
{
  if (std::abs(n.x()) < DART_EPSILON && std::abs(n.y()) < DART_EPSILON
      && std::abs(std::abs(n.z()) - 1.0) < DART_EPSILON
      && (mFirstFrictionalDirection - DART_DEFAULT_FRICTION_DIR).squaredNorm()
             < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED) {
    TangentBasisMatrix T;
    T.col(0) = -Eigen::Vector3d::UnitX();
    if (n.z() > 0.0)
      T.col(1) = Eigen::Vector3d::UnitY();
    else
      T.col(1) = -Eigen::Vector3d::UnitY();
    return T;
  }

  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
  //  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = n.cross(mFirstFrictionalDirection);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close (or opposing directions, or one of the vectors 0),
  // pick another tangent (use X-axis as arbitrary vector)
  if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED) {
    tangent = n.cross(Eigen::Vector3d::UnitX());

    // Make sure this is not zero length, otherwise normalization will lead to
    // NaN values.
    if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED) {
      tangent = n.cross(Eigen::Vector3d::UnitY());
      if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED) {
        tangent = n.cross(Eigen::Vector3d::UnitZ());

        // Now tangent shouldn't be zero-length unless the normal is
        // zero-length, which shouldn't the case because ConstraintSolver
        // shouldn't create a ContactConstraint for a contact with zero-length
        // normal.
        DART_ASSERT(
            tangent.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);
      }
    }
  }

  DART_ASSERT(tangent.norm() > 1e-06);
  tangent.normalize();

  DART_ASSERT(!dart::math::isNan(tangent));

  TangentBasisMatrix T;

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times
  // The first column is the same as mFirstFrictionalDirection unless
  // mFirstFrictionalDirection is parallel to the normal
  // Equivalent to rotating tangent 90 degrees around n, but avoids constructing
  // a quaternion for every contact in the constraint-build hot path.
  T.col(0).noalias() = n.cross(tangent);
  T.col(1) = tangent;
  return T;
}

//==============================================================================
void ContactConstraint::uniteSkeletons()
{
  if (!hasValidBodyNodes())
    return;

  if (!mIsReactiveA || !mIsReactiveB)
    return;

  if (mSkeletonA == mSkeletonB)
    return;

  const dynamics::SkeletonPtr& unionIdA
      = ConstraintBase::compressPath(mBodyNodeA->getSkeleton());
  const dynamics::SkeletonPtr& unionIdB
      = ConstraintBase::compressPath(mBodyNodeB->getSkeleton());

  if (unionIdA == unionIdB)
    return;

  if (unionIdA->mUnionSize < unionIdB->mUnionSize) {
    // Merge root1 --> root2
    unionIdA->mUnionRootSkeleton = unionIdB;
    unionIdB->mUnionSize += unionIdA->mUnionSize;
  } else {
    // Merge root2 --> root1
    unionIdB->mUnionRootSkeleton = unionIdA;
    unionIdA->mUnionSize += unionIdB->mUnionSize;
  }
}

//==============================================================================
double ContactConstraint::getPrimarySlipCompliance() const
{
  return mPrimarySlipCompliance;
}

//==============================================================================
void ContactConstraint::setPrimarySlipCompliance(double slip)
{
  mPrimarySlipCompliance = slip;
}

//==============================================================================
double ContactConstraint::getSecondarySlipCompliance() const
{
  return mSecondarySlipCompliance;
}

//==============================================================================
void ContactConstraint::setSecondarySlipCompliance(double slip)
{
  mSecondarySlipCompliance = slip;
}

//==============================================================================
const collision::Contact& ContactConstraint::getContact() const
{
  return mContact;
}

//==============================================================================
bool ContactConstraint::hasValidBodyNodes() const
{
  return mBodyNodeA != nullptr && mBodyNodeB != nullptr;
}

} // namespace constraint
} // namespace dart
