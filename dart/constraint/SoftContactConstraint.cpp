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

#include "dart/constraint/SoftContactConstraint.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"

#include <iostream>

#include <cmath>

#define DART_EPSILON 1e-6
#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM 1e-5
// #define DART_MAX_NUMBER_OF_CONTACTS 32

#define DART_RESTITUTION_COEFF_THRESHOLD 1e-3
#define DART_FRICTION_COEFF_THRESHOLD 1e-3
#define DART_BOUNCING_VELOCITY_THRESHOLD 1e-1
#define DART_MAX_BOUNCING_VELOCITY 1e+2
#define DART_CONTACT_CONSTRAINT_EPSILON 1e-6

namespace dart {
namespace constraint {

double SoftContactConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
double SoftContactConstraint::mErrorReductionParameter = DART_ERP;
double SoftContactConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double SoftContactConstraint::mConstraintForceMixing = DART_CFM;

constexpr double DART_DEFAULT_FRICTION_COEFF = 1.0;
constexpr double DART_DEFAULT_RESTITUTION_COEFF = 0.0;

//==============================================================================
SoftContactConstraint::SoftContactConstraint(
    collision::Contact& contact, double timeStep)
  : ConstraintBase(),
    mTimeStep(0.0),
    mBodyNode1(nullptr),
    mBodyNode2(nullptr),
    mSoftBodyNode1(nullptr),
    mSoftBodyNode2(nullptr),
    mPointMass1(nullptr),
    mPointMass2(nullptr),
    mContact(nullptr),
    mSoftCollInfo(nullptr),
    mFirstFrictionalDirection(Eigen::Vector3d::UnitZ()),
    mFrictionCoeff(DART_DEFAULT_FRICTION_COEFF),
    mRestitutionCoeff(DART_DEFAULT_RESTITUTION_COEFF),
    mJacobians1(Eigen::Matrix<double, 6, 3>::Zero()),
    mJacobians2(Eigen::Matrix<double, 6, 3>::Zero()),
    mIsFrictionOn(true),
    mTangentBasis(TangentBasisMatrix::Zero()),
    mAppliedImpulseIndex(-1),
    mIsBounceOn(false),
    mActive(false)
{
  reset(contact, timeStep);
}

//==============================================================================
void SoftContactConstraint::reset(collision::Contact& contact, double timeStep)
{
  mTimeStep = timeStep;
  mBodyNode1 = contact.collisionObject1->getBodyNode();
  mBodyNode2 = contact.collisionObject2->getBodyNode();
  mSoftBodyNode1 = dynamic_cast<dynamics::SoftBodyNode*>(mBodyNode1);
  mSoftBodyNode2 = dynamic_cast<dynamics::SoftBodyNode*>(mBodyNode2);
  mPointMass1 = nullptr;
  mPointMass2 = nullptr;
  mContact = &contact;
  mSoftCollInfo = static_cast<collision::SoftCollisionInfo*>(contact.userData);
  mFirstFrictionalDirection = Eigen::Vector3d::UnitZ();
  mFrictionCoeff = DART_DEFAULT_FRICTION_COEFF;
  mRestitutionCoeff = DART_DEFAULT_RESTITUTION_COEFF;
  mJacobians1.setZero();
  mJacobians2.setZero();
  mTangentBasis.setZero();
  mAppliedImpulseIndex = static_cast<std::size_t>(-1);
  mIsBounceOn = false;
  mActive = false;

  // Select colling point mass based on trimesh ID
  if (mSoftBodyNode1) {
    if (contact.collisionObject1->getShape()->getType()
        == dynamics::SoftMeshShape::getStaticType()) {
      mPointMass1 = selectCollidingPointMass(
          mSoftBodyNode1, contact.point, contact.triID1);
      mPointMass1->setColliding(true);
    }
  }
  if (mSoftBodyNode2) {
    if (contact.collisionObject2->getShape()->getType()
        == dynamics::SoftMeshShape::getStaticType()) {
      mPointMass2 = selectCollidingPointMass(
          mSoftBodyNode2, contact.point, contact.triID2);
      mPointMass2->setColliding(true);
    }
  }

  const auto* shapeNodeA = contact.collisionObject1->getShapeNode();
  const auto* shapeNodeB = contact.collisionObject2->getShapeNode();

  //----------------------------------------------------------------------------
  // Bounce
  //----------------------------------------------------------------------------
  const double restitutionCoeffA = computeRestitutionCoefficient(shapeNodeA);
  const double restitutionCoeffB = computeRestitutionCoefficient(shapeNodeB);
  mRestitutionCoeff = restitutionCoeffA * restitutionCoeffB;
  if (mRestitutionCoeff > DART_RESTITUTION_COEFF_THRESHOLD)
    mIsBounceOn = true;
  else
    mIsBounceOn = false;

  //----------------------------------------------------------------------------
  // Friction
  //----------------------------------------------------------------------------
  // TODO(JS): Assume the frictional coefficient can be changed during
  //           simulation steps.
  // Update mFrictionalCoff
  const double frictionCoeffA = computeFrictionCoefficient(shapeNodeA);
  const double frictionCoeffB = computeFrictionCoefficient(shapeNodeB);

  // TODO(JS): Consider providing various ways of the combined friction or
  // allowing to override this method by a custom method
  mFrictionCoeff = std::min(frictionCoeffA, frictionCoeffB);
  if (mFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD) {
    mIsFrictionOn = true;

    // Update frictional direction
    updateFirstFrictionalDirection();
    mTangentBasis = getTangentBasisMatrixODE(contact.normal);
  } else {
    mIsFrictionOn = false;
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

    // Intermediate variables
    int idx = 0;

    Eigen::Vector3d bodyDirection1;
    Eigen::Vector3d bodyDirection2;

    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;

    collision::Contact* ct = mContact;
    {
      // TODO(JS): Assumed that the number of tangent basis is 2.
      const TangentBasisMatrix& D = mTangentBasis;

      DART_ASSERT(std::abs(ct->normal.dot(D.col(0))) < DART_EPSILON);
      DART_ASSERT(std::abs(ct->normal.dot(D.col(1))) < DART_EPSILON);
      //      if (D.col(0).dot(D.col(1)) > 0.0)
      //        std::cout << "D.col(0).dot(D.col(1): " << D.col(0).dot(D.col(1))
      //        << std::endl;
      DART_ASSERT(std::abs(D.col(0).dot(D.col(1))) < DART_EPSILON);

      //      std::cout << "D: " << std::endl << D << std::endl;

      // Jacobian for normal contact
      bodyDirection1.noalias()
          = mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias()
          = mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias() = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias() = mBodyNode2->getTransform().inverse() * ct->point;

      mJacobians1.col(idx).head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2.col(idx).head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1.col(idx).tail<3>() = bodyDirection1;
      mJacobians2.col(idx).tail<3>() = bodyDirection2;

      ++idx;

      // Jacobian for directional friction 1
      bodyDirection1.noalias()
          = mBodyNode1->getTransform().linear().transpose() * D.col(0);
      bodyDirection2.noalias()
          = mBodyNode2->getTransform().linear().transpose() * -D.col(0);

      //      bodyPoint1.noalias()
      //          = mBodyNode1->getWorldTransform().inverse() * ct->point;
      //      bodyPoint2.noalias()
      //          = mBodyNode2->getWorldTransform().inverse() * ct->point;

      //      std::cout << "bodyDirection2: " << std::endl << bodyDirection2 <<
      //      std::endl;

      mJacobians1.col(idx).head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2.col(idx).head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1.col(idx).tail<3>() = bodyDirection1;
      mJacobians2.col(idx).tail<3>() = bodyDirection2;

      ++idx;

      // Jacobian for directional friction 2
      bodyDirection1.noalias()
          = mBodyNode1->getTransform().linear().transpose() * D.col(1);
      bodyDirection2.noalias()
          = mBodyNode2->getTransform().linear().transpose() * -D.col(1);

      //      bodyPoint1.noalias()
      //          = mBodyNode1->getWorldTransform().inverse() * ct->point;
      //      bodyPoint2.noalias()
      //          = mBodyNode2->getWorldTransform().inverse() * ct->point;

      //      std::cout << "bodyDirection2: " << std::endl << bodyDirection2 <<
      //      std::endl;

      mJacobians1.col(idx).head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2.col(idx).head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1.col(idx).tail<3>() = bodyDirection1;
      mJacobians2.col(idx).tail<3>() = bodyDirection2;

      ++idx;
    }
  } else {
    // Set the dimension of this constraint.
    mDim = 1;

    Eigen::Vector3d bodyDirection1;
    Eigen::Vector3d bodyDirection2;

    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;

    collision::Contact* ct = mContact;
    {
      bodyDirection1.noalias()
          = mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias()
          = mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias() = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias() = mBodyNode2->getTransform().inverse() * ct->point;

      mJacobians1.col(0).head<3>().noalias() = bodyPoint1.cross(bodyDirection1);
      mJacobians2.col(0).head<3>().noalias() = bodyPoint2.cross(bodyDirection2);

      mJacobians1.col(0).tail<3>().noalias() = bodyDirection1;
      mJacobians2.col(0).tail<3>().noalias() = bodyDirection2;
    }
  }

  //----------------------------------------------------------------------------
  // Union finding
  //----------------------------------------------------------------------------
  //  uniteSkeletons();
}

//==============================================================================
SoftContactConstraint::~SoftContactConstraint() {}

//==============================================================================
const std::string& SoftContactConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& SoftContactConstraint::getStaticType()
{
  static const std::string name = "SoftContactConstraint";
  return name;
}

//==============================================================================
void SoftContactConstraint::setErrorAllowance(double _allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (_allowance < 0.0) {
    dtwarn << "Error reduction parameter[" << _allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = _allowance;
}

//==============================================================================
double SoftContactConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void SoftContactConstraint::setErrorReductionParameter(double _erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (_erp < 0.0) {
    dtwarn << "Error reduction parameter[" << _erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (_erp > 1.0) {
    dtwarn << "Error reduction parameter[" << _erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = _erp;
}

//==============================================================================
double SoftContactConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void SoftContactConstraint::setMaxErrorReductionVelocity(double _erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (_erv < 0.0) {
    dtwarn << "Maximum error reduction velocity[" << _erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = _erv;
}

//==============================================================================
double SoftContactConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void SoftContactConstraint::setConstraintForceMixing(double _cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (_cfm < 1e-9) {
    dtwarn << "Constraint force mixing parameter[" << _cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = _cfm;
}

//==============================================================================
double SoftContactConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void SoftContactConstraint::setFrictionDirection(const Eigen::Vector3d& _dir)
{
  mFirstFrictionalDirection = _dir.normalized();
}

//==============================================================================
const Eigen::Vector3d& SoftContactConstraint::getFrictionDirection1() const
{
  return mFirstFrictionalDirection;
}

//==============================================================================
void SoftContactConstraint::update()
{
  DART_ASSERT(mSoftBodyNode1 || mSoftBodyNode2);

  // One of body node is soft body node and soft body node is always active
  mActive = true;
}

//==============================================================================
void SoftContactConstraint::getInformation(ConstraintInfo* _info)
{
  // Fill w, where the LCP form is Ax = b + w (x >= 0, w >= 0, x^T w = 0)
  getRelVelocity(_info->b);

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    std::size_t index = 0;
    // Bias term, w, should be zero
    DART_ASSERT(_info->w[index] == 0.0);
    DART_ASSERT(_info->w[index + 1] == 0.0);
    DART_ASSERT(_info->w[index + 2] == 0.0);

    // Upper and lower bounds of normal impulsive force
    _info->lo[index] = 0.0;
    _info->hi[index] = ::dart::lcpsolver::dantzig::kInfinity;
    DART_ASSERT(_info->findex[index] == -1);

    // Upper and lower bounds of tangential direction-1 impulsive force
    _info->lo[index + 1] = -mFrictionCoeff;
    _info->hi[index + 1] = mFrictionCoeff;
    _info->findex[index + 1] = index;

    // Upper and lower bounds of tangential direction-2 impulsive force
    _info->lo[index + 2] = -mFrictionCoeff;
    _info->hi[index + 2] = mFrictionCoeff;
    _info->findex[index + 2] = index;

    //      std::cout << "_frictionalCoff: " << _frictionalCoff << std::endl;

    //      std::cout << "_lcp->ub[_idx + 1]: " << _lcp->ub[_idx + 1] <<
    //      std::endl; std::cout << "_lcp->ub[_idx + 2]: " << _lcp->ub[_idx +
    //      2] << std::endl;

    //------------------------------------------------------------------------
    // Bouncing
    //------------------------------------------------------------------------
    // A. Penetration correction
    double bouncingVelocity = mContact->penetrationDepth - mErrorAllowance;
    if (bouncingVelocity < 0.0) {
      bouncingVelocity = 0.0;
    } else {
      bouncingVelocity *= mErrorReductionParameter * _info->invTimeStep;
      if (bouncingVelocity > mMaxErrorReductionVelocity)
        bouncingVelocity = mMaxErrorReductionVelocity;
    }

    // B. Restitution
    if (mIsBounceOn) {
      double& negativeRelativeVel = _info->b[index];
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

    //
    //      _lcp->b[_idx] = _lcp->b[_idx] * 1.1;
    //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;
    _info->b[index] += bouncingVelocity;
    //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

    // TODO(JS): Initial guess
    // x
    _info->x[index] = 0.0;
    _info->x[index + 1] = 0.0;
    _info->x[index + 2] = 0.0;
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    // Bias term, w, should be zero
    _info->w[0] = 0.0;

    // Upper and lower bounds of normal impulsive force
    _info->lo[0] = 0.0;
    _info->hi[0] = ::dart::lcpsolver::dantzig::kInfinity;
    DART_ASSERT(_info->findex[0] == -1);

    //------------------------------------------------------------------------
    // Bouncing
    //------------------------------------------------------------------------
    // A. Penetration correction
    double bouncingVelocity = mContact->penetrationDepth - DART_ERROR_ALLOWANCE;
    if (bouncingVelocity < 0.0) {
      bouncingVelocity = 0.0;
    } else {
      bouncingVelocity *= mErrorReductionParameter * _info->invTimeStep;
      if (bouncingVelocity > DART_MAX_ERV)
        bouncingVelocity = DART_MAX_ERV;
    }

    // B. Restitution
    if (mIsBounceOn) {
      double& negativeRelativeVel = _info->b[0];
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

    //
    //      _lcp->b[_idx] = _lcp->b[_idx] * 1.1;
    //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;
    _info->b[0] += bouncingVelocity;
    //      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

    // TODO(JS): Initial guess
    // x
    _info->x[0] = 0.0;
  }
}

//==============================================================================
void SoftContactConstraint::applyUnitImpulse(std::size_t _idx)
{
  DART_ASSERT(_idx < mDim && "Invalid Index.");
  DART_ASSERT(isActive());

  const auto skeleton1 = mBodyNode1->getSkeleton();
  const auto skeleton2 = mBodyNode2->getSkeleton();
  const bool isReactive1 = mBodyNode1->isReactive();
  const bool isReactive2 = mBodyNode2->isReactive();

  DART_ASSERT(isReactive1 || isReactive2);

  // Self collision case
  if (skeleton1 == skeleton2) {
    skeleton1->clearConstraintImpulses();

    if (mPointMass1) {
      skeleton1->updateBiasImpulse(
          mSoftBodyNode1, mPointMass1, mJacobians1.col(_idx).tail<3>());
    } else {
      if (isReactive1) {
        skeleton1->updateBiasImpulse(mBodyNode1, mJacobians1.col(_idx));
      }
    }

    if (mPointMass2) {
      skeleton1->updateBiasImpulse(
          mSoftBodyNode2, mPointMass2, mJacobians2.col(_idx).tail<3>());
    } else {
      if (isReactive2) {
        skeleton1->updateBiasImpulse(mBodyNode2, mJacobians2.col(_idx));
      }
    }

    skeleton1->updateVelocityChange();
  }
  // Colliding two distinct skeletons
  else {
    if (mPointMass1) {
      skeleton1->clearConstraintImpulses();
      skeleton1->updateBiasImpulse(
          mSoftBodyNode1, mPointMass1, mJacobians1.col(_idx).tail<3>());
      skeleton1->updateVelocityChange();
    } else {
      if (isReactive1) {
        skeleton1->clearConstraintImpulses();
        skeleton1->updateBiasImpulse(mBodyNode1, mJacobians1.col(_idx));
        skeleton1->updateVelocityChange();
      }
    }

    if (mPointMass2) {
      skeleton2->clearConstraintImpulses();
      skeleton2->updateBiasImpulse(
          mSoftBodyNode2, mPointMass2, mJacobians2.col(_idx).tail<3>());
      skeleton2->updateVelocityChange();
    } else {
      if (isReactive2) {
        skeleton2->clearConstraintImpulses();
        skeleton2->updateBiasImpulse(mBodyNode2, mJacobians2.col(_idx));
        skeleton2->updateVelocityChange();
      }
    }
  }

  mAppliedImpulseIndex = _idx;
}

//==============================================================================
void SoftContactConstraint::getVelocityChange(double* _vel, bool _withCfm)
{
  DART_ASSERT(_vel != nullptr && "Null pointer is not allowed.");

  const auto skeleton1 = mBodyNode1->getSkeleton();
  const auto skeleton2 = mBodyNode2->getSkeleton();
  const bool isImpulseApplied1 = skeleton1->isImpulseApplied();
  const bool isImpulseApplied2 = skeleton2->isImpulseApplied();

  const Eigen::Vector3d* pointVelChange1 = nullptr;
  const Eigen::Vector3d* pointVelChange2 = nullptr;
  const Eigen::Vector6d* bodyVelChange1 = nullptr;
  const Eigen::Vector6d* bodyVelChange2 = nullptr;

  if (isImpulseApplied1) {
    if (mPointMass1)
      pointVelChange1 = &mPointMass1->getBodyVelocityChange();
    else if (mBodyNode1->isReactive())
      bodyVelChange1 = &mBodyNode1->getBodyVelocityChange();
  }

  if (isImpulseApplied2) {
    if (mPointMass2)
      pointVelChange2 = &mPointMass2->getBodyVelocityChange();
    else if (mBodyNode2->isReactive())
      bodyVelChange2 = &mBodyNode2->getBodyVelocityChange();
  }

  for (std::size_t i = 0; i < mDim; ++i) {
    double vel = 0.0;

    if (pointVelChange1)
      vel += mJacobians1.col(i).tail<3>().dot(*pointVelChange1);
    else if (bodyVelChange1)
      vel += mJacobians1.col(i).dot(*bodyVelChange1);

    if (pointVelChange2)
      vel += mJacobians2.col(i).tail<3>().dot(*pointVelChange2);
    else if (bodyVelChange2)
      vel += mJacobians2.col(i).dot(*bodyVelChange2);

    _vel[i] = vel;
  }

  // Add small values to diagnal to keep it away from singular, similar to cfm
  // variable in ODE
  if (_withCfm) {
    _vel[mAppliedImpulseIndex]
        += _vel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }
}

//==============================================================================
void SoftContactConstraint::excite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(true);

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void SoftContactConstraint::unexcite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(false);

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void SoftContactConstraint::applyImpulse(double* _lambda)
{
  // TODO(JS): Need to optimize code

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn) {
    std::size_t index = 0;

    //      std::cout << "_lambda1: " << _lambda[_idx] << std::endl;
    //      std::cout << "_lambda2: " << _lambda[_idx + 1] << std::endl;
    //      std::cout << "_lambda3: " << _lambda[_idx + 2] << std::endl;

    //      std::cout << "imp1: " << mJacobians2[i * 3 + 0] * _lambda[_idx] <<
    //      std::endl; std::cout << "imp2: " << mJacobians2[i * 3 + 1] *
    //      _lambda[_idx + 1] << std::endl; std::cout << "imp3: " <<
    //      mJacobians2[i * 3 + 2] * _lambda[_idx + 2] << std::endl;

    DART_ASSERT(!math::isNan(_lambda[index]));

    // Store contact impulse (force) toward the normal w.r.t. world frame
    mContact->force = mContact->normal * _lambda[index] / mTimeStep;

    // Normal impulsive force
    //      mContact->lambda[0] = _lambda[_idx];
    if (mPointMass1) {
      mPointMass1->addConstraintImpulse(
          mJacobians1.col(index).tail<3>() * _lambda[index], true);
    } else {
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(
            mJacobians1.col(index) * _lambda[index]);
    }

    if (mPointMass2) {
      mPointMass2->addConstraintImpulse(
          mJacobians2.col(index).tail<3>() * _lambda[index], true);
    } else {
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(
            mJacobians2.col(index) * _lambda[index]);
    }
    //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
    index++;

    DART_ASSERT(!math::isNan(_lambda[index]));

    // Add contact impulse (force) toward the tangential w.r.t. world frame
    mContact->force += mTangentBasis.col(0) * _lambda[index] / mTimeStep;

    // Tangential direction-1 impulsive force
    //      mContact->lambda[1] = _lambda[_idx];
    if (mPointMass1) {
      mPointMass1->addConstraintImpulse(
          mJacobians1.col(index).tail<3>() * _lambda[index], true);
    } else {
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(
            mJacobians1.col(index) * _lambda[index]);
    }

    if (mPointMass2) {
      mPointMass2->addConstraintImpulse(
          mJacobians2.col(index).tail<3>() * _lambda[index], true);
    } else {
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(
            mJacobians2.col(index) * _lambda[index]);
    }
    //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
    index++;

    DART_ASSERT(!math::isNan(_lambda[index]));

    // Add contact impulse (force) toward the tangential w.r.t. world frame
    mContact->force += mTangentBasis.col(1) * _lambda[index] / mTimeStep;

    // Tangential direction-2 impulsive force
    //      mContact->lambda[2] = _lambda[_idx];
    if (mPointMass1) {
      mPointMass1->addConstraintImpulse(
          mJacobians1.col(index).tail<3>() * _lambda[index], true);
    } else {
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(
            mJacobians1.col(index) * _lambda[index]);
    }

    if (mPointMass2) {
      mPointMass2->addConstraintImpulse(
          mJacobians2.col(index).tail<3>() * _lambda[index], true);
    } else {
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(
            mJacobians2.col(index) * _lambda[index]);
    }
    //      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else {
    // Normal impulsive force
    //			pContactPts[i]->lambda[0] = _lambda[i];

    if (mPointMass1) {
      mPointMass1->addConstraintImpulse(
          mJacobians1.col(0).tail<3>() * _lambda[0], true);
    } else {
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1.col(0) * _lambda[0]);
    }

    if (mPointMass2) {
      mPointMass2->addConstraintImpulse(
          mJacobians2.col(0).tail<3>() * _lambda[0], true);
    } else {
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2.col(0) * _lambda[0]);
    }

    // Store contact impulse (force) toward the normal w.r.t. world frame
    mContact->force = mContact->normal * _lambda[0] / mTimeStep;
  }
}

//==============================================================================
void SoftContactConstraint::getRelVelocity(double* _vel)
{
  DART_ASSERT(_vel != nullptr && "Null pointer is not allowed.");

  const Eigen::Vector3d* pointVelocity1 = nullptr;
  const Eigen::Vector3d* pointVelocity2 = nullptr;
  const Eigen::Vector6d* bodyVelocity1 = nullptr;
  const Eigen::Vector6d* bodyVelocity2 = nullptr;

  if (mPointMass1)
    pointVelocity1 = &mPointMass1->getBodyVelocity();
  else
    bodyVelocity1 = &mBodyNode1->getSpatialVelocity();

  if (mPointMass2)
    pointVelocity2 = &mPointMass2->getBodyVelocity();
  else
    bodyVelocity2 = &mBodyNode2->getSpatialVelocity();

  for (std::size_t i = 0; i < mDim; ++i) {
    double vel = 0.0;

    if (pointVelocity1)
      vel -= mJacobians1.col(i).tail<3>().dot(*pointVelocity1);
    else
      vel -= mJacobians1.col(i).dot(*bodyVelocity1);

    if (pointVelocity2)
      vel -= mJacobians2.col(i).tail<3>().dot(*pointVelocity2);
    else
      vel -= mJacobians2.col(i).dot(*bodyVelocity2);

    _vel[i] = vel;
  }
}

//==============================================================================
bool SoftContactConstraint::isActive() const
{
  return mActive;
}

//==============================================================================
double SoftContactConstraint::computeFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  const double coeff = dynamicAspect->getFrictionCoeff();
  if (!std::isfinite(coeff) || coeff < 0.0) {
    dtwarn << "[ContactConstraint] Invalid friction coefficient (" << coeff
           << ") from ShapeNode [" << shapeNode->getName()
           << "]. Friction must be non-negative and finite. Using default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ").\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return coeff;
}

//==============================================================================
double SoftContactConstraint::computeRestitutionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract restitution coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_RESTITUTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_RESTITUTION_COEFF;
  }

  const double coeff = dynamicAspect->getRestitutionCoeff();
  if (!std::isfinite(coeff) || coeff < 0.0 || coeff > 1.0) {
    dtwarn << "[ContactConstraint] Invalid restitution coefficient (" << coeff
           << ") from ShapeNode [" << shapeNode->getName()
           << "]. Restitution must be in range [0, 1] and finite. Using "
           << "default value (" << DART_DEFAULT_RESTITUTION_COEFF << ").\n";
    return DART_DEFAULT_RESTITUTION_COEFF;
  }

  return coeff;
}

//==============================================================================
dynamics::SkeletonPtr SoftContactConstraint::getRootSkeleton() const
{
  if (mSoftBodyNode1 || mBodyNode1->isReactive())
    return ConstraintBase::getRootSkeleton(
        mBodyNode1->getSkeleton()->getSkeleton());
  else
    return ConstraintBase::getRootSkeleton(
        mBodyNode2->getSkeleton()->getSkeleton());
}

//==============================================================================
void SoftContactConstraint::uniteSkeletons()
{
  if (!mBodyNode1->isReactive() || !mBodyNode2->isReactive())
    return;

  if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton())
    return;

  const dynamics::SkeletonPtr& unionId1
      = ConstraintBase::compressPath(mBodyNode1->getSkeleton());
  const dynamics::SkeletonPtr& unionId2
      = ConstraintBase::compressPath(mBodyNode2->getSkeleton());

  if (unionId1 == unionId2)
    return;

  if (unionId1->mUnionSize < unionId2->mUnionSize) {
    // Merge root1 --> root2
    unionId1->mUnionRootSkeleton = unionId2;
    unionId2->mUnionSize += unionId1->mUnionSize;
  } else {
    // Merge root2 --> root1
    unionId2->mUnionRootSkeleton = unionId1;
    unionId1->mUnionSize += unionId2->mUnionSize;
  }
}

//==============================================================================
void SoftContactConstraint::updateFirstFrictionalDirection()
{
  //  std::cout <<
  //  "SoftContactConstraintTEST::_updateFirstFrictionalDirection(): "
  //            << "Not finished implementation."
  //            << std::endl;

  // TODO(JS): Not implemented
  // Refer to:
  // https://github.com/erwincoumans/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp#L910
  //  mFirstFrictionalDirection;
}

//==============================================================================
SoftContactConstraint::TangentBasisMatrix
SoftContactConstraint::getTangentBasisMatrixODE(const Eigen::Vector3d& _n)
{
  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
  //  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;

  TangentBasisMatrix T = TangentBasisMatrix::Zero();

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = mFirstFrictionalDirection.cross(_n);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close, pick another tangent (use X-axis as arbitrary vector)
  if (tangent.norm() < DART_CONTACT_CONSTRAINT_EPSILON)
    tangent = Eigen::Vector3d::UnitX().cross(_n);

  tangent.normalize();

  // Rotate the tangent 90 degrees around the unit normal. Rodrigues' formula
  // reduces to n x tangent because the tangent is perpendicular to n.
  T.col(0) = tangent;
  T.col(1) = _n.cross(tangent);
  return T;
}

//==============================================================================
template <typename PointMassT, typename SoftBodyNodeT>
static PointMassT selectCollidingPointMassT(
    SoftBodyNodeT _softBodyNode, const Eigen::Vector3d& _point, int _faceId)
{
  PointMassT pointMass = nullptr;

  const Eigen::Vector3i& face = _softBodyNode->getFace(_faceId);

  PointMassT pm0 = _softBodyNode->getPointMass(face[0]);
  PointMassT pm1 = _softBodyNode->getPointMass(face[1]);
  PointMassT pm2 = _softBodyNode->getPointMass(face[2]);

  const Eigen::Vector3d& pos1 = pm0->getWorldPosition();
  const Eigen::Vector3d& pos2 = pm1->getWorldPosition();
  const Eigen::Vector3d& pos3 = pm2->getWorldPosition();

  const double dist0 = (pos1 - _point).squaredNorm();
  const double dist1 = (pos2 - _point).squaredNorm();
  const double dist2 = (pos3 - _point).squaredNorm();

  if (dist0 > dist1) {
    if (dist1 > dist2)
      pointMass = pm2;
    else
      pointMass = pm1;
  } else {
    if (dist0 > dist2)
      pointMass = pm2;
    else
      pointMass = pm0;
  }

  return pointMass;
}

//==============================================================================
const dynamics::PointMass* SoftContactConstraint::selectCollidingPointMass(
    const dynamics::SoftBodyNode* _softBodyNode,
    const Eigen::Vector3d& _point,
    int _faceId) const
{
  return selectCollidingPointMassT<
      const dynamics::PointMass*,
      const dynamics::SoftBodyNode*>(_softBodyNode, _point, _faceId);
}

//==============================================================================
dynamics::PointMass* SoftContactConstraint::selectCollidingPointMass(
    dynamics::SoftBodyNode* _softBodyNode,
    const Eigen::Vector3d& _point,
    int _faceId) const
{
  return selectCollidingPointMassT<
      dynamics::PointMass*,
      dynamics::SoftBodyNode*>(_softBodyNode, _point, _faceId);
}

} // namespace constraint
} // namespace dart
