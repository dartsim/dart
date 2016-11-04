/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <iostream>

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/collision/CollisionObject.hpp"
#include "dart/math/Helpers.hpp"

#define DART_EPSILON 1e-6
#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP     0.01
#define DART_MAX_ERV 1e-3
#define DART_CFM     1e-5
// #define DART_MAX_NUMBER_OF_CONTACTS 32

#define DART_RESTITUTION_COEFF_THRESHOLD 1e-3
#define DART_FRICTION_COEFF_THRESHOLD    1e-3
#define DART_BOUNCING_VELOCITY_THRESHOLD 1e-1
#define DART_MAX_BOUNCING_VELOCITY       1e+2
#define DART_CONTACT_CONSTRAINT_EPSILON  1e-6

namespace dart {
namespace constraint {

double ContactConstraint::mErrorAllowance            = DART_ERROR_ALLOWANCE;
double ContactConstraint::mErrorReductionParameter   = DART_ERP;
double ContactConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double ContactConstraint::mConstraintForceMixing     = DART_CFM;

//==============================================================================
ContactConstraint::ContactConstraint(collision::Contact& _contact,
                                     double _timeStep)
  : ConstraintBase(),
    mTimeStep(_timeStep),
    mBodyNode1(const_cast<dynamics::ShapeFrame*>(_contact.collisionObject1->getShapeFrame())->asShapeNode()->getBodyNodePtr().get()),
    mBodyNode2(const_cast<dynamics::ShapeFrame*>(_contact.collisionObject2->getShapeFrame())->asShapeNode()->getBodyNodePtr().get()),
    mFirstFrictionalDirection(Eigen::Vector3d::UnitZ()),
    mIsFrictionOn(true),
    mAppliedImpulseIndex(-1),
    mIsBounceOn(false),
    mActive(false)
{
  // TODO(JS): Assumed single contact
  mContacts.push_back(&_contact);

  //----------------------------------------------
  // Bounce
  //----------------------------------------------
  mRestitutionCoeff = mBodyNode1->getRestitutionCoeff()
                      * mBodyNode2->getRestitutionCoeff();
  if (mRestitutionCoeff > DART_RESTITUTION_COEFF_THRESHOLD)
    mIsBounceOn = true;
  else
    mIsBounceOn = false;

  //----------------------------------------------
  // Friction
  //----------------------------------------------
  // TODO(JS): Assume the frictional coefficient can be changed during
  //           simulation steps.
  // Update mFrictionalCoff
  mFrictionCoeff = std::min(mBodyNode1->getFrictionCoeff(),
                            mBodyNode2->getFrictionCoeff());
  if (mFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD)
  {
    mIsFrictionOn = true;

    // Update frictional direction
    updateFirstFrictionalDirection();
  }
  else
  {
    mIsFrictionOn = false;
  }

  // Compute local contact Jacobians expressed in body frame
  if (mIsFrictionOn)
  {
    // Set the dimension of this constraint. 1 is for Normal direction constraint.
    // TODO(JS): Assumed that the number of contact is not static.
    // TODO(JS): Adjust following code once use of mNumFrictionConeBases is
    //           implemented.
    //  mDim = mContacts.size() * (1 + mNumFrictionConeBases);
    mDim = mContacts.size() * 3;

    mJacobians1.resize(mDim);
    mJacobians2.resize(mDim);

    // Intermediate variables
    std::size_t idx = 0;

    Eigen::Vector3d bodyDirection1;
    Eigen::Vector3d bodyDirection2;

    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;

    for (std::size_t i = 0; i < mContacts.size(); ++i)
    {
      collision::Contact* ct = mContacts[i];

      // TODO(JS): Assumed that the number of tangent basis is 2.
      Eigen::MatrixXd D = getTangentBasisMatrixODE(ct->normal);

      assert(std::abs(ct->normal.dot(D.col(0))) < DART_EPSILON);
      assert(std::abs(ct->normal.dot(D.col(1))) < DART_EPSILON);
//      if (D.col(0).dot(D.col(1)) > 0.0)
//        std::cout << "D.col(0).dot(D.col(1): " << D.col(0).dot(D.col(1)) << std::endl;
      assert(std::abs(D.col(0).dot(D.col(1))) < DART_EPSILON);

//      std::cout << "D: " << std::endl << D << std::endl;

      // Jacobian for normal contact
      bodyDirection1.noalias()
          = mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias()
          = mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias()
          = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias()
          = mBodyNode2->getTransform().inverse() * ct->point;

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

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

//      std::cout << "bodyDirection2: " << std::endl << bodyDirection2 << std::endl;

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

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

//      std::cout << "bodyDirection2: " << std::endl << bodyDirection2 << std::endl;

      mJacobians1[idx].head<3>() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[idx].head<3>() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[idx].tail<3>() = bodyDirection1;
      mJacobians2[idx].tail<3>() = bodyDirection2;

      ++idx;
    }
  }
  else
  {
    // Set the dimension of this constraint.
    mDim = mContacts.size();

    mJacobians1.resize(mDim);
    mJacobians2.resize(mDim);

    Eigen::Vector3d bodyDirection1;
    Eigen::Vector3d bodyDirection2;

    Eigen::Vector3d bodyPoint1;
    Eigen::Vector3d bodyPoint2;

    for (std::size_t i = 0; i < mContacts.size(); ++i)
    {
      collision::Contact* ct = mContacts[i];

      bodyDirection1.noalias()
          = mBodyNode1->getTransform().linear().transpose() * ct->normal;
      bodyDirection2.noalias()
          = mBodyNode2->getTransform().linear().transpose() * -ct->normal;

      bodyPoint1.noalias()
          = mBodyNode1->getTransform().inverse() * ct->point;
      bodyPoint2.noalias()
          = mBodyNode2->getTransform().inverse() * ct->point;

      mJacobians1[i].head<3>().noalias() = bodyPoint1.cross(bodyDirection1);
      mJacobians2[i].head<3>().noalias() = bodyPoint2.cross(bodyDirection2);

      mJacobians1[i].tail<3>().noalias() = bodyDirection1;
      mJacobians2[i].tail<3>().noalias() = bodyDirection2;
    }
  }

  //----------------------------------------------------------------------------
  // Union finding
  //----------------------------------------------------------------------------
//  uniteSkeletons();
}

//==============================================================================
ContactConstraint::~ContactConstraint()
{
}

//==============================================================================
void ContactConstraint::setErrorAllowance(double _allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (_allowance < 0.0)
  {
    dtwarn << "Error reduction parameter[" << _allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = _allowance;
}

//==============================================================================
double ContactConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void ContactConstraint::setErrorReductionParameter(double _erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (_erp < 0.0)
  {
    dtwarn << "Error reduction parameter[" << _erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (_erp > 1.0)
  {
    dtwarn << "Error reduction parameter[" << _erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = _erp;
}

//==============================================================================
double ContactConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void ContactConstraint::setMaxErrorReductionVelocity(double _erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (_erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity[" << _erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = _erv;
}

//==============================================================================
double ContactConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void ContactConstraint::setConstraintForceMixing(double _cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (_cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter[" << _cfm
           << "] is lower than 1e-9. " << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }
  if (_cfm > 1.0)
  {
    dtwarn << "Constraint force mixing parameter[" << _cfm
           << "] is greater than 1.0. " << "It is set to 1.0." << std::endl;
    mConstraintForceMixing = 1.0;
  }

  mConstraintForceMixing = _cfm;
}

//==============================================================================
double ContactConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void ContactConstraint::setFrictionDirection(
    const Eigen::Vector3d& _dir)
{
  mFirstFrictionalDirection = _dir.normalized();
}

//==============================================================================
const Eigen::Vector3d& ContactConstraint::getFrictionDirection1() const
{
  return mFirstFrictionalDirection;
}

//==============================================================================
void ContactConstraint::update()
{
  if (mBodyNode1->isReactive() || mBodyNode2->isReactive())
    mActive = true;
  else
    mActive = false;
}

//==============================================================================
void ContactConstraint::getInformation(ConstraintInfo* _info)
{
  // Fill w, where the LCP form is Ax = b + w (x >= 0, w >= 0, x^T w = 0)
  getRelVelocity(_info->b);

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn)
  {
    std::size_t index = 0;
    for (std::size_t i = 0; i < mContacts.size(); ++i)
    {
      // Bias term, w, should be zero
      assert(_info->w[index] == 0.0);
      assert(_info->w[index + 1] == 0.0);
      assert(_info->w[index + 2] == 0.0);

      // Upper and lower bounds of normal impulsive force
      _info->lo[index] = 0.0;
      _info->hi[index] = dInfinity;
      assert(_info->findex[index] == -1);

      // Upper and lower bounds of tangential direction-1 impulsive force
      _info->lo[index + 1] = -mFrictionCoeff;
      _info->hi[index + 1] =  mFrictionCoeff;
      _info->findex[index + 1] = index;

      // Upper and lower bounds of tangential direction-2 impulsive force
      _info->lo[index + 2] = -mFrictionCoeff;
      _info->hi[index + 2] =  mFrictionCoeff;
      _info->findex[index + 2] = index;

//      std::cout << "_frictionalCoff: " << _frictionalCoff << std::endl;

//      std::cout << "_lcp->ub[_idx + 1]: " << _lcp->ub[_idx + 1] << std::endl;
//      std::cout << "_lcp->ub[_idx + 2]: " << _lcp->ub[_idx + 2] << std::endl;

      //------------------------------------------------------------------------
      // Bouncing
      //------------------------------------------------------------------------
      // A. Penetration correction
      double bouncingVelocity = mContacts[i]->penetrationDepth
                                - mErrorAllowance;
      if (bouncingVelocity < 0.0)
      {
        bouncingVelocity = 0.0;
      }
      else
      {
        bouncingVelocity *= mErrorReductionParameter * _info->invTimeStep;
        if (bouncingVelocity > mMaxErrorReductionVelocity)
          bouncingVelocity = mMaxErrorReductionVelocity;
      }

      // B. Restitution
      if (mIsBounceOn)
      {
        double& negativeRelativeVel = _info->b[index];
        double restitutionVel = negativeRelativeVel * mRestitutionCoeff;

        if (restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD)
        {
          if (restitutionVel > bouncingVelocity)
          {
            bouncingVelocity = restitutionVel;

            if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY)
            {
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

      // Increase index
      index += 3;
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else
  {
    for (std::size_t i = 0; i < mContacts.size(); ++i)
    {
      // Bias term, w, should be zero
      _info->w[i] = 0.0;

      // Upper and lower bounds of normal impulsive force
      _info->lo[i] = 0.0;
      _info->hi[i] = dInfinity;
      assert(_info->findex[i] == -1);

      //------------------------------------------------------------------------
      // Bouncing
      //------------------------------------------------------------------------
      // A. Penetration correction
      double bouncingVelocity = mContacts[i]->penetrationDepth
                                - DART_ERROR_ALLOWANCE;
      if (bouncingVelocity < 0.0)
      {
        bouncingVelocity = 0.0;
      }
      else
      {
        bouncingVelocity *= mErrorReductionParameter * _info->invTimeStep;
        if (bouncingVelocity > mMaxErrorReductionVelocity)
          bouncingVelocity = mMaxErrorReductionVelocity;
      }

      // B. Restitution
      if (mIsBounceOn)
      {
        double& negativeRelativeVel = _info->b[i];
        double restitutionVel = negativeRelativeVel * mRestitutionCoeff;

        if (restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD)
        {
          if (restitutionVel > bouncingVelocity)
          {
            bouncingVelocity = restitutionVel;

            if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY)
              bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;
          }
        }
      }

      //
//      _lcp->b[_idx] = _lcp->b[_idx] * 1.1;
//      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;
      _info->b[i] += bouncingVelocity;
//      std::cout << "_lcp->b[_idx]: " << _lcp->b[_idx] << std::endl;

      // TODO(JS): Initial guess
      // x
      _info->x[i] = 0.0;

      // Increase index
    }
  }
}

//==============================================================================
void ContactConstraint::applyUnitImpulse(std::size_t _idx)
{
  assert(_idx < mDim && "Invalid Index.");
  assert(isActive());
  assert(mBodyNode1->isReactive() || mBodyNode2->isReactive());

  const dynamics::SkeletonPtr& skel1 = mBodyNode1->getSkeleton();
  const dynamics::SkeletonPtr& skel2 = mBodyNode2->getSkeleton();

  // Self collision case
  if (skel1 == skel2)
  {
    skel1->clearConstraintImpulses();

    if (mBodyNode1->isReactive())
    {
      // Both bodies are reactive
      if (mBodyNode2->isReactive())
      {
        skel1->updateBiasImpulse(mBodyNode1, mJacobians1[_idx],
                                 mBodyNode2, mJacobians2[_idx]);
      }
      // Only body1 is reactive
      else
      {
        skel1->updateBiasImpulse(mBodyNode1, mJacobians1[_idx]);
      }
    }
    else
    {
      // Only body2 is reactive
      if (mBodyNode2->isReactive())
      {
        skel2->updateBiasImpulse(mBodyNode2, mJacobians2[_idx]);
      }
      // Both bodies are not reactive
      else
      {
        // This case should not be happed
        assert(0);
      }
    }

    skel1->updateVelocityChange();
  }
  // Colliding two distinct skeletons
  else
  {
    if (mBodyNode1->isReactive())
    {
      skel1->clearConstraintImpulses();
      skel1->updateBiasImpulse(mBodyNode1, mJacobians1[_idx]);
      skel1->updateVelocityChange();
    }

    if (mBodyNode2->isReactive())
    {
      skel2->clearConstraintImpulses();
      skel2->updateBiasImpulse(mBodyNode2, mJacobians2[_idx]);
      skel2->updateVelocityChange();
    }
  }

  mAppliedImpulseIndex = _idx;
}

//==============================================================================
void ContactConstraint::getVelocityChange(double* _vel, bool _withCfm)
{
  assert(_vel != nullptr && "Null pointer is not allowed.");

  for (std::size_t i = 0; i < mDim; ++i)
  {
    _vel[i] = 0.0;

    if (mBodyNode1->getSkeleton()->isImpulseApplied()
        && mBodyNode1->isReactive())
    {
      _vel[i] += mJacobians1[i].dot(mBodyNode1->getBodyVelocityChange());
    }

    if (mBodyNode2->getSkeleton()->isImpulseApplied()
        && mBodyNode2->isReactive())
    {
      _vel[i] += mJacobians2[i].dot(mBodyNode2->getBodyVelocityChange());
    }
  }

  // Add small values to the diagnal to keep it away from singular, similar to
  // cfm variable in ODE
  if (_withCfm)
  {
    _vel[mAppliedImpulseIndex] += _vel[mAppliedImpulseIndex]
                                     * mConstraintForceMixing;
  }
}

//==============================================================================
void ContactConstraint::excite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(true);

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void ContactConstraint::unexcite()
{
  if (mBodyNode1->isReactive())
    mBodyNode1->getSkeleton()->setImpulseApplied(false);

  if (mBodyNode2->isReactive())
    mBodyNode2->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void ContactConstraint::applyImpulse(double* _lambda)
{
  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn)
  {
    std::size_t index = 0;

    for (std::size_t i = 0; i < mContacts.size(); ++i)
    {
//      std::cout << "_lambda1: " << _lambda[index] << std::endl;
//      std::cout << "_lambda2: " << _lambda[index + 1] << std::endl;
//      std::cout << "_lambda3: " << _lambda[index + 2] << std::endl;

//      std::cout << "imp1: " << mJacobians2[i * 3 + 0] * _lambda[index] << std::endl;
//      std::cout << "imp2: " << mJacobians2[i * 3 + 1] * _lambda[index + 1] << std::endl;
//      std::cout << "imp3: " << mJacobians2[i * 3 + 2] * _lambda[index + 2] << std::endl;

      assert(!math::isNan(_lambda[index]));

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force = mContacts[i]->normal * _lambda[index] / mTimeStep;

      // Normal impulsive force
//      mContacts[i]->lambda[0] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * _lambda[index]);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * _lambda[index]);
//      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
      index++;

      assert(!math::isNan(_lambda[index]));

      // Add contact impulse (force) toward the tangential w.r.t. world frame
      Eigen::MatrixXd D = getTangentBasisMatrixODE(mContacts[i]->normal);
      mContacts[i]->force += D.col(0) * _lambda[index] / mTimeStep;

      // Tangential direction-1 impulsive force
//      mContacts[i]->lambda[1] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * _lambda[index]);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * _lambda[index]);
//      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
      index++;

      assert(!math::isNan(_lambda[index]));

      // Add contact impulse (force) toward the tangential w.r.t. world frame
      mContacts[i]->force += D.col(1) * _lambda[index] / mTimeStep;

      // Tangential direction-2 impulsive force
//      mContacts[i]->lambda[2] = _lambda[_idx];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[index] * _lambda[index]);
      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[index] * _lambda[index]);
//      std::cout << "_lambda: " << _lambda[_idx] << std::endl;
      index++;
    }
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else
  {
    for (std::size_t i = 0; i < mContacts.size(); ++i)
    {
      // Normal impulsive force
//			pContactPts[i]->lambda[0] = _lambda[i];
      if (mBodyNode1->isReactive())
        mBodyNode1->addConstraintImpulse(mJacobians1[i] * _lambda[i]);

      if (mBodyNode2->isReactive())
        mBodyNode2->addConstraintImpulse(mJacobians2[i] * _lambda[i]);

      // Store contact impulse (force) toward the normal w.r.t. world frame
      mContacts[i]->force = mContacts[i]->normal * _lambda[i] / mTimeStep;
    }
  }
}

//==============================================================================
void ContactConstraint::getRelVelocity(double* _relVel)
{
  assert(_relVel != nullptr && "Null pointer is not allowed.");

  for (std::size_t i = 0; i < mDim; ++i)
  {
    _relVel[i] = 0.0;
    _relVel[i] -= mJacobians1[i].dot(mBodyNode1->getSpatialVelocity());
    _relVel[i] -= mJacobians2[i].dot(mBodyNode2->getSpatialVelocity());

//    std::cout << "_relVel[i]: " << _relVel[i] << std::endl;
  }
}

//==============================================================================
bool ContactConstraint::isActive() const
{
  return mActive;
}

//==============================================================================
dynamics::SkeletonPtr ContactConstraint::getRootSkeleton() const
{
  assert(isActive());

  if (mBodyNode1->isReactive())
    return mBodyNode1->getSkeleton()->mUnionRootSkeleton.lock();
  else
    return mBodyNode2->getSkeleton()->mUnionRootSkeleton.lock();
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
Eigen::MatrixXd ContactConstraint::getTangentBasisMatrixODE(
    const Eigen::Vector3d& _n)
{
  using namespace math::suffixes;

  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
//  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;

  Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, 2));

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = mFirstFrictionalDirection.cross(_n);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close, pick another tangent (use X-axis as arbitrary vector)
  if (tangent.norm() < DART_CONTACT_CONSTRAINT_EPSILON)
    tangent = Eigen::Vector3d::UnitX().cross(_n);

  tangent.normalize();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times
  T.col(0) = tangent;
  T.col(1) = Eigen::Quaterniond(Eigen::AngleAxisd(0.5_pi, _n)) * tangent;
  return T;
}

//==============================================================================
void ContactConstraint::uniteSkeletons()
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

  if (unionId1->mUnionSize < unionId2->mUnionSize)
  {
    // Merge root1 --> root2
    unionId1->mUnionRootSkeleton = unionId2;
    unionId2->mUnionSize += unionId1->mUnionSize;
  }
  else
  {
    // Merge root2 --> root1
    unionId2->mUnionRootSkeleton = unionId1;
    unionId1->mUnionSize += unionId2->mUnionSize;
  }
}

}  // namespace constraint
}  // namespace dart
