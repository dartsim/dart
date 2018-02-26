/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/constraint/HumanLegJointLimitConstraint.hpp"

#include <iostream>

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP     0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM     1e-9

using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

namespace dart {
namespace constraint {

double HumanLegJointLimitConstraint::mErrorAllowance            = DART_ERROR_ALLOWANCE;
double HumanLegJointLimitConstraint::mErrorReductionParameter   = DART_ERP;
double HumanLegJointLimitConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double HumanLegJointLimitConstraint::mConstraintForceMixing     = DART_CFM;

//==============================================================================
HumanLegJointLimitConstraint::HumanLegJointLimitConstraint(
    dynamics::Joint* _hipjoint, dynamics::Joint* _kneejoint, dynamics::Joint* _anklejoint, bool _isMirror)
  : ConstraintBase(),
    mHipJoint(_hipjoint),
    mKneeJoint(_kneejoint),
    mAnkleJoint(_anklejoint),
    mThighNode(_hipjoint->getChildBodyNode()),
    mLowerLegNode(_kneejoint->getChildBodyNode()),
    mFootNode(_anklejoint->getChildBodyNode()),
    mIsMirror(_isMirror),
    mAppliedImpulseIndex(0)
{
    assert(mHipJoint);
    assert(mKneeJoint);
    assert(mAnkleJoint);
    assert(mThighNode);
    assert(mLowerLegNode);
    assert(mFootNode);
    
    assert(mHipJoint->getDof() == 3);
    assert(mKneeJoint->getDof() == 1);
    assert(mAnkleJoint->getDof() == 2);
    
    assert(mThighNode->getSkeleton() == mLowerLegNode->getSkeleton());
    assert(mLowerLegNode->getSkeleton() == mFootNode->getSkeleton());
    assert(mKneeJoint->getParentBodyNode() == mThighNode);
    assert(mAnkleJoint->getParentBodyNode() == mLowerLegNode);
    
    mLifeTime = 0;
    mActive = false;
    
    // load neural net weights from external file
    mNet.load(DART_DATA_PATH"/neuralnets/net-lleg");
}
    
    

//==============================================================================
HumanLegJointLimitConstraint::~HumanLegJointLimitConstraint()
{
}

//==============================================================================
void HumanLegJointLimitConstraint::setErrorAllowance(double _allowance)
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
double HumanLegJointLimitConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void HumanLegJointLimitConstraint::setErrorReductionParameter(double _erp)
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
double HumanLegJointLimitConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void HumanLegJointLimitConstraint::setMaxErrorReductionVelocity(double _erv)
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
double HumanLegJointLimitConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void HumanLegJointLimitConstraint::setConstraintForceMixing(double _cfm)
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
double HumanLegJointLimitConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void HumanLegJointLimitConstraint::update()
{
    double qz = mHipJoint->getPosition(0);
    double qx = mHipJoint->getPosition(1);
    double qy = mHipJoint->getPosition(2);
    double qe = mKneeJoint->getPosition(0);
    double hx = mAnkleJoint->getPosition(0);
    double hy = mAnkleJoint->getPosition(1);
    
    double qz_d = mHipJoint->getVelocity(0);
    double qx_d = mHipJoint->getVelocity(1);
    double qy_d = mHipJoint->getVelocity(2);
    double qe_d = mKneeJoint->getVelocity(0);
    double hx_d = mAnkleJoint->getVelocity(0);
    double hy_d = mAnkleJoint->getVelocity(1);
    
    // if isMirror (right-lrg), set up a mirrored euler joint for hip
    // i.e. pass the mirrored config to NN
    if (mIsMirror)
    {
        qz = -qz; qy = -qy;
    }
    
    double qsin[8] = {cos(qz), sin(qz), cos(qx), sin(qx),
        cos(qy + M_PI_2), cos(qe), cos(hx + M_PI_2), cos(hy + M_PI_2)};
    vec_t input;
    input.assign(qsin, qsin+8);
    vec_t pred_vec = mNet.predict(input);
    double C = *(pred_vec.begin());

    mViolation = C - 0.5;
    
    // if not active, no variable matters, no need to update
    mActive = false;
    mDim = 0;
    
    // active: mViolation <= 0 (C(q)-0.5<=0)
    if (mViolation <= 0.0)
    {
        if (mActive)
        {
            ++mLifeTime;
        }
        else
        {
            mActive = true;
            mLifeTime = 0;
        }
        
        // do back-propogation to obtain gradient
        layer* l;
        vec_t out_grad = {1};
        vec_t in_grad;
        for (int n = mNet.layer_size()-1; n >= 0; n--)
        {
            // implement chain rule layer by layer
            l = mNet[n];
            if (l->layer_type() == "fully-connected")
            {
                auto Wb = l->weights();
                vec_t W = *(Wb[0]);
                in_grad.assign(W.size()/out_grad.size(), 0);
                for (size_t c = 0; c < in_grad.size(); c++) {
                    in_grad[c] = vectorize::dot(&out_grad[0], &W[c * out_grad.size()], out_grad.size());
                }
            }
            else
            {
                // this is activation layer
                std::vector<const tensor_t*> out_t;
                l->output(out_t);
                vec_t out_v =  (*(out_t[0]))[0];
                
                in_grad.assign(out_grad.begin(), out_grad.end());
                // first arg (x) is used only to infer the size of input, which should be the same as output y
                (dynamic_cast<activation_layer*>(l))->backward_activation(out_v, out_v, in_grad, out_grad);
            }
            out_grad.assign(in_grad.begin(), in_grad.end());
            in_grad.clear();
        }
        
        mJacobian[0] = out_grad[0] * (-sin(qz)) + out_grad[1] * (cos(qz));
        mJacobian[1] = out_grad[2] * (-sin(qx)) + out_grad[3] * (cos(qx));
        mJacobian[2] = out_grad[4] * (-sin(qy + M_PI_2));
        mJacobian[3] = out_grad[5] * (-sin(qe));
        mJacobian[4] = out_grad[6] * (-sin(hx + M_PI_2));
        mJacobian[5] = out_grad[7] * (-sin(hy + M_PI_2));
        
        // note that we also need to take the mirror of the NN gradient for right-leg
        if (mIsMirror)
        {
            mJacobian[0] = -mJacobian[0];
            mJacobian[2] = -mJacobian[2];
        }
        
        // TODO: Normalize grad seems unnecessary?
        
        Eigen::Vector6d q_d;
        q_d << qz_d, qx_d, qy_d, qe_d, hx_d, hy_d;
        mNegativeVel = -mJacobian.dot(q_d);
        
        mLowerBound = 0.0;
        mUpperBound = dInfinity;
        mDim = 1;
    }
}

//==============================================================================
void HumanLegJointLimitConstraint::getInformation(ConstraintInfo* _lcp)
{
    // if non-active, should not call getInfo()
    assert(isActive());
    
    // assume caller will allocate enough space for _lcp variables
    assert(_lcp->w[0] == 0.0);
    assert(_lcp->findex[0] == -1);
    
    double bouncingVel = -mViolation - mErrorAllowance;
    if (bouncingVel < 0.0)
    {
        bouncingVel = 0.0;
    }
    bouncingVel *= _lcp->invTimeStep * mErrorReductionParameter;
    if (bouncingVel > mMaxErrorReductionVelocity)
        bouncingVel = mMaxErrorReductionVelocity;
    
    _lcp->b[0] = mNegativeVel + bouncingVel;
    _lcp->lo[0] = mLowerBound;
    _lcp->hi[0] = mUpperBound;
    
    if (mLifeTime)
        _lcp->x[0] = mOldX;
    else
        _lcp->x[0] = 0.0;
}

//==============================================================================
void HumanLegJointLimitConstraint::applyUnitImpulse(std::size_t _index)
{
    // the dim of constraint = 1, valid _index can only be 0
    assert(_index < mDim && "Invalid Index.");
    assert(isActive());
    
    const dynamics::SkeletonPtr& skeleton = mHipJoint->getSkeleton();
    skeleton->clearConstraintImpulses();
    
    for (std::size_t i = 0; i < 3; i++)
    {
        mHipJoint->setConstraintImpulse(i, mJacobian[i]);
    }
    mKneeJoint->setConstraintImpulse(0, mJacobian[3]);
    for (std::size_t i = 0; i < 2; i++)
    {
        mAnkleJoint->setConstraintImpulse(i, mJacobian[4+i]);
    }
    
    // Use the body which is placed later in the list of body nodes in this skeleton
    skeleton->updateBiasImpulse(mFootNode);
    skeleton->updateVelocityChange();
    
    for (std::size_t i = 0; i < 3; i++)
    {
        mHipJoint->setConstraintImpulse(i, 0.0);
    }
    mKneeJoint->setConstraintImpulse(0, 0.0);
    for (std::size_t i = 0; i < 2; i++)
    {
        mAnkleJoint->setConstraintImpulse(i, 0.0);
    }
    
    mAppliedImpulseIndex = _index; // which is 0
}

//==============================================================================
void HumanLegJointLimitConstraint::getVelocityChange(double* _delVel, bool _withCfm)
{
    assert(_delVel != nullptr && "Null pointer is not allowed.");
    _delVel[0] = 0.0;
    
    if (mHipJoint->getSkeleton()->isImpulseApplied())
    {
        Eigen::Vector6d delq_d;
        for (std::size_t i = 0; i < 3; i++)
        {
            delq_d[i] = mHipJoint->getVelocityChange(i);
        }
        delq_d[3] = mKneeJoint->getVelocityChange(0);
        for (std::size_t i = 0; i < 2; i++)
        {
            delq_d[4+i] = mAnkleJoint->getVelocityChange(i);
        }
        _delVel[0] = mJacobian.dot(delq_d);
    }
    
    if (_withCfm)
    {
        _delVel[mAppliedImpulseIndex] += _delVel[mAppliedImpulseIndex]
                                     * mConstraintForceMixing;
    }
}

//==============================================================================
void HumanLegJointLimitConstraint::excite()
{
    mHipJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void HumanLegJointLimitConstraint::unexcite()
{
    mHipJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void HumanLegJointLimitConstraint::applyImpulse(double* _lambda)
{
    assert(isActive());
    
    // the dim of constraint = 1
    auto con_force = _lambda[0];
    mOldX = con_force;
    
    for (std::size_t i = 0; i < 3; i++)
    {
        mHipJoint->setConstraintImpulse(i,
            mHipJoint->getConstraintImpulse(i) + mJacobian[i] * con_force);
    }
    mKneeJoint->setConstraintImpulse(0,
            mKneeJoint->getConstraintImpulse(0) + mJacobian[3] * con_force);
    for (std::size_t i = 0; i < 2; i++)
    {
        mAnkleJoint->setConstraintImpulse(i,
            mAnkleJoint->getConstraintImpulse(i) + mJacobian[4+i] * con_force);
    }
}

//==============================================================================
dynamics::SkeletonPtr HumanLegJointLimitConstraint::getRootSkeleton() const
{
    return mHipJoint->getSkeleton()->mUnionRootSkeleton.lock();
}

//==============================================================================
bool HumanLegJointLimitConstraint::isActive() const
{
    return mActive;
}

} // namespace constraint
} // namespace dart
