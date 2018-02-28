/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "HumanLegJointLimitConstraint.hpp"

#include <iostream>

#include <dart/external/odelcpsolver/lcp.h>

#include <dart/common/Console.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM 1e-9

using namespace dart;
using namespace tiny_dnn;
using namespace tiny_dnn::activation;
using namespace tiny_dnn::layers;

double HumanLegJointLimitConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
double HumanLegJointLimitConstraint::mErrorReductionParameter = DART_ERP;
double HumanLegJointLimitConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double HumanLegJointLimitConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
HumanLegJointLimitConstraint::HumanLegJointLimitConstraint(
    dynamics::Joint* hipjoint,
    dynamics::Joint* _kneejoint,
    dynamics::Joint* _anklejoint,
    bool isMirror)
  : ConstraintBase(),
    mHipJoint(hipjoint),
    mKneeJoint(_kneejoint),
    mAnkleJoint(_anklejoint),
    mThighNode(hipjoint->getChildBodyNode()),
    mLowerLegNode(_kneejoint->getChildBodyNode()),
    mFootNode(_anklejoint->getChildBodyNode()),
    mIsMirror(isMirror),
    mAppliedImpulseIndex(0)
{
  assert(mHipJoint);
  assert(mKneeJoint);
  assert(mAnkleJoint);
  assert(mThighNode);
  assert(mLowerLegNode);
  assert(mFootNode);

  assert(mHipJoint->getNumDofs() == 3);
  assert(mKneeJoint->getNumDofs() == 1);
  assert(mAnkleJoint->getNumDofs() == 2);

  assert(mThighNode->getSkeleton() == mLowerLegNode->getSkeleton());
  assert(mLowerLegNode->getSkeleton() == mFootNode->getSkeleton());
  assert(mKneeJoint->getParentBodyNode() == mThighNode);
  assert(mAnkleJoint->getParentBodyNode() == mLowerLegNode);

  mLifeTime = 0;
  mActive = false;

  // load neural net weights from external file
  mNet.load(DART_DATA_PATH "/humanJointLimits/neuralnets/net-lleg");
}

//==============================================================================
void HumanLegJointLimitConstraint::setErrorAllowance(double allowance)
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
double HumanLegJointLimitConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void HumanLegJointLimitConstraint::setErrorReductionParameter(double erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (erp < 0.0)
  {
    dtwarn << "Error reduction parameter[" << erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (erp > 1.0)
  {
    dtwarn << "Error reduction parameter[" << erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = erp;
}

//==============================================================================
double HumanLegJointLimitConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void HumanLegJointLimitConstraint::setMaxErrorReductionVelocity(double erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity[" << erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = erv;
}

//==============================================================================
double HumanLegJointLimitConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void HumanLegJointLimitConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }
  if (cfm > 1.0)
  {
    dtwarn << "Constraint force mixing parameter[" << cfm
           << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mConstraintForceMixing = 1.0;
  }

  mConstraintForceMixing = cfm;
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
    qz = -qz;
    qy = -qy;
  }

  double qsin[8] = {cos(qz),
                    sin(qz),
                    cos(qx),
                    sin(qx),
                    cos(qy + M_PI_2),
                    cos(qe),
                    cos(hx + M_PI_2),
                    cos(hy + M_PI_2)};
  vec_t input;
  input.assign(qsin, qsin + 8);
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
    for (int n = mNet.layer_size() - 1; n >= 0; n--)
    {
      // implement chain rule layer by layer
      l = mNet[n];
      if (l->layer_type() == "fully-connected")
      {
        auto Wb = l->weights();
        vec_t W = *(Wb[0]);
        in_grad.assign(W.size() / out_grad.size(), 0);
        for (size_t c = 0; c < in_grad.size(); c++)
        {
          in_grad[c] = vectorize::dot(
              &out_grad[0], &W[c * out_grad.size()], out_grad.size());
        }
      }
      else
      {
        // this is activation layer
        std::vector<const tensor_t*> out_t;
        l->output(out_t);
        vec_t out_v = (*(out_t[0]))[0];

        in_grad.assign(out_grad.begin(), out_grad.end());
        // first arg (x) is used only to infer the size of input, which should
        // be the same as output y
        (dynamic_cast<activation_layer*>(l))
            ->backward_activation(out_v, out_v, in_grad, out_grad);
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

    // note that we also need to take the mirror of the NN gradient for
    // right-leg
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
void HumanLegJointLimitConstraint::getInformation(
    constraint::ConstraintInfo* lcp)
{
  // if non-active, should not call getInfo()
  assert(isActive());

  // assume caller will allocate enough space for _lcp variables
  assert(lcp->w[0] == 0.0);
  assert(lcp->findex[0] == -1);

  double bouncingVel = -mViolation - mErrorAllowance;
  if (bouncingVel < 0.0)
  {
    bouncingVel = 0.0;
  }
  bouncingVel *= lcp->invTimeStep * mErrorReductionParameter;
  if (bouncingVel > mMaxErrorReductionVelocity)
    bouncingVel = mMaxErrorReductionVelocity;

  lcp->b[0] = mNegativeVel + bouncingVel;
  lcp->lo[0] = mLowerBound;
  lcp->hi[0] = mUpperBound;

  if (mLifeTime)
    lcp->x[0] = mOldX;
  else
    lcp->x[0] = 0.0;
}

//==============================================================================
void HumanLegJointLimitConstraint::applyUnitImpulse(std::size_t index)
{
  // the dim of constraint = 1, valid _index can only be 0
  assert(index < mDim && "Invalid Index.");
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
    mAnkleJoint->setConstraintImpulse(i, mJacobian[4 + i]);
  }

  // Use the body which is placed later in the list of body nodes in this
  // skeleton
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

  mAppliedImpulseIndex = index; // which is 0
}

//==============================================================================
void HumanLegJointLimitConstraint::getVelocityChange(
    double* delVel, bool withCfm)
{
  assert(delVel != nullptr && "Null pointer is not allowed.");
  delVel[0] = 0.0;

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
      delq_d[4 + i] = mAnkleJoint->getVelocityChange(i);
    }
    delVel[0] = mJacobian.dot(delq_d);
  }

  if (withCfm)
  {
    delVel[mAppliedImpulseIndex]
        += delVel[mAppliedImpulseIndex] * mConstraintForceMixing;
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
void HumanLegJointLimitConstraint::applyImpulse(double* lambda)
{
  assert(isActive());

  // the dim of constraint = 1
  auto con_force = lambda[0];
  mOldX = con_force;

  for (std::size_t i = 0; i < 3; i++)
  {
    mHipJoint->setConstraintImpulse(
        i, mHipJoint->getConstraintImpulse(i) + mJacobian[i] * con_force);
  }
  mKneeJoint->setConstraintImpulse(
      0, mKneeJoint->getConstraintImpulse(0) + mJacobian[3] * con_force);
  for (std::size_t i = 0; i < 2; i++)
  {
    mAnkleJoint->setConstraintImpulse(
        i, mAnkleJoint->getConstraintImpulse(i) + mJacobian[4 + i] * con_force);
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
