/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "RelaxedPosture.hpp"

//==============================================================================
RelaxedPosture::RelaxedPosture(const Eigen::VectorXd& idealPosture, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, const Eigen::VectorXd& weights, bool enforceIdeal)
  : enforceIdealPosture(enforceIdeal),
    mIdeal(idealPosture),
    mLower(lower),
    mUpper(upper),
    mWeights(weights)
{
  int dofs = mIdeal.size();
  if (mLower.size() != dofs || mWeights.size() != dofs || mUpper.size() != dofs)
  {
    dterr << "[RelaxedPose::RelaxedPose] Dimension mismatch:\n"
          << "  ideal:   " << mIdeal.size()   << "\n"
          << "  lower:   " << mLower.size()   << "\n"
          << "  upper:   " << mUpper.size()   << "\n"
          << "  weights: " << mWeights.size() << "\n";
  }
  mResultVector.setZero(dofs);
}

//==============================================================================
double RelaxedPosture::eval(const Eigen::VectorXd& _x)
{
  computeResultVector(_x);
  return 0.5 * mResultVector.dot(mResultVector);
}

//==============================================================================
void RelaxedPosture::evalGradient(const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad)
{
  computeResultVector(_x);

  _grad.setZero();
  int smaller = std::min(mResultVector.size(), _grad.size());
  for (int i=0; i < smaller; ++i)
    _grad[i] = mResultVector[i];
}

//==============================================================================
void RelaxedPosture::computeResultVector(const Eigen::VectorXd& _x)
{
  mResultVector.setZero();

  if (enforceIdealPosture)
  {
    for (int i=0; i < _x.size(); ++i)
    {
      if (mIdeal.size() <= i)
        break;

      mResultVector[i] = mWeights[i]*(_x[i] - mIdeal[i]);
    }
  }
  else
  {
    for (int i=0; i < _x.size(); ++i)
    {
      if (mIdeal.size() <= i)
        break;

      if (_x[i] < mLower[i])
        mResultVector[i] = mWeights[i]*(_x[i] - mLower[i]);
      else if (mUpper[i] < _x[i])
        mResultVector[i] = mWeights[i]*(_x[i] - mUpper[i]);
    }
  }
}
