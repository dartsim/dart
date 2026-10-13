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

#include "demo_models/RelaxedPosture.hpp"

#include <algorithm>

namespace dart_demos {

RelaxedPosture::RelaxedPosture(
    const Eigen::VectorXd& idealPosture,
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper,
    const Eigen::VectorXd& weights)
  : enforceIdealPosture(false),
    mIdeal(idealPosture),
    mLower(lower),
    mUpper(upper),
    mWeights(weights)
{
  mResultVector.setZero(mIdeal.size());
}

double RelaxedPosture::eval(const Eigen::VectorXd& x)
{
  computeResultVector(x);
  return 0.5 * mResultVector.dot(mResultVector);
}

void RelaxedPosture::evalGradient(
    const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad)
{
  computeResultVector(x);
  grad.setZero();
  const int smaller = std::min<int>(mResultVector.size(), grad.size());
  for (int i = 0; i < smaller; ++i)
    grad[i] = mResultVector[i];
}

Eigen::Index RelaxedPosture::getNumDofs() const
{
  return mIdeal.size();
}

void RelaxedPosture::computeResultVector(const Eigen::VectorXd& x)
{
  mResultVector.setZero();
  if (enforceIdealPosture) {
    for (int i = 0; i < x.size() && i < mIdeal.size(); ++i)
      mResultVector[i] = mWeights[i] * (x[i] - mIdeal[i]);
  } else {
    for (int i = 0; i < x.size() && i < mIdeal.size(); ++i) {
      if (x[i] < mLower[i])
        mResultVector[i] = mWeights[i] * (x[i] - mLower[i]);
      else if (mUpper[i] < x[i])
        mResultVector[i] = mWeights[i] * (x[i] - mUpper[i]);
    }
  }
}

} // namespace dart_demos
