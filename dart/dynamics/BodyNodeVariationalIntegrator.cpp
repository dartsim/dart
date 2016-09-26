/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/BodyNodeVariationalIntegrator.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
BodyNodeVariationalIntegratorState::BodyNodeVariationalIntegratorState()
{
  // Do nothing
}

} // namespace detail

//==============================================================================
BodyNodeVariationalIntegrator::BodyNodeVariationalIntegrator(
    const StateData& state)
{
  mState = state;
}

//==============================================================================
void BodyNodeVariationalIntegrator::initialize(double timeStep)
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

//  mDiscreteJS.reset(new DiscreteMechanicsJS());
//  mDiscreteJS->prevMomentum = math::dexp_inv_transpose(V*timeStep, G*V);

//  joint->dm_js_impulse_initialize(timeStep);
}

//==============================================================================
void BodyNodeVariationalIntegrator::print()
{
  //std::cout << mState.mV_q << std::endl;;
}

//==============================================================================
void BodyNodeVariationalIntegrator::setComposite(
    common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  const auto* bodyNode = dynamic_cast<BodyNode*>(newComposite);
  const auto skeleton = bodyNode->getSkeleton();
  const auto numDofs = skeleton->getNumDofs();

  assert(skeleton);

//  mState.mV_q.resize(6, numDofs);
//  mState.mV_dq.resize(6, numDofs);

//  mState.mV_q_q.resize(numDofs);
//  mState.mV_q_dq.resize(numDofs);
//  mState.mV_dq_dq.resize(numDofs);

//  for (auto i = 0u; i < numDofs; ++i)
//  {
//    mState.mV_q_q[i].resize(6, numDofs);
//    mState.mV_q_dq[i].resize(6, numDofs);
//    mState.mV_dq_dq[i].resize(6, numDofs);
//  }

  // TODO(JS): These should be updated when the structure of skeleton is
  // modified. We might want to consider adding signal to skeleton for this.

  auto* joint = mComposite->getParentJoint();
  if (joint->is<RevoluteJoint>())
  {

  }
  else
  {
    dtwarn << "WARN\n";
  }
}

} // namespace dynamics
} // namespace dart
