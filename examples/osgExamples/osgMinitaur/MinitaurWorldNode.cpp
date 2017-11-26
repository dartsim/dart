/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include "MinitaurWorldNode.hpp"

//==============================================================================
MinitaurWorldNode::MinitaurWorldNode(
    const dart::simulation::WorldPtr& world,
    const dart::dynamics::SkeletonPtr& atlas)
  : dart::gui::osg::WorldNode(world),
    mExternalForce(Eigen::Vector3d::Zero()),
    mForceDuration(0.0)
{
  assert(world);
  assert(atlas);

  mController.reset(new Controller(atlas, world->getConstraintSolver()));
}

//==============================================================================
void MinitaurWorldNode::customPreStep()
{
  auto pelvis = mController->getAtlasRobot()->getBodyNode("base_chassis_link");
  pelvis->addExtForce(mExternalForce);
  mController->update();

  if (mForceDuration > 0)
    mForceDuration--;
  else
    mExternalForce.setZero();
}

//==============================================================================
void MinitaurWorldNode::reset()
{
  mExternalForce.setZero();
  mController->resetRobot();
}

//==============================================================================
void MinitaurWorldNode::pushForwardAtlas(double force, int frames)
{
  mExternalForce.x() = force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::pushBackwardAtlas(double force, int frames)
{
  mExternalForce.x() = -force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::pushLeftAtlas(double force, int frames)
{
  mExternalForce.z() = force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::pushRightAtlas(double force, int frames)
{
  mExternalForce.z() = -force;
  mForceDuration = frames;
}

//==============================================================================
void MinitaurWorldNode::switchToNormalStrideWalking()
{
  mController->changeStateMachine("walking", mWorld->getTime());
}

//==============================================================================
void MinitaurWorldNode::switchToWarmingUp()
{
  mController->changeStateMachine("warming up", mWorld->getTime());
}

//==============================================================================
void MinitaurWorldNode::switchToNoControl()
{
  mController->changeStateMachine("standing", mWorld->getTime());
}
