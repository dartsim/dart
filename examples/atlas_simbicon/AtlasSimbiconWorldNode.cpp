/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "AtlasSimbiconWorldNode.hpp"

#include <osgShadow/ShadowMap>

//==============================================================================
AtlasSimbiconWorldNode::AtlasSimbiconWorldNode(
    const dart::simulation::WorldPtr& world,
    const dart::dynamics::SkeletonPtr& atlas)
  : dart::gui::osg::RealTimeWorldNode(world),
    mExternalForce(Eigen::Vector3d::Zero()),
    mForceDuration(0.0)
{
  assert(world);
  assert(atlas);

  mController.reset(new Controller(atlas, world->getConstraintSolver()));
}

//==============================================================================
void AtlasSimbiconWorldNode::customPreStep()
{
  auto pelvis = mController->getAtlasRobot()->getBodyNode("pelvis");
  pelvis->addExtForce(mExternalForce);
  mController->update();

  if (mForceDuration > 0)
    mForceDuration--;
  else
    mExternalForce.setZero();
}

//==============================================================================
void AtlasSimbiconWorldNode::reset()
{
  mExternalForce.setZero();
  mController->resetRobot();
}

//==============================================================================
void AtlasSimbiconWorldNode::pushForwardAtlas(double force, int frames)
{
  mExternalForce.x() = force;
  mForceDuration = frames;
}

//==============================================================================
void AtlasSimbiconWorldNode::pushBackwardAtlas(double force, int frames)
{
  mExternalForce.x() = -force;
  mForceDuration = frames;
}

//==============================================================================
void AtlasSimbiconWorldNode::pushLeftAtlas(double force, int frames)
{
  mExternalForce.z() = force;
  mForceDuration = frames;
}

//==============================================================================
void AtlasSimbiconWorldNode::pushRightAtlas(double force, int frames)
{
  mExternalForce.z() = -force;
  mForceDuration = frames;
}

//==============================================================================
void AtlasSimbiconWorldNode::switchToNormalStrideWalking()
{
  mController->changeStateMachine("walking", mWorld->getTime());
}

//==============================================================================
void AtlasSimbiconWorldNode::switchToShortStrideWalking()
{
  mController->changeStateMachine("running", mWorld->getTime());
}

//==============================================================================
void AtlasSimbiconWorldNode::switchToNoControl()
{
  mController->changeStateMachine("standing", mWorld->getTime());
}

//==============================================================================
void AtlasSimbiconWorldNode::showShadow()
{
  auto shadow
      = dart::gui::osg::WorldNode::createDefaultShadowTechnique(mViewer);
  if (auto sm = dynamic_cast<::osgShadow::ShadowMap*>(shadow.get()))
  {
    auto mapResolution = static_cast<short>(std::pow(2, 12));
    sm->setTextureSize(::osg::Vec2s(mapResolution, mapResolution));
  }

  setShadowTechnique(shadow);
}

//==============================================================================
void AtlasSimbiconWorldNode::hideShadow()
{
  setShadowTechnique(nullptr);
}
