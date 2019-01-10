/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "InputHandler.hpp"

#include "WamWorld.hpp"

//==============================================================================
InputHandler::InputHandler(
    dart::gui::osg::Viewer* viewer,
    WamWorld* teleop,
    const SkeletonPtr& wam,
    const WorldPtr& world)
  : mViewer(viewer), mWamWorld(teleop), mWam(wam), mWorld(world)
{
  initialize();
}

//==============================================================================
void InputHandler::initialize()
{
  mRestConfig = mWam->getPositions();

  for (std::size_t i=0; i < mWam->getNumEndEffectors(); ++i)
  {
    const InverseKinematicsPtr ik = mWam->getEndEffector(i)->getIK();
    if (ik)
    {
      mDefaultBounds.push_back(ik->getErrorMethod().getBounds());
      mDefaultTargetTf.push_back(ik->getTarget()->getRelativeTransform());
      mConstraintActive.push_back(false);
      mEndEffectorIndex.push_back(i);
    }
  }
}

//==============================================================================
bool InputHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
  if (!mWam)
    return false;

  if (::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType())
  {
    if (ea.getKey() == 'p' || ea.getKey() == 'P')
    {
      for (std::size_t i=0; i < mWam->getNumDofs(); ++i)
        std::cout << mWam->getDof(i)->getName() << ": "
                  << mWam->getDof(i)->getPosition() << std::endl;
      return true;
    }

    if (ea.getKey() == 't' || ea.getKey() == 'T')
    {
      mWam->setPositions(mRestConfig);
      return true;
    }

    if ('1' <= ea.getKey() && ea.getKey() <= '9')
    {
      std::size_t index = ea.getKey() - '1';
      if (index < mConstraintActive.size())
      {
        EndEffector* ee = mWam->getEndEffector(mEndEffectorIndex[index]);
        const InverseKinematicsPtr& ik = ee->getIK();
        if (ik && mConstraintActive[index])
        {
          mConstraintActive[index] = false;

          ik->getErrorMethod().setBounds(mDefaultBounds[index]);
          ik->getTarget()->setRelativeTransform(mDefaultTargetTf[index]);
          mWorld->removeSimpleFrame(ik->getTarget());
        }
        else if (ik)
        {
          mConstraintActive[index] = true;

          // Use the standard default bounds instead of our custom default
          // bounds
          ik->getErrorMethod().setBounds();
          ik->getTarget()->setTransform(ee->getTransform());
          mWorld->addSimpleFrame(ik->getTarget());
        }
      }
      return true;
    }
  }

  return false;
}
