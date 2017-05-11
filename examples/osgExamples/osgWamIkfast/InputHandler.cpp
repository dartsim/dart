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

#include "InputHandler.hpp"

#include "WamWorld.hpp"
#include "RelaxedPosture.hpp"

//==============================================================================
InputHandler::InputHandler(
    dart::gui::osg::Viewer* viewer,
    WamWorld* teleop,
    const SkeletonPtr& wam,
    const WorldPtr& world)
  : mViewer(viewer),
    mTeleop(teleop),
    mWam(wam),
    mWorld(world)
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

  mPosture = std::dynamic_pointer_cast<RelaxedPosture>(
        mWam->getIK(true)->getObjective());

  mBalance = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
        mWam->getIK(true)->getProblem()->getEqConstraint(1));

  mOptimizationKey = 'r';

  mMoveComponents.resize(WamWorld::NUM_MOVE, false);
}

//==============================================================================
bool InputHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
  if (nullptr == mWam)
  {
    return false;
  }

  if ( ::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType() )
  {
    if ( ea.getKey() == 'p' )
    {
      for (std::size_t i=0; i < mWam->getNumDofs(); ++i)
        std::cout << mWam->getDof(i)->getName() << ": "
                  << mWam->getDof(i)->getPosition() << std::endl;
      return true;
    }

    if ( ea.getKey() == 't' )
    {
      // Reset all the positions except for x, y, and yaw
      for (std::size_t i=0; i < mWam->getNumDofs(); ++i)
      {
        if ( i < 2 || 4 < i )
          mWam->getDof(i)->setPosition(mRestConfig[i]);
      }
      return true;
    }

    if ( '1' <= ea.getKey() && ea.getKey() <= '9' )
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

    if ( 'x' == ea.getKey() )
    {
      EndEffector* ee = mWam->getEndEffector("l_foot");
      ee->getSupport()->setActive(!ee->getSupport()->isActive());
      return true;
    }

    if ( 'c' == ea.getKey() )
    {
      EndEffector* ee = mWam->getEndEffector("r_foot");
      ee->getSupport()->setActive(!ee->getSupport()->isActive());
      return true;
    }

    if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
      mTeleop->mAmplifyMovement = true;

    switch(ea.getKey())
    {
      case 'w': case 'W': mMoveComponents[WamWorld::MOVE_W] = true; break;
      case 'a': case 'A': mMoveComponents[WamWorld::MOVE_A] = true; break;
      case 's': case 'S': mMoveComponents[WamWorld::MOVE_S] = true; break;
      case 'd': case 'D': mMoveComponents[WamWorld::MOVE_D] = true; break;
      case 'q': case 'Q': mMoveComponents[WamWorld::MOVE_Q] = true; break;
      case 'e': case 'E': mMoveComponents[WamWorld::MOVE_E] = true; break;
      case 'f': case 'F': mMoveComponents[WamWorld::MOVE_F] = true; break;
      case 'z': case 'Z': mMoveComponents[WamWorld::MOVE_Z] = true; break;
    }

    switch(ea.getKey())
    {
      case 'w': case 'a': case 's': case 'd': case 'q': case 'e': case 'f': case 'z':
      case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
      {
        mTeleop->setMovement(mMoveComponents);
        return true;
      }
    }

    if (mOptimizationKey == ea.getKey())
    {
      if (mPosture)
        mPosture->enforceIdealPosture = true;

      if (mBalance)
        mBalance->setErrorMethod(dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);

      return true;
    }
  }

  if ( ::osgGA::GUIEventAdapter::KEYUP == ea.getEventType() )
  {
    if (ea.getKey() == mOptimizationKey)
    {
      if (mPosture)
        mPosture->enforceIdealPosture = false;

      if (mBalance)
        mBalance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);

      return true;
    }

    if (ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
      mTeleop->mAmplifyMovement = false;

    switch(ea.getKey())
    {
      case 'w': case 'W': mMoveComponents[WamWorld::MOVE_W] = false; break;
      case 'a': case 'A': mMoveComponents[WamWorld::MOVE_A] = false; break;
      case 's': case 'S': mMoveComponents[WamWorld::MOVE_S] = false; break;
      case 'd': case 'D': mMoveComponents[WamWorld::MOVE_D] = false; break;
      case 'q': case 'Q': mMoveComponents[WamWorld::MOVE_Q] = false; break;
      case 'e': case 'E': mMoveComponents[WamWorld::MOVE_E] = false; break;
      case 'f': case 'F': mMoveComponents[WamWorld::MOVE_F] = false; break;
      case 'z': case 'Z': mMoveComponents[WamWorld::MOVE_Z] = false; break;
    }

    switch(ea.getKey())
    {
      case 'w': case 'a': case 's': case 'd': case 'q': case'e': case 'f': case 'z':
      case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
      {
        mTeleop->setMovement(mMoveComponents);
        return true;
      }
    }
  }

  return false;
}
