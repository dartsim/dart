/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/common/Console.h"
#include "dart/dynamics/EndEffector.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

EndEffector::EndEffector(Frame* _refFrame, const std::string& _name,
                         const Eigen::Isometry3d& _relativeTransform)
  : Entity(_refFrame, _name, false),
    FixedFrame(_refFrame, _name, _relativeTransform),
    mParentBodyNode(nullptr),
    mDefaultTransform(_relativeTransform),
    mIndexInSkeleton(0)
{
  identifyParentBodyNode(_refFrame);
  registerWithSkeleton();
}

//==============================================================================
EndEffector::~EndEffector()
{
  // TODO(MXG): Should we notify the Skeleton that it's losing an EndEffector
  // or just assume that EndEffectors will only be destructed by the Skeleton?
}

//==============================================================================
void EndEffector::setRelativeTransform(const Eigen::Isometry3d& _newRelativeTf)
{
  mRelativeTf = _newRelativeTf;
  notifyTransformUpdate();
}

//==============================================================================
void EndEffector::setDefaultRelativeTransform(
    const Eigen::Isometry3d& _newDefaultTf, bool _useNow)
{
  mDefaultTransform = _newDefaultTf;

  if(_useNow)
    resetRelativeTransform();
}

//==============================================================================
void EndEffector::resetRelativeTransform()
{
  setRelativeTransform(mDefaultTransform);
}

//==============================================================================
BodyNode* EndEffector::getParentBodyNode()
{
  return mParentBodyNode;
}

//==============================================================================
const BodyNode* EndEffector::getParentBodyNode() const
{
  return mParentBodyNode;
}

//==============================================================================
Skeleton* EndEffector::getSkeleton()
{
  if(mParentBodyNode)
    return mParentBodyNode->getSkeleton();

  return nullptr;
}

//==============================================================================
const Skeleton* EndEffector::getSkeleton() const
{
  if(mParentBodyNode)
    return mParentBodyNode->getSkeleton();

  return nullptr;
}

//==============================================================================
size_t EndEffector::getIndex() const
{
  return mIndexInSkeleton;
}

//==============================================================================
void EndEffector::identifyParentBodyNode(Frame* _refFrame)
{
  Frame* checkFrame = _refFrame;

  while( !checkFrame->isWorld() )
  {
    mParentBodyNode = dynamic_cast<BodyNode*>(checkFrame);
    if(mParentBodyNode)
      return;

    checkFrame = checkFrame->getParentFrame();
  }

  dtwarn << "[EndEffector::identifyParentBodyNode] Could not find a parent "
         << "BodyNode to attach the EndEffector named '" << getName() << "' to. "
         << "The Frame named '" << _refFrame->getName() << "' was given as a "
         << "reference Frame.\n";
}

//==============================================================================
void EndEffector::registerWithSkeleton()
{
  if(!mParentBodyNode)
    return;

  Skeleton* skel = mParentBodyNode->getSkeleton();
  if(!skel)
    return;

  std::vector<EndEffector*>::iterator it = find(skel->mEndEffectors.begin(),
                                                skel->mEndEffectors.end(), this);
  if(it != skel->mEndEffectors.end())
  {
    dtwarn << "[EndEffector::registerWithSkeleton] Attempting to add the "
           << "EndEffector named '" << getName() << "' to the Skeleton named '"
           << skel->getName() << "' even though it has already been added!\n";
    return;
  }

  mIndexInSkeleton = skel->mEndEffectors.push_back(this) - 1;
  skel->addEntryToEndEffectorNameMgr(this);
}

} // namespace dynamics
} // namespace dart
