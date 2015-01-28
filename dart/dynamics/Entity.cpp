/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/dynamics/Entity.h"
#include "dart/dynamics/Frame.h"
#include "dart/dynamics/Shape.h"

#include "dart/renderer/RenderInterface.h"

namespace dart {
namespace dynamics {

typedef std::set<Entity*> EntityPtrSet;

//==============================================================================
Entity::Entity(Frame* _refFrame, const std::string& _name, bool _quiet)
  : mParentFrame(NULL),
    mName(_name),
    mNeedTransformUpdate(true),
    mNeedVelocityUpdate(true),
    mNeedAccelerationUpdate(true),
    mAmQuiet(_quiet)
{
  changeParentFrame(_refFrame);
}

//==============================================================================
Entity::~Entity()
{
  changeParentFrame(NULL);
}

//==============================================================================
const std::string& Entity::setName(const std::string &_name)
{
  mName = _name;
  return mName;
}

//==============================================================================
const std::string& Entity::getName() const
{
  return mName;
}

//==============================================================================
void Entity::addVisualizationShape(Shape* _p)
{
  mVizShapes.push_back(_p);
}

//==============================================================================
void Entity::draw(renderer::RenderInterface *_ri, const Eigen::Vector4d &_color,
                  bool _useDefaultColor, int) const
{
  if(NULL == _ri)
    return;

//  _ri->pushMatrix();
//  _ri->transform(mParentFrame->getTransform());
  // ^ I am skeptical about this. Shouldn't the matrix be pushed by its parent
  // frame? And then we're not popping this matrix at the end of this function.
  // This all seems questionable to me.

  // _ri->pushName(???); TODO(MXG): How should this pushName be handled for entities?
  for(size_t i=0; i < mVizShapes.size(); ++i)
  {
    _ri->pushMatrix();
    mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
    _ri->popMatrix();
  }
  // _ri->popName();
}

//==============================================================================
Frame* Entity::getParentFrame()
{
  return mParentFrame;
}

//==============================================================================
const Frame* Entity::getParentFrame() const
{
  return mParentFrame;
}

//==============================================================================
bool Entity::dependsOn(const Frame *_someFrame) const
{
  if(_someFrame == NULL)
    return false;

  if(_someFrame == this)
    return true;

  if(_someFrame->isWorld())
    return true;

  const Frame* descentCheck = getParentFrame();
  while(descentCheck)
  {
    if(descentCheck->isWorld())
      break;

    if(descentCheck == _someFrame)
      return true;
    descentCheck = descentCheck->getParentFrame();
  }

  return false;
}

//==============================================================================
bool Entity::isQuiet() const
{
  return mAmQuiet;
}

//==============================================================================
void Entity::notifyTransformUpdate()
{
  mNeedTransformUpdate = true;
}

//==============================================================================
void Entity::notifyVelocityUpdate()
{
  mNeedVelocityUpdate = true;
}

//==============================================================================
void Entity::notifyAccelerationUpdate()
{
  mNeedAccelerationUpdate = true;
}

//==============================================================================
void Entity::changeParentFrame(Frame* _newParentFrame)
{
  if(!mAmQuiet)
  {
    if(mParentFrame)
    {
      EntityPtrSet::iterator it = mParentFrame->mChildEntities.find(this);
      if(it != mParentFrame->mChildEntities.end())
        mParentFrame->mChildEntities.erase(it);
    }
  }

  if(_newParentFrame == NULL)
  {
    mParentFrame = NULL;
    return;
  }

  mParentFrame =_newParentFrame;
  if(!mAmQuiet)
  {
    mParentFrame->mChildEntities.insert(this);
    mParentFrame->processNewEntity(this);
    notifyTransformUpdate();
  }
}

//==============================================================================
Detachable::Detachable(Frame *_refFrame, const std::string &_name, bool _quiet)
  : Entity(_refFrame, _name, _quiet)
{

}

//==============================================================================
void Detachable::setParentFrame(Frame* _newParentFrame)
{
  changeParentFrame(_newParentFrame);
}


} // namespace dynamics
} // namespace dart
