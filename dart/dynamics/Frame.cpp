/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "dart/dynamics/Frame.h"
#include "dart/dynamics/Shape.h"

#include "dart/renderer/RenderInterface.h"

namespace dart {
namespace dynamics {

Frame::Frame(const Frame *_refFrame, const std::string &_name) :
  Entity(_refFrame, _name),
  mWorldTransform(Eigen::Isometry3d::Identity()),
  mAmWorld(false)
{

}

//==============================================================================
Frame::~Frame()
{
  if(isWorld())
    return;

  // Inform all child entities that this Frame is disappearing by setting their
  // reference frames to the World frame.
  EntityPtrSet::iterator it=mChildEntities.begin(), end=mChildEntities.end();
  for( ; it != end; ++it)
    (*it)->changeParentFrame(Frame::World());

  // Free the memory of the visualization shapes
  for(size_t i=0; i<mVizShapes.size(); ++i)
    delete mVizShapes[i];
  mVizShapes.clear();

  // The entity destructor takes care of informing the parent Frame that this
  // one is disappearing
}

//==============================================================================
const Frame* Frame::World()
{
  static WorldFrame world;
  return &world;
}

//==============================================================================
const Eigen::Isometry3d& Frame::getWorldTransform() const
{
  if(mAmWorld)
    return mWorldTransform;

  if(mNeedTransformUpdate)
  {
    mWorldTransform = mParentFrame->getWorldTransform()*getRelativeTransform();
    mNeedTransformUpdate = false;
  }

  return mWorldTransform;
}

//==============================================================================
Eigen::Isometry3d Frame::getTransform(const Frame* withRespectTo) const
{
  if(withRespectTo->isWorld())
    return getWorldTransform();
  else if(withRespectTo == mParentFrame)
    return getRelativeTransform();

  return withRespectTo->getWorldTransform().inverse()*getWorldTransform();
}

//==============================================================================
template <typename T>
static std::set<const T*> convertToConstSet(const std::set<T*>& _set)
{
  std::set<const T*> const_set;
  typename std::set<T*>::const_iterator it=_set.begin(), end=_set.end();
  for( ; it != end; ++it)
    const_set.insert(*it);

  return const_set;
}

//==============================================================================
const EntityPtrSet& Frame::getChildEntities()
{
  return mChildEntities;
}

//==============================================================================
ConstEntityPtrSet Frame::getChildEntities() const
{
  return convertToConstSet<Entity>(mChildEntities);
}

//==============================================================================
size_t Frame::getNumChildEntities() const
{
  return mChildEntities.size();
}

//==============================================================================
const FramePtrSet& Frame::getChildFrames()
{
  return mChildFrames;
}

//==============================================================================
ConstFramePtrSet Frame::getChildFrames() const
{
  return convertToConstSet<Frame>(mChildFrames);
}

//==============================================================================
size_t Frame::getNumChildFrames() const
{
  return mChildFrames.size();
}

//==============================================================================
bool Frame::isWorld() const
{
  return mAmWorld;
}

//==============================================================================
void Frame::draw(renderer::RenderInterface *_ri, const Eigen::Vector4d &_color,
                 bool _useDefaultColor, int _depth) const
{
  if(NULL == _ri)
    return;

  _ri->pushMatrix();

  // Use the world transform of this Frame
  _ri->transform(getWorldTransform());

  // _ri->pushName(???); TODO(MXG): What should we do about this for Frames?
  for(size_t i=0; i < mVizShapes.size(); ++i)
  {
    _ri->pushMatrix();
    mVizShapes[i]->draw(_ri, _color, _useDefaultColor);
    _ri->popMatrix();
  }
  // _ri.popName();

  // render the subtree
  EntityPtrSet::const_iterator it=mChildEntities.begin();
  EntityPtrSet::const_iterator end=mChildEntities.end();
  for( ; it != end; ++it)
    (*it)->draw(_ri, _color, _useDefaultColor);

  _ri->popMatrix();
}

//==============================================================================
void Frame::notifyTransformUpdate()
{
  notifyVelocityUpdate(); // Global Velocity depends on the Global Transform

  // If we already know we need to update, just quit
  if(mNeedTransformUpdate)
    return;

  Entity::notifyTransformUpdate();

  EntityPtrSet::iterator it=mChildEntities.begin(), end=mChildEntities.end();
  for( ; it != end; ++it)
    (*it)->notifyTransformUpdate();
}

//==============================================================================
void Frame::notifyVelocityUpdate()
{
  notifyAccelerationUpdate(); // Global Acceleration depends on Global Velocity

  // If we already know we need to update, just quit
  if(mNeedVelocityUpdate)
    return;

  Entity::notifyVelocityUpdate();

  EntityPtrSet::iterator it=mChildEntities.begin(), end=mChildEntities.end();
  for( ; it != end; ++it)
    (*it)->notifyVelocityUpdate();
}

//==============================================================================
void Frame::notifyAccelerationUpdate()
{
  // If we already know we need to update, just quit
  if(mNeedAccelerationUpdate)
    return;

  Entity::notifyAccelerationUpdate();

  EntityPtrSet::iterator it=mChildEntities.begin(), end=mChildEntities.end();
  for( ; it != end; ++it)
    (*it)->notifyAccelerationUpdate();
}

//==============================================================================
void Frame::changeParentFrame(const Frame* _newParentFrame)
{
  mParentFrame->mChildFrames.erase(this);

  if(NULL==_newParentFrame)
  {
    Entity::changeParentFrame(_newParentFrame);
    return;
  }

  Entity::changeParentFrame(_newParentFrame);
  mParentFrame->mChildFrames.insert(this);
}

//==============================================================================
Frame::Frame() :
  Entity(this, "World"),
  mWorldTransform(Eigen::Isometry3d::Identity()),
  mAmWorld(true)
{

}

//==============================================================================
const Eigen::Isometry3d& WorldFrame::getRelativeTransform() const
{
  return mRelativeTf;
}

//==============================================================================
WorldFrame::WorldFrame() :
  Entity(this, "World"),
  Frame(),
  mRelativeTf(Eigen::Isometry3d::Identity())
{

}

//==============================================================================
PureFrame::PureFrame(const Frame* _refFrame, const std::string& _name,
                     const Eigen::Isometry3d& _relativeTransform) :
  Entity(_refFrame, _name),
  Frame(_refFrame, _name),
  Detachable(_refFrame, _name),
  mRelativeTf(_relativeTransform)
{

}

//==============================================================================
PureFrame::~PureFrame()
{

}

//==============================================================================
void PureFrame::setRelativeTransform(const Eigen::Isometry3d &_newRelTransform)
{
  mRelativeTf = _newRelTransform;
  notifyTransformUpdate();
}

//==============================================================================
const Eigen::Isometry3d& PureFrame::getRelativeTransform() const
{
  return mRelativeTf;
}

} // namespace dart
} // namespace dynamics
