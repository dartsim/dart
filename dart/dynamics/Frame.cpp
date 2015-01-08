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

typedef std::set<Entity*> EntityPtrSet;
typedef std::set<Frame*> FramePtrSet;

Frame::Frame(const Frame *_refFrame, const std::string &_name) :
  Entity(_refFrame, _name, false),
  mWorldTransform(Eigen::Isometry3d::Identity()),
  mVelocity(Eigen::Vector6d::Zero()),
  mAcceleration(Eigen::Vector6d::Zero()),
  mAmWorld(false)
{

}

//==============================================================================
Frame::~Frame()
{
  if(isWorld())
    return;

//  changeParentFrame(NULL);

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
Eigen::Isometry3d Frame::getTransform(const Frame* _withRespectTo) const
{
  if(_withRespectTo->isWorld())
    return getWorldTransform();
  else if(_withRespectTo == mParentFrame)
    return getRelativeTransform();

  return _withRespectTo->getWorldTransform().inverse()*getWorldTransform();
}

//==============================================================================
const Eigen::Vector6d& Frame::getSpatialVelocity() const
{
  if(mAmWorld)
    return mVelocity;

  if(mNeedVelocityUpdate)
  {
    mVelocity = math::AdInvT(getRelativeTransform(),
                             getParentFrame()->getSpatialVelocity())
                + getRelativeSpatialVelocity();

    mNeedVelocityUpdate = false;
  }

  return mVelocity;
}

//==============================================================================
Eigen::Vector6d Frame::getSpatialVelocity(const Frame* _inCoordinatesOf) const
{
  if(this==_inCoordinatesOf)
    return getSpatialVelocity();

  if(_inCoordinatesOf->isWorld())
    return math::AdR(getWorldTransform(), getSpatialVelocity());

  return math::AdR(getTransform(_inCoordinatesOf), getSpatialVelocity());
}

//==============================================================================
Eigen::Vector6d Frame::getSpatialVelocity(const Frame* _relativeTo,
                                          const Frame* _inCoordinatesOf) const
{
  return getSpatialVelocity(_inCoordinatesOf)
      - _relativeTo->getSpatialVelocity(_inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Frame::getLinearVelocity(const Frame* _relativeTo,
                                         const Frame* _inCoordinatesOf) const
{
  return getSpatialVelocity(_relativeTo,_inCoordinatesOf).tail<3>();
}

//==============================================================================
Eigen::Vector3d Frame::getAngularVelocity(const Frame* _relativeTo,
                                          const Frame* _inCoordinatesOf) const
{
  return getSpatialVelocity(_relativeTo,_inCoordinatesOf).head<3>();
}

//==============================================================================
const Eigen::Vector6d& Frame::getSpatialAcceleration() const
{
  if(mAmWorld)
    return mAcceleration;

  if(mNeedAccelerationUpdate)
  {
    mAcceleration = math::AdInvT(getRelativeTransform(),
                                 getParentFrame()->getSpatialAcceleration())
                + getRelativeSpatialAcceleration()
                + math::ad(getSpatialVelocity(), getRelativeSpatialVelocity());

    mNeedAccelerationUpdate = false;
  }

  return mAcceleration;
}

//==============================================================================
Eigen::Vector6d Frame::getSpatialAcceleration(
    const Frame* _inCoordinatesOf) const
{
  if(this==_inCoordinatesOf)
    return getSpatialAcceleration();

  if(_inCoordinatesOf->isWorld())
    return math::AdR(getWorldTransform(), getSpatialAcceleration());

  return math::AdR(getTransform(_inCoordinatesOf), getSpatialAcceleration());
}

//==============================================================================
Eigen::Vector6d Frame::getSpatialAcceleration(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  return getSpatialAcceleration(_inCoordinatesOf)
      - _relativeTo->getSpatialAcceleration(_inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Frame::getLinearAcceleration(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  const Eigen::Vector6d& v_rel = getSpatialVelocity(_relativeTo,
                                                    _inCoordinatesOf);

  // r'' = a + w x v
  return getSpatialAcceleration(_relativeTo,_inCoordinatesOf).tail<3>()
         + v_rel.head<3>().cross(v_rel.tail<3>());
}

//==============================================================================
Eigen::Vector3d Frame::getAngularAcceleration(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  return getSpatialAcceleration(_relativeTo, _inCoordinatesOf).head<3>();
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
const std::set<Entity*>& Frame::getChildEntities()
{
  return mChildEntities;
}

//==============================================================================
std::set<const Entity*> Frame::getChildEntities() const
{
  return convertToConstSet<Entity>(mChildEntities);
}

//==============================================================================
size_t Frame::getNumChildEntities() const
{
  return mChildEntities.size();
}

//==============================================================================
const std::set<Frame*>& Frame::getChildFrames()
{
  return mChildFrames;
}

//==============================================================================
std::set<const Frame*> Frame::getChildFrames() const
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
void Frame::draw(renderer::RenderInterface* _ri, const Eigen::Vector4d& _color,
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

  mNeedTransformUpdate = true;

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

  mNeedVelocityUpdate = true;

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

  mNeedAccelerationUpdate = true;

  EntityPtrSet::iterator it=mChildEntities.begin(), end=mChildEntities.end();
  for( ; it != end; ++it)
    (*it)->notifyAccelerationUpdate();
}

//==============================================================================
void Frame::changeParentFrame(const Frame* _newParentFrame)
{
  if(mParentFrame)
  {
    FramePtrSet::iterator it = mParentFrame->mChildFrames.find(this);
    if(it != mParentFrame->mChildFrames.end())
      mParentFrame->mChildFrames.erase(it);
  }

  if(NULL==_newParentFrame)
  {
    Entity::changeParentFrame(_newParentFrame);
    return;
  }

  Entity::changeParentFrame(_newParentFrame);
//  if(!mAmQuiet)
    mParentFrame->mChildFrames.insert(this);
}

//==============================================================================
Frame::Frame() :
  Entity(this, "World", false),
  mWorldTransform(Eigen::Isometry3d::Identity()),
  mVelocity(Eigen::Vector6d::Zero()),
  mAcceleration(Eigen::Vector6d::Zero()),
  mAmWorld(true)
{

}

//==============================================================================
const Eigen::Isometry3d& WorldFrame::getRelativeTransform() const
{
  return mRelativeTf;
}

//==============================================================================
const Eigen::Vector6d& WorldFrame::getRelativeSpatialVelocity() const
{
  return mRelativeVelocity;
}

//==============================================================================
const Eigen::Vector6d& WorldFrame::getRelativeSpatialAcceleration() const
{
  return mRelativeAcceleration;
}

//==============================================================================
WorldFrame::WorldFrame() :
  Entity(NULL, "World", false),
//  Entity(NULL, "World", true),
  Frame(),
  mRelativeTf(Eigen::Isometry3d::Identity()),
  mRelativeVelocity(Eigen::Vector6d::Zero()),
  mRelativeAcceleration(Eigen::Vector6d::Zero())
{
  changeParentFrame(this);
}

} // namespace dart
} // namespace dynamics
