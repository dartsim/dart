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

#include "dart/dynamics/Entity.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/Shape.hpp"

#include "dart/renderer/RenderInterface.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
typedef std::set<Entity*> EntityPtrSet;

//==============================================================================
Entity::Properties::Properties(const std::string& _name)
  : mName(_name)
{
 // Do nothing
}

//==============================================================================
Entity::Entity(Frame* _refFrame, const std::string& _name, bool _quiet)
  : mEntityP(_name),
    mParentFrame(nullptr),
    mNeedTransformUpdate(true),
    mNeedVelocityUpdate(true),
    mNeedAccelerationUpdate(true),
    mFrameChangedSignal(),
    mNameChangedSignal(),
    mTransformUpdatedSignal(),
    mVelocityChangedSignal(),
    mAccelerationChangedSignal(),
    onFrameChanged(mFrameChangedSignal),
    onNameChanged(mNameChangedSignal),
    onTransformUpdated(mTransformUpdatedSignal),
    onVelocityChanged(mVelocityChangedSignal),
    onAccelerationChanged(mAccelerationChangedSignal),
    mAmQuiet(_quiet),
    mAmFrame(false)
{
  changeParentFrame(_refFrame);
}

//==============================================================================
Entity::~Entity()
{
  changeParentFrame(nullptr);
}

//==============================================================================
void Entity::setProperties(const Properties& _properties)
{
  // Set name
  setName(_properties.mName);
}

//==============================================================================
const Entity::Properties& Entity::getEntityProperties() const
{
  return mEntityP;
}

//==============================================================================
void Entity::copy(const Entity& _otherEntity)
{
  if(this == &_otherEntity)
    return;

  setProperties(_otherEntity.getEntityProperties());
}

//==============================================================================
void Entity::copy(const Entity *_otherEntity)
{
  if(nullptr == _otherEntity)
    return;

  copy(*_otherEntity);
}

//==============================================================================
Entity& Entity::operator=(const Entity& _otherEntity)
{
  copy(_otherEntity);
  return *this;
}

//==============================================================================
const std::string& Entity::setName(const std::string& _name)
{
  if (mEntityP.mName == _name)
    return mEntityP.mName;

  const std::string oldName = mEntityP.mName;
  mEntityP.mName = _name;
  mNameChangedSignal.raise(this, oldName, mEntityP.mName);

  return mEntityP.mName;
}

//==============================================================================
const std::string& Entity::getName() const
{
  return mEntityP.mName;
}

//==============================================================================
void Entity::draw(renderer::RenderInterface* /*_ri*/,
                  const Eigen::Vector4d& /*_color*/,
                  bool /*_useDefaultColor*/,
                  int /*depth*/) const
{
  // Do nothing
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
bool Entity::descendsFrom(const Frame *_someFrame) const
{
  if(nullptr == _someFrame)
    return true;

  if(this == _someFrame)
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
bool Entity::isFrame() const
{
  return mAmFrame;
}

//==============================================================================
void Entity::notifyTransformUpdate()
{
  mNeedTransformUpdate = true;

  // The actual transform hasn't updated yet. But when its getter is called,
  // the transformation will be updated automatically.
  mTransformUpdatedSignal.raise(this);
}

//==============================================================================
bool Entity::needsTransformUpdate() const
{
  return mNeedTransformUpdate;
}

//==============================================================================
void Entity::notifyVelocityUpdate()
{
  mNeedVelocityUpdate = true;

  // The actual velocity hasn't updated yet. But when its getter is called,
  // the velocity will be updated automatically.
  mVelocityChangedSignal.raise(this);
}

//==============================================================================
bool Entity::needsVelocityUpdate() const
{
  return mNeedVelocityUpdate;
}

//==============================================================================
void Entity::notifyAccelerationUpdate()
{
  mNeedAccelerationUpdate = true;

  // The actual acceleration hasn't updated yet. But when its getter is called,
  // the acceleration will be updated automatically.
  mAccelerationChangedSignal.raise(this);
}

//==============================================================================
bool Entity::needsAccelerationUpdate() const
{
  return mNeedAccelerationUpdate;
}

//==============================================================================
Entity::Entity(ConstructFrame_t)
  : mParentFrame(nullptr),
    mNeedTransformUpdate(true),
    mNeedVelocityUpdate(true),
    mNeedAccelerationUpdate(true),
    mFrameChangedSignal(),
    mNameChangedSignal(),
    mTransformUpdatedSignal(),
    mVelocityChangedSignal(),
    mAccelerationChangedSignal(),
    onFrameChanged(mFrameChangedSignal),
    onNameChanged(mNameChangedSignal),
    onTransformUpdated(mTransformUpdatedSignal),
    onVelocityChanged(mVelocityChangedSignal),
    onAccelerationChanged(mAccelerationChangedSignal),
    mAmQuiet(false),
    mAmFrame(false) // The Frame class will change this to true
{
  // Do nothing. The Frame class will take care of changing the parent Frame.
}

//==============================================================================
Entity::Entity(ConstructAbstract_t)
  : onFrameChanged(mFrameChangedSignal),
    onNameChanged(mNameChangedSignal),
    onTransformUpdated(mTransformUpdatedSignal),
    onVelocityChanged(mVelocityChangedSignal),
    onAccelerationChanged(mAccelerationChangedSignal),
    mAmQuiet(false)
{
  dterr << "[Entity::Entity] Your class implementation is calling the Entity "
        << "constructor that is meant to be reserved for abstract classes!\n";
  assert(false);
}

//==============================================================================
void Entity::changeParentFrame(Frame* _newParentFrame)
{
  if (mParentFrame == _newParentFrame)
    return;

  const Frame* oldParentFrame = mParentFrame;

  if (!mAmQuiet && nullptr != mParentFrame && !mParentFrame->isWorld())
  {
    // If this entity has a parent Frame, tell that parent that it is losing
    // this child
    EntityPtrSet::iterator it = mParentFrame->mChildEntities.find(this);
    if (it != mParentFrame->mChildEntities.end())
    {
      mParentFrame->mChildEntities.erase(it);
      mParentFrame->processRemovedEntity(this);
    }
  }

  mParentFrame =_newParentFrame;

  if (!mAmQuiet && nullptr != mParentFrame)
  {
    if(!mParentFrame->isWorld())
    {
      // The WorldFrame should not keep track of its children, or else we get
      // concurrency issues (race conditions).
      mParentFrame->mChildEntities.insert(this);
      mParentFrame->processNewEntity(this);
    }
    notifyTransformUpdate();
  }

  mFrameChangedSignal.raise(this, oldParentFrame, mParentFrame);
}

//==============================================================================
Detachable::Detachable(Frame* _refFrame, const std::string& _name, bool _quiet)
  : Entity(_refFrame, _name, _quiet)
{
  // Do nothing
}

//==============================================================================
void Detachable::setParentFrame(Frame* _newParentFrame)
{
  changeParentFrame(_newParentFrame);
}

//==============================================================================
Detachable::Detachable()
  : Entity(ConstructAbstract)
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart

