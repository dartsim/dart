/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/Entity.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
typedef std::set<Entity*> EntityPtrSet;

//==============================================================================
Entity::Entity(Frame* _refFrame, bool _quiet)
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
    mAmQuiet(_quiet),
    mAmFrame(false)
{
  changeParentFrame(_refFrame);
}

//==============================================================================
Entity::Entity()
  : Entity(ConstructAbstract)
{
  // Delegated to Entity(ConstructAbstract_t)
}

//==============================================================================
Entity::~Entity()
{
  changeParentFrame(nullptr);
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
Entity::Entity(ConstructFrameTag)
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
Entity::Entity(ConstructAbstractTag)
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

  if(mParentFrame)
    mFrameChangedSignal.raise(this, oldParentFrame, mParentFrame);
}

//==============================================================================
Detachable::Detachable(Frame* _refFrame, bool _quiet)
  : Entity(_refFrame, _quiet)
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

