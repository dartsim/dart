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

#include "dart/dynamics/EndEffector.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
EndEffectorProperties::EndEffectorProperties(const Eigen::Isometry3d& defaultTf)
  : mDefaultTransform(defaultTf)
{
  // Do nothing
}

//==============================================================================
void SupportUpdate(Support* support)
{
  if (EndEffector* ee = support->getComposite())
    ee->getSkeleton()->dirtySupportPolygon(ee->getTreeIndex());
}

} // namespace detail

//==============================================================================
void Support::setActive(bool _supporting)
{
  if (mState.mActive == _supporting)
    return;

  mState.mActive = _supporting;
  UpdateState(this);
}

//==============================================================================
bool Support::isActive() const
{
  return mState.mActive;
}

//==============================================================================
void EndEffector::setProperties(const BasicProperties& properties)
{
  setCompositeProperties(properties);
}

//==============================================================================
void EndEffector::setProperties(const UniqueProperties& properties, bool useNow)
{
  setDefaultRelativeTransform(properties.mDefaultTransform, useNow);
}

//==============================================================================
void EndEffector::setAspectProperties(const AspectProperties& properties)
{
  setDefaultRelativeTransform(properties.mDefaultTransform);
}

//==============================================================================
EndEffector::Properties EndEffector::getEndEffectorProperties() const
{
  return getCompositeProperties();
}

//==============================================================================
void EndEffector::copy(const EndEffector& otherEndEffector)
{
  if (this == &otherEndEffector)
    return;

  setCompositeState(otherEndEffector.getCompositeState());
  setCompositeProperties(otherEndEffector.getCompositeProperties());
}

//==============================================================================
void EndEffector::copy(const EndEffector* _otherEndEffector)
{
  if (nullptr == _otherEndEffector)
    return;

  copy(*_otherEndEffector);
}

//==============================================================================
EndEffector& EndEffector::operator=(const EndEffector& _otherEndEffector)
{
  copy(_otherEndEffector);
  return *this;
}

//==============================================================================
void EndEffector::setDefaultRelativeTransform(
    const Eigen::Isometry3d& _newDefaultTf, bool _useNow)
{
  mAspectProperties.mDefaultTransform = _newDefaultTf;

  if (_useNow)
    resetRelativeTransform();
}

//==============================================================================
void EndEffector::resetRelativeTransform()
{
  setRelativeTransform(mAspectProperties.mDefaultTransform);
}

//==============================================================================
void EndEffector::dirtyTransform()
{
  if (!mNeedTransformUpdate)
  {
    const SkeletonPtr& skel = getSkeleton();
    if (skel)
      skel->dirtySupportPolygon(getTreeIndex());
  }

  Frame::dirtyTransform();
}

//==============================================================================
EndEffector::EndEffector(BodyNode* parent, const BasicProperties& properties)
  : Entity(ConstructFrame),
    Frame(parent),
    FixedFrame(parent, properties.mDefaultTransform),
    common::EmbedPropertiesOnTopOf<
        EndEffector,
        detail::EndEffectorProperties,
        detail::EndEffectorCompositeBase>(
        std::make_tuple(parent, properties.mDefaultTransform), common::NoArg)
{
  setProperties(properties);
}

//==============================================================================
Node* EndEffector::cloneNode(BodyNode* _parent) const
{
  EndEffector* ee = new EndEffector(_parent, Properties());
  ee->duplicateAspects(this);

  ee->copy(this);

  if (mIK)
    ee->mIK = mIK->clone(ee);

  return ee;
}

} // namespace dynamics
} // namespace dart
