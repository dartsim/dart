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

#include "dart/dynamics/SimpleFrame.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

//==============================================================================
SimpleFrame::UniqueProperties::UniqueProperties(
    const Eigen::Isometry3d& _relativeTf,
    const Eigen::Vector6d& _relativeVelocity,
    const Eigen::Vector6d& _relativeAcceleration)
  : mRelativeTf(_relativeTf),
    mRelativeVelocity(_relativeVelocity),
    mRelativeAcceleration(_relativeAcceleration)
{
  // Do nothing
}

//==============================================================================
SimpleFrame::Properties::Properties(
    const Entity::Properties& _entityProperties,
    const UniqueProperties& _frameProperties)
  : Entity::Properties(_entityProperties),
    UniqueProperties(_frameProperties)
{
  // Do nothing
}

//==============================================================================
SimpleFrame::SimpleFrame(Frame* _refFrame, const std::string& _name,
                         const Eigen::Isometry3d& _relativeTransform)
  : Entity(nullptr, _name, false),
    Detachable(nullptr, _name, false),
    Frame(_refFrame, _name),
    mFrameP(_relativeTransform),
    mPartialAcceleration(Eigen::Vector6d::Zero())
{
  // Do nothing
}

//==============================================================================
SimpleFrame::SimpleFrame(const SimpleFrame& _otherFrame, Frame* _refFrame)
  : Entity(nullptr, "", false),
    Detachable(nullptr, "", false),
    Frame(_refFrame, "")
{
  copy(_otherFrame);
}

//==============================================================================
SimpleFrame::~SimpleFrame()
{
  // Do nothing
}

//==============================================================================
void SimpleFrame::setProperties(const Properties& _properties)
{
  Entity::setProperties(static_cast<const Entity::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void SimpleFrame::setProperties(const UniqueProperties& _properties)
{
  setRelativeTransform(_properties.mRelativeTf);
  setRelativeSpatialVelocity(_properties.mRelativeVelocity);
  setRelativeSpatialAcceleration(_properties.mRelativeAcceleration);
}

//==============================================================================
SimpleFrame::Properties SimpleFrame::getSimpleFrameProperties() const
{
  return Properties(getEntityProperties(), mFrameP);
}

//==============================================================================
void SimpleFrame::copy(const SimpleFrame& _otherFrame)
{
  if(this == &_otherFrame)
    return;

  setProperties(_otherFrame.getSimpleFrameProperties());
}

//==============================================================================
void SimpleFrame::copy(const SimpleFrame* _otherFrame)
{
  if(nullptr == _otherFrame)
    return;

  copy(*_otherFrame);
}

//==============================================================================
SimpleFrame& SimpleFrame::operator=(const SimpleFrame& _otherFrame)
{
  copy(_otherFrame);
  return *this;
}

//==============================================================================
void SimpleFrame::setRelativeTransform(
    const Eigen::Isometry3d& _newRelTransform)
{
  mFrameP.mRelativeTf = _newRelTransform;
  notifyTransformUpdate();
}

//==============================================================================
const Eigen::Isometry3d& SimpleFrame::getRelativeTransform() const
{
  return mFrameP.mRelativeTf;
}

//==============================================================================
void SimpleFrame::setRelativeSpatialVelocity(
    const Eigen::Vector6d& _newSpatialVelocity)
{
  mFrameP.mRelativeVelocity = _newSpatialVelocity;
  notifyVelocityUpdate();
}

//==============================================================================
void SimpleFrame::setRelativeSpatialVelocity(
    const Eigen::Vector6d& _newSpatialVelocity, const Frame* _inCoordinatesOf)
{
  if(this == _inCoordinatesOf)
    setRelativeSpatialVelocity(_newSpatialVelocity);
  else
    setRelativeSpatialVelocity(math::AdR(_inCoordinatesOf->getTransform(this),
                                         _newSpatialVelocity));
}

//==============================================================================
const Eigen::Vector6d& SimpleFrame::getRelativeSpatialVelocity() const
{
  return mFrameP.mRelativeVelocity;
}

//==============================================================================
void SimpleFrame::setRelativeSpatialAcceleration(
    const Eigen::Vector6d &_newSpatialAcceleration)
{
  mFrameP.mRelativeAcceleration = _newSpatialAcceleration;
  notifyAccelerationUpdate();
}

//==============================================================================
void SimpleFrame::setRelativeSpatialAcceleration(
    const Eigen::Vector6d& _newSpatialAcceleration,
    const Frame* _inCoordinatesOf)
{
  if(this == _inCoordinatesOf)
    setRelativeSpatialAcceleration(_newSpatialAcceleration);
  else
    setRelativeSpatialAcceleration(
          math::AdR(_inCoordinatesOf->getTransform(this),
                    _newSpatialAcceleration) );
}

//==============================================================================
const Eigen::Vector6d& SimpleFrame::getRelativeSpatialAcceleration() const
{
  return mFrameP.mRelativeAcceleration;
}

//==============================================================================
const Eigen::Vector6d& SimpleFrame::getPrimaryRelativeAcceleration() const
{
  return mFrameP.mRelativeAcceleration;
}

//==============================================================================
const Eigen::Vector6d& SimpleFrame::getPartialAcceleration() const
{
  mPartialAcceleration = math::ad(getSpatialVelocity(),
                                  getRelativeSpatialVelocity());
  return mPartialAcceleration;
}

//==============================================================================
void SimpleFrame::setClassicDerivatives(
    const Eigen::Vector3d& _linearVelocity,
    const Eigen::Vector3d& _angularVelocity,
    const Eigen::Vector3d& _linearAcceleration,
    const Eigen::Vector3d& _angularAcceleration)
{
  Eigen::Vector6d v, a;
  v << _angularVelocity,
       _linearVelocity;

  // a_spatial = |    a_angular     |
  //             | a_linear - w x v |
  a << _angularAcceleration,
       _linearAcceleration - _angularVelocity.cross(_linearVelocity);

  setRelativeSpatialVelocity(v, getParentFrame());
  setRelativeSpatialAcceleration(a, getParentFrame());
}

} // namespace dart
} // namespace dynamics
