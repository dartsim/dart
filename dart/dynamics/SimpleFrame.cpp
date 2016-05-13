/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/common/Console.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
SimpleFrame::SimpleFrame(Frame* _refFrame, const std::string& _name,
                         const Eigen::Isometry3d& _relativeTransform)
  : Entity(ConstructFrame),
    Frame(_refFrame),
    Detachable(),
    ShapeFrame(_refFrame),
    mRelativeTf(_relativeTransform),
    mRelativeVelocity(Eigen::Vector6d::Zero()),
    mRelativeAcceleration(Eigen::Vector6d::Zero()),
    mPartialAcceleration(Eigen::Vector6d::Zero())
{
  setName(_name);
}

//==============================================================================
SimpleFrame::SimpleFrame(const SimpleFrame& _otherFrame, Frame* _refFrame)
  : Entity(ConstructFrame),
    common::Composite(),
    Frame(_refFrame),
    Detachable(),
    ShapeFrame(_refFrame),
    mRelativeTf(Eigen::Isometry3d::Identity()),
    mRelativeVelocity(Eigen::Vector6d::Zero()),
    mRelativeAcceleration(Eigen::Vector6d::Zero()),
    mPartialAcceleration(Eigen::Vector6d::Zero())
{
  copy(_otherFrame, _refFrame);
  duplicateAspects(&_otherFrame);
}

//==============================================================================
SimpleFrame::~SimpleFrame()
{
  // Do nothing
}

//==============================================================================
const std::string& SimpleFrame::setName(const std::string& _name)
{
  if(_name == mName)
    return mName;

  std::string oldName = mName;
  mName = _name;

  incrementVersion();
  Entity::mNameChangedSignal.raise(this, oldName, mName);

  return mName;
}

//==============================================================================
const std::string& SimpleFrame::getName() const
{
  return mName;
}

//==============================================================================
SimpleFramePtr SimpleFrame::clone(Frame* _refFrame) const
{
  return SimpleFramePtr(new SimpleFrame(*this, _refFrame));
}

//==============================================================================
void SimpleFrame::copy(const Frame& _otherFrame, Frame* _refFrame,
                       bool _copyProperties)
{
  copy(&_otherFrame, _refFrame, _copyProperties);
}

//==============================================================================
void SimpleFrame::copy(const Frame* _otherFrame, Frame* _refFrame,
                       bool _copyProperties)
{
  if(nullptr == _otherFrame || nullptr == _refFrame)
    return;

  if( (this == _otherFrame) && (_refFrame == getParentFrame()) )
    return;

  Eigen::Isometry3d relativeTf = _otherFrame->getTransform(_refFrame);
  Eigen::Vector6d relativeVelocity =
      _otherFrame->getSpatialVelocity(_refFrame, Frame::World());
  Eigen::Vector6d relativeAcceleration =
      _otherFrame->getSpatialAcceleration(_refFrame, Frame::World());

  setParentFrame(_refFrame);
  setRelativeTransform(relativeTf);
  setRelativeSpatialVelocity(relativeVelocity, Frame::World());
  setRelativeSpatialAcceleration(relativeAcceleration, Frame::World());

  if(_copyProperties)
  {
    const auto shapeFrame = dynamic_cast<const ShapeFrame*>(_otherFrame);
    if(shapeFrame)
      setCompositeProperties(shapeFrame->getCompositeProperties());

    const auto simpleFrame = dynamic_cast<const SimpleFrame*>(_otherFrame);
    if(simpleFrame)
      setName(simpleFrame->getName());
  }
}

//==============================================================================
SimpleFrame& SimpleFrame::operator=(const SimpleFrame& _otherFrame)
{
  copy(_otherFrame, getParentFrame(), false);
  return *this;
}

//==============================================================================
std::shared_ptr<SimpleFrame> SimpleFrame::spawnChildSimpleFrame(
    const std::string& name, const Eigen::Isometry3d& relativeTransform)
{
  return std::make_shared<SimpleFrame>(this, name, relativeTransform);
}

//==============================================================================
void SimpleFrame::setRelativeTransform(
    const Eigen::Isometry3d& _newRelTransform)
{
  mRelativeTf = _newRelTransform;
  notifyTransformUpdate();
}

//==============================================================================
void SimpleFrame::setRelativeTranslation(const Eigen::Vector3d& _newTranslation)
{
  mRelativeTf.translation() = _newTranslation;
  notifyTransformUpdate();
}

//==============================================================================
void SimpleFrame::setRelativeRotation(const Eigen::Matrix3d& _newRotation)
{
  mRelativeTf.linear() = _newRotation;
  notifyTransformUpdate();
}

//==============================================================================
void SimpleFrame::setTransform(const Eigen::Isometry3d& _newTransform,
                               const Frame* _withRespectTo)
{
  setRelativeTransform(
        _withRespectTo->getTransform(getParentFrame()) * _newTransform);
}

//==============================================================================
void SimpleFrame::setTranslation(const Eigen::Vector3d& _newTranslation,
                                 const Frame* _withRespectTo)
{
  setRelativeTranslation(
        _withRespectTo->getTransform(getParentFrame()) * _newTranslation);
}

//==============================================================================
void SimpleFrame::setRotation(const Eigen::Matrix3d& _newRotation,
                              const Frame* _withRespectTo)
{
  setRelativeRotation(
        _withRespectTo->getTransform(getParentFrame()).linear()
        * _newRotation);
}

//==============================================================================
const Eigen::Isometry3d& SimpleFrame::getRelativeTransform() const
{
  return mRelativeTf;
}

//==============================================================================
void SimpleFrame::setRelativeSpatialVelocity(
    const Eigen::Vector6d& _newSpatialVelocity)
{
  mRelativeVelocity = _newSpatialVelocity;
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
  return mRelativeVelocity;
}

//==============================================================================
void SimpleFrame::setRelativeSpatialAcceleration(
    const Eigen::Vector6d &_newSpatialAcceleration)
{
  mRelativeAcceleration = _newSpatialAcceleration;
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
  return mRelativeAcceleration;
}

//==============================================================================
const Eigen::Vector6d& SimpleFrame::getPrimaryRelativeAcceleration() const
{
  return mRelativeAcceleration;
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
