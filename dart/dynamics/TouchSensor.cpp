/*
 * Copyright (c) 2011-2017, The DART development contributors
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

#include "dart/dynamics/TouchSensor.hpp"

#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
void TouchSensor::setAspectState(const AspectState& /*state*/)
{
  // TODO
}

//==============================================================================
void TouchSensor::setAspectProperties(const AspectProperties& /*properties*/)
{
  // TODO
}

//==============================================================================
void TouchSensor::setRange(double range)
{
  if (range == mAspectProperties.mRange)
    return;

  mAspectProperties.mRange = range;
  incrementVersion();
}

//==============================================================================
double TouchSensor::getRange() const
{
  return mAspectProperties.mRange;
}

//==============================================================================
TouchSensor::TouchSensor(BodyNode* parent, const BasicProperties& properties)
  : Entity(ConstructFrame),
    Frame(parent),
    common::EmbedStateAndPropertiesOnTopOf<TouchSensor,
                                           detail::TouchSensorState,
                                           detail::TouchSensorProperties,
                                           Sensor>(parent, properties)
{
  createAspect<Aspect>();
  setCompositeProperties(properties);

  auto sphereShape = std::make_shared<SphereShape>(getRange());

  mSimpleFrame = std::make_shared<SimpleFrame>(this);

  mCollisionDetector = collision::FCLCollisionDetector::create();
  mCollisionGroup = mCollisionDetector->createCollisionGroup();
}

//==============================================================================
Node* TouchSensor::cloneNode(BodyNode* parent) const
{
  TouchSensor* sensor = new TouchSensor(parent, BasicProperties());
  sensor->duplicateAspects(this);

  if (mIK)
    sensor->mIK = mIK->clone(sensor);

  return sensor;
}

} // namespace dynamics
} // namespace dart
