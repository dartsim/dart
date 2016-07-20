/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/Marker.hpp"

#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

int Marker::msMarkerCount = 0;

//==============================================================================
// These declarations are needed for linking to work
constexpr Marker::ConstraintType Marker::NO;
constexpr Marker::ConstraintType Marker::HARD;
constexpr Marker::ConstraintType Marker::SOFT;

namespace detail {

//==============================================================================
MarkerProperties::MarkerProperties(const Eigen::Vector4d& color,
                                   ConstraintType type)
  : mColor(color),
    mType(type)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
void Marker::setAspectProperties(const AspectProperties& properties)
{
  setColor(properties.mColor);
  setConstraintType(properties.mType);
}

//==============================================================================
BodyNode* Marker::getBodyNode()
{
  return getBodyNodePtr();
}

//==============================================================================
const BodyNode* Marker::getBodyNode() const
{
  return getBodyNodePtr();
}

//==============================================================================
Eigen::Vector3d Marker::getLocalPosition() const
{
  return getRelativeTransform().translation();
}

//==============================================================================
void Marker::setLocalPosition(const Eigen::Vector3d& offset)
{
  Eigen::Isometry3d tf = getRelativeTransform();
  tf.translation() = offset;
  setRelativeTransform(tf);
}

//==============================================================================
Eigen::Vector3d Marker::getWorldPosition() const
{
  return getWorldTransform().translation();
}

//==============================================================================
int Marker::getID() const
{
  return mID;
}

//==============================================================================
void Marker::setConstraintType(Marker::ConstraintType type)
{
  if(type == mAspectProperties.mType)
    return;

  mAspectProperties.mType = type;
  incrementVersion();
}

//==============================================================================
Marker::ConstraintType Marker::getConstraintType() const
{
  return mAspectProperties.mType;
}

//==============================================================================
void Marker::setColor(const Eigen::Vector4d& color)
{
  if(color == mAspectProperties.mColor)
    return;

  mAspectProperties.mColor = color;
  incrementVersion();
}

//==============================================================================
const Eigen::Vector4d& Marker::getColor() const
{
  return mAspectProperties.mColor;
}

//==============================================================================
Marker::Marker(BodyNode* parent, const BasicProperties& properties)
  : Entity(ConstructFrame),
    Frame(parent),
    FixedFrame(parent, properties.mRelativeTf),
    common::EmbedPropertiesOnTopOf<
        Marker, detail::MarkerProperties, FixedJacobianNode>(
      parent, properties.mRelativeTf),
    mID(Marker::msMarkerCount++)
{
  createAspect<Aspect>();
  setCompositeProperties(properties);
}

//==============================================================================
Node* Marker::cloneNode(BodyNode* parent) const
{
  Marker* marker = new Marker(parent, BasicProperties());
  marker->duplicateAspects(this);

  if(mIK)
    marker->mIK = mIK->clone(marker);

  return marker;
}

}  // namespace dynamics
}  // namespace dart
