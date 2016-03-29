/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include "dart/dynamics/Marker.hpp"

#include <string>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/renderer/RenderInterface.hpp"

namespace dart {
namespace dynamics {

int Marker::msMarkerCount = 0;

//==============================================================================
Marker::Properties::Properties(const std::string& name,
                               const Eigen::Vector3d& offset,
                               const Eigen::Vector4d& color,
                               ConstraintType type)
  : mName(name), mOffset(offset), mColor(color), mType(type)
{
  // Do nothing
}

//==============================================================================
Marker::Marker(const std::string& name,
               const Eigen::Vector3d& offset,
               const Eigen::Vector4d& color,
               BodyNode* bodyNode,
               ConstraintType type)
  : mProperties(name, offset, color, type),
    mBodyNode(bodyNode),
    mSkelIndex(0),
    mID(Marker::msMarkerCount++)
{
  // Do nothing
}

//==============================================================================
Marker::~Marker()
{
  // Do nothing
}

//==============================================================================
void Marker::draw(renderer::RenderInterface* ri,
                  bool offset,
                  const Eigen::Vector4d& color,
                  bool useDefaultColor) const
{
  if (!ri)
    return;

  ri->pushName(getID());

  if (mProperties.mType == HARD)
  {
    ri->setPenColor(Color::Red(1.0));
  }
  else if (mProperties.mType == SOFT)
  {
    ri->setPenColor(Color::Green(1.0));
  }
  else
  {
    if (useDefaultColor)
      ri->setPenColor(mProperties.mColor);
    else
      ri->setPenColor(color);
  }

  if (offset)
  {
    ri->pushMatrix();
    ri->translate(mProperties.mOffset);
    ri->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
    ri->popMatrix();
  }
  else
  {
    ri->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
  }

  ri->popName();
}

//==============================================================================
BodyNode* Marker::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const BodyNode* Marker::getBodyNode() const
{
  return mBodyNode;
}

//==============================================================================
const Eigen::Vector3d& Marker::getLocalPosition() const
{
  return mProperties.mOffset;
}

//==============================================================================
void Marker::setLocalPosition(const Eigen::Vector3d& offset)
{
    mProperties.mOffset = offset;
}

//==============================================================================
Eigen::Vector3d Marker::getWorldPosition() const
{
  return mBodyNode->getTransform() * mProperties.mOffset;
}

//==============================================================================
void Marker::setSkeletonIndex(int index)
{
  setIndexInSkeleton(index);
}

//==============================================================================
void Marker::setIndexInSkeleton(int index)
{
  mSkelIndex = index;
}

//==============================================================================
int Marker::getIndexInSkeleton() const
{
  return mSkelIndex;
}

//==============================================================================
int Marker::getID() const
{
  return mID;
}

//==============================================================================
void Marker::setName(const std::string& name)
{
  mProperties.mName = name;
}

//==============================================================================
const std::string& Marker::getName() const
{
  return mProperties.mName;
}

//==============================================================================
void Marker::setConstraintType(Marker::ConstraintType type)
{
  mProperties.mType = type;
}

//==============================================================================
Marker::ConstraintType Marker::getConstraintType() const
{
  return mProperties.mType;
}

//==============================================================================
Marker::Marker(const Properties& properties, BodyNode* parent)
  : mProperties(properties),
    mBodyNode(parent),
    mSkelIndex(0),
    mID(Marker::msMarkerCount++)
{
  // Do nothing
}

}  // namespace dynamics
}  // namespace dart
