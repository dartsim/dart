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

#include "DegreeOfFreedom.h"
#include "Joint.h"
#include "Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
const std::string& DegreeOfFreedom::setName(const std::string& _name,
                                            bool _preserveName)
{
  preserveName(_preserveName);

  if (mName == _name)
    return mName;

  Skeleton* skel = mJoint->getSkeleton();
  if (skel)
  {
    skel->mNameMgrForDofs.removeName(mName);
    mName = _name;
    skel->addEntryToDofNameMgr(this);
  }
  else
    mName = _name;

  return mName;
}

//==============================================================================
const std::string& DegreeOfFreedom::getName() const
{
  return mName;
}

//==============================================================================
void DegreeOfFreedom::preserveName(bool _preserve)
{
  mNamePreserved = _preserve;
}

//==============================================================================
bool DegreeOfFreedom::isNamePreserved() const
{
  return mNamePreserved;
}

//==============================================================================
size_t DegreeOfFreedom::getIndexInSkeleton() const
{
  return mIndexInSkeleton;
}

//==============================================================================
size_t DegreeOfFreedom::getIndexInJoint() const
{
  return mIndexInJoint;
}

//==============================================================================
void DegreeOfFreedom::setPosition(double _position)
{
  mJoint->setPosition(mIndexInJoint, _position);
}

//==============================================================================
double DegreeOfFreedom::getPosition() const
{
  return mJoint->getPosition(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setPositionLimits(double _lowerLimit, double _upperLimit)
{
  setPositionLowerLimit(_lowerLimit);
  setPositionUpperLimit(_upperLimit);
}

//==============================================================================
void DegreeOfFreedom::setPositionLimits(const std::pair<double,double>& _limits)
{
  setPositionLimits(_limits.first, _limits.second);
}

//==============================================================================
std::pair<double,double> DegreeOfFreedom::getPositionLimits() const
{
  return std::pair<double,double>(getPositionLowerLimit(),
                                  getPositionUpperLimit());
}

//==============================================================================

void DegreeOfFreedom::setPositionLowerLimit(double _limit)
{
  mJoint->setPositionLowerLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getPositionLowerLimit() const
{
  return mJoint->getPositionLowerLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setPositionUpperLimit(double _limit)
{
  mJoint->setPositionUpperLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getPositionUpperLimit() const
{
  return mJoint->getPositionUpperLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setVelocity(double _velocity)
{
  mJoint->setVelocity(mIndexInJoint, _velocity);
}

//==============================================================================
double DegreeOfFreedom::getVelocity() const
{
  return mJoint->getVelocity(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setVelocityLimits(double _lowerLimit, double _upperLimit)
{
  setVelocityLowerLimit(_lowerLimit);
  setVelocityUpperLimit(_upperLimit);
}

//==============================================================================
void DegreeOfFreedom::setVelocityLimits(const std::pair<double,double>& _limits)
{
  setVelocityLimits(_limits.first, _limits.second);
}

//==============================================================================
std::pair<double,double> DegreeOfFreedom::getVelocityLimits() const
{
  return std::pair<double,double>(getVelocityLowerLimit(),
                                  getVelocityUpperLimit());
}

//==============================================================================
void DegreeOfFreedom::setVelocityLowerLimit(double _limit)
{
  mJoint->setVelocityLowerLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getVelocityLowerLimit() const
{
  return mJoint->getVelocityLowerLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setVelocityUpperLimit(double _limit)
{
  mJoint->setVelocityUpperLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getVelocityUpperLimit() const
{
  return mJoint->getVelocityUpperLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setAcceleration(double _acceleration)
{
  mJoint->setAcceleration(mIndexInJoint, _acceleration);
}

//==============================================================================
double DegreeOfFreedom::getAcceleration() const
{
  return mJoint->getAcceleration(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setAccelerationLimits(double _lowerLimit,
                                            double _upperLimit)
{
  setAccelerationLowerLimit(_lowerLimit);
  setAccelerationUpperLimit(_upperLimit);
}

//==============================================================================
void DegreeOfFreedom::setAccelerationLimits(
                                        const std::pair<double,double>& _limits)
{
  setAccelerationLimits(_limits.first, _limits.second);
}

//==============================================================================
std::pair<double,double> DegreeOfFreedom::getAccelerationLimits() const
{
  return std::pair<double,double>(getAccelerationLowerLimit(),
                                  getAccelerationUpperLimit());
}

//==============================================================================
void DegreeOfFreedom::setAccelerationLowerLimit(double _limit)
{
  mJoint->setAccelerationLowerLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getAccelerationLowerLimit() const
{
  return mJoint->getAccelerationLowerLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setAccelerationUpperLimit(double _limit)
{
  mJoint->setAccelerationUpperLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getAccelerationUpperLimit() const
{
  return mJoint->getAccelerationUpperLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setForce(double _force)
{
  mJoint->setForce(mIndexInJoint, _force);
}

//==============================================================================
double DegreeOfFreedom::getForce() const
{
  return mJoint->getForce(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setForceLimits(double _lowerLimit, double _upperLimit)
{
  setForceLowerLimit(_lowerLimit);
  setForceUpperLimit(_upperLimit);
}

//==============================================================================
void DegreeOfFreedom::setForceLimits(const std::pair<double, double> &_limits)
{
  setForceLimits(_limits.first, _limits.second);
}

//==============================================================================
std::pair<double,double> DegreeOfFreedom::getForceLimits() const
{
  return std::pair<double,double>(getForceLowerLimit(), getForceUpperLimit());
}

//==============================================================================
void DegreeOfFreedom::setForceLowerLimit(double _limit)
{
  mJoint->setForceLowerLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getForceLowerLimit() const
{
  return mJoint->getForceLowerLimit(mIndexInJoint);
}

//==============================================================================
void DegreeOfFreedom::setForceUpperLimit(double _limit)
{
  mJoint->setForceUpperLimit(mIndexInJoint, _limit);
}

//==============================================================================
double DegreeOfFreedom::getForceUpperLimit() const
{
  return mJoint->getForceUpperLimit(mIndexInJoint);
}

//==============================================================================
Joint* DegreeOfFreedom::getJoint()
{
  return mJoint;
}

//==============================================================================
const Joint* DegreeOfFreedom::getJoint() const
{
  return mJoint;
}

//==============================================================================
Skeleton* DegreeOfFreedom::getSkeleton()
{
  return mJoint->getSkeleton();
}

//==============================================================================
const Skeleton* DegreeOfFreedom::getSkeleton() const
{
  return mJoint->getSkeleton();
}

//==============================================================================
BodyNode* DegreeOfFreedom::getChildBodyNode()
{
  return mJoint->getChildBodyNode();
}

//==============================================================================
const BodyNode* DegreeOfFreedom::getChildBodyNode() const
{
  return mJoint->getChildBodyNode();
}

//==============================================================================
BodyNode* DegreeOfFreedom::getParentBodyNode()
{
  return mJoint->getParentBodyNode();
}

//==============================================================================
const BodyNode* DegreeOfFreedom::getParentBodyNode() const
{
  return mJoint->getParentBodyNode();
}

//==============================================================================
DegreeOfFreedom::DegreeOfFreedom(Joint* _joint,
                                 const std::string& _name,
                                 size_t _indexInJoint)
  : mName(_name),
    mNamePreserved(false),
    mIndexInJoint(_indexInJoint),
    mIndexInSkeleton(0),
    mJoint(_joint)
{

}

} // namespace dynamics
} // namespace dart
