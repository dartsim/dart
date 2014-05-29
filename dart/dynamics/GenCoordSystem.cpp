/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/GenCoordSystem.h"

#include <iostream>
#include <string>

namespace dart {
namespace dynamics {

//==============================================================================
GenCoordSystem::GenCoordSystem()
{
}

//==============================================================================
GenCoordSystem::~GenCoordSystem()
{
}

//==============================================================================
void GenCoordSystem::integrateConfigs(double _dt)
{
  for (size_t i = 0; i < mGenCoords.size(); ++i)
    mGenCoords[i]->integrateConfig(_dt);
}

//==============================================================================
void GenCoordSystem::integrateGenVels(double _dt)
{
  for (size_t i = 0; i < mGenCoords.size(); ++i)
    mGenCoords[i]->integrateVel(_dt);
}

//==============================================================================
GenCoord* GenCoordSystem::getGenCoord(size_t _idx) const
{
  assert(0 <= _idx && _idx < getDof());
  return mGenCoords[_idx];
}

//==============================================================================
GenCoord* GenCoordSystem::getGenCoord(const std::string& _name) const
{
  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
  {
    if (mGenCoords[i]->getName() == _name)
      return mGenCoords[i];
  }

  return NULL;
}

//==============================================================================
void GenCoordSystem::setPositions(const Eigen::VectorXd& _configs)
{
  assert(_configs.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setPos(_configs[i]);
}

//==============================================================================
void GenCoordSystem::setVelocities(const Eigen::VectorXd& _vels)
{
  assert(_vels.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setVel(_vels[i]);
}

//==============================================================================
void GenCoordSystem::setAccelerations(const Eigen::VectorXd& _accs)
{
  assert(_accs.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setAcc(_accs[i]);
}

//==============================================================================
void GenCoordSystem::setGenForces(const Eigen::VectorXd& _forces)
{
  assert(_forces.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setForce(_forces[i]);
}

//==============================================================================
void GenCoordSystem::setConfigsMin(const Eigen::VectorXd& _configsMin)
{
  assert(_configsMin.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setPosMin(_configsMin[i]);
}

//==============================================================================
void GenCoordSystem::setGenVelsMin(const Eigen::VectorXd& _velsMin)
{
  assert(_velsMin.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setVelMin(_velsMin[i]);
}

//==============================================================================
void GenCoordSystem::setGenAccsMin(const Eigen::VectorXd& _accsMin)
{
  assert(_accsMin.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setAccMin(_accsMin[i]);
}

//==============================================================================
void GenCoordSystem::setGenForcesMin(const Eigen::VectorXd& _forcesMin)
{
  assert(_forcesMin.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setForceMin(_forcesMin[i]);
}

//==============================================================================
void GenCoordSystem::setConfigsMax(const Eigen::VectorXd& _configsMax)
{
  assert(_configsMax.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setPosMax(_configsMax[i]);
}

//==============================================================================
void GenCoordSystem::setGenVelsMax(const Eigen::VectorXd& _velsMax)
{
  assert(_velsMax.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setVelMax(_velsMax[i]);
}

//==============================================================================
void GenCoordSystem::setGenAccsMax(const Eigen::VectorXd& _accsMax)
{
  assert(_accsMax.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setAccMax(_accsMax[i]);
}

//==============================================================================
void GenCoordSystem::setGenForcesMax(const Eigen::VectorXd& _forcesMax)
{
  assert(_forcesMax.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setForceMax(_forcesMax[i]);
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getConfigs() const
{
  size_t size = getDof();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    q(i) = mGenCoords[i]->getPos();

  return q;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenVels() const
{
  size_t size = getDof();
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    dq(i) = mGenCoords[i]->getVel();

  return dq;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenAccs() const
{
  size_t size = getDof();
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    ddq(i) = mGenCoords[i]->getAcc();

  return ddq;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenForces() const
{
  size_t size = getDof();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    tau(i) = mGenCoords[i]->getForce();

  return tau;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getConfigsMax() const
{
  size_t size = getDof();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    q(i) = mGenCoords[i]->getPosMax();

  return q;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenVelsMax() const
{
  size_t size = getDof();
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    dq(i) = mGenCoords[i]->getVelMax();

  return dq;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenAccsMax() const
{
  size_t size = getDof();
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    ddq(i) = mGenCoords[i]->getAccMax();

  return ddq;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenForcesMax() const
{
  size_t size = getDof();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    tau(i) = mGenCoords[i]->getForceMax();

  return tau;
}

//==============================================================================
void GenCoordSystem::setConstraintImpulses(const Eigen::VectorXd& _constImps)
{
  assert(_constImps.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setConstraintImpulse(_constImps[i]);
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getConstraintImpulses() const
{
  size_t size = getDof();
  Eigen::VectorXd constraintImpulses = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    constraintImpulses(i) = mGenCoords[i]->getConstraintImpulse();

  return constraintImpulses;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getConfigsMin() const
{
  size_t size = getDof();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    q(i) = mGenCoords[i]->getPosMin();

  return q;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenVelsMin() const
{
  size_t size = getDof();
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    dq(i) = mGenCoords[i]->getVelMin();

  return dq;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenAccsMin() const
{
  size_t size = getDof();
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    ddq(i) = mGenCoords[i]->getAccMin();

  return ddq;
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getGenForcesMin() const
{
  size_t size = getDof();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    tau(i) = mGenCoords[i]->getForceMin();

  return tau;
}

//==============================================================================
size_t GenCoordSystem::getDof() const
{
  return mGenCoords.size();
}

//==============================================================================
void GenCoordSystem::setVelsChange(const Eigen::VectorXd& _velsChange)
{
  assert(_velsChange.size() == getDof());

  size_t size = getDof();

  for (size_t i = 0; i < size; ++i)
    mGenCoords[i]->setVelChange(_velsChange[i]);
}

//==============================================================================
Eigen::VectorXd GenCoordSystem::getVelsChange() const
{
  size_t size = getDof();
  Eigen::VectorXd velsChange = Eigen::VectorXd::Zero(size);

  for (size_t i = 0; i < size; ++i)
    velsChange(i) = mGenCoords[i]->getVelChange();

  return velsChange;
}

////==============================================================================
//void GenCoordSystem::setImpulses(const Eigen::VectorXd& _impulses)
//{
//  assert(_impulses.size() == getDof());

//  size_t size = getDof();

//  for (size_t i = 0; i < size; ++i)
//    mGenCoords[i]->setImpulse(_impulses[i]);
//}

////==============================================================================
//Eigen::VectorXd GenCoordSystem::getImpulses() const
//{
//  size_t size = getDof();
//  Eigen::VectorXd impulse = Eigen::VectorXd::Zero(size);

//  for (size_t i = 0; i < size; ++i)
//    impulse(i) = mGenCoords[i]->getImpulse();

//  return impulse;
//}

}  // namespace dynamics
}  // namespace dart
