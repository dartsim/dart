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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/simulation/World.h"

#include <iostream>
#include <string>
#include <vector>

#include "dart/common/Console.h"
#include "dart/integration/SemiImplicitEulerIntegrator.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintSolver.h"

namespace dart {
namespace simulation {

//==============================================================================
World::World()
  : mGravity(0.0, 0.0, -9.81),
    mTimeStep(0.001),
    mTime(0.0),
    mFrame(0),
    mIntegrator(new integration::SemiImplicitEulerIntegrator()),
    mConstraintSolver(new constraint::ConstraintSolver(mTimeStep)),
    mRecording(new Recording(mSkeletons))
{
  mIndices.push_back(0);
}

//==============================================================================
World::~World()
{
  delete mIntegrator;
  delete mConstraintSolver;
  delete mRecording;

  for (std::vector<dynamics::Skeleton*>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    delete (*it);
  }
}

//==============================================================================
void World::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0 && "Invalid timestep.");

  mTimeStep = _timeStep;
//  mConstraintHandler->setTimeStep(_timeStep);
  mConstraintSolver->setTimeStep(_timeStep);
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    (*it)->setTimeStep(_timeStep);
  }
}

//==============================================================================
double World::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void World::step()
{
  // Integrate velocity unconstrained skeletons
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if (!(*it)->isMobile())
      continue;

    (*it)->computeForwardDynamicsRecursionPartB();
    (*it)->integrateVelocities(mTimeStep);
    (*it)->computeForwardKinematics(false, true, false);
  }

  // Detect active constraints and compute constraint impulses
  mConstraintSolver->solve();

  // Compute velocity changes given constraint impulses
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if ((*it)->isImpulseApplied() && (*it)->isMobile())
    {
      (*it)->computeImpulseForwardDynamics();
      (*it)->setImpulseApplied(false);
    }
  }

  //
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if (!(*it)->isMobile())
      continue;

    (*it)->integratePositions(mTimeStep);
  }

  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if (!(*it)->isMobile())
      continue;

    (*it)->computeForwardDynamicsRecursionPartA();
    (*it)->resetForces();
    (*it)->clearExternalForces();
//    (*it)->clearConstraintImpulses();
  }

  mTime += mTimeStep;
  mFrame++;
}

//==============================================================================
void World::setTime(double _time)
{
  mTime = _time;
}

//==============================================================================
double World::getTime() const
{
  return mTime;
}

//==============================================================================
int World::getSimFrames() const
{
  return mFrame;
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& _gravity)
{
  mGravity = _gravity;
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    (*it)->setGravity(_gravity);
  }
}

//==============================================================================
const Eigen::Vector3d& World::getGravity() const
{
  return mGravity;
}

//==============================================================================
dynamics::Skeleton* World::getSkeleton(int _index) const
{
  return mSkeletons[_index];
}

//==============================================================================
dynamics::Skeleton* World::getSkeleton(const std::string& _name) const
{
  for (std::vector<dynamics::Skeleton*>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if ((*it)->getName() == _name)
      return *it;
  }

  return NULL;
}

//==============================================================================
size_t World::getNumSkeletons() const
{
  return mSkeletons.size();
}

//==============================================================================
void World::addSkeleton(dynamics::Skeleton* _skeleton)
{
  assert(_skeleton != NULL && "Invalid skeleton.");

  // If mSkeletons already has _skeleton, then we do nothing.
  if (find(mSkeletons.begin(), mSkeletons.end(), _skeleton) != mSkeletons.end())
  {
    std::cout << "Skeleton [" << _skeleton->getName()
              << "] is already in the world." << std::endl;
    return;
  }

  mSkeletons.push_back(_skeleton);
  _skeleton->init(mTimeStep, mGravity);
  mIndices.push_back(mIndices.back() + _skeleton->getDof());
  mConstraintSolver->addSkeleton(_skeleton);

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);
}

//==============================================================================
void World::removeSkeleton(dynamics::Skeleton* _skeleton)
{
  assert(_skeleton != NULL && "Invalid skeleton.");

  // Find index of _skeleton in mSkeleton.
  size_t i = 0;
  for (; i < mSkeletons.size(); ++i)
  {
    if (mSkeletons[i] == _skeleton)
      break;
  }

  // If i is equal to the number of skeletons, then _skeleton is not in
  // mSkeleton. We do nothing.
  if (i == mSkeletons.size())
  {
    std::cout << "Skeleton [" << _skeleton->getName()
              << "] is not in the world." << std::endl;
    return;
  }

  // Update mIndices.
  for (++i; i < mSkeletons.size() - 1; ++i)
    mIndices[i] = mIndices[i+1] - _skeleton->getDof();
  mIndices.pop_back();

  // Remove _skeleton from constraint handler.
//  mConstraintHandler->removeSkeleton(_skeleton);
  mConstraintSolver->removeSkeleton(_skeleton);

  // Remove _skeleton in mSkeletons and delete it.
  mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
                   mSkeletons.end());

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);

  delete _skeleton;
}

//==============================================================================
void World::removeAllSkeletons()
{
  while (getNumSkeletons() > 0)
    removeSkeleton(getSkeleton(0));
}

//==============================================================================
int World::getIndex(int _index) const
{
  return mIndices[_index];
}

//==============================================================================
bool World::checkCollision(bool _checkAllCollisions)
{
  return mConstraintSolver->getCollisionDetector()->detectCollision(
        _checkAllCollisions, false);
}

//==============================================================================
constraint::ConstraintSolver* World::getConstraintSolver() const
{
  return mConstraintSolver;
}

//==============================================================================
void World::bake()
{
  collision::CollisionDetector* cd
      = getConstraintSolver()->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  int nSkeletons = getNumSkeletons();
  Eigen::VectorXd state(getIndex(nSkeletons) + 6 * nContacts);
  for (size_t i = 0; i < getNumSkeletons(); i++)
  {
    state.segment(getIndex(i), getSkeleton(i)->getDof())
        = getSkeleton(i)->getPositions();
  }
  for (int i = 0; i < nContacts; i++)
  {
    int begin = getIndex(nSkeletons) + i * 6;
    state.segment(begin, 3)     = cd->getContact(i).point;
    state.segment(begin + 3, 3) = cd->getContact(i).force;
  }
  mRecording->addState(state);
}

//==============================================================================
Recording* World::getRecording()
{
  return mRecording;
}

}  // namespace simulation
}  // namespace dart
