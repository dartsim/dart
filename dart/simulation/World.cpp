/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "dart/integration/EulerIntegrator.h"
#include "dart/dynamics/GenCoord.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintDynamics.h"

namespace dart {
namespace simulation {

World::World()
  : integration::IntegrableSystem(),
    mGravity(0.0, 0.0, -9.81),
    mTime(0.0),
    mTimeStep(0.001),
    mFrame(0),
    mIntegrator(new integration::EulerIntegrator()),
    mConstraintHandler(
      new constraint::ConstraintDynamics(mSkeletons, mTimeStep)) {
  mIndices.push_back(0);
}

World::~World() {
  delete mIntegrator;
  delete mConstraintHandler;

  for (std::vector<dynamics::Skeleton*>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
    delete (*it);
}

Eigen::VectorXd World::getState() const {
  Eigen::VectorXd state(mIndices.back() * 2);

  for (unsigned int i = 0; i < getNumSkeletons(); i++) {
    int start = mIndices[i] * 2;
    int size = getSkeleton(i)->getNumGenCoords();
    state.segment(start, size) = getSkeleton(i)->get_q();
    state.segment(start + size, size) = getSkeleton(i)->get_dq();
  }

  return state;
}

void World::setState(const Eigen::VectorXd& _newState) {
  for (int i = 0; i < getNumSkeletons(); i++) {
    int start = 2 * mIndices[i];
    int size = 2 * getSkeleton(i)->getNumGenCoords();
    getSkeleton(i)->setState(_newState.segment(start, size));
  }
}

void World::setControlInput() {
  for (int i = 0; i < getNumSkeletons(); i++) {
    getSkeleton(i);
  }
}

Eigen::VectorXd World::evalDeriv() {
  // compute constraint (contact/contact, joint limit) forces
  mConstraintHandler->computeConstraintForces();

  // set constraint force
  for (unsigned int i = 0; i < getNumSkeletons(); i++) {
    // skip immobile objects in forward simulation
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;

    mSkeletons[i]->setConstraintForceVector(
          mConstraintHandler->getTotalConstraintForce(i) -
          mConstraintHandler->getContactForce(i));
  }

  // compute forward dynamics
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it) {
    (*it)->computeForwardDynamics();
  }

  // compute derivatives for integration
  Eigen::VectorXd deriv = Eigen::VectorXd::Zero(mIndices.back() * 2);
  for (unsigned int i = 0; i < getNumSkeletons(); i++) {
    // skip immobile objects in forward simulation
    if (!mSkeletons[i]->isMobile() || mSkeletons[i]->getNumGenCoords() == 0)
      continue;

    int start = mIndices[i] * 2;
    int size = getSkeleton(i)->getNumGenCoords();

    // set velocities
    deriv.segment(start, size) = getSkeleton(i)->get_dq()
                                 + mTimeStep * getSkeleton(i)->get_ddq();

    // set qddot (accelerations)
    deriv.segment(start + size, size) = getSkeleton(i)->get_ddq();
  }

  return deriv;
}

void World::setTimeStep(double _timeStep) {
  assert(_timeStep > 0.0 && "Invalid timestep.");

  mTimeStep = _timeStep;
  mConstraintHandler->setTimeStep(_timeStep);
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it) {
    (*it)->setTimeStep(_timeStep);
  }
}

double World::getTimeStep() const {
  return mTimeStep;
}

void World::step() {
  mIntegrator->integrate(this, mTimeStep);

  for (std::vector<dynamics::Skeleton*>::iterator itr = mSkeletons.begin();
       itr != mSkeletons.end(); ++itr) {
    (*itr)->clearInternalForceVector();
    (*itr)->clearExternalForceVector();
  }

  mTime += mTimeStep;
  mFrame++;
}

void World::setTime(double _time) {
  mTime = _time;
}

double World::getTime() const {
  return mTime;
}

int World::getSimFrames() const {
  return mFrame;
}

void World::setGravity(const Eigen::Vector3d& _gravity) {
  mGravity = _gravity;
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it) {
    (*it)->setGravity(_gravity);
  }
}

const Eigen::Vector3d&World::getGravity() const {
  return mGravity;
}

dynamics::Skeleton* World::getSkeleton(int _index) const {
  return mSkeletons[_index];
}

dynamics::Skeleton* World::getSkeleton(const std::string& _name) const {
  for (std::vector<dynamics::Skeleton*>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it) {
    if ((*it)->getName() == _name)
      return *it;
  }

  return NULL;
}

int World::getNumSkeletons() const {
  return mSkeletons.size();
}

void World::addSkeleton(dynamics::Skeleton* _skeleton) {
  assert(_skeleton != NULL && "Invalid skeleton.");

  // If mSkeletons already has _skeleton, then we do nothing.
  if (find(mSkeletons.begin(), mSkeletons.end(), _skeleton) !=
      mSkeletons.end()) {
    std::cout << "Skeleton [" << _skeleton->getName()
              << "] is already in the world." << std::endl;
    return;
  }

  mSkeletons.push_back(_skeleton);
  _skeleton->init(mTimeStep, mGravity);
  mIndices.push_back(mIndices.back() + _skeleton->getNumGenCoords());
  mConstraintHandler->addSkeleton(_skeleton);
}

void World::removeSkeleton(dynamics::Skeleton* _skeleton) {
  assert(_skeleton != NULL && "Invalid skeleton.");

  // Find index of _skeleton in mSkeleton.
  int i = 0;
  for (; i < mSkeletons.size(); ++i)
    if (mSkeletons[i] == _skeleton)
      break;

  // If i is equal to the number of skeletons, then _skeleton is not in
  // mSkeleton. We do nothing.
  if (i == mSkeletons.size()) {
    std::cout << "Skeleton [" << _skeleton->getName()
              << "] is not in the world." << std::endl;
    return;
  }

  // Update mIndices.
  for (++i; i < mSkeletons.size() - 1; ++i)
    mIndices[i] = mIndices[i+1] - _skeleton->getNumGenCoords();
  mIndices.pop_back();

  // Remove _skeleton from constraint handler.
  mConstraintHandler->removeSkeleton(_skeleton);

  // Remove _skeleton in mSkeletons and delete it.
  mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
                   mSkeletons.end());
  delete _skeleton;
}

int World::getIndex(int _index) const {
  return mIndices[_index];
}

bool World::checkCollision(bool _checkAllCollisions) {
  return mConstraintHandler->getCollisionDetector()->detectCollision(
        _checkAllCollisions, false);
}

constraint::ConstraintDynamics*World::getConstraintHandler() const {
  return mConstraintHandler;
}

}  // namespace simulation
}  // namespace dart
