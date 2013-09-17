/* Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 03/25/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include <iostream>

#include "integration/EulerIntegrator.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Skeleton.h"
#include "constraint/ConstraintDynamics.h"
#include "simulation/World.h"

namespace dart {
namespace simulation {

World::World()
    : integration::IntegrableSystem(),
      mTime(0.0),
      mTimeStep(0.001),
      mFrame(0),
      mIntegrator(new integration::EulerIntegrator()),
      mCollisionHandle(new constraint::ConstraintDynamics(mSkeletons, mTimeStep))
{
    mIndices.push_back(0);
}

World::~World()
{
    delete mIntegrator;
    delete mCollisionHandle;
}

Eigen::VectorXd World::getState() const
{
    Eigen::VectorXd state(mIndices.back() * 2);

    for (unsigned int i = 0; i < getNumSkeletons(); i++)
    {
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumGenCoords();
        state.segment(start, size) = getSkeleton(i)->get_q();
        state.segment(start + size, size) = getSkeleton(i)->get_dq();
    }

    return state;
}

void World::setState(const Eigen::VectorXd& _newState)
{
    for (int i = 0; i < getNumSkeletons(); i++)
    {
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumGenCoords();

        Eigen::VectorXd q = _newState.segment(start, size);
        Eigen::VectorXd dq = _newState.segment(start + size, size);

        getSkeleton(i)->set_q(q);
        getSkeleton(i)->set_dq(dq);
        getSkeleton(i)->updateForwardKinematics();
    }
}

void World::setControlInput()
{
    for (int i = 0; i < getNumSkeletons(); i++)
    {
        getSkeleton(i);
    }
}

Eigen::VectorXd World::evalDeriv()
{
    // Calculate M(q), M^{-1}(q)
    for (std::vector<dynamics::Skeleton*>::iterator itrSkeleton = mSkeletons.begin();
         itrSkeleton != mSkeletons.end();
         ++itrSkeleton)
    {
        (*itrSkeleton)->computeEquationsOfMotionID(mGravity);
        //(*itrSkeleton)->computeEquationsOfMotionFS(mGravity);
    }

    // compute constraint (contact/contact, joint limit) forces
    mCollisionHandle->computeConstraintForces();

    // set constraint force
    for (unsigned int i = 0; i < getNumSkeletons(); i++)
    {
        // skip immobile objects in forward simulation
        if (mSkeletons[i]->getImmobileState())
            continue;

        mSkeletons[i]->setConstraintForces(
                    mCollisionHandle->getTotalConstraintForce(i));
    }

    // compute forward dynamics
    for (std::vector<dynamics::Skeleton*>::iterator itrSkeleton = mSkeletons.begin();
         itrSkeleton != mSkeletons.end();
         ++itrSkeleton)
    {
        //(*itrSkeleton)->computeForwardDynamicsID(mGravity);
        (*itrSkeleton)->computeForwardDynamicsFS(mGravity);
    }

    // compute derivatives for integration
    Eigen::VectorXd deriv = Eigen::VectorXd::Zero(mIndices.back() * 2);
    for (unsigned int i = 0; i < getNumSkeletons(); i++)
    {
        // skip immobile objects in forward simulation
        if (mSkeletons[i]->getImmobileState())
            continue;

        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumGenCoords();

        // set velocities
        deriv.segment(start, size) = getSkeleton(i)->get_dq() + mTimeStep * getSkeleton(i)->get_ddq();

        // set qddot (accelerations)
        deriv.segment(start + size, size) = getSkeleton(i)->get_ddq();
    }

    return deriv;
}

void World::setTimeStep(double _timeStep)
{
    mTimeStep = _timeStep;
    mCollisionHandle->setTimeStep(_timeStep);
}

double World::getTimeStep() const
{
    return mTimeStep;
}

void World::reset()
{
    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
        mSkeletons[i]->restoreInitState();

    // Reset time and number of frames.
    mTime = 0;
    mFrame = 0;
}

void World::step()
{
    mIntegrator->integrate(this, mTimeStep);

    for (std::vector<dynamics::Skeleton*>::iterator itr = mSkeletons.begin();
         itr != mSkeletons.end(); ++itr)
    {
        (*itr)->clearInternalForces();
        (*itr)->clearExternalForces();
    }

    mTime += mTimeStep;
    mFrame++;
}

void World::setTime(double _time)
{
    mTime = _time;
}

double World::getTime() const
{
    return mTime;
}

int World::getSimFrames() const
{
    return mFrame;
}

void World::setGravity(const Eigen::Vector3d& _gravity)
{
    mGravity = _gravity;
}

const Eigen::Vector3d&World::getGravity() const
{
    return mGravity;
}

dynamics::Skeleton* World::getSkeleton(int _index) const
{
    return mSkeletons[_index];
}

dynamics::Skeleton* World::getSkeleton(const std::string& _name) const
{
    for (std::vector<dynamics::Skeleton*>::const_iterator itrSkeleton
         = mSkeletons.begin();
         itrSkeleton != mSkeletons.end();
         ++itrSkeleton)
    {
        if ((*itrSkeleton)->getName() == _name)
            return *itrSkeleton;
    }

    return NULL;
}

int World::getNumSkeletons() const
{
    return mSkeletons.size();
}

void World::addSkeleton(dynamics::Skeleton* _skeleton)
{
    assert(_skeleton != NULL);

    mSkeletons.push_back(_skeleton);

    //_skeleton->initKinematics();
    _skeleton->initDynamics();
    _skeleton->updateForwardKinematics();
    _skeleton->computeEquationsOfMotionID(mGravity);
    _skeleton->backupInitState();

    mIndices.push_back(mIndices.back() + _skeleton->getNumGenCoords());

    mCollisionHandle->addSkeleton(_skeleton);
}

int World::getIndex(int _index) const
{
    return mIndices[_index];
}

bool World::checkCollision(bool checkAllCollisions)
{
    return mCollisionHandle->getCollisionChecker()->checkCollision(
                checkAllCollisions, false);
}

constraint::ConstraintDynamics*World::getCollisionHandle() const
{
    return mCollisionHandle;
}

} // namespace simulation
} // namespace dart
