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

#include "kinematics/Dof.h"
#include "collision/CollisionDetector.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "simulation/World.h"

namespace simulation {

////////////////////////////////////////////////////////////////////////////////
World::World()
    : mGravity(0, 0, -9.81),
      mCollisionHandle(NULL),
      mTime(0.0),
      mTimeStep(0.001),
      mFrame(0)
{
    mIndices.push_back(0);

    mCollisionHandle = new dynamics::ConstraintDynamics(mSkeletons, mTimeStep);
}

////////////////////////////////////////////////////////////////////////////////
World::~World()
{
    delete mCollisionHandle;
}

////////////////////////////////////////////////////////////////////////////////
void World::setTimeStep(double _timeStep)
{
    mTimeStep = _timeStep;
    mCollisionHandle->setTimeStep(_timeStep);
}

////////////////////////////////////////////////////////////////////////////////
void World::reset()
{
    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
        mSkeletons[i]->restoreInitState();

    // Reset time and number of frames.
    mTime = 0;
    mFrame = 0;
}

////////////////////////////////////////////////////////////////////////////////
void World::step()
{
    // Calculate (q, qdot) by integrating with (qdot, qdotdot).
    mIntegrator.integrate(this, mTimeStep);

    // TODO: We need to consider better way.
    // Calculate body node's velocities represented in world frame.
    kinematics::BodyNode* itrBodyNode = NULL;
    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
    {
        for (unsigned int j = 0; j < mSkeletons[i]->getNumNodes(); j++)
        {
            itrBodyNode = mSkeletons[i]->getNode(j);
            itrBodyNode->evalVelocity(mSkeletons[i]->getPoseVelocity());
            itrBodyNode->evalOmega(mSkeletons[i]->getPoseVelocity());
        }
    }

    mTime += mTimeStep;
    mFrame++;
}

////////////////////////////////////////////////////////////////////////////////
dynamics::SkeletonDynamics* World::getSkeleton(int _index) const
{
    return mSkeletons[_index];
}

////////////////////////////////////////////////////////////////////////////////
dynamics::SkeletonDynamics* World::getSkeleton(const char* const _name) const
{
    dynamics::SkeletonDynamics* result = NULL;

    for (unsigned int i = 0; i < mSkeletons.size(); ++i)
    {
        if (strcmp(mSkeletons[i]->getName().c_str(), _name) == 0)
        {
            result = mSkeletons[i];
            break;
        }
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd World::getState()
{
    Eigen::VectorXd state(mIndices.back() * 2);

    for (unsigned int i = 0; i < getNumSkeletons(); i++)
    {
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumDofs();
        state.segment(start, size) = getSkeleton(i)->get_q();
        state.segment(start + size, size) = getSkeleton(i)->get_dq();
    }

    return state;
}

////////////////////////////////////////////////////////////////////////////////
void World::setState(const Eigen::VectorXd& _newState)
{
    for (int i = 0; i < getNumSkeletons(); i++)
    {
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumDofs();

        VectorXd pose = _newState.segment(start, size);
        VectorXd qDot = _newState.segment(start + size, size);
        getSkeleton(i)->clampRotation(pose, qDot);
        if (getSkeleton(i)->getImmobileState())
        {
            // need to update node transformation for collision
            getSkeleton(i)->setPose(pose, true, false);
        }
        else
        {
            // need to update first derivatives for collision
            getSkeleton(i)->setPose(pose, false, true);
            getSkeleton(i)->computeDynamics(mGravity, qDot, true);
        }

    }
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd World::evalDeriv()
{
    // compute constraint forces
    mCollisionHandle->computeConstraintForces();

    // compute derivatives for integration
    Eigen::VectorXd deriv = Eigen::VectorXd::Zero(mIndices.back() * 2);

    for (unsigned int i = 0; i < getNumSkeletons(); i++)
    {
        // skip immobile objects in forward simulation
        if (mSkeletons[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = getSkeleton(i)->getNumDofs();

        Eigen::VectorXd qddot = mSkeletons[i]->getInvMassMatrix()
                                * (-mSkeletons[i]->getCombinedVector()
                                   + mSkeletons[i]->getExternalForces()
                                   + mSkeletons[i]->getInternalForces()
                                   + mSkeletons[i]->getDampingForces()
                                   + mCollisionHandle->getTotalConstraintForce(i)
                                   );

        // set velocities
        deriv.segment(start, size) = getSkeleton(i)->get_dq() + (qddot * mTimeStep);

        // set qddot (accelerations)
        deriv.segment(start + size, size) = qddot;
    }

    return deriv;
}

////////////////////////////////////////////////////////////////////////////////
bool World::addSkeleton(dynamics::SkeletonDynamics* _skeleton)
{
    // Check if the world already has _skel.
    for (unsigned int i = 0; i < getNumSkeletons(); i++)
        if (mSkeletons[i] == _skeleton)
            return false;

    // Add _skeleton to the world.
    mSkeletons.push_back(_skeleton);

    _skeleton->initDynamics();

    // Indices update
    mIndices.push_back(mIndices.back() + _skeleton->getNumDofs());

    if(!_skeleton->getImmobileState())
    {
        _skeleton->computeDynamics(mGravity,
                                   _skeleton->get_dq(),
                                   true);

        for (unsigned int j = 0; j < _skeleton->getNumNodes(); j++)
        {
            _skeleton->getNode(j)->evalVelocity(_skeleton->get_dq());
            _skeleton->getNode(j)->evalOmega(_skeleton->get_dq());
        }
    }

    _skeleton->backupInitState();

    // create a collision handler
    mCollisionHandle->addSkeleton(_skeleton);

    return true;
}

bool World::checkCollision(bool checkAllCollisions) {
    return mCollisionHandle->getCollisionChecker()->checkCollision(checkAllCollisions, false);
}

} // namespace simulation
