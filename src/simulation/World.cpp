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

#include "simulation/World.h"
#include "dynamics/ContactDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"

#include <iostream>

namespace simulation {

////////////////////////////////////////////////////////////////////////////////
World::World()
    : mGravity(0, 0, -9.81),
      mCollisionHandle(NULL),
      mTime(0.0),
      mTimeStep(0.001),
      mFrame(0),
      mSimulating(false)
{
    mIndices.push_back(0);

    mCollisionHandle = new dynamics::ContactDynamics(mSkeletons, mTimeStep);
}

////////////////////////////////////////////////////////////////////////////////
World::~World()
{
    delete mCollisionHandle;
}

////////////////////////////////////////////////////////////////////////////////
void World::reset()
{
    // Assume that the world is initialized.
    //assert(mIsInitialized == true);

    // Reset all states: mDofs and mDofVels.
    for (unsigned int i = 0; i < getNumSkeletons(); i++)
    {
        //        mDofs[i].setZero(mSkels[i]->getNumDofs());
        //        mDofVels[i].setZero(mSkels[i]->getNumDofs());
    }

    // Claculate transformations (forward kinematics).
    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
    {
        //        mSkels[i]->setPose(mDofs[i], true, false);
    }

    // Calculate velocities represented in world frame (forward kinematics).
    dynamics::BodyNodeDynamics* itrBodyNodeDyn = NULL;
    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
    {
        for (unsigned int j = 0; j < mSkeletons[i]->getNumNodes(); j++)
        {
            itrBodyNodeDyn
                    = static_cast<dynamics::BodyNodeDynamics*>(mSkeletons[i]->getNode(j));
            //            itrBodyNodeDyn->evalVelocity(mDofVels[i]);
        }
    }

    // Contact reset.
    mCollisionHandle->reset();

    // Reset time and number of frames.
    mTime = 0;
    mFrame = 0;
}

////////////////////////////////////////////////////////////////////////////////
void World::step()
{
    step(mTimeStep);
}

////////////////////////////////////////////////////////////////////////////////
void World::step(double _timeStep)
{
    // Calculate (q, qdot) by integrating with (qdot, qdotdot).
    mIntegrator.integrate(this, _timeStep);

    // TODO: We need to consider better way.
    // Calculate body node's velocities represented in world frame.
    dynamics::BodyNodeDynamics* itrBodyNodeDyn = NULL;
    for (unsigned int i = 0; i < getNumSkeletons(); ++i)
    {
        for (unsigned int j = 0; j < mSkeletons[i]->getNumNodes(); j++)
        {
            itrBodyNodeDyn
                    = static_cast<dynamics::BodyNodeDynamics*>(mSkeletons[i]->getNode(j));
            itrBodyNodeDyn->evalVelocity(mSkeletons[i]->getQDotVector());
        }
    }

    mTime += _timeStep;
    mFrame++;
}

////////////////////////////////////////////////////////////////////////////////
void World::steps(int _steps)
{
    for (int i = 0; i < _steps; ++i)
        step();
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
        if (mSkeletons[i]->getName() == _name)
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
        state.segment(start, size) = getSkeleton(i)->getPose();
        state.segment(start + size, size) = getSkeleton(i)->getQDotVector();
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

        // The root follows the desired acceleration and the rest of the
        // body follows dynamic equations. The root acceleration will impact
        // the rest of the body correctly. Collision or other external
        // forces will alter the root acceleration
        if (getSkeleton(i)->getImmobileState())
        {
            // need to update node transformation for collision
            getSkeleton(i)->setPose(pose, true, false);
        }
        else
        {
            // need to update first derivatives for collision
            getSkeleton(i)->setPose(pose, false, false);
            getSkeleton(i)->computeDynamics(mGravity, qDot, true);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd World::evalDeriv()
{
    // compute contact forces
    mCollisionHandle->applyContactForces();

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
                                   + mCollisionHandle->getConstraintForce(i)
                                   );

        //        Eigen::VectorXd qddot = mSkeletons[i]->getMassMatrix().ldlt().solve(
        //                                    -mSkeletons[i]->getCombinedVector()
        //                                    + mSkeletons[i]->getExternalForces()
        //                                    + mSkeletons[i]->getInternalForces()
        //                                    + mCollisionHandle->getConstraintForce(i)
        //                                    );
        
        // set velocities
        deriv.segment(start, size) = getSkeleton(i)->getQDotVector() + (qddot * mTimeStep);

        // set qddot (accelerations)
        deriv.segment(start + size, size) = qddot;
    }

    return deriv;
}

////////////////////////////////////////////////////////////////////////////////
bool World::addSkeleton(dynamics::SkeletonDynamics* _skeleton)
{
    //--------------------------------------------------------------------------
    // Step 1. Check if the world already has _skel.
    //--------------------------------------------------------------------------
    for (unsigned int i = 0; i < getNumSkeletons(); i++)
        if (mSkeletons[i] == _skeleton)
            return false;

    //--------------------------------------------------------------------------
    // Step 2. Add _skel to the world.
    //--------------------------------------------------------------------------
    mSkeletons.push_back(_skeleton);
    _skeleton->initDynamics();

    // Indices update
    mIndices.push_back(mIndices.back() + _skeleton->getNumDofs());

    if(!_skeleton->getImmobileState())
    {
        // Not sure if we need this
        _skeleton->computeDynamics(mGravity,
                                   _skeleton->getQDotVector(),
                                   false);
    }

    // create a collision handler
    mCollisionHandle->addSkeleton(_skeleton);

    return true;
}

} // namespace simulation
