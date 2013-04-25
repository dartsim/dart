/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 03/25/2013
 *
 * Georgia Tech Graphics Lab
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
//      mTimeStep(0.001),
      mTime(0.0),
      mSimulating(false),
      mFrame(0),
      mIsInitialized(false)
{
}

////////////////////////////////////////////////////////////////////////////////
World::~World()
{
}

////////////////////////////////////////////////////////////////////////////////
void World::init()
{
    int sumNDofs = 0;
    mIndices.clear();
    mIndices.push_back(0);

    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        int nDofs = mSkels[i]->getNumDofs();
        sumNDofs += nDofs;
        mIndices.push_back(sumNDofs);
    }

    mDofs.resize(mSkels.size());
    mDofVels.resize(mSkels.size());

    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        mDofs[i].resize(mSkels[i]->getNumDofs());
        mDofVels[i].resize(mSkels[i]->getNumDofs());

        // Copy the values of Dofs
        for (unsigned int j = 0; j < mSkels[i]->getNumDofs(); j++)
        {
            mDofs[i][j] = mSkels[i]->getDof(j)->getValue();
        }
        mDofVels[i].setZero();
    }

//    for (unsigned int i = 0; i < mSkels.size(); i++)
//    {
//        mSkels[i]->initDynamics();

//        // Set flags to skip transformation and first-derivatives
//        // updates.
//        //mSkels[i]->setPose(mDofs[i], false, false);
//    }

    // create a collision handler
    mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);

    mIsInitialized = true;

    //dtmsg << "World is initialized.\n";
}

////////////////////////////////////////////////////////////////////////////////
void World::fini()
{
    mIsInitialized = false;
}

////////////////////////////////////////////////////////////////////////////////
void World::reset()
{
    // Assume that the world is initialized.
    assert(mIsInitialized == true);

    // Reset all states: mDofs and mDofVels.
    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        mDofs[i].setZero(mSkels[i]->getNumDofs());
        mDofVels[i].setZero(mSkels[i]->getNumDofs());
    }

    // Claculate transformations (forward kinematics).
    for (unsigned int i = 0; i < mSkels.size(); ++i)
    {
        mSkels[i]->setPose(mDofs[i], true, false);
    }

    // Calculate velocities represented in world frame (forward kinematics).
    dynamics::BodyNodeDynamics* itrBodyNodeDyn = NULL;
    for (unsigned int i = 0; i < mSkels.size(); ++i)
    {
        for (unsigned int j = 0; j < mSkels[i]->getNumNodes(); j++)
        {
            itrBodyNodeDyn
                    = static_cast<dynamics::BodyNodeDynamics*>(mSkels[i]->getNode(j));
            itrBodyNodeDyn->evalVelocity(mDofVels[i]);
        }
    }

    // Contact reset.
    mCollisionHandle->reset();

    resetTime();
}

////////////////////////////////////////////////////////////////////////////////
dynamics::BodyNodeDynamics* World::getBodyNodeDynamics(
        const char* const _name) const
{
    dynamics::BodyNodeDynamics* result = NULL;

    for (unsigned int i = 0; i < mSkels.size(); ++i)
    {
        dynamics::BodyNodeDynamics* skelResult
                = static_cast<dynamics::BodyNodeDynamics*>(
                      mSkels[i]->getBodyNode(_name));

        if (skelResult != NULL)
        {
            result = skelResult;
            return result;
        }
    }

    result;
}

////////////////////////////////////////////////////////////////////////////////
kinematics::Joint* World::getJoint(const char* const _name) const
{
    kinematics::Joint* result = NULL;

    for (unsigned int i = 0; i < mSkels.size(); ++i)
    {
        kinematics::Joint* skelResult = mSkels[i]->getJoint(_name);

        if (skelResult != NULL)
        {
            result = skelResult;
            return result;
        }
    }

    result;
}

////////////////////////////////////////////////////////////////////////////////
bool World::updatePhysics()
{
    assert(mIsInitialized);

    // Calculate (q, qdot) by integrating with (qdot, qdotdot).
    mIntegrator.integrate(this, mTimeStep);

    // Calculate body node's velocities represented in world frame.
    dynamics::BodyNodeDynamics* itrBodyNodeDyn = NULL;
    for (unsigned int i = 0; i < mSkels.size(); ++i)
    {
        for (unsigned int j = 0; j < mSkels[i]->getNumNodes(); j++)
        {
            itrBodyNodeDyn
                    = static_cast<dynamics::BodyNodeDynamics*>(mSkels[i]->getNode(j));
            itrBodyNodeDyn->evalVelocity(mDofVels[i]);
        }
    }

    // Add time
    mTime += mTimeStep;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool World::updatePhysics(double _timeStep)
{
    assert(mIsInitialized);

    // Calculate (q, qdot) by integrating with (qdot, qdotdot).
    mIntegrator.integrate(this, _timeStep);

    // Calculate body node's velocities represented in world frame.
    dynamics::BodyNodeDynamics* itrBodyNodeDyn = NULL;
    for (unsigned int i = 0; i < mSkels.size(); ++i)
    {
        for (unsigned int j = 0; j < mSkels[i]->getNumNodes(); j++)
        {
            itrBodyNodeDyn
                    = static_cast<dynamics::BodyNodeDynamics*>(mSkels[i]->getNode(j));
            itrBodyNodeDyn->evalVelocity(mDofVels[i]);
        }
    }

    mTime += _timeStep;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd World::getState()
{
    Eigen::VectorXd state(mIndices.back() * 2);

    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        state.segment(start, size) = mDofs[i];
        state.segment(start + size, size) = mDofVels[i];
    }

    return state;
}

////////////////////////////////////////////////////////////////////////////////
void World::setState(const Eigen::VectorXd& _state)
{
    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mDofs[i] = _state.segment(start, size);
        mDofVels[i] = _state.segment(start + size, size);
    }
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd World::evalDeriv()
{
    // TODO: !!!!
    // The root follows the desired acceleration and the rest of the
    // body follows dynamic equations. The root acceleration will impact
    // the rest of the body correctly. Collision or other external
    // forces will alter the root acceleration
    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        if (mSkels[i]->getImmobileState())
        {
            mSkels[i]->setPose(mDofs[i], true, false);
        }
        else
        {
            mSkels[i]->setPose(mDofs[i], false, true);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
    }

    // compute contact forces
    mCollisionHandle->applyContactForces();

    Eigen::VectorXd deriv = Eigen::VectorXd::Zero(mIndices.back() * 2);

    // compute derivatives for integration
    for (unsigned int i = 0; i < mSkels.size(); i++)
    {
        // skip immobile objects in forward simulation
        if (mSkels[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();

        Eigen::VectorXd qddot = mSkels[i]->getInvMassMatrix()
                * (-mSkels[i]->getCombinedVector()
                   + mSkels[i]->getExternalForces()
                   + mSkels[i]->getInternalForces()
                   + mCollisionHandle->getConstraintForce(i)
                   );
        mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);

        // set velocities
        deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep);

        // set qddot (accelerations)
        deriv.segment(start + size, size) = qddot;
    }

    return deriv;
}

////////////////////////////////////////////////////////////////////////////////
bool World::addSkeleton(dynamics::SkeletonDynamics* _skel)
{
    //--------------------------------------------------------------------------
    // Step 1. Check if the world already has _skel.
    //--------------------------------------------------------------------------
    for (unsigned int i = 0; i < mSkels.size(); i++)
        if (mSkels[i] == _skel)
            return false;

    //--------------------------------------------------------------------------
    // Step 2. Add _skel to the world.
    //--------------------------------------------------------------------------
    if (!mIsInitialized)
    {
        mSkels.push_back(_skel);
    }
    else
    {
        unsigned int nSkels = mSkels.size();
        unsigned int sumNDofs = 0;
        if (nSkels != 0)
            sumNDofs = mIndices[nSkels] + mSkels[nSkels-1]->getNumDofs();

        // Indices update
        mIndices.push_back(sumNDofs);

        //
        mSkels.push_back(_skel);

        Eigen::VectorXd newDofs(_skel->getNumDofs());
        Eigen::VectorXd newDofVels = Eigen::VectorXd::Zero(_skel->getNumDofs());

        // Copy the values of Dofs
        for (unsigned int i = 0; i < _skel->getNumDofs(); i++)
        {
            newDofs[i] = _skel->getDof(i)->getValue();
        }
        //newDofVels.setZero();

        // Push back
        mDofs.push_back(newDofs);
        mDofVels.push_back(newDofVels);

        //
//        _skel->initSkel();
//        _skel->initDynamics();

        // Set flags to skip transformation and first-derivatives
        // updates.
        //_skel->setPose(newDofs, false, false);

        // create a collision handler
        mCollisionHandle->addSkeleton(_skel);
    }

    return true;
}

} // namespace simulation
