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

#ifndef SIMULATION_WORLD_H
#define SIMULATION_WORLD_H

#include <vector>
#include <Eigen/Dense>

#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "dynamics/SkeletonDynamics.h"

namespace dynamics {
class ContactDynamics;
} // namespace dynamics

namespace simulation {

/// @class World
/// @brief
class World : public integration::IntegrableSystem {

public:
    /// @brief Constructor.
    World();

    /// @brief Destructor.
    virtual ~World();

    /// @brief .
    void init();

    /// @brief
    // TODO: not implemented yet.
    void fini();

    /// @breif
    // TODO: not implemented yet.
    void reset();

    /// @brief
    void prestep();

    /// @brief Calculate the dynamics and integrate the world with it.
    /// @return true if the physics is updated, false if not.
    bool updatePhysics();

    /// @brief .
    /// @param[in] _gravity
    void setGravity(const Eigen::Vector3d& _gravity);

    /// @brief .
    /// @param[in] _timeStep
    void setTimeStep(double _timeStep);

    /// @brief .
    const Eigen::Vector3d& getGravity(void) const;

    /// @brief Get the time step.
    double getTimeStep(void) const;

    /// @brief .
    virtual Eigen::VectorXd getState();

    /// @brief .
    /// @param[in] _state
    virtual void setState(const Eigen::VectorXd& _state);

    /// @brief .
    virtual Eigen::VectorXd evalDeriv();

    /// @brief .
    /// @param[in] _skel
    bool addSkeleton(dynamics::SkeletonDynamics* _skel);

    /// @brief .
    std::vector<Eigen::VectorXd> getDofs() {return mDofs;}

protected:
    /// @brief Skeletones in this world.
    std::vector<dynamics::SkeletonDynamics*> mSkels;

    /// @brief Stacks the dof values.
    std::vector<Eigen::VectorXd> mDofs;

    /// @brief Stacks the dof velocities.
    std::vector<Eigen::VectorXd> mDofVels;

    /// @brief The first indeices of each skeleton's dof in mDofs.
    ///
    /// For example, if this world has three skeletons and their dof are
    /// 6, 1 and 2 then the mIndices goes like this: [0 6 7].
    std::vector<int> mIndices;

    /// @brief The integrator.
    integration::EulerIntegrator mIntegrator;

    /// @brief The collision handler.
    dynamics::ContactDynamics* mCollisionHandle;

    /// @brief The gravity.
    Eigen::Vector3d mGravity;

    /// @brief The time step.
    double mTimeStep;

    /// @brief The running indicator.
    /// (true: running, false: not running)
    bool mRunning;

    /// @brief The simulated frame number.
    int mFrame;

    /// @brief
    bool mIsInitialized;

private:
};

} // namespace simulation

#endif // #ifndef SIMULATION_WORLD_H
