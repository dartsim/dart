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

#ifndef SIMULATION_WORLD_H
#define SIMULATION_WORLD_H

#include <vector>
#include <Eigen/Dense>

#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/Deprecated.h"

namespace dynamics {
class ConstraintDynamics;
class BodyNodeDynamics;
} // namespace dynamics

namespace simulation {

/// @class World
/// @brief
class World : public integration::IntegrableSystem
{
public:
    /// @brief Constructor.
    World();

    /// @brief Destructor.
    virtual ~World();

    /// @breif Reset the world.
    ///
    /// Set Dofs and DofVels as zero (or initial value) and update all
    /// transformations and velocities of each links.
    void reset();

    /// @brief Calculate the dynamics and integrate the world for one step.
    void step();

    /// @brief .
    /// @param[in] _gravity
    void setGravity(const Eigen::Vector3d& _gravity) { mGravity = _gravity; }

    /// @brief .
    const Eigen::Vector3d& getGravity() const { return mGravity; }

    /// @brief .
    /// @param[in] _timeStep
    void setTimeStep(double _timeStep);

    /// @brief Get the time step.
    double getTimeStep() const { return mTimeStep; }

    /// @brief
    void setTime(double _time) { mTime = _time; }

    /// @brief Get the time step.
    /// @return Time step.
    double getTime() const { return mTime; }

    /// @brief Get the indexed skeleton.
    /// @param[in] _index
    dynamics::SkeletonDynamics* getSkeleton(int _index) const;

    /// @brief Find body node by name.
    /// @param[in] The name of body node looking for.
    /// @return Searched body node. If the skeleton does not have a body
    /// node with _name, then return NULL.
    dynamics::SkeletonDynamics* getSkeleton(const char* const _name) const;

    /// @brief Get the number of skeletons.
    unsigned int getNumSkeletons() const { return mSkeletons.size(); }

    /// @brief Get the number of simulated frames.
    int getSimFrames() const { return mFrame; }

    /// @brief Get the collision handler.
    inline dynamics::ConstraintDynamics* getCollisionHandle() const
    { return mCollisionHandle; }

    /// @brief Get the dof index for the indexed skeleton.
    /// @param[in] _index
    int getIndex(int _index) const { return mIndices[_index]; }

    // Documentation inherited.
    virtual Eigen::VectorXd getState();

    // Documentation inherited.
    virtual void setState(const Eigen::VectorXd& _newState);

    // Documentation inherited.
    virtual Eigen::VectorXd evalDeriv();

    /// @brief .
    /// @param[in] _skel
    bool addSkeleton(dynamics::SkeletonDynamics* _skeleton);

    /// @brief
    bool checkCollision(bool checkAllCollisions = false);

protected:
    /// @brief Skeletones in this world.
    std::vector<dynamics::SkeletonDynamics*> mSkeletons;

    /// @brief The first indeices of each skeleton's dof in mDofs.
    ///
    /// For example, if this world has three skeletons and their dof are
    /// 6, 1 and 2 then the mIndices goes like this: [0 6 7].
    std::vector<int> mIndices;

    /// @brief The integrator.
    integration::EulerIntegrator mIntegrator;

    /// @brief The collision handler.
    dynamics::ConstraintDynamics* mCollisionHandle;

    /// @brief The gravity.
    Eigen::Vector3d mGravity;

    /// @brief The time step.
    double mTimeStep;

    /// @brief
    double mTime;

    /// @brief The simulated frame number.
    int mFrame;

private:
};

} // namespace simulation

#endif // #ifndef SIMULATION_WORLD_H
