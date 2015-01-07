/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#ifndef DART_SIMULATION_WORLD_H_
#define DART_SIMULATION_WORLD_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dart/common/Deprecated.h"
#include "dart/common/Timer.h"
#include "dart/common/NameManager.h"
#include "dart/simulation/Recording.h"

namespace dart {

namespace integration {
class Integrator;
}  // namespace integration

namespace dynamics {
class Skeleton;
}  // namespace dynamics

namespace constraint {
class ConstraintSolver;
}  // namespace constraint

namespace simulation {

/// class World
class World
{
public:
  //--------------------------------------------------------------------------
  // Constructor and Destructor
  //--------------------------------------------------------------------------

  /// Constructor
  World();

  /// Destructor
  virtual ~World();

  //--------------------------------------------------------------------------
  // Properties
  //--------------------------------------------------------------------------

  /// Set gravity
  void setGravity(const Eigen::Vector3d& _gravity);

  /// Get gravity
  const Eigen::Vector3d& getGravity() const;

  /// Set time step
  void setTimeStep(double _timeStep);

  /// Get time step
  double getTimeStep() const;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------

  /// Get the indexed skeleton
  dynamics::Skeleton* getSkeleton(size_t _index) const;

  /// Find body node by name
  /// \param[in] The name of body node looking for.
  /// \return Searched body node. If the skeleton does not have a body
  /// node with _name, then return NULL.
  dynamics::Skeleton* getSkeleton(const std::string& _name) const;

  /// Get the number of skeletons
  size_t getNumSkeletons() const;

  /// Add a skeleton to this world
  std::string addSkeleton(dynamics::Skeleton* _skeleton);

  /// Remove a skeleton in this world
  void removeSkeleton(dynamics::Skeleton* _skeleton);

  /// Remove all the skeletons in this world
  void removeAllSkeletons();

  /// Get the dof index for the indexed skeleton
  int getIndex(int _index) const;

  //--------------------------------------------------------------------------
  // Kinematics
  //--------------------------------------------------------------------------

  /// Return whether there is any collision between bodies
  bool checkCollision(bool _checkAllCollisions = false);

  //--------------------------------------------------------------------------
  // Simulation
  //--------------------------------------------------------------------------

  /// Reset the time, frame counter and recorded histories
  void reset();

  /// Calculate the dynamics and integrate the world for one step
  void step();

  /// Set current time
  void setTime(double _time);

  /// Get current time
  double getTime() const;

  /// Get the number of simulated frames
  int getSimFrames() const;

  //--------------------------------------------------------------------------
  // Constraint
  //--------------------------------------------------------------------------

  /// Get the constraint solver
  constraint::ConstraintSolver* getConstraintSolver() const;

  /// Bake simulated current state and store it into mRecording
  void bake();

  /// Get recording
  Recording* getRecording();

protected:
  /// Skeletones in this world
  std::vector<dynamics::Skeleton*> mSkeletons;

  /// NameManager for keeping track of Skeletons
  dart::common::NameManager<dynamics::Skeleton> mNameMgrForSkeletons;

  /// The first indeices of each skeleton's dof in mDofs
  ///
  /// For example, if this world has three skeletons and their dof are
  /// 6, 1 and 2 then the mIndices goes like this: [0 6 7].
  std::vector<int> mIndices;

  /// Gravity
  Eigen::Vector3d mGravity;

  /// Simulation time step
  double mTimeStep;

  /// Current simulation time
  double mTime;

  /// Current simulation frame number
  int mFrame;

  /// The integrator
  DEPRECATED(4.3)
  integration::Integrator* mIntegrator;

  /// Constraint solver
  constraint::ConstraintSolver* mConstraintSolver;

  ///
  Recording* mRecording;
};

}  // namespace simulation
}  // namespace dart

#endif  // DART_SIMULATION_WORLD_H_
