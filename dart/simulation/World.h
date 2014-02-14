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

#ifndef DART_SIMULATION_WORLD_H_
#define DART_SIMULATION_WORLD_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dart/common/Timer.h"
#include "dart/integration/Integrator.h"

namespace dart {
namespace integration {
class Integrator;
}  // namespace integration
namespace dynamics {
class Skeleton;
}  // namespace dynamics
namespace constraint {
class ConstraintDynamics;
}  // namespace constraint
}  // namespace dart

namespace dart {
namespace simulation {

/// \class World
/// \brief
class World : public integration::IntegrableSystem {
public:
  //--------------------------------------------------------------------------
  // Constructor and Destructor
  //--------------------------------------------------------------------------
  /// \brief Constructor.
  World();

  /// \brief Destructor.
  virtual ~World();

  //--------------------------------------------------------------------------
  // Virtual functions
  //--------------------------------------------------------------------------
  virtual Eigen::VectorXd getState() const;

  virtual void setState(const Eigen::VectorXd &_newState);

  virtual void setControlInput();

  virtual Eigen::VectorXd evalDeriv();

  //--------------------------------------------------------------------------
  // Simulation
  //--------------------------------------------------------------------------
  /// \brief Calculate the dynamics and integrate the world for one step.
  void step();

  /// \brief
  void setTime(double _time);

  /// \brief Get the time step.
  /// \return Time step.
  double getTime() const;

  /// \brief Get the number of simulated frames.
  int getSimFrames() const;

  //--------------------------------------------------------------------------
  // Properties
  //--------------------------------------------------------------------------

  /// \brief .
  /// \param[in] _gravity
  void setGravity(const Eigen::Vector3d& _gravity);

  /// \brief .
  const Eigen::Vector3d& getGravity() const;

  /// \brief .
  /// \param[in] _timeStep
  void setTimeStep(double _timeStep);

  /// \brief Get the time step.
  double getTimeStep() const;

  //--------------------------------------------------------------------------
  // Structueral Properties
  //--------------------------------------------------------------------------
  /// \brief Get the indexed skeleton.
  /// \param[in] _index
  dynamics::Skeleton* getSkeleton(int _index) const;

  /// \brief Find body node by name.
  /// \param[in] The name of body node looking for.
  /// \return Searched body node. If the skeleton does not have a body
  /// node with _name, then return NULL.
  dynamics::Skeleton* getSkeleton(const std::string& _name) const;

  /// \brief Get the number of skeletons.
  int getNumSkeletons() const;

  /// \brief .
  /// \param[in] _skel
  void addSkeleton(dynamics::Skeleton* _skeleton);

  void removeSkeleton(dynamics::Skeleton* _skeleton);

  /// \brief Get the dof index for the indexed skeleton.
  /// \param[in] _index
  int getIndex(int _index) const;

  //--------------------------------------------------------------------------
  // Kinematics
  //--------------------------------------------------------------------------
  /// \brief
  bool checkCollision(bool _checkAllCollisions = false);

  //--------------------------------------------------------------------------
  // Dynamics
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // Constraint
  //--------------------------------------------------------------------------
  /// \brief Get the constraint handler.
  constraint::ConstraintDynamics* getConstraintHandler() const;

protected:
  //--------------------------------------------------------------------------
  // Dynamics Algorithms
  //--------------------------------------------------------------------------
  /// \brief Skeletones in this world.
  std::vector<dynamics::Skeleton*> mSkeletons;

  /// \brief The first indeices of each skeleton's dof in mDofs.
  ///
  /// For example, if this world has three skeletons and their dof are
  /// 6, 1 and 2 then the mIndices goes like this: [0 6 7].
  std::vector<int> mIndices;

  /// \brief The gravity.
  Eigen::Vector3d mGravity;

  /// \brief The time step.
  double mTimeStep;

  /// \brief
  double mTime;

  /// \brief The simulated frame number.
  int mFrame;

  /// \brief The integrator.
  integration::Integrator* mIntegrator;

  /// \brief The constraint handler.
  constraint::ConstraintDynamics* mConstraintHandler;

private:
};

}  // namespace simulation
}  // namespace dart

#endif  // DART_SIMULATION_WORLD_H_
