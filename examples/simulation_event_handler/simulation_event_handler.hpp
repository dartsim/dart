/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef EXAMPLES_SIMULATIONEVENTHANDLER_SIMULATIONEVENTHANDLER_HPP_
#define EXAMPLES_SIMULATIONEVENTHANDLER_SIMULATIONEVENTHANDLER_HPP_

#include <dart/gui/all.hpp>

#include <dart/all.hpp>

#include <osgGA/GUIEventHandler>

/// @brief Comprehensive event handler for rigid body physics simulation
///
/// This class replaces MyWindow.cpp functionality and provides:
/// - Force application on rigid bodies
/// - Keyboard interaction for simulation control
/// - Arrow visualization for applied forces
/// - Time stepping logic
class SimulationEventHandler : public osgGA::GUIEventHandler
{
public:
  /// @brief Constructor
  /// @param world The physics world to control
  /// @param viewer The OSG viewer for visualization
  explicit SimulationEventHandler(
      dart::simulation::WorldPtr world, dart::gui::Viewer* viewer = nullptr);

  /// @brief Destructor
  virtual ~SimulationEventHandler() = default;

  /// @brief Handle GUI events (keyboard input, etc.)
  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

  /// @brief Update simulation state (called each frame)
  void update();

  /// @brief Set the time step for the simulation
  void setTimeStep(double timeStep)
  {
    mTimeStep = timeStep;
  }

  /// @brief Get the current time step
  double getTimeStep() const
  {
    return mTimeStep;
  }

  /// @brief Enable/disable force visualization arrows
  void setShowForceArrows(bool show)
  {
    mShowForceArrows = show;
  }

  /// @brief Check if force arrows are shown
  bool getShowForceArrows() const
  {
    return mShowForceArrows;
  }

  /// @brief Set the magnitude of applied forces
  void setForceMagnitude(double magnitude)
  {
    mForceMagnitude = magnitude;
  }

  /// @brief Get the force magnitude
  double getForceMagnitude() const
  {
    return mForceMagnitude;
  }

protected:
  /// @brief Apply force to the selected rigid body
  /// @param force The force vector to apply (in world coordinates)
  void applyForceToSelectedBody(const Eigen::Vector3d& force);

  /// @brief Apply torque to the selected rigid body
  /// @param torque The torque vector to apply (in world coordinates)
  void applyTorqueToSelectedBody(const Eigen::Vector3d& torque);

  /// @brief Update force visualization arrows
  void updateForceArrows();

  /// @brief Create and add force arrow visualization
  /// @param bodyNode The body node to attach arrow to
  /// @param force The force vector to visualize
  void addForceArrow(
      dart::dynamics::BodyNodePtr bodyNode, const Eigen::Vector3d& force);

  /// @brief Clear all force arrows
  void clearForceArrows();

  /// @brief Select the next rigid body in the world
  void selectNextBody();

  /// @brief Select the previous rigid body in the world
  void selectPreviousBody();

  /// @brief Reset the simulation to initial state
  void resetSimulation();

  /// @brief Step the simulation forward one time step
  void stepSimulation();

  /// @brief Toggle simulation play/pause
  void toggleSimulation();

  /// @brief Print current simulation state
  void printSimulationState();

  /// @brief Print usage instructions
  void printInstructions();

  /// @brief Get all rigid bodies (bodies with FreeJoint or similar)
  std::vector<dart::dynamics::BodyNodePtr> getRigidBodies();

private:
  /// The physics world
  dart::simulation::WorldPtr mWorld;

  /// The OSG viewer for visualization
  dart::gui::Viewer* mViewer;

  /// Currently selected body node for force application
  dart::dynamics::BodyNodePtr mSelectedBody;

  /// Index of currently selected body
  std::size_t mSelectedBodyIndex;

  /// List of all rigid bodies in the world
  std::vector<dart::dynamics::BodyNodePtr> mRigidBodies;

  /// Simulation time step
  double mTimeStep;

  /// Whether simulation is running
  bool mSimulationRunning;

  /// Force magnitude for applied forces
  double mForceMagnitude;

  /// Torque magnitude for applied torques
  double mTorqueMagnitude;

  /// Whether to show force arrows
  bool mShowForceArrows;

  /// Current applied forces for visualization
  std::vector<std::pair<dart::dynamics::BodyNodePtr, Eigen::Vector3d>>
      mAppliedForces;

  /// Current applied torques for visualization
  std::vector<std::pair<dart::dynamics::BodyNodePtr, Eigen::Vector3d>>
      mAppliedTorques;

  /// Arrow visualization nodes
  std::vector<::osg::ref_ptr<::osg::Group>> mForceArrowNodes;

  /// Arrow visualization nodes for torques
  std::vector<::osg::ref_ptr<::osg::Group>> mTorqueArrowNodes;

  /// Frame counter for arrow updates
  std::size_t mFrameCounter;

  /// Arrow update frequency (frames)
  static constexpr std::size_t ARROW_UPDATE_FREQUENCY = 5;

  /// Default time step
  static constexpr double DEFAULT_TIME_STEP = 0.001;

  /// Default force magnitude
  static constexpr double DEFAULT_FORCE_MAGNITUDE = 100.0;

  /// Default torque magnitude
  static constexpr double DEFAULT_TORQUE_MAGNITUDE = 50.0;
};

#endif // EXAMPLES_SIMULATIONEVENTHANDLER_SIMULATIONEVENTHANDLER_HPP_
