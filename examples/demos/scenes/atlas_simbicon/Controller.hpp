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

// Ported wholesale from examples/atlas_simbicon/Controller.{hpp,cpp} into the
// dart_demos::atlas_simbicon namespace. No behavior changes; see
// AtlasSimbiconScene.cpp for the parity decision on the unwired
// Controller::keyboard() harness/state-select bindings (kept, still unwired,
// exactly as in the original).
//
// Implementation of Simbicon (Simple biped locomotion control) for the Atlas
// robot. Reference: http://dl.acm.org/citation.cfm?id=1276509

#ifndef DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_CONTROLLER_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_CONTROLLER_HPP_

#include <dart/dart.hpp>

#include <vector>

namespace dart_demos {
namespace atlas_simbicon {

class StateMachine;

//==============================================================================
class Controller
{
public:
  Controller(
      dart::dynamics::SkeletonPtr atlasRobot,
      dart::constraint::ConstraintSolver* collisionSolver);
  virtual ~Controller();

  /// Called before every simulation time step; computes control force and
  /// applies it to the Atlas robot.
  virtual void update();

  dart::dynamics::SkeletonPtr getAtlasRobot();

  StateMachine* getCurrentState();

  void changeStateMachine(StateMachine* stateMachine, double currentTime);
  void changeStateMachine(const std::string& name, double currentTime);
  void changeStateMachine(std::size_t idx, double currentTime);

  /// True iff this controller is currently allowed to control the robot
  /// (pelvis height within [mMinPelvisHeight, mMaxPelvisHeight] -- an
  /// emergency cutoff for when the robot falls).
  bool isAllowingControl() const;

  /// Unwired keyboard control (h/j/k harness toggles, r reset, n force next
  /// state, 1-4 select state machine); kept for parity but never registered
  /// with any event handler -- see AtlasSimbiconScene.cpp's file comment.
  void keyboard(unsigned char key, int x, int y, double currentTime);

  void printDebugInfo() const;

  void harnessPelvis();
  void unharnessPelvis();
  void harnessLeftFoot();
  void unharnessLeftFoot();
  void harnessRightFoot();
  void unharnessRightFoot();

  void resetRobot();

  void setVerbosity(bool verbosity);

protected:
  dart::dynamics::SkeletonPtr mAtlasRobot;
  dart::constraint::ConstraintSolver* mConstratinSolver;

  std::vector<StateMachine*> mStateMachines;
  StateMachine* mCurrentStateMachine;

  bool mPelvisHarnessOn;
  bool mLeftFootHarnessOn;
  bool mRightFootHarnessOn;

  std::size_t mCoronalLeftHip;
  std::size_t mCoronalRightHip;
  std::size_t mSagitalLeftHip;
  std::size_t mSagitalRightHip;

  double mMinPelvisHeight;
  double mMaxPelvisHeight;

private:
  bool _containStateMachine(const StateMachine* stateMachine) const;
  bool _containStateMachine(const std::string& name) const;
  StateMachine* _findStateMachine(const std::string& name) const;

  void _buildStateMachines();
  StateMachine* _createStandingStateMachine();
  StateMachine* _createWalkingInPlaceStateMachine();
  StateMachine* _createWalkingStateMachine();
  StateMachine* _createRunningStateMachine();

  void _setJointDamping();

  dart::dynamics::BodyNode* _getLeftFoot() const;
  dart::dynamics::BodyNode* _getRightFoot() const;

  dart::constraint::WeldJointConstraintPtr mWeldJointConstraintPelvis;
  dart::constraint::WeldJointConstraintPtr mWeldJointConstraintLeftFoot;
  dart::constraint::WeldJointConstraintPtr mWeldJointConstraintRightFoot;

  dart::dynamics::Skeleton::Configuration mInitialState;

  bool mVerbosity;
};

} // namespace atlas_simbicon
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_CONTROLLER_HPP_
