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

// Ported wholesale from examples/atlas_simbicon/State.{hpp,cpp} into the
// dart_demos::atlas_simbicon namespace. No behavior changes. This state
// machine (and the Simbicon balance feedback in particular) is written
// against a Y-up world -- getCOMFrame() hardcodes UnitY as "the axis
// perpendicular to the ground" -- so, unlike every other Y-up .skel world in
// this catalog, AtlasSimbiconScene.cpp deliberately does NOT reorient this
// scene's world to the host's usual Z-up convention (see that file's
// comment): ZUp.hpp only rotates a skeleton's root-joint transform and
// gravity, it cannot retrofit "up" into code that reads world-Y directly
// throughout this file (getCOMFrame, the sagittal/coronal projections, and
// Controller::isAllowingControl's pelvis-height cutoff all assume Y-up).

#ifndef DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_STATE_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_STATE_HPP_

#include <dart/dart.hpp>

#include <map>
#include <string>

namespace dart_demos {
namespace atlas_simbicon {

constexpr double kAtlasDefaultKd = 1.0; // No more than 1.0
constexpr double kAtlasDefaultKp = 1e+3;

class TerminalCondition;

//==============================================================================
/// One Simbicon state: a desired joint-position target (plus sagittal/coronal
/// COM feedback gains) held via a PD controller until its terminal condition
/// fires, at which point the owning StateMachine transitions to
/// getNextState().
class State
{
public:
  State(dart::dynamics::SkeletonPtr skeleton, const std::string& name);
  virtual ~State();

  void setName(std::string& name);
  const std::string& getName() const;

  void setNextState(State* nextState);
  void setTerminalCondition(TerminalCondition* condition);
  State* getNextState() const;
  double getElapsedTime() const;

  void setDesiredJointPosition(const std::string& jointName, double val);
  double getDesiredJointPosition(int idx) const;

  void setDesiredSwingLegGlobalAngleOnSagital(double val);
  void setDesiredSwingLegGlobalAngleOnCoronal(double val);
  void setDesiredPelvisGlobalAngleOnSagital(double val);
  void setDesiredPelvisGlobalAngleOnCoronal(double val);

  void setProportionalGain(int idx, double val);
  double getProportionalGain(int idx) const;
  void setDerivativeGain(int idx, double val);
  double getDerivativeGain(int idx) const;

  void setFeedbackSagitalCOMDistance(std::size_t index, double val);
  void setFeedbackSagitalCOMVelocity(std::size_t index, double val);
  void setFeedbackCoronalCOMDistance(std::size_t index, double val);
  void setFeedbackCoronalCOMVelocity(std::size_t index, double val);

  void setStanceFootToLeftFoot();
  void setStanceFootToRightFoot();

  virtual void begin(double currentTime);
  virtual void computeControlForce(double timestep);
  virtual bool isTerminalConditionSatisfied() const;
  virtual void end(double currentTime);

protected:
  Eigen::Vector3d getCOM() const;
  Eigen::Vector3d getCOMVelocity() const;

  /// Frame whose origin is the COM, whose Y-axis is up (perpendicular to the
  /// Y-up ground -- see the file comment), and whose X-axis is the pelvis'
  /// own X-axis projected onto the plane perpendicular to Y.
  Eigen::Isometry3d getCOMFrame() const;

  double getSagitalCOMDistance();
  double getSagitalCOMVelocity();
  double getCoronalCOMDistance();
  double getCoronalCOMVelocity();

  Eigen::Vector3d getStanceAnklePosition() const;
  Eigen::Vector3d getLeftAnklePosition() const;
  Eigen::Vector3d getRightAnklePosition() const;

  double getSagitalPelvisAngle() const;
  double getCoronalPelvisAngle() const;
  double getSagitalLeftLegAngle() const;
  double getSagitalRightLegAngle() const;
  double getCoronalLeftLegAngle() const;
  double getCoronalRightLegAngle() const;

  std::string mName;
  dart::dynamics::SkeletonPtr mSkeleton;
  State* mNextState;
  TerminalCondition* mTerminalCondition;

  double mBeginTime;
  double mEndTime;
  int mFrame;
  double mElapsedTime;

  Eigen::VectorXd mDesiredJointPositions;

  double mDesiredGlobalSwingLegAngleOnSagital;
  double mDesiredGlobalSwingLegAngleOnCoronal;
  double mDesiredGlobalPelvisAngleOnSagital;
  double mDesiredGlobalPelvisAngleOnCoronal;

  Eigen::VectorXd mKp;
  Eigen::VectorXd mKd;

  Eigen::VectorXd mSagitalCd;
  Eigen::VectorXd mSagitalCv;
  Eigen::VectorXd mCoronalCd;
  Eigen::VectorXd mCoronalCv;

  Eigen::VectorXd mTorque;

  std::map<const std::string, int> mJointMap;

private:
  Eigen::Vector3d _getJointPosition(dart::dynamics::BodyNode* bodyNode) const;
  double _getAngleBetweenTwoVectors(
      const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const;
  void _updateTorqueForStanceLeg();

  dart::dynamics::BodyNode* mPelvis;
  dart::dynamics::BodyNode* mLeftThigh;
  dart::dynamics::BodyNode* mRightThigh;
  dart::dynamics::BodyNode* mStanceFoot;
  dart::dynamics::BodyNode* mLeftFoot;
  dart::dynamics::BodyNode* mRightFoot;

  std::size_t mCoronalLeftHip;
  std::size_t mCoronalRightHip;
  std::size_t mSagitalLeftHip;
  std::size_t mSagitalRightHip;

  Eigen::VectorXd mDesiredJointPositionsBalance;
};

} // namespace atlas_simbicon
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_ATLASSIMBICON_STATE_HPP_
