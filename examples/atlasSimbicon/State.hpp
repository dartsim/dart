/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef EXAMPLES_ATLASSIMBICON_STATE_HPP_
#define EXAMPLES_ATLASSIMBICON_STATE_HPP_

#include <map>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include <dart/dart.hpp>

#define ATLAS_DEFAULT_KD 1.0  // No more than 1.0
#define ATLAS_DEFAULT_KP 1e+3

#define ATLAS_DEFAULT_SAGITAL_CD 0.1
#define ATLAS_DEFAULT_SAGITAL_CV 0.1

#define ATLAS_DEFAULT_CORONAL_CD 0.1
#define ATLAS_DEFAULT_CORONAL_CV 0.1

namespace dart {
namespace dynamics {
class BodyNode;
class Joint;
class Skeleton;
}  // namespace dynamics
}  // namespace dart

class TerminalCondition;

//==============================================================================
/// \brief class State
class State
{
public:
  /// \brief Constructor
  explicit State(dart::dynamics::SkeletonPtr _skeleton, const std::string& _name);

  /// \brief Destructor
  virtual ~State();

  //------------------------------- Setting ------------------------------------
  /// \brief Set name
  void setName(std::string& _name);

  /// \brief Get name
  const std::string& getName() const;

  /// \brief Set next state
  void setNextState(State* _nextState);

  /// \brief Add terminal condition
  void setTerminalCondition(TerminalCondition* _condition);

  /// \brief Get next state
  State* getNextState() const;

  /// \brief Get elapsed time
  double getElapsedTime() const;

  //----------------------- Setting Desired Position ---------------------------
  /// \brief Set desired joint position whose name is _jointName
  void setDesiredJointPosition(const std::string& _jointName, double _val);

  /// \brief Get desired joint position whose index is _idx
  double getDesiredJointPosition(int _idx) const;

  /// \brief Get desired joint position whose name is _jointName
  double getDesiredJointPosition(const std::string& _jointName) const;

  /// \brief Set desired global angle swing leg on sagital plane
  void setDesiredSwingLegGlobalAngleOnSagital(double _val);

  /// \brief Set desired global angle swing leg on coronal plane
  void setDesiredSwingLegGlobalAngleOnCoronal(double _val);

  /// \brief Set desired global angle pelvis on sagital plane
  void setDesiredPelvisGlobalAngleOnSagital(double _val);

  /// \brief Set desired global angle of pelvis on coronal plane
  void setDesiredPelvisGlobalAngleOnCoronal(double _val);

  /// \brief Set proportional gain for PD controller
  void setProportionalGain(int _idx, double _val);

  /// \brief Set proportional gain for PD controller
  void setProportionalGain(const std::string& _jointName, double _val);

  /// \brief Get proportional gain for PD controller
  double getProportionalGain(int _idx) const;

  /// \brief Get proportional gain for PD controller
  double getProportionalGain(const std::string& _jointName) const;

  /// \brief Set derivative gain for PD controller
  void setDerivativeGain(int _idx, double _val);

  /// \brief Set derivative gain for PD controller
  void setDerivativeGain(const std::string& _jointName, double _val);

  /// \brief Get derivative gain for PD controller
  double getDerivativeGain(int _idx) const;

  // /// \brief Get derivative gain for PD controller
  // double getDerivativeGain(const std::string& _jointName) const;

  /// \brief Set balance feedback gain parameter for sagital com distance
  void setFeedbackSagitalCOMDistance(std::size_t _index, double _val);

  /// \brief Set balance feedback gain parameter for sagital com velocity
  void setFeedbackSagitalCOMVelocity(std::size_t _index, double _val);

  /// \brief Set balance feedback gain parameter for sagital com distance
  void setFeedbackCoronalCOMDistance(std::size_t _index, double _val);

  /// \brief Set balance feedback gain parameter for sagital com velocity
  void setFeedbackCoronalCOMVelocity(std::size_t _index, double _val);

  /// \brief Set stance foot to left foot
  void setStanceFootToLeftFoot();

  /// \brief Set stance foot to right foot
  void setStanceFootToRightFoot();

  //------------------------------- Control ------------------------------------
  /// \brief Initiate state. This is called when the state machine transite
  ///        from the previous state to this state.
  virtual void begin(double _currentTime);

  /// \brief Compute control force and apply it to Atlas robot
  virtual void computeControlForce(double _timestep);

  /// \brief Check if terminal condision is satisfied
  virtual bool isTerminalConditionSatisfied() const;

  /// \brief Finalize state. This is called when the state machine stransite
  ///        from this state to the next state.
  virtual void end(double _currentTime);

protected:
  /// \brief Get center of mass
  Eigen::Vector3d getCOM() const;

  /// \brief Get velocity of center of mass
  Eigen::Vector3d getCOMVelocity() const;

  /// \brief Get a frame such that:
  ///        1) The origin is at the COM
  ///        2) The z-axis is perpendicular to the ground (y-axis by default)
  ///        3) The x-axis is a projected x-axis of pelvis on to perpendicular
  ///           plane against to the z-axis
  Eigen::Isometry3d getCOMFrame() const;

  /// \brief Get sagital com distance
  double getSagitalCOMDistance();

  /// \brief Get sagital com velocity
  double getSagitalCOMVelocity();

  /// \brief Get coronal com distance
  double getCoronalCOMDistance();

  /// \brief Get coronal com velocity
  double getCoronalCOMVelocity();

  /// \brief Get stance ankle position
  Eigen::Vector3d getStanceAnklePosition() const;

  /// \brief Get left ankle position
  Eigen::Vector3d getLeftAnklePosition() const;

  /// \brief Get right ankle position
  Eigen::Vector3d getRightAnklePosition() const;

  // TODO(JS): Not implemented yet
  /// \brief Get global pelvis upvector angle on sagital plane
  double getSagitalPelvisAngle() const;

  // TODO(JS): Not implemented yet
  /// \brief Get global pelvis upvector angle on coronal plane
  double getCoronalPelvisAngle() const;

  /// \brief Get global left leg angle on sagital plane
  double getSagitalLeftLegAngle() const;

  /// \brief Get global right leg angle on sagital plane
  double getSagitalRightLegAngle() const;

  /// \brief Get global left leg angle on coronal plane
  double getCoronalLeftLegAngle() const;

  /// \brief Get global right leg angle on coronal plane
  double getCoronalRightLegAngle() const;

  /// \brief Name
  std::string mName;

  /// \brief Target skeleton for control
  dart::dynamics::SkeletonPtr mSkeleton;

  /// \brief Next state. Default is myself.
  State* mNextState;

  /// \brief Terminal condition
  TerminalCondition* mTerminalCondition;

  /// \brief Started time
  double mBeginTime;

  /// \brief Stopped time
  double mEndTime;

  /// \brief Frame number
  int mFrame;

  /// \brief Elapsed time which is stopped time minus started time
  double mElapsedTime;

  /// \brief Desired joint positions
  Eigen::VectorXd mDesiredJointPositions;

  /// \brief Desired global angle of swing leg on sagital plane
  double mDesiredGlobalSwingLegAngleOnSagital;

  /// \brief Desired global angle of swing leg on coronal plane
  double mDesiredGlobalSwingLegAngleOnCoronal;

  /// \brief Desired global angle of pelvis on sagital plane
  double mDesiredGlobalPelvisAngleOnSagital;

  /// \brief Desired global angle of pelvis on coronal plane
  double mDesiredGlobalPelvisAngleOnCoronal;

  /// \brief Proportional gain for PD controller
  Eigen::VectorXd mKp;

  /// \brief Derivative gain PD controller
  Eigen::VectorXd mKd;

  /// \brief Feedback gain for com
  Eigen::VectorXd mSagitalCd;

  /// \brief Feedback gain for velocity of com
  Eigen::VectorXd mSagitalCv;

  /// \brief Feedback gain for com
  Eigen::VectorXd mCoronalCd;

  /// \brief Feedback gain for velocity of com
  Eigen::VectorXd mCoronalCv;

  /// \brief Computeed control force
  Eigen::VectorXd mTorque;

  /// \brief Joint map
  std::map<const std::string, int> mJointMap;

private:
  /// \brief Get the parent joint's position of _bodyNode
  Eigen::Vector3d _getJointPosition(dart::dynamics::BodyNode* _bodyNode) const;

  /// \brief Compute the angle between two vectors
  double _getAngleBetweenTwoVectors(const Eigen::Vector3d& _v1,
                                    const Eigen::Vector3d& _v2) const;

  /// \brief Update torque for torso and swing hip
  void _updateTorqueForStanceLeg();

  /// \brief Pelvis body node
  dart::dynamics::BodyNode* mPelvis;

  /// \brief Left foot body node
  dart::dynamics::BodyNode* mLeftThigh;

  /// \brief Right foot body node
  dart::dynamics::BodyNode* mRightThigh;

  /// \brief Left foot body node
  dart::dynamics::BodyNode* mStanceFoot;

  /// \brief Left foot body node
  dart::dynamics::BodyNode* mLeftFoot;

  /// \brief Right foot body node
  dart::dynamics::BodyNode* mRightFoot;

  /// \brief Index for coronal left hip
  std::size_t mCoronalLeftHip;

  /// \brief Index for coronal right hip
  std::size_t mCoronalRightHip;

  /// \brief Index for sagital left hip
  std::size_t mSagitalLeftHip;

  /// \brief Index for sagital right hip
  std::size_t mSagitalRightHip;

  /// \brief Desired joint positions with balance feedback
  Eigen::VectorXd mDesiredJointPositionsBalance;
};

#endif  // EXAMPLES_ATLASSIMBICON_STATE_HPP_
