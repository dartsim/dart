/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/weld_joint_constraint.hpp>

#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/point_mass.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/helpers.hpp>

#include <dart/common/logging.hpp>
#include <dart/common/macros.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::examples::demos {

// A named (not anonymous) namespace so the ported Simbicon controller's unused
// debug/interaction members keep external linkage and do not trip
// -Werror=unused-function; the unique name avoids clashes with other scenes.
namespace atlas_simbicon_detail {

using namespace dart::dynamics;
using namespace dart::constraint;

#define ATLAS_DEFAULT_KD 1.0 // No more than 1.0
#define ATLAS_DEFAULT_KP 1e+3

#define ATLAS_DEFAULT_SAGITAL_CD 0.1
#define ATLAS_DEFAULT_SAGITAL_CV 0.1

#define ATLAS_DEFAULT_CORONAL_CD 0.1
#define ATLAS_DEFAULT_CORONAL_CV 0.1

// Macro for functions not implemented yet
#define NOT_YET(FUNCTION)                                                      \
  std::cout << #FUNCTION << "Not implemented yet." << std::endl;

class State;
class StateMachine;

//==============================================================================
/// @brief class TerminalCondition
class TerminalCondition
{
public:
  /// @brief Constructor
  TerminalCondition(State* _state);

  /// @brief Destructor
  virtual ~TerminalCondition();

  /// @brief Check if this condition is satisfied or not.
  virtual bool isSatisfied() = 0;

protected:
  /// @brief State
  State* mState;
};

//==============================================================================
/// @brief class TimerCondition
class TimerCondition : public TerminalCondition
{
public:
  /// @brief Constructor
  TimerCondition(State* _state, double _duration);

  /// @brief Destructor
  virtual ~TimerCondition();

  // Documentation inherited.
  bool isSatisfied() override;

protected:
  /// @brief Duration
  double mDuration;
};

//==============================================================================
/// @brief class BodyContactCondition
class BodyContactCondition : public TerminalCondition
{
public:
  /// @brief Constructor
  BodyContactCondition(
      State* _state,
      dart::dynamics::BodyNode* _body,
      dart::constraint::ConstraintSolver* constraintSolver);

  /// @brief Destructor
  virtual ~BodyContactCondition();

  // Documentation inherited.
  bool isSatisfied() override;

protected:
  /// @brief Body node to be tested
  dart::dynamics::BodyNode* mBodyNode;

  /// @brief Constraint solver that carries the last collision result
  dart::constraint::ConstraintSolver* mConstraintSolver;
};

//==============================================================================
/// @brief class State
class State
{
public:
  /// @brief Constructor
  explicit State(
      dart::dynamics::SkeletonPtr _skeleton, const std::string& _name);

  /// @brief Destructor
  virtual ~State();

  //------------------------------- Setting ------------------------------------
  /// @brief Set name
  void setName(std::string& _name);

  /// @brief Get name
  const std::string& getName() const;

  /// @brief Set next state
  void setNextState(State* _nextState);

  /// @brief Add terminal condition
  void setTerminalCondition(TerminalCondition* _condition);

  /// @brief Get next state
  State* getNextState() const;

  /// @brief Get elapsed time
  double getElapsedTime() const;

  //----------------------- Setting Desired Position ---------------------------
  /// @brief Set desired joint position whose name is _jointName
  void setDesiredJointPosition(const std::string& _jointName, double _val);

  /// @brief Get desired joint position whose index is _idx
  double getDesiredJointPosition(int _idx) const;

  /// @brief Get desired joint position whose name is _jointName
  double getDesiredJointPosition(const std::string& _jointName) const;

  /// @brief Set desired global angle swing leg on sagittal plane
  void setDesiredSwingLegGlobalAngleOnSagittal(double _val);

  /// @brief Set desired global angle swing leg on coronal plane
  void setDesiredSwingLegGlobalAngleOnCoronal(double _val);

  /// @brief Set desired global angle pelvis on sagittal plane
  void setDesiredPelvisGlobalAngleOnSagittal(double _val);

  /// @brief Set desired global angle of pelvis on coronal plane
  void setDesiredPelvisGlobalAngleOnCoronal(double _val);

  /// @brief Set proportional gain for PD controller
  void setProportionalGain(int _idx, double _val);

  /// @brief Set proportional gain for PD controller
  void setProportionalGain(const std::string& _jointName, double _val);

  /// @brief Get proportional gain for PD controller
  double getProportionalGain(int _idx) const;

  /// @brief Get proportional gain for PD controller
  double getProportionalGain(const std::string& _jointName) const;

  /// @brief Set derivative gain for PD controller
  void setDerivativeGain(int _idx, double _val);

  /// @brief Set derivative gain for PD controller
  void setDerivativeGain(const std::string& _jointName, double _val);

  /// @brief Get derivative gain for PD controller
  double getDerivativeGain(int _idx) const;

  // /// @brief Get derivative gain for PD controller
  // double getDerivativeGain(const std::string& _jointName) const;

  /// @brief Set balance feedback gain parameter for sagittal com distance
  void setFeedbackSagittalCOMDistance(std::size_t _index, double _val);

  /// @brief Set balance feedback gain parameter for sagittal com velocity
  void setFeedbackSagittalCOMVelocity(std::size_t _index, double _val);

  /// @brief Set balance feedback gain parameter for sagittal com distance
  void setFeedbackCoronalCOMDistance(std::size_t _index, double _val);

  /// @brief Set balance feedback gain parameter for sagittal com velocity
  void setFeedbackCoronalCOMVelocity(std::size_t _index, double _val);

  /// @brief Set stance foot to left foot
  void setStanceFootToLeftFoot();

  /// @brief Set stance foot to right foot
  void setStanceFootToRightFoot();

  //------------------------------- Control ------------------------------------
  /// @brief Initiate state. This is called when the state machine transite
  ///        from the previous state to this state.
  virtual void begin(double _currentTime);

  /// @brief Compute control force and apply it to Atlas robot
  virtual void computeControlForce(double _timestep);

  /// @brief Check if terminal condision is satisfied
  virtual bool isTerminalConditionSatisfied() const;

  /// @brief Finalize state. This is called when the state machine stransite
  ///        from this state to the next state.
  virtual void end(double _currentTime);

protected:
  /// @brief Get center of mass
  Eigen::Vector3d getCOM() const;

  /// @brief Get velocity of center of mass
  Eigen::Vector3d getCOMVelocity() const;

  /// @brief Get a frame such that:
  ///        1) The origin is at the COM
  ///        2) The z-axis is world up and perpendicular to the ground
  ///        3) The x-axis is the pelvis x-axis projected onto the ground plane
  Eigen::Isometry3d getCOMFrame() const;

  /// @brief Get sagittal com distance
  double getSagittalCOMDistance();

  /// @brief Get sagittal com velocity
  double getSagittalCOMVelocity();

  /// @brief Get coronal com distance
  double getCoronalCOMDistance();

  /// @brief Get coronal com velocity
  double getCoronalCOMVelocity();

  /// @brief Get stance ankle position
  Eigen::Vector3d getStanceAnklePosition() const;

  /// @brief Get left ankle position
  Eigen::Vector3d getLeftAnklePosition() const;

  /// @brief Get right ankle position
  Eigen::Vector3d getRightAnklePosition() const;

  // TODO(JS): Not implemented yet
  /// @brief Get global pelvis upvector angle on sagittal plane
  double getSagittalPelvisAngle() const;

  // TODO(JS): Not implemented yet
  /// @brief Get global pelvis upvector angle on coronal plane
  double getCoronalPelvisAngle() const;

  /// @brief Get global left leg angle on sagittal plane
  double getSagittalLeftLegAngle() const;

  /// @brief Get global right leg angle on sagittal plane
  double getSagittalRightLegAngle() const;

  /// @brief Get global left leg angle on coronal plane
  double getCoronalLeftLegAngle() const;

  /// @brief Get global right leg angle on coronal plane
  double getCoronalRightLegAngle() const;

  /// @brief Name
  std::string mName;

  /// @brief Target skeleton for control
  dart::dynamics::SkeletonPtr mSkeleton;

  /// @brief Next state. Default is myself.
  State* mNextState;

  /// @brief Terminal condition
  TerminalCondition* mTerminalCondition;

  /// @brief Started time
  double mBeginTime;

  /// @brief Stopped time
  double mEndTime;

  /// @brief Frame number
  int mFrame;

  /// @brief Elapsed time which is stopped time minus started time
  double mElapsedTime;

  /// @brief Desired joint positions
  Eigen::VectorXd mDesiredJointPositions;

  /// @brief Desired global angle of swing leg on sagittal plane
  double mDesiredGlobalSwingLegAngleOnSagittal;

  /// @brief Desired global angle of swing leg on coronal plane
  double mDesiredGlobalSwingLegAngleOnCoronal;

  /// @brief Desired global angle of pelvis on sagittal plane
  double mDesiredGlobalPelvisAngleOnSagittal;

  /// @brief Desired global angle of pelvis on coronal plane
  double mDesiredGlobalPelvisAngleOnCoronal;

  /// @brief Proportional gain for PD controller
  Eigen::VectorXd mKp;

  /// @brief Derivative gain PD controller
  Eigen::VectorXd mKd;

  /// @brief Feedback gain for com
  Eigen::VectorXd mSagittalCd;

  /// @brief Feedback gain for velocity of com
  Eigen::VectorXd mSagittalCv;

  /// @brief Feedback gain for com
  Eigen::VectorXd mCoronalCd;

  /// @brief Feedback gain for velocity of com
  Eigen::VectorXd mCoronalCv;

  /// @brief Computeed control force
  Eigen::VectorXd mTorque;

  /// @brief Joint map
  std::map<const std::string, int> mJointMap;

private:
  /// @brief Get the parent joint's position of _bodyNode
  Eigen::Vector3d _getJointPosition(dart::dynamics::BodyNode* _bodyNode) const;

  /// @brief Compute the angle between two vectors
  double _getAngleBetweenTwoVectors(
      const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2) const;

  /// @brief Update torque for torso and swing hip
  void _updateTorqueForStanceLeg();

  /// @brief Pelvis body node
  dart::dynamics::BodyNode* mPelvis;

  /// @brief Left foot body node
  dart::dynamics::BodyNode* mLeftThigh;

  /// @brief Right foot body node
  dart::dynamics::BodyNode* mRightThigh;

  /// @brief Left foot body node
  dart::dynamics::BodyNode* mStanceFoot;

  /// @brief Left foot body node
  dart::dynamics::BodyNode* mLeftFoot;

  /// @brief Right foot body node
  dart::dynamics::BodyNode* mRightFoot;

  /// @brief Index for coronal left hip
  std::size_t mCoronalLeftHip;

  /// @brief Index for coronal right hip
  std::size_t mCoronalRightHip;

  /// @brief Index for sagittal left hip
  std::size_t mSagittalLeftHip;

  /// @brief Index for sagittal right hip
  std::size_t mSagittalRightHip;

  /// @brief Desired joint positions with balance feedback
  Eigen::VectorXd mDesiredJointPositionsBalance;
};

//==============================================================================
/// @brief StateMachine for Atlas robot
class StateMachine
{
public:
  /// @brief Constructor
  explicit StateMachine(const std::string& _name);

  /// @brief Destructor
  virtual ~StateMachine();

  //------------------------------- Setting ------------------------------------
  /// @brief Set name
  void setName(const std::string& _name);

  /// @brief Get name
  const std::string& getName() const;

  /// @brief Add state
  void addState(State* _state);

  /// @brief Set initial state
  void setInitialState(State* _state);

  //------------------------------- Control ------------------------------------
  /// @brief Initiate state. This is called when the controller change the
  ///        current state machine to this.
  void begin(double _currentTime);

  /// @brief Compute control force and apply it to Atlas robot
  void computeControlForce(double _dt);

  /// @brief Finalize state. This is called when the state machine stransite
  ///        from this state to the next state.
  void end(double _currentTime);

  /// @brief Get current state
  State* getCurrentState();

  /// @brief Transite to the next state manually
  void transiteToNextState(double _currentTime);

  /// @brief Change state to _state
  void transiteTo(State* _state, double _currentTime);

  /// @brief Change state to a state whose names is _stateName
  void transiteTo(std::string& _stateName, double _currentTime);

  /// @brief Change state to a state whose index is _idx
  void transiteTo(std::size_t _idx, double _currentTime);

  /// @brief Set the verbosity
  void setVerbosity(bool verbosity);

protected:
  /// @brief Name
  std::string mName;

  /// @brief States
  std::vector<State*> mStates;

  /// @brief Current state
  State* mCurrentState;

  /// @brief Started time
  double mBeginTime;

  /// @brief Stopped time
  double mEndTime;

  /// @brief Frame number
  int mFrame;

  /// @brief Elapsed time which is stopped time minus started time
  double mElapsedTime;

private:
  /// @brief Check if this state machine contains _state
  bool _containState(const State* _state) const;

  /// @brief Check if this state machine contains a state whose name is _name
  bool _containState(const std::string& _name) const;

  /// @brief Find a state whose name is _name
  State* _findState(const std::string& _name) const;

  /// @brief Whether to print messages about the internal state
  bool mVerbosity;
};

//==============================================================================
/// @brief Implementation of Simbicon (Simple biped locomotion control) for
/// Atlas robot
///
/// Reference: http://dl.acm.org/citation.cfm?id=1276509
class Controller
{
public:
  /// @brief Constructor
  Controller(
      dart::dynamics::SkeletonPtr _atlasRobot,
      dart::constraint::ConstraintSolver* _collisionSolver);

  /// @brief Destructor
  virtual ~Controller();

  /// @brief Called before every simulation time step in MyWindow class.
  /// Compute control force and apply it to Atlas robot
  virtual void update();

  /// @brief
  dart::dynamics::SkeletonPtr getAtlasRobot();

  /// @brief Get current state machine
  StateMachine* getCurrentState();

  /// @brief Change state to _stateMachine
  void changeStateMachine(StateMachine* _stateMachine, double _currentTime);

  /// @brief Change state machine to a state machine whose names is _name
  void changeStateMachine(const std::string& _name, double _currentTime);

  /// @brief Change state machine to a state machine whose index is _idx
  void changeStateMachine(std::size_t _idx, double _currentTime);

  /// @brief Get true iff this controller is currently allowing to control the
  /// Atlas robot
  bool isAllowingControl() const;

  /// @brief Keyboard control
  void keyboard(unsigned char _key, int _x, int _y, double _currentTime);

  /// @brief Print debug information
  void printDebugInfo() const;

  /// @brief Harness the robot
  void harnessPelvis();

  /// @brief Unharness the robot
  void unharnessPelvis();

  /// @brief Harness the robot
  void harnessLeftFoot();

  /// @brief Harness the robot
  void unharnessLeftFoot();

  /// @brief Harness the robot
  void harnessRightFoot();

  /// @brief Harness the robot
  void unharnessRightFoot();

  /// @brief Reset the robot
  void resetRobot();

  /// @brief Set the verbosity
  void setVerbosity(bool verbosity);

protected:
  /// @brief Atlas robot skeleton
  dart::dynamics::SkeletonPtr mAtlasRobot;

  /// @brief Conllision detector
  dart::constraint::ConstraintSolver* mConstratinSolver;

  /// @brief List of state machines
  std::vector<StateMachine*> mStateMachines;

  /// @brief Current state machine
  StateMachine* mCurrentStateMachine;

  /// @brief Flag for pelvis harnessing
  bool mPelvisHarnessOn;

  /// @brief Flag for left foot harnessing
  bool mLeftFootHarnessOn;

  /// @brief Flag for right foot harnessing
  bool mRightFootHarnessOn;

  /// @brief Index for coronal left hip
  std::size_t mCoronalLeftHip;

  /// @brief Index for coronal right hip
  std::size_t mCoronalRightHip;

  /// @brief Index for sagittal left hip
  std::size_t mSagittalLeftHip;

  /// @brief Index for sagittal right hip
  std::size_t mSagittalRightHip;

  /// @brief Lower bound for emergency stop
  double mMinPelvisHeight;

  /// @brief Upper bound for emergency stop
  double mMaxPelvisHeight;

private:
  /// @brief Check if this controller contains _stateMachine
  bool _containStateMachine(const StateMachine* _stateMachine) const;

  /// @brief Check if this controller contains a state machine whose name is
  ///        _name
  bool _containStateMachine(const std::string& _name) const;

  /// @brief Find a state machine whose name is _name
  StateMachine* _findStateMachine(const std::string& _name) const;

  /// @brief Build state machines
  void _buildStateMachines();

  /// @brief Create standing controller
  StateMachine* _createStandingStateMachine();

  /// @brief Create standing controller
  StateMachine* _createWalkingInPlaceStateMachine();

  /// @brief Create standing controller
  StateMachine* _createWalkingStateMachine();

  /// @brief Create running controller
  StateMachine* _createRunningStateMachine();

  /// @brief Set joint damping
  void _setJointDamping();

  /// @brief Get left foot
  dart::dynamics::BodyNode* _getLeftFoot() const;

  /// @brief Get right foot
  dart::dynamics::BodyNode* _getRightFoot() const;

  /// @brief Weld joint constraint for pelvis harnessing
  dart::constraint::WeldJointConstraintPtr mWeldJointConstraintPelvis;

  /// @brief Weld joint constraint for left foot harnessing
  dart::constraint::WeldJointConstraintPtr mWeldJointConstraintLeftFoot;

  /// @brief Weld joint constraint for right foot harnessing
  dart::constraint::WeldJointConstraintPtr mWeldJointConstraintRightFoot;

  /// @brief Initial state of the robot
  dart::dynamics::Skeleton::Configuration mInitialState;

  /// @brief Whether to print messages about the internal state
  bool mVerbosity;
};

//==============================================================================
// TerminalCondition / TimerCondition / BodyContactCondition definitions
//==============================================================================

constexpr double kAxisEpsilon = 1e-10;

Eigen::Vector3d normalizedOr(
    const Eigen::Vector3d& vector, const Eigen::Vector3d& fallback)
{
  const double norm = vector.norm();
  if (norm <= kAxisEpsilon) {
    return fallback;
  }
  return vector / norm;
}

//==============================================================================
TerminalCondition::TerminalCondition(State* _state) : mState(_state)
{
  DART_ASSERT(_state != nullptr);
}

//==============================================================================
TerminalCondition::~TerminalCondition() {}

//==============================================================================
TimerCondition::TimerCondition(State* _state, double _duration)
  : TerminalCondition(_state), mDuration(_duration)
{
}

//==============================================================================
TimerCondition::~TimerCondition() {}

//==============================================================================
bool TimerCondition::isSatisfied()
{
  if (mState->getElapsedTime() > mDuration) {
    return true;
  } else {
    return false;
  }
}

//==============================================================================
BodyContactCondition::BodyContactCondition(
    State* _state,
    BodyNode* _body,
    dart::constraint::ConstraintSolver* constraintSolver)
  : TerminalCondition(_state),
    mBodyNode(_body),
    mConstraintSolver(constraintSolver)
{
  DART_ASSERT(_state != nullptr);
  DART_ASSERT(_body != nullptr);
}

//==============================================================================
BodyContactCondition::~BodyContactCondition() {}

//==============================================================================
bool BodyContactCondition::isSatisfied()
{
  SoftBodyNode* soft = dynamic_cast<SoftBodyNode*>(mBodyNode);
  if (soft) {
    for (const auto i :
         std::views::iota(std::size_t{0}, soft->getNumPointMasses())) {
      PointMass* pm = soft->getPointMass(i);
      if (pm->isColliding()) {
        return true;
      }
    }
  }

  if (!mConstraintSolver) {
    return false;
  }

  const auto& result = mConstraintSolver->getLastCollisionResult();
  return result.inCollision(mBodyNode);
}

//==============================================================================
// State definitions
//==============================================================================

//==============================================================================
State::State(SkeletonPtr _skeleton, const std::string& _name)
  : mName(_name),
    mSkeleton(_skeleton),
    mNextState(this),
    mTerminalCondition(nullptr),
    mBeginTime(0.0),
    mEndTime(0.0),
    mFrame(0),
    mElapsedTime(0.0),
    mDesiredGlobalSwingLegAngleOnSagittal(0.0),
    mDesiredGlobalSwingLegAngleOnCoronal(0.0),
    mDesiredGlobalPelvisAngleOnSagittal(0.0),
    mDesiredGlobalPelvisAngleOnCoronal(0.0)
{
  int dof = mSkeleton->getNumDofs();

  mDesiredJointPositions = Eigen::VectorXd::Zero(dof);
  mDesiredJointPositionsBalance = Eigen::VectorXd::Zero(dof);
  mKp = Eigen::VectorXd::Zero(dof);
  mKd = Eigen::VectorXd::Zero(dof);
  mSagittalCd = Eigen::VectorXd::Zero(dof);
  mSagittalCv = Eigen::VectorXd::Zero(dof);
  mCoronalCd = Eigen::VectorXd::Zero(dof);
  mCoronalCv = Eigen::VectorXd::Zero(dof);
  mTorque = Eigen::VectorXd::Zero(dof);

  for (int i = 0; i < dof; ++i) {
    mKp[i] = ATLAS_DEFAULT_KP;
    mKd[i] = ATLAS_DEFAULT_KD;
  }

  mPelvis = mSkeleton->getBodyNode("pelvis");
  mLeftFoot = mSkeleton->getBodyNode("l_foot");
  mRightFoot = mSkeleton->getBodyNode("r_foot");
  mLeftThigh = mSkeleton->getBodyNode("l_uleg");
  mRightThigh = mSkeleton->getBodyNode("r_uleg");
  mStanceFoot = nullptr;

  DART_ASSERT(mPelvis != nullptr);
  DART_ASSERT(mLeftFoot != nullptr);
  DART_ASSERT(mRightFoot != nullptr);
  DART_ASSERT(mLeftThigh != nullptr);
  DART_ASSERT(mRightThigh != nullptr);
  //  assert(mStanceFoot != nullptr);

  mCoronalLeftHip = mSkeleton->getDof("l_leg_hpx")->getIndexInSkeleton();  // 10
  mCoronalRightHip = mSkeleton->getDof("r_leg_hpx")->getIndexInSkeleton(); // 11
  mSagittalLeftHip = mSkeleton->getDof("l_leg_hpy")->getIndexInSkeleton(); // 13
  mSagittalRightHip
      = mSkeleton->getDof("r_leg_hpy")->getIndexInSkeleton(); // 14
}

//==============================================================================
State::~State()
{
  delete mTerminalCondition;
}

//==============================================================================
void State::setName(std::string& _name)
{
  mName = _name;
}

//==============================================================================
const std::string& State::getName() const
{
  return mName;
}

//==============================================================================
void State::setNextState(State* _nextState)
{
  mNextState = _nextState;
}

//==============================================================================
void State::setTerminalCondition(TerminalCondition* _condition)
{
  DART_ASSERT(_condition != nullptr);

  if (mTerminalCondition == _condition) {
    return;
  }

  delete mTerminalCondition;
  mTerminalCondition = _condition;
}

//==============================================================================
void State::begin(double _currentTime)
{
  mBeginTime = _currentTime;
  mFrame = 0;
  mElapsedTime = 0.0;
}

//==============================================================================
void State::computeControlForce(double _timestep)
{
  DART_ASSERT(mNextState != nullptr && "Next state should be set.");

  int dof = mSkeleton->getNumDofs();
  Eigen::VectorXd q = mSkeleton->getPositions();
  Eigen::VectorXd dq = mSkeleton->getVelocities();

  // Compute relative joint angles from desired global angles of the pelvis and
  // the swing leg

  // Update desired joint angles with balance feedback. Equation (1) in the
  // paper
  mDesiredJointPositionsBalance = mDesiredJointPositions
                                  + getSagittalCOMDistance() * mSagittalCd
                                  + getSagittalCOMVelocity() * mSagittalCv
                                  + getCoronalCOMDistance() * mCoronalCd
                                  + getCoronalCOMVelocity() * mCoronalCv;

  //  cout << "Sagittal D: " << getSagittalCOMDistance() << endl;
  //  cout << "Sagittal V: " << getSagittalCOMVelocity() << endl;
  //  cout << endl;
  //  cout << "Coronal D: " << getCoronalCOMDistance() << endl;
  //  cout << "Coronal V: " << getCoronalCOMVelocity() << endl;
  //  cout << endl;

  //  cout << "Sagittal left thigh : " << DART_DEGREE *
  //  getSagittalLeftLegAngle()
  //  << endl; cout << "Sagittal right thigh: " << DART_DEGREE *
  //  getSagittalRightLegAngle() << endl; cout << endl; cout << "Coronal left
  //  thigh : " << DART_DEGREE * getCoronalLeftLegAngle() << endl; cout <<
  //  "Coronal right thigh: " << DART_DEGREE * getCoronalRightLegAngle() <<
  //  endl; cout << endl;

  //  cout << "Sagittal pelvis: " << DART_DEGREE * getSagittalPelvisAngle() <<
  //  endl; cout << "Coronal pelvis: " << DART_DEGREE * getCoronalPelvisAngle()
  //  << endl; cout << endl;

  // Compute torques for all the joints except for hip (standing and swing)
  // joints. The first 6 dof is for base body force so it is set to zero.
  mTorque.head<6>() = Eigen::Vector6d::Zero();
  for (int i = 6; i < dof; ++i) {
    mTorque[i]
        = -mKp[i] * (q[i] - mDesiredJointPositionsBalance[i]) - mKd[i] * dq[i];
  }
  //  cout << "q: " << q.transpose() << endl;
  //  cout << "dq: " << dq.transpose() << endl;
  //  cout << "mKp: " << mKp.transpose() << endl;
  //  cout << "mKd: " << mKd.transpose() << endl;
  //  cout << "mTorque: " << mTorque.transpose() << endl;
  //  cout << "Theta_d: " << mDesiredJointPositionsBalance.transpose() << endl;

  // Torso and swing-hip control
  _updateTorqueForStanceLeg();

  // Apply control torque to the skeleton
  mSkeleton->setForces(mTorque);

  mElapsedTime += _timestep;
  mFrame++;
}

//==============================================================================
bool State::isTerminalConditionSatisfied() const
{
  DART_ASSERT(mTerminalCondition != nullptr && "Invalid terminal condition.");

  return mTerminalCondition->isSatisfied();
}

//==============================================================================
void State::end(double _currentTime)
{
  mEndTime = _currentTime;
}

//==============================================================================
Eigen::Vector3d State::getCOM() const
{
  return mSkeleton->getCOM();
}

//==============================================================================
Eigen::Vector3d State::getCOMVelocity() const
{
  return mSkeleton->getCOMLinearVelocity();
}

//==============================================================================
Eigen::Isometry3d State::getCOMFrame() const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // Z-axis
  const Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ();

  // X-axis
  Eigen::Vector3d pelvisXAxis = mPelvis->getTransform().linear().col(0);
  const double mag = zAxis.dot(pelvisXAxis);
  pelvisXAxis -= mag * zAxis;
  const Eigen::Vector3d xAxis
      = normalizedOr(pelvisXAxis, Eigen::Vector3d::UnitX());

  // Y-axis
  const Eigen::Vector3d yAxis
      = normalizedOr(zAxis.cross(xAxis), Eigen::Vector3d::UnitY());

  T.translation() = getCOM();

  T.linear().col(0) = xAxis;
  T.linear().col(1) = yAxis;
  T.linear().col(2) = zAxis;

  return T;
}

//==============================================================================
double State::getSagittalCOMDistance()
{
  Eigen::Vector3d xAxis = getCOMFrame().linear().col(0); // x-axis
  Eigen::Vector3d d = getCOM() - getStanceAnklePosition();

  return d.dot(xAxis);
}

//==============================================================================
double State::getSagittalCOMVelocity()
{
  Eigen::Vector3d xAxis = getCOMFrame().linear().col(0); // x-axis
  Eigen::Vector3d v = getCOMVelocity();

  return v.dot(xAxis);
}

//==============================================================================
double State::getCoronalCOMDistance()
{
  Eigen::Vector3d yAxis = getCOMFrame().linear().col(1); // y-axis
  Eigen::Vector3d d = getCOM() - getStanceAnklePosition();

  return d.dot(yAxis);
}

//==============================================================================
double State::getCoronalCOMVelocity()
{
  Eigen::Vector3d yAxis = getCOMFrame().linear().col(1); // y-axis
  Eigen::Vector3d v = getCOMVelocity();

  return v.dot(yAxis);
}

//==============================================================================
Eigen::Vector3d State::getStanceAnklePosition() const
{
  if (mStanceFoot == nullptr) {
    return getCOM();
  } else {
    return _getJointPosition(mStanceFoot);
  }
}

//==============================================================================
Eigen::Vector3d State::getLeftAnklePosition() const
{
  return _getJointPosition(mLeftFoot);
}

//==============================================================================
Eigen::Vector3d State::getRightAnklePosition() const
{
  return _getJointPosition(mRightFoot);
}

//==============================================================================
double State::getSagittalPelvisAngle() const
{
  Eigen::Matrix3d comR = getCOMFrame().linear();
  Eigen::Vector3d comZ = comR.col(2);

  Eigen::Vector3d pelvisZ = mPelvis->getTransform().linear().col(2);
  Eigen::Vector3d projPelvisZ = (comR.transpose() * pelvisZ);
  projPelvisZ[1] = 0.0;
  projPelvisZ = normalizedOr(projPelvisZ, Eigen::Vector3d::UnitZ());
  double angle = _getAngleBetweenTwoVectors(projPelvisZ, comZ);

  Eigen::Vector3d cross = comZ.cross(projPelvisZ);

  if (cross[1] > 0.0) {
    return angle;
  } else {
    return -angle;
  }
}

//==============================================================================
double State::getCoronalPelvisAngle() const
{
  Eigen::Matrix3d comR = getCOMFrame().linear();
  Eigen::Vector3d comZ = comR.col(2);
  Eigen::Vector3d pelvisZ = mPelvis->getTransform().linear().col(2);
  Eigen::Vector3d projPelvisZ = (comR.transpose() * pelvisZ);
  projPelvisZ[0] = 0.0;
  projPelvisZ = normalizedOr(projPelvisZ, Eigen::Vector3d::UnitZ());
  double angle = _getAngleBetweenTwoVectors(projPelvisZ, comZ);

  Eigen::Vector3d cross = comZ.cross(projPelvisZ);

  if (cross[0] > 0.0) {
    return angle;
  } else {
    return -angle;
  }
}

//==============================================================================
double State::getSagittalLeftLegAngle() const
{
  Eigen::Matrix3d comR = getCOMFrame().linear();
  Eigen::Vector3d comZ = comR.col(2);
  Eigen::Vector3d thighAxisZ = mLeftThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = (comR.transpose() * thighAxisZ);
  projThighAZ[1] = 0.0;
  projThighAZ = normalizedOr(projThighAZ, Eigen::Vector3d::UnitZ());
  double angle = _getAngleBetweenTwoVectors(projThighAZ, comZ);

  Eigen::Vector3d cross = comZ.cross(projThighAZ);

  if (cross[1] > 0.0) {
    return angle;
  } else {
    return -angle;
  }
}

//==============================================================================
double State::getSagittalRightLegAngle() const
{
  Eigen::Matrix3d comR = getCOMFrame().linear();
  Eigen::Vector3d comZ = comR.col(2);
  Eigen::Vector3d thighAxisZ = mRightThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = (comR.transpose() * thighAxisZ);
  projThighAZ[1] = 0.0;
  projThighAZ = normalizedOr(projThighAZ, Eigen::Vector3d::UnitZ());
  double angle = _getAngleBetweenTwoVectors(projThighAZ, comZ);

  Eigen::Vector3d cross = comZ.cross(projThighAZ);

  if (cross[1] > 0.0) {
    return angle;
  } else {
    return -angle;
  }
}

//==============================================================================
double State::getCoronalLeftLegAngle() const
{
  Eigen::Matrix3d comR = getCOMFrame().linear();
  Eigen::Vector3d comZ = comR.col(2);
  Eigen::Vector3d thighAxisZ = mLeftThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = (comR.transpose() * thighAxisZ);
  projThighAZ[0] = 0.0;
  projThighAZ = normalizedOr(projThighAZ, Eigen::Vector3d::UnitZ());
  double angle = _getAngleBetweenTwoVectors(projThighAZ, comZ);

  Eigen::Vector3d cross = comZ.cross(projThighAZ);

  if (cross[0] > 0.0) {
    return angle;
  } else {
    return -angle;
  }
}

//==============================================================================
double State::getCoronalRightLegAngle() const
{
  Eigen::Matrix3d comR = getCOMFrame().linear();
  Eigen::Vector3d comZ = comR.col(2);
  Eigen::Vector3d thighAxisZ = mRightThigh->getTransform().linear().col(2);
  Eigen::Vector3d projThighAZ = (comR.transpose() * thighAxisZ);
  projThighAZ[0] = 0.0;
  projThighAZ = normalizedOr(projThighAZ, Eigen::Vector3d::UnitZ());
  double angle = _getAngleBetweenTwoVectors(projThighAZ, comZ);

  Eigen::Vector3d cross = comZ.cross(projThighAZ);

  if (cross[0] > 0.0) {
    return angle;
  } else {
    return -angle;
  }
}

//==============================================================================
Eigen::Vector3d State::_getJointPosition(BodyNode* _bodyNode) const
{
  Joint* parentJoint = _bodyNode->getParentJoint();
  Eigen::Vector3d localJointPosition
      = parentJoint->getTransformFromChildBodyNode().translation();
  return _bodyNode->getTransform() * localJointPosition;
}

//==============================================================================
double State::_getAngleBetweenTwoVectors(
    const Eigen::Vector3d& _v1, const Eigen::Vector3d& _v2) const
{
  const double denominator = _v1.norm() * _v2.norm();
  if (denominator <= kAxisEpsilon) {
    return 0.0;
  }

  return std::acos(std::clamp(_v1.dot(_v2) / denominator, -1.0, 1.0));
}

//==============================================================================
void State::_updateTorqueForStanceLeg()
{
  // Stance leg is left leg
  if (mStanceFoot == mLeftFoot) {
    //    std::cout << "Sagittal Pelvis Angle: " << DART_DEGREE *
    //    getSagittalPelvisAngle() << std::endl;

    // Torso control on sagittal plane
    double pelvisSagittalAngle = getSagittalPelvisAngle();
    double tauTorsoSagittal
        = -5000.0 * (pelvisSagittalAngle + mDesiredGlobalPelvisAngleOnSagittal)
          - 1.0 * (0);
    mTorque[mSagittalLeftHip] = tauTorsoSagittal - mTorque[mSagittalRightHip];

    //    cout << "Torque[mSagittalLeftHip]     : " << mTorque[mSagittalLeftHip]
    //    << endl; cout << "Torque[mSagittalRightHip]     : " <<
    //    mTorque[mSagittalRightHip] << endl; cout << "tauTorsoSagittal: " <<
    //    tauTorsoSagittal << endl; cout << endl;

    // Torso control on coronal plane
    double pelvisCoronalAngle = getCoronalPelvisAngle();
    double tauTorsoCoronal
        = -5000.0 * (pelvisCoronalAngle - mDesiredGlobalPelvisAngleOnCoronal)
          - 1.0 * (0);
    mTorque[mCoronalLeftHip] = -tauTorsoCoronal - mTorque[mCoronalRightHip];

    //    cout << "Torque[mCoronalLeftHip]     : " << mTorque[mCoronalLeftHip]
    //    << endl; cout << "Torque[mCoronalRightHip]     : " <<
    //    mTorque[mCoronalRightHip] << endl; cout << "tauTorsoCoronal: " <<
    //    tauTorsoCoronal << endl; cout << endl;

    //    cout << "Stance foot: Left foot" << endl;
  }
  // Stance leg is right leg
  else if (mStanceFoot == mRightFoot) {
    //    cout << "Stance foot: Right foot" << endl;

    // Torso control on sagittal plane
    double pelvisSagittalAngle = getSagittalPelvisAngle();
    double tauTorsoSagittal
        = -5000.0 * (pelvisSagittalAngle + mDesiredGlobalPelvisAngleOnSagittal)
          - 1.0 * (0);
    mTorque[mSagittalRightHip] = tauTorsoSagittal - mTorque[mSagittalLeftHip];

    //    cout << "Torque[mSagittalLeftHip]     : " << mTorque[mSagittalLeftHip]
    //    << endl; cout << "Torque[mSagittalRightHip]    : " <<
    //    mTorque[mSagittalRightHip] << endl; cout << "tauTorsoSagittal: " <<
    //    tauTorsoSagittal << endl; cout << endl;

    // Torso control on coronal plane
    double pelvisCoronalAngle = getCoronalPelvisAngle();
    double tauTorsoCoronal
        = -5000.0 * (pelvisCoronalAngle - mDesiredGlobalPelvisAngleOnCoronal)
          - 1.0 * (0);
    mTorque[mCoronalRightHip] = -tauTorsoCoronal - mTorque[mCoronalLeftHip];

    //    cout << "Torque[mCoronalLeftHip]     : " << mTorque[mCoronalLeftHip]
    //    << endl; cout << "Torque[mCoronalRightHip]     : " <<
    //    mTorque[mCoronalRightHip] << endl; cout << "tauTorsoCoronal: " <<
    //    tauTorsoCoronal << endl; cout << endl;
  } else {
    // No foot is toching the ground
  }
}

//==============================================================================
State* State::getNextState() const
{
  return mNextState;
}

//==============================================================================
double State::getElapsedTime() const
{
  return mElapsedTime;
}

//==============================================================================
void State::setDesiredJointPosition(const std::string& _jointName, double _val)
{
  std::size_t index = mSkeleton->getDof(_jointName)->getIndexInSkeleton();
  mDesiredJointPositions[index] = _val;
}

//==============================================================================
double State::getDesiredJointPosition(int _idx) const
{
  DART_ASSERT(
      0 <= _idx && _idx <= mDesiredJointPositions.size()
      && "Invalid joint index.");

  return mDesiredJointPositions[_idx];
}

//==============================================================================
double State::getDesiredJointPosition(const std::string& _jointName) const
{
  // TODO(JS)
  NOT_YET(State::getDesiredJointPosition());

  DART_ASSERT(mSkeleton->getJoint(_jointName) != nullptr);

  return mDesiredJointPositions[mJointMap.at(_jointName)];
}

//==============================================================================
void State::setDesiredSwingLegGlobalAngleOnSagittal(double _val)
{
  mDesiredGlobalSwingLegAngleOnSagittal = _val;
}

//==============================================================================
void State::setDesiredSwingLegGlobalAngleOnCoronal(double _val)
{
  mDesiredGlobalSwingLegAngleOnCoronal = _val;
}

//==============================================================================
void State::setDesiredPelvisGlobalAngleOnSagittal(double _val)
{
  mDesiredGlobalPelvisAngleOnSagittal = _val;
}

//==============================================================================
void State::setDesiredPelvisGlobalAngleOnCoronal(double _val)
{
  mDesiredGlobalPelvisAngleOnCoronal = _val;
}

//==============================================================================
void State::setProportionalGain(int _idx, double _val)
{
  DART_ASSERT(0 <= _idx && _idx <= mKp.size() && "Invalid joint index.");

  mKd[_idx] = _val;
}

//==============================================================================
void State::setProportionalGain(
    const std::string& /*_jointName*/, double /*_val*/)
{
  // TODO(JS)
  NOT_YET(State::setProportionalGain());
}

//==============================================================================
double State::getProportionalGain(int _idx) const
{
  DART_ASSERT(0 <= _idx && _idx <= mKp.size() && "Invalid joint index.");

  return mKp[_idx];
}

//==============================================================================
double State::getProportionalGain(const std::string& _jointName) const
{
  // TODO(JS)
  NOT_YET(State::getProportionalGain());

  DART_ASSERT(mSkeleton->getJoint(_jointName) != nullptr);

  return mKp[mJointMap.at(_jointName)];
}

//==============================================================================
void State::setDerivativeGain(int _idx, double _val)
{
  DART_ASSERT(0 <= _idx && _idx <= mKd.size() && "Invalid joint index.");

  mKd[_idx] = _val;
}

//==============================================================================
void State::setDerivativeGain(
    const std::string& /*_jointName*/, double /*_val*/)
{
  // TODO(JS)
  NOT_YET(State::setDerivativeGain());
}

//==============================================================================
double State::getDerivativeGain(int _idx) const
{
  DART_ASSERT(0 <= _idx && _idx <= mKd.size() && "Invalid joint index.");

  return mKd[_idx];
}

//==============================================================================
// double State::getDerivativeGain(const string& _jointName) const
//{
//  // TODO(JS)
//  NOT_YET(State::getDerivativeGain());
//}

//==============================================================================
void State::setFeedbackSagittalCOMDistance(std::size_t _index, double _val)
{
  DART_ASSERT(
      std::cmp_less_equal(_index, mSagittalCd.size()) && "Invalid index.");

  mSagittalCd[_index] = _val;
}

//==============================================================================
void State::setFeedbackSagittalCOMVelocity(std::size_t _index, double _val)
{
  DART_ASSERT(
      std::cmp_less_equal(_index, mSagittalCv.size()) && "Invalid index.");

  mSagittalCv[_index] = _val;
}

//==============================================================================
void State::setFeedbackCoronalCOMDistance(std::size_t _index, double _val)
{
  DART_ASSERT(
      std::cmp_less_equal(_index, mCoronalCd.size()) && "Invalid index.");

  mCoronalCd[_index] = _val;
}

//==============================================================================
void State::setFeedbackCoronalCOMVelocity(std::size_t _index, double _val)
{
  DART_ASSERT(
      std::cmp_less_equal(_index, mCoronalCv.size()) && "Invalid index.");

  mCoronalCv[_index] = _val;
}

//==============================================================================
void State::setStanceFootToLeftFoot()
{
  mStanceFoot = mLeftFoot;
}

//==============================================================================
void State::setStanceFootToRightFoot()
{
  mStanceFoot = mRightFoot;
}

//==============================================================================
// StateMachine definitions
//==============================================================================

//==============================================================================
StateMachine::StateMachine(const std::string& _name)
  : mName(_name),
    mCurrentState(nullptr),
    mBeginTime(0.0),
    mEndTime(0.0),
    mFrame(0),
    mElapsedTime(0.0),
    mVerbosity(false)
{
  // Do nothing
}

//==============================================================================
StateMachine::~StateMachine()
{
  std::ranges::for_each(mStates, [](State* state) { delete state; });
}

//==============================================================================
void StateMachine::setName(const std::string& _name)
{
  mName = _name;
}

//==============================================================================
const std::string& StateMachine::getName() const
{
  return mName;
}

//==============================================================================
void StateMachine::addState(State* _state)
{
  DART_ASSERT(_state != nullptr && "Invalid state");
  DART_ASSERT(!_containState(_state) && "_state shouldn't be in mStates");

  mStates.push_back(_state);
}

//==============================================================================
void StateMachine::setInitialState(State* _state)
{
  DART_ASSERT(_state != nullptr);
  DART_ASSERT(_containState(_state));

  mCurrentState = _state;
}

//==============================================================================
void StateMachine::begin(double _currentTime)
{
  //  DART_INFO("StateMachine [{}]: begin().", getName());

  mBeginTime = _currentTime;
  mFrame = 0;
  mElapsedTime = 0.0;
}

//==============================================================================
void StateMachine::computeControlForce(double _dt)
{
  DART_ASSERT(mCurrentState != nullptr && "Invalid current state.");

  // Check transition is needed from current state
  if (mCurrentState->isTerminalConditionSatisfied()) {
    transiteTo(mCurrentState->getNextState(), mBeginTime + mElapsedTime);
  }

  // Update control force
  mCurrentState->computeControlForce(_dt);

  mElapsedTime += _dt;
  mFrame++;
}

//==============================================================================
void StateMachine::end(double _currentTime)
{
  mEndTime = _currentTime;

  //  DART_INFO("StateMachine [{}]: end().", getName());
}

//==============================================================================
State* StateMachine::getCurrentState()
{
  return mCurrentState;
}

//==============================================================================
void StateMachine::transiteToNextState(double _currentTime)
{
  transiteTo(mCurrentState->getNextState(), _currentTime);
}

//==============================================================================
void StateMachine::transiteTo(State* _state, double _currentTime)
{
  DART_ASSERT(_containState(_state) && "_state should be in mStates");

  std::string prevStateName = mCurrentState->getName();
  std::string nextStateName = _state->getName();

  // Finish current state
  mCurrentState->end(_currentTime);

  // Transite to _state
  mCurrentState = _state;
  mCurrentState->begin(_currentTime);

  DART_INFO_IF(
      mVerbosity, "Transition: [{}] --> [{}].", prevStateName, nextStateName);
}

//==============================================================================
void StateMachine::transiteTo(std::string& _stateName, double _currentTime)
{
  // _state should be in mStates
  State* state = _findState(_stateName);

  DART_ASSERT(state != nullptr && "Invalid state.");

  transiteTo(state, _currentTime);
}

//==============================================================================
void StateMachine::transiteTo(std::size_t _idx, double _currentTime)
{
  DART_ASSERT(_idx <= mStates.size() && "Invalid index of State.");

  transiteTo(mStates[_idx], _currentTime);
}

//==============================================================================
void StateMachine::setVerbosity(bool verbosity)
{
  mVerbosity = verbosity;
}

//==============================================================================
bool StateMachine::_containState(const State* _state) const
{
  return std::ranges::find(mStates, _state) != mStates.end();
}

//==============================================================================
bool StateMachine::_containState(const std::string& _name) const
{
  return _containState(_findState(_name));
}

//==============================================================================
State* StateMachine::_findState(const std::string& _name) const
{
  const auto it = std::ranges::find_if(mStates, [&_name](const State* state) {
    return state->getName() == _name;
  });

  return it == mStates.end() ? nullptr : *it;
}

//==============================================================================
// Controller definitions
//==============================================================================

//==============================================================================
Controller::Controller(
    SkeletonPtr _atlasRobot, ConstraintSolver* _collisionSolver)
  : mAtlasRobot(_atlasRobot),
    mConstratinSolver(_collisionSolver),
    mCurrentStateMachine(nullptr),
    mPelvisHarnessOn(false),
    mLeftFootHarnessOn(false),
    mRightFootHarnessOn(false),
    mMinPelvisHeight(-0.70),
    mMaxPelvisHeight(0.30),
    mWeldJointConstraintPelvis(nullptr),
    mWeldJointConstraintLeftFoot(nullptr),
    mWeldJointConstraintRightFoot(nullptr),
    mVerbosity(false)
{
  mCoronalLeftHip = mAtlasRobot->getDof("l_leg_hpx")->getIndexInSkeleton();
  mCoronalRightHip = mAtlasRobot->getDof("r_leg_hpx")->getIndexInSkeleton();
  mSagittalLeftHip = mAtlasRobot->getDof("l_leg_hpy")->getIndexInSkeleton();
  mSagittalRightHip = mAtlasRobot->getDof("r_leg_hpy")->getIndexInSkeleton();

  _buildStateMachines();
  _setJointDamping();

  //  harnessPelvis();
  //  harnessLeftFoot();
  //  harnessRightFoot();

  mInitialState = mAtlasRobot->getConfiguration(
      Skeleton::CONFIG_POSITIONS | Skeleton::CONFIG_VELOCITIES);
}

//==============================================================================
Controller::~Controller()
{
  for (std::vector<StateMachine*>::iterator it = mStateMachines.begin();
       it != mStateMachines.end();
       ++it) {
    delete *it;
  }
}

//==============================================================================
void Controller::update()
{
  if (isAllowingControl()) {
    // Compute control force
    mCurrentStateMachine->computeControlForce(mAtlasRobot->getTimeStep());
  }
}

//==============================================================================
SkeletonPtr Controller::getAtlasRobot()
{
  return mAtlasRobot;
}

//==============================================================================
StateMachine* Controller::getCurrentState()
{
  return mCurrentStateMachine;
}

//==============================================================================
void Controller::changeStateMachine(
    StateMachine* _stateMachine, double _currentTime)
{
  DART_ASSERT(
      _containStateMachine(_stateMachine)
      && "_stateMachine should be in mStateMachines");

  if (mCurrentStateMachine == _stateMachine) {
    return;
  }

  std::string prevName = mCurrentStateMachine->getName();
  std::string nextName = _stateMachine->getName();

  // Finish current state
  mCurrentStateMachine->end(_currentTime);

  // Transite to _state
  mCurrentStateMachine = _stateMachine;
  mCurrentStateMachine->begin(_currentTime);

  DART_INFO_IF(
      mVerbosity,
      "State machine transition: from [{}] to [{}].",
      prevName,
      nextName);
}

//==============================================================================
void Controller::changeStateMachine(
    const std::string& _name, double _currentTime)
{
  // _state should be in mStates
  StateMachine* stateMachine = _findStateMachine(_name);

  DART_ASSERT(stateMachine != nullptr && "Invalid state machine.");

  changeStateMachine(stateMachine, _currentTime);
}

//==============================================================================
void Controller::changeStateMachine(std::size_t _idx, double _currentTime)
{
  DART_ASSERT(
      _idx <= mStateMachines.size() && "Invalid index of StateMachine.");

  changeStateMachine(mStateMachines[_idx], _currentTime);
}

//==============================================================================
bool Controller::isAllowingControl() const
{
  auto pelvis = mAtlasRobot->getBodyNode("pelvis");
  const Eigen::Isometry3d tf = pelvis->getTransform();
  const Eigen::Vector3d pos = tf.translation();
  const auto z = pos[2];

  if (z < mMinPelvisHeight || mMaxPelvisHeight < z) {
    return false;
  } else {
    return true;
  }
}

//==============================================================================
void Controller::keyboard(
    unsigned char _key, int /*_x*/, int /*_y*/, double _currentTime)
{
  switch (_key) {
    case 'h': // Harness pelvis toggle
      if (mPelvisHarnessOn) {
        unharnessPelvis();
      } else {
        harnessPelvis();
      }
      break;
    case 'j': // Harness left foot toggle
      if (mLeftFootHarnessOn) {
        unharnessLeftFoot();
      } else {
        harnessLeftFoot();
      }
      break;
    case 'k': // Harness right foot toggle
      if (mRightFootHarnessOn) {
        unharnessRightFoot();
      } else {
        harnessRightFoot();
      }
      break;
    case 'r': // Reset robot
      resetRobot();
      break;
    case 'n': // Transite to the next state manually
      mCurrentStateMachine->transiteToNextState(_currentTime);
      break;
    case '1': // Standing controller
      changeStateMachine("standing", _currentTime);
      break;
    case '2': // Walking in place controller
      changeStateMachine("walking in place", _currentTime);
      break;
    case '3':
      changeStateMachine("walking", _currentTime);
      break;
    case '4':
      changeStateMachine("running", _currentTime);
      break;

    default:
      break;
  }
}

//==============================================================================
void Controller::printDebugInfo() const
{
  std::cout << "[ATLAS Robot]" << std::endl
            << " NUM NODES : " << mAtlasRobot->getNumBodyNodes() << std::endl
            << " NUM DOF   : " << mAtlasRobot->getNumDofs() << std::endl
            << " NUM JOINTS: " << mAtlasRobot->getNumBodyNodes() << std::endl;

  for (const auto i :
       std::views::iota(std::size_t{0}, mAtlasRobot->getNumBodyNodes())) {
    Joint* joint = mAtlasRobot->getJoint(i);
    BodyNode* body = mAtlasRobot->getBodyNode(i);
    BodyNode* parentBody = mAtlasRobot->getBodyNode(i)->getParentBodyNode();

    std::cout << "  Joint [" << i << "]: " << joint->getName() << " ("
              << joint->getNumDofs() << ")" << std::endl;
    if (parentBody != nullptr) {
      std::cout << "    Parent body: " << parentBody->getName() << std::endl;
    }

    std::cout << "    Child body : " << body->getName() << std::endl;
  }
}

//==============================================================================
void Controller::harnessPelvis()
{
  if (mPelvisHarnessOn) {
    return;
  }

  BodyNode* bd = mAtlasRobot->getBodyNode("pelvis");
  mWeldJointConstraintPelvis = std::make_shared<WeldJointConstraint>(bd);
  mConstratinSolver->addConstraint(mWeldJointConstraintPelvis);
  mPelvisHarnessOn = true;

  DART_INFO_IF(mVerbosity, "Pelvis is harnessed.");
}

//==============================================================================
void Controller::unharnessPelvis()
{
  if (!mPelvisHarnessOn) {
    return;
  }

  mConstratinSolver->removeConstraint(mWeldJointConstraintPelvis);
  mPelvisHarnessOn = false;

  DART_INFO_IF(mVerbosity, "Pelvis is unharnessed.");
}

//==============================================================================
void Controller::harnessLeftFoot()
{
  if (mLeftFootHarnessOn) {
    return;
  }

  BodyNode* bd = mAtlasRobot->getBodyNode("l_foot");
  mWeldJointConstraintLeftFoot = std::make_shared<WeldJointConstraint>(bd);
  mLeftFootHarnessOn = true;

  DART_INFO_IF(mVerbosity, "Left foot is harnessed.");
}

//==============================================================================
void Controller::unharnessLeftFoot()
{
  if (!mLeftFootHarnessOn) {
    return;
  }

  mConstratinSolver->removeConstraint(mWeldJointConstraintLeftFoot);
  mLeftFootHarnessOn = false;

  DART_INFO_IF(mVerbosity, "Left foot is unharnessed.");
}

//==============================================================================
void Controller::harnessRightFoot()
{
  if (mRightFootHarnessOn) {
    return;
  }

  BodyNode* bd = mAtlasRobot->getBodyNode("r_foot");
  mWeldJointConstraintRightFoot = std::make_shared<WeldJointConstraint>(bd);
  mRightFootHarnessOn = true;

  DART_INFO_IF(mVerbosity, "Right foot is harnessed.");
}

//==============================================================================
void Controller::unharnessRightFoot()
{
  if (!mRightFootHarnessOn) {
    return;
  }

  mConstratinSolver->removeConstraint(mWeldJointConstraintRightFoot);
  mRightFootHarnessOn = false;

  DART_INFO_IF(mVerbosity, "Right foot is unharnessed.");
}

//==============================================================================
void Controller::resetRobot()
{
  mAtlasRobot->setConfiguration(mInitialState);

  DART_INFO_IF(mVerbosity, "Robot is reset.");
}

//==============================================================================
void Controller::setVerbosity(bool verbosity)
{
  mVerbosity = verbosity;
}

//==============================================================================
void Controller::_buildStateMachines()
{
  // Standing controller
  mStateMachines.push_back(_createStandingStateMachine());

  // Walking in place controller
  mStateMachines.push_back(_createWalkingInPlaceStateMachine());

  // Walking controller
  mStateMachines.push_back(_createWalkingStateMachine());

  // Walking controller
  mStateMachines.push_back(_createRunningStateMachine());

  // Set initial (default) controller
  mCurrentStateMachine = mStateMachines[1]; // Standing controller

  // Begin the default controller
  mCurrentStateMachine->begin(0.0);
}

//==============================================================================
StateMachine* Controller::_createStandingStateMachine()
{
  using namespace dart::math::suffixes;

  StateMachine* standing = new StateMachine("standing");

  State* standingState0 = new State(mAtlasRobot, "0");

  TerminalCondition* tcStanding0 = new TimerCondition(standingState0, 0.3);

  standingState0->setTerminalCondition(tcStanding0);

  standingState0->setNextState(standingState0);

  standingState0->setDesiredJointPosition(
      "back_bky", 15.00_deg); // angle b/w pelvis and torso
  standingState0->setDesiredJointPosition("l_leg_hpy", -10.00_deg);
  standingState0->setDesiredJointPosition("r_leg_hpy", -10.00_deg);
  standingState0->setDesiredJointPosition("l_leg_kny", 30.00_deg); // left knee
  standingState0->setDesiredJointPosition("r_leg_kny", 30.00_deg); // right knee
  standingState0->setDesiredJointPosition(
      "l_leg_aky", -16.80_deg); // left ankle
  standingState0->setDesiredJointPosition(
      "r_leg_aky", -16.80_deg); // right ankle

  standingState0->setDesiredJointPosition(
      "l_arm_shx", -90.0_deg); // right ankle
  standingState0->setDesiredJointPosition(
      "r_arm_shx", +90.0_deg); // right ankle

  standing->addState(standingState0);

  standing->setInitialState(standingState0);

  return standing;
}

//==============================================================================
StateMachine* Controller::_createWalkingInPlaceStateMachine()
{
  using namespace dart::math::suffixes;

  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = -4.75_deg; // angle b/w pelvis and torso

  const double swh02 = 0.50;  // swing hip
  const double swk02 = -1.10; // swing knee
  const double swa02 = 0.60;  // swing angle
  const double stk02 = -0.05; // stance knee
  const double sta02 = 0.00;  // stance ankle

  const double swh13 = -0.10; // swing hip
  const double swk13 = -0.05; // swing knee
  const double swa13 = 0.15;  // swing angle
  const double stk13 = -0.10; // stance knee
  const double sta13 = 0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking in place");

  State* state0 = new State(mAtlasRobot, "0");
  State* state1 = new State(mAtlasRobot, "1");
  State* state2 = new State(mAtlasRobot, "2");
  State* state3 = new State(mAtlasRobot, "3");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1
      = new BodyContactCondition(state1, _getRightFoot(), mConstratinSolver);
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3
      = new BodyContactCondition(state3, _getLeftFoot(), mConstratinSolver);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state0->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition("r_leg_hpy", -swh02); // right hip
  state0->setDesiredJointPosition("r_leg_kny", -swk02); // right knee
  state0->setDesiredJointPosition("r_leg_aky", -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition("l_leg_kny", -stk02); // left knee
  state0->setDesiredJointPosition("l_leg_aky", -sta02); // left ankle
  //---- arm
  state0->setDesiredJointPosition("l_arm_shz", -20.00_deg); // left arm
  state0->setDesiredJointPosition("r_arm_shz", +10.00_deg); // right arm
  state0->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state0->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state0->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state0->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state0->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state0->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state0->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition("l_leg_hpy", -swh13); // left hip
  state1->setDesiredJointPosition("l_leg_kny", -swk13); // left knee
  state1->setDesiredJointPosition("l_leg_aky", -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition("r_leg_kny", -stk13); // right knee
  state1->setDesiredJointPosition("r_leg_aky", -sta13); // right ankle
  //---- arm
  state1->setDesiredJointPosition("l_arm_shz", +10.00_deg); // left arm
  state1->setDesiredJointPosition("r_arm_shz", -20.00_deg); // right arm
  state1->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state1->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state1->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state1->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state1->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state1->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state1->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition("l_leg_hpy", -swh02); // left hip
  state2->setDesiredJointPosition("l_leg_kny", -swk02); // left knee
  state2->setDesiredJointPosition("l_leg_aky", -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition("r_leg_kny", -stk02); // right knee
  state2->setDesiredJointPosition("r_leg_aky", -sta02); // right ankle
  //---- arm
  state2->setDesiredJointPosition("l_arm_shz", +10.00_deg); // left arm
  state2->setDesiredJointPosition("r_arm_shz", -20.00_deg); // right arm
  state2->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state2->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state2->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state2->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state2->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state2->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state2->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state2->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state2->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state2->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition("r_leg_hpy", -swh13); // right hip
  state3->setDesiredJointPosition("r_leg_kny", -swk13); // right knee
  state3->setDesiredJointPosition("r_leg_aky", -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition("l_leg_kny", -stk13); // left knee
  state3->setDesiredJointPosition("l_leg_aky", -sta13); // left ankle
  //---- arm
  state3->setDesiredJointPosition("l_arm_shz", -20.00_deg); // left arm
  state3->setDesiredJointPosition("r_arm_shz", +10.00_deg); // right arm
  state3->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state3->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state3->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state3->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state3->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state3->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state3->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state3->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state3->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state3->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);

  sm->setInitialState(state1);

  return sm;
}

//==============================================================================
StateMachine* Controller::_createWalkingStateMachine()
{
  using namespace dart::math::suffixes;

  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = -10.0_deg; // angle b/w pelvis and torso

  const double swh02 = 0.50;  // swing hip
  const double swk02 = -1.10; // swing knee
  const double swa02 = 0.60;  // swing angle
  const double stk02 = -0.05; // stance knee
  const double sta02 = 0.00;  // stance ankle

  const double swh13 = -0.10; // swing hip
  const double swk13 = -0.05; // swing knee
  const double swa13 = 0.15;  // swing angle
  const double stk13 = -0.10; // stance knee
  const double sta13 = 0.00;  // stance ankle

  StateMachine* sm = new StateMachine("walking");

  State* state0 = new State(mAtlasRobot, "0");
  State* state1 = new State(mAtlasRobot, "1");
  State* state2 = new State(mAtlasRobot, "2");
  State* state3 = new State(mAtlasRobot, "3");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.3);
  TerminalCondition* cond1
      = new BodyContactCondition(state1, _getRightFoot(), mConstratinSolver);
  TerminalCondition* cond2 = new TimerCondition(state2, 0.3);
  TerminalCondition* cond3
      = new BodyContactCondition(state3, _getLeftFoot(), mConstratinSolver);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);
  state2->setTerminalCondition(cond2);
  state3->setTerminalCondition(cond3);

  state0->setNextState(state1);
  state1->setNextState(state2);
  state2->setNextState(state3);
  state3->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToLeftFoot();
  state2->setStanceFootToRightFoot();
  state3->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state0->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state2->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state3->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition("r_leg_hpy", -swh02); // right hip
  state0->setDesiredJointPosition("r_leg_kny", -swk02); // right knee
  state0->setDesiredJointPosition("r_leg_aky", -swa02); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition("l_leg_kny", -stk02); // left knee
  state0->setDesiredJointPosition("l_leg_aky", -sta02); // left ankle
  //---- arm
  state0->setDesiredJointPosition("l_arm_shz", -20.00_deg); // left arm
  state0->setDesiredJointPosition("r_arm_shz", +10.00_deg); // right arm
  state0->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state0->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state0->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state0->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state0->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state0->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state0->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 1
  //---- pelvis
  state1->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition("l_leg_hpy", -swh13); // left hip
  state1->setDesiredJointPosition("l_leg_kny", -swk13); // left knee
  state1->setDesiredJointPosition("l_leg_aky", -swa13); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition("r_leg_kny", -stk13); // right knee
  state1->setDesiredJointPosition("r_leg_aky", -sta13); // right ankle
  //---- arm
  state1->setDesiredJointPosition("l_arm_shz", +10.00_deg); // left arm
  state1->setDesiredJointPosition("r_arm_shz", -20.00_deg); // right arm
  state1->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state1->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state1->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state1->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state1->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state1->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state1->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 2
  //---- pelvis
  state2->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state2->setDesiredJointPosition("l_leg_hpy", -swh02); // left hip
  state2->setDesiredJointPosition("l_leg_kny", -swk02); // left knee
  state2->setDesiredJointPosition("l_leg_aky", -swa02); // left ankle
  //---- stance leg
  state2->setDesiredJointPosition("r_leg_kny", -stk02); // right knee
  state2->setDesiredJointPosition("r_leg_aky", -sta02); // right ankle
  //---- arm
  state2->setDesiredJointPosition("l_arm_shz", +10.00_deg); // left arm
  state2->setDesiredJointPosition("r_arm_shz", -20.00_deg); // right arm
  state2->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state2->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state2->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state2->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state2->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state2->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state2->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state2->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state2->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state2->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 3
  //---- pelvis
  state3->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state3->setDesiredJointPosition("r_leg_hpy", -swh13); // right hip
  state3->setDesiredJointPosition("r_leg_kny", -swk13); // right knee
  state3->setDesiredJointPosition("r_leg_aky", -swa13); // right ankle
  //---- stance leg
  state3->setDesiredJointPosition("l_leg_kny", -stk13); // left knee
  state3->setDesiredJointPosition("l_leg_aky", -sta13); // left ankle
  //---- arm
  state3->setDesiredJointPosition("l_arm_shz", -20.00_deg); // left arm
  state3->setDesiredJointPosition("r_arm_shz", +10.00_deg); // right arm
  state3->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state3->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //---- feedback gain for hip joints
  state3->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state3->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state3->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state3->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state3->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state3->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state3->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state3->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  sm->addState(state0);
  sm->addState(state1);
  sm->addState(state2);
  sm->addState(state3);

  sm->setInitialState(state1);

  return sm;
}

//==============================================================================
StateMachine* Controller::_createRunningStateMachine()
{
  using namespace dart::math::suffixes;

  const double cd = 0.5;
  const double cv = 0.2;

  const double pelvis = -10.0_deg; // angle b/w pelvis and torso

  const double swh01 = 0.50;  // swing hip
  const double swk01 = -1.10; // swing knee
  const double swa01 = 0.60;  // swing angle
  const double stk01 = -0.05; // stance knee
  const double sta01 = 0.00;  // stance ankle

  StateMachine* sm = new StateMachine("running");

  State* state0 = new State(mAtlasRobot, "0");
  State* state1 = new State(mAtlasRobot, "1");

  TerminalCondition* cond0 = new TimerCondition(state0, 0.15);
  TerminalCondition* cond1 = new TimerCondition(state1, 0.15);

  state0->setTerminalCondition(cond0);
  state1->setTerminalCondition(cond1);

  state0->setNextState(state1);
  state1->setNextState(state0);

  // Set stance foot
  state0->setStanceFootToLeftFoot();
  state1->setStanceFootToRightFoot();

  // Set global desired pelvis angle
  state0->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnSagittal(0.0_deg);
  state0->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);
  state1->setDesiredPelvisGlobalAngleOnCoronal(0.0_deg);

  // Set desired joint position
  //-- State 0
  //---- pelvis
  state0->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state0->setDesiredJointPosition("r_leg_hpy", -swh01); // right hip
  state0->setDesiredJointPosition("r_leg_kny", -swk01); // right knee
  state0->setDesiredJointPosition("r_leg_aky", -swa01); // right ankle
  //---- stance leg
  state0->setDesiredJointPosition("l_leg_kny", -stk01); // left knee
  state0->setDesiredJointPosition("l_leg_aky", -sta01); // left ankle
  //---- arm
  state0->setDesiredJointPosition("l_arm_shz", -45.00_deg); // left arm
  state0->setDesiredJointPosition("r_arm_shz", +15.00_deg); // right arm
  state0->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state0->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //  state0->setDesiredJointPosition(23, DART_RADIAN * +90.00); // left arm
  //  state0->setDesiredJointPosition(24, DART_RADIAN * +90.00); // right arm
  //  state0->setDesiredJointPosition(27, DART_RADIAN * +90.00); // left arm
  //  state0->setDesiredJointPosition(28, DART_RADIAN * -90.00); // right arm
  //---- feedback gain for hip joints
  state0->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state0->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state0->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state0->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state0->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state0->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state0->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state0->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  //-- State 2
  //---- pelvis
  state1->setDesiredJointPosition(
      "back_bky", -pelvis); // angle b/w pelvis and torso
  //---- swing leg
  state1->setDesiredJointPosition("l_leg_hpy", -swh01); // left hip
  state1->setDesiredJointPosition("l_leg_kny", -swk01); // left knee
  state1->setDesiredJointPosition("l_leg_aky", -swa01); // left ankle
  //---- stance leg
  state1->setDesiredJointPosition("r_leg_kny", -stk01); // right knee
  state1->setDesiredJointPosition("r_leg_aky", -sta01); // right ankle
  //---- arm
  state1->setDesiredJointPosition("l_arm_shz", +15.00_deg); // left arm
  state1->setDesiredJointPosition("r_arm_shz", -45.00_deg); // right arm
  state1->setDesiredJointPosition("l_arm_shx", -80.00_deg); // left arm
  state1->setDesiredJointPosition("r_arm_shx", +80.00_deg); // right arm
  //  state1->setDesiredJointPosition(23, DART_RADIAN * +90.00); // left arm
  //  state1->setDesiredJointPosition(24, DART_RADIAN * +90.00); // right arm
  //  state1->setDesiredJointPosition(27, DART_RADIAN * +90.00); // left arm
  //  state1->setDesiredJointPosition(28, DART_RADIAN * -90.00); // right arm
  //---- feedback gain for hip joints
  state1->setFeedbackCoronalCOMDistance(
      mCoronalLeftHip, -cd); // coronal left hip
  state1->setFeedbackCoronalCOMVelocity(
      mCoronalLeftHip, -cv); // coronal left hip
  state1->setFeedbackCoronalCOMDistance(
      mCoronalRightHip, -cd); // coronal right hip
  state1->setFeedbackCoronalCOMVelocity(
      mCoronalRightHip, -cv); // coronal right hip
  state1->setFeedbackSagittalCOMDistance(
      mSagittalLeftHip, -cd); // sagittal left hip
  state1->setFeedbackSagittalCOMVelocity(
      mSagittalLeftHip, -cv); // sagittal left hip
  state1->setFeedbackSagittalCOMDistance(
      mSagittalRightHip, -cd); // sagittal right hip
  state1->setFeedbackSagittalCOMVelocity(
      mSagittalRightHip, -cv); // sagittal right hip

  sm->addState(state0);
  sm->addState(state1);

  sm->setInitialState(state0);

  return sm;
}

//==============================================================================
void Controller::_setJointDamping()
{
  for (const auto i :
       std::views::iota(std::size_t{1}, mAtlasRobot->getNumBodyNodes())) {
    Joint* joint = mAtlasRobot->getJoint(i);
    if (joint->getNumDofs() > 0) {
      for (const auto j :
           std::views::iota(std::size_t{0}, joint->getNumDofs())) {
        joint->setDampingCoefficient(j, 80.0);
      }
    }
  }
}

//==============================================================================
BodyNode* Controller::_getLeftFoot() const
{
  return mAtlasRobot->getBodyNode("l_foot");
}

//==============================================================================
BodyNode* Controller::_getRightFoot() const
{
  return mAtlasRobot->getBodyNode("r_foot");
}

//==============================================================================
bool Controller::_containStateMachine(const StateMachine* _stateMachine) const
{
  for (std::vector<StateMachine*>::const_iterator it = mStateMachines.begin();
       it != mStateMachines.end();
       ++it) {
    if (*it == _stateMachine) {
      return true;
    }
  }

  return false;
}

//==============================================================================
bool Controller::_containStateMachine(const std::string& _name) const
{
  return _containStateMachine(_findStateMachine(_name));
}

//==============================================================================
StateMachine* Controller::_findStateMachine(const std::string& _name) const
{
  StateMachine* stateMachine = nullptr;

  for (std::vector<StateMachine*>::const_iterator it = mStateMachines.begin();
       it != mStateMachines.end();
       ++it) {
    if ((*it)->getName() == _name) {
      stateMachine = *it;
      break;
    }
  }

  return stateMachine;
}

//==============================================================================
// Scene construction and runtime
//==============================================================================

constexpr const char* kAtlasUri
    = "dart://sample/sdf/atlas/atlas_v5_no_head.urdf";
constexpr double kDefaultGravity = 9.81;
constexpr double kDefaultPushForce = 500.0;
constexpr int kDefaultPushFrames = 100;
constexpr double kGroundHalfExtent = 12.5;
constexpr double kGroundThickness = 0.05;
constexpr double kGroundCenterZ = -0.95;

dart::dynamics::SkeletonPtr readRequiredSkeleton(const char* uri)
{
  auto skeleton = dart::io::readSkeleton(uri);
  if (skeleton == nullptr) {
    throw std::runtime_error(std::string("Failed to load ") + uri);
  }
  return skeleton;
}

void makeAtlasMeshVisualsReadable(const dart::dynamics::SkeletonPtr& atlas)
{
  const Eigen::Vector4d readableAtlasColor(0.15, 0.16, 0.18, 1.0);

  for (std::size_t i = 0; i < atlas->getNumBodyNodes(); ++i) {
    auto* body = atlas->getBodyNode(i);
    for (std::size_t j = 0; j < body->getNumShapeNodes(); ++j) {
      auto* shapeNode = body->getShapeNode(j);
      auto* visual = shapeNode->getVisualAspect();
      if (visual == nullptr) {
        continue;
      }

      const auto mesh = std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
          shapeNode->getShape());
      if (mesh == nullptr) {
        continue;
      }

      mesh->setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR);
      const Eigen::Vector4d rgba = visual->getRGBA();
      Eigen::Vector4d readable = readableAtlasColor;
      readable.w() = rgba.w();
      visual->setRGBA(readable);
    }
  }
}

dart::dynamics::SkeletonPtr createZUpGround()
{
  auto ground = dart::dynamics::Skeleton::create("atlas_simbicon_ground");
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  body->setName("ground_link");

  auto shape = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
      2.0 * kGroundHalfExtent, 2.0 * kGroundHalfExtent, kGroundThickness));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, kGroundCenterZ));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.54, 0.56, 0.52, 1.0));
  ground->setMobile(false);
  return ground;
}

struct AtlasSimbiconScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
};

AtlasSimbiconScene createAtlasSimbiconScene()
{
  AtlasSimbiconScene scene;
  scene.world = dart::simulation::World::create("dartsim_atlas_simbicon");

  auto ground = createZUpGround();
  scene.atlas = readRequiredSkeleton(kAtlasUri);
  makeAtlasMeshVisualsReadable(scene.atlas);

  scene.world->setGravity(-kDefaultGravity * Eigen::Vector3d::UnitZ());
  scene.world->addSkeleton(ground);
  scene.world->addSkeleton(scene.atlas);
  return scene;
}

class AtlasSimbiconRuntime
{
public:
  explicit AtlasSimbiconRuntime(AtlasSimbiconScene scene)
    : mScene(std::move(scene)),
      mController(
          std::make_unique<Controller>(
              mScene.atlas, mScene.world->getConstraintSolver()))
  {
    // Keep the controller's historical default state machine.
  }

  dart::simulation::WorldPtr world() const
  {
    return mScene.world;
  }

  void preStep()
  {
    if (auto* pelvis = mController->getAtlasRobot()->getBodyNode("pelvis")) {
      pelvis->addExtForce(mExternalForce);
    }

    mController->update();

    if (mForceDuration > 0) {
      --mForceDuration;
    } else {
      mExternalForce.setZero();
    }
  }

  void reset()
  {
    mExternalForce.setZero();
    mForceDuration = 0;
    mController->resetRobot();
  }

  void pushForward()
  {
    push(Eigen::Vector3d::UnitX() * kDefaultPushForce);
  }

  void pushBackward()
  {
    push(-Eigen::Vector3d::UnitX() * kDefaultPushForce);
  }

  void pushLeft()
  {
    push(Eigen::Vector3d::UnitY() * kDefaultPushForce);
  }

  void pushRight()
  {
    push(-Eigen::Vector3d::UnitY() * kDefaultPushForce);
  }

  void nextState()
  {
    if (auto* stateMachine = mController->getCurrentState()) {
      stateMachine->transiteToNextState(mScene.world->getTime());
    }
  }

  void switchToStanding()
  {
    changeStateMachine("standing");
  }

  void switchToWalkingInPlace()
  {
    changeStateMachine("walking in place");
  }

  void switchToWalking()
  {
    changeStateMachine("walking");
  }

  void switchToRunning()
  {
    changeStateMachine("running");
  }

  void setGravity(double gravity)
  {
    mGravity = gravity;
    mScene.world->setGravity(-mGravity * Eigen::Vector3d::UnitZ());
  }

  double gravity() const
  {
    return mGravity;
  }

  void setPelvisHarnessed(bool enabled)
  {
    if (enabled == mPelvisHarnessed) {
      return;
    }

    enabled ? mController->harnessPelvis() : mController->unharnessPelvis();
    mPelvisHarnessed = enabled;
  }

  void setLeftFootHarnessed(bool enabled)
  {
    if (enabled == mLeftFootHarnessed) {
      return;
    }

    enabled ? mController->harnessLeftFoot() : mController->unharnessLeftFoot();
    mLeftFootHarnessed = enabled;
  }

  void setRightFootHarnessed(bool enabled)
  {
    if (enabled == mRightFootHarnessed) {
      return;
    }

    enabled ? mController->harnessRightFoot()
            : mController->unharnessRightFoot();
    mRightFootHarnessed = enabled;
  }

  bool pelvisHarnessed() const
  {
    return mPelvisHarnessed;
  }

  bool leftFootHarnessed() const
  {
    return mLeftFootHarnessed;
  }

  bool rightFootHarnessed() const
  {
    return mRightFootHarnessed;
  }

  std::string currentStateMachineName() const
  {
    if (auto* stateMachine = mController->getCurrentState()) {
      return stateMachine->getName();
    }
    return "none";
  }

private:
  void push(const Eigen::Vector3d& force)
  {
    mExternalForce = force;
    mForceDuration = kDefaultPushFrames;
  }

  void changeStateMachine(const std::string& name)
  {
    mController->changeStateMachine(name, mScene.world->getTime());
  }

  AtlasSimbiconScene mScene;
  std::unique_ptr<Controller> mController;
  Eigen::Vector3d mExternalForce = Eigen::Vector3d::Zero();
  int mForceDuration = 0;
  double mGravity = kDefaultGravity;
  bool mPelvisHarnessed = false;
  bool mLeftFootHarnessed = false;
  bool mRightFootHarnessed = false;
};

dart::gui::OrbitCamera makeAtlasSimbiconCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, -0.25);
  camera.yaw = 0.55;
  camera.pitch = 0.32;
  camera.distance = 4.8;
  return camera;
}

dart::gui::KeyboardAction makeAtlasAction(
    std::string label, char key, std::function<void()> callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback
      = [callback = std::move(callback)](dart::gui::KeyboardActionContext&) {
          callback();
        };
  return action;
}

std::vector<dart::gui::KeyboardAction> createAtlasSimbiconKeyboardActions(
    const std::shared_ptr<AtlasSimbiconRuntime>& runtime)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(
      makeAtlasAction("Reset Atlas", 'r', [runtime]() { runtime->reset(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas forward", 'a', [runtime]() { runtime->pushForward(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas backward", 's', [runtime]() { runtime->pushBackward(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas left", 'd', [runtime]() { runtime->pushLeft(); }));
  actions.push_back(makeAtlasAction(
      "Push Atlas right", 'f', [runtime]() { runtime->pushRight(); }));
  actions.push_back(makeAtlasAction("Standing controller", '1', [runtime]() {
    runtime->switchToStanding();
  }));
  actions.push_back(
      makeAtlasAction("Walking-in-place controller", '2', [runtime]() {
        runtime->switchToWalkingInPlace();
      }));
  actions.push_back(makeAtlasAction(
      "Walking controller", '3', [runtime]() { runtime->switchToWalking(); }));
  actions.push_back(makeAtlasAction(
      "Running controller", '4', [runtime]() { runtime->switchToRunning(); }));
  return actions;
}

dart::gui::Panel createAtlasSimbiconPanel(
    const std::shared_ptr<AtlasSimbiconRuntime>& runtime)
{
  dart::gui::Panel panel;
  panel.title = "Atlas Control";
  panel.buildWithContext = [runtime](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Atlas robot controlled by Simbicon");
    builder.separator();
    builder.text("User Guide");
    builder.text("Press [r] to reset Atlas to the initial position.");
    builder.text("Press [a] to push forward Atlas torso.");
    builder.text("Press [s] to push backward Atlas torso.");
    builder.text("Press [d] to push left Atlas torso.");
    builder.text("Press [f] to push right Atlas torso.");
    builder.text("Press [1]/[2]/[3]/[4] to switch state machines.");
    builder.text("Select objects with left click; keyboard nudges move them.");
    builder.separator();

    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Play" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Exit")) {
        dart::gui::requestExit(*context.lifecycle);
      }
    }

    builder.separator();
    double gravity = runtime->gravity();
    if (builder.slider("Gravity Acc.", gravity, 5.0, 20.0)) {
      runtime->setGravity(gravity);
    }

    bool pelvis = runtime->pelvisHarnessed();
    if (builder.checkbox("Harness pelvis", pelvis)) {
      runtime->setPelvisHarnessed(pelvis);
    }
    bool leftFoot = runtime->leftFootHarnessed();
    if (builder.checkbox("Harness left foot", leftFoot)) {
      runtime->setLeftFootHarnessed(leftFoot);
    }
    bool rightFoot = runtime->rightFootHarnessed();
    if (builder.checkbox("Harness right foot", rightFoot)) {
      runtime->setRightFootHarnessed(rightFoot);
    }
    if (context.lighting.headlightsEnabled != nullptr) {
      bool headlights = *context.lighting.headlightsEnabled;
      if (builder.checkbox("Headlights On/Off", headlights)) {
        *context.lighting.headlightsEnabled = headlights;
      }
    }
    if (context.rendering.settings != nullptr) {
      bool shadows = context.rendering.settings->shadowsEnabled;
      if (builder.checkbox("Shadow On/Off", shadows)) {
        context.rendering.settings->shadowsEnabled = shadows;
      }

      bool depthMode = context.rendering.settings->outputMode
                       == dart::gui::RenderOutputMode::Depth;
      if (builder.checkbox("Depth mode", depthMode)) {
        context.rendering.settings->outputMode
            = depthMode ? dart::gui::RenderOutputMode::Depth
                        : dart::gui::RenderOutputMode::Color;
      }
    }

    builder.separator();
    if (builder.button("Reset Atlas")) {
      runtime->reset();
    }
    if (builder.button("No Control")) {
      runtime->switchToStanding();
    }
    builder.sameLine();
    if (builder.button("Walking In Place")) {
      runtime->switchToWalkingInPlace();
    }
    if (builder.button("Normal-Stride Walking")) {
      runtime->switchToWalking();
    }
    builder.sameLine();
    if (builder.button("Short-Stride Walking")) {
      runtime->switchToRunning();
    }
    if (builder.button("Next State")) {
      runtime->nextState();
    }

    builder.separator();
    builder.text("state machine: " + runtime->currentStateMachineName());
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace atlas_simbicon_detail

using namespace atlas_simbicon_detail;

dart::gui::ApplicationOptions makeAtlasSimbiconScene()
{
  auto runtime
      = std::make_shared<AtlasSimbiconRuntime>(createAtlasSimbiconScene());

  dart::gui::ApplicationOptions options;
  options.world = runtime->world();
  options.camera = makeAtlasSimbiconCamera();
  options.preStep = [runtime]() {
    runtime->preStep();
  };
  options.panels.push_back(createAtlasSimbiconPanel(runtime));
  options.keyboardActions = createAtlasSimbiconKeyboardActions(runtime);
  return options;
}

} // namespace dart::examples::demos
