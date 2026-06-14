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

#include "dart/simulation/World.hpp"

#include "dart/collision/CollisionGroup.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Profile.hpp"
#include "dart/constraint/BoxedLcpConstraintSolver.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/integration/SemiImplicitEulerIntegrator.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <cmath>

namespace dart {
namespace simulation {

//==============================================================================
std::shared_ptr<World> World::create(const std::string& name)
{
  return std::make_shared<World>(name);
}

//==============================================================================
World::World(const std::string& _name)
  : mName(_name),
    mNameMgrForSkeletons("World::Skeleton | " + _name, "skeleton"),
    mNameMgrForSimpleFrames("World::SimpleFrame | " + _name, "frame"),
    mGravity(0.0, 0.0, -9.81),
    mTimeStep(0.001),
    mTime(0.0),
    mFrame(0),
    mRecording(new Recording(mSkeletons)),
    onNameChanged(mNameChangedSignal)
{
  mIndices.push_back(0);

  auto solver = std::make_unique<constraint::BoxedLcpConstraintSolver>();
  setConstraintSolver(std::move(solver));
}

//==============================================================================
World::~World()
{
  delete mRecording;

  for (common::Connection& connection : mNameConnectionsForSkeletons)
    connection.disconnect();

  for (common::Connection& connection : mNameConnectionsForSimpleFrames)
    connection.disconnect();
}

//==============================================================================
WorldPtr World::clone() const
{
  WorldPtr worldClone = World::create(mName);

  worldClone->setGravity(mGravity);
  worldClone->setTimeStep(mTimeStep);
  worldClone->setDeactivationOptions(mDeactivationOptions);

  auto cd = getConstraintSolver()->getCollisionDetector();
  worldClone->getConstraintSolver()->setCollisionDetector(
      cd->cloneWithoutCollisionObjects());

  // Clone and add each Skeleton
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    worldClone->addSkeleton(mSkeletons[i]->cloneSkeleton());
  }

  // Clone and add each SimpleFrame
  for (std::size_t i = 0; i < mSimpleFrames.size(); ++i) {
    worldClone->addSimpleFrame(
        mSimpleFrames[i]->clone(mSimpleFrames[i]->getParentFrame()));
  }

  // For each newly cloned SimpleFrame, try to make its parent Frame be one of
  // the new clones if there is a match. This is meant to minimize any possible
  // interdependencies between the kinematics of different worlds.
  for (std::size_t i = 0; i < worldClone->getNumSimpleFrames(); ++i) {
    dynamics::Frame* current_parent
        = worldClone->getSimpleFrame(i)->getParentFrame();

    dynamics::SimpleFramePtr parent_candidate
        = worldClone->getSimpleFrame(current_parent->getName());

    if (parent_candidate)
      worldClone->getSimpleFrame(i)->setParentFrame(parent_candidate.get());
  }

  return worldClone;
}

//==============================================================================
void World::setTimeStep(double _timeStep)
{
  if (!std::isfinite(_timeStep) || _timeStep <= 0.0) {
    dtwarn << "[World] Attempting to set invalid timestep (" << _timeStep
           << "). Ignoring this "
           << "request because it can lead to undefined behavior.\n";
    return;
  }

  mTimeStep = _timeStep;
  DART_ASSERT(mConstraintSolver);
  mConstraintSolver->setTimeStep(_timeStep);
  for (auto& skel : mSkeletons)
    skel->setTimeStep(_timeStep);
}

//==============================================================================
double World::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void World::reset()
{
  mTime = 0.0;
  mFrame = 0;
  mRecording->clear();
  mConstraintSolver->clearLastCollisionResult();

  for (auto& skel : mSkeletons) {
    skel->clearConstraintImpulses();
    skel->setImpulseApplied(false);
  }
}

//==============================================================================
void World::step(bool _resetCommand)
{
  DART_PROFILE_FRAME;

  const bool deactivationEnabled = mDeactivationOptions.mEnabled;

  // Mirror the enable flag onto the solver so its island rest-detection / LCP
  // skipping runs only when the feature is on (otherwise it is a strict no-op).
  mConstraintSolver->setDeactivationActive(deactivationEnabled);

  // Tracks, per skeleton, whether it was disturbed (woken or kept awake) this
  // step so the post-step rest-detection pass can treat it as non-quiet even
  // after _resetCommand clears the actuation vectors below. Disturbance
  // tracking is only needed while contact islands exist; unconstrained active
  // scenes cannot sleep.
  std::vector<bool> disturbedThisStep;
  const bool trackDisturbances
      = deactivationEnabled
        && mConstraintSolver->getLastCollisionResult().getNumContacts() > 0;
  if (trackDisturbances)
    disturbedThisStep.assign(mSkeletons.size(), false);

  // Integrate velocity for unconstrained skeletons
  {
    DART_PROFILE_SCOPED_N("World::step - Integrate velocity");
    for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
      auto& skel = mSkeletons[i];
      if (!skel->isMobile())
        continue;

      const bool externallyDisturbed
          = deactivationEnabled && (trackDisturbances || skel->isResting())
            && skel->hasExternalDisturbance();
      if (externallyDisturbed && !disturbedThisStep.empty())
        disturbedThisStep[i] = true;

      // A resting skeleton skips forward dynamics (including gravity) unless a
      // user-applied force/command is pending this step, in which case it is
      // woken so the actuation is integrated rather than silently dropped.
      // Gravity alone is not a disturbance, so a body resting on its support
      // stays asleep.
      if (deactivationEnabled && skel->isResting()) {
        if (externallyDisturbed) {
          skel->setResting(false);
          skel->setSleepCandidate(false);
        } else {
          continue;
        }
      }

      skel->computeForwardDynamics();
      skel->integrateVelocities(mTimeStep);
    }
  }

  // Detect activated constraints and compute constraint impulses
  {
    DART_PROFILE_SCOPED_N("World::step - Solve constraints");
    mConstraintSolver->solve();
  }

  // Compute velocity changes given constraint impulses
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    auto& skel = mSkeletons[i];
    if (!skel->isMobile())
      continue;

    // A resting skeleton skips both the impulse-based forward dynamics and the
    // position integration. The only exception is when an impulse was applied
    // to it this step: that means a moving body contacted this island (the
    // solver united them so the group was not all-resting and was solved),
    // delivering an impulse. Such a skeleton must wake and be processed
    // normally so the impulse is applied and the position is advanced.
    if (deactivationEnabled && skel->isResting()) {
      if (skel->isImpulseApplied()) {
        skel->setResting(false);
        skel->setSleepCandidate(false);
        if (!disturbedThisStep.empty())
          disturbedThisStep[i] = true;
      } else {
        continue;
      }
    }

    if (skel->isImpulseApplied()) {
      skel->computeImpulseForwardDynamics();
      skel->setImpulseApplied(false);
    }

    skel->integratePositions(mTimeStep);

    if (_resetCommand) {
      skel->clearInternalForces();
      skel->clearExternalForces();
      skel->resetCommands();
    }
  }

  // Rest-detection pass: decide which mobile skeletons are quiet enough to
  // sleep, and wake any that have started moving again. This is a deterministic
  // function of post-step cached speeds, dwell time, and the configured
  // thresholds, so it does not depend on container or iteration order.
  if (deactivationEnabled
      && mConstraintSolver->getLastCollisionResult().getNumContacts() > 0)
    updateRestStates(disturbedThisStep);

  mTime += mTimeStep;
  mFrame++;
}

//==============================================================================
void World::updateRestStates(const std::vector<bool>& disturbedThisStep)
{
  DART_PROFILE_SCOPED_N("World::step - Rest detection");

  const double linSleep = mDeactivationOptions.mLinearSpeedThreshold;
  const double angSleep = mDeactivationOptions.mAngularSpeedThreshold;
  const double scale = mDeactivationOptions.mWakeThresholdScale;
  const double linWake = scale * linSleep;
  const double angWake = scale * angSleep;

  // EMA weight for the speed smoothing. Resting-contact solves emit small
  // per-step velocity jitter; smoothing the speed lets a genuinely settled
  // island fall below the sleep band and accumulate dwell, while a body in real
  // sustained motion keeps a high smoothed speed and never qualifies.
  const double alpha = 0.2;

  // This pass updates per-skeleton sleep CANDIDACY only. The island-atomic
  // freeze flag (isResting) is derived from candidacy by the constraint solver,
  // so a body is never frozen unless its whole island qualifies.
  for (std::size_t i = 0; i < mSkeletons.size(); ++i) {
    auto& skel = mSkeletons[i];
    if (!skel->isMobile())
      continue;

    if (skel->getIslandIndex() < 0) {
      skel->setSleepCandidate(false);
      continue;
    }

    const double linInstant = skel->computeMaxBodyLinearSpeed();
    const double angInstant = skel->computeMaxBodyAngularSpeed();
    const double linSpeed
        = (1.0 - alpha) * skel->getSmoothedLinearSpeed() + alpha * linInstant;
    const double angSpeed
        = (1.0 - alpha) * skel->getSmoothedAngularSpeed() + alpha * angInstant;
    skel->setSmoothedLinearSpeed(linSpeed);
    skel->setSmoothedAngularSpeed(angSpeed);

    const bool disturbed
        = (i < disturbedThisStep.size() && disturbedThisStep[i])
          || skel->hasExternalDisturbance();

    if (skel->isSleepCandidate()) {
      // Stay a candidate unless the body clearly started moving again (beyond
      // the wider wake band) or was disturbed/impulsed this step. The wake band
      // is larger than the sleep band (hysteresis) to avoid candidacy
      // thrashing.
      if (disturbed || linSpeed > linWake || angSpeed > angWake)
        skel->setSleepCandidate(false);
    } else {
      const bool quiet
          = (linSpeed < linSleep) && (angSpeed < angSleep) && !disturbed;
      if (quiet) {
        const double dwell = skel->getRestDwellTime() + mTimeStep;
        skel->setRestDwellTime(dwell);
        if (dwell >= mDeactivationOptions.mTimeUntilSleep)
          skel->setSleepCandidate(true);
      } else {
        skel->setRestDwellTime(0.0);
      }
    }
  }
}

//==============================================================================
void World::setTime(double _time)
{
  mTime = _time;
}

//==============================================================================
double World::getTime() const
{
  return mTime;
}

//==============================================================================
int World::getSimFrames() const
{
  return mFrame;
}

//==============================================================================
const std::string& World::setName(const std::string& _newName)
{
  if (_newName == mName)
    return mName;

  const std::string oldName = mName;
  mName = _newName;

  mNameChangedSignal.raise(oldName, mName);

  mNameMgrForSkeletons.setManagerName("World::Skeleton | " + mName);
  mNameMgrForSimpleFrames.setManagerName("World::SimpleFrame | " + mName);

  return mName;
}

//==============================================================================
const std::string& World::getName() const
{
  return mName;
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& _gravity)
{
  mGravity = _gravity;
  for (std::vector<dynamics::SkeletonPtr>::iterator it = mSkeletons.begin();
       it != mSkeletons.end();
       ++it) {
    (*it)->setGravity(_gravity);
  }
}

//==============================================================================
void World::setGravity(double x, double y, double z)
{
  setGravity(Eigen::Vector3d(x, y, z));
}

//==============================================================================
const Eigen::Vector3d& World::getGravity() const
{
  return mGravity;
}

//==============================================================================
dynamics::SkeletonPtr World::getSkeleton(std::size_t _index) const
{
  if (_index < mSkeletons.size())
    return mSkeletons[_index];

  return nullptr;
}

//==============================================================================
dynamics::SkeletonPtr World::getSkeleton(const std::string& _name) const
{
  return mNameMgrForSkeletons.getObject(_name);
}

//==============================================================================
std::size_t World::getNumSkeletons() const
{
  return mSkeletons.size();
}

//==============================================================================
std::string World::addSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  if (nullptr == _skeleton) {
    dtwarn << "[World::addSkeleton] Attempting to add a nullptr Skeleton to "
           << "the world!\n";
    return "";
  }

  // If mSkeletons already has _skeleton, then we do nothing.
  if (find(mSkeletons.begin(), mSkeletons.end(), _skeleton)
      != mSkeletons.end()) {
    dtwarn << "[World::addSkeleton] Skeleton named [" << _skeleton->getName()
           << "] is already in the world." << std::endl;
    return _skeleton->getName();
  }

  mSkeletons.push_back(_skeleton);
  mMapForSkeletons[_skeleton] = _skeleton;

  mNameConnectionsForSkeletons.push_back(_skeleton->onNameChanged.connect(
      [=](dynamics::ConstMetaSkeletonPtr skel,
          const std::string&,
          const std::string&) { this->handleSkeletonNameChange(skel); }));

  _skeleton->setName(
      mNameMgrForSkeletons.issueNewNameAndAdd(_skeleton->getName(), _skeleton));

  _skeleton->setTimeStep(mTimeStep);
  _skeleton->setGravity(mGravity);

  mIndices.push_back(mIndices.back() + _skeleton->getNumDofs());
  mConstraintSolver->addSkeleton(_skeleton);

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);

  return _skeleton->getName();
}

//==============================================================================
void World::removeSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  DART_ASSERT(
      _skeleton != nullptr
      && "Attempted to remove nullptr Skeleton from world");

  if (nullptr == _skeleton) {
    dtwarn << "[World::removeSkeleton] Attempting to remove a nullptr Skeleton "
           << "from the world!\n";
    return;
  }

  // Find index of _skeleton in mSkeleton.
  std::size_t index = 0;
  for (; index < mSkeletons.size(); ++index) {
    if (mSkeletons[index] == _skeleton)
      break;
  }

  // If i is equal to the number of skeletons, then _skeleton is not in
  // mSkeleton. We do nothing.
  if (index == mSkeletons.size()) {
    dtwarn << "[World::removeSkeleton] Skeleton [" << _skeleton->getName()
           << "] is not in the world.\n";
    return;
  }

  // Update mIndices.
  for (std::size_t i = index + 1; i < mSkeletons.size() - 1; ++i)
    mIndices[i] = mIndices[i + 1] - _skeleton->getNumDofs();
  mIndices.pop_back();

  // Remove _skeleton from constraint handler.
  mConstraintSolver->removeSkeleton(_skeleton);

  // Remove _skeleton from mSkeletons
  mSkeletons.erase(
      remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
      mSkeletons.end());

  // Disconnect the name change monitor
  mNameConnectionsForSkeletons[index].disconnect();
  mNameConnectionsForSkeletons.erase(
      mNameConnectionsForSkeletons.begin() + index);

  // Update recording
  mRecording->updateNumGenCoords(mSkeletons);

  // Remove from NameManager
  mNameMgrForSkeletons.removeName(_skeleton->getName());

  // Remove from the pointer map
  mMapForSkeletons.erase(_skeleton);
}

//==============================================================================
std::set<dynamics::SkeletonPtr> World::removeAllSkeletons()
{
  std::set<dynamics::SkeletonPtr> ptrs;
  for (std::vector<dynamics::SkeletonPtr>::iterator it = mSkeletons.begin(),
                                                    end = mSkeletons.end();
       it != end;
       ++it)
    ptrs.insert(*it);

  while (getNumSkeletons() > 0)
    removeSkeleton(getSkeleton(0));

  return ptrs;
}

//==============================================================================
bool World::hasSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const
{
  return std::find(mSkeletons.begin(), mSkeletons.end(), skeleton)
         != mSkeletons.end();
}

//==============================================================================
bool World::hasSkeleton(const std::string& skeletonName) const
{
  return mNameMgrForSkeletons.hasName(skeletonName);
}

//==============================================================================
int World::getIndex(int _index) const
{
  if (_index < 0 || static_cast<std::size_t>(_index) >= mIndices.size()) {
    dterr << "[World::getIndex] Index [" << _index << "] is out of range. "
          << "Valid range is [0, " << mIndices.size() << ").\n";
    DART_ASSERT(false);
    return -1;
  }
  return mIndices[_index];
}

//==============================================================================
dynamics::SimpleFramePtr World::getSimpleFrame(std::size_t _index) const
{
  if (_index < mSimpleFrames.size())
    return mSimpleFrames[_index];

  return nullptr;
}

//==============================================================================
dynamics::SimpleFramePtr World::getSimpleFrame(const std::string& _name) const
{
  return mNameMgrForSimpleFrames.getObject(_name);
}

//==============================================================================
std::size_t World::getNumSimpleFrames() const
{
  return mSimpleFrames.size();
}

//==============================================================================
std::string World::addSimpleFrame(const dynamics::SimpleFramePtr& _frame)
{
  DART_ASSERT(
      _frame != nullptr && "Attempted to add nullptr SimpleFrame to world");

  if (nullptr == _frame) {
    dtwarn << "[World::addFrame] Attempting to add a nullptr SimpleFrame to "
              "the world!\n";
    return "";
  }

  if (find(mSimpleFrames.begin(), mSimpleFrames.end(), _frame)
      != mSimpleFrames.end()) {
    dtwarn << "[World::addFrame] SimpleFrame named [" << _frame->getName()
           << "] is already in the world.\n";
    return _frame->getName();
  }

  mSimpleFrames.push_back(_frame);
  mSimpleFrameToShared[_frame.get()] = _frame;

  mNameConnectionsForSimpleFrames.push_back(_frame->onNameChanged.connect(
      [=](const dynamics::Entity* _entity,
          const std::string&,
          const std::string&) { this->handleSimpleFrameNameChange(_entity); }));

  _frame->setName(
      mNameMgrForSimpleFrames.issueNewNameAndAdd(_frame->getName(), _frame));

  return _frame->getName();
}

//==============================================================================
void World::removeSimpleFrame(const dynamics::SimpleFramePtr& _frame)
{
  DART_ASSERT(
      _frame != nullptr
      && "Attempted to remove nullptr SimpleFrame from world");

  std::vector<dynamics::SimpleFramePtr>::iterator it
      = find(mSimpleFrames.begin(), mSimpleFrames.end(), _frame);

  if (it == mSimpleFrames.end()) {
    dtwarn << "[World::removeFrame] Frame named [" << _frame->getName()
           << "] is not in the world.\n";
    return;
  }

  std::size_t index = it - mSimpleFrames.begin();

  // Remove the frame
  mSimpleFrames.erase(mSimpleFrames.begin() + index);

  // Disconnect the name change monitor
  mNameConnectionsForSimpleFrames[index].disconnect();
  mNameConnectionsForSimpleFrames.erase(
      mNameConnectionsForSimpleFrames.begin() + index);

  // Remove from NameManager
  mNameMgrForSimpleFrames.removeName(_frame->getName());

  // Remove from the pointer map
  mSimpleFrameToShared.erase(_frame.get());
}

//==============================================================================
std::set<dynamics::SimpleFramePtr> World::removeAllSimpleFrames()
{
  std::set<dynamics::SimpleFramePtr> ptrs;
  for (std::vector<dynamics::SimpleFramePtr>::iterator it
       = mSimpleFrames.begin(),
       end = mSimpleFrames.end();
       it != end;
       ++it)
    ptrs.insert(*it);

  while (getNumSimpleFrames() > 0)
    removeSimpleFrame(getSimpleFrame(0));

  return ptrs;
}

//==============================================================================
bool World::checkCollision(bool checkAllCollisions)
{
  collision::CollisionOption option;

  if (checkAllCollisions)
    option.maxNumContacts = 1e+3;
  else
    option.maxNumContacts = 1u;

  return checkCollision(option);
}

//==============================================================================
bool World::checkCollision(
    const collision::CollisionOption& option,
    collision::CollisionResult* result)
{
  return mConstraintSolver->getCollisionGroup()->collide(option, result);
}

//==============================================================================
const collision::CollisionResult& World::getLastCollisionResult() const
{
  return mConstraintSolver->getLastCollisionResult();
}

//==============================================================================
void World::setConstraintSolver(constraint::UniqueConstraintSolverPtr solver)
{
  if (!solver) {
    dtwarn << "[World::setConstraintSolver] nullptr for constraint solver is "
           << "not allowed. Doing nothing.";
    return;
  }

  if (mConstraintSolver)
    solver->setFromOtherConstraintSolver(*mConstraintSolver);

  mConstraintSolver = std::move(solver);
  mConstraintSolver->setTimeStep(mTimeStep);
}

//==============================================================================
constraint::ConstraintSolver* World::getConstraintSolver()
{
  return mConstraintSolver.get();
}

//==============================================================================
const constraint::ConstraintSolver* World::getConstraintSolver() const
{
  return mConstraintSolver.get();
}

//==============================================================================
void World::setDeactivationOptions(const DeactivationOptions& options)
{
  mDeactivationOptions = options;

  // If the feature is being disabled, clear any resting/candidacy state so
  // subsequent steps process every skeleton normally and behavior is identical
  // to the feature being absent.
  if (!mDeactivationOptions.mEnabled) {
    for (auto& skel : mSkeletons) {
      skel->setResting(false);
      skel->setSleepCandidate(false);
      skel->setIslandIndex(-1);
    }
  }
}

//==============================================================================
const DeactivationOptions& World::getDeactivationOptions() const
{
  return mDeactivationOptions;
}

//==============================================================================
void World::bake()
{
  const auto collisionResult = getConstraintSolver()->getLastCollisionResult();
  const auto nContacts = static_cast<int>(collisionResult.getNumContacts());
  const auto nSkeletons = getNumSkeletons();

  Eigen::VectorXd state(getIndex(nSkeletons) + 6 * nContacts);
  for (auto i = 0u; i < getNumSkeletons(); ++i) {
    state.segment(getIndex(i), getSkeleton(i)->getNumDofs())
        = getSkeleton(i)->getPositions();
  }

  for (auto i = 0; i < nContacts; ++i) {
    auto begin = getIndex(nSkeletons) + i * 6;
    state.segment(begin, 3) = collisionResult.getContact(i).point;
    state.segment(begin + 3, 3) = collisionResult.getContact(i).force;
  }

  mRecording->addState(state);
}

//==============================================================================
Recording* World::getRecording()
{
  return mRecording;
}

//==============================================================================
void World::handleSkeletonNameChange(
    const dynamics::ConstMetaSkeletonPtr& _skeleton)
{
  if (nullptr == _skeleton) {
    dterr << "[World::handleSkeletonNameChange] Received a name change "
          << "callback for a nullptr Skeleton. This is most likely a bug. "
          << "Please report this!\n";
    DART_ASSERT(false);
    return;
  }

  // Get the new name of the Skeleton
  const std::string& newName = _skeleton->getName();

  // Find the shared version of the Skeleton
  std::map<dynamics::ConstMetaSkeletonPtr, dynamics::SkeletonPtr>::iterator it
      = mMapForSkeletons.find(_skeleton);
  if (it == mMapForSkeletons.end()) {
    dterr << "[World::handleSkeletonNameChange] Could not find Skeleton named ["
          << _skeleton->getName() << "] in the shared_ptr map of World ["
          << getName() << "]. This is most likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
  dynamics::SkeletonPtr sharedSkel = it->second;

  // Inform the NameManager of the change
  std::string issuedName
      = mNameMgrForSkeletons.changeObjectName(sharedSkel, newName);

  // If the name issued by the NameManger does not match, reset the name of the
  // Skeleton to match the newly issued name.
  if ((!issuedName.empty()) && (newName != issuedName)) {
    sharedSkel->setName(issuedName);
  } else if (issuedName.empty()) {
    dterr << "[World::handleSkeletonNameChange] Skeleton named ["
          << sharedSkel->getName() << "] (" << sharedSkel << ") does not exist "
          << "in the NameManager of World [" << getName() << "]. This is most "
          << "likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
}

//==============================================================================
void World::handleSimpleFrameNameChange(const dynamics::Entity* _entity)
{
  // Check that this is actually a SimpleFrame
  const dynamics::SimpleFrame* frame
      = dynamic_cast<const dynamics::SimpleFrame*>(_entity);

  if (nullptr == frame) {
    dterr << "[World::handleFrameNameChange] Received a callback for a nullptr "
          << "enity. This is most likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }

  // Get the new name of the Frame
  const std::string& newName = frame->getName();

  // Find the shared version of the Frame
  std::map<const dynamics::SimpleFrame*, dynamics::SimpleFramePtr>::iterator it
      = mSimpleFrameToShared.find(frame);
  if (it == mSimpleFrameToShared.end()) {
    dterr << "[World::handleFrameNameChange] Could not find SimpleFrame named ["
          << frame->getName() << "] in the shared_ptr map of World ["
          << getName() << "]. This is most likely a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
  dynamics::SimpleFramePtr sharedFrame = it->second;

  std::string issuedName
      = mNameMgrForSimpleFrames.changeObjectName(sharedFrame, newName);

  if ((!issuedName.empty()) && (newName != issuedName)) {
    sharedFrame->setName(issuedName);
  } else if (issuedName.empty()) {
    dterr << "[World::handleFrameNameChange] SimpleFrame named ["
          << frame->getName() << "] (" << frame << ") does not exist in the "
          << "NameManager of World [" << getName() << "]. This is most likely "
          << "a bug. Please report this!\n";
    DART_ASSERT(false);
    return;
  }
}

} // namespace simulation
} // namespace dart
