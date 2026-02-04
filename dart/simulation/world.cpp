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

#include "dart/simulation/world.hpp"

#include "dart/collision/collision_detector.hpp"
#include "dart/collision/collision_group.hpp"
#include "dart/collision/fcl/fcl_collision_detector.hpp"
#include "dart/common/diagnostics.hpp"
#include "dart/common/exception.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/common/profile.hpp"
#include "dart/common/string.hpp"
#include "dart/constraint/boxed_lcp_constraint_solver.hpp"
#include "dart/constraint/constrained_group.hpp"
#include "dart/constraint/constraint_solver.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/pivoting/lemke_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <cmath>

namespace dart {
namespace simulation {

namespace {

using dart::collision::CollisionDetector;
using dart::collision::CollisionDetectorPtr;

void configureCollisionDetector(
    const CollisionDetectorPtr& detector, const std::string& key)
{
  if (!detector) {
    return;
  }

  if (key == "fcl") {
    auto fclDetector
        = std::dynamic_pointer_cast<collision::FCLCollisionDetector>(detector);
    if (fclDetector) {
      fclDetector->setPrimitiveShapeType(
          collision::FCLCollisionDetector::PRIMITIVE);
    }
  }
}

std::string toCollisionDetectorKey(CollisionDetectorType type)
{
  switch (type) {
    case CollisionDetectorType::Dart:
      return "dart";
    case CollisionDetectorType::Fcl:
      return "fcl";
    case CollisionDetectorType::Bullet:
      return "bullet";
    case CollisionDetectorType::Ode:
      return "ode";
  }

  DART_FATAL(
      "Encountered unsupported CollisionDetectorType value: {}.",
      static_cast<int>(type));
  return "fcl";
}

CollisionDetectorPtr tryCreateCollisionDetector(const std::string& requestedKey)
{
  if (requestedKey.empty()) {
    return nullptr;
  }

  auto key = common::toLower(requestedKey);

  auto* factory = CollisionDetector::getFactory();
  DART_ASSERT(factory);
  if (!factory->canCreate(key)) {
    return nullptr;
  }

  auto detector = factory->create(key);
  if (!detector) {
    DART_WARN(
        "Failed to create collision detector '{}' even though the factory "
        "reported it was available.",
        key);
  }

  configureCollisionDetector(detector, key);
  return detector;
}

CollisionDetectorPtr tryCreateCollisionDetector(CollisionDetectorType type)
{
  return tryCreateCollisionDetector(toCollisionDetectorKey(type));
}

CollisionDetectorPtr resolveCollisionDetector(const WorldConfig& config)
{
  const auto requestedType = config.collisionDetector;
  const auto requestedKey = toCollisionDetectorKey(requestedType);
  if (auto detector = tryCreateCollisionDetector(requestedType)) {
    return detector;
  }

  if (requestedType != CollisionDetectorType::Fcl) {
    DART_WARN(
        "WorldConfig requested collision detector '{}', but it is not "
        "available. "
        "Falling back to the default 'fcl' detector.",
        requestedKey);

    if (auto fallback
        = tryCreateCollisionDetector(CollisionDetectorType::Fcl)) {
      return fallback;
    }
  }

  DART_WARN(
      "Collision detector '{}' is not available and fallback 'fcl' also "
      "failed. "
      "The world will keep its existing collision detector.",
      requestedKey);
  return nullptr;
}

math::LcpSolverPtr createLcpSolver(LcpSolverType type)
{
  switch (type) {
    case LcpSolverType::Dantzig:
      return std::make_shared<math::DantzigSolver>();
    case LcpSolverType::Pgs:
      return std::make_shared<math::PgsSolver>();
    case LcpSolverType::Lemke:
      return std::make_shared<math::LemkeSolver>();
  }

  DART_FATAL(
      "Encountered unsupported LcpSolverType value: {}.",
      static_cast<int>(type));
  return std::make_shared<math::DantzigSolver>();
}

std::unique_ptr<constraint::ConstraintSolver> createConstraintSolver(
    const WorldConfig& config)
{
  // Create BoxedLcpConstraintSolver (not base ConstraintSolver) to maintain
  // backward compatibility with gz-physics, which does dynamic_cast to
  // BoxedLcpConstraintSolver to access getBoxedLcpSolver()/getType() etc.
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto solver = std::make_unique<constraint::BoxedLcpConstraintSolver>();
  DART_SUPPRESS_DEPRECATED_END

  auto primary = createLcpSolver(config.primaryLcpSolver);
  solver->setLcpSolver(std::move(primary));

  if (config.secondaryLcpSolver.has_value()) {
    auto secondary = createLcpSolver(config.secondaryLcpSolver.value());
    solver->setSecondaryLcpSolver(std::move(secondary));
  } else {
    solver->setSecondaryLcpSolver(nullptr);
  }

  return solver;
}

} // namespace

//==============================================================================
std::shared_ptr<World> World::create(std::string_view name)
{
  return std::make_shared<World>(name);
}

//==============================================================================
std::shared_ptr<World> World::create(const WorldConfig& config)
{
  return std::make_shared<World>(config);
}

//==============================================================================
World::World(std::string_view name) : World(WorldConfig(name)) {}

//==============================================================================
World::World(const WorldConfig& config)
  : mName(config.name),
    mNameMgrForSkeletons("World::Skeleton | " + config.name, "skeleton"),
    mNameMgrForSimpleFrames("World::SimpleFrame | " + config.name, "frame"),
    mSensorManager("World::Sensor | " + config.name, "sensor"),
    mGravity(0.0, 0.0, -9.81),
    mTimeStep(0.001),
    mTime(0.0),
    mFrame(0),
    mMemoryManager(
        config.baseAllocator ? *config.baseAllocator
                             : common::MemoryAllocator::GetDefault()),
    mRecording(new Recording(mSkeletons)),
    onNameChanged(mNameChangedSignal)
{
  mIndices.push_back(0);

  auto solver = createConstraintSolver(config);
  setConstraintSolver(std::move(solver));

  if (auto detector = resolveCollisionDetector(config)) {
    setCollisionDetector(detector);
  }
}

//==============================================================================
World::~World()
{
  delete mRecording;

  for (common::Connection& connection : mNameConnectionsForSkeletons) {
    connection.disconnect();
  }

  for (common::Connection& connection : mNameConnectionsForSimpleFrames) {
    connection.disconnect();
  }
}

//==============================================================================
WorldPtr World::clone() const
{
  WorldConfig config;
  config.name = mName;
  config.baseAllocator = const_cast<common::MemoryAllocator*>(
      &mMemoryManager.getBaseAllocator());
  WorldPtr worldClone = World::create(config);

  worldClone->setGravity(mGravity);
  worldClone->setTimeStep(mTimeStep);

  auto cd = getConstraintSolver()->getCollisionDetector();
  if (cd) {
    worldClone->setCollisionDetector(cd->cloneWithoutCollisionObjects());
  }

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

    if (parent_candidate) {
      worldClone->getSimpleFrame(i)->setParentFrame(parent_candidate.get());
    }
  }

  return worldClone;
}

//==============================================================================
void World::setTimeStep(double _timeStep)
{
  if (!std::isfinite(_timeStep) || _timeStep <= 0.0) {
    DART_WARN(
        "[World] Attempting to set an invalid timestep ({}). Ignoring this "
        "request because it can lead to undefined behavior.",
        _timeStep);
    return;
  }

  mTimeStep = _timeStep;
  DART_ASSERT(mConstraintSolver);
  mConstraintSolver->setTimeStep(_timeStep);
  for (auto& skel : mSkeletons) {
    skel->setTimeStep(_timeStep);
  }
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

  mSensorManager.resetSensors();
}

//==============================================================================
void World::step(bool _resetCommand)
{
  DART_PROFILE_FRAME;

  // Integrate velocity for unconstrained skeletons
  {
    DART_PROFILE_SCOPED_N("World::step - Integrate velocity");
    for (auto& skel : mSkeletons) {
      if (!skel->isMobile()) {
        continue;
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
  for (auto& skel : mSkeletons) {
    if (!skel->isMobile()) {
      continue;
    }

    if (skel->isImpulseApplied()) {
      skel->computeImpulseForwardDynamics();
      skel->setImpulseApplied(false);
    }

    if (skel->isPositionImpulseApplied()) {
      skel->integratePositions(mTimeStep, skel->getPositionVelocityChanges());
      skel->setPositionImpulseApplied(false);
      skel->clearPositionVelocityChanges();
    } else {
      skel->integratePositions(mTimeStep);
    }

    if (_resetCommand) {
      skel->clearInternalForces();
      skel->clearExternalForces();
      skel->resetCommands();
    }
  }

  mTime += mTimeStep;
  mFrame++;
  mSensorManager.updateSensors(*this);
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
const std::string& World::setName(std::string_view newName)
{
  if (newName == mName) {
    return mName;
  }

  const std::string oldName = mName;
  mName = newName;

  mNameChangedSignal.raise(oldName, mName);

  mNameMgrForSkeletons.setManagerName("World::Skeleton | " + mName);
  mNameMgrForSimpleFrames.setManagerName("World::SimpleFrame | " + mName);
  mSensorManager.setManagerName("World::Sensor | " + mName);

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
  if (_index < mSkeletons.size()) {
    return mSkeletons[_index];
  }

  return nullptr;
}

//==============================================================================
dynamics::SkeletonPtr World::getSkeleton(std::string_view name) const
{
  return mNameMgrForSkeletons.getObject(std::string(name));
}

//==============================================================================
std::size_t World::getNumSkeletons() const
{
  return mSkeletons.size();
}

//==============================================================================
std::string World::addSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  DART_THROW_T_IF(
      _skeleton == nullptr,
      common::NullPointerException,
      "Cannot add nullptr Skeleton to the world");

  // If mSkeletons already has _skeleton, then we do nothing.
  if (std::ranges::find(mSkeletons, _skeleton) != mSkeletons.end()) {
    DART_WARN(
        "Skeleton named [{}] is already in the world.", _skeleton->getName());
    return _skeleton->getName();
  }

  mSkeletons.push_back(_skeleton);
  mMapForSkeletons[_skeleton] = _skeleton;

  mNameConnectionsForSkeletons.push_back(_skeleton->onNameChanged.connect(
      [this](
          dynamics::ConstMetaSkeletonPtr skel,
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
  DART_THROW_T_IF(
      _skeleton == nullptr,
      common::NullPointerException,
      "Cannot remove nullptr Skeleton from the world");

  // Find index of _skeleton in mSkeleton.
  std::size_t index = 0;
  for (; index < mSkeletons.size(); ++index) {
    if (mSkeletons[index] == _skeleton) {
      break;
    }
  }

  // If i is equal to the number of skeletons, then _skeleton is not in
  // mSkeleton. We do nothing.
  if (index == mSkeletons.size()) {
    DART_WARN("Skeleton [{}] is not in the world.", _skeleton->getName());
    return;
  }

  // Update mIndices.
  for (std::size_t i = index + 1; i < mSkeletons.size() - 1; ++i) {
    mIndices[i] = mIndices[i + 1] - _skeleton->getNumDofs();
  }
  mIndices.pop_back();

  // Remove _skeleton from constraint handler.
  mConstraintSolver->removeSkeleton(_skeleton);

  // Remove _skeleton from mSkeletons
  std::erase(mSkeletons, _skeleton);

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
  for (const auto& skeleton : mSkeletons) {
    ptrs.insert(skeleton);
  }

  while (getNumSkeletons() > 0) {
    removeSkeleton(getSkeleton(0));
  }

  return ptrs;
}

//==============================================================================
bool World::hasSkeleton(const dynamics::ConstSkeletonPtr& skeleton) const
{
  return std::ranges::find(mSkeletons, skeleton) != mSkeletons.end();
}

//==============================================================================
bool World::hasSkeleton(std::string_view skeletonName) const
{
  return mNameMgrForSkeletons.hasName(std::string(skeletonName));
}

//==============================================================================
int World::getIndex(int _index) const
{
  if (_index < 0 || static_cast<std::size_t>(_index) >= mIndices.size()) {
    DART_ERROR(
        "World::getIndex: index [{}] is out of range. Valid range is [0, {}).",
        _index,
        mIndices.size());
    DART_ASSERT(false);
    return -1;
  }
  return mIndices[_index];
}

//==============================================================================
dynamics::SimpleFramePtr World::getSimpleFrame(std::size_t _index) const
{
  if (_index < mSimpleFrames.size()) {
    return mSimpleFrames[_index];
  }

  return nullptr;
}

//==============================================================================
dynamics::SimpleFramePtr World::getSimpleFrame(std::string_view name) const
{
  return mNameMgrForSimpleFrames.getObject(std::string(name));
}

//==============================================================================
std::size_t World::getNumSimpleFrames() const
{
  return mSimpleFrames.size();
}

//==============================================================================
std::string World::addSimpleFrame(const dynamics::SimpleFramePtr& _frame)
{
  DART_THROW_T_IF(
      _frame == nullptr,
      common::NullPointerException,
      "Cannot add nullptr SimpleFrame to the world");

  if (std::ranges::find(mSimpleFrames, _frame) != mSimpleFrames.end()) {
    DART_WARN(
        "SimpleFrame named [{}] is already in the world.", _frame->getName());
    return _frame->getName();
  }

  mSimpleFrames.push_back(_frame);
  mSimpleFrameToShared[_frame.get()] = _frame;

  mNameConnectionsForSimpleFrames.push_back(_frame->onNameChanged.connect(
      [this](
          const dynamics::Entity* _entity,
          const std::string&,
          const std::string&) { this->handleSimpleFrameNameChange(_entity); }));

  _frame->setName(
      mNameMgrForSimpleFrames.issueNewNameAndAdd(_frame->getName(), _frame));

  return _frame->getName();
}

//==============================================================================
void World::removeSimpleFrame(const dynamics::SimpleFramePtr& _frame)
{
  DART_THROW_T_IF(
      _frame == nullptr,
      common::NullPointerException,
      "Cannot remove nullptr SimpleFrame from the world");

  auto it = std::ranges::find(mSimpleFrames, _frame);

  if (it == mSimpleFrames.end()) {
    DART_WARN("Frame named [{}] is not in the world.", _frame->getName());
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
       ++it) {
    ptrs.insert(*it);
  }

  while (getNumSimpleFrames() > 0) {
    removeSimpleFrame(getSimpleFrame(0));
  }

  return ptrs;
}

//==============================================================================
sensor::SensorPtr World::getSensor(std::size_t index) const
{
  return mSensorManager.getSensor(index);
}

//==============================================================================
sensor::SensorPtr World::getSensor(std::string_view name) const
{
  return mSensorManager.getSensor(name);
}

//==============================================================================
std::size_t World::getNumSensors() const
{
  return mSensorManager.getNumSensors();
}

//==============================================================================
std::string World::addSensor(const sensor::SensorPtr& sensor)
{
  return mSensorManager.addSensor(sensor);
}

//==============================================================================
void World::removeSensor(const sensor::SensorPtr& sensor)
{
  mSensorManager.removeSensor(sensor);
}

//==============================================================================
std::set<sensor::SensorPtr> World::removeAllSensors()
{
  return mSensorManager.removeAllSensors();
}

//==============================================================================
bool World::hasSensor(const sensor::SensorPtr& sensor) const
{
  return mSensorManager.hasSensor(sensor);
}

//==============================================================================
bool World::hasSensor(std::string_view sensorName) const
{
  return mSensorManager.hasSensor(sensorName);
}

//==============================================================================
sensor::SensorManager& World::getSensorManager()
{
  return mSensorManager;
}

//==============================================================================
const sensor::SensorManager& World::getSensorManager() const
{
  return mSensorManager;
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
void World::setCollisionDetector(
    const collision::CollisionDetectorPtr& collisionDetector)
{
  if (!collisionDetector) {
    DART_WARN(
        "Attempted to assign a null collision detector to world '{}'.", mName);
    return;
  }

  mConstraintSolver->setCollisionDetector(collisionDetector);
}

//==============================================================================
void World::setCollisionDetector(CollisionDetectorType collisionDetector)
{
  auto detector = tryCreateCollisionDetector(collisionDetector);
  if (!detector) {
    auto current = mConstraintSolver->getCollisionDetector();
    DART_WARN(
        "Collision detector '{}' is not available for world '{}'. Keeping the "
        "current detector '{}'.",
        toCollisionDetectorKey(collisionDetector),
        mName,
        current ? current->getTypeView() : "unknown");
    return;
  }

  setCollisionDetector(detector);
}

//==============================================================================
collision::CollisionDetectorPtr World::getCollisionDetector()
{
  return mConstraintSolver->getCollisionDetector();
}

//==============================================================================
collision::ConstCollisionDetectorPtr World::getCollisionDetector() const
{
  return mConstraintSolver->getCollisionDetector();
}

//==============================================================================
void World::setConstraintSolver(constraint::UniqueConstraintSolverPtr solver)
{
  DART_THROW_T_IF(
      !solver,
      common::NullPointerException,
      "Cannot set nullptr as constraint solver");

  if (mConstraintSolver) {
    solver->setFromOtherConstraintSolver(*mConstraintSolver);
  }

  mConstraintSolver = std::move(solver);
  mConstraintSolver->setTimeStep(mTimeStep);
  mConstraintSolver->setFrameAllocator(&mMemoryManager.getFrameAllocator());
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
common::MemoryManager& World::getMemoryManager()
{
  return mMemoryManager;
}

//==============================================================================
const common::MemoryManager& World::getMemoryManager() const
{
  return mMemoryManager;
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
    DART_ERROR(
        "Received a name change callback for a nullptr Skeleton. This is most "
        "likely a bug. Please report this!");
    DART_ASSERT(false);
    return;
  }

  // Get the new name of the Skeleton
  const std::string& newName = _skeleton->getName();

  // Find the shared version of the Skeleton
  std::map<dynamics::ConstMetaSkeletonPtr, dynamics::SkeletonPtr>::iterator it
      = mMapForSkeletons.find(_skeleton);
  if (it == mMapForSkeletons.end()) {
    DART_ERROR(
        "Could not find Skeleton named [{}] in the shared_ptr map of World "
        "[{}]. This is most likely a bug. Please report this!",
        _skeleton->getName(),
        getName());
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
    DART_ERROR(
        "Skeleton named [{}] ({}) does not exist in the NameManager of World "
        "[{}]. This is most likely a bug. Please report this!",
        sharedSkel->getName(),
        sharedSkel,
        getName());
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
    DART_ERROR(
        "Received a callback for a nullptr entity. This is most likely a bug. "
        "Please report this!");
    DART_ASSERT(false);
    return;
  }

  // Get the new name of the Frame
  const std::string& newName = frame->getName();

  // Find the shared version of the Frame
  std::map<const dynamics::SimpleFrame*, dynamics::SimpleFramePtr>::iterator it
      = mSimpleFrameToShared.find(frame);
  if (it == mSimpleFrameToShared.end()) {
    DART_ERROR(
        "Could not find SimpleFrame named [{}] in the shared_ptr map of World "
        "[{}]. This is most likely a bug. Please report this!",
        frame->getName(),
        getName());
    DART_ASSERT(false);
    return;
  }
  dynamics::SimpleFramePtr sharedFrame = it->second;

  std::string issuedName
      = mNameMgrForSimpleFrames.changeObjectName(sharedFrame, newName);

  if ((!issuedName.empty()) && (newName != issuedName)) {
    sharedFrame->setName(issuedName);
  } else if (issuedName.empty()) {
    DART_ERROR(
        "SimpleFrame named [{}] ({}) does not exist in the NameManager of "
        "World [{}]. This is most likely a bug. Please report this!",
        frame->getName(),
        frame,
        getName());
    DART_ASSERT(false);
    return;
  }
}

} // namespace simulation
} // namespace dart
