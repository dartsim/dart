/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/fcl/FCLCollisionDetector.hpp"
#include "dart/common/Diagnostics.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Profile.hpp"
#include "dart/common/String.hpp"
#include "dart/constraint/ConstrainedGroup.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/detail/WorldEcsAccess.hpp"
#include "dart/simulation/solver/classic_rigid/ClassicRigidSolver.hpp"
#include "dart/simulation/solver/rigid/RigidSolver.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <cmath>

namespace dart {
namespace simulation {

struct World::EcsData final
{
  entt::registry entityManager;
};

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

} // namespace

//==============================================================================
std::shared_ptr<World> World::create(const std::string& name)
{
  return std::make_shared<World>(name);
}

//==============================================================================
std::shared_ptr<World> World::create(const WorldConfig& config)
{
  return std::make_shared<World>(config);
}

//==============================================================================
World::World(const std::string& _name) : World(WorldConfig(_name)) {}

//==============================================================================
World::World(const WorldConfig& config)
  : mName(config.name),
    mNameMgrForSkeletons("World::Skeleton | " + config.name, "skeleton"),
    mNameMgrForSimpleFrames("World::SimpleFrame | " + config.name, "frame"),
    mGravity(0.0, 0.0, -9.81),
    mTimeStep(0.001),
    mTime(0.0),
    mFrame(0),
    mRecording(new Recording(mSkeletons)),
    mEcsData(std::make_unique<EcsData>()),
    mSkeletonSolverType(config.solverRouting.skeletons),
    mObjectSolverType(config.solverRouting.objects),
    onNameChanged(mNameChangedSignal)
{
  mIndices.push_back(0);

  addSolver(std::make_unique<ClassicRigidSolver>());
  addSolver(std::make_unique<RigidSolver>());

  if (auto* collisionSolver = getCollisionCapableSolver()) {
    if (auto detector = resolveCollisionDetector(config)) {
      collisionSolver->setCollisionDetector(detector);
    }
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
  WorldConfig config(mName);
  config.solverRouting.skeletons = mSkeletonSolverType;
  config.solverRouting.objects = mObjectSolverType;

  WorldPtr worldClone = World::create(config);

  worldClone->setGravity(mGravity);
  worldClone->setTimeStep(mTimeStep);

  if (auto* constraintSolver = getConstraintSolver()) {
    auto cd = constraintSolver->getCollisionDetector();
    if (cd) {
      worldClone->setCollisionDetector(cd->cloneWithoutCollisionObjects());
    }
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
  for (auto& entry : mSolvers) {
    entry.solver->setTimeStep(_timeStep);
  }
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
  for (auto& entry : mSolvers) {
    entry.solver->reset(*this);
  }
  for (auto& skel : mSkeletons) {
    skel->clearConstraintImpulses();
    skel->setImpulseApplied(false);
  }
}

//==============================================================================
void World::step(bool _resetCommand)
{
  DART_PROFILE_FRAME;

  std::size_t steppedSolvers = 0;

  for (auto& entry : mSolvers) {
    if (!entry.enabled) {
      continue;
    }
    entry.solver->step(*this, _resetCommand);
    ++steppedSolvers;
  }

  if (steppedSolvers == 0) {
    DART_WARN(
        "World '{}' has no enabled solvers. Skipping simulation step.", mName);
    return;
  }

  mTime += mTimeStep;
  mFrame++;
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
  if (_newName == mName) {
    return mName;
  }

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
  if (_index < mSkeletons.size()) {
    return mSkeletons[_index];
  }

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
    DART_WARN("Attempting to add a nullptr Skeleton to the world!");
    return "";
  }

  // If mSkeletons already has _skeleton, then we do nothing.
  if (find(mSkeletons.begin(), mSkeletons.end(), _skeleton)
      != mSkeletons.end()) {
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

  auto* skeletonSolver = getSkeletonSolver();
  if (skeletonSolver) {
    if (skeletonSolver->supportsSkeletons()) {
      skeletonSolver->handleSkeletonAdded(*this, _skeleton);
    } else {
      DART_WARN(
          "World '{}' routes Skeletons to solver '{}', but it does not "
          "support Skeletons.",
          mName,
          skeletonSolver->getName());
    }
  } else {
    DART_WARN(
        "World '{}' routes Skeletons to solver type {}, but no matching "
        "solver is registered.",
        mName,
        static_cast<int>(mSkeletonSolverType));
  }

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
    DART_WARN("Attempting to remove a nullptr Skeleton from the world!");
    return;
  }

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

  // Notify the configured skeleton solver.
  auto* skeletonSolver = getSkeletonSolver();
  if (skeletonSolver) {
    if (skeletonSolver->supportsSkeletons()) {
      skeletonSolver->handleSkeletonRemoved(*this, _skeleton);
    } else {
      DART_WARN(
          "World '{}' routes Skeletons to solver '{}', but it does not "
          "support Skeletons.",
          mName,
          skeletonSolver->getName());
    }
  } else {
    DART_WARN(
        "World '{}' routes Skeletons to solver type {}, but no matching "
        "solver is registered.",
        mName,
        static_cast<int>(mSkeletonSolverType));
  }

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
       ++it) {
    ptrs.insert(*it);
  }

  while (getNumSkeletons() > 0) {
    removeSkeleton(getSkeleton(0));
  }

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
    DART_WARN("Attempting to add a nullptr SimpleFrame to the world!");
    return "";
  }

  if (find(mSimpleFrames.begin(), mSimpleFrames.end(), _frame)
      != mSimpleFrames.end()) {
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
  DART_ASSERT(
      _frame != nullptr
      && "Attempted to remove nullptr SimpleFrame from world");

  std::vector<dynamics::SimpleFramePtr>::iterator it
      = find(mSimpleFrames.begin(), mSimpleFrames.end(), _frame);

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
bool World::checkCollision(
    const collision::CollisionOption& option,
    collision::CollisionResult* result)
{
  auto* solver = getCollisionCapableSolver();
  if (!solver) {
    DART_WARN("World '{}' does not have a collision-capable solver.", mName);
    return false;
  }

  return solver->checkCollision(option, result);
}

//==============================================================================
const collision::CollisionResult& World::getLastCollisionResult() const
{
  auto* solver = getCollisionCapableSolver();
  if (!solver) {
    DART_WARN("World '{}' does not have a collision-capable solver.", mName);
    static const collision::CollisionResult emptyResult;
    return emptyResult;
  }

  return solver->getLastCollisionResult();
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

  auto* solver = getCollisionCapableSolver();
  if (!solver) {
    DART_WARN(
        "World '{}' does not have a collision-capable solver. Ignoring "
        "collision detector assignment.",
        mName);
    return;
  }

  solver->setCollisionDetector(collisionDetector);
}

//==============================================================================
void World::setCollisionDetector(CollisionDetectorType collisionDetector)
{
  auto detector = tryCreateCollisionDetector(collisionDetector);
  if (!detector) {
    auto* solver = getCollisionCapableSolver();
    auto current = solver ? solver->getCollisionDetector() : nullptr;
    DART_WARN(
        "Collision detector '{}' is not available for world '{}'. Keeping the "
        "current detector '{}'.",
        toCollisionDetectorKey(collisionDetector),
        mName,
        current ? current->getType() : "unknown");
    return;
  }

  setCollisionDetector(detector);
}

//==============================================================================
collision::CollisionDetectorPtr World::getCollisionDetector()
{
  auto* solver = getCollisionCapableSolver();
  return solver ? solver->getCollisionDetector() : nullptr;
}

//==============================================================================
collision::ConstCollisionDetectorPtr World::getCollisionDetector() const
{
  auto* solver = getCollisionCapableSolver();
  return solver ? solver->getCollisionDetector() : nullptr;
}

//==============================================================================
void World::setConstraintSolver(constraint::UniqueConstraintSolverPtr solver)
{
  auto* constraintSolver = getConstraintCapableSolver();
  if (!constraintSolver) {
    DART_ERROR("No constraint-capable solver has been registered.");
    return;
  }

  constraintSolver->setConstraintSolver(std::move(solver));
}

//==============================================================================
constraint::ConstraintSolver* World::getConstraintSolver()
{
  auto* solver = getConstraintCapableSolver();
  return solver ? solver->getConstraintSolver() : nullptr;
}

//==============================================================================
const constraint::ConstraintSolver* World::getConstraintSolver() const
{
  auto* solver = getConstraintCapableSolver();
  return solver ? solver->getConstraintSolver() : nullptr;
}

//==============================================================================
void World::bake()
{
  const auto* constraintSolver = getConstraintSolver();
  if (!constraintSolver) {
    DART_WARN("Cannot bake state because no constraint-capable solver exists.");
    return;
  }

  const auto collisionResult = constraintSolver->getLastCollisionResult();
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
entt::registry& detail::WorldEcsAccess::getEntityManager(World& world)
{
  DART_ASSERT(world.mEcsData);
  return world.mEcsData->entityManager;
}

//==============================================================================
const entt::registry& detail::WorldEcsAccess::getEntityManager(
    const World& world)
{
  DART_ASSERT(world.mEcsData);
  return world.mEcsData->entityManager;
}

//==============================================================================
EcsEntity detail::WorldEcsAccess::createEntity(World& world)
{
  if (!world.mEcsData) {
    DART_WARN(
        "World '{}' is missing ECS storage; cannot create ECS entity.",
        world.getName());
    return EcsEntity();
  }

  const auto entity = world.mEcsData->entityManager.create();
  const auto ecsEntity = toEcsEntity(entity);

  auto* objectSolver = world.getObjectSolver();
  if (objectSolver) {
    objectSolver->handleEntityAdded(world, ecsEntity);
  } else {
    DART_WARN(
        "World '{}' routes ECS objects to solver type {}, but no matching "
        "solver is registered.",
        world.getName(),
        static_cast<int>(world.mObjectSolverType));
  }

  return ecsEntity;
}

//==============================================================================
bool detail::WorldEcsAccess::destroyEntity(World& world, EcsEntity entity)
{
  if (!world.mEcsData) {
    return false;
  }

  const auto enttEntity = toEntt(entity);
  if (enttEntity == entt::null) {
    return false;
  }

  auto& registry = world.mEcsData->entityManager;
  if (!registry.valid(enttEntity)) {
    return false;
  }

  auto* objectSolver = world.getObjectSolver();
  if (objectSolver) {
    objectSolver->handleEntityRemoved(world, entity);
  } else {
    DART_WARN(
        "World '{}' routes ECS objects to solver type {}, but no matching "
        "solver is registered.",
        world.getName(),
        static_cast<int>(world.mObjectSolverType));
  }

  registry.destroy(enttEntity);
  return true;
}

//==============================================================================
bool detail::WorldEcsAccess::isEntityValid(const World& world, EcsEntity entity)
{
  if (!world.mEcsData) {
    return false;
  }

  const auto enttEntity = toEntt(entity);
  if (enttEntity == entt::null) {
    return false;
  }

  return world.mEcsData->entityManager.valid(enttEntity);
}

//==============================================================================
entt::entity detail::WorldEcsAccess::toEntt(EcsEntity entity)
{
  if (entity.isNull()) {
    return entt::null;
  }

  using underlying = std::underlying_type_t<entt::entity>;
  const auto maxValue = static_cast<EcsEntity::ValueType>(
      std::numeric_limits<underlying>::max());
  const auto rawValue = entity.value() - 1u;

  if (rawValue > maxValue) {
    DART_ASSERT(
        false && "EcsEntity value does not fit in entt::entity storage.");
    return entt::null;
  }

  return static_cast<entt::entity>(static_cast<underlying>(rawValue));
}

//==============================================================================
EcsEntity detail::WorldEcsAccess::toEcsEntity(entt::entity entity)
{
  if (entity == entt::null) {
    return EcsEntity();
  }

  using underlying = std::underlying_type_t<entt::entity>;
  return EcsEntity{
      static_cast<EcsEntity::ValueType>(static_cast<underlying>(entity)) + 1u};
}

//==============================================================================
Solver* World::addSolver(std::unique_ptr<Solver> solver)
{
  return addSolver(std::move(solver), true);
}

//==============================================================================
Solver* World::addSolver(std::unique_ptr<Solver> solver, bool enabled)
{
  if (!solver) {
    DART_WARN("Attempted to add a null solver to world '{}'.", mName);
    return nullptr;
  }

  solver->setTimeStep(mTimeStep);
  mSolvers.emplace_back(SolverEntry{std::move(solver), enabled});
  auto* solverPtr = mSolvers.back().solver.get();

  const auto solverRigidType = solverPtr->getRigidSolverType();
  if (solverRigidType && *solverRigidType == mSkeletonSolverType
      && getSkeletonSolver() == solverPtr) {
    if (solverPtr->supportsSkeletons()) {
      for (auto& skeleton : mSkeletons) {
        solverPtr->handleSkeletonAdded(*this, skeleton);
      }
    } else {
      DART_WARN(
          "World '{}' routes Skeletons to solver '{}', but it does not "
          "support Skeletons.",
          mName,
          solverPtr->getName());
    }
  }

  if (solverRigidType && *solverRigidType == mObjectSolverType
      && getObjectSolver() == solverPtr) {
    if (mEcsData) {
      auto& registry = mEcsData->entityManager;
      auto& entities = registry.storage<entt::entity>();
      for (const auto entity : entities) {
        solverPtr->handleEntityAdded(
            *this, detail::WorldEcsAccess::toEcsEntity(entity));
      }
    }
  }

  if (enabled) {
    solverPtr->sync(*this);
  }

  return solverPtr;
}

//==============================================================================
std::size_t World::getNumSolvers() const
{
  return mSolvers.size();
}

//==============================================================================
Solver* World::getSolver(std::size_t index)
{
  if (index >= mSolvers.size()) {
    return nullptr;
  }
  return mSolvers[index].solver.get();
}

//==============================================================================
const Solver* World::getSolver(std::size_t index) const
{
  if (index >= mSolvers.size()) {
    return nullptr;
  }
  return mSolvers[index].solver.get();
}

//==============================================================================
std::size_t World::getSolverIndex(const Solver* solver) const
{
  if (!solver) {
    return mSolvers.size();
  }

  for (std::size_t index = 0; index < mSolvers.size(); ++index) {
    if (mSolvers[index].solver.get() == solver) {
      return index;
    }
  }

  return mSolvers.size();
}

//==============================================================================
Solver* World::getSolver(RigidSolverType type)
{
  for (auto& entry : mSolvers) {
    const auto rigidType = entry.solver->getRigidSolverType();
    if (rigidType && *rigidType == type) {
      return entry.solver.get();
    }
  }
  return nullptr;
}

//==============================================================================
const Solver* World::getSolver(RigidSolverType type) const
{
  for (const auto& entry : mSolvers) {
    const auto rigidType = entry.solver->getRigidSolverType();
    if (rigidType && *rigidType == type) {
      return entry.solver.get();
    }
  }
  return nullptr;
}

//==============================================================================
Solver* World::getSolver(const std::string& name)
{
  for (auto& entry : mSolvers) {
    if (entry.solver->getName() == name) {
      return entry.solver.get();
    }
  }
  return nullptr;
}

//==============================================================================
const Solver* World::getSolver(const std::string& name) const
{
  for (const auto& entry : mSolvers) {
    if (entry.solver->getName() == name) {
      return entry.solver.get();
    }
  }
  return nullptr;
}

//==============================================================================
Solver* World::getSkeletonSolver()
{
  return getSolver(mSkeletonSolverType);
}

//==============================================================================
const Solver* World::getSkeletonSolver() const
{
  return getSolver(mSkeletonSolverType);
}

//==============================================================================
Solver* World::getObjectSolver()
{
  return getSolver(mObjectSolverType);
}

//==============================================================================
const Solver* World::getObjectSolver() const
{
  return getSolver(mObjectSolverType);
}

//==============================================================================
Solver* World::getActiveRigidSolver()
{
  auto* solver = getSkeletonSolver();
  if (!solver) {
    return nullptr;
  }

  if (!isSolverEnabled(solver)) {
    return nullptr;
  }

  return solver;
}

//==============================================================================
const Solver* World::getActiveRigidSolver() const
{
  auto* solver = getSkeletonSolver();
  if (!solver) {
    return nullptr;
  }

  if (!isSolverEnabled(solver)) {
    return nullptr;
  }

  return solver;
}

//==============================================================================
bool World::setSolverEnabled(std::size_t index, bool enabled)
{
  if (index >= mSolvers.size()) {
    return false;
  }

  auto& entry = mSolvers[index];
  if (entry.enabled == enabled) {
    return true;
  }

  entry.enabled = enabled;
  if (enabled) {
    entry.solver->sync(*this);
  }

  return true;
}

//==============================================================================
bool World::isSolverEnabled(std::size_t index) const
{
  if (index >= mSolvers.size()) {
    return false;
  }

  return mSolvers[index].enabled;
}

//==============================================================================
bool World::setSolverEnabled(Solver* solver, bool enabled)
{
  if (!solver) {
    return false;
  }

  for (std::size_t i = 0; i < mSolvers.size(); ++i) {
    if (mSolvers[i].solver.get() == solver) {
      return setSolverEnabled(i, enabled);
    }
  }

  return false;
}

//==============================================================================
bool World::isSolverEnabled(const Solver* solver) const
{
  if (!solver) {
    return false;
  }

  for (const auto& entry : mSolvers) {
    if (entry.solver.get() == solver) {
      return entry.enabled;
    }
  }

  return false;
}

//==============================================================================
bool World::moveSolver(std::size_t fromIndex, std::size_t toIndex)
{
  if (fromIndex >= mSolvers.size() || toIndex >= mSolvers.size()) {
    return false;
  }

  Solver* previousSkeletonSolver = getSkeletonSolver();
  Solver* previousObjectSolver = getObjectSolver();

  if (fromIndex == toIndex) {
    return true;
  }

  auto begin = mSolvers.begin();
  if (fromIndex < toIndex) {
    std::rotate(begin + fromIndex, begin + fromIndex + 1, begin + toIndex + 1);
  } else {
    std::rotate(begin + toIndex, begin + fromIndex, begin + fromIndex + 1);
  }

  Solver* currentSkeletonSolver = getSkeletonSolver();
  if (currentSkeletonSolver != previousSkeletonSolver) {
    if (previousSkeletonSolver && previousSkeletonSolver->supportsSkeletons()) {
      for (auto& skeleton : mSkeletons) {
        previousSkeletonSolver->handleSkeletonRemoved(*this, skeleton);
      }
    }

    if (currentSkeletonSolver) {
      if (currentSkeletonSolver->supportsSkeletons()) {
        for (auto& skeleton : mSkeletons) {
          currentSkeletonSolver->handleSkeletonAdded(*this, skeleton);
        }
      } else {
        DART_WARN(
            "World '{}' routes Skeletons to solver '{}', but it does not "
            "support Skeletons.",
            mName,
            currentSkeletonSolver->getName());
      }
    }
  }

  Solver* currentObjectSolver = getObjectSolver();
  if (currentObjectSolver != previousObjectSolver && mEcsData) {
    auto& registry = mEcsData->entityManager;
    auto& entities = registry.storage<entt::entity>();
    if (previousObjectSolver) {
      for (const auto entity : entities) {
        previousObjectSolver->handleEntityRemoved(
            *this, detail::WorldEcsAccess::toEcsEntity(entity));
      }
    }

    if (currentObjectSolver) {
      for (const auto entity : entities) {
        currentObjectSolver->handleEntityAdded(
            *this, detail::WorldEcsAccess::toEcsEntity(entity));
      }
    }
  }

  return true;
}

//==============================================================================
Solver* World::getConstraintCapableSolver()
{
  auto* activeSolver = getActiveRigidSolver();
  if (!activeSolver) {
    return nullptr;
  }

  if (activeSolver->supportsConstraints()) {
    return activeSolver;
  }

  return nullptr;
}

//==============================================================================
const Solver* World::getConstraintCapableSolver() const
{
  const auto* activeSolver = getActiveRigidSolver();
  if (!activeSolver) {
    return nullptr;
  }

  if (activeSolver->supportsConstraints()) {
    return activeSolver;
  }

  return nullptr;
}

//==============================================================================
Solver* World::getCollisionCapableSolver()
{
  auto* activeSolver = getActiveRigidSolver();
  if (!activeSolver) {
    return nullptr;
  }

  if (activeSolver->supportsCollision()) {
    return activeSolver;
  }

  return nullptr;
}

//==============================================================================
const Solver* World::getCollisionCapableSolver() const
{
  const auto* activeSolver = getActiveRigidSolver();
  if (!activeSolver) {
    return nullptr;
  }

  if (activeSolver->supportsCollision()) {
    return activeSolver;
  }

  return nullptr;
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
