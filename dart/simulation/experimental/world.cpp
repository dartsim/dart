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

#include "dart/simulation/experimental/world.hpp"

#include "dart/collision/native/collision_object.hpp"
#include "dart/collision/native/collision_world.hpp"
#include "dart/collision/native/contact_manifold.hpp"
#include "dart/collision/native/contact_point.hpp"
#include "dart/collision/native/shapes/shape.hpp"
#include "dart/collision/native/types.hpp"
#include "dart/simulation/experimental/body/contact.hpp"
#include "dart/simulation/experimental/body/rigid_body.hpp"
#include "dart/simulation/experimental/common/ecs_utils.hpp"
#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/all.hpp"
#include "dart/simulation/experimental/compute/multibody_dynamics.hpp"
#include "dart/simulation/experimental/compute/sequential_executor.hpp"
#include "dart/simulation/experimental/compute/world_kinematics_graph.hpp"
#include "dart/simulation/experimental/compute/world_step_stage.hpp"
#include "dart/simulation/experimental/constraint/loop_closure.hpp"
#include "dart/simulation/experimental/constraint/loop_closure_spec.hpp"
#include "dart/simulation/experimental/frame/fixed_frame.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/frame/free_frame.hpp"
#include "dart/simulation/experimental/io/binary_io.hpp"
#include "dart/simulation/experimental/io/serializer.hpp"
#include "dart/simulation/experimental/multibody/multibody.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <format>
#include <istream>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace {

template <typename... Components>
std::size_t countEntities(const entt::registry& registry)
{
  std::size_t count = 0;
  auto view = registry.view<Components...>();
  for (auto entity : view) {
    (void)entity;
    ++count;
  }
  return count;
}

template <typename Component>
bool hasEntityWithName(const entt::registry& registry, std::string_view name)
{
  auto view
      = registry.view<Component, dart::simulation::experimental::comps::Name>();
  for (auto entity : view) {
    const auto& info
        = view.template get<dart::simulation::experimental::comps::Name>(
            entity);
    if (info.name == name) {
      return true;
    }
  }
  return false;
}

} // namespace

namespace dart::simulation::experimental {

namespace {

//==============================================================================
void executeKinematicsGraph(World& world, compute::ComputeExecutor& executor)
{
  compute::WorldKinematicsGraph graph(world);
  graph.execute(executor);
}

//==============================================================================
bool isValidWorldSyncStage(WorldSyncStage stage)
{
  switch (stage) {
    case WorldSyncStage::Kinematics:
      return true;
  }

  return false;
}

//==============================================================================
void validateLoopClosureKinematicsPolicySupport(const World& world)
{
  auto view = world.getRegistry().view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& closure = view.get<comps::LoopClosure>(entity);
    if (!closure.runtimePolicy.enabled
        || closure.runtimePolicy.kinematics
               == ClosureKinematicsPolicy::ResidualOnly) {
      continue;
    }

    const auto& name = view.get<comps::Name>(entity);
    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException,
        "LoopClosure '{}' requests kinematic projection, but the active "
        "pipeline does not include a loop-closure projection stage",
        name.name);
  }
}

//==============================================================================
void validateLoopClosureDynamicsPolicySupport(const World& world)
{
  auto view = world.getRegistry().view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& closure = view.get<comps::LoopClosure>(entity);
    if (!closure.runtimePolicy.enabled
        || closure.runtimePolicy.dynamics
               == ClosureDynamicsPolicy::ResidualOnly) {
      continue;
    }

    const auto& name = view.get<comps::Name>(entity);
    DART_EXPERIMENTAL_THROW_T(
        InvalidOperationException,
        "LoopClosure '{}' requests dynamic solving, but the active pipeline "
        "does not include a loop-closure solving stage",
        name.name);
  }
}

//==============================================================================
bool isSymmetricPositiveDefinite(const Eigen::Matrix3d& matrix)
{
  if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-12)) {
    return false;
  }

  Eigen::LLT<Eigen::Matrix3d> factorization(matrix);
  return factorization.info() == Eigen::Success;
}

//==============================================================================
void validateFiniteVector(
    const Eigen::Vector3d& value, std::string_view fieldName)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !value.allFinite(),
      InvalidArgumentException,
      "RigidBodyOptions.{} must contain only finite values",
      fieldName);
}

//==============================================================================
bool isValidLoopClosureFamily(LoopClosureFamily family)
{
  switch (family) {
    case LoopClosureFamily::Rigid:
    case LoopClosureFamily::Point:
    case LoopClosureFamily::Distance:
      return true;
  }

  return false;
}

//==============================================================================
void validateLoopClosureOffset(
    const Eigen::Isometry3d& offset, std::string_view fieldName)
{
  constexpr double tolerance = 1e-9;

  DART_EXPERIMENTAL_THROW_T_IF(
      !offset.matrix().allFinite(),
      InvalidArgumentException,
      "LoopClosureSpec.{} must contain only finite values",
      fieldName);

  const auto& rotation = offset.linear();
  const double orthonormalError
      = (rotation * rotation.transpose() - Eigen::Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff();
  DART_EXPERIMENTAL_THROW_T_IF(
      orthonormalError > tolerance
          || std::abs(rotation.determinant() - 1.0) > tolerance,
      InvalidArgumentException,
      "LoopClosureSpec.{} rotation must be orthonormal",
      fieldName);
}

//==============================================================================
entt::entity resolveLoopClosureFrame(
    const World& world, const Frame& frame, std::string_view fieldName)
{
  if (frame.isWorld()) {
    return entt::null;
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !frame.isValid(),
      InvalidArgumentException,
      "LoopClosureSpec.{} is invalid or has been destroyed",
      fieldName);

  DART_EXPERIMENTAL_THROW_T_IF(
      frame.getWorld() != &world,
      InvalidArgumentException,
      "LoopClosureSpec.{} belongs to a different world",
      fieldName);

  return frame.getEntity();
}

//==============================================================================
void validateLoopClosureSpec(const World& world, const LoopClosureSpec& spec)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidLoopClosureFamily(spec.family),
      InvalidArgumentException,
      "LoopClosureSpec.family is invalid");

  const auto frameA = resolveLoopClosureFrame(world, spec.frameA, "frameA");
  const auto frameB = resolveLoopClosureFrame(world, spec.frameB, "frameB");
  DART_EXPERIMENTAL_THROW_T_IF(
      frameA == frameB,
      InvalidArgumentException,
      "LoopClosureSpec endpoints must be distinct frames");

  validateLoopClosureOffset(spec.offsetA, "offsetA");
  validateLoopClosureOffset(spec.offsetB, "offsetB");
}

//==============================================================================
void validateRigidBodyOptions(const RigidBodyOptions& options)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(options.mass) || options.mass <= 0.0,
      InvalidArgumentException,
      "RigidBodyOptions.mass must be positive and finite");

  DART_EXPERIMENTAL_THROW_T_IF(
      !isSymmetricPositiveDefinite(options.inertia),
      InvalidArgumentException,
      "RigidBodyOptions.inertia must be symmetric positive definite");

  validateFiniteVector(options.position, "position");
  validateFiniteVector(options.linearVelocity, "linearVelocity");
  validateFiniteVector(options.angularVelocity, "angularVelocity");

  const auto orientationNorm = options.orientation.norm();
  DART_EXPERIMENTAL_THROW_T_IF(
      !options.orientation.coeffs().allFinite()
          || !std::isfinite(orientationNorm) || orientationNorm <= 0.0,
      InvalidArgumentException,
      "RigidBodyOptions.orientation must be finite and non-zero");
}

//==============================================================================
Eigen::Quaterniond normalizeOrIdentity(const Eigen::Quaterniond& orientation)
{
  const auto norm = orientation.norm();
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }

  auto normalized = orientation;
  normalized.coeffs() /= norm;
  return normalized;
}

//==============================================================================
Eigen::Isometry3d toIsometry(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  transform.linear() = orientation.toRotationMatrix();
  return transform;
}

} // namespace

World::World() = default;

//==============================================================================
World::~World() = default;

//==============================================================================
entt::registry& World::getRegistry()
{
  return m_registry;
}

//==============================================================================
const entt::registry& World::getRegistry() const
{
  return m_registry;
}

//==============================================================================
void World::clear()
{
  m_registry.clear();
  m_simulationMode = false;
  m_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  m_timeStep = 0.001;
  m_time = 0.0;
  m_frame = 0;
  m_freeFrameCounter = 0;
  m_fixedFrameCounter = 0;
  m_multibodyCounter = 0;
  m_loopClosureCounter = 0;
  m_rigidBodyCounter = 0;
  m_linkCounter = 0;
  m_jointCounter = 0;
}

//==============================================================================
void World::ensureDesignMode() const
{
  DART_EXPERIMENTAL_THROW_T_IF(
      m_simulationMode,
      InvalidOperationException,
      "World modifications are not allowed while in simulation mode");
}

//==============================================================================
FreeFrame World::addFreeFrame()
{
  return addFreeFrame("", Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name)
{
  return addFreeFrame(name, Frame::world());
}

//==============================================================================
FreeFrame World::addFreeFrame(std::string_view name, const Frame& parent)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      Eigen::Isometry3d::Identity(),
      &m_freeFrameCounter,
      "free_frame",
      false,
      actualName);

  return FreeFrame(entity, this);
}

//==============================================================================
FixedFrame World::addFixedFrame(std::string_view name, const Frame& parent)
{
  return addFixedFrame(name, parent, Eigen::Isometry3d::Identity());
}

//==============================================================================
FixedFrame World::addFixedFrame(
    std::string_view name, const Frame& parent, const Eigen::Isometry3d& offset)
{
  ensureDesignMode();
  Frame parentFrame = resolveParentFrame(parent);

  DART_EXPERIMENTAL_THROW_T_IF(
      name.empty(),
      InvalidArgumentException,
      "FixedFrame requires a non-empty name");

  DART_EXPERIMENTAL_THROW_T_IF(
      parentFrame.isWorld(),
      InvalidArgumentException,
      "FixedFrame cannot be attached directly to the world frame");

  std::string actualName;
  auto entity = createFrameEntity(
      name,
      parentFrame,
      offset,
      &m_fixedFrameCounter,
      "fixed_frame",
      true,
      actualName);

  return FixedFrame(entity, this);
}

//==============================================================================
entt::entity World::createFrameEntity(
    std::string_view name,
    const Frame& parentFrame,
    const Eigen::Isometry3d& localTransform,
    std::size_t* autoNameCounter,
    std::string_view autoNamePrefix,
    bool isFixedFrame,
    std::string& outName)
{
  std::string actualName;
  if (name.empty()) {
    if (autoNameCounter) {
      actualName
          = std::format("{}_{:03d}", autoNamePrefix, ++(*autoNameCounter));
    } else {
      actualName = std::string(autoNamePrefix);
    }
  } else {
    actualName = std::string(name);
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, actualName);
  m_registry.emplace<comps::FrameTag>(entity);

  if (isFixedFrame) {
    m_registry.emplace<comps::FixedFrameTag>(entity);
  } else {
    m_registry.emplace<comps::FreeFrameTag>(entity);
  }

  auto& state = m_registry.emplace<comps::FrameState>(entity);
  state.parentFrame = parentFrame.getEntity();

  auto& cache = m_registry.emplace<comps::FrameCache>(entity);
  cache.worldTransform = Eigen::Isometry3d::Identity();
  cache.needTransformUpdate = true;

  if (isFixedFrame) {
    auto& props = m_registry.emplace<comps::FixedFrameProperties>(entity);
    props.localTransform = localTransform;
  } else {
    auto& props = m_registry.emplace<comps::FreeFrameProperties>(entity);
    props.localTransform = localTransform;
  }

  outName = actualName;
  return entity;
}

//==============================================================================
Frame World::resolveParentFrame(const Frame& parent) const
{
  if (parent.isWorld()) {
    return Frame(entt::null, const_cast<World*>(this));
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !parent.isValid(),
      InvalidArgumentException,
      "Parent frame is invalid or has been destroyed");

  DART_EXPERIMENTAL_THROW_T_IF(
      parent.getWorld() != this,
      InvalidArgumentException,
      "Parent frame belongs to a different world");

  return parent;
}

//==============================================================================
Multibody World::addMultibody(std::string_view name)
{
  ensureDesignMode();

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName = std::format("multibody_{:03d}", ++m_multibodyCounter);
    } while (hasEntityWithName<comps::MultibodyTag>(m_registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::MultibodyTag>(m_registry, candidateName),
        InvalidArgumentException,
        "Multibody '{}' already exists",
        candidateName);
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, candidateName);
  m_registry.emplace<comps::MultibodyTag>(entity);
  m_registry.emplace<comps::MultibodyStructure>(entity);

  return Multibody(entity, this);
}

//==============================================================================
std::optional<Multibody> World::getMultibody(std::string_view name)
{
  auto view = m_registry.view<comps::MultibodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return Multibody(entity, this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasMultibody(std::string_view name) const
{
  return hasEntityWithName<comps::MultibodyTag>(m_registry, name);
}

//==============================================================================
std::size_t World::getMultibodyCount() const
{
  return countEntities<comps::MultibodyTag>(m_registry);
}

//==============================================================================
LoopClosure World::addLoopClosure(
    std::string_view name, const LoopClosureSpec& spec)
{
  ensureDesignMode();
  validateLoopClosureSpec(*this, spec);

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName
          = std::format("loop_closure_{:03d}", ++m_loopClosureCounter);
    } while (hasEntityWithName<comps::LoopClosure>(m_registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::LoopClosure>(m_registry, candidateName),
        InvalidArgumentException,
        "LoopClosure '{}' already exists",
        candidateName);
  }

  auto entity = m_registry.create();
  m_registry.emplace<comps::Name>(entity, candidateName);

  auto& closure = m_registry.emplace<comps::LoopClosure>(entity);
  closure.family = spec.family;
  closure.frameA = resolveLoopClosureFrame(*this, spec.frameA, "frameA");
  closure.frameB = resolveLoopClosureFrame(*this, spec.frameB, "frameB");
  closure.offsetA = spec.offsetA;
  closure.offsetB = spec.offsetB;

  return LoopClosure(entity, this);
}

//==============================================================================
std::optional<LoopClosure> World::getLoopClosure(std::string_view name)
{
  auto view = m_registry.view<comps::LoopClosure, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return LoopClosure(entity, this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasLoopClosure(std::string_view name) const
{
  return hasEntityWithName<comps::LoopClosure>(m_registry, name);
}

//==============================================================================
std::size_t World::getLoopClosureCount() const
{
  return countEntities<comps::LoopClosure>(m_registry);
}

//==============================================================================
RigidBody World::addRigidBody(
    std::string_view name, const RigidBodyOptions& options)
{
  ensureDesignMode();

  std::string candidateName;
  if (name.empty()) {
    do {
      candidateName = std::format("rigid_body_{:03d}", ++m_rigidBodyCounter);
    } while (hasEntityWithName<comps::RigidBodyTag>(m_registry, candidateName));
  } else {
    candidateName = std::string(name);
    DART_EXPERIMENTAL_THROW_T_IF(
        hasEntityWithName<comps::RigidBodyTag>(m_registry, candidateName),
        InvalidArgumentException,
        "RigidBody '{}' already exists",
        candidateName);
  }

  validateRigidBodyOptions(options);

  Frame parent = Frame(entt::null, this);
  const auto orientation = normalizeOrIdentity(options.orientation);
  const auto initialTransform = toIsometry(options.position, orientation);

  std::string actualName;
  auto entity = createFrameEntity(
      candidateName,
      parent,
      initialTransform,
      &m_rigidBodyCounter,
      "rigid_body",
      false,
      actualName);

  m_registry.emplace<comps::RigidBodyTag>(entity);

  auto& transform = m_registry.emplace<comps::Transform>(entity);
  transform.position = options.position;
  transform.orientation = orientation;

  auto& velocity = m_registry.emplace<comps::Velocity>(entity);
  velocity.linear = options.linearVelocity;
  velocity.angular = options.angularVelocity;

  auto& mass = m_registry.emplace<comps::MassProperties>(entity);
  mass.mass = options.mass;
  mass.inertia = options.inertia;

  m_registry.emplace<comps::Force>(entity);

  if (options.isStatic) {
    m_registry.emplace<comps::StaticBodyTag>(entity);
  }

  return RigidBody(entity, this);
}

//==============================================================================
std::optional<RigidBody> World::getRigidBody(std::string_view name)
{
  auto view = m_registry.view<comps::RigidBodyTag, comps::Name>();
  for (auto entity : view) {
    const auto& info = view.get<comps::Name>(entity);
    if (info.name == name) {
      return RigidBody(entity, this);
    }
  }
  return std::nullopt;
}

//==============================================================================
bool World::hasRigidBody(std::string_view name) const
{
  return hasEntityWithName<comps::RigidBodyTag>(m_registry, name);
}

//==============================================================================
std::size_t World::getRigidBodyCount() const
{
  return countEntities<comps::RigidBodyTag>(m_registry);
}

//==============================================================================
void World::enterSimulationMode()
{
  DART_EXPERIMENTAL_THROW_T_IF(
      m_simulationMode,
      InvalidArgumentException,
      "World is already in simulation mode");

  validateLoopClosureKinematicsPolicySupport(*this);
  m_simulationMode = true;

  // Initial bake so that cached transforms are up-to-date.
  updateKinematics();
}

//==============================================================================
void World::setGravity(const Eigen::Vector3d& gravity)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !gravity.array().isFinite().all(),
      InvalidArgumentException,
      "Gravity must contain only finite coordinates");

  m_gravity = gravity;
}

//==============================================================================
const Eigen::Vector3d& World::getGravity() const noexcept
{
  return m_gravity;
}

//==============================================================================
void World::setTimeStep(double timeStep)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(timeStep) || timeStep <= 0.0,
      InvalidArgumentException,
      "Time step must be positive and finite");

  m_timeStep = timeStep;
}

//==============================================================================
double World::getTimeStep() const noexcept
{
  return m_timeStep;
}

//==============================================================================
void World::setTime(double time)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !std::isfinite(time) || time < 0.0,
      InvalidArgumentException,
      "Time must be non-negative and finite");

  m_time = time;
}

//==============================================================================
double World::getTime() const noexcept
{
  return m_time;
}

//==============================================================================
std::size_t World::getFrame() const noexcept
{
  return m_frame;
}

//==============================================================================
void World::sync(WorldSyncStage stage)
{
  compute::SequentialExecutor executor;
  sync(stage, executor);
}

//==============================================================================
void World::sync(WorldSyncStage stage, compute::ComputeExecutor& executor)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      !m_simulationMode,
      InvalidArgumentException,
      "World::sync() requires simulation mode");

  DART_EXPERIMENTAL_THROW_T_IF(
      !isValidWorldSyncStage(stage),
      InvalidArgumentException,
      "World::sync() stage is invalid");

  switch (stage) {
    case WorldSyncStage::Kinematics:
      validateLoopClosureKinematicsPolicySupport(*this);
      executeKinematicsGraph(*this, executor);
      return;
  }
}

//==============================================================================
void World::updateKinematics()
{
  sync(WorldSyncStage::Kinematics);
}

//==============================================================================
void World::updateKinematics(compute::ComputeExecutor& executor)
{
  sync(WorldSyncStage::Kinematics, executor);
}

//==============================================================================
void World::step()
{
  compute::SequentialExecutor executor;
  step(executor);
}

//==============================================================================
void World::step(std::size_t count)
{
  compute::SequentialExecutor executor;
  step(count, executor);
}

//==============================================================================
void World::step(compute::ComputeExecutor& executor)
{
  compute::RigidBodyVelocityStage rigidBodyVelocity;
  compute::RigidBodyContactStage rigidBodyContact;
  compute::RigidBodyPositionStage rigidBodyPosition;
  compute::MultibodyForwardDynamicsStage multibodyDynamics;
  compute::KinematicsStage kinematics;
  compute::WorldStepPipeline pipeline;
  // Integrate rigid-body positions after the multibody stage so two-sided
  // link-vs-rigid-body contact impulses (applied to rigid-body velocities in
  // the multibody solve) take effect in the same step's pose update.
  pipeline.addStage(rigidBodyVelocity)
      .addStage(rigidBodyContact)
      .addStage(multibodyDynamics)
      .addStage(rigidBodyPosition)
      .addStage(kinematics);
  step(executor, pipeline);
}

//==============================================================================
void World::step(std::size_t count, compute::ComputeExecutor& executor)
{
  compute::RigidBodyVelocityStage rigidBodyVelocity;
  compute::RigidBodyContactStage rigidBodyContact;
  compute::RigidBodyPositionStage rigidBodyPosition;
  compute::MultibodyForwardDynamicsStage multibodyDynamics;
  compute::KinematicsStage kinematics;
  compute::WorldStepPipeline pipeline;
  // Integrate rigid-body positions after the multibody stage so two-sided
  // link-vs-rigid-body contact impulses (applied to rigid-body velocities in
  // the multibody solve) take effect in the same step's pose update.
  pipeline.addStage(rigidBodyVelocity)
      .addStage(rigidBodyContact)
      .addStage(multibodyDynamics)
      .addStage(rigidBodyPosition)
      .addStage(kinematics);
  step(count, executor, pipeline);
}

//==============================================================================
void World::step(
    compute::ComputeExecutor& executor, compute::WorldStepStage& stage)
{
  compute::RigidBodyIntegrationStage rigidBodyIntegration;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(rigidBodyIntegration).addStage(stage);
  step(executor, pipeline);
}

//==============================================================================
void World::step(
    std::size_t count,
    compute::ComputeExecutor& executor,
    compute::WorldStepStage& stage)
{
  compute::RigidBodyIntegrationStage rigidBodyIntegration;
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(rigidBodyIntegration).addStage(stage);
  step(count, executor, pipeline);
}

//==============================================================================
void World::step(
    compute::ComputeExecutor& executor, compute::WorldStepPipeline& pipeline)
{
  validateLoopClosureKinematicsPolicySupport(*this);
  validateLoopClosureDynamicsPolicySupport(*this);

  if (!m_simulationMode) {
    enterSimulationMode();
  }

  pipeline.execute(*this, executor);

  m_time += m_timeStep;
  ++m_frame;
}

//==============================================================================
void World::step(
    std::size_t count,
    compute::ComputeExecutor& executor,
    compute::WorldStepPipeline& pipeline)
{
  for (std::size_t i = 0; i < count; ++i) {
    step(executor, pipeline);
  }
}

//==============================================================================
std::vector<Contact> World::collide()
{
  namespace ncol = dart::collision::native;

  ncol::CollisionWorld collisionWorld;

  // Build one native collision object per shape-bearing rigid body, remembering
  // which experimental entity each object came from.
  struct ObjectEntry
  {
    entt::entity entity;
    ncol::CollisionObject object;
  };
  std::vector<ObjectEntry> entries;

  const auto addEntry = [&](entt::entity entity,
                            const CollisionShape& collisionShape,
                            const Eigen::Isometry3d& pose) {
    std::unique_ptr<ncol::Shape> shape;
    switch (collisionShape.type) {
      case CollisionShapeType::Sphere:
        shape = std::make_unique<ncol::SphereShape>(collisionShape.radius);
        break;
      case CollisionShapeType::Box:
        shape = std::make_unique<ncol::BoxShape>(collisionShape.halfExtents);
        break;
    }
    entries.push_back(
        {entity, collisionWorld.createObject(std::move(shape), pose)});
  };

  // Rigid bodies pose their collision shape from the rigid-body transform.
  auto rigidBodyView = m_registry.view<
      comps::CollisionGeometry,
      comps::Transform,
      comps::RigidBodyTag>();
  for (auto entity : rigidBodyView) {
    const auto& geometry = rigidBodyView.get<comps::CollisionGeometry>(entity);
    const auto& transform = rigidBodyView.get<comps::Transform>(entity);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = transform.orientation.normalized().toRotationMatrix();
    pose.translation() = transform.position;
    addEntry(entity, geometry.shape, pose);
  }

  // Multibody links pose their collision shape through the frame accessor so
  // dirty joint-driven caches are refreshed before the query.
  auto linkView
      = m_registry
            .view<comps::CollisionGeometry, comps::Link, comps::FrameCache>();
  for (auto entity : linkView) {
    const auto& geometry = linkView.get<comps::CollisionGeometry>(entity);
    const Link link(entity, this);
    addEntry(entity, geometry.shape, link.getWorldTransform());
  }

  // Pairwise narrow-phase queries. Each pair's bodies are known here, so the
  // contacts map back to the right rigid bodies without relying on the result
  // carrying object identity. This is O(n^2); a future slice can use the native
  // broad phase to prune candidate pairs.
  const auto option = ncol::CollisionOption::fullContacts();
  std::vector<Contact> contacts;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    for (std::size_t j = i + 1; j < entries.size(); ++j) {
      ncol::CollisionResult result;
      if (!collisionWorld.collide(
              entries[i].object, entries[j].object, option, result)) {
        continue;
      }

      for (std::size_t k = 0; k < result.numContacts(); ++k) {
        const auto& point = result.getContact(k);
        // The native narrow phase reports the normal pointing from the second
        // object toward the first; the public Contact convention points from
        // bodyA (entries[i]) toward bodyB (entries[j]), so negate it.
        contacts.push_back(
            Contact{
                CollisionBody(entries[i].entity, this),
                CollisionBody(entries[j].entity, this),
                point.position,
                -point.normal,
                point.depth});
      }
    }
  }

  return contacts;
}

//==============================================================================
void World::saveBinary(std::ostream& output) const
{
  io::writeFormatHeader(output);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().saveAllEntities(
      output, m_registry, entityMap);

  const std::uint8_t simulationFlag = m_simulationMode ? 1 : 0;
  io::writePOD(output, simulationFlag);
  io::writePOD(output, m_freeFrameCounter);
  io::writePOD(output, m_fixedFrameCounter);
  io::writePOD(output, m_multibodyCounter);
  io::writePOD(output, m_rigidBodyCounter);
  io::writePOD(output, m_linkCounter);
  io::writePOD(output, m_jointCounter);
  io::writePOD(output, m_timeStep);
  io::writePOD(output, m_time);
  io::writePOD(output, m_frame);
  io::writePOD(output, m_gravity.x());
  io::writePOD(output, m_gravity.y());
  io::writePOD(output, m_gravity.z());
}

//==============================================================================
void World::loadBinary(std::istream& input)
{
  clear();

  const auto formatVersion = io::readFormatHeader(input);

  io::EntityMap entityMap;
  io::SerializerRegistry::instance().loadAllEntities(
      input, m_registry, entityMap);

  // World metadata (optional for forward-compatibility)
  if (input.peek() != std::char_traits<char>::eof()) {
    std::uint8_t simulationFlag = 0;
    io::readPOD(input, simulationFlag);
    m_simulationMode = simulationFlag != 0;

    io::readPOD(input, m_freeFrameCounter);
    io::readPOD(input, m_fixedFrameCounter);
    io::readPOD(input, m_multibodyCounter);
    io::readPOD(input, m_rigidBodyCounter);
    io::readPOD(input, m_linkCounter);
    io::readPOD(input, m_jointCounter);

    if (input.peek() != std::char_traits<char>::eof()) {
      io::readPOD(input, m_timeStep);
      io::readPOD(input, m_time);
      io::readPOD(input, m_frame);
    }

    if (formatVersion >= 2) {
      double gravityX = 0.0;
      double gravityY = 0.0;
      double gravityZ = 0.0;
      io::readPOD(input, gravityX);
      io::readPOD(input, gravityY);
      io::readPOD(input, gravityZ);
      m_gravity = Eigen::Vector3d(gravityX, gravityY, gravityZ);
    }
  }

  // Ensure all frame entities have cache components (not serialized)
  auto frameView = m_registry.view<comps::FrameTag>();
  for (auto entity : frameView) {
    if (!m_registry.any_of<comps::FrameCache>(entity)) {
      auto& cache = m_registry.emplace<comps::FrameCache>(entity);
      cache.worldTransform = Eigen::Isometry3d::Identity();
      cache.needTransformUpdate = true;
    } else {
      auto& cache = m_registry.get<comps::FrameCache>(entity);
      cache.needTransformUpdate = true;
    }
  }

  resetCountersFromRegistry();

  if (m_simulationMode) {
    updateKinematics();
  }
}

//==============================================================================
void World::resetCountersFromRegistry()
{
  m_freeFrameCounter = std::max(
      m_freeFrameCounter, countEntities<comps::FreeFrameTag>(m_registry));
  m_fixedFrameCounter = std::max(
      m_fixedFrameCounter, countEntities<comps::FixedFrameTag>(m_registry));
  m_multibodyCounter = std::max(
      m_multibodyCounter, countEntities<comps::MultibodyTag>(m_registry));
  m_loopClosureCounter = std::max(
      m_loopClosureCounter, countEntities<comps::LoopClosure>(m_registry));
  m_rigidBodyCounter = std::max(
      m_rigidBodyCounter, countEntities<comps::RigidBodyTag>(m_registry));
  m_linkCounter
      = std::max(m_linkCounter, countEntities<comps::Link>(m_registry));
  m_jointCounter
      = std::max(m_jointCounter, countEntities<comps::Joint>(m_registry));
}

} // namespace dart::simulation::experimental
