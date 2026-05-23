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

#include "dart/simulation/experimental/compute/rigid_body_state_batch.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"
#include "dart/simulation/experimental/comps/dynamics.hpp"
#include "dart/simulation/experimental/comps/rigid_body.hpp"
#include "dart/simulation/experimental/compute/rigid_body_integration_kernel.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <cmath>

namespace dart::simulation::experimental::compute {

namespace {

//==============================================================================
bool allFinite(const Eigen::Vector3d& value)
{
  return value.array().isFinite().all();
}

//==============================================================================
bool allFinite(const Eigen::Matrix3d& value)
{
  return value.array().isFinite().all();
}

//==============================================================================
// Add the angular acceleration from torque to each body's angular velocity,
// mirroring the per-entity integrateAngularVelocity: it forms the world-frame
// inertia (R I R^T) from the normalized orientation, LDLT-solves the torque,
// and leaves the angular velocity unchanged on a non-finite or
// non-positive-definite system. Reads the pre-update orientation, so it must
// run before the orientation step.
void integrateAngularVelocitiesFromTorque(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& torque,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  for (std::size_t b = 0; b < bodies; ++b) {
    const Eigen::Vector3d tau(
        torque[3 * b + 0], torque[3 * b + 1], torque[3 * b + 2]);
    const Eigen::Matrix3d inertia
        = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            model.inertia.data() + 9 * b);
    if (!allFinite(tau) || !allFinite(inertia)) {
      continue;
    }

    // Normalize the orientation exactly as the per-entity path does, falling
    // back to identity for a degenerate quaternion.
    Eigen::Quaterniond orientation(
        state.orientation[4 * b + 0],
        state.orientation[4 * b + 1],
        state.orientation[4 * b + 2],
        state.orientation[4 * b + 3]);
    const double norm = orientation.norm();
    Eigen::Matrix3d rotation;
    if (norm <= 0.0 || !std::isfinite(norm)) {
      rotation = Eigen::Matrix3d::Identity();
    } else {
      orientation.coeffs() /= norm;
      rotation = orientation.toRotationMatrix();
    }

    const Eigen::Matrix3d worldInertia
        = rotation * inertia * rotation.transpose();
    Eigen::LDLT<Eigen::Matrix3d> solver(worldInertia);
    if (solver.info() != Eigen::Success || !solver.isPositive()) {
      continue;
    }

    const Eigen::Vector3d angularAcceleration = solver.solve(tau);
    if (solver.info() != Eigen::Success || !allFinite(angularAcceleration)) {
      continue;
    }

    state.angularVelocity[3 * b + 0] += angularAcceleration.x() * timeStep;
    state.angularVelocity[3 * b + 1] += angularAcceleration.y() * timeStep;
    state.angularVelocity[3 * b + 2] += angularAcceleration.z() * timeStep;
  }
}

} // namespace

//==============================================================================
RigidBodyStateBatch extractRigidBodyState(const World& world)
{
  const auto& registry = world.getRegistry();
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  RigidBodyStateBatch batch;
  batch.worldCount = 1;
  for (const auto entity : view) {
    ++batch.bodyCount;

    const auto& transform = view.get<comps::Transform>(entity);
    const auto& velocity = view.get<comps::Velocity>(entity);

    batch.position.push_back(transform.position.x());
    batch.position.push_back(transform.position.y());
    batch.position.push_back(transform.position.z());

    batch.orientation.push_back(transform.orientation.w());
    batch.orientation.push_back(transform.orientation.x());
    batch.orientation.push_back(transform.orientation.y());
    batch.orientation.push_back(transform.orientation.z());

    batch.linearVelocity.push_back(velocity.linear.x());
    batch.linearVelocity.push_back(velocity.linear.y());
    batch.linearVelocity.push_back(velocity.linear.z());

    batch.angularVelocity.push_back(velocity.angular.x());
    batch.angularVelocity.push_back(velocity.angular.y());
    batch.angularVelocity.push_back(velocity.angular.z());
  }

  return batch;
}

//==============================================================================
void applyRigidBodyState(World& world, const RigidBodyStateBatch& state)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      state.worldCount != 1,
      InvalidArgumentException,
      "applyRigidBodyState supports single-world batches only, got worldCount "
      "{}",
      state.worldCount);

  // The fields are independently public, so validate their sizes against
  // bodyCount before indexing to avoid out-of-bounds reads on a malformed
  // batch.
  DART_EXPERIMENTAL_THROW_T_IF(
      state.position.size() != 3 * state.bodyCount
          || state.linearVelocity.size() != 3 * state.bodyCount
          || state.angularVelocity.size() != 3 * state.bodyCount
          || state.orientation.size() != 4 * state.bodyCount,
      InvalidArgumentException,
      "RigidBodyStateBatch arrays are inconsistent with bodyCount {}",
      state.bodyCount);

  auto& registry = world.getRegistry();
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  std::size_t index = 0;
  for (const auto entity : view) {
    DART_EXPERIMENTAL_THROW_T_IF(
        index >= state.bodyCount,
        InvalidArgumentException,
        "RigidBodyStateBatch has fewer bodies ({}) than the world",
        state.bodyCount);

    auto& transform = view.get<comps::Transform>(entity);
    auto& velocity = view.get<comps::Velocity>(entity);

    transform.position = Eigen::Vector3d(
        state.position[3 * index + 0],
        state.position[3 * index + 1],
        state.position[3 * index + 2]);
    transform.orientation = Eigen::Quaterniond(
        state.orientation[4 * index + 0],
        state.orientation[4 * index + 1],
        state.orientation[4 * index + 2],
        state.orientation[4 * index + 3]);
    velocity.linear = Eigen::Vector3d(
        state.linearVelocity[3 * index + 0],
        state.linearVelocity[3 * index + 1],
        state.linearVelocity[3 * index + 2]);
    velocity.angular = Eigen::Vector3d(
        state.angularVelocity[3 * index + 0],
        state.angularVelocity[3 * index + 1],
        state.angularVelocity[3 * index + 2]);

    ++index;
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      index != state.bodyCount,
      InvalidArgumentException,
      "RigidBodyStateBatch body count ({}) does not match the world ({})",
      state.bodyCount,
      index);
}

//==============================================================================
RigidBodyStateBatch extractRigidBodyStateBatch(
    const std::vector<const World*>& worlds)
{
  RigidBodyStateBatch batch;
  batch.worldCount = worlds.size();
  if (worlds.empty()) {
    return batch;
  }

  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_EXPERIMENTAL_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "extractRigidBodyStateBatch received a null world at index {}",
        w);

    const auto single = extractRigidBodyState(*worlds[w]);
    if (w == 0) {
      batch.bodyCount = single.bodyCount;
    } else {
      DART_EXPERIMENTAL_THROW_T_IF(
          single.bodyCount != batch.bodyCount,
          InvalidArgumentException,
          "Heterogeneous body counts: world 0 has {}, world {} has {}",
          batch.bodyCount,
          w,
          single.bodyCount);
    }

    batch.position.insert(
        batch.position.end(), single.position.begin(), single.position.end());
    batch.orientation.insert(
        batch.orientation.end(),
        single.orientation.begin(),
        single.orientation.end());
    batch.linearVelocity.insert(
        batch.linearVelocity.end(),
        single.linearVelocity.begin(),
        single.linearVelocity.end());
    batch.angularVelocity.insert(
        batch.angularVelocity.end(),
        single.angularVelocity.begin(),
        single.angularVelocity.end());
  }

  return batch;
}

//==============================================================================
void applyRigidBodyStateBatch(
    const std::vector<World*>& worlds, const RigidBodyStateBatch& state)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      state.worldCount != worlds.size(),
      InvalidArgumentException,
      "RigidBodyStateBatch worldCount ({}) does not match the number of worlds "
      "({})",
      state.worldCount,
      worlds.size());

  const auto bodies = state.worldCount * state.bodyCount;
  DART_EXPERIMENTAL_THROW_T_IF(
      state.position.size() != 3 * bodies
          || state.linearVelocity.size() != 3 * bodies
          || state.angularVelocity.size() != 3 * bodies
          || state.orientation.size() != 4 * bodies,
      InvalidArgumentException,
      "RigidBodyStateBatch arrays are inconsistent with worldCount {} and "
      "bodyCount {}",
      state.worldCount,
      state.bodyCount);

  const auto bodyCount = state.bodyCount;
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_EXPERIMENTAL_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "applyRigidBodyStateBatch received a null world at index {}",
        w);

    const auto p0 = static_cast<std::ptrdiff_t>(3 * w * bodyCount);
    const auto p1 = static_cast<std::ptrdiff_t>(3 * (w + 1) * bodyCount);
    const auto o0 = static_cast<std::ptrdiff_t>(4 * w * bodyCount);
    const auto o1 = static_cast<std::ptrdiff_t>(4 * (w + 1) * bodyCount);

    RigidBodyStateBatch single;
    single.worldCount = 1;
    single.bodyCount = bodyCount;
    single.position.assign(
        state.position.begin() + p0, state.position.begin() + p1);
    single.orientation.assign(
        state.orientation.begin() + o0, state.orientation.begin() + o1);
    single.linearVelocity.assign(
        state.linearVelocity.begin() + p0, state.linearVelocity.begin() + p1);
    single.angularVelocity.assign(
        state.angularVelocity.begin() + p0, state.angularVelocity.begin() + p1);

    applyRigidBodyState(*worlds[w], single);
  }
}

//==============================================================================
void integrateRigidBodyStateBatchLinear(
    RigidBodyStateBatch& state,
    const std::vector<double>& force,
    const std::vector<double>& inverseMass,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_EXPERIMENTAL_THROW_T_IF(
      state.position.size() != 3 * bodies
          || state.linearVelocity.size() != 3 * bodies
          || force.size() != 3 * bodies || inverseMass.size() != bodies,
      InvalidArgumentException,
      "integrateRigidBodyStateBatchLinear arrays are inconsistent with "
      "worldCount {} and bodyCount {}",
      state.worldCount,
      state.bodyCount);

  integrateVelocitiesSemiImplicit(
      state.linearVelocity.data(),
      force.data(),
      inverseMass.data(),
      timeStep,
      bodies);
  integratePositionsSemiImplicit(
      state.position.data(),
      state.linearVelocity.data(),
      timeStep,
      state.position.size());
}

//==============================================================================
RigidBodyModelBatch extractRigidBodyModelBatch(const World& world)
{
  const auto& registry = world.getRegistry();
  // Use the same view as extractRigidBodyState so the model order matches the
  // state order body-for-body, then fetch the mass per entity.
  auto view
      = registry.view<comps::RigidBodyTag, comps::Transform, comps::Velocity>();

  RigidBodyModelBatch model;
  model.worldCount = 1;
  for (const auto entity : view) {
    ++model.bodyCount;
    const auto& mass = registry.get<comps::MassProperties>(entity);
    const double inverse
        = (mass.mass > 0.0 && std::isfinite(mass.mass)) ? 1.0 / mass.mass : 0.0;
    model.inverseMass.push_back(inverse);

    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        model.inertia.push_back(mass.inertia(row, col));
      }
    }
  }

  return model;
}

//==============================================================================
void integrateRigidBodyStateBatchLinear(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      model.worldCount != state.worldCount
          || model.bodyCount != state.bodyCount,
      InvalidArgumentException,
      "RigidBodyModelBatch ({}x{}) does not match the state batch ({}x{})",
      model.worldCount,
      model.bodyCount,
      state.worldCount,
      state.bodyCount);

  integrateRigidBodyStateBatchLinear(state, force, model.inverseMass, timeStep);
}

//==============================================================================
void integrateRigidBodyStateBatch(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_EXPERIMENTAL_THROW_T_IF(
      state.orientation.size() != 4 * bodies,
      InvalidArgumentException,
      "RigidBodyStateBatch orientation array is inconsistent with worldCount "
      "{} "
      "and bodyCount {}",
      state.worldCount,
      state.bodyCount);

  // Linear step (validates model, force, and the 3-component arrays), then the
  // orientation step from the batch angular velocity.
  integrateRigidBodyStateBatchLinear(state, model, force, timeStep);
  integrateOrientationsSemiImplicit(
      state.orientation.data(), state.angularVelocity.data(), timeStep, bodies);
}

//==============================================================================
void integrateRigidBodyStateBatch(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    const std::vector<double>& force,
    const std::vector<double>& torque,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_EXPERIMENTAL_THROW_T_IF(
      state.orientation.size() != 4 * bodies || torque.size() != 3 * bodies
          || model.inertia.size() != 9 * bodies,
      InvalidArgumentException,
      "integrateRigidBodyStateBatch (torque) arrays are inconsistent with "
      "worldCount {} and bodyCount {}",
      state.worldCount,
      state.bodyCount);

  // Match the per-entity order: linear velocity and position (the linear helper
  // validates model, force, and the 3-component arrays), then angular velocity
  // from torque using the pre-update orientation, then orientation from the
  // updated angular velocity.
  integrateRigidBodyStateBatchLinear(state, model, force, timeStep);
  integrateAngularVelocitiesFromTorque(state, model, torque, timeStep);
  integrateOrientationsSemiImplicit(
      state.orientation.data(), state.angularVelocity.data(), timeStep, bodies);
}

//==============================================================================
RigidBodyStateBatch rolloutRigidBodyStateBatch(
    const RigidBodyStateBatch& initialState,
    const RigidBodyModelBatch& model,
    const std::vector<std::vector<double>>& controlSequence,
    double timeStep)
{
  RigidBodyStateBatch state = initialState;
  for (const auto& force : controlSequence) {
    integrateRigidBodyStateBatch(state, model, force, timeStep);
  }
  return state;
}

} // namespace dart::simulation::experimental::compute
