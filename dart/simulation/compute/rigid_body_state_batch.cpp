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

#include "dart/simulation/compute/rigid_body_state_batch.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/comps/dynamics.hpp"
#include "dart/simulation/comps/rigid_body.hpp"
#include "dart/simulation/compute/rigid_body_batch_ops.hpp"
#include "dart/simulation/compute/rigid_body_integration_kernel.hpp"
#include "dart/simulation/detail/world_registry_access.hpp"
#include "dart/simulation/detail/world_storage.hpp"
#include "dart/simulation/world.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <algorithm>
#include <unordered_set>
#include <vector>

#include <cmath>

namespace dart::simulation::compute {

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
bool sameRigidBodyModelIdentity(
    const dart::simulation::detail::BakedWorldModel& lhs,
    const dart::simulation::detail::BakedWorldModel& rhs)
{
  return lhs.rigidBodyEntities.size() == rhs.rigidBodyEntities.size()
         && lhs.rigidBodyIsDynamic == rhs.rigidBodyIsDynamic
         && lhs.rigidBodyInverseMass == rhs.rigidBodyInverseMass
         && lhs.rigidBodyInertia == rhs.rigidBodyInertia;
}

//==============================================================================
template <typename LhsRange, typename RhsRange>
bool rangeEquals(const LhsRange& lhs, const RhsRange& rhs)
{
  return lhs.size() == rhs.size()
         && std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

//==============================================================================
void appendRigidBodyStateFromBakedModel(
    const World& world,
    const dart::simulation::detail::BakedWorldModel& model,
    RigidBodyStateBatch& batch)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  for (const auto entity : model.rigidBodyEntities) {
    const auto& transform = registry.get<comps::Transform>(entity);
    const auto& velocity = registry.get<comps::Velocity>(entity);

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
}

//==============================================================================
void appendRigidBodyControlFromBakedModel(
    const World& world,
    const dart::simulation::detail::BakedWorldModel& model,
    std::vector<double>& forceBatch)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  for (const auto entity : model.rigidBodyEntities) {
    const auto& force = registry.get<comps::Force>(entity);
    forceBatch.push_back(force.force.x());
    forceBatch.push_back(force.force.y());
    forceBatch.push_back(force.force.z());
  }
}

//==============================================================================
std::size_t rigidBodyControlStepSize(
    const RigidBodyControlSequenceBatch& controlSequence)
{
  return 3 * controlSequence.worldCount * controlSequence.bodyCount;
}

//==============================================================================
void validateRigidBodyControlSequenceStorage(
    const RigidBodyControlSequenceBatch& controlSequence)
{
  const auto stepSize = rigidBodyControlStepSize(controlSequence);
  const auto expectedSize = controlSequence.stepCount * stepSize;
  DART_SIMULATION_THROW_T_IF(
      controlSequence.force.size() != expectedSize,
      InvalidArgumentException,
      "RigidBodyControlSequenceBatch force array has size {}, expected {} "
      "(stepCount {}, worldCount {}, bodyCount {})",
      controlSequence.force.size(),
      expectedSize,
      controlSequence.stepCount,
      controlSequence.worldCount,
      controlSequence.bodyCount);
}

//==============================================================================
// Add the angular acceleration from torque to each body's angular velocity,
// mirroring the per-entity integrateAngularVelocity: it forms the world-frame
// inertia (R I R^T) from the normalized orientation, LDLT-solves the torque,
// and leaves the angular velocity unchanged on a non-finite or
// non-positive-definite system. Reads the pre-update orientation, so it must
// run before the orientation step.
void integrateAngularVelocitiesFromTorque(
    rigid_body_batch_ops::MutableRigidBodyStateBatchView state,
    rigid_body_batch_ops::RigidBodyModelBatchView model,
    std::span<const double> torque,
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

//==============================================================================
// SIMD orientation update: the same angle-axis exponential map as the
// scalar-generic integrateOrientationsSemiImplicit kernel, expressed as Eigen
// array operations so the per-body transcendentals (sqrt/sin/cos) vectorize.
// The array-of-quaternions layout is deinterleaved into per-component arrays so
// the vectorized math stays contiguous. This is the double-only fast path; the
// scalar-generic kernel remains the reference and the fallback.
void integrateOrientationsSimd(
    double* orientations,
    const double* angularVelocities,
    double timeStep,
    std::size_t bodyCount)
{
  if (bodyCount == 0) {
    return;
  }

  using Array = Eigen::ArrayXd;
  const auto n = static_cast<Eigen::Index>(bodyCount);

  Array w(n);
  Array x(n);
  Array y(n);
  Array z(n);
  Array ox(n);
  Array oy(n);
  Array oz(n);
  for (Eigen::Index b = 0; b < n; ++b) {
    w[b] = orientations[4 * b + 0];
    x[b] = orientations[4 * b + 1];
    y[b] = orientations[4 * b + 2];
    z[b] = orientations[4 * b + 3];
    ox[b] = angularVelocities[3 * b + 0];
    oy[b] = angularVelocities[3 * b + 1];
    oz[b] = angularVelocities[3 * b + 2];
  }

  const Array speed = (ox * ox + oy * oy + oz * oz).sqrt();
  const Array halfAngle = 0.5 * speed * timeStep;
  const auto spinning = speed > 0.0;
  // Fold the axis normalization (1/speed) into the sin factor; guard the
  // zero-spin case so it leaves the quaternion unchanged, matching the scalar
  // kernel exactly.
  const Array safeSpeed = spinning.select(speed, 1.0);
  const Array axisScale = spinning.select(halfAngle.sin() / safeSpeed, 0.0);
  const Array dw = spinning.select(halfAngle.cos(), 1.0);
  // Mask the whole delta vector on non-spinning lanes (not just axisScale): a
  // non-finite angular velocity would otherwise survive as omega * 0 == NaN,
  // corrupting the SIMD path while the scalar path leaves it unchanged.
  const Array dx = spinning.select(ox * axisScale, 0.0);
  const Array dy = spinning.select(oy * axisScale, 0.0);
  const Array dz = spinning.select(oz * axisScale, 0.0);

  // dq * q (Hamilton product), left-multiplying the world-frame delta.
  Array nw = dw * w - dx * x - dy * y - dz * z;
  Array nx = dw * x + dx * w + dy * z - dz * y;
  Array ny = dw * y - dx * z + dy * w + dz * x;
  Array nz = dw * z + dx * y - dy * x + dz * w;

  // A non-normalizable (zero-norm or non-finite) quaternion maps to identity,
  // matching the scalar kernel and the per-entity normalizeOrIdentity, so a
  // degenerate input cannot propagate and the SIMD path stays consistent with
  // the scalar path.
  const Array norm = (nw * nw + nx * nx + ny * ny + nz * nz).sqrt();
  const auto normalizable = (norm > 0.0) && norm.isFinite();
  nw = normalizable.select(nw / norm, 1.0);
  nx = normalizable.select(nx / norm, 0.0);
  ny = normalizable.select(ny / norm, 0.0);
  nz = normalizable.select(nz / norm, 0.0);

  for (Eigen::Index b = 0; b < n; ++b) {
    orientations[4 * b + 0] = nw[b];
    orientations[4 * b + 1] = nx[b];
    orientations[4 * b + 2] = ny[b];
    orientations[4 * b + 3] = nz[b];
  }
}

//==============================================================================
// Dispatch the orientation update: the deinterleave/vectorize/reinterleave SIMD
// path only pays off once there are enough bodies to amortize its array setup,
// so small batches use the scalar-generic kernel. The threshold is a
// conservative default; profiling can refine it.
constexpr std::size_t kOrientationSimdThreshold = 64;

void integrateOrientationsBatch(
    double* orientations,
    const double* angularVelocities,
    double timeStep,
    std::size_t bodyCount)
{
  if (bodyCount >= kOrientationSimdThreshold) {
    integrateOrientationsSimd(
        orientations, angularVelocities, timeStep, bodyCount);
  } else {
    integrateOrientationsSemiImplicit(
        orientations, angularVelocities, timeStep, bodyCount);
  }
}

} // namespace

//==============================================================================
void BakedRigidBodyBatchOwner::clear()
{
  m_model = RigidBodyModelBatch{};
  m_state = RigidBodyStateBatch{};
  m_diagnostics = BakedRigidBodyBatchOwnerDiagnostics{};
  m_rigidBodyIsDynamic.clear();
  m_modelValid = false;
}

//==============================================================================
void BakedRigidBodyBatchOwner::captureFromWorlds(
    const std::vector<const World*>& worlds)
{
  m_state.worldCount = worlds.size();
  m_state.bodyCount = 0;
  m_state.position.clear();
  m_state.orientation.clear();
  m_state.linearVelocity.clear();
  m_state.angularVelocity.clear();

  if (worlds.empty()) {
    m_model = RigidBodyModelBatch{};
    m_rigidBodyIsDynamic.clear();
    m_modelValid = false;
    return;
  }

  std::vector<const dart::simulation::detail::BakedWorldModel*> laneModels;
  laneModels.reserve(worlds.size());
  const dart::simulation::detail::BakedWorldModel* referenceModel = nullptr;
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_SIMULATION_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "BakedRigidBodyBatchOwner received a null world at lane {}",
        w);

    const auto& model
        = dart::simulation::detail::ensureBakedWorldModelCurrent(*worlds[w]);
    if (w == 0) {
      referenceModel = &model;
    } else {
      DART_SIMULATION_THROW_T_IF(
          model.rigidBodyEntities.size()
              != referenceModel->rigidBodyEntities.size(),
          InvalidArgumentException,
          "Heterogeneous body counts: lane 0 has {}, lane {} has {}",
          referenceModel->rigidBodyEntities.size(),
          w,
          model.rigidBodyEntities.size());
      DART_SIMULATION_THROW_T_IF(
          !sameRigidBodyModelIdentity(model, *referenceModel),
          InvalidArgumentException,
          "BakedRigidBodyBatchOwner lane {} has a different rigid-body "
          "dense-index Model identity than lane 0; all worlds must expose the "
          "same ordered Model",
          w);
    }
    laneModels.push_back(&model);
  }

  const auto bodyCount = referenceModel->rigidBodyEntities.size();
  const bool reuseModel = modelMatches(
      std::span<const std::uint8_t>{
          referenceModel->rigidBodyIsDynamic.data(),
          referenceModel->rigidBodyIsDynamic.size()},
      std::span<const double>{
          referenceModel->rigidBodyInverseMass.data(),
          referenceModel->rigidBodyInverseMass.size()},
      std::span<const double>{
          referenceModel->rigidBodyInertia.data(),
          referenceModel->rigidBodyInertia.size()},
      worlds.size(),
      bodyCount);

  if (reuseModel) {
    ++m_diagnostics.modelReuseCount;
  } else {
    m_model.worldCount = worlds.size();
    m_model.bodyCount = bodyCount;
    m_model.inverseMass.clear();
    m_model.inertia.clear();
    m_model.inverseMass.reserve(worlds.size() * bodyCount);
    m_model.inertia.reserve(9 * worlds.size() * bodyCount);
    for (const auto* model : laneModels) {
      m_model.inverseMass.insert(
          m_model.inverseMass.end(),
          model->rigidBodyInverseMass.begin(),
          model->rigidBodyInverseMass.end());
      m_model.inertia.insert(
          m_model.inertia.end(),
          model->rigidBodyInertia.begin(),
          model->rigidBodyInertia.end());
    }
    m_rigidBodyIsDynamic.assign(
        referenceModel->rigidBodyIsDynamic.begin(),
        referenceModel->rigidBodyIsDynamic.end());
    m_modelValid = true;
    ++m_diagnostics.modelRefreshCount;
  }

  m_state.worldCount = worlds.size();
  m_state.bodyCount = bodyCount;
  m_state.position.reserve(3 * worlds.size() * bodyCount);
  m_state.orientation.reserve(4 * worlds.size() * bodyCount);
  m_state.linearVelocity.reserve(3 * worlds.size() * bodyCount);
  m_state.angularVelocity.reserve(3 * worlds.size() * bodyCount);
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    appendRigidBodyStateFromBakedModel(*worlds[w], *laneModels[w], m_state);
  }
  ++m_diagnostics.stateCaptureCount;
}

//==============================================================================
void BakedRigidBodyBatchOwner::captureFromWorld(const World& world)
{
  const std::vector<const World*> worlds{&world};
  captureFromWorlds(worlds);
}

//==============================================================================
void BakedRigidBodyBatchOwner::applyToWorlds(
    const std::vector<World*>& worlds) const
{
  applyRigidBodyStateBatch(worlds, m_state);
}

//==============================================================================
void BakedRigidBodyBatchOwner::applyToWorld(World& world) const
{
  applyRigidBodyState(world, m_state);
}

//==============================================================================
const RigidBodyModelBatch& BakedRigidBodyBatchOwner::model() const noexcept
{
  return m_model;
}

//==============================================================================
RigidBodyStateBatch& BakedRigidBodyBatchOwner::mutableState() noexcept
{
  return m_state;
}

//==============================================================================
const RigidBodyStateBatch& BakedRigidBodyBatchOwner::state() const noexcept
{
  return m_state;
}

//==============================================================================
const BakedRigidBodyBatchOwnerDiagnostics&
BakedRigidBodyBatchOwner::diagnostics() const noexcept
{
  return m_diagnostics;
}

//==============================================================================
bool BakedRigidBodyBatchOwner::modelMatches(
    std::span<const std::uint8_t> dynamicMask,
    std::span<const double> inverseMass,
    std::span<const double> inertia,
    std::size_t worldCount,
    std::size_t bodyCount) const
{
  if (!m_modelValid || m_model.worldCount != worldCount
      || m_model.bodyCount != bodyCount
      || m_model.inverseMass.size() != worldCount * bodyCount
      || m_model.inertia.size() != 9 * worldCount * bodyCount
      || !rangeEquals(m_rigidBodyIsDynamic, dynamicMask)) {
    return false;
  }

  const std::span<const double> cachedInverseMass{
      m_model.inverseMass.data(), inverseMass.size()};
  const std::span<const double> cachedInertia{
      m_model.inertia.data(), inertia.size()};
  return rangeEquals(cachedInverseMass, inverseMass)
         && rangeEquals(cachedInertia, inertia);
}

namespace rigid_body_batch_ops {

//==============================================================================
void integrateRigidBodyStateBatchLinear(
    MutableRigidBodyStateBatchView state,
    std::span<const double> force,
    std::span<const double> inverseMass,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
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
void integrateRigidBodyStateBatchLinear(
    MutableRigidBodyStateBatchView state,
    RigidBodyModelBatchView model,
    std::span<const double> force,
    double timeStep)
{
  DART_SIMULATION_THROW_T_IF(
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
    MutableRigidBodyStateBatchView state,
    RigidBodyModelBatchView model,
    std::span<const double> force,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
      state.orientation.size() != 4 * bodies
          || state.angularVelocity.size() != 3 * bodies,
      InvalidArgumentException,
      "RigidBodyStateBatch orientation/angular-velocity arrays are "
      "inconsistent "
      "with worldCount {} and bodyCount {}",
      state.worldCount,
      state.bodyCount);

  DART_SIMULATION_THROW_T_IF(
      model.worldCount != state.worldCount
          || model.bodyCount != state.bodyCount,
      InvalidArgumentException,
      "RigidBodyModelBatch ({}x{}) does not match the state batch ({}x{})",
      model.worldCount,
      model.bodyCount,
      state.worldCount,
      state.bodyCount);

  integrateRigidBodyStateBatchLinear(state, model, force, timeStep);
  integrateOrientationsBatch(
      state.orientation.data(), state.angularVelocity.data(), timeStep, bodies);
}

//==============================================================================
void integrateRigidBodyStateBatch(
    MutableRigidBodyStateBatchView state,
    RigidBodyModelBatchView model,
    std::span<const double> force,
    std::span<const double> torque,
    double timeStep)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
      state.orientation.size() != 4 * bodies
          || state.angularVelocity.size() != 3 * bodies
          || torque.size() != 3 * bodies || model.inertia.size() != 9 * bodies,
      InvalidArgumentException,
      "integrateRigidBodyStateBatch (torque) arrays are inconsistent with "
      "worldCount {} and bodyCount {}",
      state.worldCount,
      state.bodyCount);

  DART_SIMULATION_THROW_T_IF(
      model.worldCount != state.worldCount
          || model.bodyCount != state.bodyCount,
      InvalidArgumentException,
      "RigidBodyModelBatch ({}x{}) does not match the state batch ({}x{})",
      model.worldCount,
      model.bodyCount,
      state.worldCount,
      state.bodyCount);

  integrateRigidBodyStateBatchLinear(state, force, model.inverseMass, timeStep);
  integrateAngularVelocitiesFromTorque(state, model, torque, timeStep);
  integrateOrientationsBatch(
      state.orientation.data(), state.angularVelocity.data(), timeStep, bodies);
}

} // namespace rigid_body_batch_ops

//==============================================================================
RigidBodyStateBatch extractRigidBodyState(const World& world)
{
  const auto& registry = dart::simulation::detail::registryOf(world);
  const auto& model
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);

  RigidBodyStateBatch batch;
  batch.worldCount = 1;
  batch.bodyCount = model.rigidBodyEntities.size();
  batch.position.reserve(3 * batch.bodyCount);
  batch.orientation.reserve(4 * batch.bodyCount);
  batch.linearVelocity.reserve(3 * batch.bodyCount);
  batch.angularVelocity.reserve(3 * batch.bodyCount);
  for (const auto entity : model.rigidBodyEntities) {
    const auto& transform = registry.get<comps::Transform>(entity);
    const auto& velocity = registry.get<comps::Velocity>(entity);

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
  DART_SIMULATION_THROW_T_IF(
      state.worldCount != 1,
      InvalidArgumentException,
      "applyRigidBodyState supports single-world batches only, got worldCount "
      "{}",
      state.worldCount);

  // The fields are independently public, so validate their sizes against
  // bodyCount before indexing to avoid out-of-bounds reads on a malformed
  // batch.
  DART_SIMULATION_THROW_T_IF(
      state.position.size() != 3 * state.bodyCount
          || state.linearVelocity.size() != 3 * state.bodyCount
          || state.angularVelocity.size() != 3 * state.bodyCount
          || state.orientation.size() != 4 * state.bodyCount,
      InvalidArgumentException,
      "RigidBodyStateBatch arrays are inconsistent with bodyCount {}",
      state.bodyCount);

  auto& registry = dart::simulation::detail::registryOf(world);
  const auto& model
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);

  std::size_t index = 0;
  for (const auto entity : model.rigidBodyEntities) {
    DART_SIMULATION_THROW_T_IF(
        index >= state.bodyCount,
        InvalidArgumentException,
        "RigidBodyStateBatch has fewer bodies ({}) than the world",
        state.bodyCount);

    auto& transform = registry.get<comps::Transform>(entity);
    auto& velocity = registry.get<comps::Velocity>(entity);

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

  DART_SIMULATION_THROW_T_IF(
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
  BakedRigidBodyBatchOwner owner;
  owner.captureFromWorlds(worlds);
  return owner.state();
}

//==============================================================================
void applyRigidBodyStateBatch(
    const std::vector<World*>& worlds, const RigidBodyStateBatch& state)
{
  DART_SIMULATION_THROW_T_IF(
      state.worldCount != worlds.size(),
      InvalidArgumentException,
      "RigidBodyStateBatch worldCount ({}) does not match the number of worlds "
      "({})",
      state.worldCount,
      worlds.size());

  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
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

  // Validate every target world (non-null, unique, and matching rigid-body
  // count) before mutating any of them. Because the batch is applied slice by
  // slice, an invalid later world would otherwise leave earlier worlds already
  // mutated -- a non-atomic failure that silently corrupts a multi-world batch
  // (this also makes rolloutWorldsBatched, which applies before stepping, fail
  // atomically on a duplicate world).
  std::unordered_set<const World*> seen;
  const dart::simulation::detail::BakedWorldModel* referenceModel = nullptr;
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_SIMULATION_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "applyRigidBodyStateBatch received a null world at lane {}",
        w);
    DART_SIMULATION_THROW_T_IF(
        !seen.insert(worlds[w]).second,
        InvalidArgumentException,
        "applyRigidBodyStateBatch received a duplicate world at lane {}; each "
        "world slice must target a distinct world",
        w);

    const auto& model
        = dart::simulation::detail::ensureBakedWorldModelCurrent(*worlds[w]);
    DART_SIMULATION_THROW_T_IF(
        model.rigidBodyEntities.size() != bodyCount,
        InvalidArgumentException,
        "applyRigidBodyStateBatch lane {} has {} rigid bodies, expected {}",
        w,
        model.rigidBodyEntities.size(),
        bodyCount);
    if (w == 0) {
      referenceModel = &model;
    } else {
      DART_SIMULATION_THROW_T_IF(
          !sameRigidBodyModelIdentity(model, *referenceModel),
          InvalidArgumentException,
          "applyRigidBodyStateBatch lane {} has a different rigid-body "
          "dense-index Model identity than lane 0; all worlds must expose the "
          "same ordered Model",
          w);
    }
  }

  for (std::size_t w = 0; w < worlds.size(); ++w) {
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
    std::span<const double> force,
    std::span<const double> inverseMass,
    double timeStep)
{
  rigid_body_batch_ops::integrateRigidBodyStateBatchLinear(
      rigid_body_batch_ops::mutableStateBatchView(state),
      force,
      inverseMass,
      timeStep);
}

//==============================================================================
RigidBodyModelBatch extractRigidBodyModelBatch(const World& world)
{
  const auto& bakedModel
      = dart::simulation::detail::ensureBakedWorldModelCurrent(world);

  RigidBodyModelBatch model;
  model.worldCount = 1;
  model.bodyCount = bakedModel.rigidBodyEntities.size();
  model.inverseMass.assign(
      bakedModel.rigidBodyInverseMass.begin(),
      bakedModel.rigidBodyInverseMass.end());
  model.inertia.assign(
      bakedModel.rigidBodyInertia.begin(), bakedModel.rigidBodyInertia.end());

  return model;
}

//==============================================================================
RigidBodyControlSequenceBatch extractRigidBodyControlSequenceBatch(
    const std::vector<const World*>& worlds, std::size_t stepCount)
{
  RigidBodyControlSequenceBatch controlSequence;
  controlSequence.stepCount = stepCount;
  controlSequence.worldCount = worlds.size();

  if (worlds.empty()) {
    controlSequence.worldCount = 0;
    return controlSequence;
  }

  std::vector<const dart::simulation::detail::BakedWorldModel*> laneModels;
  laneModels.reserve(worlds.size());
  const dart::simulation::detail::BakedWorldModel* referenceModel = nullptr;
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    DART_SIMULATION_THROW_T_IF(
        worlds[w] == nullptr,
        InvalidArgumentException,
        "extractRigidBodyControlSequenceBatch received a null world at lane {}",
        w);

    const auto& model
        = dart::simulation::detail::ensureBakedWorldModelCurrent(*worlds[w]);
    if (w == 0) {
      referenceModel = &model;
    } else {
      DART_SIMULATION_THROW_T_IF(
          model.rigidBodyEntities.size()
              != referenceModel->rigidBodyEntities.size(),
          InvalidArgumentException,
          "Heterogeneous body counts: lane 0 has {}, lane {} has {}",
          referenceModel->rigidBodyEntities.size(),
          w,
          model.rigidBodyEntities.size());
      DART_SIMULATION_THROW_T_IF(
          !sameRigidBodyModelIdentity(model, *referenceModel),
          InvalidArgumentException,
          "extractRigidBodyControlSequenceBatch lane {} has a different "
          "rigid-body dense-index Model identity than lane 0; all worlds must "
          "expose the same ordered Model",
          w);
    }
    laneModels.push_back(&model);
  }

  controlSequence.bodyCount = referenceModel->rigidBodyEntities.size();
  std::vector<double> oneStepForce;
  oneStepForce.reserve(3 * worlds.size() * controlSequence.bodyCount);
  for (std::size_t w = 0; w < worlds.size(); ++w) {
    appendRigidBodyControlFromBakedModel(
        *worlds[w], *laneModels[w], oneStepForce);
  }

  controlSequence.force.reserve(stepCount * oneStepForce.size());
  for (std::size_t step = 0; step < stepCount; ++step) {
    controlSequence.force.insert(
        controlSequence.force.end(), oneStepForce.begin(), oneStepForce.end());
  }
  return controlSequence;
}

//==============================================================================
std::span<double> rigidBodyControlForcesAtStep(
    RigidBodyControlSequenceBatch& controlSequence, std::size_t step)
{
  validateRigidBodyControlSequenceStorage(controlSequence);
  DART_SIMULATION_THROW_T_IF(
      step >= controlSequence.stepCount,
      InvalidArgumentException,
      "RigidBodyControlSequenceBatch step {} is out of range for stepCount {}",
      step,
      controlSequence.stepCount);

  const auto stepSize = rigidBodyControlStepSize(controlSequence);
  if (stepSize == 0) {
    return {};
  }
  return std::span<double>{
      controlSequence.force.data() + step * stepSize, stepSize};
}

//==============================================================================
std::span<const double> rigidBodyControlForcesAtStep(
    const RigidBodyControlSequenceBatch& controlSequence, std::size_t step)
{
  validateRigidBodyControlSequenceStorage(controlSequence);
  DART_SIMULATION_THROW_T_IF(
      step >= controlSequence.stepCount,
      InvalidArgumentException,
      "RigidBodyControlSequenceBatch step {} is out of range for stepCount {}",
      step,
      controlSequence.stepCount);

  const auto stepSize = rigidBodyControlStepSize(controlSequence);
  if (stepSize == 0) {
    return {};
  }
  return std::span<const double>{
      controlSequence.force.data() + step * stepSize, stepSize};
}

//==============================================================================
void integrateRigidBodyStateBatchLinear(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::span<const double> force,
    double timeStep)
{
  rigid_body_batch_ops::integrateRigidBodyStateBatchLinear(
      rigid_body_batch_ops::mutableStateBatchView(state),
      rigid_body_batch_ops::modelBatchView(model),
      force,
      timeStep);
}

//==============================================================================
void integrateRigidBodyStateBatch(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::span<const double> force,
    double timeStep)
{
  rigid_body_batch_ops::integrateRigidBodyStateBatch(
      rigid_body_batch_ops::mutableStateBatchView(state),
      rigid_body_batch_ops::modelBatchView(model),
      force,
      timeStep);
}

//==============================================================================
void integrateRigidBodyStateBatch(
    RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::span<const double> force,
    std::span<const double> torque,
    double timeStep)
{
  rigid_body_batch_ops::integrateRigidBodyStateBatch(
      rigid_body_batch_ops::mutableStateBatchView(state),
      rigid_body_batch_ops::modelBatchView(model),
      force,
      torque,
      timeStep);
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

//==============================================================================
RigidBodyStateBatch rolloutRigidBodyStateBatch(
    const RigidBodyStateBatch& initialState,
    const RigidBodyModelBatch& model,
    const RigidBodyControlSequenceBatch& controlSequence,
    double timeStep,
    RigidBodyBatchRolloutDiagnostics* diagnostics)
{
  DART_SIMULATION_THROW_T_IF(
      model.worldCount != initialState.worldCount
          || model.bodyCount != initialState.bodyCount,
      InvalidArgumentException,
      "RigidBodyModelBatch ({}x{}) does not match the initial state batch "
      "({}x{})",
      model.worldCount,
      model.bodyCount,
      initialState.worldCount,
      initialState.bodyCount);
  DART_SIMULATION_THROW_T_IF(
      controlSequence.worldCount != initialState.worldCount
          || controlSequence.bodyCount != initialState.bodyCount,
      InvalidArgumentException,
      "RigidBodyControlSequenceBatch ({} steps, {}x{}) does not match the "
      "initial state batch ({}x{})",
      controlSequence.stepCount,
      controlSequence.worldCount,
      controlSequence.bodyCount,
      initialState.worldCount,
      initialState.bodyCount);
  validateRigidBodyControlSequenceStorage(controlSequence);

  if (diagnostics != nullptr) {
    *diagnostics = RigidBodyBatchRolloutDiagnostics{
        RigidBodyBatchExecutionShape::HomogeneousBatch,
        controlSequence.stepCount,
        initialState.worldCount,
        initialState.bodyCount};
  }

  RigidBodyStateBatch state = initialState;
  for (std::size_t step = 0; step < controlSequence.stepCount; ++step) {
    integrateRigidBodyStateBatch(
        state,
        model,
        rigidBodyControlForcesAtStep(controlSequence, step),
        timeStep);
  }
  return state;
}

//==============================================================================
double totalKineticEnergy(
    const RigidBodyStateBatch& state,
    const RigidBodyModelBatch& model,
    std::size_t chunkSize)
{
  const auto bodies = state.worldCount * state.bodyCount;
  DART_SIMULATION_THROW_T_IF(
      state.linearVelocity.size() != 3 * bodies
          || model.inverseMass.size() != bodies,
      InvalidArgumentException,
      "totalKineticEnergy arrays are inconsistent with worldCount {} and "
      "bodyCount {}",
      state.worldCount,
      state.bodyCount);

  if (bodies == 0) {
    return 0.0;
  }

  const std::size_t chunk = std::max<std::size_t>(1, chunkSize);

  // Sum each fixed-size chunk independently (a chunk is the unit a worker would
  // own), then merge the partials in chunk order. The fixed merge order makes
  // the result independent of how chunks are scheduled.
  std::vector<double> partials;
  partials.reserve((bodies + chunk - 1) / chunk);
  for (std::size_t begin = 0; begin < bodies; begin += chunk) {
    const auto end = std::min(begin + chunk, bodies);
    double partial = 0.0;
    for (std::size_t b = begin; b < end; ++b) {
      const double inverseMass = model.inverseMass[b];
      if (inverseMass <= 0.0) {
        continue;
      }
      const double vx = state.linearVelocity[3 * b + 0];
      const double vy = state.linearVelocity[3 * b + 1];
      const double vz = state.linearVelocity[3 * b + 2];
      partial += 0.5 * (vx * vx + vy * vy + vz * vz) / inverseMass;
    }
    partials.push_back(partial);
  }

  double total = 0.0;
  for (const double partial : partials) {
    total += partial;
  }
  return total;
}

} // namespace dart::simulation::compute
