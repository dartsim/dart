/*
 * Copyright (c) 2011-2026, The DART development contributors
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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Headless soft-body driver for deterministic checksums and text profiling.
// Usage:
//   soft_body_headless [scene=soft_bodies] [steps=2000] [checkpoint=500]
// THREADS=N sets World::setNumSimulationThreads(N).
// COLLISION_DETECTOR=name selects a collision backend from the DART factory.

#include <dart/collision/CollisionDetector.hpp>
#include <dart/common/Profile.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/SkelParser.hpp>

#if HAVE_BULLET
  #include <dart/collision/bullet/bullet.hpp>
#endif
#if HAVE_ODE
  #include <dart/collision/ode/ode.hpp>
#endif

#include <Eigen/Dense>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>

namespace {

struct SoftScene
{
  const char* name;
  const char* uri;
};

constexpr std::array<SoftScene, 7> kScenes = {{
    {"drop_box", "dart://sample/skel/test/test_drop_box.skel"},
    {"drop_low_stiffness",
     "dart://sample/skel/test/test_drop_low_stiffness.skel"},
    {"double_pendulum",
     "dart://sample/skel/test/test_double_pendulum.skel"},
    {"adaptive_deformable",
     "dart://sample/skel/test/test_adaptive_deformable.skel"},
    {"soft_cubes", "dart://sample/skel/soft_cubes.skel"},
    {"soft_bodies", "dart://sample/skel/softBodies.skel"},
    {"soft_open_chain", "dart://sample/skel/soft_open_chain.skel"},
}};

struct SceneRef
{
  std::string name;
  std::string uri;
};

// This harness is overlaid onto older revisions by the benchmark comparison
// script, so adaptive-activation touchpoints feature-detect the API instead
// of assuming it exists.
template <typename T, typename = void>
struct HasAdaptiveContactActivation : std::false_type
{
};

template <typename T>
struct HasAdaptiveContactActivation<
    T,
    std::void_t<decltype(std::declval<T&>().setAdaptiveContactActivationEnabled(
        true))>> : std::true_type
{
};

template <typename SoftBody>
std::size_t getActivePointMassCount(const SoftBody& softBody)
{
  if constexpr (HasAdaptiveContactActivation<SoftBody>::value)
    return softBody.getNumActivePointMasses();
  else
    return softBody.getNumPointMasses();
}

template <typename SoftBody>
bool tryEnableAdaptiveContactActivation(SoftBody& softBody)
{
  if constexpr (HasAdaptiveContactActivation<SoftBody>::value) {
    softBody.setAdaptiveContactActivationEnabled(true);
    return true;
  } else {
    return false;
  }
}

struct Checksum
{
  double skelPosL1 = 0.0;
  double skelPosSq = 0.0;
  double skelVelL1 = 0.0;
  double skelVelSq = 0.0;
  double pointPosL1 = 0.0;
  double pointPosSq = 0.0;
  double pointVelL1 = 0.0;
  double pointVelSq = 0.0;
  double pointWorldPosL1 = 0.0;
  double pointWorldPosSq = 0.0;
  std::size_t dofs = 0u;
  std::size_t softBodies = 0u;
  std::size_t pointMasses = 0u;
  std::size_t activePointMasses = 0u;
};

std::size_t parseSize(const char* value, std::size_t fallback)
{
  if (value == nullptr)
    return fallback;

  char* end = nullptr;
  const auto parsed = std::strtoull(value, &end, 10);
  if (end == value)
    return fallback;

  return static_cast<std::size_t>(parsed);
}

bool parseBoolEnv(const char* value)
{
  if (value == nullptr || value[0] == '\0')
    return false;

  const std::string text(value);
  return text == "1" || text == "true" || text == "TRUE" || text == "on"
         || text == "ON" || text == "yes" || text == "YES";
}

SceneRef resolveScene(const char* requested)
{
  const std::string value = requested != nullptr ? requested : "soft_bodies";
  for (const auto& scene : kScenes) {
    if (value == scene.name)
      return {scene.name, scene.uri};
  }

  return {"custom", value};
}

template <typename Derived>
void accumulateVector(
    const Eigen::MatrixBase<Derived>& vector, double& l1, double& squared)
{
  l1 += vector.cwiseAbs().sum();
  squared += vector.squaredNorm();
}

Checksum computeChecksum(const dart::simulation::WorldPtr& world)
{
  Checksum checksum;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    const Eigen::VectorXd positions = skeleton->getPositions();
    const Eigen::VectorXd velocities = skeleton->getVelocities();
    accumulateVector(positions, checksum.skelPosL1, checksum.skelPosSq);
    accumulateVector(velocities, checksum.skelVelL1, checksum.skelVelSq);
    checksum.dofs += static_cast<std::size_t>(positions.size());

    checksum.softBodies += skeleton->getNumSoftBodyNodes();
    for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
      const auto* softBody = skeleton->getSoftBodyNode(j);
      if (softBody == nullptr)
        continue;

      checksum.activePointMasses += getActivePointMassCount(*softBody);
      for (std::size_t k = 0; k < softBody->getNumPointMasses(); ++k) {
        const auto* pointMass = softBody->getPointMass(k);
        if (pointMass == nullptr)
          continue;

        accumulateVector(
            pointMass->getPositions(),
            checksum.pointPosL1,
            checksum.pointPosSq);
        accumulateVector(
            pointMass->getVelocities(),
            checksum.pointVelL1,
            checksum.pointVelSq);
        accumulateVector(
            pointMass->getWorldPosition(),
            checksum.pointWorldPosL1,
            checksum.pointWorldPosSq);
        ++checksum.pointMasses;
      }
    }
  }

  return checksum;
}

void printChecksum(
    std::size_t step, const Checksum& checksum, bool printActivation)
{
  std::printf(
      "step %6zu  dofs %5zu  soft_bodies %4zu  point_masses %5zu  "
      "skelPosL1 %.17g  skelPosSq %.17g  skelVelL1 %.17g  skelVelSq %.17g  "
      "pointPosL1 %.17g  pointPosSq %.17g  pointVelL1 %.17g  pointVelSq %.17g  "
      "pointWorldPosL1 %.17g  pointWorldPosSq %.17g",
      step,
      checksum.dofs,
      checksum.softBodies,
      checksum.pointMasses,
      checksum.skelPosL1,
      checksum.skelPosSq,
      checksum.skelVelL1,
      checksum.skelVelSq,
      checksum.pointPosL1,
      checksum.pointPosSq,
      checksum.pointVelL1,
      checksum.pointVelSq,
      checksum.pointWorldPosL1,
      checksum.pointWorldPosSq);
  if (printActivation) {
    std::printf(
        "  active_point_masses %5zu  inactive_point_masses %5zu",
        checksum.activePointMasses,
        checksum.pointMasses - checksum.activePointMasses);
  }
  std::printf("\n");
}

void keepOptionalCollisionBackendsLinked()
{
#if HAVE_BULLET
  const auto* bulletType
      = &dart::collision::BulletCollisionDetector::getStaticType();
  (void)bulletType;
#endif
#if HAVE_ODE
  const auto* odeType = &dart::collision::OdeCollisionDetector::getStaticType();
  (void)odeType;
#endif
}

} // namespace

int main(int argc, char** argv)
{
  keepOptionalCollisionBackendsLinked();

  const SceneRef scene = resolveScene(argc > 1 ? argv[1] : nullptr);
  const std::size_t steps = parseSize(argc > 2 ? argv[2] : nullptr, 2000u);
  const std::size_t checkpoint
      = parseSize(argc > 3 ? argv[3] : nullptr, 500u);

  dart::simulation::WorldPtr world = dart::utils::SkelParser::readWorld(
      scene.uri);
  if (!world) {
    std::fprintf(stderr, "failed to load soft-body scene: %s\n", scene.uri.c_str());
    return 2;
  }

  std::size_t threads = 1u;
  if (const char* threadsEnv = std::getenv("THREADS"))
    threads = parseSize(threadsEnv, 1u);

  const char* detectorEnv = std::getenv("COLLISION_DETECTOR");
  if (detectorEnv != nullptr && detectorEnv[0] != '\0') {
    auto detector
        = dart::collision::CollisionDetector::getFactory()->create(detectorEnv);
    if (!detector) {
      std::fprintf(
          stderr, "failed to create collision detector: %s\n", detectorEnv);
      return 3;
    }
    world->getConstraintSolver()->setCollisionDetector(detector);
  }

  const bool enableAdaptiveContactActivation
      = parseBoolEnv(std::getenv("ADAPTIVE_CONTACT_ACTIVATION"));
  if (enableAdaptiveContactActivation) {
    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      const auto skeleton = world->getSkeleton(i);
      if (!skeleton)
        continue;
      for (std::size_t j = 0; j < skeleton->getNumSoftBodyNodes(); ++j) {
        auto* softBody = skeleton->getSoftBodyNode(j);
        if (softBody && !tryEnableAdaptiveContactActivation(*softBody)) {
          std::fprintf(
              stderr,
              "ADAPTIVE_CONTACT_ACTIVATION requested but this revision has "
              "no adaptive activation API; running all-active\n");
        }
      }
    }
  }

  world->setNumSimulationThreads(threads);

  const auto detector = world->getConstraintSolver()->getCollisionDetector();
  const std::string detectorName = detector ? detector->getType() : "none";

  const Checksum initial = computeChecksum(world);
  std::printf(
      "# soft_body_headless scene=%s uri=%s steps=%zu checkpoint=%zu "
      "threads=%zu detector=%s time_step=%.17g\n",
      scene.name.c_str(),
      scene.uri.c_str(),
      steps,
      checkpoint,
      threads,
      detectorName.c_str(),
      world->getTimeStep());
  printChecksum(0u, initial, enableAdaptiveContactActivation);

  const bool previousProfileRecording
      = dart::common::profile::setProfileRecordingEnabled(true);
  dart::common::profile::resetProfile();
  const auto start = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < steps; ++i) {
    world->step();
    dart::common::profile::markProfileFrame();

    const std::size_t done = i + 1u;
    if ((checkpoint != 0u && done % checkpoint == 0u) || done == steps)
      printChecksum(
          done, computeChecksum(world), enableAdaptiveContactActivation);
  }
  const auto stop = std::chrono::steady_clock::now();

  const double elapsedMs
      = std::chrono::duration<double, std::milli>(stop - start).count();
  std::printf(
      "elapsed_ms %.3f  steps_per_s %.1f\n",
      elapsedMs,
      elapsedMs > 0.0 ? (1000.0 * static_cast<double>(steps) / elapsedMs)
                      : 0.0);

  DART_PROFILE_TEXT_DUMP();
  dart::common::profile::setProfileRecordingEnabled(previousProfileRecording);

  return 0;
}
