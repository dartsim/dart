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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SoftFootSimbiconModel.hpp"

#include "../atlas_simbicon/Controller.hpp"

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <stdexcept>
#include <string>

namespace dart_demos {
namespace soft_foot_simbicon_model {

namespace {

// Steps to let the biped settle into its walking-in-place gait before a push.
constexpr int kSettleSteps = 500;

// Steps to observe recovery after a push window closes, and the settled tail
// (final steps) over which the biped must stay upright to count as recovered.
constexpr int kRecoverySteps = 800;
constexpr int kSettledTailSteps = 250;

// Lateral (coronal, world +Z) push-magnitude sweep. Lateral perturbations are
// the harder push-recovery axis (narrow coronal base of support) and are the
// perturbation the Jain/Liu row highlights. The range brackets the biped's
// actual topple threshold: both foot types comfortably recover the lower
// magnitudes and topple at the top of the range, so the returned value is a
// real measured threshold, not a saturated sweep ceiling.
constexpr double kPushSweepStart = 2000.0;
constexpr double kPushSweepEnd = 12000.0;
constexpr double kPushSweepStep = 2000.0;
const Eigen::Vector3d kPushDirection = Eigen::Vector3d::UnitZ();

//==============================================================================
std::string sdfUriFor(Feet feet)
{
  return feet == Feet::Soft
             ? "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf"
             : "dart://sample/sdf/atlas/atlas_v3_no_head.sdf";
}

//==============================================================================
/// Builds a fresh model, settles it, applies a single lateral push of the given
/// magnitude, and reports whether the biped stays finite and upright throughout
/// the settled tail of the recovery window. Requiring uprightness across the
/// whole tail (not just the final instant) rejects a biped that is still
/// toppling or momentarily passes through an upright pose while tumbling.
bool survivesPush(Feet feet, double magnitude)
{
  Model model = createModel(feet);

  for (int i = 0; i < kSettleSteps; ++i) {
    step(model);
    if (!isFinite(model))
      return false;
  }

  applyPush(model, kPushDirection * magnitude, kPushSteps);

  const int tailStart = kRecoverySteps - kSettledTailSteps;
  for (int i = 0; i < kRecoverySteps; ++i) {
    step(model);
    if (!isFinite(model))
      return false;
    if (i >= tailStart && !isUpright(model))
      return false;
  }

  return true;
}

} // namespace

//==============================================================================
Model createModel(Feet feet)
{
  Model model;
  model.feet = feet;

  auto world = dart::simulation::World::create();
  world->setTimeStep(kTimeStep);
  // Single-threaded stepping so two runs of the same model are bit-identical.
  world->setNumSimulationThreads(1);

  dart::utils::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton("dart://sample/sdf/atlas/ground.urdf");
  if (!ground)
    throw std::runtime_error(
        "failed to load dart://sample/sdf/atlas/ground.urdf");

  auto atlas = dart::utils::SdfParser::readSkeleton(sdfUriFor(feet));
  if (!atlas)
    throw std::runtime_error("failed to load " + sdfUriFor(feet));

  world->addSkeleton(ground);
  world->addSkeleton(atlas);

  // Y-up, deliberately not reoriented: State::getCOMFrame() and
  // Controller::isAllowingControl() hardcode world-Y as vertical (see the
  // AtlasSimbiconScene file comment). The root free-joint spin stands Atlas up.
  atlas->setPosition(0, -0.5 * dart::math::constantsd::pi());
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto* pelvis = atlas->getBodyNode("pelvis");
  auto* leftFoot = atlas->getBodyNode("l_foot");
  auto* rightFoot = atlas->getBodyNode("r_foot");
  if (!pelvis || !leftFoot || !rightFoot)
    throw std::runtime_error(
        sdfUriFor(feet) + ": missing 'pelvis', 'l_foot', or 'r_foot'");

  model.world = world;
  model.atlas = atlas;
  model.pelvis = pelvis;
  model.leftFoot = leftFoot;
  model.rightFoot = rightFoot;
  model.controller = std::make_shared<atlas_simbicon::Controller>(
      atlas, world->getConstraintSolver());

  return model;
}

//==============================================================================
void applyPush(Model& model, const Eigen::Vector3d& force, int steps)
{
  model.externalForce = force;
  model.forceDuration = steps;
}

//==============================================================================
void prepareStep(Model& model)
{
  // Order matches AtlasSimbiconScene::preStep: apply the pending pelvis force,
  // run one controller update, then count the force window down.
  model.pelvis->addExtForce(model.externalForce);
  model.controller->update();

  if (model.forceDuration > 0)
    --model.forceDuration;
  else
    model.externalForce.setZero();
}

//==============================================================================
void step(Model& model)
{
  prepareStep(model);
  model.world->step();
}

//==============================================================================
double pelvisHeightY(const Model& model)
{
  return model.pelvis->getTransform().translation().y();
}

//==============================================================================
double meanFootHeightY(const Model& model)
{
  return 0.5
         * (model.leftFoot->getTransform().translation().y()
            + model.rightFoot->getTransform().translation().y());
}

//==============================================================================
bool isUpright(const Model& model)
{
  return (pelvisHeightY(model) - meanFootHeightY(model)) > kUprightGap;
}

//==============================================================================
std::size_t footContactCount(const Model& model)
{
  const auto& result = model.world->getLastCollisionResult();
  std::size_t count = 0;
  for (const auto& contact : result.getContacts()) {
    const auto* body1 = contact.getBodyNodePtr1().get();
    const auto* body2 = contact.getBodyNodePtr2().get();
    if (body1 == model.leftFoot || body1 == model.rightFoot
        || body2 == model.leftFoot || body2 == model.rightFoot)
      ++count;
  }
  return count;
}

//==============================================================================
bool isFinite(const Model& model)
{
  const auto& atlas = model.atlas;
  if (!atlas->getPositions().allFinite() || !atlas->getVelocities().allFinite())
    return false;

  for (std::size_t i = 0; i < atlas->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = atlas->getSoftBodyNode(i);
    if (!softBody->getTransform().matrix().allFinite())
      return false;
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const auto* point = softBody->getPointMass(j);
      if (!point->getPositions().allFinite()
          || !point->getVelocities().allFinite()
          || !point->getWorldPosition().allFinite()) {
        return false;
      }
    }
  }
  return true;
}

//==============================================================================
double checksum(const Model& model)
{
  long double checksum = 0.0;
  std::size_t component = 1;
  const auto positions = model.atlas->getPositions();
  for (Eigen::Index i = 0; i < positions.size(); ++i)
    checksum += static_cast<long double>(component++) * positions[i];

  for (std::size_t i = 0; i < model.atlas->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = model.atlas->getSoftBodyNode(i);
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const Eigen::Vector3d& position
          = softBody->getPointMass(j)->getWorldPosition();
      for (int axis = 0; axis < 3; ++axis)
        checksum += static_cast<long double>(component++) * position[axis];
    }
  }
  return static_cast<double>(checksum);
}

//==============================================================================
double maxRecoverablePush(Feet feet)
{
  double largestRecovered = 0.0;
  for (double magnitude = kPushSweepStart; magnitude <= kPushSweepEnd + 1e-9;
       magnitude += kPushSweepStep) {
    if (survivesPush(feet, magnitude))
      largestRecovered = magnitude;
  }
  return largestRecovered;
}

} // namespace soft_foot_simbicon_model
} // namespace dart_demos
