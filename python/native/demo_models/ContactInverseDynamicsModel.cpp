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

#include "ContactInverseDynamicsModel.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

#include <cmath>

namespace dart_demos {
namespace {

using dart::dynamics::BodyNode;

//==============================================================================
void findTorqueDofs(ContactInverseDynamicsModel& state)
{
  state.torqueNames.clear();
  state.torqueDofs.clear();

  static const char* const candidates[]
      = {"j_thigh_left_z",
         "j_shin_left",
         "j_heel_left_1",
         "j_thigh_right_z",
         "j_shin_right",
         "j_heel_right_1"};

  for (const char* name : candidates) {
    auto* dof = state.biped->getDof(name);
    if (dof && state.torqueNames.size() < 3) {
      state.torqueNames.emplace_back(name);
      state.torqueDofs.push_back(dof->getIndexInSkeleton());
    }
  }
}

//==============================================================================
void generateSquatKeyframes(ContactInverseDynamicsModel& state, double depth)
{
  state.biped->setPositions(state.initialPositions);

  constexpr std::size_t numKeyframes = 60;
  state.keyframes.clear();
  state.keyframes.reserve(numKeyframes);

  auto leftHeelIK = state.leftHeel->getIK(true);
  auto rightHeelIK = state.rightHeel->getIK(true);

  auto leftHeelTarget = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(),
      "leftHeelTarget",
      state.leftHeel->getWorldTransform());
  auto rightHeelTarget = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(),
      "rightHeelTarget",
      state.rightHeel->getWorldTransform());

  leftHeelIK->setTarget(leftHeelTarget);
  leftHeelIK->setHierarchyLevel(0);
  rightHeelIK->setTarget(rightHeelTarget);
  rightHeelIK->setHierarchyLevel(0);

  auto* pelvis = state.biped->getBodyNode(0);
  auto pelvisIK = pelvis->getIK(true);
  auto pelvisTarget = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(),
      "pelvisTarget",
      pelvis->getWorldTransform());
  pelvisIK->setTarget(pelvisTarget);
  pelvisIK->setHierarchyLevel(1);

  const Eigen::Isometry3d pelvisInitial = pelvis->getWorldTransform();

  auto wbIK = state.biped->getIK(true);
  if (auto solver = wbIK->getSolver()) {
    if (auto gd
        = std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
            solver))
      gd->setNumMaxIterations(200);
  }

  for (std::size_t i = 0; i < numKeyframes; ++i) {
    const double phase
        = static_cast<double>(i) / static_cast<double>(numKeyframes);
    const double offset
        = depth * 0.5
          * (1.0 - std::cos(2.0 * dart::math::constantsd::pi() * phase));

    Eigen::Isometry3d pelvisTf = pelvisInitial;
    // Z is "up" in this host's reoriented world (was Y in the original
    // Y-up asset) -- see the file comment.
    pelvisTf.translation()[2] -= offset;
    pelvisTarget->setTransform(pelvisTf);

    wbIK->solveAndApply(true);
    state.keyframes.push_back(state.biped->getPositions());

    if (i == 0) {
      if (auto solver = wbIK->getSolver()) {
        if (auto gd
            = std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
                solver))
          gd->setNumMaxIterations(50);
      }
    }
  }

  state.biped->setPositions(state.initialPositions);
}

//==============================================================================
Eigen::VectorXd catmullRomInterpolate(
    const ContactInverseDynamicsModel& state, double t)
{
  if (state.keyframes.empty())
    return state.biped->getPositions();

  const std::size_t n = state.keyframes.size();
  const double scaledT = t * static_cast<double>(n);
  const std::size_t i1 = static_cast<std::size_t>(scaledT) % n;
  const double u = scaledT - std::floor(scaledT);

  const std::size_t i0 = (i1 + n - 1) % n;
  const std::size_t i2 = (i1 + 1) % n;
  const std::size_t i3 = (i1 + 2) % n;

  const auto& p0 = state.keyframes[i0];
  const auto& p1 = state.keyframes[i1];
  const auto& p2 = state.keyframes[i2];
  const auto& p3 = state.keyframes[i3];

  const double u2 = u * u;
  const double u3 = u2 * u;

  return 0.5
         * ((-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2 + (-p0 + p2) * u
            + 2.0 * p1);
}

//==============================================================================
Eigen::VectorXd catmullRomVelocity(
    const ContactInverseDynamicsModel& state, double t)
{
  if (state.keyframes.empty())
    return Eigen::VectorXd::Zero(state.biped->getNumDofs());

  const std::size_t n = state.keyframes.size();
  const double scaledT = t * static_cast<double>(n);
  const std::size_t i1 = static_cast<std::size_t>(scaledT) % n;
  const double u = scaledT - std::floor(scaledT);

  const std::size_t i0 = (i1 + n - 1) % n;
  const std::size_t i2 = (i1 + 1) % n;
  const std::size_t i3 = (i1 + 2) % n;

  const auto& p0 = state.keyframes[i0];
  const auto& p1 = state.keyframes[i1];
  const auto& p2 = state.keyframes[i2];
  const auto& p3 = state.keyframes[i3];

  const double u2 = u * u;

  return 0.5 * static_cast<double>(n)
         * ((3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3)) * u2
            + (2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3)) * u + (-p0 + p2));
}

//==============================================================================
Eigen::VectorXd catmullRomAcceleration(
    const ContactInverseDynamicsModel& state, double t)
{
  if (state.keyframes.empty())
    return Eigen::VectorXd::Zero(state.biped->getNumDofs());

  const std::size_t n = state.keyframes.size();
  const double scaledT = t * static_cast<double>(n);
  const std::size_t i1 = static_cast<std::size_t>(scaledT) % n;
  const double u = scaledT - std::floor(scaledT);

  const std::size_t i0 = (i1 + n - 1) % n;
  const std::size_t i2 = (i1 + 1) % n;
  const std::size_t i3 = (i1 + 2) % n;

  const auto& p0 = state.keyframes[i0];
  const auto& p1 = state.keyframes[i1];
  const auto& p2 = state.keyframes[i2];
  const auto& p3 = state.keyframes[i3];

  const double nSq = static_cast<double>(n * n);

  return 0.5 * nSq
         * ((6.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3)) * u
            + 2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3));
}

//==============================================================================
/// Kinematic replay and contact-consistent inverse dynamics for one refresh.
void updateFrameImpl(ContactInverseDynamicsModel& state)
{
  // Consume a deferred keyframe-regeneration request at the top of the frame,
  // before the spline is evaluated below, so the pose drawn this frame comes
  // from the fresh keyframes rather than popping to initialPositions for one
  // frame (mirrors the original's deferred mRegenerateRequested).
  if (state.regenerateRequested) {
    generateSquatKeyframes(state, state.squatDepth);
    state.regenerateRequested = false;
  }

  constexpr double cycleDuration = 3.0;
  if (state.playing)
    state.time += 1.0 / 60.0 * static_cast<double>(state.playbackSpeed);

  const double phase = std::fmod(state.time, cycleDuration) / cycleDuration;
  const double phaseRate
      = state.playing ? static_cast<double>(state.playbackSpeed) / cycleDuration
                      : 0.0;

  const Eigen::VectorXd q = catmullRomInterpolate(state, phase);
  const Eigen::VectorXd dq = catmullRomVelocity(state, phase) * phaseRate;
  const Eigen::VectorXd ddq
      = catmullRomAcceleration(state, phase) * phaseRate * phaseRate;

  state.biped->setPositions(q);
  state.biped->setVelocities(dq);
  state.biped->setAccelerations(ddq);

  const Eigen::Vector3d upDir = -state.world->getGravity().normalized();

  std::vector<dart::dynamics::ContactInverseDynamics::Contact> contacts;
  const std::array<BodyNode*, 4> contactBodies
      = {state.leftHeel, state.leftToe, state.rightHeel, state.rightToe};

  for (auto* body : contactBodies) {
    dart::dynamics::ContactInverseDynamics::Contact contact;
    contact.bodyNode = body;
    contact.localOffset = Eigen::Vector3d::Zero();
    contact.normal = upDir;
    contact.frictionCoeff = state.frictionCoeff;
    contact.numBasis = 8;
    contacts.push_back(contact);
  }

  state.solver->setContacts(contacts);
  const auto result = state.solver->compute();

  state.feasible = result.feasible;
  state.residualNorm = result.unactuatedResidual.lpNorm<Eigen::Infinity>();
  state.contactForces = result.contactForces;

  state.totalVerticalForce = 0.0;
  for (const auto& f : state.contactForces)
    state.totalVerticalForce += f.dot(upDir);
  state.plotSamples[state.sampleIndex]
      = static_cast<float>(state.totalVerticalForce);

  for (std::size_t i = 0;
       i < state.torqueDofs.size() && i < state.torquePlots.size();
       ++i) {
    if (state.torqueDofs[i]
        < static_cast<std::size_t>(result.jointForces.size()))
      state.torquePlots[i][state.sampleIndex]
          = static_cast<float>(result.jointForces[state.torqueDofs[i]]);
  }
  state.sampleIndex = (state.sampleIndex + 1) % state.plotSamples.size();

  for (std::size_t i = 0;
       i < result.contactForces.size() && i < state.contactFrames.size();
       ++i) {
    auto* visual = state.contactFrames[i]->getVisualAspect(true);
    const Eigen::Vector3d force = result.contactForces[i];
    if (force.norm() < 1e-8) {
      visual->setHidden(true);
      continue;
    }
    visual->setHidden(false);

    const Eigen::Vector3d worldPos
        = contactBodies[i]->getWorldTransform() * contacts[i].localOffset;
    const Eigen::Vector3d headPos = worldPos + state.forceScale * force;
    state.arrows[i]->setPositions(worldPos, headPos);

    const Eigen::Vector4d color = result.feasible
                                      ? Eigen::Vector4d(0.0, 1.0, 0.0, 1.0)
                                      : Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
    state.arrows[i]->notifyColorUpdated(color);
  }
}

//==============================================================================
float finiteControl(float value, float min, float max)
{
  if (!std::isfinite(value))
    throw std::invalid_argument("control value must be finite");
  return std::clamp(value, min, max);
}

} // namespace

//==============================================================================
ContactInverseDynamicsModel::ContactInverseDynamicsModel(
    dart::simulation::WorldPtr sceneWorld)
  : world(std::move(sceneWorld))
{
  if (!world)
    throw std::invalid_argument("world must not be null");
  if (!world->getGravity().allFinite() || world->getGravity().norm() == 0.0)
    throw std::invalid_argument("world gravity must be finite and nonzero");
  biped = world->getSkeleton("fullbody1");
  if (!biped)
    throw std::runtime_error("fullbody1.skel: missing skeleton 'fullbody1'");

  const auto setDofPosition = [&](const char* name, double position) {
    if (auto* dof = biped->getDof(name))
      dof->setPosition(position);
  };
  setDofPosition("j_pelvis_rot_y", -0.20);
  setDofPosition("j_thigh_left_z", 0.15);
  setDofPosition("j_shin_left", -0.40);
  setDofPosition("j_heel_left_1", 0.25);
  setDofPosition("j_thigh_right_z", 0.15);
  setDofPosition("j_shin_right", -0.40);
  setDofPosition("j_heel_right_1", 0.25);
  setDofPosition("j_abdomen_2", 0.00);

  solver = std::make_unique<dart::dynamics::ContactInverseDynamics>(biped);
  // A stronger-than-default regularization keeps the (statically
  // indeterminate) force distribution even and steady across frames; the
  // residual tolerance is widened to match the equality bias it introduces.
  solver->setRegularization(1e-4);
  solver->setResidualTolerance(1e-4);

  leftHeel = biped->getBodyNode("h_heel_left");
  leftToe = biped->getBodyNode("h_toe_left");
  rightHeel = biped->getBodyNode("h_heel_right");
  rightToe = biped->getBodyNode("h_toe_right");
  if (!leftHeel || !leftToe || !rightHeel || !rightToe)
    throw std::runtime_error(
        "fullbody1.skel: missing one or more foot body nodes");

  for (std::size_t i = 0; i < 4; ++i) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World());
    contactFrames.push_back(frame);
    world->addSimpleFrame(frame);

    auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitZ() * 0.1,
        dart::dynamics::ArrowShape::Properties(0.01, 2.0, 0.15),
        Eigen::Vector4d(0.0, 1.0, 0.0, 1.0));
    arrows.push_back(arrow);
    frame->setShape(arrow);
    frame->getVisualAspect(true);
  }

  findTorqueDofs(*this);
  initialPositions = biped->getPositions();
  generateSquatKeyframes(*this, 0.15);
}

//==============================================================================
void ContactInverseDynamicsModel::updateFrame()
{
  updateFrameImpl(*this);
}

//==============================================================================
void ContactInverseDynamicsModel::requestRegeneration()
{
  regenerateRequested = true;
}

//==============================================================================
void ContactInverseDynamicsModel::setPlaybackSpeed(float value)
{
  playbackSpeed = finiteControl(value, 0.1f, 2.0f);
}

//==============================================================================
void ContactInverseDynamicsModel::setSquatDepth(float value)
{
  squatDepth = finiteControl(value, 0.0f, 0.25f);
}

//==============================================================================
void ContactInverseDynamicsModel::setFrictionCoeff(float value)
{
  frictionCoeff = finiteControl(value, 0.0f, 1.5f);
}

//==============================================================================
void ContactInverseDynamicsModel::setForceScale(float value)
{
  forceScale = finiteControl(value, 0.0001f, 0.01f);
}

} // namespace dart_demos
