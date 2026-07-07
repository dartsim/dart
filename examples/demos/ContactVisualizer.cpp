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

#include "ContactVisualizer.hpp"

#include <algorithm>

#include <cmath>

namespace dart_demos {

namespace {

// Arrow length per unit force magnitude and the cool -> hot color ramp used
// to encode magnitude, both ported as-is from RigidCubesScene's original
// per-scene contact-force visualization (kLiveContactForceScale there).
constexpr double kForceLengthScale = 0.1;

Eigen::Vector4d magnitudeColor(double normalized)
{
  normalized = std::clamp(normalized, 0.0, 1.0);
  // Cool blue (low) -> hot red (high), with a bright yellow midpoint.
  const Eigen::Vector3d low(0.15, 0.35, 0.95);
  const Eigen::Vector3d mid(1.0, 0.85, 0.15);
  const Eigen::Vector3d high(0.95, 0.15, 0.1);
  Eigen::Vector3d color;
  if (normalized < 0.5)
    color = low + (mid - low) * (normalized / 0.5);
  else
    color = mid + (high - mid) * ((normalized - 0.5) / 0.5);
  return Eigen::Vector4d(color.x(), color.y(), color.z(), 1.0);
}

} // namespace

//==============================================================================
void ContactVisualizer::onSceneInstalled(
    const dart::simulation::WorldPtr& world)
{
  mWorld = world;
  mFrames.clear();
  mArrows.clear();
  mLastVisualizedCount = 0;
}

//==============================================================================
void ContactVisualizer::reset()
{
  mWorld.reset();
  mFrames.clear();
  mArrows.clear();
  mLastVisualizedCount = 0;
}

//==============================================================================
void ContactVisualizer::ensurePool(std::size_t count)
{
  if (!mWorld)
    return;

  while (mFrames.size() < count) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World());
    auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitZ() * 0.01,
        dart::dynamics::ArrowShape::Properties(0.004, 2.0, 0.15),
        Eigen::Vector4d(0.2, 0.2, 0.8, 1.0));

    frame->setShape(arrow);
    frame->createVisualAspect();
    frame->getVisualAspect()->setHidden(true);
    mWorld->addSimpleFrame(frame);

    mFrames.push_back(frame);
    mArrows.push_back(arrow);
  }
}

//==============================================================================
void ContactVisualizer::hideFrom(std::size_t start)
{
  for (std::size_t i = start; i < mFrames.size(); ++i)
    mFrames[i]->getVisualAspect(true)->setHidden(true);
}

//==============================================================================
void ContactVisualizer::applyPostStep()
{
  if (!mEnabled || !mWorld) {
    hideFrom(0);
    mLastVisualizedCount = 0;
    return;
  }

  const auto& result = mWorld->getLastCollisionResult();
  const auto& contacts = result.getContacts();
  const std::size_t count = std::min(contacts.size(), kMaxArrows);
  ensurePool(count);

  // Cap the drawn arrow length so a huge (but finite) contact force on a
  // diverging scene cannot fling the arrow head far enough to blow out OSG's
  // automatic near/far and render the whole scene undrawable.
  constexpr double kMaxArrowLength = 5.0; // meters

  double maxMag = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    if (contacts[i].force.allFinite())
      maxMag = std::max(maxMag, contacts[i].force.norm());
  }

  for (std::size_t i = 0; i < count; ++i) {
    const Eigen::Vector3d& point = contacts[i].point;
    const Eigen::Vector3d& force = contacts[i].force;
    const double mag = force.norm();
    auto* visual = mFrames[i]->getVisualAspect(true);

    // NaN/Inf force or point (a diverging LCP solve is the common case for an
    // interactive debugging tool) would poison the arrow mesh vertices and
    // OSG's bounding-sphere math -- skip such contacts entirely. `mag < 1e-8`
    // also filters resting near-zero contacts (NaN < 1e-8 is false, so the
    // allFinite guard must come first).
    if (!point.allFinite() || !force.allFinite() || mag < 1e-8) {
      visual->setHidden(true);
      continue;
    }

    Eigen::Vector3d disp = kForceLengthScale * force;
    const double dispLen = disp.norm();
    if (dispLen > kMaxArrowLength)
      disp *= kMaxArrowLength / dispLen;

    visual->setHidden(false);
    mArrows[i]->setPositions(point, point + disp);
    visual->setColor(magnitudeColor(maxMag > 1e-8 ? mag / maxMag : 0.0));
  }

  hideFrom(count);
  mLastVisualizedCount = count;
}

//==============================================================================
void ContactVisualizer::renderToggle()
{
  // Hide synchronously on toggle-off: applyPostStep only runs while the sim is
  // stepping, so a toggle-off while paused would otherwise leave the last
  // step's arrows frozen on screen until play resumes.
  if (ImGui::Checkbox("Contact force visualizer", &mEnabled) && !mEnabled) {
    hideFrom(0);
    mLastVisualizedCount = 0;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("(capped at %zu arrows)", kMaxArrows);
}

} // namespace dart_demos
