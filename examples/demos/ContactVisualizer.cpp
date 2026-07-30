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

namespace dart_demos {

namespace {

// The cool -> hot color ramp used to encode magnitude, ported as-is from
// RigidCubesScene's original per-scene contact-force visualization. The arrow
// *length* mapping that came with it was a fixed 0.1 m/N, which is only right
// for scenes the size of that one's blocks; ContactArrowLayout derives it from
// the scene instead.
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
  if (world)
    mLayout.resetForWorld(*world);
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

  // The shaft is sized from the same scene-derived length as the arrows, so a
  // tabletop scene does not get a shaft built for a humanoid and vice versa.
  const double shaftRadius
      = std::clamp(0.02 * mLayout.getReferenceLength(), 0.0015, 0.05);

  while (mFrames.size() < count) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World());
    auto arrow = std::make_shared<dart::dynamics::ArrowShape>(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::UnitZ() * 0.01,
        dart::dynamics::ArrowShape::Properties(shaftRadius, 2.0, 0.15),
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
  const auto& arrows = mLayout.update(result.getContacts(), kMaxArrows);
  const std::size_t count = arrows.size();
  ensurePool(count);

  for (std::size_t i = 0; i < count; ++i) {
    mFrames[i]->getVisualAspect(true)->setHidden(false);
    mArrows[i]->setPositions(arrows[i].tail, arrows[i].head);
    mFrames[i]->getVisualAspect(true)->setColor(
        magnitudeColor(arrows[i].normalizedMagnitude));
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
  ImGui::TextDisabled("Capped at %zu arrows", kMaxArrows);
  ImGui::TextDisabled(
      "Full-length arrow = %.0f N over %.2f m",
      mLayout.getReferenceForce(),
      mLayout.getReferenceLength());
}

} // namespace dart_demos
