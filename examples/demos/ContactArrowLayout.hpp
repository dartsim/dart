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

#ifndef DART_DEMOS_CONTACTARROWLAYOUT_HPP_
#define DART_DEMOS_CONTACTARROWLAYOUT_HPP_

#include <dart/simulation/World.hpp>

#include <dart/collision/CollisionResult.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <vector>

#include <cstddef>

namespace dart_demos {

//==============================================================================
/// World-space endpoints for one contact-force arrow.
struct ContactArrow
{
  /// The contact point. The arrow is drawn from here along the contact force.
  Eigen::Vector3d tail = Eigen::Vector3d::Zero();

  /// The arrow head, always within ContactArrowLayout::getReferenceLength()
  /// of the tail.
  Eigen::Vector3d head = Eigen::Vector3d::Zero();

  /// Force magnitude relative to the current reference force, in [0, 1].
  /// Drives both the arrow length and the color ramp, so the two agree.
  double normalizedMagnitude = 0.0;
};

//==============================================================================
/// Turns contact forces into arrow geometry for the demo contact visualizer.
///
/// Contact force has no natural length unit, so any fixed newtons-to-meters
/// constant is only right for the scene it was tuned on. The layout derives
/// both of its references from the scene instead:
///
/// - the *length* reference from the size of the mobile skeletons, so arrows
///   stay proportional to whatever is being simulated; and
/// - the *force* reference from the peak contact force actually observed, so
///   the longest arrow on screen is the largest force on screen.
///
/// The class is deliberately free of OSG and ImGui so it can be exercised
/// headlessly.
class ContactArrowLayout
{
public:
  /// Longest arrow drawn when the scene scale cannot be measured, and the
  /// bounds the measured reference is clamped into. The upper bound also
  /// keeps a diverging solve from flinging an arrow head far enough to blow
  /// out OSG's automatic near/far and render the scene undrawable.
  static constexpr double kFallbackReferenceLength = 0.25; // meters
  static constexpr double kMinReferenceLength = 0.02;      // meters
  static constexpr double kMaxReferenceLength = 5.0;       // meters

  /// Fraction of the scene's diagonal drawn for a full-scale contact force.
  static constexpr double kSceneLengthFraction = 0.25;

  /// Floor on the force reference, as a fraction of the weight the contacts
  /// support. Without it, a scene whose contacts all carry near-zero force
  /// would normalize numerical noise up to full-length arrows.
  static constexpr double kFloorForceWeightFraction = 0.05;

  /// Time constant for the force reference's decay back toward the floor, in
  /// simulated seconds. The decay follows the world clock, so it advances
  /// while the visualizer is disabled but the simulation runs, and holds while
  /// the simulation is paused.
  static constexpr double kForceDecayTime = 0.5; // simulated seconds

  /// Rebinds to a newly installed scene and re-derives both references from
  /// it. Safe to call on a world with no skeletons; the fallback length is
  /// used then.
  void resetForWorld(const dart::simulation::World& world);

  /// Lays out at most `maxArrows` arrows for `contacts` and returns them.
  ///
  /// Takes the world rather than the pieces it needs from it, because every one
  /// of those pieces is live: scenes spawn and remove skeletons, toggle
  /// collidability, and change gravity while running, and the world clock
  /// drives the peak's decay. A caller that passed them separately would have
  /// to remember to re-read each one every step, and forgetting silently
  /// freezes the scale at whatever it was when the scene was installed.
  ///
  /// Contacts with a non-finite point or force, and contacts carrying
  /// negligible force, are dropped rather than laid out, so the result is
  /// often shorter than `contacts`.
  const std::vector<ContactArrow>& update(
      const dart::simulation::World& world,
      const std::vector<dart::collision::Contact>& contacts,
      std::size_t maxArrows);

  /// The arrows produced by the most recent update().
  const std::vector<ContactArrow>& getArrows() const
  {
    return mArrows;
  }

  /// Length of an arrow whose contact carries the reference force.
  double getReferenceLength() const
  {
    return mReferenceLength;
  }

  /// Force that currently maps to a full-length arrow: the largest contact
  /// force recently observed, floored so a contact-free scene does not
  /// normalize numerical noise up to full length.
  ///
  /// The two are combined here rather than folded together as they are
  /// computed, so the floor is never mistaken for something that was measured
  /// and cannot outlive the body whose weight set it.
  double getReferenceForce() const
  {
    return std::max(mPeakForce, mFloorForce);
  }

private:
  /// Re-derives the scene-dependent references when the world has changed in a
  /// way that affects them, and does nothing otherwise. Called by update() so
  /// it cannot be skipped.
  void refreshForWorld(const dart::simulation::World& world);

  std::vector<ContactArrow> mArrows;

  /// Cheap stand-in for "the scale-relevant scene changed": skeleton names and
  /// counts, degree-of-freedom counts, per-body collidability, and quantized
  /// collision-shape extents together. World exposes no signal for this, and a
  /// full re-derivation every step would let the arrow scale drift with
  /// ordinary motion.
  static std::size_t sceneFingerprint(const dart::simulation::World& world);

  double mReferenceLength = kFallbackReferenceLength;
  double mPeakForce = 0.0;
  double mFloorForce = 1.0;
  double mMobileMass = 0.0;
  double mLastWorldTime = 0.0;
  std::size_t mSceneFingerprint = 0;
};

} // namespace dart_demos

#endif // DART_DEMOS_CONTACTARROWLAYOUT_HPP_
