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

  /// Time constant for the force reference's decay back toward the floor.
  static constexpr double kForceDecayTime = 0.5; // seconds

  /// Rebinds to a newly installed scene and re-derives both references from
  /// it. Safe to call on a world with no skeletons; the fallback length is
  /// used then.
  void resetForWorld(const dart::simulation::World& world);

  /// Re-derives the scene-dependent references if the world's set of bodies has
  /// changed since the last call, and does nothing otherwise. Scenes such as
  /// `add_delete_skels` and `rigid_shapes` spawn and remove skeletons while
  /// running, so a scale derived only at install time would be stuck on
  /// whatever the world held at startup.
  ///
  /// Unlike resetForWorld() this keeps the force reference it has been
  /// tracking, so adding a body does not discard the current peak; it is only
  /// raised if the new floor demands it. Cheap enough to call every step.
  void refreshForWorld(const dart::simulation::World& world);

  /// Lays out at most `maxArrows` arrows for `contacts` and returns them.
  ///
  /// `timeStep` is the world's current timestep, passed in per call rather than
  /// captured in resetForWorld() because the demo host lets it change while a
  /// scene runs; it sets how fast the force reference decays.
  ///
  /// Contacts with a non-finite point or force, and contacts carrying
  /// negligible force, are dropped rather than laid out, so the result is
  /// often shorter than `contacts`.
  const std::vector<ContactArrow>& update(
      const std::vector<dart::collision::Contact>& contacts,
      std::size_t maxArrows,
      double timeStep);

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

  /// Force that currently maps to a full-length arrow.
  double getReferenceForce() const
  {
    return mReferenceForce;
  }

private:
  std::vector<ContactArrow> mArrows;

  /// Cheap stand-in for "the set of bodies changed": skeleton count, body
  /// count and degree-of-freedom count together. World exposes no signal for
  /// this, and a full re-derivation every step would let the arrow scale drift
  /// with ordinary motion.
  static std::size_t sceneFingerprint(const dart::simulation::World& world);

  double mReferenceLength = kFallbackReferenceLength;
  double mReferenceForce = 1.0;
  double mFloorForce = 1.0;
  double mMobileMass = 0.0;
  std::size_t mSceneFingerprint = 0;
};

} // namespace dart_demos

#endif // DART_DEMOS_CONTACTARROWLAYOUT_HPP_
