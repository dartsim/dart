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

#include "ContactArrowLayout.hpp"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <algorithm>
#include <functional>
#include <limits>
#include <string>

#include <cmath>

namespace dart_demos {

namespace {

/// Below this the contact is treated as carrying no force at all: it is a
/// resting near-zero contact or a filtered sentinel, and drawing it would add
/// a degenerate zero-length arrow.
constexpr double kNegligibleForce = 1e-8;

//==============================================================================
/// Accumulates the world-space extent of one body node into `min` / `max`:
/// its origin, plus the largest half-extent of any collision shape it carries
/// so a single-body scene still has a nonzero size. Only collision geometry
/// counts, since that is what the contacts these arrows annotate come from.
///
/// Returns whether the body contributed, so the caller can skip its mass too.
bool accumulateBodyExtent(
    const dart::dynamics::BodyNode& body,
    Eigen::Vector3d& min,
    Eigen::Vector3d& max,
    double& maxShapeHalfExtent)
{
  // A body that cannot collide cannot produce a contact, so it must not size
  // the arrows that annotate contacts. The `sleeping` scene is the case that
  // matters: it parks its projectile pool at (60, 60, 10) with the bodies made
  // noncollidable, and counting those would stretch the reference length to
  // the clamp and pull the force floor up by their idle mass -- reproducing
  // the very detached-arrow symptom this layout exists to prevent.
  if (!body.isCollidable())
    return false;

  const Eigen::Vector3d origin = body.getWorldTransform().translation();
  if (!origin.allFinite())
    return false;

  min = min.cwiseMin(origin);
  max = max.cwiseMax(origin);

  body.eachShapeNodeWith<dart::dynamics::CollisionAspect>(
      [&maxShapeHalfExtent](const dart::dynamics::ShapeNode* shapeNode) {
        const auto& shape = shapeNode->getShape();
        if (!shape)
          return;

        const Eigen::Vector3d halfExtents
            = shape->getBoundingBox().computeHalfExtents();
        if (halfExtents.allFinite()) {
          maxShapeHalfExtent
              = std::max(maxShapeHalfExtent, halfExtents.maxCoeff());
        }
      });

  return true;
}

} // namespace

//==============================================================================
std::size_t ContactArrowLayout::sceneFingerprint(
    const dart::simulation::World& world)
{
  // Everything the references are derived from goes in, not just topology.
  // Counts alone miss two things the demos actually do: `sleeping` launches and
  // reparks a preallocated projectile pool by toggling setCollidable() without
  // adding or removing anything, and `rigid_shapes` can delete a shape and
  // spawn a differently-sized replacement with identical counts between two
  // updates. Names and shape extents distinguish both.
  const auto mix = [](std::size_t seed, std::size_t value) {
    return seed * 131 + value;
  };
  const auto mixDouble = [&mix](std::size_t seed, double value) {
    // Quantized so ordinary motion, which does not change the derived scale,
    // does not churn the fingerprint.
    return mix(
        seed,
        static_cast<std::size_t>(
            static_cast<long long>(std::round(value * 1000.0))));
  };

  std::size_t fingerprint = world.getNumSkeletons();
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto& skeleton = world.getSkeleton(i);
    if (!skeleton)
      continue;
    fingerprint
        = mix(fingerprint, std::hash<std::string>{}(skeleton->getName()));
    fingerprint = mix(fingerprint, skeleton->getNumBodyNodes());
    fingerprint = mix(fingerprint, skeleton->getNumDofs());

    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      const auto* body = skeleton->getBodyNode(j);
      if (!body || !body->isCollidable())
        continue;
      fingerprint = mix(fingerprint, j + 1);
      body->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
          [&](const dart::dynamics::ShapeNode* shapeNode) {
            const auto& shape = shapeNode->getShape();
            if (!shape)
              return;
            const Eigen::Vector3d extents
                = shape->getBoundingBox().computeFullExtents();
            if (!extents.allFinite())
              return;
            for (int axis = 0; axis < 3; ++axis)
              fingerprint = mixDouble(fingerprint, extents[axis]);
          });
    }
  }
  return fingerprint;
}

//==============================================================================
void ContactArrowLayout::refreshForWorld(const dart::simulation::World& world)
{
  // The floor is re-derived every call rather than compared, because gravity is
  // a live control -- the host has a Gravity checkbox and some scenes have a
  // gravity slider -- and it is only a multiply against the mass already
  // measured. Leaving it cached would keep low-force contacts compressed by the
  // old weight after gravity is switched off, and leave the floor at its
  // noise-suppression minimum after gravity is switched on in a zero-gravity
  // scene.
  const std::size_t fingerprint = sceneFingerprint(world);
  if (fingerprint != mSceneFingerprint) {
    // Only the observed peak survives a re-derivation. Carrying the old floor
    // across would let a body that has just left the scale keep suppressing
    // everything else, as though its weight had been measured at a contact:
    // reparking the `sleeping` scene's projectile would go on crushing the box
    // arrows until the decay caught up with a floor that no longer applies.
    // The decay baseline survives too. resetForWorld() re-anchors it to now,
    // which is right for a scene switch but wrong here: if the fingerprint
    // change happened while updates were skipped (a projectile launched with
    // the visualizer off), anchoring to now would erase the skipped interval
    // and resurrect the stale peak undecayed.
    const double observedPeak = mPeakForce;
    const double decayBaseline = mLastWorldTime;
    resetForWorld(world);
    mPeakForce = observedPeak;
    mLastWorldTime = decayBaseline;
  }

  // The floor is re-derived every call rather than compared, because gravity is
  // a live control -- the host has a Gravity checkbox and some scenes have a
  // gravity slider -- and it is only a multiply against the mass already
  // measured. Leaving it cached would keep low-force contacts compressed by the
  // old weight after gravity is switched off, and leave the floor at its
  // noise-suppression minimum after gravity is switched on in a zero-gravity
  // scene.
  mFloorForce = std::max(
      kNegligibleForce,
      kFloorForceWeightFraction * mMobileMass * world.getGravity().norm());
}

//==============================================================================
void ContactArrowLayout::resetForWorld(const dart::simulation::World& world)
{
  mArrows.clear();
  mSceneFingerprint = sceneFingerprint(world);

  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d max
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
  double maxShapeHalfExtent = 0.0;
  double mobileMass = 0.0;
  bool sawMobileBody = false;

  // Static geometry is deliberately excluded. A ground plane is routinely
  // modeled as a very large box, and letting it into the extent would size
  // every arrow to the floor rather than to the objects resting on it.
  //
  // A welded ground still reports isMobile(), which is a user-set flag rather
  // than a property of the joints, so the degree-of-freedom count is what
  // actually distinguishes something that can move.
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto& skeleton = world.getSkeleton(i);
    if (!skeleton || !skeleton->isMobile() || skeleton->getNumDofs() == 0)
      continue;

    // Mass is accumulated per body rather than per skeleton for the same
    // reason: only what can actually reach a contact should set the scale.
    for (const auto* body : skeleton->getBodyNodes()) {
      if (!body)
        continue;
      if (!accumulateBodyExtent(*body, min, max, maxShapeHalfExtent))
        continue;
      mobileMass += body->getMass();
      sawMobileBody = true;
    }
  }

  if (sawMobileBody && (max.array() >= min.array()).all()) {
    const double diagonal = (max - min).norm() + 2.0 * maxShapeHalfExtent;
    mReferenceLength = std::clamp(
        kSceneLengthFraction * diagonal,
        kMinReferenceLength,
        kMaxReferenceLength);
  } else {
    mReferenceLength = kFallbackReferenceLength;
  }

  // The weight the contacts have to carry is the natural floor for the force
  // reference: it is what a resting scene's contact forces sum to. The mass is
  // kept so refreshForWorld() can re-derive the floor when gravity changes.
  mMobileMass = mobileMass;
  const double weight = mobileMass * world.getGravity().norm();
  mFloorForce = std::max(kNegligibleForce, kFloorForceWeightFraction * weight);
  mPeakForce = 0.0;
  mLastWorldTime = world.getTime();
}

//==============================================================================
const std::vector<ContactArrow>& ContactArrowLayout::update(
    const dart::simulation::World& world,
    const std::vector<dart::collision::Contact>& contacts,
    std::size_t maxArrows)
{
  refreshForWorld(world);

  mArrows.clear();

  const std::size_t count = std::min(contacts.size(), maxArrows);

  // A non-finite force or point (a diverging LCP solve is the common case for
  // an interactive debugging tool) would poison both the peak below and the
  // arrow mesh vertices, so those contacts are rejected up front. Note that
  // `mag < kNegligibleForce` is false for NaN, hence the allFinite() first.
  //
  // The magnitude is checked rather than the components, because a force whose
  // components are each finite can still overflow to infinity under norm() --
  // and an infinite reference would normalize that same contact as inf/inf,
  // putting a NaN in the arrow head and pinning the reference at infinity for
  // every later step.
  double peakForce = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    if (!contacts[i].point.allFinite() || !contacts[i].force.allFinite())
      continue;
    const double magnitude = contacts[i].force.norm();
    if (std::isfinite(magnitude))
      peakForce = std::max(peakForce, magnitude);
  }

  // Track the peak instantly and release it slowly. Rising at once keeps the
  // longest arrow pinned to the largest force; decaying over kForceDecayTime
  // lets resting contacts become readable again shortly after an impact,
  // instead of staying crushed to invisibility by a spike seconds in the past.
  //
  // The decay follows the world clock, not the number of update() calls. The
  // two are the same while the visualizer runs every step, but they diverge
  // exactly when it matters: with the visualizer toggled off the simulation
  // keeps advancing without updates, and a per-call decay would freeze the
  // peak and restore a stale spike on re-enable; while paused, no simulated
  // time passes and the peak correctly holds. A backwards clock (the scene's
  // world was reset) counts as no elapsed time.
  const double now = world.getTime();
  const double elapsed = now > mLastWorldTime ? now - mLastWorldTime : 0.0;
  mLastWorldTime = now;
  const double decay = std::exp(-elapsed / kForceDecayTime);
  mPeakForce = std::max(peakForce, decay * mPeakForce);

  mArrows.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const Eigen::Vector3d& point = contacts[i].point;
    const Eigen::Vector3d& force = contacts[i].force;
    if (!point.allFinite() || !force.allFinite())
      continue;

    const double magnitude = force.norm();
    if (!std::isfinite(magnitude) || magnitude < kNegligibleForce)
      continue;

    const double normalized
        = std::clamp(magnitude / getReferenceForce(), 0.0, 1.0);

    ContactArrow arrow;
    arrow.tail = point;
    // Scale the *direction* by the normalized magnitude rather than scaling
    // the force itself, so the arrow can never be longer than the reference
    // length no matter how large the force is.
    arrow.head = point + force * (normalized * mReferenceLength / magnitude);
    arrow.normalizedMagnitude = normalized;
    mArrows.push_back(arrow);
  }

  return mArrows;
}

} // namespace dart_demos
