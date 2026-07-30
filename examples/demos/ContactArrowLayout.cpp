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
#include <limits>

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
void accumulateBodyExtent(
    const dart::dynamics::BodyNode& body,
    Eigen::Vector3d& min,
    Eigen::Vector3d& max,
    double& maxShapeHalfExtent)
{
  const Eigen::Vector3d origin = body.getWorldTransform().translation();
  if (!origin.allFinite())
    return;

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
}

} // namespace

//==============================================================================
void ContactArrowLayout::resetForWorld(const dart::simulation::World& world)
{
  mArrows.clear();

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

    mobileMass += skeleton->getMass();
    for (const auto* body : skeleton->getBodyNodes()) {
      if (!body)
        continue;
      accumulateBodyExtent(*body, min, max, maxShapeHalfExtent);
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
  // reference: it is what a resting scene's contact forces sum to.
  const double weight = mobileMass * world.getGravity().norm();
  mFloorForce = std::max(kNegligibleForce, kFloorForceWeightFraction * weight);
  mReferenceForce = mFloorForce;

  const double timeStep = world.getTimeStep();
  mDecayPerStep = (timeStep > 0.0 && std::isfinite(timeStep))
                      ? std::exp(-timeStep / kForceDecayTime)
                      : 0.0;
}

//==============================================================================
const std::vector<ContactArrow>& ContactArrowLayout::update(
    const std::vector<dart::collision::Contact>& contacts,
    std::size_t maxArrows)
{
  mArrows.clear();

  const std::size_t count = std::min(contacts.size(), maxArrows);

  // A non-finite force or point (a diverging LCP solve is the common case for
  // an interactive debugging tool) would poison both the peak below and the
  // arrow mesh vertices, so those contacts are rejected up front. Note that
  // `mag < kNegligibleForce` is false for NaN, hence the allFinite() first.
  double peakForce = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    if (contacts[i].point.allFinite() && contacts[i].force.allFinite())
      peakForce = std::max(peakForce, contacts[i].force.norm());
  }

  // Track the peak instantly and release it slowly. Rising at once keeps the
  // longest arrow pinned to the largest force; decaying over kForceDecayTime
  // lets resting contacts become readable again shortly after an impact,
  // instead of staying crushed to invisibility by a spike seconds in the past.
  mReferenceForce
      = std::max({peakForce, mDecayPerStep * mReferenceForce, mFloorForce});

  mArrows.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const Eigen::Vector3d& point = contacts[i].point;
    const Eigen::Vector3d& force = contacts[i].force;
    if (!point.allFinite() || !force.allFinite())
      continue;

    const double magnitude = force.norm();
    if (magnitude < kNegligibleForce)
      continue;

    const double normalized = std::clamp(magnitude / mReferenceForce, 0.0, 1.0);

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
