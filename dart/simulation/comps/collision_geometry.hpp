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

#pragma once

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/comps/component_category.hpp>

#include <vector>

#include <cstdint>

namespace dart::simulation::comps {

/// Collision geometry attached to a body or link.
///
/// **Internal Implementation Detail** - Not exposed in public API. The public
/// surface is the `CollisionShape` value object plus body/link accessors.
struct CollisionGeometry
{
  DART_SIMULATION_PROPERTY_COMPONENT(
      CollisionGeometry, "comps.CollisionGeometry");

  std::vector<CollisionShape> shapes;
  std::uint64_t revision = 0;

  [[nodiscard]] bool hasShapes() const
  {
    return !shapes.empty();
  }

  [[nodiscard]] const CollisionShape* getPrimaryShape() const
  {
    if (shapes.empty()) {
      return nullptr;
    }
    return &shapes.front();
  }
};

/// Tag marking static collision geometry as a one-sided ground barrier for
/// deformable bodies.
struct DeformableGroundBarrierTag
{
  DART_SIMULATION_TAG_COMPONENT(
      DeformableGroundBarrierTag, "comps.DeformableGroundBarrierTag");
};

/// Tag marking static box collision geometry as a stationary surface obstacle
/// for the experimental deformable CCD line-search limiter.
struct DeformableSurfaceCcdObstacleTag
{
  DART_SIMULATION_TAG_COMPONENT(
      DeformableSurfaceCcdObstacleTag, "comps.DeformableSurfaceCcdObstacleTag");
};

/// Tag marking a deformable obstacle as barrier-only: it still exerts its
/// clamped-log contact barrier (and so participates in friction), but is
/// excluded from the surface-CCD line-search limiter. The CCD scales the whole
/// step (normal + tangential) to prevent penetration, which masks tangential
/// sliding and hence obstacle friction; opting an obstacle out lets a
/// deformable slide and be decelerated by friction (the barrier still prevents
/// penetration for the quasi-static contact this is intended for). Only
/// meaningful on a body that also carries DeformableSurfaceCcdObstacleTag.
struct DeformableObstacleNoCcdTag
{
  DART_SIMULATION_TAG_COMPONENT(
      DeformableObstacleNoCcdTag, "comps.DeformableObstacleNoCcdTag");
};

} // namespace dart::simulation::comps
