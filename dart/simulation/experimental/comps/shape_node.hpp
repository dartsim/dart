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

#include <dart/simulation/experimental/comps/component_category.hpp>

#include <dart/dynamics/Fwd.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

namespace dart::simulation::experimental::comps {

/// Tag marking entity as a ShapeNode
struct ShapeNodeTag
{
  DART_EXPERIMENTAL_TAG_COMPONENT(ShapeNodeTag);
};

/// ShapeNode component storing collision geometry metadata.
///
/// This component is runtime-only and is not serialized.
struct ShapeNode
{
  DART_EXPERIMENTAL_CACHE_COMPONENT(ShapeNode);

  entt::entity parentEntity = entt::null;
  Eigen::Isometry3d relativeTransform = Eigen::Isometry3d::Identity();
  dart::dynamics::ShapePtr shape;

  bool collidable = true;
  double frictionCoeff = 1.0;
  double restitutionCoeff = 0.0;

  dart::dynamics::ShapeNode* classicShapeNode = nullptr;
};

} // namespace dart::simulation::experimental::comps
