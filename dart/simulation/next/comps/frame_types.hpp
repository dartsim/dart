/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/simulation/next/comps/component_category.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <tuple>

namespace dart::simulation::next::comps {

//==============================================================================
// Frame Tag Components
//==============================================================================

/// Tag component marking an entity as a Frame
struct FrameTag
{
  DART8_TAG_COMPONENT(FrameTag);
};

/// Tag component marking an entity as a FixedFrame
struct FixedFrameTag
{
  DART8_TAG_COMPONENT(FixedFrameTag);
};

/// Tag component marking an entity as a FreeFrame
struct FreeFrameTag
{
  DART8_TAG_COMPONENT(FreeFrameTag);
};

//==============================================================================
// Frame State and Cache Components
//==============================================================================

/// Frame state - Logical parent-child relationship
///
/// This component stores the frame hierarchy structure with automatic entity
/// remapping during serialization.
struct FrameState
{
  DART8_STATE_COMPONENT(FrameState);

  /// Parent frame entity (entt::null = world frame)
  entt::entity parentFrame = entt::null;

  /// Declare which fields need entity remapping during serialization
  static constexpr auto entityFields()
  {
    return std::tuple{&FrameState::parentFrame};
  }
};

/// Frame cache - Lazy-evaluated world transforms
///
/// This component stores cached computation results that are NOT serialized.
/// Reconstructed automatically after deserialization.
struct FrameCache
{
  DART8_CACHE_COMPONENT(FrameCache);

  /// Cached world-frame transform
  Eigen::Isometry3d worldTransform = Eigen::Isometry3d::Identity();

  /// Dirty flag for lazy evaluation (true = needs recompute)
  bool needTransformUpdate = true;
};

/// FixedFrameProperties component
///
/// Stores properties for a frame that is rigidly attached to its parent.
/// Automatically serialized via DART8_PROPERTY_COMPONENT macro.
struct FixedFrameProperties
{
  DART8_PROPERTY_COMPONENT(FixedFrameProperties);

  /// Fixed transform offset relative to parent frame (local transform)
  Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
};

/// FreeFrameProperties component
///
/// Stores properties for a frame that can be freely positioned.
/// Automatically serialized via DART8_PROPERTY_COMPONENT macro.
struct FreeFrameProperties
{
  DART8_PROPERTY_COMPONENT(FreeFrameProperties);

  /// Transform relative to parent frame (local transform)
  Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
};

} // namespace dart::simulation::next::comps
