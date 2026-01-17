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

#include <dart/simulation/experimental/frame/frame.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dart::simulation::experimental {

/// FixedFrame - fixed offset from parent frame
///
/// A FixedFrame is rigidly attached to a parent frame with a fixed
/// offset. It has zero velocity and acceleration relative to its parent.
/// It inherits from Frame to provide coordinate frame functionality.
///
/// Use cases:
/// - Rigidly mounted sensors
/// - Attachment points on links
/// - Fixed coordinate frames for collision geometries
/// - End-effector frames
///
/// DART6 equivalent: FixedFrame
class DART_EXPERIMENTAL_API FixedFrame
  : public Frame,
    public EntityObjectWith<
        TagComps<comps::FixedFrameTag>,
        ReadOnlyComps<>,
        WriteOnlyComps<>,
        ReadWriteComps<comps::FixedFrameProperties>>
{
public:
  /// Constructor (package-private, use World::addFixedFrame)
  FixedFrame(entt::entity entity, World* world);

  //--------------------------------------------------------------------------
  // Transform API
  //--------------------------------------------------------------------------

  /// Set the fixed local transform offset
  ///
  /// @param transform New fixed transform offset from parent frame
  void setLocalTransform(const Eigen::Isometry3d& transform);

  /// Get the fixed local transform offset
  ///
  /// @return Transform offset from parent frame (local)
  [[nodiscard]] const Eigen::Isometry3d& getLocalTransform() const;
};

} // namespace dart::simulation::experimental
