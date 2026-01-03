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

#include <dart/simulation/next/frame/frame.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dart::simulation::next {

/// FreeFrame - freely positionable frame
///
/// A FreeFrame can be positioned and oriented freely in space relative
/// to its parent frame. The transform can be set independently by the user.
///
/// Use cases:
/// - Moving coordinate frames
/// - User-controlled reference frames
/// - Camera frames
/// - Target frames for IK
///
/// DART6 equivalent: SimpleFrame (with modifiable transform)
class DART8_API FreeFrame : public Frame,
                            public EntityObjectWith<
                                TagComps<comps::FreeFrameTag>,
                                ReadOnlyComps<>,
                                WriteOnlyComps<>,
                                ReadWriteComps<comps::FreeFrameProperties>>
{
public:
  /// Constructor (package-private, use World::addFreeFrame)
  FreeFrame(entt::entity entity, World* world);

  //--------------------------------------------------------------------------
  // Transform API
  //--------------------------------------------------------------------------

  /// Set the fixed local transform offset
  ///
  /// @param transform New fixed transform offset from parent frame
  void setLocalTransform(const Eigen::Isometry3d& transform);

  /// Get the local transform (relative to parent frame)
  ///
  /// @return Transform offset from parent frame (local)
  [[nodiscard]] const Eigen::Isometry3d& getLocalTransform() const;
};

} // namespace dart::simulation::next
