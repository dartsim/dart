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

#include <dart/simulation/constraint/loop_closure_family.hpp>
#include <dart/simulation/frame/frame.hpp>

#include <Eigen/Geometry>

namespace dart::simulation {

/// Value object describing a closed-chain relation between two public frames.
///
/// `frameA` and `frameB` are symmetric endpoints. `offsetA` and `offsetB`
/// define the closure endpoint frames relative to those endpoint frames.
/// Kinematic projection, residual reporting, and dynamic solving policies are
/// intentionally not part of this topology object.
struct LoopClosureSpec
{
  Frame frameA = Frame::world();
  Frame frameB = Frame::world();
  LoopClosureFamily family = LoopClosureFamily::Rigid;
  Eigen::Isometry3d offsetA = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d offsetB = Eigen::Isometry3d::Identity();

  /// Target separation for the `Distance` family (the fixed distance to
  /// maintain between the endpoint positions). Ignored by the `Point` and
  /// `Rigid` families. Defaults to 0 (coincident endpoints).
  double distance = 0.0;
};

} // namespace dart::simulation
