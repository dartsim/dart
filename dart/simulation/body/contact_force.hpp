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

#include <dart/simulation/body/collision_body.hpp>

#include <Eigen/Core>

namespace dart::simulation {

/// A per-contact reaction force recovered from the most recent world step.
///
/// Unlike `Contact` (pure collision-query output), this is a post-solve
/// diagnostic: `force` is the world-space reaction acting on `bodyB` at `point`
/// (Newtons), reconstructed from the solved contact impulse as
/// `impulse / timeStep`. It is captured for the rigid-body contact paths
/// (sequential-impulse and boxed-LCP); the unified/variational rigid path and
/// multibody/deformable contacts do not yet contribute (see
/// `docs/design/agent_sim_verification.md`). Available through
/// `World::getLastContactForces()` after `step()`; empty after a reset or a
/// step with no rigid contacts.
struct ContactForce
{
  /// World-space contact point.
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  /// World-space reaction force on `bodyB` (Newtons); `bodyA` feels `-force`.
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  CollisionBody bodyA;
  CollisionBody bodyB;
};

} // namespace dart::simulation
