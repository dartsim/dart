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

#ifndef DART_EXAMPLES_DEMOS_SCENES_SCENES_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_SCENES_HPP_

#include "../DemoScene.hpp"

namespace dart_demos {

/// Getting Started > empty: minimal world with an InteractiveFrame gizmo
/// target, ported from examples/empty.
[[nodiscard]] DemoScene makeEmptyScene();

/// Visualization > simple_frames: nested SimpleFrames with ellipsoid markers
/// and an arrow shape, ported from examples/simple_frames.
[[nodiscard]] DemoScene makeSimpleFramesScene();

/// Rigid Body > boxes: a 5x5x5 grid of boxes dropped onto a ground plane,
/// ported from examples/boxes.
[[nodiscard]] DemoScene makeBoxesScene();

/// Rigid Body > rigid_cubes: a loaded cube-stack skel with directional
/// impulse forces, playback, and contact-force visualization, ported from
/// examples/rigid_cubes.
[[nodiscard]] DemoScene makeRigidCubesScene();

/// Rigid Body > rigid_chain: a damped articulated chain loaded from a
/// skeleton file, ported from examples/rigid_chain.
[[nodiscard]] DemoScene makeRigidChainScene();

/// Rigid Body > add_delete_skels: spawn and delete dynamic cubes at runtime,
/// ported from examples/add_delete_skels.
[[nodiscard]] DemoScene makeAddDeleteSkelsScene();

/// Rigid Body > simulation_event_handler: sensor markers, force/torque
/// arrows, and body selection, ported from examples/simulation_event_handler.
[[nodiscard]] DemoScene makeSimulationEventHandlerScene();

/// Rigid Body > rigid_shapes: spawn assorted rigid shapes onto a ground
/// plane, ported from examples/rigid_shapes.
[[nodiscard]] DemoScene makeRigidShapesScene();

/// Rigid Body > sleeping: automatic body deactivation across four
/// independent box-stack solver islands, ported from examples/sleeping.
[[nodiscard]] DemoScene makeSleepingScene();

/// Constraints & Joints > hardcoded_design: a hand-built revolute chain
/// driven from the keyboard, ported from examples/hardcoded_design.
[[nodiscard]] DemoScene makeHardcodedDesignScene();

/// Constraints & Joints > rigid_loop: a chain closed into a loop with a
/// ball-joint constraint, ported from examples/rigid_loop.
[[nodiscard]] DemoScene makeRigidLoopScene();

/// Constraints & Joints > box_stacking: stacked boxes comparing Dantzig and
/// PGS LCP solvers, ported from examples/box_stacking.
[[nodiscard]] DemoScene makeBoxStackingScene();

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SCENES_HPP_
