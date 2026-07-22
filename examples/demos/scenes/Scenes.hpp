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

#include <dart/config.hpp>

#include <string>

namespace dart_demos {

/// Getting Started > empty: minimal world with an InteractiveFrame gizmo
/// target, ported from examples/empty.
[[nodiscard]] DemoScene makeEmptyScene();

/// Getting Started > hello_world: the canonical free-falling box first program,
/// mirrored from examples/hello_world while keeping that standalone example.
[[nodiscard]] DemoScene makeHelloWorldScene();

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

/// Research > FBF incline: exact-Coulomb paper incline threshold fixture.
[[nodiscard]] DemoScene makeFbfPaperInclineScene();

/// Research > FBF backspin: exact-Coulomb paper backspin sphere fixture.
[[nodiscard]] DemoScene makeFbfPaperBackspinScene();

/// Research > FBF turntable: exact-Coulomb paper turntable capture/ejection.
[[nodiscard]] DemoScene makeFbfPaperTurntableScene();

/// Research > FBF turntable: fixed mu=.2, omega=2 capture cell.
[[nodiscard]] DemoScene makeFbfPaperTurntableMu02Omega2Scene();

/// Research > FBF turntable: fixed mu=.2, omega=5 capture cell.
[[nodiscard]] DemoScene makeFbfPaperTurntableMu02Omega5Scene();

/// Research > FBF turntable: fixed mu=.5, omega=5 capture cell.
[[nodiscard]] DemoScene makeFbfPaperTurntableMu05Omega5Scene();

/// Research > FBF author turntable: source-pinned mu=.2, omega=2 cell.
[[nodiscard]] DemoScene makeFbfAuthorTurntableMu02Omega2Scene();

/// Research > FBF author turntable: source-pinned mu=.2, omega=5 cell.
[[nodiscard]] DemoScene makeFbfAuthorTurntableMu02Omega5Scene();

/// Research > FBF author turntable: source-pinned mu=.5, omega=2 cell.
[[nodiscard]] DemoScene makeFbfAuthorTurntableMu05Omega2Scene();

/// Research > FBF author turntable: source-pinned mu=.5, omega=5 cell.
[[nodiscard]] DemoScene makeFbfAuthorTurntableMu05Omega5Scene();

/// Returns the canonical runtime-inspected physics/control contract for one
/// fbf_author_turntable_* scene. Renderer assets are intentionally excluded.
/// Throws std::invalid_argument for any other scene id.
[[nodiscard]] std::string fbfAuthorTurntablePhysicsContractJson(
    const std::string& sceneId);

/// Research > FBF author card house: source-pinned five-level construction.
[[nodiscard]] DemoScene makeFbfAuthorCardHouseScene();

/// Research > FBF author card house: source-parameterized four-level impact.
[[nodiscard]] DemoScene makeFbfAuthorCardHouse4ImpactCurrentSourceScene();

/// Research > FBF author card house: source-parameterized ten-level impact.
[[nodiscard]] DemoScene makeFbfAuthorCardHouse10ImpactCurrentSourceScene();

/// Research > FBF author card house: exact source-continuation impact lane.
[[nodiscard]] DemoScene
makeFbfAuthorCardHouse4ImpactSourceContinuationCurrentSourceScene();

/// Research > FBF author masonry arch: source-pinned 25-wedge crown impact.
[[nodiscard]] DemoScene
makeFbfAuthorMasonryArch25CrownImpactCurrentSourceScene();

/// Research > FBF author masonry arch: source-supported 101-wedge standing
/// lane over the source-default no-release horizon.
[[nodiscard]] DemoScene makeFbfAuthorMasonryArch101StandingCurrentSourceScene();

/// Returns the canonical runtime-inspected configuration-only contract for
/// fbf_author_card_house_5_construction. Renderer styling is disclosed but
/// intentionally excluded from source parity. Throws std::invalid_argument
/// for any other scene id.
[[nodiscard]] std::string fbfAuthorCardHouseConfigurationContractJson(
    const std::string& sceneId);

/// Research > FBF Painleve proxy: headless paper-proxy fixture as a GUI scene.
[[nodiscard]] DemoScene makeFbfPaperPainleveScene();

/// Research > FBF Painleve proxy: fixed mu=.55 outcome cell.
[[nodiscard]] DemoScene makeFbfPaperPainleveMu055Scene();

/// Research > FBF author Painleve: source-pinned mu=.5 adapter cell.
[[nodiscard]] DemoScene makeFbfAuthorPainleveMu05Scene();

/// Research > FBF author Painleve: source-pinned mu=.55 adapter cell.
[[nodiscard]] DemoScene makeFbfAuthorPainleveMu055Scene();

/// Research > FBF card A-frame: two-card Fig. 6 precursor.
[[nodiscard]] DemoScene makeFbfPaperCardAFrameScene();

/// Research > FBF card house: reduced-contact dynamic 26-card Fig. 6 scaffold.
[[nodiscard]] DemoScene makeFbfPaperCardHouse26Scene();

/// Research > FBF card house: construction-only 10-level GPU-table scaffold.
[[nodiscard]] DemoScene makeFbfPaperCardHouse10Scene();

/// Research > FBF card house: dynamic exact-FBF 10-level capped adapter.
[[nodiscard]] DemoScene makeFbfPaperCardHouse10DynamicScene();

/// Research > FBF masonry arch: reduced-contact 25-stone Fig. 7 scaffold.
[[nodiscard]] DemoScene makeFbfPaperMasonryArch25Scene();

/// Research > FBF masonry arch: literal-wedge 25-stone standing contract.
[[nodiscard]] DemoScene makeFbfPaperMasonryArch25LiteralStandingScene();

/// Research > FBF masonry arch: reduced-contact 101-stone Fig. 8 scaffold.
[[nodiscard]] DemoScene makeFbfPaperMasonryArch101Scene();

/// Constraints & Joints > hardcoded_design: a hand-built revolute chain
/// driven from the keyboard, ported from examples/hardcoded_design.
[[nodiscard]] DemoScene makeHardcodedDesignScene();

/// Constraints & Joints > rigid_loop: a chain closed into a loop with a
/// ball-joint constraint, ported from examples/rigid_loop.
[[nodiscard]] DemoScene makeRigidLoopScene();

/// Constraints & Joints > box_stacking: stacked boxes comparing Dantzig and
/// PGS LCP solvers, ported from examples/box_stacking.
[[nodiscard]] DemoScene makeBoxStackingScene();

/// Constraints & Joints > dynamic_joint_constraints: ball, cylindrical, and
/// weld dynamic joint constraints on three free-flying bodies, ported from
/// examples/dynamic_joint_constraints.
[[nodiscard]] DemoScene makeDynamicJointConstraintsScene();

/// Control & IK > joint_constraints: balanced full-body PD control with a
/// harness constraint, ported from examples/joint_constraints.
[[nodiscard]] DemoScene makeJointConstraintsScene();

/// Control & IK > hybrid_dynamics: velocity-driven sinusoidal gait with a
/// lockable pelvis harness, ported from examples/hybrid_dynamics.
[[nodiscard]] DemoScene makeHybridDynamicsScene();

/// Control & IK > biped_stand: SPD-balanced standing biped with an ankle
/// strategy and push tests, ported from examples/biped_stand.
[[nodiscard]] DemoScene makeBipedStandScene();

/// Control & IK > operational_space_control: KR5 arm chasing a draggable
/// target with task-space control, ported from
/// examples/operational_space_control.
[[nodiscard]] DemoScene makeOperationalSpaceControlScene();

/// Control & IK > contact_inverse_dynamics: kinematic squat replay solved for
/// contact-consistent joint torques, ported from
/// examples/contact_inverse_dynamics.
[[nodiscard]] DemoScene makeContactInverseDynamicsScene();

#ifdef DART_DEMOS_HAVE_PYTHON
/// Control & IK > ssik_ik_gui: analytical IK against 19 prebuilt robot arms
/// via the optional ssik Python package, ported from examples/ssik_ik_gui.
/// Only declared/registered when Python3 development files were found at
/// configure time (see examples/demos/CMakeLists.txt).
[[nodiscard]] DemoScene makeSsikIkGuiScene();
#endif

/// Control & IK > wam_ikfast: WAM arm posed by IkFast analytical whole-body
/// IK with draggable body nodes and an IK target, ported from
/// examples/wam_ikfast.
[[nodiscard]] DemoScene makeWamIkFastScene();

#ifdef DART_DEMOS_HAVE_TINY_DNN
/// Constraints & Joints > human_joint_limits: a human skeleton with
/// neural-network joint-limit constraints, ported from
/// examples/human_joint_limits. Only declared/registered when TinyDNN +
/// Threads were found at configure time (see examples/demos/CMakeLists.txt).
[[nodiscard]] DemoScene makeHumanJointLimitsScene();
#endif

/// Soft Bodies > mixed_chain: apply impulses to a soft link in a mixed
/// rigid/soft chain, ported from examples/mixed_chain.
[[nodiscard]] DemoScene makeMixedChainScene();

/// Soft Bodies > soft_bodies: soft-body simulation with recorded-state
/// playback, ported from examples/soft_bodies.
[[nodiscard]] DemoScene makeSoftBodiesScene();

/// Soft Bodies > soft_cubes: soft cube stack with recorded-state playback,
/// ported from examples/soft_bodies.
[[nodiscard]] DemoScene makeSoftCubesScene();

/// Soft Bodies > soft_open_chain: open chain of soft links with recorded-state
/// playback, ported from examples/soft_bodies.
[[nodiscard]] DemoScene makeSoftOpenChainScene();

/// Constraints & Joints > tinkertoy: interactively assemble jointed
/// structures from blocks, ported from examples/tinkertoy.
[[nodiscard]] DemoScene makeTinkertoyScene();

/// Visualization > heightmap: an interactive procedural heightmap with a
/// configurable grid, ported from examples/heightmap.
[[nodiscard]] DemoScene makeHeightmapScene();

/// Control & IK > atlas_puppet: a purely kinematic Atlas whole-body IK
/// puppet with balance and support-polygon overlays, ported from
/// examples/atlas_puppet.
[[nodiscard]] DemoScene makeAtlasPuppetScene();

/// Control & IK > atlas_simbicon: an Atlas humanoid balanced by a SIMBICON
/// state-machine controller, ported from examples/atlas_simbicon
/// (multi-file subsystem in scenes/atlas_simbicon/).
[[nodiscard]] DemoScene makeAtlasSimbiconScene();

/// Control & IK > hubo_puppet: a purely kinematic DRC-Hubo whole-body IK
/// puppet driven by analytical arm/leg IK, ported from examples/hubo_puppet.
[[nodiscard]] DemoScene makeHuboPuppetScene();

/// Robots > fetch: a Fetch robot's end effector follows a draggable
/// interactive target via a welded mocap body, ported from examples/fetch
/// (MJCF asset).
[[nodiscard]] DemoScene makeFetchScene();

/// Robots > vehicle: a steerable car driven with servo-controlled steering
/// and wheel-spin dofs, ported from examples/vehicle.
[[nodiscard]] DemoScene makeVehicleScene();

#if HAVE_OCTOMAP
/// Visualization > point_cloud: animated point-cloud and voxel-grid sensor
/// rendering on a robot, ported from examples/point_cloud. Only
/// declared/registered when DART was built with octomap support (the same
/// dart/config.hpp macro dart/dynamics/{PointCloud,VoxelGrid}Shape.hpp are
/// themselves guarded by).
[[nodiscard]] DemoScene makePointCloudScene();
#endif

/// Visualization > drag_and_drop: a gizmo-draggable frame carrying a child
/// box, plus axis markers, ported from examples/drag_and_drop.
[[nodiscard]] DemoScene makeDragAndDropScene();

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_SCENES_HPP_
