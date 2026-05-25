/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_SCENES_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_HPP_

#include <dart/gui/application.hpp>

#include <vector>

namespace dart::examples::demos {

// Each scene is a focused factory returning the per-scene ApplicationOptions
// (world, panels, gizmos, handlers, camera). The demos host owns the shared
// chrome (window, sidebar, camera, simulation controls) and swaps the active
// scene at runtime.

dart::gui::ApplicationOptions makeHelloWorldScene();
dart::gui::ApplicationOptions makeEmptyScene();
dart::gui::ApplicationOptions makeShapesScene();
dart::gui::ApplicationOptions makeSimpleFramesScene();
dart::gui::ApplicationOptions makeDragAndDropScene();
dart::gui::ApplicationOptions makePolyhedronVisualScene();
dart::gui::ApplicationOptions makeBoxesScene();
dart::gui::ApplicationOptions makeRigidCubesScene();
dart::gui::ApplicationOptions makeAddDeleteSkelsScene();
dart::gui::ApplicationOptions makeCapsuleGroundContactScene();
dart::gui::ApplicationOptions makeHardcodedDesignScene();
dart::gui::ApplicationOptions makeBoxStackingScene();
dart::gui::ApplicationOptions makeHybridDynamicsScene();
dart::gui::ApplicationOptions makeVehicleScene();
dart::gui::ApplicationOptions makeImguiScene();
dart::gui::ApplicationOptions makeSimulationEventHandlerScene();
dart::gui::ApplicationOptions makeMixedChainScene();
dart::gui::ApplicationOptions makeCouplerConstraintScene();
dart::gui::ApplicationOptions makeMimicPendulumsScene();
dart::gui::ApplicationOptions makeRigidChainScene();
dart::gui::ApplicationOptions makeRigidLoopScene();
dart::gui::ApplicationOptions makeFreeJointCasesScene();
dart::gui::ApplicationOptions makeJointConstraintsScene();
dart::gui::ApplicationOptions makeSoftBodiesScene();
dart::gui::ApplicationOptions makeRigidShapesScene();
dart::gui::ApplicationOptions makeHeightmapScene();
dart::gui::ApplicationOptions makeLcpPhysicsScene();
dart::gui::ApplicationOptions makeBipedStandScene();
dart::gui::ApplicationOptions makeTinkertoyScene();
dart::gui::ApplicationOptions makeOperationalSpaceControlScene();
dart::gui::ApplicationOptions makePointCloudScene();
dart::gui::ApplicationOptions makeFetchScene();
dart::gui::ApplicationOptions makeAtlasPuppetScene();
dart::gui::ApplicationOptions makeHuboPuppetScene();
dart::gui::ApplicationOptions makeExperimentalRigidBodyScene();
dart::gui::ApplicationOptions makeG1PuppetScene();
dart::gui::ApplicationOptions makeCollisionSandboxScene();
dart::gui::ApplicationOptions makeHumanJointLimitsScene();
dart::gui::ApplicationOptions makeAtlasSimbiconScene();

/// Ordered demo catalog. Categories are shown in first-appearance order; scenes
/// appear in this vector's order within each category.
std::vector<dart::gui::DemoSceneEntry> makeDemoScenes();

} // namespace dart::examples::demos

#endif // DART_EXAMPLES_DEMOS_SCENES_HPP_
