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

dart::gui::ApplicationOptions makeRigidBodyScene();
dart::gui::ApplicationOptions makeDynamicJointConstraintsScene();
dart::gui::ApplicationOptions makeDeformableBodyScene();
dart::gui::ApplicationOptions makeVbdDeformableScene();

/// Ordered demo catalog. Categories are shown in first-appearance order; scenes
/// appear in this vector's order within each category.
std::vector<dart::gui::DemoSceneEntry> makeDemoScenes();

} // namespace dart::examples::demos

#endif // DART_EXAMPLES_DEMOS_SCENES_HPP_
