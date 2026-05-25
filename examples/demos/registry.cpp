/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

namespace dart::examples::demos {

std::vector<dart::gui::DemoSceneEntry> makeDemoScenes()
{
  std::vector<dart::gui::DemoSceneEntry> scenes;
  scenes.push_back(
      {"hello_world",
       "Hello World",
       "Getting Started",
       "A single box falling onto a ground plane.",
       &makeHelloWorldScene});
  scenes.push_back(
      {"shapes",
       "Shapes",
       "Visualization",
       "Assorted primitive shapes resting on the ground.",
       &makeShapesScene});
  scenes.push_back(
      {"boxes",
       "Boxes",
       "Rigid Body",
       "A grid of rigid boxes dropped onto the ground.",
       &makeBoxesScene});
  return scenes;
}

} // namespace dart::examples::demos
