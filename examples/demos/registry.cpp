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
      {"rigid_body",
       "Rigid Body",
       "World Rigid Body",
       "Falling rigid bodies on the World solver.",
       &makeRigidBodyScene});
  scenes.push_back(
      {"deformable_body",
       "Deformable Body",
       "IPC Deformable",
       "A spring-net deformable body on the World solver.",
       &makeDeformableBodyScene});
  scenes.push_back(
      {"vbd_deformable",
       "Deformable VBD",
       "Vertex Block Descent",
       "A contact-free hanging cloth driven by the Vertex Block Descent "
       "inner solver (graph-colored Gauss-Seidel block descent).",
       &makeVbdDeformableScene});

  return scenes;
}

} // namespace dart::examples::demos
