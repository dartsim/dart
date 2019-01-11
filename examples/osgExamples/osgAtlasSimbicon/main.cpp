/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>

#include "AtlasSimbiconWorldNode.hpp"
#include "AtlasSimbiconEventHandler.hpp"
#include "AtlasSimbiconWidget.hpp"

int main()
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Load ground and Atlas robot and add them to the world
  dart::io::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton("dart://sample/sdf/atlas/ground.urdf");
  auto atlas = dart::io::SdfParser::readSkeleton(
        "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  world->addSkeleton(ground);
  world->addSkeleton(atlas);

  // Set initial configuration for Atlas robot
  atlas->setPosition(0, -0.5 * dart::math::constantsd::pi());

  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  // Wrap a WorldNode around it
  osg::ref_ptr<AtlasSimbiconWorldNode> node
      = new AtlasSimbiconWorldNode(world, atlas);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(
        std::make_shared<AtlasSimbiconWidget>(&viewer, node.get()));

  // Pass in the custom event handler
  viewer.addEventHandler(new AtlasSimbiconEventHandler(node));

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("Atlas Simbicon");

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*2.0,
        ::osg::Vec3d( 1.00,  0.00, 0.00),
        ::osg::Vec3d( 0.00,  0.1, 0.00));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}
