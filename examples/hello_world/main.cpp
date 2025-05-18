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

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <osgShadow/ShadowMap>

using namespace dart;

int main()
{
  // Create a box-shaped rigid body
  auto skeleton = dynamics::Skeleton::create();
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto body = jointAndBody.second;
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() << 0, 0, 1;
  tf.linear() = math::expMapRot(Eigen::Vector3d::Random());
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  auto shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.3, 0.3, 0.3)));
  body->setInertia(dynamics::Inertia(
      1, Eigen::Vector3d::Zero(), shapeNode->getShape()->computeInertia(1.0)));
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Create ground
  auto ground = dynamics::Skeleton::create("ground");
  auto groundBody
      = ground->createJointAndBodyNodePair<dynamics::WeldJoint>().second;
  auto groundShapeNode = groundBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  // Create a world and add the rigid body and ground to the world
  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->addSkeleton(ground);

  // Wrap a WorldNode around it
  ::osg::ref_ptr<gui::osg::RealTimeWorldNode> node
      = new gui::osg::RealTimeWorldNode(world);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = gui::osg::Viewer();
  viewer.addWorldNode(node);

  // Enable shadow
  auto shadow
      = dart::gui::osg::WorldNode::createDefaultShadowTechnique(&viewer);
  if (auto sm = dynamic_cast<::osgShadow::ShadowMap*>(shadow.get())) {
    auto mapResolution = static_cast<short>(std::pow(2, 12));
    sm->setTextureSize(::osg::Vec2s(mapResolution, mapResolution));
  }
  node->setShadowTechnique(shadow);

  viewer.addInstructionText("Press space to start free falling the box.\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57f, 3.14f, 1.64f),
      ::osg::Vec3(0.00f, 0.00f, 0.50f),
      ::osg::Vec3(-0.24f, -0.25f, 0.94f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
