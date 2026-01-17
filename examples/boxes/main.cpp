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

#include <dart/gui/osg/osg.hpp>

#include <dart/collision/bullet/bullet.hpp>

#include <dart/dart.hpp>

#include <osgShadow/ShadowMap>

using namespace dart;

[[nodiscard]] dynamics::SkeletonPtr createBox(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size = Eigen::Vector3d(1, 1, 1),
    const Eigen::Vector3d& color
    = dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0))
{
  static size_t index = 0;
  dynamics::SkeletonPtr boxSkel
      = dynamics::Skeleton::create("box" + std::to_string(index++));

  // Give the floor a body
  dynamics::BodyNodePtr boxBody
      = boxSkel->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr)
            .second;

  // Give the body a shape
  auto boxShape = std::make_shared<dynamics::BoxShape>(size);
  dynamics::ShapeNode* shapeNode = boxBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.9);

  // Put the body into position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

int main()
{
  // Create an empty world
  auto world = simulation::World::create();

  // Set collision detector type
  world->getConstraintSolver()->setCollisionDetector(
      collision::BulletCollisionDetector::create());

  // Create dim x dim x dim boxes
  auto dim = 5;
  for (auto i = 0; i < dim; ++i) {
    for (auto j = 0; j < dim; ++j) {
      for (auto k = 0; k < dim; ++k) {
        auto x = i - dim / 2;
        auto y = j - dim / 2;
        auto z = k + 5;
        auto position = Eigen::Vector3d(x, y, z);
        auto size = Eigen::Vector3d(0.9, 0.9, 0.9);
        auto color = Eigen::Vector3d(
            static_cast<double>(i) / dim,
            static_cast<double>(j) / dim,
            static_cast<double>(k) / dim);
        auto box = createBox(position, size, color);
        world->addSkeleton(box);
      }
    }
  }

  // Create ground
  auto ground = dynamics::Skeleton::create("ground");
  auto groundBody
      = ground->createJointAndBodyNodePair<dynamics::WeldJoint>().second;
  auto groundShapeNode = groundBody->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(25.0, 25.0, 0.1)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());
  groundShapeNode->getDynamicsAspect()->setRestitutionCoeff(0.9);
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

  viewer.addInstructionText(
      "[Experimental] Please note: This example is in an experimental phase "
      "and may not be fully functional at this time.\n");
  viewer.addInstructionText("Press space to start free falling the box.\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 1360x768 (HD)
  viewer.setUpViewInWindow(0, 0, 1360, 768);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(20.0f, 20.0f, 15.0f),
      ::osg::Vec3(0.00f, 0.00f, 3.00f),
      ::osg::Vec3(0, 0, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
