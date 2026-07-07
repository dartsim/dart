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

// Mirrored from examples/hello_world: a single free box falling onto a static
// ground body. The standalone example remains the minimal first-program sample.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <memory>

namespace dart_demos {

namespace {

//==============================================================================
dart::dynamics::BodyNode* makeFallingBox(
    const dart::simulation::WorldPtr& world)
{
  auto skeleton = dart::dynamics::Skeleton::create("box");
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = pair.second;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() << 0.0, 0.0, 1.0;
  tf.linear() = dart::math::expMapRot(Eigen::Vector3d::Random());
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.3, 0.3, 0.3)));
  body->setInertia(dart::dynamics::Inertia(
      1.0,
      Eigen::Vector3d::Zero(),
      shapeNode->getShape()->computeInertia(1.0)));
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  world->addSkeleton(skeleton);
  return body;
}

//==============================================================================
void addGround(const dart::simulation::WorldPtr& world)
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, 0.1)));
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  world->addSkeleton(ground);
}

} // namespace

//==============================================================================
DemoScene makeHelloWorldScene()
{
  DemoScene scene;
  scene.id = "hello_world";
  scene.title = "Hello World";
  scene.category = "Getting Started";
  scene.summary = "A single free box falling onto a static ground body.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    auto* boxBody = makeFallingBox(world);
    addGround(world);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.57, 3.14, 1.64),
        ::osg::Vec3d(0.00, 0.00, 0.50),
        ::osg::Vec3d(-0.24, -0.25, 0.94)};

    setup.renderPanel = [boxBody] {
      const Eigen::Vector3d p = boxBody->getWorldTransform().translation();
      ImGui::Text("Box height: %.2f m", p.z());
      ImGui::Text(
          "Box velocity: %.2f m/s",
          boxBody->getSpatialVelocity().tail<3>().z());
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
