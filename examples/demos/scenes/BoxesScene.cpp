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

// Ported from examples/boxes: a 5x5x5 grid of colored boxes free-falls onto a
// static ground plane.
//
// Deviation from the original: examples/boxes explicitly switches to a
// Bullet collision detector. dart-demos relies on the host-selected collision
// detector instead, defaulting to the same backend as the rest of DART;
// box-drop behavior is otherwise unchanged.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

namespace dart_demos {

namespace {

//==============================================================================
dart::dynamics::SkeletonPtr createBox(
    std::size_t& index,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color)
{
  auto boxSkel
      = dart::dynamics::Skeleton::create("box" + std::to_string(index++));

  auto* boxBody
      = boxSkel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr)
            .second;

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto* shapeNode = boxBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.9);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  boxBody->getParentJoint()->setTransformFromParentBodyNode(tf);

  return boxSkel;
}

} // namespace

//==============================================================================
DemoScene makeBoxesScene()
{
  DemoScene scene;
  scene.id = "boxes";
  scene.title = "Boxes";
  scene.category = "Rigid Body";
  scene.summary = "A 5x5x5 grid of colored boxes dropped onto the ground.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();

    constexpr int dim = 5;
    std::size_t boxIndex = 0;
    for (int i = 0; i < dim; ++i) {
      for (int j = 0; j < dim; ++j) {
        for (int k = 0; k < dim; ++k) {
          const double x = i - dim / 2;
          const double y = j - dim / 2;
          const double z = k + 5;
          const Eigen::Vector3d position(x, y, z);
          const Eigen::Vector3d size(0.9, 0.9, 0.9);
          const Eigen::Vector3d color(
              static_cast<double>(i) / dim,
              static_cast<double>(j) / dim,
              static_cast<double>(k) / dim);
          world->addSkeleton(createBox(boxIndex, position, size, color));
        }
      }
    }

    auto ground = dart::dynamics::Skeleton::create("ground");
    auto* groundBody
        = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>()
              .second;
    auto* groundShape = groundBody->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(25.0, 25.0, 0.1)));
    groundShape->getVisualAspect()->setColor(dart::Color::LightGray());
    groundShape->getDynamicsAspect()->setRestitutionCoeff(0.9);
    world->addSkeleton(ground);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(20.0, 20.0, 15.0),
        ::osg::Vec3d(0.00, 0.00, 3.00),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.renderPanel = [world] {
      const std::size_t numBoxes
          = world->getNumSkeletons() > 0 ? world->getNumSkeletons() - 1 : 0;
      ImGui::Text("Boxes: %zu", numBoxes);
      ImGui::TextWrapped(
          "A 5x5x5 grid of boxes free-falls onto the ground plane under "
          "gravity; each box has a restitution coefficient of 0.9.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
