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

#include <dart/gui/raylib/raylib_backend.hpp>
#include <dart/gui/scene_viewer.hpp>

#include <dart/all.hpp>

#include <string_view>

#include <cstdlib>

int main(int argc, char* argv[])
{
  int maxFrames = -1;
  for (int i = 1; i < argc; ++i) {
    if (std::string_view(argv[i]) == "--frames" && i + 1 < argc) {
      maxFrames = std::atoi(argv[i + 1]);
      ++i;
    }
  }

  // Create a world with some shapes
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  // Create ground plane skeleton
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto [groundJoint, groundBody]
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto groundShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10, 10, 0.1));
  auto groundShapeNode = groundBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(groundShape);
  groundShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.7, 0.7, 0.7));
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation().z() = -0.05;
  groundJoint->setTransformFromParentBodyNode(groundTf);
  world->addSkeleton(ground);

  // Create a falling box
  auto box = dart::dynamics::Skeleton::create("box");
  auto [boxJoint, boxBody]
      = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.3, 0.3, 0.3));
  auto boxShapeNode = boxBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(boxShape);
  boxShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.2, 0.2));
  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(boxShape->computeInertia(1.0));
  boxBody->setInertia(inertia);
  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.translation() = Eigen::Vector3d(0, 0, 1.0);
  boxJoint->setTransformFromParentBodyNode(boxTf);
  world->addSkeleton(box);

  // Create a sphere
  auto sphereSkel = dart::dynamics::Skeleton::create("sphere");
  auto [sphereJoint, sphereBody]
      = sphereSkel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto sphereShape = std::make_shared<dart::dynamics::SphereShape>(0.15);
  auto sphereShapeNode = sphereBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(sphereShape);
  sphereShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.6, 0.8));
  dart::dynamics::Inertia sphereInertia;
  sphereInertia.setMass(0.5);
  sphereInertia.setMoment(sphereShape->computeInertia(0.5));
  sphereBody->setInertia(sphereInertia);
  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.5, 0.0, 2.0);
  sphereJoint->setTransformFromParentBodyNode(sphereTf);
  world->addSkeleton(sphereSkel);

  // Create a cylinder
  auto cylSkel = dart::dynamics::Skeleton::create("cylinder");
  auto [cylJoint, cylBody]
      = cylSkel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto cylShape = std::make_shared<dart::dynamics::CylinderShape>(0.12, 0.5);
  auto cylShapeNode = cylBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(cylShape);
  cylShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.3, 0.8, 0.3));
  dart::dynamics::Inertia cylInertia;
  cylInertia.setMass(0.8);
  cylInertia.setMoment(cylShape->computeInertia(0.8));
  cylBody->setInertia(cylInertia);
  Eigen::Isometry3d cylTf = Eigen::Isometry3d::Identity();
  cylTf.translation() = Eigen::Vector3d(-0.4, 0.3, 1.5);
  cylJoint->setTransformFromParentBodyNode(cylTf);
  world->addSkeleton(cylSkel);

  // Create and run the viewer
  auto config = dart::gui::ViewerConfig{};
  config.width = 1280;
  config.height = 720;
  config.title = "DART + Raylib Visualization";

  auto backend = std::make_unique<dart::gui::RaylibBackend>();
  auto viewer = dart::gui::SceneViewer(std::move(backend), config);
  viewer.setWorld(world);

  if (maxFrames >= 0) {
    for (int i = 0; i < maxFrames && viewer.frame(); ++i) {
    }
  } else {
    viewer.run();
  }

  return 0;
}
