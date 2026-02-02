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

  // Create world
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  // Ground plane
  {
    auto ground = dart::dynamics::Skeleton::create("ground");
    auto [joint, body]
        = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
    auto shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(10, 10, 0.1));
    auto shapeNode = body->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect>(shape);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.7, 0.7, 0.7));
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation().z() = -0.05;
    joint->setTransformFromParentBodyNode(tf);
    world->addSkeleton(ground);
  }

  // Draggable SimpleFrame with a red box
  auto draggableFrame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "draggable");
  {
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(-1.0, 1.0, 0.5);
    draggableFrame->setTransform(tf);
    auto shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.3, 0.3, 0.3));
    draggableFrame->setShape(shape);
    draggableFrame->getVisualAspect(true)->setColor(
        Eigen::Vector3d(0.9, 0.2, 0.2));
  }

  // Robot arm for BodyNode IK dragging
  dart::dynamics::SkeletonPtr arm;
  dart::dynamics::BodyNode* endEffector = nullptr;
  {
    arm = dart::dynamics::Skeleton::create("arm");

    // Base link — fixed to world
    auto [baseJoint, baseBody]
        = arm->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr);
    baseBody->setName("base");
    auto baseShape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.2, 0.2, 0.1));
    baseBody->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect>(baseShape);
    Eigen::Isometry3d baseTf = Eigen::Isometry3d::Identity();
    baseTf.translation() = Eigen::Vector3d(1.0, 0.0, 0.05);
    baseJoint->setTransformFromParentBodyNode(baseTf);

    // Link 1 — revolute
    dart::dynamics::RevoluteJoint::Properties props1;
    props1.mName = "joint1";
    props1.mAxis = Eigen::Vector3d::UnitY();
    props1.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 0.05);
    auto [j1, link1]
        = arm->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
            baseBody, props1);
    link1->setName("link1");
    auto link1Shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.08, 0.08, 0.4));
    auto link1SN
        = link1->createShapeNodeWith<dart::dynamics::VisualAspect>(link1Shape);
    link1SN->getVisualAspect()->setColor(Eigen::Vector3d(0.3, 0.5, 0.8));
    Eigen::Isometry3d shapeTf1 = Eigen::Isometry3d::Identity();
    shapeTf1.translation().z() = 0.2;
    link1SN->setRelativeTransform(shapeTf1);

    // Link 2 — revolute
    dart::dynamics::RevoluteJoint::Properties props2;
    props2.mName = "joint2";
    props2.mAxis = Eigen::Vector3d::UnitY();
    props2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 0.4);
    auto [j2, link2]
        = arm->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
            link1, props2);
    link2->setName("link2");
    auto link2Shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.06, 0.06, 0.3));
    auto link2SN
        = link2->createShapeNodeWith<dart::dynamics::VisualAspect>(link2Shape);
    link2SN->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.5, 0.3));
    Eigen::Isometry3d shapeTf2 = Eigen::Isometry3d::Identity();
    shapeTf2.translation().z() = 0.15;
    link2SN->setRelativeTransform(shapeTf2);

    endEffector = link2;
    world->addSkeleton(arm);
  }

  // Axis markers (X=red, Y=green, Z=blue)
  for (int i = 0; i < 3; ++i) {
    auto skel = dart::dynamics::Skeleton::create("axis_" + std::to_string(i));
    auto [joint, body]
        = skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
    auto shape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.15, 0.15, 0.15));
    auto sn = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
    Eigen::Vector3d color = Eigen::Vector3d::Zero();
    color[i] = 1.0;
    sn->getVisualAspect()->setColor(color);
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation()[i] = 3.0;
    joint->setTransformFromParentBodyNode(tf);
    world->addSkeleton(skel);
  }

  // Set up viewer
  auto config = dart::gui::ViewerConfig{};
  config.title = "DART Raylib - Drag and Drop";
  if (maxFrames >= 0) {
    config.headless = true;
    config.width = 320;
    config.height = 240;
  }

  auto backend = std::make_unique<dart::gui::RaylibBackend>();
  auto viewer = dart::gui::SceneViewer(std::move(backend), config);
  viewer.setWorld(world);

  // Register the draggable SimpleFrame (adds green sphere marker)
  world->addSimpleFrame(draggableFrame);
  viewer.enableDragAndDrop(draggableFrame.get());

  // Enable IK drag on the end effector
  if (endEffector) {
    viewer.enableDragAndDrop(endEffector, true, false);
  }

  // Position camera to see the whole scene
  viewer.camera().position = Eigen::Vector3d(4.0, -4.0, 3.0);
  viewer.camera().target = Eigen::Vector3d(0.0, 0.0, 0.5);

  if (maxFrames >= 0) {
    for (int i = 0; i < maxFrames && viewer.frame(); ++i) {
    }
  } else {
    viewer.run();
  }

  return 0;
}
