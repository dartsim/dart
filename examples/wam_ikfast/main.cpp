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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kWamPackageName = "herb_description";
constexpr const char* kWamPackagePath = "urdf/wam";
constexpr const char* kWamUrdfPath = "urdf/wam/wam.urdf";
constexpr const char* kWamSkeletonName = "visual_wam_ikfast_robot";
constexpr const char* kTargetFrameName = "wam_ikfast_target";
constexpr const char* kGroundSkeletonName = "visual_wam_ikfast_ground";

void makeVisualOnlySkeleton(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return;
  }

  skeleton->setMobile(false);
  skeleton->disableSelfCollisionCheck();
  skeleton->setAdjacentBodyCheck(false);
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->setCollidable(false);
    body->setGravityMode(false);
    body->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
        [](dart::dynamics::ShapeNode* shapeNode) {
          shapeNode->getCollisionAspect()->setCollidable(false);
        });
  }
}

dart::dynamics::SkeletonPtr loadWamIkFastSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      kWamPackageName, dart::config::dataPath(kWamPackagePath));
  const auto wamUri
      = dart::common::Uri::createFromPath(dart::config::dataPath(kWamUrdfPath));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (wam == nullptr) {
    throw std::runtime_error(
        "Failed to load WAM IKFast robot from " + wamUri.toString());
  }

  wam->setName(kWamSkeletonName);
  const std::array<std::pair<const char*, double>, 7> jointPositions{{
      {"/j1", 0.0},
      {"/j2", 0.0},
      {"/j3", 0.0},
      {"/j4", 0.0},
      {"/j5", 0.0},
      {"/j6", 0.0},
      {"/j7", 0.0},
  }};
  for (const auto& [name, position] : jointPositions) {
    auto* dof = wam->getDof(name);
    if (dof == nullptr) {
      throw std::runtime_error(
          "WAM IKFast robot is missing expected DOF " + std::string(name));
    }
    dof->setPosition(position);
  }

  makeVisualOnlySkeleton(wam);
  return wam;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, 0.01)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.005));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.18, 0.32, 0.58, 1.0));
  return ground;
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createTargetHandleShape(
    double radius)
{
  auto handle = std::make_shared<dart::dynamics::LineSegmentShape>(7.0f);
  const std::size_t center = handle->addVertex(Eigen::Vector3d::Zero());
  const auto addAxis = [&](const Eigen::Vector3d& axis) {
    handle->addVertex(axis, center);
    handle->addVertex(-axis, center);
  };
  addAxis(Eigen::Vector3d(radius, 0.0, 0.0));
  addAxis(Eigen::Vector3d(0.0, radius, 0.0));
  addAxis(Eigen::Vector3d(0.0, 0.0, 0.75 * radius));
  return handle;
}

dart::simulation::WorldPtr createWamIkFastWorld()
{
  auto world = dart::simulation::World::create("dartsim_wam_ikfast");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createGround());

  auto wam = loadWamIkFastSkeleton();
  auto* endEffector = wam->getBodyNode("/wam7");
  if (endEffector == nullptr) {
    throw std::runtime_error("WAM IKFast robot is missing /wam7 body node");
  }

  Eigen::Isometry3d targetTransform = endEffector->getWorldTransform();
  targetTransform.translate(Eigen::Vector3d(0.0, 0.0, -0.09));
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kTargetFrameName, targetTransform);
  target->setShape(createTargetHandleShape(0.15));
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.18, 0.55, 1.0, 0.92));
  world->addSimpleFrame(target);
  world->addSkeleton(wam);

  return world;
}

dart::gui::Panel createWamIkFastPanel()
{
  dart::gui::Panel panel;
  panel.title = "WAM IKFast";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("WAM IKFast visual target scene");
    builder.text("Select the blue target handle.");
    builder.text("Ctrl-left drag moves the selected handle.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    dart::gui::ApplicationOptions options;
    options.world = createWamIkFastWorld();
    options.panels.push_back(createWamIkFastPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "wam_ikfast: " << e.what() << "\n";
    return 1;
  }
}
