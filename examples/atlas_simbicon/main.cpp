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

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kAtlasUri
    = "dart://sample/sdf/atlas/atlas_v3_no_head.sdf";
constexpr const char* kAtlasSkeletonName = "visual_atlas_robot";
constexpr const char* kGroundSkeletonName = "visual_atlas_simbicon_ground";

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

dart::dynamics::SkeletonPtr loadAtlasSimbiconSkeleton()
{
  const auto atlasUri = dart::common::Uri::createFromString(kAtlasUri);
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (atlas == nullptr) {
    throw std::runtime_error(
        "Failed to load Atlas Simbicon model from " + atlasUri.toString());
  }

  atlas->setName(kAtlasSkeletonName);
  if (atlas->getNumDofs() == 0) {
    throw std::runtime_error("Atlas Simbicon model has no root DOF");
  }

  constexpr double halfPi = 1.5707963267948966;
  atlas->setPosition(0, -halfPi);
  auto* rootBody = atlas->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation() = Eigen::Vector3d(0.0, 0.92, 0.0);
    dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
  }

  makeVisualOnlySkeleton(atlas);
  return atlas;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto [joint, body]
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();

  constexpr double groundThickness = 0.08;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().y() = -0.5 * groundThickness;
  joint->setTransformFromParentBodyNode(transform);

  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(5.5, groundThickness, 4.0)));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.74, 0.76, 0.72, 1.0));
  return ground;
}

dart::simulation::WorldPtr createAtlasSimbiconWorld()
{
  auto world = dart::simulation::World::create("dartsim_atlas_simbicon");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createGround());
  world->addSkeleton(loadAtlasSimbiconSkeleton());
  return world;
}

dart::gui::Panel createAtlasSimbiconPanel()
{
  dart::gui::Panel panel;
  panel.title = "Atlas Simbicon";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Atlas Simbicon visual model");
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
    options.world = createAtlasSimbiconWorld();
    options.panels.push_back(createAtlasSimbiconPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "atlas_simbicon: " << e.what() << "\n";
    return 1;
  }
}
