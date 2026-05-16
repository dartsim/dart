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
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

constexpr const char* kGroundUri = "dart://sample/skel/ground.skel";
constexpr const char* kGroundName = "add_delete_ground";
constexpr const char* kCubePrefix = "spawned_cube_";

dart::dynamics::SkeletonPtr createCube(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color,
    double mass = 0.1)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(mass);
  body.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(size, mass));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(position);

  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, joint, body);
  auto shape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto* shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  return skeleton;
}

struct AddDeleteState
{
  explicit AddDeleteState(dart::simulation::WorldPtr inputWorld)
    : world(std::move(inputWorld))
  {
  }

  void spawnCube()
  {
    static const std::array<Eigen::Vector3d, 6> kPositions{{
        Eigen::Vector3d(-0.8, 0.65, -0.7),
        Eigen::Vector3d(-0.35, 1.05, 0.25),
        Eigen::Vector3d(0.15, 0.75, -0.15),
        Eigen::Vector3d(0.55, 1.25, 0.55),
        Eigen::Vector3d(0.9, 0.95, -0.45),
        Eigen::Vector3d(-0.1, 1.45, 0.75),
    }};
    static const std::array<Eigen::Vector3d, 6> kSizes{{
        Eigen::Vector3d(0.25, 0.22, 0.30),
        Eigen::Vector3d(0.18, 0.28, 0.22),
        Eigen::Vector3d(0.32, 0.18, 0.24),
        Eigen::Vector3d(0.22, 0.34, 0.20),
        Eigen::Vector3d(0.28, 0.24, 0.18),
        Eigen::Vector3d(0.30, 0.20, 0.26),
    }};
    static const std::array<Eigen::Vector3d, 6> kColors{{
        Eigen::Vector3d(0.85, 0.30, 0.24),
        Eigen::Vector3d(0.25, 0.55, 0.90),
        Eigen::Vector3d(0.95, 0.72, 0.25),
        Eigen::Vector3d(0.35, 0.78, 0.45),
        Eigen::Vector3d(0.68, 0.38, 0.85),
        Eigen::Vector3d(0.25, 0.75, 0.70),
    }};

    if (world == nullptr) {
      return;
    }

    const std::size_t index = nextCubeIndex++;
    world->addSkeleton(createCube(
        std::string(kCubePrefix) + std::to_string(index),
        kPositions[index % kPositions.size()],
        kSizes[index % kSizes.size()],
        kColors[index % kColors.size()]));
  }

  void deleteLastCube()
  {
    if (world == nullptr || world->getNumSkeletons() <= 1) {
      return;
    }

    for (std::size_t i = world->getNumSkeletons(); i > 0; --i) {
      auto skeleton = world->getSkeleton(i - 1);
      if (skeleton != nullptr
          && skeleton->getName().rfind(kCubePrefix, 0) == 0) {
        world->removeSkeleton(skeleton);
        return;
      }
    }
  }

  dart::simulation::WorldPtr world;
  std::size_t nextCubeIndex = 0;
};

std::shared_ptr<AddDeleteState> createAddDeleteState(
    const dart::simulation::WorldPtr& world)
{
  auto state = std::make_shared<AddDeleteState>(world);
  for (int i = 0; i < 5; ++i) {
    state->spawnCube();
  }
  return state;
}

dart::simulation::WorldPtr createAddDeleteWorld()
{
  auto world = dart::io::readWorld(kGroundUri);
  if (world == nullptr) {
    throw std::runtime_error("Failed to load dart://sample/skel/ground.skel");
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  if (auto ground = world->getSkeleton(0)) {
    ground->setName(kGroundName);
  }
  return world;
}

dart::gui::Panel createControlsPanel(
    const std::shared_ptr<AddDeleteState>& state)
{
  dart::gui::Panel panel;
  panel.title = "Add/Delete Skeletons";
  panel.buildWithContext = [state](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Spawn or delete dynamic cube skeletons.");
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
    if (builder.button("Spawn Cube")) {
      state->spawnCube();
    }
    builder.sameLine();
    if (builder.button("Delete Cube")) {
      state->deleteLastCube();
    }
    if (context.world != nullptr) {
      builder.text(
          "skeletons: " + std::to_string(context.world->getNumSkeletons()));
    }
    builder.text("time: " + std::to_string(context.simulationTime));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto world = createAddDeleteWorld();
    auto state = createAddDeleteState(world);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.panels.push_back(createControlsPanel(state));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "add_delete_skels: " << e.what() << "\n";
    return 1;
  }
}
