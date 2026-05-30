/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "atlas_simbicon/controller.hpp"
#include "atlas_simbicon/state_machine.hpp"
#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace dart::examples::demos {

namespace {

constexpr const char* kAtlasUri
    = "dart://sample/sdf/atlas/atlas_v5_no_head.urdf";
constexpr double kDefaultGravity = 9.81;
constexpr double kGroundHalfExtent = 12.5;
constexpr double kGroundThickness = 0.05;
constexpr double kGroundCenterZ = -0.95;

dart::dynamics::SkeletonPtr readRequiredSkeleton(const char* uri)
{
  auto skeleton = dart::io::readSkeleton(uri);
  if (skeleton == nullptr) {
    throw std::runtime_error(std::string("Failed to load ") + uri);
  }
  return skeleton;
}

void makeAtlasMeshVisualsReadable(const dart::dynamics::SkeletonPtr& atlas)
{
  const Eigen::Vector4d readableAtlasColor(0.15, 0.16, 0.18, 1.0);
  for (std::size_t i = 0; i < atlas->getNumBodyNodes(); ++i) {
    auto* body = atlas->getBodyNode(i);
    for (std::size_t j = 0; j < body->getNumShapeNodes(); ++j) {
      auto* shapeNode = body->getShapeNode(j);
      auto* visual = shapeNode->getVisualAspect();
      if (visual == nullptr) {
        continue;
      }
      const auto mesh = std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
          shapeNode->getShape());
      if (mesh == nullptr) {
        continue;
      }
      mesh->setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR);
      const Eigen::Vector4d rgba = visual->getRGBA();
      Eigen::Vector4d readable = readableAtlasColor;
      readable.w() = rgba.w();
      visual->setRGBA(readable);
    }
  }
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("atlas_simbicon_ground");
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  body->setName("ground_link");
  auto shape = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
      2.0 * kGroundHalfExtent, 2.0 * kGroundHalfExtent, kGroundThickness));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, kGroundCenterZ));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.54, 0.56, 0.52, 1.0));
  ground->setMobile(false);
  return ground;
}

struct AtlasSimbiconState
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
  std::unique_ptr<Controller> controller;

  void preStep()
  {
    if (controller != nullptr) {
      controller->update();
    }
  }
};

std::shared_ptr<AtlasSimbiconState> createState()
{
  auto state = std::make_shared<AtlasSimbiconState>();
  state->world = dart::simulation::World::create("dartsim_atlas_simbicon");
  state->world->setGravity(-kDefaultGravity * Eigen::Vector3d::UnitZ());
  state->world->addSkeleton(createGround());

  state->atlas = readRequiredSkeleton(kAtlasUri);
  makeAtlasMeshVisualsReadable(state->atlas);
  state->world->addSkeleton(state->atlas);

  state->controller = std::make_unique<Controller>(
      state->atlas, state->world->getConstraintSolver());
  return state;
}

dart::gui::Panel makePanel(const std::shared_ptr<AtlasSimbiconState>& state)
{
  dart::gui::Panel panel;
  panel.title = "Atlas SIMBICON";
  panel.buildWithContext
      = [state](dart::gui::PanelBuilder& builder, dart::gui::PanelContext&) {
          builder.text("Atlas humanoid driven by the SIMBICON controller.");
          builder.text("Controller advances every frame via preStep.");
          if (state->controller != nullptr) {
            builder.text("Controller active: yes");
          }
        };
  return panel;
}

dart::gui::OrbitCamera makeCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.5);
  camera.distance = 4.0;
  camera.yaw = 0.6;
  camera.pitch = 0.25;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeAtlasSimbiconScene()
{
  const auto state = createState();

  dart::gui::ApplicationOptions options;
  options.world = state->world;
  options.camera = makeCamera();
  options.preStep = [state]() {
    state->preStep();
  };
  options.panels.push_back(makePanel(state));
  return options;
}

} // namespace dart::examples::demos
