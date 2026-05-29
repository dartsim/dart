/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/config.hpp>

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

#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

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
    : world(std::move(inputWorld)), randomEngine(std::random_device{}())
  {
  }

  void spawnRandomCube()
  {
    if (world == nullptr) {
      return;
    }

    std::uniform_real_distribution<double> positionX(-1.0, 1.0);
    std::uniform_real_distribution<double> positionY(0.5, 1.0);
    std::uniform_real_distribution<double> positionZ(-1.0, 1.0);
    std::uniform_real_distribution<double> size(0.1, 0.5);
    std::uniform_real_distribution<double> color(0.0, 1.0);

    const std::size_t index = nextCubeIndex++;
    world->addSkeleton(createCube(
        std::string(kCubePrefix) + std::to_string(index),
        Eigen::Vector3d(
            positionX(randomEngine),
            positionY(randomEngine),
            positionZ(randomEngine)),
        Eigen::Vector3d(
            size(randomEngine), size(randomEngine), size(randomEngine)),
        Eigen::Vector3d(
            color(randomEngine), color(randomEngine), color(randomEngine))));
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
  std::mt19937 randomEngine;
};

std::shared_ptr<AddDeleteState> createAddDeleteState(
    const dart::simulation::WorldPtr& world)
{
  return std::make_shared<AddDeleteState>(world);
}

void preferBulletCollisionDetector(dart::simulation::World& world)
{
#if DART_HAVE_BULLET
  world.setCollisionDetector(dart::simulation::CollisionDetectorType::Bullet);
#else
  (void)world;
#endif
}

dart::gui::OrbitCamera makeAddDeleteCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.5404195002705842;
  camera.pitch = 0.4758822496604165;
  camera.distance = 6.557438524302;
  return camera;
}

dart::simulation::WorldPtr createAddDeleteWorld()
{
  auto world = dart::io::readWorld(kGroundUri);
  if (world == nullptr) {
    throw std::runtime_error("Failed to load dart://sample/skel/ground.skel");
  }

  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  preferBulletCollisionDetector(*world);
  if (auto ground = world->getSkeleton(0)) {
    ground->setName(kGroundName);
  }
  return world;
}

std::vector<dart::gui::KeyboardAction> createAddDeleteKeyboardActions(
    const std::shared_ptr<AddDeleteState>& state)
{
  std::vector<dart::gui::KeyboardAction> actions;

  dart::gui::KeyboardAction spawn;
  spawn.label = "Spawn random cube";
  spawn.shortcut = dart::gui::KeyboardShortcut::characterKey('q');
  spawn.callback = [state](dart::gui::KeyboardActionContext&) {
    state->spawnRandomCube();
  };
  actions.push_back(std::move(spawn));

  dart::gui::KeyboardAction remove;
  remove.label = "Delete last spawned cube";
  remove.shortcut = dart::gui::KeyboardShortcut::characterKey('w');
  remove.callback = [state](dart::gui::KeyboardActionContext&) {
    state->deleteLastCube();
  };
  actions.push_back(std::move(remove));

  return actions;
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
    builder.text("'q': spawn a random cube");
    builder.text("'w': delete a spawned cube");
    builder.text("space bar: simulation on/off");
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
      state->spawnRandomCube();
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

dart::gui::ApplicationOptions makeAddDeleteSkelsScene()
{
  auto world = createAddDeleteWorld();
  auto state = createAddDeleteState(world);

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.camera = makeAddDeleteCamera();
  options.keyboardActions = createAddDeleteKeyboardActions(state);
  options.panels.push_back(createControlsPanel(state));
  return options;
}

} // namespace dart::examples::demos
