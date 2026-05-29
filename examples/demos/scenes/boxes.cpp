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

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart::examples::demos {

namespace {

constexpr int kGridSize = 3;
constexpr double kBoxExtent = 0.18;
constexpr double kSpacing = 0.32;

dart::dynamics::SkeletonPtr createBox(
    int index, const Eigen::Vector3d& position)
{
  auto skeleton
      = dart::dynamics::Skeleton::create("box_" + std::to_string(index));
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(kBoxExtent));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  const double hue = static_cast<double>(index % 6) / 6.0;
  shapeNode->getVisualAspect()->setColor(
      Eigen::Vector3d(0.35 + 0.5 * hue, 0.45, 0.9 - 0.4 * hue));

  constexpr double mass = 1.0;
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));

  return skeleton;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto jointAndBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = jointAndBody.first;
  auto* body = jointAndBody.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10.0, 10.0, 0.1));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  return ground;
}

} // namespace

dart::gui::ApplicationOptions makeBoxesScene()
{
  auto world = dart::simulation::World::create("boxes");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  int index = 0;
  const double origin = -0.5 * (kGridSize - 1) * kSpacing;
  for (int x = 0; x < kGridSize; ++x) {
    for (int y = 0; y < kGridSize; ++y) {
      for (int z = 0; z < kGridSize; ++z) {
        const Eigen::Vector3d position(
            origin + x * kSpacing, origin + y * kSpacing, 0.4 + z * kSpacing);
        world->addSkeleton(createBox(index++, position));
      }
    }
  }
  world->addSkeleton(createGround());

  dart::gui::ApplicationOptions options;
  options.world = world;

  dart::gui::Panel panel;
  panel.title = "Boxes";
  panel.initialPosition = std::array<double, 2>{312.0, 12.0};
  panel.buildWithContext
      = [](dart::gui::PanelBuilder& builder, dart::gui::PanelContext& context) {
          builder.text("A grid of rigid boxes dropped onto the ground.");
          builder.text("contacts: " + std::to_string(context.contactCount));
        };
  options.panels.push_back(panel);

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.30);
  camera.distance = 3.6;
  options.camera = camera;
  return options;
}

} // namespace dart::examples::demos
