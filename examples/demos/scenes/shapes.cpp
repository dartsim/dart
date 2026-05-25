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
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart::examples::demos {

namespace {

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

dart::dynamics::SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                   .second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d::Constant(0.24));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);
  body->setInertia(
      dart::dynamics::Inertia(
          1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));
  return skeleton;
}

dart::dynamics::SkeletonPtr createSphere(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
                   .second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::SphereShape>(0.14);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);
  body->setInertia(
      dart::dynamics::Inertia(
          1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));
  return skeleton;
}

} // namespace

dart::gui::ApplicationOptions makeShapesScene()
{
  auto world = dart::simulation::World::create("shapes");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createGround());
  world->addSkeleton(createBox("box_a", {-0.5, 0.0, 0.4}, {0.90, 0.45, 0.20}));
  world->addSkeleton(
      createSphere("sphere_a", {0.0, 0.0, 0.5}, {0.20, 0.55, 0.90}));
  world->addSkeleton(createBox("box_b", {0.5, 0.0, 0.6}, {0.40, 0.80, 0.35}));

  dart::gui::ApplicationOptions options;
  options.world = world;

  dart::gui::Panel panel;
  panel.title = "Shapes";
  panel.initialPosition = std::array<double, 2>{312.0, 12.0};
  panel.buildWithContext
      = [](dart::gui::PanelBuilder& builder, dart::gui::PanelContext&) {
          builder.text("Assorted primitive shapes settling on the ground.");
        };
  options.panels.push_back(panel);

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.30);
  camera.distance = 3.2;
  options.camera = camera;
  return options;
}

} // namespace dart::examples::demos
