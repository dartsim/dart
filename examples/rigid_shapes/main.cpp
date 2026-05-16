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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr const char* kGroundName = "rigid_shapes_ground";
constexpr const char* kShapePrefix = "rigid_shape_";

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundName);
  auto pair = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(7.0, 7.0, 0.08));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.76, 0.78, 0.76));

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.04;
  pair.first->setTransformFromParentBodyNode(transform);

  return ground;
}

dart::dynamics::SkeletonPtr createDynamicShape(
    const std::string& name,
    std::shared_ptr<dart::dynamics::Shape> shape,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color,
    double mass = 1.0)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  joint->setTransformFromParentBodyNode(transform);

  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.35);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);

  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));
  return skeleton;
}

Eigen::Isometry3d makePose(std::size_t index)
{
  static const std::array<Eigen::Vector3d, 6> kPositions{{
      Eigen::Vector3d(-1.4, -0.25, 0.45),
      Eigen::Vector3d(-0.85, 0.25, 0.75),
      Eigen::Vector3d(-0.25, -0.15, 1.05),
      Eigen::Vector3d(0.35, 0.25, 1.35),
      Eigen::Vector3d(0.95, -0.20, 1.65),
      Eigen::Vector3d(1.55, 0.25, 1.95),
  }};

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = kPositions[index % kPositions.size()];
  transform.rotate(
      Eigen::AngleAxisd(
          0.22 * static_cast<double>(index + 1), Eigen::Vector3d::UnitX()));
  transform.rotate(
      Eigen::AngleAxisd(
          0.17 * static_cast<double>(index + 2), Eigen::Vector3d::UnitY()));
  return transform;
}

Eigen::Vector3d makeColor(std::size_t index)
{
  static const std::array<Eigen::Vector3d, 6> kColors{{
      Eigen::Vector3d(0.85, 0.25, 0.22),
      Eigen::Vector3d(0.22, 0.55, 0.88),
      Eigen::Vector3d(0.95, 0.72, 0.24),
      Eigen::Vector3d(0.30, 0.75, 0.42),
      Eigen::Vector3d(0.68, 0.35, 0.82),
      Eigen::Vector3d(0.25, 0.74, 0.72),
  }};
  return kColors[index % kColors.size()];
}

struct RigidShapesState
{
  explicit RigidShapesState(dart::simulation::WorldPtr inputWorld)
    : world(std::move(inputWorld))
  {
  }

  template <class ShapeType, class... Args>
  void spawn(std::string label, Args&&... args)
  {
    if (world == nullptr) {
      return;
    }

    const std::size_t index = nextShapeIndex++;
    world->addSkeleton(createDynamicShape(
        std::string(kShapePrefix) + std::move(label) + "_"
            + std::to_string(index),
        std::make_shared<ShapeType>(std::forward<Args>(args)...),
        makePose(index),
        makeColor(index)));
  }

  void deleteLast()
  {
    if (world == nullptr || world->getNumSkeletons() <= 1) {
      return;
    }

    for (std::size_t i = world->getNumSkeletons(); i > 0; --i) {
      auto skeleton = world->getSkeleton(i - 1);
      if (skeleton != nullptr
          && skeleton->getName().rfind(kShapePrefix, 0) == 0) {
        world->removeSkeleton(skeleton);
        return;
      }
    }
  }

  dart::simulation::WorldPtr world;
  std::size_t nextShapeIndex = 0;
};

std::shared_ptr<RigidShapesState> createRigidShapesState(
    const dart::simulation::WorldPtr& world)
{
  auto state = std::make_shared<RigidShapesState>(world);
  state->spawn<dart::dynamics::BoxShape>(
      "box", Eigen::Vector3d(0.42, 0.28, 0.24));
  state->spawn<dart::dynamics::EllipsoidShape>(
      "ellipsoid", Eigen::Vector3d(0.30, 0.20, 0.18));
  state->spawn<dart::dynamics::CylinderShape>("cylinder", 0.18, 0.42);
  state->spawn<dart::dynamics::SphereShape>("sphere", 0.22);
  state->spawn<dart::dynamics::ConeShape>("cone", 0.20, 0.45);
  return state;
}

dart::simulation::WorldPtr createRigidShapesWorld()
{
  auto world = dart::simulation::World::create("rigid_shapes");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createGround());
  return world;
}

dart::gui::Panel createControlsPanel(
    const std::shared_ptr<RigidShapesState>& state)
{
  bool gravityEnabled = true;

  dart::gui::Panel panel;
  panel.title = "Rigid Shapes";
  panel.buildWithContext = [state, gravityEnabled](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) mutable {
    builder.text("Spawn rigid bodies with different collision shapes.");
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

    if (context.world != nullptr
        && builder.checkbox("Gravity", gravityEnabled)) {
      context.world->setGravity(
          gravityEnabled ? Eigen::Vector3d(0.0, 0.0, -9.81)
                         : Eigen::Vector3d::Zero());
    }

    if (builder.button("Box")) {
      state->spawn<dart::dynamics::BoxShape>(
          "box", Eigen::Vector3d(0.42, 0.28, 0.24));
    }
    builder.sameLine();
    if (builder.button("Ellipsoid")) {
      state->spawn<dart::dynamics::EllipsoidShape>(
          "ellipsoid", Eigen::Vector3d(0.30, 0.20, 0.18));
    }
    if (builder.button("Cylinder")) {
      state->spawn<dart::dynamics::CylinderShape>("cylinder", 0.18, 0.42);
    }
    builder.sameLine();
    if (builder.button("Sphere")) {
      state->spawn<dart::dynamics::SphereShape>("sphere", 0.22);
    }
    if (builder.button("Cone")) {
      state->spawn<dart::dynamics::ConeShape>("cone", 0.20, 0.45);
    }
    if (builder.button("Delete Last")) {
      state->deleteLast();
    }
    if (context.world != nullptr) {
      builder.text(
          "skeletons: " + std::to_string(context.world->getNumSkeletons()));
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto world = createRigidShapesWorld();
    auto state = createRigidShapesState(world);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.panels.push_back(createControlsPanel(state));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "rigid_shapes: " << e.what() << "\n";
    return 1;
  }
}
