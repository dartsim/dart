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

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/weld_joint_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <string>

#include <cstddef>

namespace {

constexpr const char* kFetchWorldUri
    = "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml";
constexpr const char* kFetchRobotName = "robot0:base_link";
constexpr const char* kFetchObjectName = "object0";
constexpr const char* kFetchMocapName = "robot0:mocap";
constexpr const char* kFetchTargetName = "interactive frame";
constexpr const char* kFetchGridName = "fetch_pick_and_place_grid";
constexpr const char* kFetchGridBodyName = "fetch_pick_and_place_grid_body";
constexpr const char* kFetchGridShapeName = "fetch_pick_and_place_grid_lines";

struct FetchScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  dart::dynamics::BodyNode* mocapRoot = nullptr;
};

Eigen::Isometry3d makeTargetTransform()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.3, 0.75, 0.50);
  transform.linear()
      = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  return transform;
}

void preferBulletCollisionDetector(dart::simulation::World& world)
{
#if DART_HAVE_BULLET
  world.setCollisionDetector(dart::simulation::CollisionDetectorType::Bullet);
#else
  (void)world;
#endif
}

void configureRobot(const dart::dynamics::SkeletonPtr& robot)
{
  robot->getJoint(0)->setActuatorType(
      dart::dynamics::Joint::ActuatorType::LOCKED);
  robot->eachDof([](dart::dynamics::DegreeOfFreedom* dof) {
    dof->setSpringStiffness(1e+3);
  });

  if (auto* torso = robot->getJoint("robot0:torso_lift_joint")) {
    torso->setSpringStiffness(0, 1e+7);
  }

  robot->setPosition(0, 0.405);
  robot->setPosition(1, 0.480);
  robot->setPosition(2, 0.000);
  robot->setPosition(6, 0.01);
  robot->setPosition(7, -0.73);
  robot->setPosition(8, 0.00);
  robot->setPosition(9, 1.64);
  robot->setPosition(10, 0.0);
  robot->setPosition(11, 0.66);
  robot->setPosition(12, 0.01);
}

void configureObject(const dart::dynamics::SkeletonPtr& object)
{
  object->setPosition(3, 1.25);
  object->setPosition(4, 0.53);
  object->setPosition(5, 0.40);
}

void resetFirstWeldConstraint(const dart::simulation::WorldPtr& world)
{
  auto* constraintSolver = world->getConstraintSolver();
  if (constraintSolver == nullptr
      || constraintSolver->getNumConstraints() == 0) {
    return;
  }

  auto weldJointConstraint
      = std::dynamic_pointer_cast<dart::constraint::WeldJointConstraint>(
          constraintSolver->getConstraint(0));
  if (weldJointConstraint != nullptr) {
    weldJointConstraint->setRelativeTransform(Eigen::Isometry3d::Identity());
  }
}

void setFrameColor(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& frame,
    const Eigen::Vector4d& color)
{
  frame->getVisualAspect(true)->setRGBA(color);
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createTargetCrossShape()
{
  auto cross = std::make_shared<dart::dynamics::LineSegmentShape>(8.0f);
  const std::size_t center = cross->addVertex(Eigen::Vector3d::Zero());
  cross->addVertex(Eigen::Vector3d(0.24, 0.0, 0.0), center);
  cross->addVertex(Eigen::Vector3d(-0.24, 0.0, 0.0), center);
  cross->addVertex(Eigen::Vector3d(0.0, 0.24, 0.0), center);
  cross->addVertex(Eigen::Vector3d(0.0, -0.24, 0.0), center);
  cross->addVertex(Eigen::Vector3d(0.0, 0.0, 0.18), center);
  cross->addVertex(Eigen::Vector3d(0.0, 0.0, -0.18), center);
  return cross;
}

std::shared_ptr<dart::dynamics::SimpleFrame> createTargetFrame()
{
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kFetchTargetName, makeTargetTransform());
  target->setShape(createTargetCrossShape());
  setFrameColor(target, Eigen::Vector4d(0.18, 0.86, 0.34, 0.92));
  return target;
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createFetchGridShape()
{
  auto grid = std::make_shared<dart::dynamics::LineSegmentShape>(2.0f);
  constexpr double halfExtent = 1.6;
  constexpr double spacing = 0.2;
  constexpr int lineCount = 16;
  for (int i = -lineCount / 2; i <= lineCount / 2; ++i) {
    const double coordinate = static_cast<double>(i) * spacing;
    const auto startX
        = grid->addVertex(Eigen::Vector3d(-halfExtent, coordinate, 0.0));
    grid->addVertex(Eigen::Vector3d(halfExtent, coordinate, 0.0), startX);
    const auto startY
        = grid->addVertex(Eigen::Vector3d(coordinate, -halfExtent, 0.0));
    grid->addVertex(Eigen::Vector3d(coordinate, halfExtent, 0.0), startY);
  }
  return grid;
}

dart::dynamics::SkeletonPtr createFetchGrid()
{
  auto grid = dart::dynamics::Skeleton::create(kFetchGridName);
  auto pair = grid->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  body->setName(kFetchGridBodyName);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.3, 0.75, 0.0);
  joint->setTransformFromParentBodyNode(transform);

  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      createFetchGridShape());
  shapeNode->setName(kFetchGridShapeName);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.44, 0.62, 0.50, 0.62));
  return grid;
}

dart::gui::OrbitCamera makeFetchCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.1, -0.3, 0.3);
  camera.yaw = 0.8341400147073799;
  camera.pitch = 0.3622488790595554;
  camera.distance = 6.208059278067503;
  return camera;
}

dart::gui::RunOptions makeFetchRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

FetchScene createFetchScene()
{
  FetchScene scene;
  scene.world = dart::io::readWorld(kFetchWorldUri);
  if (scene.world == nullptr) {
    throw std::runtime_error("Failed to load Fetch pick-and-place MJCF world");
  }
  preferBulletCollisionDetector(*scene.world);

  auto robot = scene.world->getSkeleton(kFetchRobotName);
  if (robot == nullptr) {
    throw std::runtime_error("Fetch world is missing robot0:base_link");
  }
  configureRobot(robot);

  auto object = scene.world->getSkeleton(kFetchObjectName);
  if (object == nullptr) {
    throw std::runtime_error("Fetch world is missing object0");
  }
  configureObject(object);

  auto mocap = scene.world->getSkeleton(kFetchMocapName);
  if (mocap == nullptr || mocap->getRootBodyNode() == nullptr) {
    throw std::runtime_error("Fetch world is missing robot0:mocap");
  }
  scene.mocapRoot = mocap->getRootBodyNode();
  resetFirstWeldConstraint(scene.world);

  scene.target = createTargetFrame();
  scene.world->addSkeleton(createFetchGrid());
  scene.world->addSimpleFrame(scene.target);
  return scene;
}

void syncMocapTarget(const FetchScene& scene)
{
  if (scene.mocapRoot == nullptr || scene.target == nullptr) {
    return;
  }

  auto* joint = scene.mocapRoot->getParentJoint();
  if (joint != nullptr) {
    joint->setTransformFromParentBodyNode(scene.target->getTransform());
  }
}

dart::gui::Panel createFetchPanel()
{
  dart::gui::Panel panel;
  panel.title = "Fetch";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Point cloud and voxel grid rendering example");
    builder.text("Fetch pick-and-place MJCF world");
    builder.text("Whole-body motion follows the green cross target.");
    builder.text("The offset grid marks the pick-and-place work area.");
    builder.text("Select the cross, then Ctrl-left drag or use arrow keys.");
    builder.text("X/Y/Z constrain Ctrl-left drag to one world axis.");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button("Play")) {
        context.lifecycle->paused = false;
      }
      builder.sameLine();
      if (builder.button("Pause")) {
        context.lifecycle->paused = true;
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Exit")) {
        dart::gui::requestExit(*context.lifecycle);
      }
    }
    builder.separator();
    builder.text("Help");
    builder.text("The end-effector follows the interactive target.");
    builder.text("About DART: project and libdart simulation libraries.");
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    FetchScene scene = createFetchScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.runDefaults = makeFetchRunDefaults();
    options.camera = makeFetchCamera();
    options.preStep = [scene]() {
      syncMocapTarget(scene);
    };
    options.panels.push_back(createFetchPanel());

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "fetch: " << e.what() << "\n";
    return 1;
  }
}
