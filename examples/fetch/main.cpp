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
#include <dart/gui/gizmo.hpp>
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
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
constexpr double kTargetRotationStep = 0.08726646259971647;
constexpr double kTargetBarLength = 0.52;
constexpr double kTargetBarWidth = 0.045;

struct FetchScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  std::vector<dart::gui::Gizmo> gizmos;
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

void addBoxToMesh(
    dart::math::TriMesh<double>& mesh,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& size)
{
  const Eigen::Vector3d half = 0.5 * size;
  const Eigen::Vector3d min = center - half;
  const Eigen::Vector3d max = center + half;
  const auto base = mesh.getVertices().size();

  mesh.addVertex(min.x(), min.y(), min.z());
  mesh.addVertex(max.x(), min.y(), min.z());
  mesh.addVertex(max.x(), max.y(), min.z());
  mesh.addVertex(min.x(), max.y(), min.z());
  mesh.addVertex(min.x(), min.y(), max.z());
  mesh.addVertex(max.x(), min.y(), max.z());
  mesh.addVertex(max.x(), max.y(), max.z());
  mesh.addVertex(min.x(), max.y(), max.z());

  const auto addFace
      = [&](std::size_t a, std::size_t b, std::size_t c, std::size_t d) {
          mesh.addTriangle(base + a, base + b, base + c);
          mesh.addTriangle(base + a, base + c, base + d);
        };

  addFace(0, 1, 2, 3);
  addFace(4, 7, 6, 5);
  addFace(0, 4, 5, 1);
  addFace(1, 5, 6, 2);
  addFace(2, 6, 7, 3);
  addFace(3, 7, 4, 0);
}

std::shared_ptr<dart::dynamics::MeshShape> createTargetCrossShape()
{
  auto mesh = std::make_shared<dart::math::TriMesh<double>>();
  mesh->reserveVertices(16);
  mesh->reserveTriangles(24);
  addBoxToMesh(
      *mesh,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(kTargetBarLength, kTargetBarWidth, kTargetBarWidth));
  addBoxToMesh(
      *mesh,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(kTargetBarWidth, kTargetBarWidth, kTargetBarLength));
  mesh->computeVertexNormals();

  auto shape = std::make_shared<dart::dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), mesh);
  shape->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
  shape->setAlphaMode(dart::dynamics::MeshShape::SHAPE_ALPHA);
  return shape;
}

std::shared_ptr<dart::dynamics::SimpleFrame> createTargetFrame()
{
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kFetchTargetName, makeTargetTransform());
  target->setShape(createTargetCrossShape());
  setFrameColor(target, Eigen::Vector4d(0.18, 0.86, 0.34, 0.45));
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
  dart::gui::Gizmo targetGizmo;
  targetGizmo.label = kFetchTargetName;
  targetGizmo.target = scene.target;
  targetGizmo.size = 0.24;
  scene.gizmos.push_back(std::move(targetGizmo));
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

void pauseAfterTargetEdit(dart::gui::KeyboardActionContext& context)
{
  if (context.lifecycle != nullptr) {
    context.lifecycle->paused = true;
  }
}

void rotateFetchTarget(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& target,
    const Eigen::Vector3d& localAxis,
    double angle,
    dart::gui::KeyboardActionContext& context)
{
  if (target == nullptr) {
    return;
  }

  Eigen::Isometry3d transform = target->getTransform();
  const Eigen::Vector3d worldAxis = transform.linear() * localAxis.normalized();
  transform.linear() = Eigen::AngleAxisd(angle, worldAxis).toRotationMatrix()
                       * transform.linear();
  target->setTransform(transform);
  pauseAfterTargetEdit(context);
}

void resetFetchTarget(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& target,
    dart::gui::KeyboardActionContext& context)
{
  if (target == nullptr) {
    return;
  }

  target->setTransform(makeTargetTransform());
  pauseAfterTargetEdit(context);
}

dart::gui::KeyboardAction makeFetchTargetAction(
    std::string label,
    char key,
    bool repeat,
    std::function<void(dart::gui::KeyboardActionContext&)> callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.repeat = repeat;
  action.callback = std::move(callback);
  return action;
}

std::vector<dart::gui::KeyboardAction> createFetchKeyboardActions(
    const std::shared_ptr<dart::dynamics::SimpleFrame>& target)
{
  std::vector<dart::gui::KeyboardAction> actions;
  const auto addRotation = [&](std::string label,
                               char key,
                               const Eigen::Vector3d& axis,
                               double angle) {
    actions.push_back(makeFetchTargetAction(
        std::move(label),
        key,
        true,
        [target, axis, angle](dart::gui::KeyboardActionContext& context) {
          rotateFetchTarget(target, axis, angle, context);
        }));
  };

  addRotation(
      "Rotate target +X", 'u', Eigen::Vector3d::UnitX(), kTargetRotationStep);
  addRotation(
      "Rotate target -X", 'j', Eigen::Vector3d::UnitX(), -kTargetRotationStep);
  addRotation(
      "Rotate target +Y", 'i', Eigen::Vector3d::UnitY(), kTargetRotationStep);
  addRotation(
      "Rotate target -Y", 'k', Eigen::Vector3d::UnitY(), -kTargetRotationStep);
  addRotation(
      "Rotate target +Z", 'o', Eigen::Vector3d::UnitZ(), kTargetRotationStep);
  addRotation(
      "Rotate target -Z", 'l', Eigen::Vector3d::UnitZ(), -kTargetRotationStep);
  actions.push_back(makeFetchTargetAction(
      "Reset target pose",
      'r',
      false,
      [target](dart::gui::KeyboardActionContext& context) {
        resetFetchTarget(target, context);
      }));

  return actions;
}

dart::gui::Panel createFetchPanel()
{
  dart::gui::Panel panel;
  panel.title = "Fetch robot example";
  panel.initialPosition = std::array<double, 2>{10.0, 20.0};
  panel.initialSize = std::array<double, 2>{360.0, 600.0};
  panel.backgroundAlpha = 0.5;
  panel.autoResize = false;
  panel.horizontalScrollbar = true;
  panel.menuBar = true;

  auto showAbout = std::make_shared<bool>(false);
  panel.buildWithContext = [showAbout](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    if (builder.beginMenuBar()) {
      if (builder.beginMenu("Menu")) {
        if (builder.menuItem("Exit") && context.lifecycle != nullptr) {
          dart::gui::requestExit(*context.lifecycle);
        }
        builder.endMenu();
      }
      if (builder.beginMenu("Help")) {
        if (builder.menuItem("About DART")) {
          *showAbout = true;
        }
        builder.endMenu();
      }
      builder.endMenuBar();
    }

    builder.text("Point cloud and voxel grid rendering example");
    builder.text(
        "The whole body motion of the Fetch robot is determined by the "
        "location of the end-effector.");
    builder.text(
        "The end-effector follows the invisible dummy object indicated at the "
        "cross of the two transparent green bars.");
    builder.text("The offset grid marks the pick-and-place work area.");

    if (builder.collapsingHeader("Help")) {
      builder.text("User Guid:");
      builder.text(
          "Left-drag target gizmo arrows/planes/rings at the green cross.");
      builder.text("Arrow/PageUp/PageDown nudge the selected target.");
      builder.text("u/j, i/k, o/l rotate the target about local X/Y/Z.");
      builder.text("r resets the target pose.");
      builder.text("Left drag orbits; right or middle drag pans; wheel zooms.");
      builder.text("Space pauses; n steps once; Escape exits.");
      builder.text(
          "--screenshot captures one frame; --out writes a PPM sequence.");
    }

    if (builder.collapsingHeader("Simulation", true)
        && context.lifecycle != nullptr) {
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
    builder.text("The end-effector follows the interactive target.");
    builder.text("Mouse manipulation uses public target gizmo handles.");
    if (*showAbout) {
      builder.text("About DART: project and libdart simulation libraries.");
    }
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
    options.gizmos = scene.gizmos;
    options.runDefaults = makeFetchRunDefaults();
    options.camera = makeFetchCamera();
    options.preStep = [scene]() {
      syncMocapTarget(scene);
    };
    options.panels.push_back(createFetchPanel());
    options.keyboardActions = createFetchKeyboardActions(scene.target);

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "fetch: " << e.what() << "\n";
    return 1;
  }
}
