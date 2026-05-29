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

#include <dart/gui/debug.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/package_resource_retriever.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/geometry.hpp>

#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kG1SkeletonName = "G1";
constexpr const char* kGroundSkeletonName = "ground";
constexpr const char* kGroundBodyName = "ground_body";
constexpr const char* kG1GridName = "g1_xy_grid";
constexpr const char* kG1GridBodyName = "g1_xy_grid_body";
constexpr const char* kG1GridShapeName = "g1_xy_grid_lines";
constexpr const char* kG1SupportOverlayName = "g1_support_polygon_overlay";
constexpr double kSupportVisualElevation = 0.02;

struct G1Options
{
  std::string packageName = "g1_description";
  std::string packageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string robotUri = "package://g1_description/g1_29dof.urdf";
};

void disableSkeletonCollisionAndGravity(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return;
  }

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

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
    return std::nullopt;
  }

  bool hasBounds = false;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();

  const auto includePoint = [&](const Eigen::Vector3d& point) {
    if (!hasBounds) {
      min = point;
      max = point;
      hasBounds = true;
      return;
    }
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  };

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }

    body->eachShapeNodeWith<dart::dynamics::VisualAspect>(
        [&](const dart::dynamics::ShapeNode* shapeNode) {
          if (shapeNode == nullptr || shapeNode->getShape() == nullptr
              || shapeNode->getVisualAspect()->isHidden()) {
            return;
          }

          const auto& bounds = shapeNode->getShape()->getBoundingBox();
          const Eigen::Vector3d localMin = bounds.getMin();
          const Eigen::Vector3d localMax = bounds.getMax();
          if (!localMin.allFinite() || !localMax.allFinite()) {
            return;
          }

          const Eigen::Isometry3d transform = shapeNode->getWorldTransform();
          for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
              for (int z = 0; z < 2; ++z) {
                includePoint(
                    transform
                    * Eigen::Vector3d(
                        x == 0 ? localMin.x() : localMax.x(),
                        y == 0 ? localMin.y() : localMax.y(),
                        z == 0 ? localMin.z() : localMax.z()));
              }
            }
          }
        });
  }

  if (!hasBounds) {
    return std::nullopt;
  }

  return std::make_pair(min, max);
}

dart::common::ResourceRetrieverPtr createG1ResourceRetriever(
    const G1Options& options)
{
  auto local = std::make_shared<dart::common::LocalResourceRetriever>();
  auto dartRetriever = std::make_shared<dart::utils::DartResourceRetriever>();
  auto http = std::make_shared<dart::utils::HttpResourceRetriever>();

  auto passthrough
      = std::make_shared<dart::utils::CompositeResourceRetriever>();
  passthrough->addSchemaRetriever("file", local);
  passthrough->addSchemaRetriever("dart", dartRetriever);
  passthrough->addSchemaRetriever("http", http);
  passthrough->addSchemaRetriever("https", http);
  passthrough->addDefaultRetriever(local);

  auto packageRetriever
      = std::make_shared<dart::utils::PackageResourceRetriever>(passthrough);
  packageRetriever->addPackageDirectory(
      options.packageName, options.packageUri);

  auto resolver = std::make_shared<dart::utils::CompositeResourceRetriever>();
  resolver->addSchemaRetriever("package", packageRetriever);
  resolver->addSchemaRetriever("file", local);
  resolver->addSchemaRetriever("dart", dartRetriever);
  resolver->addSchemaRetriever("http", http);
  resolver->addSchemaRetriever("https", http);
  resolver->addDefaultRetriever(local);

  return resolver;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  body->setName(kGroundBodyName);

  constexpr double thickness = 0.1;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(8.0, 8.0, thickness)));
  shapeNode->setRelativeTranslation(
      Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.4, 0.4, 0.4, 1.0));
  return ground;
}

void addDisconnectedLine(
    dart::dynamics::LineSegmentShape& shape,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to)
{
  const auto start = shape.addVertex(from);
  if (start > 0u) {
    shape.removeConnection(start - 1u, start);
  }
  shape.addVertex(to, start);
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createG1GridShape()
{
  auto grid = std::make_shared<dart::dynamics::LineSegmentShape>(1.0f);
  constexpr int cellCount = 40;
  constexpr double spacing = 0.1;
  constexpr double halfExtent = cellCount * spacing * 0.5;
  for (int i = -cellCount / 2; i <= cellCount / 2; ++i) {
    const double coordinate = static_cast<double>(i) * spacing;
    addDisconnectedLine(
        *grid,
        Eigen::Vector3d(-halfExtent, coordinate, 0.0),
        Eigen::Vector3d(halfExtent, coordinate, 0.0));
    addDisconnectedLine(
        *grid,
        Eigen::Vector3d(coordinate, -halfExtent, 0.0),
        Eigen::Vector3d(coordinate, halfExtent, 0.0));
  }
  return grid;
}

dart::dynamics::SkeletonPtr createG1Grid()
{
  auto grid = dart::dynamics::Skeleton::create(kG1GridName);
  auto* body
      = grid->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  body->setName(kG1GridBodyName);

  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      createG1GridShape());
  shapeNode->setName(kG1GridShapeName);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.72, 0.72, 0.72, 0.58));
  return grid;
}

dart::dynamics::SkeletonPtr loadG1Skeleton(const G1Options& options)
{
  dart::io::ReadOptions readOptions;
  readOptions.resourceRetriever = createG1ResourceRetriever(options);
  const dart::common::Uri robotUri(options.robotUri);
  auto robot = dart::io::readSkeleton(robotUri, readOptions);
  if (robot == nullptr) {
    throw std::runtime_error(
        "Failed to load G1 robot model from " + options.robotUri);
  }

  robot->setName(kG1SkeletonName);
  auto* rootBody = robot->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             != nullptr) {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation().z() = 0.75;
    dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(robot)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    }
  }

  disableSkeletonCollisionAndGravity(robot);
  return robot;
}

dart::math::SupportGeometry makeG1FootSupportGeometry()
{
  dart::math::SupportGeometry geometry;
  geometry.emplace_back(Eigen::Vector3d(0.12, 0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, 0.06, 0.0));
  return geometry;
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createLineShape(
    const std::vector<dart::gui::DebugLineDescriptor>& lines)
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(3.0f);
  for (const auto& line : lines) {
    addDisconnectedLine(*shape, line.from, line.to);
  }
  return shape;
}

std::vector<dart::gui::DebugLineDescriptor> makeG1SupportPolygonLines(
    const dart::dynamics::SkeletonPtr& robot)
{
  if (robot == nullptr) {
    return {};
  }

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawSupportPolygons = true;
  options.supportPolygonElevation = kSupportVisualElevation;
  return dart::gui::makeSupportPolygonDebugLines(*robot, options, "g1");
}

dart::dynamics::SimpleFramePtr createG1SupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& robot)
{
  auto overlay = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kG1SupportOverlayName);
  overlay->setShape(createLineShape(makeG1SupportPolygonLines(robot)));
  overlay->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.97, 0.78, 0.24, 0.86));
  return overlay;
}

void updateG1SupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& robot,
    const dart::dynamics::SimpleFramePtr& overlay)
{
  if (overlay == nullptr) {
    return;
  }

  overlay->setShape(createLineShape(makeG1SupportPolygonLines(robot)));
}

struct G1Scene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr robot;
  dart::dynamics::SimpleFramePtr supportOverlay;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
  struct TargetState
  {
    dart::simulation::WorldPtr world;
    dart::dynamics::EndEffector* effector = nullptr;
    dart::dynamics::InverseKinematicsPtr ik;
    dart::dynamics::SimpleFramePtr target;
    std::string label;
    char hotkey = '\0';
    bool active = false;

    void activate(bool resetTargetTransform = true)
    {
      if (active || world == nullptr || effector == nullptr
          || target == nullptr) {
        return;
      }
      if (resetTargetTransform) {
        target->setTransform(effector->getWorldTransform());
      }
      world->addSimpleFrame(target);
      active = true;
      std::cout << "Activated IK target '" << effector->getName() << "'.\n";
      solve();
    }

    void deactivate()
    {
      if (!active || world == nullptr || target == nullptr) {
        return;
      }
      world->removeSimpleFrame(target);
      active = false;
      std::cout << "Deactivated IK target '" << effector->getName() << "'.\n";
    }

    void toggle()
    {
      if (active) {
        deactivate();
      } else {
        activate();
      }
    }

    void solve()
    {
      if (active && ik != nullptr) {
        ik->solveAndApply(true);
      }
    }
  };

  std::vector<std::shared_ptr<TargetState>> targetStates;
};

void addG1IkTargets(G1Scene& scene, const dart::dynamics::SkeletonPtr& robot)
{
  struct Config
  {
    const char* bodyNode;
    const char* effectorName;
    const char* targetName;
    const char* label;
    int hotkey;
    bool supportContact;
  };

  const std::array<Config, 4> configs{{
      {"left_rubber_hand",
       "left_rubber_hand_target",
       "ik_target_left_hand",
       "1 left hand",
       '1',
       false},
      {"right_rubber_hand",
       "right_rubber_hand_target",
       "ik_target_right_hand",
       "2 right hand",
       '2',
       false},
      {"left_ankle_roll_link",
       "left_ankle_roll_link_target",
       "ik_target_left_foot",
       "3 left foot",
       '3',
       true},
      {"right_ankle_roll_link",
       "right_ankle_roll_link_target",
       "ik_target_right_foot",
       "4 right foot",
       '4',
       true},
  }};

  const auto footSupportGeometry = makeG1FootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = robot->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "G1 robot model is missing body node "
          + std::string(config.bodyNode));
    }

    auto* endEffector = bodyNode->createEndEffector(config.effectorName);
    if (config.supportContact) {
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    }

    auto ik = endEffector->getIK(true);
    ik->setGradientMethod<
        dart::dynamics::InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);

    auto target = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    ik->setTarget(target);

    auto state = std::make_shared<G1Scene::TargetState>();
    state->world = scene.world;
    state->effector = endEffector;
    state->ik = ik;
    state->target = target;
    state->label = config.label;
    state->hotkey = config.hotkey;

    dart::gui::InverseKinematicsHandle handle;
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = target;
    handle.ik = ik;
    scene.ikHandles.push_back(std::move(handle));

    dart::gui::Gizmo gizmo;
    gizmo.label = config.targetName;
    gizmo.target = target;
    gizmo.size = 0.15;
    gizmo.isVisible = [state]() {
      return state->active;
    };
    gizmo.onChanged = [state](const Eigen::Isometry3d&) {
      state->solve();
    };
    scene.gizmos.push_back(std::move(gizmo));
    scene.targetStates.push_back(std::move(state));
  }
}

G1Scene createG1Scene(const G1Options& options)
{
  G1Scene scene;
  scene.world = dart::simulation::World::create("dartsim_g1");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(createGround());
  scene.world->addSkeleton(createG1Grid());

  auto robot = loadG1Skeleton(options);
  std::cout << "Loaded G1 robot from '" << options.robotUri << "'.\n"
            << "Package root for '" << options.packageName << "' set to '"
            << options.packageUri << "'.\n";
  scene.robot = robot;
  scene.world->addSkeleton(scene.robot);
  addG1IkTargets(scene, scene.robot);
  scene.supportOverlay = createG1SupportPolygonOverlay(scene.robot);
  scene.world->addSimpleFrame(scene.supportOverlay);
  return scene;
}

void solveActiveG1Targets(
    const std::vector<std::shared_ptr<G1Scene::TargetState>>& targetStates)
{
  for (const auto& state : targetStates) {
    if (state != nullptr) {
      state->solve();
    }
  }
}

std::vector<dart::gui::KeyboardAction> createG1KeyboardActions(
    const std::vector<std::shared_ptr<G1Scene::TargetState>>& targetStates)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(targetStates.size());
  for (const auto& state : targetStates) {
    if (state == nullptr || state->hotkey == '\0') {
      continue;
    }

    dart::gui::KeyboardAction action;
    action.label = "Toggle G1 target " + state->label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(state->hotkey);
    action.callback = [state](dart::gui::KeyboardActionContext& context) {
      state->toggle();
      if (context.lifecycle != nullptr) {
        context.lifecycle->paused = true;
      }
    };
    actions.push_back(std::move(action));
  }
  return actions;
}

std::vector<dart::gui::BodyNodeDragHandle> createG1BodyNodeDragHandles(
    const dart::dynamics::SkeletonPtr& robot)
{
  std::vector<dart::gui::BodyNodeDragHandle> handles;
  if (robot == nullptr) {
    return handles;
  }

  handles.reserve(robot->getNumBodyNodes());
  for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
    auto* bodyNode = robot->getBodyNode(i);
    if (bodyNode == nullptr) {
      continue;
    }

    dart::gui::BodyNodeDragHandle handle;
    handle.label = bodyNode->getName();
    handle.bodyNode = bodyNode;
    handles.push_back(handle);
  }
  return handles;
}

dart::gui::Panel createG1Panel(const G1Options& options)
{
  dart::gui::Panel panel;
  panel.title = "G1 Puppet";
  panel.buildWithContext = [options](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("G1 whole-body IK puppet");
    builder.text("Press 1-4 to toggle/select targets.");
    builder.text("Left-drag active target gizmo handles.");
    builder.text("Alt-drag translates body nodes; Ctrl-drag rotates them.");
    builder.text("Shift-drag moves a body with only its parent joint.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.text(
        "The support polygon overlay follows the active foot targets.");
    builder.text("Only active targets solve each simulation step.");
    builder.separator();
    builder.text("robot: " + options.robotUri);
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

dart::gui::OrbitCamera makeG1Camera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.75);
  camera.yaw = 0.48995732625372834;
  camera.pitch = 0.18889718087762267;
  camera.distance = 3.4615747861341952;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeG1PuppetScene()
{
  const G1Options g1Options;
  G1Scene scene = createG1Scene(g1Options);

  dart::gui::ApplicationOptions options;
  options.world = scene.world;
  options.ikHandles = scene.ikHandles;
  options.gizmos = scene.gizmos;
  options.camera = makeG1Camera();
  options.preStep = [targetStates = scene.targetStates,
                     robot = scene.robot,
                     supportOverlay = scene.supportOverlay]() {
    solveActiveG1Targets(targetStates);
    updateG1SupportPolygonOverlay(robot, supportOverlay);
  };
  options.keyboardActions = createG1KeyboardActions(scene.targetStates);
  options.bodyNodeDragHandles = createG1BodyNodeDragHandles(scene.robot);
  options.panels.push_back(createG1Panel(g1Options));
  return options;
}

} // namespace dart::examples::demos
